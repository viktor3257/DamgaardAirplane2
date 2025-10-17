#!/usr/bin/env python3
"""
airplane_pi.py — async process for Raspberry Pi.

Components:
- GPS reader (SIM7600G-H NMEA → lat/lon, heading, ground speed)
- ESP32 telemetry simulator (until real UART is wired)
- Web poster (POSTs location/telemetry to your API)
- Website poller (stream-state, circling-point, search-status)
- Stream state manager
- CSV logger
- Visual search loop using OpenAi API (snapshot every ~5s, detect object, post hit)
"""

import os
import sys
import csv
import time
import json
import math
import signal
import random
import asyncio
import logging
import subprocess
import aiohttp
import base64
import shutil
from dataclasses import dataclass, field
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import List, Optional


# ---------------------------
# Configuration
# ---------------------------
BASE_DIR = Path(__file__).resolve().parent
REPO_ROOT = BASE_DIR.parent

@dataclass
class Config:
    # Paths
    log_dir: Path = BASE_DIR / "log"
    git_remote: str = "origin"

    # Devices (tweak when wiring real hardware)
    uart_dev: str = "/dev/ttyS0"
    uart_baud: int = 115_200
    gps_dev: str = "/dev/ttyAMA0"

    # Rates
    poll_period_s: float = 1.0   # website poll
    log_rate_hz: float = 5.0     # CSV logging
    post_rate_hz: float = 1.0    # HTTP POST

    # Logging verbosity
    debug: bool = True

    # Streaming
    rtmp_url: str = "rtmp://a.rtmp.youtube.com/live2"
    rtmp_key: str = "bq45-5jky-84s0-cy0y-e8uy"
    cam_width: int = 1280
    cam_height: int = 720
    cam_fps: int = 30
    cam_rotation_deg: int = 180   # 180 to flip image
    cam_bitrate_kbps: int = 3500  # tune if needed

    # Snapshots for visual search
    snap_dir: Path = BASE_DIR / "snaps"
    snap_name: str = "snap_latest.jpg"

    # OpenAI (vision)
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "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")
    openai_model: str = "gpt-4o-mini"  # vision-capable & fast

@dataclass
class DetectConfig:
    enabled: bool = False
    query: str = ""
    period_s: float = 5.0         # snapshot/sample cadence
    min_conf: float = 0.5         # accept at/above this confidence
    confirmations: int = 1        # consecutive positives required

@dataclass
class DetectState:
    local_search_locked: bool = False  # set True after a hit; cleared when website disables search
    consecutive_positives: int = 0
    last_confidence: float = 0.0
    last_error: str = ""

# ---------------------------
# Data models
# ---------------------------
@dataclass
class LatLon:
    lat: float
    lon: float

@dataclass
class GpsFix:
    time_utc: float = math.nan
    lat: float = math.nan
    lon: float = math.nan
    alt_m: float = math.nan
    fix_ok: bool = False
    sats: int = 0

@dataclass
class Telemetry:
    time_utc: float = math.nan
    batt_v: float = math.nan
    airspeed_ms: float = math.nan
    groundspeed_ms: float = math.nan
    roll_deg: float = math.nan
    pitch_deg: float = math.nan
    yaw_deg: float = math.nan
    lat: float = math.nan
    lon: float = math.nan
    alt_m: float = math.nan
    mode: int = 1                 # 1=Neutral, 2=Manual, 3=Circling (from ESP32)
    battery_pct: float = math.nan # 0..100 (from ESP32)

@dataclass
class RuntimeState:
    stream_enabled: bool = False
    target_points: List[LatLon] = field(default_factory=list)
    latest_gps: GpsFix = field(default_factory=GpsFix)
    latest_esp32: Telemetry = field(default_factory=Telemetry)
    shutting_down: bool = False

    # NEW: visual search
    detect_cfg: DetectConfig = field(default_factory=DetectConfig)
    detect_state: DetectState = field(default_factory=DetectState)

# ---------------------------
# GPS (SIM7600G-H via NMEA)
# ---------------------------
async def gps_reader(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    Read NMEA sentences from SIM7600G-H and update:
      - state.latest_gps.lat / lon / time_utc / fix_ok
      - state.latest_esp32.groundspeed_ms (from RMC/VTG)
      - state.latest_esp32.yaw_deg (course/heading from RMC/VTG)
    Notes:
      - NMEA is often on /dev/ttyUSB1; AT commands on /dev/ttyUSB2.
      - Tries to power GNSS once via AT; continues even if that fails.
    """
    import serial  # local import keeps import errors isolated

    nmea_port = "/dev/ttyUSB1"  # change if your NMEA port differs
    at_port   = "/dev/ttyUSB2"  # AT command port (best-effort power on)

    # Power GNSS (best-effort)
    try:
        at = serial.Serial(at_port, 115200, timeout=1)
        for cmd in ("AT", "AT+CGNSPWR=1", "AT+CGNSSEQ=RMC", "AT+CGNSURC=0"):
            at.write((cmd + "\r\n").encode())
            at.read_until(b"OK\r\n")
        at.close()
    except Exception as e:
        logger.debug(f"[SIM7600] AT init skipped: {e}")

    # Open NMEA stream
    try:
        s = serial.Serial(nmea_port, 115200, timeout=0.2)
    except Exception as e:
        logger.info(f"[SIM7600] cannot open {nmea_port}: {e}")
        while not state.shutting_down:
            await asyncio.sleep(1.0)
        return

    logger.info(f"[SIM7600] NMEA on {nmea_port} started")

    # Inline converters
    def nmea_deg(dm, hemi):
        try:
            v = float(dm)
            deg = int(v // 100)
            minutes = v - deg * 100
            d = deg + minutes / 60.0
            if hemi in ("S", "W"):
                d = -d
            return d
        except Exception:
            return math.nan

    def to_float(x):
        try:
            return float(x)
        except Exception:
            return math.nan

    while not state.shutting_down:
        await asyncio.sleep(0)  # yield to event loop
        try:
            line = s.readline().decode(errors="ignore").strip()
            if not line or line[0] != "$":
                continue
            if "*" in line:
                line = line.split("*", 1)[0]
            parts = line.split(",")
            kind = parts[0]

            # RMC: pos/speed/track
            if kind.endswith("RMC") and len(parts) >= 12:
                if parts[2] == "A":  # valid
                    lat = nmea_deg(parts[3], parts[4])
                    lon = nmea_deg(parts[5], parts[6])
                    sog_knots = to_float(parts[7])
                    cog_deg = to_float(parts[8])

                    state.latest_gps.time_utc = time.time()
                    state.latest_gps.lat = lat
                    state.latest_gps.lon = lon
                    state.latest_gps.fix_ok = True

                    if math.isfinite(sog_knots):
                        state.latest_esp32.groundspeed_ms = sog_knots * 0.514444
                    if math.isfinite(cog_deg):
                        state.latest_esp32.yaw_deg = cog_deg % 360.0

            # VTG: course/speed
            elif kind.endswith("VTG") and len(parts) >= 9:
                cog = to_float(parts[1])       # true course (deg)
                sog_knots = to_float(parts[5]) # speed in knots
                if math.isfinite(cog):
                    state.latest_esp32.yaw_deg = cog % 360.0
                if math.isfinite(sog_knots):
                    state.latest_esp32.groundspeed_ms = sog_knots * 0.514444

        except Exception:
            continue

    try:
        s.close()
    except Exception:
        pass
    logger.info("[SIM7600] NMEA reader stopped")

# ---------------------------
# ESP32 telemetry (simulated)
# ---------------------------

async def esp32_uart_rx(state: RuntimeState, cfg: Config, logger: logging.Logger):
    t0 = time.time()
    while not state.shutting_down:
        await asyncio.sleep(0.05)
        t = time.time() - t0
        mode_sim = [1, 2, 3][int(t // 10) % 3]  # cycles every 10s
        pct_sim  = 80 + 15 * math.sin(t / 30)   # ~65..95%
        state.latest_esp32 = Telemetry(
            time_utc=time.time(),
            batt_v=11.1 + 0.2 * math.sin(t / 10),
            airspeed_ms=10 + 2 * math.sin(t),
            groundspeed_ms=state.latest_esp32.groundspeed_ms,  # updated by GPS task
            roll_deg=10 * math.sin(t * 0.7),
            pitch_deg=5 * math.sin(t * 0.5),
            yaw_deg=state.latest_esp32.yaw_deg,                # updated by GPS task
            lat=state.latest_gps.lat,
            lon=state.latest_gps.lon,
            alt_m=state.latest_gps.alt_m,
            mode=mode_sim,
            battery_pct=max(0, min(100, pct_sim)),
        )

async def esp32_uart_tx(uart_tx_q: asyncio.Queue, state: RuntimeState, cfg: Config, logger: logging.Logger):
    """Send commands to ESP32 (simulated: print)."""
    while not state.shutting_down:
        try:
            msg = await asyncio.wait_for(uart_tx_q.get(), timeout=0.5)
        except asyncio.TimeoutError:
            continue
        logger.info(f"[UART->ESP32] {msg}")
        uart_tx_q.task_done()

# ---------------------------
# Website poller
# ---------------------------
async def website_poller(state: RuntimeState, uart_tx_q: asyncio.Queue, cfg: Config, logger: logging.Logger):
    """
    Poll every cfg.poll_period_s:
      - GET /api/stream-state -> update stream_enabled
      - GET /api/circling-point -> set target_points and send a TARGETS:lat,lon line on change
      - (NEW) GET /api/search-status -> update visual search config (enabled/query)
    """
    base = "https://studio--sky-pointer.us-central1.hosted.app"
    period = cfg.poll_period_s
    last_sent_point = None  # (lat, lon)

    async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=5)) as session:
        while not state.shutting_down:
            await asyncio.sleep(period)

            # stream-state
            try:
                resp = await session.get(base + "/api/stream-state")
                if resp.status == 200:
                    data = await resp.json(content_type=None)
                    is_on = bool(data.get("isOn"))
                    if is_on != state.stream_enabled:
                        state.stream_enabled = is_on
                        logger.info(f"[API] stream_enabled -> {state.stream_enabled}")
                else:
                    logger.info(f"[API] stream-state HTTP {resp.status}")
            except Exception as e:
                logger.info(f"[API] stream-state failed: {e}")

            # circling-point
            try:
                resp = await session.get(base + "/api/circling-point")
                if resp.status == 200:
                    data = await resp.json(content_type=None)
                    lat = data.get("lat")
                    lng = data.get("lng")
                    if isinstance(lat, (int, float)) and isinstance(lng, (int, float)):
                        state.target_points = [LatLon(float(lat), float(lng))]
                        key = (float(lat), float(lng))
                        if key != last_sent_point:
                            await uart_tx_q.put(f"TARGETS:{lat:.6f},{lng:.6f}")
                            last_sent_point = key
                            logger.info(f"[API] circling point -> {lat:.6f},{lng:.6f}")
                elif resp.status == 404:
                    state.target_points = []
                else:
                    logger.info(f"[API] circling-point HTTP {resp.status}")
            except Exception as e:
                logger.info(f"[API] circling-point failed: {e}")

            # object search status
            try:
                resp = await session.get(base + "/api/search-status")
                if resp.status == 200:
                    data = await resp.json(content_type=None)
                    is_searching = bool(data.get("isSearching", False))
                    query = str(data.get("query", "") or "")

                    dc = state.detect_cfg
                    prev_enabled = dc.enabled
                    dc.enabled = is_searching and len(query.strip()) > 0
                    dc.query = query.strip()

                    # If website turned OFF, clear local lock & streak
                    if prev_enabled and not dc.enabled:
                        state.detect_state.local_search_locked = False
                        state.detect_state.consecutive_positives = 0
                else:
                    logger.info(f"[API] search-status HTTP {resp.status}")
            except Exception as e:
                logger.info(f"[API] search-status failed: {e}")

# ---------------------------
# Stream manager
# ---------------------------
async def stream_manager(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    Start/stop YouTube stream based on state.stream_enabled.
    rpicam-vid (H.264) --nopreview -> ffmpeg (copy video + silent AAC) -> RTMP.

    NOTE: The visual search uses rpicam-still/libcamera-still for snapshots.
    When streaming is ON, the camera is occupied; snapshot capture may fail.
    (If you want snapshots during streaming, extend this to add an ffmpeg
     branch that writes a JPEG every N seconds.)
    """
    import subprocess, time, shutil

    cam_bin = shutil.which("rpicam-vid")
    if not cam_bin:
        logger.info("[STREAM] rpicam-vid not found (install 'rpicam-apps').")
        while not state.shutting_down:
            await asyncio.sleep(5.0)
        return

    cam_cmd = [
        cam_bin,
        "--nopreview",
        "--inline",
        "--codec", "h264",
        "--profile", "high",
        "--level", "4.1",
        "--width", str(cfg.cam_width),
        "--height", str(cfg.cam_height),
        "--framerate", str(cfg.cam_fps),
        "--rotation", str(cfg.cam_rotation_deg),
        "--bitrate", str(cfg.cam_bitrate_kbps * 1000),
        "--intra", str(cfg.cam_fps * 2),   # ~2s keyframe interval
        "-t", "0",
        "-o", "-",                          # H.264 to stdout
    ]

    # Add a silent audio source; some YouTube ingest paths prefer A/V
    ffmpeg_cmd = [
        "ffmpeg",
        "-re",
        "-thread_queue_size", "1024",
        "-f", "lavfi", "-i", "anullsrc=channel_layout=stereo:sample_rate=44100",
        "-thread_queue_size", "1024",
        "-fflags", "+genpts",
        "-use_wallclock_as_timestamps", "1",
        "-i", "pipe:0",                    # video from rpicam-vid
        "-c:v", "copy",                    # copy H.264 (no re-encode)
        "-c:a", "aac", "-b:a", "128k",
        "-f", "flv",
        f"{cfg.rtmp_url}/{cfg.rtmp_key}",
    ]

    cam_proc = None
    ffmpeg_proc = None
    running = False

    def start_pipeline():
        nonlocal cam_proc, ffmpeg_proc, running
        if running:
            return
        try:
            cam_proc = subprocess.Popen(
                cam_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0
            )
            ffmpeg_proc = subprocess.Popen(
                ffmpeg_cmd,
                stdin=cam_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            running = True
            logger.info("[STREAM] started (rpicam-vid → ffmpeg → RTMP)")
        except Exception as e:
            logger.info(f"[STREAM] failed to start: {e}")
            try:
                if ffmpeg_proc and ffmpeg_proc.poll() is None:
                    ffmpeg_proc.terminate()
            except Exception:
                pass
            try:
                if cam_proc and cam_proc.poll() is None:
                    cam_proc.terminate()
            except Exception:
                pass
            cam_proc = None
            ffmpeg_proc = None
            running = False

    def stop_pipeline():
        nonlocal cam_proc, ffmpeg_proc, running
        if not running:
            return
        logger.info("[STREAM] stopping")
        for p in (ffmpeg_proc, cam_proc):
            try:
                if p and p.poll() is None:
                    p.terminate()
            except Exception:
                pass
        time.sleep(0.4)
        for p in (ffmpeg_proc, cam_proc):
            try:
                if p and p.poll() is None:
                    p.kill()
            except Exception:
                pass
        cam_proc, ffmpeg_proc = None, None
        running = False

    try:
        while not state.shutting_down:
            await asyncio.sleep(0.25)
            if state.stream_enabled and not running:
                start_pipeline()
            elif not state.stream_enabled and running:
                stop_pipeline()

            if state.stream_enabled and running:
                dead = False
                try:
                    if cam_proc and cam_proc.poll() is not None:
                        dead = True
                    if ffmpeg_proc and ffmpeg_proc.poll() is not None:
                        dead = True
                except Exception:
                    dead = True
                if dead:
                    logger.info("[STREAM] pipeline exited; restarting")
                    stop_pipeline()
                    start_pipeline()
    finally:
        stop_pipeline()

# ---------------------------
# CSV logger
# ---------------------------
async def file_logger(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    On start:
      - Rename previous flight to local (UTC+2) 'YYYY-mm-dd_HH-MM.csv'
      - git add/commit/push; delete the file locally if push succeeds
    Then:
      - Create 'flight_YYYYmmdd_%H%M.csv' for this boot and append rows
    """
    cfg.log_dir.mkdir(parents=True, exist_ok=True)

    # Push previous flight (best-effort)
    prevs = sorted(cfg.log_dir.glob("flight_*.csv"))
    if prevs and (REPO_ROOT / ".git").exists():
        local_now = datetime.utcnow() + timedelta(hours=2)
        pushed_name = local_now.strftime("%Y-%m-%d_%H-%M") + ".csv"
        prev = prevs[-1]
        target = prev.with_name(pushed_name)
        try:
            prev.rename(target)
            subprocess.run(["git", "-C", str(REPO_ROOT), "add", "-f", str(target)], check=False)
            subprocess.run(["git", "-C", str(REPO_ROOT), "commit", "-m", f"Flight log {target.name}"], check=False)
            r = subprocess.run(["git", "-C", str(REPO_ROOT), "push", "origin", "HEAD"], check=False)
            if r.returncode == 0:
                try:
                    target.unlink()
                    logger.info(f"Pushed and removed previous log: {target.name}")
                except Exception:
                    logger.info(f"Pushed previous log (kept local copy): {target.name}")
            else:
                logger.info("Previous log push failed; kept file on disk.")
        except Exception as e:
            logger.info(f"Skip push/rename of previous log: {e}")
    else:
        logger.info("No previous log to push or not a git repo.")

    # New log for this boot
    local_now = datetime.utcnow() + timedelta(hours=2)
    current_path = cfg.log_dir / f"flight_{local_now.strftime('%Y%m%d_%H%M')}.csv"
    f = current_path.open("w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "ts_iso","batt_v","airspeed_ms","groundspeed_ms",
        "roll_deg","pitch_deg","yaw_deg","lat","lon","alt_m"
    ])
    f.flush()

    # latest symlink
    latest = cfg.log_dir / "latest.csv"
    try:
        if latest.exists() or latest.is_symlink():
            latest.unlink()
        latest.symlink_to(current_path.name)
    except Exception:
        pass
    logger.info(f"Logging to {current_path}")

    # Append rows
    period = 1.0 / max(cfg.log_rate_hz, 1.0)

    def ts_iso(ts_val: Optional[float]) -> str:
        base = datetime.fromtimestamp(ts_val, tz=timezone.utc) if (isinstance(ts_val, (int, float)) and math.isfinite(ts_val)) else datetime.now(tz=timezone.utc)
        return base.isoformat()

    def fmt(x, nd):
        return "" if not isinstance(x, (int, float)) or not math.isfinite(x) else f"{x:.{nd}f}"

    try:
        while not state.shutting_down:
            t0 = time.perf_counter()
            te = state.latest_esp32
            writer.writerow([
                ts_iso(te.time_utc),
                fmt(te.batt_v,3), fmt(te.airspeed_ms,3), fmt(te.groundspeed_ms,3),
                fmt(te.roll_deg,3), fmt(te.pitch_deg,3), fmt(te.yaw_deg,3),
                fmt(te.lat,6), fmt(te.lon,6), fmt(te.alt_m,2),
            ])
            f.flush()
            dt = period - (time.perf_counter() - t0)
            if dt > 0:
                await asyncio.sleep(dt)
    finally:
        f.close()

# ---------------------------
# Web poster (telemetry)
# ---------------------------
async def web_poster(state: RuntimeState, cfg: Config, logger: logging.Logger):
    url = "https://studio--sky-pointer.us-central1.hosted.app/api/drone-location"
    period = 1.0 / max(cfg.post_rate_hz, 1.0)
    headers = {"Content-Type": "application/json"}

    async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=5)) as session:
        while not state.shutting_down:
            await asyncio.sleep(period)
            te = state.latest_esp32
            gp = state.latest_gps

            mode_str = {1: "Neutral", 2: "Manual", 3: "Circling"}.get(
                int(te.mode) if isinstance(te.mode, (int, float)) else 1, "Neutral"
            )

            payload = {
                "lat": float(f"{gp.lat:.6f}") if isinstance(gp.lat, (int,float)) and math.isfinite(gp.lat) else 0.0,
                "lng": float(f"{gp.lon:.6f}") if isinstance(gp.lon, (int,float)) and math.isfinite(gp.lon) else 0.0,
                "heading": float(f"{te.yaw_deg:.2f}") if isinstance(te.yaw_deg, (int,float)) and math.isfinite(te.yaw_deg) else 0.0,
                "height": float(f"{gp.alt_m:.1f}") if isinstance(gp.alt_m, (int,float)) and math.isfinite(gp.alt_m) else 0.0,
                "airspeed": float(f"{te.airspeed_ms:.2f}") if isinstance(te.airspeed_ms, (int,float)) and math.isfinite(te.airspeed_ms) else 0.0,
                "batteryPercentage": int(te.battery_pct) if isinstance(te.battery_pct, (int,float)) and math.isfinite(te.battery_pct) else 0,
                "mode": mode_str,
            }

            try:
                r = await session.post(url, json=payload, headers=headers)
                if r.status >= 400:
                    txt = await r.text()
                    logger.info(f"[POST] {r.status} {txt[:200]}")
                else:
                    logger.debug(f"[POST] ok {payload}")
            except Exception as e:
                logger.info(f"[POST] failed: {e}")

# ---------------------------
# Visual search 
# ---------------------------
async def visual_search_loop(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    Every detect_cfg.period_s:
      - capture JPEG (rpicam-still/libcamera-still)
      - ask OpenAI if the image contains `detect_cfg.query`
      - model must answer strictly 'True' or 'False'
      - when `confirmations` consecutive True -> post GPS + image and lock locally
    """
    import asyncio, base64, os, shutil, math
    from datetime import datetime
    from aiohttp import ClientSession, ClientTimeout

    logger.info("[DETECT] visual search loop started")
    cfg.snap_dir.mkdir(parents=True, exist_ok=True)

    OPENAI_URL  = "https://api.openai.com/v1/chat/completions"
    SERVER_BASE = "https://studio--sky-pointer.us-central1.hosted.app"

    # Resolve camera tool once
    tool = shutil.which("rpicam-still") or shutil.which("libcamera-still")
    if not tool:
        logger.info("[DETECT] no snapshot tool found (install rpicam-apps/libcamera-still).")
        return

    # Resolve API key once
    api_key = (getattr(cfg, "openai_api_key", "") or os.getenv("OPENAI_API_KEY", "")).strip()
    if not api_key:
        logger.info("[OPENAI] missing API key (cfg.openai_api_key or env OPENAI_API_KEY).")
        return
    openai_headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    json_headers   = {"Content-Type": "application/json"}

    # Reuse HTTP sessions
    async with ClientSession(timeout=ClientTimeout(total=12), headers=openai_headers) as s_ai, \
               ClientSession(timeout=ClientTimeout(total=10)) as s_srv:

        while not state.shutting_down:
            await asyncio.sleep(max(1.0, float(getattr(state.detect_cfg, "period_s", 5.0))))

            dc, ds = state.detect_cfg, state.detect_state
            if not dc.enabled:
                ds.local_search_locked = False
                ds.consecutive_positives = 0
                continue
            if ds.local_search_locked:
                continue

            # ---- Snapshot ----
            tmp = cfg.snap_dir / (cfg.snap_name + ".tmp")
            out = cfg.snap_dir / cfg.snap_name
            cmd = [
                tool, "-n", "-t", "1",
                "--width", str(max(cfg.cam_width, 640)),
                "--height", str(max(cfg.cam_height, 360)),
                "--rotation", str(cfg.cam_rotation_deg),
                "-q", "85", "-o", str(tmp),
            ]
            try:
                p = await asyncio.create_subprocess_exec(
                    *cmd, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL
                )
                await asyncio.wait_for(p.wait(), timeout=6.0)
                if p.returncode != 0 or not tmp.exists():
                    continue
                tmp.replace(out)
                img = out.read_bytes()
            except Exception as e:
                logger.debug(f"[DETECT] snapshot error/timeout: {e}")
                continue

            # ---- Ask OpenAI for strict True/False ----
            data_uri = "data:image/jpeg;base64," + base64.b64encode(img).decode("ascii")
            body = {
                "model": getattr(cfg, "openai_model", "gpt-4o-mini"),
                "temperature": 0,
                "messages": [{
                    "role": "user",
                    "content": [
                        {"type": "text",
                         "text": f"Does this photo contain '{dc.query}'? "
                                 "Answer strictly 'True' or 'False' with no other text."},
                        {"type": "image_url", "image_url": {"url": data_uri}},
                    ],
                }],
            }

            try:
                async with s_ai.post(OPENAI_URL, json=body) as r:
                    if r.status >= 400:
                        logger.info(f"[OPENAI] HTTP {r.status}: {(await r.text())[:200]}")
                        continue
                    data = await r.json()
                raw = (data.get("choices", [{}])[0]
                          .get("message", {})
                          .get("content", "") or "").strip().lower()
                # Robust to yes/no, but we expect exactly true/false
                present = raw.startswith("true") or raw.startswith("yes")
            except Exception as e:
                logger.info(f"[OPENAI] error: {e}")
                continue

            # Streak logic (boolean only)
            ds.consecutive_positives = (ds.consecutive_positives + 1) if present else 0
            logger.info(f"[DETECT] '{dc.query}': present={present} "
                        f"(streak {ds.consecutive_positives}/{dc.confirmations})")

            if ds.consecutive_positives < dc.confirmations:
                continue

            # ---- Post hit & lock ----
            ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
            try:
                (cfg.snap_dir / f"hit_{ts}.jpg").write_bytes(img)
            except Exception:
                pass

            lat = state.latest_gps.lat if math.isfinite(state.latest_gps.lat) else 0.0
            lng = state.latest_gps.lon if math.isfinite(state.latest_gps.lon) else 0.0
            img_uri = "data:image/jpeg;base64," + base64.b64encode(img).decode("ascii")

            try:
                await asyncio.gather(
                    s_srv.post(SERVER_BASE + "/api/target-location",
                               json={"lat": float(f"{lat:.6f}"), "lng": float(f"{lng:.6f}")},
                               headers=json_headers),
                    s_srv.post(SERVER_BASE + "/api/target-image",
                               json={"image": img_uri},
                               headers=json_headers),
                )
            except Exception as e:
                logger.info(f"[HIT] post failed: {e}")
            finally:
                ds.local_search_locked = True
                ds.consecutive_positives = 0
                logger.info("[DETECT] HIT posted; search locally locked until website toggles OFF/ON.")

# ---------------------------
# Main
# ---------------------------
def setup_logging(debug: bool) -> logging.Logger:
    logging.basicConfig(
        level=(logging.DEBUG if debug else logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    return logging.getLogger("airplane_pi")

async def main_async():
    cfg = Config()
    logger = setup_logging(cfg.debug)
    state = RuntimeState()

    uart_tx_q: asyncio.Queue[str] = asyncio.Queue(maxsize=100)

    # Ensure snapshot directory exists
    try:
        cfg.snap_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass

    # Signals
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, lambda s=sig: setattr(state, "shutting_down", True))
        except NotImplementedError:
            pass

    # --- start tasks with per-step delay before each start ---
    tasks = []

    # Each tuple: (delay_before_seconds, start_task_callable)
    startup_steps = [
        (5.0, lambda: asyncio.create_task(gps_reader(state, cfg, logger))),
        (3.0, lambda: asyncio.create_task(file_logger(state, cfg, logger))),
        (1.0, lambda: asyncio.create_task(website_poller(state, uart_tx_q, cfg, logger))),
        # NEW: visual search loop (after website_poller so config is ready)
        (0.8, lambda: asyncio.create_task(visual_search_loop(state, cfg, logger))),
        (0.5, lambda: asyncio.create_task(web_poster(state, cfg, logger))),
        (0.5, lambda: asyncio.create_task(esp32_uart_rx(state, cfg, logger))),
        (0.5, lambda: asyncio.create_task(esp32_uart_tx(uart_tx_q, state, cfg, logger))),
        (0.5, lambda: asyncio.create_task(stream_manager(state, cfg, logger))),
    ]

    for delay_before, start_fn in startup_steps:
        if delay_before > 0:
            await asyncio.sleep(delay_before)
        tasks.append(start_fn())

    logger.info("airplane_pi started. Press Ctrl+C to stop.")
    try:
        while not state.shutting_down:
            await asyncio.sleep(0.5)
    finally:
        logger.info("Shutting down...")
        await asyncio.sleep(0.3)
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        logger.info("Goodbye.")

def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
