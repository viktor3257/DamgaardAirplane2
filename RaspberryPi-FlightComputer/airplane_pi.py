#!/usr/bin/env python3
"""
airplane_pi.py — async process for Raspberry Pi.

Components:
- GPS reader (SIM7600G-H NMEA → lat/lon, heading, ground speed)
- ESP32 UART link (bi-directional nav + telemetry)
- Web poster (POSTs location/telemetry to your API)
- Website poller (stream-state, circling-point, search-status)
- Stream state manager (RTMP + periodic JPEG snapshot for vision)
- CSV logger
- Visual search loop using OpenAI API (reads JPEG from stream when available,
  falls back to still capture when stream is off — with camera-lock to avoid races)

This version:
- Stream snapshots at 2 Hz and scale snapshots so the shortest side is 768 px
  (matches OpenAI `detail:"high"` internal resize; avoids server-side rescale).
- Visual search prompts at 2 Hz (period_s = 0.5).
- Uses separate files for stream snapshots vs. still captures.
- Serializes camera access to avoid contention between ffmpeg and rpicam-still.
- Streams JSON telemetry to/from ESP32 over UART with auto-reconnect.
"""

import os
import sys
import csv
import time
import json
import math
import signal
import asyncio
import logging
import subprocess
import aiohttp
import base64
import shutil
from dataclasses import dataclass, field
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional

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

    # API
    api_base: str = field(default_factory=lambda: os.getenv("AIRPLANE_API_BASE", "https://studio--sky-pointer.us-central1.hosted.app").rstrip("/"))

    # Streaming
    rtmp_url: str = "rtmp://a.rtmp.youtube.com/live2"
    rtmp_key: str = field(default_factory=lambda: os.getenv("AIRPLANE_RTMP_KEY", "bq45-5jky-84s0-cy0y-e8uy"))
    cam_width: int = 1280
    cam_height: int = 720
    cam_fps: int = 30
    cam_rotation_deg: int = 180   # 180 to flip image
    cam_bitrate_kbps: int = 3500  # tune if needed

    # Snapshots for visual search
    snap_dir: Path = BASE_DIR / "snaps"
    # Use distinct filenames to avoid collisions/races
    snap_stream_name: str = "stream.jpg"  # ffmpeg -update 1 writes here
    snap_still_name: str  = "still.jpg"   # rpicam-still writes here

    # OpenAI (vision)
    openai_api_key: str = field(default_factory=lambda: os.getenv("OPENAI_API_KEY", "").strip())
    openai_model: str = "gpt-4o-mini"  # vision-capable & fast

@dataclass
class DetectConfig:
    enabled: bool = False
    query: str = ""
    period_s: float = 0.5         # 2 Hz prompts
    confirmations: int = 1        # consecutive positives required

@dataclass
class DetectState:
    local_search_locked: bool = False  # set True after a hit; cleared when website disables search
    consecutive_positives: int = 0
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
    current_a: float = math.nan
    current_sensor_v: float = math.nan

@dataclass
class RuntimeState:
    stream_enabled: bool = False
    target_points: List[LatLon] = field(default_factory=list)
    latest_gps: GpsFix = field(default_factory=GpsFix)
    latest_esp32: Telemetry = field(default_factory=Telemetry)
    shutting_down: bool = False

    # Visual search
    detect_cfg: DetectConfig = field(default_factory=DetectConfig)
    detect_state: DetectState = field(default_factory=DetectState)

    # Camera coordination
    camera_lock: asyncio.Lock = field(default_factory=asyncio.Lock)  # serialize camera open
    stream_status: str = "off"  # "off" | "starting" | "on" | "error"

# ---------------------------
# ESP32 UART helper
# ---------------------------
class Esp32SerialLink:
    """Shared UART connection with automatic reconnect."""

    def __init__(self, state: RuntimeState, cfg: Config, logger: logging.Logger):
        self._state = state
        self._cfg = cfg
        self._logger = logger
        self._serial = None
        self._lock = asyncio.Lock()

    async def get_serial(self):
        import serial  # local import to keep dependency scoped

        while not self._state.shutting_down:
            if self._serial and self._serial.is_open:
                return self._serial

            async with self._lock:
                if self._serial and self._serial.is_open:
                    return self._serial
                try:
                    ser = serial.Serial(
                        self._cfg.uart_dev,
                        self._cfg.uart_baud,
                        timeout=0,
                        write_timeout=1.0,
                    )
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    self._serial = ser
                    self._logger.info(
                        f"[UART] Connected to {self._cfg.uart_dev} @ {self._cfg.uart_baud}"
                    )
                    return self._serial
                except Exception as e:
                    self._logger.info(
                        f"[UART] open {self._cfg.uart_dev} failed: {e}; retrying in 2 s"
                    )
                    await asyncio.sleep(2.0)

        raise RuntimeError("Shutting down")

    def drop_connection(self):
        ser = self._serial
        self._serial = None
        if ser:
            try:
                ser.close()
            except Exception:
                pass


class Esp32UartBridge:
    """Encapsulates UART RX/TX flows for the ESP32 link."""

    def __init__(
        self,
        state: RuntimeState,
        cfg: Config,
        link: Esp32SerialLink,
        logger: logging.Logger,
    ) -> None:
        self._state = state
        self._cfg = cfg
        self._link = link
        self._logger = logger
        self._buffer = bytearray()
        self._send_period = 0.2
        self._last_send = 0.0
        self._send_event = asyncio.Event()

    def request_nav_send(self) -> None:
        """Schedule an immediate nav payload transmission."""
        self._send_event.set()

    def _finite_or_none(self, value: Optional[float]) -> Optional[float]:
        if isinstance(value, (int, float)) and math.isfinite(value):
            return float(value)
        return None

    def _build_nav_payload(self) -> Dict[str, Any]:
        target_lat = target_lon = None
        if self._state.target_points:
            target_lat = self._state.target_points[0].lat
            target_lon = self._state.target_points[0].lon

        payload: Dict[str, Any] = {
            "type": "nav",
            "timestamp": time.time(),
        }

        for key, value in (
            ("lat", self._finite_or_none(self._state.latest_gps.lat)),
            ("lon", self._finite_or_none(self._state.latest_gps.lon)),
            ("target_lat", self._finite_or_none(target_lat)),
            ("target_lon", self._finite_or_none(target_lon)),
            ("heading_deg", self._finite_or_none(self._state.latest_esp32.yaw_deg)),
            (
                "groundspeed_ms",
                self._finite_or_none(self._state.latest_esp32.groundspeed_ms),
            ),
        ):
            if value is not None:
                payload[key] = value

        return payload

    def _apply_telemetry(self, doc: Dict[str, Any]) -> None:
        tel = self._state.latest_esp32
        now = time.time()
        tel.time_utc = now

        if "mode" in doc:
            try:
                tel.mode = int(doc["mode"])
            except Exception:
                pass

        for key, attr in (
            ("battery_pct", "battery_pct"),
            ("battery_v", "batt_v"),
            ("airspeed_ms", "airspeed_ms"),
            ("groundspeed_ms", "groundspeed_ms"),
            ("pitch_deg", "pitch_deg"),
            ("roll_deg", "roll_deg"),
            ("heading_deg", "yaw_deg"),
            ("height_m", "alt_m"),
            ("lat", "lat"),
            ("lon", "lon"),
        ):
            val = doc.get(key)
            if isinstance(val, (int, float)) and math.isfinite(val):
                setattr(tel, attr, float(val))

        if isinstance(doc.get("current_a"), (int, float)):
            tel.current_a = float(doc["current_a"])
        if isinstance(doc.get("current_sensor_v"), (int, float)):
            tel.current_sensor_v = float(doc["current_sensor_v"])

        self._state.latest_esp32 = tel

    async def run_rx(self) -> None:
        """Consume telemetry frames emitted by the ESP32."""

        while not self._state.shutting_down:
            try:
                ser = await self._link.get_serial()
            except RuntimeError:
                break

            try:
                data = ser.read(ser.in_waiting or 128)
            except Exception as e:
                self._logger.info(f"[UART<-ESP32] read error: {e}")
                self._link.drop_connection()
                await asyncio.sleep(0.5)
                continue

            if not data:
                await asyncio.sleep(0.05)
                continue

            self._buffer.extend(data)
            while b"\n" in self._buffer:
                line, _, remainder = self._buffer.partition(b"\n")
                self._buffer = bytearray(remainder)
                text = line.strip().decode(errors="ignore")
                if not text:
                    continue
                try:
                    doc = json.loads(text)
                except json.JSONDecodeError:
                    self._logger.debug(
                        f"[UART<-ESP32] ignoring non-JSON line: {text}"
                    )
                    continue

                if isinstance(doc, dict):
                    self._apply_telemetry(doc)

    async def run_tx(self) -> None:
        """Stream navigation data (GPS + targets) to the ESP32."""

        while not self._state.shutting_down:
            try:
                ser = await self._link.get_serial()
            except RuntimeError:
                break

            wait_time = max(0.0, self._send_period - (time.time() - self._last_send))
            if wait_time > 0:
                try:
                    await asyncio.wait_for(self._send_event.wait(), timeout=wait_time)
                except asyncio.TimeoutError:
                    pass

            triggered = self._send_event.is_set()
            if triggered:
                self._send_event.clear()

            if (time.time() - self._last_send) < self._send_period and not triggered:
                continue

            payload = self._build_nav_payload()

            try:
                encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False) + "\n"
            except (TypeError, ValueError):
                self._logger.debug(
                    "[UART->ESP32] nav payload had non-serializable values; skipping"
                )
                self._last_send = time.time()
                continue

            try:
                ser.write(encoded.encode("utf-8"))
                ser.flush()
                self._last_send = time.time()
            except Exception as e:
                self._logger.info(f"[UART->ESP32] write error: {e}")
                self._link.drop_connection()
                await asyncio.sleep(0.5)


# ---------------------------
# GPS (SIM7600G-H via NMEA)
# ---------------------------
async def gps_reader(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """Read NMEA sentences and update GPS/telemetry speed/heading."""
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
# Website poller
# ---------------------------
async def website_poller(
    state: RuntimeState,
    uart_bridge: Esp32UartBridge,
    cfg: Config,
    logger: logging.Logger,
):
    """
    Poll every cfg.poll_period_s:
      - GET /api/stream-state -> update stream_enabled
      - GET /api/circling-point -> set target_points and trigger a UART nav update on change
      - GET /api/search-status -> update visual search config (enabled/query)
    """
    period = cfg.poll_period_s
    last_sent_point = None  # (lat, lon)

    async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=5)) as session:
        while not state.shutting_down:
            await asyncio.sleep(period)

            # stream-state
            try:
                resp = await session.get(cfg.api_base + "/api/stream-state")
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
                resp = await session.get(cfg.api_base + "/api/circling-point")
                if resp.status == 200:
                    data = await resp.json(content_type=None)
                    lat = data.get("lat")
                    lng = data.get("lng")
                    if isinstance(lat, (int, float)) and isinstance(lng, (int, float)):
                        state.target_points = [LatLon(float(lat), float(lng))]
                        key = (float(lat), float(lng))
                        if key != last_sent_point:
                            uart_bridge.request_nav_send()
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
                resp = await session.get(cfg.api_base + "/api/search-status")
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
# Stream manager (hardened)
# ---------------------------
async def stream_manager(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    Start/stop YouTube stream based on state.stream_enabled.
    rpicam-vid (H.264) -> ffmpeg (copy video + AAC) -> RTMP
    and a periodic JPEG snapshot for the vision loop.
    Captures child stderr; backs off on repeated failures.

    Uses a camera_lock to avoid startup races with still-capture.
    Updates state.stream_status: "off" | "starting" | "on" | "error".
    """
    import threading
    import shutil
    import time
    import subprocess

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
        "--intra", str(cfg.cam_fps * 2),
        "-t", "0",
        "-o", "-",
    ]

    cfg.snap_dir.mkdir(parents=True, exist_ok=True)
    snap_file = cfg.snap_dir / cfg.snap_stream_name
    snap_path = str(snap_file)  # <-- stream snapshot file

    # Remove any stale file so ffmpeg can overwrite cleanly even on strict builds
    try:
        if snap_file.exists():
            snap_file.unlink()
    except Exception as e:
        logger.debug(f"[STREAM] could not delete stale snapshot: {e}")

    # Snapshot branch:
    # - 2 frames per second
    # - scale so SHORTEST side is 768 (for 1280x720 this sets height=768, width≈1365)
    # - overwrite same file each time
    ffmpeg_cmd = [
        "ffmpeg", "-y",                 # <-- force overwrite existing files
        "-hide_banner", "-loglevel", "warning",

        # input 0: silent audio (keeps YT happy)
        "-thread_queue_size", "1024",
        "-f", "lavfi", "-i", "anullsrc=channel_layout=stereo:sample_rate=44100",

        # input 1: h264 elementary stream from rpicam-vid
        "-thread_queue_size", "1024",
        "-fflags", "+genpts",
        "-use_wallclock_as_timestamps", "1",
        "-i", "pipe:0",

        # Output #1: RTMP (video copy + AAC)
        "-map", "0:a", "-map", "1:v",
        "-c:v", "copy",
        "-c:a", "aac", "-b:a", "128k",
        "-f", "flv", "-rtmp_live", "live", f"{cfg.rtmp_url}/{cfg.rtmp_key}",

        # Output #2: periodic JPEG snapshots for detector (2 Hz, shortest side = 768)
        "-map", "1:v",
        "-vf", "fps=2,scale=-1:768",
        "-c:v", "mjpeg",
        "-q:v", "5",
        "-f", "image2",
        "-update", "1",
        snap_path,
    ]

    cam_proc = None
    ffmpeg_proc = None
    running = False
    backoff_s = 1.0

    def _pump_stderr(pipe, tag):
        try:
            for line in iter(pipe.readline, b""):
                txt = line.decode("utf-8", "ignore").strip()
                if txt:
                    logger.warning(f"{tag}: {txt}")
        except Exception:
            pass

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
        time.sleep(0.5)
        for p in (ffmpeg_proc, cam_proc):
            try:
                if p and p.poll() is None:
                    p.kill()
            except Exception:
                pass
        cam_proc, ffmpeg_proc = None, None
        running = False
        state.stream_status = "off"

    def start_pipeline_locked():
        """Start child processes. Must be called with camera_lock held."""
        nonlocal cam_proc, ffmpeg_proc, running
        try:
            cam_proc = subprocess.Popen(
                cam_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
            ffmpeg_proc = subprocess.Popen(
                ffmpeg_cmd,
                stdin=cam_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                bufsize=0
            )

            # stderr readers
            threading.Thread(target=_pump_stderr, args=(cam_proc.stderr, "[rpicam-vid]"), daemon=True).start()
            threading.Thread(target=_pump_stderr, args=(ffmpeg_proc.stderr, "[ffmpeg]"), daemon=True).start()

            running = True
            state.stream_status = "on"
            logger.info("[STREAM] started (rpicam-vid → ffmpeg → RTMP + JPEG @ 2 Hz, shortest side 768)")
        except Exception as e:
            state.stream_status = "error"
            logger.info(f"[STREAM] failed to start: {e}")
            stop_pipeline()

    try:
        while not state.shutting_down:
            await asyncio.sleep(0.25)

            # start
            if state.stream_enabled and not running:
                state.stream_status = "starting"
                # Hold the camera lock briefly while we open the camera to avoid races
                got = False
                try:
                    got = await asyncio.wait_for(state.camera_lock.acquire(), timeout=1.0)
                except Exception:
                    got = False
                try:
                    if got:
                        start_pipeline_locked()
                    else:
                        logger.info("[STREAM] could not acquire camera_lock; will retry soon")
                finally:
                    if state.camera_lock.locked():
                        state.camera_lock.release()

                backoff_s = 1.0  # reset backoff after an attempt

            # stop
            elif not state.stream_enabled and running:
                stop_pipeline()

            # supervise
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
                    logger.info(f"[STREAM] pipeline exited; restarting in {backoff_s:.1f}s")
                    stop_pipeline()
                    await asyncio.sleep(backoff_s)
                    backoff_s = min(backoff_s * 2, 30.0)
                    # Try to restart on next loop iteration

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
    url = cfg.api_base + "/api/drone-location"
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
      - If stream is ON/starting: read JPEG written by ffmpeg (no camera contention)
      - Else: still-capture (rpicam-still/libcamera-still) if available (guarded by camera_lock)
      - Ask OpenAI if the image contains `detect_cfg.query` (strict True/False)
      - On `confirmations` consecutive True -> post GPS + image and lock locally
    """
    from aiohttp import ClientSession, ClientTimeout

    logger.info("[DETECT] visual search loop started")
    cfg.snap_dir.mkdir(parents=True, exist_ok=True)

    OPENAI_URL  = "https://api.openai.com/v1/chat/completions"

    # Still-capture tool is optional (only required when stream is OFF)
    tool = shutil.which("rpicam-still") or shutil.which("libcamera-still")
    if not tool:
        logger.info("[DETECT] no still-capture tool found. Will use stream snapshots when available.")

    # Resolve API key (cfg or env)
    api_key = (cfg.openai_api_key or os.getenv("OPENAI_API_KEY", "")).strip()
    if not api_key:
        logger.error("[OPENAI] missing API key (OPENAI_API_KEY).")
        return
    openai_headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    json_headers   = {"Content-Type": "application/json"}

    snap_stream = cfg.snap_dir / cfg.snap_stream_name  # ffmpeg writes here
    snap_still  = cfg.snap_dir / cfg.snap_still_name   # we write here

    # Reuse HTTP sessions
    async with ClientSession(timeout=ClientTimeout(total=12), headers=openai_headers) as s_ai, \
               ClientSession(timeout=ClientTimeout(total=10)) as s_srv:

        while not state.shutting_down:
            # Allow sub-second cadence; clamp to avoid zero/negative
            period = float(getattr(state.detect_cfg, "period_s", 0.5))
            await asyncio.sleep(max(0.05, period))

            dc, ds = state.detect_cfg, state.detect_state
            if not dc.enabled:
                ds.local_search_locked = False
                ds.consecutive_positives = 0
                continue
            if ds.local_search_locked:
                continue

            # ---- Snapshot (safe hybrid) ----
            img = None
            prefer_stream = (getattr(state, "stream_status", "off") in ("starting", "on")) \
                            or getattr(state, "stream_enabled", False)

            if prefer_stream:
                # Only read from stream snapshot; never still-capture while stream is on/starting
                try:
                    if snap_stream.exists():
                        last_mt = getattr(visual_search_loop, "_last_mt", 0.0)
                        cur_mt = snap_stream.stat().st_mtime
                        if cur_mt > last_mt:
                            img = snap_stream.read_bytes()
                            visual_search_loop._last_mt = cur_mt
                        else:
                            # no new frame yet
                            continue
                    else:
                        # stream likely just starting; wait for first frame
                        continue
                except Exception as e:
                    logger.debug(f"[DETECT] stream snapshot read failed: {e}")
                    continue
            else:
                # Stream is explicitly off -> try still-capture guarded by camera_lock
                if not tool:
                    continue
                got = False
                try:
                    got = await asyncio.wait_for(state.camera_lock.acquire(), timeout=0.05)
                except Exception:
                    got = False
                try:
                    if not got:
                        # Someone is using camera; skip this cycle
                        continue
                    tmp = snap_still.with_suffix(snap_still.suffix + ".tmp")
                    # Match the same size policy as stream snapshots (shortest side 768)
                    cmd = [
                        tool, "-n", "-t", "1",
                        "--width", str(max(cfg.cam_width, 640)),
                        "--height", str(max(cfg.cam_height, 360)),
                        "--rotation", str(cfg.cam_rotation_deg),
                        "-q", "85", "-o", str(tmp),
                        # rpicam-still doesn't scale like ffmpeg's -vf; we keep native and let API upscale.
                        # If you want local scaling here, swap to `libcamera-still` with `--width/--height` set to ~1365x768.
                    ]
                    pcap = await asyncio.create_subprocess_exec(
                        *cmd, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL
                    )
                    await asyncio.wait_for(pcap.wait(), timeout=6.0)
                    if pcap.returncode == 0 and tmp.exists():
                        tmp.replace(snap_still)
                        img = snap_still.read_bytes()
                except Exception as e:
                    logger.debug(f"[DETECT] still snapshot error/timeout: {e}")
                finally:
                    if state.camera_lock.locked():
                        state.camera_lock.release()

            if not img:
                continue

            # ---- Ask OpenAI for strict True/False (force high detail) ----
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
                        {"type": "image_url", "image_url": {"url": data_uri, "detail": "high"}},
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
                    s_srv.post(cfg.api_base + "/api/target-location",
                               json={"lat": float(f"{lat:.6f}"), "lng": float(f"{lng:.6f}")},
                               headers=json_headers),
                    s_srv.post(cfg.api_base + "/api/target-image",
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

    esp32_link = Esp32SerialLink(state, cfg, logger)
    esp32_bridge = Esp32UartBridge(state, cfg, esp32_link, logger)

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
        (1.0, lambda: asyncio.create_task(website_poller(state, esp32_bridge, cfg, logger))),
        (0.8, lambda: asyncio.create_task(visual_search_loop(state, cfg, logger))),  # after poller
        (0.5, lambda: asyncio.create_task(web_poster(state, cfg, logger))),
        (0.5, lambda: asyncio.create_task(esp32_bridge.run_rx())),
        (0.5, lambda: asyncio.create_task(esp32_bridge.run_tx())),
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
