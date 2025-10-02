
#!/usr/bin/env python3
"""
airplane_pi.py — runnable dummy skeleton (single-file) for your Pi process.

- Async tasks simulate: GPS read, UART RX/TX, website poll, stream manager,
  5 Hz file logger, 1 Hz web poster, and boot-time log rotation + fake git push.
- Everything is hardware-agnostic so it runs on any machine.
- Replace the "SIMULATION" sections with your real I/O later.
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
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

# ---------------------------
# Configuration (env-backed)
# ---------------------------
@dataclass
class Config:
    log_dir: Path = Path("./log")
    log_current: str = "flight_current.csv"
    git_remote: str = os.getenv("AIRPLANE_GIT_REMOTE", "origin")
    api_base: str = os.getenv("AIRPLANE_API_BASE", "https://example.com")  # dummy
    youtube_key: str = os.getenv("AIRPLANE_YT_KEY", "DUMMY-KEY")
    uart_dev: str = os.getenv("AIRPLANE_UART_DEV", "/dev/ttyS0")
    uart_baud: int = int(os.getenv("AIRPLANE_UART_BAUD", "115200"))
    gps_dev: str = os.getenv("AIRPLANE_GPS_DEV", "/dev/ttyAMA0")
    poll_period_s: float = float(os.getenv("AIRPLANE_POLL_PERIOD_S", "1.0"))
    log_rate_hz: float = float(os.getenv("AIRPLANE_LOG_RATE_HZ", "5.0"))
    poster_rate_hz: float = float(os.getenv("AIRPLANE_POST_RATE_HZ", "1.0"))
    debug: bool = os.getenv("AIRPLANE_DEBUG", "1") == "1"


# ---------------------------
# Data structures
# ---------------------------
@dataclass
class LatLon:
    lat: float = math.nan
    lon: float = math.nan

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

@dataclass
class RuntimeState:
    stream_enabled: bool = False
    target_points: List[LatLon] = field(default_factory=list)
    latest_gps: GpsFix = field(default_factory=GpsFix)
    latest_esp32: Telemetry = field(default_factory=Telemetry)
    shutting_down: bool = False


# ---------------------------
# Utilities
# ---------------------------
def utc_ts() -> float:
    return time.time()

def utc_iso(ts: Optional[float] = None) -> str:
    dt = datetime.fromtimestamp(ts if ts is not None else time.time(), tz=timezone.utc)
    return dt.isoformat()

def ensure_log_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)

def rotate_and_git_push(cfg: Config, logger: logging.Logger) -> None:
    """
    Rotate log/flight_current.csv to a timestamped name (if exists), then try git add/commit/push.
    Safe no-op if nothing to rotate or no git repo is present.
    """
    ensure_log_dir(cfg.log_dir)
    current = cfg.log_dir / cfg.log_current
    if current.exists():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        rotated = cfg.log_dir / f"flight_{ts}.csv"
        try:
            current.rename(rotated)
            logger.info(f"Rotated log to {rotated}")
        except Exception as e:
            logger.warning(f"Failed to rotate log: {e}")

        # Try git add/commit/push if .git exists. This is best-effort.
        if (Path(".") / ".git").exists():
            try:
                import subprocess
                subprocess.run(["git", "add", str(rotated)], check=False)
                subprocess.run(["git", "commit", "-m", f"Flight log {rotated.name}"], check=False)
                subprocess.run(["git", "push", cfg.git_remote, "HEAD"], check=False)
                logger.info("Attempted git push of rotated log.")
            except Exception as e:
                logger.warning(f"Git push failed: {e}")
        else:
            logger.info("No .git directory; skipping git push.")
    else:
        logger.info("No existing current log to rotate.")


# ---------------------------
# Tasks (SIMULATION versions)
# ---------------------------
async def gps_reader(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    SIMULATION: generate a plausible GPS fix around Aarhus, DK every 0.2 s.
    Replace with real NMEA parsing later.
    """
    lat0, lon0 = 56.1629, 10.2039
    while not state.shutting_down:
        await asyncio.sleep(0.2)
        jitter_lat = (random.random() - 0.5) * 1e-4
        jitter_lon = (random.random() - 0.5) * 1e-4
        state.latest_gps = GpsFix(
            time_utc=utc_ts(),
            lat=lat0 + jitter_lat,
            lon=lon0 + jitter_lon,
            alt_m=60.0 + (random.random() - 0.5) * 5.0,
            fix_ok=True,
            sats=10 + random.randint(-2, 3),
        )


async def esp32_uart_rx(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    SIMULATION: synthesize telemetry at ~20 Hz.
    Replace with real UART reads + parsing later.
    """
    t0 = utc_ts()
    while not state.shutting_down:
        await asyncio.sleep(0.05)  # 20 Hz
        t = utc_ts() - t0
        state.latest_esp32 = Telemetry(
            time_utc=utc_ts(),
            batt_v=11.1 + 0.2 * math.sin(t / 10),
            airspeed_ms=10 + 2 * math.sin(t),
            groundspeed_ms=8 + 1.5 * math.sin(t / 2),
            roll_deg=10 * math.sin(t * 0.7),
            pitch_deg=5 * math.sin(t * 0.5),
            yaw_deg=(t * 20) % 360,
            lat=state.latest_gps.lat,
            lon=state.latest_gps.lon,
            alt_m=state.latest_gps.alt_m,
        )


async def esp32_uart_tx(uart_tx_q: asyncio.Queue, state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    SIMULATION: print outbound commands instead of writing to UART.
    Replace with serial.write(...) later.
    """
    while not state.shutting_down:
        try:
            msg = await asyncio.wait_for(uart_tx_q.get(), timeout=0.5)
        except asyncio.TimeoutError:
            continue
        logger.info(f"[UART->ESP32] {msg!r}")
        uart_tx_q.task_done()


async def website_poller(state: RuntimeState, uart_tx_q: asyncio.Queue, cfg: Config, logger: logging.Logger):
    """
    SIMULATION: every poll toggles stream flag every ~15 s and updates targets with a simple square.
    Replace with real HTTP GET to your API.
    """
    toggle_every = 15.0
    last_toggle = utc_ts()
    k = 0
    while not state.shutting_down:
        await asyncio.sleep(cfg.poll_period_s)
        now = utc_ts()
        # Simulate stream toggle
        if now - last_toggle > toggle_every:
            state.stream_enabled = not state.stream_enabled
            last_toggle = now
            logger.info(f"[API] stream_enabled -> {state.stream_enabled}")
        # Simulate a target set that changes slowly
        k += 1
        base_lat, base_lon = 56.1629, 10.2039
        square = [
            LatLon(base_lat + 0.001, base_lon + 0.001),
            LatLon(base_lat + 0.001, base_lon - 0.001),
            LatLon(base_lat - 0.001, base_lon - 0.001),
            LatLon(base_lat - 0.001, base_lon + 0.001),
        ]
        state.target_points = square
        # Enqueue a transmit to ESP32 occasionally
        if k % 5 == 0:
            line = "TARGETS:" + ";".join(f"{p.lat:.6f},{p.lon:.6f}" for p in state.target_points)
            await uart_tx_q.put(line)


async def stream_manager(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    SIMULATION: no real ffmpeg; just maintain a 'running' flag.
    Replace with subprocess management to run ffmpeg with your RTMPS URL.
    """
    running = False
    while not state.shutting_down:
        await asyncio.sleep(0.2)
        if state.stream_enabled and not running:
            running = True
            logger.info("[STREAM] Starting simulated stream...")
        elif not state.stream_enabled and running:
            running = False
            logger.info("[STREAM] Stopping simulated stream...")
    # On shutdown
    if running:
        logger.info("[STREAM] Stopping simulated stream (shutdown).")


async def file_logger(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    Write CSV rows at ~5 Hz with selected telemetry.
    """
    ensure_log_dir(cfg.log_dir)
    path = cfg.log_dir / cfg.log_current
    # Create with header if new
    new_file = not path.exists()
    f = path.open("a", newline="")
    writer = csv.writer(f)
    if new_file:
        writer.writerow([
            "ts_iso","batt_v","airspeed_ms","groundspeed_ms",
            "roll_deg","pitch_deg","yaw_deg","lat","lon","alt_m"
        ])
        f.flush()

    period = 1.0 / max(cfg.log_rate_hz, 1.0)
    try:
        while not state.shutting_down:
            t_start = time.perf_counter()
            te = state.latest_esp32
            writer.writerow([
                utc_iso(te.time_utc), f"{te.batt_v:.3f}", f"{te.airspeed_ms:.3f}", f"{te.groundspeed_ms:.3f}",
                f"{te.roll_deg:.3f}", f"{te.pitch_deg:.3f}", f"{te.yaw_deg:.3f}",
                f"{te.lat:.6f}", f"{te.lon:.6f}", f"{te.alt_m:.2f}",
            ])
            f.flush()
            dt = period - (time.perf_counter() - t_start)
            if dt > 0:
                await asyncio.sleep(dt)
    finally:
        f.close()


async def web_poster(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    SIMULATION: log a JSON payload once per second instead of real HTTP POST.
    Replace with aiohttp/requests to send to your endpoint.
    """
    period = 1.0 / max(cfg.poster_rate_hz, 1.0)
    while not state.shutting_down:
        await asyncio.sleep(period)
        te = state.latest_esp32
        gp = state.latest_gps
        payload = {
            "ts": utc_iso(),
            "battery_v": round(te.batt_v, 3),
            "gps": {"lat": round(gp.lat, 6), "lon": round(gp.lon, 6), "alt_m": round(gp.alt_m, 1), "sats": gp.sats, "fix_ok": gp.fix_ok},
            "stream": state.stream_enabled,
        }
        logger.info(f"[POST] {json.dumps(payload)}")


async def health_task(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    Periodic heartbeat every ~5 s.
    """
    while not state.shutting_down:
        await asyncio.sleep(5.0)
        logger.debug("[HEALTH] alive")


# ---------------------------
# Main entry
# ---------------------------
def setup_logging(debug: bool) -> logging.Logger:
    lvl = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=lvl,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    return logging.getLogger("airplane_pi")

async def main_async():
    cfg = Config()
    logger = setup_logging(cfg.debug)
    state = RuntimeState()

    # Boot housekeeping
    rotate_and_git_push(cfg, logger)

    # Queues
    uart_tx_q: asyncio.Queue[str] = asyncio.Queue(maxsize=100)

    # Install signal handlers
    loop = asyncio.get_running_loop()
    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(s, lambda s=s: setattr(state, "shutting_down", True))
        except NotImplementedError:
            # Windows / restricted env
            pass

    # Start tasks
    tasks = [
        asyncio.create_task(gps_reader(state, cfg, logger)),
        asyncio.create_task(esp32_uart_rx(state, cfg, logger)),
        asyncio.create_task(esp32_uart_tx(uart_tx_q, state, cfg, logger)),
        asyncio.create_task(website_poller(state, uart_tx_q, cfg, logger)),
        asyncio.create_task(stream_manager(state, cfg, logger)),
        asyncio.create_task(file_logger(state, cfg, logger)),
        asyncio.create_task(web_poster(state, cfg, logger)),
        asyncio.create_task(health_task(state, cfg, logger)),
    ]

    logger.info("airplane_pi started. Press Ctrl+C to stop.")
    try:
        while not state.shutting_down:
            await asyncio.sleep(0.5)
    finally:
        logger.info("Shutting down...")
        # Allow tasks to see shutting_down and finish
        await asyncio.sleep(0.5)
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
