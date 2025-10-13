#!/usr/bin/env python3
"""
airplane_pi.py — simple, self-contained async skeleton for the Pi.

What it does:
- Simulates GPS, UART RX/TX, a "website" poller, a stream flag manager,
  a CSV logger, a 1 Hz "web post" printer, and a heartbeat.
- The CSV logger:
    * On entry: finds the most recent previous flight log, renames it to the
      current UTC time (e.g., 2025-10-13_09-58-22Z.csv), commits+pushes it,
      and deletes it locally on success.
    * Creates a new per-boot log: log/flight_YYYYmmdd_HHMMSSZ.csv
    * Maintains log/latest.csv -> current flight (symlink).
    * Flushes every row; fsyncs every 25 rows.

Replace the SIMULATION parts with real hardware I/O when ready.
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
from dataclasses import dataclass, field
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import List, Optional

# ---------------------------
# Configuration (baked-in)
# ---------------------------
BASE_DIR = Path(__file__).resolve().parent
REPO_ROOT = BASE_DIR.parent   # your Git repo root: DamgaardAirplane2

@dataclass
class Config:
    # Paths
    log_dir: Path = BASE_DIR / "log"
    # Git remote (only used if REPO_ROOT/.git exists)
    git_remote: str = "origin"

    # Devices / I/O (used when you switch to real hardware)
    uart_dev: str = "/dev/ttyS0"
    uart_baud: int = 115_200
    gps_dev: str = "/dev/ttyAMA0"

    # Rates
    poll_period_s: float = 1.0   # "website" poll period
    log_rate_hz: float = 5.0     # CSV logging frequency
    post_rate_hz: float = 1.0    # "web post" frequency

    # Logging verbosity
    debug: bool = True


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

@dataclass
class RuntimeState:
    stream_enabled: bool = False
    target_points: List[LatLon] = field(default_factory=list)
    latest_gps: GpsFix = field(default_factory=GpsFix)
    latest_esp32: Telemetry = field(default_factory=Telemetry)
    shutting_down: bool = False


# ---------------------------
# SIMULATION tasks (swap later)
# ---------------------------
async def gps_reader(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """Simulate a GPS near Aarhus at 5 Hz."""
    lat0, lon0 = 56.1629, 10.2039
    while not state.shutting_down:
        await asyncio.sleep(0.2)
        state.latest_gps = GpsFix(
            time_utc=time.time(),
            lat=lat0 + (random.random() - 0.5) * 1e-4,
            lon=lon0 + (random.random() - 0.5) * 1e-4,
            alt_m=60.0 + (random.random() - 0.5) * 5.0,
            fix_ok=True,
            sats=10 + random.randint(-2, 3),
        )

async def esp32_uart_rx(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """Simulate telemetry from ESP32 at ~20 Hz."""
    t0 = time.time()
    while not state.shutting_down:
        await asyncio.sleep(0.05)
        t = time.time() - t0
        state.latest_esp32 = Telemetry(
            time_utc=time.time(),
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
    """Simulate sending commands to ESP32 by printing them."""
    while not state.shutting_down:
        try:
            msg = await asyncio.wait_for(uart_tx_q.get(), timeout=0.5)
        except asyncio.TimeoutError:
            continue
        logger.info(f"[UART->ESP32] {msg}")
        uart_tx_q.task_done()

async def website_poller(state: RuntimeState, uart_tx_q: asyncio.Queue, cfg: Config, logger: logging.Logger):
    """Simulate a website poll: toggle stream every ~15 s and update simple targets."""
    last_toggle = time.time()
    toggle_every = 15.0
    k = 0
    while not state.shutting_down:
        await asyncio.sleep(cfg.poll_period_s)
        now = time.time()
        if now - last_toggle > toggle_every:
            state.stream_enabled = not state.stream_enabled
            last_toggle = now
            logger.info(f"[API] stream_enabled -> {state.stream_enabled}")
        k += 1
        base_lat, base_lon = 56.1629, 10.2039
        state.target_points = [
            LatLon(base_lat + 0.001, base_lon + 0.001),
            LatLon(base_lat + 0.001, base_lon - 0.001),
            LatLon(base_lat - 0.001, base_lon - 0.001),
            LatLon(base_lat - 0.001, base_lon + 0.001),
        ]
        if k % 5 == 0:
            line = "TARGETS:" + ";".join(f"{p.lat:.6f},{p.lon:.6f}" for p in state.target_points)
            await uart_tx_q.put(line)

async def stream_manager(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """Simulated stream process start/stop based on stream flag."""
    running = False
    while not state.shutting_down:
        await asyncio.sleep(0.2)
        if state.stream_enabled and not running:
            running = True
            logger.info("[STREAM] Starting simulated stream...")
        elif not state.stream_enabled and running:
            running = False
            logger.info("[STREAM] Stopping simulated stream...")
    if running:
        logger.info("[STREAM] Stopping simulated stream (shutdown).")

async def file_logger(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """
    Simple CSV logger.
    - On entry: rename previous flight to local(UTC+2) time 'YYYY-mm-dd_HH-MM.csv',
      git add/commit/push it, and delete it if push succeeds.
    - Then create 'flight_YYYYmmdd_HHMM.csv' for this boot and append rows.
    """
    cfg.log_dir.mkdir(parents=True, exist_ok=True)

    # ---- 1) Push previous flight (best-effort) ----
    prevs = sorted(cfg.log_dir.glob("flight_*.csv"))
    if prevs and (REPO_ROOT / ".git").exists():
        # Use "local" name = UTC+2, no trailing 'Z'
        local_now = datetime.utcnow() + timedelta(hours=2)
        pushed_name = local_now.strftime("%Y-%m-%d_%H-%M") + ".csv"
        prev = prevs[-1]
        target = prev.with_name(pushed_name)
        try:
            prev.rename(target)
            # If logs are ignored by .gitignore, keep "-f" to force-add
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

    # ---- 2) New log for this boot ----
    local_now = datetime.utcnow() + timedelta(hours=2)
    current_path = cfg.log_dir / f"flight_{local_now.strftime('%Y%m%d_%H%M')}.csv"
    f = current_path.open("w", newline="")
    writer = csv.writer(f)
    writer.writerow(["ts_iso","batt_v","airspeed_ms","groundspeed_ms",
                     "roll_deg","pitch_deg","yaw_deg","lat","lon","alt_m"])
    f.flush()

    # latest symlink (optional, best-effort)
    latest = cfg.log_dir / "latest.csv"
    try:
        if latest.exists() or latest.is_symlink():
            latest.unlink()
        latest.symlink_to(current_path.name)
    except Exception:
        pass
    logger.info(f"Logging to {current_path}")

    # ---- 3) Append rows ----
    period = 1.0 / max(cfg.log_rate_hz, 1.0)

    def ts_iso(ts_val: Optional[float]) -> str:
        # Use provided ts if it looks valid; otherwise "now" (UTC, ISO8601)
        base = datetime.fromtimestamp(ts_val, tz=timezone.utc) if (isinstance(ts_val, (int,float)) and math.isfinite(ts_val)) else datetime.now(tz=timezone.utc)
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


async def web_poster(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """Simulate posting a JSON payload once per second."""
    period = 1.0 / max(cfg.post_rate_hz, 1.0)
    while not state.shutting_down:
        await asyncio.sleep(period)
        te = state.latest_esp32
        gp = state.latest_gps
        # Inline ISO timestamp (no shared helpers)
        ts = datetime.fromtimestamp(time.time(), tz=timezone.utc).isoformat()
        payload = {
            "ts": ts,
            "battery_v": round(te.batt_v, 3) if math.isfinite(te.batt_v) else None,
            "gps": {
                "lat": round(gp.lat, 6) if math.isfinite(gp.lat) else None,
                "lon": round(gp.lon, 6) if math.isfinite(gp.lon) else None,
                "alt_m": round(gp.alt_m, 1) if math.isfinite(gp.alt_m) else None,
                "sats": gp.sats,
                "fix_ok": gp.fix_ok,
            },
            "stream": state.stream_enabled,
        }
        logger.info(f"[POST] {json.dumps(payload)}")

async def health_task(state: RuntimeState, cfg: Config, logger: logging.Logger):
    """Heartbeat every 5 seconds."""
    while not state.shutting_down:
        await asyncio.sleep(5.0)
        logger.debug("[HEALTH] alive")


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

    # Shared queue for outbound UART lines (simulated)
    uart_tx_q: asyncio.Queue[str] = asyncio.Queue(maxsize=100)

    # Signals for graceful stop
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, lambda s=sig: setattr(state, "shutting_down", True))
        except NotImplementedError:
            pass

    tasks = [
        asyncio.create_task(gps_reader(state, cfg, logger)),
        asyncio.create_task(esp32_uart_rx(state, cfg, logger)),
        asyncio.create_task(esp32_uart_tx(uart_tx_q, state, cfg, logger)),
        asyncio.create_task(website_poller(state, uart_tx_q, cfg, logger)),
        asyncio.create_task(stream_manager(state, cfg, logger)),
        asyncio.create_task(file_logger(state, cfg, logger)),   # handles push-previous + new-log creation
        asyncio.create_task(web_poster(state, cfg, logger)),
        asyncio.create_task(health_task(state, cfg, logger)),
    ]

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
