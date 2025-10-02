
# Dummy Pi Runner (single file)

## Run locally
```bash
python3 airplane_pi.py
```
You'll see simulated tasks printing logs, writing CSV to `./log/flight_current.csv`, and rotating/pushing on next start if you have a `.git` repo.

## Install as systemd service (on the Pi)
1. Copy files:
   - `airplane_pi.py` to your repo folder (e.g., `/home/pi/Damgaard-Airplane/`).
   - `airplane.service` to `/etc/systemd/system/airplane.service` (root required).

2. Edit `airplane.service` paths to match your setup.

3. Enable + start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable airplane.service
sudo systemctl start airplane.service
sudo systemctl status airplane.service
```

## Env vars (optional)
- `AIRPLANE_DEBUG=1`
- `AIRPLANE_API_BASE=https://example.com`
- `AIRPLANE_YT_KEY=rtmps://...`
- `AIRPLANE_UART_DEV=/dev/ttyS0`
- `AIRPLANE_UART_BAUD=115200`
- `AIRPLANE_GPS_DEV=/dev/ttyAMA0`
- `AIRPLANE_POLL_PERIOD_S=1.0`
- `AIRPLANE_LOG_RATE_HZ=5.0`
- `AIRPLANE_POST_RATE_HZ=1.0`
