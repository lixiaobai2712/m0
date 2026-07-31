# MSPM0 line-follow car deployment

This deployment uses COM15 at 115200 baud and never sends `RUN` automatically.
Start and stop the car locally with PB21. Keep the main power switch within reach.

## Prerequisites

1. Revoke the API key that appeared in the screenshot and create a new key.
2. Install 64-bit Python 3.11 or 3.12 and enable "Add Python to PATH".
3. Close every serial-terminal program that may already own COM15.
4. Copy `config.car.example.json` to `config.json` and enter the exact model name.
   Prefer leaving the API key placeholder and setting it in PowerShell:

```powershell
$env:LLM_API_KEY = "your-new-key"
```

## Install and check communication

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe doctor.py
```

First put the car on a stand with its wheels clear of the ground. Flash the
firmware, power it on, and verify telemetry on COM15. The car remains stopped
until PB21 is pressed.

## Tune on the track

Place the car on the track at a safe, low base speed. Keep a hand at PB21 and
another person ready at the main power switch, then run:

```powershell
.\.venv\Scripts\python.exe tuner.py --plain
```

Alternatively, run `start_car_tuning.ps1`. It prompts for the API token with
hidden input and keeps the token only in the current process.

The tuner collects 300 samples (about six seconds at 50 Hz) per round. It sends
only `SET P:... I:... D:...`; it does not start the motors. Lost-line samples
(`input` equal to +110 or -110) receive the highest penalty. Edge hits and rapid
left/right reversals are penalized next. If a PID is substantially worse than
the historical best stable PID, the existing rollback logic writes the best PID
back automatically.

Press PB21 immediately if the car leaves the usable track area. Ctrl+C stops
the PC tuner but is not a physical emergency stop.
