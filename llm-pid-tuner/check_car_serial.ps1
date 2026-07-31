$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot
$python = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"

@'
import time
import serial

with serial.Serial("COM15", 115200, timeout=0.25, write_timeout=1) as port:
    port.reset_input_buffer()
    port.write(b"STATUS\n")
    deadline = time.time() + 5.0
    valid_csv = 0
    status_seen = False
    while time.time() < deadline:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="backslashreplace").strip()
        print(line.encode("ascii", errors="backslashreplace").decode("ascii"))
        if line.startswith("# STATUS"):
            status_seen = True
        parts = line.split(",")
        if len(parts) == 9:
            try:
                [float(value) for value in parts]
                valid_csv += 1
            except ValueError:
                pass

    if not status_seen or valid_csv == 0:
        raise SystemExit("FAILED: no STATUS response or valid 8-column telemetry")
    print(f"PASS: STATUS received and {valid_csv} telemetry rows parsed")
'@ | & $python -
exit $LASTEXITCODE
