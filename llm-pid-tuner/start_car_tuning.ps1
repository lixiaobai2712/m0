$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

$python = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python)) {
    throw "Virtual environment not found: $python"
}

$ports = @(& $python -c "from serial.tools.list_ports import comports; print(*[p.device for p in comports()], sep='\n')") |
    ForEach-Object { ([string]$_).Trim() } |
    Where-Object { $_ }
if (-not ($ports | Where-Object { $_ -eq "COM15" })) {
    throw "COM15 is not available. Check Bluetooth pairing and the JDY-31 connection."
}

if (-not $env:LLM_API_KEY) {
    $secureKey = Read-Host "Enter the new API token (input is hidden)" -AsSecureString
    $keyPointer = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secureKey)
    try {
        $env:LLM_API_KEY = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($keyPointer)
    } finally {
        [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($keyPointer)
    }
}

if (-not $env:LLM_API_KEY) {
    throw "API token cannot be empty."
}

Write-Host "COM15 is present. The tuner will not send RUN; use PB21 to start/stop the car."
Write-Host "Keep PB21 and the main power switch within reach."
& $python tuner.py COM15 --plain
exit $LASTEXITCODE
