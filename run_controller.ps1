# run_controller.ps1
#
# Shortcut target:
#   powershell.exe -ExecutionPolicy Bypass -File "C:\Users\Caltech6mUser\srt-py\run_controller.ps1"
#
# The script:
#   1. Enforces single-instance via a named mutex (shows a dialog if already running)
#   2. Sets PYTHONUNBUFFERED so logs appear in real time
#   3. Invokes the env's python.exe directly (no conda run wrapper) so PowerShell
#      owns the process — when Tkinter closes, this window closes too.

$scriptPath = "C:\Users\Caltech6mUser\srt-py\bin\Moore6mController.py"
$configDir  = "C:\Users\Caltech6mUser\srt-py\config"
$serialPort = "COM1"

# ---- Single-instance lock ----
Add-Type -AssemblyName PresentationFramework   # for MessageBox
$mutex = New-Object System.Threading.Mutex($false, "Global\Moore6mControllerMutex")
if (-not $mutex.WaitOne(0)) {
    [System.Windows.MessageBox]::Show(
        "Moore 6m Controller is already running.",
        "Already Running",
        [System.Windows.MessageBoxButton]::OK,
        [System.Windows.MessageBoxImage]::Information
    ) | Out-Null
    exit 0
}

$env:PYTHONUNBUFFERED = "1"

try {
    conda run --no-capture-output -n srt-dev python $scriptPath `
        --port        $serialPort `
        --config_dir  $configDir `
        --autostart 2>&1
} finally {
    $mutex.ReleaseMutex()
}
