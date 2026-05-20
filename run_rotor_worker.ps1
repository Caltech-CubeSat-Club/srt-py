# run_rotor_worker.ps1
$envName = "srt-dev"
$scriptPath = "C:\path\to\srt-py\bin\rotor_worker.py"

# Single-instance lock (per user session)
$mutex = New-Object System.Threading.Mutex($false, "Global\RotorWorkerMutex")
if (-not $mutex.WaitOne(0)) {
    Write-Host "rotor_worker is already running."
    exit 0
}

try {
    conda run -n $envName python $scriptPath
} finally {
    $mutex.ReleaseMutex()
}