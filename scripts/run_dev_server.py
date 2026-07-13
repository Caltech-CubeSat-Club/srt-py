import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).parent.parent

print("Starting Moore6mController...")
python_proc = subprocess.Popen(
    [sys.executable, "bin/Moore6mController.py", "--config_dir", "config", "--autostart"],
    cwd=REPO_ROOT
)

print("Starting pnpm dev...")
pnpm_proc = subprocess.Popen(
    ["pnpm", "dev"],
    cwd=REPO_ROOT / "srt" / "svelte-frontend"
)

print("\nPress Ctrl+C to stop.\n")

try:
    # Keep the meta-script alive so the background processes don't close early
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Clean up and terminate both processes gracefully when you press Ctrl+C
    print("\nShutting down processes...")
    python_proc.terminate()
    pnpm_proc.terminate()
    
    # Wait for them to actually close
    python_proc.wait()
    pnpm_proc.wait()
    print("Done.")
