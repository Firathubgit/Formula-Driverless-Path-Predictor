"""Launch the native desktop app, starting with its car-selection videos."""

from pathlib import Path
import shutil
import subprocess
import sys


def main() -> int:
    root = Path(__file__).resolve().parent
    shell = shutil.which("pwsh") or shutil.which("powershell")
    if shell is None:
        print("PowerShell is required to launch the desktop app.", file=sys.stderr)
        return 1
    return subprocess.call(
        [shell, "-NoProfile", "-File", str(root / "scripts" / "run.ps1"), *sys.argv[1:]],
        cwd=root,
    )


if __name__ == "__main__":
    raise SystemExit(main())
