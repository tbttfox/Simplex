import os
import subprocess
import sys

VERSION_FILE = "version.txt"


def main():
    # If version.txt already exists, use it
    if os.path.exists(VERSION_FILE):
        with open(VERSION_FILE) as f:
            v = f.read().strip()
        print(v)
        return

    # Otherwise, try git describe
    try:
        # fmt: off
        v = subprocess.check_output(
            ["git", "describe", "--tags", "--always", "--match", "v[0-9]*", "--abbrev=0"],
            text=True,
        ).strip()
        # fmt: on
    except Exception:
        v = "0.0.1"  # fallback if not in a git repo

    # Write version.txt so future builds can reuse it
    with open(VERSION_FILE, "w") as f:
        f.write(v)

    print(v)


if __name__ == "__main__":
    sys.exit(main())
