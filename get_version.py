import os
import subprocess
import sys

VERSION_FILE = "version.txt"


def main(root):
    # If version.txt already exists, use it
    ver_file = os.path.join(root, VERSION_FILE)

    if os.path.exists(ver_file):
        with open(ver_file) as f:
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
    with open(ver_file, "w") as f:
        f.write(v)

    print(v)


if __name__ == "__main__":
    root = sys.argv[1]
    sys.exit(main(root))
