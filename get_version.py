import os
import subprocess
import sys

VERSION_FILE = "version.txt"


def main(srcroot):
    # If version.txt already exists, use it
    src_file = os.path.join(srcroot, VERSION_FILE)
    if os.path.exists(src_file):
        with open(src_file) as f:
            v = f.read().strip()
    else:
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

    print(v)


if __name__ == "__main__":
    srcroot = sys.argv[1]
    sys.exit(main(srcroot))
