import os
import subprocess
import sys

VERSION_FILE = "version.txt"


def main(srcroot, gitpath):
    # If version.txt already exists, use it
    src_file = os.path.join(srcroot, VERSION_FILE)
    if os.path.exists(src_file):
        with open(src_file) as f:
            v = f.read().strip()
    else:
        # Otherwise, try git describe
        # fmt: off
        v = subprocess.check_output(
            [gitpath, "describe", "--tags", "--always", "--match", "v[0-9]*", "--abbrev=0"],
            text=True,
        ).strip()
        # fmt: on

    print(v)

if __name__ == "__main__":
    srcroot = sys.argv[1]
    gitpath = sys.argv[2]
    sys.exit(main(srcroot, gitpath))
