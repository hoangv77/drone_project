"""Tie recording subprocess lifetime to the supervisor on Linux."""
import ctypes
import os
import signal
import sys


def main():
    parent, executable, *arguments = sys.argv[1:]
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), 'Cannot monitor recording supervisor lifetime')
    if os.getppid() != int(parent):
        return 1
    os.execvp(executable, [executable, *arguments])


if __name__ == '__main__':
    sys.exit(main())
