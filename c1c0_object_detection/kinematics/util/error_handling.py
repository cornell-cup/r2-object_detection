""" File to process any error messages.

"Written" (copied from Stack Overflow) by Simon Kapen '24, Spring 2022."""

import contextlib
import sys


@contextlib.contextmanager
def nostderr():
    """Suppresses an unrelevant error when the arm URDF file is opened.

    Adapted from https://stackoverflow.com/questions/1809958/hide-stderr-output-in-unit-tests.
    """
    savestderr = sys.stderr

    class Devnull(object):
        def write(self, _): pass
        def flush(self): pass
    sys.stderr = Devnull()
    try:
        yield
    finally:
        sys.stderr = savestderr
