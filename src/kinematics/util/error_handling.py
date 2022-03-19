""" File to process any error messages. """

import contextlib
import sys


@contextlib.contextmanager
def nostderr():
    """Suppresses unwanted/unrelevant errors."""
    savestderr = sys.stderr

    class Devnull(object):
        def write(self, _): pass
        def flush(self): pass
    sys.stderr = Devnull()
    try:
        yield
    finally:
        sys.stderr = savestderr
