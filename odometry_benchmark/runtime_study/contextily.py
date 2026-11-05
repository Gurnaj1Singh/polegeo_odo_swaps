"""Stub for the optional basemap in the original script (not installed in this venv). add_basemap raises, which the
original's try/except reports as "skipping basemap"; the basemap is drawn once before the timed frame loop."""
class _P:
    def __getattr__(self, k):
        return self
providers = _P()


def add_basemap(*a, **k):
    raise RuntimeError("contextily not installed (runtime study stub)")
