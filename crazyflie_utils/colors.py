"""Minimal ANSI colour helpers for console output.

Colours are disabled automatically when stdout is not a TTY (e.g. when the
node's output is piped into a log file) or when ``NO_COLOR`` is set, so the
JSON/CSV artefacts and log files stay free of escape codes.
"""

import os
import sys

_ENABLED = sys.stdout.isatty() and os.environ.get('NO_COLOR') is None


def _wrap(code: str, text: str) -> str:
    if not _ENABLED:
        return text
    return f"\033[{code}m{text}\033[0m"


def green(text: str) -> str:
    return _wrap('92', text)


def yellow(text: str) -> str:
    return _wrap('93', text)


def red(text: str) -> str:
    return _wrap('91', text)


def cyan(text: str) -> str:
    return _wrap('96', text)


def bold(text: str) -> str:
    return _wrap('1', text)


def dim(text: str) -> str:
    return _wrap('2', text)


def set_enabled(enabled: bool) -> None:
    """Force colours on/off (used by ``--no-color``)."""
    global _ENABLED
    _ENABLED = bool(enabled)
