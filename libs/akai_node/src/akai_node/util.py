"""Small helpers nodes share."""

from __future__ import annotations

import socket
from datetime import datetime


def message_time(metadata: dict, key: str, stream: str) -> float:
    """A message's time under metadata `key`, in seconds since the epoch: dora's send `timestamp`
    (a datetime) or a number of seconds, like `capture_time`. A missing key raises, naming `stream`."""
    if key not in metadata:
        raise ValueError(f"{stream}: no `{key}` in its metadata")
    t = metadata[key]
    return t.timestamp() if isinstance(t, datetime) else float(t)


def port_open(host: str, port: int, timeout: float = 0.25) -> bool:
    """True if something accepts TCP connections at host:port; 0.0.0.0 is checked on 127.0.0.1.
    Servers that only log a failed bind (uvicorn, rerun) check this first."""
    host = "127.0.0.1" if host == "0.0.0.0" else host
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        return s.connect_ex((host, port)) == 0
