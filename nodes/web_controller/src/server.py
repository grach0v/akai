"""The web server every mode runs its page on: uvicorn on a background thread."""

from __future__ import annotations

import logging
import socket
import threading
import time

import uvicorn
from fastapi import FastAPI

logger = logging.getLogger("web_controller")


class WebServer:
    """Serves `app` on a daemon thread. Its asyncio loop is its own; the dora loop never
    runs on it, so a mode shares state with its routes through a lock."""

    def __init__(self, app: FastAPI, host: str, port: int, timeout: float):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:  # uvicorn would only log a failed bind
            if s.connect_ex(("127.0.0.1", port)) == 0:
                raise RuntimeError(f"port {port} is taken. Close what holds it or set PORT to a free port.")
        self.url = f"http://{host}:{port}"
        self._server = uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level="warning"))
        self._thread = threading.Thread(target=self._server.run, name="web-server", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + timeout
        while not self._server.started:
            if not self._thread.is_alive():
                raise RuntimeError(f"web server on {self.url} failed to start (see the log above)")
            if time.monotonic() > deadline:
                raise RuntimeError(f"web server on {self.url} did not start within {timeout}s")
            time.sleep(0.05)
        logger.info("serving %s", self.url)

    def stop(self) -> None:
        self._server.should_exit = True
        self._thread.join(timeout=2.0)
