"""A background thread for work that must not block the dora loop (a camera, physics, rendering)."""

from __future__ import annotations

import threading


class Worker:
    """Runs `_run` on a daemon thread until `stop`. `_run` checks `_stop`; if it ends on its own
    (an exception, printed by threading's excepthook, or a device that stopped), `ended` is True."""

    def __init__(self, name: str):
        self._stop = threading.Event()
        self._ended = threading.Event()
        self._thread = threading.Thread(target=self._main, name=name, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    @property
    def ended(self) -> bool:
        return self._ended.is_set() and not self._stop.is_set()

    def _main(self) -> None:
        try:
            self._run()
        finally:
            self._ended.set()

    def _run(self) -> None:
        raise NotImplementedError
