# akai_node

What every node shares, so a node's own code is only its behaviour.

| name | what |
| --- | --- |
| `run(modes, Config)` | the whole `main.py`: read the config (env, then CLI flags), build the mode `mode.name` names, join the dataflow, run the event loop, close the mode |
| `Mode` | the node protocol (`mode.py`): `start`, `handle` dispatching inputs by id to `handlers`, `close`; stops on `program_state == disconnect`; `set_state` reports `node_state`, edge-triggered |
| `Worker` | a background thread for work that must not block the dora loop, with `ended` once it died |
| `message_time(metadata, key, stream)` | a message's time in seconds: dora's `timestamp` or `capture_time` |
| `port_open(host, port)` | whether a server already listens there |

A node's `main.py`:

```python
from akai_node import run

from config import CameraConfig
from modes import MODES

if __name__ == "__main__":
    run(MODES, CameraConfig)
```

A mode:

```python
from akai_node import Mode

class CameraMode(Mode):
    def __init__(self, cfg):
        super().__init__()          # handlers = {"program_state": ...}
        self.handlers["tick"] = self._on_tick
        ...                         # open the device: before the node joins

    def _on_tick(self, event) -> bool:
        ...
        return False                # True stops the node
```

Depend on it by path, editable, like the other libs:

```toml
dependencies = [..., "akai_node"]

[tool.uv.sources]
akai_node = { path = "../../libs/akai_node", editable = true }
```
