from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
import json

from guidance.contracts import CommandAck, MissionCommand


class JsonlReplayLogger:
    def __init__(self, file_path: str | Path):
        self.file_path = Path(file_path)
        self.file_path.parent.mkdir(parents=True, exist_ok=True)

    def log_dispatch(self, command: MissionCommand, ack: CommandAck) -> None:
        event = {
            "ts_utc": datetime.now(timezone.utc).isoformat(),
            "kind": "dispatch",
            "command": self._serialize(command),
            "ack": self._serialize(ack),
        }
        with self.file_path.open("a", encoding="utf-8") as replay_file:
            replay_file.write(json.dumps(event, separators=(",", ":"), sort_keys=True))
            replay_file.write("\n")

    def _serialize(self, value):
        if is_dataclass(value):
            data = asdict(value)
        elif isinstance(value, dict):
            data = dict(value)
        else:
            return value

        return {key: self._serialize_enum(item) for key, item in data.items()}

    @staticmethod
    def _serialize_enum(value):
        enum_value = getattr(value, "value", None)
        if enum_value is not None:
            return enum_value
        return value
