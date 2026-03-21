# Mock MAVLink interfaces for tests
from guidance.contracts import AckStatus, CommandAck, MissionCommand


class FakeMavlinkClient:
    """In-memory MAVLink client that records all calls."""

    def __init__(self):
        self.calls: list[tuple[str, tuple]] = []

    def arm(self) -> None:
        self.calls.append(("arm", ()))

    def takeoff(self, altitude_m: float) -> None:
        self.calls.append(("takeoff", (altitude_m,)))

    def goto(self, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        self.calls.append(("goto", (lat_deg, lon_deg, alt_m)))

    def hold(self) -> None:
        self.calls.append(("hold", ()))

    def rtl(self) -> None:
        self.calls.append(("rtl", ()))

    def land(self) -> None:
        self.calls.append(("land", ()))

    def payload_release(self) -> None:
        self.calls.append(("payload_release", ()))

    def payload_secure(self) -> None:
        self.calls.append(("payload_secure", ()))


class FailingMavlinkClient(FakeMavlinkClient):
    """Client that raises on every call — for testing error paths."""

    def arm(self) -> None:
        raise RuntimeError("link_down")

    def takeoff(self, altitude_m: float) -> None:
        raise RuntimeError("link_down")

    def goto(self, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        raise RuntimeError("link_down")

    def hold(self) -> None:
        raise RuntimeError("link_down")

    def rtl(self) -> None:
        raise RuntimeError("link_down")

    def land(self) -> None:
        raise RuntimeError("link_down")
