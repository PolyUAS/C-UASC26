from guidance.adapters.mavlink_sitl import SitlMissionAdapter
from guidance.contracts import AckStatus, CommandType, MissionCommand


class FakeMavlinkClient:
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


class FakeEventLogger:
    def __init__(self):
        self.events: list[tuple[str, str]] = []

    def log_dispatch(self, command: MissionCommand, ack):
        self.events.append((command.command_id, ack.status.value))


def test_dispatch_goto_command_maps_to_client_call():
    client = FakeMavlinkClient()
    adapter = SitlMissionAdapter(client=client)
    command = MissionCommand(
        command_id="cmd-1",
        mission_id="mission-waypoint",
        command_type=CommandType.GOTO,
        target={
            "lat_deg": 32.123,
            "lon_deg": -117.456,
            "alt_m": 52.0,
        },
    )

    ack = adapter.dispatch(command)

    assert ack.status == AckStatus.ACCEPTED
    assert client.calls == [("goto", (32.123, -117.456, 52.0))]


def test_dispatch_rejects_missing_required_target_fields():
    client = FakeMavlinkClient()
    adapter = SitlMissionAdapter(client=client)
    command = MissionCommand(
        command_id="cmd-2",
        mission_id="mission-waypoint",
        command_type=CommandType.GOTO,
        target={"lat_deg": 1.0},
    )

    ack = adapter.dispatch(command)

    assert ack.status == AckStatus.REJECTED
    assert "invalid_command" in (ack.reason or "")
    assert client.calls == []


def test_dispatch_reports_adapter_failure():
    class FailingClient(FakeMavlinkClient):
        def rtl(self) -> None:
            raise RuntimeError("link_down")

    adapter = SitlMissionAdapter(client=FailingClient())
    command = MissionCommand(
        command_id="cmd-3",
        mission_id="mission-failsafe",
        command_type=CommandType.RTL,
    )

    ack = adapter.dispatch(command)

    assert ack.status == AckStatus.FAILED
    assert "adapter_error" in (ack.reason or "")


def test_dispatch_logs_event_when_logger_present():
    client = FakeMavlinkClient()
    event_logger = FakeEventLogger()
    adapter = SitlMissionAdapter(client=client, event_logger=event_logger)
    command = MissionCommand(
        command_id="cmd-4",
        mission_id="mission-waypoint",
        command_type=CommandType.ARM,
    )

    ack = adapter.dispatch(command)

    assert ack.status == AckStatus.ACCEPTED
    assert event_logger.events == [("cmd-4", "accepted")]
