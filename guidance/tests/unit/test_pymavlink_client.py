from guidance.adapters.mavlink_pymavlink import PymavlinkSitlClient


class FakeMav:
    def __init__(self):
        self.command_long_calls: list[tuple] = []
        self.position_calls: list[tuple] = []

    def command_long_send(self, *args):
        self.command_long_calls.append(args)

    def set_position_target_global_int_send(self, *args):
        self.position_calls.append(args)


class FakeConnection:
    def __init__(self):
        self.mav = FakeMav()
        self.mode_calls: list[str] = []

    def set_mode(self, mode_name: str):
        self.mode_calls.append(mode_name)


class FakeMavlinkConsts:
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_CMD_NAV_LAND = 21
    MAV_CMD_DO_SET_SERVO = 183
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6


def test_pymavlink_client_arm_and_takeoff_issue_command_long():
    connection = FakeConnection()
    client = PymavlinkSitlClient(
        connection=connection,
        mavlink=FakeMavlinkConsts,
        target_system=1,
        target_component=1,
    )

    client.arm()
    client.takeoff(35.0)

    assert connection.mav.command_long_calls[0][2] == FakeMavlinkConsts.MAV_CMD_COMPONENT_ARM_DISARM
    assert connection.mav.command_long_calls[1][2] == FakeMavlinkConsts.MAV_CMD_NAV_TAKEOFF
    assert connection.mav.command_long_calls[1][-1] == 35.0


def test_pymavlink_client_goto_emits_position_target():
    connection = FakeConnection()
    client = PymavlinkSitlClient(
        connection=connection,
        mavlink=FakeMavlinkConsts,
        target_system=1,
        target_component=1,
    )

    client.goto(32.1234567, -117.7654321, 40.0)

    assert len(connection.mav.position_calls) == 1
    call = connection.mav.position_calls[0]
    assert call[1] == 1
    assert call[2] == 1
    assert call[3] == FakeMavlinkConsts.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert call[5] == 321234567
    assert call[6] == -1177654321
    assert call[7] == 40.0


def test_pymavlink_client_hold_uses_set_mode_loiter():
    connection = FakeConnection()
    client = PymavlinkSitlClient(
        connection=connection,
        mavlink=FakeMavlinkConsts,
        target_system=1,
        target_component=1,
    )

    client.hold()

    assert connection.mode_calls == ["LOITER"]
