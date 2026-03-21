"""MAVLink integration tests — require SITL or hardware.

These tests are skipped by default because they need a live MAVLink
connection (e.g. ArduPilot SITL on tcp:127.0.0.1:5760).

Run with:  pytest -m sitl  (after starting SITL externally)
"""

import pytest


pytestmark = pytest.mark.skip(reason="requires_sitl: start ArduPilot SITL externally")


def test_sitl_heartbeat_received():
    """Verify pymavlink receives a HEARTBEAT from SITL within timeout."""
    from guidance.adapters.mavlink_pymavlink import PymavlinkSitlClient

    client = PymavlinkSitlClient.connect(
        connection_string="tcp:127.0.0.1:5760",
        heartbeat_timeout_sec=10.0,
    )
    assert client.target_system > 0


def test_sitl_arm_and_disarm():
    """Verify ARM command is accepted by SITL."""
    from guidance.adapters.mavlink_pymavlink import PymavlinkSitlClient

    client = PymavlinkSitlClient.connect(connection_string="tcp:127.0.0.1:5760")
    client.arm()
    # If no exception, the command was sent successfully
