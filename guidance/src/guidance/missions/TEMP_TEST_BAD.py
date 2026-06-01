import asyncio
import os
import sys
import time
from enum import Enum

from statemachine import State
from statemachine.transition_list import TransitionList
from zenoh import Session

from guidance.generated.Geofence import Coords as GuidanceCoordsModule
from guidance.generated.Geofence.GeofencePacket import GeofencePacket
from guidance.geo_utils import WaypointStruct
from guidance.lawnmower import generate_lawnmower_sweep
from guidance.missionFSM import AbstractFSMBaseClass

sys.modules.setdefault("Geofence.Coords", GuidanceCoordsModule)

TEST_SPEED = 0.50  # m/s
TAKEOFF_ALT_MM = 12_000.0
FALLBACK_FENCE_FILE = "/home/mvela/ardupilot/fence-items.txt"


class MissionMode(str, Enum):
    """Temporary selector while the real mission selector is still being wired in."""

    BASE_HOP = "base_hop"
    OBJECT_LOCALIZATION = "object_localization"


class BAD_FSM(AbstractFSMBaseClass):
    # States
    HOPPING = State()
    INIT = State(initial=True)
    LAUNCH = State()
    COMPLETE = State(final=True)
    LAND = State()
    RETURN_TO_HOME = State()

    # Transitions
    on_reinit: TransitionList = INIT.to(INIT)
    on_go: TransitionList = INIT.to(LAUNCH)
    on_return: TransitionList = RETURN_TO_HOME.to(LAND)
    on_finish: TransitionList = LAND.to(COMPLETE)

    begin_hop: TransitionList = LAUNCH.to(HOPPING)
    end_hop: TransitionList = HOPPING.to(RETURN_TO_HOME)

    def __init__(
        self,
        session: Session,
        home: WaypointStruct,
        mission_mode: MissionMode | str | None = None,
    ):
        super().__init__(session, home)
        self.pos = home

        mode = mission_mode or os.getenv("MISSION_MODE", MissionMode.BASE_HOP.value)
        self.mission_mode = MissionMode(mode)

    async def run(self) -> None:
        await self.activate_initial_state()

        await self.on_go()  # INIT -> LAUNCH
        await self.begin_hop()  # LAUNCH -> HOPPING
        await self.end_hop()  # HOPPING -> RETURN_TO_HOME
        await self.on_return()  # RETURN_TO_HOME -> LAND
        await self.on_finish()  # LAND -> COMPLETE

    async def on_enter_INIT(self) -> None:
        print(f"mission mode: {self.mission_mode.value}")

        if self.mission_mode == MissionMode.OBJECT_LOCALIZATION:
            print("requesting geofence from control/query/geofence")
            self.update_geofence()
            time.sleep(1)
            print(f"geofence points: {len(self.geofence)}")

    async def on_enter_LAUNCH(self) -> None:
        print("waiting to be at takeoff alt")
        while self.pos.alt < TAKEOFF_ALT_MM:
            print(f"alt (mm): {self.pos.alt}")
            await asyncio.sleep(0.300)

    async def on_enter_HOPPING(self) -> None:
        if self.mission_mode == MissionMode.OBJECT_LOCALIZATION:
            await self._run_object_localization_lawnmower()
        else:
            await self._run_base_hop()

    async def _run_base_hop(self) -> None:
        print("hopping")

        target = WaypointStruct(
            self.pos.lat + 5_000,
            self.pos.lon,
            self.pos.alt,
        )

        print(f"current: alt={self.pos.alt}, lat={self.pos.lat}, lon={self.pos.lon}")
        print(f"target: alt={target.alt}, lat={target.lat}, lon={target.lon}")

        there = False
        while not there:
            print(f"pos: {self.pos.alt}, {self.pos.lat}, {self.pos.lon}")
            there = await self.navigate_waypoint(target, 0.4, speed=TEST_SPEED)

    async def _run_object_localization_lawnmower(self) -> None:
        print("starting object-localization lawnmower sweep")

        if len(self.geofence) < 3:
            print("no valid geofence was received; skipping lawnmower sweep")
            return

        sweep_points = generate_lawnmower_sweep(
            geofence=self.geofence,
            alt_mm=self.pos.alt,
            spacing_m=8.0,
            margin_m=8.0,
            sample_step_m=1.0,
        )

        print(f"generated {len(sweep_points)} sweep waypoints")

        for i, waypoint in enumerate(sweep_points):
            print(f"going to sweep waypoint {i + 1}/{len(sweep_points)}")

            success = await self.navigate_waypoint(
                waypoint,
                a_rad=1.0,
                speed=5.0,
                timeout=60.0,
            )

            if not success:
                print("failed to reach sweep waypoint")
                break

    async def on_enter_RETURN_TO_HOME(self) -> None:
        # await self.navigate_waypoint(self.home, 0.5)
        pass

    async def on_enter_LAND(self) -> None:
        print("LAND")
        # ground = WaypointStruct(
        #     self.home.lat,
        #     self.home.lon,
        #     self.home.alt + 500.0,  # half a meter agl
        # )
        # await self.navigate_waypoint(ground, 0.4, speed=TEST_SPEED)
