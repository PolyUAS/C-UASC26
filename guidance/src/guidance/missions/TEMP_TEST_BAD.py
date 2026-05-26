import asyncio
import time
from operator import ge

from statemachine import State
from statemachine.transition_list import TransitionList
from zenoh import Session

from guidance.generated.waypoint.Waypoint import Waypoint
from guidance.geo_utils import WaypointStruct
from guidance.lawnmower import generate_lawnmower_sweep
from guidance.missionFSM import AbstractFSMBaseClass

TEST_SPEED = 1.0  # ms/s


class BAD_FSM(AbstractFSMBaseClass):
    # ── States ────────────────────────────────────────────────────────────────
    HOPPING = State()
    INIT = State(initial=True)
    LAUNCH = State()
    COMPLETE = State(final=True)
    LAND = State()
    RETURN_TO_HOME = State()

    # ── Transitions ───────────────────────────────────────────────────────────
    on_reinit: TransitionList = INIT.to(INIT)
    on_go: TransitionList = INIT.to(LAUNCH)
    on_return: TransitionList = RETURN_TO_HOME.to(LAND)
    on_finish: TransitionList = LAND.to(COMPLETE)

    begin_hop: TransitionList = LAUNCH.to(HOPPING)
    end_hop: TransitionList = HOPPING.to(RETURN_TO_HOME)

    def __init__(self, session: Session, home: WaypointStruct):

        super().__init__(session, home)

    async def run(self) -> None:
        # prev = None
        await self.activate_initial_state()
        # print(f"Prev = {prev}, Current = {self.current_state.id}... Next = LAUNCH")  # type: ignore

        await self.on_go()  # INIT → LAUNCH
        # print(f"Prev = INIT, Current = {self.current_state.id}... Next = HOPPING")  # type: ignore

        await self.begin_hop()  # LAUNCH → HOPPING
        # print(
        #     f"Prev = LAUNCH, Current = {self.current_state.id}... Next = RETURN_TO_HOME"  # type: ignore
        # )

        await self.end_hop()  # HOPPING → RETURN_TO_HOME
        # print(f"Prev = HOPPING, Current = {self.current_state.id}... Next = LAND")  # type: ignore

        await self.on_return()  # RETURN_TO_HOME → LAND
        # print(
        #     f"Prev = RETURN_TO_HOME, Current = {self.current_state.id}... Next = COMPLETE"  # type: ignore
        # )

        await self.on_finish()  # LAND → COMPLETE
        # print(f"Prev = LAND, Current = {self.current_state.id}... Next = None")  # type: ignore

    async def on_enter_INIT(self):
        while len(self.geofence) == 0:
            print("waiting for geofence")
            self.update_geofence()
            print(f"geofence points: {len(self.geofence)}")
            await asyncio.sleep(0.3)

    async def on_enter_LAUNCH(self) -> None:
        print("waiting to be at takeoff alt")

        while self.pos.alt < 12_000.0:
            # self.update_geofence()
            print(f"alt: {self.pos.alt}")
            await asyncio.sleep(0.300)

    async def on_enter_HOPPING(self) -> None:
        print("starting lawnmower sweep")

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

           # await self.navigate_waypoint(target, 0.4, speed=TEST_SPEED)

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
