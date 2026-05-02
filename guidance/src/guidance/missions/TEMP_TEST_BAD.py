import asyncio

from statemachine import State
from statemachine.transition_list import TransitionList
from zenoh import Session

from guidance.generated.waypoint.Waypoint import Waypoint
from guidance.geo_utils import WaypointStruct
from guidance.missionFSM import AbstractFSMBaseClass


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
        prev = None
        await self.activate_initial_state()
        print(f"Prev = {prev}, Current = {self.current_state.id}, Next = LAUNCH")

        await self.on_go()  # INIT → LAUNCH
        print(f"Prev = INIT, Current = {self.current_state.id}, Next = HOPPING")

        await self.begin_hop()  # LAUNCH → HOPPING
        print(
            f"Prev = LAUNCH, Current = {self.current_state.id}, Next = RETURN_TO_HOME"
        )

        await self.end_hop()  # HOPPING → RETURN_TO_HOME
        print(f"Prev = HOPPING, Current = {self.current_state.id}, Next = LAND")

        await self.on_return()  # RETURN_TO_HOME → LAND
        print(
            f"Prev = RETURN_TO_HOME, Current = {self.current_state.id}, Next = COMPLETE"
        )

        await self.on_finish()  # LAND → COMPLETE
        print(f"Prev = LAND, Current = {self.current_state.id}, Next = None")

    async def on_enter_LAUNCH(self) -> None:
        pass

    async def on_enter_HOPPING(self) -> None:
        target = WaypointStruct(
            self.home.lat,
            self.home.lon,
            self.home.alt + 8_000,
        )
        await self.navigate_waypoint(target, 0.5)

    async def on_enter_RETURN_TO_HOME(self) -> None:
        await self.navigate_waypoint(self.home, 0.5)

    async def on_enter_LAND(self) -> None:
        ground = WaypointStruct(
            self.home.lat,
            self.home.lon,
            0,
        )
        await self.navigate_waypoint(ground, 0.5)
