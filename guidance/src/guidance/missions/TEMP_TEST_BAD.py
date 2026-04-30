import asyncio

from statemachine import State
from zenoh import Session

from guidance.generated.waypoint.Waypoint import Waypoint
from guidance.geo_utils import WaypointStruct
from guidance.missionFSM import AbstractFSMBaseClass


class BAD_FSM(AbstractFSMBaseClass):
    HOPPING = State()

    # Define transition with callback at class level
    begin_fsm = AbstractFSMBaseClass.LAUNCH.to(HOPPING)

    def __init__(self, session: Session, home: WaypointStruct):
        super().__init__(session, home)

    async def run(self) -> None:
        self.activate_initial_state()
        await self.begin_fsm()  # actually trigger the transition

    # on_<transition_name> is the idiomatic callback hook in python-statemachine
    async def on_begin_fsm(self) -> None:

        target = WaypointStruct(
            self.home.lat,
            self.home.lon,
            self.home.alt + 300,
        )
        await self.navigate_waypoint(target, 0.5)  # await, don't asyncio.run()
