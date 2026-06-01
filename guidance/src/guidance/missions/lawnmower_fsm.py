import os
import sys
import time

from statemachine import State
from statemachine.transition_list import TransitionList
from zenoh import Session

from guidance.generated.Geofence import Coords as GuidanceCoordsModule
from guidance.geo_utils import WaypointStruct
from guidance.lawnmower import generate_lawnmower_sweep
from guidance.missionFSM import AbstractFSMBaseClass

sys.modules.setdefault("Geofence.Coords", GuidanceCoordsModule)

TEST_SPEED = 0.50  # m/s
TAKEOFF_ALT_MM = 12_000.0


class LAWNMOWER_FSM(AbstractFSMBaseClass):
    # ------------------------------------------------------------------
    # States
    # ------------------------------------------------------------------
    INIT = State(initial=True)
    LAUNCH = State()
    HOPPING = State()
    RETURN_TO_HOME = State()
    LAND = State()
    COMPLETE = State(final=True)

    # ------------------------------------------------------------------
    # Transitions
    # ------------------------------------------------------------------
    on_reinit: TransitionList = INIT.to(INIT)
    on_go: TransitionList = INIT.to(LAUNCH)
    begin_hop: TransitionList = LAUNCH.to(HOPPING)
    end_hop: TransitionList = HOPPING.to(RETURN_TO_HOME)
    on_return: TransitionList = RETURN_TO_HOME.to(LAND)
    on_finish: TransitionList = LAND.to(COMPLETE)

    def __init__(self, session: Session, home: WaypointStruct):
        super().__init__(session, home)

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        self.on_go()  # INIT -> LAUNCH
        self.begin_hop()  # LAUNCH -> HOPPING
        self.end_hop()  # HOPPING -> RETURN_TO_HOME
        self.on_return()  # RETURN_TO_HOME -> LAND
        self.on_finish()  # LAND -> COMPLETE

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def on_enter_INIT(self) -> None:
        print("requesting geofence from control/query/geofence")
        self.update_geofence()
        print(f"geofence points: {len(self.geofence)}")

    def on_enter_LAUNCH(self) -> None:
        print("waiting to be at takeoff alt")
        while self.pos.alt < TAKEOFF_ALT_MM:
            print(f"alt (mm): {self.pos.alt}")
            time.sleep(0.300)

    def on_enter_HOPPING(self) -> None:
        self._run_object_localization_lawnmower()

    def on_enter_RETURN_TO_HOME(self) -> None:
        # self.navigate_waypoint(self.home, 0.5)
        pass

    def on_enter_LAND(self) -> None:
        print("LAND")
        # ground = WaypointStruct(
        #     self.home.lat,
        #     self.home.lon,
        #     self.home.alt + 500.0,  # half a meter agl
        # )
        # self.navigate_waypoint(ground, 0.4, speed=TEST_SPEED)

    # ------------------------------------------------------------------
    # Mission logic
    # ------------------------------------------------------------------

    def _run_object_localization_lawnmower(self) -> None:
        print("starting object-localization lawnmower sweep")
        print(f"geofence = {self.geofence}")

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

        for pt in sweep_points:
            print(f"pt! = lat: {pt.lat}, lon: {pt.lon}, alt: {pt.alt}")

        print(f"generated {len(sweep_points)} sweep waypoints")

        for i, waypoint in enumerate(sweep_points):
            print(f"going to sweep waypoint {i + 1}/{len(sweep_points)}")
            print(
                f"→ SENT WAYPOINT {i + 1}: lat={waypoint.lat}, "
                f"lon={waypoint.lon}, alt={waypoint.alt}"
            )

            success = self.navigate_waypoint(
                waypoint,
                a_rad=2.0,
                timeout=60.0,
            )

            time.sleep(1.0)

            if not success:
                print("failed to reach sweep waypoint")
