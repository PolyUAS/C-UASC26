from dataclasses import dataclass
from typing import Iterable

from guidance.contracts import (
    ComponentHealth,
    SupervisorAction,
    SupervisorSignal,
    SupervisorState,
)


@dataclass(slots=True)
class SupervisorThresholds:
    degraded_heartbeat_ms: int = 1000
    critical_heartbeat_ms: int = 3000
    failsafe_heartbeat_ms: int = 7000


class MissionSupervisor:
    def __init__(self, thresholds: SupervisorThresholds | None = None):
        self.thresholds = thresholds or SupervisorThresholds()
        self.current_state = SupervisorState.NOMINAL

    def evaluate(self, health: Iterable[ComponentHealth], timestamp_ms: int) -> SupervisorSignal:
        worst_state = SupervisorState.NOMINAL
        reasons: list[str] = []

        for item in health:
            state = self._state_for_age(item.heartbeat_age_ms)
            if self._severity_rank(state) > self._severity_rank(worst_state):
                worst_state = state
            if state != SupervisorState.NOMINAL:
                reasons.append(f"{item.component_name}:{item.heartbeat_age_ms}ms")

        self.current_state = worst_state
        action = self._action_for_state(worst_state)
        reason = ", ".join(reasons) if reasons else "all_components_nominal"

        return SupervisorSignal(
            timestamp_ms=timestamp_ms,
            state=worst_state,
            action=action,
            reason=reason,
        )

    def _state_for_age(self, heartbeat_age_ms: int) -> SupervisorState:
        if heartbeat_age_ms >= self.thresholds.failsafe_heartbeat_ms:
            return SupervisorState.FAILSAFE
        if heartbeat_age_ms >= self.thresholds.critical_heartbeat_ms:
            return SupervisorState.CRITICAL
        if heartbeat_age_ms >= self.thresholds.degraded_heartbeat_ms:
            return SupervisorState.DEGRADED
        return SupervisorState.NOMINAL

    @staticmethod
    def _action_for_state(state: SupervisorState) -> SupervisorAction:
        if state == SupervisorState.NOMINAL:
            return SupervisorAction.CONTINUE
        if state == SupervisorState.DEGRADED:
            return SupervisorAction.HOLD
        if state == SupervisorState.CRITICAL:
            return SupervisorAction.RTL
        return SupervisorAction.ABORT

    @staticmethod
    def _severity_rank(state: SupervisorState) -> int:
        return {
            SupervisorState.NOMINAL: 0,
            SupervisorState.DEGRADED: 1,
            SupervisorState.CRITICAL: 2,
            SupervisorState.FAILSAFE: 3,
        }[state]
