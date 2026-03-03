use std::collections::HashMap;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use zenoh::Wait;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Component {
    FlightController,
    Camera,
    Gimbal,
    AirUnit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SupervisorState {
    Nominal,
    Degraded,
    Critical,
    Failsafe,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SupervisorAction {
    Continue,
    Hold,
    Rtl,
    Abort,
}

#[derive(Debug, Clone, Copy)]
struct Thresholds {
    degraded_ms: u64,
    critical_ms: u64,
    failsafe_ms: u64,
}

impl Default for Thresholds {
    fn default() -> Self {
        Self {
            degraded_ms: 1000,
            critical_ms: 3000,
            failsafe_ms: 7000,
        }
    }
}

struct Watchdog {
    thresholds: Thresholds,
    last_seen: HashMap<Component, Instant>,
}

impl Watchdog {
    fn new(thresholds: Thresholds) -> Self {
        Self {
            thresholds,
            last_seen: HashMap::new(),
        }
    }

    fn mark_seen(&mut self, component: Component) {
        self.last_seen.insert(component, Instant::now());
    }

    fn evaluate(&self) -> (SupervisorState, SupervisorAction, String) {
        let now = Instant::now();
        let mut worst = SupervisorState::Nominal;
        let mut reasons: Vec<String> = Vec::new();

        for component in [
            Component::FlightController,
            Component::Camera,
            Component::Gimbal,
            Component::AirUnit,
        ] {
            let age_ms = self
                .last_seen
                .get(&component)
                .map(|last| now.duration_since(*last).as_millis() as u64)
                .unwrap_or(u64::MAX);

            let state = self.state_for_age(age_ms);
            if rank(state) > rank(worst) {
                worst = state;
            }
            if state != SupervisorState::Nominal {
                reasons.push(format!("{component:?}:{age_ms}ms"));
            }
        }

        let action = action_for_state(worst);
        let reason = if reasons.is_empty() {
            "all_components_nominal".to_string()
        } else {
            reasons.join(",")
        };

        (worst, action, reason)
    }

    fn state_for_age(&self, age_ms: u64) -> SupervisorState {
        if age_ms >= self.thresholds.failsafe_ms {
            return SupervisorState::Failsafe;
        }
        if age_ms >= self.thresholds.critical_ms {
            return SupervisorState::Critical;
        }
        if age_ms >= self.thresholds.degraded_ms {
            return SupervisorState::Degraded;
        }
        SupervisorState::Nominal
    }
}

fn rank(state: SupervisorState) -> i32 {
    match state {
        SupervisorState::Nominal => 0,
        SupervisorState::Degraded => 1,
        SupervisorState::Critical => 2,
        SupervisorState::Failsafe => 3,
    }
}

fn action_for_state(state: SupervisorState) -> SupervisorAction {
    match state {
        SupervisorState::Nominal => SupervisorAction::Continue,
        SupervisorState::Degraded => SupervisorAction::Hold,
        SupervisorState::Critical => SupervisorAction::Rtl,
        SupervisorState::Failsafe => SupervisorAction::Abort,
    }
}

fn now_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::from_millis(0))
        .as_millis()
}

fn main() -> zenoh::Result<()> {
    let session = zenoh::open(zenoh::Config::from_env().unwrap_or_default()).wait()?;

    let fc_sub = session.declare_subscriber("vehicle/heartbeat/fc").wait()?;
    let camera_sub = session
        .declare_subscriber("vehicle/heartbeat/camera")
        .wait()?;
    let gimbal_sub = session
        .declare_subscriber("vehicle/heartbeat/gimbal")
        .wait()?;
    let air_unit_sub = session
        .declare_subscriber("vehicle/heartbeat/air_unit")
        .wait()?;
    let supervisor_pub = session
        .declare_publisher("guidance/supervisor/signal")
        .wait()?;

    let mut watchdog = Watchdog::new(Thresholds::default());

    loop {
        if fc_sub.try_recv().is_ok() {
            watchdog.mark_seen(Component::FlightController);
        }
        if camera_sub.try_recv().is_ok() {
            watchdog.mark_seen(Component::Camera);
        }
        if gimbal_sub.try_recv().is_ok() {
            watchdog.mark_seen(Component::Gimbal);
        }
        if air_unit_sub.try_recv().is_ok() {
            watchdog.mark_seen(Component::AirUnit);
        }

        let (state, action, reason) = watchdog.evaluate();
        let payload = format!(
            "{{\"ts_ms\":{},\"state\":\"{:?}\",\"action\":\"{:?}\",\"reason\":\"{}\"}}",
            now_ms(),
            state,
            action,
            reason
        );
        supervisor_pub.put(payload).wait()?;

        std::thread::sleep(Duration::from_millis(100));
    }
}
