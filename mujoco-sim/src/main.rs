use std::time::Duration;
use zenoh::Wait;

mod actuation;
mod publisher;
mod sim;

fn main() -> anyhow::Result<()> {
    let session = zenoh::open(
        zenoh::Config::from_env().unwrap_or(zenoh::Config::default()),
    )
    .wait()
    .map_err(|e| anyhow::anyhow!("zenoh open failed: {e}"))?;

    let state_pub = session
        .declare_publisher("sim/mujoco/state")
        .wait()
        .map_err(|e| anyhow::anyhow!("{e}"))?;
    let cmd_sub = session
        .declare_subscriber("guidance/mission/command")
        .wait()
        .map_err(|e| anyhow::anyhow!("{e}"))?;

    // Publish heartbeats so navigation watchdog does not escalate in sim mode.
    let fc_heartbeat_pub = session
        .declare_publisher("vehicle/heartbeat/fc")
        .wait()
        .map_err(|e| anyhow::anyhow!("{e}"))?;
    let gimbal_heartbeat_pub = session
        .declare_publisher("vehicle/heartbeat/gimbal")
        .wait()
        .map_err(|e| anyhow::anyhow!("{e}"))?;
    let camera_heartbeat_pub = session
        .declare_publisher("vehicle/heartbeat/camera")
        .wait()
        .map_err(|e| anyhow::anyhow!("{e}"))?;
    let air_unit_heartbeat_pub = session
        .declare_publisher("vehicle/heartbeat/air_unit")
        .wait()
        .map_err(|e| anyhow::anyhow!("{e}"))?;

    let mut drone_sim = sim::DroneSim::load("models/drone.xml");
    let mut tick: u32 = 0;

    eprintln!("[mujoco-sim] running at 50 Hz — publishing sim/mujoco/state");

    loop {
        drone_sim.step();
        let state = drone_sim.state();
        let payload = publisher::serialize_state(&state);
        state_pub
            .put(payload)
            .wait()
            .map_err(|e| anyhow::anyhow!("{e}"))?;

        // Navigation watchdog needs heartbeats.
        // Publish at ~10 Hz (every 5 ticks of 50 Hz loop).
        if tick % 5 == 0 {
            let _ = fc_heartbeat_pub.put(b"1".as_ref()).wait();
            let _ = gimbal_heartbeat_pub.put(b"1".as_ref()).wait();
            let _ = camera_heartbeat_pub.put(b"1".as_ref()).wait();
            let _ = air_unit_heartbeat_pub.put(b"1".as_ref()).wait();
        }

        // Drain pending commands non-blocking.
        while let Some(sample) = cmd_sub.try_recv().ok().flatten() {
            let bytes = sample.payload().to_bytes();
            actuation::apply_command(&mut drone_sim, &bytes);
        }

        tick = tick.wrapping_add(1);
        std::thread::sleep(Duration::from_millis(20)); // 50 Hz
    }
}
