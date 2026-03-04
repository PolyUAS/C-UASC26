use crate::sim::DroneSim;

/// Apply a command payload to the drone sim.
///
/// Receives JSON bytes from `guidance/mission/command`.
/// Translates recognised command types to sim physics calls.
///
/// If decode fails, log and skip — never panic in the sim loop.
///
/// TODO: replace JSON fallback with FlatBuffers `MissionCommand` decode
/// once build-time codegen is wired for this crate.
pub fn apply_command(sim: &mut DroneSim, payload: &[u8]) {
    let text = match std::str::from_utf8(payload) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("[mujoco-sim] command payload not UTF-8: {e}");
            return;
        }
    };

    let val: serde_json::Value = match serde_json::from_str(text) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("[mujoco-sim] command JSON parse error: {e}");
            return;
        }
    };

    match val.get("command_type").and_then(|v| v.as_str()) {
        Some("goto") => {
            let lat = val
                .get("target_lat_deg")
                .and_then(|v| v.as_f64())
                .unwrap_or(sim.origin_lat());
            let lon = val
                .get("target_lon_deg")
                .and_then(|v| v.as_f64())
                .unwrap_or(sim.origin_lon());
            let alt = val
                .get("target_alt_m")
                .and_then(|v| v.as_f64())
                .unwrap_or(10.0);
            eprintln!("[mujoco-sim] GOTO lat={lat:.6} lon={lon:.6} alt={alt:.1}m");
            sim.apply_goto(lat, lon, alt);
        }
        Some("rtl") => {
            eprintln!("[mujoco-sim] RTL");
            sim.apply_goto(sim.origin_lat(), sim.origin_lon(), 10.0);
            sim.mode = "RTL".to_string();
        }
        Some("hold") => {
            eprintln!("[mujoco-sim] HOLD");
            sim.vel = [0.0; 3];
            sim.mode = "LOITER".to_string();
        }
        Some("land") => {
            eprintln!("[mujoco-sim] LAND");
            sim.vel = [0.0, 0.0, 0.5]; // vd positive = descend (NED)
            sim.mode = "LAND".to_string();
        }
        Some("arm") => {
            eprintln!("[mujoco-sim] ARM (no-op in sim)");
        }
        Some("takeoff") => {
            let alt = val
                .get("target_alt_m")
                .and_then(|v| v.as_f64())
                .unwrap_or(10.0);
            eprintln!("[mujoco-sim] TAKEOFF to {alt:.1}m");
            // Fly to current lat/lon at target altitude
            let state = sim.state();
            sim.apply_goto(state.lat_deg, state.lon_deg, alt);
            sim.mode = "GUIDED".to_string();
        }
        Some(other) => {
            eprintln!("[mujoco-sim] unknown command_type: {other}");
        }
        None => {
            eprintln!("[mujoco-sim] command missing command_type field");
        }
    }
}
