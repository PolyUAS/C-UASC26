use std::time::{SystemTime, UNIX_EPOCH};

/// Snapshot of sim state in telemetry units, ready for serialisation.
pub struct SimState {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    pub vn_mps: f64,
    pub ve_mps: f64,
    pub vd_mps: f64,
    pub battery_pct: f64,
    pub flight_mode: String,
}

/// Serialise [`SimState`] to JSON bytes for the `sim/mujoco/state` topic.
///
/// This topic is sim-internal only. The guidance `MuJoCoTelemetryAdapter`
/// consumes this and translates to FlatBuffers `VehicleTelemetry` on
/// `vehicle/telemetry/fc`.
///
/// Uses the nested format expected by `MuJoCoTelemetryAdapter`
/// (see `mujoco_telemetry.py`).
pub fn serialize_state(state: &SimState) -> Vec<u8> {
    let ts_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;

    format!(
        r#"{{"timestamp_ms":{ts_ms},"position":{{"lat_deg":{lat},"lon_deg":{lon},"alt_m":{alt}}},"velocity_ned":{{"vn_mps":{vn},"ve_mps":{ve},"vd_mps":{vd}}},"battery_pct":{bat},"flight_mode":"{mode}","link_quality":1.0}}"#,
        ts_ms = ts_ms,
        lat = state.lat_deg,
        lon = state.lon_deg,
        alt = state.alt_m,
        vn = state.vn_mps,
        ve = state.ve_mps,
        vd = state.vd_mps,
        bat = state.battery_pct,
        mode = state.flight_mode,
    )
    .into_bytes()
}
