// control — MAVLink ↔ Zenoh bridge for ArduPilot + SIYI A8 Mini gimbal.
//
// Build Priorities implemented:
//   BP1: HEARTBEAT / SYS_STATUS extraction → vehicle/heartbeat/fc,
//        VehicleTelemetry FlatBuffer on vehicle/telemetry/fc.
//   BP2: Command subscriber on guidance/mission/command → MAVLink dispatch
//        → FlatBuffer CommandAck on guidance/mission/ack.
//
// Preserves control/out/state (legacy raw StateVector) for debug consumers.

use flatbuffers::FlatBufferBuilder;
use mavlink::Message;
use mavlink::ardupilotmega::{
    MavCmd, MavMessage, MavModeFlag, COMMAND_LONG_DATA, HEARTBEAT_DATA,
    HIL_STATE_QUATERNION_DATA, SET_POSITION_TARGET_GLOBAL_INT_DATA,
};
use mavlink::error::MessageReadError;
use mavlink::{Connection, MavConnection, MavlinkVersion};

use siyi_protocol::a8mini_udp::{self, MAX_MESSAGE_SIZE};

use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use std::time::Duration;

use zenoh::Wait;

// ── Legacy FlatBuffers (checked-in, from mavlink branch) ────────────────────
#[allow(unused_imports, dead_code, clippy::all)]
mod gimbal_generated;
#[allow(unused_imports, dead_code, clippy::all)]
mod state_generated;

// ── Build-time generated FlatBuffers from schemas/ ──────────────────────────
mod generated;

use generated::guidance_ack;
use generated::guidance_cmd;
use generated::vehicle;

// ─────────────────────────────────────────────────────────────────────────────
// Shared state passed between threads via Arc<Mutex<_>>
// ─────────────────────────────────────────────────────────────────────────────

struct SharedVehicleState {
    battery_pct: f32,
    flight_mode: String,
}

impl Default for SharedVehicleState {
    fn default() -> Self {
        Self {
            battery_pct: 100.0,
            flight_mode: "GUIDED".to_string(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ArduPilot Copter mode decode
// ─────────────────────────────────────────────────────────────────────────────

fn decode_ardupilot_copter_mode(custom_mode: u32) -> &'static str {
    match custom_mode {
        0 => "STABILIZE",
        2 => "ALT_HOLD",
        3 => "AUTO",
        4 => "GUIDED",
        5 => "LOITER",
        6 => "RTL",
        7 => "CIRCLE",
        9 => "LAND",
        16 => "POSHOLD",
        17 => "BRAKE",
        18 => "THROW",
        20 => "GUIDED_NOGPS",
        21 => "SMART_RTL",
        27 => "AUTO_RTL",
        _ => "UNKNOWN",
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Command dispatch — translates FlatBuffer MissionCommand → MAVLink
// ─────────────────────────────────────────────────────────────────────────────

fn dispatch_command(
    vehicle_conn: &Arc<Connection<MavMessage>>,
    cmd_buf: &[u8],
) -> (String, guidance_ack::AckStatus, String) {
    let cmd = match flatbuffers::root::<guidance_cmd::MissionCommand>(cmd_buf) {
        Ok(c) => c,
        Err(e) => {
            return (
                "unknown".to_string(),
                guidance_ack::AckStatus::REJECTED,
                format!("flatbuf_decode_error:{e}"),
            );
        }
    };

    let command_id = cmd.command_id().to_string();
    let cmd_type = cmd.command_type();

    let (status, reason) = match cmd_type {
        guidance_cmd::CommandType::ARM => send_command_long(
            vehicle_conn,
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),

        guidance_cmd::CommandType::TAKEOFF => {
            let alt = cmd.target_alt_m();
            send_command_long(
                vehicle_conn,
                MavCmd::MAV_CMD_NAV_TAKEOFF,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt],
            )
        }

        guidance_cmd::CommandType::GOTO => {
            let lat = cmd.target_lat_deg();
            let lon = cmd.target_lon_deg();
            let alt = cmd.target_alt_m();
            send_goto(vehicle_conn, lat, lon, alt)
        }

        guidance_cmd::CommandType::HOLD => send_command_long(
            vehicle_conn,
            MavCmd::MAV_CMD_DO_PAUSE_CONTINUE,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),

        guidance_cmd::CommandType::RTL => send_command_long(
            vehicle_conn,
            MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),

        guidance_cmd::CommandType::LAND => send_command_long(
            vehicle_conn,
            MavCmd::MAV_CMD_NAV_LAND,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),

        guidance_cmd::CommandType::PAYLOAD_RELEASE => send_command_long(
            vehicle_conn,
            MavCmd::MAV_CMD_DO_SET_SERVO,
            [9.0, 1100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),

        guidance_cmd::CommandType::PAYLOAD_SECURE => send_command_long(
            vehicle_conn,
            MavCmd::MAV_CMD_DO_SET_SERVO,
            [9.0, 1900.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),

        guidance_cmd::CommandType::CAMERA_POINT | guidance_cmd::CommandType::CAMERA_TRACK => {
            (guidance_ack::AckStatus::ACCEPTED, "routed_to_gimbal".to_string())
        }

        guidance_cmd::CommandType::MISSION_ABORT => send_command_long(
            vehicle_conn,
            MavCmd::MAV_CMD_NAV_LAND,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),

        _ => (
            guidance_ack::AckStatus::REJECTED,
            format!("unsupported_command_type:{}", cmd_type.0),
        ),
    };

    (command_id, status, reason)
}

fn send_command_long(
    vehicle_conn: &Arc<Connection<MavMessage>>,
    command: MavCmd,
    params: [f32; 7],
) -> (guidance_ack::AckStatus, String) {
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command,
        confirmation: 0,
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        param5: params[4],
        param6: params[5],
        param7: params[6],
    });

    match vehicle_conn.send_default(&msg) {
        Ok(_) => (guidance_ack::AckStatus::ACCEPTED, String::new()),
        Err(e) => (
            guidance_ack::AckStatus::FAILED,
            format!("mavlink_send_error:{e}"),
        ),
    }
}

fn send_goto(
    vehicle_conn: &Arc<Connection<MavMessage>>,
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f32,
) -> (guidance_ack::AckStatus, String) {
    let msg = MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
        time_boot_ms: 0,
        target_system: 1,
        target_component: 1,
        coordinate_frame: mavlink::ardupilotmega::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        type_mask: mavlink::ardupilotmega::PositionTargetTypemask::from_bits_truncate(
            0b0000_1111_1111_1000,
        ),
        lat_int: (lat_deg * 1e7) as i32,
        lon_int: (lon_deg * 1e7) as i32,
        alt: alt_m,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        afx: 0.0,
        afy: 0.0,
        afz: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
    });

    match vehicle_conn.send_default(&msg) {
        Ok(_) => (guidance_ack::AckStatus::ACCEPTED, String::new()),
        Err(e) => (
            guidance_ack::AckStatus::FAILED,
            format!("mavlink_send_error:{e}"),
        ),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FlatBuffer builders for outbound messages
// ─────────────────────────────────────────────────────────────────────────────

fn build_vehicle_telemetry(
    data: &HIL_STATE_QUATERNION_DATA,
    shared: &SharedVehicleState,
) -> Vec<u8> {
    let mut builder = FlatBufferBuilder::with_capacity(256);

    let flight_mode = builder.create_string(&shared.flight_mode);

    let telem = vehicle::VehicleTelemetry::create(
        &mut builder,
        &vehicle::VehicleTelemetryArgs {
            timestamp_ms: data.time_usec / 1000,
            lat_deg: data.lat as f64 / 1e7,
            lon_deg: data.lon as f64 / 1e7,
            alt_m: data.alt as f64 / 1000.0,
            vn_mps: data.vx as f32 / 100.0,
            ve_mps: data.vy as f32 / 100.0,
            vd_mps: data.vz as f32 / 100.0,
            battery_pct: shared.battery_pct,
            flight_mode: Some(flight_mode),
            link_quality: 1.0,
        },
    );

    builder.finish(telem, None);
    builder.finished_data().to_vec()
}

fn build_command_ack(command_id: &str, status: guidance_ack::AckStatus, reason: &str) -> Vec<u8> {
    let mut builder = FlatBufferBuilder::with_capacity(128);

    let cmd_id = builder.create_string(command_id);
    let reason_str = if reason.is_empty() {
        None
    } else {
        Some(builder.create_string(reason))
    };

    let ack = guidance_ack::CommandAck::create(
        &mut builder,
        &guidance_ack::CommandAckArgs {
            command_id: Some(cmd_id),
            status,
            reason: reason_str,
        },
    );

    builder.finish(ack, None);
    builder.finished_data().to_vec()
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    // ── Gimbal socket (SIYI A8 Mini) ────────────────────────────────────────
    let cam_socket = std::net::UdpSocket::bind("192.168.144.25:37260")
        .inspect_err(|e| eprintln!("[control] WARN: gimbal socket unavailable: {e}"))
        .ok();
    let cam_arc = cam_socket.map(Arc::new);

    // ── MAVLink connection ──────────────────────────────────────────────────
    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());
    let mut mavconn =
        mavlink::connect::<MavMessage>(&mavlink_conn_str).expect("Failed to connect to MAVLink");
    mavconn.set_protocol_version(MavlinkVersion::V2);
    let vehicle_conn = Arc::new(mavconn);
    println!("[control] Connected to MAVLink: {mavlink_conn_str}");

    // ── Zenoh session ───────────────────────────────────────────────────────
    let session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).wait()?;

    // ── Shared mutable state ────────────────────────────────────────────────
    let shared_state = Arc::new(Mutex::new(SharedVehicleState::default()));

    // ── Zenoh publishers ────────────────────────────────────────────────────
    let state_pub = session.declare_publisher("control/out/state").wait()?;
    let telemetry_pub = session.declare_publisher("vehicle/telemetry/fc").wait()?;
    let fc_heartbeat_pub = session.declare_publisher("vehicle/heartbeat/fc").wait()?;
    let ack_pub = session.declare_publisher("guidance/mission/ack").wait()?;

    // ── Gimbal subscriber ───────────────────────────────────────────────────
    if let Some(ref cam) = cam_arc {
        let cam_sock = cam.clone();
        // Gimbal heartbeat publisher — separate instance for the callback thread
        let gimbal_hb = session.declare_publisher("vehicle/heartbeat/gimbal").wait()?;
        session
            .declare_subscriber("control/in/gimbal")
            .callback(move |sample| {
                let bytes = sample.payload().to_bytes();
                match gimbal_generated::gimbal::root_as_state(&bytes) {
                    Ok(state) => {
                        let mut set_att = a8mini_udp::SetGimbalAttitudeRequest::default();
                        set_att.pitch =
                            ((state.pitch() * 10.0).round() as i16).clamp(-1350, 1350);
                        set_att.yaw = ((state.yaw() * 10.0).round() as i16).clamp(-900, 250);

                        let mut set_zoom = a8mini_udp::AbsoluteZoomRequest::default();
                        let zoom = state.zoom().clamp(1.0, 6.0);
                        set_zoom.zoom_int = zoom.floor() as u8;
                        set_zoom.zoom_float =
                            (((zoom - set_zoom.zoom_int as f32) * 10.0).round() as u8).clamp(0, 9);

                        let mut buffer = [0u8; MAX_MESSAGE_SIZE];
                        let _ = set_att.encode(&mut buffer);
                        let _ = cam_sock.send(&buffer);
                        let _ = set_zoom.encode(&mut buffer);
                        let _ = cam_sock.send(&buffer);

                        let _ = gimbal_hb.put(b"1".as_ref()).wait();
                    }
                    Err(e) => eprintln!("[control] gimbal parse error: {e}"),
                }
            })
            .wait()?;
    }

    // ── BP2: Command subscriber → MAVLink dispatch → ack ────────────────────
    let cmd_sub = session
        .declare_subscriber("guidance/mission/command")
        .wait()?;
    let vehicle_cmd = vehicle_conn.clone();
    let cmd_thread = thread::spawn(move || {
        loop {
            match cmd_sub.recv() {
                Ok(sample) => {
                    let payload = sample.payload().to_bytes();
                    let (command_id, status, reason) = dispatch_command(&vehicle_cmd, &payload);
                    let ack_bytes = build_command_ack(&command_id, status, &reason);
                    if let Err(e) = ack_pub.put(ack_bytes).wait() {
                        eprintln!("[control] ack publish error: {e}");
                    }
                }
                Err(e) => {
                    eprintln!("[control] command recv error: {e}");
                    break;
                }
            }
        }
    });

    sleep(Duration::from_millis(500));

    // ── Shutdown signal ─────────────────────────────────────────────────────
    let running = Arc::new(AtomicBool::new(true));

    // ── Heartbeat thread ────────────────────────────────────────────────────
    let vehicle_hb = vehicle_conn.clone();
    let running_hb = running.clone();
    let hb_thread = thread::spawn(move || heartbeat_loop(&vehicle_hb, &running_hb));

    // ── Receiver thread ─────────────────────────────────────────────────────
    let vehicle_rx = vehicle_conn.clone();
    let running_rx = running.clone();
    let shared_rx = shared_state.clone();
    let rx_thread = thread::spawn(move || {
        receiver_loop(
            &vehicle_rx,
            &state_pub,
            &telemetry_pub,
            &fc_heartbeat_pub,
            &shared_rx,
            &running_rx,
        )
    });

    // ── Request HIL_STATE_QUATERNION at 50 Hz ───────────────────────────────
    initialize_state_stream(&vehicle_conn, Duration::from_millis(20))
        .inspect_err(|e| eprintln!("[control] state stream init error: {e:?}"))?;

    // ── Ctrl+C handler ──────────────────────────────────────────────────────
    ctrlc::set_handler({
        let r = running.clone();
        move || r.store(false, Ordering::Release)
    })
    .expect("Error setting Ctrl-C handler");

    hb_thread.join().unwrap();
    rx_thread.join().unwrap();
    drop(cmd_thread);

    session.close().wait().expect("Failed to close Zenoh session");
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread functions
// ─────────────────────────────────────────────────────────────────────────────

fn heartbeat_loop(vehicle: &Arc<Connection<MavMessage>>, running: &Arc<AtomicBool>) {
    let heartbeat_msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_GCS,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });

    while running.load(Ordering::Acquire) {
        if let Err(e) = vehicle.send_default(&heartbeat_msg) {
            eprintln!("[control] heartbeat send error: {e:?}");
        }
        sleep(Duration::from_secs(1));
    }
}

fn receiver_loop(
    vehicle: &Arc<Connection<MavMessage>>,
    state_pub: &zenoh::pubsub::Publisher,
    telemetry_pub: &zenoh::pubsub::Publisher,
    fc_heartbeat_pub: &zenoh::pubsub::Publisher,
    shared_state: &Arc<Mutex<SharedVehicleState>>,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Acquire) {
        match vehicle.recv() {
            Ok((_header, msg)) => match msg {
                MavMessage::HIL_STATE_QUATERNION(ref data) => {
                    if let Err(e) = publish_state_legacy(state_pub, data) {
                        eprintln!("[control] legacy state pub error: {e}");
                    }
                    let shared = shared_state.lock().unwrap_or_else(|e| e.into_inner());
                    let telem_bytes = build_vehicle_telemetry(data, &shared);
                    drop(shared);
                    if let Err(e) = telemetry_pub.put(telem_bytes).wait() {
                        eprintln!("[control] telemetry pub error: {e}");
                    }
                }
                MavMessage::HEARTBEAT(ref data) => {
                    let _ = fc_heartbeat_pub.put(b"1".as_ref()).wait();
                    let mode = decode_ardupilot_copter_mode(data.custom_mode);
                    if let Ok(mut state) = shared_state.lock() {
                        state.flight_mode = mode.to_string();
                    }
                }
                MavMessage::SYS_STATUS(ref data) => {
                    let pct = if data.battery_remaining >= 0 {
                        data.battery_remaining as f32
                    } else {
                        -1.0
                    };
                    if let Ok(mut state) = shared_state.lock() {
                        state.battery_pct = pct;
                    }
                }
                MavMessage::FENCE_STATUS(ref data) => {
                    if data.breach_status != 0 {
                        eprintln!(
                            "[control] GEOFENCE BREACH type={} count={}",
                            data.breach_type as u8, data.breach_count
                        );
                    }
                }
                _ => {}
            },
            Err(MessageReadError::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                sleep(Duration::from_micros(50));
            }
            Err(e) => {
                eprintln!("[control] recv error: {e:?}");
            }
        }
    }
}

#[inline]
fn publish_state_legacy(
    publisher: &zenoh::pubsub::Publisher,
    data: &HIL_STATE_QUATERNION_DATA,
) -> Result<(), Box<dyn Error + Send + Sync>> {
    let mut builder = FlatBufferBuilder::new();
    let state = state_generated::mavlink_fb::StateVector::new(
        data.time_usec,
        &data.attitude_quaternion,
        data.rollspeed,
        data.pitchspeed,
        data.yawspeed,
        &[data.lat, data.lon, data.alt],
        &[data.vx, data.vy, data.vz],
        &[data.xacc, data.yacc, data.zacc],
        data.ind_airspeed,
        data.true_airspeed,
    );
    builder.push(state);
    publisher.put(builder.finished_data()).wait()?;
    Ok(())
}

fn initialize_state_stream(
    vehicle: &Arc<Connection<MavMessage>>,
    interval: Duration,
) -> Result<(), mavlink::error::MessageWriteError> {
    use mavlink::ardupilotmega::ALTITUDE_DATA;
    let interval_us = interval.as_micros();
    let altitude_msg_id =
        MavMessage::ALTITUDE(ALTITUDE_DATA::default()).message_id() as f32;

    let cmd_msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        confirmation: 0,
        param1: altitude_msg_id,
        param2: interval_us as f32,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });

    for attempt in 0..10 {
        match vehicle.send_default(&cmd_msg) {
            Ok(_) => println!(
                "[control] Requested ALTITUDE at ~{} Hz (attempt {})",
                1_000_000 / interval_us,
                attempt + 1
            ),
            Err(e) => eprintln!("[control] SET_MESSAGE_INTERVAL error: {e:?}"),
        }
        sleep(Duration::from_secs(2));
    }
    Ok(())
}