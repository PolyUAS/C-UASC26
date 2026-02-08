use mavlink::Connection;
use mavlink::ardupilotmega::{MavMessage, MavModeFlag};
use mavlink::error::MessageReadError;
use mavlink::{self, MavConnection, MavlinkVersion, Message};
use std::error::Error;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, sleep};
use std::time::Duration;
use zenoh::{self, Wait};

mod state_capnp;

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    // Configuration
    let mavlink_conn_str = std::env::var("MAVLINK_CONN").unwrap_or_else(|_| {
        "file:/Users/isaacpruett/Documents/PolyUAS/CUASC-2026/mavlink_bridge/test.txt".to_string()
    });

    // Setup Zenoh
    let session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).wait()?;

    // Setup MAVLink
    let mut mavconn =
        mavlink::connect::<MavMessage>(&mavlink_conn_str).expect("Failed to connect to MAVLink");
    mavconn.set_protocol_version(MavlinkVersion::V2);
    let vehicle = Arc::new(mavconn);

    // Setup publisher
    let state_pub = session.declare_publisher("mavlink/state").wait()?;
    sleep(Duration::from_millis(500)); // Wait for subscribers

    // Shutdown signal
    let running = Arc::new(AtomicBool::new(true));

    // Heartbeat thread
    let vehicle_heartbeat = vehicle.clone();
    let running_heartbeat = running.clone();
    let heartbeat_handle =
        thread::spawn(move || heartbeat_loop(&vehicle_heartbeat, &running_heartbeat));

    // Initialize state stream
    initialize_state_stream(&vehicle, Duration::from_millis(20))
        .inspect_err(|e| eprintln!("Failed to initialize state stream: {:?}", e))?;

    // Receiver thread
    let vehicle_recv = vehicle.clone();
    let running_recv = running.clone();
    let receiver_handle =
        thread::spawn(move || receiver_loop(&vehicle_recv, &state_pub, &running_recv));

    // Wait for Ctrl+C
    ctrlc::set_handler(move || {
        running.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    // Join threads
    heartbeat_handle.join().unwrap();
    receiver_handle.join().unwrap();

    Ok(())
}

fn heartbeat_loop(vehicle: &Arc<Connection<MavMessage>>, running: &Arc<AtomicBool>) {
    let heartbeat_msg = MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_QUADROTOR,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    });
    // println!("Heartbeat ENABLED!");

    while running.load(Ordering::Acquire) {
        if let Err(e) = vehicle.send_default(&heartbeat_msg) {
            eprintln!("Heartbeat send failed: {:?}", e);
        }
        sleep(Duration::from_secs(1));
    }
}

fn receiver_loop(
    vehicle: &Arc<Connection<MavMessage>>,
    state_pub: &zenoh::pubsub::Publisher,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Acquire) {
        match vehicle.recv() {
            Ok((_header, MavMessage::HIL_STATE_QUATERNION(data))) => {
                publish_state(state_pub, &data).expect("Failed to publish state: ");
            }
            Ok(_) => {} // Ignore other messages
            Err(MessageReadError::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                sleep(Duration::from_millis(100));
            }
            Err(e) => {
                eprintln!("Message reciever read error: {:?}", e);
            }
        }
    }
}

fn publish_state(
    publisher: &zenoh::pubsub::Publisher,
    data: &mavlink::ardupilotmega::HIL_STATE_QUATERNION_DATA,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Build Cap'n Proto message
    let mut message = capnp::message::Builder::new_default();
    let mut state = message.init_root::<state_capnp::hil_state_quaternion::Builder>();

    // Set timestamp
    state.set_time_usec(data.time_usec);

    // Set attitude quaternion (w, x, y, z)
    let mut quat = state.reborrow().init_attitude_quaternion();
    quat.set_w(data.attitude_quaternion[0]);
    quat.set_x(data.attitude_quaternion[1]);
    quat.set_y(data.attitude_quaternion[2]);
    quat.set_z(data.attitude_quaternion[3]);

    // Set angular rates
    state.set_rollspeed(data.rollspeed);
    state.set_pitchspeed(data.pitchspeed);
    state.set_yawspeed(data.yawspeed);

    // Set position
    state.set_lat(data.lat);
    state.set_lon(data.lon);
    state.set_alt(data.alt);

    // Set velocities
    state.set_vx(data.vx);
    state.set_vy(data.vy);
    state.set_vz(data.vz);

    // Set airspeed
    state.set_ind_airspeed(data.ind_airspeed);
    state.set_true_airspeed(data.true_airspeed);

    // Set accelerations
    state.set_xacc(data.xacc);
    state.set_yacc(data.yacc);
    state.set_zacc(data.zacc);

    // Serialize to bytes
    let payload = capnp::serialize::write_message_segments_to_words(&message);

    // Publish
    publisher.put(payload).wait()?;

    Ok(())
}

fn initialize_state_stream(
    vehicle: &Arc<Connection<MavMessage>>,
    interval: Duration,
) -> Result<usize, mavlink::error::MessageWriteError> {
    vehicle.send_default(&MavMessage::COMMAND_LONG(
        mavlink::ardupilotmega::COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            confirmation: 0,
            param1: MavMessage::HIL_STATE_QUATERNION(Default::default()).message_id() as f32,
            param2: interval.as_micros() as f32,
            ..Default::default()
        },
    ))
}
