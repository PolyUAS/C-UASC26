use a8mini_camera_rs::A8Mini;
use flatbuffers::{self, FlatBufferBuilder};
use mavlink::Connection;
use mavlink::ardupilotmega::{
    ALTITUDE_DATA, ATTITUDE_DATA, COMMAND_LONG_DATA, CONTROL_SYSTEM_STATE_DATA,
    EXTENDED_SYS_STATE_DATA, HEARTBEAT_DATA, HIL_STATE_QUATERNION_DATA, MavCmd, MavMessage,
    MavModeFlag, STATUSTEXT_DATA,
};

use mavlink::error::MessageReadError;

use mavlink::{self, MavConnection, MavlinkVersion, Message};
use std::error::Error;
use std::future;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, sleep};
use std::time::Duration;
use zenoh::{self, Wait};

mod state_generated;

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let rt = tokio::runtime::Runtime::new().expect("Failed to initialize runtime");
    let camera = rt
        .block_on(A8Mini::connect())
        .expect("Failed to connect to camera");

    // Configuration
    // Recommended – TCP (very reliable for localhost)
    // let mavlink_conn_str = "tcp:127.0.0.1:5760".to_string();

    // or UDP (if you started SITL with --out udp:...)
    // let mavlink_conn_str = "udpout:127.0.0.1:14560".to_string();
    // or even bidirectional UDP
    // let mavlink_conn_str = "udpin:0.0.0.0:14560".to_string();  // if Rust binds & waits
    //
    // println!("Compiled with TCP support? {}", cfg!(feature = "tcp"));

    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());

    // Setup Zenoh
    let session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).wait()?;

    // Setup MAVLink
    let mut mavconn =
        mavlink::connect::<MavMessage>(&mavlink_conn_str).expect("Failed to connect to MAVLink");
    mavconn.set_protocol_version(MavlinkVersion::V2);
    let vehicle = Arc::new(mavconn);

    println!("Connected to Mavlink on {:?}", mavlink_conn_str);

    // Setup publisher
    let state_pub = session.declare_publisher("mavlink/state").wait()?;
    sleep(Duration::from_millis(500)); // Wait for subscribers

    // Shutdown signal ptr
    let running = Arc::new(AtomicBool::new(true));

    // Heartbeat thread
    let vehicle_heartbeat = vehicle.clone();
    let running_heartbeat = running.clone();
    let heartbeat_handle =
        thread::spawn(move || heartbeat_loop(&vehicle_heartbeat, &running_heartbeat));

    // Receiver thread
    let vehicle_recv = vehicle.clone();
    let running_recv = running.clone();
    let receiver_handle =
        thread::spawn(move || receiver_loop(&vehicle_recv, &state_pub, &running_recv));

    // Initialize state stream
    initialize_state_stream(&vehicle, Duration::from_millis(20))
        .inspect_err(|e| eprintln!("Failed to initialize state stream: {:?}", e))?;

    // Wait for Ctrl+C
    ctrlc::set_handler(move || {
        running.store(false, Ordering::Release);
    })
    .expect("Error setting Ctrl-C handler");

    // Join threads
    heartbeat_handle.join().unwrap();
    receiver_handle.join().unwrap();

    session
        .close()
        .wait()
        .expect("Failed to close mavlink's zenoh session:");

    Ok(())
}

fn heartbeat_loop(vehicle: &Arc<Connection<MavMessage>>, running: &Arc<AtomicBool>) {
    let heartbeat_msg = MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_GCS, // ← change to GCS (common for ground stations)
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID, // GCS doesn't have autopilot
        base_mode: MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3, // usually 3 or 4
    });
    // println!("Heartbeat ENABLED!");

    while running.load(Ordering::Acquire) {
        if let Err(e) = vehicle.send_default(&heartbeat_msg) {
            eprintln!("Heartbeat send failed: {:?}", e);
        }
        sleep(Duration::from_secs(1));

        // println!("Heartbeat.");
    }
}

fn receiver_loop(
    vehicle: &Arc<Connection<MavMessage>>,
    state_pub: &zenoh::pubsub::Publisher,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Acquire) {
        // println!("Receiver thread running!");
        match vehicle.recv() {
            Ok((header, msg)) => {
                match msg {
                    MavMessage::HIL_STATE_QUATERNION(data) => {
                        publish_state(state_pub, &data).expect("Failed to publish state: ");
                    }

                    // MavMessage::GPS_RAW_INT(data) => {
                    //     dbg!(&data);
                    // }
                    // MavMessage::GLOBAL_POSITION_INT(data) => {
                    //     println!("lat: {}, lon: {}, alt: {}", data.lat, data.lon, data.alt);
                    // }

                    // MavMessage::ATTITUDE(data) => {
                    //     println!(
                    //         "pitch: {}, roll: {}, yaw: {}",
                    //         data.pitch, data.roll, data.yaw,
                    //     );
                    // }
                    _ => {
                        // Ignore other messages
                        // println!(
                        //     "Header ==== Component Id: {}, System Id : {}, Sequence Number: {}",
                        //     header.component_id, header.system_id, header.sequence
                        // );
                        println!(
                            "Message ==== Name: {}, Id: {}, ",
                            msg.message_name(),
                            msg.message_id()
                        );
                    }
                }
            }

            Err(MessageReadError::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                sleep(Duration::from_micros(50));
            }
            Err(e) => {
                eprintln!("Control recv error: {:?}", e);
            }
        }
    }
}

#[inline]
fn publish_state(
    publisher: &zenoh::pubsub::Publisher,
    data: &mavlink::ardupilotmega::HIL_STATE_QUATERNION_DATA,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Build FlatBuffer message
    let mut builder = FlatBufferBuilder::new();

    //Create the struct directly
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

    // Get the serialized bytes
    let payload = builder.finished_data();

    println!("Published state!");
    dbg!(&data);
    // Publish
    publisher.put(payload).wait()?;
    Ok(())
}

fn initialize_state_stream(
    vehicle: &Arc<Connection<MavMessage>>,
    interval: Duration, // e.g., 20 ms → 50 Hz
) -> Result<(), mavlink::error::MessageWriteError> {
    let interval_us = interval.as_micros(); // microseconds

    let cmd_msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        confirmation: 0,
        param1: MavMessage::ALTITUDE(ALTITUDE_DATA::default()).message_id() as f32,
        param2: interval_us as f32, // interval in us (positive = start streaming)
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });

    let mut attempts = 0;
    loop {
        match vehicle.send_default(&cmd_msg) {
            Ok(_) => {
                println!(
                    "Requested ALTITUDE at ~{} Hz (attempt {})",
                    1000000 / interval_us,
                    attempts + 1
                );
            }
            Err(e) => eprintln!("SET_MESSAGE_INTERVAL failed: {:?}", e),
        }
        attempts += 1;
        if attempts >= 10 {
            eprintln!("Gave up after {} attempts", attempts);
            break;
        }
        sleep(Duration::from_secs(2)); // retry slower to avoid spam
    }
    Ok(())
}
