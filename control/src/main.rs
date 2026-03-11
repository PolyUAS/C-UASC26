use flatbuffers::{self, FlatBufferBuilder};
use mavlink::Connection;
use mavlink::ardupilotmega::{ALTITUDE_DATA, COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag};

use mavlink::error::MessageReadError;

use mavlink::{self, MavConnection, MavlinkVersion, Message};
use siyi_protocol::a8mini_udp::{self, MAX_MESSAGE_SIZE};
use std::error::Error;

use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, sleep};
use std::time::Duration;

use zenoh::{self, Wait};

use crate::state_generated::mavlink_fb::{
    Int16Vec3, Int32Vec3, Quaternion, StateVector, StateVectorArgs,
};

mod gimbal_generated;
mod state_generated;

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let cam_socket = std::net::UdpSocket::bind("192.168.144.25:37260")
        .expect("Failed to connect to A8 Mini's UDP socket");

    let cam_arc = Arc::new(cam_socket);

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
    let state_pub = session.declare_publisher("control/out/vehicle").wait()?;

    // Setup gimbal subscriber
    let gimbal_sub = session
        .declare_queryable("control/in/gimbal")
        .callback(move |sample| gimbal_motion(sample, cam_arc.clone()))
        .wait()?;

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
        .expect("Failed to close control's zenoh session:");

    Ok(())
}

fn gimbal_motion(query: zenoh::query::Query, cam_arc: Arc<UdpSocket>) {
    let cam_sock = cam_arc.clone();

    macro_rules! encode_and_send {
        ($msg:expr, $buf:expr, $sock:expr) => {
            match $msg.encode(&mut $buf) {
                Ok(_) => {
                    let _ = $sock.send(&$buf);
                }
                Err(e) => eprintln!("Encode error: {:?}", e),
            }
        };
    }

    let mut buffer = [0u8; MAX_MESSAGE_SIZE];
    encode_and_send!(
        &a8mini_udp::GimbalAttitudeRequest::default(),
        buffer,
        cam_sock
    );

    // Read the gimbal response
    let mut recv_buf = [0u8; MAX_MESSAGE_SIZE];
    let gimbal_state = match cam_sock.recv(&mut recv_buf) {
        Ok(len) => match a8mini_udp::GimbalAttitudeResponse::decode(&recv_buf[..len]) {
            Ok(attitude) => Some(attitude),
            Err(e) => {
                eprintln!("Failed to decode gimbal response: {:?}", e);
                None
            }
        },
        Err(e) => {
            eprintln!("Failed to receive gimbal response: {e}");
            None
        }
    };

    if let Some(attitude) = gimbal_state {
        let mut builder = FlatBufferBuilder::new();
        let mut packet = gimbal_generated::gimbal::StateBuilder::new(&mut builder);
        packet.add_pitch(attitude.pitch as f32 / 10.0);
        packet.add_yaw(attitude.yaw as f32 / 10.0);
        packet.add_roll(attitude.roll as f32 / 10.0);

        let payload = packet.finish();
        builder.finish(payload, None);
        let buf = builder.finished_data();

        let _ = query.reply("control/out/gimbal", buf).wait();
        println!("Current gimbal attitude: {:?}", attitude);
    }

    let bytes = match query.payload() {
        Some(b) => b.to_bytes().into_owned(),
        None => Vec::<u8>::new(),
    };

    match gimbal_generated::gimbal::root_as_state(&bytes) {
        Ok(state) => {
            let mut set_att = a8mini_udp::SetGimbalAttitudeRequest::default();
            set_att.pitch = ((state.pitch() * 10.0).round() as i16).clamp(-1350, 1350);
            set_att.yaw = ((state.yaw() * 10.0).round() as i16).clamp(-900, 250);

            let zoom = state.zoom().clamp(1.0, 6.0);
            let zoom_int = zoom.floor() as u8;
            let zoom_frac = (((zoom - zoom_int as f32) * 10.0).round() as u8).clamp(0, 9);

            let mut set_zoom = a8mini_udp::AbsoluteZoomRequest::default();
            set_zoom.zoom_int = zoom_int;
            set_zoom.zoom_float = zoom_frac;

            println!("Gimbal: yaw={} pitch={}", state.yaw(), state.pitch());
            println!("Zoom: int={zoom_int}, float={zoom_frac}, overall: {zoom_int}.{zoom_frac}");

            encode_and_send!(&set_att, buffer, cam_sock);
            encode_and_send!(&set_zoom, buffer, cam_sock);
        }
        Err(e) => eprintln!("Parse failed: {e}"),
    }
}

fn heartbeat_loop(vehicle: &Arc<Connection<MavMessage>>, running: &Arc<AtomicBool>) {
    let heartbeat_msg = MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_GCS,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });

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
        match vehicle.try_recv() {
            Ok((header, msg)) => {
                match msg {
                    MavMessage::HIL_STATE_QUATERNION(data) => {
                        publish_state(state_pub, &data);
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
    let state = StateVector::create(
        &mut builder,
        &StateVectorArgs {
            time_usec: data.time_usec,
            quaternion: Some(&Quaternion::new(
                data.attitude_quaternion[0],
                data.attitude_quaternion[1],
                data.attitude_quaternion[2],
                data.attitude_quaternion[3],
            )),
            rollspeed: data.rollspeed,
            pitchspeed: data.pitchspeed,
            yawspeed: data.yawspeed,
            pos: Some(&Int32Vec3::new(data.lat, data.lon, data.alt)),
            vel: Some(&Int16Vec3::new(data.vx, data.vy, data.vz)),
            acc: Some(&Int16Vec3::new(data.xacc, data.yacc, data.zacc)),
            ind_airspeed: data.ind_airspeed,
            true_airspeed: data.true_airspeed,
        },
    );
    builder.finish(state, None);

    // Get the serialized bytes
    let payload = builder.finished_data().to_vec();

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
