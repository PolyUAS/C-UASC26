use a8mini_camera_rs::A8Mini;
use a8mini_camera_rs::control::A8MiniComplexCommand;
use flatbuffers::{self, FlatBufferBuilder};
use mavlink::ardupilotmega::{
    ALTITUDE_DATA, COMMAND_LONG_DATA, MavCmd, MavMessage, MavModeFlag, MavTunnelPayloadType,
};
use mavlink::{AsyncMavConnection, Connection};

use mavlink::error::MessageReadError;

use mavlink::{self, MavConnection, MavlinkVersion, Message};
use zenoh::query::Query;

use std::error::Error;

use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, sleep};
use std::time::Duration;

use zenoh::{self};

use crate::state_generated::mavlink_fb::{
    Int16Vec3, Int32Vec3, Quaternion, StateVector, StateVectorArgs,
};

mod gimbal_generated;
mod state_generated;

type MavConn = Box<dyn AsyncMavConnection<MavMessage> + Send + Sync>;
type SharedMavConn = Arc<MavConn>;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());

    let a8mini_camera = a8mini_camera_rs::A8Mini::connect().await.unwrap();

    // Setup Zenoh
    let session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).await?;

    // Setup MAVLink
    let mut mavconn = mavlink::connect_async::<MavMessage>(&mavlink_conn_str)
        .await
        .expect("Failed to connect to MAVLink");
    mavconn.set_protocol_version(MavlinkVersion::V2);
    let vehicle = Arc::new(mavconn);

    println!("Connected to Mavlink on {:?}", mavlink_conn_str);

    // Setup publisher
    let state_pub = session.declare_publisher("control/out/vehicle").await.ok();

    let shared_camera = Arc::new(a8mini_camera);
    // Setup gimbal subscriber
    let gimbal_sub = session
        .declare_queryable("control/in/gimbal")
        .callback(move |sample| {
            let cam = shared_camera.clone();
            tokio::spawn(async move {
                gimbal_motion(sample, &cam).await;
            });
        })
        .await?;

    sleep(Duration::from_millis(500)); // Wait for subscribers

    // Shutdown signal ptr
    let running = Arc::new(AtomicBool::new(true));

    // Heartbeat thread
    let vehicle_heartbeat = vehicle.clone();
    let running_heartbeat = running.clone();

    let heartbeat_handle = tokio::spawn(async move {
        heartbeat_loop(&vehicle_heartbeat, &running_heartbeat).await;
    });

    // Receiver task
    let vehicle_recv = vehicle.clone();
    let running_recv = running.clone();

    let receiver_handle = tokio::spawn(async move {
        receiver_loop(
            &vehicle_recv,
            &state_pub
                .expect("Can't initialize reciever thread: state_publisher failed to initialize"),
            &running_recv,
        )
        .await;
    });
    // Initialize state stream
    initialize_state_stream(&vehicle, Duration::from_millis(20))
        .await
        .inspect_err(|e| eprintln!("Failed to initialize state stream: {:?}", e))
        .ok();

    // Wait for Ctrl+C
    ctrlc::set_handler(move || {
        running.store(false, Ordering::Release);
    })
    .expect("Error setting Ctrl-C handler");

    // Join threads
    heartbeat_handle.await.unwrap();
    receiver_handle.await.unwrap();

    session
        .close()
        .await
        .expect("Failed to close control's zenoh session:");

    Ok(())
}

async fn gimbal_motion(query: Query, shared_camera: &Arc<A8Mini>) {
    let payload = query.payload().unwrap().to_bytes().into_owned();
    let gimbal_packet = gimbal_generated::gimbal::root_as_state(payload.as_slice()).unwrap();

    let cam = shared_camera.clone();

    let pitch = gimbal_packet.pitch();
    let yaw = gimbal_packet.yaw();

    match cam
        .send_command(A8MiniComplexCommand::SetYawPitchAngle(
            pitch.clamp(-90.0, 25.0) as i16,
            yaw.clamp(-135.0, 135.0) as i16,
        ))
        .await
    {
        Ok(readback) => {
            let yaw = i16::from_ne_bytes(readback[0..2].try_into().unwrap());
            let pitch = i16::from_ne_bytes(readback[2..4].try_into().unwrap());
            let roll = i16::from_ne_bytes(readback[4..6].try_into().unwrap());
            let mut builder = FlatBufferBuilder::new();
            let gimbal_state = gimbal_generated::gimbal::State::create(
                &mut builder,
                &gimbal_generated::gimbal::StateArgs {
                    yaw: yaw as f32 / 10.0,
                    pitch: pitch as f32 / 10.0,
                    roll: roll as f32 / 10.0,
                    zoom: -999.9,
                },
            );
            builder.finish(gimbal_state, None);
            let payload = builder.finished_data();
            query.reply("control/out/gimbal", payload).await.ok();
        }
        Err(_) => {}
    };
}

#[inline]
async fn heartbeat_loop(vehicle: &SharedMavConn, running: &Arc<AtomicBool>) {
    let heartbeat_msg = MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_GCS,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });

    while running.load(Ordering::Acquire) {
        if let Err(e) = vehicle.send_default(&heartbeat_msg).await {
            eprintln!("Heartbeat send failed: {:?}", e);
        }
        sleep(Duration::from_secs(1));
    }
}

async fn receiver_loop<'a>(
    vehicle: &SharedMavConn,
    state_pub: &zenoh::pubsub::Publisher<'a>,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Acquire) {
        match vehicle.recv().await {
            Ok((header, msg)) => {
                match msg {
                    MavMessage::HIL_STATE_QUATERNION(data) => {
                        publish_state(state_pub, &data).await.ok();
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
async fn publish_state<'a>(
    publisher: &zenoh::pubsub::Publisher<'a>,
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
    publisher.put(payload).await?;
    Ok(())
}

async fn initialize_state_stream(
    vehicle: &SharedMavConn,
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
        match vehicle.send_default(&cmd_msg).await {
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
