use a8mini_camera_rs::A8Mini;
use a8mini_camera_rs::control::A8MiniComplexCommand;
use a8mini_camera_rs::control::A8MiniSimpleCommand;
use flatbuffers::{self, FlatBufferBuilder};
use mavlink::AsyncMavConnection;
use mavlink::ardupilotmega::MavMessage;

use mavlink::error::MessageReadError;
use mavlink::{self, MavlinkVersion};

use zenoh::sample::Sample;

use std::error::Error;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use std::time::Duration;

use tokio::time::sleep;

use zenoh::{self};

use crate::attitude_generated::attitude::AttitudePacket;
use crate::attitude_generated::attitude::AttitudePacketArgs;
use crate::position_generated::position::GlobalPositionInt;
use crate::position_generated::position::GlobalPositionIntArgs;

mod attitude_generated;
mod gimbal_generated;
mod position_generated;

type AsyncMavConn = Box<dyn AsyncMavConnection<MavMessage> + Send + Sync>;
type SharedAsyncMavConn = Arc<AsyncMavConn>;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());

    let a8mini_camera = tokio::time::timeout(
        Duration::from_millis(500),
        a8mini_camera_rs::A8Mini::connect(),
    )
    .await
    .map_err(|_| "Connection timed out after 500 ms")?
    .map_err(|e| format!("Failed to connect to A8 Mini: {e}"))?;

    // Setup Zenoh
    let session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).await?;

    // Setup MAVLink
    let mut mavconn = mavlink::connect_async::<MavMessage>(&mavlink_conn_str)
        .await
        .expect("Failed to connect to MAVLink");
    mavconn.set_protocol_version(MavlinkVersion::V2);
    mavconn.set_allow_recv_any_version(true);
    let vehicle = Arc::new(mavconn);

    println!("Connected to Mavlink on {:?}", mavlink_conn_str);

    // Setup publisher
    let position_pub = session
        .declare_publisher("control/telem/gl_pos_int")
        .await?;
    let attitude_pub = session.declare_publisher("control/telem/attitude").await?;

    let shared_camera = Arc::new(a8mini_camera);

    match shared_camera
        .send_command(A8MiniSimpleCommand::AutoCenter)
        .await
    {
        Ok(..) => {}
        Err(e) => {
            println!("Failed to autocenter gimbal: {e}")
        }
    };

    // Setup gimbal subscriber
    let gimbal_sub = session
        .declare_subscriber("control/in/gimbal")
        .callback(move |sample| {
            let cam = shared_camera.clone();
            tokio::spawn(async move {
                gimbal_motion(sample, &cam).await;
            });
        })
        .await?;

    sleep(Duration::from_millis(500)).await; // Wait for subscribers

    // Shutdown signal ptr
    let running = Arc::new(AtomicBool::new(true));

    // Receiver task
    let vehicle_recv = vehicle.clone();
    let running_recv = running.clone();

    let receiver_handle = tokio::spawn(async move {
        receiver_loop(&vehicle_recv, &position_pub, &attitude_pub, &running_recv).await;
    });

    // Wait for Ctrl+C
    ctrlc::set_handler(move || {
        running.store(false, Ordering::Release);
    })
    .expect("Error setting Ctrl-C handler");

    // Join threads

    let tup = tokio::join!(receiver_handle);
    tup.0.ok();

    session
        .close()
        .await
        .expect("Failed to close control's zenoh session:");

    Ok(())
}

async fn gimbal_motion(sample: Sample, shared_camera: &Arc<A8Mini>) {
    let bytes = sample.payload().to_bytes(); // &[u8]
    let gimbal_packet = gimbal_generated::gimbal::root_as_state(bytes.as_ref()).unwrap();

    let cam = shared_camera.clone();

    let pitch = gimbal_packet.pitch();
    let yaw = gimbal_packet.yaw();

    // println!("Recieved Query! pitch: {pitch}, yaw: {yaw}");

    match cam
        .send_command(A8MiniComplexCommand::SetYawPitchAngle(
            (yaw.clamp(-135.0, 135.0) * 10.0) as i16,
            (pitch.clamp(-90.0, 25.0) * 10.0) as i16,
        ))
        .await
    {
        Ok(readback) => {
            // let yaw = i16::from_le_bytes(readback[0..2].try_into().unwrap());
            // let pitch = i16::from_le_bytes(readback[2..4].try_into().unwrap());
            // let roll = i16::from_le_bytes(readback[4..6].try_into().unwrap());
            // let mut builder = FlatBufferBuilder::new();
            // let gimbal_state = gimbal_generated::gimbal::State::create(
            //     &mut builder,
            //     &gimbal_generated::gimbal::StateArgs {
            //         yaw: yaw as f32 / 10.0,
            //         pitch: pitch as f32 / 10.0,
            //         roll: roll as f32 / 10.0,
            //         zoom: -999.9,
            //     },
            // );
            // builder.finish(gimbal_state, None);
            // let payload = builder.finished_data();
        }
        Err(_) => {}
    };
}

async fn receiver_loop<'a>(
    vehicle: &SharedAsyncMavConn,
    position_pub: &zenoh::pubsub::Publisher<'a>,
    attitude_pub: &zenoh::pubsub::Publisher<'a>,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Acquire) {
        match vehicle.recv().await {
            Ok((header, msg)) => {
                match msg {
                    MavMessage::GLOBAL_POSITION_INT(data) => {
                        let mut builder = FlatBufferBuilder::new();
                        let root = GlobalPositionInt::create(
                            &mut builder,
                            &GlobalPositionIntArgs {
                                lat: data.lat,
                                lon: data.lon,
                                alt: data.alt,
                                alt_above_home: data.relative_alt,
                                vx: data.vx,
                                vy: data.vy,
                                vz: data.vz,
                                heading: data.hdg,
                            },
                        );
                        builder.finish(root, None);
                        let payload = builder.finished_data();
                        position_pub.put(payload).await.ok();
                    }
                    MavMessage::ATTITUDE(data) => {
                        let mut builder = FlatBufferBuilder::new();
                        let root = AttitudePacket::create(
                            &mut builder,
                            &AttitudePacketArgs {
                                roll: data.roll,
                                pitch: data.pitch,
                                yaw: data.yaw,
                                vroll: data.rollspeed,
                                vpitch: data.pitchspeed,
                                vyaw: data.yawspeed,
                            },
                        );
                        builder.finish(root, None);
                        let payload = builder.finished_data();

                        attitude_pub.put(payload).await;
                    }
                    MavMessage::BATTERY_STATUS(data) => {}
                    MavMessage::POWER_STATUS(data) => {}
                    MavMessage::SYS_STATUS(data) => {}
                    MavMessage::RC_CHANNELS(data) => {}
                    MavMessage::MISSION_CURRENT(data) => {}
                    MavMessage::SERVO_OUTPUT_RAW(data) => {}
                    MavMessage::HEARTBEAT(data) => {}
                    MavMessage::NAV_CONTROLLER_OUTPUT(data) => {}
                    MavMessage::GPS_RAW_INT(data) => {}
                    MavMessage::TERRAIN_REPORT(data) => {}
                    MavMessage::AHRS(data) => {}
                    MavMessage::AHRS2(data) => {}
                    MavMessage::EKF_STATUS_REPORT(data) => {}
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
                        // println!(
                        //     "Message ==== Name: {}, Id: {}, ",
                        //     msg.message_name(),
                        //     msg.message_id()
                        // );
                    }
                }
            }

            Err(MessageReadError::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                sleep(Duration::from_micros(50)).await;
            }
            Err(e) => {
                eprintln!("Control recv error: {:?}", e);
            }
        }
    }
}

// #[inline]
// async fn publish_state<'a>(
//     publisher: &zenoh::pubsub::Publisher<'a>,
//     data: &mavlink::ardupilotmega::HIL_STATE_QUATERNION_DATA,
// ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
//     // Build FlatBuffer message
//     let mut builder = FlatBufferBuilder::new();
//     let state = StateVector::create(
//         &mut builder,
//         &StateVectorArgs {
//             time_usec: data.time_usec,
//             quaternion: Some(&Quaternion::new(
//                 data.attitude_quaternion[0],
//                 data.attitude_quaternion[1],
//                 data.attitude_quaternion[2],
//                 data.attitude_quaternion[3],
//             )),
//             rollspeed: data.rollspeed,
//             pitchspeed: data.pitchspeed,
//             yawspeed: data.yawspeed,
//             pos: Some(&Int32Vec3::new(data.lat, data.lon, data.alt)),
//             vel: Some(&Int16Vec3::new(data.vx, data.vy, data.vz)),
//             acc: Some(&Int16Vec3::new(data.xacc, data.yacc, data.zacc)),
//             ind_airspeed: data.ind_airspeed,
//             true_airspeed: data.true_airspeed,
//         },
//     );
//     builder.finish(state, None);

//     // Get the serialized bytes
//     let payload = builder.finished_data().to_vec();

//     println!("Published state!");
//     dbg!(&data);
//     // Publish
//     publisher.put(payload).await?;
//     Ok(())
// }
