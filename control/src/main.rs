use flatbuffers::{self, FlatBufferBuilder};
use mavlink::AsyncMavConnection;

use mavlink::MavHeader;
use mavlink::MavlinkVersion;
use mavlink::ardupilotmega::MavMessage;

use tokio::sync::broadcast::Receiver;

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

mod generated;
mod gimbal;

use gimbal::create_gimbal_subscriber;
use gimbal::spawn_gimbal_task;

use generated::attitude_generated;
use generated::geofence_generated;
use generated::gimbal_generated;
use generated::position_generated;

mod geofence;

use geofence::create_geofence_queryable;

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

    let (bc_tx, _) = tokio::sync::broadcast::channel::<(mavlink::MavHeader, MavMessage)>(128);
    // Spawn one dedicated recv loop that only calls vehicle.recv() and bc_tx.send()
    // Each subsystem takes bc_tx.subscribe()
    //

    println!("Connected to Mavlink on {:?}", mavlink_conn_str);

    // Setup publisher
    let position_pub = session
        .declare_publisher("control/telem/gl_pos_int")
        .await?;

    let attitude_pub = session.declare_publisher("control/telem/attitude").await?;

    let shared_camera = Arc::new(a8mini_camera);

    let (gimbal_tx, gimbal_rx) = tokio::sync::watch::channel((0.0f32, 0.0f32));
    // Setup gimbal subscriber
    let _gimbal_sub = create_gimbal_subscriber(&session, gimbal_tx).await;

    // In a dedicated task:
    spawn_gimbal_task(shared_camera.clone(), gimbal_rx);

    sleep(Duration::from_millis(500)).await; // Wait for subscribers

    // Shutdown signal ptr
    let running = Arc::new(AtomicBool::new(true));

    let active = running.clone();
    let rx_bc_rx = bc_tx.clone();
    let reciever_clone = vehicle.clone();
    // Receiver task
    tokio::spawn(async move {
        while active.load(Ordering::Acquire) {
            if let Some(msg_tuple) = reciever_clone.recv().await.ok() {
                rx_bc_rx.send(msg_tuple).ok();
            }
        }
    });

    let running_telem = running.clone();

    let mut telem_rx = bc_tx.subscribe();
    let telem_handle = tokio::spawn(async move {
        telemetry_loop(&mut telem_rx, &position_pub, &attitude_pub, &running_telem).await;
    });

    let geofence_vehicle = vehicle.clone();
    let _geofence_query = create_geofence_queryable(&session, geofence_vehicle, bc_tx.clone());

    // Wait for Ctrl+C
    ctrlc::set_handler(move || {
        running.store(false, Ordering::Release);
    })
    .expect("Error setting Ctrl-C handler");

    // Join threads

    let tup = tokio::join!(telem_handle);
    tup.0.ok();

    session
        .close()
        .await
        .expect("Failed to close control's zenoh session:");

    Ok(())
}

async fn telemetry_loop<'a>(
    vehicle: &mut Receiver<(MavHeader, MavMessage)>,
    position_pub: &zenoh::pubsub::Publisher<'a>,
    attitude_pub: &zenoh::pubsub::Publisher<'a>,
    running: &Arc<AtomicBool>,
) {
    while running.load(Ordering::Acquire) {
        match vehicle.recv().await {
            Ok((_header, msg)) => {
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

                        attitude_pub.put(payload).await.ok();
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

            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                eprintln!("telemetry subscriber lagged, dropped {n} messages");
            }
            Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                break; // sender dropped, shut down
            }
        }
    }
}
