use crate::gimbal_utils::spawn_gimbal_task;
use mavlink::AsyncMavConnection;
use mavlink::MavHeader;
use mavlink::MavlinkVersion;
use mavlink::ardupilotmega::MavMessage;

use std::error::Error;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use tokio::sync::broadcast::Sender;
use tokio::time::sleep;

use zenoh::{self};

mod fc_header_utils;
mod generated;
mod geofence;
mod gimbal_utils;
mod missions;
mod receiver_utils;
mod telemetry;

use generated::*;

use geofence::create_geofence_queryable;

use gimbal_utils::create_gimbal_subscriber;

use missions::get_missions;

use telemetry::telemetry_loop;

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

    let shared_camera = Arc::new(a8mini_camera);

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
    // shutdown signal
    let running = Arc::new(AtomicBool::new(true));

    let (bc_tx, _) = tokio::sync::broadcast::channel::<(mavlink::MavHeader, MavMessage)>(128);
    // Spawn one dedicated recv loop that only calls vehicle.recv() and bc_tx.send()
    // Each subsystem takes bc_tx.subscribe()

    println!("Connected to Mavlink on {:?}", mavlink_conn_str);

    // Setup publisher
    let position_pub = session
        .declare_publisher("control/telem/gl_pos_int")
        .await?;

    let attitude_pub = session.declare_publisher("control/telem/attitude").await?;
    let gimbal_pub = session.declare_publisher("control/telem/gimbal").await?;

    let battery_publisher = session.declare_publisher("control/health/battery").await?;
    let heartbeat_publisher = session
        .declare_publisher("control/health/heartbeat")
        .await?;

    let (gimbal_tx, gimbal_rx) = tokio::sync::watch::channel((0.0f32, 0.0f32, 1.0f32)); // zoom 1.0 by default
    // Setup gimbal subscriber
    let _gimbal_sub = create_gimbal_subscriber(&session, gimbal_tx).await;

    // In a dedicated task:
    spawn_gimbal_task(shared_camera.clone(), gimbal_rx.clone());

    sleep(Duration::from_millis(500)).await; // Wait for subscribers

    let running_telem = running.clone(); // assuming running: Arc<AtomicBool> or similar

    let telem_handle = tokio::spawn({
        let mut telem_rx = bc_tx.subscribe(); // cheap, can move in directly
        async move {
            telemetry_loop(
                &mut telem_rx,
                &position_pub,
                &attitude_pub,
                &battery_publisher,
                &heartbeat_publisher,
                &running_telem,
            )
            .await;
        }
    });

    create_mavlink_broadcast_thread(&running, bc_tx.clone(), &vehicle);

    let geofence_vehicle = vehicle.clone();

    let _geofence_query =
        create_geofence_queryable(&session, geofence_vehicle, bc_tx.clone()).await;

    get_missions(&vehicle.clone(), &mut bc_tx.subscribe()).await;

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

fn create_mavlink_broadcast_thread(
    running: &Arc<AtomicBool>,
    bc_tx: Sender<(MavHeader, MavMessage)>,
    vehicle: &SharedAsyncMavConn,
) {
    let active = running.clone();
    let broadcaster = bc_tx.clone();
    let vehicle_local = vehicle.clone();
    // Receiver task
    tokio::spawn(async move {
        while active.load(Ordering::Acquire) {
            if let Some(msg_tuple) = vehicle_local.recv().await.ok() {
                let header = msg_tuple.0;

                broadcaster.send(msg_tuple).ok();

                // ik what im doing here
                fc_header_utils::set_headers(header.system_id, header.component_id);
            }
        }
    });
}
