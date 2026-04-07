use crate::world_generated::EntityArgs;
use std::sync::Arc;
use std::time::SystemTime;
use std::{error::Error, time::Duration};

use flatbuffers::FlatBufferBuilder;

use tokio::sync::RwLock;
use zenoh::pubsub::Publisher;
use zenoh::{Config, time::Timestamp};

use crate::generated::detections_generated::root_as_frame;
use crate::generated::world_generated;
use crate::generated::{
    attitude_generated::attitude::root_as_attitude_packet, gimbal_generated::gimbal::root_as_state,
    position_generated::position::root_as_global_position_int,
};
use crate::mapping::GeoprojectionCamParams;

mod generated;
mod mapping;

const MAX_VALID_LAG: Duration = Duration::from_millis(600);
const A8_PARAMS: GeoprojectionCamParams = GeoprojectionCamParams::new(81.0, 45.694);

#[derive(Default, Clone, Debug)]
pub struct NavState {
    pub frame: Option<Arc<[u8]>>,
    pub position: Option<Arc<[u8]>>,
    pub attitude: Option<Arc<[u8]>>,
    pub gimbal: Option<Arc<[u8]>>,
    pub latest: Option<Timestamp>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let gimbal_offset: [i32; 3] = std::env::var("GIMBAL_OFFSET_MM")
        .expect("Critical Nav Setup err: GIMBAL_OFFSET_MM environment variable is not set")
        .split(|c: char| c == ',' || c.is_whitespace()) // supports commas or spaces
        .filter(|s| !s.trim().is_empty())
        .map(|s| {
            s.trim().parse::<i32>().expect(
                "Critical Nav Setup err: GIMBAL_OFFSET_MM must contain three valid integers",
            )
        })
        .collect::<Vec<i32>>()
        .try_into()
        .expect("Critical Nav Setup err: GIMBAL_OFFSET_MM must contain exactly three integers");

    let session = zenoh::open(Config::from_env().unwrap_or(Config::default()))
        .await
        .expect("Failed to open zenoh session from navigation");

    let camera_frame_sub = session.declare_subscriber("vision/detections").await?;

    let position_sub = session
        .declare_subscriber("control/telem/gl_pos_int")
        .await?;

    let attitude_sub = session.declare_subscriber("control/telem/attitude").await?;

    let gimbal_sub = session
        .declare_subscriber("control/telem/gimbal") // fixed the double slash
        .await?;

    let entity_publisher = session
        .declare_publisher("navigation/world/entities")
        .await?;

    let nav_collected: Arc<RwLock<NavState>> = Arc::new(RwLock::new(NavState::default()));
    // Main loop: receive from all 4 concurrently
    let mut begin = SystemTime::now();
    loop {
        tokio::select! {
            Ok(sample) = camera_frame_sub.recv_async() => {
                let bytes: Arc<[u8]> = Arc::from(sample.payload().to_bytes().into_owned());

                let mut state = nav_collected.write().await;
                state.frame = Some(bytes);
            }

            Ok(sample) = position_sub.recv_async() => {
                let bytes: Arc<[u8]> = Arc::from(sample.payload().to_bytes().into_owned());

                let mut state = nav_collected.write().await;
                state.position = Some(bytes);
            }

            Ok(sample) = attitude_sub.recv_async() => {
                let bytes: Arc<[u8]> = Arc::from(sample.payload().to_bytes().into_owned());

                let mut state = nav_collected.write().await;
                state.attitude = Some(bytes);
            }

            Ok(sample) = gimbal_sub.recv_async() => {
                let bytes: Arc<[u8]> = Arc::from(sample.payload().to_bytes().into_owned());

                let mut state = nav_collected.write().await;
                state.gimbal = Some(bytes);
            }
        }

        let mut state = nav_collected.write().await;

        let now = SystemTime::now();
        if let Some(delta) = now.duration_since(begin).ok() {
            if delta > MAX_VALID_LAG {
                // Reset all cached data
                state.frame = None;
                state.position = None;
                state.attitude = None;
                state.gimbal = None;

                begin = now;
            } else {
                if let (Some(frame), Some(position), Some(attitude), Some(gimbal)) = (
                    &state.frame,
                    &state.position,
                    &state.attitude,
                    &state.gimbal,
                ) {
                    let _ = broadcast_entities(state.to_owned(), &entity_publisher, gimbal_offset)
                        .await;
                }
            }
        }
    }
}

macro_rules! unwrap_navstate_part {
    ($func:expr, $struct:expr, $data:ident) => {
        $func($data.as_ref())
    };
}

async fn broadcast_entities<'a>(
    state: NavState,
    publisher: &Publisher<'a>,
    gimbal_pos: [i32; 3], // in mm
) -> Result<(), Box<dyn Error + Send + Sync>> {
    let frame_data = state
        .frame
        .expect("Critical Navigation err, failed to unwrap NavState");
    let gimbal_data = state
        .gimbal
        .expect("Critical Navigation err, failed to unwrap NavState");
    let attitude_data = state
        .attitude
        .expect("Critical Navigation err, failed to unwrap NavState");
    let position_data = state
        .position
        .expect("Critical Navigation err, failed to unwrap NavState");

    let frame = unwrap_navstate_part!(root_as_frame, state.frame, frame_data)?;
    let gimbal = unwrap_navstate_part!(root_as_state, state.gimbal, gimbal_data)?;
    let attitude = unwrap_navstate_part!(root_as_attitude_packet, state.attitude, attitude_data)?;
    let position =
        unwrap_navstate_part!(root_as_global_position_int, state.position, position_data)?;

    if let Some(detections) = frame.detections() {
        let mut fbb = FlatBufferBuilder::new();

        // TODO modify entities to be placed in frames
        for det in detections {
            if let Some(pos) = mapping::map_detection(
                &attitude,
                &position,
                &det,
                &gimbal,
                gimbal_pos,
                frame.width(),
                frame.height(),
                &A8_PARAMS,
            ) {
                let metadata = det.metadata().map(|s| fbb.create_string(s));
                let class_id = det.class_id().map(|s| fbb.create_string(s));
                let pack = world_generated::Entity::create(
                    &mut fbb,
                    &EntityArgs {
                        lat: pos[0],
                        lon: pos[1],
                        alt: 0.0,
                        confidence: det.confidence(),
                        class_id: class_id,
                        metadata: metadata,
                    },
                );

                fbb.finish(pack, None);
                let payload = fbb.finished_data();

                let _ = publisher.put(payload).await; // TODO make this try to send over and over instead of wasting msg + compute
            }
        }
    }

    Ok(())
}
