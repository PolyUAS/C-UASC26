use std::time::Duration;

use flatbuffers::Follow;
use tokio::sync::broadcast;
use zenoh::Config;

mod generated;

const MAX_VALID_DX: Duration = Duration::from_millis(50);

#[tokio::main]
async fn main() {
    println!("Hello, World!");
    let session = zenoh::open(Config::from_env().unwrap_or(Config::default()))
        .await
        .expect("Failed to open zenoh session from navigation");

    let detections_sub = session
        .declare_subscriber("vision/detections")
        .callback(move |sample| {
            let cow_bytes = sample.payload().to_bytes();
            let packet = generated::detections_generated::root_as_frame(&cow_bytes);
            if let Some(frame) = packet.ok() {}
        });

    let position_sub = session.declare_subscriber("control/telem/");
}
