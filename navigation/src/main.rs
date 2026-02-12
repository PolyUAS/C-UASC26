use std::{thread::sleep, time::Duration};
use zenoh;
use zenoh::Wait;

fn main() -> zenoh::Result<()> {
    let session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).wait()?;

    let detections_sub = session.declare_subscriber("vision/detections");

    session.close().wait()?;
    Ok(())
}
