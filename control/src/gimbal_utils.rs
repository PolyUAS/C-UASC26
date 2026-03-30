use a8mini_camera_rs::A8Mini;
use a8mini_camera_rs::control::A8MiniComplexCommand;

use std::sync::Arc;

use zenoh::{self};

use crate::gimbal_generated;

pub fn spawn_gimbal_task(
    cam: Arc<A8Mini>,
    mut gimbal_rx: tokio::sync::watch::Receiver<(f32, f32, f32)>,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        loop {
            gimbal_rx.changed().await.ok();
            let (yaw, pitch, zoom) = *gimbal_rx.borrow();
            cam.send_command(A8MiniComplexCommand::SetYawPitchAngle(
                (yaw.clamp(-135.0, 135.0) * 10.0) as i16,
                (pitch.clamp(-90.0, 25.0) * 10.0) as i16,
            ))
            .await
            .ok();
        }
    })
}

pub async fn create_gimbal_subscriber(
    session: &zenoh::Session,
    gimbal_tx: tokio::sync::watch::Sender<(f32, f32, f32)>,
) -> Option<zenoh::pubsub::Subscriber<()>> {
    session
        .declare_subscriber("control/in/gimbal")
        .callback(move |sample| {
            let bytes = sample.payload().to_bytes(); // &[u8]
            let gimbal_packet = gimbal_generated::gimbal::root_as_state(bytes.as_ref()).unwrap();

            let pitch = gimbal_packet.pitch();
            let yaw = gimbal_packet.yaw();
            let zoom = gimbal_packet.zoom();
            gimbal_tx.send((yaw, pitch, zoom)).ok();
        })
        .await
        .ok()
}
