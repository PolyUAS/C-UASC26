use a8mini_camera_rs::control::{A8MiniComplexCommand, A8MiniSimpleCommand};
use a8mini_camera_rs::{A8Mini, control::Command};
use flatbuffers::FlatBufferBuilder;

use std::sync::Arc;
use std::time::Duration;

use zenoh::{self, Session};

use crate::gimbal_generated;

const COMMAND_RATE: Duration = Duration::from_millis(500);

pub fn spawn_gimbal_task(
    cam: Arc<A8Mini>,
    gimbal_rx: tokio::sync::watch::Receiver<(f32, f32, f32)>,
    session: &Session,
) -> tokio::task::JoinHandle<()> {
    let session = session.clone();

    tokio::spawn(async move {
        let gimbal_pub = session
            .declare_publisher("control/telem/gimbal")
            .await
            .expect("control critical err: failed to initialize gimbal telem publisher");

        cam.send_command(A8MiniSimpleCommand::SetVideoOutputOff)
            .await // disable the other video feeds bc we dont use them
            .ok();

        let mut fbb = FlatBufferBuilder::new();

        loop {
            if gimbal_rx.has_changed().ok() == Some(true) {
                let (yaw, pitch, zoom) = *gimbal_rx.borrow();

                cam.send_command(A8MiniComplexCommand::SetYawPitchAngle(
                    (yaw.clamp(-135.0, 135.0) * 10.0) as i16,
                    (pitch.clamp(-90.0, 25.0) * 10.0) as i16,
                ))
                .await
                .ok();

                cam.send_command(ZoomCmd::new(zoom)).await.ok();
            }

            if let Some(info) = cam.get_attitude_information().await.ok() {
                fbb.reset();
                let pack = gimbal_generated::gimbal::State::create(
                    &mut fbb,
                    &gimbal_generated::gimbal::StateArgs {
                        yaw: info.theta_yaw as f32,
                        pitch: info.theta_pitch as f32,
                        roll: info.theta_roll as f32,
                        zoom: -1.0, // impossible value, we dont do zoom telem
                    },
                );
                fbb.finish(pack, None);
                let payload = fbb.finished_data().to_vec();

                gimbal_pub.put(payload).await.ok();
            }

            tokio::time::sleep(COMMAND_RATE).await;
        }
    })
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
struct ZoomCmd {
    zoom: f32,
}

impl ZoomCmd {
    fn new(zoom: f32) -> Self {
        ZoomCmd {
            zoom: zoom.clamp(1.0, 6.0),
        }
    }
}

impl Command for ZoomCmd {
    fn to_bytes(&self) -> Vec<u8> {
        let int_zoom = self.zoom.floor() as u8;
        let frac_zoom = ((self.zoom - int_zoom as f32) * 10.0).floor() as u8;

        let mut byte_arr: Vec<u8> = vec![0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, 0x0f];

        byte_arr.push(int_zoom.clamp(0x1, 0x1e));
        byte_arr.push(frac_zoom.clamp(0x00, 0x9));

        byte_arr.extend_from_slice(&a8mini_camera_rs::checksum::crc16_calc(&byte_arr, 0));

        byte_arr
    }
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
