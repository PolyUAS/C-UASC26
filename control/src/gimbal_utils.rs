use flatbuffers::FlatBufferBuilder;

use mavlink::MavHeader;
use mavlink::ardupilotmega::{COMMAND_INT_DATA, MavCmd, MavMessage};

use tokio::sync::broadcast::Receiver;

use std::time::Duration;

use zenoh::{self, Session};

use crate::{SharedAsyncMavConn, fc_header_utils, gimbal_generated};

const COMMAND_RATE: Duration = Duration::from_millis(500);

pub fn spawn_gimbal_task(
    conn: SharedAsyncMavConn,
    mut gimbal_rx: tokio::sync::watch::Receiver<(f32, f32, f32)>,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
    session: &Session,
) -> tokio::task::JoinHandle<()> {
    let session = session.clone();
    let mut bc_rx = bc_rx.resubscribe();

    tokio::spawn(async move {
        let gimbal_pub = session
            .declare_publisher("control/telem/gimbal")
            .await
            .expect("control critical err: failed to initialize gimbal telem publisher");

        let mut fbb = FlatBufferBuilder::new();
        let (sys, comp) = fc_header_utils::wait_for_headers().await;

        loop {
            tokio::select! {
                // --- Inbound: gimbal attitude telemetry from FC ---
                result = bc_rx.recv() => {
                    match result {
                        Ok((_, MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(data))) => {
                            // Quaternion → Euler. ArduPilot sends q as [w, x, y, z]
                            let [w, x, y, z] = data.q;

                            let roll  = (2.0 * (w*x + y*z))
                                .atan2(1.0 - 2.0 * (x*x + y*y));
                            let pitch = (2.0 * (w*y - z*x)).asin();
                            let yaw   = (2.0 * (w*z + x*y))
                                .atan2(1.0 - 2.0 * (y*y + z*z));

                            fbb.reset();
                            let pack = gimbal_generated::gimbal::State::create(
                                &mut fbb,
                                &gimbal_generated::gimbal::StateArgs {
                                    yaw:   yaw.to_degrees(),
                                    pitch: pitch.to_degrees(),
                                    roll:  roll.to_degrees(),
                                    zoom:  -1.0,
                                },
                            );
                            fbb.finish(pack, None);
                            gimbal_pub.put(fbb.finished_data().to_vec()).await.ok();
                        }

                        _ => {}
                    }
                }

                // --- Outbound: send pitch/yaw commands when changed ---
                _ = tokio::time::sleep(COMMAND_RATE) => {
                    if gimbal_rx.has_changed().ok() == Some(true) {
                        let (yaw, pitch, _zoom) = *gimbal_rx.borrow_and_update();

                        let pitch = pitch.clamp(-135.0, 45.0).to_radians();
                        let yaw   = yaw.clamp(-160.0, 160.0).to_radians();

                        conn.send_default(&MavMessage::COMMAND_INT(COMMAND_INT_DATA {
                            param1: pitch,
                            param2: yaw,
                            param3: f32::NAN,
                            param4: f32::NAN,
                            x: 0,
                            y: 0,
                            z: f32::NAN,
                            command: MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                            target_system: sys,
                            target_component: 154,
                            frame: mavlink::ardupilotmega::MavFrame::MAV_FRAME_GLOBAL,
                            current: 0,
                            autocontinue: 0,
                        })).await.ok();
                    }
                }
            }
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
            let gimbal_packet = gimbal_generated::gimbal::root_as_state(bytes.as_ref()).ok();
            if let Some(gimbal_packet) = gimbal_packet {
                // shadowing
                let pitch = gimbal_packet.pitch();
                let yaw = gimbal_packet.yaw();
                let zoom = gimbal_packet.zoom();
                gimbal_tx.send((yaw, pitch, zoom)).ok();
            }
        })
        .await
        .ok()
}
