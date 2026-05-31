use flatbuffers::{self, FlatBufferBuilder};
use mavlink::MavHeader;
use mavlink::Message;
use mavlink::ardupilotmega::{MavMessage, MavModeFlag};

use tokio::sync::broadcast::Receiver;
use zenoh::pubsub::Publisher;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

pub static IS_GUIDED_ENABLED: AtomicBool = AtomicBool::new(false);

pub async fn telemetry_loop<'a>(
    vehicle: &mut Receiver<(MavHeader, MavMessage)>,
    position_pub: &Publisher<'a>,
    attitude_pub: &Publisher<'a>,
    battery_pub: &Publisher<'a>,
    heartbeat_pub: &Publisher<'a>,
    running: &Arc<AtomicBool>,
) {
    // Start with a healthy capacity to minimize internal re-allocations
    let mut builder = FlatBufferBuilder::with_capacity(1024);

    while running.load(Ordering::Acquire) {
        match vehicle.recv().await {
            Ok((_header, msg)) => {
                match msg {
                    MavMessage::ATTITUDE(data) => {
                        use crate::attitude_generated::attitude::{
                            AttitudePacket, AttitudePacketArgs,
                        };

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

                        // Copy the bytes into a lightweight stack allocation or vector before resetting
                        let payload = builder.finished_data().to_vec();
                        builder.reset(); // Reset early to free the builder reference

                        let _ = attitude_pub.put(payload).await;
                    }

                    MavMessage::GLOBAL_POSITION_INT(data) => {
                        use crate::position_generated::position::{
                            GlobalPositionInt, GlobalPositionIntArgs,
                        };

                        static LAST_POS_PUBLISH: std::sync::atomic::AtomicU64 =
                            std::sync::atomic::AtomicU64::new(0);

                        let now = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_millis() as u64;

                        let last = LAST_POS_PUBLISH.load(Ordering::Relaxed);
                        if now - last < 200 {
                            // 5Hz max
                            continue; // skip this message
                        }
                        LAST_POS_PUBLISH.store(now, Ordering::Relaxed);
                        // println!("Publishing position: lat={} lon={}", data.lat, data.lon);

                        builder.reset();

                        let root = GlobalPositionInt::create(
                            &mut builder,
                            &GlobalPositionIntArgs {
                                lat: data.lat,
                                lon: data.lon,
                                alt: data.alt,
                                alt_above_home: data.relative_alt,
                                vx: data.vx,
                                vy: data.vy,
                                vz: -data.vz,
                                heading: data.hdg,
                            },
                        );
                        builder.finish(root, None);

                        let payload = builder.finished_data().to_vec();
                        builder.reset();

                        let _ = position_pub.put(payload).await;
                    }

                    MavMessage::HEARTBEAT(data) => {
                        use crate::generated::heartbeat_generated::{
                            self,
                            heartbeat::{HeartbeatHealthPacket, HeartbeatHealthPacketArgs},
                        };

                        let mode = &data.base_mode;
                        let auto = mode.contains(MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED);
                        let guided = mode.contains(MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED);
                        let manual = mode.contains(MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
                        let armed = mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);

                        if guided {
                            IS_GUIDED_ENABLED.store(true, Ordering::Release);
                        } else {
                            IS_GUIDED_ENABLED.store(false, Ordering::Release);
                        }

                        let status_enum = match data.system_status {
                            mavlink::ardupilotmega::MavState::MAV_STATE_ACTIVE => {
                                heartbeat_generated::heartbeat::Status::Active
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_BOOT => {
                                heartbeat_generated::heartbeat::Status::Boot
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_CALIBRATING => {
                                heartbeat_generated::heartbeat::Status::Calibrating
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY => {
                                heartbeat_generated::heartbeat::Status::Standby
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_CRITICAL => {
                                heartbeat_generated::heartbeat::Status::Critical
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_EMERGENCY => {
                                heartbeat_generated::heartbeat::Status::Emergency
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_FLIGHT_TERMINATION => {
                                heartbeat_generated::heartbeat::Status::Flight_Terminating
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_POWEROFF => {
                                heartbeat_generated::heartbeat::Status::Power_Off
                            }
                            mavlink::ardupilotmega::MavState::MAV_STATE_UNINIT => {
                                heartbeat_generated::heartbeat::Status::Uninit
                            }
                        };

                        let pack = HeartbeatHealthPacket::create(
                            &mut builder,
                            &HeartbeatHealthPacketArgs {
                                system_status: status_enum,
                                auto,
                                guided,
                                manual,
                                armed,
                            },
                        );
                        builder.finish(pack, None);

                        let payload = builder.finished_data().to_vec();
                        builder.reset();

                        let _ = heartbeat_pub.put(payload).await;
                    }

                    _ => {}
                }
            }

            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                // Changing this to a simple log so a temporary spike doesn't spam your stderr channels
                println!("Telemetry packet parser lagged behind by {} frames.", n);
            }
            Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                break;
            }
        }

        // Final safety net reset
        builder.reset();
    }
}
