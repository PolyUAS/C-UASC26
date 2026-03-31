use flatbuffers::{self, FlatBufferBuilder};

use mavlink::MavHeader;

use mavlink::ardupilotmega::MavMessage;

use tokio::sync::broadcast::Receiver;
use zenoh::pubsub::Publisher;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use zenoh::{self};

use crate::attitude_generated::attitude::AttitudePacket;
use crate::attitude_generated::attitude::AttitudePacketArgs;

use crate::battery_generated::battery::BatteryStatus;
use crate::battery_generated::battery::BatteryStatusArgs;

use crate::generated::heartbeat_generated;
use crate::generated::heartbeat_generated::heartbeat::HeartbeatHealthPacket;
use crate::position_generated::position::GlobalPositionInt;
use crate::position_generated::position::GlobalPositionIntArgs;

pub async fn telemetry_loop<'a>(
    vehicle: &mut Receiver<(MavHeader, MavMessage)>,
    position_pub: &Publisher<'a>,
    attitude_pub: &Publisher<'a>,
    battery_pub: &Publisher<'a>,
    heartbeat_pub: &Publisher<'a>,
    running: &Arc<AtomicBool>,
) {
    let mut builder = FlatBufferBuilder::new();
    while running.load(Ordering::Acquire) {
        match vehicle.recv().await {
            Ok((header, msg)) => {
                match msg {
                    MavMessage::AHRS(data) => {}
                    MavMessage::AHRS2(data) => {}
                    MavMessage::ATTITUDE(data) => {
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
                        let payload = builder.finished_data().to_vec();

                        attitude_pub.put(payload).await.ok();
                    }
                    MavMessage::BATTERY_STATUS(data) => {
                        let root = BatteryStatus::create(
                            &mut builder,
                            &BatteryStatusArgs {
                                consumed_current: data.current_consumed,
                                consumed_energy: data.energy_consumed,
                                temperature: data.temperature,
                                battery_current: data.current_battery,
                                id: data.id,
                                remaining: data.battery_remaining,
                            },
                        );
                        builder.finish(root, None);
                        let payload = builder.finished_data().to_vec();
                        let _ = battery_pub.put(payload).await;
                    }
                    MavMessage::EKF_STATUS_REPORT(data) => {}
                    MavMessage::GLOBAL_POSITION_INT(data) => {
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
                        position_pub.put(payload).await.ok();
                    }
                    MavMessage::GPS2_RAW(data) => {}
                    MavMessage::GPS_RAW_INT(data) => {}
                    MavMessage::HEARTBEAT(data) => {
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
                            &heartbeat_generated::heartbeat::HeartbeatHealthPacketArgs {
                                system_status: status_enum,
                            },
                        );
                        match data.autopilot {
                            mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_GENERIC => {}
                            _ => {}
                        }
                        builder.finish(pack, None);
                        let payload = builder.finished_data().to_vec();
                        heartbeat_pub.put(payload).await.ok();
                    }
                    MavMessage::MCU_STATUS(data) => { /* Id: 11039 — ArduPilot-specific */ }
                    MavMessage::MEMINFO(data) => { /* Id: 152 */ }
                    MavMessage::MISSION_CURRENT(data) => {
                        data.seq;
                    }
                    MavMessage::NAV_CONTROLLER_OUTPUT(data) => {}
                    MavMessage::POWER_STATUS(data) => {}
                    MavMessage::RAW_IMU(data) => { /* Id: 27 */ }
                    MavMessage::RC_CHANNELS(data) => {}
                    MavMessage::SCALED_IMU2(data) => { /* Id: 116 */ }
                    MavMessage::SCALED_PRESSURE(data) => { /* Id: 29 */ }
                    MavMessage::SERVO_OUTPUT_RAW(data) => {}
                    MavMessage::SYSTEM_TIME(data) => { /* Id: 2 */ }
                    MavMessage::SYS_STATUS(data) => {}
                    MavMessage::TERRAIN_REPORT(data) => {}
                    MavMessage::VFR_HUD(data) => { /* Id: 74 */ }
                    MavMessage::VIBRATION(data) => { /* Id: 241 */ }
                    _ => {
                        // Discover other messages
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
        builder.reset();
    }
}
