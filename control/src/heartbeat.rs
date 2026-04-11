use std::sync::{Arc, atomic::AtomicBool};

use tokio::task::JoinHandle;

use crate::SharedAsyncMavConn;

pub fn create_heartbeat_advertiser(
    running: Arc<AtomicBool>,
    vehicle: SharedAsyncMavConn,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        while running.load(std::sync::atomic::Ordering::Acquire) {
            if let Err(e) = vehicle
                .send_default(&mavlink::ardupilotmega::MavMessage::HEARTBEAT(
                    mavlink::ardupilotmega::HEARTBEAT_DATA {
                        custom_mode: 0x0,
                        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
                        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
                        base_mode:
                            mavlink::ardupilotmega::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_ACTIVE,
                        mavlink_version: 3,
                    },
                ))
                .await
            {
                eprintln!("Heartbeat Failed! : {e}");
            }
            tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        }
    })
}
