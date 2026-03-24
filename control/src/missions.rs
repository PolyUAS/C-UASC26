use crate::SharedAsyncMavConn;
use crate::fc_header_utils::get_headers;
use crate::fc_header_utils::wait_for_headers;
use crate::receiver_utils::recv_message_within;
use flatbuffers::{self, FlatBufferBuilder};

use mavlink::ardupilotmega::MISSION_ACK_DATA;
use mavlink::ardupilotmega::MISSION_COUNT_DATA;
use mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA;
use mavlink::ardupilotmega::PARAM_REQUEST_READ_DATA;

use mavlink::MavHeader;
use mavlink::ardupilotmega::FENCE_FETCH_POINT_DATA;
use mavlink::ardupilotmega::FENCE_POINT_DATA;
use mavlink::ardupilotmega::MavMessage;

use mavlink::common::MISSION_ITEM_INT_DATA;
use mavlink::types::CharArray;
use mavlink::{self};

use tokio::sync::broadcast::Receiver;
use tokio::sync::broadcast::Sender;
use zenoh::Session;

use zenoh::query::Queryable;

use std::error::Error;

use std::time::Duration;

use zenoh::{self};

pub async fn get_missions(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
) {
    let (sys, comp) = wait_for_headers().await;

    vehicle
        .send_default(&MavMessage::MISSION_REQUEST_LIST(
            mavlink::ardupilotmega::MISSION_REQUEST_LIST_DATA {
                target_system: sys,
                target_component: comp,
            },
        ))
        .await
        .ok();

    let count_packet = {
        // closure to remove the duplicate subscriber
        let mut sub = bc_rx.resubscribe();
        let timeout = Duration::from_secs(2);

        recv_message_within(timeout, &mut sub, |m| {
            matches!(m, MavMessage::MISSION_COUNT(_))
        })
        .await
    }
    .ok();

    let mission_count = match count_packet {
        Some(MavMessage::MISSION_COUNT((data))) => Some(data.count),
        _ => None,
    };

    let mut packets = Vec::new();

    for i in 0..mission_count.expect("Failed to collect missions") {
        let _ = vehicle
            .send_default(&MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
                seq: i,
                target_component: comp,
                target_system: sys,
            }))
            .await;

        let pack = {
            let mut sub = bc_rx.resubscribe();
            recv_message_within(Duration::from_secs(2), &mut sub, |m| {
                matches!(m, MavMessage::MISSION_ITEM_INT(_))
            })
            .await
            .ok()
        };

        packets.push(pack);
    }

    for elem in packets {
        match elem {
            Some(pack) => match pack {
                MavMessage::MISSION_ITEM_INT(data) => {
                    let cmd = data.command;
                    match cmd {
                        mavlink::ardupilotmega::MavCmd::MAV_CMD_NAV_WAYPOINT => {}
                        _ => {}
                    }
                }
                _ => {}
            },
            _ => {}
        }
    }

    _ = vehicle
        .send_default(&MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: sys,
            target_component: comp,
            mavtype: mavlink::ardupilotmega::MavMissionResult::MAV_MISSION_ACCEPTED,
        }))
        .await;
}
