use crate::SharedAsyncMavConn;

use crate::fc_header_utils::wait_for_headers;

use crate::generated::waypoint_generated::waypoint;
use crate::generated::waypoint_generated::waypoint::Waypoint;
use crate::generated::waypoint_generated::waypoint::WaypointArgs;
use crate::receiver_utils::recv_message_within;

use flatbuffers::FlatBufferBuilder;
use mavlink::MavHeader;
use mavlink::ardupilotmega::MISSION_ACK_DATA;
use mavlink::ardupilotmega::MavFrame;

use mavlink::ardupilotmega::MISSION_ITEM_INT_DATA;
use mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA;
use mavlink::ardupilotmega::MavCmd;
use mavlink::ardupilotmega::MavMessage;
use mavlink::{self};

use tokio::sync::broadcast::Receiver;

use std::time::Duration;

pub async fn get_missions(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
) -> Vec<MissionSegment> {
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
        Some(MavMessage::MISSION_COUNT(data)) => Some(data.count),
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

    let mut missions = Vec::new();

    for elem in packets {
        match elem {
            Some(pack) => match pack {
                MavMessage::MISSION_ITEM_INT(data) => {
                    let cmd = data.command;

                    missions.push(MissionSegment::new(cmd, data));

                    // match cmd {
                    //     mavlink::ardupilotmega::MavCmd::MAV_CMD_NAV_WAYPOINT => {
                    //         let mut fbb = FlatBufferBuilder::new();
                    //         Waypoint::create(
                    //             &mut fbb,
                    //             &WaypointArgs {
                    //                 hold_time: data.param1,
                    //                 heading_target: data.param2,
                    //                 pass_rad: data.param3,
                    //                 accept_rad: data.param4,
                    //                 lat: data.x,
                    //                 lon: data.y,
                    //                 alt: data.z,
                    //             },
                    //         );

                    //     }
                    //     _ => {}
                    // }
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

    return missions;
}

pub fn get_all_waypoints(items: Vec<MissionSegment>) {
    for i in items {
        match i.cmd {
            mavlink::ardupilotmega::MavCmd::MAV_CMD_NAV_WAYPOINT => {
                let mut fbb = FlatBufferBuilder::new();

                let frame = match i.data.frame {
                    MavFrame::MAV_FRAME_GLOBAL => waypoint::FrameType::GLOBAL,
                    MavFrame::MAV_FRAME_BODY_FRD => waypoint::FrameType::BODY_FRD,
                    MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT => {
                        waypoint::FrameType::GLOBAL_RELATIVE_ALT
                    }

                    MavFrame::MAV_FRAME_LOCAL_ENU => waypoint::FrameType::LOCAL_ENU,
                    MavFrame::MAV_FRAME_LOCAL_NED => waypoint::FrameType::LOCAL_NED,
                    MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT => {
                        waypoint::FrameType::GLOBAL_TERRAIN_ALT
                    }

                    _ => waypoint::FrameType::GLOBAL,
                };

                Waypoint::create(
                    &mut fbb,
                    &WaypointArgs {
                        hold_time: i.data.param1,
                        heading: i.data.param2,
                        pass_radius: i.data.param3,
                        acceptance_radius: i.data.param4,
                        frame: frame,
                        lat: i.data.x,
                        lon: i.data.y,
                        alt: i.data.z,
                    },
                );
            }
            _ => {}
        }
    }
}

pub struct MissionSegment {
    pub cmd: mavlink::ardupilotmega::MavCmd,
    pub data: mavlink::ardupilotmega::MISSION_ITEM_INT_DATA,
}

impl MissionSegment {
    fn new(cmd: MavCmd, data: MISSION_ITEM_INT_DATA) -> Self {
        MissionSegment { cmd, data }
    }
}
