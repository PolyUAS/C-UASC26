use crate::SharedAsyncMavConn;
use crate::missions::waypoint::WaypointListArgs;

use crate::fc_header_utils::wait_for_headers;

use crate::generated::waypoint_generated::waypoint;
use crate::generated::waypoint_generated::waypoint::Waypoint;
use crate::generated::waypoint_generated::waypoint::WaypointArgs;
use crate::generated::waypoint_generated::waypoint::WaypointList;
use crate::receiver_utils::recv_message_within;

use flatbuffers::FlatBufferBuilder;
use flatbuffers::WIPOffset;
use mavlink::MavHeader;
use mavlink::ardupilotmega::MISSION_ACK_DATA;
use mavlink::ardupilotmega::MavFrame;

use mavlink::ardupilotmega::MISSION_ITEM_INT_DATA;
use mavlink::ardupilotmega::MISSION_REQUEST_INT_DATA;
use mavlink::ardupilotmega::MavCmd;
use mavlink::ardupilotmega::MavMessage;
use mavlink::{self};

use tokio::io::unix::AsyncFdTryNewError;
use tokio::sync::Mutex;
use zenoh::Session;
use zenoh::pubsub::Publisher;

use tokio::sync::broadcast::Receiver;
use zenoh::query::Query;
use zenoh::query::Queryable;

use std::sync::Arc;
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

pub fn filter_waypoints<'a>(items: Vec<MissionSegment>) -> Vec<MissionSegment> {
    let mut v = Vec::new();
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
                        speed: 0.0, // invalid anyways
                        heading: i.data.param2,
                        acceptance_radius: i.data.param4,
                        frame: frame,
                        lat: i.data.x,
                        lon: i.data.y,
                        alt: i.data.z,
                    },
                );
                v.push(i);
            }
            _ => {}
        }
    }
    v
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

async fn init_mission_queryable(
    conn: SharedAsyncMavConn,
    session: &Session,
    bc_rx: Receiver<(MavHeader, MavMessage)>,
) -> Queryable<()> {
    let bc_rx = Arc::new(Mutex::new(bc_rx));
    // These are what the closure captures — they must live for all N calls
    let conn = Arc::new(conn);

    session
        .declare_queryable("control/mission/query_in")
        .callback(move |query: Query| {
            // These cheap clones happen on EVERY call, giving each task its own handle
            // The closure itself still holds `conn` and `bc_rx` after this
            let conn = Arc::clone(&conn);
            let bc_rx = Arc::clone(&bc_rx);

            let _h = tokio::spawn(async move {
                let mut bc_rx = bc_rx.lock().await.resubscribe();
                let ms = get_missions(&conn, &mut bc_rx).await;
                let wps = filter_waypoints(ms);
                let mut fbb = FlatBufferBuilder::new();
                let mut v = Vec::new();
                for i in wps {
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
                    let w = Waypoint::create(
                        &mut fbb,
                        &WaypointArgs {
                            heading: i.data.param2,
                            speed: -1.0, // irrelevant
                            acceptance_radius: i.data.param4,
                            frame,
                            lat: i.data.x,
                            lon: i.data.y,
                            alt: i.data.z,
                        },
                    );
                    v.push(w);
                }
                let vwps = fbb.create_vector(&v);
                let table = WaypointList::create(&mut fbb, &WaypointListArgs { wps: Some(vwps) });
                fbb.finish(table, None);
                let payload = fbb.finished_data().to_vec();
                query.reply("control/mission/query_out", payload).await.ok();
            });
        })
        .await
        .expect("failed to initialize mission queryable")
}
