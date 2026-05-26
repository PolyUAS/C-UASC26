use mavlink::ardupilotmega::{MavCmd, MavFrame, MavMessage};
use zenoh::{Session, pubsub::Subscriber};

use crate::{
    SharedAsyncMavConn,
    fc_header_utils::get_headers,
    generated::waypoint_generated::waypoint::{self, Waypoint, root_as_waypoint},
};

pub async fn create_movement_subscriber(
    session: Session,
    conn: SharedAsyncMavConn,
) -> Option<Subscriber<()>> {
    session
        .declare_subscriber("control/in/target_pos")
        .callback(move |sample| {
            let payload = sample.payload().to_bytes().into_owned(); // ensure on MY heap

            let conn = conn.clone(); // clone the conenction for the closure

            tokio::spawn(async move {
                let target = root_as_waypoint(&*payload).ok();
                if let Some(target) = target {
                    move_to(conn, target).await;
                }
            });
        })
        .await
        .ok()
}

pub async fn move_to<'a>(conn: SharedAsyncMavConn, target: Waypoint<'a>) {
    let (sys, comp) = get_headers();
    if let Some(sys) = sys
        && let Some(comp) = comp
    {
        let frame = match target.frame() {
            waypoint::FrameType::GLOBAL => MavFrame::MAV_FRAME_GLOBAL,
            waypoint::FrameType::BODY_FRD => MavFrame::MAV_FRAME_BODY_FRD,
            waypoint::FrameType::GLOBAL_RELATIVE_ALT => MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            waypoint::FrameType::LOCAL_ENU => MavFrame::MAV_FRAME_LOCAL_ENU,
            waypoint::FrameType::LOCAL_NED => MavFrame::MAV_FRAME_LOCAL_NED,
            waypoint::FrameType::GLOBAL_TERRAIN_ALT => MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
            _ => MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        };

        // {
        //     let lat = target.lat();
        //     let lon = target.lon();
        //     let alt = target.alt();
        //     let hdg = target.heading();
        //     let ar = target.acceptance_radius();
        //     let spd = target.speed();

        //     println!(
        //         "
        //     Target:
        //     lat {lat} lon {lon} alt {alt}
        //     hdg {hdg} spd {spd}
        //     ar {ar}
        //     "
        //     );
        // }

        println!("RECV MSG");
        let res = conn
            .send_default(&MavMessage::COMMAND_INT(
                mavlink::ardupilotmega::COMMAND_INT_DATA {
                    param1: target.speed(),             // speed: -1 = use default
                    param2: 0.0,                      // bitmask: no flags
                    param3: target.acceptance_radius(), // radius: 0 = default
                    param4: target.heading(),           // yaw: NaN = unchanged
                    x: target.lat(),
                    y: target.lon(),
                    z: target.alt(),
                    command: MavCmd::MAV_CMD_DO_REPOSITION,
                    target_system: 1,
                    target_component: 0,
                    frame: frame,
                    current: 0,
                    autocontinue: 0,
                },
            ))
            .await;

        match res {
            Ok(_) => {}
            Err(e) => {
                println!("Error moving drone, write err: {e}");
            }
        }
    }
}
