use mavlink::{
    ardupilotmega::{MavCmd, MavFrame, MavMessage, MavModeFlag},
    common::MavDoRepositionFlags::MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
};
use zenoh::{Session, pubsub::Subscriber};

use crate::{
    SharedAsyncMavConn,
    fc_header_utils::get_headers,
    generated::waypoint_generated::waypoint::{self, Waypoint, root_as_waypoint},
    telemetry::IS_GUIDED_ENABLED,
};

pub async fn create_movement_subscriber(
    session: Session,
    conn: SharedAsyncMavConn,
) -> Option<Subscriber<()>> {
    session
        .declare_subscriber("control/in/target_pos")
        .callback(move |sample| {
            let payload = sample.payload().to_bytes().into_owned();
            let conn = conn.clone();

            tokio::spawn(async move {
                if let Ok(target) = root_as_waypoint(&*payload) {
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

        if !IS_GUIDED_ENABLED.load(std::sync::atomic::Ordering::Acquire) {
            println!("GUIDED NOT ENABLED!!! Reposition rejected.");
            return;
        }

        println!("Sending Reposition Command to Sys: {sys}, Comp: {comp}");

        // Use COMMAND_INT for precision movement to the proper components
        let res = conn
            .send_default(&MavMessage::COMMAND_INT(
                mavlink::ardupilotmega::COMMAND_INT_DATA {
                    param1: target.speed(),
                    param2: MAV_DO_REPOSITION_FLAGS_CHANGE_MODE as u32 as f32, // Let FC know it can apply guided updates
                    param3: target.acceptance_radius(),
                    param4: target.heading(),
                    x: target.lat(),
                    y: target.lon(),
                    z: target.alt(),
                    command: MavCmd::MAV_CMD_DO_REPOSITION,
                    target_system: sys,     // Fixed from hardcoded 1
                    target_component: comp, // Fixed from hardcoded 0
                    frame,
                    current: 0,
                    autocontinue: 0,
                },
            ))
            .await;

        if let Err(e) = res {
            println!("Error moving drone, write err: {e}");
        }
    } else {
        println!("Cannot execute move_to: No valid flight controller headers found yet!");
    }
}
