use mavlink::{
    ardupilotmega::{COMMAND_INT_DATA, GLOBAL_POSITION_INT_DATA, MavCmd, MavFrame, MavMessage},
    common::SET_POSITION_TARGET_GLOBAL_INT_DATA,
};

use crate::{
    SharedAsyncMavConn, fc_header_utils::get_headers, generated::goto_target_generated::Pose,
};

pub async fn move_to<'a>(conn: SharedAsyncMavConn, target: Pose<'a>) {
    let (sys, comp) = get_headers();
    if let Some(sys) = sys
        && let Some(comp) = comp
    {
        let res = conn
            .send_default(&MavMessage::COMMAND_INT(
                mavlink::ardupilotmega::COMMAND_INT_DATA {
                    param1: target.hold_time(),
                    param2: target.acceptance_radius(),
                    param3: target.pass_radius(),
                    param4: target.heading(),
                    x: target.lat(),
                    y: target.lon(),
                    z: target.alt(),
                    command: MavCmd::MAV_CMD_NAV_WAYPOINT,
                    target_system: sys,
                    target_component: comp,
                    frame: MavFrame::MAV_FRAME_GLOBAL,
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
