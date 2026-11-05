use std::time::{Duration, Instant};

use mavlink::ardupilotmega::{MavAutopilot, MavCmd, MavFrame, MavMessage};
use zenoh::{Session, pubsub::Subscriber};

use crate::{
    SharedAsyncMavConn,
    fc_header_utils::{GpiSnapshot, fc, last_gpi},
    generated::waypoint_generated::waypoint::{self, Waypoint, root_as_waypoint},
};

/// Oldest GLOBAL_POSITION_INT accepted for the PX4 home-altitude offset.
/// ASSUMED: 5x the 200 ms GLOBAL_POSITION_INT interval that main.rs requests.
pub const GPI_MAX_AGE: Duration = Duration::from_secs(1);

#[derive(Debug, PartialEq)]
pub enum RepositionError {
    /// No GLOBAL_POSITION_INT from the FC yet, so the home AMSL altitude is unknown.
    NoPositionOffset,
    /// The last GLOBAL_POSITION_INT is older than GPI_MAX_AGE.
    StalePosition(Duration),
    /// This frame cannot be sent safely to this autopilot.
    FrameNotSupported(MavFrame),
    /// Only PX4 and ArduPilot are handled.
    AutopilotNotSupported(MavAutopilot),
}

/// Pure: choose the DO_REPOSITION frame and z (m) for the autopilot.
///
/// ArduPilot honours the frame, so the input passes through unchanged.
/// PX4 ignores the frame for z (mavlink_receiver copies z to param7 and the navigator uses
/// param7 as AMSL), so GLOBAL_RELATIVE_ALT is converted to GLOBAL with
/// z_amsl = z_rel + (alt_mm - relative_alt_mm) / 1000, i.e. z_rel + home AMSL.
/// The conversion needs a fresh GLOBAL_POSITION_INT; without one the command is refused.
/// GLOBAL passes through. LOCAL, BODY and TERRAIN frames are refused on PX4.
pub fn plan_reposition(
    autopilot: MavAutopilot,
    frame: MavFrame,
    z_m: f32,
    gpi: Option<GpiSnapshot>,
    now: Instant,
) -> Result<(MavFrame, f32), RepositionError> {
    match autopilot {
        MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA => Ok((frame, z_m)),
        MavAutopilot::MAV_AUTOPILOT_PX4 => match frame {
            MavFrame::MAV_FRAME_GLOBAL => Ok((frame, z_m)),
            MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT => {
                let g = gpi.ok_or(RepositionError::NoPositionOffset)?;
                let age = now.saturating_duration_since(g.at);
                if age > GPI_MAX_AGE {
                    return Err(RepositionError::StalePosition(age));
                }
                let home_amsl_m = (f64::from(g.alt_mm) - f64::from(g.relative_alt_mm)) / 1000.0;
                Ok((MavFrame::MAV_FRAME_GLOBAL, (f64::from(z_m) + home_amsl_m) as f32))
            }
            other => Err(RepositionError::FrameNotSupported(other)),
        },
        other => Err(RepositionError::AutopilotNotSupported(other)),
    }
}

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
    if let Some(fc) = fc() {
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

        let (frame, z) =
            match plan_reposition(fc.autopilot, frame, target.alt(), last_gpi(), Instant::now()) {
                Ok(v) => v,
                Err(e) => {
                    eprintln!("control: DO_REPOSITION refused, not sent: {e:?}");
                    return;
                }
            };

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
                    z,
                    command: MavCmd::MAV_CMD_DO_REPOSITION,
                    target_system: fc.sysid,
                    target_component: fc.compid,
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
    } else {
        eprintln!("control: DO_REPOSITION not sent: FC identity unknown (no FC HEARTBEAT yet)");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PX4: MavAutopilot = MavAutopilot::MAV_AUTOPILOT_PX4;
    const APM: MavAutopilot = MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA;
    const REL: MavFrame = MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT;

    fn snap(alt_mm: i32, relative_alt_mm: i32, at: Instant) -> Option<GpiSnapshot> {
        Some(GpiSnapshot {
            alt_mm,
            relative_alt_mm,
            at,
        })
    }

    #[test]
    fn reposition_alt_px4_rel_to_amsl() {
        let t = Instant::now();
        let (f, z) = plan_reposition(PX4, REL, 20.0, snap(312_000, 12_000, t), t).unwrap();
        assert_eq!(f, MavFrame::MAV_FRAME_GLOBAL);
        assert!((z - 320.0).abs() <= 0.01, "z={z}");
    }

    #[test]
    fn reposition_alt_px4_sih_home() {
        // SIH default home 489.4 m AMSL: vehicle at 501.4 m AMSL, 12 m above home.
        let t = Instant::now();
        let (f, z) = plan_reposition(PX4, REL, 20.0, snap(501_400, 12_000, t), t).unwrap();
        assert_eq!(f, MavFrame::MAV_FRAME_GLOBAL);
        assert!((z - 509.4).abs() <= 0.001, "z={z}");
    }

    #[test]
    fn reposition_alt_px4_global_passes() {
        let t = Instant::now();
        let r = plan_reposition(PX4, MavFrame::MAV_FRAME_GLOBAL, 505.0, None, t);
        assert_eq!(r, Ok((MavFrame::MAV_FRAME_GLOBAL, 505.0)));
    }

    #[test]
    fn reposition_alt_px4_unknown_offset_refused() {
        let t = Instant::now();
        assert_eq!(
            plan_reposition(PX4, REL, 20.0, None, t),
            Err(RepositionError::NoPositionOffset)
        );
    }

    #[test]
    fn reposition_alt_px4_stale_refused() {
        let t0 = Instant::now();
        let now = t0 + Duration::from_millis(1500);
        assert!(matches!(
            plan_reposition(PX4, REL, 20.0, snap(312_000, 12_000, t0), now),
            Err(RepositionError::StalePosition(_))
        ));
        // exactly at the limit is still accepted
        let now = t0 + GPI_MAX_AGE;
        assert!(plan_reposition(PX4, REL, 20.0, snap(312_000, 12_000, t0), now).is_ok());
    }

    #[test]
    fn reposition_alt_px4_local_body_terrain_refused() {
        let t = Instant::now();
        for f in [
            MavFrame::MAV_FRAME_LOCAL_NED,
            MavFrame::MAV_FRAME_LOCAL_ENU,
            MavFrame::MAV_FRAME_BODY_FRD,
            MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
        ] {
            assert_eq!(
                plan_reposition(PX4, f, 20.0, snap(312_000, 12_000, t), t),
                Err(RepositionError::FrameNotSupported(f))
            );
        }
    }

    #[test]
    fn reposition_alt_ardupilot_unchanged() {
        let t = Instant::now();
        assert_eq!(plan_reposition(APM, REL, 20.0, None, t), Ok((REL, 20.0)));
        assert_eq!(
            plan_reposition(APM, REL, 20.0, snap(312_000, 12_000, t), t),
            Ok((REL, 20.0))
        );
        assert_eq!(
            plan_reposition(APM, MavFrame::MAV_FRAME_LOCAL_NED, 3.0, None, t),
            Ok((MavFrame::MAV_FRAME_LOCAL_NED, 3.0))
        );
    }

    #[test]
    fn reposition_alt_other_autopilot_refused() {
        let t = Instant::now();
        assert_eq!(
            plan_reposition(MavAutopilot::MAV_AUTOPILOT_GENERIC, REL, 20.0, None, t),
            Err(RepositionError::AutopilotNotSupported(
                MavAutopilot::MAV_AUTOPILOT_GENERIC
            ))
        );
    }
}
