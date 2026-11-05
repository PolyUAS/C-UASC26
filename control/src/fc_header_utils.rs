//! Flight-controller (FC) identity and latest position, learned from the MAVLink link.
//!
//! The FC identity (system id, component id, autopilot type) is taken from the first
//! HEARTBEAT that comes from an autopilot: `autopilot != MAV_AUTOPILOT_INVALID` and
//! `mavtype != MAV_TYPE_GCS`. Companion computers, gimbals and cameras send
//! MAV_AUTOPILOT_INVALID, and ground stations send MAV_TYPE_GCS, so neither is mistaken
//! for the FC. The identity is set once (`OnceLock`) and never changes while control runs.
//!
//! Optional env `CU_FC_SYSID=<1..255>`: only accept an autopilot with that system id
//! (useful if one link carries several vehicles). Unset = first autopilot seen wins.

use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::ardupilotmega::{HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage, MavType};

/// How often `wait_for_fc` logs while no FC heartbeat has been seen.
pub const IDENTITY_LOG_PERIOD: Duration = Duration::from_secs(5);
/// Poll period of `wait_for_fc`.
const IDENTITY_POLL_PERIOD: Duration = Duration::from_millis(100);
/// Env var that restricts which autopilot system id is accepted.
pub const FC_SYSID_ENV: &str = "CU_FC_SYSID";

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FcIdentity {
    pub sysid: u8,
    pub compid: u8,
    pub autopilot: MavAutopilot,
}

/// Latest GLOBAL_POSITION_INT altitudes from the FC (MAVLink units: mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpiSnapshot {
    /// Altitude AMSL, mm.
    pub alt_mm: i32,
    /// Altitude above home, mm.
    pub relative_alt_mm: i32,
    /// Local receive time.
    pub at: Instant,
}

static FC: OnceLock<FcIdentity> = OnceLock::new();
static EXPECTED_SYSID: OnceLock<Option<u8>> = OnceLock::new();
static LAST_GPI: Mutex<Option<GpiSnapshot>> = Mutex::new(None);

/// Pure classifier: is this HEARTBEAT from an autopilot we should treat as the FC?
pub fn identity_from_heartbeat(
    header: &MavHeader,
    hb: &HEARTBEAT_DATA,
    expected_sysid: Option<u8>,
) -> Option<FcIdentity> {
    if header.system_id == 0 {
        return None; // 0 is the broadcast id, never a valid source
    }
    if hb.autopilot == MavAutopilot::MAV_AUTOPILOT_INVALID || hb.mavtype == MavType::MAV_TYPE_GCS {
        return None;
    }
    if let Some(want) = expected_sysid
        && want != header.system_id
    {
        return None;
    }
    Some(FcIdentity {
        sysid: header.system_id,
        compid: header.component_id,
        autopilot: hb.autopilot,
    })
}

/// Pure filter: should this inbound message be republished as FC telemetry?
/// Nothing is republished before the FC is known. Only the FC system id passes,
/// and a HEARTBEAT must also come from the FC component (so a gimbal or companion
/// heartbeat on the same system id is not reported as FC health).
pub fn should_republish(fc: Option<FcIdentity>, header: &MavHeader, msg: &MavMessage) -> bool {
    let Some(fc) = fc else { return false };
    if header.system_id != fc.sysid {
        return false;
    }
    match msg {
        MavMessage::HEARTBEAT(_) => header.component_id == fc.compid,
        _ => true,
    }
}

fn expected_sysid() -> Option<u8> {
    *EXPECTED_SYSID.get_or_init(|| match std::env::var(FC_SYSID_ENV) {
        Ok(v) => match v.trim().parse::<u8>() {
            Ok(id) if id != 0 => Some(id),
            _ => {
                eprintln!("control: ignoring invalid {FC_SYSID_ENV}={v:?} (want 1..255)");
                None
            }
        },
        Err(_) => None,
    })
}

/// Feed every inbound message here (from the single MAVLink receive task).
pub fn observe(header: &MavHeader, msg: &MavMessage) {
    match msg {
        MavMessage::HEARTBEAT(hb) => {
            if FC.get().is_none()
                && let Some(id) = identity_from_heartbeat(header, hb, expected_sysid())
                && FC.set(id).is_ok()
            {
                println!(
                    "control: FC sysid={} compid={} autopilot={:?} type={:?}",
                    id.sysid, id.compid, id.autopilot, hb.mavtype
                );
            }
        }
        MavMessage::GLOBAL_POSITION_INT(d) => {
            if let Some(fc) = FC.get()
                && header.system_id == fc.sysid
            {
                let snap = GpiSnapshot {
                    alt_mm: d.alt,
                    relative_alt_mm: d.relative_alt,
                    at: Instant::now(),
                };
                *LAST_GPI.lock().unwrap_or_else(|e| e.into_inner()) = Some(snap);
            }
        }
        MavMessage::COMMAND_ACK(ack) if ack.command == MavCmd::MAV_CMD_DO_REPOSITION => {
            if let Some(fc) = FC.get()
                && header.system_id == fc.sysid
            {
                println!("control: DO_REPOSITION ack {:?}", ack.result);
            }
        }
        _ => {}
    }
}

/// The FC identity, if a FC heartbeat has been seen.
pub fn fc() -> Option<FcIdentity> {
    FC.get().copied()
}

/// Latest FC GLOBAL_POSITION_INT altitudes, if any.
pub fn last_gpi() -> Option<GpiSnapshot> {
    *LAST_GPI.lock().unwrap_or_else(|e| e.into_inner())
}

/// Wait until the FC identity is known. Logs every `IDENTITY_LOG_PERIOD` while waiting.
pub async fn wait_for_fc(who: &str) -> FcIdentity {
    let start = Instant::now();
    let mut next_log = start;
    loop {
        if let Some(id) = fc() {
            return id;
        }
        if Instant::now() >= next_log {
            println!(
                "control: {who}: waiting for FC HEARTBEAT (autopilot != INVALID, type != GCS), {:.0} s so far",
                start.elapsed().as_secs_f32()
            );
            next_log += IDENTITY_LOG_PERIOD;
        }
        tokio::time::sleep(IDENTITY_POLL_PERIOD).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::ardupilotmega::GLOBAL_POSITION_INT_DATA;

    fn hdr(sysid: u8, compid: u8) -> MavHeader {
        MavHeader {
            system_id: sysid,
            component_id: compid,
            sequence: 0,
        }
    }

    fn hb(mavtype: MavType, autopilot: MavAutopilot) -> HEARTBEAT_DATA {
        HEARTBEAT_DATA {
            mavtype,
            autopilot,
            ..Default::default()
        }
    }

    #[test]
    fn fc_identity_accepts_px4_sysid2_comp1() {
        let id = identity_from_heartbeat(
            &hdr(2, 1),
            &hb(MavType::MAV_TYPE_QUADROTOR, MavAutopilot::MAV_AUTOPILOT_PX4),
            None,
        );
        assert_eq!(
            id,
            Some(FcIdentity {
                sysid: 2,
                compid: 1,
                autopilot: MavAutopilot::MAV_AUTOPILOT_PX4
            })
        );
    }

    #[test]
    fn fc_identity_accepts_ardupilot() {
        let id = identity_from_heartbeat(
            &hdr(7, 1),
            &hb(MavType::MAV_TYPE_QUADROTOR, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA),
            None,
        )
        .expect("ArduPilot heartbeat must be accepted");
        assert_eq!((id.sysid, id.compid), (7, 1));
        assert_eq!(id.autopilot, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA);
    }

    #[test]
    fn fc_identity_rejects_gcs_255() {
        // QGC sends type GCS, autopilot INVALID; some GCS tools send a non-INVALID autopilot.
        for ap in [MavAutopilot::MAV_AUTOPILOT_INVALID, MavAutopilot::MAV_AUTOPILOT_GENERIC] {
            assert_eq!(
                identity_from_heartbeat(&hdr(255, 190), &hb(MavType::MAV_TYPE_GCS, ap), None),
                None
            );
        }
    }

    #[test]
    fn fc_identity_rejects_onboard_controller_invalid() {
        // This is what control itself sends (heartbeat.rs).
        assert_eq!(
            identity_from_heartbeat(
                &hdr(1, 191),
                &hb(MavType::MAV_TYPE_ONBOARD_CONTROLLER, MavAutopilot::MAV_AUTOPILOT_INVALID),
                None
            ),
            None
        );
    }

    #[test]
    fn fc_identity_rejects_gimbal_and_sysid0() {
        assert_eq!(
            identity_from_heartbeat(
                &hdr(1, 154),
                &hb(MavType::MAV_TYPE_GIMBAL, MavAutopilot::MAV_AUTOPILOT_INVALID),
                None
            ),
            None
        );
        assert_eq!(
            identity_from_heartbeat(
                &hdr(0, 1),
                &hb(MavType::MAV_TYPE_QUADROTOR, MavAutopilot::MAV_AUTOPILOT_PX4),
                None
            ),
            None
        );
    }

    #[test]
    fn fc_identity_respects_expected_sysid() {
        let px4 = hb(MavType::MAV_TYPE_QUADROTOR, MavAutopilot::MAV_AUTOPILOT_PX4);
        assert_eq!(identity_from_heartbeat(&hdr(3, 1), &px4, Some(2)), None);
        assert_eq!(
            identity_from_heartbeat(&hdr(2, 1), &px4, Some(2)).map(|i| i.sysid),
            Some(2)
        );
    }

    #[test]
    fn fc_identity_republish_filter() {
        let fc = FcIdentity {
            sysid: 2,
            compid: 1,
            autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
        };
        let gpi = MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA::default());
        let beat = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        assert!(!should_republish(None, &hdr(2, 1), &gpi), "nothing before FC known");
        assert!(should_republish(Some(fc), &hdr(2, 1), &gpi));
        assert!(!should_republish(Some(fc), &hdr(3, 1), &gpi), "other vehicle dropped");
        assert!(!should_republish(Some(fc), &hdr(255, 190), &beat), "GCS dropped");
        assert!(should_republish(Some(fc), &hdr(2, 1), &beat));
        assert!(!should_republish(Some(fc), &hdr(2, 154), &beat), "gimbal HB on FC sysid dropped");
    }
}
