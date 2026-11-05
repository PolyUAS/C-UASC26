use crate::SharedAsyncMavConn;
use crate::fc_header_utils;
use crate::receiver_utils::recv_message_within;
use flatbuffers::{self, FlatBufferBuilder};

use mavlink::ardupilotmega::PARAM_REQUEST_READ_DATA;

use mavlink::MavHeader;
use mavlink::ardupilotmega::FENCE_FETCH_POINT_DATA;
use mavlink::ardupilotmega::FENCE_POINT_DATA;
use mavlink::ardupilotmega::MavMessage;

use mavlink::types::CharArray;
use mavlink::{self};

use tokio::sync::broadcast::Receiver;
use tokio::sync::broadcast::Sender;
use zenoh::Session;

use zenoh::query::Queryable;

use std::error::Error;

use std::time::Duration;

use zenoh::{self};

use crate::geofence_generated::geofence::Coords;
use crate::geofence_generated::geofence::GeofencePacket;
use crate::geofence_generated::geofence::GeofencePacketArgs;

/// True if a MAVLink param_id (up to 16 bytes, NUL-padded, no NUL if exactly 16 long)
/// is exactly `name`: the name bytes match and are followed by a NUL or the end of the id.
pub fn param_id_is(id: &[u8], name: &str) -> bool {
    let n = name.as_bytes();
    n.len() <= id.len() && id[..n.len()] == *n && id.get(n.len()).is_none_or(|&b| b == 0)
}

pub async fn get_geofence(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
) -> Result<Vec<FENCE_POINT_DATA>, Box<dyn Error + Send + Sync>> {
    // 1. Request FENCE_TOTAL parameter
    let name = "FENCE_TOTAL";
    assert!(name.len() <= 16, "Param name too long");
    let mut name_as_bytes = [0u8; 16];
    name_as_bytes[..name.len()].copy_from_slice(name.as_bytes());

    let fc = fc_header_utils::wait_for_fc("geofence query").await;
    let (sys, comp) = (fc.sysid, fc.compid);

    vehicle
        .send_default(&MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
            target_system: sys, // adjust if needed
            target_component: comp,
            param_id: CharArray::new(name_as_bytes),
            param_index: -1, // use name, not index
        }))
        .await?;

    let timeout = Duration::from_secs(2);

    // bounded wait (the old loop could block forever in recv and slept between messages)
    let total_points: u8 = match recv_message_within(timeout, bc_rx, |m| {
        matches!(m, MavMessage::PARAM_VALUE(pv) if param_id_is(&pv.param_id[..], name))
    })
    .await
    {
        Ok(MavMessage::PARAM_VALUE(pv)) => pv.param_value as u8,
        _ => 0,
    };

    if total_points == 0 {
        return Ok(Vec::new());
    }

    // 2. Fetch each point — indices usually 0 = return point, 1..total = boundary
    let mut points = Vec::with_capacity(total_points as usize + 1);

    for idx in 0..total_points {
        vehicle
            .send_default(&MavMessage::FENCE_FETCH_POINT(FENCE_FETCH_POINT_DATA {
                target_system: sys,
                target_component: comp,
                idx: idx as u8, // note: idx is u8
                ..Default::default()
            }))
            .await?;

        if let Ok(MavMessage::FENCE_POINT(point)) =
            recv_message_within(Duration::from_millis(500), bc_rx, |m| {
                matches!(m, MavMessage::FENCE_POINT(p) if p.idx == idx)
            })
            .await
        {
            println!(
                "Fence point {}: lat={}, lng={}",
                point.idx, point.lat, point.lng
            );
            points.push(point);
        }
    }

    Ok(points)
}

pub async fn create_geofence_queryable(
    session: &Session,
    vehicle: SharedAsyncMavConn,
    bc_tx: Sender<(MavHeader, MavMessage)>,
) -> Option<Queryable<()>> {
    session
        .declare_queryable("control/query/geofence")
        .callback({
            move |query| {
                let vehicle = vehicle.clone();
                let mut bc_rx = bc_tx.subscribe();
                tokio::spawn(async move {
                    match get_geofence(&vehicle, &mut bc_rx).await {
                        Ok(points) if !points.is_empty() => {
                            let mut builder = FlatBufferBuilder::new();
                            let coords: Vec<Coords> =
                                points.iter().map(|p| Coords::new(p.lat, p.lng)).collect();
                            let points_offset = builder.create_vector(&coords);
                            let packet = GeofencePacket::create(
                                &mut builder,
                                &GeofencePacketArgs {
                                    points: Some(points_offset),
                                },
                            );
                            builder.finish(packet, None);
                            let payload =
                                zenoh::bytes::ZBytes::from(builder.finished_data().to_vec());
                            let _ = query.reply("control/query/geofence", payload).await;
                        }
                        _ => {
                            let mut builder = FlatBufferBuilder::new();
                            let packet = GeofencePacket::create(
                                &mut builder,
                                &GeofencePacketArgs { points: None },
                            );
                            builder.finish(packet, None);
                            let payload =
                                zenoh::bytes::ZBytes::from(builder.finished_data().to_vec());
                            let _ = query.reply("control/query/geofence", payload).await;
                        }
                    }
                });
            }
        })
        .await
        .ok()
}

#[cfg(test)]
mod tests {
    use super::param_id_is;

    fn id16(s: &[u8]) -> [u8; 16] {
        let mut a = [0u8; 16];
        a[..s.len()].copy_from_slice(s);
        a
    }

    #[test]
    fn param_id_matches_nul_padded() {
        assert!(param_id_is(&id16(b"FENCE_TOTAL"), "FENCE_TOTAL"));
    }

    #[test]
    fn param_id_rejects_longer_name() {
        assert!(!param_id_is(&id16(b"FENCE_TOTALX"), "FENCE_TOTAL"));
        assert!(!param_id_is(&id16(b"FENCE_TOTAL_ABCD"), "FENCE_TOTAL"));
    }

    #[test]
    fn param_id_rejects_prefix() {
        assert!(!param_id_is(&id16(b"FENCE_TOTA"), "FENCE_TOTAL"));
        assert!(!param_id_is(&id16(b"FENCE_TOT"), "FENCE_TOTAL"));
    }

    #[test]
    fn param_id_full_16_bytes_no_nul() {
        assert!(param_id_is(&id16(b"ABCDEFGHIJKLMNOP"), "ABCDEFGHIJKLMNOP"));
        assert!(!param_id_is(&id16(b"ABCDEFGHIJKLMNOP"), "ABCDEFGHIJKLMNOPQ"));
    }

    #[test]
    fn param_id_empty() {
        assert!(!param_id_is(&[0u8; 16], "FENCE_TOTAL"));
        assert!(param_id_is(&[0u8; 16], ""));
    }
}
