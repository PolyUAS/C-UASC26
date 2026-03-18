use crate::SharedAsyncMavConn;
use flatbuffers::{self, FlatBufferBuilder};

use mavlink::ardupilotmega::PARAM_REQUEST_READ_DATA;

use mavlink::MavHeader;
use mavlink::ardupilotmega::FENCE_FETCH_POINT_DATA;
use mavlink::ardupilotmega::FENCE_POINT_DATA;
use mavlink::ardupilotmega::MavMessage;

use mavlink::types::CharArray;
use mavlink::{self, MavlinkVersion};

use tokio::sync::broadcast::Receiver;
use tokio::sync::broadcast::Sender;
use zenoh::Session;

use zenoh::query::Queryable;

use std::error::Error;
use std::sync::Arc;

use std::time::Duration;

use zenoh::{self};

use crate::geofence_generated::geofence::Coords;
use crate::geofence_generated::geofence::GeofencePacket;
use crate::geofence_generated::geofence::GeofencePacketArgs;

pub async fn get_geofence(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
) -> Result<Vec<FENCE_POINT_DATA>, Box<dyn Error + Send + Sync>> {
    // 1. Request FENCE_TOTAL parameter
    let name = "FENCE_TOTAL";
    assert!(name.len() <= 16, "Param name too long");
    let mut bytes = [0u8; 16];
    bytes[..name.len()].copy_from_slice(name.as_bytes());

    vehicle
        .send_default(&MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
            target_system: 1, // adjust if needed
            target_component: 1,
            param_id: CharArray::new(bytes),
            param_index: -1, // use name, not index
        }))
        .await?;

    let mut total_points: u8 = 0;
    let timeout = Duration::from_secs(2);

    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        if let Ok((_, msg)) = bc_rx.recv().await {
            if let MavMessage::PARAM_VALUE(pv) = msg {
                if pv.param_id == CharArray::new(b"FENCE_TOTAL\0\0\0\0\0".to_owned()) {
                    // null-padded comparison
                    total_points = pv.param_value as u8; // or as u32 then check range
                    break;
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    if total_points == 0 {
        return Ok(Vec::new());
    }

    // 2. Fetch each point — indices usually 0 = return point, 1..total = boundary
    let mut points = Vec::with_capacity(total_points as usize + 1);

    for idx in 0..=total_points {
        // include return point
        vehicle
            .send_default(&MavMessage::FENCE_FETCH_POINT(FENCE_FETCH_POINT_DATA {
                target_system: 1,
                target_component: 1,
                idx: idx as u8, // note: idx is u8
                ..Default::default()
            }))
            .await?;

        let start = std::time::Instant::now();
        while start.elapsed() < Duration::from_millis(500) {
            if let Ok((_, msg)) = bc_rx.recv().await {
                if let MavMessage::FENCE_POINT(point) = msg {
                    if point.idx == idx as u8 {
                        println!(
                            "Fence point {}: lat={}, lng={}",
                            point.idx, point.lat, point.lng
                        );
                        points.push(point);
                        break;
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
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
