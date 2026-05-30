use crate::SharedAsyncMavConn;
use crate::fc_header_utils;
use flatbuffers::FlatBufferBuilder;

use mavlink::MavHeader;
use mavlink::ardupilotmega::FENCE_FETCH_POINT_DATA;
use mavlink::ardupilotmega::PARAM_REQUEST_READ_DATA;
use mavlink::ardupilotmega::{
    MISSION_ACK_DATA, MISSION_REQUEST_INT_DATA, MISSION_REQUEST_LIST_DATA, MavMessage,
    MavMissionResult,
};

use mavlink::error::MessageWriteError;
use mavlink::types::CharArray;
use tokio::sync::broadcast::{Receiver, Sender};
use zenoh::Session;
use zenoh::query::Queryable;

use std::error::Error;
use std::time::Duration;

use crate::geofence_generated::geofence::{Coords, GeofencePacket, GeofencePacketArgs};

const COUNT_TIMEOUT: Duration = Duration::from_secs(3);
const ITEM_TIMEOUT: Duration = Duration::from_millis(1000);
const ITEM_MAX_RETRIES: u8 = 3;

async fn fetch_fence_count(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
    sys: u8,
    comp: u8,
) -> Result<u8, Box<dyn Error + Send + Sync>> {
    let name = "FENCE_TOTAL";
    let mut name_as_bytes = [0u8; 16];
    name_as_bytes[..name.len()].copy_from_slice(name.as_bytes());

    vehicle
        .send_default(&MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
            target_system: sys,
            target_component: comp,
            param_id: CharArray::new(name_as_bytes),
            param_index: -1,
        }))
        .await?;

    let deadline = std::time::Instant::now();
    while deadline.elapsed() < COUNT_TIMEOUT {
        tokio::select! {
                    Ok((_, msg)) = bc_rx.recv() => {
                        if let MavMessage::PARAM_VALUE(pv) = msg {
                            if pv.param_id.as_slice().starts_with(b"FENCE_TOTAL") {
                                let total_points = pv.param_value as u8;
                                return Ok(total_points);
                            } else {
                                continue;
                            }
                        }
                    }
                    _ = tokio::time::sleep(Duration::from_millis(100)) => {}
        }
    }

    Ok(0)
}

async fn fetch_fence_item(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
    sys: u8,
    comp: u8,
    idx: u8,
) -> Option<(f32, f32)> {
    for attempt in 0..ITEM_MAX_RETRIES {
        println!("grabbing fence item: {idx}, attempt: {attempt}");

        if let Err(e) = vehicle
            .send_default(&&MavMessage::FENCE_FETCH_POINT(FENCE_FETCH_POINT_DATA {
                target_system: sys,
                target_component: comp,
                idx: idx,
            }))
            .await
        {
            println!("Failed to send FENCE_FETCH_POINT for idx {idx}: {e}");
            continue;
        }

        let deadline = std::time::Instant::now();
        while deadline.elapsed() < ITEM_TIMEOUT {
            tokio::select! {
                result = bc_rx.recv() => {
                    match result {

                        Ok((_, MavMessage::FENCE_POINT(item))) if item.idx == idx => {
                            // MISSION_ITEM_INT stores lat/lng as 1e7 fixed-point integers
                            let lat = item.lat as f32 / 1e7;
                            let lng = item.lng as f32 / 1e7;
                            println!("Fence item {idx}: lat={lat}, lng={lng}");
                            return Some((lat, lng));
                        }
                        Ok(_) => {}
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                            println!("Receiver lagged by {n} on idx {idx}, retrying");
                            break;
                        }
                        Err(e) => {
                            println!("Broadcast recv error on idx {idx}: {e:?}");
                            return None;
                        }
                    }
                }
                _ = tokio::time::sleep(ITEM_TIMEOUT) => {
                    println!("Timeout waiting for fence item {idx}, attempt {attempt}");
                    break;
                }
            }
        }
    }

    println!("Failed to retrieve fence item {idx} after {ITEM_MAX_RETRIES} attempts");
    None
}

async fn send_mission_ack(
    vehicle: &SharedAsyncMavConn,
    sys: u8,
    comp: u8,
) -> Result<usize, MessageWriteError> {
    vehicle
        .send_default(&MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: sys,
            target_component: comp,
            mavtype: MavMissionResult::MAV_MISSION_ACCEPTED,
        }))
        .await
}

pub async fn get_geofence(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
) -> Result<Vec<(f32, f32)>, Box<dyn Error + Send + Sync>> {
    let (sys, comp) = fc_header_utils::wait_for_headers().await;

    let count = fetch_fence_count(vehicle, bc_rx, sys, comp).await?;

    if count == 0 {
        println!("Fence count was 0 or timed out — no geofence stored on FC");
        return Ok(Vec::new());
    }

    println!("Fetching {count} fence items...");

    let mut points = Vec::with_capacity(count as usize);
    for idx in 0..count {
        match fetch_fence_item(vehicle, bc_rx, sys, comp, idx).await {
            Some(point) => points.push(point),
            None => println!("Skipping missing fence item {idx}"),
        }
    }

    if let Err(e) = send_mission_ack(vehicle, sys, comp).await {
        println!("Warning: failed to send MISSION_ACK: {e}");
    }

    Ok(points)
}

fn build_geofence_payload(points: &[(f32, f32)]) -> Vec<u8> {
    let mut builder = FlatBufferBuilder::new();
    let coords: Vec<Coords> = points
        .iter()
        .map(|(lat, lng)| Coords::new(*lat, *lng))
        .collect();
    let points_offset = builder.create_vector(&coords);
    let packet = GeofencePacket::create(
        &mut builder,
        &GeofencePacketArgs {
            points: Some(points_offset),
        },
    );
    builder.finish(packet, None);
    builder.finished_data().to_vec()
}

fn build_empty_geofence_payload() -> Vec<u8> {
    let mut builder = FlatBufferBuilder::new();
    let packet = GeofencePacket::create(&mut builder, &GeofencePacketArgs { points: None });
    builder.finish(packet, None);
    builder.finished_data().to_vec()
}

pub async fn create_geofence_queryable(
    session: &Session,
    vehicle: SharedAsyncMavConn,
    bc_tx: Sender<(MavHeader, MavMessage)>,
) -> Option<Queryable<()>> {
    session
        .declare_queryable("control/query/geofence")
        .callback(move |query| {
            let vehicle = vehicle.clone();
            let mut bc_rx = bc_tx.subscribe();

            tokio::spawn(async move {
                let payload = match get_geofence(&vehicle, &mut bc_rx).await {
                    Ok(points) if !points.is_empty() => {
                        println!("GEO GOOD! ({} points)", points.len());
                        build_geofence_payload(&points)
                    }
                    _ => {
                        println!("GEO BAD!");
                        build_empty_geofence_payload()
                    }
                };

                let _ = query
                    .reply(
                        "control/query/geofence",
                        zenoh::bytes::ZBytes::from(payload),
                    )
                    .await;
            });
        })
        .await
        .ok()
}
