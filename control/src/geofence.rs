use crate::SharedAsyncMavConn;
use crate::fc_header_utils;
use flatbuffers::FlatBufferBuilder;

use mavlink::MavHeader;
use mavlink::ardupilotmega::FENCE_FETCH_POINT_DATA;
use mavlink::ardupilotmega::MISSION_ITEM_INT_DATA;
use mavlink::ardupilotmega::MavMissionType;
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
) -> Result<u16, Box<dyn Error + Send + Sync>> {
    // 1. Tell the FC we want to download the fence items
    vehicle
        .send_default(&MavMessage::MISSION_REQUEST_LIST(
            MISSION_REQUEST_LIST_DATA {
                target_system: sys,
                target_component: comp,
                mission_type: MavMissionType::MAV_MISSION_TYPE_FENCE,
            },
        ))
        .await?;

    let deadline = std::time::Instant::now();
    while deadline.elapsed() < COUNT_TIMEOUT {
        tokio::select! {
            Ok((_, msg)) = bc_rx.recv() => {
                // 2. Expect a MISSION_COUNT response
                if let MavMessage::MISSION_COUNT(cnt) = msg {
                    if cnt.mission_type == MavMissionType::MAV_MISSION_TYPE_FENCE {
                        println!("FC reported {} fence points.", cnt.count);
                        return Ok(cnt.count);
                    }
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(50)) => {}
        }
    }

    Err("Timeout waiting for MISSION_COUNT".into())
}

async fn fetch_fence_item(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
    sys: u8,
    comp: u8,
    idx: u16,
) -> Option<(f32, f32)> {
    for attempt in 0..ITEM_MAX_RETRIES {
        println!("grabbing fence item: {idx}, attempt: {attempt}");

        if let Err(e) = vehicle
            .send_default(&MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
                target_system: sys,
                target_component: comp,
                seq: idx as u16,
                mission_type: mavlink::ardupilotmega::MavMissionType::MAV_MISSION_TYPE_FENCE,
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
                        Ok((_, MavMessage::MISSION_ITEM_INT(item))) => {
                            let lat = item.x as f32 / 1e7;
                            let lng = item.y as f32 / 1e7;
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
            mission_type: mavlink::ardupilotmega::MavMissionType::MAV_MISSION_TYPE_FENCE,
            opaque_id: 42,
            mavtype: MavMissionResult::MAV_MISSION_ACCEPTED,
        }))
        .await
}

async fn send_mission_error_ack(
    vehicle: &SharedAsyncMavConn,
    sys: u8,
    comp: u8,
    error_type: MavMissionResult,
) {
    let _ = vehicle
        .send_default(&MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: sys,
            target_component: comp,
            mission_type: mavlink::ardupilotmega::MavMissionType::MAV_MISSION_TYPE_FENCE,
            opaque_id: 42,
            mavtype: error_type,
        }))
        .await;
}

pub async fn get_geofence(
    vehicle: &SharedAsyncMavConn,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
) -> Result<Vec<(f32, f32)>, Box<dyn Error + Send + Sync>> {
    let (sys, comp) = fc_header_utils::wait_for_headers().await;

    // Use the proper sequence initialization
    let count = match fetch_fence_count(vehicle, bc_rx, sys, comp).await {
        Ok(c) => c,
        Err(e) => {
            println!("Failed to get fence count: {e}");
            return Ok(Vec::new());
        }
    };

    if count == 0 {
        // Even if count is 0, send an ACK to be safe and clear state
        let _ = send_mission_ack(vehicle, sys, comp).await;
        return Ok(Vec::new());
    }

    let mut points = Vec::with_capacity(count as usize);
    let mut success = true;

    for idx in 0..count {
        match fetch_fence_item(vehicle, bc_rx, sys, comp, idx).await {
            Some(point) => points.push(point),
            None => {
                println!("Critical: Failed to retrieve fence item {idx}!");
                success = false;
                break; // Break early so we don't hang endlessly
            }
        }
    }

    if success {
        if let Err(e) = send_mission_ack(vehicle, sys, comp).await {
            println!("Warning: failed to send MISSION_ACK: {e}");
        }
    } else {
        // Tell the FC we are abandoning the transfer so it unlocks its state machine
        println!("Sending MAV_MISSION_ERROR to abort stuck transaction");
        send_mission_error_ack(vehicle, sys, comp, MavMissionResult::MAV_MISSION_ERROR).await;
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

                if let Err(e) = query
                    .reply(
                        "control/query/geofence",
                        zenoh::bytes::ZBytes::from(payload),
                    )
                    .await
                {
                    println!("Failed to reply to Zenoh query: {e:?}");
                }
            });
        })
        .await
        .ok()
}
