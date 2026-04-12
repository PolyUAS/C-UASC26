use std::f64::{self};

use nalgebra::{UnitQuaternion, Vector3};

use std::sync::OnceLock;
static CAM_TO_BODY: OnceLock<nalgebra::UnitQuaternion<f64>> = OnceLock::new();

fn cam_to_body() -> &'static nalgebra::UnitQuaternion<f64> {
    CAM_TO_BODY.get_or_init(|| {
        // nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0) // unrotated detection frame
        nalgebra::UnitQuaternion::from_euler_angles(
            0.0,                         // upside-down
            std::f64::consts::FRAC_PI_2, // pitched to be parallel to x-unit-vec (straight out from drone pointing in xy plane)
            0.0,
        ) * nalgebra::UnitQuaternion::from_euler_angles(std::f64::consts::PI, 0.0, 0.0) // rotated detection frame
    })
}

use crate::generated::{
    attitude_generated::attitude::AttitudePacket, detections_generated::Detection,
    gimbal_generated::gimbal::State, position_generated::position::GlobalPositionInt,
};

pub struct GeoprojectionCamParams {
    hfov_deg: f32,
    #[allow(unused)]
    vfov_deg: f32,
}

impl GeoprojectionCamParams {
    pub const fn new(hfov_deg: f32, vfov_deg: f32) -> Self {
        GeoprojectionCamParams { hfov_deg, vfov_deg }
    }
}

pub fn map_detection(
    attitude: &AttitudePacket,
    position: &GlobalPositionInt,
    det: &Detection,
    gimbal_att: &State,
    body_to_gimbal_mm: [i32; 3],
    frame_w: u16,
    frame_h: u16,
    geoproj_params: &GeoprojectionCamParams,
) -> Option<[i32; 2]> {
    let (body_q, cam_q) = pull_quaternions(&attitude, &gimbal_att);

    let px = det.ctr_x(); // pixel x, 0 = left
    let py = det.ctr_y(); // pixel y, 0 = top

    let f_px = (frame_w as f64 / 2.0) / (geoproj_params.hfov_deg as f64 / 2.0).to_radians().tan(); // get focal vector

    // Normalised offsets from principal point (centre of image)
    let dx = px as f64 - frame_w as f64 / 2.0; // + = right
    let dy = py as f64 - frame_h as f64 / 2.0; // + = down

    // Ray in camera frame: (right, down, forward)
    let ray_cam = nalgebra::Vector3::new(dx, dy, f_px).normalize();

    let drone_alt_mm = position.alt_above_home() as f64;

    if drone_alt_mm <= 0.0 {
        return None;
    }

    let drone_lat = position.lat() as f64 * 1e-7;
    let drone_lon = position.lon() as f64 * 1e-7;

    let r_cam_to_body = cam_to_body(); // TODO adjust to your mount
    let ray_world = body_q * cam_q * r_cam_to_body * ray_cam;

    // above-horizon handling
    if ray_world.z >= -1e-6 {
        return None;
    }

    // Offset ray origin by gimbal mount position (in body frame, rotated to NED)
    let gimbal_offset_vec =
        body_q * nalgebra::Vector3::from(body_to_gimbal_mm.map(|x| x as f64 * 1e-3));

    let t = (drone_alt_mm / 1000.0 + gimbal_offset_vec.z) / (-ray_world.z);

    let delta_north = gimbal_offset_vec.x + t * ray_world.x;
    let delta_east = gimbal_offset_vec.y + t * ray_world.y;

    // Convert NED offsets to lat/lon deltas (flat Earth approximation)
    const EARTH_RADIUS_METERS: f64 = 6_371_000.0_f64; // meters
    let lat_rad = (drone_lat).to_radians();

    let delta_lat = (delta_north / EARTH_RADIUS_METERS).to_degrees();
    let delta_lon = (delta_east / (EARTH_RADIUS_METERS * lat_rad.cos())).to_degrees();

    Some([
        ((drone_lat + delta_lat) * 1e7) as i32,
        ((drone_lon + delta_lon) * 1e7) as i32,
    ])
}

fn pull_quaternions(
    att: &AttitudePacket,
    gim: &State,
) -> (UnitQuaternion<f64>, UnitQuaternion<f64>) {
    let body_roll_rads = (att.roll() as f64).to_radians(); // y
    let body_pitch_rads = (att.pitch() as f64).to_radians(); // b
    let body_yaw_rads = (att.yaw() as f64).to_radians(); // a

    let body_q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), body_yaw_rads)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), body_pitch_rads)
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), body_roll_rads);

    let (gimbal_roll_rads, gimbal_pitch_rads, gimbal_yaw_rads) = (
        (gim.roll() as f64).to_radians(),
        (gim.pitch() as f64).to_radians(),
        (gim.yaw() as f64).to_radians(),
    );
    let cam_q = nalgebra::UnitQuaternion::from_euler_angles(
        gimbal_roll_rads,
        gimbal_pitch_rads,
        gimbal_yaw_rads,
    );

    (body_q, cam_q)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::generated::attitude_generated::attitude::{
        AttitudePacket, AttitudePacketArgs, root_as_attitude_packet,
    };
    use crate::generated::detections_generated::{Detection, DetectionArgs, Frame, FrameArgs};
    use crate::generated::gimbal_generated::gimbal::{State, StateArgs, root_as_state};
    use crate::generated::position_generated::position::{
        GlobalPositionInt, GlobalPositionIntArgs, root_as_global_position_int,
    };
    use crate::root_as_frame;
    use flatbuffers::FlatBufferBuilder;

    // ─── helpers ────────────────────────────────────────────────────────────────

    fn make_attitude(roll: f32, pitch: f32, yaw: f32) -> Vec<u8> {
        let mut fbb = FlatBufferBuilder::new();
        let pkt = AttitudePacket::create(
            &mut fbb,
            &AttitudePacketArgs {
                roll,
                pitch,
                yaw,
                vroll: 0.0,
                vpitch: 0.0,
                vyaw: 0.0,
            },
        );
        fbb.finish(pkt, None);
        fbb.finished_data().to_vec()
    }

    fn make_gimbal(roll: f32, pitch: f32, yaw: f32) -> Vec<u8> {
        let mut fbb = FlatBufferBuilder::new();
        let pkt = State::create(
            &mut fbb,
            &StateArgs {
                roll,
                pitch,
                yaw,
                zoom: 1.0,
            },
        );
        fbb.finish(pkt, None);
        fbb.finished_data().to_vec()
    }

    fn make_position(lat_deg: f64, lon_deg: f64, alt_mm: i32) -> Vec<u8> {
        let mut fbb = FlatBufferBuilder::new();
        let pkt = GlobalPositionInt::create(
            &mut fbb,
            &GlobalPositionIntArgs {
                lat: (lat_deg * 1e7) as i32,
                lon: (lon_deg * 1e7) as i32,
                alt: alt_mm,
                vx: 0,
                vy: 0,
                vz: 0,
                heading: 0,
                alt_above_home: alt_mm,
            },
        );
        fbb.finish(pkt, None);
        fbb.finished_data().to_vec()
    }

    fn make_frame_detection(frame_w: u16, frame_h: u16, ctr_x: u16, ctr_y: u16) -> Vec<u8> {
        let mut fbb = FlatBufferBuilder::new();
        let det = Detection::create(
            &mut fbb,
            &DetectionArgs {
                ctr_x,
                ctr_y,
                width: 10,
                height: 10,
                class_id: None,
                metadata: None,
                confidence: 0.99,
            },
        );
        let dets = fbb.create_vector(&[det]);
        let frame = Frame::create(
            &mut fbb,
            &FrameArgs {
                width: frame_w,
                height: frame_h,
                detections: Some(dets),
                metadata: None,
            },
        );
        fbb.finish(frame, None);
        fbb.finished_data().to_vec()
    }

    fn default_params() -> GeoprojectionCamParams {
        GeoprojectionCamParams::new(78.0, 52.0)
    }

    // ─── tests ───────────────────────────────────────────────────────────────────

    /// Straight down, center pixel → result must equal drone position.
    #[test]
    fn test_straight_down_center_pixel() {
        // === Setup Test Data ===

        // 1. Drone attitude: perfectly level (roll=0, pitch=0), arbitrary yaw
        let mut fbb = FlatBufferBuilder::new();
        let attitude_packet = AttitudePacket::create(
            &mut fbb,
            &AttitudePacketArgs {
                roll: 0.0,
                pitch: 0.0,
                yaw: 45.0,
                vroll: 0.0,
                vpitch: 0.0,
                vyaw: 0.0,
            },
        );
        fbb.finish(attitude_packet, None);
        let buf = fbb.finished_data().to_vec();
        let attitude = root_as_attitude_packet(&buf).expect("failed to deserialize attitude");

        // 2. Gimbal attitude: looking straight down (pitch = -90°)
        fbb.reset();
        let gimbal_packet = State::create(
            &mut fbb,
            &StateArgs {
                roll: 0.0,
                pitch: -90.0,
                yaw: 0.0,
                zoom: 1.0,
            },
        );
        fbb.finish(gimbal_packet, None);
        let buf = fbb.finished_data().to_vec();
        let gimbal_state = root_as_state(&buf).expect("failed to deserialize gimbal state");

        // 3. Drone position
        let drone_lat = 37.7749_f64;
        let drone_lon = -122.4194_f64;
        let drone_alt_mm: i32 = 10_000; // 10 meters above home

        println!("input lat: {}", (drone_lat * 1e7) as i32);
        println!("input lon: {}", (drone_lon * 1e7) as i32);

        fbb.reset();
        let position_packet = GlobalPositionInt::create(
            &mut fbb,
            &GlobalPositionIntArgs {
                lat: (drone_lat * 1e7) as i32,
                lon: (drone_lon * 1e7) as i32,
                alt: drone_alt_mm,
                vx: 0,
                vy: 0,
                vz: 0,
                heading: 0,
                alt_above_home: drone_alt_mm,
            },
        );
        fbb.finish(position_packet, None);
        let buf = fbb.finished_data().to_vec();
        let position = root_as_global_position_int(&buf).expect("failed to deserialize position");

        // 4. Detection: exactly center of frame
        let frame_w: u16 = 1920;
        let frame_h: u16 = 1080;
        fbb.reset();
        // Detection is a nested table, not a root — read it back via Frame instead
        fbb.reset();
        let det_offset = Detection::create(
            &mut fbb,
            &DetectionArgs {
                ctr_x: frame_w / 2,
                ctr_y: frame_h / 2,
                width: 10,
                height: 10,
                class_id: None,
                metadata: None,
                confidence: 0.99,
            },
        );
        // Build the detections vector, then the Frame root
        let detections_vec = fbb.create_vector(&[det_offset]);
        let frame_offset = Frame::create(
            &mut fbb,
            &FrameArgs {
                width: frame_w,
                height: frame_h,
                detections: Some(detections_vec),
                metadata: None,
            },
        );

        fbb.finish(frame_offset, None);
        let buf = fbb.finished_data().to_vec();
        let frame = crate::generated::detections_generated::root_as_frame(&buf)
            .expect("failed to deserialize frame");
        let det = frame.detections().expect("frame has no detections").get(0);

        // 5. Gimbal mount offset (camera is 15cm below IMU)
        let body_to_gimbal_mm: [i32; 3] = [0, 0, -150];

        // 6. Camera parameters
        let geoproj_params = GeoprojectionCamParams::new(78.0, 52.0);

        // === Execute ===
        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            body_to_gimbal_mm,
            frame_w,
            frame_h,
            &geoproj_params,
        );

        // === Assert ===
        let expected_lat = (drone_lat * 1e7) as i32;
        let expected_lon = (drone_lon * 1e7) as i32;
        match result {
            Some([lat, lon]) => {
                assert!(
                    (lat - expected_lat).abs() <= 1,
                    "Latitude should match drone position : lat = {lat}, expected = {expected_lat} "
                );
                assert!(
                    (lon - expected_lon).abs() <= 1,
                    "Longitude should match drone position : lon = {lon}, expected = {expected_lon}"
                );
            }
            None => panic!("Straight-down center detection should not return None"),
        }
    }

    // ── Location diversity ─────────────────────────────────────────────────────
    /// Equator crossing — straight down, center pixel must match drone position.
    #[test]
    fn test_equator_straight_down() {
        let drone_lat = 0.0_f64;
        let drone_lon = 36.8219_f64; // Nairobi longitude

        let att_buf = make_attitude(0.0, 0.0, 0.0);
        let gim_buf = make_gimbal(0.0, -90.0, 0.0);
        let pos_buf = make_position(drone_lat, drone_lon, 20_000);
        let frm_buf = make_frame_detection(1920, 1080, 960, 540);

        let attitude = root_as_attitude_packet(&att_buf).unwrap();
        let gimbal_state = root_as_state(&gim_buf).unwrap();
        let position = root_as_global_position_int(&pos_buf).unwrap();
        let frame = root_as_frame(&frm_buf).unwrap();
        let det = frame.detections().unwrap().get(0);

        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            [0, 0, -150],
            1920,
            1080,
            &default_params(),
        )
        .expect("equator straight-down should project");

        let expected_lat = (drone_lat * 1e7) as i32;
        let expected_lon = (drone_lon * 1e7) as i32;
        assert!(
            (result[0] - expected_lat).abs() <= 1,
            "lat mismatch at equator"
        );
        assert!(
            (result[1] - expected_lon).abs() <= 1,
            "lon mismatch at equator"
        );
    }

    /// Negative longitude (western hemisphere) — straight down, center pixel.
    #[test]
    fn test_sydney_straight_down() {
        let drone_lat = -33.8688_f64; // Sydney
        let drone_lon = 151.2093_f64;

        let att_buf = make_attitude(0.0, 0.0, 0.0);
        let gim_buf = make_gimbal(0.0, -90.0, 0.0);
        let pos_buf = make_position(drone_lat, drone_lon, 30_000);
        let frm_buf = make_frame_detection(1920, 1080, 960, 540);

        let attitude = root_as_attitude_packet(&att_buf).unwrap();
        let gimbal_state = root_as_state(&gim_buf).unwrap();
        let position = root_as_global_position_int(&pos_buf).unwrap();
        let frame = root_as_frame(&frm_buf).unwrap();
        let det = frame.detections().unwrap().get(0);

        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            [0, 0, -150],
            1920,
            1080,
            &default_params(),
        )
        .expect("Sydney straight-down should project");

        let expected_lat = (drone_lat * 1e7) as i32;
        let expected_lon = (drone_lon * 1e7) as i32;
        assert!(
            (result[0] - expected_lat).abs() <= 1,
            "lat mismatch at Sydney"
        );
        assert!(
            (result[1] - expected_lon).abs() <= 1,
            "lon mismatch at Sydney"
        );
    }

    #[test]
    fn test_horizon_none() {
        let drone_lat = -33.8688_f64; // Sydney
        let drone_lon = 151.2093_f64;

        let att_buf = make_attitude(0.0, 0.0, 0.0);
        let gim_buf = make_gimbal(0.0, 0.0, 0.0);
        let pos_buf = make_position(drone_lat, drone_lon, 30_000);
        let frm_buf = make_frame_detection(1920, 1080, 960, 540);

        let attitude = root_as_attitude_packet(&att_buf).unwrap();
        let gimbal_state = root_as_state(&gim_buf).unwrap();
        let position = root_as_global_position_int(&pos_buf).unwrap();
        let frame = root_as_frame(&frm_buf).unwrap();
        let det = frame.detections().unwrap().get(0);

        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            [0, 0, -150],
            1920,
            1080,
            &default_params(),
        );

        assert!(
            result == None,
            "horizontal camera should not map to a value."
        );
    }

    #[test]
    fn test_45p_gimbal_north() {
        let drone_lat = -33.8688_f64; // Sydney
        let drone_lon = 151.2093_f64;

        let att_buf = make_attitude(0.0, 0.0, 0.0);
        let gim_buf = make_gimbal(0.0, 45.0, 0.0);
        let pos_buf = make_position(drone_lat, drone_lon, 30_000);
        let frm_buf = make_frame_detection(1920, 1080, 960, 540);

        let attitude = root_as_attitude_packet(&att_buf).unwrap();
        let gimbal_state = root_as_state(&gim_buf).unwrap();
        let position = root_as_global_position_int(&pos_buf).unwrap();
        let frame = root_as_frame(&frm_buf).unwrap();
        let det = frame.detections().unwrap().get(0);

        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            [0, 0, -150],
            1920,
            1080,
            &default_params(),
        );

        let drone_lat_e7 = (drone_lat * 1e7) as i32;
        let drone_lon_e7 = (drone_lon * 1e7) as i32;

        if let Some(result) = result {
            let lat = result[0];
            let lon = result[1];

            assert!(
                lat > drone_lat_e7,
                "result: lat: {lat} <= lat_e7 {drone_lat_e7}, but this should not be the case."
            );
            assert!(
                (lon - drone_lon_e7).abs() <= 1,
                "longitude moved: lon: {lon}, lon_e7: {drone_lon_e7}"
            );
        }
    }

    #[test]
    fn test_45p_90y_gimbal_east() {
        let drone_lat = -33.8688_f64; // Sydney
        let drone_lon = 151.2093_f64;

        let att_buf = make_attitude(0.0, 0.0, 0.0);
        let gim_buf = make_gimbal(0.0, 45.0, 90.0);
        let pos_buf = make_position(drone_lat, drone_lon, 30_000);
        let frm_buf = make_frame_detection(1920, 1080, 960, 540);

        let attitude = root_as_attitude_packet(&att_buf).unwrap();
        let gimbal_state = root_as_state(&gim_buf).unwrap();
        let position = root_as_global_position_int(&pos_buf).unwrap();
        let frame = root_as_frame(&frm_buf).unwrap();
        let det = frame.detections().unwrap().get(0);

        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            [0, 0, -150],
            1920,
            1080,
            &default_params(),
        );

        let drone_lat_e7 = (drone_lat * 1e7) as i32;
        let drone_lon_e7 = (drone_lon * 1e7) as i32;

        if let Some(result) = result {
            let lat = result[0];
            let lon = result[1];

            assert!(
                lon > drone_lon_e7,
                "result: lon: {lon} <= lon_e7 {drone_lon_e7}, but this should not be the case."
            );
            assert!(
                (lat - drone_lat_e7).abs() <= 1,
                "latitude moved: lat: {lat}, lat_e7: {drone_lat_e7}"
            );
        }
    }

    #[test]
    fn test_45p_basic_east() {
        let drone_lat = -33.8688_f64; // Sydney
        let drone_lon = 151.2093_f64;

        let att_buf = make_attitude(0.0, 45.0, 0.0);
        let gim_buf = make_gimbal(0.0, 0.0, 0.0);
        let pos_buf = make_position(drone_lat, drone_lon, 30_000);
        let frm_buf = make_frame_detection(1920, 1080, 960, 540);

        let attitude = root_as_attitude_packet(&att_buf).unwrap();
        let gimbal_state = root_as_state(&gim_buf).unwrap();
        let position = root_as_global_position_int(&pos_buf).unwrap();
        let frame = root_as_frame(&frm_buf).unwrap();
        let det = frame.detections().unwrap().get(0);

        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            [0, 0, -150],
            1920,
            1080,
            &default_params(),
        );

        let drone_lat_e7 = (drone_lat * 1e7) as i32;
        let drone_lon_e7 = (drone_lon * 1e7) as i32;

        if let Some(result) = result {
            let lat = result[0];
            let lon = result[1];

            assert!(
                lon > drone_lon_e7,
                "result: lon: {lon} <= lon_e7 {drone_lon_e7}, but this should not be the case."
            );
            assert!(
                (lat - drone_lat_e7).abs() <= 1,
                "latitude moved: lat: {lat}, lat_e7: {drone_lat_e7}"
            );
        }
    }

    #[test]
    fn test_45p_90y_basic_east() {
        let drone_lat = -33.8688_f64; // Sydney
        let drone_lon = 151.2093_f64;

        let att_buf = make_attitude(0.0, 45.0, 90.0);
        let gim_buf = make_gimbal(0.0, 0.0, 0.0);
        let pos_buf = make_position(drone_lat, drone_lon, 30_000);
        let frm_buf = make_frame_detection(1920, 1080, 960, 540);

        let attitude = root_as_attitude_packet(&att_buf).unwrap();
        let gimbal_state = root_as_state(&gim_buf).unwrap();
        let position = root_as_global_position_int(&pos_buf).unwrap();
        let frame = root_as_frame(&frm_buf).unwrap();
        let det = frame.detections().unwrap().get(0);

        let result = map_detection(
            &attitude,
            &position,
            &det,
            &gimbal_state,
            [0, 0, -150],
            1920,
            1080,
            &default_params(),
        );

        let drone_lat_e7 = (drone_lat * 1e7) as i32;
        let drone_lon_e7 = (drone_lon * 1e7) as i32;

        if let Some(result) = result {
            let lat = result[0];
            let lon = result[1];

            assert!(
                lon > drone_lon_e7,
                "result: lon: {lon} <= lon_e7 {drone_lon_e7}, but this should not be the case."
            );
            assert!(
                (lat - drone_lat_e7).abs() <= 1,
                "latitude moved: lat: {lat}, lat_e7: {drone_lat_e7}"
            );
        }
    }
}
