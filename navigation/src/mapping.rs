use std::f64;

use std::sync::OnceLock;
static CAM_TO_BODY: OnceLock<nalgebra::UnitQuaternion<f64>> = OnceLock::new();

fn cam_to_body() -> &'static nalgebra::UnitQuaternion<f64> {
    CAM_TO_BODY.get_or_init(|| {
        nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0)
        //nalgebra::UnitQuaternion::from_euler_angles(std::f64::consts::PI, 0.0, 0.0)
    })
}

use nalgebra::{Quaternion, UnitQuaternion};

use crate::generated::{
    attitude_generated::attitude::AttitudePacket, detections_generated::Detection,
    gimbal_generated::gimbal::State, position_generated::position::GlobalPositionInt,
};

pub struct GeoprojectionCamParams {
    hfov_deg: f32,
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

    let view_q = &body_q * &cam_q;

    let px = det.ctr_x(); // pixel x, 0 = left
    let py = det.ctr_y(); // pixel y, 0 = top

    let f_px = (frame_w as f64 / 2.0) / (geoproj_params.hfov_deg as f64 / 2.0).to_radians().tan();

    // Normalised offsets from principal point (centre of image)
    let dx = px as f64 - frame_w as f64 / 2.0; // + = right
    let dy = py as f64 - frame_h as f64 / 2.0; // + = down

    // Ray in camera frame: (right, down, forward)
    let ray_cam = nalgebra::Vector3::new(dx, dy, f_px).normalize();

    let drone_alt_mm = position.alt_above_home() as f64;

    let drone_lat = position.lat() as f64 * 1e-7;
    let drone_lon = position.lon() as f64 * 1e-7;

    let r_cam_to_body = cam_to_body(); // TODO adjust to your mount
    let ray_world = view_q * r_cam_to_body * ray_cam;

    if ray_world.z <= 0.0 {
        return None;
    }

    // Offset ray origin by gimbal mount position (in body frame, rotated to NED)
    let gimbal_offset_vec =
        body_q * nalgebra::Vector3::from(body_to_gimbal_mm.map(|x| x as f64 * 1e-3));

    let t = (drone_alt_mm / 1000.0 + gimbal_offset_vec.z) / (-ray_world.z);
    let delta_north = t * ray_world.x - gimbal_offset_vec.x;
    let delta_east = t * ray_world.y - gimbal_offset_vec.y;

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

    let body_q = nalgebra::UnitQuaternion::from_euler_angles(
        body_roll_rads as f64,
        body_pitch_rads as f64,
        body_yaw_rads as f64,
    );

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
