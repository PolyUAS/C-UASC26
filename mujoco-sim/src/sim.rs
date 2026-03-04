// Wraps MuJoCo model and data lifecycle.
//
// STUB: replace mj_step call with actual mujoco-sys unsafe calls when
// mujoco-sys is wired. The public API (load, step, state, apply_throttle,
// apply_goto) must not change.
//
// TODO: integrate mujoco-sys (or raw bindgen) for real MuJoCo 3.x physics.
// The current implementation uses simple kinematic integration for bring-up.

/// Simulated drone state using kinematic integration.
///
/// Coordinate frame: local NED from a geodetic origin (z-down).
pub struct DroneSim {
    /// Position: [north_m, east_m, down_m] (z negative = above ground)
    pub pos: [f64; 3],
    /// Velocity: [vn_mps, ve_mps, vd_mps]
    pub vel: [f64; 3],
    /// Battery percentage (0.0 – 100.0)
    pub battery: f64,
    /// Current flight mode string (e.g. "GUIDED", "LOITER", "RTL")
    pub mode: String,
    /// Geodetic origin latitude (degrees)
    origin_lat: f64,
    /// Geodetic origin longitude (degrees)
    origin_lon: f64,
}

impl DroneSim {
    /// Load a MuJoCo model (currently a no-op stub).
    pub fn load(_model_path: &str) -> Self {
        // TODO: mujoco_sys::MjModel::load_xml(model_path)
        Self {
            pos: [0.0, 0.0, -10.0], // 10 m AGL (NED: z negative = up)
            vel: [0.0; 3],
            battery: 100.0,
            mode: "GUIDED".to_string(),
            // Cal Poly SLO airfield approximate origin
            origin_lat: 35.3272,
            origin_lon: -120.7821,
        }
    }

    /// Advance physics one timestep (50 Hz ⇒ dt = 0.02 s).
    pub fn step(&mut self) {
        // TODO: mj_step(self.model, self.data)
        // Stub: integrate position from velocity at 50 Hz dt.
        let dt = 0.02_f64;
        self.pos[0] += self.vel[0] * dt;
        self.pos[1] += self.vel[1] * dt;
        self.pos[2] += self.vel[2] * dt;
        // Drain battery slowly: 0.001% per step ≈ 28 minutes flight.
        self.battery = (self.battery - 0.001).max(0.0);
    }

    /// Snapshot current state as a [`super::publisher::SimState`].
    pub fn state(&self) -> super::publisher::SimState {
        // Convert local NED metres to geodetic degrees.
        let lat = self.origin_lat + self.pos[0] / 111_111.0;
        let lon = self.origin_lon
            + self.pos[1] / (111_111.0 * self.origin_lat.to_radians().cos());
        let alt = -self.pos[2]; // NED z-down → altitude positive-up

        super::publisher::SimState {
            lat_deg: lat,
            lon_deg: lon,
            alt_m: alt,
            vn_mps: self.vel[0],
            ve_mps: self.vel[1],
            vd_mps: self.vel[2],
            battery_pct: self.battery,
            flight_mode: self.mode.clone(),
        }
    }

    /// Apply a normalised throttle (0.0–1.0) to control climb rate.
    #[allow(dead_code)]
    pub fn apply_throttle(&mut self, throttle_normalized: f64) {
        // throttle 0.0–1.0 maps to climb rate [-5.0, +5.0] m/s (NED: negative = up)
        self.vel[2] = -(throttle_normalized - 0.5) * 10.0;
    }

    /// Fly toward a geodetic waypoint using naïve proportional guidance.
    pub fn apply_goto(&mut self, lat_deg: f64, lon_deg: f64, alt_m: f64) {
        let north_offset = (lat_deg - self.origin_lat) * 111_111.0 - self.pos[0];
        let east_offset = (lon_deg - self.origin_lon)
            * (111_111.0 * self.origin_lat.to_radians().cos())
            - self.pos[1];
        let down_offset = -alt_m - self.pos[2]; // NED z-down
        let dist = (north_offset.powi(2) + east_offset.powi(2) + down_offset.powi(2)).sqrt();
        let speed = 5.0_f64; // m/s cruise
        if dist > 0.5 {
            self.vel[0] = speed * north_offset / dist;
            self.vel[1] = speed * east_offset / dist;
            self.vel[2] = speed * down_offset / dist;
        } else {
            self.vel = [0.0; 3]; // arrived — hold position
        }
    }

    /// Getter for the geodetic origin latitude.
    pub fn origin_lat(&self) -> f64 {
        self.origin_lat
    }

    /// Getter for the geodetic origin longitude.
    pub fn origin_lon(&self) -> f64 {
        self.origin_lon
    }
}
