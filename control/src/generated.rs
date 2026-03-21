//! Auto-generated FlatBuffers code — included at build time from `$OUT_DIR`.
//!
//! Do NOT edit these files. Modify the `.fbs` schemas in `schemas/` instead.
//! `cargo build` triggers `build.rs` which runs `flatc --rust`.
//!
//! Re-exports all generated namespaces so call-sites use:
//!   `use crate::generated::vehicle;`
//!   `use crate::generated::guidance;`
//!   `use crate::generated::gimbal;`

#[allow(unused_imports, dead_code, clippy::all, clippy::pedantic)]
mod vehicle_telemetry_generated {
    include!(concat!(env!("OUT_DIR"), "/vehicle_telemetry_generated.rs"));
}
pub use vehicle_telemetry_generated::vehicle;

#[allow(unused_imports, dead_code, clippy::all, clippy::pedantic)]
mod mission_command_generated {
    include!(concat!(env!("OUT_DIR"), "/mission_command_generated.rs"));
}
pub use mission_command_generated::guidance as guidance_cmd;

#[allow(unused_imports, dead_code, clippy::all, clippy::pedantic)]
mod command_ack_generated {
    include!(concat!(env!("OUT_DIR"), "/command_ack_generated.rs"));
}
pub use command_ack_generated::guidance as guidance_ack;

#[allow(unused_imports, dead_code, clippy::all, clippy::pedantic)]
mod supervisor_signal_generated {
    include!(concat!(env!("OUT_DIR"), "/supervisor_signal_generated.rs"));
}
#[allow(unused_imports)]
pub use supervisor_signal_generated::guidance as guidance_sup;

#[allow(unused_imports, dead_code, clippy::all, clippy::pedantic)]
mod gimbal_state_generated_fb {
    include!(concat!(env!("OUT_DIR"), "/gimbal_state_generated.rs"));
}
#[allow(unused_imports)]
pub use gimbal_state_generated_fb::gimbal as gimbal_fb;
