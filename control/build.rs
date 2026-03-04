//! Build script — generates Rust code from shared FlatBuffers schemas.
//!
//! Schema lookup order:
//! 1. `SCHEMAS_DIR` env var (set by Nix build or CI)
//! 2. `../schemas/` relative to `CARGO_MANIFEST_DIR` (dev-shell / workspace)
//!
//! `flatc` must be on PATH (provided by Nix dev shell or `nativeBuildInputs`).

use std::path::{Path, PathBuf};
use std::process::Command;

/// FlatBuffers schemas that control needs for its Zenoh topics.
const SCHEMAS: &[&str] = &[
    "vehicle_telemetry.fbs",
    "mission_command.fbs",
    "command_ack.fbs",
    "supervisor_signal.fbs",
    "gimbal_state.fbs",
];

fn find_schema_dir() -> PathBuf {
    // Prefer explicit env var (Nix build)
    if let Ok(dir) = std::env::var("SCHEMAS_DIR") {
        let p = PathBuf::from(dir);
        if p.exists() {
            return p;
        }
    }
    // Fallback: workspace-relative
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    let workspace_schemas = manifest.join("../schemas");
    if workspace_schemas.exists() {
        return workspace_schemas;
    }
    panic!(
        "Cannot find FlatBuffers schemas. Set SCHEMAS_DIR env var or ensure \
         ../schemas/ exists relative to {}",
        manifest.display()
    );
}

fn main() {
    let schema_dir = find_schema_dir();
    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR not set");

    for schema in SCHEMAS {
        let schema_path = schema_dir.join(schema);
        assert!(
            schema_path.exists(),
            "FlatBuffers schema not found: {}",
            schema_path.display()
        );

        let status = Command::new("flatc")
            .arg("--rust")
            .arg("-o")
            .arg(&out_dir)
            .arg(&schema_path)
            .status()
            .unwrap_or_else(|e| panic!("Failed to run flatc: {e}. Is flatc on PATH?"));

        assert!(
            status.success(),
            "flatc failed for schema {}",
            schema_path.display()
        );

        println!("cargo:rerun-if-changed={}", schema_path.display());
    }

    println!("cargo:rerun-if-changed=build.rs");
}
