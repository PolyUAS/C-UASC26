use crate::{SharedAsyncMavConn, generated::squidarm_generated::control_squid::root_as_squidarm};

use mavlink::ardupilotmega::{COMMAND_LONG_DATA, MavCmd, MavMessage};
use tokio::runtime::Runtime;
use zenoh::pubsub::Subscriber;

// WARN: placeholder Values for actual pins
const X_PIN: f32 = 0 as f32;
const Y_PIN: f32 = 1 as f32;

// TODO set these with actual specs
const SERVO_MIN_US: f32 = 500.0;
const SERVO_MAX_US: f32 = 2500.0;

// NOTE:  remove annotation once this function has been called.
#[allow(unused, dead_code)]
///
/// # Arguments
/// - `conn` A Connection to the flight controller over which servo package will be sent.
/// - `session` a refrence to the universal zenoh session over which a the following
///   subscriber will be declared "control/squidarm/"
/// # Safety
/// `conn` is a assumed to be a cloned value, Therefore only pass a cloned value to `conn`
/// into this function.
///
pub async fn init_squidarm(conn: SharedAsyncMavConn, session: &zenoh::Session) -> Subscriber<()> {
    let rt = Runtime::new().expect("failed to initialize tokio RT in sqarm");

    session
        .declare_subscriber("control/in/squidarm/")
        .callback(move |sample| {
            let conn = conn.clone(); // shallow shadow

            let data = sample.payload().to_bytes();
            let packet_res = root_as_squidarm(&data);

            // return early if there is no squidarm
            // justification: to Avoid deeply nested code
            //  if an operation on failure should happen, Convert to a match statement,.
            let Ok(sq_packet) = packet_res else {
                return;
            };
            // proceed as normal

            let x = sq_packet.x().clamp(-1., 1.) as f32; // [-1.0 to 1.0]
            let y = sq_packet.y().clamp(-1., 1.) as f32; // [-1.0 to 1.0]

            let x_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (x + 1.0) / 2.0;
            let y_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (y + 1.0) / 2.0;

            /* please refer to https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO */

            tokio::spawn(async move {
                // Sends cmd to flight controller for x-axis.
                if let Err(e) = conn
                    .send_default(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                        param1: X_PIN,
                        param2: x_us,
                        command: MavCmd::MAV_CMD_DO_SET_SERVO,
                        ..Default::default()
                    }))
                    .await
                {
                    eprintln!("[squidarm] X send failed: {e}");
                }
                // Sends cmd to flight controller for y-axis.
                if let Err(e) = conn
                    .send_default(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                        param1: Y_PIN,
                        param2: y_us,
                        command: MavCmd::MAV_CMD_DO_SET_SERVO,
                        ..Default::default()
                    }))
                    .await
                {
                    eprintln!("[squidarm] Y send failed: {e}");
                }
            });
        })
        .await
        .expect("failed to initialize squidarm subscriber")
}
