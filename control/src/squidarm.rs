use crate::{SharedAsyncMavConn, generated::squidarm_generated::control_squid::root_as_squidarm};

use mavlink::ardupilotmega::{COMMAND_LONG_DATA, MavCmd, MavMessage};
use tokio::runtime::Runtime;
use zenoh::pubsub::Subscriber;

// WARN: placeholder Values for actual pins
const X_PIN: f32 = 0 as f32;
const Y_PIN: f32 = 1 as f32;

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
        .declare_subscriber("control/squidarm/")
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

            let x = sq_packet.x() as f32; /* in microseconds */
            let y = sq_packet.y() as f32; /* in microseconds */

            /* please refer to https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO */

            tokio::spawn(async move {
                // Sends cmd to flight controller for x-axis.
                if let Err(e) = conn
                    .send_default(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                        param1: X_PIN,
                        param2: x,
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
                        param2: y,
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
