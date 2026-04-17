use crate::{SharedAsyncMavConn, generated::squidarm_generated::control_squid::root_as_squidarm};

use mavlink::ardupilotmega::{COMMAND_LONG_DATA, MavCmd, MavMessage};
use zenoh::pubsub::Subscriber;
// WARN: placeholder Values for actual pins
const X_PIN: f32 = 0f32;
const Y_PIN: f32 = 0f32;

// NOTE:  remove annotation once this function has been called.
#[allow(unused, dead_code)]
///
/// # Aguments
/// - `conn` A Connection to the flight controller over which servo package will be sent.
/// - `session` a refrence to the universal zenoh session over which a the following
///   subscriber will be declared "control/squidarm/"
/// # Safety
/// `conn` is a asummed to be a cloned value, Therefore only pass in a cloned value to `conn`
/// into this function.
///
pub async fn init_squidarm(conn: SharedAsyncMavConn, session: &zenoh::Session) -> Subscriber<()> {
    session
        .declare_subscriber("control/squidarm/")
        .callback(move |sample| {
            let data = sample.payload().to_bytes();
            let result_squidarm = root_as_squidarm(&data);

            // return early if there is no squidarm
            // justification: to Avoid deeply nested code
            //  if an operation on failure should happen, Convert to a match statement,.
            let Ok(squidarm) = result_squidarm else {
                return;
            };
            // proceed as normal
            let x = squidarm.x(); /* in microseconds */
            let y = squidarm.y(); /* in microseconds */

            /* please refer to https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO */

            // Sends cmd to flight controller for x-axis.
            conn.send_default(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                param1: X_PIN,
                param2: x as f32,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
                command: MavCmd::MAV_CMD_DO_SET_SERVO,
                ..Default::default()
            }));

            // Sends cmd to flight controller for y-axis.
            conn.send_default(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                param1: Y_PIN,
                param2: y as f32,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
                command: MavCmd::MAV_CMD_DO_SET_SERVO,
                ..Default::default()
            }));
        })
        .await
        .expect("Squidarm should be avaliable.")
}
