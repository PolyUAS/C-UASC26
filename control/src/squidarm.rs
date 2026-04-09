use crate::{SharedAsyncMavConn, generated::squidarm_generated::control_squid::root_as_squidarm};

pub async fn _init_squidarm_subscriber(conn: SharedAsyncMavConn, session: &zenoh::Session) -> () {
    let _ = session
        .declare_subscriber("control/squidarm/")
        .callback(move |sample| {
            let data = sample.payload().to_bytes();
            let squidarm = root_as_squidarm(data.as_ref()).ok();

        });
}
