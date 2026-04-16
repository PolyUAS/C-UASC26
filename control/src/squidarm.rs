use crate::{SharedAsyncMavConn, generated::squidarm_generated::control_squid::root_as_squidarm};

pub async fn _init_squidarm_subscriber(conn: SharedAsyncMavConn, session: &zenoh::Session) -> () {
    let _ = session
        .declare_subscriber("control/squidarm/")
        .callback(move |sample| {
            let data = sample.payload().to_bytes();
            let result_squidarm = root_as_squidarm(data.as_ref());
            match result_squidarm.ok() {
                Some(squidarm) => {
                    let x = squidarm.x();
                    let y = squidarm.y();

                    let _ = do_something_with_x(x);
                    let _ = do_something_with_y(y);

                    conn.send_default()

                }
                _ => {}
            }
        });
}

fn do_something_with_x(x: i32) -> i32 {}
fn do_something_with_y(y: i32) -> i32 {}
