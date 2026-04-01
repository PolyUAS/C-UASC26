use std::time::Duration;

static mut FC_SYS_ID: Option<u8> = None;
static mut FC_SYS_COMP: Option<u8> = None;

///----------------------------------
// This looks really, really scary, but I PROMISE know what im doing here.
// I'm basically just implementing a OnceLock here, but the stdlib is more
// cooercive towards a singleton, hench why this exists.
// -Isaac
///----------------------------------

pub fn get_headers() -> (Option<u8>, Option<u8>) {
    unsafe { (FC_SYS_ID, FC_SYS_COMP) }
}

pub fn set_headers_safely(sys_id: u8, comp_id: u8) {
    unsafe {
        if FC_SYS_ID == None {
            FC_SYS_ID = Some(sys_id);
        }
        if FC_SYS_COMP == None {
            FC_SYS_COMP = Some(comp_id);
        }
    }
}

pub async fn wait_for_headers() -> (u8, u8) {
    let mut sys = None;
    let mut comp = None;
    while sys == None || comp == None {
        (sys, comp) = get_headers();
        tokio::time::sleep(Duration::from_millis(500)).await
    }

    let sys = sys.expect("Fatal Err: fc sys id not assigned to");
    let comp = comp.expect("Fatal Err: fc sys id not assigned to");
    (sys, comp)
}
