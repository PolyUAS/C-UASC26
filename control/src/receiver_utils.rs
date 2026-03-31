use mavlink::MavHeader;
use mavlink::ardupilotmega::{MavMessage, MavResult};

use std::error::Error;
use std::time::Duration;

use tokio::sync::broadcast::Receiver;
use tokio::time::Instant;
use tokio::time::timeout;

pub async fn recv_message_within<F>(
    dur: Duration,
    bc_rx: &mut Receiver<(MavHeader, MavMessage)>,
    matches: F,
) -> Result<MavMessage, Box<dyn Error + Send + Sync>>
where
    F: Fn(&MavMessage) -> bool,
{
    let deadline = Instant::now() + dur;

    loop {
        let now = Instant::now();
        if now >= deadline {
            return Err("timed out waiting for requested message".into());
        }

        let remaining = deadline - now;

        match timeout(remaining, bc_rx.recv()).await {
            Err(_) => return Err("timed out waiting for requested message".into()),
            Ok(Err(tokio::sync::broadcast::error::RecvError::Lagged(_))) => continue,
            Ok(Err(tokio::sync::broadcast::error::RecvError::Closed)) => {
                return Err("broadcast channel closed".into());
            }
            Ok(Ok((_, recv_msg))) => {
                if matches(&recv_msg) {
                    return Ok(recv_msg);
                }
            }
        }
    }
}

pub async fn wait_for_cmd_status(
    timeout: Duration,
    sub: &mut Receiver<(MavHeader, MavMessage)>,
    expected_cmd: mavlink::ardupilotmega::MavCmd,
) -> Result<MavResult, Box<dyn Error + Send + Sync>> {
    let mut owned_sub = sub.resubscribe();

    let m = recv_message_within(timeout, &mut owned_sub, |m| match m {
        MavMessage::COMMAND_ACK(data) => data.command == expected_cmd,
        _ => false,
    })
    .await?;

    match m {
        MavMessage::COMMAND_ACK(data) => Ok(data.result),
        _ => Err("received unexpected non-ack message".into()),
    }
}
