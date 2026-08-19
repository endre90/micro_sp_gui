//! The push feed.
//!
//! One socket per browser. Each gets a full snapshot on connect and then only
//! deltas, all off the shared broadcast channel - the poller does not know or
//! care how many are listening.

use crate::{AppState, LOG_TARGET};
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::response::IntoResponse;
use micro_sp_gui_protocol as proto;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

pub async fn handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| run(socket, state))
}

fn encode(msg: &proto::ServerMsg) -> Option<Message> {
    match serde_json::to_string(msg) {
        Ok(text) => Some(Message::Text(text.into())),
        Err(e) => {
            log::error!(target: LOG_TARGET, "Could not encode a server message: {e}");
            None
        }
    }
}

async fn send_full_snapshot(socket: &mut WebSocket, state: &Arc<AppState>) -> bool {
    let (hello, entries, transforms, robots, goals) = {
        let snap = state.snapshot.read().await;
        (
            proto::ServerMsg::Hello(snap.info.clone()),
            proto::ServerMsg::State(snap.as_state_snapshot()),
            proto::ServerMsg::Transforms(snap.transforms.clone()),
            snap.robots.values().cloned().collect::<Vec<_>>(),
            snap.goals.values().cloned().collect::<Vec<_>>(),
        )
    };

    let mut batch = vec![hello, entries, transforms];
    batch.extend(robots.into_iter().map(proto::ServerMsg::RobotStatus));
    batch.extend(goals.into_iter().map(proto::ServerMsg::Goals));

    for msg in batch {
        if let Some(frame) = encode(&msg)
            && socket.send(frame).await.is_err()
        {
            return false;
        }
    }
    true
}

async fn run(mut socket: WebSocket, state: Arc<AppState>) {
    // Subscribe before sending the snapshot, so nothing that happens while it is
    // being written is missed.
    let mut rx = state.bus.subscribe();

    if !send_full_snapshot(&mut socket, &state).await {
        return;
    }

    // A default backfill so the Logs tab has history immediately; the frontend
    // narrows it with its own filters.
    let backfill = state.query_logs(&proto::LogQuery::default()).await;
    if !backfill.is_empty()
        && let Some(frame) = encode(&proto::ServerMsg::Logs(backfill))
        && socket.send(frame).await.is_err()
    {
        return;
    }

    log::debug!(target: LOG_TARGET, "A client connected.");

    loop {
        tokio::select! {
            broadcast = rx.recv() => match broadcast {
                Ok(msg) => {
                    let Some(frame) = encode(&msg) else { continue };
                    if socket.send(frame).await.is_err() {
                        break;
                    }
                }
                // This client fell behind and lost messages. Deltas are only
                // meaningful in sequence, so resynchronise rather than carry on
                // with a state that is now missing keys.
                Err(RecvError::Lagged(n)) => {
                    log::warn!(target: LOG_TARGET, "A client lagged {n} message(s); resending the snapshot.");
                    if !send_full_snapshot(&mut socket, &state).await {
                        break;
                    }
                }
                Err(RecvError::Closed) => break,
            },

            incoming = socket.recv() => match incoming {
                Some(Ok(Message::Text(text))) => {
                    match serde_json::from_str::<proto::ClientMsg>(&text) {
                        Ok(proto::ClientMsg::Subscribe { log_query, .. }) => {
                            // The only thing a subscription changes server-side
                            // is which history to send; live state is broadcast
                            // to everyone and filtered in the browser.
                            let lines = state.query_logs(&log_query).await;
                            if let Some(frame) = encode(&proto::ServerMsg::Logs(lines))
                                && socket.send(frame).await.is_err()
                            {
                                break;
                            }
                        }
                        Ok(proto::ClientMsg::Ping) => {}
                        Err(e) => {
                            log::warn!(target: LOG_TARGET, "Ignoring an unreadable client message: {e}");
                        }
                    }
                }
                Some(Ok(Message::Close(_))) | None => break,
                Some(Ok(_)) => {}
                Some(Err(e)) => {
                    log::debug!(target: LOG_TARGET, "Client socket error: {e}");
                    break;
                }
            },
        }
    }

    log::debug!(target: LOG_TARGET, "A client disconnected.");
}
