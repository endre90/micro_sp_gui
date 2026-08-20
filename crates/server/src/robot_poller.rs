//! The fast read loop for the Robot tab.
//!
//! `ur_redis_driver` republishes measured state every 5 ms, but [`crate::poller`]
//! runs at `--poll-ms` (250 by default) because each of its ticks is a
//! whole-keyspace `SCAN`, an `MGET` of everything in it, and a full transform
//! read. Joint states and TCP pose arriving a quarter of a second late makes the
//! Robot tab feel detached from the robot, and lowering the global period pays
//! for that everywhere.
//!
//! So this second loop reads *only* the ~21 `{robot_id}_*` keys the tab shows -
//! one small `MGET`, no `SCAN`, no transforms - and owns `Snapshot::robots`
//! outright. The main poller deliberately no longer touches that field: two
//! loops writing it would leave the slower one overwriting the faster one with
//! stale values, and both would republish forever.
//!
//! Discovery stays in the main poller. This one only reads the robot ids it
//! already found.

use crate::status::{self, Entries};
use crate::{AppState, LOG_TARGET};
use micro_sp::*;
use micro_sp_gui_protocol as proto;
use std::collections::BTreeMap;
use std::sync::Arc;

/// One `MGET` for a known key list, parsed the same way the main poller parses
/// its own: a value that does not deserialise is kept with its raw text rather
/// than dropped.
async fn read_keys(con: &mut SPConnection, keys: &[String]) -> redis::RedisResult<Entries> {
    if keys.is_empty() {
        return Ok(Entries::new());
    }
    let raws: Vec<Option<String>> = redis::cmd("MGET").arg(keys).query_async(con).await?;

    let mut entries = Entries::new();
    for (key, raw) in keys.iter().zip(raws) {
        // A key the driver has never written is simply absent, which
        // `status::robot_status` already reports faithfully.
        let Some(raw) = raw else { continue };
        let value = serde_json::from_str::<proto::GuiValue>(&raw).ok();
        entries.insert(key.clone(), proto::StateEntry { key: key.clone(), value, raw });
    }
    Ok(entries)
}

/// Poll until the process ends.
pub async fn run(state: Arc<AppState>) {
    let mut con = state.cm.get_connection().await;
    let mut ticker = tokio::time::interval(std::time::Duration::from_millis(
        state.cfg.robot_poll_ms.max(10),
    ));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    // Only logged once per distinct failure: at this tick rate a broken Redis
    // would otherwise write twenty identical lines a second, and the main poller
    // is already the one that tells the browser about it.
    let mut last_error: Option<String> = None;

    loop {
        ticker.tick().await;

        let robot_ids = state.snapshot.read().await.info.robot_ids.clone();
        if robot_ids.is_empty() {
            continue;
        }

        let keys: Vec<String> =
            robot_ids.iter().flat_map(|id| status::robot_keys(id)).collect();

        let entries = match read_keys(&mut con, &keys).await {
            Ok(entries) => entries,
            Err(e) => {
                let msg = format!("Redis robot read failed: {e}");
                if last_error.as_deref() != Some(msg.as_str()) {
                    log::error!(target: LOG_TARGET, "{msg}");
                    last_error = Some(msg);
                }
                state.cm.handle_redis_error(&e, LOG_TARGET).await;
                continue;
            }
        };
        last_error = None;

        let robots: BTreeMap<String, proto::RobotStatus> = robot_ids
            .iter()
            .map(|id| (id.clone(), status::robot_status(&entries, id)))
            .collect();

        // Work out what to announce, then release the lock before publishing.
        let mut messages: Vec<proto::ServerMsg> = Vec::new();
        {
            let mut snap = state.snapshot.write().await;
            for (id, status) in &robots {
                if snap.robots.get(id) != Some(status) {
                    messages.push(proto::ServerMsg::RobotStatus(status.clone()));
                }
            }
            snap.robots = robots;
        }

        for msg in messages {
            state.publish(msg);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The whole point of the loop: one round trip per tick, whatever the
    /// keyspace looks like.
    #[test]
    fn one_mget_covers_every_robot() {
        let ids = ["r1".to_string(), "r2".to_string()];
        let keys: Vec<String> = ids.iter().flat_map(|id| status::robot_keys(id)).collect();
        assert_eq!(keys.len(), status::ROBOT_SUFFIXES.len() * 2);
        assert!(keys.contains(&"r2_joint_states".to_string()));
    }
}
