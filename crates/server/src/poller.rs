//! The single Redis read loop.
//!
//! Runs once per process and broadcasts what changed. Two deliberate choices:
//!
//! * **`SCAN`, not `KEYS`.** `StateManager::get_full_state` is `KEYS *` + `MGET`,
//!   and micro_sp's own notes (`running/runner_keys.rs`) explain that `KEYS` is
//!   O(total keyspace) and blocks the single-threaded server for its whole
//!   duration. A GUI polling several times a second must not do that, so this
//!   scans in bounded batches like `get_all_transforms` does.
//! * **Deltas, not snapshots.** A quiet system produces no websocket traffic at
//!   all; a busy one sends only the keys that moved.

use crate::status::{self, Entries};
use crate::{AppState, LOG_TARGET, convert, discovery};
use micro_sp::*;
use micro_sp_gui_protocol as proto;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::Arc;

/// How many keys to ask for per `SCAN` round trip.
const SCAN_COUNT: usize = 1000;

/// Every non-transform key in the database.
///
/// Transform keys are excluded here and fetched by the transform path instead,
/// so a big scene does not bloat the state table.
async fn scan_state_keys(con: &mut SPConnection) -> redis::RedisResult<Vec<String>> {
    let mut cursor: u64 = 0;
    let mut keys: Vec<String> = Vec::new();
    loop {
        let (next, batch): (u64, Vec<String>) = redis::cmd("SCAN")
            .arg(cursor)
            .arg("MATCH")
            .arg("*")
            .arg("COUNT")
            .arg(SCAN_COUNT)
            .query_async(con)
            .await?;
        keys.extend(batch.into_iter().filter(|k| !k.starts_with(TF_PREFIX)));
        cursor = next;
        if cursor == 0 {
            // SCAN can return the same key twice across a resize.
            keys.sort_unstable();
            keys.dedup();
            return Ok(keys);
        }
    }
}

/// One `MGET` for the whole key list, parsed into entries.
///
/// A value that does not deserialise is kept with `value: None` and its raw
/// text, so the operator can see and fix it. `micro_sp`'s own `build_state`
/// drops such keys silently, which later shows up as a baffling
/// missing-variable panic in a runner.
async fn read_entries(con: &mut SPConnection, keys: Vec<String>) -> redis::RedisResult<Entries> {
    if keys.is_empty() {
        return Ok(Entries::new());
    }
    let raws: Vec<Option<String>> = redis::cmd("MGET").arg(&keys).query_async(con).await?;

    let mut entries = Entries::new();
    for (key, raw) in keys.into_iter().zip(raws) {
        // A key that vanished between the SCAN and the MGET is simply gone.
        let Some(raw) = raw else { continue };
        let value = serde_json::from_str::<proto::GuiValue>(&raw).ok();
        entries.insert(key.clone(), proto::StateEntry { key, value, raw });
    }
    Ok(entries)
}

async fn read_transforms(con: &mut SPConnection) -> proto::TransformsSnapshot {
    let buffer: HashMap<String, SPTransformStamped> =
        match TransformsManager::get_all_transforms(con).await {
            Ok(tfs) => tfs,
            // An empty transform keyspace is reported as an error rather than an
            // empty map, so it must not be treated as a failure.
            Err(e) => {
                log::debug!(target: LOG_TARGET, "No transforms to read: {e}");
                HashMap::new()
            }
        };

    let mut frames: Vec<proto::GuiTransformStamped> =
        buffer.values().map(convert::stamped_to_proto).collect();
    frames.sort_by(|a, b| a.child_frame_id.cmp(&b.child_frame_id));

    proto::TransformsSnapshot {
        revision: 0,
        tree_text: if buffer.is_empty() {
            String::new()
        } else {
            update_tree_visualization_once(&buffer)
        },
        cyclic: !buffer.is_empty() && is_cyclic_all(&buffer),
        frames,
    }
}

/// What changed between two entry maps.
fn diff(old: &Entries, new: &Entries) -> (Vec<proto::StateEntry>, Vec<String>) {
    let changed: Vec<proto::StateEntry> = new
        .iter()
        .filter(|(key, entry)| old.get(*key).map(|prev| prev != *entry).unwrap_or(true))
        .map(|(_, entry)| entry.clone())
        .collect();
    let removed: Vec<String> =
        old.keys().filter(|key| !new.contains_key(*key)).cloned().collect();
    (changed, removed)
}

/// Poll until the process ends.
pub async fn run(state: Arc<AppState>) {
    let mut con = state.cm.get_connection().await;
    let mut ticker =
        tokio::time::interval(std::time::Duration::from_millis(state.cfg.poll_ms.max(20)));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    // Remembered across ticks so a transient Redis error does not look like
    // "every key was deleted".
    let mut last_error: Option<String> = None;

    loop {
        ticker.tick().await;

        let keys = match scan_state_keys(&mut con).await {
            Ok(keys) => keys,
            Err(e) => {
                let msg = format!("Redis scan failed: {e}");
                if last_error.as_deref() != Some(msg.as_str()) {
                    log::error!(target: LOG_TARGET, "{msg}");
                    state.publish(proto::ServerMsg::Error(msg.clone()));
                    last_error = Some(msg);
                }
                state.cm.handle_redis_error(&e, LOG_TARGET).await;
                continue;
            }
        };

        let entries = match read_entries(&mut con, keys).await {
            Ok(entries) => entries,
            Err(e) => {
                let msg = format!("Redis read failed: {e}");
                if last_error.as_deref() != Some(msg.as_str()) {
                    log::error!(target: LOG_TARGET, "{msg}");
                    state.publish(proto::ServerMsg::Error(msg.clone()));
                    last_error = Some(msg);
                }
                state.cm.handle_redis_error(&e, LOG_TARGET).await;
                continue;
            }
        };

        if last_error.take().is_some() {
            log::info!(target: LOG_TARGET, "Redis is answering again.");
        }

        let transforms = read_transforms(&mut con).await;

        let key_set: BTreeSet<String> = entries.keys().cloned().collect();
        let (sp_ids, robot_ids) = discovery::discover(
            &key_set,
            state.cfg.sp_id.as_deref(),
            state.cfg.robot_id.as_deref(),
        );

        // Robot status is *not* derived here: `robot_poller` owns it, on a much
        // shorter period. Computing it in both loops would leave this one
        // overwriting the fresh values with quarter-second-old ones, and the two
        // would republish against each other forever.
        let goals: BTreeMap<String, proto::GoalsStatus> = sp_ids
            .iter()
            .map(|id| (id.clone(), status::goals_status(&entries, id)))
            .collect();

        let info = proto::ServerInfo {
            redis_addr: state.cm.redis_addr().to_string(),
            sp_ids,
            robot_ids,
            log_dir: state
                .cfg
                .log_dir
                .as_ref()
                .filter(|d| d.is_dir())
                .map(|d| d.display().to_string()),
            frames_dir: state.cfg.frames_dir.as_ref().map(|d| d.display().to_string()),
        };

        // Work out what to announce, then release the lock before publishing.
        let mut messages: Vec<proto::ServerMsg> = Vec::new();
        {
            let mut snap = state.snapshot.write().await;
            let (changed, removed) = diff(&snap.entries, &entries);
            if !changed.is_empty() || !removed.is_empty() {
                snap.revision += 1;
                messages.push(proto::ServerMsg::StateDelta(proto::StateDelta {
                    revision: snap.revision,
                    changed,
                    removed,
                }));
            }
            snap.entries = entries;

            let transforms = proto::TransformsSnapshot {
                revision: snap.transforms.revision + 1,
                ..transforms
            };
            if transforms.frames != snap.transforms.frames
                || transforms.cyclic != snap.transforms.cyclic
            {
                messages.push(proto::ServerMsg::Transforms(transforms.clone()));
                snap.transforms = transforms;
            }

            for (id, status) in &goals {
                if snap.goals.get(id) != Some(status) {
                    messages.push(proto::ServerMsg::Goals(status.clone()));
                }
            }
            snap.goals = goals;

            if snap.info.sp_ids != info.sp_ids
                || snap.info.robot_ids != info.robot_ids
                || snap.info.redis_addr != info.redis_addr
                || snap.info.log_dir != info.log_dir
            {
                messages.push(proto::ServerMsg::Hello(info.clone()));
            }
            snap.info = info;
        }

        for msg in messages {
            state.publish(msg);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(key: &str, n: i64) -> proto::StateEntry {
        let value = proto::GuiValue::Int64(proto::IntOrUnknown::Int64(n));
        proto::StateEntry {
            key: key.to_string(),
            raw: serde_json::to_string(&value).unwrap(),
            value: Some(value),
        }
    }

    fn map(pairs: Vec<proto::StateEntry>) -> Entries {
        pairs.into_iter().map(|e| (e.key.clone(), e)).collect()
    }

    #[test]
    fn an_unchanged_map_produces_no_delta() {
        let a = map(vec![entry("x", 1), entry("y", 2)]);
        let (changed, removed) = diff(&a, &a.clone());
        assert!(changed.is_empty());
        assert!(removed.is_empty());
    }

    #[test]
    fn reports_only_what_moved() {
        let old = map(vec![entry("x", 1), entry("y", 2), entry("gone", 3)]);
        let new = map(vec![entry("x", 1), entry("y", 99), entry("fresh", 4)]);
        let (changed, removed) = diff(&old, &new);
        let mut names: Vec<&str> = changed.iter().map(|e| e.key.as_str()).collect();
        names.sort();
        assert_eq!(names, vec!["fresh", "y"]);
        assert_eq!(removed, vec!["gone".to_string()]);
    }
}
