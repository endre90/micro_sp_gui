//! State snapshots and the write requests that go back.

use crate::value::{GuiValue, GuiValueType};
use serde::{Deserialize, Serialize};

/// One Redis key and what it holds.
///
/// `raw` is the exact JSON string Redis returned, and `value` is `None` when
/// that string does not parse as an `SPValue`. Keeping both means a malformed
/// value is *visible* in the UI rather than silently dropped - `micro_sp`'s own
/// `StateManager::build_state` drops such keys, which is how a bad value turns
/// into a confusing missing-variable panic somewhere else.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct StateEntry {
    pub key: String,
    pub value: Option<GuiValue>,
    pub raw: String,
}

impl StateEntry {
    pub fn type_label(&self) -> String {
        match &self.value {
            Some(v) => v.type_of().to_string(),
            None => "unparsed".to_string(),
        }
    }

    pub fn display(&self) -> String {
        match &self.value {
            Some(v) => v.display(),
            None => format!("<unparsed: {}>", truncate(&self.raw, 60)),
        }
    }
}

fn truncate(s: &str, max: usize) -> String {
    if s.chars().count() <= max {
        return s.to_string();
    }
    let mut out: String = s.chars().take(max).collect();
    out.push('…');
    out
}

/// The whole keyspace, sent once when a socket opens.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct StateSnapshot {
    pub revision: u64,
    pub entries: Vec<StateEntry>,
}

/// What changed since `revision - 1`. Sent on every poll that found a change.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct StateDelta {
    pub revision: u64,
    pub changed: Vec<StateEntry>,
    pub removed: Vec<String>,
}

/// Only `key` and `value` are read; `raw` is ignored on the way in.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct SetValuesRequest {
    pub values: Vec<SetValue>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SetValue {
    pub key: String,
    pub value: GuiValue,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct DeleteKeysRequest {
    pub keys: Vec<String>,
}

/// Per-key outcome, so a partly-rejected batch says which half landed.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct WriteReport {
    pub ok: Vec<String>,
    pub failed: Vec<(String, String)>,
}

impl WriteReport {
    pub fn summary(&self) -> String {
        if self.failed.is_empty() {
            format!("{} key(s) written", self.ok.len())
        } else {
            format!(
                "{} written, {} failed: {}",
                self.ok.len(),
                self.failed.len(),
                self.failed
                    .iter()
                    .map(|(k, e)| format!("{k}: {e}"))
                    .collect::<Vec<_>>()
                    .join("; ")
            )
        }
    }
}

/// What the server is connected to and what it found there.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ServerInfo {
    pub redis_addr: String,
    /// Discovered `sp_id`s - see `crates/server/src/discovery.rs`.
    pub sp_ids: Vec<String>,
    /// Discovered robot ids, i.e. `ur_redis_driver` instances.
    pub robot_ids: Vec<String>,
    /// `None` when no activity-log directory is configured or readable.
    pub log_dir: Option<String>,
    /// `None` when transform export is not configured.
    pub frames_dir: Option<String>,
}

/// A brand new key the operator is inventing in the State tab.
#[derive(Clone, Debug)]
pub struct NewKeyDraft {
    pub key: String,
    pub value_type: GuiValueType,
}

impl Default for NewKeyDraft {
    fn default() -> Self {
        Self { key: String::new(), value_type: GuiValueType::String }
    }
}
