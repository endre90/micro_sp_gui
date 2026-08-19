//! Shared server state.
//!
//! There is exactly **one** poller and **one** log tailer per process, no matter
//! how many browsers are connected: each websocket subscribes to a broadcast
//! channel and filters. Polling per client would multiply Redis load by the
//! number of open tabs.

use crate::config::Config;
use micro_sp::ConnectionManager;
use micro_sp_gui_protocol as proto;
use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;
use tokio::sync::{RwLock, broadcast};

/// The latest known picture of the system.
#[derive(Default, Debug)]
pub struct Snapshot {
    pub revision: u64,
    /// Keyed by Redis key, sorted, so the frontend's table order is stable and
    /// diffing is cheap.
    pub entries: BTreeMap<String, proto::StateEntry>,
    pub transforms: proto::TransformsSnapshot,
    pub info: proto::ServerInfo,
    pub robots: BTreeMap<String, proto::RobotStatus>,
    pub goals: BTreeMap<String, proto::GoalsStatus>,
}

impl Snapshot {
    pub fn as_state_snapshot(&self) -> proto::StateSnapshot {
        proto::StateSnapshot {
            revision: self.revision,
            entries: self.entries.values().cloned().collect(),
        }
    }
}

pub struct AppState {
    pub cm: Arc<ConnectionManager>,
    pub snapshot: Arc<RwLock<Snapshot>>,
    pub bus: broadcast::Sender<proto::ServerMsg>,
    pub logs: Arc<RwLock<VecDeque<proto::LogLine>>>,
    pub cfg: Config,
}

impl AppState {
    pub fn new(cm: Arc<ConnectionManager>, cfg: Config) -> Arc<Self> {
        // Generous, because a full-state broadcast is one big message and a
        // browser that is repainting slowly should not be dropped for it.
        let (bus, _) = broadcast::channel(256);
        Arc::new(Self {
            cm,
            snapshot: Arc::new(RwLock::new(Snapshot::default())),
            bus,
            logs: Arc::new(RwLock::new(VecDeque::new())),
            cfg,
        })
    }

    /// Send to every connected socket. A send with no receivers is not an error.
    pub fn publish(&self, msg: proto::ServerMsg) {
        let _ = self.bus.send(msg);
    }

    /// Append tailed lines to the ring and push them out.
    pub async fn push_logs(&self, lines: Vec<proto::LogLine>) {
        if lines.is_empty() {
            return;
        }
        {
            let mut ring = self.logs.write().await;
            for line in &lines {
                ring.push_back(line.clone());
            }
            while ring.len() > self.cfg.log_ring {
                ring.pop_front();
            }
        }
        self.publish(proto::ServerMsg::Logs(lines));
    }

    /// The tail of the ring that matches `query`, oldest first.
    pub async fn query_logs(&self, query: &proto::LogQuery) -> Vec<proto::LogLine> {
        let ring = self.logs.read().await;
        let re = query.grep.as_ref().filter(|g| !g.is_empty() && query.regex).and_then(|g| {
            regex::RegexBuilder::new(g).case_insensitive(true).build().ok()
        });
        let matcher = re.map(|re| move |s: &str| re.is_match(s));
        let pred: Option<&dyn Fn(&str) -> bool> =
            matcher.as_ref().map(|m| m as &dyn Fn(&str) -> bool);

        let mut out: Vec<proto::LogLine> = ring
            .iter()
            .rev()
            .filter(|l| query.matches(l, pred))
            .take(query.limit.max(1))
            .cloned()
            .collect();
        out.reverse();
        out
    }
}
