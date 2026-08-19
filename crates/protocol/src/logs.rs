//! The micro_sp on-disk activity log, as a wire type.
//!
//! `micro_sp` no longer publishes log blobs to Redis (the old
//! `{sp_id}_logger_*` keys are gone). It writes a rotating text file instead,
//! opt-in via `MICRO_SP_ACTIVITY_LOG_DIR`. The server tails that file and
//! parses each line back into a `LogLine`; see `crates/server/src/logs.rs`.

use serde::{Deserialize, Serialize};

// Ord so the frontend can keep the selected kinds in a BTreeSet; the ordering
// itself is not meaningful.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum LogKind {
    /// `OP` - an operation changed lifecycle state.
    Operation,
    /// `TRANS` - an automatic transition fired.
    Transition,
    /// `SOP` - a SOP was started, advanced or torn down.
    Sop,
    /// `VAR` - one variable took a new value.
    Variable,
    /// A line that did not carry a recognised tag.
    Other,
}

impl LogKind {
    pub const ALL: &'static [LogKind] = &[
        LogKind::Operation,
        LogKind::Transition,
        LogKind::Sop,
        LogKind::Variable,
        LogKind::Other,
    ];

    /// The tag as it appears in the file.
    pub fn tag(&self) -> &'static str {
        match self {
            LogKind::Operation => "OP",
            LogKind::Transition => "TRANS",
            LogKind::Sop => "SOP",
            LogKind::Variable => "VAR",
            LogKind::Other => "-",
        }
    }

    /// What to call it in the filter row.
    pub fn label(&self) -> &'static str {
        match self {
            LogKind::Operation => "Operations",
            LogKind::Transition => "Transitions",
            LogKind::Sop => "SOPs",
            LogKind::Variable => "Variables",
            LogKind::Other => "Other",
        }
    }

    pub fn from_tag(tag: &str) -> LogKind {
        match tag.trim() {
            "OP" => LogKind::Operation,
            "TRANS" => LogKind::Transition,
            "SOP" => LogKind::Sop,
            "VAR" => LogKind::Variable,
            _ => LogKind::Other,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct LogLine {
    /// Monotonic within one server run; the frontend dedupes and orders on it.
    pub seq: u64,
    /// `%Y-%m-%d %H:%M:%S%.3f`, exactly as written.
    pub at: String,
    pub kind: LogKind,
    /// The runner that emitted it, e.g. `sp_operation_runner`.
    pub source: String,
    /// The operation, transition, SOP or variable name.
    pub subject: String,
    /// `from -> to`, `old -> new`, `taken as '<id>'`, ...
    pub detail: String,
    /// The unparsed line, for grep and for lines that did not fit the format.
    pub raw: String,
}

/// What the frontend asks for as a backfill. Live lines arrive over the socket
/// unfiltered and are filtered client-side, so this only shapes the history.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct LogQuery {
    /// Empty means "every kind".
    pub kinds: Vec<LogKind>,
    pub grep: Option<String>,
    pub regex: bool,
    pub source_contains: Option<String>,
    pub limit: usize,
}

impl Default for LogQuery {
    fn default() -> Self {
        Self { kinds: Vec::new(), grep: None, regex: false, source_contains: None, limit: 2000 }
    }
}

impl LogQuery {
    /// Whether a line survives this query. Shared by the server's backfill and
    /// the frontend's live filter so both agree.
    pub fn matches(&self, line: &LogLine, compiled: Option<&dyn Fn(&str) -> bool>) -> bool {
        if !self.kinds.is_empty() && !self.kinds.contains(&line.kind) {
            return false;
        }
        if let Some(src) = &self.source_contains
            && !src.is_empty()
            && !line.source.to_lowercase().contains(&src.to_lowercase())
        {
            return false;
        }
        match (&self.grep, compiled) {
            (Some(g), _) if g.is_empty() => true,
            // A caller that wants regex passes a compiled matcher; the regex
            // crate is not a dependency here.
            (Some(_), Some(is_match)) => is_match(&line.raw),
            (Some(g), None) => line.raw.to_lowercase().contains(&g.to_lowercase()),
            (None, _) => true,
        }
    }
}
