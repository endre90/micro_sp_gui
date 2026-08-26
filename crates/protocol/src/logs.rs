//! The micro_sp on-disk activity log, as a wire type.
//!
//! `micro_sp` no longer publishes log blobs to Redis (the old
//! `{sp_id}_logger_*` keys are gone). It writes a rotating text file instead,
//! opt-in via `MICRO_SP_ACTIVITY_LOG_DIR`. The server tails that file and
//! parses each line back into a `LogLine`; see `crates/server/src/logs.rs`.
//!
//! Since micro_sp v0.5.0 that file carries two families of line in the same
//! kind column: the structured events saying *what* happened (`OP`, `TRANS`,
//! `SOP`, `VAR`) and the console log lines saying *why* (`ERR`, `WARN`, `INFO`,
//! `DEBUG`, `TRACE`), which the logger `initialize_env_logger` installs mirrors
//! into the file as it prints them. They are the same shape - one tag, one
//! source, one subject, one detail - so one filter, and one grep, sees both.

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
    /// `ERR` - a console `log::error!`, mirrored into the file.
    Error,
    /// `WARN` - a console `log::warn!`.
    Warn,
    /// `INFO` - a console `log::info!`.
    Info,
    /// `DEBUG` - a console `log::debug!`.
    Debug,
    /// `TRACE` - a console `log::trace!`.
    Trace,
    /// A line that did not carry a recognised tag.
    Other,
}

impl LogKind {
    pub const ALL: &'static [LogKind] = &[
        LogKind::Operation,
        LogKind::Transition,
        LogKind::Sop,
        LogKind::Variable,
        LogKind::Error,
        LogKind::Warn,
        LogKind::Info,
        LogKind::Debug,
        LogKind::Trace,
        LogKind::Other,
    ];

    /// The structured events: something the system did.
    pub const EVENTS: &'static [LogKind] = &[
        LogKind::Operation,
        LogKind::Transition,
        LogKind::Sop,
        LogKind::Variable,
    ];

    /// The mirrored console lines, most severe first.
    pub const LEVELS: &'static [LogKind] =
        &[LogKind::Error, LogKind::Warn, LogKind::Info, LogKind::Debug, LogKind::Trace];

    /// The tag as it appears in the file.
    ///
    /// micro_sp pads this column to five characters, and writes `ERR` rather
    /// than `ERROR` so every tag fits it. Keep these byte-for-byte identical to
    /// `micro_sp::activity_log::ActivityKind::tag`, or a grep that works on the
    /// file stops working in the GUI.
    pub fn tag(&self) -> &'static str {
        match self {
            LogKind::Operation => "OP",
            LogKind::Transition => "TRANS",
            LogKind::Sop => "SOP",
            LogKind::Variable => "VAR",
            LogKind::Error => "ERR",
            LogKind::Warn => "WARN",
            LogKind::Info => "INFO",
            LogKind::Debug => "DEBUG",
            LogKind::Trace => "TRACE",
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
            LogKind::Error => "ERR",
            LogKind::Warn => "WARN",
            LogKind::Info => "INFO",
            LogKind::Debug => "DEBUG",
            LogKind::Trace => "TRACE",
            LogKind::Other => "Other",
        }
    }

    /// Whether this is a mirrored console line rather than a structured event.
    /// Their columns mean different things - `subject` is a `file:line` rather
    /// than a name - so the two are rendered a little differently.
    pub fn is_console(&self) -> bool {
        matches!(
            self,
            LogKind::Error | LogKind::Warn | LogKind::Info | LogKind::Debug | LogKind::Trace
        )
    }

    pub fn from_tag(tag: &str) -> LogKind {
        match tag.trim() {
            "OP" => LogKind::Operation,
            "TRANS" => LogKind::Transition,
            "SOP" => LogKind::Sop,
            "VAR" => LogKind::Variable,
            // The long spellings are not what micro_sp writes, but a hand-edited
            // or third-party line is cheap to accept.
            "ERR" | "ERROR" => LogKind::Error,
            "WARN" | "WARNING" => LogKind::Warn,
            "INFO" => LogKind::Info,
            "DEBUG" => LogKind::Debug,
            "TRACE" => LogKind::Trace,
            _ => LogKind::Other,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct LogLine {
    /// Monotonic within one server run; the frontend dedupes and orders on it.
    pub seq: u64,
    /// `%Y-%m-%d %H:%M:%S%.3f`, exactly as written.
    ///
    /// Note that this carries **no timezone**: micro_sp formats it with
    /// `chrono::Local`, so a runner in a container with no `TZ` writes UTC and a
    /// runner on the host writes local time, and the two are indistinguishable
    /// here. Use `at_utc_ms` to render an instant; keep this for grep and for
    /// showing what the file literally says.
    pub at: String,
    /// `at` resolved to milliseconds since the Unix epoch, using the UTC offset
    /// the file's banner records. `None` when the banner was not seen or the
    /// timestamp did not parse, in which case only `at` is meaningful.
    #[serde(default)]
    pub at_utc_ms: Option<i64>,
    pub kind: LogKind,
    /// The runner that emitted it, e.g. `sp_operation_runner`. For a console
    /// line this is the `log` target, which is the same string.
    pub source: String,
    /// The operation, transition, SOP or variable name - or, for a console
    /// line, the `file:line` the statement was written at.
    pub subject: String,
    /// `from -> to`, `old -> new`, `taken as '<id>'`, ... - or, for a console
    /// line, the message itself.
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
