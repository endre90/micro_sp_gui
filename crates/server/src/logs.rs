//! Tailing the micro_sp activity log.
//!
//! micro_sp no longer publishes log blobs to Redis - the old
//! `{sp_id}_logger_planned_operations` and friends are gone. It writes a
//! rotating text file instead, enabled by `MICRO_SP_ACTIVITY_LOG_DIR`, in the
//! format fixed by `micro_sp::activity_log::format_record`:
//!
//! ```text
//! # micro_sp activity log - opened 2026-08-19 09:00:00.000 +02:00
//! # columns: timestamp | kind | source | subject | detail
//! 2026-08-19 09:00:01.120 | OP    | sp_operation_runner        | op_move                            | initial -> executing  (Starting)
//! ```
//!
//! Columns are space-padded to fixed minimum widths and separated by `" | "`.
//! Only `detail` is free-form, so splitting into five fields and keeping the
//! remainder as the detail is safe even when the detail itself contains a pipe.

use crate::{AppState, LOG_TARGET};
use micro_sp_gui_protocol as proto;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::io::{AsyncReadExt, AsyncSeekExt};

/// How often to look for new lines.
const TAIL_INTERVAL_MS: u64 = 200;

/// How much of the active file to read back at startup.
const BACKFILL_BYTES: u64 = 2 * 1024 * 1024;

static SEQ: AtomicU64 = AtomicU64::new(0);

fn next_seq() -> u64 {
    SEQ.fetch_add(1, Ordering::Relaxed)
}

/// Parse one line. `None` for blanks and the `#` banner.
pub fn parse_line(raw: &str) -> Option<proto::LogLine> {
    let trimmed = raw.trim_end_matches(['\r', '\n']);
    if trimmed.trim().is_empty() || trimmed.starts_with('#') {
        return None;
    }

    let fields: Vec<&str> = trimmed.splitn(5, " | ").collect();
    if fields.len() < 5 {
        // Something that is not a record - a panic backtrace, a partially
        // flushed line. Keep it rather than dropping it: an unexplained gap in
        // a log is worse than an unparsed line in it.
        return Some(proto::LogLine {
            seq: next_seq(),
            at: String::new(),
            kind: proto::LogKind::Other,
            source: String::new(),
            subject: String::new(),
            detail: trimmed.to_string(),
            raw: trimmed.to_string(),
        });
    }

    Some(proto::LogLine {
        seq: next_seq(),
        at: fields[0].trim().to_string(),
        kind: proto::LogKind::from_tag(fields[1]),
        source: fields[2].trim_end().to_string(),
        subject: fields[3].trim_end().to_string(),
        detail: fields[4].to_string(),
        raw: trimmed.to_string(),
    })
}

fn parse_chunk(chunk: &str) -> Vec<proto::LogLine> {
    chunk.lines().filter_map(parse_line).collect()
}

/// Rotated files, oldest first.
///
/// micro_sp names them `{stem}-{YYYYMMDD}-{HHMMSS}.log` with a zero-padded
/// counter for same-second rotations, precisely so that lexicographic order is
/// chronological order.
async fn rotated_files(dir: &Path, stem: &str) -> Vec<PathBuf> {
    let prefix = format!("{stem}-");
    let active = format!("{stem}.log");
    let mut out = Vec::new();
    let Ok(mut entries) = tokio::fs::read_dir(dir).await else {
        return out;
    };
    while let Ok(Some(entry)) = entries.next_entry().await {
        let path = entry.path();
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else { continue };
        if name != active && name.starts_with(&prefix) && name.ends_with(".log") {
            out.push(path);
        }
    }
    out.sort();
    out
}

/// Read `path` from `from` to its end, returning the text and the new offset.
async fn read_from(path: &Path, from: u64) -> std::io::Result<(String, u64)> {
    let mut file = tokio::fs::File::open(path).await?;
    let len = file.metadata().await?.len();
    if from >= len {
        return Ok((String::new(), len));
    }
    file.seek(std::io::SeekFrom::Start(from)).await?;
    let mut buf = Vec::with_capacity((len - from) as usize);
    file.read_to_end(&mut buf).await?;
    // A log line is UTF-8, but a torn write could split a multi-byte char.
    Ok((String::from_utf8_lossy(&buf).into_owned(), len))
}

/// Tracks one active file across rotations.
struct Tailer {
    dir: PathBuf,
    stem: String,
    active: PathBuf,
    offset: u64,
    /// Inode of the file `offset` refers to. A change means it was rotated out
    /// from under us.
    inode: Option<u64>,
    /// A trailing line that had not been terminated yet when we last read.
    partial: String,
}

impl Tailer {
    fn new(dir: PathBuf, stem: String) -> Self {
        let active = dir.join(format!("{stem}.log"));
        Self { dir, stem, active, offset: 0, inode: None, partial: String::new() }
    }

    async fn inode_of(path: &Path) -> Option<u64> {
        #[cfg(unix)]
        {
            use std::os::unix::fs::MetadataExt;
            tokio::fs::metadata(path).await.ok().map(|m| m.ino())
        }
        #[cfg(not(unix))]
        {
            let _ = path;
            None
        }
    }

    /// Start at the tail of the current file rather than replaying its whole
    /// history, so a long-lived log does not flood the ring on startup.
    async fn backfill(&mut self) -> Vec<proto::LogLine> {
        let Ok(meta) = tokio::fs::metadata(&self.active).await else {
            return Vec::new();
        };
        let len = meta.len();
        let from = len.saturating_sub(BACKFILL_BYTES);
        self.inode = Self::inode_of(&self.active).await;
        match read_from(&self.active, from).await {
            Ok((text, new_offset)) => {
                self.offset = new_offset;
                // A mid-file start almost certainly lands inside a line.
                let text = if from > 0 {
                    match text.find('\n') {
                        Some(i) => &text[i + 1..],
                        None => "",
                    }
                } else {
                    &text[..]
                };
                parse_chunk(text)
            }
            Err(e) => {
                log::warn!(target: LOG_TARGET, "Could not backfill {}: {e}", self.active.display());
                Vec::new()
            }
        }
    }

    /// Everything appended since the last call.
    async fn poll(&mut self) -> Vec<proto::LogLine> {
        let Ok(meta) = tokio::fs::metadata(&self.active).await else {
            // The file has not been created yet, or the directory went away.
            return Vec::new();
        };
        let len = meta.len();
        let inode = Self::inode_of(&self.active).await;

        let mut chunks: Vec<String> = Vec::new();

        let rotated = self.inode.is_some() && inode.is_some() && inode != self.inode;
        if rotated {
            // The file we were reading was renamed aside. Finish it from the
            // archive before switching, or those lines are lost forever.
            if let Some(newest) = rotated_files(&self.dir, &self.stem).await.pop()
                && let Ok((text, _)) = read_from(&newest, self.offset).await
            {
                chunks.push(text);
            }
            self.offset = 0;
        } else if len < self.offset {
            // Truncated in place - start over rather than seeking past the end.
            log::info!(
                target: LOG_TARGET,
                "{} shrank; re-reading from the start.", self.active.display()
            );
            self.offset = 0;
            self.partial.clear();
        }
        self.inode = inode;

        if len > self.offset || rotated {
            match read_from(&self.active, self.offset).await {
                Ok((text, new_offset)) => {
                    self.offset = new_offset;
                    chunks.push(text);
                }
                Err(e) => {
                    log::warn!(target: LOG_TARGET, "Could not read {}: {e}", self.active.display());
                }
            }
        }

        if chunks.is_empty() {
            return Vec::new();
        }

        // Stitch a line that was split across two reads back together.
        let mut text = std::mem::take(&mut self.partial);
        text.push_str(&chunks.concat());
        if !text.ends_with('\n') {
            if let Some(i) = text.rfind('\n') {
                self.partial = text[i + 1..].to_string();
                text.truncate(i + 1);
            } else {
                self.partial = text;
                return Vec::new();
            }
        }
        parse_chunk(&text)
    }
}

/// Tail until the process ends. Returns immediately if logging is not
/// configured, which the frontend learns from `ServerInfo::log_dir`.
pub async fn run(state: Arc<AppState>) {
    let Some(dir) = state.cfg.log_dir.clone() else {
        log::info!(
            target: LOG_TARGET,
            "No activity log directory configured; the Logs tab will only show live information. \
             Set MICRO_SP_ACTIVITY_LOG_DIR (for micro_sp and for this server) to get history."
        );
        return;
    };

    let mut tailer = Tailer::new(dir.clone(), state.cfg.log_stem.clone());
    let backfill = tailer.backfill().await;
    if !backfill.is_empty() {
        log::info!(target: LOG_TARGET, "Backfilled {} activity log line(s).", backfill.len());
        state.push_logs(backfill).await;
    }

    let mut ticker =
        tokio::time::interval(std::time::Duration::from_millis(TAIL_INTERVAL_MS));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        ticker.tick().await;
        let lines = tailer.poll().await;
        state.push_logs(lines).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = concat!(
        "# micro_sp activity log - opened 2026-08-19 09:00:00.000 +02:00\n",
        "# columns: timestamp | kind | source | subject | detail\n",
        "#\n",
        "2026-08-19 09:00:01.120 | OP    | sp_operation_runner        | op_move                            | initial -> executing  (Starting)\n",
        "2026-08-19 09:00:01.125 | TRANS | sp_auto_transition_runner  | t_tick                             | taken as 'aB3dE5fG7h'\n",
        "2026-08-19 09:00:01.130 | SOP   | sp_sop_runner              | sop_1                              | executing -> completed\n",
        "2026-08-19 09:00:01.140 | VAR   | sp_operation_runner        | r1_command_type                    | UNKNOWN -> unsafe_move_j\n",
    );

    #[test]
    fn skips_the_banner_and_parses_every_kind() {
        let lines = parse_chunk(SAMPLE);
        assert_eq!(lines.len(), 4, "the three '#' lines must not become records");
        assert_eq!(lines[0].kind, proto::LogKind::Operation);
        assert_eq!(lines[0].at, "2026-08-19 09:00:01.120");
        assert_eq!(lines[0].source, "sp_operation_runner");
        assert_eq!(lines[0].subject, "op_move");
        assert_eq!(lines[0].detail, "initial -> executing  (Starting)");
        assert_eq!(lines[1].kind, proto::LogKind::Transition);
        assert_eq!(lines[2].kind, proto::LogKind::Sop);
        assert_eq!(lines[3].kind, proto::LogKind::Variable);
        assert_eq!(lines[3].subject, "r1_command_type");
    }

    /// A variable's value can itself contain the column separator.
    #[test]
    fn a_pipe_inside_the_detail_stays_in_the_detail() {
        let line = parse_line(
            "2026-08-19 09:00:02.000 | VAR   | sp_op_runner               | note                               | UNKNOWN -> a | b | c",
        )
        .expect("parses");
        assert_eq!(line.kind, proto::LogKind::Variable);
        assert_eq!(line.subject, "note");
        assert_eq!(line.detail, "UNKNOWN -> a | b | c");
    }

    /// A name longer than its column pushes the line right instead of being cut.
    #[test]
    fn an_overlong_column_still_parses() {
        let long = "a_very_long_operation_name_that_exceeds_the_column_width";
        let raw = format!("2026-08-19 09:00:03.000 | OP    | src | {long} | initial -> executing");
        let line = parse_line(&raw).expect("parses");
        assert_eq!(line.subject, long);
        assert_eq!(line.source, "src");
    }

    #[test]
    fn an_unrecognised_line_is_kept_as_other() {
        let line = parse_line("thread 'main' panicked at src/lib.rs:1:1").expect("kept");
        assert_eq!(line.kind, proto::LogKind::Other);
        assert_eq!(line.detail, "thread 'main' panicked at src/lib.rs:1:1");
    }

    #[test]
    fn blank_and_banner_lines_are_dropped() {
        assert!(parse_line("").is_none());
        assert!(parse_line("   ").is_none());
        assert!(parse_line("# columns: a | b").is_none());
    }

    #[test]
    fn sequence_numbers_increase() {
        let a = parse_line("x | y | z | w | v").unwrap();
        let b = parse_line("x | y | z | w | v").unwrap();
        assert!(b.seq > a.seq);
    }
}
