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
//!
//! **The records carry no timezone.** micro_sp formats them with
//! `chrono::Local`, so a runner in a container with no `TZ` writes UTC and one
//! on the host writes local time - and the file gives no hint which. The banner
//! does: it is the only line with a `%:z` offset on it. Parsing that and
//! resolving each record against it is what stops a dockerised runner's log from
//! reading two hours early in the browser.

use crate::{AppState, LOG_TARGET};
use chrono::{DateTime, FixedOffset, NaiveDateTime};
use micro_sp_gui_protocol as proto;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::io::{AsyncReadExt, AsyncSeekExt};

/// How often to look for new lines.
const TAIL_INTERVAL_MS: u64 = 200;

/// How much of the active file to read back at startup.
const BACKFILL_BYTES: u64 = 2 * 1024 * 1024;

/// How much of the *head* of a file to read looking for its banner. The banner
/// is the first line, so this only has to be comfortably longer than one.
const BANNER_BYTES: u64 = 512;

/// The format micro_sp writes a record's timestamp in, with no offset on it.
const RECORD_FORMAT: &str = "%Y-%m-%d %H:%M:%S%.3f";

static SEQ: AtomicU64 = AtomicU64::new(0);

fn next_seq() -> u64 {
    SEQ.fetch_add(1, Ordering::Relaxed)
}

/// The UTC offset out of the banner micro_sp writes when it opens a file:
///
/// ```text
/// # micro_sp activity log - opened 2026-08-20 07:27:54.979 +00:00
/// ```
///
/// `None` for every other line, including the other `#` ones.
pub fn parse_banner_offset(raw: &str) -> Option<FixedOffset> {
    let rest = raw.trim().strip_prefix("# micro_sp activity log - opened ")?;
    DateTime::parse_from_str(rest.trim(), "%Y-%m-%d %H:%M:%S%.3f %:z")
        .ok()
        .map(|dt| *dt.offset())
}

/// A record's timestamp as milliseconds since the epoch, read against the offset
/// its file was opened with.
fn to_utc_ms(at: &str, offset: Option<FixedOffset>) -> Option<i64> {
    let offset = offset?;
    let naive = NaiveDateTime::parse_from_str(at, RECORD_FORMAT).ok()?;
    // `single()` rather than `unwrap`: a local time inside a DST fold is
    // ambiguous, and a fixed offset never is - but the type does not know that.
    naive.and_local_timezone(offset).single().map(|dt| dt.timestamp_millis())
}

/// The head of `path`, far enough in to contain the banner.
async fn banner_offset(path: &Path) -> Option<FixedOffset> {
    let file = tokio::fs::File::open(path).await.ok()?;
    let mut buf = Vec::new();
    file.take(BANNER_BYTES).read_to_end(&mut buf).await.ok()?;
    String::from_utf8_lossy(&buf).lines().find_map(parse_banner_offset)
}

/// Parse one line. `None` for blanks and the `#` banner.
///
/// `offset` is whatever the file's banner said; it only fills `at_utc_ms`, so
/// `None` degrades to exactly the behaviour before offsets were read at all.
pub fn parse_line(raw: &str, offset: Option<FixedOffset>) -> Option<proto::LogLine> {
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
            at_utc_ms: None,
            kind: proto::LogKind::Other,
            source: String::new(),
            subject: String::new(),
            detail: trimmed.to_string(),
            raw: trimmed.to_string(),
        });
    }

    let at = fields[0].trim().to_string();
    Some(proto::LogLine {
        seq: next_seq(),
        at_utc_ms: to_utc_ms(&at, offset),
        at,
        kind: proto::LogKind::from_tag(fields[1]),
        source: fields[2].trim_end().to_string(),
        subject: fields[3].trim_end().to_string(),
        detail: fields[4].to_string(),
        raw: trimmed.to_string(),
    })
}

/// Parse a chunk, picking the offset up from any banner inside it.
///
/// A rotation puts the tail of the old file and the head of the new one in the
/// same chunk, and the new one's banner sits between them - so the offset has to
/// be able to change mid-chunk rather than being decided up front.
fn parse_chunk(chunk: &str, offset: &mut Option<FixedOffset>) -> Vec<proto::LogLine> {
    let mut lines = Vec::new();
    for raw in chunk.lines() {
        if let Some(found) = parse_banner_offset(raw) {
            *offset = Some(found);
        }
        if let Some(line) = parse_line(raw, *offset) {
            lines.push(line);
        }
    }
    lines
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
    /// The UTC offset the current file's banner records, i.e. the clock its
    /// records were written against. `None` until a banner has been seen.
    tz: Option<FixedOffset>,
}

impl Tailer {
    fn new(dir: PathBuf, stem: String) -> Self {
        let active = dir.join(format!("{stem}.log"));
        Self { dir, stem, active, offset: 0, inode: None, partial: String::new(), tz: None }
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
        // The backfill starts at the *end* of the file, so it will not see the
        // banner; read the head separately for it.
        self.tz = banner_offset(&self.active).await;
        if self.tz.is_none() {
            log::warn!(
                target: LOG_TARGET,
                "{} has no readable banner; log timestamps will be shown exactly as written, \
                 which is wrong if micro_sp ran in a different timezone than the viewer.",
                self.active.display()
            );
        }
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
                let mut tz = self.tz;
                let lines = parse_chunk(text, &mut tz);
                self.tz = tz;
                lines
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
        let mut tz = self.tz;
        let lines = parse_chunk(&text, &mut tz);
        self.tz = tz;
        lines
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
        let lines = parse_chunk(SAMPLE, &mut None);
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
            None,
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
        let line = parse_line(&raw, None).expect("parses");
        assert_eq!(line.subject, long);
        assert_eq!(line.source, "src");
    }

    #[test]
    fn an_unrecognised_line_is_kept_as_other() {
        let line = parse_line("thread 'main' panicked at src/lib.rs:1:1", None).expect("kept");
        assert_eq!(line.kind, proto::LogKind::Other);
        assert_eq!(line.detail, "thread 'main' panicked at src/lib.rs:1:1");
    }

    #[test]
    fn blank_and_banner_lines_are_dropped() {
        assert!(parse_line("", None).is_none());
        assert!(parse_line("   ", None).is_none());
        assert!(parse_line("# columns: a | b", None).is_none());
    }

    /// The whole point of reading the banner: the same wall-clock string means
    /// two different instants depending on the clock the writer was on, and only
    /// the banner says which.
    #[test]
    fn the_banner_offset_decides_what_instant_a_record_is() {
        let stockholm = parse_banner_offset(
            "# micro_sp activity log - opened 2026-08-20 09:05:10.728 +02:00",
        )
        .expect("an offset");
        let utc = parse_banner_offset(
            "# micro_sp activity log - opened 2026-08-20 07:27:54.979 +00:00",
        )
        .expect("an offset");
        assert_eq!(stockholm.local_minus_utc(), 2 * 3600);
        assert_eq!(utc.local_minus_utc(), 0);

        // Both files say "07:27:54.979", and they are two hours apart.
        let at = "2026-08-20 07:27:54.979";
        let in_utc = to_utc_ms(at, Some(utc)).expect("resolves");
        let in_stockholm = to_utc_ms(at, Some(stockholm)).expect("resolves");
        assert_eq!(in_utc - in_stockholm, 2 * 3600 * 1000);
    }

    #[test]
    fn the_other_banner_lines_carry_no_offset() {
        assert!(
            parse_banner_offset("# columns: timestamp | kind | source | subject | detail")
                .is_none()
        );
        assert!(parse_banner_offset("#").is_none());
        assert!(parse_banner_offset("2026-08-19 09:00:01.120 | OP | a | b | c").is_none());
    }

    /// Without a banner the timestamp is still shown, just not relocated - which
    /// is exactly what happened before any of this existed.
    #[test]
    fn no_banner_means_no_instant_but_still_a_line() {
        let line = parse_line(
            "2026-08-19 09:00:01.120 | OP    | sp_operation_runner        | op_move                            | initial -> executing",
            None,
        )
        .expect("parses");
        assert_eq!(line.at, "2026-08-19 09:00:01.120");
        assert_eq!(line.at_utc_ms, None);
    }

    /// The sample's own banner is `+02:00`, and `parse_chunk` has to pick it up
    /// on the way past rather than being told about it.
    #[test]
    fn a_chunk_learns_its_offset_from_its_own_banner() {
        let lines = parse_chunk(SAMPLE, &mut None);
        let at_utc_ms = lines[0].at_utc_ms.expect("the banner was read");
        // 2026-08-19 09:00:01.120 +02:00 is 07:00:01.120 UTC.
        let expected = chrono::DateTime::parse_from_rfc3339("2026-08-19T09:00:01.120+02:00")
            .unwrap()
            .timestamp_millis();
        assert_eq!(at_utc_ms, expected);
    }

    /// A rotation puts two files' worth of text in one chunk. The lines before
    /// the new banner belong to the old file's clock, the ones after to the new
    /// one's.
    #[test]
    fn a_banner_mid_chunk_switches_the_offset_from_there_on() {
        let chunk = concat!(
            "2026-08-20 09:00:00.000 | OP    | r | a | before\n",
            "# micro_sp activity log - opened 2026-08-20 07:00:00.000 +00:00\n",
            "2026-08-20 09:00:00.000 | OP    | r | a | after\n",
        );
        let mut tz = chrono::FixedOffset::east_opt(2 * 3600);
        let lines = parse_chunk(chunk, &mut tz);
        assert_eq!(lines.len(), 2, "the banner is not a record");
        let before = lines[0].at_utc_ms.expect("resolves");
        let after = lines[1].at_utc_ms.expect("resolves");
        assert_eq!(after - before, 2 * 3600 * 1000);
        assert_eq!(tz, chrono::FixedOffset::east_opt(0));
    }

    #[test]
    fn sequence_numbers_increase() {
        let a = parse_line("x | y | z | w | v", None).unwrap();
        let b = parse_line("x | y | z | w | v", None).unwrap();
        assert!(b.seq > a.seq);
    }
}
