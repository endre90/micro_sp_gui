//! What the system has been doing.
//!
//! micro_sp used to publish rolling log strings to Redis under
//! `{sp_id}_logger_planned_operations` and friends; those keys are gone. History
//! now comes from the on-disk activity log, which the server tails, and the
//! *current* one-line status of each runner and operation comes from the
//! `{name}_information` variables that are already in the state feed.
//!
//! Since micro_sp v0.5.0 that file also carries the console log - every
//! `log::error!`/`warn!`/`info!`/`debug!`/`trace!` the runners print is mirrored
//! into it, tagged `ERR`/`WARN`/`INFO`/`DEBUG`/`TRACE` in the same kind column
//! as `OP`/`TRANS`/`SOP`/`VAR`. So they are shown as rows of the same table,
//! filtered by the same toggles and matched by the same grep: whatever finds a
//! line here finds it in the file too.
//!
//! Filtering happens here rather than on the server, over the ring buffer the
//! browser already holds, so toggling a kind is instant. The same filter is sent
//! along as a backfill query so widening it fetches the history to match.

use crate::api::Api;
use crate::widgets::{self, ansi::strip_ansi, ansi_to_layout_job};
use micro_sp_gui_protocol as proto;
use std::collections::BTreeSet;

pub struct LogsTab {
    /// Which kinds to show. Empty would mean "nothing", so it starts full.
    kinds: BTreeSet<proto::LogKind>,
    grep: String,
    use_regex: bool,
    source: String,
    paused: bool,
    follow: bool,
    /// Lines are hidden rather than dropped, so unpausing shows the backlog.
    hidden_before: u64,
    show_information: bool,
    /// The last query sent to the server, so a backfill is only requested when
    /// the filter actually changes.
    last_query: Option<proto::LogQuery>,
}

impl Default for LogsTab {
    fn default() -> Self {
        Self {
            kinds: proto::LogKind::ALL.iter().copied().collect(),
            grep: String::new(),
            use_regex: false,
            source: String::new(),
            paused: false,
            follow: true,
            hidden_before: 0,
            show_information: true,
            last_query: None,
        }
    }
}

impl LogsTab {
    pub fn ui(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        self.filter_row(ui, api);
        ui.separator();

        if self.show_information {
            egui::SidePanel::right("live_information")
                .resizable(true)
                .default_width(420.0)
                .show_inside(ui, |ui| self.information_panel(ui, api));
        }

        self.scrollback(ui, api);
    }

    fn filter_row(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.horizontal_wrapped(|ui| {
            // Events first, then the mirrored console levels: two families of
            // line in one file, and the separator is the only hint that the
            // right-hand ones came from stderr.
            for kind in proto::LogKind::EVENTS {
                self.kind_toggle(ui, *kind);
            }
            ui.separator();
            for kind in proto::LogKind::LEVELS {
                self.kind_toggle(ui, *kind);
            }
            ui.separator();
            self.kind_toggle(ui, proto::LogKind::Other);

            ui.separator();
            ui.label("grep");
            ui.add(
                egui::TextEdit::singleline(&mut self.grep)
                    .hint_text("substring")
                    .desired_width(180.0),
            );
            ui.checkbox(&mut self.use_regex, "regex");
            if self.use_regex && !self.grep.is_empty() && self.compiled().is_none() {
                ui.label(egui::RichText::new("invalid regex").color(widgets::WARN));
            }

            ui.label("source");
            ui.add(
                egui::TextEdit::singleline(&mut self.source)
                    .hint_text("runner")
                    .desired_width(120.0),
            );
        });

        ui.horizontal_wrapped(|ui| {
            let pause_label = if self.paused { "▶ Resume" } else { "⏸ Pause" };
            ui.toggle_value(&mut self.paused, pause_label)
                .on_hover_text("Pausing only freezes the view; lines keep arriving.");
            ui.checkbox(&mut self.follow, "follow");
            if ui
                .button("Clear")
                .on_hover_text("Hides everything received so far. The file is untouched.")
                .clicked()
            {
                self.hidden_before = api.logs.back().map(|l| l.seq + 1).unwrap_or(0);
            }
            ui.checkbox(&mut self.show_information, "live information");

            ui.separator();
            match &api.info.log_dir {
                Some(dir) => {
                    ui.label(egui::RichText::new(dir).weak().monospace());
                }
                None => {
                    ui.label(
                        egui::RichText::new("no activity log configured")
                            .color(widgets::WARN),
                    )
                    .on_hover_text(
                        "Set MICRO_SP_ACTIVITY_LOG_DIR for the micro_sp process and start this \
                         server with the same directory. Without it there is no history, only \
                         the live information on the right.",
                    );
                }
            }
        });

        // Ask for history matching the current filter, but only when it changed.
        let query = self.query();
        if self.last_query.as_ref() != Some(&query) {
            self.last_query = Some(query.clone());
            api.send_subscribe(query);
        }
    }

    /// One kind's toggle. The hover text names the tag rather than the label,
    /// because the tag is what a `grep` over the file would use.
    fn kind_toggle(&mut self, ui: &mut egui::Ui, kind: proto::LogKind) {
        let mut on = self.kinds.contains(&kind);
        let text = egui::RichText::new(kind.label()).color(kind_colour(kind));
        if ui
            .toggle_value(&mut on, text)
            .on_hover_text(format!("Lines tagged {}", kind.tag()))
            .changed()
        {
            if on {
                self.kinds.insert(kind);
            } else {
                self.kinds.remove(&kind);
            }
        }
    }

    fn compiled(&self) -> Option<regex::Regex> {
        if !self.use_regex || self.grep.is_empty() {
            return None;
        }
        regex::RegexBuilder::new(&self.grep).case_insensitive(true).build().ok()
    }

    fn query(&self) -> proto::LogQuery {
        proto::LogQuery {
            // An explicitly complete selection is sent as "no filter".
            kinds: if self.kinds.len() == proto::LogKind::ALL.len() {
                Vec::new()
            } else {
                self.kinds.iter().copied().collect()
            },
            grep: if self.grep.is_empty() { None } else { Some(self.grep.clone()) },
            regex: self.use_regex,
            source_contains: if self.source.is_empty() {
                None
            } else {
                Some(self.source.clone())
            },
            limit: 5000,
        }
    }

    fn matches(&self, line: &proto::LogLine, re: Option<&regex::Regex>) -> bool {
        if line.seq < self.hidden_before {
            return false;
        }
        if !self.kinds.contains(&line.kind) {
            return false;
        }
        if !self.source.is_empty()
            && !line.source.to_lowercase().contains(&self.source.to_lowercase())
        {
            return false;
        }
        if self.grep.is_empty() {
            return true;
        }
        match re {
            Some(re) => re.is_match(&line.raw),
            None => line.raw.to_lowercase().contains(&self.grep.to_lowercase()),
        }
    }

    fn scrollback(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        let re = self.compiled();
        let visible: Vec<&proto::LogLine> =
            api.logs.iter().filter(|l| self.matches(l, re.as_ref())).collect();

        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(format!(
                    "{} of {} lines",
                    visible.len(),
                    api.logs.len()
                ))
                .weak(),
            );
            if visible.is_empty() && !api.logs.is_empty() {
                ui.label(egui::RichText::new("(everything filtered out)").weak());
            }
        });

        let row_height = ui.text_style_height(&egui::TextStyle::Monospace);
        egui::Frame::new()
            .fill(ui.visuals().extreme_bg_color)
            .inner_margin(6.0)
            .show(ui, |ui| {
                ui.set_min_height(ui.available_height());
                egui::ScrollArea::both()
                    .id_salt("log_scroll")
                    // Only follow when asked and not paused, or reading history
                    // is impossible.
                    .stick_to_bottom(self.follow && !self.paused)
                    .auto_shrink([false, false])
                    .show_rows(ui, row_height, visible.len(), |ui, range| {
                        for line in &visible[range] {
                            log_row(ui, line);
                        }
                    });
            });
    }

    /// Everything the system currently *says*, as opposed to what it did.
    fn information_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.heading("Live information");
        ui.label(
            egui::RichText::new(
                "The current value of every '_information' variable: what each runner, \
                 operation and SOP is saying right now.",
            )
            .weak()
            .small(),
        );
        ui.separator();

        // Runner-level lines first: they are the summary, the per-operation ones
        // are the detail.
        let sp_id = api.sp_id.clone().unwrap_or_default();
        let runner_keys: Vec<String> = [
            "main_runner_information",
            "planner_information",
            "plan_runner_information",
            "goal_runner_information",
            "goal_scheduler_information",
            "sop_runner_information",
        ]
        .iter()
        .map(|suffix| format!("{sp_id}_{suffix}"))
        .collect();

        egui::ScrollArea::vertical().id_salt("info_scroll").show(ui, |ui| {
            for key in &runner_keys {
                if let Some(value) = api.value(key) {
                    information_row(ui, key, &value.display());
                }
            }

            let others: Vec<(String, String)> = api
                .entries
                .iter()
                .filter(|(key, _)| key.ends_with("_information") && !runner_keys.contains(key))
                .map(|(key, entry)| (key.clone(), entry.display()))
                .collect();

            if !others.is_empty() {
                ui.separator();
                ui.label(
                    egui::RichText::new(format!("Operations and SOPs ({})", others.len()))
                        .strong(),
                );
                for (key, value) in others {
                    information_row(ui, &key, &value);
                }
            }
        });
    }
}

fn information_row(ui: &mut egui::Ui, key: &str, value: &str) {
    ui.horizontal_wrapped(|ui| {
        // The suffix is the same on every row; the name is the information.
        let short = key.strip_suffix("_information").unwrap_or(key);
        ui.label(egui::RichText::new(short).monospace().weak());
        let colour = if value == "UNKNOWN" {
            widgets::UNKNOWN
        } else {
            ui.visuals().text_color()
        };
        ui.label(egui::RichText::new(strip_ansi(value)).monospace().color(colour));
    });
}

/// The timestamp to show, and what the file itself says if the two differ.
///
/// micro_sp writes records with `chrono::Local` and no offset on them, so the
/// same file reads two hours early when the runner was in a UTC container and
/// the operator is not. The server resolves each record against the offset in
/// the file's banner, leaving only the rendering to do here - in the *viewer's*
/// timezone, which is the browser's under wasm.
fn timestamp(line: &proto::LogLine) -> (String, Option<String>) {
    match line.at_utc_ms.and_then(chrono::DateTime::from_timestamp_millis) {
        Some(instant) => {
            let local = instant
                .with_timezone(&chrono::Local)
                .format("%Y-%m-%d %H:%M:%S%.3f")
                .to_string();
            let source = (local != line.at).then(|| {
                format!(
                    "Shown in your timezone.\nThe file says {}, on the clock micro_sp ran on.",
                    line.at
                )
            });
            (local, source)
        }
        // No banner in the file, so there is nothing to resolve it against.
        None => (line.at.clone(), None),
    }
}

fn kind_colour(kind: proto::LogKind) -> egui::Color32 {
    match kind {
        proto::LogKind::Operation => egui::Color32::from_rgb(0x64, 0xb5, 0xf6),
        proto::LogKind::Transition => egui::Color32::from_rgb(0xba, 0x68, 0xc8),
        proto::LogKind::Sop => egui::Color32::from_rgb(0x4d, 0xd0, 0xe1),
        proto::LogKind::Variable => egui::Color32::from_rgb(0x9e, 0x9e, 0x9e),
        // The console levels keep the severity colours used everywhere else, so
        // an ERR in the log reads as the same red as a failed operation.
        proto::LogKind::Error => widgets::BAD,
        proto::LogKind::Warn => widgets::WARN,
        proto::LogKind::Info => egui::Color32::from_rgb(0x81, 0xc7, 0x84),
        proto::LogKind::Debug => egui::Color32::from_rgb(0x78, 0x90, 0x9c),
        proto::LogKind::Trace => egui::Color32::from_rgb(0x60, 0x60, 0x60),
        proto::LogKind::Other => widgets::WARN,
    }
}

/// One line, in the same five columns the file has: timestamp, tag, source,
/// subject, detail. Coloured by kind, with the state change - or the severity of
/// a mirrored console line - picked out.
fn log_row(ui: &mut egui::Ui, line: &proto::LogLine) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 6.0;

        if line.kind == proto::LogKind::Other {
            // Not a record - a panic, a partial flush. Show it as it came.
            ui.label(
                egui::RichText::new(&line.detail).monospace().color(widgets::WARN),
            );
            return;
        }

        let (at, source) = timestamp(line);
        let stamp = ui.label(egui::RichText::new(at).monospace().weak());
        if let Some(source) = source {
            stamp.on_hover_text(source);
        }
        ui.label(
            egui::RichText::new(format!("{:<5}", line.kind.tag()))
                .monospace()
                .color(kind_colour(line.kind))
                .strong(),
        );
        ui.label(egui::RichText::new(&line.source).monospace().weak());
        // For a console line the subject is the `file:line` that printed it -
        // useful, but not the point of the row, so it stays out of the way.
        let subject = egui::RichText::new(&line.subject).monospace();
        ui.label(if line.kind.is_console() { subject.weak() } else { subject });

        // Details can carry SGR escapes when a logged value came from a
        // coloured console string.
        if line.detail.contains('\u{1b}') {
            ui.label(ansi_to_layout_job(
                &line.detail,
                ui.text_style_height(&egui::TextStyle::Monospace) * 0.85,
                ui.visuals().text_color(),
            ));
        } else {
            ui.label(egui::RichText::new(&line.detail).monospace().color(detail_colour(line)));
        }
    });
}

/// Terminal outcomes are worth spotting in a wall of text.
fn detail_colour(line: &proto::LogLine) -> egui::Color32 {
    // A console line's detail is the message, and its severity is already the
    // whole of what the row says; colour it like the tag rather than hunting for
    // state-change words in prose that never contains any.
    match line.kind {
        proto::LogKind::Error | proto::LogKind::Warn => return kind_colour(line.kind),
        proto::LogKind::Debug | proto::LogKind::Trace => return widgets::UNKNOWN,
        proto::LogKind::Info => return egui::Color32::PLACEHOLDER,
        _ => {}
    }

    let d = line.detail.as_str();
    if d.contains("-> failed") || d.contains("-> fatal") || d.contains("-> timedout") {
        widgets::BAD
    } else if d.contains("-> completed") || d.contains("-> terminated_completed") {
        widgets::OK
    } else if d.contains("-> cancelled") || d.contains("-> bypassed") {
        widgets::WARN
    } else {
        egui::Color32::PLACEHOLDER
    }
}
