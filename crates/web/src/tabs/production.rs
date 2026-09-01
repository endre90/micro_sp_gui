//! What the cell has produced, and how fast.
//!
//! Deliberately generic: this tab knows the *shape* of production metrics, not
//! any one cell's variables. Anything in the state under `kpi_` shows up here,
//! so a new figure is one line in the publisher and nothing at all in the GUI.
//!
//! The convention, which is all this file understands:
//!
//!   * `kpi_<group>_<name>` is one figure. Groups become sections, in the order
//!     [`GROUP_ORDER`] gives, and anything else lands alphabetically after them.
//!   * a `_ms` suffix means a duration in milliseconds; an `_at` suffix, or a
//!     `Time` value, means a timestamp.
//!   * `kpi_log_*` holding an array of maps is a table: one row per entry, one
//!     column per key, in the order the map was built.
//!
//! `campx_delidding`'s `metrics` node is the first publisher of it - see that
//! repository's README - but nothing here is specific to it.
//!
//! Read-only by design. Production figures are a record of what happened, and a
//! text field that lets an operator retype yesterday's pallet count is worse
//! than no text field.

use std::collections::BTreeMap;

use micro_sp_gui_protocol as proto;

use crate::api::Api;
use crate::widgets;

/// Keys the tab reads. Everything else in the state is somebody else's.
const PREFIX: &str = "kpi_";

/// Group name for the tables, which are laid out below the cards rather than as
/// a section of them.
const LOG_GROUP: &str = "log";

/// Sections in reading order: what is happening now, then how fast, then the
/// two sets of totals. Groups not listed follow, alphabetically.
const GROUP_ORDER: [&str; 4] = ["current", "rate", "run", "total"];

/// Human names for the groups we expect. An unlisted group is title-cased from
/// its own name, which is why an unknown publisher still reads sensibly.
fn group_heading(group: &str) -> String {
    match group {
        "current" => "Current pallet".to_string(),
        "rate" => "Rate".to_string(),
        "run" => "Since this run started".to_string(),
        "total" => "Lifetime totals".to_string(),
        other => title_case(other),
    }
}

#[derive(Default)]
pub struct ProductionTab {
    /// Which log table is expanded. Empty until the operator picks one; the
    /// first table is open by default.
    collapsed: BTreeMap<String, bool>,
}

impl ProductionTab {
    pub fn ui(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        let figures = collect(api);

        if figures.is_empty() {
            self.empty_hint(ui);
            return;
        }

        egui::ScrollArea::vertical().id_salt("production").show(ui, |ui| {
            ui.heading("Production");
            ui.label(
                egui::RichText::new(
                    "Everything published under kpi_. Read-only: these are a record of \
                     what happened.",
                )
                .weak()
                .small(),
            );
            ui.add_space(4.0);

            for group in ordered_groups(&figures) {
                if group == LOG_GROUP {
                    continue;
                }
                let Some(entries) = figures.get(&group) else { continue };
                ui.separator();
                ui.label(egui::RichText::new(group_heading(&group)).strong());
                cards(ui, &group, entries);
                ui.add_space(6.0);
            }

            if let Some(logs) = figures.get(LOG_GROUP) {
                for (name, value) in logs {
                    ui.separator();
                    self.table(ui, name, value);
                }
            }
        });
    }

    /// Shown when nothing publishes the convention, which is far more likely to
    /// be "the metrics node is not running" than "this tab is broken". Say what
    /// the tab is looking for rather than drawing a blank panel.
    fn empty_hint(&self, ui: &mut egui::Ui) {
        ui.heading("Production");
        ui.add_space(8.0);
        ui.label("Nothing in the state publishes production metrics yet.");
        ui.add_space(4.0);
        ui.label(
            egui::RichText::new(
                "This tab renders any variable named kpi_<group>_<name>: groups become \
                 sections, a _ms suffix is shown as a duration and an _at suffix as a \
                 timestamp. A kpi_log_* array of maps becomes a table, one row per entry.\n\n\
                 In campx_delidding these come from the `metrics` service - check that it \
                 is running and past its runner_ready gate.",
            )
            .weak(),
        );
    }

    /// One `kpi_log_*` array of maps, as a table.
    fn table(&mut self, ui: &mut egui::Ui, name: &str, value: &proto::GuiValue) {
        let rows = rows_of(value);
        let key = name.to_string();
        let collapsed = *self.collapsed.get(&key).unwrap_or(&false);

        ui.horizontal(|ui| {
            let arrow = if collapsed { "▶" } else { "▼" };
            if ui.small_button(arrow).clicked() {
                self.collapsed.insert(key.clone(), !collapsed);
            }
            ui.label(egui::RichText::new(title_case(name)).strong());
            ui.label(egui::RichText::new(format!("{} entries", rows.len())).weak().small());
        });

        if collapsed {
            return;
        }
        if rows.is_empty() {
            ui.label(egui::RichText::new("Nothing recorded yet.").weak());
            return;
        }

        // Columns come from the first row: the publisher builds every record the
        // same way, and a Map keeps its insertion order, so this is the order the
        // record was designed to be read in.
        let columns: Vec<String> = rows[0].iter().map(|(k, _)| k.clone()).collect();

        // Its own horizontal scroll: a wide record must not push the page sideways.
        egui::ScrollArea::horizontal().id_salt(format!("{name}_h")).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_salt(format!("{name}_v"))
                .max_height(320.0)
                .show(ui, |ui| {
                    egui::Grid::new(format!("{name}_grid"))
                        .striped(true)
                        .num_columns(columns.len())
                        .spacing([12.0, 4.0])
                        .show(ui, |ui| {
                            for column in &columns {
                                ui.label(
                                    egui::RichText::new(title_case(column)).strong().small(),
                                );
                            }
                            ui.end_row();

                            for row in &rows {
                                let lookup: BTreeMap<&str, &proto::GuiValue> =
                                    row.iter().map(|(k, v)| (k.as_str(), v)).collect();
                                for column in &columns {
                                    match lookup.get(column.as_str()) {
                                        Some(value) => {
                                            ui.label(figure_text(column, value).small());
                                        }
                                        // A record that lacks a column the first
                                        // one had: leave the cell empty rather
                                        // than shifting the whole row left.
                                        None => {
                                            ui.label(widgets::cell("").small());
                                        }
                                    }
                                }
                                ui.end_row();
                            }
                        });
                });
        });
    }
}

/// One row of cards per group, wrapping to the panel width.
fn cards(ui: &mut egui::Ui, group: &str, entries: &[(String, proto::GuiValue)]) {
    ui.horizontal_wrapped(|ui| {
        for (name, value) in entries {
            card(ui, group, name, value);
        }
    });
}

/// Width of one card. Fixed so the cards line up in a grid as they wrap, instead
/// of each one sizing to its own number and the layout reflowing every time a
/// counter gains a digit.
const CARD_WIDTH: f32 = 132.0;

fn card(ui: &mut egui::Ui, group: &str, name: &str, value: &proto::GuiValue) {
    egui::Frame::new()
        .fill(ui.visuals().extreme_bg_color)
        .inner_margin(6.0)
        .corner_radius(4.0)
        .show(ui, |ui| {
            ui.set_width(CARD_WIDTH);
            ui.vertical(|ui| {
                ui.add(
                    egui::Label::new(egui::RichText::new(title_case(name)).weak().small())
                        .truncate(),
                )
                .on_hover_text(format!("{PREFIX}{group}_{name}"));
                ui.add(egui::Label::new(figure_text(name, value)).truncate())
                    .on_hover_text(raw_hover(name, value));
            });
        });
}

/// The value as it should read on a card or in a cell.
///
/// Formatting is driven by the key's suffix rather than by a per-key table, so
/// a publisher gets durations and timestamps rendered properly just by naming
/// its variables to the convention.
fn figure_text(name: &str, value: &proto::GuiValue) -> egui::RichText {
    if value.is_unknown() {
        return widgets::cell("");
    }
    match value {
        proto::GuiValue::Int64(proto::IntOrUnknown::Int64(i)) if name.ends_with("_ms") => {
            egui::RichText::new(duration(*i)).monospace()
        }
        proto::GuiValue::Int64(proto::IntOrUnknown::Int64(i)) => {
            egui::RichText::new(i.to_string()).monospace()
        }
        proto::GuiValue::Float64(proto::FloatOrUnknown::Float64(f)) => {
            egui::RichText::new(format!("{f:.1}")).monospace()
        }
        proto::GuiValue::Time(proto::TimeOrUnknown::Time(t)) => {
            egui::RichText::new(clock(t)).monospace()
        }
        proto::GuiValue::Bool(proto::BoolOrUnknown::Bool(b)) => {
            // A `partial` or `failed` flag being true is the interesting case, so
            // colour it rather than leaving it to be read as ordinary text.
            let text = egui::RichText::new(if *b { "yes" } else { "no" }).monospace();
            if *b { text.color(widgets::WARN) } else { text }
        }
        proto::GuiValue::String(proto::StringOrUnknown::String(s)) => widgets::cell(s),
        other => egui::RichText::new(other.display()).monospace(),
    }
}

/// The unrounded value, for the hover. A rate shown as `6.0` and a cycle time
/// shown as `4m 12s` both hide something somebody will eventually want.
fn raw_hover(name: &str, value: &proto::GuiValue) -> String {
    match value {
        proto::GuiValue::Int64(proto::IntOrUnknown::Int64(i)) if name.ends_with("_ms") => {
            format!("{i} ms")
        }
        proto::GuiValue::Float64(proto::FloatOrUnknown::Float64(f)) => f.to_string(),
        proto::GuiValue::Time(proto::TimeOrUnknown::Time(t)) => full_clock(t),
        other if other.is_unknown() => {
            "Not published yet: no process has written this variable.".to_string()
        }
        other => other.display(),
    }
}

/// `812ms` / `12.3s` / `4m 12s` / `1h 04m 12s`.
fn duration(ms: i64) -> String {
    if ms < 0 {
        return "—".to_string();
    }
    let total_s = ms / 1000;
    if total_s < 1 {
        return format!("{ms}ms");
    }
    if total_s < 60 {
        return format!("{:.1}s", ms as f64 / 1000.0);
    }
    let (h, m, s) = (total_s / 3600, (total_s % 3600) / 60, total_s % 60);
    if h > 0 {
        format!("{h}h {m:02}m {s:02}s")
    } else {
        format!("{m}m {s:02}s")
    }
}

/// Wall-clock time only. The date is on the hover: these are all from the last
/// few hours, so the time of day is the part that carries information.
fn clock(t: &proto::GuiSystemTime) -> String {
    match datetime(t) {
        Some(dt) => dt.format("%H:%M:%S").to_string(),
        None => "—".to_string(),
    }
}

fn full_clock(t: &proto::GuiSystemTime) -> String {
    match datetime(t) {
        Some(dt) => dt.format("%Y-%m-%d %H:%M:%S%.3f %:z").to_string(),
        None => format!("{}s since the epoch", t.secs_since_epoch),
    }
}

/// In the browser `chrono::Local` is the viewer's timezone (chrono's `wasmbind`
/// feature), so this reads the same in the container's GUI and in a dev shell.
fn datetime(t: &proto::GuiSystemTime) -> Option<chrono::DateTime<chrono::Local>> {
    let ms = i64::try_from(t.secs_since_epoch).ok()?.checked_mul(1000)?
        + (t.nanos_since_epoch / 1_000_000) as i64;
    chrono::DateTime::from_timestamp_millis(ms).map(|dt| dt.with_timezone(&chrono::Local))
}

/// `boxes_780` -> `Boxes 780`, `pallets_per_hour` -> `Pallets per hour`.
///
/// Only the first word is capitalised: `Cycle Last Ms` reads like a spreadsheet
/// column, `Cycle last` reads like a label. The `_ms` suffix is dropped because
/// the formatting already says it is a duration.
fn title_case(name: &str) -> String {
    let trimmed = name.strip_suffix("_ms").unwrap_or(name);
    let trimmed = trimmed.strip_suffix("_at").unwrap_or(trimmed);
    let spaced = trimmed.replace('_', " ");
    let mut chars = spaced.chars();
    match chars.next() {
        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
        None => name.to_string(),
    }
}

/// Every `kpi_` variable, bucketed by group and sorted by name inside it.
///
/// A key that is only `kpi_<something>` with no group goes into a `""` group,
/// which sorts last and is still visible - better than silently dropping a
/// figure somebody published.
fn collect(api: &Api) -> BTreeMap<String, Vec<(String, proto::GuiValue)>> {
    let mut out: BTreeMap<String, Vec<(String, proto::GuiValue)>> = BTreeMap::new();
    for (key, entry) in &api.entries {
        let Some(rest) = key.strip_prefix(PREFIX) else { continue };
        let Some(value) = entry.value.as_ref() else { continue };
        let (group, name) = match rest.split_once('_') {
            Some((group, name)) => (group.to_string(), name.to_string()),
            None => (String::new(), rest.to_string()),
        };
        out.entry(group).or_default().push((name, value.clone()));
    }
    for entries in out.values_mut() {
        entries.sort_by(|a, b| a.0.cmp(&b.0));
    }
    out
}

/// `GROUP_ORDER` first, then whatever else exists, alphabetically.
fn ordered_groups(
    figures: &BTreeMap<String, Vec<(String, proto::GuiValue)>>,
) -> Vec<String> {
    let mut ordered: Vec<String> = GROUP_ORDER
        .iter()
        .filter(|group| figures.contains_key(**group))
        .map(|group| group.to_string())
        .collect();
    for group in figures.keys() {
        if !GROUP_ORDER.contains(&group.as_str()) {
            ordered.push(group.clone());
        }
    }
    ordered
}

/// An array of maps as `Vec<row>`, each row `Vec<(key, value)>` in map order.
/// Anything that is not a map is skipped rather than rendered as a broken row.
fn rows_of(value: &proto::GuiValue) -> Vec<Vec<(String, proto::GuiValue)>> {
    let proto::GuiValue::Array(proto::ArrayOrUnknown::Array(items)) = value else {
        return Vec::new();
    };
    items
        .iter()
        .filter_map(|item| {
            let proto::GuiValue::Map(proto::MapOrUnknown::Map(pairs)) = item else {
                return None;
            };
            Some(
                pairs
                    .iter()
                    .map(|(k, v)| (map_key(k), v.clone()))
                    .collect::<Vec<_>>(),
            )
        })
        .collect()
}

/// Map keys are values too. A string key is the normal case and its quotes are
/// noise here, so strip them; anything else falls back to how it displays.
fn map_key(key: &proto::GuiValue) -> String {
    match key {
        proto::GuiValue::String(proto::StringOrUnknown::String(s)) => s.clone(),
        other => other.display(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn int(i: i64) -> proto::GuiValue {
        proto::GuiValue::Int64(proto::IntOrUnknown::Int64(i))
    }

    fn text(s: &str) -> proto::GuiValue {
        proto::GuiValue::String(proto::StringOrUnknown::String(s.to_string()))
    }

    #[test]
    fn durations_read_as_durations() {
        assert_eq!(duration(0), "0ms");
        assert_eq!(duration(812), "812ms");
        assert_eq!(duration(12_340), "12.3s");
        assert_eq!(duration(252_000), "4m 12s");
        assert_eq!(duration(3_852_000), "1h 04m 12s");
        assert_eq!(duration(-1), "—");
    }

    #[test]
    fn labels_drop_the_unit_suffix_and_the_underscores() {
        assert_eq!(title_case("boxes_780"), "Boxes 780");
        assert_eq!(title_case("cycle_last_ms"), "Cycle last");
        assert_eq!(title_case("run_started_at"), "Run started");
        assert_eq!(title_case("pallets"), "Pallets");
    }

    #[test]
    fn groups_come_out_in_reading_order_with_extras_last() {
        let mut figures: BTreeMap<String, Vec<(String, proto::GuiValue)>> = BTreeMap::new();
        for group in ["total", "zebra", "current", "rate"] {
            figures.insert(group.to_string(), vec![("x".to_string(), int(1))]);
        }
        assert_eq!(
            ordered_groups(&figures),
            vec!["current", "rate", "total", "zebra"]
        );
    }

    #[test]
    fn a_key_without_a_group_is_still_shown() {
        let mut figures: BTreeMap<String, Vec<(String, proto::GuiValue)>> = BTreeMap::new();
        figures.insert(String::new(), vec![("orphan".to_string(), int(1))]);
        assert_eq!(ordered_groups(&figures), vec![""]);
    }

    #[test]
    fn rows_keep_the_column_order_the_record_was_built_in() {
        let record = proto::GuiValue::Map(proto::MapOrUnknown::Map(vec![
            (text("pallet"), int(1)),
            (text("box_type"), text("780")),
            (text("cycle_ms"), int(252_000)),
        ]));
        let array = proto::GuiValue::Array(proto::ArrayOrUnknown::Array(vec![record]));
        let rows = rows_of(&array);

        assert_eq!(rows.len(), 1);
        let columns: Vec<&str> = rows[0].iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(columns, vec!["pallet", "box_type", "cycle_ms"]);
    }

    #[test]
    fn a_non_array_or_a_non_map_entry_yields_no_rows() {
        assert!(rows_of(&int(3)).is_empty());
        assert!(rows_of(&proto::GuiValue::Array(proto::ArrayOrUnknown::UNKNOWN)).is_empty());
        let junk = proto::GuiValue::Array(proto::ArrayOrUnknown::Array(vec![int(1)]));
        assert!(rows_of(&junk).is_empty());
    }
}
