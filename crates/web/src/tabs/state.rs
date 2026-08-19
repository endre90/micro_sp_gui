//! Viewing and editing the whole Redis state.
//!
//! Replaces both the Streamlit "View State"/"Set State"/"Details" tabs and gives
//! the same job a typed editor: every value keeps its `SPValue` type, so a
//! `Float64` cannot be turned into a string by accident.
//!
//! Edits are staged rather than applied per keystroke - dragging a number would
//! otherwise write to Redis on every frame - and go out as one batch.

use crate::api::Api;
use crate::widgets::{value_detail, value_editor};
use micro_sp_gui_protocol as proto;
use std::collections::BTreeMap;

#[derive(Default)]
pub struct StateTab {
    filter: String,
    use_regex: bool,
    hide_unknown: bool,
    only_unparsed: bool,
    selected: Option<String>,
    /// Staged edits, by key. Cleared on Apply or Revert.
    drafts: BTreeMap<String, proto::GuiValue>,
    new_key: proto::NewKeyDraft,
    show_new_key: bool,
}

impl StateTab {
    pub fn ui(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        self.filter_row(ui, api);
        ui.separator();

        let matching = self.matching_keys(api);

        egui::SidePanel::right("state_detail")
            .resizable(true)
            .default_width(460.0)
            .show_inside(ui, |ui| self.detail_panel(ui, api));

        self.table(ui, api, &matching);
    }

    fn filter_row(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.horizontal_wrapped(|ui| {
            ui.label("Filter");
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .hint_text("key or value")
                    .desired_width(240.0),
            );
            ui.checkbox(&mut self.use_regex, "regex")
                .on_hover_text("Match keys and values with a regular expression.");
            if self.use_regex && !self.filter.is_empty() && self.compiled().is_none() {
                ui.label(egui::RichText::new("invalid regex").color(crate::widgets::WARN));
            }
            ui.separator();
            ui.checkbox(&mut self.hide_unknown, "hide UNKNOWN");
            ui.checkbox(&mut self.only_unparsed, "only unparsed")
                .on_hover_text("Values Redis holds that are not valid SPValue JSON.");
            ui.separator();
            ui.label(
                egui::RichText::new(format!("{} keys", api.entries.len())).weak(),
            );
            if !self.drafts.is_empty() {
                ui.separator();
                ui.label(
                    egui::RichText::new(format!("{} staged", self.drafts.len()))
                        .color(crate::widgets::WARN),
                );
                if ui.button("Apply all").clicked() {
                    self.apply(api);
                }
                if ui.button("Revert all").clicked() {
                    self.drafts.clear();
                }
            }
        });
    }

    fn compiled(&self) -> Option<regex::Regex> {
        if !self.use_regex || self.filter.is_empty() {
            return None;
        }
        regex::RegexBuilder::new(&self.filter).case_insensitive(true).build().ok()
    }

    fn matching_keys(&self, api: &Api) -> Vec<String> {
        let re = self.compiled();
        let needle = self.filter.to_lowercase();
        api.entries
            .values()
            .filter(|entry| {
                if self.only_unparsed && entry.value.is_some() {
                    return false;
                }
                if self.hide_unknown
                    && entry.value.as_ref().map(|v| v.is_unknown()).unwrap_or(false)
                {
                    return false;
                }
                if self.filter.is_empty() {
                    return true;
                }
                match &re {
                    Some(re) => re.is_match(&entry.key) || re.is_match(&entry.display()),
                    None => {
                        entry.key.to_lowercase().contains(&needle)
                            || entry
                                .value
                                .as_ref()
                                .map(|v| v.contains_text(&needle))
                                .unwrap_or(false)
                    }
                }
            })
            .map(|entry| entry.key.clone())
            .collect()
    }

    fn table(&mut self, ui: &mut egui::Ui, api: &mut Api, keys: &[String]) {
        use egui_extras::{Column, TableBuilder};

        let row_height = ui.text_style_height(&egui::TextStyle::Monospace) + 4.0;
        TableBuilder::new(ui)
            .striped(true)
            .resizable(true)
            .column(Column::auto().at_least(220.0).clip(true))
            .column(Column::auto().at_least(70.0))
            .column(Column::remainder().clip(true))
            .header(20.0, |mut header| {
                header.col(|ui| {
                    ui.strong("Key");
                });
                header.col(|ui| {
                    ui.strong("Type");
                });
                header.col(|ui| {
                    ui.strong("Value");
                });
            })
            .body(|body| {
                // Virtualised: only visible rows are built, so a keyspace with
                // thousands of variables still scrolls smoothly.
                body.rows(row_height, keys.len(), |mut row| {
                    let key = &keys[row.index()];
                    let Some(entry) = api.entries.get(key) else { return };
                    let staged = self.drafts.contains_key(key);
                    let selected = self.selected.as_deref() == Some(key.as_str());
                    row.set_selected(selected);

                    row.col(|ui| {
                        let mut text = egui::RichText::new(key).monospace();
                        if staged {
                            text = text.color(crate::widgets::WARN);
                        }
                        if ui.selectable_label(selected, text).clicked() {
                            self.selected = Some(key.clone());
                        }
                    });
                    row.col(|ui| {
                        let label = entry.type_label();
                        let colour = if entry.value.is_none() {
                            crate::widgets::BAD
                        } else {
                            ui.visuals().weak_text_color()
                        };
                        ui.label(egui::RichText::new(label).monospace().color(colour));
                    });
                    row.col(|ui| {
                        let unknown =
                            entry.value.as_ref().map(|v| v.is_unknown()).unwrap_or(false);
                        let colour = if unknown {
                            crate::widgets::UNKNOWN
                        } else {
                            ui.visuals().text_color()
                        };
                        ui.label(egui::RichText::new(entry.display()).monospace().color(colour));
                    });
                });
            });
    }

    fn detail_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.horizontal(|ui| {
            ui.heading("Details");
            if ui.button("➕ New key").clicked() {
                self.show_new_key = !self.show_new_key;
            }
        });

        if self.show_new_key {
            self.new_key_form(ui, api);
            ui.separator();
        }

        let Some(key) = self.selected.clone() else {
            ui.label(egui::RichText::new("Select a key on the left.").weak());
            return;
        };
        let Some(entry) = api.entries.get(&key).cloned() else {
            ui.label(egui::RichText::new(format!("'{key}' is no longer in Redis.")).weak());
            return;
        };

        ui.label(egui::RichText::new(&key).monospace().strong());

        egui::ScrollArea::vertical().show(ui, |ui| {
            match &entry.value {
                Some(value) => {
                    egui::CollapsingHeader::new("Value")
                        .default_open(true)
                        .show(ui, |ui| value_detail(ui, value));

                    ui.separator();
                    ui.label(egui::RichText::new("Edit").strong());

                    // Staging starts from what Redis currently holds. The draft
                    // is edited as an owned value and written back at the end, so
                    // the buttons below are free to touch `self.drafts`.
                    let mut draft =
                        self.drafts.get(&key).cloned().unwrap_or_else(|| value.clone());
                    value_editor(ui, egui::Id::new(("editor", &key)), &mut draft);
                    let dirty = &draft != value;

                    let mut forget = !dirty;
                    ui.horizontal(|ui| {
                        if ui
                            .add_enabled(dirty, egui::Button::new("Apply"))
                            .on_hover_text("Write this key to Redis.")
                            .clicked()
                        {
                            api.set_values(vec![proto::SetValue {
                                key: key.clone(),
                                value: draft.clone(),
                            }]);
                            forget = true;
                        }
                        if ui.add_enabled(dirty, egui::Button::new("Revert")).clicked() {
                            forget = true;
                        }
                        if ui
                            .button(egui::RichText::new("Delete key").color(crate::widgets::BAD))
                            .on_hover_text(
                                "Removes the variable. A runner that reads a variable which \
                                 is not in the state panics, so only delete what nothing uses.",
                            )
                            .clicked()
                        {
                            api.delete_keys(vec![key.clone()]);
                            forget = true;
                            self.selected = None;
                        }
                    });

                    if forget {
                        self.drafts.remove(&key);
                    } else {
                        self.drafts.insert(key.clone(), draft);
                    }
                }
                None => {
                    ui.label(
                        egui::RichText::new(
                            "This value is not valid SPValue JSON, so it cannot be edited \
                             as a typed value. The raw text is below.",
                        )
                        .color(crate::widgets::BAD),
                    );
                    if ui.button("Delete key").clicked() {
                        api.delete_keys(vec![key.clone()]);
                        self.selected = None;
                    }
                }
            }

            ui.separator();
            egui::CollapsingHeader::new("Raw JSON in Redis").show(ui, |ui| {
                let mut raw = entry.raw.clone();
                ui.add(
                    egui::TextEdit::multiline(&mut raw)
                        .code_editor()
                        .desired_rows(4)
                        .desired_width(f32::INFINITY)
                        .interactive(false),
                );
                if ui.button("Copy").clicked() {
                    ui.ctx().copy_text(entry.raw.clone());
                }
            });
        });
    }

    fn new_key_form(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.add(
                    egui::TextEdit::singleline(&mut self.new_key.key)
                        .hint_text("sp_my_variable")
                        .desired_width(200.0),
                );
                egui::ComboBox::from_id_salt("new_key_type")
                    .selected_text(self.new_key.value_type.label())
                    .show_ui(ui, |ui| {
                        for t in proto::GuiValueType::ALL {
                            ui.selectable_value(&mut self.new_key.value_type, *t, t.label());
                        }
                    });
                let valid = !self.new_key.key.trim().is_empty();
                if ui.add_enabled(valid, egui::Button::new("Create")).clicked() {
                    let key = self.new_key.key.trim().to_string();
                    // Created as UNKNOWN, which is how micro_sp seeds every
                    // variable - it exists and has a type, nothing has set it.
                    let value = proto::GuiValue::unknown_of(self.new_key.value_type);
                    api.set_values(vec![proto::SetValue { key: key.clone(), value }]);
                    self.selected = Some(key);
                    self.new_key.key.clear();
                    self.show_new_key = false;
                }
            });
            ui.label(
                egui::RichText::new(
                    "New variables are created as UNKNOWN of the chosen type.",
                )
                .weak()
                .small(),
            );
        });
    }

    fn apply(&mut self, api: &mut Api) {
        let values: Vec<proto::SetValue> = self
            .drafts
            .iter()
            .map(|(key, value)| proto::SetValue { key: key.clone(), value: value.clone() })
            .collect();
        if !values.is_empty() {
            api.set_values(values);
        }
        self.drafts.clear();
    }
}
