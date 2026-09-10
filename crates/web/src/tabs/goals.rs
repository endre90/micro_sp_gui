//! Asking the system for something.
//!
//! A goal is a predicate the planner has to reach. It goes into
//! `{sp_id}_incoming_goals`; the goal runner drains that, merges it into
//! `scheduled_goals` by priority, and the planner works on the top one.
//!
//! The status panel here is shared with the Orders tab and is the confirmation
//! that a Send landed: the goal shows up in the queues that were read back out of
//! Redis, not just in a local list.

use crate::api::Api;
use crate::widgets;
use micro_sp_gui_protocol as proto;

struct GoalRow {
    id: String,
    predicate: String,
    priority: proto::GuiGoalPriority,
}

impl GoalRow {
    fn new(counter: usize) -> Self {
        Self {
            id: fresh_id(counter),
            predicate: "var:picked_silver_box == true".to_string(),
            priority: proto::GuiGoalPriority::Normal,
        }
    }
}

/// A readable, unique-enough id the operator can still overwrite.
///
/// Deliberately not random: `getrandom` on wasm is a needless dependency for
/// something whose only job is to be distinguishable in a queue.
pub fn fresh_id(counter: usize) -> String {
    format!("gui_{counter:03}")
}

pub struct GoalsTab {
    rows: Vec<GoalRow>,
    counter: usize,
    replace: bool,
}

impl Default for GoalsTab {
    fn default() -> Self {
        Self { rows: vec![GoalRow::new(1)], counter: 1, replace: false }
    }
}

impl GoalsTab {
    pub fn ui(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        egui::SidePanel::right("goal_status")
            .resizable(true)
            .default_width(440.0)
            .show_inside(ui, |ui| status_panel(ui, api));

        ui.heading("Goals");
        ui.label(
            egui::RichText::new(
                "A predicate over state variables, e.g. `var:picked_silver_box == true`. \
                 Names are resolved against the runner's state, so a goal naming a variable \
                 that does not exist fails in the planner, not here.",
            )
            .weak()
            .small(),
        );
        ui.separator();

        let mut remove: Option<usize> = None;
        egui::ScrollArea::vertical().id_salt("goal_rows").max_height(360.0).show(ui, |ui| {
            for (i, row) in self.rows.iter_mut().enumerate() {
                ui.horizontal_wrapped(|ui| {
                    ui.label(egui::RichText::new(format!("#{}", i + 1)).weak());
                    ui.add(
                        egui::TextEdit::singleline(&mut row.id)
                            .hint_text("id")
                            .desired_width(110.0),
                    );
                    ui.add(
                        egui::TextEdit::singleline(&mut row.predicate)
                            .hint_text("var:name == value")
                            .desired_width(340.0),
                    );
                    // A combo, not a free-text integer: the old tab parsed the
                    // text with `unwrap_or(0)`, so any typo silently became the
                    // highest priority.
                    egui::ComboBox::from_id_salt(("goal_priority", i))
                        .selected_text(row.priority.label())
                        .width(90.0)
                        .show_ui(ui, |ui| {
                            for option in proto::GuiGoalPriority::ALL {
                                ui.selectable_value(&mut row.priority, *option, option.label());
                            }
                        });
                    if ui.small_button("✖").on_hover_text("Remove this goal").clicked() {
                        remove = Some(i);
                    }
                });
                if row.predicate.trim().is_empty() {
                    ui.label(egui::RichText::new("predicate is empty").color(widgets::WARN));
                }
            }
        });
        if let Some(i) = remove {
            self.rows.remove(i);
        }

        ui.separator();
        ui.horizontal_wrapped(|ui| {
            if ui.button("➕ Add goal").clicked() {
                self.counter += 1;
                self.rows.push(GoalRow::new(self.counter));
            }

            let ready = api.sp_id.is_some()
                && !self.rows.is_empty()
                && self.rows.iter().all(|r| {
                    !r.id.trim().is_empty() && !r.predicate.trim().is_empty()
                });
            if ui
                .add_enabled(ready, egui::Button::new(egui::RichText::new("Send goals").strong()))
                .clicked()
            {
                let goals = self
                    .rows
                    .iter()
                    .map(|r| proto::GuiGoal {
                        id: r.id.trim().to_string(),
                        priority: r.priority,
                        predicate: r.predicate.trim().to_string(),
                    })
                    .collect();
                api.send_goals(proto::SendGoalsRequest {
                    sp_id: api.sp_id.clone().unwrap_or_default(),
                    goals,
                    replace: self.replace,
                });
            }

            ui.checkbox(&mut self.replace, "replace the queue").on_hover_text(
                "Off (the default) appends, keeping goals the runner has not picked up yet. \
                 On discards them first.",
            );

            if api.sp_id.is_none() {
                ui.label(
                    egui::RichText::new("no sp_id to send to").color(widgets::WARN),
                );
            }
        });
    }
}

/// The read-only goal and plan pipeline. Shared with the Orders tab.
pub fn status_panel(ui: &mut egui::Ui, api: &mut Api) {
    ui.heading("Pipeline");
    let Some(status) = api.goals_status().cloned() else {
        ui.label(
            egui::RichText::new(
                "No micro_sp runner found in Redis. Start one, or pass --sp-id to the server.",
            )
            .weak(),
        );
        return;
    };

    egui::ScrollArea::vertical().id_salt("pipeline").show(ui, |ui| {
        ui.label(egui::RichText::new("Current goal").strong());
        widgets::field(ui, "id", &status.current_goal_id);
        widgets::field(ui, "predicate", &status.current_goal_predicate);
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("state:").weak());
            let colour = match status.current_goal_state.as_str() {
                "completed" => widgets::OK,
                "executing" => widgets::WARN,
                "failed" => widgets::BAD,
                "cancelled" => widgets::WARN,
                _ => widgets::UNKNOWN,
            };
            ui.label(
                egui::RichText::new(&status.current_goal_state).color(colour).strong(),
            );
        });

        ui.separator();
        ui.label(egui::RichText::new("Planner").strong());
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("planner:").weak());
            let colour = match status.planner_state.as_str() {
                "found" | "ready" => widgets::OK,
                "not_found" => widgets::BAD,
                _ => widgets::UNKNOWN,
            };
            ui.label(egui::RichText::new(&status.planner_state).color(colour));
            ui.label(egui::RichText::new("plan:").weak());
            let colour = match status.plan_state.as_str() {
                "completed" => widgets::OK,
                "executing" => widgets::WARN,
                "failed" => widgets::BAD,
                _ => widgets::UNKNOWN,
            };
            ui.label(egui::RichText::new(&status.plan_state).color(colour));
        });
        if status.replan_counter > 0 {
            widgets::field(ui, "replans", &status.replan_counter.to_string());
        }

        if status.plan.is_empty() {
            ui.label(egui::RichText::new("(no plan)").weak());
        } else {
            for (i, step) in status.plan.iter().enumerate() {
                let current = i as i64 == status.plan_current_step;
                let mut text = egui::RichText::new(format!("{i}. {step}")).monospace();
                if current {
                    text = text.strong().color(widgets::WARN);
                }
                ui.label(text);
            }
        }

        ui.separator();
        queue(ui, "Scheduled", &status.scheduled);
        queue(ui, "Incoming (not yet picked up)", &status.incoming);

        if !status.undecodable.is_empty() {
            ui.separator();
            ui.label(egui::RichText::new("Malformed queue entries").color(widgets::BAD));
            for problem in &status.undecodable {
                ui.label(egui::RichText::new(problem).small().color(widgets::BAD));
            }
        }

        ui.separator();
        egui::CollapsingHeader::new("Runner information").default_open(true).show(ui, |ui| {
            for (label, text) in [
                ("goal runner", &status.goal_runner_information),
                ("scheduler", &status.goal_scheduler_information),
                ("planner", &status.planner_information),
                ("plan runner", &status.plan_runner_information),
            ] {
                widgets::field(ui, label, text);
            }
        });
    });
}

fn queue(ui: &mut egui::Ui, label: &str, goals: &[proto::GuiGoal]) {
    ui.label(egui::RichText::new(format!("{label} ({})", goals.len())).strong());
    if goals.is_empty() {
        ui.label(egui::RichText::new("(empty)").weak());
        return;
    }
    for goal in goals {
        ui.horizontal_wrapped(|ui| {
            ui.label(egui::RichText::new(&goal.id).monospace().weak());
            ui.label(egui::RichText::new(goal.priority.label()).small());
            ui.label(egui::RichText::new(&goal.predicate).monospace());
        });
    }
}
