//! Ordering items, as a shortcut for typing the same goals by hand.
//!
//! Each non-zero count becomes one goal, `var:count_picked_{item} == {count}`,
//! and the whole order goes out as a single batch - the old native tab sent them
//! one at a time, so an order could land half-applied.

use crate::api::Api;
use crate::tabs::goals::status_panel;
use crate::widgets;
use micro_sp_gui_protocol as proto;

pub struct OrdersTab {
    counts: Vec<u32>,
    priority: proto::GuiGoalPriority,
    history: Vec<String>,
    order_counter: usize,
}

impl Default for OrdersTab {
    fn default() -> Self {
        Self {
            counts: vec![0; proto::ORDER_ITEMS.len()],
            priority: proto::GuiGoalPriority::Normal,
            history: Vec::new(),
            order_counter: 0,
        }
    }
}

impl OrdersTab {
    pub fn ui(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        egui::SidePanel::right("order_status")
            .resizable(true)
            .default_width(440.0)
            .show_inside(ui, |ui| status_panel(ui, api));

        ui.heading("Order handler");
        ui.label(
            egui::RichText::new(
                "Counts become goals on the corresponding count_picked_* variables.",
            )
            .weak()
            .small(),
        );
        ui.separator();

        egui::Grid::new("order_items").num_columns(4).spacing([12.0, 6.0]).show(ui, |ui| {
            for (i, item) in proto::ORDER_ITEMS.iter().enumerate() {
                ui.label(egui::RichText::new(*item).monospace());
                ui.add(egui::DragValue::new(&mut self.counts[i]).speed(0.2).range(0..=999));
                if ui.small_button("➕").clicked() {
                    self.counts[i] += 1;
                }
                // The goal each row will produce, so there is no guessing.
                if self.counts[i] > 0 {
                    ui.label(
                        egui::RichText::new(proto::order_predicate(item, self.counts[i]))
                            .monospace()
                            .weak()
                            .small(),
                    );
                } else {
                    ui.label("");
                }
                ui.end_row();
            }
        });

        ui.separator();
        let total: u32 = self.counts.iter().sum();
        ui.horizontal_wrapped(|ui| {
            egui::ComboBox::from_id_salt("order_priority")
                .selected_text(self.priority.label())
                .width(100.0)
                .show_ui(ui, |ui| {
                    for option in proto::GuiGoalPriority::ALL {
                        ui.selectable_value(&mut self.priority, *option, option.label());
                    }
                });

            let ready = total > 0 && api.sp_id.is_some();
            if ui
                .add_enabled(ready, egui::Button::new(egui::RichText::new("Send order").strong()))
                .clicked()
            {
                self.send(api);
            }
            if ui.button("Clear").clicked() {
                self.counts.iter_mut().for_each(|c| *c = 0);
            }
            ui.label(egui::RichText::new(format!("{total} item(s)")).weak());
            if api.sp_id.is_none() {
                ui.label(egui::RichText::new("no sp_id to send to").color(widgets::WARN));
            }
        });

        if !self.history.is_empty() {
            ui.separator();
            ui.label(egui::RichText::new("Sent this session").strong());
            egui::Frame::new()
                .fill(ui.visuals().extreme_bg_color)
                .inner_margin(6.0)
                .show(ui, |ui| {
                    egui::ScrollArea::vertical()
                        .id_salt("order_history")
                        .max_height(160.0)
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for line in &self.history {
                                ui.label(egui::RichText::new(line).monospace().small());
                            }
                        });
                });
        }
    }

    fn send(&mut self, api: &mut Api) {
        self.order_counter += 1;
        let goals: Vec<proto::GuiGoal> = proto::ORDER_ITEMS
            .iter()
            .enumerate()
            .filter(|(i, _)| self.counts[*i] > 0)
            .map(|(i, item)| proto::GuiGoal {
                // Scoped to the order so two orders for the same item do not
                // collide in the queue.
                id: format!("order_{:03}_{item}", self.order_counter),
                priority: self.priority,
                predicate: proto::order_predicate(item, self.counts[i]),
            })
            .collect();

        for goal in &goals {
            self.history.push(format!("{} · {}", goal.id, goal.predicate));
        }

        api.send_goals(proto::SendGoalsRequest {
            sp_id: api.sp_id.clone().unwrap_or_default(),
            goals,
            replace: false,
        });
        self.counts.iter_mut().for_each(|c| *c = 0);
    }
}
