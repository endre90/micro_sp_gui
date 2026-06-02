use std::sync::Arc;

use eframe::egui;
use micro_sp::{running::goal_runner::{GoalPriority, goal_string_to_sp_value}, *};

pub static ITEM_TYPES: usize = 7;

// #[derive(Debug, PartialEq)]
// enum OrderPriority {
//     Low,
//     Medium,
//     High,
// }

        

pub struct ItemOrderTab {
    sp_id: String,
    item_names: [&'static str; ITEM_TYPES],
    counts: [u32; ITEM_TYPES],
    // State for the priority dropdown
    priority: GoalPriority,
    // Store past orders as formatted strings
    order_history: Vec<String>,
    // Just to give orders a unique number
    order_counter: u32,
    // Status message
    last_action: Option<String>,
}

impl ItemOrderTab {
    pub fn new(sp_id: &str) -> Self {
        Self {
            sp_id: sp_id.to_string(),
            item_names: [
                "silver_box",
                "silver_gun",
                "silver_plate",
                "black_hose",
                "black_plate",
                "black_plug",
                "motor_valve",
            ],
            counts: [0; 7],
            priority: GoalPriority::Normal, // Default to Medium
            order_history: Vec::new(),
            order_counter: 0,
            last_action: Some("".to_string()),
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, connection: &Arc<ConnectionManager>) {
        ui.heading("Item Order Terminal");
        ui.separator();
        ui.add_space(10.0);

        // --- SECTION 1: ITEM GRID ---
        egui::Grid::new("item_grid")
            .striped(true)
            .min_col_width(100.0)
            .spacing([40.0, 15.0])
            .show(ui, |ui| {
                ui.label(egui::RichText::new("Item Name").strong());
                ui.label(egui::RichText::new("Current Qty").strong());
                ui.label(""); 
                ui.end_row();

                for i in 0..ITEM_TYPES {
                    ui.label(self.item_names[i]);
                    ui.label(egui::RichText::new(self.counts[i].to_string()).size(18.0));
                    
                    if ui.add(egui::Button::new("➕ Add").min_size(egui::vec2(60.0, 20.0))).clicked() {
                        self.counts[i] += 1;
                        self.last_action = Some("".to_string()); 
                    }
                    ui.end_row();
                }
            });

        ui.add_space(20.0);
        ui.separator();
        ui.add_space(10.0);

        // --- SECTION 2: CONTROLS (Priority + Send) ---
        ui.horizontal(|ui| {
            // Priority Selector
            ui.label("Priority:");
            egui::ComboBox::from_id_salt("priority_selector")
                .selected_text(format!("{:?}", self.priority))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.priority, GoalPriority::Low, "Low");
                    ui.selectable_value(&mut self.priority, GoalPriority::Normal, "Normal");
                    ui.selectable_value(&mut self.priority, GoalPriority::High, "High");
                });

            ui.add_space(20.0);

            // Send Button
            if ui.button(egui::RichText::new("Send Order").size(20.0)).clicked() {
                self.send_order(connection);
            }

            // Clear Button
            if ui.button("Clear").clicked() {
                self.counts = [0; ITEM_TYPES];
                self.last_action = Some("Selection cleared.".to_string());
            }
        });

        // Status Message
        if let Some(msg) = &self.last_action {
            ui.add_space(5.0);
            ui.label(egui::RichText::new(msg).color(egui::Color32::LIGHT_GREEN));
        }

        ui.add_space(20.0);

        // --- SECTION 3: ORDER HISTORY WINDOW ---
        ui.separator();
        ui.label(egui::RichText::new("Order History Log").strong());
        
        // We draw a framed box (Group) to look like a "window" or terminal output
        egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .max_height(200.0) // Limit height so it doesn't take over screen
                .auto_shrink([false; 2]) // Occupy full width
                .show(ui, |ui| {
                    ui.set_min_height(100.0); // Minimum height for the log box
                    
                    if self.order_history.is_empty() {
                        ui.label(egui::RichText::new("No orders sent yet.").weak().italics());
                    } else {
                        // Iterate in reverse to show newest at top
                        for log_entry in self.order_history.iter().rev() {
                            ui.label(log_entry);
                            ui.separator();
                        }
                    }
                });
        });
    }

    fn send_order(&mut self, connection: &Arc<ConnectionManager>) {
        // Check if empty
        let has_items = self.counts.iter().any(|&c| c > 0);
        if !has_items {
            self.last_action = Some("Cannot send empty order!".to_string());
            return;
        }

        self.order_counter += 1;

        // 1. Build the Receipt String
        // We use string building to create a nice summary for the log
        let mut receipt = format!(
            "Order #{} [{:?}] - {}:", 
            self.order_counter, 
            self.priority,
            chrono::Local::now().format("%H:%M:%S") // Requires `chrono`, or remove if you don't have it
        );

        // If you don't have chrono in Cargo.toml, just use:
        // let mut receipt = format!("Order #{} [{:?}]", self.order_counter, self.priority);

        for i in 0..ITEM_TYPES {
            if self.counts[i] > 0 {
                receipt.push_str(&format!("\n   • {}: {}", self.item_names[i], self.counts[i]));
            }
        }

        let counts = self.counts.clone(); 
        let item_names = self.item_names.clone();
        let priority = self.priority;
        let sp_id = self.sp_id.clone();
        let connection_handle = connection.clone();
        tokio::spawn(async move {
            let mut incoming_goals = vec!();
            for i in 0..ITEM_TYPES {
                if counts[i] > 0 {
                    let goal = format!("var:count_picked_{} == {}", item_names[i], counts[i]);
                    let uq_goal = goal_string_to_sp_value(&goal, priority);
                    incoming_goals.push(uq_goal);
                }
            }
            let mut con = connection_handle.get_connection().await;
            StateManager::set_sp_value(
                &mut con, 
                &format!("{}_incoming_goals", sp_id), 
                &incoming_goals.to_spvalue()
            ).await;
        });

        println!("{}", receipt);

        self.order_history.push(receipt);

        self.counts = [0; ITEM_TYPES];
        self.last_action = Some("Order sent successfully!".to_string());
    }
}