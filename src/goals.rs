use eframe::egui;
use std::sync::Arc;
use micro_sp::*;
use nanoid;

/// Holds the temporary state of a single goal row in the UI
struct GoalInput {
    id: String,
    predicate: String,
    priority: String, // String because text inputs edit strings
}

impl Default for GoalInput {
    fn default() -> Self {
        Self {
            // We pre-fill a random ID for convenience, but the user can edit it
            id: nanoid::nanoid!(10), 
            predicate: "var:picked_silver_box == true".to_string(),
            priority: "2".to_string(),
        }
    }
}

pub struct GoalsTab {
    sp_id: String,
    /// The list of goals currently being edited
    goals: Vec<GoalInput>,
    /// Status message to show result of "Send" (optional)
    status_msg: Option<String>,
}

impl GoalsTab {
    pub fn new(sp_id: &str) -> Self {
        Self {
            sp_id: sp_id.to_string(),
            goals: vec![GoalInput::default()], // Start with one empty row
            status_msg: None,
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, handle: &tokio::runtime::Handle, connection: &Arc<ConnectionManager>) {
        ui.heading("Goal Setter");
        ui.separator();

        // 1. Render the list of inputs
        // We use a scroll area in case there are many goals
        egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
            let mut index_to_remove = None;

            for (i, goal) in self.goals.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("#{}:", i + 1));
                    
                    // ID Input
                    ui.vertical(|ui| {
                        ui.label("ID");
                        ui.text_edit_singleline(&mut goal.id) //.hint_text("Goal ID");
                    });

                    // Predicate Input
                    ui.vertical(|ui| {
                        ui.label("Predicate");
                        ui.text_edit_singleline(&mut goal.predicate) // .hint_text("e.g. (robot_at pos)");
                    });

                    // Priority Input
                    ui.vertical(|ui| {
                        ui.label("Priority");
                        ui.text_edit_singleline(&mut goal.priority) // .hint_text("0");
                    });

                    // Delete Button
                    ui.add_space(10.0);
                    if ui.button("❌").on_hover_text("Remove this goal").clicked() {
                        index_to_remove = Some(i);
                    }
                });
                ui.separator();
            }

            // Handle deletion outside the loop to avoid iterator invalidation
            if let Some(i) = index_to_remove {
                self.goals.remove(i);
            }
        });

        ui.add_space(10.0);

        // 2. Control Buttons
        ui.horizontal(|ui| {
            // The "+" button adds a new set of text boxes
            if ui.button("➕ Add Goal").clicked() {
                self.goals.push(GoalInput::default());
            }

            // The SEND button
            if ui.button("Send Goals").clicked() {
                self.send_goals(handle, connection);
            }
        });

        if let Some(msg) = &self.status_msg {
            ui.label(msg);
        }
    }

    fn send_goals(&mut self, handle: &tokio::runtime::Handle, connection: &Arc<ConnectionManager>) {
        // Convert UI state (Vec<GoalInput>) to SPValue
        let mut goal_list = Vec::new();

        for goal in &self.goals {
            // 1. Wrap ID
            let id_val = SPValue::String(StringOrUnknown::String(goal.id.clone()));
            
            // 2. Wrap Priority (Parse string to i64, default to 0 if invalid)
            let prio_int = goal.priority.parse::<i64>().unwrap_or(0);
            let priority_val = SPValue::Int64(IntOrUnknown::Int64(prio_int));

            // 3. Wrap Predicate
            let predicate_val = SPValue::String(StringOrUnknown::String(goal.predicate.clone()));

            // Combine into [id, priority, predicate]
            let single_goal = SPValue::Array(ArrayOrUnknown::Array(vec![
                id_val,
                priority_val,
                predicate_val,
            ]));

            goal_list.push(single_goal);
        }

        // Wrap the whole list into the outer Array
        let payload = SPValue::Array(ArrayOrUnknown::Array(goal_list));

        // Clone Arc to move into async block
        let con = connection.clone();
        
        // Update UI status
        self.status_msg = Some("Sending...".to_string());
        let sp_id_clone = self.sp_id.clone();

        handle.spawn(async move {
            // Replace "incoming_goals" with the actual key you want to write to in Redis/SP
            // Assuming your ConnectionManager has a method like `set_value` or `publish`
            // If ConnectionManager uses a different method to set state, adapt this line:
            let mut connection = con.get_connection().await;
            StateManager::set_sp_value(&mut connection, &format!("{}_incoming_goals", sp_id_clone), &payload).await;
            // let result = conn.set_value("{sp_id}_incoming_goals", payload).await;
            
            // Note: You can't easily write back to `self.status_msg` from here without message passing
            // or Arc<Mutex<State>>, but for fire-and-forget this is sufficient.
            // match result {
            //     Ok(_) => println!("Goals sent successfully"),
            //     Err(e) => println!("Error sending goals: {:?}", e),
            // }
        });
    }
}
