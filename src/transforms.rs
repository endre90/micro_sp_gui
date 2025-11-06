use eframe::egui;

/// Holds all the state for the "Another" tab
pub struct TransformsTab {
    // You could add tab-specific state here
    // counter: i32,
}

impl TransformsTab {
    /// Create a new `AnotherTab` with default state
    pub fn new() -> Self {
        Self {
            // counter: 0,
        }
    }
    /// Draw the UI for the "Another" tab
    pub fn ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("This is Another Tab");
        ui.label("You can put completely different UI elements here.");
        ui.add_space(10.0);
        ui.label("For example, this could be a settings page, a log viewer, or another tool.");

        // Example of stateful widget
        // if ui.button("Click me").clicked() {
        //     self.counter += 1;
        // }
        // ui.label(format!("Counter: {}", self.counter));
    }
}