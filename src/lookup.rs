use eframe::egui;
use micro_sp::{ConnectionManager, SPTransform, SPTransformStamped, TransformsManager};
use poll_promise::Promise;
use serde::Serialize;
use std::{collections::HashMap, sync::Arc};

// --- Helper Functions (moved from lib.rs) ---

#[derive(Serialize)]
struct JsonOutputTransform {
    child_frame_id: String,
    parent_frame_id: String,
    transform: SPTransform,
}

async fn get_all_transforms(con: Arc<ConnectionManager>) -> HashMap<String, SPTransformStamped> {
    let mut connection = con.get_connection().await;
    match TransformsManager::get_all_transforms(&mut connection).await {
        Ok(tfs) => tfs,
        Err(e) => {
            log::error!("GUI Failed to get all transforms with: {e}!");
            HashMap::new()
        }
    }
}

async fn lookup_transform(
    con: Arc<ConnectionManager>,
    parent: &str,
    child: &str,
) -> Result<SPTransformStamped, String> {
    let mut connection = con.get_connection().await;
    match TransformsManager::lookup_transform(&mut connection, parent, child).await {
        Ok(tfs) => Ok(tfs),
        Err(e) => {
            log::error!("GUI Failed to lookup transform with: {e}!");
            Err(format!("GUI Failed to lookup transform with: {e}"))
        }
    }
}

pub struct TransformsTab {
    get_all_transforms_promise: Option<Promise<HashMap<String, SPTransformStamped>>>,
    transform_keys: Vec<String>,
    parent: Option<String>,
    child: Option<String>,
    lookup_promise: Option<Promise<Result<SPTransformStamped, String>>>,
    lookup_result_json: Option<String>,
    lookup_error: Option<String>,
}

impl TransformsTab {
    pub fn new() -> Self {
        Self {
            get_all_transforms_promise: None,
            transform_keys: Vec::new(),
            parent: None,
            child: None,
            lookup_promise: None,
            lookup_result_json: None,
            lookup_error: None,
        }
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        ui.heading("Transforms Lookup GUI");
        ui.separator();

        // --- Window 1: Controls (like MyApp's input frame) ---
        egui::Frame::default()
            .inner_margin(egui::Margin::same(10))
            .stroke(egui::Stroke::new(1.0, egui::Color32::DARK_GRAY))
            .show(ui, |ui| {
                ui.set_min_width(300.0); // Give it some base width

                // --- Fetching Transforms ---
                ui.horizontal(|ui| {
                    let is_fetching_list = self.poll_transforms_promise(ui);
                    if !is_fetching_list && ui.button("Fetch Transforms").clicked() {
                        self.spawn_transforms_promise(handle, connection);
                    }
                    if is_fetching_list {
                        ui.label("Loading data...");
                    }
                });

                ui.separator();

                // --- Selectors ---
                draw_transform_selector(
                    ui,
                    "Parent:",
                    "parent_select",
                    &mut self.parent,
                    &self.transform_keys,
                );
                draw_transform_selector(
                    ui,
                    "Child:",
                    "child_select",
                    &mut self.child,
                    &self.transform_keys,
                );

                ui.add_space(10.0);

                // --- Lookup Controls (moved from draw_lookup_section) ---
                let both_selected = self.parent.is_some() && self.child.is_some();
                let is_loading_lookup = self.lookup_promise.is_some();

                ui.horizontal(|ui| {
                    ui.add_enabled_ui(both_selected && !is_loading_lookup, |ui| {
                        if ui.button("Lookup").clicked() {
                            self.spawn_lookup_promise(handle, connection);
                        }
                    });

                    if is_loading_lookup {
                        ui.spinner();
                    }
                });
            });

        // Poll the lookup promise *after* drawing the controls
        if self.lookup_promise.is_some() {
            self.poll_lookup_promise();
        }

        ui.add_space(10.0);

        // --- Window 2: Output (like MyApp's solution section) ---
        egui::Frame::default()
            .inner_margin(egui::Margin::same(0))
            .stroke(egui::Stroke::new(1.0, egui::Color32::DARK_GRAY))
            .show(ui, |ui| {
                self.draw_output_section(ui);
            });
    }

    /// Draws the output section (JSON result or error)
    fn draw_output_section(&mut self, ui: &mut egui::Ui) {
        ui.set_min_width(480.0); // Match control panel
        ui.set_min_height(240.0); // Give output area some space

         if let Some(error) = &self.lookup_error {
            ui.colored_label(egui::Color32::RED, format!("Error: {}", error));
        } else if let Some(json_string) = &mut self.lookup_result_json {
            // ui.label("Resulting JSON:");
            // Use ScrollArea like in MyApp
            egui::ScrollArea::both()
                .id_salt("lookup_json_scroll_area")
                .auto_shrink([false; 2]) // Don't shrink
                // .max_height(300.0)
                .show(ui, |ui| {
                    ui.add(
                        egui::TextEdit::multiline(json_string)
                            .font(egui::FontId::monospace(12.0))
                            .desired_width(f32::INFINITY),
                    );
                });
        } else {
            ui.label("\n    Result will appear here.");
        }

        // if let Some(error) = &self.lookup_error {
        //     ui.colored_label(egui::Color32::RED, format!("Error: {}", error));
        // } else if let Some(json_string) = &mut self.lookup_result_json {
        //     // ui.label("Resulting JSON:");
        //     // Use ScrollArea like in MyApp
        //     egui::ScrollArea::both()
        //         .id_salt("lookup_json_scroll_area")
        //         .auto_shrink([false; 2]) // Don't shrink
        //         // .max_height(300.0)
        //         .show(ui, |ui| {
        //             ui.add(
        //                 egui::TextEdit::multiline(json_string)
        //                     .font(egui::FontId::monospace(12.0))
        //                     .desired_width(f32::INFINITY),
        //             );
        //         });
        // }
    }

    fn spawn_lookup_promise(
        &mut self,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        self.lookup_result_json = None;
        self.lookup_error = None;

        if let (Some(parent), Some(child)) = (self.parent.clone(), self.child.clone()) {
            let handle = handle.clone();
            let con_clone = connection.clone();
            self.lookup_promise = Some(Promise::spawn_thread("lookup_fetcher", move || {
                handle.block_on(lookup_transform(con_clone, &parent, &child))
            }));
        }
    }

    fn poll_lookup_promise(&mut self) {
        if let Some(promise) = &self.lookup_promise {
            if let std::task::Poll::Ready(result) = promise.poll() {
                match result {
                    Ok(transform) => {
                        let output = JsonOutputTransform {
                            child_frame_id: self.child.clone().unwrap_or_default(),
                            parent_frame_id: self.parent.clone().unwrap_or_default(),
                            transform: transform.transform.clone(),
                        };

                        match serde_json::to_string_pretty(&output) {
                            Ok(json_string) => self.lookup_result_json = Some(json_string),
                            Err(e) => {
                                self.lookup_error = Some(format!("JSON serialization error: {}", e))
                            }
                        }
                    }
                    Err(err) => self.lookup_error = Some(err.clone()),
                }
                self.lookup_promise = None;
            }
        }
    }

    /// Polls the transforms promise.
    /// Returns true if the promise is still pending, false otherwise.
    fn poll_transforms_promise(&mut self, ui: &mut egui::Ui) -> bool {
        let Some(promise) = self.get_all_transforms_promise.take() else {
            return false;
        };

        match promise.poll() {
            std::task::Poll::Ready(result) => {
                self.process_transforms_result(result);
                false
            }
            std::task::Poll::Pending => {
                self.get_all_transforms_promise = Some(promise);
                ui.spinner();
                true
            }
        }
    }

    fn process_transforms_result(&mut self, result: &HashMap<String, SPTransformStamped>) {
        let mut keys: Vec<String> = result.keys().cloned().collect();
        keys.sort_unstable();
        self.transform_keys = keys;

        if let Some(parent) = &self.parent {
            if !self.transform_keys.contains(parent) {
                self.parent = None;
            }
        }
        if let Some(child) = &self.child {
            if !self.transform_keys.contains(child) {
                self.child = None;
            }
        }
    }

    fn spawn_transforms_promise(
        &mut self,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        let handle = handle.clone();
        let con_clone = connection.clone();
        self.get_all_transforms_promise = Some(Promise::spawn_thread("fetcher", move || {
            handle.block_on(get_all_transforms(con_clone))
        }));
    }
}

fn draw_transform_selector(
    ui: &mut egui::Ui,
    label_text: &str,
    id_source: &str,
    selection: &mut Option<String>,
    keys: &[String],
) {
    ui.horizontal(|ui| {
        ui.label(label_text);
        let selected_text = selection.as_deref().unwrap_or("Select...");

        egui::ComboBox::from_id_salt(id_source)
            .selected_text(selected_text)
            .show_ui(ui, |ui| {
                ui.selectable_value(selection, None, "None");
                for key in keys {
                    ui.selectable_value(selection, Some(key.clone()), key);
                }
            });
    });
}