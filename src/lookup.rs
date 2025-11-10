use eframe::egui;
use micro_sp::{
    ConnectionManager, FloatOrUnknown, SPTransform, SPTransformStamped, SPValue, StateManager,
    TransformsManager,
};
use ordered_float::OrderedFloat;
use poll_promise::Promise;
use rfd::FileDialog;
use serde::Serialize;
use std::{collections::HashMap, sync::Arc};

#[derive(Serialize)]
struct PreferredJointConfiguration(HashMap<String, f64>);

#[derive(Serialize)]
struct Metadata {
    tcp_id: String,
    preferred_joint_configuration: PreferredJointConfiguration,
    enable_transform: bool,
    active_transform: bool,
    gantry: f64,
}

#[derive(Serialize)]
struct JsonOutputWithMetadata {
    child_frame_id: String,
    parent_frame_id: String,
    transform: SPTransform,
    metadata: Metadata,
}

fn vec_to_joint_map(joints: Vec<f64>) -> PreferredJointConfiguration {
    let map = joints
        .into_iter()
        .enumerate()
        .map(|(i, val)| (format!("j{}", i), val))
        .collect::<HashMap<String, f64>>();
    PreferredJointConfiguration(map)
}

struct LookupData {
    transform: SPTransformStamped,
    joint_states: Vec<f64>,
    gantry_position: f64,
}

type LookupResult = Result<LookupData, String>;

async fn get_lookup_data(
    con: Arc<ConnectionManager>,
    robot_id: &str,
    parent: String,
    child: String,
) -> LookupResult {
    let (transform_res, joints_res, gantry_res) = tokio::join!(
        lookup_transform(con.clone(), &parent, &child),
        get_joint_states(con.clone(), &robot_id),
        get_opc_current_position(con.clone())
    );

    match transform_res {
        Ok(transform) => Ok(LookupData {
            transform,
            joint_states: joints_res,
            gantry_position: gantry_res,
        }),
        Err(e) => Err(e),
    }
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

async fn get_opc_current_position(con: Arc<ConnectionManager>) -> f64 {
    let mut connection = con.get_connection().await;
    match StateManager::get_sp_value(&mut connection, "opc_current_position").await {
        Some(sp_value) => {
            if let SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(x))) = sp_value {
                x
            } else {
                0.0
            }
        }
        None => 0.0,
    }
}

async fn get_joint_states(con: Arc<ConnectionManager>, robot_id: &str) -> Vec<f64> {
    let mut connection = con.get_connection().await;
    match StateManager::get_sp_value(&mut connection, &format!("{}_joint_states", robot_id)).await {
        Some(joint_states) => {
            if let SPValue::Array(micro_sp::ArrayOrUnknown::Array(joint_states_of_sp_value)) =
                joint_states
            {
                joint_states_of_sp_value
                    .iter()
                    .map(|j| {
                        if let SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(
                            joint_value,
                        ))) = j
                        {
                            joint_value.to_owned()
                        } else {
                            0.0
                        }
                    })
                    .collect::<Vec<f64>>()
            } else {
                vec![]
            }
        }
        None => {
            log::error!("GUI Failed to get joint states for robot {}", robot_id);
            vec![]
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

pub struct LookupTab {
    robot_id_input: String,
    get_all_transforms_promise: Option<Promise<HashMap<String, SPTransformStamped>>>,
    transform_keys: Vec<String>,
    parent: Option<String>,
    child: Option<String>,
    lookup_promise: Option<Promise<LookupResult>>,
    // lookup_result_json: Option<String>,
    lookup_output: Option<(JsonOutputWithMetadata, String)>,
    lookup_error: Option<String>,
}

impl LookupTab {
    pub fn new() -> Self {
        Self {
            robot_id_input: "r1".to_string(),
            get_all_transforms_promise: None,
            transform_keys: Vec::new(),
            parent: None,
            child: None,
            lookup_promise: None,
            // lookup_result_json: None,
            lookup_output: None,
            lookup_error: None,
        }
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        ui.horizontal(|ui| {
            ui.heading("Transforms Lookup GUI"); // This stays on the left

            // Add all right-aligned items here, in reverse order
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // 1. The Button (will be furthest right)

                // 2. The Text Box (will be to the left of the button)
                // We make it "small" by setting a desired_width.
                let text_box =
                    egui::TextEdit::singleline(&mut self.robot_id_input).desired_width(50.0); // Adjust width as needed
                ui.add(text_box);
                ui.label("Robot ID:");
                // 3. The Label (will be to the left of the text box)
            });
        });
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

        if self.lookup_output.is_some() {
            ui.horizontal(|ui| {
                // This layout pushes the button to the far right
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Save As").clicked() {
                        self.save_json_to_file();
                    }
                });
            });
            ui.add_space(2.0); // Small space between button and output box
        }

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
        // } else if let Some(json_string) = &mut self.lookup_result_json {
        } else if let Some((_, json_string)) = &mut self.lookup_output {
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
    }

    fn spawn_lookup_promise(
        &mut self,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        // self.lookup_result_json = None;
        self.lookup_output = None;
        self.lookup_error = None;

        if let (Some(parent), Some(child), robot_id_input) = (
            self.parent.clone(),
            self.child.clone(),
            self.robot_id_input.clone(),
        ) {
            let handle = handle.clone();
            let con_clone = connection.clone();
            self.lookup_promise = Some(Promise::spawn_thread("lookup_fetcher", move || {
                handle.block_on(get_lookup_data(con_clone, &robot_id_input, parent, child))
            }));
        }
    }

    // fn poll_lookup_promise(&mut self) {
    //     if let Some(promise) = &self.lookup_promise {
    //         if let std::task::Poll::Ready(result) = promise.poll() {
    //             match result {
    //                 Ok(data) => {
    //                     let child_frame_id = self.child.clone().unwrap_or_default();
    //                     let joint_config_map = vec_to_joint_map(data.joint_states.clone());

    //                     let output = JsonOutputWithMetadata {
    //                         child_frame_id: child_frame_id.clone(),
    //                         parent_frame_id: self.parent.clone().unwrap_or_default(),
    //                         transform: data.transform.transform.clone(),
    //                         metadata: Metadata {
    //                             tcp_id: child_frame_id,
    //                             preferred_joint_configuration: joint_config_map,
    //                             enable_transform: true,
    //                             active_transform: false,
    //                             gantry: data.gantry_position, // Use the fetched value here
    //                         },
    //                     };

    //                     match serde_json::to_string_pretty(&output) {
    //                         Ok(json_string) => self.lookup_result_json = Some(json_string),
    //                         Err(e) => {
    //                             self.lookup_error = Some(format!("JSON serialization error: {}", e))
    //                         }
    //                     }
    //                 }
    //                 Err(err) => self.lookup_error = Some(err.clone()),
    //             }
    //             self.lookup_promise = None;
    //         }
    //     }
    // }

    fn poll_lookup_promise(&mut self) {
        if let Some(promise) = &self.lookup_promise {
            if let std::task::Poll::Ready(result) = promise.poll() {
                match result {
                    Ok(data) => {
                        let child_frame_id = self.child.clone().unwrap_or_default();
                        let joint_config_map = vec_to_joint_map(data.joint_states.clone());

                        let output = JsonOutputWithMetadata {
                            // <--- We will store this
                            child_frame_id: child_frame_id.clone(),
                            parent_frame_id: self.parent.clone().unwrap_or_default(),
                            transform: data.transform.transform.clone(),
                            metadata: Metadata {
                                tcp_id: child_frame_id,
                                preferred_joint_configuration: joint_config_map,
                                enable_transform: true,
                                active_transform: false,
                                gantry: data.gantry_position,
                            },
                        };

                        match serde_json::to_string_pretty(&output) {
                            // OLD: Ok(json_string) => self.lookup_result_json = Some(json_string),
                            // NEW:
                            Ok(json_string) => self.lookup_output = Some((output, json_string)),
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

    fn save_json_to_file(&self) {
        // We use the data stored in self.lookup_output
        if let Some((output_data, json_content)) = &self.lookup_output {
            // Create a default filename like "parent_to_child.json"
            let default_filename = format!(
                "{}_to_{}.json",
                output_data.parent_frame_id, output_data.child_frame_id
            );

            // Open the native "Save File" dialog
            let file_path = FileDialog::new()
                .add_filter("JSON", &["json"])
                .set_file_name(&default_filename)
                .save_file();

            // If the user selected a path (didn't cancel)
            if let Some(path) = file_path {
                match std::fs::write(&path, json_content) {
                    Ok(_) => log::info!("Successfully saved JSON to {:?}", path),
                    Err(e) => log::error!("Failed to save file: {}", e),
                }
            }
        }
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
