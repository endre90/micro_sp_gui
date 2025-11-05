use eframe::egui;
use micro_sp::{ConnectionManager, SPTransformStamped, TransformsManager};
use poll_promise::Promise;
use std::{collections::HashMap, sync::Arc};

#[derive(Debug, Clone, PartialEq)]
enum SavedPayload {
    Gripper,
    Svt,
    Bvt,
    Photoneo,
    Sponge,
    None,
}

impl std::fmt::Display for SavedPayload {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SavedPayload::Gripper => write!(f, "gripper"),
            SavedPayload::Svt => write!(f, "svt"),
            SavedPayload::Bvt => write!(f, "bvt"),
            SavedPayload::Photoneo => write!(f, "photoneo"),
            SavedPayload::Sponge => write!(f, "sponge"),
            SavedPayload::None => write!(f, "none"),
        }
    }
}

impl SavedPayload {
    fn variants() -> &'static [SavedPayload] {
        &[
            SavedPayload::None,
            SavedPayload::Gripper,
            SavedPayload::Svt,
            SavedPayload::Bvt,
            SavedPayload::Photoneo,
            SavedPayload::Sponge,
        ]
    }
}

/// Represents a manual payload configuration.
#[derive(Debug, Clone)]
pub struct Payload {
    /// Payload Mass in kilograms.
    pub mass: f32,
    /// Payload Center of Gravity offsets (in meters) from the tool mount.
    pub cog_x: f32,
    pub cog_y: f32,
    pub cog_z: f32,
    /// Payload Inertia Matrix (in kg*m^2) with origin at the CoG and axes aligned with the tool flange axes.
    pub ixx: f32,
    pub iyy: f32,
    pub izz: f32,
    pub ixy: f32,
    pub ixz: f32,
    pub iyz: f32,
}

impl Default for Payload {
    fn default() -> Self {
        Self {
            mass: 0.0,
            cog_x: 0.0,
            cog_y: 0.0,
            cog_z: 0.0,
            ixx: 0.0,
            iyy: 0.0,
            izz: 0.0,
            ixy: 0.0,
            ixz: 0.0,
            iyz: 0.0,
        }
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

// --- RobotTab Specific ---

#[derive(Debug, Clone, PartialEq)]
enum CommandType {
    MoveL,
    MoveJ,
    SafeMoveL,
    SafeMoveJ,
    PickVacuum,
    PlaceVacuum,
}

impl std::fmt::Display for CommandType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CommandType::MoveL => write!(f, "move_l"),
            CommandType::MoveJ => write!(f, "move_j"),
            CommandType::SafeMoveL => write!(f, "safe_move_l"),
            CommandType::SafeMoveJ => write!(f, "safe_move_j"),
            CommandType::PickVacuum => write!(f, "pick_vacuum"),
            CommandType::PlaceVacuum => write!(f, "place_vacuum"),
        }
    }
}

impl CommandType {
    fn variants() -> &'static [CommandType] {
        &[
            CommandType::MoveL,
            CommandType::MoveJ,
            CommandType::SafeMoveL,
            CommandType::SafeMoveJ,
            CommandType::PickVacuum,
            CommandType::PlaceVacuum,
        ]
    }
}

pub struct RobotTab {
    // --- Transform State ---
    get_all_transforms_promise: Option<Promise<HashMap<String, SPTransformStamped>>>,
    transform_keys: Vec<String>,
    selected_pose: Option<String>,
    tcp_keys: Vec<String>,
    selected_tcp: Option<String>,
    selected_faceplate: Option<String>,
    selected_baseframe: Option<String>,
    selected_root: Option<String>,

    // --- Command State ---
    command_type: CommandType,
    acceleration: f32,
    velocity: f32,
    global_acceleration_scaling: f32,
    global_velocity_scaling: f32,
    use_execution_time: bool,
    execution_time_ms: u32,

    // --- New Blend/Joint State ---
    use_blend_radius: bool,
    blend_radius: f32,
    use_joint_positions: bool,
    joint_positions: [f32; 6],
    use_preferred_joint_config: bool,
    preferred_joint_config: [f32; 6],

    use_payload: bool,
    set_manual_payload: bool,
    saved_payload: SavedPayload,
    manual_payload: Payload,
}

impl RobotTab {
    pub fn new() -> Self {
        Self {
            // --- Transform State ---
            get_all_transforms_promise: None,
            transform_keys: Vec::new(),
            selected_pose: None,
            tcp_keys: Vec::new(),
            selected_tcp: None,
            selected_faceplate: Some("tool0".to_string()),
            selected_baseframe: Some("base_link".to_string()),
            selected_root: Some("world".to_string()),
            // --- Command State ---
            command_type: CommandType::MoveL,
            acceleration: 0.1,
            velocity: 0.1,
            global_acceleration_scaling: 1.0,
            global_velocity_scaling: 1.0,
            use_execution_time: false,
            execution_time_ms: 0,
            // --- New State ---
            use_blend_radius: false,
            blend_radius: 0.0,
            use_joint_positions: false,
            joint_positions: [0.0; 6],
            use_preferred_joint_config: false,
            preferred_joint_config: [0.0; 6],

            use_payload: false,
            set_manual_payload: false,
            saved_payload: SavedPayload::None,
            manual_payload: Payload::default(),
        }
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        // This is now the root UI element for this tab.
        // The parent (e.g., in main.rs) should put this inside a ScrollArea
        // if the main window can be smaller than this tab's content.

        ui.heading("Robot Command GUI");
        ui.separator();

        // --- Top Section: Pose/Motion and Command Config ---
        // Allocate a fixed height for this section
        ui.allocate_ui(egui::vec2(ui.available_width(), 160.0), |ui| {
            ui.horizontal_top(|ui| {
                // --- Group 1: Pose & Motion (Left Column) ---
                ui.vertical(|ui| {
                    ui.set_min_width(250.0); // Ensure column has a reasonable width
                    ui.heading("Pose Config");
                    ui.horizontal(|ui| {
                        let is_fetching_list = self.poll_transforms_promise(ui);
                        if !is_fetching_list && ui.button("Fetch Transforms").clicked() {
                            self.spawn_transforms_promise(handle, connection);
                        }
                        if is_fetching_list {
                            ui.label("Loading...");
                        }
                    });

                    draw_pose_selector(
                        ui,
                        "Goal Feature ID (Where to go):",
                        "pose_select",
                        &mut self.selected_pose,
                        &self.transform_keys,
                    );
                    draw_pose_selector(
                        ui,
                        "TCP ID (With what frame):",
                        "tcp_select",
                        &mut self.selected_tcp,
                        &self.transform_keys,
                    );
                    draw_pose_selector(
                        ui,
                        "Faceplate ID (Robot's final link):",
                        "faceplate_select",
                        &mut self.selected_faceplate,
                        &self.transform_keys,
                    );
                    draw_pose_selector(
                        ui,
                        "Baseframe ID (base or base_link):",
                        "baseframe_select",
                        &mut self.selected_baseframe,
                        &self.transform_keys,
                    );
                    draw_pose_selector(
                        ui,
                        "Root ID (Max IK root):",
                        "root_select",
                        &mut self.selected_root,
                        &self.transform_keys,
                    );
                });

                // --- Vertical Separator ---
                ui.add(egui::Separator::default().vertical());

                // --- Group 2: Command Config (Right Column) ---
                ui.vertical(|ui| {
                    ui.set_min_width(250.0); // Ensure column has a reasonable width
                    ui.heading("Motion Config");
                    ui.horizontal(|ui| {
                        ui.label("Command Type:");
                        egui::ComboBox::from_id_salt("command_type_select")
                            .selected_text(self.command_type.to_string())
                            .show_ui(ui, |ui| {
                                for variant in CommandType::variants() {
                                    ui.selectable_value(
                                        &mut self.command_type,
                                        variant.clone(),
                                        variant.to_string(),
                                    );
                                }
                            });
                    });

                    ui.horizontal(|ui| {
                        ui.label("Acceleration:");
                        ui.add(
                            egui::DragValue::new(&mut self.acceleration)
                                .suffix(" m/s²")
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });

                    ui.horizontal(|ui| {
                        ui.label("Velocity:");
                        ui.add(
                            egui::DragValue::new(&mut self.velocity)
                                .suffix(" m/s")
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });

                    ui.horizontal(|ui| {
                        ui.label("Global Acceleration Scaling:");
                        ui.add(
                            egui::DragValue::new(&mut self.global_acceleration_scaling)
                                .suffix(" (0.0-1.0)")
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });

                    ui.horizontal(|ui| {
                        ui.label("Global Velocity Scaling:");
                        ui.add(
                            egui::DragValue::new(&mut self.global_velocity_scaling)
                                .suffix(" (0.0-1.0)")
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });
                });
            });
        });

        ui.separator(); // --- Horizontal Separator ---

        // --- Bottom Section: Blend and Joint Configs ---
        // Allocate a fixed height for this section
        ui.allocate_ui(egui::vec2(ui.available_width(), 100.0), |ui| {
            ui.horizontal_top(|ui| {
                // --- Group 3: Blend & Joint Positions (Bottom-Left) ---
                ui.vertical(|ui| {
                    ui.set_min_width(250.0); // Ensure column has a reasonable width
                    ui.heading("Joint Positions (Optional)");
                    // ui.checkbox(&mut self.use_blend_radius, "Use Blend Radius");
                    // ui.add_enabled_ui(self.use_blend_radius, |ui| {
                    //     ui.horizontal(|ui| {
                    //         ui.label("Blend Radius:");
                    //         ui.add(
                    //             egui::DragValue::new(&mut self.blend_radius)
                    //                 .suffix(" m")
                    //                 .speed(0.001)
                    //                 .range(0.0..=0.5), // Example range
                    //         );
                    //     });
                    // });

                    // ui.separator();

                    ui.checkbox(&mut self.use_joint_positions, "Use Joint Positions");
                    ui.add_enabled_ui(self.use_joint_positions, |ui| {
                        draw_joint_inputs(ui, &mut self.joint_positions, "joint_pos");
                    });

                    // ui.separator();

                    // ui.checkbox(
                    //     &mut self.use_preferred_joint_config,
                    //     "Use Preferred Joint Config",
                    // );
                    // ui.add_enabled_ui(self.use_preferred_joint_config, |ui| {
                    //     draw_joint_inputs(ui, &mut self.preferred_joint_config, "joint_config");
                    // });
                });

                // // --- Vertical Separator ---
                ui.add(egui::Separator::default().vertical());

                // // --- Group 4: Preferred Joint Config (Bottom-Right) ---
                ui.vertical(|ui| {
                    // ui.heading("Joint Configurations (Optional)");
                    ui.horizontal(|ui| {
                        ui.heading("Joint Configurations (Optional)");
                        ui.label("ℹ").on_hover_text(
                            "Sets a 'hint' for the robot's inverse kinematics solver (IK).\n\
                             If the preferred joint configuration (qnear) is defined, the \n\
                             solution closest to qnear is returned. Otherwise, the \n\
                             solution closest to the current joint positions is returned.",
                        );
                    });

                    ui.checkbox(
                        &mut self.use_preferred_joint_config,
                        "Use Preferred Joint Config",
                    );
                    ui.add_enabled_ui(self.use_preferred_joint_config, |ui| {
                        draw_joint_inputs(ui, &mut self.preferred_joint_config, "joint_config");
                    });
                });
            });
        });

        ui.separator(); // --- Horizontal Separator ---

        // --- Bottom Section: Payload ---
        // Allocate a static height for this section
        ui.allocate_ui(egui::vec2(ui.available_width(), 260.0), |ui| {
            ui.horizontal_top(|ui| {
                ui.vertical(|ui| {
                    ui.heading("Payload (Optional)");
                    ui.checkbox(&mut self.use_payload, "Use Payload");

                    // Everything in this section is disabled if `use_payload` is false
                    ui.add_enabled_ui(self.use_payload, |ui| {
                        ui.checkbox(&mut self.set_manual_payload, "Set Manual Payload");

                        // --- Dropdown for saved payloads ---
                        // Disabled if "Set Manual" is checked
                        ui.add_enabled_ui(!self.set_manual_payload, |ui| {
                            ui.horizontal(|ui| {
                                ui.label("Saved Payloads:");
                                egui::ComboBox::from_id_salt("saved_payload_select")
                                    .selected_text(self.saved_payload.to_string())
                                    .show_ui(ui, |ui| {
                                        for variant in SavedPayload::variants() {
                                            ui.selectable_value(
                                                &mut self.saved_payload,
                                                variant.clone(),
                                                variant.to_string(),
                                            );
                                        }
                                    });
                            });
                        });

                        // ui.separator();

                        // --- Manual Payload Inputs ---
                        // Enabled *only if* "Set Manual" is checked
                        ui.add_enabled_ui(self.set_manual_payload, |ui| {
                            egui::Frame::default()
                                // .inner_margin(egui::Margin::same(5))
                                // .stroke(egui::Stroke::new(1.0, egui::Color32::))
                                .show(ui, |ui| {
                                    ui.label("Manual Payload Configuration:");

                                    ui.horizontal(|ui| {
                                        ui.label("Mass (kg):");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.mass)
                                                .speed(0.01)
                                                .range(0.0..=f32::MAX),
                                        );
                                    });

                                    ui.label("Center of Gravity (m):");
                                    ui.horizontal(|ui| {
                                        ui.label("CoG X:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.cog_x)
                                                .speed(0.001),
                                        );
                                        ui.label("CoG Y:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.cog_y)
                                                .speed(0.001),
                                        );
                                        ui.label("CoG Z:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.cog_z)
                                                .speed(0.001),
                                        );
                                    });

                                    ui.label("Inertia Matrix (kg*m^2):");
                                    ui.horizontal(|ui| {
                                        ui.label("Ixx:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.ixx)
                                                .speed(0.001),
                                        );
                                        ui.label("Iyy:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.iyy)
                                                .speed(0.001),
                                        );
                                        ui.label("Izz:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.izz)
                                                .speed(0.001),
                                        );
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Ixy:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.ixy)
                                                .speed(0.001),
                                        );
                                        ui.label("Ixz:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.ixz)
                                                .speed(0.001),
                                        );
                                        ui.label("Iyz:");
                                        ui.add(
                                            egui::DragValue::new(&mut self.manual_payload.iyz)
                                                .speed(0.001),
                                        );
                                    });
                                });
                        });
                    });
                   
                });
                 ui.add(egui::Separator::default().vertical());
                    // ui.allocate_ui(egui::vec2(ui.available_width(), 260.0), |ui| {
                    ui.vertical(|ui| {
                        ui.heading("Miscelaneous (Optional)");
                        ui.checkbox(&mut self.use_execution_time, "Use Execution Time");

                        ui.add_enabled_ui(self.use_execution_time, |ui| {
                            ui.horizontal(|ui| {
                                ui.label("Execution Time:");
                                ui.add(
                                    egui::DragValue::new(&mut self.execution_time_ms)
                                        .suffix(" ms")
                                        .speed(10.0),
                                );
                            });
                        });
                    });
            });
        });

        // --- Vertical Separator ---

        // });
        // });

        ui.separator(); // --- Final Separator ---

        // --- Group 5: Send Command Button ---
        ui.add_space(10.0);
        ui.vertical_centered(|ui| {
            ui.add_enabled(false, egui::Button::new("Send Command"));
        });
    }

    // --- Transform Polling Functions (Copied) ---

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

        if let Some(pose) = &self.selected_pose {
            if !self.transform_keys.contains(pose) {
                self.selected_pose = None;
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

// --- Helper UI Functions (Copied & New) ---

/// Helper to draw the dropdown for selecting a pose
fn draw_pose_selector(
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

/// Helper to draw 6 joint input fields in a grid
fn draw_joint_inputs(ui: &mut egui::Ui, joints: &mut [f32; 6], id_prefix: &str) {
    let rad_range = -6.28..=6.28;

    egui::Grid::new(id_prefix)
        .num_columns(4)
        .spacing([20.0, 4.0])
        .striped(true)
        .show(ui, |ui| {
            ui.label("J1:");
            ui.add(
                egui::DragValue::new(&mut joints[0])
                    .suffix(" rad")
                    .range(rad_range.clone())
                    .speed(0.01),
            );
            ui.label("J2:");
            ui.add(
                egui::DragValue::new(&mut joints[1])
                    .suffix(" rad")
                    .range(rad_range.clone())
                    .speed(0.01),
            );
            ui.end_row();

            ui.label("J3:");
            ui.add(
                egui::DragValue::new(&mut joints[2])
                    .suffix(" rad")
                    .range(rad_range.clone())
                    .speed(0.01),
            );
            ui.label("J4:");
            ui.add(
                egui::DragValue::new(&mut joints[3])
                    .suffix(" rad")
                    .range(rad_range.clone())
                    .speed(0.01),
            );
            ui.end_row();

            ui.label("J5:");
            ui.add(
                egui::DragValue::new(&mut joints[4])
                    .suffix(" rad")
                    .range(rad_range.clone())
                    .speed(0.01),
            );
            ui.label("J6:");
            ui.add(
                egui::DragValue::new(&mut joints[5])
                    .suffix(" rad")
                    .range(rad_range.clone())
                    .speed(0.01),
            );
            ui.end_row();
        });
}
