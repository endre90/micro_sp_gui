use eframe::egui;
use micro_sp::*;
use ordered_float::OrderedFloat;
use poll_promise::Promise;
use std::{collections::HashMap, fmt, sync::Arc};

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
    pub mass: f64,
    /// Payload Center of Gravity offsets (in meters) from the tool mount.
    pub cog_x: f64,
    pub cog_y: f64,
    pub cog_z: f64,
    /// Payload Inertia Matrix (in kg*m^2) with origin at the CoG and axes aligned with the tool flange axes.
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyz: f64,
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

impl fmt::Display for Payload {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{},[{},{},{}],[{},{},{},{},{},{}]",
            self.mass,
            self.cog_x,
            self.cog_y,
            self.cog_z,
            self.ixx,
            self.iyy,
            self.izz,
            self.ixy,
            self.ixz,
            self.iyz
        )
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

async fn send_robot_command(state: &State, con: Arc<ConnectionManager>) -> () {
    let mut connection = con.get_connection().await;
    StateManager::set_state(&mut connection, &state).await;
}

// --- RobotTab Specific ---

#[derive(Debug, Clone, PartialEq)]
enum CommandType {
    UnsafeMoveL,
    UnsafeMoveJ,
    SafeMoveL,
    SafeMoveJ,
    PickVacuum,
    PlaceVacuum,
}

impl std::fmt::Display for CommandType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CommandType::UnsafeMoveL => write!(f, "unsafe_move_l"),
            CommandType::UnsafeMoveJ => write!(f, "unsafe_move_j"),
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
            CommandType::UnsafeMoveL,
            CommandType::UnsafeMoveJ,
            CommandType::SafeMoveL,
            CommandType::SafeMoveJ,
            CommandType::PickVacuum,
            CommandType::PlaceVacuum,
        ]
    }
}

pub struct RobotTab {
    // --- Transform State ---
    robot_id_input: String,
    get_all_transforms_promise: Option<Promise<HashMap<String, SPTransformStamped>>>,
    robot_control_promise: Option<Promise<()>>,
    transform_keys: Vec<String>,
    selected_goal_feature_id: Option<String>,
    tcp_keys: Vec<String>,
    selected_tcp: Option<String>,
    selected_faceplate: Option<String>,
    selected_baseframe: Option<String>,
    // selected_root: Option<String>,

    // --- Command State ---
    command_type: CommandType,
    acceleration: f64,
    velocity: f64,
    global_acceleration_scaling: f64,
    global_velocity_scaling: f64,

    // --- New Blend/Joint State ---
    use_blend_radius: bool,
    blend_radius: f64,
    use_joint_positions: bool,
    set_manual_joint_positions: bool,
    joint_positions: [f64; 6],
    use_preferred_joint_config: bool,
    preferred_joint_config: [f64; 6],
    set_manual_joint_config: bool,

    use_payload: bool,
    set_manual_payload: bool,
    saved_payload: SavedPayload,
    manual_payload: Payload,

    use_execution_time: bool,
    execution_time_s: f64,
    force_threshold: f64,
    use_relative_pose: bool,
    relative_pose: [f64; 6]
}

impl RobotTab {
    pub fn new() -> Self {
        Self {
            // --- Transform State ---
            robot_id_input: "r1".to_string(),
            get_all_transforms_promise: None,
            robot_control_promise: None,
            transform_keys: Vec::new(),
            selected_goal_feature_id: None,
            tcp_keys: Vec::new(),
            selected_tcp: None,
            selected_faceplate: Some("tool0".to_string()),
            selected_baseframe: Some("base_link".to_string()),
            // selected_root: Some("world".to_string()),
            // --- Command State ---
            command_type: CommandType::UnsafeMoveL,
            acceleration: 0.1,
            velocity: 0.1,
            global_acceleration_scaling: 1.0,
            global_velocity_scaling: 1.0,

            // --- New State ---
            use_blend_radius: false,
            blend_radius: 0.0,
            use_joint_positions: false,
            set_manual_joint_positions: false,
            joint_positions: [0.0; 6],
            use_preferred_joint_config: false,
            preferred_joint_config: [0.0; 6],
            set_manual_joint_config: false,

            use_payload: false,
            set_manual_payload: false,
            saved_payload: SavedPayload::None,
            manual_payload: Payload::default(),

            use_execution_time: false,
            execution_time_s: 0.0,
            force_threshold: 20.0,
            use_relative_pose: false,
            relative_pose: [0.0; 6]
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

        // ui.horizontal(|ui| {
        //     ui.heading("Robot Controller");
        //     ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        //         ui.add_enabled(true, egui::Button::new("Send Command"));
        //     });
        // });
        // Add all right-aligned items here, in reverse order
        ui.horizontal(|ui| {
            ui.heading("Robot Controller"); // This stays on the left

            // Add all right-aligned items here, in reverse order
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // 1. The Button (will be furthest right)
                ui.add_enabled(true, egui::Button::new("Stop"));

                // The `.clicked()` method returns true on the frame the button is pressed
                if ui
                    .add_enabled(true, egui::Button::new("Send Command"))
                    .clicked()
                {
                    self.spawn_robot_control_promise(handle, connection)
                }

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

        // --- Top Section: Pose/Motion and Command Config ---
        // Allocate a fixed height for this sectionc
        ui.allocate_ui(egui::vec2(ui.available_width(), 130.0), |ui| {
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
                        &mut self.selected_goal_feature_id,
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
                    // draw_pose_selector(
                    //     ui,
                    //     "Root ID (Max IK root):",
                    //     "root_select",
                    //     &mut self.selected_root,
                    //     &self.transform_keys,
                    // );
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
                        ui.label("ℹ").on_hover_text(
                            "Specify what the robot should do. \n\
                             MoveL is a linear move in tool space. \n\
                             MoveJ is a linear move in joint space. \n\
                             Unsafe means that the robot will enter protective \n\
                             stop if an ostacle is hit along the way. \n\
                             Safe means that another thread is monitoring the \n\
                             force exerted on the robot. If that force exceeds \n\
                             the 'Force Threshold' (set below in Misc), the robot \n\
                             will stop moving without entering protective stop.",
                        );
                    });

                    let accel_vel_suffix = match self.command_type {
                        CommandType::UnsafeMoveL => " m/s²",
                        CommandType::UnsafeMoveJ => " rad/s²",
                        CommandType::SafeMoveL => " m/s²",
                        CommandType::SafeMoveJ => " rad/s²",
                        CommandType::PickVacuum => " m/s²",
                        CommandType::PlaceVacuum => " m/s²",
                    };

                    ui.horizontal(|ui| {
                        ui.label("Acceleration:");

                        ui.add(
                            egui::DragValue::new(&mut self.acceleration)
                                .suffix(accel_vel_suffix) // Use the dynamically set suffix
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });

                    ui.horizontal(|ui| {
                        ui.label("Velocity:");
                        ui.add(
                            egui::DragValue::new(&mut self.velocity)
                                .suffix(accel_vel_suffix)
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
                    ui.horizontal(|ui| {
                        ui.heading("Joint Positions (Optional)");
                        ui.label("ℹ")
                            .on_hover_text("Use joint positions instead of a goal pose.");
                    });



                    // ui.separator();

                    ui.checkbox(&mut self.use_joint_positions, "Use Joint Positions");

                    // Everything in this section is disabled if `use_payload` is false
                    ui.add_enabled_ui(self.use_joint_positions, |ui| {
                        // --- Dropdown for saved payloads ---
                        // Disabled if "Set Manual" is checked
                        ui.add_enabled_ui(!self.set_manual_joint_positions, |ui| {
                            ui.horizontal(|ui| {
                                ui.label("Saved Joint Positions:");
                                egui::ComboBox::from_id_salt("saved_joint_positions_select")
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

                        ui.checkbox(
                            &mut self.set_manual_joint_positions,
                            "Set Manual Joint Positions",
                        );

                        ui.add_enabled_ui(self.set_manual_joint_positions, |ui| {
                            draw_joint_inputs(ui, &mut self.joint_positions, "joint_pos");
                        });
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
                        // --- Dropdown for saved payloads ---
                        // Disabled if "Set Manual" is checked
                        ui.add_enabled_ui(!self.set_manual_joint_config, |ui| {
                            ui.horizontal(|ui| {
                                ui.label("Saved Joint Configurations:");
                                egui::ComboBox::from_id_salt("saved_joint_configuration_select")
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

                        ui.checkbox(
                            &mut self.set_manual_joint_config,
                            "Set Manual Preferred Joint Config",
                        );

                        ui.add_enabled_ui(self.set_manual_joint_config, |ui| {
                            draw_joint_inputs(ui, &mut self.preferred_joint_config, "joint_config");
                        });
                    });
                });
            });
        });

        ui.separator(); // --- Horizontal Separator ---

        // --- Bottom Section: Payload ---
        // Allocate a static height for this section
        ui.allocate_ui(egui::vec2(ui.available_width(), 200.0), |ui| {
            ui.horizontal_top(|ui| {
                ui.vertical(|ui| {
                    ui.heading("Payload (Optional)");
                    ui.checkbox(&mut self.use_payload, "Use Payload");

                    // Everything in this section is disabled if `use_payload` is false
                    ui.add_enabled_ui(self.use_payload, |ui| {
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

                        ui.checkbox(&mut self.set_manual_payload, "Set Manual Payload");

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
                                                .range(0.0..=f64::MAX),
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
                                egui::DragValue::new(&mut self.execution_time_s)
                                    .suffix(" ms")
                                    .speed(10.0),
                            );
                        });
                    });
                    ui.checkbox(&mut self.use_blend_radius, "Use Blend Radius");
                    ui.add_enabled_ui(self.use_blend_radius, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Blend Radius:");
                            ui.add(
                                egui::DragValue::new(&mut self.blend_radius)
                                    .suffix(" m")
                                    .speed(0.001)
                                    .range(0.0..=0.5), // Example range
                            );
                        });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Force Threshold:");
                        ui.add(
                            egui::DragValue::new(&mut self.force_threshold)
                                .suffix(" N")
                                .speed(0.1)
                                .range(0.0..=200.0),
                        );
                    });
                    ui.checkbox(&mut self.use_relative_pose, "Use Relative Pose");
                    ui.add_enabled_ui(self.use_relative_pose, |ui| {
                        draw_relative_pose_inputs(ui, &mut self.relative_pose, "relative_pose");
                    });
                });
            });
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

        if let Some(pose) = &self.selected_goal_feature_id {
            if !self.transform_keys.contains(pose) {
                self.selected_goal_feature_id = None;
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

    fn spawn_robot_control_promise(
        &mut self,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        let handle = handle.clone();
        let con_clone = connection.clone();
        match robot_command_tab_to_state(&self) {
            Ok(state) => {
                self.robot_control_promise =
                    Some(Promise::spawn_thread("robot_control", move || {
                        handle.block_on(send_robot_command(&state, con_clone))
                    }));
            }
            Err(_) => (),
        }
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
fn draw_joint_inputs(ui: &mut egui::Ui, joints: &mut [f64; 6], id_prefix: &str) {
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

fn draw_relative_pose_inputs(ui: &mut egui::Ui, poses: &mut [f64; 6], id_prefix: &str) {
    egui::Grid::new(id_prefix)
        .num_columns(4)
        .spacing([20.0, 4.0])
        .striped(true)
        .show(ui, |ui| {
            ui.label("x:");
            ui.add(
                egui::DragValue::new(&mut poses[0])
                    .suffix(" m")
                    .speed(0.001),
            );
            ui.label("rx:");
            ui.add(
                egui::DragValue::new(&mut poses[3])
                    .suffix(" rad")
                    .speed(0.01),
            );
            ui.end_row();

            ui.label("y:");
            ui.add(
                egui::DragValue::new(&mut poses[1])
                    .suffix(" m")
                    .speed(0.001),
            );
            ui.label("ry:");
            ui.add(
                egui::DragValue::new(&mut poses[4])
                    .suffix(" rad")
                    .speed(0.01),
            );
            ui.end_row();

            ui.label("z:");
            ui.add(
                egui::DragValue::new(&mut poses[2])
                    .suffix(" m")
                    .speed(0.001),
            );
            ui.label("rz:");
            ui.add(
                egui::DragValue::new(&mut poses[5])
                    .suffix(" rad")
                    .speed(0.01),
            );
            ui.end_row();
        });
}

// Should have one for dashboard as well
pub fn robot_command_tab_to_state(tab: &RobotTab) -> Result<State, String> {
    println!("trigerred");
    let robot_name = &tab.robot_id_input;
    let state = State::new();

    let request_trigger = bv!(&&format!("{}_request_trigger", robot_name));
    // let dashboard_request_trigger = bv!(&&format!("{}_dashboard_request_trigger", robot_name));

    let state = state.add(assign!(request_trigger, true.to_spvalue()));
    // let state = state.add(assign!(dashboard_request_trigger, false.to_spvalue()));

    let command_type = v!(&&format!("{}_command_type", robot_name));
    let accelleration = fv!(&&format!("{}_accelleration", robot_name));
    let velocity = fv!(&&format!("{}_velocity", robot_name));

    // Is this Dashboard? We should also have protective stop / violation release, pause and continue, get into remote control, set max force (safety)
    // let global_acceleration_scaling = fv!(&&format!("{}_global_acceleration_scaling", robot_name));
    // let global_velocity_scaling = fv!(&&format!("{}_global_velocity_scaling", robot_name));
    let use_execution_time = bv!(&&format!("{}_use_execution_time", robot_name));
    let execution_time = fv!(&&format!("{}_execution_time", robot_name));
    let use_blend_radius = bv!(&&format!("{}_use_blend_radius", robot_name));
    let blend_radius = fv!(&&format!("{}_blend_radius", robot_name));
    let use_joint_positions = bv!(&&format!("{}_use_joint_positions", robot_name));
    let joint_positions = av!(&&format!("{}_joint_positions", robot_name));

    // Input could be put in jpint positions eventually
    // let joint_states = av!(&&format!("{}_joint_states", robot_name));
    let use_preferred_joint_config = bv!(&&format!("{}_use_preferred_joint_config", robot_name));
    let preferred_joint_config = av!(&&format!("{}_preferred_joint_config", robot_name));
    let use_payload = bv!(&&format!("{}_use_payload", robot_name));
    let payload = v!(&&format!("{}_payload", robot_name));
    let baseframe_id = v!(&&format!("{}_baseframe_id", robot_name));
    let faceplate_id = v!(&&format!("{}_faceplate_id", robot_name));
    let goal_feature_id = v!(&&format!("{}_goal_feature_id", robot_name));
    let tcp_id = v!(&&format!("{}_tcp_id", robot_name));
    let root_frame_id = v!(&&format!("{}_root_frame_id", robot_name));
    // let cancel_current_goal = bv!(&&format!("{}_cancel_current_goal", robot_name));
    let force_threshold = fv!(&&format!("{}_force_threshold", robot_name));
    // let force_feedback = fv!(&&format!("{}_force_feedback", robot_name));
    // let estimated_position = v!(&&format!("{}_estimated_position", robot_name));
    let use_relative_pose = bv!(&&format!("{}_use_relative_pose", robot_name));
    let relative_pose = av!(&&format!("{}_relative_pose", robot_name));

    let state = state.add(assign!(
        command_type,
        SPValue::String(StringOrUnknown::String(tab.command_type.to_string()))
    ));

    let state = state.add(assign!(
        accelleration,
        SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(tab.acceleration)))
    ));
    let state = state.add(assign!(
        velocity,
        SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(tab.velocity)))
    ));

    // Is this dashboard?
    // let state = state.add(assign!(
    //     global_acceleration_scaling,
    //     SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(tab.global_acceleration_scaling)))
    // ));
    // let state = state.add(assign!(
    //     global_velocity_scaling,
    //     SPValue::Float64(FloatOrUnknown::UNKNOWN)
    // ));
    let state = state.add(assign!(
        use_execution_time,
        SPValue::Bool(BoolOrUnknown::Bool(tab.use_execution_time))
    ));
    let state = state.add(assign!(
        execution_time,
        SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(tab.execution_time_s)))
    ));
    let state = state.add(assign!(
        use_blend_radius,
        SPValue::Bool(BoolOrUnknown::Bool(tab.use_blend_radius))
    ));
    let state = state.add(assign!(
        blend_radius,
        SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(tab.blend_radius)))
    ));
    let state = state.add(assign!(
        use_joint_positions,
        SPValue::Bool(BoolOrUnknown::Bool(tab.use_joint_positions))
    ));
    let state = state.add(assign!(
        joint_positions,
        SPValue::Array(ArrayOrUnknown::Array(
            tab.joint_positions.iter().map(|x| x.to_spvalue()).collect()
        ))
    ));

    // Could be good to read this as input and put it in the joint positions eventually
    // let state = state.add(assign!(
    //     joint_states,
    //     SPValue::Array(ArrayOrUnknown::UNKNOWN)
    // ));
    let state = state.add(assign!(
        use_preferred_joint_config,
        SPValue::Bool(BoolOrUnknown::Bool(tab.use_preferred_joint_config))
    ));
    let state = state.add(assign!(
        preferred_joint_config,
        SPValue::Array(ArrayOrUnknown::Array(
            tab.preferred_joint_config
                .iter()
                .map(|x| x.to_spvalue())
                .collect()
        ))
    ));
    let state = state.add(assign!(
        use_payload,
        SPValue::Bool(BoolOrUnknown::Bool(tab.use_payload))
    ));
    let state = state.add(assign!(
        payload,
        SPValue::String(StringOrUnknown::String(tab.saved_payload.to_string()))
    ));
    let state = match &tab.selected_baseframe {
        Some(baseframe) => state.add(assign!(
            baseframe_id,
            SPValue::String(StringOrUnknown::String(baseframe.to_owned()))
        )),
        None => {
            log::error!("Baseframe not selected");
            return Err(format!("Baseframe not selected"));
        }
    };
    let state = match &tab.selected_faceplate {
        Some(faceplate) => state.add(assign!(
            faceplate_id,
            SPValue::String(StringOrUnknown::String(faceplate.to_owned()))
        )),
        None => {
            log::error!("Faceplate not selected");
            return Err(format!("Faceplate not selected"));
        }
    };
    let state = match &tab.selected_goal_feature_id {
        Some(goal_feature) => state.add(assign!(
            goal_feature_id,
            SPValue::String(StringOrUnknown::String(goal_feature.to_owned()))
        )),
        None => {
            log::error!("Goal feature not selected");
            return Err(format!("Goal feature not selected"));
        }
    };
    let state = match &tab.selected_tcp {
        Some(tcp) => state.add(assign!(
            tcp_id,
            SPValue::String(StringOrUnknown::String(tcp.to_owned()))
        )),
        None => {
            log::error!("Tcp not selected");
            return Err(format!("Tcp not selected"));
        }
    };

    let state = state.add(assign!(
        root_frame_id,
        SPValue::String(StringOrUnknown::String("world".to_string()))
    ));

    // Add later, connect to the Stop button. This is the action client and the stop is the dachboard
    // let state = state.add(assign!(
    //     cancel_current_goal,
    //     SPValue::Bool(BoolOrUnknown::UNKNOWN)
    // ));
    // let state = state.add(assign!(
    //     estimated_position,
    //     SPValue::String(StringOrUnknown::UNKNOWN)
    // ));

    let state = state.add(assign!(
        force_threshold,
        SPValue::Float64(FloatOrUnknown::Float64(OrderedFloat(tab.force_threshold)))
    ));

    // Add later as input to see what's happening
    // let state = state.add(assign!(
    //     force_feedback,
    //     SPValue::Float64(FloatOrUnknown::UNKNOWN)
    // ));
    let state = state.add(assign!(
        use_relative_pose,
        SPValue::Bool(BoolOrUnknown::Bool(tab.use_relative_pose))
    ));
    let state = state.add(assign!(
        relative_pose,
        SPValue::Array(ArrayOrUnknown::Array(
            tab.relative_pose.iter().map(|x| x.to_spvalue()).collect()
        ))
    ));

    Ok(state)
}
