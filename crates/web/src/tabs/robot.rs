//! Driving the UR through `ur_redis_driver`.
//!
//! The old native tab composed the driver's `{robot_id}_*` variables by hand and
//! showed none of the feedback the driver publishes. This one builds a
//! `RobotCommand` and lets the server do the naming, and puts the driver's own
//! status and result strings on screen - which is what tells you *why* a request
//! failed rather than just that it did.

use crate::api::Api;
use crate::widgets;
use micro_sp_gui_protocol as proto;

pub struct RobotTab {
    cmd: proto::RobotCommand,
    /// Which payload preset is selected. The old tab had one field backing three
    /// unrelated combo boxes; only the payload ever had presets behind it.
    payload_preset: usize,
    manual_payload: bool,
    payload: proto::Payload,

    dashboard_command: String,
    dashboard_arg: String,
    /// Filled from the live joint states on request, rather than every frame -
    /// otherwise the fields could never be edited.
    copy_joints_requested: bool,
}

impl Default for RobotTab {
    fn default() -> Self {
        Self {
            cmd: proto::RobotCommand::default(),
            payload_preset: 0,
            manual_payload: false,
            payload: proto::Payload::default(),
            dashboard_command: "reset_protective_stop".to_string(),
            dashboard_arg: String::new(),
            copy_joints_requested: false,
        }
    }
}

impl RobotTab {
    pub fn ui(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        // The header picker owns which robot we are talking to.
        if let Some(id) = &api.robot_id
            && *id != self.cmd.robot_id
        {
            self.cmd.robot_id = id.clone();
        }

        if api.robot_id.is_none() {
            ui.label(
                egui::RichText::new(
                    "No robot interface found in Redis. Start ur_redis_driver, or pass \
                     --robot-id to the server to address one that has not seeded its keys yet.",
                )
                .color(widgets::WARN),
            );
            ui.separator();
        }

        self.status_strip(ui, api);
        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            self.command_section(ui, api);
            ui.separator();
            self.send_row(ui, api);
            ui.separator();
            self.dashboard_section(ui, api);
        });
    }

    fn status_strip(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        let Some(status) = api.robot_status().cloned() else {
            ui.label(egui::RichText::new("No status published for this robot yet.").weak());
            return;
        };

        ui.horizontal_wrapped(|ui| {
            widgets::light(
                ui,
                status.robot_connected,
                "robot",
                "The driver's realtime connection to the controller.",
            );
            ui.separator();
            widgets::light(
                ui,
                status.dashboard_connected,
                "dashboard",
                "The driver's dashboard connection.",
            );
            ui.separator();

            // Safety mode decides whether a goal is admitted at all.
            let safety_colour = match status.safety_mode.as_str() {
                "NORMAL" | "REDUCED" => widgets::OK,
                "UNKNOWN" => widgets::UNKNOWN,
                _ => widgets::BAD,
            };
            ui.label(egui::RichText::new("safety").weak());
            ui.label(egui::RichText::new(&status.safety_mode).color(safety_colour).strong())
                .on_hover_text("The driver only accepts a goal in NORMAL or REDUCED.");

            ui.separator();
            ui.label(egui::RichText::new("mode").weak());
            ui.label(egui::RichText::new(&status.robot_mode).monospace());

            ui.separator();
            // `program_state` describes the loaded pendant program, so it sits at
            // STOPPED while an injected script runs; `program_running` is the one
            // that tracks this driver's scripts.
            ui.label(egui::RichText::new("script").weak());
            ui.label(
                egui::RichText::new(if status.program_running { "running" } else { "idle" })
                    .monospace(),
            )
            .on_hover_text(format!(
                "program_running={} (pendant program_state is '{}', which stays STOPPED while \
                 an injected script runs)",
                status.program_running, status.program_state
            ));

            if status.remote_control {
                ui.separator();
                ui.label(egui::RichText::new("remote").color(widgets::OK));
            }
            ui.separator();
            ui.label(egui::RichText::new(format!("speed ×{:.2}", status.speed_scaling)).weak());
        });

        // The request row: state, and the reason behind it.
        ui.horizontal_wrapped(|ui| {
            let (label, colour) = if status.was_cancelled() {
                // A cancel terminates as `succeeded` with result `cancelled`.
                // Painting that red would be a lie.
                ("cancelled", widgets::WARN)
            } else if status.request_failed() {
                ("failed", widgets::BAD)
            } else {
                match status.request_state.as_str() {
                    "succeeded" => ("succeeded", widgets::OK),
                    "executing" => ("executing", widgets::WARN),
                    "initial" => ("initial", widgets::UNKNOWN),
                    other => (other, widgets::UNKNOWN),
                }
            };
            ui.label(egui::RichText::new("request").weak());
            ui.label(egui::RichText::new(label).color(colour).strong());

            if status.request_result != "UNKNOWN" && !status.request_result.is_empty() {
                ui.separator();
                ui.label(egui::RichText::new(&status.request_result).monospace())
                    .on_hover_text("The driver's own reason for the outcome.");
            }
            if status.request_feedback != "UNKNOWN" && !status.request_feedback.is_empty() {
                ui.separator();
                ui.label(
                    egui::RichText::new(&status.request_feedback).monospace().weak(),
                )
                .on_hover_text("The last line the running URScript sent back.");
            }
            if status.subsequent_fail_counter > 0 || status.total_fail_counter > 0 {
                ui.separator();
                let colour = if status.subsequent_fail_counter > 0 {
                    widgets::BAD
                } else {
                    widgets::UNKNOWN
                };
                ui.label(
                    egui::RichText::new(format!(
                        "fails {}/{}",
                        status.subsequent_fail_counter, status.total_fail_counter
                    ))
                    .color(colour),
                )
                .on_hover_text("Consecutive / total. The consecutive count resets on success.");
            }
        });

        ui.horizontal_wrapped(|ui| {
            widgets::vector_label(ui, "joints", &status.joint_states);
            ui.separator();
            widgets::vector_label(ui, "tcp", &status.tcp_pose);
            ui.separator();
            widgets::field(ui, "force", &format!("{:.2} N", status.force_feedback));
        });

        if status.dashboard_request_result != "UNKNOWN"
            && !status.dashboard_request_result.is_empty()
        {
            widgets::field(ui, "dashboard says", &status.dashboard_request_result);
        }
    }

    fn command_section(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        let frames = api.frame_names();

        ui.horizontal_wrapped(|ui| {
            ui.label("command");
            egui::ComboBox::from_id_salt("command_type")
                .selected_text(&self.cmd.command_type)
                .width(220.0)
                .show_ui(ui, |ui| {
                    for name in proto::COMMAND_TYPES {
                        ui.selectable_value(
                            &mut self.cmd.command_type,
                            name.to_string(),
                            *name,
                        );
                    }
                });
            ui.label(
                egui::RichText::new(format!("{} templates", proto::COMMAND_TYPES.len()))
                    .weak()
                    .small(),
            );
        });

        ui.columns(2, |columns| {
            let ui = &mut columns[0];
            ui.group(|ui| {
                ui.label(egui::RichText::new("Frames").strong());
                if !self.cmd.needs_frames() {
                    ui.label(
                        egui::RichText::new(
                            "This command does not resolve frames, so these are ignored.",
                        )
                        .weak()
                        .small(),
                    );
                }
                picker(ui, "goal_feature", "goal feature", &mut self.cmd.goal_feature_id, &frames);
                picker(ui, "tcp", "tcp", &mut self.cmd.tcp_id, &frames);
                picker(ui, "faceplate", "faceplate", &mut self.cmd.faceplate_id, &frames);
                picker(ui, "baseframe", "baseframe", &mut self.cmd.baseframe_id, &frames);
                ui.horizontal(|ui| {
                    ui.label("root");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.cmd.root_frame_id)
                            .desired_width(120.0),
                    )
                    .on_hover_text("Read by the driver but currently unused by it.");
                });
            });

            let ui = &mut columns[1];
            ui.group(|ui| {
                ui.label(egui::RichText::new("Motion").strong());
                slider(ui, "acceleration", &mut self.cmd.acceleration, 0.0..=5.0);
                slider(ui, "velocity", &mut self.cmd.velocity, 0.0..=5.0);
                // Both of these are read by the driver; the old GUI had the
                // fields but never wrote them.
                slider(
                    ui,
                    "global accel scaling",
                    &mut self.cmd.global_acceleration_scaling,
                    0.0..=1.0,
                );
                slider(
                    ui,
                    "global vel scaling",
                    &mut self.cmd.global_velocity_scaling,
                    0.0..=1.0,
                );
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.cmd.use_execution_time, "execution time");
                    ui.add_enabled(
                        self.cmd.use_execution_time,
                        egui::DragValue::new(&mut self.cmd.execution_time)
                            .speed(0.1)
                            .suffix(" s"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.cmd.use_blend_radius, "blend radius");
                    ui.add_enabled(
                        self.cmd.use_blend_radius,
                        egui::DragValue::new(&mut self.cmd.blend_radius)
                            .speed(0.001)
                            .suffix(" m"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("force threshold");
                    ui.add(
                        egui::DragValue::new(&mut self.cmd.force_threshold)
                            .speed(0.5)
                            .suffix(" N"),
                    );
                });
            });
        });

        egui::CollapsingHeader::new("Joints").default_open(false).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.cmd.use_joint_positions, "move to joint positions")
                    .on_hover_text("Skips the frame lookup entirely.");
                if ui
                    .button("Copy from robot")
                    .on_hover_text("Fill these with the robot's current joint states.")
                    .clicked()
                {
                    self.copy_joints_requested = true;
                }
            });
            joint_row(ui, "target", &mut self.cmd.joint_positions);

            ui.separator();
            ui.checkbox(
                &mut self.cmd.use_preferred_joint_config,
                "prefer a joint configuration",
            )
            .on_hover_text("Guides the inverse kinematics towards this configuration.");
            joint_row(ui, "preferred", &mut self.cmd.preferred_joint_config);
        });

        if self.copy_joints_requested {
            self.copy_joints_requested = false;
            match api.robot_status().map(|s| s.joint_states.clone()) {
                Some(joints) if joints.len() == 6 => {
                    for (target, value) in self.cmd.joint_positions.iter_mut().zip(joints) {
                        *target = value;
                    }
                    api.note("Copied the robot's current joint states.");
                }
                _ => api.warn("The robot has not published six joint values."),
            }
        }

        egui::CollapsingHeader::new("Relative pose").show(ui, |ui| {
            ui.checkbox(&mut self.cmd.use_relative_pose, "move relative to where it is")
                .on_hover_text("Also skips the frame lookup.");
            joint_row(ui, "dx dy dz rx ry rz", &mut self.cmd.relative_pose);
        });

        egui::CollapsingHeader::new("Payload").show(ui, |ui| {
            ui.checkbox(&mut self.cmd.use_payload, "set the payload");
            ui.checkbox(&mut self.manual_payload, "enter it manually");

            if self.manual_payload {
                ui.horizontal_wrapped(|ui| {
                    ui.label("mass");
                    ui.add(
                        egui::DragValue::new(&mut self.payload.mass).speed(0.01).suffix(" kg"),
                    );
                    ui.label("cog");
                    ui.add(egui::DragValue::new(&mut self.payload.cog_x).speed(0.001));
                    ui.add(egui::DragValue::new(&mut self.payload.cog_y).speed(0.001));
                    ui.add(egui::DragValue::new(&mut self.payload.cog_z).speed(0.001));
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("inertia");
                    for value in [
                        &mut self.payload.ixx,
                        &mut self.payload.iyy,
                        &mut self.payload.izz,
                        &mut self.payload.ixy,
                        &mut self.payload.ixz,
                        &mut self.payload.iyz,
                    ] {
                        ui.add(egui::DragValue::new(value).speed(0.001));
                    }
                });
                self.cmd.payload = self.payload.to_string();
            } else {
                egui::ComboBox::from_id_salt("payload_preset")
                    .selected_text(proto::PAYLOAD_PRESETS[self.payload_preset].0)
                    .show_ui(ui, |ui| {
                        for (i, (label, _)) in proto::PAYLOAD_PRESETS.iter().enumerate() {
                            ui.selectable_value(&mut self.payload_preset, i, *label);
                        }
                    });
                self.cmd.payload = proto::PAYLOAD_PRESETS[self.payload_preset].1.to_string();
            }
            ui.label(egui::RichText::new(&self.cmd.payload).monospace().weak().small());
        });
    }

    fn send_row(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        // Say up front what the driver would reject, instead of after the fact.
        let problem = self.cmd.validate().err();

        ui.horizontal_wrapped(|ui| {
            let can_send = problem.is_none() && api.conn.is_live();
            let send = ui.add_enabled(
                can_send,
                egui::Button::new(egui::RichText::new("Send command").strong()),
            );
            if send.clicked() {
                let mut cmd = self.cmd.clone();
                cmd.trigger = true;
                api.robot_command(cmd);
            }

            if ui
                .add_enabled(api.robot_id.is_some(), egui::Button::new("Write without triggering"))
                .on_hover_text("Stage every parameter but leave request_trigger false.")
                .clicked()
            {
                let mut cmd = self.cmd.clone();
                cmd.trigger = false;
                api.robot_command(cmd);
            }

            ui.separator();
            if ui
                .add_enabled(
                    api.robot_id.is_some(),
                    egui::Button::new(egui::RichText::new("Cancel").color(widgets::BAD)),
                )
                .on_hover_text(
                    "Asks the driver to abandon the running goal. It terminates as \
                     'succeeded' with result 'cancelled'.",
                )
                .clicked()
                && let Some(id) = api.robot_id.clone()
            {
                api.robot_cancel(id);
            }
        });

        if let Some(problem) = problem {
            ui.label(egui::RichText::new(format!("⚠ {problem}")).color(widgets::WARN));
        }
    }

    fn dashboard_section(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.label(egui::RichText::new("Dashboard").strong());
        ui.label(
            egui::RichText::new(
                "A separate handshake from motion requests, so a slow dashboard round trip \
                 cannot stall the driver's control loop.",
            )
            .weak()
            .small(),
        );

        let takes_arg = proto::DASHBOARD_COMMANDS
            .iter()
            .find(|(name, _)| *name == self.dashboard_command.as_str())
            .map(|(_, arg)| *arg)
            .unwrap_or(false);

        ui.horizontal_wrapped(|ui| {
            egui::ComboBox::from_id_salt("dashboard_command")
                .selected_text(&self.dashboard_command)
                .width(220.0)
                .show_ui(ui, |ui| {
                    for (name, _) in proto::DASHBOARD_COMMANDS {
                        ui.selectable_value(
                            &mut self.dashboard_command,
                            name.to_string(),
                            *name,
                        );
                    }
                });

            ui.add_enabled(
                takes_arg,
                egui::TextEdit::singleline(&mut self.dashboard_arg)
                    .hint_text(if takes_arg { "argument" } else { "no argument" })
                    .desired_width(200.0),
            );

            let ready = api.robot_id.is_some()
                && (!takes_arg || !self.dashboard_arg.trim().is_empty());
            if ui.add_enabled(ready, egui::Button::new("Send")).clicked()
                && let Some(robot_id) = api.robot_id.clone()
            {
                api.robot_dashboard(proto::RobotDashboardCommand {
                    robot_id,
                    command: self.dashboard_command.clone(),
                    arg: if takes_arg {
                        Some(self.dashboard_arg.trim().to_string())
                    } else {
                        None
                    },
                });
            }
        });

        // The handful that get reached for in an actual recovery.
        ui.horizontal_wrapped(|ui| {
            for quick in ["reset_protective_stop", "power_on", "brake_release", "stop", "play"] {
                if ui.small_button(quick).clicked()
                    && let Some(robot_id) = api.robot_id.clone()
                {
                    api.robot_dashboard(proto::RobotDashboardCommand {
                        robot_id,
                        command: quick.to_string(),
                        arg: None,
                    });
                }
            }
        });
    }
}

fn picker(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    current: &mut String,
    frames: &[String],
) {
    ui.horizontal(|ui| {
        ui.label(label);
        egui::ComboBox::from_id_salt(id)
            .selected_text(if current.is_empty() { "—".to_string() } else { current.clone() })
            .width(150.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(current, String::new(), "—");
                for frame in frames {
                    ui.selectable_value(current, frame.clone(), frame);
                }
            });
    });
}

fn slider(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f64,
    range: std::ops::RangeInclusive<f64>,
) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::DragValue::new(value).speed(0.01).range(range));
    });
}

fn joint_row(ui: &mut egui::Ui, label: &str, values: &mut [f64; 6]) {
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new(label).weak());
        for value in values.iter_mut() {
            ui.add(egui::DragValue::new(value).speed(0.01).max_decimals(5));
        }
    });
}
