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

#[derive(Default)]
pub struct RobotTab {
    cmd: proto::RobotCommand,
    /// Which payload preset is selected. The old tab had one field backing three
    /// unrelated combo boxes; only the payload ever had presets behind it.
    payload_preset: usize,
    manual_payload: bool,
    payload: proto::Payload,

    /// The argument each dashboard command was last given, keyed by command
    /// name. Per-command rather than one shared field, so typing a program name
    /// does not silently become the text of the next popup.
    dashboard_args: std::collections::BTreeMap<&'static str, String>,
    /// Which destructive dashboard command is armed, if any. See `group_row`.
    pending_danger: Option<&'static str>,
    /// Filled from the live joint states on request, rather than every frame -
    /// otherwise the fields could never be edited.
    copy_joints_requested: bool,
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
            if status.motion_paused {
                ui.separator();
                // The driver's own latch, not the controller's program state:
                // while it is set no new move is admitted, so a request that
                // just sits there is explained by this and nothing else.
                ui.label(egui::RichText::new("paused").color(widgets::WARN).strong())
                    .on_hover_text(
                        "The driver's pause latch is set. It holds off new moves until a                          dashboard 'Resume' clears it.",
                    );
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

    /// The driver's dashboard interface, one button per command it accepts.
    ///
    /// Every row is generated from `proto::DASHBOARD_COMMANDS`, so a command the
    /// driver gains appears here by editing that table and nothing else.
    fn dashboard_section(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.label(egui::RichText::new("Dashboard").strong());
        ui.label(
            egui::RichText::new(
                "A separate handshake from motion requests, so a slow dashboard round trip \
                 cannot stall the driver's control loop. A query's answer comes back in \
                 'dashboard says' below.",
            )
            .weak()
            .small(),
        );

        let status = api.robot_status().cloned();
        self.dashboard_status(ui, status.as_ref());

        // Collected while drawing, sent afterwards: the rows below borrow the
        // tab's own fields, and `api` is needed to send.
        let mut to_send: Vec<(&'static str, Option<String>)> = Vec::new();
        let enabled = api.robot_id.is_some();
        let args = &mut self.dashboard_args;
        let pending = &mut self.pending_danger;

        for group in proto::DashboardGroup::ALL {
            // The two slow ones - minutes, not seconds - stay folded away.
            if *group == proto::DashboardGroup::Diagnostics {
                egui::CollapsingHeader::new(group.title()).default_open(false).show(ui, |ui| {
                    ui.label(
                        egui::RichText::new(
                            "These do not answer until the controller has finished: up to 5 \
                             minutes for a flight report, 10 for a support file.",
                        )
                        .weak()
                        .small(),
                    );
                    group_row(ui, *group, args, pending, enabled, &mut to_send);
                });
            } else {
                group_row(ui, *group, args, pending, enabled, &mut to_send);
            }
        }

        if let Some(armed) = pending.and_then(proto::dashboard_spec) {
            ui.label(
                egui::RichText::new(format!(
                    "⚠ '{}' is armed - click it again to send it, or click any other command \
                     to drop it.",
                    armed.label
                ))
                .color(widgets::WARN),
            );
        }

        for (command, arg) in to_send {
            if let Some(robot_id) = api.robot_id.clone() {
                api.robot_dashboard(proto::RobotDashboardCommand {
                    robot_id,
                    command: command.to_string(),
                    arg,
                });
            }
        }
    }

    /// What the dashboard half of the driver is currently reporting.
    fn dashboard_status(&self, ui: &mut egui::Ui, status: Option<&proto::RobotStatus>) {
        let Some(status) = status else { return };

        ui.horizontal_wrapped(|ui| {
            widgets::light(
                ui,
                status.dashboard_connected,
                "dashboard",
                "The driver's socket to port 29999. Nothing below can work without it.",
            );
            ui.separator();

            // Local control is the most common reason a dashboard action is
            // refused, and the controller's own "Failed to execute" does not
            // say so - so say it here, before the click.
            let (remote_label, remote_colour) = if status.remote_control {
                ("remote control", widgets::OK)
            } else {
                ("local control", widgets::WARN)
            };
            ui.label(egui::RichText::new(remote_label).color(remote_colour))
                .on_hover_text(
                    "Most of these commands are marked 'Only Remote Control' in UR's manual \
                     and the controller refuses them in Local control.",
                );

            ui.separator();
            ui.label(egui::RichText::new("op mode").weak());
            ui.label(egui::RichText::new(&status.operational_mode).monospace()).on_hover_text(
                "MANUAL, AUTOMATIC, or NONE. NONE means no mode password has been set, so the \
                 mode is simply not in use on this robot.",
            );

            if status.motion_paused {
                ui.separator();
                ui.label(egui::RichText::new("motion paused").color(widgets::WARN).strong())
                    .on_hover_text(
                        "The driver's own pause latch is set, so it will hold off new moves \
                         until Resume clears it.",
                    );
            }
        });

        // The request row for this handshake, separate from the motion one.
        ui.horizontal_wrapped(|ui| {
            let (label, colour) = match status.dashboard_request_state.as_str() {
                "succeeded" => ("succeeded", widgets::OK),
                "failed" => ("failed", widgets::BAD),
                "executing" => ("executing", widgets::WARN),
                other => (other, widgets::UNKNOWN),
            };
            ui.label(egui::RichText::new("last command").weak());
            ui.label(egui::RichText::new(label).color(colour).strong()).on_hover_text(
                "The driver's dashboard handshake state: initial → executing → \
                 succeeded|failed.",
            );

            if status.dashboard_request_result != "UNKNOWN"
                && !status.dashboard_request_result.is_empty()
            {
                ui.separator();
                ui.label(egui::RichText::new("dashboard says").weak());
                let colour = if status.dashboard_failed() {
                    widgets::BAD
                } else {
                    ui.visuals().text_color()
                };
                ui.label(
                    egui::RichText::new(&status.dashboard_request_result)
                        .monospace()
                        .color(colour),
                )
                .on_hover_text(
                    "The controller's own reply, verbatim. For a query this line is the answer.",
                );
            }
        });

        // Read once per dashboard connection, and blanked rather than left stale
        // when that socket drops - so an empty row here means "no dashboard".
        if status.robot_model != "UNKNOWN" && !status.robot_model.is_empty() {
            ui.horizontal_wrapped(|ui| {
                widgets::field(ui, "controller", &status.robot_model);
                ui.separator();
                widgets::field(ui, "serial", &status.serial_number);
                ui.separator();
                widgets::field(ui, "polyscope", &status.polyscope_version);
            });
        }

        ui.separator();
    }
}

/// One row of the dashboard panel: every command in `group`, each preceded by
/// its argument widget when it takes one.
///
/// `pending` arms the destructive commands: the first click on one only marks
/// it, and a second click sends it. Powering the arm down or restarting the
/// safety system in the middle of a shift because a button was next to the one
/// being aimed at is not a recoverable mistake.
fn group_row(
    ui: &mut egui::Ui,
    group: proto::DashboardGroup,
    args: &mut std::collections::BTreeMap<&'static str, String>,
    pending: &mut Option<&'static str>,
    enabled: bool,
    to_send: &mut Vec<(&'static str, Option<String>)>,
) {
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new(group.title()).weak());

        for spec in proto::dashboard_commands_in(group) {
            // A choice starts on the driver's own default, which the table names
            // first; free text starts empty.
            let arg = args.entry(spec.name).or_insert_with(|| {
                spec.arg.options().first().copied().unwrap_or_default().to_string()
            });

            match spec.arg {
                proto::DashboardArg::None => {}
                proto::DashboardArg::Text { hint, .. } => {
                    ui.add(
                        egui::TextEdit::singleline(arg)
                            .hint_text(hint)
                            .desired_width(150.0),
                    );
                }
                proto::DashboardArg::Choice { options, .. } => {
                    egui::ComboBox::from_id_salt(spec.name)
                        .selected_text(arg.clone())
                        .width(110.0)
                        .show_ui(ui, |ui| {
                            for option in options {
                                ui.selectable_value(arg, option.to_string(), *option);
                            }
                        });
                }
            }

            let armed = *pending == Some(spec.name);
            let mut text = egui::RichText::new(if armed {
                format!("{} ‽", spec.label)
            } else {
                spec.label.to_string()
            });
            if spec.danger {
                text = text.color(widgets::BAD);
            }
            if armed {
                text = text.strong();
            }

            let ready = enabled && spec.arg_is_satisfied(arg);
            // A dozen queries in one row would dominate the panel; they are
            // read-only, so they get the small treatment.
            let mut button = egui::Button::new(text);
            if spec.query {
                button = button.small();
            }
            let clicked = ui
                .add_enabled(ready, button)
                .on_hover_text(hover_text(spec, armed))
                .clicked();

            if clicked {
                if spec.danger && !armed {
                    *pending = Some(spec.name);
                } else {
                    *pending = None;
                    to_send.push((
                        spec.name,
                        if spec.takes_arg() { Some(arg.clone()) } else { None },
                    ));
                }
            }
        }
    });
}

/// The tooltip for one command: what it does, what actually goes on the socket,
/// and the two facts that explain most refusals.
fn hover_text(spec: &proto::DashboardCommandSpec, armed: bool) -> String {
    let mut text = format!("{}\n\nSends '{}' on port 29999.", spec.help, spec.wire);
    if spec.query {
        text.push_str("\n\nA query: the controller's answer comes back in 'dashboard says'.");
    }
    if spec.remote_control {
        text.push_str("\n\nUR marks this 'Only Remote Control' - the controller refuses it in \
                       Local control.");
    }
    if spec.danger {
        text.push_str(if armed {
            "\n\nArmed. Click again to send it."
        } else {
            "\n\nDestructive: the first click only arms it."
        });
    }
    text
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
