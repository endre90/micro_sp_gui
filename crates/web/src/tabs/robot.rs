//! Driving the UR through `ur_redis_driver`.
//!
//! The old native tab composed the driver's `{robot_id}_*` variables by hand and
//! showed none of the feedback the driver publishes. This one builds a
//! `RobotCommand` and lets the server do the naming, and puts the driver's own
//! status and result strings on screen - which is what tells you *why* a request
//! failed rather than just that it did.
//!
//! # Why the layout is nailed down
//!
//! Every button here does something to a robot that is moving, and the panel
//! next to them shows values that change several times a second. Laid out as one
//! flowing column of `horizontal_wrapped` rows - which is what this was - a
//! longer driver message reflows a row, a field the driver has not published yet
//! is simply absent, and both of those move every button below them. Aiming at
//! "Stop" and pressing "Power off" because the row grew a line between the two
//! is not a recoverable mistake.
//!
//! So the tab is three regions of constant geometry rather than one column:
//!
//! * a status header of a fixed number of rows, where a value nobody has
//!   published is drawn as a dash instead of left out,
//! * the dashboard in its own side panel, every button the same size, all of
//!   them in one column at a fixed x,
//! * the motion request in the middle, its command picker pinned to the top of
//!   the panel and Send/Cancel pinned to the bottom, so only the parameters in
//!   between ever scroll.
//!
//! Text that comes and goes - a validation failure, an armed destructive command
//! - goes in a [`widgets::reserved_line`], which occupies its row whether or not
//! there is anything to say in it.

use crate::api::Api;
use crate::widgets;
use micro_sp_gui_protocol as proto;

/// Starting width of the dashboard side panel, and the range the drag handle
/// allows. Wide enough for two buttons and their argument slot.
const DASHBOARD_WIDTH: f32 = 470.0;
const DASHBOARD_MIN_WIDTH: f32 = 430.0;
const DASHBOARD_MAX_WIDTH: f32 = 760.0;

/// Every dashboard button is exactly this wide, so the labels do not decide the
/// geometry: "Stop" and "Unlock protective stop" occupy the same column.
const DASH_BUTTON_WIDTH: f32 = 190.0;
/// The slot a dashboard command's argument widget sits in, left empty for the
/// commands that take none. This is what puts the buttons at a fixed x whether
/// or not the row above them has a text field.
const DASH_ARG_WIDTH: f32 = 160.0;
/// Buttons per row for a group whose commands all take no argument. Fixed, not
/// derived from the panel width, so resizing the panel does not rearrange them.
const DASH_COLUMNS: usize = 2;

/// The Send / Write / Cancel buttons, all one size.
const ACTION_BUTTON_WIDTH: f32 = 160.0;
/// The command-type picker in the motion panel's header.
const COMMAND_PICKER_WIDTH: f32 = 240.0;
/// The frame pickers, all the same width so the four rows line up.
const FRAME_PICKER_WIDTH: f32 = 170.0;
/// The label column in the motion sections. Sized for the longest of them,
/// `global accel scaling`.
const MOTION_LABEL_WIDTH: f32 = 158.0;
/// The width at which the parameter column splits into two. Below it a single
/// column is the only thing wide enough for a six-value joint row, which is the
/// widest thing in this panel.
///
/// This is the one piece of geometry here that reacts to anything, and it reacts
/// to the operator dragging the panel divider - never to the robot.
const TWO_COLUMN_WIDTH: f32 = 920.0;

/// A single numeric field, and one of the six in a joint vector.
const NUMBER_WIDTH: f32 = 78.0;
const JOINT_WIDTH: f32 = 62.0;

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
    /// Which destructive dashboard command is armed, if any. See
    /// [`command_button`].
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

        egui::TopBottomPanel::top("robot_status").show_inside(ui, |ui| {
            self.status_header(ui, api);
        });

        egui::SidePanel::right("robot_dashboard")
            .resizable(true)
            .default_width(DASHBOARD_WIDTH)
            .width_range(DASHBOARD_MIN_WIDTH..=DASHBOARD_MAX_WIDTH)
            .show_inside(ui, |ui| {
                self.dashboard_panel(ui, api);
            });

        // No frame: the tab is already inside the app's central panel, and a
        // second set of margins would just inset the motion column from the
        // status header above it.
        egui::CentralPanel::default().frame(egui::Frame::NONE).show_inside(ui, |ui| {
            self.command_panel(ui, api);
        });
    }

    /// Three groups of five rows each, always in the same place.
    fn status_header(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        let status = api.robot_status().cloned();

        widgets::reserved_line(ui, |ui| {
            let text = if api.robot_id.is_none() {
                egui::RichText::new(
                    "No robot interface found in Redis. Start ur_redis_driver, or pass \
                     --robot-id to the server to address one that has not seeded its keys yet.",
                )
                .color(widgets::WARN)
            } else if status.is_none() {
                egui::RichText::new("No status published for this robot yet.").weak()
            } else {
                egui::RichText::new(format!("robot: {}", self.cmd.robot_id)).monospace().weak()
            };
            ui.add(egui::Label::new(text).truncate());
        });

        ui.columns(3, |columns| {
            connection_group(&mut columns[0], status.as_ref());
            request_group(&mut columns[1], status.as_ref());
            measured_group(&mut columns[2], status.as_ref());
        });
        ui.add_space(2.0);
    }

    // ---- the motion request -------------------------------------------------

    /// The middle column: a pinned header, a pinned action bar, and only the
    /// parameters scrolling in between.
    fn command_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        egui::TopBottomPanel::top("robot_command_header").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Motion request").strong());
                ui.add_space(8.0);
                egui::ComboBox::from_id_salt("command_type")
                    .selected_text(&self.cmd.command_type)
                    .width(COMMAND_PICKER_WIDTH)
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
        });

        // Bottom, not below the parameters: the two buttons that start and stop
        // a move are the ones that must never move, and down here their
        // position does not depend on anything the driver publishes.
        egui::TopBottomPanel::bottom("robot_command_actions").show_inside(ui, |ui| {
            self.action_bar(ui, api);
        });

        egui::ScrollArea::vertical()
            .id_salt("robot_parameters")
            .auto_shrink([false; 2])
            // Always visible, so the parameter column is not a few pixels
            // narrower once the content outgrows the panel.
            .scroll_bar_visibility(
                egui::containers::scroll_area::ScrollBarVisibility::AlwaysVisible,
            )
            .show(ui, |ui| self.parameters(ui, api));
    }

    /// The parameter sections, in one column or two depending on how much room
    /// the operator has left this panel.
    fn parameters(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        if ui.available_width() >= TWO_COLUMN_WIDTH {
            // Cloned rather than borrowed across the two closures: `columns`
            // hands out two `Ui`s at once and both halves need the frame list.
            let frames = api.frame_names();
            let mut copy_joints = false;
            ui.columns(2, |columns| {
                self.frames_section(&mut columns[0], &frames);
                self.joints_section(&mut columns[0], &mut copy_joints);
                self.relative_section(&mut columns[0]);
                self.motion_section(&mut columns[1]);
                self.payload_section(&mut columns[1]);
            });
            self.copy_joints_requested |= copy_joints;
        } else {
            let frames = api.frame_names();
            let mut copy_joints = false;
            self.frames_section(ui, &frames);
            self.motion_section(ui);
            self.joints_section(ui, &mut copy_joints);
            self.relative_section(ui);
            self.payload_section(ui);
            self.copy_joints_requested |= copy_joints;
        }

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
    }

    fn frames_section(&mut self, ui: &mut egui::Ui, frames: &[String]) {
        block(ui, "Frames", |ui| {
            widgets::reserved_line(ui, |ui| {
                let text = if self.cmd.needs_frames() {
                    egui::RichText::new("The server resolves these into the driver's goal.")
                        .weak()
                        .small()
                } else {
                    egui::RichText::new(
                        "This command does not resolve frames, so these are ignored.",
                    )
                    .color(widgets::WARN)
                    .small()
                };
                ui.add(egui::Label::new(text).truncate());
            });
            picker(ui, "goal_feature", "goal feature", &mut self.cmd.goal_feature_id, &frames);
            picker(ui, "tcp", "tcp", &mut self.cmd.tcp_id, &frames);
            picker(ui, "faceplate", "faceplate", &mut self.cmd.faceplate_id, &frames);
            picker(ui, "baseframe", "baseframe", &mut self.cmd.baseframe_id, &frames);
            ui.horizontal(|ui| {
                widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                    ui.label("root");
                });
                ui.add(
                    egui::TextEdit::singleline(&mut self.cmd.root_frame_id)
                        .desired_width(FRAME_PICKER_WIDTH),
                )
                .on_hover_text("Read by the driver but currently unused by it.");
            });
        });
    }

    fn motion_section(&mut self, ui: &mut egui::Ui) {
        block(ui, "Motion", |ui| {
            number_row(ui, "acceleration", &mut self.cmd.acceleration, 0.0..=5.0, 0.01);
            number_row(ui, "velocity", &mut self.cmd.velocity, 0.0..=5.0, 0.01);
            // Both of these are read by the driver; the old GUI had the fields
            // but never wrote them.
            number_row(
                ui,
                "global accel scaling",
                &mut self.cmd.global_acceleration_scaling,
                0.0..=1.0,
                0.01,
            );
            number_row(
                ui,
                "global vel scaling",
                &mut self.cmd.global_velocity_scaling,
                0.0..=1.0,
                0.01,
            );
            ui.separator();
            toggle_number_row(
                ui,
                "execution time",
                &mut self.cmd.use_execution_time,
                &mut self.cmd.execution_time,
                0.1,
                " s",
            );
            toggle_number_row(
                ui,
                "blend radius",
                &mut self.cmd.use_blend_radius,
                &mut self.cmd.blend_radius,
                0.001,
                " m",
            );
            number_row(ui, "force threshold", &mut self.cmd.force_threshold, 0.0..=500.0, 0.5);
        });
    }

    /// `copy_joints` is an out-parameter rather than a write to
    /// `self.copy_joints_requested`, so this can be called from inside a
    /// `columns` closure that has already borrowed the tab.
    fn joints_section(&mut self, ui: &mut egui::Ui, copy_joints: &mut bool) {
        block(ui, "Joints", |ui| {
            ui.horizontal(|ui| {
                widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                    ui.checkbox(&mut self.cmd.use_joint_positions, "use joint positions")
                        .on_hover_text("Skips the frame lookup entirely.");
                });
                if ui
                    .add(egui::Button::new("Copy from robot"))
                    .on_hover_text("Fill the target row with the robot's current joint states.")
                    .clicked()
                {
                    *copy_joints = true;
                }
            });
            joint_row(ui, "target", &mut self.cmd.joint_positions);
            ui.separator();
            ui.horizontal(|ui| {
                widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                    ui.checkbox(
                        &mut self.cmd.use_preferred_joint_config,
                        "prefer a configuration",
                    )
                    .on_hover_text("Guides the inverse kinematics towards this configuration.");
                });
            });
            joint_row(ui, "preferred", &mut self.cmd.preferred_joint_config);
        });
    }

    fn relative_section(&mut self, ui: &mut egui::Ui) {
        block(ui, "Relative pose", |ui| {
            ui.horizontal(|ui| {
                widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                    ui.checkbox(&mut self.cmd.use_relative_pose, "move relative")
                        .on_hover_text("Also skips the frame lookup.");
                });
            });
            joint_row(ui, "dx dy dz rx ry rz", &mut self.cmd.relative_pose);
        });
    }

    /// Last in its column on purpose: switching between the preset and the
    /// manual fields changes this block's height, and nothing below it can be
    /// pushed around by that if there is nothing below it.
    fn payload_section(&mut self, ui: &mut egui::Ui) {
        block(ui, "Payload", |ui| {
            ui.horizontal(|ui| {
                widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                    ui.checkbox(&mut self.cmd.use_payload, "set the payload");
                });
                ui.checkbox(&mut self.manual_payload, "enter it manually");
            });

            if self.manual_payload {
                ui.horizontal(|ui| {
                    widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                        ui.label("mass");
                    });
                    number(ui, &mut self.payload.mass, 0.01, " kg");
                });
                ui.horizontal(|ui| {
                    widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                        ui.label("cog x y z");
                    });
                    for value in
                        [&mut self.payload.cog_x, &mut self.payload.cog_y, &mut self.payload.cog_z]
                    {
                        number(ui, value, 0.001, "");
                    }
                });
                ui.horizontal(|ui| {
                    widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                        ui.label("ixx iyy izz");
                    });
                    for value in
                        [&mut self.payload.ixx, &mut self.payload.iyy, &mut self.payload.izz]
                    {
                        number(ui, value, 0.001, "");
                    }
                });
                ui.horizontal(|ui| {
                    widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                        ui.label("ixy ixz iyz");
                    });
                    for value in
                        [&mut self.payload.ixy, &mut self.payload.ixz, &mut self.payload.iyz]
                    {
                        number(ui, value, 0.001, "");
                    }
                });
                self.cmd.payload = self.payload.to_string();
            } else {
                ui.horizontal(|ui| {
                    widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
                        ui.label("preset");
                    });
                    egui::ComboBox::from_id_salt("payload_preset")
                        .selected_text(proto::PAYLOAD_PRESETS[self.payload_preset].0)
                        .width(FRAME_PICKER_WIDTH)
                        .show_ui(ui, |ui| {
                            for (i, (label, _)) in proto::PAYLOAD_PRESETS.iter().enumerate() {
                                ui.selectable_value(&mut self.payload_preset, i, *label);
                            }
                        });
                });
                self.cmd.payload = proto::PAYLOAD_PRESETS[self.payload_preset].1.to_string();
            }

            widgets::kv(
                ui,
                "as written",
                egui::RichText::new(&self.cmd.payload).monospace().weak().small(),
                &self.cmd.payload,
            );
        });
    }

    /// Send, stage and cancel, in a bar of constant height at the bottom of the
    /// motion panel.
    fn action_bar(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        // Say up front what the driver would reject, instead of after the fact.
        let problem = self.cmd.validate().err();

        widgets::reserved_line(ui, |ui| {
            let text = match &problem {
                Some(problem) => {
                    egui::RichText::new(format!("⚠ {problem}")).color(widgets::WARN)
                }
                None if !api.conn.is_live() => {
                    egui::RichText::new("Offline: nothing can be sent.").color(widgets::BAD)
                }
                None => egui::RichText::new("Ready to send.").weak(),
            };
            ui.add(egui::Label::new(text).truncate());
        });

        let size = egui::vec2(ACTION_BUTTON_WIDTH, ui.spacing().interact_size.y);
        ui.horizontal(|ui| {
            let can_send = problem.is_none() && api.conn.is_live();
            let send = ui.add_enabled(
                can_send,
                egui::Button::new(egui::RichText::new("Send command").strong()).min_size(size),
            );
            if send.clicked() {
                let mut cmd = self.cmd.clone();
                cmd.trigger = true;
                api.robot_command(cmd);
            }

            if ui
                .add_enabled(
                    api.robot_id.is_some(),
                    egui::Button::new("Write, do not trigger").min_size(size),
                )
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
                    egui::Button::new(egui::RichText::new("Cancel").color(widgets::BAD))
                        .min_size(size),
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
        ui.add_space(2.0);
    }

    // ---- the dashboard ------------------------------------------------------

    /// The driver's dashboard interface: its own status block, then one button
    /// per command it accepts.
    ///
    /// Every button is generated from `proto::DASHBOARD_COMMANDS`, so a command
    /// the driver gains appears here by editing that table and nothing else.
    fn dashboard_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Dashboard").strong());
            ui.label(egui::RichText::new("port 29999").weak().small()).on_hover_text(
                "A separate handshake from motion requests, so a slow dashboard round trip \
                 cannot stall the driver's control loop. A query's answer comes back in \
                 'reply'.",
            );
        });
        ui.separator();

        let status = api.robot_status().cloned();
        dashboard_status(ui, status.as_ref());
        ui.separator();

        // Above the buttons and permanently allocated: an arming warning that
        // appeared below them would push the whole column down by a row at the
        // exact moment the operator is about to click the second time.
        widgets::reserved_line(ui, |ui| {
            if let Some(armed) = self.pending_danger.and_then(proto::dashboard_spec) {
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(format!(
                            "⚠ '{}' is armed - click it again to send it, or any other \
                             command to drop it.",
                            armed.label
                        ))
                        .color(widgets::WARN),
                    )
                    .truncate(),
                );
            }
        });

        egui::ScrollArea::vertical()
            .id_salt("robot_dashboard_commands")
            .auto_shrink([false; 2])
            .scroll_bar_visibility(
                egui::containers::scroll_area::ScrollBarVisibility::AlwaysVisible,
            )
            .show(ui, |ui| self.dashboard_commands(ui, api));
    }

    fn dashboard_commands(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        // Collected while drawing, sent afterwards: the rows below borrow the
        // tab's own fields, and `api` is needed to send.
        let mut to_send: Vec<(&'static str, Option<String>)> = Vec::new();
        let enabled = api.robot_id.is_some();
        let args = &mut self.dashboard_args;
        let pending = &mut self.pending_danger;

        for (index, group) in proto::DashboardGroup::ALL.iter().enumerate() {
            if index > 0 {
                ui.add_space(4.0);
                ui.separator();
            }

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
                    group_block(ui, *group, args, pending, enabled, &mut to_send);
                });
            } else {
                ui.label(egui::RichText::new(group.title()).strong());
                group_block(ui, *group, args, pending, enabled, &mut to_send);
            }
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
}

// ---- the status blocks ------------------------------------------------------

/// One titled block that fills the width it is given, so the widgets inside it
/// all start at the same x no matter what is in the blocks around it.
fn block(ui: &mut egui::Ui, title: &str, rows: impl FnOnce(&mut egui::Ui)) {
    ui.group(|ui| {
        ui.set_min_width(ui.available_width());
        ui.label(egui::RichText::new(title).strong());
        ui.separator();
        rows(ui);
    });
    ui.add_space(4.0);
}

/// The driver's two sockets and what the controller says about itself.
fn connection_group(ui: &mut egui::Ui, status: Option<&proto::RobotStatus>) {
    block(ui, "Connection", |ui| {
        ui.horizontal(|ui| {
            widgets::fixed_slot(ui, widgets::KV_LABEL_WIDTH, |ui| {
                ui.label(egui::RichText::new("link").weak());
            });
            widgets::light(
                ui,
                status.and_then(|s| s.robot_connected),
                "robot",
                "The driver's realtime connection to the controller.",
            );
            ui.add_space(10.0);
            widgets::light(
                ui,
                status.and_then(|s| s.dashboard_connected),
                "dashboard",
                "The driver's dashboard connection.",
            );
        });

        // Safety mode decides whether a goal is admitted at all.
        let safety = status.map_or("", |s| s.safety_mode.as_str());
        let safety_colour = match safety {
            "NORMAL" | "REDUCED" => widgets::OK,
            "" | "UNKNOWN" => widgets::UNKNOWN,
            _ => widgets::BAD,
        };
        widgets::kv(
            ui,
            "safety",
            widgets::cell(safety).color(safety_colour).strong(),
            "The driver only accepts a goal in NORMAL or REDUCED.",
        );

        widgets::kv(
            ui,
            "mode",
            widgets::cell(status.map_or("", |s| s.robot_mode.as_str())),
            "The controller's robot mode: RUNNING, IDLE, POWER_OFF and so on.",
        );

        // `program_state` describes the loaded pendant program, so it sits at
        // STOPPED while an injected script runs; `program_running` is the one
        // that tracks this driver's scripts.
        let script = match status {
            Some(status) if status.program_running => "running",
            Some(_) => "idle",
            None => "",
        };
        widgets::kv(
            ui,
            "script",
            widgets::cell(script),
            &match status {
                Some(status) => format!(
                    "program_running={} (the pendant's own program_state is '{}', which stays \
                     STOPPED while an injected script runs)",
                    status.program_running, status.program_state
                ),
                None => "Nothing has published this yet.".to_string(),
            },
        );

        // Local control is the most common reason a dashboard action is refused,
        // and the controller's own "Failed to execute" does not say so - so say
        // it here, before the click.
        let (control, control_colour) = match status {
            Some(status) if status.remote_control => ("remote", widgets::OK),
            Some(_) => ("local", widgets::WARN),
            None => ("", widgets::UNKNOWN),
        };
        widgets::kv(
            ui,
            "control",
            widgets::cell(control).color(control_colour),
            "Most dashboard commands are marked 'Only Remote Control' in UR's manual and the \
             controller refuses them in Local control.",
        );
    });
}

/// The motion request handshake, and the reason behind whatever state it is in.
fn request_group(ui: &mut egui::Ui, status: Option<&proto::RobotStatus>) {
    block(ui, "Request", |ui| {
        let (state, state_colour) = match status {
            // A cancel terminates as `succeeded` with result `cancelled`.
            // Painting that red would be a lie.
            Some(status) if status.was_cancelled() => ("cancelled", widgets::WARN),
            Some(status) if status.request_failed() => ("failed", widgets::BAD),
            Some(status) => match status.request_state.as_str() {
                "succeeded" => ("succeeded", widgets::OK),
                "executing" => ("executing", widgets::WARN),
                "initial" => ("initial", widgets::UNKNOWN),
                other => (other, widgets::UNKNOWN),
            },
            None => ("", widgets::UNKNOWN),
        };
        widgets::kv(
            ui,
            "state",
            widgets::cell(state).color(state_colour).strong(),
            "The driver's motion handshake: initial → executing → succeeded|failed.",
        );

        let result = status.map_or("", |s| s.request_result.as_str());
        widgets::kv(ui, "result", widgets::cell(result), &hover_or(result, "The driver's own reason for the outcome."));

        let feedback = status.map_or("", |s| s.request_feedback.as_str());
        widgets::kv(
            ui,
            "feedback",
            widgets::cell(feedback).weak(),
            &hover_or(feedback, "The last line the running URScript sent back."),
        );

        let (fails, fails_colour) = match status {
            Some(status) => (
                format!(
                    "{} / {}",
                    status.subsequent_fail_counter, status.total_fail_counter
                ),
                if status.subsequent_fail_counter > 0 { widgets::BAD } else { widgets::UNKNOWN },
            ),
            None => (String::new(), widgets::UNKNOWN),
        };
        widgets::kv(
            ui,
            "fails",
            widgets::cell(&fails).color(fails_colour),
            "Consecutive / total. The consecutive count resets on success.",
        );

        // The driver's own latch, not the controller's program state: while it
        // is set no new move is admitted, so a request that just sits there is
        // explained by this and nothing else.
        let (motion, motion_colour) = match status {
            Some(status) if status.motion_paused => ("paused", widgets::WARN),
            Some(_) => ("free", widgets::OK),
            None => ("", widgets::UNKNOWN),
        };
        widgets::kv(
            ui,
            "motion",
            widgets::cell(motion).color(motion_colour),
            "The driver's pause latch. While it is set it holds off new moves until a \
             dashboard 'Resume' clears it.",
        );
    });
}

/// What the robot is actually doing right now.
fn measured_group(ui: &mut egui::Ui, status: Option<&proto::RobotStatus>) {
    block(ui, "Measured", |ui| {
        let joints = vector_cell(status.map(|s| s.joint_states.as_slice()));
        widgets::kv(ui, "joints", widgets::cell(&joints), &joints);

        let tcp = vector_cell(status.map(|s| s.tcp_pose.as_slice()));
        widgets::kv(ui, "tcp", widgets::cell(&tcp), &tcp);

        let force = status.map_or(String::new(), |s| format!("{:.2} N", s.force_feedback));
        widgets::kv(ui, "force", widgets::cell(&force), "");

        let speed = status.map_or(String::new(), |s| format!("×{:.2}", s.speed_scaling));
        widgets::kv(ui, "speed", widgets::cell(&speed), "The controller's speed slider.");

        widgets::kv(
            ui,
            "op mode",
            widgets::cell(status.map_or("", |s| s.operational_mode.as_str())),
            "MANUAL, AUTOMATIC, or NONE. NONE means no mode password has been set, so the mode \
             is simply not in use on this robot.",
        );
    });
}

/// What the dashboard half of the driver is currently reporting: five rows,
/// always the same five.
fn dashboard_status(ui: &mut egui::Ui, status: Option<&proto::RobotStatus>) {
    ui.horizontal(|ui| {
        widgets::fixed_slot(ui, widgets::KV_LABEL_WIDTH, |ui| {
            ui.label(egui::RichText::new("link").weak());
        });
        widgets::light(
            ui,
            status.and_then(|s| s.dashboard_connected),
            "dashboard",
            "The driver's socket to port 29999. Nothing below can work without it.",
        );
    });

    let (last, last_colour) = match status.map(|s| s.dashboard_request_state.as_str()) {
        Some("succeeded") => ("succeeded", widgets::OK),
        Some("failed") => ("failed", widgets::BAD),
        Some("executing") => ("executing", widgets::WARN),
        Some(other) => (other, widgets::UNKNOWN),
        None => ("", widgets::UNKNOWN),
    };
    widgets::kv(
        ui,
        "last",
        widgets::cell(last).color(last_colour).strong(),
        "The driver's dashboard handshake state: initial → executing → succeeded|failed.",
    );

    let reply = status.map_or("", |s| s.dashboard_request_result.as_str());
    let reply_colour = match status {
        Some(status) if status.dashboard_failed() => widgets::BAD,
        _ => ui.visuals().text_color(),
    };
    widgets::kv(
        ui,
        "reply",
        widgets::cell(reply).color(reply_colour),
        &hover_or(
            reply,
            "The controller's own reply, verbatim. For a query this line is the answer.",
        ),
    );

    // Read once per dashboard connection, and blanked rather than left stale
    // when that socket drops - so a dash here means "no dashboard".
    widgets::kv(
        ui,
        "controller",
        widgets::cell(status.map_or("", |s| s.robot_model.as_str())),
        &format!("Serial {}.", display(status.map_or("", |s| s.serial_number.as_str()))),
    );
    widgets::kv(
        ui,
        "polyscope",
        widgets::cell(status.map_or("", |s| s.polyscope_version.as_str())),
        "",
    );
}

/// One group of dashboard commands.
///
/// A group whose commands all take no argument is laid out in a fixed number of
/// columns; anything else gets one command per row, its argument widget in a
/// slot of its own. Either way every button is the same size and sits at an x
/// that does not depend on the labels around it.
fn group_block(
    ui: &mut egui::Ui,
    group: proto::DashboardGroup,
    args: &mut std::collections::BTreeMap<&'static str, String>,
    pending: &mut Option<&'static str>,
    enabled: bool,
    to_send: &mut Vec<(&'static str, Option<String>)>,
) {
    let specs: Vec<&'static proto::DashboardCommandSpec> =
        proto::dashboard_commands_in(group).collect();
    let bare = specs.iter().all(|spec| !spec.takes_arg());

    if bare {
        for chunk in specs.chunks(DASH_COLUMNS) {
            ui.horizontal(|ui| {
                for spec in chunk {
                    let arg = args.entry(spec.name).or_default().clone();
                    command_button(ui, spec, &arg, pending, enabled, to_send);
                }
            });
        }
        return;
    }

    for spec in specs {
        ui.horizontal(|ui| {
            // A choice starts on the driver's own default, which the table names
            // first; free text starts empty.
            let arg = args.entry(spec.name).or_insert_with(|| {
                spec.arg.options().first().copied().unwrap_or_default().to_string()
            });
            widgets::fixed_slot(ui, DASH_ARG_WIDTH, |ui| arg_widget(ui, spec, arg));
            let arg = arg.clone();
            command_button(ui, spec, &arg, pending, enabled, to_send);
        });
    }
}

/// The argument widget for one command, at a fixed width so the button after it
/// does not move when the text does.
fn arg_widget(ui: &mut egui::Ui, spec: &proto::DashboardCommandSpec, arg: &mut String) {
    match spec.arg {
        proto::DashboardArg::None => {}
        proto::DashboardArg::Text { hint, .. } => {
            ui.add(
                egui::TextEdit::singleline(arg)
                    .hint_text(hint)
                    .desired_width(DASH_ARG_WIDTH - 8.0),
            );
        }
        proto::DashboardArg::Choice { options, .. } => {
            egui::ComboBox::from_id_salt(spec.name)
                .selected_text(arg.clone())
                .width(DASH_ARG_WIDTH - 36.0)
                .show_ui(ui, |ui| {
                    for option in options {
                        ui.selectable_value(arg, option.to_string(), *option);
                    }
                });
        }
    }
}

/// One dashboard button, always [`DASH_BUTTON_WIDTH`] wide.
///
/// `pending` arms the destructive commands: the first click on one only marks
/// it, and a second click sends it. Powering the arm down or restarting the
/// safety system in the middle of a shift because a button was next to the one
/// being aimed at is not a recoverable mistake. The armed state changes the
/// button's colour and nothing about its size - the warning text it also raises
/// is drawn in a row reserved for it at the top of the panel.
fn command_button(
    ui: &mut egui::Ui,
    spec: &'static proto::DashboardCommandSpec,
    arg: &str,
    pending: &mut Option<&'static str>,
    enabled: bool,
    to_send: &mut Vec<(&'static str, Option<String>)>,
) {
    let armed = *pending == Some(spec.name);

    let mut text = egui::RichText::new(spec.label);
    if spec.danger {
        text = text.color(widgets::BAD);
    } else if spec.query {
        // Read-only, so they are drawn back: a dozen of them in one column
        // would otherwise shout as loudly as the ones that move the robot.
        text = text.weak();
    }
    if armed {
        text = text.strong().color(widgets::BAD);
    }

    let size = egui::vec2(DASH_BUTTON_WIDTH, ui.spacing().interact_size.y);
    let ready = enabled && spec.arg_is_satisfied(arg);
    let clicked = ui
        .add_enabled(ready, egui::Button::new(text).min_size(size))
        .on_hover_text(hover_text(spec, armed))
        .clicked();

    if clicked {
        if spec.danger && !armed {
            *pending = Some(spec.name);
        } else {
            *pending = None;
            to_send.push((spec.name, if spec.takes_arg() { Some(arg.to_string()) } else { None }));
        }
    }
}

/// The tooltip for one command: what it does, what actually goes on the socket,
/// and the two facts that explain most refusals.
fn hover_text(spec: &proto::DashboardCommandSpec, armed: bool) -> String {
    let mut text = format!("{}\n\nSends '{}' on port 29999.", spec.help, spec.wire);
    if spec.query {
        text.push_str("\n\nA query: the controller's answer comes back in 'reply'.");
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

// ---- the small pieces -------------------------------------------------------

fn picker(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    current: &mut String,
    frames: &[String],
) {
    ui.horizontal(|ui| {
        widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
            ui.label(label);
        });
        egui::ComboBox::from_id_salt(id)
            .selected_text(if current.is_empty() { "—".to_string() } else { current.clone() })
            .width(FRAME_PICKER_WIDTH)
            .show_ui(ui, |ui| {
                ui.selectable_value(current, String::new(), "—");
                for frame in frames {
                    ui.selectable_value(current, frame.clone(), frame);
                }
            });
    });
}

fn number(ui: &mut egui::Ui, value: &mut f64, speed: f64, suffix: &str) {
    let size = egui::vec2(NUMBER_WIDTH, ui.spacing().interact_size.y);
    ui.add_sized(size, egui::DragValue::new(value).speed(speed).suffix(suffix));
}

fn number_row(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f64,
    range: std::ops::RangeInclusive<f64>,
    speed: f64,
) {
    ui.horizontal(|ui| {
        widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
            ui.label(label);
        });
        let size = egui::vec2(NUMBER_WIDTH, ui.spacing().interact_size.y);
        ui.add_sized(size, egui::DragValue::new(value).speed(speed).range(range));
    });
}

/// A checkbox that owns the field beside it: the field stays in place and is
/// greyed out rather than disappearing when the box is cleared.
fn toggle_number_row(
    ui: &mut egui::Ui,
    label: &str,
    on: &mut bool,
    value: &mut f64,
    speed: f64,
    suffix: &str,
) {
    ui.horizontal(|ui| {
        widgets::fixed_slot(ui, MOTION_LABEL_WIDTH, |ui| {
            ui.checkbox(on, label);
        });
        let enabled = *on;
        ui.add_enabled_ui(enabled, |ui| number(ui, value, speed, suffix));
    });
}

/// Six numbers with their caption above rather than beside them.
///
/// Beside them the row is wider than half the motion panel, which is what
/// decides [`TWO_COLUMN_WIDTH`]; above them the six fields fit, and each stays
/// wide enough for a full `-1.57080`.
fn joint_row(ui: &mut egui::Ui, label: &str, values: &mut [f64; 6]) {
    ui.label(egui::RichText::new(label).weak());
    ui.horizontal(|ui| {
        let size = egui::vec2(JOINT_WIDTH, ui.spacing().interact_size.y);
        for value in values.iter_mut() {
            ui.add_sized(size, egui::DragValue::new(value).speed(0.01).max_decimals(5));
        }
    });
}

/// Six numbers on one line, the shape joint vectors and poses come in.
fn vector_cell(values: Option<&[f64]>) -> String {
    match values {
        Some(values) if !values.is_empty() => {
            values.iter().map(|v| format!("{v:.4}")).collect::<Vec<_>>().join(", ")
        }
        _ => String::new(),
    }
}

/// The hover for a driver string: the string itself when there is one, since it
/// is the row most likely to be truncated, and the explanation otherwise.
fn hover_or(value: &str, fallback: &str) -> String {
    if value.is_empty() || value == "UNKNOWN" {
        fallback.to_string()
    } else {
        format!("{value}\n\n{fallback}")
    }
}

fn display(value: &str) -> &str {
    if value.is_empty() || value == "UNKNOWN" { "unknown" } else { value }
}
