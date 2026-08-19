//! The transform tree: browse it, move frames around, resolve lookups, and
//! export a frame as a scenario file.
//!
//! Replaces the old "Transforms" placeholder and absorbs the old Lookup tab,
//! whose real job was producing a frame JSON file with joint-configuration
//! metadata. That used a native file dialog, which a browser cannot offer, so the
//! server writes the file instead.

use crate::api::Api;
use crate::widgets::{self, tree::transform_tree, value_detail, value_editor};
use micro_sp_gui_protocol as proto;

pub struct TransformsTab {
    selected: Option<String>,
    /// Pose being edited, and the frame it was taken from - so switching frames
    /// discards the draft instead of applying it to the wrong one.
    draft: Option<(String, proto::GuiTransform)>,
    reparent_to: String,

    lookup_parent: String,
    lookup_child: String,
    lookup_result: Option<proto::LookupResponse>,

    export_filename: String,
    export_tcp_id: String,
    export_result: Option<proto::ExportFrameResponse>,

    new_parent: String,
    new_child: String,
    show_new_frame: bool,
    show_tree_text: bool,
}

impl Default for TransformsTab {
    fn default() -> Self {
        Self {
            selected: None,
            draft: None,
            reparent_to: String::new(),
            lookup_parent: String::new(),
            lookup_child: String::new(),
            lookup_result: None,
            export_filename: String::new(),
            export_tcp_id: String::new(),
            export_result: None,
            new_parent: "world".to_string(),
            new_child: String::new(),
            show_new_frame: false,
            show_tree_text: false,
        }
    }
}

impl TransformsTab {
    pub fn ui(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        // Claim any responses aimed at this tab.
        if let Some(result) = api.take_lookup() {
            self.lookup_result = Some(result);
        }
        if let Some(result) = api.take_export() {
            self.export_result = Some(result);
        }

        if api.transforms.cyclic {
            ui.label(
                egui::RichText::new(
                    "⚠ The transform tree contains a cycle. Every lookup through it will fail \
                     until a frame is reparented.",
                )
                .color(widgets::BAD)
                .strong(),
            );
            ui.separator();
        }

        egui::SidePanel::left("tf_tree")
            .resizable(true)
            .default_width(320.0)
            .show_inside(ui, |ui| self.tree_panel(ui, api));

        egui::ScrollArea::vertical().show(ui, |ui| {
            self.frame_panel(ui, api);
            ui.separator();
            self.lookup_panel(ui, api);
            ui.separator();
            self.export_panel(ui, api);
        });
    }

    fn tree_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.horizontal(|ui| {
            ui.heading("Frames");
            ui.label(
                egui::RichText::new(format!("{}", api.transforms.frames.len())).weak(),
            );
            if ui.small_button("➕").on_hover_text("Insert a new frame").clicked() {
                self.show_new_frame = !self.show_new_frame;
            }
            ui.toggle_value(&mut self.show_tree_text, "text")
                .on_hover_text("The tree as micro_sp renders it, for copying.");
        });

        if self.show_new_frame {
            ui.group(|ui| {
                ui.horizontal(|ui| {
                    ui.label("parent");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.new_parent).desired_width(90.0),
                    );
                    ui.label("child");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.new_child).desired_width(90.0),
                    );
                });
                let valid =
                    !self.new_child.trim().is_empty() && !self.new_parent.trim().is_empty();
                if ui.add_enabled(valid, egui::Button::new("Insert at parent origin")).clicked()
                {
                    let frame = value_editor::new_frame(
                        self.new_parent.trim(),
                        self.new_child.trim(),
                    );
                    api.transform_command(proto::TransformCommand::Insert {
                        frames: vec![frame],
                    });
                    self.selected = Some(self.new_child.trim().to_string());
                    self.new_child.clear();
                    self.show_new_frame = false;
                }
            });
        }

        ui.separator();
        egui::ScrollArea::vertical().id_salt("tf_tree_scroll").show(ui, |ui| {
            if self.show_tree_text {
                if api.transforms.tree_text.is_empty() {
                    ui.label(egui::RichText::new("No frames in Redis.").weak());
                } else {
                    let mut text = api.transforms.tree_text.clone();
                    ui.add(
                        egui::TextEdit::multiline(&mut text)
                            .code_editor()
                            .desired_width(f32::INFINITY)
                            .interactive(false),
                    );
                    if ui.button("Copy").clicked() {
                        ui.ctx().copy_text(api.transforms.tree_text.clone());
                    }
                }
            } else if transform_tree(ui, &api.transforms.frames, &mut self.selected) {
                // Selecting a different frame must not carry the old pose over.
                self.draft = None;
            }
        });
    }

    fn frame_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        let Some(name) = self.selected.clone() else {
            ui.label(egui::RichText::new("Select a frame in the tree.").weak());
            return;
        };
        let Some(frame) = api.frame(&name).cloned() else {
            // A root like `world` has no frame of its own - nothing declares its
            // pose, it is just the reference everything hangs off.
            ui.heading(&name);
            ui.label(
                egui::RichText::new(
                    "This name is only referenced as a parent; it has no frame of its own.",
                )
                .weak(),
            );
            return;
        };

        ui.horizontal(|ui| {
            ui.heading(&name);
            if proto::is_driver_owned(&name) {
                ui.label(egui::RichText::new("driver-owned").color(widgets::WARN))
                    .on_hover_text(
                        "ur_redis_driver publishes this frame and reasserts its parent and \
                         pose on every tick, so a Move or Reparent here will be overwritten \
                         almost immediately.",
                    );
            }
        });
        widgets::field(ui, "parent", &frame.parent_frame_id);
        widgets::field(
            ui,
            "flags",
            &format!(
                "active={} enabled={}",
                frame.active_transform, frame.enable_transform
            ),
        );

        ui.separator();
        ui.label(egui::RichText::new("Pose in parent").strong());

        // The draft starts as what Redis holds and only moves when dragged.
        let mut pose = match &self.draft {
            Some((for_frame, pose)) if *for_frame == name => *pose,
            _ => frame.transform,
        };
        value_editor::transform_pose_editor(ui, egui::Id::new(("tf_pose", &name)), &mut pose);
        let dirty = pose != frame.transform;
        if dirty {
            self.draft = Some((name.clone(), pose));
        }

        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(dirty, egui::Button::new("Move"))
                .on_hover_text("Write the new pose, keeping the same parent.")
                .clicked()
            {
                api.transform_command(proto::TransformCommand::Move {
                    frame: name.clone(),
                    transform: pose,
                });
                self.draft = None;
            }
            if ui.add_enabled(dirty, egui::Button::new("Discard")).clicked() {
                self.draft = None;
            }
        });

        ui.separator();
        ui.label(egui::RichText::new("Reparent").strong());
        ui.horizontal_wrapped(|ui| {
            let names = api.frame_names();
            egui::ComboBox::from_id_salt("reparent_target")
                .selected_text(if self.reparent_to.is_empty() {
                    "pick a new parent".to_string()
                } else {
                    self.reparent_to.clone()
                })
                .show_ui(ui, |ui| {
                    for option in &names {
                        if *option != name {
                            ui.selectable_value(&mut self.reparent_to, option.clone(), option);
                        }
                    }
                });

            let can = !self.reparent_to.is_empty();
            if ui
                .add_enabled(can, egui::Button::new("Reparent"))
                .on_hover_text("The frame keeps its place in the world; its pose is recomputed.")
                .clicked()
            {
                api.transform_command(proto::TransformCommand::Reparent {
                    parent: self.reparent_to.clone(),
                    child: name.clone(),
                });
                self.draft = None;
            }
            if ui
                .add_enabled(can, egui::Button::new("Snap to parent"))
                .on_hover_text("Reparent and move the frame onto the parent's origin.")
                .clicked()
            {
                api.transform_command(proto::TransformCommand::SnapToParent {
                    parent: self.reparent_to.clone(),
                    child: name.clone(),
                });
                self.draft = None;
            }
            if ui
                .button(egui::RichText::new("Remove frame").color(widgets::BAD))
                .on_hover_text("Children of this frame will be left without a parent.")
                .clicked()
            {
                api.transform_command(proto::TransformCommand::Remove { frame: name.clone() });
                self.selected = None;
                self.draft = None;
            }
        });

        egui::CollapsingHeader::new("Everything stored for this frame").show(ui, |ui| {
            value_detail(
                ui,
                &proto::GuiValue::Transform(proto::TransformOrUnknown::Transform(frame)),
            );
        });
    }

    fn lookup_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.label(egui::RichText::new("Lookup").strong());
        ui.label(
            egui::RichText::new(
                "Resolve the transform between any two frames, walking the tree.",
            )
            .weak()
            .small(),
        );

        let names = api.frame_names();
        ui.horizontal_wrapped(|ui| {
            frame_picker(ui, "lookup_parent", "from", &mut self.lookup_parent, &names);
            frame_picker(ui, "lookup_child", "to", &mut self.lookup_child, &names);
            let can = !self.lookup_parent.is_empty() && !self.lookup_child.is_empty();
            if ui.add_enabled(can, egui::Button::new("Look up")).clicked() {
                api.lookup(self.lookup_parent.clone(), self.lookup_child.clone());
            }
            if ui.button("Use selected as 'to'").clicked()
                && let Some(name) = &self.selected
            {
                self.lookup_child = name.clone();
            }
        });

        match &self.lookup_result {
            Some(proto::LookupResponse { error: Some(e), .. }) => {
                ui.label(egui::RichText::new(e).color(widgets::BAD));
            }
            Some(proto::LookupResponse { result: Some(tf), .. }) => {
                let tf = tf.clone();
                ui.group(|ui| {
                    value_detail(
                        ui,
                        &proto::GuiValue::Transform(proto::TransformOrUnknown::Transform(tf)),
                    );
                });
            }
            _ => {}
        }
    }

    fn export_panel(&mut self, ui: &mut egui::Ui, api: &mut Api) {
        ui.label(egui::RichText::new("Export a frame as a scenario file").strong());
        ui.label(
            egui::RichText::new(
                "Writes the frame plus the robot's current joint configuration and gantry \
                 position as metadata, in the layout load_transforms_from_path expects.",
            )
            .weak()
            .small(),
        );

        // The frame to export is whichever the lookup produced, else the
        // selected one - the same choice the old native tab made.
        let frame = self
            .lookup_result
            .as_ref()
            .and_then(|r| r.result.clone())
            .or_else(|| self.selected.as_ref().and_then(|n| api.frame(n).cloned()));

        let Some(frame) = frame else {
            ui.label(
                egui::RichText::new("Select a frame or run a lookup first.").weak(),
            );
            return;
        };

        // These live in the state feed, so no extra request is needed.
        let joints = api
            .robot_id
            .as_ref()
            .map(|r| api.floats(&format!("{r}_joint_states")))
            .unwrap_or_default();
        let gantry = api.float("opc_current_position");

        if self.export_filename.is_empty() {
            self.export_filename =
                format!("{}_to_{}", frame.parent_frame_id, frame.child_frame_id);
        }
        if self.export_tcp_id.is_empty() {
            self.export_tcp_id = frame.child_frame_id.clone();
        }

        ui.horizontal_wrapped(|ui| {
            ui.label("filename");
            ui.add(
                egui::TextEdit::singleline(&mut self.export_filename).desired_width(200.0),
            );
            ui.label("tcp_id");
            ui.add(egui::TextEdit::singleline(&mut self.export_tcp_id).desired_width(140.0));
        });
        widgets::vector_label(ui, "joint_states", &joints);
        widgets::field(ui, "gantry", &format!("{gantry:.4}"));
        if joints.len() != 6 {
            ui.label(
                egui::RichText::new(format!(
                    "⚠ {} joint value(s) available; the metadata wants six.",
                    joints.len()
                ))
                .color(widgets::WARN),
            );
        }

        ui.horizontal(|ui| {
            if ui.button("Export").clicked() {
                api.export_frame(proto::ExportFrameRequest {
                    frame: frame.clone(),
                    filename: self.export_filename.clone(),
                    tcp_id: self.export_tcp_id.clone(),
                    joints: joints.clone(),
                    gantry,
                });
            }
            if let Some(dir) = &api.info.frames_dir {
                ui.label(egui::RichText::new(format!("→ {dir}")).weak().monospace());
            } else {
                ui.label(
                    egui::RichText::new("no --frames-dir set; the JSON is shown but not saved")
                        .color(widgets::WARN),
                );
            }
        });

        if let Some(result) = self.export_result.clone() {
            if let Some(path) = &result.path {
                ui.label(egui::RichText::new(format!("Saved to {path}")).color(widgets::OK));
            }
            if let Some(e) = &result.error {
                ui.label(egui::RichText::new(e).color(widgets::WARN));
            }
            egui::CollapsingHeader::new("Frame JSON").default_open(result.path.is_none()).show(
                ui,
                |ui| {
                    let mut json = result.json.clone();
                    ui.add(
                        egui::TextEdit::multiline(&mut json)
                            .code_editor()
                            .desired_rows(12)
                            .desired_width(f32::INFINITY)
                            .interactive(false),
                    );
                    if ui.button("Copy").clicked() {
                        ui.ctx().copy_text(result.json.clone());
                    }
                },
            );
        }
    }
}

fn frame_picker(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    current: &mut String,
    names: &[String],
) {
    ui.label(label);
    egui::ComboBox::from_id_salt(id)
        .selected_text(if current.is_empty() { "—".to_string() } else { current.clone() })
        .width(160.0)
        .show_ui(ui, |ui| {
            for name in names {
                ui.selectable_value(current, name.clone(), name);
            }
        });
}
