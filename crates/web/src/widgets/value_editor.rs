//! A recursive editor for any `SPValue`.
//!
//! Covers all eight types including nested arrays, maps and transforms, and lets
//! any value be switched between known and `UNKNOWN` - which matters because
//! `UNKNOWN` is a first-class value in micro_sp, not a missing one, and every
//! variable starts there.
//!
//! Returns `true` when the operator changed something.

use micro_sp_gui_protocol as proto;
use proto::{
    ArrayOrUnknown, BoolOrUnknown, FloatOrUnknown, GuiRotation, GuiSystemTime, GuiTransform,
    GuiTransformStamped, GuiTranslation, GuiValue, GuiValueType, IntOrUnknown, MapOrUnknown,
    StringOrUnknown, TimeOrUnknown, TransformOrUnknown,
};

/// Deeper than this and the value is almost certainly a mistake; refusing to
/// recurse keeps a cyclic-looking structure from taking the tab down.
const MAX_DEPTH: usize = 16;

pub fn value_editor(ui: &mut egui::Ui, id: egui::Id, value: &mut GuiValue) -> bool {
    edit(ui, id, value, 0)
}

/// The type picker plus the UNKNOWN toggle, which every value has.
fn header(ui: &mut egui::Ui, id: egui::Id, value: &mut GuiValue) -> bool {
    let mut changed = false;
    let mut chosen = value.type_of();

    egui::ComboBox::from_id_salt(id.with("type"))
        .selected_text(chosen.label())
        .width(96.0)
        .show_ui(ui, |ui| {
            for t in GuiValueType::ALL {
                ui.selectable_value(&mut chosen, *t, t.label());
            }
        });
    if chosen != value.type_of() {
        // Changing the type cannot preserve the value in general, so start from
        // that type's default rather than guessing a conversion.
        *value = GuiValue::default_for(chosen);
        changed = true;
    }

    let mut unknown = value.is_unknown();
    if ui
        .checkbox(&mut unknown, "UNKNOWN")
        .on_hover_text(
            "UNKNOWN is a real, typed value in micro_sp - what a variable reads \
             before anything has measured it.",
        )
        .changed()
    {
        *value = if unknown {
            GuiValue::unknown_of(chosen)
        } else {
            GuiValue::default_for(chosen)
        };
        changed = true;
    }
    changed
}

fn edit(ui: &mut egui::Ui, id: egui::Id, value: &mut GuiValue, depth: usize) -> bool {
    if depth > MAX_DEPTH {
        ui.label(egui::RichText::new("… too deeply nested to edit").weak());
        return false;
    }

    let mut changed = false;
    ui.horizontal_wrapped(|ui| {
        changed |= header(ui, id, value);

        // An UNKNOWN has nothing more to edit.
        if value.is_unknown() {
            return;
        }

        match value {
            GuiValue::Bool(BoolOrUnknown::Bool(b)) => {
                changed |= ui.checkbox(b, if *b { "true" } else { "false" }).changed();
            }
            GuiValue::Int64(IntOrUnknown::Int64(i)) => {
                changed |= ui.add(egui::DragValue::new(i).speed(1.0)).changed();
            }
            GuiValue::Float64(FloatOrUnknown::Float64(f)) => {
                changed |= ui.add(egui::DragValue::new(f).speed(0.01)).changed();
            }
            GuiValue::String(StringOrUnknown::String(s)) => {
                changed |= ui
                    .add(egui::TextEdit::singleline(s).desired_width(320.0))
                    .changed();
                // `ToSPValue for String` folds this spelling into the unknown
                // variant, so typing it does not do what it looks like.
                if s.trim().eq_ignore_ascii_case("unknown") {
                    ui.label(
                        egui::RichText::new("⚠ micro_sp reads this as the UNKNOWN variant")
                            .color(super::WARN),
                    );
                }
            }
            GuiValue::Time(TimeOrUnknown::Time(t)) => {
                changed |= time_editor(ui, t);
            }
            _ => {}
        }
    });

    // Composites get their own block below the header row.
    match value {
        GuiValue::Array(ArrayOrUnknown::Array(items)) => {
            ui.indent(id.with("array"), |ui| {
                changed |= array_editor(ui, id, items, depth);
            });
        }
        GuiValue::Map(MapOrUnknown::Map(pairs)) => {
            ui.indent(id.with("map"), |ui| {
                changed |= map_editor(ui, id, pairs, depth);
            });
        }
        GuiValue::Transform(TransformOrUnknown::Transform(tf)) => {
            ui.indent(id.with("tf"), |ui| {
                changed |= transform_editor(ui, id, tf, depth);
            });
        }
        _ => {}
    }

    changed
}

fn time_editor(ui: &mut egui::Ui, t: &mut GuiSystemTime) -> bool {
    let mut changed = false;
    ui.label("s");
    changed |= ui.add(egui::DragValue::new(&mut t.secs_since_epoch).speed(1.0)).changed();
    ui.label("ns");
    changed |= ui
        .add(egui::DragValue::new(&mut t.nanos_since_epoch).speed(1.0).range(0..=999_999_999))
        .changed();
    changed
}

/// The type to use for the next added element, remembered per widget.
fn pending_type(ui: &mut egui::Ui, id: egui::Id) -> GuiValueType {
    ui.data(|d| d.get_temp::<GuiValueType>(id)).unwrap_or(GuiValueType::String)
}

fn set_pending_type(ui: &mut egui::Ui, id: egui::Id, t: GuiValueType) {
    ui.data_mut(|d| d.insert_temp(id, t));
}

fn add_row(ui: &mut egui::Ui, id: egui::Id, label: &str) -> Option<GuiValue> {
    let mut added = None;
    ui.horizontal(|ui| {
        let mut t = pending_type(ui, id);
        egui::ComboBox::from_id_salt(id.with("new_type"))
            .selected_text(t.label())
            .width(96.0)
            .show_ui(ui, |ui| {
                for option in GuiValueType::ALL {
                    ui.selectable_value(&mut t, *option, option.label());
                }
            });
        set_pending_type(ui, id, t);
        if ui.button(label).clicked() {
            added = Some(GuiValue::default_for(t));
        }
    });
    added
}

fn array_editor(
    ui: &mut egui::Ui,
    id: egui::Id,
    items: &mut Vec<GuiValue>,
    depth: usize,
) -> bool {
    let mut changed = false;
    let mut remove: Option<usize> = None;

    for (i, item) in items.iter_mut().enumerate() {
        let row = id.with(("item", i));
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(format!("[{i}]")).weak().monospace());
            if ui.small_button("✖").on_hover_text("Remove this item").clicked() {
                remove = Some(i);
            }
            ui.vertical(|ui| {
                changed |= edit(ui, row, item, depth + 1);
            });
        });
    }

    // Deferred so the iteration above is not invalidated.
    if let Some(i) = remove {
        items.remove(i);
        changed = true;
    }
    if let Some(new) = add_row(ui, id.with("add"), "➕ Add item") {
        items.push(new);
        changed = true;
    }
    changed
}

fn map_editor(
    ui: &mut egui::Ui,
    id: egui::Id,
    pairs: &mut Vec<(GuiValue, GuiValue)>,
    depth: usize,
) -> bool {
    let mut changed = false;
    let mut remove: Option<usize> = None;

    for (i, (key, val)) in pairs.iter_mut().enumerate() {
        let row = id.with(("pair", i));
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("pair {i}")).weak());
                if ui.small_button("✖").on_hover_text("Remove this pair").clicked() {
                    remove = Some(i);
                }
            });
            ui.label(egui::RichText::new("key").weak());
            changed |= edit(ui, row.with("k"), key, depth + 1);
            ui.label(egui::RichText::new("value").weak());
            changed |= edit(ui, row.with("v"), val, depth + 1);
        });
    }

    if let Some(i) = remove {
        pairs.remove(i);
        changed = true;
    }
    if ui.button("➕ Add pair").clicked() {
        // A string key is what metadata and every practical map uses.
        pairs.push((
            GuiValue::String(StringOrUnknown::String(String::new())),
            GuiValue::String(StringOrUnknown::String(String::new())),
        ));
        changed = true;
    }
    changed
}

fn transform_editor(
    ui: &mut egui::Ui,
    id: egui::Id,
    tf: &mut GuiTransformStamped,
    depth: usize,
) -> bool {
    let mut changed = false;

    ui.horizontal(|ui| {
        ui.label("parent");
        changed |= ui
            .add(egui::TextEdit::singleline(&mut tf.parent_frame_id).desired_width(140.0))
            .changed();
        ui.label("child");
        changed |= ui
            .add(egui::TextEdit::singleline(&mut tf.child_frame_id).desired_width(140.0))
            .changed();
    });
    ui.horizontal(|ui| {
        changed |= ui
            .checkbox(&mut tf.active_transform, "active")
            .on_hover_text("Actively maintained rather than static.")
            .changed();
        changed |= ui
            .checkbox(&mut tf.enable_transform, "enabled")
            .on_hover_text(
                "Disabled frames are dropped when a scenario is loaded from disk - \
                 that is how a frame file is commented out.",
            )
            .changed();
    });

    changed |= transform_pose_editor(ui, id, &mut tf.transform);

    egui::CollapsingHeader::new("metadata")
        .id_salt(id.with("meta"))
        .show(ui, |ui| {
            let mut as_value = GuiValue::Map(tf.metadata.clone());
            if edit(ui, id.with("meta_edit"), &mut as_value, depth + 1)
                && let GuiValue::Map(map) = as_value
            {
                tf.metadata = map;
                changed = true;
            }
        });

    changed
}

/// Translation and quaternion drag fields, shared with the Transforms tab.
pub fn transform_pose_editor(
    ui: &mut egui::Ui,
    id: egui::Id,
    transform: &mut GuiTransform,
) -> bool {
    let mut changed = false;
    ui.push_id(id.with("pose"), |ui| {
        egui::Grid::new("pose_grid").num_columns(5).spacing([8.0, 4.0]).show(ui, |ui| {
            ui.label("translation");
            changed |= drag(ui, "x", &mut transform.translation.x, 0.001);
            changed |= drag(ui, "y", &mut transform.translation.y, 0.001);
            changed |= drag(ui, "z", &mut transform.translation.z, 0.001);
            ui.end_row();

            ui.label("rotation");
            changed |= drag(ui, "x", &mut transform.rotation.x, 0.001);
            changed |= drag(ui, "y", &mut transform.rotation.y, 0.001);
            changed |= drag(ui, "z", &mut transform.rotation.z, 0.001);
            changed |= drag(ui, "w", &mut transform.rotation.w, 0.001);
            ui.end_row();
        });

        // An unnormalised quaternion is not a rotation; micro_sp will happily
        // store it and every lookup downstream will be subtly wrong.
        // Copied out, so the normalise button can write the field back.
        let r = transform.rotation;
        let norm = (r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w).sqrt();
        if (norm - 1.0).abs() > 1e-3 {
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new(format!("⚠ quaternion norm is {norm:.4}, not 1"))
                        .color(super::WARN),
                );
                if ui.small_button("Normalise").clicked() && norm > f64::EPSILON {
                    transform.rotation = GuiRotation {
                        x: r.x / norm,
                        y: r.y / norm,
                        z: r.z / norm,
                        w: r.w / norm,
                    };
                    changed = true;
                }
            });
        }
    });
    changed
}

fn drag(ui: &mut egui::Ui, label: &str, value: &mut f64, speed: f64) -> bool {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(label).weak().monospace());
        ui.add(egui::DragValue::new(value).speed(speed).max_decimals(6)).changed()
    })
    .inner
}

/// A fresh identity frame, for the "insert frame" flow.
pub fn new_frame(parent: &str, child: &str) -> GuiTransformStamped {
    GuiTransformStamped {
        transform: GuiTransform {
            translation: GuiTranslation::default(),
            rotation: GuiRotation::default(),
        },
        ..GuiTransformStamped::identity(parent, child)
    }
}
