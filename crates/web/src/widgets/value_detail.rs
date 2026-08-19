//! Read-only breakdown of an `SPValue`.
//!
//! The nesting rules follow the old Streamlit "Details" tab: arrays show indexed
//! items, maps show key/value pairs, and a transform is split into its frames,
//! translation, rotation and metadata.

use micro_sp_gui_protocol as proto;
use proto::{
    ArrayOrUnknown, BoolOrUnknown, FloatOrUnknown, GuiValue, IntOrUnknown, MapOrUnknown,
    StringOrUnknown, TimeOrUnknown, TransformOrUnknown,
};

/// Nested values are indented rather than boxed, so deep structures stay narrow.
pub fn value_detail(ui: &mut egui::Ui, value: &GuiValue) {
    detail(ui, value, 0);
}

fn scalar(ui: &mut egui::Ui, type_name: &str, text: &str, unknown: bool) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(type_name).weak().monospace());
        let colour =
            if unknown { super::UNKNOWN } else { ui.visuals().strong_text_color() };
        ui.label(egui::RichText::new(text).monospace().color(colour));
    });
}

fn detail(ui: &mut egui::Ui, value: &GuiValue, depth: usize) {
    // Bail out rather than blowing the stack on a pathological value.
    if depth > 24 {
        ui.label(egui::RichText::new("… nesting too deep to display").weak());
        return;
    }

    match value {
        GuiValue::Bool(BoolOrUnknown::Bool(b)) => scalar(ui, "Bool", &b.to_string(), false),
        GuiValue::Int64(IntOrUnknown::Int64(i)) => scalar(ui, "Int64", &i.to_string(), false),
        GuiValue::Float64(FloatOrUnknown::Float64(f)) => {
            scalar(ui, "Float64", &f.to_string(), false)
        }
        GuiValue::String(StringOrUnknown::String(s)) => {
            scalar(ui, "String", &format!("\"{s}\""), false)
        }
        GuiValue::Time(TimeOrUnknown::Time(t)) => scalar(
            ui,
            "Time",
            &format!("{}s + {}ns since the epoch", t.secs_since_epoch, t.nanos_since_epoch),
            false,
        ),

        GuiValue::Array(ArrayOrUnknown::Array(items)) => {
            ui.label(egui::RichText::new(format!("Array ({} items)", items.len())).strong());
            if items.is_empty() {
                ui.label(egui::RichText::new("(empty)").weak());
            }
            for (i, item) in items.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(format!("[{i}]")).weak().monospace());
                    ui.vertical(|ui| detail(ui, item, depth + 1));
                });
            }
        }

        GuiValue::Map(MapOrUnknown::Map(pairs)) => {
            ui.label(egui::RichText::new(format!("Map ({} pairs)", pairs.len())).strong());
            if pairs.is_empty() {
                ui.label(egui::RichText::new("(empty)").weak());
            }
            // Keys are values too, but a string key is the overwhelming case and
            // deserves to read as one line rather than a nested block.
            for (key, val) in pairs {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(key.display()).monospace().weak());
                    ui.label("→");
                    ui.vertical(|ui| detail(ui, val, depth + 1));
                });
            }
        }

        GuiValue::Transform(TransformOrUnknown::Transform(tf)) => {
            ui.label(egui::RichText::new("Transform").strong());
            super::field(ui, "parent", &tf.parent_frame_id);
            super::field(ui, "child", &tf.child_frame_id);
            super::field(ui, "active", &tf.active_transform.to_string());
            super::field(ui, "enabled", &tf.enable_transform.to_string());
            super::field(
                ui,
                "stamp",
                &format!("{}.{:09}", tf.time_stamp.secs_since_epoch, tf.time_stamp.nanos_since_epoch),
            );
            super::field(
                ui,
                "translation",
                &format!(
                    "x={:.6}, y={:.6}, z={:.6}",
                    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
                ),
            );
            super::field(
                ui,
                "rotation",
                &format!(
                    "x={:.6}, y={:.6}, z={:.6}, w={:.6}",
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w
                ),
            );
            ui.label(egui::RichText::new("metadata").weak());
            ui.indent("tf_metadata", |ui| {
                detail(ui, &GuiValue::Map(tf.metadata.clone()), depth + 1)
            });
        }

        // Every unknown keeps its type, which is worth showing: an UNKNOWN
        // Float64 and an UNKNOWN String are different variables.
        other => scalar(ui, other.type_of().label(), "UNKNOWN", true),
    }
}
