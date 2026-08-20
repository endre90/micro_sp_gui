//! Reusable pieces shared by the tabs.

pub mod ansi;
pub mod tree;
pub mod value_detail;
pub mod value_editor;

pub use ansi::ansi_to_layout_job;
pub use value_detail::value_detail;
pub use value_editor::value_editor;

/// The colour used for something that is fine.
pub const OK: egui::Color32 = egui::Color32::from_rgb(0x4c, 0xaf, 0x50);
/// Something that needs attention but is not broken.
pub const WARN: egui::Color32 = egui::Color32::from_rgb(0xff, 0xb3, 0x00);
/// Something is wrong.
pub const BAD: egui::Color32 = egui::Color32::from_rgb(0xe5, 0x39, 0x35);
/// A value nobody has measured yet.
pub const UNKNOWN: egui::Color32 = egui::Color32::from_rgb(0x90, 0x90, 0x90);

/// A small round status light with a tooltip.
///
/// `None` is grey, not red: "nobody has published this" and "the publisher says
/// no" are different facts, and painting the first as the second sent us
/// hunting for a disconnected robot that had simply never been described.
pub fn light(ui: &mut egui::Ui, on: Option<bool>, label: &str, hover: &str) {
    let colour = match on {
        Some(true) => OK,
        Some(false) => BAD,
        None => UNKNOWN,
    };
    let hover = match on {
        Some(value) => format!("{hover}\n\nCurrently {value}."),
        None => format!("{hover}\n\nNot published: no process has written this variable."),
    };
    ui.label(egui::RichText::new("⏺").color(colour)).on_hover_text(&hover);
    ui.label(label).on_hover_text(&hover);
}

/// A labelled read-only value.
pub fn field(ui: &mut egui::Ui, label: &str, value: &str) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("{label}:")).weak());
        let colour = if value == "UNKNOWN" { UNKNOWN } else { ui.visuals().text_color() };
        ui.label(egui::RichText::new(value).color(colour).monospace());
    });
}

/// Six numbers on one line, the shape joint vectors and poses come in.
pub fn vector_label(ui: &mut egui::Ui, label: &str, values: &[f64]) {
    let text = if values.is_empty() {
        "UNKNOWN".to_string()
    } else {
        values.iter().map(|v| format!("{v:.4}")).collect::<Vec<_>>().join(", ")
    };
    field(ui, label, &text);
}
