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

/// Width of the label column in a [`kv`] row.
///
/// Every read-only row in the robot tab uses the same one, so the values line up
/// down a single edge instead of each row finding its own.
pub const KV_LABEL_WIDTH: f32 = 78.0;

/// Lay `add` out inside a slot of exactly `width`, whatever it draws.
///
/// `allocate_ui_with_layout` on its own allocates whatever the contents turn
/// out to need, so the width has to be pinned from the inside with
/// `set_min_width`; the `max_rect` it hands down is what keeps a truncating
/// label from spilling back out.
pub fn fixed_slot(ui: &mut egui::Ui, width: f32, add: impl FnOnce(&mut egui::Ui)) {
    let size = egui::vec2(width, ui.spacing().interact_size.y);
    ui.allocate_ui_with_layout(size, egui::Layout::left_to_right(egui::Align::Center), |ui| {
        ui.set_min_width(width);
        add(ui);
    });
}

/// One full-width row of exactly the standard row height, drawn whether or not
/// `add` puts anything in it.
///
/// This is what a warning that comes and goes belongs in: the line is paid for
/// on every frame, so the widgets underneath it do not move the moment the
/// warning appears - which, for a row of buttons, means moving out from under
/// the pointer that was already aimed at one.
pub fn reserved_line(ui: &mut egui::Ui, add: impl FnOnce(&mut egui::Ui)) {
    let height = ui.spacing().interact_size.y;
    let size = egui::vec2(ui.available_width(), height);
    ui.allocate_ui_with_layout(size, egui::Layout::left_to_right(egui::Align::Center), |ui| {
        ui.set_min_height(height);
        add(ui);
    });
}

/// A label and its value on one row, the label in a fixed-width slot and the
/// value truncated rather than wrapped.
///
/// Truncation is the point: a long driver message wrapping onto a second line
/// would change the height of the block it is in, and everything below it would
/// slide. The full text is on the hover instead.
pub fn kv(ui: &mut egui::Ui, label: &str, value: egui::RichText, hover: &str) {
    ui.horizontal(|ui| {
        fixed_slot(ui, KV_LABEL_WIDTH, |ui| {
            ui.label(egui::RichText::new(label).weak());
        });
        let response = ui.add(egui::Label::new(value).truncate());
        if !hover.is_empty() {
            response.on_hover_text(hover);
        }
    });
}

/// The value cell for a string the driver may not have published yet.
///
/// The driver writes the literal `UNKNOWN` for "not measured", so that and an
/// empty string are the same fact and are drawn the same way - as a dash, which
/// keeps the row the same height as a filled one.
pub fn cell(value: &str) -> egui::RichText {
    if value.is_empty() || value == "UNKNOWN" {
        egui::RichText::new("—").monospace().color(UNKNOWN)
    } else {
        egui::RichText::new(value).monospace()
    }
}
