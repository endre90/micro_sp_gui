//! ANSI colour rendering.
//!
//! micro_sp's console output is coloured with SGR escapes, and a value copied
//! from a log can carry them. Rendering them as colour rather than as `\x1b[32m`
//! keeps the Logs tab readable. Ported from the old native operation-state tab.

use egui::text::{LayoutJob, TextFormat};

/// Split `text` into coloured runs. Unknown codes reset to the default.
pub fn ansi_to_layout_job(text: &str, size: f32, default: egui::Color32) -> LayoutJob {
    let mut job = LayoutJob::default();
    let font = egui::FontId::monospace(size);

    let mut colour = default;
    let mut rest = text;

    while let Some(start) = rest.find('\u{1b}') {
        if start > 0 {
            job.append(&rest[..start], 0.0, TextFormat {
                font_id: font.clone(),
                color: colour,
                ..Default::default()
            });
        }
        // An escape we cannot parse is emitted as-is rather than swallowed.
        let after = &rest[start..];
        let Some(end) = after.find('m') else {
            job.append(after, 0.0, TextFormat {
                font_id: font.clone(),
                color: colour,
                ..Default::default()
            });
            return job;
        };
        let codes = &after[2..end];
        colour = sgr_colour(codes, default, colour);
        rest = &after[end + 1..];
    }

    if !rest.is_empty() {
        job.append(rest, 0.0, TextFormat {
            font_id: font,
            color: colour,
            ..Default::default()
        });
    }
    job
}

fn sgr_colour(
    codes: &str,
    default: egui::Color32,
    current: egui::Color32,
) -> egui::Color32 {
    let mut colour = current;
    for code in codes.split(';') {
        colour = match code.trim() {
            "" | "0" => default,
            "1" => colour,
            "2" => egui::Color32::GRAY,
            "30" => egui::Color32::DARK_GRAY,
            "31" => super::BAD,
            "32" => super::OK,
            "33" => super::WARN,
            "34" => egui::Color32::from_rgb(0x42, 0xa5, 0xf5),
            "35" => egui::Color32::from_rgb(0xab, 0x47, 0xbc),
            "36" => egui::Color32::from_rgb(0x26, 0xc6, 0xda),
            "37" => egui::Color32::LIGHT_GRAY,
            _ => default,
        };
    }
    colour
}

/// Strip escapes without rendering them, for grep and for copying.
pub fn strip_ansi(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(start) = rest.find('\u{1b}') {
        out.push_str(&rest[..start]);
        let after = &rest[start..];
        match after.find('m') {
            Some(end) => rest = &after[end + 1..],
            None => return out,
        }
    }
    out.push_str(rest);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_text_is_one_run() {
        let job = ansi_to_layout_job("hello", 12.0, egui::Color32::WHITE);
        assert_eq!(job.text, "hello");
    }

    #[test]
    fn escapes_do_not_appear_in_the_rendered_text() {
        let job = ansi_to_layout_job("\u{1b}[32mok\u{1b}[0m done", 12.0, egui::Color32::WHITE);
        assert_eq!(job.text, "ok done");
        assert!(job.sections.len() >= 2, "colour changes should split the job");
    }

    #[test]
    fn strip_removes_every_escape() {
        assert_eq!(strip_ansi("\u{1b}[31mred\u{1b}[0m/plain"), "red/plain");
        // An unterminated escape must not panic or eat the whole string.
        assert_eq!(strip_ansi("before\u{1b}[3"), "before");
    }
}
