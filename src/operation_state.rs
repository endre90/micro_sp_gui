use eframe::egui;
use micro_sp::{ConnectionManager, OperationLog, SPValue, StateManager, StringOrUnknown, format_log_rows};
use std::sync::{mpsc, Arc};
use std::time::Duration;
use regex::Regex;

// Ensure these are imported from your project modules
// use crate::your_module::{OperationLog, StringOrUnknown, format_log_rows}; 

// struct LogMonitor {
//     label: String,
//     key: String,
//     terminal_output: String,
//     receiver: Option<mpsc::Receiver<String>>,
// }

// impl LogMonitor {
//     pub fn new(label: &str, key: String) -> Self {
//         Self {
//             label: label.to_string(),
//             key,
//             terminal_output: String::new(),
//             receiver: None,
//         }
//     }

//     pub fn ui(
//         &mut self,
//         ui: &mut egui::Ui,
//         handle: &tokio::runtime::Handle,
//         connection: &Arc<ConnectionManager>,
//     ) {
//         // 1. Initialize Background Task
//         if self.receiver.is_none() {
//             let (tx, rx) = mpsc::channel();
//             self.receiver = Some(rx);
//             let con_clone = connection.clone();
//             let key_clone = self.key.clone();

//             handle.spawn(async move {
//                 let mut last_known: Option<String> = None;

//                 loop {
//                     // Use the formatting logic here
//                     let current_val = fetch_and_format(&con_clone, &key_clone).await;

//                     if current_val != last_known {
//                         let text = current_val.clone().unwrap_or_else(|| "No Data".to_string());
//                         let _ = tx.send(text);
//                         last_known = current_val;
//                     }

//                     tokio::time::sleep(Duration::from_millis(500)).await;
//                 }
//             });
//         }

//         // 2. Poll for updates
//         if let Some(rx) = &self.receiver {
//             while let Ok(new_text) = rx.try_recv() {
//                 self.terminal_output = new_text;
//             }
//         }

//         // 3. Render UI
//         ui.push_id(&self.key, |ui| {
//             ui.vertical(|ui| {
//                 ui.label(
//                     egui::RichText::new(&self.label)
//                         .strong()
//                         .heading()
//                         .color(egui::Color32::WHITE),
//                 );

//                 egui::Frame::default()
//                     .fill(egui::Color32::from_rgb(10, 10, 10))
//                     .inner_margin(10.0)
//                     .stroke(egui::Stroke::new(1.0, egui::Color32::DARK_GRAY))
//                     .show(ui, |ui| {
//                         ui.set_min_width(ui.available_width());
                        
//                         egui::ScrollArea::vertical()
//                             .stick_to_bottom(true)
//                             .show(ui, |ui| {
//                                 ui.add(
//                                     egui::Label::new(
//                                         egui::RichText::new(&self.terminal_output)
//                                             .monospace()
//                                             .size(12.0)
//                                             .color(egui::Color32::LIGHT_GREEN),
//                                     )
//                                     .wrap(),
//                                 );
//                             });
//                     });
//             });
//         });
//     }
// }


struct LogMonitor {
    label: String,
    key: String,
    // Store the pre-calculated layout job here
    display_job: egui::text::LayoutJob, 
    receiver: Option<mpsc::Receiver<String>>,
}

impl LogMonitor {
    pub fn new(label: &str, key: String) -> Self {
        Self {
            label: label.to_string(),
            key,
            display_job: egui::text::LayoutJob::default(),
            receiver: None,
        }
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        // --- 1. Background Task (Same as before) ---
        if self.receiver.is_none() {
            let (tx, rx) = mpsc::channel();
            self.receiver = Some(rx);
            let con_clone = connection.clone();
            let key_clone = self.key.clone();

            handle.spawn(async move {
                let mut last_known: Option<String> = None;
                loop {
                    // NOTE: use the fetch WITHOUT stripping regex here.
                    // We want the raw string with colors.
                    let current_val = fetch_raw_string(&con_clone, &key_clone).await;

                    if current_val != last_known {
                        let text = current_val.clone().unwrap_or_default();
                        let _ = tx.send(text);
                        last_known = current_val;
                    }
                    tokio::time::sleep(Duration::from_millis(500)).await;
                }
            });
        }

        // --- 2. Update Logic ---
        if let Some(rx) = &self.receiver {
            // If we receive new text, parse it immediately into a LayoutJob
            while let Ok(new_text) = rx.try_recv() {
                self.display_job = ansi_to_layout_job(&new_text);
            }
        }

        // --- 3. Render Logic ---
        ui.push_id(&self.key, |ui| {
            ui.vertical(|ui| {
                ui.label(egui::RichText::new(&self.label).strong().heading().color(egui::Color32::WHITE));

                egui::Frame::default()
                    .fill(egui::Color32::from_rgb(10, 10, 10))
                    .inner_margin(10.0)
                    .stroke(egui::Stroke::new(1.0, egui::Color32::DARK_GRAY))
                    .show(ui, |ui| {
                        ui.set_min_width(ui.available_width());
                        
                        egui::ScrollArea::vertical()
                            .stick_to_bottom(true)
                            .show(ui, |ui| {
                                // RENDER THE LAYOUT JOB
                                ui.label(self.display_job.clone());
                            });
                    });
            });
        });
    }
}

pub struct OperationStateTab {
    planned_ops: LogMonitor,
    auto_ops: LogMonitor,
    sop_ops: LogMonitor,
}

impl OperationStateTab {
    pub fn new(sp_id: &str) -> Self {
        Self {
            planned_ops: LogMonitor::new(
                "Planned Operations",
                format!("{}_logger_planned_operations", sp_id),
            ),
            auto_ops: LogMonitor::new(
                "Automatic Operations",
                format!("{}_logger_automatic_operations", sp_id),
            ),
            sop_ops: LogMonitor::new(
                "SOP Operations",
                format!("{}_logger_sop_operations", sp_id),
            ),
        }
    }

    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        handle: &tokio::runtime::Handle,
        connection: &Arc<ConnectionManager>,
    ) {
        egui::CentralPanel::default().show_inside(ui, |ui| {
            let available_height = ui.available_height();
            let height_per_window = (available_height / 3.0) - 10.0; 

            ui.allocate_ui_with_layout(
                egui::vec2(ui.available_width(), height_per_window),
                egui::Layout::top_down(egui::Align::Min),
                |ui| self.planned_ops.ui(ui, handle, connection),
            );
            ui.add_space(5.0);

            ui.allocate_ui_with_layout(
                egui::vec2(ui.available_width(), height_per_window),
                egui::Layout::top_down(egui::Align::Min),
                |ui| self.auto_ops.ui(ui, handle, connection),
            );
            ui.add_space(5.0);

            ui.allocate_ui_with_layout(
                egui::vec2(ui.available_width(), height_per_window),
                egui::Layout::top_down(egui::Align::Min),
                |ui| self.sop_ops.ui(ui, handle, connection),
            );
        });
    }
}

fn ansi_to_layout_job(text: &str) -> egui::text::LayoutJob {
    let mut job = egui::text::LayoutJob::default();
    
    // Default style
    let mut color = egui::Color32::LIGHT_GRAY;
    let mut strong = false;
    let mut dimmed = false;

    // Regex to find ANSI codes: \x1b[...m
    let re = Regex::new(r"\x1b\[([0-9;]*)m").unwrap();

    let mut last_end = 0;

    for cap in re.captures_iter(text) {
        let match_start = cap.get(0).unwrap().start();
        let match_end = cap.get(0).unwrap().end();

        // 1. Add the text BEFORE the code with the CURRENT style
        if match_start > last_end {
            let chunk = &text[last_end..match_start];
            
            // Apply current style state
            let mut text_format = egui::TextFormat {
                color,
                font_id: egui::FontId::monospace(13.0),
                ..Default::default()
            };
            
            // Adjust for bold/dimmed
            if strong {
                text_format.font_id = egui::FontId::monospace(13.0); 
                // Note: Monospace usually doesn't have a bold variant in default egui, 
                // but we can fake it with color intensity or a different font family if needed.
                // For now, we will stick to the requested color.
            }
            if dimmed {
                text_format.color = egui::Color32::GRAY;
            }

            job.append(chunk, 0.0, text_format);
        }

        // 2. Parse the ANSI code to update the style for the NEXT chunk
        let codes = cap.get(1).map_or("", |m| m.as_str());
        
        // Split by semicolon (e.g., "1;34" -> ["1", "34"])
        if codes.is_empty() {
             // Empty usually means reset, but "0" is explicit reset
             color = egui::Color32::LIGHT_GRAY;
             strong = false;
             dimmed = false;
        } else {
            for code in codes.split(';') {
                match code {
                    "0" => { // Reset
                        color = egui::Color32::LIGHT_GRAY;
                        strong = false;
                        dimmed = false;
                    }
                    "1" => strong = true,
                    "2" => dimmed = true,
                    "31" => color = egui::Color32::from_rgb(255, 100, 100), // Red
                    "32" => color = egui::Color32::GREEN,
                    "33" => color = egui::Color32::YELLOW,
                    "34" => color = egui::Color32::from_rgb(100, 150, 255), // Blue
                    "37" => color = egui::Color32::LIGHT_GRAY,
                    _ => {} // Ignore unknown codes
                }
            }
        }

        last_end = match_end;
    }

    // 3. Add any remaining text after the last code
    if last_end < text.len() {
        let chunk = &text[last_end..];
        let mut text_format = egui::TextFormat {
            color,
            font_id: egui::FontId::monospace(13.0),
            ..Default::default()
        };
        if dimmed { text_format.color = egui::Color32::GRAY; }
        job.append(chunk, 0.0, text_format);
    }

    job
}

// async fn fetch_and_format(con: &Arc<ConnectionManager>, key: &str) -> Option<String> {
//     let mut connection = con.get_connection().await;
    
//     let logger_sp_value = StateManager::get_sp_value(&mut connection, key).await?;

//     if let SPValue::String(StringOrUnknown::String(logger_string)) = logger_sp_value {
//         if let Ok(logger) = serde_json::from_str::<Vec<Vec<OperationLog>>>(&logger_string) {
            
//             // 1. Generate the table (includes the "broken" squares/codes)
//             let raw_formatted = format_log_rows(&logger);

//             // 2. Regex to find ANSI escape sequences
//             // Matches "\x1b" followed by "[" followed by codes and ending in "m"
//             let re = Regex::new(r"\x1b\[[0-9;]*m").unwrap();

//             // 3. Replace them with empty strings
//             let clean_text = re.replace_all(&raw_formatted, "").to_string();

//             return Some(clean_text);
//         }
//     }

//     None
// }

async fn fetch_raw_string(con: &Arc<ConnectionManager>, key: &str) -> Option<String> {
    let mut connection = con.get_connection().await;
    let logger_sp_value = StateManager::get_sp_value(&mut connection, key).await?;

    if let SPValue::String(StringOrUnknown::String(logger_string)) = logger_sp_value {
        if let Ok(logger) = serde_json::from_str::<Vec<Vec<OperationLog>>>(&logger_string) {
            
            // Ensure colors are ENABLED for the string generation
            colored::control::set_override(true); 
            let formatted = format_log_rows(&logger);
            colored::control::unset_override();

            return Some(formatted);
        }
    }
    None
}