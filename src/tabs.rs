// This file declares the modules within the `tabs` directory
// and now also contains the main App composer.

use eframe::egui;
use micro_sp::{ConnectionManager, SPTransform, SPTransformStamped, TransformsManager};
use poll_promise::Promise;
use serde::Serialize;
use std::sync::Arc;

#[derive(PartialEq, Eq, Debug)]
enum AppTab {
    Transforms,
    RobotTab,
    AnotherTab,
}

pub struct MyApp {
    handle: tokio::runtime::Handle,
    connection: Arc<ConnectionManager>,

    transforms_tab: crate::lookup::TransformsTab,
    robot_tab: crate::robot::RobotTab,
    another_tab: crate::another::AnotherTab,
    active_tab: AppTab,
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();
        egui::CentralPanel::default().show(ctx, |ui| {
            self.ui(ui);
        });
    }
}

impl MyApp {
    pub async fn new(handle: tokio::runtime::Handle) -> Self {
        let connection = Arc::new(ConnectionManager::new().await);
        Self {
            handle,
            connection,
            transforms_tab: crate::lookup::TransformsTab::new(),
            robot_tab: crate::robot::RobotTab::new(),
            another_tab: crate::another::AnotherTab::new(),
            active_tab: AppTab::Transforms,
        }
    }

    // Main UI function now acts as a tab controller
    fn ui(&mut self, ui: &mut egui::Ui) {
        // Draw the horizontal tab bar
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.active_tab, AppTab::Transforms, "Transforms");
            ui.selectable_value(&mut self.active_tab, AppTab::RobotTab, "Robot Controller");
            ui.selectable_value(&mut self.active_tab, AppTab::AnotherTab, "Another Tab");
        });

        ui.separator();

        // Match on the active tab and call the `ui` method for that specific tab,
        // passing in any shared state it needs (like the handle and connection).
        match self.active_tab {
            AppTab::Transforms => {
                self.transforms_tab
                    .ui(ui, &self.handle, &self.connection);
            }
            AppTab::RobotTab => {
                self.robot_tab
                    .ui(ui, &self.handle, &self.connection);
            }
            AppTab::AnotherTab => {
                self.another_tab.ui(ui);
            }
        }
    }
}