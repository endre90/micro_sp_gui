//! The window: a header with the connection state and system pickers, and the
//! tab bar.

use crate::api::{Api, Connection};
use crate::tabs;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Tab {
    State,
    Robot,
    Transforms,
    Logs,
    Goals,
    Orders,
}

pub struct App {
    api: Api,
    tab: Tab,
    state_tab: tabs::state::StateTab,
    robot_tab: tabs::robot::RobotTab,
    transforms_tab: tabs::transforms::TransformsTab,
    logs_tab: tabs::logs::LogsTab,
    goals_tab: tabs::goals::GoalsTab,
    orders_tab: tabs::orders::OrdersTab,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            api: Api::new(cc.egui_ctx.clone()),
            tab: Tab::State,
            state_tab: Default::default(),
            robot_tab: Default::default(),
            transforms_tab: Default::default(),
            logs_tab: Default::default(),
            goals_tab: Default::default(),
            orders_tab: Default::default(),
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = ctx.input(|i| i.time);
        self.api.poll(now);

        egui::TopBottomPanel::top("header").show(ctx, |ui| self.header(ui));
        egui::TopBottomPanel::bottom("status").show(ctx, |ui| self.status_bar(ui));

        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            Tab::State => self.state_tab.ui(ui, &mut self.api),
            Tab::Robot => self.robot_tab.ui(ui, &mut self.api),
            Tab::Transforms => self.transforms_tab.ui(ui, &mut self.api),
            Tab::Logs => self.logs_tab.ui(ui, &mut self.api),
            Tab::Goals => self.goals_tab.ui(ui, &mut self.api),
            Tab::Orders => self.orders_tab.ui(ui, &mut self.api),
        });
    }
}

impl App {
    fn header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.heading("micro_sp");

            ui.separator();
            match &self.api.conn {
                Connection::Live => {
                    ui.label(egui::RichText::new("⏺").color(crate::widgets::OK));
                    ui.label(
                        egui::RichText::new(&self.api.info.redis_addr).monospace().weak(),
                    );
                }
                Connection::Connecting => {
                    ui.spinner();
                    ui.label("connecting…");
                }
                Connection::Down(reason) => {
                    ui.label(egui::RichText::new("⏺").color(crate::widgets::BAD));
                    ui.label(
                        egui::RichText::new(format!("offline: {reason}"))
                            .color(crate::widgets::BAD),
                    )
                    .on_hover_text("Retrying automatically.");
                }
            }

            ui.separator();
            self.picker(ui, "sp_id");
            self.picker(ui, "robot");

            ui.separator();
            ui.selectable_value(&mut self.tab, Tab::State, "State");
            ui.selectable_value(&mut self.tab, Tab::Robot, "Robot");
            ui.selectable_value(&mut self.tab, Tab::Transforms, "Transforms");
            ui.selectable_value(&mut self.tab, Tab::Logs, "Logs");
            ui.selectable_value(&mut self.tab, Tab::Goals, "Goals");
            ui.selectable_value(&mut self.tab, Tab::Orders, "Orders");
        });
    }

    /// The sp_id and robot pickers, filled from what the server discovered.
    fn picker(&mut self, ui: &mut egui::Ui, which: &str) {
        let (options, current) = match which {
            "sp_id" => (self.api.info.sp_ids.clone(), &mut self.api.sp_id),
            _ => (self.api.info.robot_ids.clone(), &mut self.api.robot_id),
        };

        if options.is_empty() {
            ui.label(egui::RichText::new(format!("no {which}")).weak()).on_hover_text(
                "Nothing in Redis looks like one yet. Start the runner or the driver, \
                 or pass --sp-id / --robot-id to the server.",
            );
            return;
        }

        let selected = current.clone().unwrap_or_else(|| "-".to_string());
        egui::ComboBox::from_id_salt(which)
            .selected_text(format!("{which}: {selected}"))
            .show_ui(ui, |ui| {
                for option in &options {
                    ui.selectable_value(current, Some(option.clone()), option);
                }
            });
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| match &self.api.status {
            Some((text, true)) => {
                ui.label(egui::RichText::new("⚠").color(crate::widgets::BAD));
                ui.label(egui::RichText::new(text).color(crate::widgets::BAD));
            }
            Some((text, false)) => {
                ui.label(egui::RichText::new(text).weak());
            }
            None => {
                ui.label(
                    egui::RichText::new(format!(
                        "{} keys · {} frames · {} log lines",
                        self.api.entries.len(),
                        self.api.transforms.frames.len(),
                        self.api.logs.len()
                    ))
                    .weak(),
                );
            }
        });
    }
}
