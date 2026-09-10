//! The window: a header with the connection state and the configured system
//! ids, and the tab bar.

use crate::api::{Api, Connection};
use crate::tabs;
use crate::widgets::fixed_slot;

/// Width reserved for the connection indicator in the header.
///
/// The three `Connection` states draw wildly different lengths, and the header
/// is one wrapping row, so letting them size themselves shifts the id slots and
/// the whole tab bar every time the socket flaps. Wide enough for the longest
/// of them, `offline: connection closed`.
const CONNECTION_SLOT_WIDTH: f32 = 240.0;

/// Width of each of the two id readouts, for the same reason: the first
/// `Hello` replacing `no sp_id` with `sp_id: sp1` must not move the tabs.
const ID_SLOT_WIDTH: f32 = 150.0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Tab {
    State,
    Robot,
    Transforms,
    Logs,
    Goals,
    Orders,
    Production,
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
    production_tab: tabs::production::ProductionTab,
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
            production_tab: Default::default(),
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
            Tab::Production => self.production_tab.ui(ui, &mut self.api),
        });
    }
}

impl App {
    fn header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.heading("micro_sp");

            ui.separator();
            self.connection_slot(ui);

            ui.separator();
            self.id_slot(ui, "sp_id", self.api.sp_id.clone());
            self.id_slot(ui, "robot", self.api.robot_id.clone());

            ui.separator();
            ui.selectable_value(&mut self.tab, Tab::State, "State");
            ui.selectable_value(&mut self.tab, Tab::Robot, "Robot");
            ui.selectable_value(&mut self.tab, Tab::Transforms, "Transforms");
            ui.selectable_value(&mut self.tab, Tab::Logs, "Logs");
            ui.selectable_value(&mut self.tab, Tab::Goals, "Goals");
            ui.selectable_value(&mut self.tab, Tab::Orders, "Orders");
            ui.selectable_value(&mut self.tab, Tab::Production, "Production");
        });
    }

    /// The connection indicator, drawn into a fixed-width slot so the rest of
    /// the header does not move when the state changes.
    fn connection_slot(&self, ui: &mut egui::Ui) {
        fixed_slot(ui, CONNECTION_SLOT_WIDTH, |ui| match &self.api.conn {
            Connection::Live => {
                ui.label(egui::RichText::new("⏺").color(crate::widgets::OK));
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(&self.api.info.redis_addr).monospace().weak(),
                    )
                    .truncate(),
                );
            }
            Connection::Connecting => {
                ui.spinner();
                ui.label("connecting…");
            }
            Connection::Down(reason) => {
                ui.label(egui::RichText::new("⏺").color(crate::widgets::BAD));
                // Truncated rather than wrapped: a long Redis error would
                // otherwise break the header onto a second line, which is the
                // same jump this slot exists to prevent.
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(format!("offline: {reason}"))
                            .color(crate::widgets::BAD),
                    )
                    .truncate(),
                )
                .on_hover_text(format!("{reason}\n\nRetrying automatically."));
            }
        });
    }

    /// Which system the whole GUI is addressing.
    ///
    /// Not a picker: the server is configured with one `SP_ID` and one
    /// `ROBOT_ID` and every key is built from those, so there is nothing here
    /// for the operator to choose - and nothing to guess wrong. Drawn into a
    /// fixed-width slot so the tab bar does not shift when the first `Hello`
    /// fills the ids in.
    fn id_slot(&self, ui: &mut egui::Ui, which: &str, id: Option<String>) {
        fixed_slot(ui, ID_SLOT_WIDTH, |ui| match id {
            Some(id) => {
                ui.label(format!("{which}:"));
                ui.add(
                    egui::Label::new(egui::RichText::new(id).monospace().strong())
                        .truncate(),
                );
            }
            None => {
                ui.label(egui::RichText::new(format!("no {which}")).weak()).on_hover_text(
                    format!(
                        "The server was started without {}. Set it and restart the \
                         server; the tabs that need it cannot write until then.",
                        if which == "sp_id" { "SP_ID" } else { "ROBOT_ID" }
                    ),
                );
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
