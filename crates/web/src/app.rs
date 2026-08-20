//! The window: a header with the connection state and system pickers, and the
//! tab bar.

use crate::api::{Api, Connection};
use crate::tabs;

/// Width reserved for the connection indicator in the header.
///
/// The three `Connection` states draw wildly different lengths, and the header
/// is one wrapping row, so letting them size themselves shifts the pickers and
/// the whole tab bar every time the socket flaps. Wide enough for the longest
/// of them, `offline: connection closed`.
const CONNECTION_SLOT_WIDTH: f32 = 240.0;

/// Width of each of the two system pickers, for the same reason: discovery
/// replacing `no sp_id` with `sp_id: sp1` must not move the tabs.
const PICKER_WIDTH: f32 = 150.0;

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
            self.connection_slot(ui);

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

    /// The sp_id and robot pickers, filled from what the server discovered.
    ///
    /// Both states are the same combo box at the same width - an empty one is
    /// disabled rather than replaced by a label - so the row does not reflow the
    /// moment discovery finds a system. A fixed-width slot would not be enough
    /// here: `ComboBox::width` sizes the *inside* of the button, and the arrow
    /// and padding it adds on top would still make the two states differ.
    fn picker(&mut self, ui: &mut egui::Ui, which: &str) {
        let (options, current) = match which {
            "sp_id" => (self.api.info.sp_ids.clone(), &mut self.api.sp_id),
            _ => (self.api.info.robot_ids.clone(), &mut self.api.robot_id),
        };
        let empty = options.is_empty();

        let selected = match (empty, current.clone()) {
            (true, _) => format!("no {which}"),
            (false, Some(id)) => format!("{which}: {id}"),
            (false, None) => format!("{which}: -"),
        };

        let slot = ui.add_enabled_ui(!empty, |ui| {
            egui::ComboBox::from_id_salt(which)
                .selected_text(selected)
                .width(PICKER_WIDTH)
                .show_ui(ui, |ui| {
                    for option in &options {
                        ui.selectable_value(current, Some(option.clone()), option);
                    }
                });
        });

        if empty {
            slot.response.on_hover_text(
                "Nothing in Redis looks like one yet. Start the runner or the driver, \
                 or pass --sp-id / --robot-id to the server.",
            );
        }
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

/// Lay `add` out inside a slot of exactly `width`, whatever it draws.
///
/// `allocate_ui_with_layout` on its own allocates whatever the contents turn
/// out to need, so the width has to be pinned from the inside with
/// `set_min_width`; the `max_rect` it hands down is what keeps a truncating
/// label from spilling back out.
fn fixed_slot(ui: &mut egui::Ui, width: f32, add: impl FnOnce(&mut egui::Ui)) {
    let size = egui::vec2(width, ui.spacing().interact_size.y);
    ui.allocate_ui_with_layout(size, egui::Layout::left_to_right(egui::Align::Center), |ui| {
        ui.set_min_width(width);
        add(ui);
    });
}
