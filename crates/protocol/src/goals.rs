//! Goals and item orders.
//!
//! Mirrors `micro_sp::running::goal_runner`. A goal is `[id, priority,
//! predicate]` on the wire; the server builds that with the crate's own
//! `goal_string_to_sp_value` rather than assembling the array here.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Declaration order *is* the ordering, as in `micro_sp`.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GuiGoalPriority {
    Top,
    High,
    #[default]
    Normal,
    Low,
}

impl GuiGoalPriority {
    pub const ALL: &'static [GuiGoalPriority] = &[
        GuiGoalPriority::Top,
        GuiGoalPriority::High,
        GuiGoalPriority::Normal,
        GuiGoalPriority::Low,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            GuiGoalPriority::Top => "Top",
            GuiGoalPriority::High => "High",
            GuiGoalPriority::Normal => "Normal",
            GuiGoalPriority::Low => "Low",
        }
    }

    /// Matches `micro_sp::GoalPriority::to_int`/`from_int`.
    pub fn to_int(&self) -> i64 {
        match self {
            GuiGoalPriority::Top => 0,
            GuiGoalPriority::High => 1,
            GuiGoalPriority::Normal => 2,
            GuiGoalPriority::Low => 3,
        }
    }

    pub fn from_int(i: i64) -> Self {
        match i {
            0 => GuiGoalPriority::Top,
            1 => GuiGoalPriority::High,
            3 => GuiGoalPriority::Low,
            _ => GuiGoalPriority::Normal,
        }
    }
}

impl fmt::Display for GuiGoalPriority {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct GuiGoal {
    pub id: String,
    pub priority: GuiGoalPriority,
    pub predicate: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct SendGoalsRequest {
    pub sp_id: String,
    pub goals: Vec<GuiGoal>,
    /// `false` (the default) appends to whatever the goal runner has not drained
    /// yet. `true` clears the incoming queue first - a deliberate "forget the
    /// backlog and do this" action.
    pub replace: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct SendGoalsResponse {
    pub accepted: usize,
    /// How many goals were already queued and were preserved by the append.
    pub queued_before: usize,
    pub error: Option<String>,
}

/// Read-only view of the goal and plan pipeline, assembled from `{sp_id}_*`
/// keys that are already in the state snapshot.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct GoalsStatus {
    pub sp_id: String,
    pub incoming: Vec<GuiGoal>,
    pub scheduled: Vec<GuiGoal>,
    /// Goals that were in the arrays but did not decode, with the reason.
    pub undecodable: Vec<String>,
    pub current_goal_id: String,
    pub current_goal_predicate: String,
    /// `initial` | `executing` | `failed` | `cancelled` | `completed` | `UNKNOWN`
    pub current_goal_state: String,
    /// `found` | `not_found` | `ready` | `UNKNOWN`
    pub planner_state: String,
    /// `initial` | `executing` | `failed` | `completed` | `cancelled` | `UNKNOWN`
    pub plan_state: String,
    pub plan: Vec<String>,
    pub plan_current_step: i64,
    pub replan_counter: i64,
    pub goal_runner_information: String,
    pub goal_scheduler_information: String,
    pub planner_information: String,
    pub plan_runner_information: String,
}

/// The items the Order tab can order, from the old native Order Handler.
/// Each non-zero count becomes the goal `var:count_picked_{item} == {count}`.
pub const ORDER_ITEMS: &[&str] = &[
    "silver_box",
    "silver_gun",
    "silver_plate",
    "black_hose",
    "black_plate",
    "black_plug",
    "motor_valve",
];

/// The predicate one order row turns into.
pub fn order_predicate(item: &str, count: u32) -> String {
    format!("var:count_picked_{item} == {count}")
}
