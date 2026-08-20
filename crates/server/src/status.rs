//! Deriving the Robot and Goals views from the state snapshot.
//!
//! Everything here reads out of the already-fetched entry map, so it costs no
//! extra Redis traffic. It also never builds a `micro_sp::State`: that type's
//! accessors *panic* on a key that is not in the map, which is the last thing a
//! long-running server should do because a variable has not been created yet.

use micro_sp_gui_protocol as proto;
use proto::{
    ArrayOrUnknown, BoolOrUnknown, FloatOrUnknown, GuiValue, IntOrUnknown, StringOrUnknown,
};
use std::collections::BTreeMap;

pub type Entries = BTreeMap<String, proto::StateEntry>;

fn value<'a>(entries: &'a Entries, key: &str) -> Option<&'a GuiValue> {
    entries.get(key).and_then(|e| e.value.as_ref())
}

/// String, or `"UNKNOWN"` when absent, unknown or the wrong type.
pub fn get_string(entries: &Entries, key: &str) -> String {
    match value(entries, key) {
        Some(GuiValue::String(StringOrUnknown::String(s))) => s.clone(),
        _ => "UNKNOWN".to_string(),
    }
}

pub fn get_bool(entries: &Entries, key: &str) -> bool {
    matches!(value(entries, key), Some(GuiValue::Bool(BoolOrUnknown::Bool(true))))
}

/// A bool that keeps "nobody has written this" apart from "this is false".
///
/// [`get_bool`] folds an absent key, an `UNKNOWN` and a real `false` into one
/// answer, which is right for a flag that gates behaviour and wrong for
/// anything drawn as a status light: an older `ur_redis_driver` never writes
/// `{robot}_robot_connected` at all, and reporting that as "not connected" sends
/// the operator looking for a network fault that does not exist.
pub fn get_tribool(entries: &Entries, key: &str) -> Option<bool> {
    match value(entries, key) {
        Some(GuiValue::Bool(BoolOrUnknown::Bool(b))) => Some(*b),
        _ => None,
    }
}

pub fn get_int(entries: &Entries, key: &str) -> i64 {
    match value(entries, key) {
        Some(GuiValue::Int64(IntOrUnknown::Int64(i))) => *i,
        _ => 0,
    }
}

pub fn get_float(entries: &Entries, key: &str) -> f64 {
    match value(entries, key) {
        Some(GuiValue::Float64(FloatOrUnknown::Float64(f))) => *f,
        // An Int64 where a float is expected is common enough in hand-set state
        // to be worth accepting rather than reporting as zero.
        Some(GuiValue::Int64(IntOrUnknown::Int64(i))) => *i as f64,
        _ => 0.0,
    }
}

pub fn get_array(entries: &Entries, key: &str) -> Vec<GuiValue> {
    match value(entries, key) {
        Some(GuiValue::Array(ArrayOrUnknown::Array(items))) => items.clone(),
        _ => Vec::new(),
    }
}

/// An array read as floats, tolerating integer elements.
pub fn get_floats(entries: &Entries, key: &str) -> Vec<f64> {
    get_array(entries, key)
        .into_iter()
        .filter_map(|v| match v {
            GuiValue::Float64(FloatOrUnknown::Float64(f)) => Some(f),
            GuiValue::Int64(IntOrUnknown::Int64(i)) => Some(i as f64),
            _ => None,
        })
        .collect()
}

pub fn get_strings(entries: &Entries, key: &str) -> Vec<String> {
    get_array(entries, key)
        .into_iter()
        .map(|v| match v {
            GuiValue::String(StringOrUnknown::String(s)) => s,
            // A plan step is a string, but render anything else rather than
            // dropping it - a hole in the plan list would be misleading.
            other => other.display(),
        })
        .collect()
}

/// Every `{robot_id}_*` suffix [`robot_status`] reads.
///
/// The fast robot poller builds its `MGET` from this rather than from the whole
/// keyspace, so the two must not drift apart - the test at the bottom of this
/// module fails if a field is read that is not listed here.
pub const ROBOT_SUFFIXES: &[&str] = &[
    "safety_mode",
    "robot_mode",
    "program_state",
    "program_running",
    "robot_connected",
    "dashboard_connected",
    "remote_control",
    "tcp_pose",
    "tcp_force",
    "joint_states",
    "force_feedback",
    "speed_scaling",
    "digital_inputs",
    "digital_outputs",
    "request_state",
    "request_result",
    "request_feedback",
    "dashboard_request_state",
    "dashboard_request_result",
    "total_fail_counter",
    "subsequent_fail_counter",
];

/// Every Redis key [`robot_status`] reads for `robot_id`.
pub fn robot_keys(robot_id: &str) -> Vec<String> {
    ROBOT_SUFFIXES.iter().map(|suffix| format!("{robot_id}_{suffix}")).collect()
}

/// Assemble what the Robot tab shows for one driver instance.
pub fn robot_status(entries: &Entries, robot_id: &str) -> proto::RobotStatus {
    let k = |suffix: &str| format!("{robot_id}_{suffix}");
    proto::RobotStatus {
        robot_id: robot_id.to_string(),
        safety_mode: get_string(entries, &k("safety_mode")),
        robot_mode: get_string(entries, &k("robot_mode")),
        program_state: get_string(entries, &k("program_state")),
        program_running: get_bool(entries, &k("program_running")),
        robot_connected: get_tribool(entries, &k("robot_connected")),
        dashboard_connected: get_tribool(entries, &k("dashboard_connected")),
        remote_control: get_bool(entries, &k("remote_control")),
        tcp_pose: get_floats(entries, &k("tcp_pose")),
        tcp_force: get_floats(entries, &k("tcp_force")),
        joint_states: get_floats(entries, &k("joint_states")),
        force_feedback: get_float(entries, &k("force_feedback")),
        speed_scaling: get_float(entries, &k("speed_scaling")),
        digital_inputs: get_int(entries, &k("digital_inputs")),
        digital_outputs: get_int(entries, &k("digital_outputs")),
        request_state: get_string(entries, &k("request_state")),
        request_result: get_string(entries, &k("request_result")),
        request_feedback: get_string(entries, &k("request_feedback")),
        dashboard_request_state: get_string(entries, &k("dashboard_request_state")),
        dashboard_request_result: get_string(entries, &k("dashboard_request_result")),
        total_fail_counter: get_int(entries, &k("total_fail_counter")),
        subsequent_fail_counter: get_int(entries, &k("subsequent_fail_counter")),
    }
}

/// Decode one goal from its `[id, priority, predicate]` array form.
///
/// Mirrors `micro_sp::running::goal_runner::sp_value_to_goal`, but reports why a
/// value was rejected instead of dropping it, so a malformed queue entry is
/// visible in the UI rather than just missing.
pub fn decode_goal(v: &GuiValue) -> Result<proto::GuiGoal, String> {
    let items = match v {
        GuiValue::Array(ArrayOrUnknown::Array(items)) => items,
        other => return Err(format!("goal is not an array: {}", other.display())),
    };
    if items.len() != 3 {
        return Err(format!("goal has {} fields, expected 3", items.len()));
    }
    let id = match &items[0] {
        GuiValue::String(StringOrUnknown::String(s)) => s.clone(),
        other => return Err(format!("goal id is not a string: {}", other.display())),
    };
    let priority = match &items[1] {
        GuiValue::Int64(IntOrUnknown::Int64(i)) => proto::GuiGoalPriority::from_int(*i),
        other => return Err(format!("goal '{id}' priority is not an int: {}", other.display())),
    };
    let predicate = match &items[2] {
        GuiValue::String(StringOrUnknown::String(s)) => s.clone(),
        other => {
            return Err(format!("goal '{id}' predicate is not a string: {}", other.display()));
        }
    };
    Ok(proto::GuiGoal { id, priority, predicate })
}

fn decode_goal_array(
    entries: &Entries,
    key: &str,
    problems: &mut Vec<String>,
) -> Vec<proto::GuiGoal> {
    let mut out = Vec::new();
    for item in get_array(entries, key) {
        match decode_goal(&item) {
            Ok(goal) => out.push(goal),
            Err(e) => problems.push(format!("{key}: {e}")),
        }
    }
    out
}

/// Assemble what the Goals and Orders tabs show for one `sp_id`.
pub fn goals_status(entries: &Entries, sp_id: &str) -> proto::GoalsStatus {
    let k = |suffix: &str| format!("{sp_id}_{suffix}");
    let mut undecodable = Vec::new();
    proto::GoalsStatus {
        sp_id: sp_id.to_string(),
        incoming: decode_goal_array(entries, &k("incoming_goals"), &mut undecodable),
        scheduled: decode_goal_array(entries, &k("scheduled_goals"), &mut undecodable),
        undecodable,
        current_goal_id: get_string(entries, &k("current_goal_id")),
        current_goal_predicate: get_string(entries, &k("current_goal_predicate")),
        current_goal_state: get_string(entries, &k("current_goal_state")),
        planner_state: get_string(entries, &k("planner_state")),
        plan_state: get_string(entries, &k("plan_state")),
        plan: get_strings(entries, &k("plan")),
        plan_current_step: get_int(entries, &k("plan_current_step")),
        replan_counter: get_int(entries, &k("replan_counter")),
        goal_runner_information: get_string(entries, &k("goal_runner_information")),
        goal_scheduler_information: get_string(entries, &k("goal_scheduler_information")),
        planner_information: get_string(entries, &k("planner_information")),
        plan_runner_information: get_string(entries, &k("plan_runner_information")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(key: &str, value: GuiValue) -> (String, proto::StateEntry) {
        (
            key.to_string(),
            proto::StateEntry {
                key: key.to_string(),
                raw: serde_json::to_string(&value).unwrap(),
                value: Some(value),
            },
        )
    }

    fn goal_value(id: &str, priority: i64, predicate: &str) -> GuiValue {
        GuiValue::Array(ArrayOrUnknown::Array(vec![
            GuiValue::String(StringOrUnknown::String(id.to_string())),
            GuiValue::Int64(IntOrUnknown::Int64(priority)),
            GuiValue::String(StringOrUnknown::String(predicate.to_string())),
        ]))
    }

    #[test]
    fn missing_keys_read_as_defaults_instead_of_panicking() {
        let entries = Entries::new();
        assert_eq!(get_string(&entries, "nope"), "UNKNOWN");
        assert!(!get_bool(&entries, "nope"));
        assert_eq!(get_int(&entries, "nope"), 0);
        assert_eq!(get_float(&entries, "nope"), 0.0);
        assert!(get_floats(&entries, "nope").is_empty());
        // And the whole derived view still builds.
        let status = robot_status(&entries, "r1");
        assert_eq!(status.robot_id, "r1");
        assert_eq!(status.robot_connected, None, "an absent key is not a measured false");
    }

    #[test]
    fn decodes_a_goal_queue_and_reports_the_broken_entry() {
        let entries: Entries = vec![entry(
            "sp_incoming_goals",
            GuiValue::Array(ArrayOrUnknown::Array(vec![
                goal_value("g1", 1, "var:done == true"),
                GuiValue::String(StringOrUnknown::String("nonsense".to_string())),
            ])),
        )]
        .into_iter()
        .collect();

        let status = goals_status(&entries, "sp");
        assert_eq!(status.incoming.len(), 1);
        assert_eq!(status.incoming[0].id, "g1");
        assert_eq!(status.incoming[0].priority, proto::GuiGoalPriority::High);
        assert_eq!(status.undecodable.len(), 1, "the bad entry should be reported");
    }

    #[test]
    fn a_measured_false_is_not_an_absent_key() {
        let entries: Entries =
            vec![entry("r1_robot_connected", GuiValue::Bool(BoolOrUnknown::Bool(false)))]
                .into_iter()
                .collect();
        assert_eq!(get_tribool(&entries, "r1_robot_connected"), Some(false));
        assert_eq!(get_tribool(&entries, "r1_dashboard_connected"), None);
        // An explicit UNKNOWN is no more a measurement than a missing key.
        let unknown: Entries =
            vec![entry("r1_robot_connected", GuiValue::Bool(BoolOrUnknown::UNKNOWN))]
                .into_iter()
                .collect();
        assert_eq!(get_tribool(&unknown, "r1_robot_connected"), None);
    }

    /// The drift guard for the fast robot poller: it fetches only
    /// `robot_keys(id)`, so anything `robot_status` reads that is missing from
    /// `ROBOT_SUFFIXES` would silently read as absent there and as its real
    /// value in the 250 ms poll.
    #[test]
    fn robot_suffixes_cover_every_key_robot_status_reads() {
        // A distinctive value per type, so a field that falls back to its
        // default when the key is missing changes observably.
        let mut full = Entries::new();
        full.extend(ROBOT_SUFFIXES.iter().map(|suffix| {
            entry(
                &format!("r1_{suffix}"),
                GuiValue::String(StringOrUnknown::String(format!("v_{suffix}"))),
            )
        }));
        for (suffix, value) in [
            ("program_running", GuiValue::Bool(BoolOrUnknown::Bool(true))),
            ("robot_connected", GuiValue::Bool(BoolOrUnknown::Bool(true))),
            ("dashboard_connected", GuiValue::Bool(BoolOrUnknown::Bool(false))),
            ("remote_control", GuiValue::Bool(BoolOrUnknown::Bool(true))),
            ("force_feedback", GuiValue::Float64(FloatOrUnknown::Float64(1.5))),
            ("speed_scaling", GuiValue::Float64(FloatOrUnknown::Float64(0.5))),
            ("digital_inputs", GuiValue::Int64(IntOrUnknown::Int64(3))),
            ("digital_outputs", GuiValue::Int64(IntOrUnknown::Int64(4))),
            ("total_fail_counter", GuiValue::Int64(IntOrUnknown::Int64(5))),
            ("subsequent_fail_counter", GuiValue::Int64(IntOrUnknown::Int64(6))),
        ] {
            let (key, e) = entry(&format!("r1_{suffix}"), value);
            full.insert(key, e);
        }
        for suffix in ["tcp_pose", "tcp_force", "joint_states"] {
            let (key, e) = entry(
                &format!("r1_{suffix}"),
                GuiValue::Array(ArrayOrUnknown::Array(vec![GuiValue::Float64(
                    FloatOrUnknown::Float64(0.25),
                )])),
            );
            full.insert(key, e);
        }
        // Noise the poller would never fetch, to prove it does not need to.
        let (key, e) = entry("sp_planner_state", GuiValue::Int64(IntOrUnknown::Int64(1)));
        full.insert(key, e);

        let keys = robot_keys("r1");
        let restricted: Entries =
            keys.iter().filter_map(|k| full.get(k).map(|e| (k.clone(), e.clone()))).collect();

        assert_eq!(
            robot_status(&full, "r1"),
            robot_status(&restricted, "r1"),
            "robot_status reads a key that ROBOT_SUFFIXES does not list"
        );
    }

    #[test]
    fn an_int_where_a_float_belongs_is_accepted() {
        let entries: Entries =
            vec![entry("r1_speed_scaling", GuiValue::Int64(IntOrUnknown::Int64(1)))]
                .into_iter()
                .collect();
        assert_eq!(get_float(&entries, "r1_speed_scaling"), 1.0);
    }
}
