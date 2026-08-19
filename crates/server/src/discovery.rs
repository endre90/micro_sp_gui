//! Working out which systems are in the keyspace.
//!
//! Nothing in Redis records what `sp_id`s or robots exist, so they are inferred
//! from which bookkeeping variables are present. This replaces the hardcoded
//! `"sp1"` / `"r1"` the old native GUI carried.

use std::collections::BTreeSet;

/// Variables only a micro_sp runner creates. `_dashboard_command` is
/// deliberately **not** in this list: `ur_redis_driver` seeds
/// `{robot}_dashboard_command` too, so it would report every robot as an sp_id.
const SP_MARKERS: &[&str] = &[
    "_planner_state",
    "_main_runner_information",
    "_goal_runner_information",
    "_sop_state",
    "_plan_runner_information",
];

/// A robot interface needs both of these, which no runner creates.
const ROBOT_MARKERS_ALL: &[&str] = &["_request_trigger", "_command_type"];

fn prefixes_with_any(keys: &BTreeSet<String>, markers: &[&str]) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for key in keys {
        for marker in markers {
            if let Some(prefix) = key.strip_suffix(marker)
                && !prefix.is_empty()
            {
                out.insert(prefix.to_string());
            }
        }
    }
    out
}

fn prefixes_with_all(keys: &BTreeSet<String>, markers: &[&str]) -> BTreeSet<String> {
    let candidates = prefixes_with_any(keys, &markers[..1]);
    candidates
        .into_iter()
        .filter(|p| markers.iter().all(|m| keys.contains(&format!("{p}{m}"))))
        .collect()
}

/// Returns `(sp_ids, robot_ids)`, sorted, with the configured seeds folded in.
pub fn discover(
    keys: &BTreeSet<String>,
    seed_sp: Option<&str>,
    seed_robot: Option<&str>,
) -> (Vec<String>, Vec<String>) {
    let mut robots = prefixes_with_all(keys, ROBOT_MARKERS_ALL);
    if let Some(seed) = seed_robot.filter(|s| !s.trim().is_empty()) {
        robots.insert(seed.to_string());
    }

    let mut sps = prefixes_with_any(keys, SP_MARKERS);
    if let Some(seed) = seed_sp.filter(|s| !s.trim().is_empty()) {
        sps.insert(seed.to_string());
    }
    // A robot is never an sp_id, whatever else its keys look like.
    for robot in &robots {
        sps.remove(robot);
    }

    (sps.into_iter().collect(), robots.into_iter().collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn keys(list: &[&str]) -> BTreeSet<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn finds_a_runner_and_a_robot_separately() {
        let k = keys(&[
            "sp_planner_state",
            "sp_main_runner_information",
            "sp_dashboard_command",
            "r1_request_trigger",
            "r1_command_type",
            "r1_dashboard_command",
            "unrelated_variable",
        ]);
        let (sps, robots) = discover(&k, None, None);
        assert_eq!(sps, vec!["sp".to_string()]);
        assert_eq!(robots, vec!["r1".to_string()]);
    }

    /// The regression this module exists for: a robot's `_dashboard_command`
    /// must not make it look like a micro_sp instance.
    #[test]
    fn a_robot_is_never_reported_as_an_sp_id() {
        let k = keys(&["r1_request_trigger", "r1_command_type", "r1_dashboard_command"]);
        let (sps, robots) = discover(&k, None, None);
        assert!(sps.is_empty(), "expected no sp_ids, got {sps:?}");
        assert_eq!(robots, vec!["r1".to_string()]);
    }

    /// Half an interface is not an interface.
    #[test]
    fn a_partial_robot_interface_is_not_a_robot() {
        let k = keys(&["r1_request_trigger"]);
        let (_, robots) = discover(&k, None, None);
        assert!(robots.is_empty());
    }

    #[test]
    fn seeds_show_up_even_on_an_empty_keyspace() {
        let (sps, robots) = discover(&BTreeSet::new(), Some("sp"), Some("r1"));
        assert_eq!(sps, vec!["sp".to_string()]);
        assert_eq!(robots, vec!["r1".to_string()]);
    }

    #[test]
    fn blank_seeds_are_ignored() {
        let (sps, robots) = discover(&BTreeSet::new(), Some("  "), Some(""));
        assert!(sps.is_empty());
        assert!(robots.is_empty());
    }
}
