//! Sending goals.
//!
//! A goal reaches the runner through `{sp_id}_incoming_goals`, an array of
//! `[id, priority, predicate]` triples that the goal runner drains and merges
//! into `scheduled_goals`.
//!
//! **This appends.** Both of the old native tabs wrote the whole array with
//! `set_sp_value`, which throws away anything the runner has not drained yet -
//! and two operators pressing Send at the same time lost one batch entirely.
//! Reading first and appending narrows that to the gap between the read and the
//! write. It cannot close it: micro_sp's Redis layer has no `WATCH`/`MULTI`, and
//! its own runners carry the same caveat.

use crate::LOG_TARGET;
use micro_sp::running::goal_runner::{GoalPriority, goal_string_to_sp_value};
use micro_sp::*;
use micro_sp_gui_protocol as proto;
use std::sync::Arc;

fn priority_from_proto(p: proto::GuiGoalPriority) -> GoalPriority {
    GoalPriority::from_int(&p.to_int())
}

pub async fn send(
    cm: &Arc<ConnectionManager>,
    req: &proto::SendGoalsRequest,
) -> proto::SendGoalsResponse {
    let sp_id = req.sp_id.trim();
    if sp_id.is_empty() {
        return proto::SendGoalsResponse {
            error: Some("No sp_id selected.".to_string()),
            ..Default::default()
        };
    }
    if req.goals.is_empty() {
        return proto::SendGoalsResponse {
            error: Some("Nothing to send.".to_string()),
            ..Default::default()
        };
    }
    for goal in &req.goals {
        if goal.id.trim().is_empty() {
            return proto::SendGoalsResponse {
                error: Some("A goal has an empty id.".to_string()),
                ..Default::default()
            };
        }
        if goal.predicate.trim().is_empty() {
            return proto::SendGoalsResponse {
                error: Some(format!("Goal '{}' has an empty predicate.", goal.id)),
                ..Default::default()
            };
        }
    }

    let key = format!("{sp_id}_incoming_goals");
    let mut con = cm.get_connection().await;

    // Whatever is still queued. An absent key and a typed UNKNOWN both mean
    // "nothing queued", which is the normal case.
    let existing: Vec<SPValue> = if req.replace {
        Vec::new()
    } else {
        match StateManager::get_sp_value(&mut con, &key).await {
            Some(SPValue::Array(ArrayOrUnknown::Array(items))) => items,
            _ => Vec::new(),
        }
    };
    let queued_before = existing.len();

    let mut goals = existing;
    for goal in &req.goals {
        // Built with micro_sp's own encoder, so the layout stays correct if
        // `goal_to_sp_value` ever changes.
        goals.push(goal_string_to_sp_value(
            goal.id.trim(),
            &goal.predicate.trim().to_string(),
            priority_from_proto(goal.priority),
        ));
    }

    let payload = SPValue::Array(ArrayOrUnknown::Array(goals));
    StateManager::set_sp_value(&mut con, &key, &payload).await;
    log::info!(
        target: LOG_TARGET,
        "Queued {} goal(s) on '{sp_id}' ({queued_before} already waiting, replace={}).",
        req.goals.len(),
        req.replace
    );

    proto::SendGoalsResponse {
        accepted: req.goals.len(),
        queued_before,
        error: None,
    }
}
