//! Writing state variables.

use crate::{LOG_TARGET, convert};
use micro_sp::*;
use micro_sp_gui_protocol as proto;
use std::sync::Arc;

/// Write a batch in one MSET.
///
/// `StateManager::set_state` merges rather than replaces, so only the listed
/// keys change. Every value is validated first and the whole batch is reported
/// per key, because a half-applied edit the operator cannot see is worse than a
/// rejected one.
pub async fn set_values(
    cm: &Arc<ConnectionManager>,
    req: &proto::SetValuesRequest,
) -> proto::WriteReport {
    let mut report = proto::WriteReport::default();
    let mut state = State::new();

    for item in &req.values {
        let key = item.key.trim();
        if key.is_empty() {
            report.failed.push((item.key.clone(), "key is empty".to_string()));
            continue;
        }
        if key.starts_with(TF_PREFIX) {
            // Frames have their own API, which refuses to create a cycle. Letting
            // them be written as raw state would happily corrupt the tree.
            report.failed.push((
                item.key.clone(),
                format!("'{TF_PREFIX}' keys are frames; use the Transforms tab"),
            ));
            continue;
        }

        let value = convert::from_proto(&item.value);
        // Typing the variable from the value it is being given means
        // `SPAssignment::new` cannot hit its type-mismatch panic.
        let var = SPVariable::new(key, value.has_type());
        state = state.add(SPAssignment::new(var, value), LOG_TARGET);
        report.ok.push(key.to_string());
    }

    if !state.state.is_empty() {
        let mut con = cm.get_connection().await;
        StateManager::set_state(&mut con, &state).await;
        log::info!(target: LOG_TARGET, "Wrote {} state key(s).", report.ok.len());
    }
    report
}

pub async fn delete_keys(
    cm: &Arc<ConnectionManager>,
    req: &proto::DeleteKeysRequest,
) -> proto::WriteReport {
    let mut report = proto::WriteReport::default();
    let keys: Vec<String> = req
        .keys
        .iter()
        .filter_map(|k| {
            let k = k.trim();
            if k.is_empty() {
                None
            } else {
                report.ok.push(k.to_string());
                Some(k.to_string())
            }
        })
        .collect();

    if !keys.is_empty() {
        let mut con = cm.get_connection().await;
        StateManager::remove_sp_values(&mut con, &keys).await;
        log::info!(target: LOG_TARGET, "Deleted {} key(s).", keys.len());
    }
    report
}
