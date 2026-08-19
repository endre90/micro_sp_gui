//! Driving `ur_redis_driver` from the GUI.
//!
//! The driver's protocol (see its README) is: write every parameter key, set
//! `request_state` to `initial`, then raise `request_trigger`. It refuses a
//! request whose whole key set is not present and readable, so this module
//! always writes the complete set - there is no partial update. All of it goes
//! out in one `StateManager::set_state`, i.e. a single MSET, so the trigger
//! cannot be observed before the parameters it applies to.

use crate::LOG_TARGET;
use micro_sp::*;
use micro_sp_gui_protocol as proto;
use std::sync::Arc;

/// Build the full request state for one motion command.
///
/// Kept separate from the write so `tests/mirror.rs` can check every key it
/// produces against `ur_redis_driver::generate_robot_interface_state`.
pub fn command_to_state(cmd: &proto::RobotCommand) -> State {
    let r = &cmd.robot_id;
    let state = State::new();

    // Handshake. `request_cancel` is cleared deliberately: a stale cancel left
    // over from a previous goal would otherwise abort this one immediately.
    let state = state.add(
        assign!(bv!(&&format!("{}_request_trigger", r)), cmd.trigger.to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(v!(&&format!("{}_request_state", r)), "initial".to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(bv!(&&format!("{}_request_cancel", r)), false.to_spvalue()),
        LOG_TARGET,
    );

    // Motion parameters.
    let state = state.add(
        assign!(v!(&&format!("{}_command_type", r)), cmd.command_type.clone().to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(fv!(&&format!("{}_acceleration", r)), cmd.acceleration.to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(fv!(&&format!("{}_velocity", r)), cmd.velocity.to_spvalue()),
        LOG_TARGET,
    );
    // The driver reads both scalings; the old native GUI had the fields but
    // never wrote them, so they silently stayed UNKNOWN.
    let state = state.add(
        assign!(
            fv!(&&format!("{}_global_acceleration_scaling", r)),
            cmd.global_acceleration_scaling.to_spvalue()
        ),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(
            fv!(&&format!("{}_global_velocity_scaling", r)),
            cmd.global_velocity_scaling.to_spvalue()
        ),
        LOG_TARGET,
    );

    let state = state.add(
        assign!(
            bv!(&&format!("{}_use_execution_time", r)),
            cmd.use_execution_time.to_spvalue()
        ),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(fv!(&&format!("{}_execution_time", r)), cmd.execution_time.to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(bv!(&&format!("{}_use_blend_radius", r)), cmd.use_blend_radius.to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(fv!(&&format!("{}_blend_radius", r)), cmd.blend_radius.to_spvalue()),
        LOG_TARGET,
    );

    let state = state.add(
        assign!(
            bv!(&&format!("{}_use_joint_positions", r)),
            cmd.use_joint_positions.to_spvalue()
        ),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(
            av!(&&format!("{}_joint_positions", r)),
            cmd.joint_positions.to_vec().to_spvalue()
        ),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(
            bv!(&&format!("{}_use_preferred_joint_config", r)),
            cmd.use_preferred_joint_config.to_spvalue()
        ),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(
            av!(&&format!("{}_preferred_joint_config", r)),
            cmd.preferred_joint_config.to_vec().to_spvalue()
        ),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(bv!(&&format!("{}_use_relative_pose", r)), cmd.use_relative_pose.to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(av!(&&format!("{}_relative_pose", r)), cmd.relative_pose.to_vec().to_spvalue()),
        LOG_TARGET,
    );

    let state = state.add(
        assign!(bv!(&&format!("{}_use_payload", r)), cmd.use_payload.to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(v!(&&format!("{}_payload", r)), cmd.payload.clone().to_spvalue()),
        LOG_TARGET,
    );

    // Frames. Written even when unused, because the driver's key-set check does
    // not care whether this particular command_type will read them.
    let state = state.add(
        assign!(v!(&&format!("{}_baseframe_id", r)), cmd.baseframe_id.clone().to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(v!(&&format!("{}_faceplate_id", r)), cmd.faceplate_id.clone().to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(
            v!(&&format!("{}_goal_feature_id", r)),
            cmd.goal_feature_id.clone().to_spvalue()
        ),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(v!(&&format!("{}_tcp_id", r)), cmd.tcp_id.clone().to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(v!(&&format!("{}_root_frame_id", r)), cmd.root_frame_id.clone().to_spvalue()),
        LOG_TARGET,
    );
    state.add(
        assign!(fv!(&&format!("{}_force_threshold", r)), cmd.force_threshold.to_spvalue()),
        LOG_TARGET,
    )
}

/// Validate, then write the request in one MSET.
pub async fn send_command(
    cm: &Arc<ConnectionManager>,
    cmd: &proto::RobotCommand,
) -> Result<(), String> {
    cmd.validate()?;
    let state = command_to_state(cmd);
    let mut con = cm.get_connection().await;
    StateManager::set_state(&mut con, &state).await;
    log::info!(
        target: LOG_TARGET,
        "Sent '{}' to '{}' (trigger {}).", cmd.command_type, cmd.robot_id, cmd.trigger
    );
    Ok(())
}

/// Ask the driver to abandon the running goal.
///
/// Only `request_cancel` is written; the driver clears it, sends a dashboard
/// `stop`, and terminates the goal as **succeeded** with result `cancelled`.
/// That is not a failure, and the UI must not paint it as one.
pub async fn cancel(cm: &Arc<ConnectionManager>, robot_id: &str) -> Result<(), String> {
    if robot_id.trim().is_empty() {
        return Err("Robot id is empty.".to_string());
    }
    let mut con = cm.get_connection().await;
    StateManager::set_sp_value(
        &mut con,
        &format!("{robot_id}_request_cancel"),
        &true.to_spvalue(),
    )
    .await;
    log::info!(target: LOG_TARGET, "Requested cancel on '{robot_id}'.");
    Ok(())
}

/// Send a dashboard command.
///
/// The dashboard has its own handshake keys so a slow dashboard round trip
/// cannot stall the 5 ms motion loop. Raising the trigger is what makes it
/// happen - the old native GUI wrote `dashboard_command` but never the trigger,
/// so its "Stop" button did nothing.
pub async fn dashboard(
    cm: &Arc<ConnectionManager>,
    req: &proto::RobotDashboardCommand,
) -> Result<(), String> {
    if req.robot_id.trim().is_empty() {
        return Err("Robot id is empty.".to_string());
    }
    let entry = proto::DASHBOARD_COMMANDS
        .iter()
        .find(|(name, _)| *name == req.command.as_str())
        .ok_or_else(|| format!("Unknown dashboard command '{}'.", req.command))?;
    let arg = req.arg.clone().unwrap_or_default();
    if entry.1 && arg.trim().is_empty() {
        return Err(format!("Dashboard command '{}' needs an argument.", req.command));
    }

    let r = &req.robot_id;
    let state = State::new();
    let state = state.add(
        assign!(v!(&&format!("{}_dashboard_command", r)), req.command.clone().to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(v!(&&format!("{}_dashboard_command_arg", r)), arg.to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(v!(&&format!("{}_dashboard_request_state", r)), "initial".to_spvalue()),
        LOG_TARGET,
    );
    let state = state.add(
        assign!(bv!(&&format!("{}_dashboard_request_trigger", r)), true.to_spvalue()),
        LOG_TARGET,
    );

    let mut con = cm.get_connection().await;
    StateManager::set_state(&mut con, &state).await;
    log::info!(target: LOG_TARGET, "Dashboard '{}' -> '{}'.", req.command, r);
    Ok(())
}
