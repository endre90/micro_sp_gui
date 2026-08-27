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

/// Everything the driver would reject a dashboard request for, checked here so
/// the operator is told before anything is written.
///
/// Returns the resolved command name: [`proto::dashboard_spec`] maps the driver's
/// aliases onto their canonical spelling, and writing the canonical one keeps
/// `dashboard_command` readable next to the reply it produced.
fn validate_dashboard(req: &proto::RobotDashboardCommand) -> Result<(&'static str, String), String> {
    if req.robot_id.trim().is_empty() {
        return Err("Robot id is empty.".to_string());
    }
    let spec = proto::dashboard_spec(&req.command)
        .ok_or_else(|| format!("Unknown dashboard command '{}'.", req.command))?;

    // The driver reads the argument key for every command, so an argument aimed
    // at a command that ignores one is dropped rather than sent - otherwise a
    // stale value would sit in Redis looking like it applied.
    let arg = if spec.takes_arg() {
        req.arg.clone().unwrap_or_default().trim().to_string()
    } else {
        String::new()
    };

    if !spec.arg_is_satisfied(&arg) {
        return Err(format!(
            "Dashboard command '{}' needs an argument ({}).",
            spec.name,
            spec.arg.hint()
        ));
    }
    // A choice with a spelling the driver does not know fails in the driver with
    // a message nobody sees unless they are watching Redis.
    let options = spec.arg.options();
    if !options.is_empty()
        && !arg.is_empty()
        && !options.iter().any(|o| o.eq_ignore_ascii_case(&arg))
    {
        return Err(format!(
            "Dashboard command '{}' takes one of {}, not '{}'.",
            spec.name,
            options.join(", "),
            arg
        ));
    }

    Ok((spec.name, arg))
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
    let (command, arg) = validate_dashboard(req)?;

    let r = &req.robot_id;
    let state = State::new();
    let state = state.add(
        assign!(v!(&&format!("{}_dashboard_command", r)), command.to_spvalue()),
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
    log::info!(target: LOG_TARGET, "Dashboard '{}' -> '{}'.", command, r);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn req(command: &str, arg: Option<&str>) -> proto::RobotDashboardCommand {
        proto::RobotDashboardCommand {
            robot_id: "r1".to_string(),
            command: command.to_string(),
            arg: arg.map(str::to_string),
        }
    }

    #[test]
    fn an_alias_resolves_to_the_canonical_name() {
        let (command, arg) = validate_dashboard(&req("reset_protective_stop", None)).unwrap();
        assert_eq!(command, "unlock_protective_stop");
        assert!(arg.is_empty());
    }

    #[test]
    fn a_missing_required_argument_is_refused() {
        assert!(validate_dashboard(&req("load", None)).is_err());
        assert!(validate_dashboard(&req("load", Some("   "))).is_err());
        assert_eq!(
            validate_dashboard(&req("load", Some(" prog.urp "))).unwrap(),
            ("load", "prog.urp".to_string())
        );
    }

    #[test]
    fn an_optional_argument_may_be_left_out() {
        // `generate_flight_report` defaults to `system` in the driver.
        assert_eq!(
            validate_dashboard(&req("generate_flight_report", None)).unwrap(),
            ("generate_flight_report", String::new())
        );
    }

    #[test]
    fn a_choice_outside_the_options_is_refused() {
        assert!(validate_dashboard(&req("set_operational_mode", Some("sideways"))).is_err());
        assert!(validate_dashboard(&req("set_operational_mode", Some("Manual"))).is_ok());
    }

    #[test]
    fn an_argument_to_an_argument_less_command_is_dropped() {
        let (_, arg) = validate_dashboard(&req("stop", Some("nonsense"))).unwrap();
        assert!(arg.is_empty(), "a stale arg must not be written as if it applied");
    }

    #[test]
    fn an_unknown_command_and_an_empty_robot_id_are_both_refused() {
        assert!(validate_dashboard(&req("no_such_command", None)).is_err());
        let mut bad = req("stop", None);
        bad.robot_id = "  ".to_string();
        assert!(validate_dashboard(&bad).is_err());
    }
}
