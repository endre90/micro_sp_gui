//! The `ur_redis_driver` request interface, as a DTO.
//!
//! The driver's protocol is a flat set of `{robot_id}_*` Redis keys polled on
//! both sides; there is no channel. This module carries one field per key the
//! driver reads, so the server's only job is naming and typing them. See
//! `crates/server/src/api_robot.rs`.
//!
//! The constants here are duplicated from `ur_redis_driver` (which cannot be
//! compiled to wasm). `crates/server/tests/mirror.rs` asserts they still match
//! the driver's own definitions.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Payload as the driver's URScript templates want it:
/// `mass,[cogx,cogy,cogz],[ixx,iyy,izz,ixy,ixz,iyz]`.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Default)]
pub struct Payload {
    /// Mass in kilograms.
    pub mass: f64,
    /// Centre of gravity offsets in metres, from the tool mount.
    pub cog_x: f64,
    pub cog_y: f64,
    pub cog_z: f64,
    /// Inertia matrix in kg*m^2, origin at the CoG, axes aligned with the flange.
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyz: f64,
}

impl fmt::Display for Payload {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{},[{},{},{}],[{},{},{},{},{},{}]",
            self.mass,
            self.cog_x,
            self.cog_y,
            self.cog_z,
            self.ixx,
            self.iyy,
            self.izz,
            self.ixy,
            self.ixz,
            self.iyz
        )
    }
}

pub const RSP_ONLY_PAYLOAD: &str = "0.69,[0.026,-0.008,0.012],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_SPONGE_PAYLOAD: &str = "1.88,[0.002,0.003,0.071],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_GRIPPER_PAYLOAD: &str = "2.24,[-0.001,0.002,0.068],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_BVT_PAYLOAD: &str = "1.3,[0.001,0.005,0.06],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_SVT_PAYLOAD: &str = "1.3,[0.001,0.005,0.06],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_PHOTONEO_PAYLOAD: &str = "3.29,[0.008,0.003,0.082],[0.0,0.0,0.0,0.0,0.0,0.0]";

/// Measured payload presets, as `(label, payload string)`.
pub const PAYLOAD_PRESETS: &[(&str, &str)] = &[
    ("rsp only", RSP_ONLY_PAYLOAD),
    ("rsp + gripper", RSP_AND_GRIPPER_PAYLOAD),
    ("rsp + sponge", RSP_AND_SPONGE_PAYLOAD),
    ("rsp + bvt", RSP_AND_BVT_PAYLOAD),
    ("rsp + svt", RSP_AND_SVT_PAYLOAD),
    ("rsp + photoneo", RSP_AND_PHOTONEO_PAYLOAD),
];

/// Every URScript template the driver can render, i.e. the filenames in
/// `ur_redis_driver/templates/` without the `.script` suffix. An unrecognised
/// `command_type` is rejected before it reaches the templating engine, so this
/// list is the complete set of legal values.
pub const COMMAND_TYPES: &[&str] = &[
    "safe_move_j",
    "safe_move_l",
    "safe_move_l_relative",
    "unsafe_move_j",
    "unsafe_move_l",
    "unsafe_move_l_relative",
    "trajectory_unsafe_move_j",
    "trajectory_unsafe_move_j_old",
    "trajectory_unsafe_move_l",
    "pick_vacuum",
    "pick_vacuum_broken",
    "place_vacuum",
    "start_vacuum",
    "stop_vacuum",
    "lock_rsp",
    "unlock_rsp",
    "set_payload",
    "get_force",
    "get_force_old",
];

/// Dashboard commands the driver accepts, with whether each takes an argument.
/// Mirrors `ur_redis_driver::DashboardCommand::parse`.
pub const DASHBOARD_COMMANDS: &[(&str, bool)] = &[
    ("stop", false),
    ("pause", false),
    ("play", false),
    ("power_on", false),
    ("power_off", false),
    ("brake_release", false),
    ("unlock_protective_stop", false),
    ("reset_protective_stop", false),
    ("close_safety_popup", false),
    ("close_popup", false),
    ("restart_safety", false),
    ("shutdown", false),
    ("robot_mode", false),
    ("safety_status", false),
    ("program_state", false),
    ("is_program_running", false),
    ("is_in_remote_control", false),
    ("get_loaded_program", false),
    ("get_robot_model", false),
    ("polyscope_version", false),
    ("load", true),
    ("load_installation", true),
    ("popup", true),
    ("add_to_log", true),
];

/// Everything the driver reads for one motion request.
///
/// The driver refuses a request whose whole key set is not present and
/// readable, so the server always writes every field - there is no partial
/// update. `trigger` is what actually starts it.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct RobotCommand {
    pub robot_id: String,
    pub command_type: String,
    pub acceleration: f64,
    pub velocity: f64,
    pub global_acceleration_scaling: f64,
    pub global_velocity_scaling: f64,
    pub use_execution_time: bool,
    pub execution_time: f64,
    pub use_blend_radius: bool,
    pub blend_radius: f64,
    pub use_joint_positions: bool,
    pub joint_positions: [f64; 6],
    pub use_preferred_joint_config: bool,
    pub preferred_joint_config: [f64; 6],
    pub use_relative_pose: bool,
    pub relative_pose: [f64; 6],
    pub use_payload: bool,
    pub payload: String,
    pub baseframe_id: String,
    pub faceplate_id: String,
    pub goal_feature_id: String,
    pub tcp_id: String,
    pub root_frame_id: String,
    pub force_threshold: f64,
    pub trigger: bool,
}

impl Default for RobotCommand {
    fn default() -> Self {
        Self {
            robot_id: "r1".to_string(),
            command_type: "unsafe_move_j".to_string(),
            acceleration: 0.1,
            velocity: 0.1,
            global_acceleration_scaling: 1.0,
            global_velocity_scaling: 1.0,
            use_execution_time: false,
            execution_time: 0.0,
            use_blend_radius: false,
            blend_radius: 0.0,
            use_joint_positions: false,
            joint_positions: [0.0; 6],
            use_preferred_joint_config: false,
            preferred_joint_config: [0.0; 6],
            use_relative_pose: false,
            relative_pose: [0.0; 6],
            use_payload: false,
            payload: RSP_ONLY_PAYLOAD.to_string(),
            baseframe_id: "base_link".to_string(),
            faceplate_id: "tool0".to_string(),
            goal_feature_id: String::new(),
            tcp_id: String::new(),
            root_frame_id: "world".to_string(),
            force_threshold: 20.0,
            trigger: true,
        }
    }
}

impl RobotCommand {
    /// Frame ids are only needed when the driver will actually look them up:
    /// joint-space and relative moves skip the lookup entirely, as do the
    /// trajectory and rsp-lock templates.
    pub fn needs_frames(&self) -> bool {
        !self.use_joint_positions
            && !self.use_relative_pose
            && !matches!(
                self.command_type.as_str(),
                "trajectory_unsafe_move_j" | "lock_rsp" | "unlock_rsp"
            )
    }

    /// The checks the driver would fail the request on, done up front so the
    /// operator gets told before anything is written.
    pub fn validate(&self) -> Result<(), String> {
        if self.robot_id.trim().is_empty() {
            return Err("Robot id is empty.".to_string());
        }
        if !COMMAND_TYPES.contains(&self.command_type.as_str()) {
            return Err(format!("Unknown command_type '{}'.", self.command_type));
        }
        if self.needs_frames() {
            if self.goal_feature_id.trim().is_empty() {
                return Err("Goal feature frame not selected.".to_string());
            }
            if self.tcp_id.trim().is_empty() {
                return Err("TCP frame not selected.".to_string());
            }
            if self.baseframe_id.trim().is_empty() {
                return Err("Baseframe not selected.".to_string());
            }
            if self.faceplate_id.trim().is_empty() {
                return Err("Faceplate frame not selected.".to_string());
            }
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct RobotDashboardCommand {
    pub robot_id: String,
    pub command: String,
    pub arg: Option<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct RobotCancelRequest {
    pub robot_id: String,
}

/// Read-only mirror of what the driver publishes.
///
/// The driver only rewrites these when they change (with a 0.25 N force epsilon
/// and 1e-4 pose epsilon), so a parked robot stops updating - never read
/// staleness as "the driver died", use `robot_connected` for that.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct RobotStatus {
    pub robot_id: String,
    pub safety_mode: String,
    pub robot_mode: String,
    /// Describes the *loaded pendant program*, so it stays "STOPPED" while an
    /// injected script runs. `program_running` is the one that tracks the
    /// driver's own scripts.
    pub program_state: String,
    pub program_running: bool,
    /// `None` when nothing has published the key at all - which is not the same
    /// thing as the driver reporting `false`, and must not be drawn as if it
    /// were. An older `ur_redis_driver` never writes it.
    pub robot_connected: Option<bool>,
    /// `None` when unpublished; see [`RobotStatus::robot_connected`].
    pub dashboard_connected: Option<bool>,
    pub remote_control: bool,
    pub tcp_pose: Vec<f64>,
    pub tcp_force: Vec<f64>,
    pub joint_states: Vec<f64>,
    pub force_feedback: f64,
    pub speed_scaling: f64,
    pub digital_inputs: i64,
    pub digital_outputs: i64,
    pub request_state: String,
    pub request_result: String,
    pub request_feedback: String,
    pub dashboard_request_state: String,
    pub dashboard_request_result: String,
    pub total_fail_counter: i64,
    pub subsequent_fail_counter: i64,
}

impl RobotStatus {
    /// A cancelled goal terminates as `succeeded` with result `cancelled`, so
    /// "not succeeded" is not the same as "failed".
    pub fn was_cancelled(&self) -> bool {
        self.request_result == "cancelled"
    }

    pub fn request_failed(&self) -> bool {
        self.request_state == "failed"
    }

    /// Safety modes in which the driver will accept a goal at all.
    pub fn safety_accepts_goal(&self) -> bool {
        matches!(self.safety_mode.as_str(), "NORMAL" | "REDUCED")
    }
}
