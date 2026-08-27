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

/// What a dashboard command does with `{robot_id}_dashboard_command_arg`.
///
/// `required` is exactly the driver's own `require_arg` check, so the server can
/// refuse an incomplete request before writing anything rather than letting the
/// driver fail it a poll later. `tests/mirror.rs` asserts the two agree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DashboardArg {
    /// The driver ignores the argument key for this command.
    None,
    /// Free text - a filename, a directory, a message.
    Text { hint: &'static str, required: bool },
    /// One of a fixed set of spellings. A non-required choice has a default the
    /// driver falls back to, named first.
    Choice { options: &'static [&'static str], required: bool },
}

impl DashboardArg {
    pub fn takes_arg(&self) -> bool {
        !matches!(self, DashboardArg::None)
    }

    pub fn is_required(&self) -> bool {
        match self {
            DashboardArg::None => false,
            DashboardArg::Text { required, .. } | DashboardArg::Choice { required, .. } => {
                *required
            }
        }
    }

    /// The legal spellings, or empty for free text.
    pub fn options(&self) -> &'static [&'static str] {
        match self {
            DashboardArg::Choice { options, .. } => options,
            _ => &[],
        }
    }

    pub fn hint(&self) -> &'static str {
        match self {
            DashboardArg::None => "no argument",
            DashboardArg::Text { hint, .. } => hint,
            DashboardArg::Choice { options, .. } => options.first().copied().unwrap_or(""),
        }
    }
}

/// Which section of the Dashboard panel a command belongs in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DashboardGroup {
    /// Start, stop, pause and resume.
    Program,
    /// The 48 V supply and the brakes.
    Power,
    /// Getting out of a protective or safeguard stop, and the popups that block one.
    Safety,
    /// Loading a program or an installation.
    Load,
    /// The PolyScope operational mode.
    Mode,
    /// Read-only: the reply *is* the answer.
    Query,
    /// Flight reports and support files.
    Diagnostics,
    /// Popups, the pendant log, and shutting the controller down.
    Misc,
}

impl DashboardGroup {
    pub fn title(&self) -> &'static str {
        match self {
            DashboardGroup::Program => "Program control",
            DashboardGroup::Power => "Power",
            DashboardGroup::Safety => "Safety recovery",
            DashboardGroup::Load => "Load",
            DashboardGroup::Mode => "Operational mode",
            DashboardGroup::Query => "Queries",
            DashboardGroup::Diagnostics => "Diagnostics",
            DashboardGroup::Misc => "Misc",
        }
    }

    /// In the order the panel draws them: what an operator reaches for during a
    /// recovery first, the slow and destructive ones last.
    pub const ALL: &'static [DashboardGroup] = &[
        DashboardGroup::Program,
        DashboardGroup::Power,
        DashboardGroup::Safety,
        DashboardGroup::Mode,
        DashboardGroup::Load,
        DashboardGroup::Query,
        DashboardGroup::Misc,
        DashboardGroup::Diagnostics,
    ];
}

/// One command the driver's dashboard interface accepts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DashboardCommandSpec {
    /// The string written to `{robot_id}_dashboard_command`.
    pub name: &'static str,
    /// What the button says.
    pub label: &'static str,
    pub group: DashboardGroup,
    pub arg: DashboardArg,
    /// A query: there is no fixed reply to match, so the controller's answer
    /// lands in `{robot_id}_dashboard_request_result` and *is* the result.
    pub query: bool,
    /// The manual marks this one "Only Remote Control". The driver does not
    /// pre-reject it - the controller's own refusal is the truthful answer - so
    /// this only explains a failure the operator is already looking at.
    pub remote_control: bool,
    /// Loses state or takes the robot down. Drawn in red, never in a quick row.
    pub danger: bool,
    /// What goes on the socket to port 29999, for the tooltip. Commands with an
    /// argument append it, so this is the prefix.
    pub wire: &'static str,
    pub help: &'static str,
}

impl DashboardCommandSpec {
    pub fn takes_arg(&self) -> bool {
        self.arg.takes_arg()
    }

    /// Whether `arg` is enough to send this command.
    pub fn arg_is_satisfied(&self, arg: &str) -> bool {
        !self.arg.is_required() || !arg.trim().is_empty()
    }
}

/// Every dashboard command the driver accepts, mirroring
/// `ur_redis_driver::DashboardCommand::parse`.
///
/// `quit` is deliberately absent from both: it would close the socket the driver
/// keeps open for the life of the process.
pub const DASHBOARD_COMMANDS: &[DashboardCommandSpec] = &[
    // ---- program control ---------------------------------------------------
    DashboardCommandSpec {
        name: "stop",
        label: "Stop",
        group: DashboardGroup::Program,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "stop",
        help: "Stop the running program. This is also what a cancelled goal sends.",
    },
    DashboardCommandSpec {
        name: "pause",
        label: "Pause",
        group: DashboardGroup::Program,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "pause",
        help: "Pause motion and latch it: the driver confirms PAUSED and then holds off new \
               moves until a resume. Reported back as 'motion paused'.",
    },
    DashboardCommandSpec {
        name: "resume",
        label: "Resume",
        group: DashboardGroup::Program,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "play",
        help: "Release the pause. Sends 'play' and confirms it; when the controller will not \
               resume an injected script, the live goal is re-issued instead. Which path ran \
               is in the reply. Fails with 'motion is not paused' if nothing is paused.",
    },
    DashboardCommandSpec {
        name: "play",
        label: "Play",
        group: DashboardGroup::Program,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "play",
        help: "The bare 'play' primitive - starts the loaded pendant program and does not \
               touch the driver's pause latch. Use Resume to release a pause.",
    },
    // ---- power -------------------------------------------------------------
    DashboardCommandSpec {
        name: "power_on",
        label: "Power on",
        group: DashboardGroup::Power,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "power on",
        help: "Power the arm on. It stops at IDLE - the brakes still need releasing.",
    },
    DashboardCommandSpec {
        name: "power_off",
        label: "Power off",
        group: DashboardGroup::Power,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: true,
        wire: "power off",
        help: "Power the arm off and engage the brakes.",
    },
    DashboardCommandSpec {
        name: "brake_release",
        label: "Brake release",
        group: DashboardGroup::Power,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "brake release",
        help: "Release the brakes, taking the arm from IDLE to RUNNING. Nothing moves until \
               this has happened.",
    },
    // ---- safety recovery ---------------------------------------------------
    DashboardCommandSpec {
        name: "unlock_protective_stop",
        label: "Unlock protective stop",
        group: DashboardGroup::Safety,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "unlock protective stop",
        help: "Clear a protective stop. The controller refuses for the first 5 s of the stop, \
               so a failure here often just means 'too early, try again'.",
    },
    DashboardCommandSpec {
        name: "close_safety_popup",
        label: "Close safety popup",
        group: DashboardGroup::Safety,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: false,
        wire: "close safety popup",
        help: "Dismiss the safety popup. A safety popup blocks everything else, so this \
               usually comes before the rest of a recovery.",
    },
    DashboardCommandSpec {
        name: "close_popup",
        label: "Close popup",
        group: DashboardGroup::Safety,
        arg: DashboardArg::None,
        query: false,
        remote_control: false,
        danger: false,
        wire: "close popup",
        help: "Dismiss an ordinary popup, including one this GUI put up.",
    },
    DashboardCommandSpec {
        name: "restart_safety",
        label: "Restart safety",
        group: DashboardGroup::Safety,
        arg: DashboardArg::None,
        query: false,
        remote_control: true,
        danger: true,
        wire: "restart safety",
        help: "Restart the safety system after a violation or a fault. The robot powers down \
               and has to be powered on and brake-released again afterwards.",
    },
    // ---- operational mode --------------------------------------------------
    DashboardCommandSpec {
        name: "set_operational_mode",
        label: "Set operational mode",
        group: DashboardGroup::Mode,
        arg: DashboardArg::Choice { options: &["manual", "automatic"], required: true },
        query: false,
        remote_control: false,
        danger: false,
        wire: "set operational mode",
        help: "Override the operational mode from here, ignoring the pendant's mode selector \
               until it is cleared. Needs a mode password set in Settings.",
    },
    DashboardCommandSpec {
        name: "get_operational_mode",
        label: "Get operational mode",
        group: DashboardGroup::Mode,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "get operational mode",
        help: "MANUAL, AUTOMATIC, or NONE. NONE means no mode password has been set, which is \
               not an error - the mode is simply not in use on this robot.",
    },
    DashboardCommandSpec {
        name: "clear_operational_mode",
        label: "Clear operational mode",
        group: DashboardGroup::Mode,
        arg: DashboardArg::None,
        query: false,
        remote_control: false,
        danger: false,
        wire: "clear operational mode",
        help: "Hand the operational mode back to the pendant's selector.",
    },
    // ---- load --------------------------------------------------------------
    DashboardCommandSpec {
        name: "load",
        label: "Load program",
        group: DashboardGroup::Load,
        arg: DashboardArg::Text { hint: "program.urp", required: true },
        query: false,
        remote_control: true,
        danger: false,
        wire: "load",
        help: "Load a program file. The controller does not answer until the program and its \
               installation have finished loading, so the driver allows 30 s for this one.",
    },
    DashboardCommandSpec {
        name: "load_installation",
        label: "Load installation",
        group: DashboardGroup::Load,
        arg: DashboardArg::Text { hint: "default.installation", required: true },
        query: false,
        remote_control: true,
        danger: false,
        wire: "load installation",
        help: "Load an installation file. Also allowed 30 s.",
    },
    // ---- queries -----------------------------------------------------------
    DashboardCommandSpec {
        name: "robot_mode",
        label: "Robot mode",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "robotmode",
        help: "The power/boot state of the arm: POWER_OFF, IDLE, RUNNING and so on. The same \
               value the realtime stream publishes to {robot}_robot_mode.",
    },
    DashboardCommandSpec {
        name: "safety_status",
        label: "Safety status",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "safetystatus",
        help: "Finer-grained than safety mode: it distinguishes the kinds of safeguard stop \
               (AUTOMATIC_MODE_SAFEGUARD_STOP, SYSTEM_THREE_POSITION_ENABLING_STOP). \
               Dashboard-only - the realtime stream has no equivalent field.",
    },
    DashboardCommandSpec {
        name: "safety_mode",
        label: "Safety mode",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "safetymode",
        help: "Deprecated by UR in favour of safety status, but it is what the realtime stream \
               reports, so it is the query that cross-checks {robot}_safety_mode.",
    },
    DashboardCommandSpec {
        name: "program_state",
        label: "Program state",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "programState",
        help: "STOPPED, PLAYING or PAUSED, plus the program name. Describes the loaded pendant \
               program, so it stays STOPPED while an injected script runs.",
    },
    DashboardCommandSpec {
        name: "is_program_running",
        label: "Program running?",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "running",
        help: "Whether a program is executing. Unlike program state, this does track the \
               driver's own injected scripts.",
    },
    DashboardCommandSpec {
        name: "is_program_saved",
        label: "Program saved?",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "isProgramSaved",
        help: "Whether the loaded program has unsaved changes. The reply carries the program \
               name after the flag.",
    },
    DashboardCommandSpec {
        name: "is_in_remote_control",
        label: "Remote control?",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "is in remote control",
        help: "Whether the robot is in Remote Control. Local control is the single most common \
               reason a dashboard action is refused.",
    },
    DashboardCommandSpec {
        name: "get_loaded_program",
        label: "Loaded program",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "get loaded program",
        help: "The path of the loaded program, or 'No program loaded'.",
    },
    DashboardCommandSpec {
        name: "get_robot_model",
        label: "Robot model",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "get robot model",
        help: "UR5, UR10e and so on. Also published to {robot}_robot_model once per dashboard \
               connection, so the strip above already shows it.",
    },
    DashboardCommandSpec {
        name: "get_serial_number",
        label: "Serial number",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "get serial number",
        help: "The controller's serial number. Also published to {robot}_serial_number.",
    },
    DashboardCommandSpec {
        name: "polyscope_version",
        label: "PolyScope version",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "PolyscopeVersion",
        help: "The full PolyScope version string. Also published to \
               {robot}_polyscope_version.",
    },
    DashboardCommandSpec {
        name: "version",
        label: "Version",
        group: DashboardGroup::Query,
        arg: DashboardArg::None,
        query: true,
        remote_control: false,
        danger: false,
        wire: "version",
        help: "The short version number. PolyScope 5.13.0 and later only - older controllers \
               answer with an error.",
    },
    // ---- misc --------------------------------------------------------------
    DashboardCommandSpec {
        name: "popup",
        label: "Show popup",
        group: DashboardGroup::Misc,
        arg: DashboardArg::Text { hint: "message", required: false },
        query: false,
        remote_control: false,
        danger: false,
        wire: "popup",
        help: "Put a popup on the pendant. It blocks the operator until dismissed there or by \
               Close popup.",
    },
    DashboardCommandSpec {
        name: "add_to_log",
        label: "Add to log",
        group: DashboardGroup::Misc,
        arg: DashboardArg::Text { hint: "message", required: false },
        query: false,
        remote_control: false,
        danger: false,
        wire: "addToLog",
        help: "Write a line into the pendant's log, without interrupting anyone.",
    },
    DashboardCommandSpec {
        name: "shutdown",
        label: "Shut down",
        group: DashboardGroup::Misc,
        arg: DashboardArg::None,
        query: false,
        remote_control: false,
        danger: true,
        wire: "shutdown",
        help: "Shut the controller down. Nothing here can bring it back - somebody has to \
               press the power button on the control box.",
    },
    // ---- diagnostics -------------------------------------------------------
    DashboardCommandSpec {
        name: "generate_flight_report",
        label: "Generate flight report",
        group: DashboardGroup::Diagnostics,
        arg: DashboardArg::Choice {
            options: &["system", "controller", "software"],
            required: false,
        },
        query: true,
        remote_control: false,
        danger: false,
        wire: "generate flight report",
        help: "Produce a flight report and answer with its id. The manual allows this a few \
               minutes; the driver waits up to 5. Defaults to 'system'.",
    },
    DashboardCommandSpec {
        name: "generate_support_file",
        label: "Generate support file",
        group: DashboardGroup::Diagnostics,
        arg: DashboardArg::Text { hint: "directory under /programs", required: true },
        query: false,
        remote_control: false,
        danger: false,
        wire: "generate support file",
        help: "Bundle the support file into a directory inside the programs directory. The \
               manual says up to 10 minutes, and the driver waits that long.",
    },
];

/// Names the driver still accepts that are not offered as commands of their own.
///
/// `reset_protective_stop` is what this GUI and the old native one wrote before
/// the driver grew the full command set; the driver kept it as an alias, so the
/// server resolves it rather than rejecting a request that would have worked.
pub const DASHBOARD_ALIASES: &[(&str, &str)] = &[
    ("reset_protective_stop", "unlock_protective_stop"),
];

/// Look up a command by the name written to `{robot_id}_dashboard_command`,
/// resolving [`DASHBOARD_ALIASES`].
pub fn dashboard_spec(name: &str) -> Option<&'static DashboardCommandSpec> {
    let name = name.trim();
    let name = DASHBOARD_ALIASES
        .iter()
        .find(|(alias, _)| *alias == name)
        .map_or(name, |(_, target)| *target);
    DASHBOARD_COMMANDS.iter().find(|spec| spec.name == name)
}

/// The commands in one group, in table order.
pub fn dashboard_commands_in(group: DashboardGroup) -> impl Iterator<Item = &'static DashboardCommandSpec> {
    DASHBOARD_COMMANDS.iter().filter(move |spec| spec.group == group)
}

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
    /// MANUAL, AUTOMATIC or NONE. NONE is the answer when no mode password has
    /// been set, which means the mode is not in use rather than that the query
    /// failed.
    pub operational_mode: String,
    /// The driver's own pause latch, set by the `pause` dashboard command and
    /// cleared by `resume`. While it is set the driver holds off new motion, so
    /// this is not the same fact as `program_state == "PAUSED"`.
    pub motion_paused: bool,
    /// Read once per dashboard connection, and blanked while that socket is
    /// down rather than left stale.
    pub robot_model: String,
    pub serial_number: String,
    pub polyscope_version: String,
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

    /// The dashboard handshake failed, so the last command did not take effect.
    pub fn dashboard_failed(&self) -> bool {
        self.dashboard_request_state == "failed"
    }

    /// Whether a `resume` would be accepted: the driver refuses one with
    /// "motion is not paused" when its latch is clear.
    pub fn can_resume(&self) -> bool {
        self.motion_paused
    }

    /// Safety modes in which the driver will accept a goal at all.
    pub fn safety_accepts_goal(&self) -> bool {
        matches!(self.safety_mode.as_str(), "NORMAL" | "REDUCED")
    }
}
