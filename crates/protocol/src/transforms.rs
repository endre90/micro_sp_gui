//! Transform tree snapshots and the commands that mutate it.

use crate::value::{GuiTransform, GuiTransformStamped};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct TransformsSnapshot {
    pub revision: u64,
    /// Every frame, keyed in Redis by its own `child_frame_id`.
    pub frames: Vec<GuiTransformStamped>,
    /// `micro_sp::update_tree_visualization_once` output, kept as a copyable
    /// fallback view next to the interactive tree.
    pub tree_text: String,
    /// `micro_sp::is_cyclic_all` - a cyclic tree makes every lookup fail, so it
    /// is worth shouting about.
    pub cyclic: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum TransformCommand {
    /// Change a frame's pose, keeping its parent.
    Move { frame: String, transform: GuiTransform },
    /// Change a frame's parent, recomputing the pose so it does not move.
    Reparent { parent: String, child: String },
    /// Change a frame's parent and put it at the parent's origin.
    SnapToParent { parent: String, child: String },
    Insert { frames: Vec<GuiTransformStamped> },
    Remove { frame: String },
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct LookupRequest {
    pub parent: String,
    pub child: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct LookupResponse {
    pub result: Option<GuiTransformStamped>,
    pub error: Option<String>,
}

/// Write a frame out as a scenario file, the way the old native Lookup tab did
/// with a file dialog. The extra fields become the frame's metadata, matching
/// what `ur_redis_driver` and the scenario loader expect.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ExportFrameRequest {
    pub frame: GuiTransformStamped,
    pub filename: String,
    pub tcp_id: String,
    /// Six joint values, stored as `j0..j5`.
    pub joints: Vec<f64>,
    pub gantry: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ExportFrameResponse {
    pub path: Option<String>,
    pub error: Option<String>,
    /// The JSON that was written, so the UI can show it even when no
    /// `--frames-dir` is configured.
    pub json: String,
}

/// Frames that `ur_redis_driver`'s state publisher owns and reasserts on every
/// tick. Moving or reparenting one of these from the GUI will not stick, so the
/// UI warns instead of letting the operator think it worked.
pub const DRIVER_OWNED_FRAMES: &[&str] = &[
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "flange",
    "tool0",
    "ft_frame",
];

/// `true` for a driver-published frame (including the `_visual` children).
pub fn is_driver_owned(frame: &str) -> bool {
    DRIVER_OWNED_FRAMES.contains(&frame) || frame.ends_with("_visual")
}
