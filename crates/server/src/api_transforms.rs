//! Reading and mutating the transform tree.
//!
//! These call `TransformsManager` directly rather than going through the
//! `{sp_id}_tf_*` interface variables, for two reasons: the GUI has to work with
//! no micro_sp runner running at all, and the direct calls already refuse to
//! introduce a cycle.

use crate::{LOG_TARGET, convert};
use micro_sp::*;
use micro_sp_gui_protocol as proto;
use serde::Serialize;
use std::sync::Arc;

pub async fn command(
    cm: &Arc<ConnectionManager>,
    cmd: &proto::TransformCommand,
) -> Result<(), String> {
    let mut con = cm.get_connection().await;
    let result = match cmd {
        proto::TransformCommand::Move { frame, transform } => {
            TransformsManager::move_transform(
                &mut con,
                frame,
                convert::transform_from_proto(transform),
            )
            .await
        }
        proto::TransformCommand::Reparent { parent, child } => {
            TransformsManager::reparent_transform(&mut con, parent, child).await
        }
        proto::TransformCommand::SnapToParent { parent, child } => {
            TransformsManager::snap_to_parent_transform(&mut con, parent, child).await
        }
        proto::TransformCommand::Insert { frames } => {
            let frames: Vec<SPTransformStamped> = frames
                .iter()
                .map(|f| {
                    let mut tf = convert::stamped_from_proto(f);
                    // The browser has no trustworthy clock, and an epoch stamp
                    // on a live frame is misleading.
                    tf.time_stamp = std::time::SystemTime::now();
                    tf
                })
                .collect();
            TransformsManager::insert_transforms(&mut con, &frames).await
        }
        proto::TransformCommand::Remove { frame } => {
            TransformsManager::remove_transform(&mut con, frame).await
        }
    };

    match result {
        Ok(()) => {
            log::info!(target: LOG_TARGET, "Transform command applied: {cmd:?}");
            Ok(())
        }
        // These errors are worth showing verbatim: "would introduce a cycle" and
        // "no such frame" are exactly what the operator needs to read.
        Err(e) => Err(e.to_string()),
    }
}

pub async fn lookup(
    cm: &Arc<ConnectionManager>,
    req: &proto::LookupRequest,
) -> proto::LookupResponse {
    if req.parent.trim().is_empty() || req.child.trim().is_empty() {
        return proto::LookupResponse {
            result: None,
            error: Some("Pick both a parent and a child frame.".to_string()),
        };
    }
    let mut con = cm.get_connection().await;
    match TransformsManager::lookup_transform(&mut con, &req.parent, &req.child).await {
        Ok(tf) => {
            proto::LookupResponse { result: Some(convert::stamped_to_proto(&tf)), error: None }
        }
        Err(e) => proto::LookupResponse { result: None, error: Some(e.to_string()) },
    }
}

// The scenario-file shape the transform loader and `ur_redis_driver` expect.
// Same output the old native Lookup tab wrote through a file dialog, which is
// not available in a browser.

#[derive(Serialize)]
struct ExportMetadata {
    tcp_id: String,
    preferred_joint_configuration: std::collections::BTreeMap<String, f64>,
    enable_transform: bool,
    active_transform: bool,
    gantry: f64,
}

#[derive(Serialize)]
struct ExportedFrame {
    child_frame_id: String,
    parent_frame_id: String,
    transform: proto::GuiTransform,
    metadata: ExportMetadata,
}

/// `j0..j5`, as the driver's `preferred_joint_config` metadata is keyed.
fn joint_map(joints: &[f64]) -> std::collections::BTreeMap<String, f64> {
    joints.iter().enumerate().map(|(i, v)| (format!("j{i}"), *v)).collect()
}

/// Reject anything that could escape the frames directory.
fn safe_filename(raw: &str) -> Result<String, String> {
    let name = raw.trim();
    if name.is_empty() {
        return Err("Filename is empty.".to_string());
    }
    if name.contains('/') || name.contains('\\') || name.contains("..") {
        return Err("Filename must not contain a path.".to_string());
    }
    Ok(if name.ends_with(".json") { name.to_string() } else { format!("{name}.json") })
}

/// Render the frame as scenario JSON and, when a frames directory is
/// configured, write it there. The JSON comes back either way so the tab can
/// show it even with no directory set.
pub async fn export(
    frames_dir: Option<&std::path::Path>,
    req: &proto::ExportFrameRequest,
) -> proto::ExportFrameResponse {
    let exported = ExportedFrame {
        child_frame_id: req.frame.child_frame_id.clone(),
        parent_frame_id: req.frame.parent_frame_id.clone(),
        transform: req.frame.transform,
        metadata: ExportMetadata {
            tcp_id: req.tcp_id.clone(),
            preferred_joint_configuration: joint_map(&req.joints),
            enable_transform: req.frame.enable_transform,
            active_transform: req.frame.active_transform,
            gantry: req.gantry,
        },
    };

    let json = match serde_json::to_string_pretty(&exported) {
        Ok(json) => json,
        Err(e) => {
            return proto::ExportFrameResponse {
                path: None,
                error: Some(format!("Could not render the frame: {e}")),
                json: String::new(),
            };
        }
    };

    let Some(dir) = frames_dir else {
        return proto::ExportFrameResponse {
            path: None,
            error: Some(
                "No --frames-dir configured, so nothing was saved. The JSON is shown here."
                    .to_string(),
            ),
            json,
        };
    };

    let filename = match safe_filename(&req.filename) {
        Ok(name) => name,
        Err(e) => return proto::ExportFrameResponse { path: None, error: Some(e), json },
    };

    if let Err(e) = tokio::fs::create_dir_all(dir).await {
        return proto::ExportFrameResponse {
            path: None,
            error: Some(format!("Could not create {}: {e}", dir.display())),
            json,
        };
    }

    let path = dir.join(&filename);
    match tokio::fs::write(&path, json.as_bytes()).await {
        Ok(()) => {
            log::info!(target: LOG_TARGET, "Exported frame to {}", path.display());
            proto::ExportFrameResponse {
                path: Some(path.display().to_string()),
                error: None,
                json,
            }
        }
        Err(e) => proto::ExportFrameResponse {
            path: None,
            error: Some(format!("Could not write {}: {e}", path.display())),
            json,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filenames_cannot_escape_the_frames_directory() {
        assert!(safe_filename("../../etc/passwd").is_err());
        assert!(safe_filename("a/b.json").is_err());
        assert!(safe_filename("").is_err());
        assert_eq!(safe_filename("world_to_tcp").unwrap(), "world_to_tcp.json");
        assert_eq!(safe_filename(" frame.json ").unwrap(), "frame.json");
    }

    #[test]
    fn joints_are_keyed_j0_to_j5() {
        let map = joint_map(&[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
        assert_eq!(map.len(), 6);
        assert_eq!(map["j0"], 0.1);
        assert_eq!(map["j5"], 0.6);
    }
}
