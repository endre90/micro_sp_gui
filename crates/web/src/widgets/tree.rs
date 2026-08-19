//! The transform tree, as collapsible headers.
//!
//! Frames are stored flat, keyed by `child_frame_id`, each naming its parent.
//! The roots are therefore the parents that are not themselves anybody's child -
//! `world` in a typical scene, or `base_link`, which `ur_redis_driver`
//! deliberately never publishes because something else has to place it.

use micro_sp_gui_protocol as proto;
use std::collections::{BTreeMap, BTreeSet};

/// Same ceiling micro_sp's own lookup uses, so a cycle cannot hang the UI.
const MAX_DEPTH: usize = 64;

struct Tree<'a> {
    children: BTreeMap<&'a str, Vec<&'a proto::GuiTransformStamped>>,
    roots: Vec<&'a str>,
}

fn build<'a>(frames: &'a [proto::GuiTransformStamped]) -> Tree<'a> {
    let owned: BTreeSet<&str> = frames.iter().map(|f| f.child_frame_id.as_str()).collect();

    let mut children: BTreeMap<&str, Vec<&proto::GuiTransformStamped>> = BTreeMap::new();
    let mut roots: BTreeSet<&str> = BTreeSet::new();
    for frame in frames {
        children.entry(frame.parent_frame_id.as_str()).or_default().push(frame);
        if !owned.contains(frame.parent_frame_id.as_str()) {
            roots.insert(frame.parent_frame_id.as_str());
        }
    }
    for list in children.values_mut() {
        list.sort_by(|a, b| a.child_frame_id.cmp(&b.child_frame_id));
    }

    // Every frame in a cycle is somebody's child, so a purely cyclic component
    // has no root and would otherwise be invisible. Surface it anyway.
    if roots.is_empty() && !frames.is_empty() {
        roots.insert(frames[0].parent_frame_id.as_str());
    }

    Tree { children, roots: roots.into_iter().collect() }
}

/// Draw the tree. Returns `true` if the selection changed.
pub fn transform_tree(
    ui: &mut egui::Ui,
    frames: &[proto::GuiTransformStamped],
    selected: &mut Option<String>,
) -> bool {
    if frames.is_empty() {
        ui.label(egui::RichText::new("No frames in Redis.").weak());
        return false;
    }

    let tree = build(frames);
    let mut changed = false;
    let mut seen = BTreeSet::new();
    for root in &tree.roots {
        changed |= node(ui, &tree, root, None, selected, 0, &mut seen);
    }
    changed
}

/// One frame and its subtree. `frame` is `None` for a root that has no frame of
/// its own (nothing declares `world`'s pose; it just is).
fn node(
    ui: &mut egui::Ui,
    tree: &Tree<'_>,
    name: &str,
    frame: Option<&proto::GuiTransformStamped>,
    selected: &mut Option<String>,
    depth: usize,
    seen: &mut BTreeSet<String>,
) -> bool {
    if depth > MAX_DEPTH || !seen.insert(name.to_string()) {
        ui.label(
            egui::RichText::new(format!("↺ {name} (already shown - the tree has a cycle)"))
                .color(super::BAD),
        );
        return false;
    }

    let mut changed = false;
    let children = tree.children.get(name).cloned().unwrap_or_default();

    let label = {
        let mut text = egui::RichText::new(name).monospace();
        if let Some(frame) = frame {
            if !frame.active_transform {
                text = text.weak();
            }
            if !frame.enable_transform {
                text = text.color(super::UNKNOWN);
            }
        } else {
            // A root with no frame of its own is a reference, not a thing.
            text = text.strong();
        }
        text
    };

    if children.is_empty() {
        ui.horizontal(|ui| {
            // Leaves get the same click target as branches, just no arrow.
            ui.add_space(18.0);
            changed |= select_button(ui, name, label, frame, selected);
        });
    } else {
        let header = egui::CollapsingHeader::new(label)
            .id_salt(("tf_node", name))
            .default_open(depth < 2);
        let response = header.show(ui, |ui| {
            let mut inner = false;
            for child in &children {
                inner |= node(
                    ui,
                    tree,
                    &child.child_frame_id,
                    Some(child),
                    selected,
                    depth + 1,
                    seen,
                );
            }
            inner
        });
        changed |= response.body_returned.unwrap_or(false);
        if response.header_response.clicked() {
            *selected = Some(name.to_string());
            changed = true;
        }
    }

    changed
}

fn select_button(
    ui: &mut egui::Ui,
    name: &str,
    label: egui::RichText,
    frame: Option<&proto::GuiTransformStamped>,
    selected: &mut Option<String>,
) -> bool {
    let is_selected = selected.as_deref() == Some(name);
    let mut response = ui.selectable_label(is_selected, label);
    if let Some(frame) = frame {
        response = response.on_hover_text(format!(
            "parent: {}\ntranslation: {:.4}, {:.4}, {:.4}{}",
            frame.parent_frame_id,
            frame.transform.translation.x,
            frame.transform.translation.y,
            frame.transform.translation.z,
            if proto::is_driver_owned(name) {
                "\n\nPublished by ur_redis_driver, which reasserts it every tick."
            } else {
                ""
            }
        ));
    }
    if response.clicked() {
        *selected = Some(name.to_string());
        return true;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(parent: &str, child: &str) -> proto::GuiTransformStamped {
        proto::GuiTransformStamped::identity(parent, child)
    }

    #[test]
    fn a_parent_without_a_frame_of_its_own_is_the_root() {
        let frames = vec![frame("world", "table"), frame("table", "plate")];
        let tree = build(&frames);
        assert_eq!(tree.roots, vec!["world"]);
        assert_eq!(tree.children["world"].len(), 1);
        assert_eq!(tree.children["table"][0].child_frame_id, "plate");
    }

    #[test]
    fn several_roots_are_all_reported() {
        let frames = vec![frame("world", "a"), frame("other_root", "b")];
        let tree = build(&frames);
        assert_eq!(tree.roots, vec!["other_root", "world"]);
    }

    /// A cycle leaves nothing unparented; the component must still be reachable
    /// or the operator cannot see what to fix.
    #[test]
    fn a_pure_cycle_still_gets_a_root() {
        let frames = vec![frame("b", "a"), frame("a", "b")];
        let tree = build(&frames);
        assert_eq!(tree.roots.len(), 1);
    }

    #[test]
    fn no_frames_means_no_roots() {
        let tree = build(&[]);
        assert!(tree.roots.is_empty());
    }
}
