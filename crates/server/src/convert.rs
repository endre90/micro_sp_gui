//! `SPValue` <-> `GuiValue` conversion.
//!
//! The two types have identical serde shapes (see `micro_sp_gui_protocol::value`),
//! so these could be done by round-tripping through JSON. They are written out
//! explicitly instead, because that way a change in `micro_sp` breaks the build
//! here rather than silently producing values the runtime will refuse.

use micro_sp::*;
use micro_sp_gui_protocol as proto;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub fn time_to_proto(t: SystemTime) -> proto::GuiSystemTime {
    // A pre-epoch stamp is not something micro_sp produces; clamping it is
    // better than refusing to display the frame that carries it.
    let d = t.duration_since(UNIX_EPOCH).unwrap_or_default();
    proto::GuiSystemTime { secs_since_epoch: d.as_secs(), nanos_since_epoch: d.subsec_nanos() }
}

pub fn time_from_proto(t: proto::GuiSystemTime) -> SystemTime {
    UNIX_EPOCH + Duration::new(t.secs_since_epoch, t.nanos_since_epoch)
}

pub fn transform_to_proto(tf: &SPTransform) -> proto::GuiTransform {
    proto::GuiTransform {
        translation: proto::GuiTranslation {
            x: tf.translation.x.into_inner(),
            y: tf.translation.y.into_inner(),
            z: tf.translation.z.into_inner(),
        },
        rotation: proto::GuiRotation {
            x: tf.rotation.x.into_inner(),
            y: tf.rotation.y.into_inner(),
            z: tf.rotation.z.into_inner(),
            w: tf.rotation.w.into_inner(),
        },
    }
}

pub fn transform_from_proto(tf: &proto::GuiTransform) -> SPTransform {
    SPTransform {
        translation: SPTranslation {
            x: tf.translation.x.into(),
            y: tf.translation.y.into(),
            z: tf.translation.z.into(),
        },
        rotation: SPRotation {
            x: tf.rotation.x.into(),
            y: tf.rotation.y.into(),
            z: tf.rotation.z.into(),
            w: tf.rotation.w.into(),
        },
    }
}

pub fn stamped_to_proto(tf: &SPTransformStamped) -> proto::GuiTransformStamped {
    proto::GuiTransformStamped {
        active_transform: tf.active_transform,
        enable_transform: tf.enable_transform,
        time_stamp: time_to_proto(tf.time_stamp),
        parent_frame_id: tf.parent_frame_id.clone(),
        child_frame_id: tf.child_frame_id.clone(),
        transform: transform_to_proto(&tf.transform),
        metadata: map_to_proto(&tf.metadata),
    }
}

pub fn stamped_from_proto(tf: &proto::GuiTransformStamped) -> SPTransformStamped {
    SPTransformStamped {
        active_transform: tf.active_transform,
        enable_transform: tf.enable_transform,
        time_stamp: time_from_proto(tf.time_stamp),
        parent_frame_id: tf.parent_frame_id.clone(),
        child_frame_id: tf.child_frame_id.clone(),
        transform: transform_from_proto(&tf.transform),
        metadata: map_from_proto(&tf.metadata),
    }
}

fn map_to_proto(m: &MapOrUnknown) -> proto::MapOrUnknown {
    match m {
        MapOrUnknown::Map(pairs) => proto::MapOrUnknown::Map(
            pairs.iter().map(|(k, v)| (to_proto(k), to_proto(v))).collect(),
        ),
        MapOrUnknown::UNKNOWN => proto::MapOrUnknown::UNKNOWN,
    }
}

fn map_from_proto(m: &proto::MapOrUnknown) -> MapOrUnknown {
    match m {
        proto::MapOrUnknown::Map(pairs) => MapOrUnknown::Map(
            pairs.iter().map(|(k, v)| (from_proto(k), from_proto(v))).collect(),
        ),
        proto::MapOrUnknown::UNKNOWN => MapOrUnknown::UNKNOWN,
    }
}

pub fn to_proto(v: &SPValue) -> proto::GuiValue {
    match v {
        SPValue::Bool(b) => proto::GuiValue::Bool(match b {
            BoolOrUnknown::Bool(x) => proto::BoolOrUnknown::Bool(*x),
            BoolOrUnknown::UNKNOWN => proto::BoolOrUnknown::UNKNOWN,
        }),
        SPValue::Float64(f) => proto::GuiValue::Float64(match f {
            FloatOrUnknown::Float64(x) => proto::FloatOrUnknown::Float64(x.into_inner()),
            FloatOrUnknown::UNKNOWN => proto::FloatOrUnknown::UNKNOWN,
        }),
        SPValue::Int64(i) => proto::GuiValue::Int64(match i {
            IntOrUnknown::Int64(x) => proto::IntOrUnknown::Int64(*x),
            IntOrUnknown::UNKNOWN => proto::IntOrUnknown::UNKNOWN,
        }),
        SPValue::String(s) => proto::GuiValue::String(match s {
            StringOrUnknown::String(x) => proto::StringOrUnknown::String(x.clone()),
            StringOrUnknown::UNKNOWN => proto::StringOrUnknown::UNKNOWN,
        }),
        SPValue::Time(t) => proto::GuiValue::Time(match t {
            TimeOrUnknown::Time(x) => proto::TimeOrUnknown::Time(time_to_proto(*x)),
            TimeOrUnknown::UNKNOWN => proto::TimeOrUnknown::UNKNOWN,
        }),
        SPValue::Array(a) => proto::GuiValue::Array(match a {
            ArrayOrUnknown::Array(items) => {
                proto::ArrayOrUnknown::Array(items.iter().map(to_proto).collect())
            }
            ArrayOrUnknown::UNKNOWN => proto::ArrayOrUnknown::UNKNOWN,
        }),
        SPValue::Map(m) => proto::GuiValue::Map(map_to_proto(m)),
        SPValue::Transform(t) => proto::GuiValue::Transform(match t {
            TransformOrUnknown::Transform(tf) => {
                proto::TransformOrUnknown::Transform(stamped_to_proto(tf))
            }
            TransformOrUnknown::UNKNOWN => proto::TransformOrUnknown::UNKNOWN,
        }),
    }
}

pub fn from_proto(v: &proto::GuiValue) -> SPValue {
    match v {
        proto::GuiValue::Bool(b) => SPValue::Bool(match b {
            proto::BoolOrUnknown::Bool(x) => BoolOrUnknown::Bool(*x),
            proto::BoolOrUnknown::UNKNOWN => BoolOrUnknown::UNKNOWN,
        }),
        proto::GuiValue::Float64(f) => SPValue::Float64(match f {
            proto::FloatOrUnknown::Float64(x) => FloatOrUnknown::Float64((*x).into()),
            proto::FloatOrUnknown::UNKNOWN => FloatOrUnknown::UNKNOWN,
        }),
        proto::GuiValue::Int64(i) => SPValue::Int64(match i {
            proto::IntOrUnknown::Int64(x) => IntOrUnknown::Int64(*x),
            proto::IntOrUnknown::UNKNOWN => IntOrUnknown::UNKNOWN,
        }),
        proto::GuiValue::String(s) => SPValue::String(match s {
            proto::StringOrUnknown::String(x) => StringOrUnknown::String(x.clone()),
            proto::StringOrUnknown::UNKNOWN => StringOrUnknown::UNKNOWN,
        }),
        proto::GuiValue::Time(t) => SPValue::Time(match t {
            proto::TimeOrUnknown::Time(x) => TimeOrUnknown::Time(time_from_proto(*x)),
            proto::TimeOrUnknown::UNKNOWN => TimeOrUnknown::UNKNOWN,
        }),
        proto::GuiValue::Array(a) => SPValue::Array(match a {
            proto::ArrayOrUnknown::Array(items) => {
                ArrayOrUnknown::Array(items.iter().map(from_proto).collect())
            }
            proto::ArrayOrUnknown::UNKNOWN => ArrayOrUnknown::UNKNOWN,
        }),
        proto::GuiValue::Map(m) => SPValue::Map(map_from_proto(m)),
        proto::GuiValue::Transform(t) => SPValue::Transform(match t {
            proto::TransformOrUnknown::Transform(tf) => {
                TransformOrUnknown::Transform(stamped_from_proto(tf))
            }
            proto::TransformOrUnknown::UNKNOWN => TransformOrUnknown::UNKNOWN,
        }),
    }
}
