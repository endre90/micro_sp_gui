//! A mirror of `micro_sp::SPValue`.
//!
//! The frontend runs in a browser, so it cannot depend on `micro_sp` (that crate
//! pulls in `tokio` with `full` and `redis` with `tokio-comp`, neither of which
//! builds for `wasm32-unknown-unknown`). Instead these types reproduce
//! `SPValue`'s **serde representation exactly**, so a `GuiValue` deserialises
//! straight out of the JSON string Redis holds and serialises back into
//! something `micro_sp` accepts.
//!
//! Concretely, `SPValue` is `#[serde(tag = "type", content = "value")]` over
//! payload enums that are externally tagged, which produces a doubly-nested
//! shape:
//!
//! ```text
//! {"type":"Bool","value":{"Bool":true}}
//! {"type":"Bool","value":"UNKNOWN"}
//! {"type":"Time","value":{"Time":{"secs_since_epoch":1,"nanos_since_epoch":2}}}
//! ```
//!
//! `crates/server/tests/mirror.rs` asserts byte-identical round trips against
//! the real `SPValue` for every variant. If that test fails, `micro_sp` changed
//! and this file has to follow.

use serde::{Deserialize, Serialize};
use std::fmt;

/// `std::time::SystemTime`'s serde shape.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct GuiSystemTime {
    pub secs_since_epoch: u64,
    pub nanos_since_epoch: u32,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum BoolOrUnknown {
    Bool(bool),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum FloatOrUnknown {
    // `micro_sp` uses `OrderedFloat<f64>`, which is serde-transparent.
    Float64(f64),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum IntOrUnknown {
    Int64(i64),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum StringOrUnknown {
    String(String),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum TimeOrUnknown {
    Time(GuiSystemTime),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum ArrayOrUnknown {
    Array(Vec<GuiValue>),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum MapOrUnknown {
    /// A map in insertion order. `micro_sp` uses a `Vec` of pairs, not a
    /// `HashMap`, and keys are themselves values - keep it that way.
    Map(Vec<(GuiValue, GuiValue)>),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum TransformOrUnknown {
    Transform(GuiTransformStamped),
    UNKNOWN,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Default)]
pub struct GuiTranslation {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct GuiRotation {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for GuiRotation {
    /// The identity quaternion - `w = 1`, not all zeroes.
    fn default() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Default)]
pub struct GuiTransform {
    pub translation: GuiTranslation,
    pub rotation: GuiRotation,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct GuiTransformStamped {
    pub active_transform: bool,
    pub enable_transform: bool,
    pub time_stamp: GuiSystemTime,
    pub parent_frame_id: String,
    pub child_frame_id: String,
    pub transform: GuiTransform,
    pub metadata: MapOrUnknown,
}

impl Default for GuiTransformStamped {
    fn default() -> Self {
        Self::identity("world", "")
    }
}

impl GuiTransformStamped {
    /// A frame at the origin of `parent`, stamped at the epoch.
    ///
    /// The timestamp is left at zero on purpose: the server stamps frames it
    /// writes with the real clock, and wasm has no business inventing one.
    pub fn identity(parent: &str, child: &str) -> Self {
        Self {
            active_transform: true,
            enable_transform: true,
            time_stamp: GuiSystemTime { secs_since_epoch: 0, nanos_since_epoch: 0 },
            parent_frame_id: parent.to_string(),
            child_frame_id: child.to_string(),
            transform: GuiTransform::default(),
            metadata: MapOrUnknown::UNKNOWN,
        }
    }
}

/// The mirror of `micro_sp::SPValue`.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(tag = "type", content = "value")]
pub enum GuiValue {
    Bool(BoolOrUnknown),
    Float64(FloatOrUnknown),
    Int64(IntOrUnknown),
    String(StringOrUnknown),
    Time(TimeOrUnknown),
    Array(ArrayOrUnknown),
    Map(MapOrUnknown),
    Transform(TransformOrUnknown),
}

/// The mirror of `micro_sp::SPValueType`.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GuiValueType {
    Bool,
    Float64,
    Int64,
    String,
    Time,
    Array,
    Map,
    Transform,
}

impl GuiValueType {
    pub const ALL: &'static [GuiValueType] = &[
        GuiValueType::Bool,
        GuiValueType::Int64,
        GuiValueType::Float64,
        GuiValueType::String,
        GuiValueType::Time,
        GuiValueType::Array,
        GuiValueType::Map,
        GuiValueType::Transform,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            GuiValueType::Bool => "Bool",
            GuiValueType::Float64 => "Float64",
            GuiValueType::Int64 => "Int64",
            GuiValueType::String => "String",
            GuiValueType::Time => "Time",
            GuiValueType::Array => "Array",
            GuiValueType::Map => "Map",
            GuiValueType::Transform => "Transform",
        }
    }
}

impl fmt::Display for GuiValueType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

impl GuiValue {
    pub fn type_of(&self) -> GuiValueType {
        match self {
            GuiValue::Bool(_) => GuiValueType::Bool,
            GuiValue::Float64(_) => GuiValueType::Float64,
            GuiValue::Int64(_) => GuiValueType::Int64,
            GuiValue::String(_) => GuiValueType::String,
            GuiValue::Time(_) => GuiValueType::Time,
            GuiValue::Array(_) => GuiValueType::Array,
            GuiValue::Map(_) => GuiValueType::Map,
            GuiValue::Transform(_) => GuiValueType::Transform,
        }
    }

    pub fn is_unknown(&self) -> bool {
        match self {
            GuiValue::Bool(v) => matches!(v, BoolOrUnknown::UNKNOWN),
            GuiValue::Float64(v) => matches!(v, FloatOrUnknown::UNKNOWN),
            GuiValue::Int64(v) => matches!(v, IntOrUnknown::UNKNOWN),
            GuiValue::String(v) => matches!(v, StringOrUnknown::UNKNOWN),
            GuiValue::Time(v) => matches!(v, TimeOrUnknown::UNKNOWN),
            GuiValue::Array(v) => matches!(v, ArrayOrUnknown::UNKNOWN),
            GuiValue::Map(v) => matches!(v, MapOrUnknown::UNKNOWN),
            GuiValue::Transform(v) => matches!(v, TransformOrUnknown::UNKNOWN),
        }
    }

    /// A typed `UNKNOWN` - the state every variable starts in.
    pub fn unknown_of(t: GuiValueType) -> GuiValue {
        match t {
            GuiValueType::Bool => GuiValue::Bool(BoolOrUnknown::UNKNOWN),
            GuiValueType::Float64 => GuiValue::Float64(FloatOrUnknown::UNKNOWN),
            GuiValueType::Int64 => GuiValue::Int64(IntOrUnknown::UNKNOWN),
            GuiValueType::String => GuiValue::String(StringOrUnknown::UNKNOWN),
            GuiValueType::Time => GuiValue::Time(TimeOrUnknown::UNKNOWN),
            GuiValueType::Array => GuiValue::Array(ArrayOrUnknown::UNKNOWN),
            GuiValueType::Map => GuiValue::Map(MapOrUnknown::UNKNOWN),
            GuiValueType::Transform => GuiValue::Transform(TransformOrUnknown::UNKNOWN),
        }
    }

    /// A known, empty/zero value of the given type - what a "new value" button
    /// should hand the editor.
    pub fn default_for(t: GuiValueType) -> GuiValue {
        match t {
            GuiValueType::Bool => GuiValue::Bool(BoolOrUnknown::Bool(false)),
            GuiValueType::Float64 => GuiValue::Float64(FloatOrUnknown::Float64(0.0)),
            GuiValueType::Int64 => GuiValue::Int64(IntOrUnknown::Int64(0)),
            GuiValueType::String => GuiValue::String(StringOrUnknown::String(String::new())),
            GuiValueType::Time => GuiValue::Time(TimeOrUnknown::UNKNOWN),
            GuiValueType::Array => GuiValue::Array(ArrayOrUnknown::Array(Vec::new())),
            GuiValueType::Map => GuiValue::Map(MapOrUnknown::Map(Vec::new())),
            GuiValueType::Transform => {
                GuiValue::Transform(TransformOrUnknown::Transform(GuiTransformStamped::identity(
                    "world", "new_frame",
                )))
            }
        }
    }

    /// A one-line rendering for the state table. Deliberately short: arrays and
    /// maps report their size rather than their contents, and a transform
    /// reports the edge it describes.
    pub fn display(&self) -> String {
        match self {
            GuiValue::Bool(BoolOrUnknown::Bool(b)) => b.to_string(),
            GuiValue::Float64(FloatOrUnknown::Float64(f)) => f.to_string(),
            GuiValue::Int64(IntOrUnknown::Int64(i)) => i.to_string(),
            GuiValue::String(StringOrUnknown::String(s)) => format!("\"{s}\""),
            GuiValue::Time(TimeOrUnknown::Time(t)) => {
                format!("{}.{:09}", t.secs_since_epoch, t.nanos_since_epoch)
            }
            GuiValue::Array(ArrayOrUnknown::Array(items)) => format!("[{} items]", items.len()),
            GuiValue::Map(MapOrUnknown::Map(pairs)) => format!("[{} pairs]", pairs.len()),
            GuiValue::Transform(TransformOrUnknown::Transform(tf)) => format!(
                "{} -> {}{}",
                tf.parent_frame_id,
                tf.child_frame_id,
                if tf.active_transform { "" } else { " (inactive)" }
            ),
            _ => "UNKNOWN".to_string(),
        }
    }

    /// `true` when the rendered value or a nested one contains `needle`
    /// (case-insensitively). Used by the state filter so searching finds a
    /// string buried in an array.
    pub fn contains_text(&self, needle_lower: &str) -> bool {
        match self {
            GuiValue::Array(ArrayOrUnknown::Array(items)) => {
                items.iter().any(|i| i.contains_text(needle_lower))
            }
            GuiValue::Map(MapOrUnknown::Map(pairs)) => pairs
                .iter()
                .any(|(k, v)| k.contains_text(needle_lower) || v.contains_text(needle_lower)),
            other => other.display().to_lowercase().contains(needle_lower),
        }
    }
}
