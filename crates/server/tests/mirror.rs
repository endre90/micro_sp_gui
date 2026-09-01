//! Pins `micro_sp_gui_protocol` to `micro_sp` and `ur_redis_driver`.
//!
//! The frontend cannot depend on either of those crates, so the protocol crate
//! duplicates their types and constants. These tests are the only thing that
//! keeps the duplicates honest: if one fails, the upstream crate changed and
//! `crates/protocol` has to follow.

use micro_sp::*;
use micro_sp_gui_protocol as proto;
use micro_sp_gui_server::convert;
use std::time::{Duration, UNIX_EPOCH};

/// One of every `SPValue` variant, both known and `UNKNOWN`, including nesting.
fn every_variant() -> Vec<SPValue> {
    let stamped = SPTransformStamped {
        active_transform: true,
        enable_transform: false,
        time_stamp: UNIX_EPOCH + Duration::new(1_777_000_000, 123_456_789),
        parent_frame_id: "world".to_string(),
        child_frame_id: "gripper".to_string(),
        transform: SPTransform {
            translation: SPTranslation { x: 1.5.into(), y: (-2.25).into(), z: 0.0.into() },
            // A quarter turn about z. Spelled with the constant because 0.7071
            // trips clippy::approx_constant.
            rotation: SPRotation {
                x: 0.0.into(),
                y: 0.0.into(),
                z: std::f64::consts::FRAC_1_SQRT_2.into(),
                w: std::f64::consts::FRAC_1_SQRT_2.into(),
            },
        },
        metadata: MapOrUnknown::Map(vec![(
            SPValue::String(StringOrUnknown::String("mesh_file".to_string())),
            SPValue::String(StringOrUnknown::String("wrist3.dae".to_string())),
        )]),
    };

    vec![
        SPValue::Bool(BoolOrUnknown::Bool(true)),
        SPValue::Bool(BoolOrUnknown::Bool(false)),
        SPValue::Bool(BoolOrUnknown::UNKNOWN),
        SPValue::Int64(IntOrUnknown::Int64(-42)),
        SPValue::Int64(IntOrUnknown::UNKNOWN),
        SPValue::Float64(FloatOrUnknown::Float64(0.125.into())),
        SPValue::Float64(FloatOrUnknown::UNKNOWN),
        // A literal "UNKNOWN" string is a real value, not the unknown variant.
        SPValue::String(StringOrUnknown::String("hello \"world\"".to_string())),
        SPValue::String(StringOrUnknown::UNKNOWN),
        SPValue::Time(TimeOrUnknown::Time(UNIX_EPOCH + Duration::new(12, 34))),
        SPValue::Time(TimeOrUnknown::UNKNOWN),
        SPValue::Array(ArrayOrUnknown::Array(vec![])),
        SPValue::Array(ArrayOrUnknown::Array(vec![
            SPValue::Int64(IntOrUnknown::Int64(1)),
            SPValue::Bool(BoolOrUnknown::UNKNOWN),
            // Nesting, so the recursive arms are covered too.
            SPValue::Array(ArrayOrUnknown::Array(vec![SPValue::Float64(
                FloatOrUnknown::Float64(9.0.into()),
            )])),
        ])),
        SPValue::Array(ArrayOrUnknown::UNKNOWN),
        SPValue::Map(MapOrUnknown::Map(vec![])),
        SPValue::Map(MapOrUnknown::Map(vec![
            (
                SPValue::String(StringOrUnknown::String("k".to_string())),
                SPValue::Int64(IntOrUnknown::Int64(7)),
            ),
            (
                SPValue::Int64(IntOrUnknown::Int64(0)),
                SPValue::Map(MapOrUnknown::Map(vec![(
                    SPValue::String(StringOrUnknown::String("deep".to_string())),
                    SPValue::Bool(BoolOrUnknown::Bool(true)),
                )])),
            ),
        ])),
        SPValue::Map(MapOrUnknown::UNKNOWN),
        // An array of maps: the shape a table of records comes in, and the one
        // combination the two nesting cases above do not reach - an Array arm
        // recursing into a Map arm. The Production tab renders exactly this.
        SPValue::Array(ArrayOrUnknown::Array(vec![
            SPValue::Map(MapOrUnknown::Map(vec![
                (
                    SPValue::String(StringOrUnknown::String("id".to_string())),
                    SPValue::Int64(IntOrUnknown::Int64(1)),
                ),
                (
                    SPValue::String(StringOrUnknown::String("at".to_string())),
                    SPValue::Time(TimeOrUnknown::Time(UNIX_EPOCH + Duration::new(7, 8))),
                ),
                (
                    SPValue::String(StringOrUnknown::String("ok".to_string())),
                    SPValue::Bool(BoolOrUnknown::Bool(true)),
                ),
            ])),
            SPValue::Map(MapOrUnknown::Map(vec![])),
        ])),
        SPValue::Transform(TransformOrUnknown::Transform(stamped)),
        SPValue::Transform(TransformOrUnknown::UNKNOWN),
    ]
}

/// The load-bearing test: the JSON `micro_sp` writes to Redis must deserialise
/// into a `GuiValue` and serialise back byte-identically.
#[test]
fn gui_value_json_is_identical_to_sp_value_json() {
    for original in every_variant() {
        let sp_json = serde_json::to_string(&original).expect("SPValue serialises");

        let gui: proto::GuiValue = serde_json::from_str(&sp_json)
            .unwrap_or_else(|e| panic!("GuiValue cannot read {sp_json}: {e}"));

        let gui_json = serde_json::to_string(&gui).expect("GuiValue serialises");
        assert_eq!(
            sp_json, gui_json,
            "the mirror drifted: micro_sp writes {sp_json} but the protocol crate writes {gui_json}"
        );

        // And a GuiValue built in the browser must read back as the same SPValue.
        let back: SPValue = serde_json::from_str(&gui_json).expect("SPValue reads the mirror");
        assert_eq!(original, back);
    }
}

/// The explicit conversions must agree with the serde round trip.
#[test]
fn explicit_conversions_agree_with_serde() {
    for original in every_variant() {
        let gui = convert::to_proto(&original);
        let via_serde: proto::GuiValue =
            serde_json::from_str(&serde_json::to_string(&original).unwrap()).unwrap();
        assert_eq!(gui, via_serde, "to_proto disagrees with serde for {original:?}");
        assert_eq!(convert::from_proto(&gui), original, "from_proto is not the inverse");
    }
}

/// `SPValue::to_string()` is what the runtime logs; the table rendering should
/// not claim a different value.
#[test]
fn display_matches_for_scalars() {
    let cases = vec![
        SPValue::Bool(BoolOrUnknown::Bool(true)),
        SPValue::Int64(IntOrUnknown::Int64(-42)),
        SPValue::Bool(BoolOrUnknown::UNKNOWN),
        SPValue::String(StringOrUnknown::UNKNOWN),
    ];
    for case in cases {
        let gui = convert::to_proto(&case);
        assert_eq!(
            case.to_string().trim_matches('"'),
            gui.display().trim_matches('"'),
            "display drifted for {case:?}"
        );
    }
}

/// The payload presets and template names are copied out of `ur_redis_driver`.
#[test]
fn robot_constants_match_the_driver() {
    assert_eq!(proto::RSP_ONLY_PAYLOAD, ur_redis_driver::RSP_ONLY_PAYLOAD);
    assert_eq!(proto::RSP_AND_SPONGE_PAYLOAD, ur_redis_driver::RSP_AND_SPONGE_PAYLOAD);
    assert_eq!(proto::RSP_AND_GRIPPER_PAYLOAD, ur_redis_driver::RSP_AND_GRIPPER_PAYLOAD);
    assert_eq!(proto::RSP_AND_BVT_PAYLOAD, ur_redis_driver::RSP_AND_BVT_PAYLOAD);
    assert_eq!(proto::RSP_AND_SVT_PAYLOAD, ur_redis_driver::RSP_AND_SVT_PAYLOAD);
    assert_eq!(proto::RSP_AND_PHOTONEO_PAYLOAD, ur_redis_driver::RSP_AND_PHOTONEO_PAYLOAD);
}

/// Every key the server writes for a motion request must be one the driver
/// seeds, or the driver will reject the request as having unreadable keys.
#[test]
fn every_written_robot_key_is_one_the_driver_knows() {
    let seeded = ur_redis_driver::generate_robot_interface_state("r1", "test");
    let cmd = proto::RobotCommand::default();
    let written = micro_sp_gui_server::api_robot::command_to_state(&cmd);

    let mut unknown: Vec<&String> = written
        .state
        .keys()
        .filter(|k| !seeded.contains(k))
        .collect();
    unknown.sort();
    assert!(
        unknown.is_empty(),
        "these keys are not part of the driver's interface: {unknown:?}"
    );
}

/// Every dashboard command the protocol offers must be one the driver parses,
/// with the same argument rules and the same wire spelling.
///
/// This is the test that matters most for the Dashboard panel: the frontend
/// builds its buttons from `proto::DASHBOARD_COMMANDS`, so a name the driver has
/// renamed would show up as a button that always fails.
#[test]
fn dashboard_commands_match_the_driver() {
    use ur_redis_driver::DashboardCommand;

    for spec in proto::DASHBOARD_COMMANDS {
        // A sample argument the driver should accept: the first legal spelling
        // for a choice, anything non-empty for free text.
        let sample = match spec.arg {
            proto::DashboardArg::None => "",
            proto::DashboardArg::Text { .. } => "sample",
            proto::DashboardArg::Choice { options, .. } => options[0],
        };

        let parsed = DashboardCommand::parse(spec.name, sample).unwrap_or_else(|e| {
            panic!("the driver rejects '{}' with arg '{sample}': {e}", spec.name)
        });

        // Commands with an argument append it, so the spec carries the prefix.
        let wire = parsed.wire();
        if spec.takes_arg() {
            assert!(
                wire.starts_with(spec.wire),
                "'{}' sends '{wire}', not '{}...'",
                spec.name,
                spec.wire
            );
        } else {
            assert_eq!(wire, spec.wire, "wire spelling drifted for '{}'", spec.name);
        }

        // `expect() == None` is exactly the driver's definition of a query.
        assert_eq!(
            parsed.expect().is_none(),
            spec.query,
            "'{}' is {} a query in the driver",
            spec.name,
            if spec.query { "not" } else { "" }
        );

        assert_eq!(
            parsed.requires_remote_control(),
            spec.remote_control,
            "the remote-control requirement drifted for '{}'",
            spec.name
        );

        // The whole point of `DashboardArg::required`: the server refuses what
        // the driver would have refused, one poll earlier.
        assert_eq!(
            DashboardCommand::parse(spec.name, "").is_err(),
            spec.arg.is_required(),
            "'{}' disagrees with the driver about needing an argument",
            spec.name
        );

        for option in spec.arg.options() {
            assert!(
                DashboardCommand::parse(spec.name, option).is_ok(),
                "the driver rejects '{}' for '{}'",
                option,
                spec.name
            );
        }
    }

    for (alias, target) in proto::DASHBOARD_ALIASES {
        assert!(
            DashboardCommand::parse(alias, "").is_ok(),
            "the driver no longer accepts the alias '{alias}'"
        );
        let spec = proto::dashboard_spec(alias).expect("an alias resolves to a command");
        assert_eq!(spec.name, *target);
        assert_eq!(
            DashboardCommand::parse(alias, "").unwrap(),
            DashboardCommand::parse(target, "").unwrap(),
            "'{alias}' and '{target}' are no longer the same command"
        );
    }

    // And the negative case, so the checks above are not vacuous.
    assert!(DashboardCommand::parse("no_such_command", "").is_err());
    assert!(proto::dashboard_spec("no_such_command").is_none());
}

/// The table is well-formed: no command listed twice, and every group the panel
/// draws has something in it.
///
/// The driver exposes `parse` but no list of names, so nothing can prove the
/// panel covers every command it accepts. This at least catches the mistakes
/// that are easy to make while editing the table by hand.
#[test]
fn the_dashboard_panel_offers_every_command_once() {
    let mut names: Vec<&str> = proto::DASHBOARD_COMMANDS.iter().map(|s| s.name).collect();
    let total = names.len();
    names.sort();
    names.dedup();
    assert_eq!(names.len(), total, "a dashboard command is listed twice");

    // Every command lands in a group the panel actually draws.
    for spec in proto::DASHBOARD_COMMANDS {
        assert!(
            proto::DashboardGroup::ALL.contains(&spec.group),
            "'{}' is in a group the panel never draws",
            spec.name
        );
    }
    for group in proto::DashboardGroup::ALL {
        assert!(
            proto::dashboard_commands_in(*group).next().is_some(),
            "the panel draws an empty '{}' group",
            group.title()
        );
    }
}

/// Every key the Robot tab reads must be one the driver seeds, or the tab shows
/// a permanently blank field and nothing says why.
#[test]
fn every_robot_status_key_is_one_the_driver_seeds() {
    let seeded = ur_redis_driver::generate_robot_interface_state("r1", "test");
    let missing: Vec<String> = micro_sp_gui_server::status::robot_keys("r1")
        .into_iter()
        .filter(|key| !seeded.contains(key))
        .collect();
    assert!(
        missing.is_empty(),
        "the driver does not publish these keys the Robot tab reads: {missing:?}"
    );
}

/// `GoalPriority`'s integer encoding is what goes on the wire.
#[test]
fn goal_priority_encoding_matches() {
    use micro_sp::running::goal_runner::GoalPriority;
    let pairs = [
        (proto::GuiGoalPriority::Top, GoalPriority::Top),
        (proto::GuiGoalPriority::High, GoalPriority::High),
        (proto::GuiGoalPriority::Normal, GoalPriority::Normal),
        (proto::GuiGoalPriority::Low, GoalPriority::Low),
    ];
    for (gui, sp) in pairs {
        assert_eq!(gui.to_int(), sp.to_int(), "priority int drifted for {gui:?}");
        assert_eq!(proto::GuiGoalPriority::from_int(sp.to_int()), gui);
    }
}
