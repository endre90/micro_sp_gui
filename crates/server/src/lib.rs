//! The native half of `micro_sp_gui`.
//!
//! Holds the one [`micro_sp::ConnectionManager`] in the system, polls Redis, and
//! serves the wasm frontend plus a websocket push feed and a small write API.
//! See [`crate::poller`] for the read path and the `api_*` modules for writes.

pub mod api_goals;
pub mod api_robot;
pub mod api_state;
pub mod api_transforms;
pub mod app_state;
pub mod config;
pub mod convert;
pub mod discovery;
pub mod logs;
pub mod poller;
pub mod status;
pub mod ws;

pub use app_state::AppState;
pub use config::Config;

/// The `log` target for everything this crate logs, and the `log_target` handed
/// to `micro_sp` calls so its own messages are attributable to the GUI.
pub const LOG_TARGET: &str = "micro_sp_gui";
