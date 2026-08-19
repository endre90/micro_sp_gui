//! Wire types shared by the `micro_sp_gui` server and its wasm frontend.
//!
//! # Why this crate exists
//!
//! The frontend is egui compiled to `wasm32-unknown-unknown`, which has no TCP
//! sockets, so it cannot talk to Redis. `micro_sp` is not wasm-buildable either
//! (`tokio` with `full`, `redis` with `tokio-comp`, both unconditional), so the
//! frontend cannot even borrow its types. This crate is therefore a
//! dependency-light mirror: `serde` and `serde_json`, nothing else.
//!
//! # Transport
//!
//! Reads are pushed over one websocket (`/ws`, [`ServerMsg`]); writes are plain
//! `POST`s to `/api/*`. One direction per transport, so no request/response
//! correlation ids are needed anywhere.

pub mod goals;
pub mod logs;
pub mod robot;
pub mod state;
pub mod transforms;
pub mod value;

pub use goals::*;
pub use logs::*;
pub use robot::*;
pub use state::*;
pub use transforms::*;
pub use value::*;

use serde::{Deserialize, Serialize};

/// Everything the server pushes.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ServerMsg {
    /// Sent once per connection, before anything else.
    Hello(ServerInfo),
    /// The whole keyspace. Sent on connect and after a reconnect.
    State(StateSnapshot),
    /// Only what changed. Sent on every poll that found a difference.
    StateDelta(StateDelta),
    Transforms(TransformsSnapshot),
    /// A backfill batch or newly tailed lines.
    Logs(Vec<LogLine>),
    RobotStatus(RobotStatus),
    Goals(GoalsStatus),
    /// A server-side problem worth showing, e.g. Redis went away.
    Error(String),
}

/// Everything the frontend sends over the socket. Writes do not go here - they
/// are `POST`s.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ClientMsg {
    Subscribe {
        sp_id: Option<String>,
        robot_id: Option<String>,
        log_query: LogQuery,
    },
    Ping,
}

/// Paths the frontend posts to, in one place so the two halves cannot drift.
pub mod api {
    pub const STATE_SET: &str = "/api/state/set";
    pub const STATE_DELETE: &str = "/api/state/delete";
    pub const TRANSFORMS_CMD: &str = "/api/transforms/cmd";
    pub const TRANSFORMS_LOOKUP: &str = "/api/transforms/lookup";
    pub const TRANSFORMS_EXPORT: &str = "/api/transforms/export";
    pub const ROBOT_COMMAND: &str = "/api/robot/command";
    pub const ROBOT_CANCEL: &str = "/api/robot/cancel";
    pub const ROBOT_DASHBOARD: &str = "/api/robot/dashboard";
    pub const GOALS_SEND: &str = "/api/goals/send";
    pub const WS: &str = "/ws";
}
