//! `micro_sp_gui` - serves the web GUI and bridges it to Redis.
//!
//! ```text
//! browser (egui/wasm) --- /ws  push feed ---> one poller ---> Redis
//!                     \-- /api/* POSTs ----> StateManager / TransformsManager
//! ```
//!
//! Run it next to a micro_sp system:
//!
//! ```bash
//! trunk build --release                # produces ./dist
//! cargo run -p micro_sp_gui_server -- --log-dir /tmp/msp_logs
//! ```

use axum::extract::State;
use axum::http::StatusCode;
use axum::response::IntoResponse;
use axum::routing::{get, post};
use axum::{Json, Router};
use clap::Parser;
use micro_sp::ConnectionManager;
use micro_sp_gui_protocol as proto;
use micro_sp_gui_server::{
    AppState, Config, LOG_TARGET, api_goals, api_robot, api_state, api_transforms, logs, poller, ws,
};
use std::sync::Arc;
use tower_http::services::{ServeDir, ServeFile};

#[tokio::main]
async fn main() {
    let cfg = Config::parse();
    // micro_sp's own initialiser, so RUST_LOG and LOG_SHOW_TIME behave the same
    // here as in every other process in the system.
    micro_sp::initialize_env_logger();

    log::info!(target: LOG_TARGET, "Connecting to Redis...");
    // Retries forever by design: starting the GUI before Redis should just mean
    // it waits, not that it dies.
    let cm = Arc::new(ConnectionManager::new().await);
    log::info!(target: LOG_TARGET, "Connected to {}.", cm.redis_addr());

    if let Some(dir) = &cfg.log_dir {
        if dir.is_dir() {
            log::info!(target: LOG_TARGET, "Tailing activity log in {}.", dir.display());
        } else {
            log::warn!(
                target: LOG_TARGET,
                "Activity log directory {} does not exist yet; it will be picked up once micro_sp creates it.",
                dir.display()
            );
        }
    }

    let state = AppState::new(cm, cfg.clone());

    tokio::spawn(poller::run(state.clone()));
    tokio::spawn(logs::run(state.clone()));

    let index = cfg.dist.join("index.html");
    if !index.is_file() {
        log::warn!(
            target: LOG_TARGET,
            "{} not found - run `trunk build` (or `trunk serve` for development). \
             The API still works; only the page is missing.",
            index.display()
        );
    }
    // The frontend is a single-page app, so an unknown path serves index.html
    // rather than 404ing.
    let frontend = ServeDir::new(&cfg.dist).fallback(ServeFile::new(&index));

    let app = Router::new()
        .route(proto::api::WS, get(ws::handler))
        .route(proto::api::STATE_SET, post(set_values))
        .route(proto::api::STATE_DELETE, post(delete_keys))
        .route(proto::api::TRANSFORMS_CMD, post(transform_command))
        .route(proto::api::TRANSFORMS_LOOKUP, post(transform_lookup))
        .route(proto::api::TRANSFORMS_EXPORT, post(transform_export))
        .route(proto::api::ROBOT_COMMAND, post(robot_command))
        .route(proto::api::ROBOT_CANCEL, post(robot_cancel))
        .route(proto::api::ROBOT_DASHBOARD, post(robot_dashboard))
        .route(proto::api::GOALS_SEND, post(goals_send))
        .route("/api/logs", post(logs_query))
        .fallback_service(frontend)
        .with_state(state);

    let listener = match tokio::net::TcpListener::bind(&cfg.bind).await {
        Ok(listener) => listener,
        Err(e) => {
            log::error!(target: LOG_TARGET, "Could not bind {}: {e}", cfg.bind);
            std::process::exit(1);
        }
    };
    log::info!(target: LOG_TARGET, "Serving on http://{}", cfg.bind);
    // No authentication: this exposes unrestricted state writes and robot
    // commands, so keep it on localhost or a trusted network.
    if let Err(e) = axum::serve(listener, app).await {
        log::error!(target: LOG_TARGET, "Server stopped: {e}");
    }
}

/// A rejected write is a 400 with the reason, so the tab can show it verbatim.
fn bad_request(message: String) -> (StatusCode, String) {
    (StatusCode::BAD_REQUEST, message)
}

async fn set_values(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::SetValuesRequest>,
) -> impl IntoResponse {
    Json(api_state::set_values(&state.cm, &req).await)
}

async fn delete_keys(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::DeleteKeysRequest>,
) -> impl IntoResponse {
    Json(api_state::delete_keys(&state.cm, &req).await)
}

async fn transform_command(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::TransformCommand>,
) -> Result<(), (StatusCode, String)> {
    api_transforms::command(&state.cm, &req).await.map_err(bad_request)
}

async fn transform_lookup(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::LookupRequest>,
) -> impl IntoResponse {
    Json(api_transforms::lookup(&state.cm, &req).await)
}

async fn transform_export(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::ExportFrameRequest>,
) -> impl IntoResponse {
    Json(api_transforms::export(state.cfg.frames_dir.as_deref(), &req).await)
}

async fn robot_command(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::RobotCommand>,
) -> Result<(), (StatusCode, String)> {
    api_robot::send_command(&state.cm, &req).await.map_err(bad_request)
}

async fn robot_cancel(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::RobotCancelRequest>,
) -> Result<(), (StatusCode, String)> {
    api_robot::cancel(&state.cm, &req.robot_id).await.map_err(bad_request)
}

async fn robot_dashboard(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::RobotDashboardCommand>,
) -> Result<(), (StatusCode, String)> {
    api_robot::dashboard(&state.cm, &req).await.map_err(bad_request)
}

async fn goals_send(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::SendGoalsRequest>,
) -> impl IntoResponse {
    Json(api_goals::send(&state.cm, &req).await)
}

async fn logs_query(
    State(state): State<Arc<AppState>>,
    Json(req): Json<proto::LogQuery>,
) -> impl IntoResponse {
    Json(state.query_logs(&req).await)
}
