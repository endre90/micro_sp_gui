//! Command-line and environment configuration.
//!
//! Redis itself is not configured here: `micro_sp::ConnectionManager::new()`
//! reads `REDIS_HOST` and `REDIS_PORT` so the GUI agrees with every other
//! micro_sp process by default.

use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug, Clone)]
#[command(
    name = "micro_sp_gui",
    about = "Web GUI for a micro_sp system: state, transforms, robot, logs, goals."
)]
pub struct Config {
    /// Address to serve on.
    #[arg(long, env = "MICRO_SP_GUI_BIND", default_value = "0.0.0.0:8080")]
    pub bind: String,

    /// Directory holding the built frontend (`trunk build` output).
    #[arg(long, env = "MICRO_SP_GUI_DIST", default_value = "dist")]
    pub dist: PathBuf,

    /// How often to poll Redis, in milliseconds. One poller serves every
    /// connected browser, so this is the total load the GUI puts on Redis.
    #[arg(long, env = "MICRO_SP_GUI_POLL_MS", default_value_t = 250)]
    pub poll_ms: u64,

    /// Where micro_sp writes its activity log. Defaults to the same variable
    /// micro_sp itself reads, so setting it once covers both.
    #[arg(long, env = "MICRO_SP_ACTIVITY_LOG_DIR")]
    pub log_dir: Option<PathBuf>,

    /// Base name of the activity log files; the active one is `{stem}.log`.
    #[arg(long, env = "MICRO_SP_GUI_LOG_STEM", default_value = "micro_sp")]
    pub log_stem: String,

    /// How many log lines to keep in memory for backfill.
    #[arg(long, env = "MICRO_SP_GUI_LOG_RING", default_value_t = 20_000)]
    pub log_ring: usize,

    /// Where the Transforms tab writes exported frame JSON. Without it, export
    /// only shows the JSON instead of saving it.
    #[arg(long, env = "MICRO_SP_GUI_FRAMES_DIR")]
    pub frames_dir: Option<PathBuf>,

    /// Seed the sp_id picker even before discovery finds anything.
    #[arg(long, env = "SP_ID")]
    pub sp_id: Option<String>,

    /// Seed the robot picker likewise.
    #[arg(long, env = "ROBOT_ID")]
    pub robot_id: Option<String>,
}

impl Config {
    /// The active activity-log file, if logging is configured at all.
    pub fn active_log_path(&self) -> Option<PathBuf> {
        self.log_dir.as_ref().map(|d| d.join(format!("{}.log", self.log_stem)))
    }
}
