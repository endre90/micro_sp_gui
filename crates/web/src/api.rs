//! The frontend's link to the server.
//!
//! Reads arrive on one websocket and are applied to a local mirror of the state;
//! writes are `POST`s whose outcome lands in an inbox the tabs drain. Nothing
//! here blocks - egui runs on a single thread and wasm has no threads to block
//! on anyway.

use micro_sp_gui_protocol as proto;
use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};

/// Seconds to wait before retrying a dropped socket.
const RECONNECT_DELAY_S: f64 = 2.0;

/// How many log lines the browser keeps.
const LOG_CAP: usize = 20_000;

#[derive(Clone, Debug, PartialEq)]
pub enum Connection {
    Connecting,
    Live,
    Down(String),
}

impl Connection {
    pub fn is_live(&self) -> bool {
        matches!(self, Connection::Live)
    }
}

/// What came back from a `POST`.
#[derive(Debug)]
pub enum PostOutcome {
    Status { text: String, error: bool },
    Lookup(proto::LookupResponse),
    Export(proto::ExportFrameResponse),
}

/// Which typed response a request expects.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Expect {
    /// Just report success or the server's error text.
    Status,
    Lookup,
    Export,
    /// A JSON body with a `summary()` worth showing.
    WriteReport,
    SendGoals,
}

#[derive(Default)]
struct Inbox {
    outcomes: Vec<PostOutcome>,
}

pub struct Api {
    ctx: egui::Context,
    http_base: String,
    ws_url: String,

    ws: Option<(ewebsock::WsSender, ewebsock::WsReceiver)>,
    retry_at: Option<f64>,

    pub conn: Connection,
    pub info: proto::ServerInfo,

    /// Local mirror of the keyspace, kept in sync by snapshots and deltas.
    pub entries: BTreeMap<String, proto::StateEntry>,
    pub revision: u64,
    pub transforms: proto::TransformsSnapshot,
    pub logs: VecDeque<proto::LogLine>,
    pub robots: BTreeMap<String, proto::RobotStatus>,
    pub goals: BTreeMap<String, proto::GoalsStatus>,

    /// Which system the tabs are looking at. `None` until discovery reports one.
    pub sp_id: Option<String>,
    pub robot_id: Option<String>,

    /// The last thing that happened, shown in the header.
    pub status: Option<(String, bool)>,

    inbox: Arc<Mutex<Inbox>>,
    /// Typed responses waiting for the tab that asked for them.
    parked: Vec<PostOutcome>,
    /// Set while a socket is open so `Subscribe` is only sent once per connect.
    subscribed: bool,
}

/// `(http base, ws base)` for the page we were served from.
#[cfg(target_arch = "wasm32")]
fn origins() -> (String, String) {
    let Some(window) = web_sys::window() else {
        return ("".to_string(), "ws://localhost:8080".to_string());
    };
    let location = window.location();
    let protocol = location.protocol().unwrap_or_else(|_| "http:".to_string());
    let host = location.host().unwrap_or_else(|_| "localhost:8080".to_string());
    let ws_scheme = if protocol == "https:" { "wss" } else { "ws" };
    // Relative URLs for HTTP so trunk's dev proxy works unchanged.
    (String::new(), format!("{ws_scheme}://{host}"))
}

/// Natively (used by `cargo check` and by running the frontend as a desktop
/// window against a local server) there is no page to ask.
#[cfg(not(target_arch = "wasm32"))]
fn origins() -> (String, String) {
    ("http://localhost:8080".to_string(), "ws://localhost:8080".to_string())
}

impl Api {
    pub fn new(ctx: egui::Context) -> Self {
        let (http_base, ws_base) = origins();
        let mut api = Self {
            ctx,
            http_base,
            ws_url: format!("{ws_base}{}", proto::api::WS),
            ws: None,
            retry_at: None,
            conn: Connection::Connecting,
            info: proto::ServerInfo::default(),
            entries: BTreeMap::new(),
            revision: 0,
            transforms: proto::TransformsSnapshot::default(),
            logs: VecDeque::new(),
            robots: BTreeMap::new(),
            goals: BTreeMap::new(),
            sp_id: None,
            robot_id: None,
            status: None,
            inbox: Arc::new(Mutex::new(Inbox::default())),
            parked: Vec::new(),
            subscribed: false,
        };
        api.connect();
        api
    }

    fn connect(&mut self) {
        self.conn = Connection::Connecting;
        self.subscribed = false;
        let ctx = self.ctx.clone();
        // Wake the UI when a frame arrives, instead of spinning at 60 fps.
        let wake = move || ctx.request_repaint();
        match ewebsock::connect_with_wakeup(&self.ws_url, ewebsock::Options::default(), wake) {
            Ok(pair) => self.ws = Some(pair),
            Err(e) => {
                self.conn = Connection::Down(e.to_string());
                self.ws = None;
            }
        }
    }

    /// Drain the socket and the POST inbox. Call once per frame, first thing.
    pub fn poll(&mut self, now: f64) {
        self.drain_inbox();

        if self.ws.is_none() {
            match self.retry_at {
                Some(at) if now >= at => {
                    self.retry_at = None;
                    self.connect();
                }
                Some(_) => {}
                None => self.retry_at = Some(now + RECONNECT_DELAY_S),
            }
            // Keep a slow heartbeat so the retry actually happens while idle.
            self.ctx.request_repaint_after(std::time::Duration::from_millis(500));
            return;
        }

        let mut events = Vec::new();
        if let Some((_, receiver)) = &self.ws {
            while let Some(event) = receiver.try_recv() {
                events.push(event);
            }
        }
        for event in events {
            self.handle_ws_event(event);
        }
    }

    fn handle_ws_event(&mut self, event: ewebsock::WsEvent) {
        match event {
            ewebsock::WsEvent::Opened => {
                self.conn = Connection::Live;
                self.retry_at = None;
                if !self.subscribed {
                    self.subscribed = true;
                    self.send_subscribe(proto::LogQuery::default());
                }
            }
            ewebsock::WsEvent::Message(ewebsock::WsMessage::Text(text)) => {
                match serde_json::from_str::<proto::ServerMsg>(&text) {
                    Ok(msg) => self.apply(msg),
                    Err(e) => log::warn!("Unreadable server message: {e}"),
                }
            }
            ewebsock::WsEvent::Message(_) => {}
            ewebsock::WsEvent::Error(e) => {
                self.conn = Connection::Down(e);
                self.drop_socket();
            }
            ewebsock::WsEvent::Closed => {
                self.conn = Connection::Down("connection closed".to_string());
                self.drop_socket();
            }
        }
    }

    fn drop_socket(&mut self) {
        self.ws = None;
        self.subscribed = false;
        self.retry_at = None;
    }

    pub fn send_subscribe(&mut self, log_query: proto::LogQuery) {
        let msg = proto::ClientMsg::Subscribe {
            sp_id: self.sp_id.clone(),
            robot_id: self.robot_id.clone(),
            log_query,
        };
        if let (Some((sender, _)), Ok(text)) = (self.ws.as_mut(), serde_json::to_string(&msg)) {
            sender.send(ewebsock::WsMessage::Text(text));
        }
    }

    fn apply(&mut self, msg: proto::ServerMsg) {
        match msg {
            proto::ServerMsg::Hello(info) => {
                // Keep a selection the operator made; otherwise take the first
                // system that exists.
                if self.sp_id.as_ref().is_none_or(|id| !info.sp_ids.contains(id)) {
                    self.sp_id = info.sp_ids.first().cloned();
                }
                if self.robot_id.as_ref().is_none_or(|id| !info.robot_ids.contains(id)) {
                    self.robot_id = info.robot_ids.first().cloned();
                }
                self.info = info;
            }
            proto::ServerMsg::State(snapshot) => {
                self.revision = snapshot.revision;
                self.entries =
                    snapshot.entries.into_iter().map(|e| (e.key.clone(), e)).collect();
            }
            proto::ServerMsg::StateDelta(delta) => {
                self.revision = delta.revision;
                for entry in delta.changed {
                    self.entries.insert(entry.key.clone(), entry);
                }
                for key in delta.removed {
                    self.entries.remove(&key);
                }
            }
            proto::ServerMsg::Transforms(snapshot) => self.transforms = snapshot,
            proto::ServerMsg::Logs(lines) => {
                // A backfill can arrive after live lines (the server answers a
                // Subscribe with history), so order and dedupe on `seq` rather
                // than trusting arrival order.
                let mut all: Vec<proto::LogLine> = std::mem::take(&mut self.logs).into();
                all.extend(lines);
                all.sort_by_key(|l| l.seq);
                all.dedup_by_key(|l| l.seq);
                if all.len() > LOG_CAP {
                    all.drain(0..all.len() - LOG_CAP);
                }
                self.logs = all.into();
            }
            proto::ServerMsg::RobotStatus(status) => {
                self.robots.insert(status.robot_id.clone(), status);
            }
            proto::ServerMsg::Goals(status) => {
                self.goals.insert(status.sp_id.clone(), status);
            }
            proto::ServerMsg::Error(text) => self.status = Some((text, true)),
        }
    }

    // ---- writes ------------------------------------------------------------

    fn post<T: serde::Serialize>(&self, path: &str, body: &T, expect: Expect) {
        let Ok(bytes) = serde_json::to_vec(body) else {
            self.push(PostOutcome::Status {
                text: "Could not encode the request.".to_string(),
                error: true,
            });
            return;
        };
        let mut request = ehttp::Request::post(format!("{}{}", self.http_base, path), bytes);
        request.headers.insert("Content-Type".to_string(), "application/json".to_string());

        let inbox = self.inbox.clone();
        let ctx = self.ctx.clone();
        let what = path.to_string();
        ehttp::fetch(request, move |result| {
            let outcome = decode_response(&what, expect, result);
            if let Ok(mut inbox) = inbox.lock() {
                inbox.outcomes.push(outcome);
            }
            ctx.request_repaint();
        });
    }

    fn push(&self, outcome: PostOutcome) {
        if let Ok(mut inbox) = self.inbox.lock() {
            inbox.outcomes.push(outcome);
        }
    }

    /// Outcomes that are not claimed by a tab become the header status line.
    fn drain_inbox(&mut self) {
        let outcomes = match self.inbox.lock() {
            Ok(mut inbox) => std::mem::take(&mut inbox.outcomes),
            Err(_) => return,
        };
        for outcome in outcomes {
            match outcome {
                PostOutcome::Status { text, error } => self.status = Some((text, error)),
                // Typed responses are parked for whichever tab asked.
                other => self.parked.push(other),
            }
        }
    }

    pub fn set_values(&mut self, values: Vec<proto::SetValue>) {
        self.post(
            proto::api::STATE_SET,
            &proto::SetValuesRequest { values },
            Expect::WriteReport,
        );
    }

    pub fn delete_keys(&mut self, keys: Vec<String>) {
        self.post(
            proto::api::STATE_DELETE,
            &proto::DeleteKeysRequest { keys },
            Expect::WriteReport,
        );
    }

    pub fn transform_command(&mut self, cmd: proto::TransformCommand) {
        self.post(proto::api::TRANSFORMS_CMD, &cmd, Expect::Status);
    }

    pub fn lookup(&mut self, parent: String, child: String) {
        self.post(
            proto::api::TRANSFORMS_LOOKUP,
            &proto::LookupRequest { parent, child },
            Expect::Lookup,
        );
    }

    pub fn export_frame(&mut self, req: proto::ExportFrameRequest) {
        self.post(proto::api::TRANSFORMS_EXPORT, &req, Expect::Export);
    }

    pub fn robot_command(&mut self, cmd: proto::RobotCommand) {
        self.post(proto::api::ROBOT_COMMAND, &cmd, Expect::Status);
    }

    pub fn robot_cancel(&mut self, robot_id: String) {
        self.post(
            proto::api::ROBOT_CANCEL,
            &proto::RobotCancelRequest { robot_id },
            Expect::Status,
        );
    }

    pub fn robot_dashboard(&mut self, cmd: proto::RobotDashboardCommand) {
        self.post(proto::api::ROBOT_DASHBOARD, &cmd, Expect::Status);
    }

    pub fn send_goals(&mut self, req: proto::SendGoalsRequest) {
        self.post(proto::api::GOALS_SEND, &req, Expect::SendGoals);
    }

    // ---- convenience -------------------------------------------------------

    /// Claim a lookup response, if one has arrived.
    pub fn take_lookup(&mut self) -> Option<proto::LookupResponse> {
        let i = self.parked.iter().position(|o| matches!(o, PostOutcome::Lookup(_)))?;
        match self.parked.remove(i) {
            PostOutcome::Lookup(r) => Some(r),
            _ => None,
        }
    }

    /// Claim an export response, if one has arrived.
    pub fn take_export(&mut self) -> Option<proto::ExportFrameResponse> {
        let i = self.parked.iter().position(|o| matches!(o, PostOutcome::Export(_)))?;
        match self.parked.remove(i) {
            PostOutcome::Export(r) => Some(r),
            _ => None,
        }
    }

    pub fn robot_status(&self) -> Option<&proto::RobotStatus> {
        self.robot_id.as_ref().and_then(|id| self.robots.get(id))
    }

    pub fn goals_status(&self) -> Option<&proto::GoalsStatus> {
        self.sp_id.as_ref().and_then(|id| self.goals.get(id))
    }

    /// Frame names, for the pickers.
    pub fn frame_names(&self) -> Vec<String> {
        let mut names: Vec<String> =
            self.transforms.frames.iter().map(|f| f.child_frame_id.clone()).collect();
        // Parents that have no frame of their own are still valid lookup targets.
        for frame in &self.transforms.frames {
            if !names.contains(&frame.parent_frame_id) {
                names.push(frame.parent_frame_id.clone());
            }
        }
        names.sort();
        names.dedup();
        names
    }

    pub fn frame(&self, name: &str) -> Option<&proto::GuiTransformStamped> {
        self.transforms.frames.iter().find(|f| f.child_frame_id == name)
    }

    /// A state value read the safe way: absent and malformed both give `None`.
    pub fn value(&self, key: &str) -> Option<&proto::GuiValue> {
        self.entries.get(key).and_then(|e| e.value.as_ref())
    }

    pub fn floats(&self, key: &str) -> Vec<f64> {
        match self.value(key) {
            Some(proto::GuiValue::Array(proto::ArrayOrUnknown::Array(items))) => items
                .iter()
                .filter_map(|v| match v {
                    proto::GuiValue::Float64(proto::FloatOrUnknown::Float64(f)) => Some(*f),
                    proto::GuiValue::Int64(proto::IntOrUnknown::Int64(i)) => Some(*i as f64),
                    _ => None,
                })
                .collect(),
            _ => Vec::new(),
        }
    }

    pub fn float(&self, key: &str) -> f64 {
        match self.value(key) {
            Some(proto::GuiValue::Float64(proto::FloatOrUnknown::Float64(f))) => *f,
            Some(proto::GuiValue::Int64(proto::IntOrUnknown::Int64(i))) => *i as f64,
            _ => 0.0,
        }
    }

    pub fn note(&mut self, text: impl Into<String>) {
        self.status = Some((text.into(), false));
    }

    pub fn warn(&mut self, text: impl Into<String>) {
        self.status = Some((text.into(), true));
    }
}

fn decode_response(
    what: &str,
    expect: Expect,
    result: Result<ehttp::Response, String>,
) -> PostOutcome {
    let response = match result {
        Ok(response) => response,
        Err(e) => {
            return PostOutcome::Status { text: format!("{what}: {e}"), error: true };
        }
    };
    let body = response.text().unwrap_or_default().to_string();

    if !response.ok {
        // The server puts its reason in the body for a 400, which is exactly
        // what the operator needs to see.
        let text = if body.trim().is_empty() {
            format!("{what}: HTTP {}", response.status)
        } else {
            body
        };
        return PostOutcome::Status { text, error: true };
    }

    match expect {
        Expect::Status => {
            PostOutcome::Status { text: format!("{what}: ok"), error: false }
        }
        Expect::WriteReport => match serde_json::from_str::<proto::WriteReport>(&body) {
            Ok(report) => PostOutcome::Status {
                text: report.summary(),
                error: !report.failed.is_empty(),
            },
            Err(e) => PostOutcome::Status { text: format!("{what}: {e}"), error: true },
        },
        Expect::SendGoals => match serde_json::from_str::<proto::SendGoalsResponse>(&body) {
            Ok(r) => match r.error {
                Some(e) => PostOutcome::Status { text: e, error: true },
                None => PostOutcome::Status {
                    text: format!(
                        "Queued {} goal(s); {} were already waiting.",
                        r.accepted, r.queued_before
                    ),
                    error: false,
                },
            },
            Err(e) => PostOutcome::Status { text: format!("{what}: {e}"), error: true },
        },
        Expect::Lookup => match serde_json::from_str::<proto::LookupResponse>(&body) {
            Ok(r) => PostOutcome::Lookup(r),
            Err(e) => PostOutcome::Status { text: format!("{what}: {e}"), error: true },
        },
        Expect::Export => match serde_json::from_str::<proto::ExportFrameResponse>(&body) {
            Ok(r) => PostOutcome::Export(r),
            Err(e) => PostOutcome::Status { text: format!("{what}: {e}"), error: true },
        },
    }
}
