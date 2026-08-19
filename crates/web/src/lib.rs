//! The micro_sp_gui frontend: egui, compiled to wasm and served by
//! `micro_sp_gui_server`.
//!
//! It never talks to Redis - it cannot, there are no sockets in wasm. Everything
//! goes through the server (see [`api`]).

pub mod api;
pub mod app;
pub mod tabs;
pub mod widgets;

pub use app::App;

/// Entry point for the browser build.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen::prelude::wasm_bindgen(start)]
pub fn start() -> Result<(), wasm_bindgen::JsValue> {
    use eframe::wasm_bindgen::JsCast as _;

    // Otherwise a panic is a silently frozen page.
    console_error_panic_hook::set_once();
    eframe::WebLogger::init(log::LevelFilter::Info).ok();

    let document = web_sys::window()
        .ok_or("no window")?
        .document()
        .ok_or("no document")?;
    let canvas = document
        .get_element_by_id("micro_sp_gui_canvas")
        .ok_or("no canvas element with id 'micro_sp_gui_canvas'")?
        .dyn_into::<web_sys::HtmlCanvasElement>()
        .map_err(|_| "'micro_sp_gui_canvas' is not a canvas")?;

    wasm_bindgen_futures::spawn_local(async move {
        let result = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| Ok(Box::new(App::new(cc)))),
            )
            .await;

        // Replace the loading text with the reason, so a failure is visible on
        // the page rather than only in the console.
        if let Some(loading) = document.get_element_by_id("loading_text") {
            match result {
                Ok(_) => loading.remove(),
                Err(e) => {
                    loading.set_inner_html(
                        "<p>The app failed to start. See the developer console.</p>",
                    );
                    log::error!("Failed to start eframe: {e:?}");
                }
            }
        }
    });

    Ok(())
}
