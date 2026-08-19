//! A native window running the same frontend against a local server.
//!
//! Not the delivery target - the GUI is a web app - but invaluable while
//! developing, because it gives real compiler errors and a debugger without a
//! wasm round trip. It talks to `http://localhost:8080` like the browser does.
//!
//! Trunk builds every target in this crate for wasm, so on that target this
//! becomes an empty `main` rather than a build failure.

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    env_logger::init();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 860.0]),
        ..Default::default()
    };
    eframe::run_native(
        "micro_sp_gui (native dev shell)",
        options,
        Box::new(|cc| Ok(Box::new(micro_sp_gui_web::App::new(cc)))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {}
