use eframe::egui;
mod tabs;
mod lookup;
mod another;

#[tokio::main]
async fn main() -> Result<(), eframe::Error> {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([500.0, 400.0]),
        ..Default::default()
    };

    let handle = tokio::runtime::Handle::current();
    let my_app = tabs::MyApp::new(handle).await;

    eframe::run_native(
        "Transforms Lookup App",
        options,
        Box::new(|_cc| Ok(Box::new(my_app))),
    )
}
