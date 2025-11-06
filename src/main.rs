use eframe::egui;
mod transforms;
mod another;
mod lookup;
mod robot;
mod tabs;

#[tokio::main]
async fn main() -> Result<(), eframe::Error> {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([750.0, 750.0]),
        ..Default::default()
    };

    let handle = tokio::runtime::Handle::current();
    let my_app = tabs::MyApp::new(handle).await;

    eframe::run_native(
        "micro_sp controller",
        options,
        Box::new(|cc| {
            cc.egui_ctx.set_pixels_per_point(1.25);
            Ok(Box::new(my_app))
        }),
    )
}
