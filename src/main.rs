use eframe::egui;
mod another;
mod lookup;
mod robot;
mod tabs;

#[tokio::main]
async fn main() -> Result<(), eframe::Error> {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1300.0, 1000.0]),
        ..Default::default()
    };

    let handle = tokio::runtime::Handle::current();
    let my_app = tabs::MyApp::new(handle).await;

    eframe::run_native(
        "micro_sp controller",
        options,
        Box::new(|cc| {
            cc.egui_ctx.set_pixels_per_point(1.5);
            Ok(Box::new(my_app))
        }),
    )
}
