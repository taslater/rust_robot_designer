mod capsule;
mod joint;
mod capsule_app;

use eframe::egui;
use capsule_app::CapsuleApp;

fn main() -> Result<(), eframe::Error> {
    env_logger::init();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([800.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Capsule App",
        options,
        Box::new(|_cc| Box::new(CapsuleApp::default())),
    )
}
