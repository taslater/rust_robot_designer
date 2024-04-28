use std::cell::RefCell;
use std::rc::Rc;

use eframe::{egui, NativeOptions};
use egui_dock::{DockArea, DockState, Style, TabViewer};

mod editor;
mod model;

use editor::robot_editor::RobotEditor;
use model::robot::Robot;

struct RobotDesignerApp {
    robot: Rc<RefCell<Robot>>,
    robot_editor: RobotEditor,
    dock_state: DockState<String>,
}

impl Default for RobotDesignerApp {
    fn default() -> Self {
        let robot = Rc::new(RefCell::new(Robot::new()));
        let dock_state = DockState::new(vec![
            "Editor".to_owned(),
            "Simulator".to_owned(),
            "Trainer".to_owned(),
        ]);

        RobotDesignerApp {
            robot: robot.clone(),
            robot_editor: RobotEditor::new(),
            dock_state,
        }
    }
}

impl eframe::App for RobotDesignerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        DockArea::new(&mut self.dock_state)
            .style(Style::from_egui(ctx.style().as_ref()))
            .show(ctx, &mut RobotDesignerTabViewer {
                robot: &mut self.robot,
                robot_editor: &mut self.robot_editor,
                ctx,
            });
    }
}

struct RobotDesignerTabViewer<'a> {
    robot: &'a mut Rc<RefCell<Robot>>,
    robot_editor: &'a mut RobotEditor,
    ctx: &'a egui::Context,
}

impl<'a> RobotDesignerTabViewer<'a> {
    fn draw_simulator_ui(&self, ui: &mut egui::Ui, robot: &Robot) {
        egui::Frame::canvas(ui.style()).show(ui, |ui| {
            let (_response, _painter) = ui.allocate_painter(
                egui::Vec2::new(400.0, 300.0),
                egui::Sense::click_and_drag(),
            );
            ui.label(format!("{:#?}", robot));
        });
    }

    fn draw_trainer_ui(&self, ui: &mut egui::Ui, robot: &Robot) {
        egui::Frame::canvas(ui.style()).show(ui, |ui| {
            let (_response, _painter) = ui.allocate_painter(
                egui::Vec2::new(400.0, 300.0),
                egui::Sense::click_and_drag(),
            );
            ui.label(format!("{:#?}", robot));
        });
    }
}

impl<'a> TabViewer for RobotDesignerTabViewer<'a> {
    type Tab = String;

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        (&*tab).into()
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Self::Tab) {
        match &**tab {
            "Editor" => {
                self.robot_editor.draw_editor_ui(ui, &mut self.robot.borrow_mut(), self.ctx);
            }
            "Simulator" => {
                self.draw_simulator_ui(ui, &self.robot.borrow());
            }
            "Trainer" => {
                self.draw_trainer_ui(ui, &self.robot.borrow());
            }
            _ => {}
        }
    }
}

fn main() -> eframe::Result<()> {
    env_logger::init();
    let options = NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([800.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Robot Designer App",
        options,
        Box::new(|_cc| Box::new(RobotDesignerApp::default())),
    )
}