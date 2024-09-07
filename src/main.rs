use std::cell::RefCell;
use std::rc::Rc;

use eframe::{egui, NativeOptions};
use egui_dock::{DockArea, DockState, Style, TabViewer};

mod editor;
mod model;
mod robot_simulator;
mod physics_world;
mod robot_physics;
pub mod constants;
mod robot_trainer;
pub mod brain;
pub mod cma_es;
pub mod cma_es_fns;

use robot_trainer::RobotTrainer;

use editor::robot_editor::RobotEditor;
use model::robot::Robot;
use robot_simulator::RobotSimulator;

struct RobotDesignerApp {
    robot: Rc<RefCell<Robot>>,
    robot_editor: RobotEditor,
    robot_simulator: RobotSimulator,
    robot_trainer: RobotTrainer,
    dock_state: DockState<String>,
    current_tab: String,
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
            robot_simulator: RobotSimulator::new(),
            robot_trainer: RobotTrainer::new(),
            dock_state,
            current_tab: "Editor".to_string(),
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
                robot_simulator: &mut self.robot_simulator,
                robot_trainer: &mut self.robot_trainer,
                ctx,
                current_tab: &mut self.current_tab,
            });
    }
}

struct RobotDesignerTabViewer<'a> {
    robot: &'a mut Rc<RefCell<Robot>>,
    robot_editor: &'a mut RobotEditor,
    robot_simulator: &'a mut RobotSimulator,
    robot_trainer: &'a mut RobotTrainer,
    ctx: &'a egui::Context,
    current_tab: &'a mut String,
}

impl<'a> TabViewer for RobotDesignerTabViewer<'a> {
    type Tab = String;

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        (&*tab).into()
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Self::Tab) {
        let previous_tab = self.current_tab.clone();
        *self.current_tab = (*tab).clone();

        match &**tab {
            "Editor" => {
                self.robot_editor.draw_editor_ui(ui, &mut self.robot.borrow_mut(), self.ctx);
            }
            "Simulator" => {
                if previous_tab != "Simulator" {
                    println!("Switched to Simulator tab");
                    self.robot_simulator.init_physics(&self.robot.borrow());
                }
                self.robot_simulator.ui(ui, &mut self.robot.borrow_mut());
            }
            "Trainer" => {
                // self.draw_trainer_ui(ui, &self.robot.borrow());
                if previous_tab != "Trainer" {
                    println!("Switched to Trainer tab");
                    // self.robot_trainer.init_physics(&self.robot.borrow());
                }
                self.robot_trainer.ui(ui, &mut self.robot.borrow_mut());
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