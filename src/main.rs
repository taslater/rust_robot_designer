use std::cell::RefCell;
use std::rc::Rc;
mod editor;
mod model;


use eframe::egui;
use egui::{Color32, Frame, Slider, Stroke};

use crate::editor::common::EditingPart;
use editor::common::EditingState;
use editor::robot_editor::RobotEditor;
use model::robot::Robot;

pub struct RobotDesignerApp {
    editing_part: EditingPart,
    editing_state: EditingState,
    robot: Rc<RefCell<Robot>>,
    robot_editor: RobotEditor,
}

impl RobotDesignerApp {
    fn on_editing_state_changed(&mut self) {
        match self.editing_part {
            EditingPart::Capsule => {
                self.robot_editor
                    .capsule_editor
                    .on_editing_state_changed(self.editing_state);
            }
            EditingPart::Joint => {
                self.robot_editor.capsule_editor.clear_capsule_selection();
                self.robot_editor
                    .joint_editor
                    .on_editing_state_changed(self.editing_state);
            }
        }
    }
}

impl Default for RobotDesignerApp {
    fn default() -> Self {
        let robot = Rc::new(RefCell::new(Robot::new()));
        RobotDesignerApp {
            editing_part: EditingPart::Capsule,
            editing_state: EditingState::Create,
            robot: robot.clone(),
            robot_editor: RobotEditor::new(robot),
        }
    }
}

impl eframe::App for RobotDesignerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Robot Designer App");

            ui.horizontal(|ui| {
                ui.label("Part Type:");
                let robot_clicked = ui
                    .radio_value(&mut self.editing_part, EditingPart::Capsule, "Capsule")
                    .clicked();

                let has_overlapping_capsules = self
                    .robot_editor
                    .capsule_editor
                    .robot
                    .borrow()
                    .capsules
                    .windows(2)
                    .any(|window| {
                        let capsule1 = &window[0];
                        let capsule2 = &window[1];
                        capsule1.intersects_capsule(capsule2)
                    });

                ui.scope(|ui| {
                    ui.set_enabled(has_overlapping_capsules);
                    let joint_clicked = ui
                        .radio_value(&mut self.editing_part, EditingPart::Joint, "Joint")
                        .clicked();

                    if robot_clicked || (joint_clicked && has_overlapping_capsules) {
                        self.on_editing_state_changed();
                    }
                });
            });

            ui.horizontal(|ui| {
                ui.label("Editing Mode:");
                let create_clicked = ui
                    .radio_value(&mut self.editing_state, EditingState::Create, "Create")
                    .clicked();
                let update_clicked = ui
                    .radio_value(&mut self.editing_state, EditingState::Update, "Update")
                    .clicked();
                let delete_clicked = ui
                    .radio_value(&mut self.editing_state, EditingState::Delete, "Delete")
                    .clicked();

                if create_clicked || update_clicked || delete_clicked {
                    self.on_editing_state_changed();
                }
            });

            Frame::canvas(ui.style())
                .stroke(Stroke::new(1.0, Color32::BLACK))
                .show(ui, |ui| {
                    let (response, painter) = ui.allocate_painter(
                        egui::Vec2::new(400.0, 300.0),
                        egui::Sense::click_and_drag(),
                    );

                    let pointer_pos = response.hover_pos().unwrap_or_default();

                    self.robot.borrow().draw(&painter);
                    self.robot_editor.draw_editor(
                        &painter,
                        pointer_pos,
                        self.editing_state,
                        self.editing_part,
                        self.robot_editor.capsule_editor.radius,
                    );
                    

                    self.robot_editor.update(
                        ui,
                        ctx,
                        pointer_pos,
                        self.editing_part,
                        self.editing_state,
                        &response,
                    );
                });

            match self.editing_part {
                EditingPart::Capsule => {
                    let slider_response = ui.add(
                        Slider::new(&mut self.robot_editor.capsule_editor.radius, 10.0..=30.0)
                            .text("Capsule radius"),
                    );

                    if slider_response.changed() {
                        self.robot_editor
                            .capsule_editor
                            .on_capsule_radius_slider_changed(
                                self.robot_editor.capsule_editor.radius,
                            );
                    }

                    if slider_response.hovered() {
                        ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
                    }
                }
                EditingPart::Joint => {
                    // Add joint-specific controls if needed
                }
            }
        });
    }
}

fn main() -> Result<(), eframe::Error> {
    env_logger::init();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([800.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Robot Designer App",
        options,
        Box::new(|_cc| Box::new(RobotDesignerApp::default())),
    )
}
