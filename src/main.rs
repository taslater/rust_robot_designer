mod editor;
mod model;
mod robot_renderer;

use eframe::egui;
use egui::{Color32, Frame, Slider, Stroke};

use editor::capsule_editor::CapsuleEditor;
use editor::common::EditingState;
use editor::joint_editor::JointEditor;
use crate::editor::common::RobotPart;
use robot_renderer::RobotRenderer;

pub struct RobotDesignerApp {
    capsule_editor: CapsuleEditor,
    joint_editor: JointEditor,
    robot_part: RobotPart,
    editing_state: EditingState,
    robot_renderer: RobotRenderer,
}

impl RobotDesignerApp {
    fn on_editing_state_changed(&mut self) {
        println!("State changed to {:?} with robot part {:?}", self.editing_state, self.robot_part);
        match self.robot_part {
            RobotPart::Capsule => {
                self.capsule_editor
                    .on_editing_state_changed(self.editing_state);
            }
            RobotPart::Joint => {
                self.capsule_editor.clear_capsule_selection();
                self.joint_editor
                    .on_editing_state_changed(self.editing_state);
                println!("Joint editor selected capsules: {:?}", self.joint_editor.selected_capsules);
            }
        }
    }
}

impl Default for RobotDesignerApp {
    fn default() -> Self {
        RobotDesignerApp {
            capsule_editor: CapsuleEditor::default(),
            joint_editor: JointEditor::default(),
            robot_part: RobotPart::Capsule,
            editing_state: EditingState::Create,
            // capsule_radius: 20.0,
            robot_renderer: RobotRenderer::new(),
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
                    .radio_value(&mut self.robot_part, RobotPart::Capsule, "Capsule")
                    .clicked();

                let has_overlapping_capsules =
                    self.capsule_editor.capsules.windows(2).any(|window| {
                        let capsule1 = &window[0];
                        let capsule2 = &window[1];
                        capsule1.intersects_capsule(capsule2)
                    });

                ui.scope(|ui| {
                    ui.set_enabled(has_overlapping_capsules);
                    let joint_clicked = ui
                        .radio_value(&mut self.robot_part, RobotPart::Joint, "Joint")
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

                    let capsule_render_data = self
                        .capsule_editor
                        .get_capsule_render_data(pointer_pos, self.editing_state);

                    self.robot_renderer.set_capsules(
                        self.capsule_editor.capsules.clone(),
                        self.capsule_editor.create_capsule_start_point,
                        capsule_render_data,
                        self.capsule_editor.overlapping_capsules.clone(),
                    );
                    self.robot_renderer
                        .set_joints(self.joint_editor.joints.clone());
                    self.robot_renderer.draw(
                        &painter,
                        self.editing_state,
                        pointer_pos,
                        self.capsule_editor.radius,
                    );

                    match self.robot_part {
                        RobotPart::Capsule => {
                            self.capsule_editor.update(
                                ui,
                                ctx,
                                &mut self.editing_state,
                                pointer_pos,
                                &response,
                            );
                        }
                        RobotPart::Joint => {
                            self.joint_editor.update(
                                ui,
                                // ctx,
                                &mut self.editing_state,
                                pointer_pos,
                                &response,
                                &self.capsule_editor.capsules,
                            );
                        }
                    }
                });

            match self.robot_part {
                RobotPart::Capsule => {
                    let slider_response = ui.add(
                        Slider::new(&mut self.capsule_editor.radius, 10.0..=30.0)
                            .text("Capsule radius"),
                    );

                    if slider_response.changed() {
                        self.capsule_editor
                            .on_capsule_radius_slider_changed(self.capsule_editor.radius);
                    }

                    if slider_response.hovered() {
                        ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
                    }
                }
                RobotPart::Joint => {
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
