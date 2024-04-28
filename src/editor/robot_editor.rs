use super::capsule_editor::CapsuleEditor;
use super::common::{EditingPart, EditingState};
use super::joint_editor::JointEditor;
use crate::model::robot::Robot;
use eframe::egui;
use egui::Pos2;

pub struct RobotEditor {
    pub capsule_editor: CapsuleEditor,
    pub joint_editor: JointEditor,
    pub editing_part: EditingPart,
    pub editing_state: EditingState,
}

impl RobotEditor {
    pub fn new() -> Self {
        RobotEditor {
            capsule_editor: CapsuleEditor::new(),
            joint_editor: JointEditor::new(),
            editing_part: EditingPart::Capsule,
            editing_state: EditingState::Create,
        }
    }

    pub fn draw_editor_ui(&mut self, ui: &mut egui::Ui, robot: &mut Robot, ctx: &egui::Context) {
        ui.horizontal(|ui| {
            ui.label("Part Type:");
            let robot_clicked = ui
                .radio_value(&mut self.editing_part, EditingPart::Capsule, "Capsule")
                .clicked();

            let has_overlapping_capsules = robot.has_overlapping_capsules();

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

        egui::Frame::canvas(ui.style()).show(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(egui::Vec2::new(400.0, 300.0), egui::Sense::click_and_drag());

            let pointer_pos = response.hover_pos().unwrap_or_default();

            self.draw_editor(
                &painter,
                pointer_pos,
                self.editing_state,
                self.editing_part,
                self.capsule_editor.radius,
                robot,
            );

            self.update(ui, ctx, pointer_pos, &response, robot);
        });

        match self.editing_part {
            EditingPart::Capsule => {
                let slider_response = ui.add(
                    egui::Slider::new(&mut self.capsule_editor.radius, 10.0..=30.0)
                        .text("Capsule radius"),
                );

                if slider_response.changed() {
                    self.capsule_editor
                        .on_capsule_radius_slider_changed(self.capsule_editor.radius, robot);
                }
            }
            EditingPart::Joint => {
                // Add joint-specific controls if needed
            }
        }
    }

    fn on_editing_state_changed(&mut self) {
        match self.editing_part {
            EditingPart::Capsule => {
                self.capsule_editor
                    .on_editing_state_changed(self.editing_state);
            }
            EditingPart::Joint => {
                self.capsule_editor.clear_capsule_selection();
                self.joint_editor
                    .on_editing_state_changed(self.editing_state);
            }
        }
    }

    pub fn draw_editor(
        &self,
        painter: &egui::Painter,
        pointer_pos: Pos2,
        editing_state: EditingState,
        editing_part: EditingPart,
        capsule_radius: f32,
        robot: &mut Robot,
    ) {
        match editing_part {
            EditingPart::Capsule => {
                self.capsule_editor.draw_editor(
                    painter,
                    pointer_pos,
                    editing_state,
                    capsule_radius,
                    robot,
                );
            }
            EditingPart::Joint => {
                self.joint_editor
                    .draw_editor(painter, pointer_pos, editing_state, robot);
            }
        }
    }

    pub fn update(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        pointer_pos: Pos2,
        response: &egui::Response,
        robot: &mut Robot,
    ) {
        match self.editing_part {
            EditingPart::Capsule => {
                self.capsule_editor
                    .update(ui, ctx, &mut self.editing_state, pointer_pos, response, robot);
            }
            EditingPart::Joint => {
                self.joint_editor
                    .update(ui, &mut self.editing_state, pointer_pos, response, robot);
            }
        }
    }
}
