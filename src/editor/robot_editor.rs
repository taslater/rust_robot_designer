use std::cell::RefCell;
use std::rc::Rc;
use super::common::{EditingPart, EditingState};
use super::capsule_editor::CapsuleEditor;
use super::joint_editor::JointEditor;
use crate::model::robot::Robot;
use eframe::egui;
use egui::Pos2;

pub struct RobotEditor {
    pub capsule_editor: CapsuleEditor,
    pub joint_editor: JointEditor,
}

impl RobotEditor {
    pub fn new(robot: Rc<RefCell<Robot>>) -> Self {
        RobotEditor {
            capsule_editor: CapsuleEditor::new(robot.clone()),
            joint_editor: JointEditor::new(robot),
        }
    }

    pub fn draw_editor(
        &self,
        painter: &egui::Painter,
        robot: &Robot,
        pointer_pos: Pos2,
        editing_state: EditingState,
        editing_part: EditingPart,
        capsule_radius: f32,
    ) {
        match editing_part {
            EditingPart::Capsule => {
                self.capsule_editor.draw_editor(
                    painter,
                    robot,
                    pointer_pos,
                    editing_state,
                    capsule_radius,
                );
            }
            EditingPart::Joint => {
                self.joint_editor.draw_editor(painter, robot, pointer_pos, editing_state);
            }
        }
    }

    pub fn update(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        pointer_pos: Pos2,
        editing_part: EditingPart,
        mut editing_state: EditingState,
        response: &egui::Response,
        // robot: &mut Robot,
    ) {
        match editing_part {
            EditingPart::Capsule => {
                self.capsule_editor
                    .update(ui, ctx, &mut editing_state, pointer_pos, response);
            }
            EditingPart::Joint => {
                self.joint_editor.update(ui, &mut editing_state, pointer_pos, response);
            }
        }
    }
}

