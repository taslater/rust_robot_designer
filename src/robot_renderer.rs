use crate::editor::common::{EditingState, EditingPart};
use crate::editor::robot_editor::RobotEditor;
use crate::model::capsule::Capsule;
use crate::model::joint::Joint;
use crate::model::robot::Robot;
use eframe::egui::{Color32, Pos2, Stroke};
use egui::epaint::Shape;

use crate::editor::capsule_editor::{CapsuleColors, CapsuleRenderData, OverlappingCapsules};

pub struct RobotRenderer {
    pub robot: Robot,
    create_capsule_start_point: Option<Pos2>,
    capsule_render_data: Vec<CapsuleRenderData>,
    overlapping_capsules: Vec<OverlappingCapsules>,
    selected_capsule_ids: Vec<usize>,
}

impl RobotRenderer {
    pub fn new() -> Self {
        RobotRenderer {
            robot: Robot::new(),
            create_capsule_start_point: None,
            capsule_render_data: Vec::new(),
            overlapping_capsules: Vec::new(),
            selected_capsule_ids: Vec::new(),
        }
    }

    pub fn set_joint_data(
        &mut self,
        selected_capsule_ids: Vec<usize>,
    ) {
        self.selected_capsule_ids = selected_capsule_ids;
    }

    pub fn set_capsules(
        &mut self,
        capsules: Vec<Capsule>,
        create_capsule_start_point: Option<Pos2>,
        capsule_render_data: Vec<CapsuleRenderData>,
        overlapping_capsules: Vec<OverlappingCapsules>,
    ) {
        // self.capsules = capsules;
        self.robot.capsules = capsules;
        self.create_capsule_start_point = create_capsule_start_point;
        self.capsule_render_data = capsule_render_data;
        self.overlapping_capsules = overlapping_capsules;
    }

    pub fn set_joints(&mut self, joints: Vec<Joint>) {
        // self.joints = joints;
        self.robot.joints = joints;
    }

    // pub fn draw(
    //     &self,
    //     painter: &egui::Painter,
    //     editing_state: EditingState,
    //     pointer_pos: Pos2,
    //     radius: f32,
    //     capsule_editor: &CapsuleEditor,
    //     robot_part: RobotPart,
    // ) {
    //     for (i, data) in self.capsule_render_data.iter().enumerate() {
    //         let is_selected = self.selected_capsule_ids.contains(&i);
    //         let colors = capsule_editor.get_capsule_colors(
    //             &data.capsule,
    //             pointer_pos,
    //             editing_state,
    //             is_selected,
    //         );
    //         data.capsule.draw(
    //             painter,
    //             colors.0.to_color32(),
    //             colors.1.to_color32(),
    //             colors.2.to_color32(),
    //         );
    //     }
    
    //     match robot_part {
    //         RobotPart::Capsule => {
    //             if editing_state == EditingState::Create {
    //                 self.render_add_capsule(painter, pointer_pos, radius);
    //             }
    //         }
    //         RobotPart::Joint => {
    //             if editing_state == EditingState::Create {
    //                 let stroke = Stroke::new(2.0, Color32::GREEN);
    //                 let radius = 5.0;
    //                 painter.circle_stroke(pointer_pos, radius, stroke);
    //             }
    //         }
    //     }
    
    //     for joint in &self.joints {
    //         joint.draw(painter, Color32::BLUE);
    //     }
    
    //     self.render_overlapping_capsules(painter);
    // }
    pub fn draw(
        &self,
        painter: &egui::Painter,
        robot: &Robot,
        robot_editor: &RobotEditor,
        editing_state: EditingState,
        editing_part: EditingPart,
        pointer_pos: Pos2,
        capsule_radius: f32,
    ) {
        robot.draw(painter);
        robot_editor.draw_editor(painter, robot, pointer_pos, editing_state, editing_part, capsule_radius);
    }

    fn render_add_capsule(&self, painter: &egui::Painter, pointer_pos: Pos2, radius: f32) {
        if let Some(start_point) = self.create_capsule_start_point {
            let stroke = Stroke::new(10.0, CapsuleColors::Add.to_color32());
            painter.line_segment([start_point, pointer_pos], stroke);

            let fill = CapsuleColors::Add.to_color32();
            painter.add(Shape::circle_filled(start_point, radius, fill));
            painter.add(Shape::circle_filled(pointer_pos, radius, fill));
        } else {
            let fill = CapsuleColors::Add.to_color32();
            painter.add(Shape::circle_filled(pointer_pos, radius, fill));
        }
    }

    fn render_overlapping_capsules(&self, painter: &egui::Painter) {
        for overlapping in &self.overlapping_capsules {
            if let (Some(capsule1), Some(capsule2)) = (
                // self.capsules.get(overlapping.capsule1_id),
                // self.capsules.get(overlapping.capsule2_id),
                self.robot.capsules.iter().find(|c| c.id == overlapping.capsule1_id),
                self.robot.capsules.iter().find(|c| c.id == overlapping.capsule2_id),
            ) {
                let stroke = Stroke::new(2.0, Color32::RED);
                painter.line_segment([capsule1.get_center(), capsule2.get_center()], stroke);
            }
        }
    }
}
