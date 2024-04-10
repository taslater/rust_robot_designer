use crate::editor::common::EditingState;
use crate::model::capsule::Capsule;
use crate::model::joint::Joint;
use eframe::egui::{Color32, Pos2, Stroke};
use egui::epaint::Shape;

use crate::editor::capsule_editor::{CapsuleColors, CapsuleRenderData, OverlappingCapsules};

pub struct RobotRenderer {
    capsules: Vec<Capsule>,
    joints: Vec<Joint>,
    create_capsule_start_point: Option<Pos2>,
    capsule_render_data: Vec<CapsuleRenderData>,
    overlapping_capsules: Vec<OverlappingCapsules>,
}

impl RobotRenderer {
    pub fn new() -> Self {
        RobotRenderer {
            capsules: Vec::new(),
            joints: Vec::new(),
            create_capsule_start_point: None,
            capsule_render_data: Vec::new(),
            overlapping_capsules: Vec::new(),
        }
    }

    pub fn set_capsules(
        &mut self,
        capsules: Vec<Capsule>,
        create_capsule_start_point: Option<Pos2>,
        capsule_render_data: Vec<CapsuleRenderData>,
        overlapping_capsules: Vec<OverlappingCapsules>,
    ) {
        self.capsules = capsules;
        self.create_capsule_start_point = create_capsule_start_point;
        self.capsule_render_data = capsule_render_data;
        self.overlapping_capsules = overlapping_capsules;
    }

    pub fn set_joints(&mut self, joints: Vec<Joint>) {
        self.joints = joints;
    }

    pub fn draw(
        &self,
        painter: &egui::Painter,
        editing_state: EditingState,
        pointer_pos: Pos2,
        radius: f32,
    ) {
        for data in &self.capsule_render_data {
            data.capsule.draw(
                painter,
                data.circle1_color.to_color32(),
                data.circle2_color.to_color32(),
                data.body_color.to_color32(),
            );
        }

        if editing_state == EditingState::Create {
            self.render_add_capsule(painter, pointer_pos, radius);
        }

        for joint in &self.joints {
            joint.draw(painter, Color32::BLUE);
        }

        self.render_overlapping_capsules(painter);
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
                self.capsules.get(overlapping.capsule1_id),
                self.capsules.get(overlapping.capsule2_id),
            ) {
                let stroke = Stroke::new(2.0, Color32::RED);
                painter.line_segment([capsule1.get_center(), capsule2.get_center()], stroke);
            }
        }
    }
}
