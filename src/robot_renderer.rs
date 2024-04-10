use crate::editor::common::EditingState;
use crate::model::capsule::{Capsule, PointInsideCapsule};
use crate::model::joint::Joint;
use eframe::egui::{Color32, Pos2, Stroke};
use egui::epaint::Shape;

use crate::editor::common::RobotPart;

#[derive(Debug, Clone, Copy)]
pub enum CapsuleColors {
    Add,
    Default,
    Highlighted,
    Selected,
}

impl CapsuleColors {
    fn to_color32(&self) -> Color32 {
        match self {
            CapsuleColors::Add => Color32::from_rgb(230, 230, 250), // Lavender
            CapsuleColors::Default => Color32::GREEN,
            CapsuleColors::Highlighted => Color32::LIGHT_BLUE,
            CapsuleColors::Selected => Color32::LIGHT_RED,
        }
    }
}

pub struct RobotRenderer {
    capsules: Vec<Capsule>,
    joints: Vec<Joint>,
    create_capsule_start_point: Option<Pos2>,
    selected_capsule_points: Vec<(usize, String, f32, f32)>,
    overlapping_capsules: Vec<(usize, usize)>,
}

impl RobotRenderer {
    pub fn new() -> Self {
        RobotRenderer {
            capsules: Vec::new(),
            joints: Vec::new(),
            create_capsule_start_point: None,
            selected_capsule_points: Vec::new(),
            overlapping_capsules: Vec::new(),
        }
    }

    pub fn set_capsules(
        &mut self,
        capsules: Vec<Capsule>,
        create_capsule_start_point: Option<Pos2>,
        selected_capsule_points: Vec<(usize, String, f32, f32)>,
        overlapping_capsules: Vec<(usize, usize)>,
    ) {
        self.capsules = capsules;
        self.create_capsule_start_point = create_capsule_start_point;
        self.selected_capsule_points = selected_capsule_points;
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
        robot_part: RobotPart
    ) {
        for capsule in &self.capsules {
            let (circle1_color, circle2_color, body_color) =
                self.get_capsule_colors(capsule, pointer_pos, editing_state, robot_part);
            capsule.draw(
                painter,
                circle1_color.to_color32(),
                circle2_color.to_color32(),
                body_color.to_color32(),
            );
        }

        if editing_state == EditingState::Create {
            self.render_add_capsule(painter, pointer_pos, radius);
        }

        for joint in &self.joints {
            joint.draw(painter, Color32::BLUE);
        }
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

    fn get_capsule_colors(
        &self,
        capsule: &Capsule,
        pointer_pos: Pos2,
        editing_state: EditingState,
        robot_part: RobotPart,
    ) -> (CapsuleColors, CapsuleColors, CapsuleColors) {
        if robot_part == RobotPart::Joint {
            return (
                CapsuleColors::Default,
                CapsuleColors::Default,
                CapsuleColors::Default,
            );
        }

        match editing_state {
            EditingState::Create => (
                CapsuleColors::Default,
                CapsuleColors::Default,
                CapsuleColors::Default,
            ),
            EditingState::Delete => {
                if capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y) {
                    (
                        CapsuleColors::Highlighted,
                        CapsuleColors::Highlighted,
                        CapsuleColors::Highlighted,
                    )
                } else {
                    (
                        CapsuleColors::Default,
                        CapsuleColors::Default,
                        CapsuleColors::Default,
                    )
                }
            }
            EditingState::Update => {
                let is_endcap1_selected = self
                    .selected_capsule_points
                    .iter()
                    .any(|(id, point, _, _)| *id == capsule.id && point == "x1");
                let is_endcap2_selected = self
                    .selected_capsule_points
                    .iter()
                    .any(|(id, point, _, _)| *id == capsule.id && point == "x2");
                let inside_detail = capsule.is_inside_detail(pointer_pos.x, pointer_pos.y);

                let circle1_color = if is_endcap1_selected {
                    CapsuleColors::Selected
                } else if inside_detail == PointInsideCapsule::InsideEndcap1 {
                    CapsuleColors::Highlighted
                } else {
                    CapsuleColors::Default
                };

                let circle2_color = if is_endcap2_selected {
                    CapsuleColors::Selected
                } else if inside_detail == PointInsideCapsule::InsideEndcap2 {
                    CapsuleColors::Highlighted
                } else {
                    CapsuleColors::Default
                };

                let body_color = if is_endcap1_selected && is_endcap2_selected {
                    CapsuleColors::Selected
                } else if inside_detail == PointInsideCapsule::InsideBody {
                    CapsuleColors::Highlighted
                } else {
                    CapsuleColors::Default
                };

                (circle1_color, circle2_color, body_color)
            }
        }
    }
}
