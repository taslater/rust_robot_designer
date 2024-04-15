// robot.rs
use crate::editor::capsule_editor::{OverlappingCapsules, SelectedCapsulePoint};
use crate::model::capsule::{Capsule, CapsuleColors, PointInsideCapsule, PointType};
use crate::model::joint::Joint;
use eframe::egui;
use egui::{Color32, Pos2};

#[derive(Debug)]
pub struct Robot {
    capsules: Vec<Capsule>,
    joints: Vec<Joint>,
}

impl Robot {
    pub fn new() -> Self {
        Robot {
            capsules: Vec::new(),
            joints: Vec::new(),
        }
    }

    pub fn get_capsule_count(&self) -> usize {
        self.capsules.len()
    }

    pub fn add_capsule(&mut self, start_point: Pos2, pointer_pos: Pos2, radius: f32) {
        let capsule = Capsule {
            id: self.get_capsule_count(),
            radius: radius,
            x1: start_point.x,
            y1: start_point.y,
            x2: pointer_pos.x,
            y2: pointer_pos.y,
        };
        self.capsules.push(capsule);
    }

    // pub fn remove_capsule(&mut self, capsule_id: usize) {
    //     self.capsules.retain(|capsule| capsule.id != capsule_id);
    // }

    pub fn remove_capsules_by_point(&mut self, x: f32, y: f32) {
        self.capsules
            .retain(|capsule| !capsule.is_inside_at_all(x, y));
    }

    // pub fn update_capsule(&mut self, capsule: Capsule) {
    //     if let Some(c) = self.capsules.iter_mut().find(|c| c.id == capsule.id) {
    //         *c = capsule;
    //     }
    // }

    pub fn update_capsule_endcap(&mut self, capsule_id: usize, point_type: PointType, x: f32, y: f32) {
        if let Some(capsule) = self.capsules.get_mut(capsule_id) {
            match point_type {
                PointType::Pt1 => {
                    capsule.x1 = x;
                    capsule.y1 = y;
                }
                PointType::Pt2 => {
                    capsule.x2 = x;
                    capsule.y2 = y;
                }
            }
        }
    }

    pub fn update_capsule_body(&mut self, capsule_id: usize, dx: f32, dy: f32) {
        if let Some(capsule) = self.capsules.get_mut(capsule_id) {
            capsule.x1 += dx;
            capsule.y1 += dy;
            capsule.x2 += dx;
            capsule.y2 += dy;
        }
    }

    pub fn update_capsule_radius(&mut self, capsule_id: usize, radius: f32) {
        if let Some(capsule) = self.capsules.get_mut(capsule_id) {
            capsule.radius = radius;
        }
    }

    pub fn get_capsule(&self, capsule_id: usize) -> Option<&Capsule> {
        self.capsules.iter().find(|c| c.id == capsule_id)
    }

    pub fn get_capsule_point(&self, capsule_id: usize, point_type: &str) -> Option<(f32, f32)> {
        if let Some(capsule) = self.capsules.get(capsule_id) {
            match point_type {
                "x1" => Some((capsule.x1, capsule.y1)),
                "x2" => Some((capsule.x2, capsule.y2)),
                _ => None,
            }
        } else {
            None
        }
    }

    pub fn has_overlapping_capsules(&self) -> bool {
        self.capsules.windows(2).any(|window| {
            let capsule1 = &window[0];
            let capsule2 = &window[1];
            capsule1.intersects_capsule(capsule2)
        })
    }

    fn find_capsule_points_at_position_internal(&self, pointer_pos: Pos2) -> Vec<(usize, String)> {
        self.capsules
            .iter()
            .enumerate()
            .flat_map(|(index, capsule)| {
                let inside_detail = capsule.is_inside_detail(pointer_pos.x, pointer_pos.y);
                match inside_detail {
                    PointInsideCapsule::InsideEndcap1 => vec![(index, "x1".to_string())],
                    PointInsideCapsule::InsideEndcap2 => vec![(index, "x2".to_string())],
                    PointInsideCapsule::InsideBody => {
                        vec![(index, "x1".to_string()), (index, "x2".to_string())]
                    }
                    _ => vec![],
                }
            })
            .collect()
    }

    pub fn find_capsule_points_at_position(&self, pointer_pos: Pos2) -> Vec<SelectedCapsulePoint> {
        self.find_capsule_points_at_position_internal(pointer_pos)
            .iter()
            .map(|(index, point_type)| {
                let capsule = &self.capsules[*index];
                SelectedCapsulePoint {
                    capsule_id: capsule.id,
                    point_type: point_type.clone(),
                    x: if point_type == "x1" {
                        capsule.x1
                    } else {
                        capsule.x2
                    },
                    y: if point_type == "x1" {
                        capsule.y1
                    } else {
                        capsule.y2
                    },
                }
            })
            .collect()
    }

    pub fn find_hovered_capsule_points(&self, pointer_pos: Pos2) -> Vec<usize> {
        self.find_capsule_points_at_position_internal(pointer_pos)
            .iter()
            .map(|(index, point_type)| index * 2 + if point_type == "x1" { 0 } else { 1 })
            .collect()
    }

    pub fn get_overlapping_capsules(&self) -> Vec<OverlappingCapsules> {
        let mut overlapping_capsules: Vec<OverlappingCapsules> = Vec::new();
        for (i, capsule1) in self.capsules.iter().enumerate() {
            for (j, capsule2) in self.capsules.iter().enumerate() {
                if i < j && capsule1.intersects_capsule(capsule2) {
                    overlapping_capsules.push(OverlappingCapsules {
                        capsule1_id: i,
                        capsule2_id: j,
                    });
                }
            }
        }
        overlapping_capsules
    }

    pub fn check_joint_placement(&self, pointer_pos: Pos2, selected_capsules: &[usize]) -> bool {
        if let Some(capsule1) = selected_capsules
            .get(0)
            .and_then(|&id| self.capsules.get(id))
        {
            if let Some(capsule2) = selected_capsules
                .get(1)
                .and_then(|&id| self.capsules.get(id))
            {
                return capsule1.is_inside_at_all(pointer_pos.x, pointer_pos.y)
                    && capsule2.is_inside_at_all(pointer_pos.x, pointer_pos.y);
            }
        }
        false
    }

    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn remove_joint(&mut self, pointer_pos: Pos2) {
        self.joints
            .retain(|joint| !joint.is_inside(pointer_pos.x, pointer_pos.y));
    }

    // pub fn update_joint(&mut self, joint: Joint) {
    //     if let Some(j) = self.joints.iter_mut().find(|j| j.id == joint.id) {
    //         *j = joint;
    //     }
    // }

    fn find_items_by_point<T>(&self, items: &[T], pointer_pos: Pos2) -> Vec<usize>
    where
        T: HasPosition,
    {
        items
            .iter()
            .enumerate()
            .filter(|(_, item)| item.is_inside(pointer_pos.x, pointer_pos.y))
            .map(|(index, _)| index)
            .collect()
    }

    pub fn find_capsules_by_point(&self, pointer_pos: Pos2) -> Vec<usize> {
        self.find_items_by_point(&self.capsules, pointer_pos)
    }

    pub fn find_joints_by_point(&self, pointer_pos: Pos2) -> Vec<usize> {
        self.find_items_by_point(&self.joints, pointer_pos)
    }

    pub fn get_joint_count(&self) -> usize {
        self.joints.len()
    }

    pub fn draw(
        &self,
        painter: &egui::Painter,
        selected_capsule_points: &[usize],
        hovered_capsule_points: &[usize],
        selected_joints: &[usize],
        hovered_joints: &[usize],
    ) {
        for (index, capsule) in self.capsules.iter().enumerate() {
            let (circle1_color, circle2_color, body_color) =
                self.get_capsule_colors(index, selected_capsule_points, hovered_capsule_points);
            capsule.draw(painter, circle1_color, circle2_color, body_color);
        }

        for (index, joint) in self.joints.iter().enumerate() {
            let color = self.get_joint_color(index, selected_joints, hovered_joints);
            joint.draw(painter, color);
        }
    }

    fn get_capsule_colors(
        &self,
        capsule_index: usize,
        selected_capsule_points: &[usize],
        hovered_capsule_points: &[usize],
    ) -> (Color32, Color32, Color32) {
        let is_endcap1_selected = selected_capsule_points.contains(&(capsule_index * 2));
        let is_endcap2_selected = selected_capsule_points.contains(&(capsule_index * 2 + 1));
        let is_endcap1_hovered = hovered_capsule_points.contains(&(capsule_index * 2));
        let is_endcap2_hovered = hovered_capsule_points.contains(&(capsule_index * 2 + 1));

        let endcap1_color = if is_endcap1_selected {
            CapsuleColors::Selected
        } else if is_endcap1_hovered {
            CapsuleColors::Highlighted
        } else {
            CapsuleColors::Default
        };

        let endcap2_color = if is_endcap2_selected {
            CapsuleColors::Selected
        } else if is_endcap2_hovered {
            CapsuleColors::Highlighted
        } else {
            CapsuleColors::Default
        };

        let body_color = if is_endcap1_selected && is_endcap2_selected {
            CapsuleColors::Selected
        } else if is_endcap1_hovered && is_endcap2_hovered {
            CapsuleColors::Highlighted
        } else {
            CapsuleColors::Default
        };

        (
            endcap1_color.to_color32(),
            endcap2_color.to_color32(),
            body_color.to_color32(),
        )
    }

    fn get_joint_color(
        &self,
        joint_index: usize,
        selected_joints: &[usize],
        hovered_joints: &[usize],
    ) -> Color32 {
        if selected_joints.contains(&joint_index) {
            Color32::LIGHT_RED
        } else if hovered_joints.contains(&joint_index) {
            Color32::LIGHT_BLUE
        } else {
            Color32::BLUE
        }
    }
}


trait HasPosition {
    fn is_inside(&self, x: f32, y: f32) -> bool;
}

impl HasPosition for Capsule {
    fn is_inside(&self, x: f32, y: f32) -> bool {
        self.is_inside_at_all(x, y)
    }
}

impl HasPosition for Joint {
    fn is_inside(&self, x: f32, y: f32) -> bool {
        self.is_inside(x, y)
    }
}