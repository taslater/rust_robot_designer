// robot.rs
use super::capsule::{
    Capsule, CapsuleColors, CapsulePoint, CapsulePointId, PointInsideCapsule, PointType,
    SelectionLevel,
};
use crate::editor::capsule_editor::OverlappingCapsules;
use crate::model::joint::Joint;
use eframe::egui;
use egui::{Color32, Pos2};

use std::sync::atomic::{AtomicUsize, Ordering};

static CAPSULE_ID_COUNTER: AtomicUsize = AtomicUsize::new(1);

fn generate_capsule_id() -> usize {
    CAPSULE_ID_COUNTER.fetch_add(1, Ordering::SeqCst)
}

#[derive(Debug, Clone)]
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

    pub fn get_capsules(&self) -> &Vec<Capsule> {
        &self.capsules
    }

    // pub fn get_capsule(&self, capsule_id: usize) -> Option<&Capsule> {
    //     self.capsules.get(capsule_id)
    // }

    pub fn get_capsules_mut(&mut self) -> &mut Vec<Capsule> {
        &mut self.capsules
    }

    // pub fn get_capsule_mut(&mut self, capsule_id: usize) -> Option<&mut Capsule> {
    //     self.capsules.get_mut(capsule_id)
    // }

    pub fn get_joints(&self) -> &Vec<Joint> {
        &self.joints
    }

    // pub fn get_joint(&self, joint_id: usize) -> Option<&Joint> {
    //     self.joints.get(joint_id)
    // }

    // pub fn get_joint_mut(&mut self, joint_id: usize) -> Option<&mut Joint> {
    //     self.joints.get_mut(joint_id)
    // }

    pub fn add_capsule(&mut self, start_point: Pos2, pointer_pos: Pos2, radius: f32) {
        let capsule_id = generate_capsule_id();
        let capsule = Capsule {
            id: capsule_id.clone(),
            radius,
            point1: CapsulePoint {
                capsule_point_id: CapsulePointId {
                    capsule_id: capsule_id.clone(),
                    point_type: PointType::Pt1,
                },
                x: start_point.x,
                y: start_point.y,
            },
            point2: CapsulePoint {
                capsule_point_id: CapsulePointId {
                    capsule_id: capsule_id.clone(),
                    point_type: PointType::Pt2,
                },
                x: pointer_pos.x,
                y: pointer_pos.y,
            },
            length: f32::sqrt(
                (pointer_pos.x - start_point.x).powi(2) + (pointer_pos.y - start_point.y).powi(2),
            ),
            rotation_offset: f32::atan2(pointer_pos.y - start_point.y, pointer_pos.x - start_point.x),
        };
        self.capsules.push(capsule);
    }

    pub fn remove_capsules_by_point(&mut self, x: f32, y: f32) {
        self.capsules
            .retain(|capsule| !capsule.is_inside_at_all(x, y));
    }

    pub fn update_capsule_point_pos(&mut self, capsule_point_id: CapsulePointId, x: f32, y: f32) {
        if let Some(capsule) = self
            .capsules
            .iter_mut()
            .find(|c| c.id == capsule_point_id.capsule_id)
        {
            match capsule_point_id.point_type {
                PointType::Pt1 => {
                    capsule.point1.x = x;
                    capsule.point1.y = y;
                }
                PointType::Pt2 => {
                    capsule.point2.x = x;
                    capsule.point2.y = y;
                }
            }
            capsule.length = f32::sqrt((capsule.point2.x - capsule.point1.x).powi(2) + (capsule.point2.y - capsule.point1.y).powi(2));
            capsule.rotation_offset = f32::atan2(capsule.point2.y - capsule.point1.y, capsule.point2.x - capsule.point1.x);
        }
    }

    pub fn update_capsule_radius(&mut self, capsule_id: usize, radius: f32) {
        if let Some(capsule) = self.capsules.get_mut(capsule_id) {
            capsule.radius = radius;
        }
    }

    pub fn get_capsule_point_pos(&self, capsule_point_id: CapsulePointId) -> Option<Pos2> {
        if let Some(capsule) = self
            .capsules
            .iter()
            .find(|c| c.id == capsule_point_id.capsule_id)
        {
            match capsule_point_id.point_type {
                PointType::Pt1 => Some(Pos2::new(capsule.point1.x, capsule.point1.y)),
                PointType::Pt2 => Some(Pos2::new(capsule.point2.x, capsule.point2.y)),
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

    fn find_capsule_points_at_position_internal(&self, pointer_pos: Pos2) -> Vec<CapsulePointId> {
        self.capsules
            .iter()
            .flat_map(|capsule| {
                let inside_detail = capsule.is_inside_detail(pointer_pos.x, pointer_pos.y);
                match inside_detail {
                    PointInsideCapsule::InsideEndcap1 => vec![CapsulePointId {
                        capsule_id: capsule.id,
                        point_type: PointType::Pt1,
                    }],
                    PointInsideCapsule::InsideEndcap2 => vec![CapsulePointId {
                        capsule_id: capsule.id,
                        point_type: PointType::Pt2,
                    }],
                    PointInsideCapsule::InsideBody => {
                        vec![
                            CapsulePointId {
                                capsule_id: capsule.id,
                                point_type: PointType::Pt1,
                            },
                            CapsulePointId {
                                capsule_id: capsule.id,
                                point_type: PointType::Pt2,
                            },
                        ]
                    }
                    _ => vec![],
                }
            })
            .collect()
    }

    pub fn find_capsule_points_at_position(&self, pointer_pos: Pos2) -> Vec<CapsulePoint> {
        self.find_capsule_points_at_position_internal(pointer_pos)
            .iter()
            .filter_map(
                |CapsulePointId {
                     capsule_id,
                     point_type,
                 }| {
                    // self.get_capsule(*capsule_id).and_then(|capsule| {
                    self.capsules
                        .iter()
                        .filter(|c| c.id == *capsule_id)
                        .find_map(|capsule| {
                            let point = match point_type {
                                PointType::Pt1 => &capsule.point1,
                                PointType::Pt2 => &capsule.point2,
                            };
                            Some(CapsulePoint {
                                capsule_point_id: CapsulePointId {
                                    capsule_id: *capsule_id,
                                    point_type: *point_type,
                                },
                                x: point.x,
                                y: point.y,
                            })
                        })
                },
            )
            .collect()
    }

    pub fn find_hovered_capsule_points(&self, pointer_pos: Pos2) -> Vec<CapsulePointId> {
        self.find_capsule_points_at_position_internal(pointer_pos)
            .iter()
            .map(
                |CapsulePointId {
                     capsule_id,
                     point_type,
                 }| CapsulePointId {
                    capsule_id: *capsule_id,
                    point_type: *point_type,
                },
            )
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
            .and_then(|&id| self.capsules.iter().find(|c| c.id == id))
        {
            if let Some(capsule2) = selected_capsules
                .get(1)
                .and_then(|&id| self.capsules.iter().find(|c| c.id == id))
            {
                return capsule1.is_inside_at_all(pointer_pos.x, pointer_pos.y)
                    && capsule2.is_inside_at_all(pointer_pos.x, pointer_pos.y);
            }
        }
        false
    }

    pub fn are_capsules_already_joined(&self, capsule1_id: usize, capsule2_id: usize) -> bool {
        self.joints.iter().any(|joint| {
            (joint.capsule1_id == capsule1_id && joint.capsule2_id == capsule2_id)
                || (joint.capsule1_id == capsule2_id && joint.capsule2_id == capsule1_id)
        })
    }

    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn remove_joint(&mut self, pointer_pos: Pos2) {
        self.joints
            .retain(|joint| !joint.is_inside(pointer_pos.x, pointer_pos.y));
    }

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

    pub fn find_capsules_by_point(&self, pointer_pos: Pos2) -> Vec<CapsulePointId> {
        self.find_capsule_points_at_position_internal(pointer_pos)
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
        selected_capsule_points: &Vec<CapsulePointId>,
        hovered_capsule_points: &Vec<CapsulePointId>,
        selected_joints: &Vec<usize>,
        hovered_joints: &Vec<usize>,
    ) {
        for capsule in self.capsules.iter() {
            let mut selection_level1 = SelectionLevel::None;
            let mut selection_level2 = SelectionLevel::None;
            let mut selection_level_body = SelectionLevel::None;

            for CapsulePointId {
                capsule_id,
                point_type,
            } in hovered_capsule_points.iter()
            {
                if capsule.id == *capsule_id {
                    match point_type {
                        PointType::Pt1 => selection_level1 = SelectionLevel::Hovered,
                        PointType::Pt2 => selection_level2 = SelectionLevel::Hovered,
                    };
                }
            }
            if selection_level1 == SelectionLevel::Hovered
                && selection_level2 == SelectionLevel::Hovered
            {
                selection_level_body = SelectionLevel::Hovered;
            }

            for CapsulePointId {
                capsule_id,
                point_type,
            } in selected_capsule_points.iter()
            {
                if capsule.id == *capsule_id {
                    match point_type {
                        PointType::Pt1 => selection_level1 = SelectionLevel::Selected,
                        PointType::Pt2 => selection_level2 = SelectionLevel::Selected,
                    };
                }
            }
            if selection_level1 == SelectionLevel::Selected
                && selection_level2 == SelectionLevel::Selected
            {
                selection_level_body = SelectionLevel::Selected;
            }

            let body_color = match selection_level_body {
                SelectionLevel::Selected => CapsuleColors::Selected.to_color32(),
                SelectionLevel::Hovered => CapsuleColors::Highlighted.to_color32(),
                _ => CapsuleColors::Default.to_color32(),
            };
            let endcap1_color = match selection_level1 {
                SelectionLevel::Selected => CapsuleColors::Selected.to_color32(),
                SelectionLevel::Hovered => CapsuleColors::Highlighted.to_color32(),
                _ => CapsuleColors::Default.to_color32(),
            };
            let endcap2_color = match selection_level2 {
                SelectionLevel::Selected => CapsuleColors::Selected.to_color32(),
                SelectionLevel::Hovered => CapsuleColors::Highlighted.to_color32(),
                _ => CapsuleColors::Default.to_color32(),
            };

            capsule.draw(painter, endcap1_color, endcap2_color, body_color);
        }

        for (index, joint) in self.joints.iter().enumerate() {
            let color = self.get_joint_color(index, selected_joints, hovered_joints);
            joint.draw(painter, color);
        }
    }

    fn get_joint_color(
        &self,
        joint_index: usize,
        selected_joints: &Vec<usize>,
        hovered_joints: &Vec<usize>,
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
