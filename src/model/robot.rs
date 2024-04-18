// robot.rs
use super::capsule::{
    self, Capsule, CapsuleColors, CapsulePoint, CapsulePointId, PointInsideCapsule, PointType,
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

    // pub fn get_capsule_count(&self) -> usize {
    //     self.capsules.len()
    // }

    pub fn add_capsule(&mut self, start_point: Pos2, pointer_pos: Pos2, radius: f32) {
        let capsule = Capsule {
            id: generate_capsule_id(),
            radius,
            point1: CapsulePoint {
                capsule_point_id: CapsulePointId {
                    capsule_id: generate_capsule_id(),
                    point_type: PointType::Pt1,
                },
                x: start_point.x,
                y: start_point.y,
            },
            point2: CapsulePoint {
                capsule_point_id: CapsulePointId {
                    capsule_id: generate_capsule_id(),
                    point_type: PointType::Pt2,
                },
                x: pointer_pos.x,
                y: pointer_pos.y,
            },
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

    pub fn move_endcap(&mut self, capsule_id: usize, point_type: PointType, x: f32, y: f32) {
        if let Some(capsule) = self.capsules.get_mut(capsule_id) {
            capsule.update_point_pos(point_type, x, y);
        }
    }

    pub fn update_capsule_body(&mut self, capsule_id: usize, dx: f32, dy: f32) {
        if let Some(capsule) = self.capsules.get_mut(capsule_id) {
            capsule.point1.x += dx;
            capsule.point1.y += dy;
            capsule.point2.x += dx;
            capsule.point2.y += dy;
        }
    }

    // fn get_capsule_point_by_id(&mut self, capsule_point_id: CapsulePointId) -> Option<&CapsulePoint> {
    //     if let Some(capsule) = self.capsules.get_mut(capsule_point_id.capsule_id) {
    //         match capsule_point_id.point_type {
    //             PointType::Pt1 => Some(&CapsulePoint {
    //                 capsule_point_id,
    //                 // x: capsule.x1,
    //                 // y: capsule.y1,
    //                 x: capsule.point1.x,
    //                 y: capsule.point1.y,
    //             }),
    //             PointType::Pt2 => Some(&CapsulePoint {
    //                 capsule_point_id,
    //                 // x: capsule.x2,
    //                 // y: capsule.y2,
    //                 x: capsule.point2.x,
    //                 y: capsule.point2.y,
    //             }),
    //         }
    //     } else {
    //         None
    //     }
    // }

    pub fn update_capsule_point_pos(
        &mut self,
        capsule_point_id: CapsulePointId,
        x: f32,
        y: f32,
    ) {
        if let Some(capsule) = self.capsules.get_mut(capsule_point_id.capsule_id) {
            capsule.update_point_pos(capsule_point_id.point_type, x, y);
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

    pub fn get_capsule_point(
        &self,
        // capsule_id: usize,
        // point_type: &PointType,
        capsule_point_id: CapsulePointId,
    ) -> Option<CapsulePointId> {
        if let Some(capsule) = self.capsules.get(capsule_point_id.capsule_id) {
            match capsule_point_id.point_type {
                PointType::Pt1 => Some(CapsulePointId {
                    capsule_id: capsule.id,
                    point_type: PointType::Pt1,
                }),
                PointType::Pt2 => Some(CapsulePointId {
                    capsule_id: capsule.id,
                    point_type: PointType::Pt2,
                }),
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
            .enumerate()
            .flat_map(|(index, capsule)| {
                let inside_detail = capsule.is_inside_detail(pointer_pos.x, pointer_pos.y);
                match inside_detail {
                    PointInsideCapsule::InsideEndcap1 => vec![CapsulePointId {
                        capsule_id: index,
                        point_type: PointType::Pt1,
                    }],
                    PointInsideCapsule::InsideEndcap2 => vec![CapsulePointId {
                        capsule_id: index,
                        point_type: PointType::Pt2,
                    }],
                    PointInsideCapsule::InsideBody => {
                        vec![
                            CapsulePointId {
                                capsule_id: index,
                                point_type: PointType::Pt1,
                            },
                            CapsulePointId {
                                capsule_id: index,
                                point_type: PointType::Pt2,
                            },
                        ]
                    }
                    _ => vec![],
                }
            })
            .collect()
    }

    // pub fn find_capsule_points_at_position(&self, pointer_pos: Pos2) -> Vec<CapsulePoint> {
    //     self.find_capsule_points_at_position_internal(pointer_pos)
    //         .iter()
    //         .map(
    //             |CapsulePointId {
    //                  capsule_id,
    //                  point_type,
    //              }| {
    //                 let capsule = self.get_capsule(*capsule_id).unwrap();
    //                 let x = match point_type {
    //                     // PointType::Pt1 => capsule.x1,
    //                     // PointType::Pt2 => capsule.x2,
    //                     PointType::Pt1 => capsule.point1.x,
    //                     PointType::Pt2 => capsule.point2.x,
    //                 };
    //                 let y = match point_type {
    //                     // PointType::Pt1 => capsule.y1,
    //                     // PointType::Pt2 => capsule.y2,
    //                     PointType::Pt1 => capsule.point1.y,
    //                     PointType::Pt2 => capsule.point2.y,
    //                 };
    //                 CapsulePoint {
    //                     capsule_point_id: CapsulePointId {
    //                         capsule_id: *capsule_id,
    //                         point_type: *point_type,
    //                     },
    //                     x,
    //                     y,
    //                 }
    //             },
    //         )
    //         .collect()
    // }

    pub fn find_capsule_points_at_position(&self, pointer_pos: Pos2) -> Vec<CapsulePoint> {
        self.find_capsule_points_at_position_internal(pointer_pos)
            .iter()
            .filter_map(
                |CapsulePointId {
                     capsule_id,
                     point_type,
                 }| {
                    self.get_capsule(*capsule_id).and_then(|capsule| {
                        let x = match point_type {
                            PointType::Pt1 => capsule.point1.x,
                            PointType::Pt2 => capsule.point2.x,
                        };
                        let y = match point_type {
                            PointType::Pt1 => capsule.point1.y,
                            PointType::Pt2 => capsule.point2.y,
                        };
                        Some(CapsulePoint {
                            capsule_point_id: CapsulePointId {
                                capsule_id: *capsule_id,
                                point_type: *point_type,
                            },
                            x,
                            y,
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
        selected_capsule_points: &Vec<usize>,
        hovered_capsule_points: &Vec<usize>,
        selected_joints: &Vec<usize>,
        hovered_joints: &Vec<usize>,
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
