use super::common::{EditingState, JointCreationState};
use crate::model::capsule::{CapsulePointId, PointType};
use crate::model::joint::Joint;
use crate::model::robot::Robot;
use std::collections::HashSet;

use eframe::egui;
use egui::{Color32, Pos2, Stroke};

pub struct JointEditor {
    selected_joints: Vec<usize>,
    selected_capsules: Vec<usize>,
    joint_creation_state: JointCreationState,
}

impl JointEditor {
    pub fn new() -> Self {
        JointEditor {
            selected_joints: Vec::new(),
            selected_capsules: Vec::new(),
            joint_creation_state: JointCreationState::SelectFirstCapsule,
        }
    }

    pub fn update(
        &mut self,
        ui: &mut egui::Ui,
        editing_state: &mut EditingState,
        pointer_pos: Pos2,
        response: &egui::Response,
        robot: &mut Robot,
    ) {
        match *editing_state {
            EditingState::Create => self.handle_create_joint(pointer_pos, ui, response, robot),
            EditingState::Delete => self.handle_delete_joint(pointer_pos, robot),
            EditingState::Update => self.handle_update_joint(pointer_pos, robot),
        }
        self.handle_joint_delete_key(ui, pointer_pos, robot);
    }

    pub fn on_editing_state_changed(&mut self, editing_state: EditingState) {
        match editing_state {
            EditingState::Create | EditingState::Update | EditingState::Delete => {
                self.clear_selections();
                self.joint_creation_state = JointCreationState::SelectFirstCapsule;
            }
        }
    }

    fn clear_selections(&mut self) {
        // self.selected_joint = None;
        self.selected_joints.clear();
        self.selected_capsules.clear();
        self.joint_creation_state = JointCreationState::SelectFirstCapsule;
    }

    fn handle_create_joint(
        &mut self,
        pointer_pos: Pos2,
        ui: &mut egui::Ui,
        response: &egui::Response,
        robot: &mut Robot,
    ) {
        match self.joint_creation_state {
            JointCreationState::SelectFirstCapsule => {
                ui.label("Select first capsule");
                if response.clicked() {
                    self.update_capsule_selection(pointer_pos, 1, ui, robot);
                }
            }
            JointCreationState::SelectSecondCapsule => {
                ui.label("Select second capsule");
                if response.clicked() {
                    self.update_capsule_selection(pointer_pos, 2, ui, robot);
                }
            }
            JointCreationState::PlaceJoint => {
                ui.label("Place joint in the intersection");
                if response.clicked() {
                    self.create_joint(pointer_pos, ui, robot);
                }
            }
        }
    }

    fn handle_delete_joint(&mut self, pointer_pos: Pos2, robot: &mut Robot) {
        robot.remove_joint(pointer_pos);
    }

    fn handle_update_joint(&mut self, pointer_pos: Pos2, robot: &mut Robot) {
        self.selected_joints = robot.find_joints_by_point(pointer_pos);
    }

    fn handle_joint_delete_key(&mut self, ui: &mut egui::Ui, pointer_pos: Pos2, robot: &mut Robot) {
        if ui.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.handle_delete_joint(pointer_pos, robot);
        }
    }

    fn create_joint(&mut self, pointer_pos: Pos2, ui: &mut egui::Ui, robot: &mut Robot) {
        println!("Creating joint");
        let mut joint_created = false;

        if self.selected_capsules.len() == 2
            && robot.check_joint_placement(pointer_pos, &self.selected_capsules)
        {
            let joint = Joint {
                id: robot.get_joint_count(),
                x: pointer_pos.x,
                y: pointer_pos.y,
                capsule1_id: self.selected_capsules[0],
                capsule2_id: self.selected_capsules[1],
                min: 0.0,
                max: 1.0,
            };
            robot.add_joint(joint);
            joint_created = true;
        } else {
            egui::Window::new("Error")
                .collapsible(false)
                .resizable(false)
                .show(ui.ctx(), |ui| {
                    ui.label("The second capsule must intersect with the first capsule.");
                });
        }

        if joint_created {
            self.clear_selections();
            self.joint_creation_state = JointCreationState::SelectFirstCapsule;
        }
    }

    // fn update_joint_selection(&mut self, pointer_pos: Pos2) {
    //     self.selected_joint = self.robot.borrow().find_joint_at_position(pointer_pos);
    // }

    fn update_capsule_selection(
        &mut self,
        pointer_pos: Pos2,
        max_selections: usize,
        ui: &mut egui::Ui,
        robot: &Robot,
    ) {
        // let capsules = &robot.capsules;
        // let clicked_capsule = capsules
        //     .iter()
        //     .position(|capsule| capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y));
        let clicked_capsule_id_set: HashSet<usize> = robot
            .find_capsules_by_point(pointer_pos)
            .iter()
            .map(|capsule_point_id| capsule_point_id.capsule_id)
            .collect();
        let clicked_capsule_ids: Vec<usize> = clicked_capsule_id_set.iter().cloned().collect();
        // check if the length of clicked capsules is one
        // otherwise, return
        if clicked_capsule_ids.len() != 1 {
            ui.label("Only one capsule can be selected at a time.");
            return;
        }
        let clicked_capsule: Option<usize> = clicked_capsule_ids.get(0).cloned();

        if let Some(capsule_id) = clicked_capsule {
            if self.selected_capsules.contains(&capsule_id) {
                // Deselect the capsule if it's already selected
                self.selected_capsules.retain(|&id| id != capsule_id);
            } else if (self.selected_capsules.len() == 1)
                && (robot.are_capsules_already_joined(self.selected_capsules[0], capsule_id))
            {
                println!("The capsules are already joined.");
                return;
            } else if self.selected_capsules.len() < max_selections {
                self.selected_capsules.push(capsule_id);
            }

            if self.selected_capsules.len() == max_selections {
                self.joint_creation_state = match self.joint_creation_state {
                    JointCreationState::SelectFirstCapsule => {
                        JointCreationState::SelectSecondCapsule
                    }
                    JointCreationState::SelectSecondCapsule => JointCreationState::PlaceJoint,
                    _ => self.joint_creation_state,
                };
            }
        } else {
            self.selected_capsules.clear();
            self.joint_creation_state = JointCreationState::SelectFirstCapsule;
        }
    }

    pub fn draw_editor(
        &self,
        painter: &egui::Painter,
        pointer_pos: Pos2,
        editing_state: EditingState,
        robot: &Robot,
    ) {
        let selected_capsule_points: Vec<CapsulePointId> = self
            .selected_capsules
            .iter()
            .flat_map(|&capsule_id| {
                [
                    CapsulePointId {
                        capsule_id,
                        point_type: PointType::Pt1,
                    },
                    CapsulePointId {
                        capsule_id,
                        point_type: PointType::Pt2,
                    },
                ]
            })
            .collect();

        let hovered_capsule_ids: HashSet<usize> = robot
            .find_hovered_capsule_points(pointer_pos)
            .iter()
            .map(|capsule_point| capsule_point.capsule_id)
            .collect();
        let hovered_capsule_points: Vec<CapsulePointId> = hovered_capsule_ids
            .iter()
            .flat_map(|capsule_id| {
                [
                    CapsulePointId {
                        capsule_id: *capsule_id,
                        point_type: PointType::Pt1,
                    },
                    CapsulePointId {
                        capsule_id: *capsule_id,
                        point_type: PointType::Pt2,
                    },
                ]
            })
            .collect();
        // let selected_joints: Vec<usize> = self.selected_joint.into_iter().collect();
        let hovered_joints = vec![];

        robot.draw(
            painter,
            &selected_capsule_points,
            &hovered_capsule_points,
            &self.selected_joints,
            &hovered_joints,
        );

        self.draw_joint_visualization(painter, pointer_pos, robot);
        self.draw_editing_visualization(painter, pointer_pos, editing_state);
    }

    fn draw_joint_visualization(&self, painter: &egui::Painter, pointer_pos: Pos2, robot: &Robot) {
        if self.selected_capsules.len() == 2 {
            if robot.check_joint_placement(pointer_pos, &self.selected_capsules) {
                painter.circle_filled(pointer_pos, 5.0, Color32::GREEN);
            } else {
                painter.circle_filled(pointer_pos, 5.0, Color32::GRAY);
            }
        }
    }

    fn draw_editing_visualization(
        &self,
        painter: &egui::Painter,
        pointer_pos: Pos2,
        editing_state: EditingState,
    ) {
        match editing_state {
            EditingState::Create => self.draw_create_joint_visualization(painter, pointer_pos),
            // EditingState::Create => {},
            EditingState::Update => self.draw_update_visualization(painter, pointer_pos),
            EditingState::Delete => self.draw_delete_visualization(painter, pointer_pos),
        }
    }

    fn draw_create_joint_visualization(&self, painter: &egui::Painter, pointer_pos: Pos2) {
        // if self.selected_capsules.len() == 2 {
        //     if let Ok(robot) = self.robot.try_borrow() {
        //         if let (Some(capsule1), Some(capsule2)) = (
        //             robot.get_capsule(self.selected_capsules[0]),
        //             robot.get_capsule(self.selected_capsules[1]),
        //         ) {
        //             painter.line_segment(
        //                 [capsule1.get_center(), capsule2.get_center()],
        //                 Stroke::new(2.0, Color32::GREEN),
        //             );
        //         }
        //     }
        // }
        painter.circle_stroke(pointer_pos, 5.0, Stroke::new(2.0, Color32::GREEN));
    }

    fn draw_update_visualization(&self, painter: &egui::Painter, pointer_pos: Pos2) {
        // TODO: Implement update visualization for joints
    }

    fn draw_delete_visualization(&self, painter: &egui::Painter, pointer_pos: Pos2) {
        // TODO: Implement delete visualization for joints
    }
}
