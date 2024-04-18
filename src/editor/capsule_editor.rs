use super::common::EditingState;
use crate::model::capsule::{CapsuleColors, CapsulePoint, CapsulePointId};
use crate::model::robot::Robot;
use eframe::egui;
use egui::{Pos2, Shape, Stroke};
use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

#[derive(Clone)]
pub struct OverlappingCapsules {
    pub capsule1_id: usize,
    pub capsule2_id: usize,
}

pub struct CapsuleEditor {
    robot: Rc<RefCell<Robot>>,
    pub radius: f32,
    create_capsule_start_point: Option<Pos2>,
    selected_capsule_points: Vec<CapsulePoint>,
    // selected_capsules: Vec<SelectedCapsule>,
    overlapping_capsules: Vec<OverlappingCapsules>,
    capsule_drag_offsets: Vec<Pos2>,
}

impl CapsuleEditor {
    pub fn new(robot: Rc<RefCell<Robot>>) -> Self {
        let create_capsule_start_point: Option<Pos2> = None;
        let selected_capsule_points: Vec<CapsulePoint> = Vec::new();
        let overlapping_capsules: Vec<OverlappingCapsules> = Vec::new();
        let capsule_drag_offsets: Vec<Pos2> = Vec::new();
        CapsuleEditor {
            robot,
            radius: 20.0,
            create_capsule_start_point,
            selected_capsule_points,
            // selected_capsules: Vec::new(),
            overlapping_capsules,
            capsule_drag_offsets,
        }
    }

    fn create_capsule(&mut self, pointer_pos: Pos2) {
        if let Some(start_point) = self.create_capsule_start_point {
            match self.robot.try_borrow_mut() {
                Ok(mut robot) => {
                    robot.add_capsule(start_point, pointer_pos, self.radius);
                }
                Err(_) => eprintln!("Could not borrow robot mutably"),
            }
            self.create_capsule_start_point = None;
        } else {
            self.create_capsule_start_point = Some(pointer_pos);
        }
    }

    fn delete_capsules(&mut self, pointer_pos: Pos2) {
        self.robot
            .borrow_mut()
            .remove_capsules_by_point(pointer_pos.x, pointer_pos.y);
    }

    pub fn on_capsule_radius_slider_changed(&mut self, new_radius: f32) {
        self.set_selected_capsules_radius(new_radius);
    }

    pub fn clear_capsule_selection(&mut self) {
        self.selected_capsule_points.clear();
        // self.selected_capsules.clear();
    }

    // fn update_selected_capsules(&mut self) {
    //     let mut selected_capsules: Vec<SelectedCapsule> = Vec::new();
    //     for point in &self.selected_capsule_points {
    //         let mut found = false;
    //         for capsule in &mut selected_capsules {
    //             if point.capsule_id == capsule.capsule_id {
    //                 found = true;
    //                 match capsule.selection_type {
    //                     CapsulePtsSelected::Circle1 => {
    //                         if point.point_type == "x2" {
    //                             capsule.selection_type = CapsulePtsSelected::Body;
    //                         }
    //                     }
    //                     CapsulePtsSelected::Circle2 => {
    //                         if point.point_type == "x1" {
    //                             capsule.selection_type = CapsulePtsSelected::Body;
    //                         }
    //                     }
    //                     CapsulePtsSelected::Body => {}
    //                 }
    //                 break;
    //             }
    //         }
    //         if !found {
    //             let selection_type = match point.point_type.as_str() {
    //                 "x1" => CapsulePtsSelected::Circle1,
    //                 "x2" => CapsulePtsSelected::Circle2,
    //                 _ => CapsulePtsSelected::Body,
    //             };
    //             selected_capsules.push(SelectedCapsule {
    //                 capsule_id: point.capsule_id,
    //                 selection_type,
    //             });
    //         }
    //     }
    //     self.selected_capsules = selected_capsules;
    // }

    fn update_overlapping_capsules(&mut self) {
        let robot = match self.robot.try_borrow() {
            Ok(robot) => robot,
            Err(_) => {
                eprintln!("Could not borrow robot");
                return;
            }
        };
        self.overlapping_capsules = robot.get_overlapping_capsules();
    }

    fn handle_capsule_selection(&mut self, ctx: &egui::Context, pointer_pos: Pos2) {
        let clicked_points = self
            .robot
            .borrow()
            .find_capsule_points_at_position(pointer_pos);

        if clicked_points.is_empty() {
            self.clear_capsule_selection();
        } else if ctx.input(|i| i.modifiers.shift) {
            self.toggle_selected_capsule_points(&clicked_points);
        } else {
            self.selected_capsule_points = clicked_points;
        }
    }

    fn toggle_selected_capsule_points(&mut self, clicked_points: &[CapsulePoint]) {
        for CapsulePoint {
            capsule_point_id,
            x,
            y,
        } in clicked_points
        {
            // if let Some(index) = self.selected_capsule_points.iter().position(|capsule_point| {
            //     capsule_point.capsule_id == clicked_point.capsule_id
            //         && capsule_point.point_type == clicked_point.point_type
            // }) {
            //     self.selected_capsule_points.remove(index);
            // } else {
            //     self.selected_capsule_points.push(clicked_point.clone());
            // }
            if let Some(index) = self
                .selected_capsule_points
                .iter()
                .position(|point| point.capsule_point_id == *capsule_point_id)
            {
                self.selected_capsule_points.remove(index);
            } else {
                self.selected_capsule_points.push(CapsulePoint {
                    capsule_point_id: *capsule_point_id,
                    x: *x,
                    y: *y,
                });
            }
        }
    }

    fn is_drag_start_inside_selected(&self, pointer_pos: Pos2) -> bool {
        let robot = match self.robot.try_borrow() {
            Ok(robot) => robot,
            Err(_) => {
                eprintln!("Could not borrow robot");
                return false;
            }
        };
        robot.find_capsule_points_at_position(pointer_pos).len() > 0
    }

    fn handle_dragging(&mut self, response: &egui::Response, pointer_pos: Pos2) {
        if response.drag_started()
            // && !self.selected_capsules.is_empty()
            && !self.selected_capsule_points.is_empty()
            && self.is_drag_start_inside_selected(pointer_pos)
        {
            self.set_drag_offsets(pointer_pos);
        } else if response.dragged() {
            self.drag_capsules(pointer_pos);
        } else if response.drag_stopped() {
            self.stop_dragging();
        }
    }

    fn set_drag_offsets(&mut self, pointer_pos: Pos2) {
        self.capsule_drag_offsets = self
            .selected_capsule_points
            .iter()
            .map(|point| Pos2::new(pointer_pos.x - point.x, pointer_pos.y - point.y))
            .collect();
    }

    // fn set_drag_offsets(&mut self, pointer_pos: Pos2) {
    //     self.capsule_drag_offsets = self
    //         .selected_capsules
    //         .iter()
    //         .map(|selected_capsule| {
    //             let capsule = self.robot.borrow().get_capsule(selected_capsule.capsule_id).unwrap();
    //             let (x, y) = match selected_capsule.selection_type {
    //                 CapsulePtsSelected::Circle1 => (capsule.x1, capsule.y1),
    //                 CapsulePtsSelected::Circle2 => (capsule.x2, capsule.y2),
    //                 CapsulePtsSelected::Body => (capsule.x1, capsule.y1),
    //             };
    //             Pos2::new(pointer_pos.x - x, pointer_pos.y - y)
    //         })
    //         .collect();
    // }

    fn stop_dragging(&mut self) {
        self.update_selected_capsule_points();
        self.capsule_drag_offsets.clear();
        self.update_overlapping_capsules();
    }

    // fn drag_capsules(&mut self, pointer_pos: Pos2) {
    //     for (index, selected_capsule) in self.selected_capsules.iter().enumerate() {
    //         let offset = self.capsule_drag_offsets[index];
    //         match selected_capsule.selection_type {
    //             CapsulePtsSelected::Circle1 => {
    //                 self.robot.borrow_mut().update_capsule_endcap(
    //                     selected_capsule.capsule_id,
    //                     PointType::Pt1,
    //                     pointer_pos.x - offset.x,
    //                     pointer_pos.y - offset.y,
    //                 );
    //             }
    //             CapsulePtsSelected::Circle2 => {
    //                 self.robot.borrow_mut().update_capsule_endcap(
    //                     selected_capsule.capsule_id,
    //                     PointType::Pt2,
    //                     pointer_pos.x - offset.x,
    //                     pointer_pos.y - offset.y,
    //                 );
    //             }
    //             CapsulePtsSelected::Body => {
    //                 let dx = pointer_pos.x
    //                     - offset.x
    //                     - self
    //                         .robot
    //                         .borrow()
    //                         .get_capsule(selected_capsule.capsule_id)
    //                         .unwrap()
    //                         .x1;
    //                 let dy = pointer_pos.y
    //                     - offset.y
    //                     - self
    //                         .robot
    //                         .borrow()
    //                         .get_capsule(selected_capsule.capsule_id)
    //                         .unwrap()
    //                         .y1;
    //                 self.robot.borrow_mut().update_capsule_body(
    //                     selected_capsule.capsule_id,
    //                     dx,
    //                     dy,
    //                 );
    //             }
    //         }
    //     }
    // }

    // fn drag_capsules(&mut self, pointer_pos: Pos2) {
    //     for (index, selected_capsule) in self.selected_capsules.iter().enumerate() {
    //         let offset = self.capsule_drag_offsets[index];
    //         let mut robot = match self.robot.try_borrow_mut() {
    //             Ok(robot) => robot,
    //             Err(_) => {
    //                 eprintln!("Could not borrow robot");
    //                 return;
    //             }
    //         };
    //         match selected_capsule.selection_type {
    //             CapsulePtsSelected::Circle1 => {
    //                 robot.update_capsule_endcap(
    //                     selected_capsule.capsule_id,
    //                     PointType::Pt1,
    //                     pointer_pos.x - offset.x,
    //                     pointer_pos.y - offset.y,
    //                 );
    //             }
    //             CapsulePtsSelected::Circle2 => {
    //                 robot.update_capsule_endcap(
    //                     selected_capsule.capsule_id,
    //                     PointType::Pt2,
    //                     pointer_pos.x - offset.x,
    //                     pointer_pos.y - offset.y,
    //                 );
    //             }
    //             CapsulePtsSelected::Body => {
    //                 let dx = pointer_pos.x - offset.x;
    //                 let dy = pointer_pos.y - offset.y;
    //                 let capsule = robot.get_capsule(selected_capsule.capsule_id).unwrap();
    //                 robot.update_capsule_body(
    //                     selected_capsule.capsule_id,
    //                     dx - capsule.x1,
    //                     dy - capsule.y1,
    //                 );
    //             }
    //         }
    //     }
    // }

    fn drag_capsules(&mut self, pointer_pos: Pos2) {
        for (index, point) in self.selected_capsule_points.iter().enumerate() {
            let offset = self.capsule_drag_offsets[index];
            let mut robot = match self.robot.try_borrow_mut() {
                Ok(robot) => robot,
                Err(_) => {
                    eprintln!("Could not borrow robot");
                    return;
                }
            };
            robot.update_capsule_point_pos(
                point.capsule_point_id,
                pointer_pos.x - offset.x,
                pointer_pos.y - offset.y,
            );
        }
    }

    fn set_selected_capsules_radius(&mut self, new_radius: f32) {
        // for capsule in &mut self.selected_capsules {
        //     self.robot
        //         .borrow_mut()
        //         .update_capsule_radius(capsule.capsule_id, new_radius);
        // }
        let mut capsule_ids: HashSet<usize> = HashSet::new();
        for capsule_point in &self.selected_capsule_points {
            capsule_ids.insert(capsule_point.capsule_point_id.capsule_id);
        }
    }

    fn update_capsules(&mut self, ctx: &egui::Context, pointer_pos: Pos2) {
        self.handle_capsule_selection(ctx, pointer_pos);
        // self.update_selected_capsules();
        self.update_overlapping_capsules();
    }

    pub fn update(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        editing_state: &mut EditingState,
        pointer_pos: Pos2,
        response: &egui::Response,
    ) {
        match *editing_state {
            EditingState::Create => {
                if response.clicked() {
                    self.create_capsule(pointer_pos);
                }
            }
            EditingState::Delete => {
                if response.clicked() {
                    self.delete_capsules(pointer_pos);
                }
            }
            EditingState::Update => {
                if response.clicked() {
                    self.update_capsules(ctx, pointer_pos);
                }
                self.handle_dragging(response, pointer_pos);
            }
        }

        if ui.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.delete_capsules(pointer_pos);
        }
    }

    fn update_selected_capsule_points(&mut self) {
        // self.selected_capsule_points = self
        //     .selected_capsule_points
        //     .iter()
        //     .map(|point| {
        //         if let Some((x, y)) = self
        //             .robot
        //             .borrow()
        //             .get_capsule_point(point.capsule_point_id)
        //         {
        //             CapsulePoint {
        //                 // capsule_id: point.capsule_id,
        //                 // point_type: point.point_type.clone(),
        //                 capsule_point_id: point.capsule_point_id,
        //                 x,
        //                 y,
        //             }
        //         } else {
        //             point.clone()
        //         }
        //     })
        //     .collect();
    }

    pub fn on_editing_state_changed(&mut self, editing_state: EditingState) {
        match editing_state {
            EditingState::Create | EditingState::Update | EditingState::Delete => {
                self.clear_capsule_selection();
            }
        }
    }

    pub fn draw_editor(
        &self,
        painter: &egui::Painter,
        pointer_pos: Pos2,
        editing_state: EditingState,
        capsule_radius: f32,
    ) {
        let robot = match self.robot.try_borrow() {
            Ok(robot) => robot,
            Err(_) => {
                eprintln!("Could not borrow robot");
                return;
            }
        };

        let selected_capsule_points: Vec<usize> = self
            .selected_capsule_points
            .iter()
            // .map(|point| point.capsule_id * 2 + if point.point_type == "x1" { 0 } else { 1 })
            .map(|point| point.capsule_point_id.capsule_id)
            .collect();

        let hovered_capsule_points: Vec<usize> = robot
            .find_hovered_capsule_points(pointer_pos)
            .iter()
            .map(|point| point.capsule_id)
            .collect();
        // println!("Len hovered: {:?}", hovered_capsule_points.len());

        let selected_joints = vec![];
        let hovered_joints = vec![];

        robot.draw(
            painter,
            &selected_capsule_points,
            &hovered_capsule_points,
            &selected_joints,
            &hovered_joints,
        );

        // self.draw_overlapping_capsules(painter);
        self.draw_editing_visualization(painter, pointer_pos, editing_state, capsule_radius);
    }

    // fn draw_overlapping_capsules(&self, painter: &egui::Painter) {
    //     if let Ok(robot) = self.robot.try_borrow() {
    //         for overlapping in &self.overlapping_capsules {
    //             if let (Some(capsule1), Some(capsule2)) = (
    //                 robot.get_capsule(overlapping.capsule1_id),
    //                 robot.get_capsule(overlapping.capsule2_id),
    //             ) {
    //                 painter.line_segment(
    //                     [capsule1.get_center(), capsule2.get_center()],
    //                     Stroke::new(2.0, Color32::RED),
    //                 );
    //             }
    //         }
    //     }
    // }

    fn draw_editing_visualization(
        &self,
        painter: &egui::Painter,
        pointer_pos: Pos2,
        editing_state: EditingState,
        capsule_radius: f32,
    ) {
        match editing_state {
            EditingState::Create => {
                self.draw_create_capsule_visualization(painter, pointer_pos, capsule_radius)
            }
            EditingState::Update => self.draw_update_visualization(painter, pointer_pos),
            EditingState::Delete => self.draw_delete_visualization(painter, pointer_pos),
        }
    }

    fn draw_create_capsule_visualization(
        &self,
        painter: &egui::Painter,
        pointer_pos: Pos2,
        capsule_radius: f32,
    ) {
        if let Some(start_point) = self.create_capsule_start_point {
            let stroke = Stroke::new(10.0, CapsuleColors::Add.to_color32());
            painter.line_segment([start_point, pointer_pos], stroke);

            let fill = CapsuleColors::Add.to_color32();
            painter.add(Shape::circle_filled(start_point, capsule_radius, fill));
            painter.add(Shape::circle_filled(pointer_pos, capsule_radius, fill));
        } else {
            let fill = CapsuleColors::Add.to_color32();
            painter.add(Shape::circle_filled(pointer_pos, capsule_radius, fill));
        }
    }

    fn draw_update_visualization(&self, painter: &egui::Painter, pointer_pos: Pos2) {
        // TODO: Implement update visualization for capsules
    }

    fn draw_delete_visualization(&self, painter: &egui::Painter, pointer_pos: Pos2) {
        // TODO: Implement delete visualization for capsules
    }
}

#[derive(Debug, Clone, Copy)]
pub enum CapsulePtsSelected {
    Circle1,
    Circle2,
    Body,
}
