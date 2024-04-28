use super::common::EditingState;
use crate::model::capsule::{CapsuleColors, CapsulePoint, CapsulePointId};
use crate::model::robot::Robot;
use eframe::egui;
use egui::{Pos2, Shape, Stroke};
use std::collections::HashSet;

#[derive(Clone)]
pub struct OverlappingCapsules {
    pub capsule1_id: usize,
    pub capsule2_id: usize,
}

pub struct CapsuleEditor {
    pub radius: f32,
    create_capsule_start_point: Option<Pos2>,
    selected_capsule_points: Vec<CapsulePoint>,
    overlapping_capsules: Vec<OverlappingCapsules>,
    capsule_drag_offsets: Vec<Pos2>,
}

impl CapsuleEditor {
    pub fn new() -> Self {
        let create_capsule_start_point: Option<Pos2> = None;
        let selected_capsule_points: Vec<CapsulePoint> = Vec::new();
        let overlapping_capsules: Vec<OverlappingCapsules> = Vec::new();
        let capsule_drag_offsets: Vec<Pos2> = Vec::new();
        CapsuleEditor {
            radius: 20.0,
            create_capsule_start_point,
            selected_capsule_points,
            overlapping_capsules,
            capsule_drag_offsets,
        }
    }

    fn create_capsule(&mut self, pointer_pos: Pos2, robot: &mut Robot) {
        if let Some(start_point) = self.create_capsule_start_point {
            robot.add_capsule(start_point, pointer_pos, self.radius);
            self.create_capsule_start_point = None;
        } else {
            self.create_capsule_start_point = Some(pointer_pos);
        }
    }

    fn delete_capsules(&mut self, pointer_pos: Pos2, robot: &mut Robot) {
        robot.remove_capsules_by_point(pointer_pos.x, pointer_pos.y);
    }

    pub fn on_capsule_radius_slider_changed(&mut self, new_radius: f32, robot: &mut Robot) {
        self.set_selected_capsules_radius(new_radius, robot);
    }

    pub fn clear_capsule_selection(&mut self) {
        self.selected_capsule_points.clear();
    }

    fn update_overlapping_capsules(&mut self, robot: &Robot) {
        self.overlapping_capsules = robot.get_overlapping_capsules();
    }

    fn handle_capsule_selection(&mut self, ctx: &egui::Context, pointer_pos: Pos2, robot: &Robot) {
        let clicked_points = robot.find_capsule_points_at_position(pointer_pos);

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

    fn is_drag_start_inside_selected(&self, pointer_pos: Pos2, robot: &Robot) -> bool {
        robot.find_capsule_points_at_position(pointer_pos).len() > 0
    }

    fn handle_dragging(&mut self, response: &egui::Response, pointer_pos: Pos2, robot: &mut Robot) {
        if response.drag_started()
            && !self.selected_capsule_points.is_empty()
            && self.is_drag_start_inside_selected(pointer_pos, robot)
        {
            self.set_drag_offsets(pointer_pos);
        } else if response.dragged() {
            self.drag_capsules(pointer_pos, robot);
        } else if response.drag_stopped() {
            self.stop_dragging(robot);
        }
    }

    fn set_drag_offsets(&mut self, pointer_pos: Pos2) {
        self.capsule_drag_offsets = self
            .selected_capsule_points
            .iter()
            .map(|point| Pos2::new(pointer_pos.x - point.x, pointer_pos.y - point.y))
            .collect();
    }

    fn stop_dragging(&mut self, robot: &mut Robot) {
        self.update_selected_capsule_points(robot);
        self.capsule_drag_offsets.clear();
        self.update_overlapping_capsules(robot);
    }

    fn drag_capsules(&mut self, pointer_pos: Pos2, robot: &mut Robot) {
        for (index, point) in self.selected_capsule_points.iter().enumerate() {
            let offset = self.capsule_drag_offsets[index];
            robot.update_capsule_point_pos(
                point.capsule_point_id,
                pointer_pos.x - offset.x,
                pointer_pos.y - offset.y,
            );
        }
    }

    fn set_selected_capsules_radius(&mut self, new_radius: f32, robot: &mut Robot) {
        let mut capsule_ids: HashSet<usize> = HashSet::new();
        for capsule_point in &self.selected_capsule_points {
            capsule_ids.insert(capsule_point.capsule_point_id.capsule_id);
        }
        for capsule_id in capsule_ids {
            robot.update_capsule_radius(capsule_id, new_radius);
        }
    }

    fn update_capsules(&mut self, ctx: &egui::Context, pointer_pos: Pos2, robot: &mut Robot) {
        self.handle_capsule_selection(ctx, pointer_pos, robot);
        self.update_overlapping_capsules(robot);
    }

    pub fn update(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        editing_state: &mut EditingState,
        pointer_pos: Pos2,
        response: &egui::Response,
        robot: &mut Robot
    ) {
        match *editing_state {
            EditingState::Create => {
                if response.clicked() {
                    self.create_capsule(pointer_pos, robot);
                }
            }
            EditingState::Delete => {
                if response.clicked() {
                    self.delete_capsules(pointer_pos, robot);
                }
            }
            EditingState::Update => {
                if response.clicked() {
                    self.update_capsules(ctx, pointer_pos, robot);
                }
                self.handle_dragging(response, pointer_pos, robot);
            }
        }

        if ui.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.delete_capsules(pointer_pos, robot);
        }
    }

    fn update_selected_capsule_points(&mut self, robot: &Robot) {
        println!("Before: {:?}", self.selected_capsule_points);
        self.selected_capsule_points = self
            .selected_capsule_points
            .iter_mut()
            .map(|point| {
                if let Some(Pos2 { x, y }) = 
                    robot
                    .get_capsule_point_pos(point.capsule_point_id)
                {
                    println!("x: {}, y: {}", x, y);
                    CapsulePoint {
                        capsule_point_id: point.capsule_point_id,
                        x,
                        y,
                    }
                } else {
                    point.clone()
                }
            })
            .collect();
        println!("After:  {:?}", self.selected_capsule_points);
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
        robot: &mut Robot,
    ) {
        let selected_capsule_points: Vec<CapsulePointId> = self
            .selected_capsule_points
            .iter()
            .map(|point| point.capsule_point_id)
            .collect();

        let hovered_capsule_points: Vec<CapsulePointId> =
            robot.find_hovered_capsule_points(pointer_pos).clone();

        let selected_joints = vec![];
        let hovered_joints = vec![];

        robot.draw(
            painter,
            &selected_capsule_points,
            &hovered_capsule_points,
            &selected_joints,
            &hovered_joints,
        );

        self.draw_editing_visualization(painter, pointer_pos, editing_state, capsule_radius);
    }

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
