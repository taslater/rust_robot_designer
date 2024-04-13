use super::common::EditingState;
use crate::model::capsule::{Capsule, PointInsideCapsule};
use crate::model::robot::Robot;
use eframe::egui;
use egui::{Color32, Pos2, Shape, Stroke};
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug, Clone)]
pub struct SelectedCapsulePoint {
    pub capsule_id: usize,
    pub point_type: String,
    x: f32,
    y: f32,
}

struct SelectedCapsule {
    capsule_id: usize,
    selection_type: CapsulePtsSelected,
}

#[derive(Clone)]
pub struct OverlappingCapsules {
    pub capsule1_id: usize,
    pub capsule2_id: usize,
}

// pub struct CapsuleRenderData {
//     pub capsule: Capsule,
//     pub circle1_color: CapsuleColors,
//     pub circle2_color: CapsuleColors,
//     pub body_color: CapsuleColors,
// }

pub struct CapsuleEditor {
    pub robot: Rc<RefCell<Robot>>,
    pub radius: f32,
    is_dragging: bool,
    pub create_capsule_start_point: Option<Pos2>,
    pub selected_capsule_points: Vec<SelectedCapsulePoint>,
    selected_capsules: Vec<SelectedCapsule>,
    pub overlapping_capsules: Vec<OverlappingCapsules>,
    capsule_drag_offsets: Vec<Pos2>,
}

impl CapsuleEditor {
    pub fn new(robot: Rc<RefCell<Robot>>) -> Self {
        CapsuleEditor {
            robot,
            radius: 20.0,
            is_dragging: false,
            create_capsule_start_point: None,
            selected_capsule_points: Vec::new(),
            selected_capsules: Vec::new(),
            overlapping_capsules: Vec::new(),
            capsule_drag_offsets: Vec::new(),
        }
    }

    fn create_capsule(&mut self, pointer_pos: Pos2) {
        if let Some(start_point) = self.create_capsule_start_point {
            match self.robot.try_borrow_mut() {
                Ok(mut robot) => {
                    let id = robot.capsules.len();
                    let capsule = Capsule {
                        id,
                        radius: self.radius,
                        x1: start_point.x,
                        y1: start_point.y,
                        x2: pointer_pos.x,
                        y2: pointer_pos.y,
                    };
                    robot.capsules.push(capsule);
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
            .capsules
            .retain(|capsule| !capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y));
    }

    fn drag_capsules(&mut self, pointer_pos: Pos2) {
        for (index, selected_capsule) in self.selected_capsules.iter().enumerate() {
            if let Some(capsule) = self
                .robot
                .borrow_mut()
                .capsules
                .get_mut(selected_capsule.capsule_id)
            {
                let offset = self.capsule_drag_offsets[index];
                match selected_capsule.selection_type {
                    CapsulePtsSelected::Circle1 => {
                        capsule.x1 = pointer_pos.x - offset.x;
                        capsule.y1 = pointer_pos.y - offset.y;
                    }
                    CapsulePtsSelected::Circle2 => {
                        capsule.x2 = pointer_pos.x - offset.x;
                        capsule.y2 = pointer_pos.y - offset.y;
                    }
                    CapsulePtsSelected::Body => {
                        let dx = pointer_pos.x - offset.x - capsule.x1;
                        let dy = pointer_pos.y - offset.y - capsule.y1;
                        capsule.x1 += dx;
                        capsule.y1 += dy;
                        capsule.x2 += dx;
                        capsule.y2 += dy;
                    }
                }
            }
        }
    }

    pub fn on_capsule_radius_slider_changed(&mut self, new_radius: f32) {
        self.set_selected_capsules_radius(new_radius);
    }

    pub fn clear_capsule_selection(&mut self) {
        self.selected_capsule_points.clear();
        self.selected_capsules.clear();
    }

    fn update_selected_capsules(&mut self) {
        let mut selected_capsules: Vec<SelectedCapsule> = Vec::new();
        for point in &self.selected_capsule_points {
            let mut found = false;
            for capsule in &mut selected_capsules {
                if point.capsule_id == capsule.capsule_id {
                    found = true;
                    match capsule.selection_type {
                        CapsulePtsSelected::Circle1 => {
                            if point.point_type == "x2" {
                                capsule.selection_type = CapsulePtsSelected::Body;
                            }
                        }
                        CapsulePtsSelected::Circle2 => {
                            if point.point_type == "x1" {
                                capsule.selection_type = CapsulePtsSelected::Body;
                            }
                        }
                        CapsulePtsSelected::Body => {}
                    }
                    break;
                }
            }
            if !found {
                let selection_type = match point.point_type.as_str() {
                    "x1" => CapsulePtsSelected::Circle1,
                    "x2" => CapsulePtsSelected::Circle2,
                    _ => CapsulePtsSelected::Body,
                };
                selected_capsules.push(SelectedCapsule {
                    capsule_id: point.capsule_id,
                    selection_type,
                });
            }
        }
        self.selected_capsules = selected_capsules;
    }

    fn update_overlapping_capsules(&mut self) {
        let mut overlapping_capsules: Vec<OverlappingCapsules> = Vec::new();
        for (i, capsule1) in self.robot.borrow().capsules.iter().enumerate() {
            for (j, capsule2) in self.robot.borrow().capsules.iter().enumerate() {
                if i < j && capsule1.intersects_capsule(capsule2) {
                    overlapping_capsules.push(OverlappingCapsules {
                        capsule1_id: i,
                        capsule2_id: j,
                    });
                }
            }
        }
        self.overlapping_capsules = overlapping_capsules;
    }

    fn handle_capsule_selection(&mut self, ctx: &egui::Context, pointer_pos: Pos2) {
        let clicked_points: Vec<SelectedCapsulePoint> = self
            .robot
            .borrow()
            .capsules
            .iter()
            .flat_map(|capsule| {
                let inside_detail = capsule.is_inside_detail(pointer_pos.x, pointer_pos.y);
                match inside_detail {
                    PointInsideCapsule::InsideEndcap1 => {
                        vec![SelectedCapsulePoint {
                            capsule_id: capsule.id,
                            point_type: "x1".to_string(),
                            x: capsule.x1,
                            y: capsule.y1,
                        }]
                    }
                    PointInsideCapsule::InsideEndcap2 => {
                        vec![SelectedCapsulePoint {
                            capsule_id: capsule.id,
                            point_type: "x2".to_string(),
                            x: capsule.x2,
                            y: capsule.y2,
                        }]
                    }
                    PointInsideCapsule::InsideBody => {
                        vec![
                            SelectedCapsulePoint {
                                capsule_id: capsule.id,
                                point_type: "x1".to_string(),
                                x: capsule.x1,
                                y: capsule.y1,
                            },
                            SelectedCapsulePoint {
                                capsule_id: capsule.id,
                                point_type: "x2".to_string(),
                                x: capsule.x2,
                                y: capsule.y2,
                            },
                        ]
                    }
                    _ => vec![],
                }
            })
            .collect();

        if clicked_points.is_empty() {
            self.clear_capsule_selection();
        } else if ctx.input(|i| i.modifiers.shift) {
            for clicked_point in &clicked_points {
                if let Some(index) = self.selected_capsule_points.iter().position(|point| {
                    point.capsule_id == clicked_point.capsule_id
                        && point.point_type == clicked_point.point_type
                }) {
                    self.selected_capsule_points.remove(index);
                } else {
                    self.selected_capsule_points.push(clicked_point.clone());
                }
            }
        } else {
            self.selected_capsule_points = clicked_points;
        }
    }

    fn is_drag_start_inside_selected(&self, pointer_pos: Pos2) -> bool {
        for selected_capsule in &self.selected_capsules {
            if let Some(capsule) = self
                .robot
                .borrow()
                .capsules
                .get(selected_capsule.capsule_id)
            {
                match selected_capsule.selection_type {
                    CapsulePtsSelected::Circle1 => {
                        if capsule.is_inside_circle1(pointer_pos.x, pointer_pos.y) {
                            return true;
                        }
                    }
                    CapsulePtsSelected::Circle2 => {
                        if capsule.is_inside_circle2(pointer_pos.x, pointer_pos.y) {
                            return true;
                        }
                    }
                    CapsulePtsSelected::Body => {
                        if capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y) {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    fn handle_dragging(&mut self, response: &egui::Response, pointer_pos: Pos2) {
        if response.clicked() && !self.selected_capsules.is_empty() {
            if self.is_drag_start_inside_selected(pointer_pos) {
                self.set_drag_offsets(pointer_pos);
                self.start_dragging();
            }
        } else if response.dragged() && self.is_dragging {
            self.drag_capsules(pointer_pos);
        } else if response.drag_stopped() {
            self.update_selected_capsule_points();
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

    fn start_dragging(&mut self) {
        self.is_dragging = true;
    }

    fn stop_dragging(&mut self) {
        self.is_dragging = false;
        self.capsule_drag_offsets.clear();
        self.update_overlapping_capsules();
    }

    fn set_selected_capsules_radius(&mut self, new_radius: f32) {
        for capsule in &mut self.selected_capsules {
            if let Some(capsule) = self.robot.borrow_mut().capsules.get_mut(capsule.capsule_id) {
                capsule.radius = new_radius;
            }
        }
    }

    fn update_capsules(&mut self, ctx: &egui::Context, pointer_pos: Pos2) {
        self.handle_capsule_selection(ctx, pointer_pos);
        self.update_selected_capsules();
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
        self.selected_capsule_points = self
            .selected_capsule_points
            .iter()
            .map(|point| {
                if let Some(capsule) = self
                    .robot
                    .borrow()
                    .capsules
                    .iter()
                    .find(|c| c.id == point.capsule_id)
                {
                    match point.point_type.as_str() {
                        "x1" => SelectedCapsulePoint {
                            capsule_id: point.capsule_id,
                            point_type: point.point_type.clone(),
                            x: capsule.x1,
                            y: capsule.y1,
                        },
                        "x2" => SelectedCapsulePoint {
                            capsule_id: point.capsule_id,
                            point_type: point.point_type.clone(),
                            x: capsule.x2,
                            y: capsule.y2,
                        },
                        _ => SelectedCapsulePoint {
                            capsule_id: point.capsule_id,
                            point_type: point.point_type.clone(),
                            x: capsule.x1,
                            y: capsule.y1,
                        },
                    }
                } else {
                    point.clone()
                }
            })
            .collect();
    }

    pub fn on_editing_state_changed(&mut self, editing_state: EditingState) {
        match editing_state {
            EditingState::Create | EditingState::Update | EditingState::Delete => {
                self.clear_capsule_selection();
            }
        }
    }

    // pub fn get_capsule_render_data(
    //     &self,
    //     pointer_pos: Pos2,
    //     editing_state: EditingState,
    // ) -> Vec<CapsuleRenderData> {
    //     let selected_capsule_ids: Vec<usize> = self
    //         .selected_capsules
    //         .iter()
    //         .map(|capsule| capsule.capsule_id)
    //         .collect();

    //     self.robot
    //         .borrow()
    //         .capsules
    //         .iter()
    //         .map(|capsule| {
    //             let (circle1_color, circle2_color, body_color) =
    //                 self.get_capsule_colors(capsule, pointer_pos, editing_state, &selected_capsule_ids);
    //             CapsuleRenderData {
    //                 capsule: *capsule,
    //                 circle1_color,
    //                 circle2_color,
    //                 body_color,
    //             }
    //         })
    //         .collect()
    // }

    pub fn get_capsule_colors(
        &self,
        capsule: &Capsule,
        pointer_pos: Pos2,
        // editing_state: EditingState,
        // selected_capsule_ids: &[usize],
    ) -> (CapsuleColors, CapsuleColors, CapsuleColors) {
        let is_endcap1_selected = self
            .selected_capsule_points
            .iter()
            .any(|point| point.capsule_id == capsule.id && point.point_type == "x1");
        let is_endcap2_selected = self
            .selected_capsule_points
            .iter()
            .any(|point| point.capsule_id == capsule.id && point.point_type == "x2");

        let is_endcap1_hovered = capsule.is_inside_circle1(pointer_pos.x, pointer_pos.y);
        let is_endcap2_hovered = capsule.is_inside_circle2(pointer_pos.x, pointer_pos.y);

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

        (endcap1_color, endcap2_color, body_color)
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

        // let selected_capsule_ids: Vec<usize> = self
        //     .selected_capsule_points
        //     .iter()
        //     .map(|p| p.capsule_id)
        //     .collect();

        for capsule in &robot.capsules {
            let (endcap1_color, endcap2_color, body_color) =
                self.get_capsule_colors(capsule, pointer_pos);
            capsule.draw(
                painter,
                endcap1_color.to_color32(),
                endcap2_color.to_color32(),
                body_color.to_color32(),
            );
        }

        // Draw joints
        for joint in &robot.joints {
            joint.draw(painter, Color32::BLUE);
        }

        self.draw_overlapping_capsules(painter);
        self.draw_editing_visualization(painter, pointer_pos, editing_state, capsule_radius);
    }

    // fn draw_selected_capsules(&self, painter: &egui::Painter) {
    //     for capsule_id in &self.selected_capsule_points {
    //         match self.robot.try_borrow() {
    //             Ok(robot) => {
    //                 if let Some(capsule) = robot
    //                     .capsules
    //                     .iter()
    //                     .find(|c| c.id == capsule_id.capsule_id)
    //                 {
    //                     capsule.draw_one_color(painter, Color32::LIGHT_RED);
    //                 }
    //             }
    //             Err(_) => eprintln!("Could not borrow robot"),
    //         }
    //     }
    // }

    fn draw_overlapping_capsules(&self, painter: &egui::Painter) {
        match self.robot.try_borrow() {
            Ok(robot) => {
                for overlapping in &self.overlapping_capsules {
                    if let (Some(capsule1), Some(capsule2)) = (
                        robot.capsules.get(overlapping.capsule1_id),
                        robot.capsules.get(overlapping.capsule2_id),
                    ) {
                        painter.line_segment(
                            [capsule1.get_center(), capsule2.get_center()],
                            Stroke::new(2.0, Color32::RED),
                        );
                    }
                }
            }
            Err(_) => eprintln!("Could not borrow robot"),
        }
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

    // fn draw_update_visualization(
    //     &self,
    //     painter: &egui::Painter,
    //     pointer_pos: Pos2,
    // ) {
    //     match self.robot.try_borrow() {
    //         Ok(robot) => {
    //             for capsule in &robot.capsules {
    //                 if capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y) {
    //                     let stroke = Stroke::new(2.0, Color32::LIGHT_BLUE);
    //                     painter.rect_stroke(capsule.get_rect(), 5.0, stroke);
    //                 }
    //             }
    //         }
    //         Err(_) => eprintln!("Could not borrow robot"),
    //     }
    // }

    // fn draw_delete_visualization(
    //     &self,
    //     painter: &egui::Painter,
    //     robot: &Robot,
    //     pointer_pos: Pos2,
    // ) {
    //     for capsule in &robot.borrow().capsules {
    //         if capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y) {
    //             let stroke = Stroke::new(2.0, Color32::RED);
    //             painter.rect_stroke(capsule.get_rect(), 5.0, stroke);
    //         }
    //     }
    // }
}

#[derive(Debug, Clone, Copy)]
pub enum CapsulePtsSelected {
    Circle1,
    Circle2,
    Body,
}

#[derive(Debug, Clone, Copy)]
pub enum CapsuleColors {
    Default,
    Highlighted,
    Selected,
    Add,
}

impl CapsuleColors {
    pub fn to_color32(&self) -> Color32 {
        match self {
            CapsuleColors::Default => Color32::GREEN,
            CapsuleColors::Highlighted => Color32::LIGHT_BLUE,
            CapsuleColors::Selected => Color32::LIGHT_RED,
            CapsuleColors::Add => Color32::from_rgb(230, 230, 250),
        }
    }
}
