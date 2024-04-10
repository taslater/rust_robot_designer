use super::common::EditingState;
use crate::model::capsule::{Capsule, PointInsideCapsule};

use eframe::egui;
use egui::{pos2, Pos2};

#[derive(Debug, Clone, Copy)]
pub enum CapsulePtsSelected {
    Circle1,
    Circle2,
    Body,
}

pub struct CapsuleEditor {
    pub capsules: Vec<Capsule>,
    pub radius: f32,
    is_dragging: bool,
    multiple_radii_selected: bool,
    pub create_capsule_start_point: Option<Pos2>,
    pub selected_capsule_points: Vec<(usize, String, f32, f32)>,
    selected_capsules: Vec<(usize, CapsulePtsSelected)>,
    pub overlapping_capsules: Vec<(usize, usize)>,
    capsule_drag_offsets: Vec<Pos2>,
}

impl Default for CapsuleEditor {
    fn default() -> Self {
        CapsuleEditor {
            capsules: Vec::new(),
            radius: 20.0,
            is_dragging: false,
            multiple_radii_selected: false,
            create_capsule_start_point: None,
            selected_capsule_points: Vec::new(),
            selected_capsules: Vec::new(),
            overlapping_capsules: Vec::new(),
            capsule_drag_offsets: Vec::new(),
        }
    }
}

impl CapsuleEditor {
    fn add_capsule(&mut self, pointer_pos: Pos2) {
        if let Some(start_point) = self.create_capsule_start_point {
            let capsule = Capsule {
                id: self.capsules.len(),
                radius: self.radius,
                x1: start_point.x,
                y1: start_point.y,
                x2: pointer_pos.x,
                y2: pointer_pos.y,
            };
            self.capsules.push(capsule);
            self.create_capsule_start_point = None;
        }
    }

    fn delete_capsules(&mut self, pointer_pos: Pos2) {
        self.capsules
            .retain(|capsule| !capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y));
    }

    fn drag_capsules(&mut self, pointer_pos: Pos2) {
        for (index, (capsule_id, point, _, _)) in self.selected_capsule_points.iter().enumerate() {
            if let Some(capsule) = self.capsules.iter_mut().find(|c| c.id == *capsule_id) {
                let offset = self.capsule_drag_offsets[index];
                if point == "x1" {
                    capsule.x1 = pointer_pos.x - offset.x;
                    capsule.y1 = pointer_pos.y - offset.y;
                } else if point == "x2" {
                    capsule.x2 = pointer_pos.x - offset.x;
                    capsule.y2 = pointer_pos.y - offset.y;
                }
            }
        }
    }

    fn update_selected_capsules_radius(&mut self, new_radius: f32) {
        for (capsule_id, _) in &self.selected_capsules {
            if let Some(capsule) = self.capsules.iter_mut().find(|c| c.id == *capsule_id) {
                capsule.radius = new_radius;
            }
        }
    }

    pub fn on_capsule_radius_slider_changed(&mut self, new_radius: f32) {
        self.update_selected_capsules_radius(new_radius);
    }

    fn update_capsule_slider(&mut self) {
        let selected_capsule_ids: Vec<usize> =
            self.selected_capsules.iter().map(|(id, _)| *id).collect();

        if selected_capsule_ids.len() == 1 {
            if let Some(selected_capsule) = self
                .capsules
                .iter()
                .find(|c| c.id == selected_capsule_ids[0])
            {
                self.radius = selected_capsule.radius;
                self.multiple_radii_selected = false;
            }
        } else if selected_capsule_ids.len() > 1 {
            let selected_capsules: Vec<&Capsule> = selected_capsule_ids
                .iter()
                .filter_map(|id| self.capsules.iter().find(|c| c.id == *id))
                .collect();

            let all_radii_equal = selected_capsules
                .windows(2)
                .all(|w| w[0].radius == w[1].radius);
            if all_radii_equal {
                if let Some(first_capsule) = selected_capsules.first() {
                    self.radius = first_capsule.radius;
                }
            }
            self.multiple_radii_selected = !all_radii_equal;
        } else {
            self.multiple_radii_selected = false;
        }
    }

    fn clear_capsule_selection(&mut self) {
        self.selected_capsule_points.clear();
        self.selected_capsules.clear();
        self.multiple_radii_selected = false;
    }

    pub fn clear_selections(&mut self) {
        self.clear_capsule_selection();
    }

    fn handle_create_capsule(&mut self, pointer_pos: Pos2) {
        if self.create_capsule_start_point.is_none() {
            self.create_capsule_start_point = Some(pointer_pos);
        } else {
            self.add_capsule(pointer_pos);
        }
    }

    fn handle_delete_capsules(&mut self, pointer_pos: Pos2) {
        self.delete_capsules(pointer_pos);
    }

    fn update_capsules(&mut self) {
        let mut selected_capsules: Vec<(usize, CapsulePtsSelected)> = Vec::new();
        for (point_capsule_id, point_str, _, _) in &self.selected_capsule_points {
            // check if point_capsule_id is already in selected_capsules
            let mut found = false;
            for (capsule_capsule_id, capsule_state_enum) in &mut selected_capsules {
                if *point_capsule_id == *capsule_capsule_id {
                    found = true;
                    match capsule_state_enum {
                        CapsulePtsSelected::Circle1 => {
                            if point_str == "x2" {
                                *capsule_state_enum = CapsulePtsSelected::Body;
                            }
                        }
                        CapsulePtsSelected::Circle2 => {
                            if point_str == "x1" {
                                *capsule_state_enum = CapsulePtsSelected::Body;
                            }
                        }
                        CapsulePtsSelected::Body => {}
                    }
                    break;
                }
            }
            if !found {
                let capsule_state_enum = match point_str.as_str() {
                    "x1" => CapsulePtsSelected::Circle1,
                    "x2" => CapsulePtsSelected::Circle2,
                    _ => CapsulePtsSelected::Body,
                };
                selected_capsules.push((*point_capsule_id, capsule_state_enum));
            }
        }
        self.selected_capsules = selected_capsules;

        let mut overlapping_capsules: Vec<(usize, usize)> = Vec::new();
        for (i, capsule1) in self.capsules.iter().enumerate() {
            for (j, capsule2) in self.capsules.iter().enumerate() {
                if i < j {
                    if capsule1.intersects_capsule(capsule2) {
                        overlapping_capsules.push((i, j));
                    }
                }
            }
        }
        self.overlapping_capsules = overlapping_capsules;
    }

    fn handle_update_capsules(&mut self, ctx: &egui::Context, pointer_pos: Pos2) {
        let clicked_points: Vec<(usize, String, f32, f32)> = self
            .capsules
            .iter()
            .flat_map(|capsule| {
                let inside_detail = capsule.is_inside_detail(pointer_pos.x, pointer_pos.y);
                match inside_detail {
                    PointInsideCapsule::InsideEndcap1 => {
                        vec![(capsule.id, "x1".to_string(), capsule.x1, capsule.y1)]
                    }
                    PointInsideCapsule::InsideEndcap2 => {
                        vec![(capsule.id, "x2".to_string(), capsule.x2, capsule.y2)]
                    }
                    PointInsideCapsule::InsideBody => {
                        vec![
                            (capsule.id, "x1".to_string(), capsule.x1, capsule.y1),
                            (capsule.id, "x2".to_string(), capsule.x2, capsule.y2),
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
                if let Some(index) = self
                    .selected_capsule_points
                    .iter()
                    .position(|point| point.0 == clicked_point.0 && point.1 == clicked_point.1)
                {
                    self.selected_capsule_points.remove(index);
                } else {
                    self.selected_capsule_points.push(clicked_point.clone());
                }
            }
        } else {
            self.selected_capsule_points = clicked_points;
        }

        self.update_capsule_slider();
    }

    fn handle_dragging(
        &mut self,
        response: &egui::Response,
        pointer_pos: Pos2,
        editing_state: EditingState,
    ) {
        if response.dragged() && editing_state == EditingState::Update {
            if !self.is_dragging {
                let mut drag_start_inside_selected = false;
                for (capsule_id, capsule_state_enum) in &self.selected_capsules {
                    if let Some(capsule) = self.capsules.iter().find(|c| c.id == *capsule_id) {
                        match capsule_state_enum {
                            CapsulePtsSelected::Circle1 => {
                                drag_start_inside_selected =
                                    capsule.is_inside_circle1(pointer_pos.x, pointer_pos.y);
                            }
                            CapsulePtsSelected::Circle2 => {
                                drag_start_inside_selected =
                                    capsule.is_inside_circle2(pointer_pos.x, pointer_pos.y);
                            }
                            CapsulePtsSelected::Body => {
                                drag_start_inside_selected =
                                    capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y);
                            }
                        }
                        if drag_start_inside_selected {
                            break;
                        }
                    }
                }

                if drag_start_inside_selected {
                    self.selected_capsule_points = self
                        .selected_capsule_points
                        .iter()
                        .map(|(id, point, _, _)| {
                            if let Some(capsule) = self.capsules.iter().find(|c| c.id == *id) {
                                match point.as_str() {
                                    "x1" => (*id, point.clone(), capsule.x1, capsule.y1),
                                    "x2" => (*id, point.clone(), capsule.x2, capsule.y2),
                                    _ => (*id, point.clone(), pointer_pos.x, pointer_pos.y),
                                }
                            } else {
                                (*id, point.clone(), 0.0, 0.0)
                            }
                        })
                        .collect();

                    self.capsule_drag_offsets = self
                        .selected_capsule_points
                        .iter()
                        .map(|(_, _, x, y)| pos2(pointer_pos.x - x, pointer_pos.y - y))
                        .collect();

                    self.is_dragging = true;
                }
            }

            if self.is_dragging {
                self.drag_capsules(pointer_pos);
            }
        } else if response.drag_stopped() {
            self.is_dragging = false;
            self.capsule_drag_offsets.clear();
            self.update_capsules();
        }
    }

    fn handle_capsule_delete_key(&mut self, ui: &mut egui::Ui, pointer_pos: Pos2) {
        if ui.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.delete_capsules(pointer_pos);
            self.update_capsules();
        }
    }
    pub fn update(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        editing_state: &mut EditingState,
        pointer_pos: Pos2,
        response: &egui::Response,
    ) {
        if response.clicked() {
            match *editing_state {
                EditingState::Create => self.handle_create_capsule(pointer_pos),
                EditingState::Delete => self.handle_delete_capsules(pointer_pos),
                EditingState::Update => self.handle_update_capsules(ctx, pointer_pos),
            }
            self.update_capsules();
        }

        self.handle_dragging(response, pointer_pos, *editing_state);
        self.handle_capsule_delete_key(ui, pointer_pos);
    }

    pub fn on_editing_state_changed(&mut self, editing_state: EditingState) {
        match editing_state {
            EditingState::Create | EditingState::Update | EditingState::Delete => {
                self.clear_capsule_selection();
            }
        }
    }
}
