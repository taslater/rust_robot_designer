use super::common::EditingState;
use crate::model::capsule::{Capsule, PointInsideCapsule};
use eframe::egui;
use egui::{Color32, Pos2};

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

pub struct CapsuleRenderData {
    pub capsule: Capsule,
    pub circle1_color: CapsuleColors,
    pub circle2_color: CapsuleColors,
    pub body_color: CapsuleColors,
}

pub struct CapsuleEditor {
    pub capsules: Vec<Capsule>,
    pub radius: f32,
    is_dragging: bool,
    pub create_capsule_start_point: Option<Pos2>,
    pub selected_capsule_points: Vec<SelectedCapsulePoint>,
    selected_capsules: Vec<SelectedCapsule>,
    pub overlapping_capsules: Vec<OverlappingCapsules>,
    capsule_drag_offsets: Vec<Pos2>,
}

impl Default for CapsuleEditor {
    fn default() -> Self {
        CapsuleEditor {
            capsules: Vec::new(),
            radius: 20.0,
            is_dragging: false,
            create_capsule_start_point: None,
            selected_capsule_points: Vec::new(),
            selected_capsules: Vec::new(),
            overlapping_capsules: Vec::new(),
            capsule_drag_offsets: Vec::new(),
        }
    }
}

impl CapsuleEditor {
    fn create_capsule(&mut self, pointer_pos: Pos2) {
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
        } else {
            self.create_capsule_start_point = Some(pointer_pos);
        }
    }

    fn delete_capsules(&mut self, pointer_pos: Pos2) {
        self.capsules
            .retain(|capsule| !capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y));
    }

    fn drag_capsules(&mut self, pointer_pos: Pos2) {
        for (index, selected_capsule) in self.selected_capsules.iter().enumerate() {
            if let Some(capsule) = self.capsules.get_mut(selected_capsule.capsule_id) {
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
        self.overlapping_capsules = overlapping_capsules;
    }

    fn handle_capsule_selection(&mut self, ctx: &egui::Context, pointer_pos: Pos2) {
        let clicked_points: Vec<SelectedCapsulePoint> = self
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
            if let Some(capsule) = self.capsules.get(selected_capsule.capsule_id) {
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
        if response.dragged() && !self.selected_capsules.is_empty() {
            if self.is_drag_start_inside_selected(pointer_pos) {
                if !self.is_dragging {
                    self.set_drag_offsets(pointer_pos);
                    self.start_dragging();
                }
                self.drag_capsules(pointer_pos);
            }
        } else if response.drag_stopped() {
            self.update_selected_capsule_points();
            self.stop_dragging();
        }
    }

    fn update_capsules(&mut self, ctx: &egui::Context, pointer_pos: Pos2) {
        self.handle_capsule_selection(ctx, pointer_pos);
        self.update_selected_capsules();
        self.update_overlapping_capsules();
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
            if let Some(capsule) = self.capsules.get_mut(capsule.capsule_id) {
                capsule.radius = new_radius;
            }
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
                if let Some(capsule) = self.capsules.iter().find(|c| c.id == point.capsule_id) {
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

    pub fn get_capsule_render_data(
        &self,
        pointer_pos: Pos2,
        editing_state: EditingState,
    ) -> Vec<CapsuleRenderData> {
        self.capsules
            .iter()
            .map(|capsule| {
                let (circle1_color, circle2_color, body_color) =
                    self.get_capsule_colors(capsule, pointer_pos, editing_state, false);
                CapsuleRenderData {
                    capsule: *capsule,
                    circle1_color,
                    circle2_color,
                    body_color,
                }
            })
            .collect()
    }

    pub fn get_capsule_colors(
        &self,
        capsule: &Capsule,
        pointer_pos: Pos2,
        editing_state: EditingState,
        is_selected: bool,
    ) -> (CapsuleColors, CapsuleColors, CapsuleColors) {
        if is_selected {
            return (CapsuleColors::Selected, CapsuleColors::Selected, CapsuleColors::Selected);
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
                    .any(|point| point.capsule_id == capsule.id && point.point_type == "x1");
                let is_endcap2_selected = self
                    .selected_capsule_points
                    .iter()
                    .any(|point| point.capsule_id == capsule.id && point.point_type == "x2");
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
