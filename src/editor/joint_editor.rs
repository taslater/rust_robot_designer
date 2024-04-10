use super::common::{EditingState, JointCreationState};
use crate::model::capsule::Capsule;
use crate::model::joint::Joint;

use eframe::egui;
use egui::Pos2;

pub struct JointEditor {
    pub joints: Vec<Joint>,
    selected_joint: Option<usize>,
    selected_capsules: Vec<usize>,
    joint_creation_state: JointCreationState,
}

impl Default for JointEditor {
    fn default() -> Self {
        JointEditor {
            joints: Vec::new(),
            selected_joint: None,
            selected_capsules: Vec::new(),
            joint_creation_state: JointCreationState::SelectFirstCapsule,
        }
    }
}

impl JointEditor {
    pub fn update(
        &mut self,
        ui: &mut egui::Ui,
        // ctx: &egui::Context,
        editing_state: &mut EditingState,
        pointer_pos: Pos2,
        response: &egui::Response,
        capsules: &[Capsule],
    ) {
        if *editing_state == EditingState::Create && response.clicked() {
            println!("update method clicked");
            println!("before match {:?}", self.selected_capsules);
            match self.joint_creation_state {
                JointCreationState::SelectFirstCapsule => {
                    println!("Selecting first capsule");
                    ui.label("Select first capsule");
                    self.update_capsule_selection(pointer_pos, capsules, 1);
                }
                JointCreationState::SelectSecondCapsule => {
                    println!("Selecting second capsule");
                    ui.label("Select second capsule");
                    self.update_capsule_selection(pointer_pos, capsules, 2);
                }
                JointCreationState::PlaceJoint => {
                    println!("Place joint");
                    ui.label("Place joint in the intersection");
                    if response.clicked() {
                        self.handle_create_joint(pointer_pos, capsules, ui);
                    }
                }
            }
            println!("after match {:?}", self.selected_capsules);
        } else {
            if response.clicked() {
                match *editing_state {
                    EditingState::Delete => self.handle_delete_joint(pointer_pos),
                    EditingState::Update => self.handle_update_joint(pointer_pos),
                    _ => (),
                }
            }

            self.handle_joint_pointer_events(response, pointer_pos, *editing_state);
            self.handle_joint_delete_key(ui, pointer_pos);
        }
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
        self.selected_joint = None;
        self.selected_capsules.clear();
    }

    fn handle_create_joint(&mut self, pointer_pos: Pos2, capsules: &[Capsule], ui: &mut egui::Ui) {
        println!("Creating joint");
        if self.selected_capsules.len() == 2 && self.check_joint_placement(pointer_pos, capsules) {
            let joint = Joint {
                id: self.joints.len(),
                x: pointer_pos.x,
                y: pointer_pos.y,
                capsule1_id: self.selected_capsules[0],
                capsule2_id: self.selected_capsules[1],
                min: 0.0,
                max: 1.0,
            };
            self.joints.push(joint);
            self.clear_selections();
            self.joint_creation_state = JointCreationState::SelectFirstCapsule;
        } else {
            // Display an error message if the joint placement is invalid
            // You can use egui::Window or egui::Area to display the message
            // For example:
            egui::Window::new("Error")
                .collapsible(false)
                .resizable(false)
                .show(ui.ctx(), |ui| {
                    ui.label("The second capsule must intersect with the first capsule.");
                });
        }
    }

    fn handle_delete_joint(&mut self, pointer_pos: Pos2) {
        self.joints
            .retain(|joint| !joint.is_inside(pointer_pos.x, pointer_pos.y));
    }

    fn handle_update_joint(&mut self, pointer_pos: Pos2) {
        self.update_joint_selection(pointer_pos);
    }

    fn handle_joint_pointer_events(
        &mut self,
        response: &egui::Response,
        pointer_pos: Pos2,
        editing_state: EditingState,
    ) {
        if response.clicked() {
            println!("Handling click event");
            match editing_state {
                EditingState::Create => self.clear_selections(),
                EditingState::Update => self.update_joint_selection(pointer_pos),
                _ => (),
            }
        }
    }

    fn handle_joint_delete_key(&mut self, ui: &mut egui::Ui, pointer_pos: Pos2) {
        if ui.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.handle_delete_joint(pointer_pos);
        }
    }

    fn update_joint_selection(&mut self, pointer_pos: Pos2) {
        self.selected_joint = self
            .joints
            .iter()
            .position(|joint| joint.is_inside(pointer_pos.x, pointer_pos.y));
    }

    fn update_capsule_selection(
        &mut self,
        // ctx: &egui::Context,
        pointer_pos: Pos2,
        capsules: &[Capsule],
        max_selections: usize,
    ) {
        let clicked_capsule = capsules
            .iter()
            .position(|capsule| capsule.is_inside_at_all(pointer_pos.x, pointer_pos.y));

        println!("Clicked capsule {:?} for joint creation", clicked_capsule);
        if let Some(capsule_id) = clicked_capsule {
            //if let Some(index) = self.selected_capsules.iter().position(|&id| id == capsule_id) {
            if self.selected_capsules.contains(&capsule_id) {
                println!("Pick a second capsule or click the canvas to clear the selection");
            } else if self.selected_capsules.len() < max_selections {
                println!("Selecting capsule {}", capsule_id);
                self.selected_capsules.push(capsule_id);
            }

            if self.selected_capsules.len() == max_selections {
                println!("Selected capsules: {:?}", self.selected_capsules);
                self.joint_creation_state = match self.joint_creation_state {
                    JointCreationState::SelectFirstCapsule => {
                        JointCreationState::SelectSecondCapsule
                    }
                    JointCreationState::SelectSecondCapsule => JointCreationState::PlaceJoint,
                    _ => self.joint_creation_state,
                };
            }
        } else {
            println!("Clicked on the canvas to clear the selection");
            self.selected_capsules.clear();
            self.joint_creation_state = JointCreationState::SelectFirstCapsule;
        }
    }

    fn check_joint_placement(&self, pointer_pos: Pos2, capsules: &[Capsule]) -> bool {
        if let Some(capsule1) = self
            .selected_capsules
            .get(0)
            .and_then(|&id| capsules.get(id))
        {
            if let Some(capsule2) = self
                .selected_capsules
                .get(1)
                .and_then(|&id| capsules.get(id))
            {
                return capsule1.is_inside_at_all(pointer_pos.x, pointer_pos.y)
                    && capsule2.is_inside_at_all(pointer_pos.x, pointer_pos.y);
            }
        }
        false
    }
}
