// robot.rs
use crate::model::capsule::Capsule;
use crate::model::joint::Joint;
use eframe::egui;
use egui::Color32;

#[derive(Debug)]
pub struct Robot {
    pub capsules: Vec<Capsule>,
    pub joints: Vec<Joint>,
}

impl Robot {
    pub fn new() -> Self {
        Robot {
            capsules: Vec::new(),
            joints: Vec::new(),
        }
    }

    pub fn add_capsule(&mut self, capsule: Capsule) {
        self.capsules.push(capsule);
    }

    pub fn remove_capsule(&mut self, capsule_id: usize) {
        self.capsules.retain(|capsule| capsule.id != capsule_id);
    }

    pub fn update_capsule(&mut self, capsule: Capsule) {
        if let Some(c) = self.capsules.iter_mut().find(|c| c.id == capsule.id) {
            *c = capsule;
        }
    }

    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn remove_joint(&mut self, joint_id: usize) {
        self.joints.retain(|joint| joint.id != joint_id);
    }

    pub fn update_joint(&mut self, joint: Joint) {
        if let Some(j) = self.joints.iter_mut().find(|j| j.id == joint.id) {
            *j = joint;
        }
    }

    pub fn draw(&self, painter: &egui::Painter) {
        for capsule in &self.capsules {
            capsule.draw_one_color(painter, Color32::GREEN);
        }

        for joint in &self.joints {
            joint.draw(painter, Color32::BLUE);
        }
    }
}