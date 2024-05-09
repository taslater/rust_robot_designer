use eframe::egui;
use egui::epaint::Shape;
use egui::{pos2, Color32};
use rapier2d::dynamics::ImpulseJointHandle;
use rapier2d::na::Point2;

use crate::physics_world::PhysicsWorld;

#[derive(Debug, Clone, Copy)]
pub struct Joint {
    pub id: usize,
    pub x: f32,
    pub y: f32,
    pub capsule1_id: usize,
    pub capsule2_id: usize,
    pub min: f32,
    pub max: f32,
    pub impulse_joint_handle: Option<ImpulseJointHandle>,
}

const OUTLINE_COLOR: Color32 = Color32::from_rgb(0, 0, 0);
const OUTLINE_WIDTH: f32 = 1.0;
const RADIUS: f32 = 5.0;

impl Joint {
    pub fn position(&self) -> Point2<f32> {
        Point2::new(self.x, self.y)
    }

    pub fn set_position(&mut self, x: f32, y: f32) {
        self.x = x;
        self.y = y;
    }

    pub fn is_inside(&self, x: f32, y: f32) -> bool {
        let dx = self.x - x;
        let dy = self.y - y;
        let d_squared = dx * dx + dy * dy;
        d_squared < RADIUS * RADIUS
    }

    pub fn set_handle(&mut self, impulse_joint_handle: ImpulseJointHandle) {
        self.impulse_joint_handle = Some(impulse_joint_handle);
    }

    pub fn set_motor_direction(&mut self, motor_direction: f32, physics_world: &mut PhysicsWorld) {
        if let Some(impulse_joint_handle) = self.impulse_joint_handle {
            // let impulse_joint = physics_world
            //     .get_impulse_joint_mut(impulse_joint_handle)
            //     .unwrap();
            // impulse_joint.data.set_motor_velocity(
            //     JointAxis::AngX,
            //     TARGET_VELOCITY * motor_direction,
            //     MOTOR_DAMPING,
            // );
            physics_world.set_impulse_joint_motor_direction(impulse_joint_handle, motor_direction);
        }
    }

    pub fn draw(&self, painter: &egui::Painter, color: Color32) {
        painter.add(Shape::circle_filled(
            pos2(self.x, self.y),
            RADIUS,
            OUTLINE_COLOR,
        ));
        painter.add(Shape::circle_filled(
            pos2(self.x, self.y),
            RADIUS - OUTLINE_WIDTH,
            color,
        ));
    }
}
