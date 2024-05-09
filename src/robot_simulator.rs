use crate::model::robot::Robot;
use crate::physics_world::{to_rendering_coords, PhysicsWorld, flat_ground_collider};
use crate::robot_physics_builder::RobotPhysicsBuilder;
use egui::pos2;
use rapier2d::prelude::*;
use std::collections::HashMap;

const STEP_COUNT: usize = 500; // Number of steps before flipping direction

pub(crate) struct RobotSimulator {
    robot: Robot,
    robot_physics_map: RobotPhysicsMap,
    physics_world: PhysicsWorld,
    is_playing: bool,
    step_counter: usize,
    motor_direction: f32,
}

impl RobotSimulator {
    pub fn new() -> Self {
        RobotSimulator {
            robot: Robot::new(),
            robot_physics_map: RobotPhysicsMap::new(),
            physics_world: PhysicsWorld::new(),
            is_playing: false,
            step_counter: 0,
            motor_direction: 1.0,
        }
    }

    fn clear(&mut self) {
        self.physics_world.clear();
        self.robot_physics_map.clear();
    }

    pub fn init_physics(&mut self, robot: &Robot) {
        self.clear();
        self.robot = robot.clone();

        let robot_handles =
            RobotPhysicsBuilder::build_robot(&mut self.robot, &mut self.physics_world);
        // Populate the robot_physics_map
        self.robot_physics_map.capsule_id_to_handle = robot_handles.capsule_handles;
        self.robot_physics_map.joint_id_to_handle = robot_handles.joint_handles;

        // Create the ground
        let _ = self.physics_world.add_collider(flat_ground_collider());
    }

    fn update(&mut self) {
        if !self.is_playing {
            return;
        }
        self.step_counter += 1;
        if self.step_counter % STEP_COUNT == 0 {
            self.motor_direction *= -1.0;
            self.step_counter = 0;
            println!("Flipping direction: {}", self.motor_direction);
        }

        let motor_directions: &[f32] = &vec![self.motor_direction; self.robot.get_joints().len()];
        // self.physics_world
        //     .update_impulse_joints(motor_directions);
        self.robot
            .update_joint_motor_directions(motor_directions, &mut self.physics_world);
        self.physics_world.step();

        // Update the robot's capsule positions based on the simulation
        for capsule in self.robot.get_capsules_mut() {
            if let Some(body_handle) = self.robot_physics_map.get_capsule(capsule.id) {
                if let Some(body) = self.physics_world.get_rigid_body(body_handle) {
                    let physics_position = body.position().translation;
                    // extremely important to add the initial rotation offset here
                    let rotation =
                        body.position().rotation.angle() + capsule.get_initial_rotation_offset();
                    let rendering_position =
                        to_rendering_coords(pos2(physics_position.x, physics_position.y));
                    let rendering_half_length = capsule.half_length();
                    capsule.update_endpoints(rendering_position, rendering_half_length, rotation);
                }
            }
        }
        // Update the robot's joint positions based on the simulation
        for joint in self.robot.get_joints_mut() {
            if let Some(impulse_joint_handle) = self.robot_physics_map.get_joint(joint.id) {
                if let Some(impulse_joint) =
                    self.physics_world.get_impulse_joint(impulse_joint_handle)
                {
                    let body1_pos: &nalgebra::Isometry<
                        f32,
                        nalgebra::Unit<nalgebra::Complex<f32>>,
                        2,
                    > = self
                        .physics_world
                        .rigid_body_set
                        .get(impulse_joint.body1)
                        .unwrap()
                        .position();
                    let local_frame1: nalgebra::Isometry<
                        f32,
                        nalgebra::Unit<nalgebra::Complex<f32>>,
                        2,
                    > = impulse_joint.data.local_frame1;
                    let local_combined1: nalgebra::Isometry<
                        f32,
                        nalgebra::Unit<nalgebra::Complex<f32>>,
                        2,
                    > = body1_pos * local_frame1;
                    let rendering_position1 = to_rendering_coords(pos2(
                        local_combined1.translation.vector.x,
                        local_combined1.translation.vector.y,
                    ));
                    joint.set_position(rendering_position1.x, rendering_position1.y);
                }
            }
        }
    }

    pub fn toggle_playback(&mut self) {
        self.is_playing = !self.is_playing;
    }

    pub fn reset(&mut self, robot: &Robot) {
        self.init_physics(robot);
        self.is_playing = false;
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, robot: &Robot) {
        ui.horizontal(|ui| {
            if ui
                .button(if self.is_playing { "Pause" } else { "Play" })
                .clicked()
            {
                self.toggle_playback();
            }
            if ui.button("Reset").clicked() {
                self.reset(robot);
            }
        });

        egui::Frame::canvas(ui.style()).show(ui, |ui| {
            self.update();
            self.robot.draw(
                ui.painter(),
                &Vec::new(),
                &Vec::new(),
                &Vec::new(),
                &Vec::new(),
            );
            ui.ctx().request_repaint();
        });
    }
}

pub struct RobotPhysicsMap {
    capsule_id_to_handle: HashMap<usize, RigidBodyHandle>,
    joint_id_to_handle: HashMap<usize, ImpulseJointHandle>,
}

impl RobotPhysicsMap {
    fn new() -> Self {
        RobotPhysicsMap {
            capsule_id_to_handle: HashMap::new(),
            joint_id_to_handle: HashMap::new(),
        }
    }

    fn clear(&mut self) {
        self.capsule_id_to_handle.clear();
        self.joint_id_to_handle.clear();
    }

    fn get_capsule(&self, capsule_id: usize) -> Option<RigidBodyHandle> {
        self.capsule_id_to_handle.get(&capsule_id).copied()
    }

    fn get_joint(&self, joint_id: usize) -> Option<ImpulseJointHandle> {
        self.joint_id_to_handle.get(&joint_id).copied()
    }
}
