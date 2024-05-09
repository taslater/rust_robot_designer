use crate::model::robot::Robot;
use crate::physics_world::{flat_ground_collider, PhysicsWorld};
use crate::robot_physics_builder::{RobotPhysicsBuilder, RobotPhysicsHandles};
use crate::robot_physics_updater::RobotPhysicsUpdater;

const STEP_COUNT: usize = 500; // Number of steps before flipping direction

pub(crate) struct RobotSimulator {
    robot: Robot,
    robot_physics_handles: RobotPhysicsHandles,
    physics_world: PhysicsWorld,
    is_playing: bool,
    step_counter: usize,
    motor_direction: f32,
}

impl RobotSimulator {
    pub fn new() -> Self {
        RobotSimulator {
            robot: Robot::new(),
            robot_physics_handles: RobotPhysicsHandles::new(),
            physics_world: PhysicsWorld::new(),
            is_playing: false,
            step_counter: 0,
            motor_direction: 1.0,
        }
    }

    fn clear(&mut self) {
        self.physics_world.clear();
        self.robot_physics_handles.clear();
    }

    pub fn init_physics(&mut self, robot: &Robot) {
        self.clear();
        self.robot = robot.clone();

        let robot_handles =
            RobotPhysicsBuilder::build_robot(&mut self.robot, &mut self.physics_world);
        // Populate the robot_physics_map
        self.robot_physics_handles.capsule_handles = robot_handles.capsule_handles;
        self.robot_physics_handles.joint_handles = robot_handles.joint_handles;

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
        self.robot
            .update_joint_motor_directions(motor_directions, &mut self.physics_world);
        self.physics_world.step();

        // Update the robot's capsule positions based on the simulation
        RobotPhysicsUpdater::update_robot_physics(
            &mut self.robot,
            &self.physics_world,
            &self.robot_physics_handles,
        );
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
