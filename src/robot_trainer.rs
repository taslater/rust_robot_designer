use crate::model::robot::Robot;
use crate::physics_world::{to_rendering_coords, PhysicsWorld, flat_ground_collider};
use crate::robot_physics_builder::{RobotPhysicsBuilder, RobotPhysicsHandles};
use egui::pos2;
use rand::Rng;

const POPULATION_SIZE: usize = 10;

pub struct RobotTrainer {
    physics_world: PhysicsWorld,
    population: Vec<Robot>,
    population_handles: Vec<RobotPhysicsHandles>,
    is_playing: bool,
}

impl RobotTrainer {
    pub fn new() -> Self {
        RobotTrainer {
            physics_world: PhysicsWorld::new(),
            population: Vec::with_capacity(POPULATION_SIZE),
            population_handles: Vec::with_capacity(POPULATION_SIZE),
            is_playing: false,
        }
    }

    fn clear(&mut self) {
        self.physics_world.clear();
        // self.robot_physics_map.clear();
    }

    pub fn init_physics(&mut self, robot: &Robot) {
        println!("Initializing physics");
        self.clear();

        let mut population = Vec::with_capacity(POPULATION_SIZE);
        let mut population_handles: Vec<RobotPhysicsHandles> = Vec::with_capacity(POPULATION_SIZE);

        for _ in 0..POPULATION_SIZE {
            let mut robot = robot.clone();
            let robot_handles =
                RobotPhysicsBuilder::build_robot(&mut robot, &mut self.physics_world);
            population.push(robot);
            population_handles.push(robot_handles);
        }

        // Create the ground
        let _ = self.physics_world.add_collider(flat_ground_collider());

        self.population = population;
        self.population_handles = population_handles;
    }

    // pub fn train(&mut self, num_generations: usize) {
    //     for generation in 0..num_generations {
    //         println!("Generation: {}", generation);

    //         // TODO: Implement the training loop
    //         // 1. Create a population of robots
    //         // 2. Evaluate each robot in the population
    //         // 3. Select the best robots
    //         // 4. Create the next generation
    //     }
    // }

    // fn evaluate_robot(&mut self, robot: &mut Robot, num_steps: usize) -> f32 {
    //     // TODO: Implement the robot evaluation
    //     // 1. Reset the robot's position and the physics world
    //     // 2. Simulate the robot's behavior for a fixed number of steps
    //     // 3. Calculate the fitness score based on the robot's performance

    //     // Placeholder fitness score
    //     0.0
    // }

    fn update(&mut self) {
        if !self.is_playing {
            return;
        }

        // Generate random motor directions for each robot
        let mut motor_directions: Vec<Vec<f32>> = Vec::new();
        for robot_handle in &self.population_handles {
            let num_joints = robot_handle.joint_handles.len();
            let mut robot_motor_directions: Vec<f32> = Vec::with_capacity(num_joints);
            for _ in 0..num_joints {
                let random_direction = rand::thread_rng().gen_range(-1.0..=1.0);
                robot_motor_directions.push(random_direction);
            }
            motor_directions.push(robot_motor_directions);
        }

        // Update the impulse joints with the random motor directions
        for (robot_index, robot_motor_directions) in motor_directions.iter().enumerate() {
            // self.physics_world
            //     .update_impulse_joints(robot_motor_directions);
            // self.robot
            // .update_joint_motor_directions(motor_directions, &mut self.physics_world);
            self.population
                .get_mut(robot_index)
                .unwrap()
                .update_joint_motor_directions(robot_motor_directions, &mut self.physics_world);
        }

        self.physics_world.step();

        // Update the robots' positions and rotations based on the physics simulation
        for (robot, robot_handles) in self.population.iter_mut().zip(&self.population_handles) {
            // Update the robot's capsule positions and rotations
            for capsule in robot.get_capsules_mut() {
                if let Some(body_handle) = robot_handles.capsule_handles.get(&capsule.id) {
                    if let Some(body) = self.physics_world.get_rigid_body(*body_handle) {
                        let physics_position = body.position().translation;
                        let rotation = body.position().rotation.angle()
                            + capsule.get_initial_rotation_offset();
                        let rendering_position =
                            to_rendering_coords(pos2(physics_position.x, physics_position.y));
                        let rendering_half_length = capsule.half_length();
                        capsule.update_endpoints(
                            rendering_position,
                            rendering_half_length,
                            rotation,
                        );
                    }
                }
            }

            // Update the robot's joint positions
            for joint in robot.get_joints_mut() {
                if let Some(impulse_joint_handle) = robot_handles.joint_handles.get(&joint.id) {
                    if let Some(impulse_joint) =
                        self.physics_world.get_impulse_joint(*impulse_joint_handle)
                    {
                        let body1_pos = self
                            .physics_world
                            .get_rigid_body(impulse_joint.body1)
                            .unwrap()
                            .position();
                        let local_frame1 = impulse_joint.data.local_frame1;
                        let local_combined1 = body1_pos * local_frame1;
                        let rendering_position1 = to_rendering_coords(pos2(
                            local_combined1.translation.vector.x,
                            local_combined1.translation.vector.y,
                        ));
                        joint.set_position(rendering_position1.x, rendering_position1.y);
                    }
                }
            }
        }
    }

    fn toggle_playback(&mut self) {
        self.is_playing = !self.is_playing;
    }

    fn reset(&mut self, robot: &Robot) {
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
            for robot in &self.population {
                robot.draw(
                    ui.painter(),
                    &Vec::new(),
                    &Vec::new(),
                    &Vec::new(),
                    &Vec::new(),
                );
            }
            ui.ctx().request_repaint();
        });
    }
}
