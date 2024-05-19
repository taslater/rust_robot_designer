use crate::brain::Sequential;
use crate::model::robot::Robot;
use crate::physics_world::{flat_ground_collider, PhysicsWorld};
use crate::robot_physics::RobotPhysics;

const POPULATION_SIZE: usize = 10;

pub struct RobotTrainer {
    physics_world: PhysicsWorld,
    robots: Vec<Robot>,
    robots_physics: Vec<RobotPhysics>,
    brains: Vec<Sequential>,
    is_playing: bool,
}

impl RobotTrainer {
    pub fn new() -> Self {
        RobotTrainer {
            physics_world: PhysicsWorld::new(),
            robots: Vec::with_capacity(POPULATION_SIZE),
            robots_physics: Vec::with_capacity(POPULATION_SIZE),
            brains: Vec::with_capacity(POPULATION_SIZE),
            is_playing: false,
        }
    }

    fn clear(&mut self) {
        self.physics_world.clear();
    }

    pub fn init_physics(&mut self, robot: &Robot) {
        self.clear();

        let mut robots: Vec<Robot> = Vec::with_capacity(POPULATION_SIZE);
        let mut robots_physics: Vec<RobotPhysics> = Vec::with_capacity(POPULATION_SIZE);

        for _ in 0..POPULATION_SIZE {
            let mut robot = robot.clone();
            let mut robot_physics = RobotPhysics::new();
            robot_physics.build_robot(&mut robot, &mut self.physics_world);
            robots.push(robot);
            robots_physics.push(robot_physics);
        }

        // let n_joints: usize = robot.joints_count();

        self.robots = robots;
        self.robots_physics = robots_physics;

        let _ = self.physics_world.add_collider(flat_ground_collider());
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

        let all_positions_velocities_angles: std::collections::HashMap<
            rapier2d::prelude::RigidBodyHandle,
            crate::physics_world::RigidBodyObservation,
        > = self
            .physics_world
            .get_all_rigid_body_positions_velocities_angles();

        for robot in &mut self.robots {
            robot.update_motors(&mut self.physics_world);
        }

        self.physics_world.step();

        for (robot, robot_physics) in self.robots.iter_mut().zip(&self.robots_physics) {
            robot_physics.update_robot_physics(robot, &self.physics_world);
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
            for robot in &self.robots {
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
