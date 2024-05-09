use crate::model::robot::Robot;
use crate::physics_world::{to_rendering_coords, PhysicsWorld, flat_ground_collider};
use crate::robot_physics_builder::{RobotPhysicsBuilder, RobotPhysicsHandles};
use egui::pos2;
use crate::robot_physics_updater::RobotPhysicsUpdater;

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

        for robot in &mut self.population {
            robot.update_motors(&mut self.physics_world);
        }

        self.physics_world.step();

        for (robot, robot_handles) in self.population.iter_mut().zip(&self.population_handles) {
            RobotPhysicsUpdater::update_robot_physics(robot, &self.physics_world, robot_handles);
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
