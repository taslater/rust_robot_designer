use std::collections::{HashMap, HashSet};

use rapier2d::dynamics::RigidBodyHandle;

use crate::brain::{get_brain, Sequential};
use crate::model::robot::Robot;
use crate::physics_world::{flat_ground_collider, PhysicsWorld, RigidBodyObservation};
use crate::robot_physics::RobotPhysics;

// const POPULATION_SIZE: usize = 10;
use crate::constants::{BEST_RATIO, MUTATION_AMOUNT, MUTATION_RATE, POPULATION_SIZE, STEPS_PER_GENERATION};

fn get_observations(
    physics_observations: &HashMap<RigidBodyHandle, RigidBodyObservation>,
    capsule_handles: &HashMap<usize, RigidBodyHandle>,
) -> Vec<f32> {
    let mut observations: Vec<f32> = Vec::new();
    // iterate over capsule_handles
    for (_capsule_id, rigid_body_handle) in capsule_handles.iter() {
        let rigid_body_observation = physics_observations.get(rigid_body_handle).unwrap();
        // pub struct RigidBodyObservation {
        //     pub y: f32,
        //     pub sin: f32,
        //     pub cos: f32,
        //     pub vel_x: f32,
        //     pub vel_y: f32,
        //     pub angvel: f32,
        // }
        observations.push(rigid_body_observation.y);
        observations.push(rigid_body_observation.sin);
        observations.push(rigid_body_observation.cos);
        observations.push(rigid_body_observation.vel_x);
        observations.push(rigid_body_observation.vel_y);
        observations.push(rigid_body_observation.angvel);
    }
    observations
}

fn get_evaluations(
    physics_evaluations: &HashMap<RigidBodyHandle, f32>,
    capsule_handles: &HashMap<usize, RigidBodyHandle>,
) -> Vec<f32> {
    let mut evaluations: Vec<f32> = Vec::new();
    for (_capsule_id, rigid_body_handle) in capsule_handles.iter() {
        let rigid_body_evaluation = physics_evaluations.get(rigid_body_handle).unwrap();
        evaluations.push(*rigid_body_evaluation);
    }
    evaluations
}

pub struct RobotTrainer {
    physics_world: PhysicsWorld,
    robots: Vec<Robot>,
    robots_physics: Vec<RobotPhysics>,
    brains: Vec<Sequential>,
    is_playing: bool,
    generation_step: usize,
}

impl RobotTrainer {
    pub fn new() -> Self {
        RobotTrainer {
            physics_world: PhysicsWorld::new(),
            robots: Vec::with_capacity(POPULATION_SIZE),
            robots_physics: Vec::with_capacity(POPULATION_SIZE),
            brains: Vec::with_capacity(POPULATION_SIZE),
            is_playing: false,
            generation_step: 0,
        }
    }

    // fn clear(&mut self) {
    //     self.physics_world.clear();
    //     self.robot_physics.clear();
    // }

    fn clear(&mut self) {
        self.physics_world.clear();
        self.robots.clear();
        self.robots_physics.clear();
        // self.brains.clear();
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

        let n_outputs: usize = robot.joints_count();
        let all_positions_velocities_angles: std::collections::HashMap<
            RigidBodyHandle,
            RigidBodyObservation,
        > = self.physics_world.get_all_rigid_body_observations();
        let observations = get_observations(
            &all_positions_velocities_angles,
            robots_physics[0].get_capsule_handles(),
        );
        let n_inputs: usize = observations.len();

        for _ in 0..POPULATION_SIZE {
            self.brains.push(get_brain(
                n_inputs,
                vec![2 * n_inputs, 2 * n_inputs],
                n_outputs,
            ));
        }

        self.robots = robots;
        self.robots_physics = robots_physics;

        let _ = self.physics_world.add_collider(flat_ground_collider());
    }

    pub fn evaluate_and_update(&mut self) {
        let all_rigid_body_evaluations: HashMap<RigidBodyHandle, f32> =
            self.physics_world.get_all_rigid_body_evaluations();

        let mut fitnesses: Vec<f32> = Vec::with_capacity(POPULATION_SIZE);
        for i in 0..POPULATION_SIZE {
            let robot_physics = &self.robots_physics[i];
            let evaluations = get_evaluations(
                &all_rigid_body_evaluations,
                robot_physics.get_capsule_handles(),
            );
            // take the mean of the evaluations
            let evaluation: f32 = evaluations.iter().sum::<f32>() / evaluations.len() as f32;
            fitnesses.push(evaluation);
        }
        // // clone of fitnesses sorted for print
        // let mut sorted_fitnesses = fitnesses.clone();
        // sorted_fitnesses.sort_by(|a, b| b.partial_cmp(a).unwrap());
        // println!("Fitnesses: {:?}", sorted_fitnesses);

        // use BEST_RATIO to get the N best brains
        let n_best: usize = (BEST_RATIO * POPULATION_SIZE as f32) as usize;
        let mut best_flat_weights_and_biases: Vec<Vec<f64>> = Vec::with_capacity(n_best);
        let mut best_indices: Vec<usize> = (0..fitnesses.len()).collect();
        // sort from largest to smallest
        best_indices.sort_by(|&i, &j| fitnesses[i].partial_cmp(&fitnesses[j]).unwrap());

        print!("Best fitnesses: ");
        for _ in 0..n_best {
            let i = best_indices.pop().unwrap();
            print!("{}, ", fitnesses[i]);
            best_flat_weights_and_biases.push(self.brains[i].get_flat_weights_and_biases());
        }
        println!();

        // mutate the best brains
        for i in 0..POPULATION_SIZE {
            let mut new_brain = self.brains[i].clone();
            let best_brain = &best_flat_weights_and_biases[i % n_best];
            new_brain.set_weights_and_biases(best_brain.clone());
            new_brain.mutate(MUTATION_RATE, MUTATION_AMOUNT);
            self.brains[i] = new_brain;
        }

        // reset the robots
        for i in 0..POPULATION_SIZE {
            let robot_physics = &mut self.robots_physics[i];
            robot_physics.reset_robot(&mut self.physics_world);
        }
    }

    fn update(&mut self) {
        if !self.is_playing {
            return;
        }

        let all_positions_velocities_angles: std::collections::HashMap<
            RigidBodyHandle,
            RigidBodyObservation,
        > = self.physics_world.get_all_rigid_body_observations();

        // for robot in &mut self.robots {
        //     robot.update_motors(&mut self.physics_world);
        // }

        for i in 0..self.robots.len() {
            let robot = &self.robots[i];
            let robot_physics = &self.robots_physics[i];
            let observations = get_observations(
                &all_positions_velocities_angles,
                robot_physics.get_capsule_handles(),
            );
            let outputs_mat: nalgebra::Matrix<
                f64,
                nalgebra::Dyn,
                nalgebra::Dyn,
                nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Dyn>,
            > = self.brains[i].forward(&observations);
            // convert outputs_mat to Vec<f32>
            let outputs: Vec<f32> = outputs_mat.iter().map(|&v| v as f32).collect();
            for (joint_id, output) in robot.get_joints().iter().zip(outputs) {
                self.physics_world.set_impulse_joint_motor_direction(
                    robot_physics.get_impulse_joint_handle(joint_id.id),
                    output,
                );
            }
        }

        self.physics_world.step();

        for (robot, robot_physics) in self.robots.iter_mut().zip(&self.robots_physics) {
            robot_physics.update_robot_physics(robot, &self.physics_world);
        }

        self.generation_step += 1;

        if self.generation_step >= STEPS_PER_GENERATION {
            self.generation_step = 0;
            self.evaluate_and_update();
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
                    &HashSet::new(),
                    &Vec::new(),
                );
            }
            ui.ctx().request_repaint();
        });
    }
}
