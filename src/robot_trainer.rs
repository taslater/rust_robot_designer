use std::collections::{HashMap, HashSet};

use rapier2d::dynamics::RigidBodyHandle;

use crate::brain::{get_brain, Sequential};
use crate::cma_es::CMAES;
use crate::model::robot::Robot;
use crate::physics_world::{flat_ground_collider, PhysicsWorld, RigidBodyObservation};
use crate::robot_physics::{EvaluationData, RobotPhysics};
use nalgebra::DVector;

use crate::constants::STEPS_PER_GENERATION;

fn get_observations(
    physics_observations: &HashMap<RigidBodyHandle, RigidBodyObservation>,
    capsule_handles: &HashMap<usize, RigidBodyHandle>,
) -> Vec<f32> {
    let mut observations: Vec<f32> = Vec::new();
    // iterate over capsule_handles
    for (_capsule_id, rigid_body_handle) in capsule_handles.iter() {
        let rigid_body_observation: &RigidBodyObservation = physics_observations.get(rigid_body_handle).unwrap();
        observations.push(rigid_body_observation.y);
        observations.push(rigid_body_observation.sin);
        observations.push(rigid_body_observation.cos);
        observations.push(rigid_body_observation.vel_x);
        observations.push(rigid_body_observation.vel_y);
        observations.push(rigid_body_observation.angvel);
        // observations.push(rigid_body_observation.d_vel_x);
        // observations.push(rigid_body_observation.d_vel_y);
        // observations.push(rigid_body_observation.d_angvel);
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
    optimizer: Option<CMAES>,
    population: Vec<
        nalgebra::Matrix<
            f64,
            nalgebra::Dyn,
            nalgebra::Const<1>,
            nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>,
        >,
    >,
}

impl RobotTrainer {
    pub fn new() -> Self {
        RobotTrainer {
            physics_world: PhysicsWorld::new(),
            robots: Vec::new(),
            robots_physics: Vec::new(),
            brains: Vec::new(),
            is_playing: false,
            generation_step: 0,
            optimizer: None,
            population: Vec::new(),
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
        println!("Initializing physics");
        self.clear();

        let mut test_robot = robot.clone();
        let mut test_robot_physics = RobotPhysics::new();
        test_robot_physics.build_robot(&mut test_robot, &mut self.physics_world);

        let n_outputs: usize = test_robot.joints_count();
        let all_positions_velocities_angles: std::collections::HashMap<
            RigidBodyHandle,
            RigidBodyObservation,
        > = self.physics_world.get_all_rigid_body_observations();
        let observations = get_observations(
            &all_positions_velocities_angles,
            &test_robot_physics.get_capsule_handles(),
        );
        let n_inputs: usize = observations.len();
        println!("n_inputs: {}", n_inputs);
        println!("n_outputs: {}", n_outputs);

        let test_brain = get_brain(n_inputs, vec![1 * n_inputs, 1 * n_inputs], n_outputs);
        let test_flat: Vec<f64> = test_brain.get_flat_weights_and_biases();
        println!("n_weights_and_biases: {}", test_flat.len());

        let mean = DVector::from_element(test_flat.len(), 0.0);
        let sigma = 2.0;
        let population_size: Option<usize> = None;

        // let mut optimizer = CMAES::new(mean, sigma, population_multiplier);
        let mut optimizer = CMAES::new(mean, sigma, population_size);
        self.population = optimizer.ask();

        let pop_size: usize = self.population.len();
        println!("pop_size: {}", pop_size);

        self.clear();

        for brain_stuff in self.population.clone() {
            let mut brain = test_brain.clone();
            brain.set_weights_and_biases(brain_stuff.iter().map(|&v| v as f64).collect());
            let mut robot = robot.clone();
            let mut robot_physics = RobotPhysics::new();
            robot_physics.build_robot(&mut robot, &mut self.physics_world);
            self.robots.push(robot);
            self.robots_physics.push(robot_physics);
            self.brains.push(brain);
        }

        let _ = self.physics_world.add_collider(flat_ground_collider());
        self.optimizer = Some(optimizer);
    }

    pub fn evaluate_and_update(&mut self) {
        let all_rigid_body_evaluations: HashMap<RigidBodyHandle, f32> =
            self.physics_world.get_all_rigid_body_evaluations();

        let mut fitnesses: Vec<f32> = Vec::with_capacity(self.brains.len());
        let mut dist_parts: Vec<f32> = Vec::with_capacity(self.brains.len());
        let mut motion_parts: Vec<f32> = Vec::with_capacity(self.brains.len());
        let mut output_parts: Vec<f32> = Vec::with_capacity(self.brains.len());
        for i in 0..self.brains.len() {
            let robot_physics = &self.robots_physics[i];
            let evaluations = get_evaluations(
                &all_rigid_body_evaluations,
                &robot_physics.get_capsule_handles(),
            );
            // take the mean of the evaluations
            // let mut evaluation: f32 = 1e3
            //     / ((evaluations.iter().sum::<f32>() / evaluations.len() as f32).max(0.0) + 1e-6)
            //         .sqrt();
            let mut evaluation: f32 = evaluations.iter().sum::<f32>() / evaluations.len() as f32;
            evaluation *= evaluation * evaluation * 1e0;
            dist_parts.push(evaluation);
            let output_part: f32 = robot_physics.get_evaluation_data().output_sum;
            output_parts.push(output_part);
            evaluation -= output_part;
            // let evaluation_data: EvaluationData = robot_physics.get_evaluation_data();
            // let motion_part: f32 = 1e5 / evaluation_data.motion_sum.max(1e-6).sqrt() as f32;
            let motion_part: f32 = robot_physics.get_evaluation_data().motion_sum;
            motion_parts.push(motion_part);
            evaluation += motion_part;
            // fitnesses[i] = evaluation;
            fitnesses.push(-1.0 * evaluation);
        }
        // print min, mean, max of dist_parts
        let min_dist_part: f32 = dist_parts.iter().cloned().fold(f32::INFINITY, f32::min);
        let mean_dist_part: f32 = dist_parts.iter().sum::<f32>() / dist_parts.len() as f32;
        let max_dist_part: f32 = dist_parts.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        println!("min_dist_part:  {}", min_dist_part);
        println!("mean_dist_part: {}", mean_dist_part);
        println!("max_dist_part:  {}\n", max_dist_part);
        // print min, mean, max of motion_parts
        let min_motion_part: f32 = motion_parts.iter().cloned().fold(f32::INFINITY, f32::min);
        let mean_motion_part: f32 = motion_parts.iter().sum::<f32>() / motion_parts.len() as f32;
        let max_motion_part: f32 = motion_parts
            .iter()
            .cloned()
            .fold(f32::NEG_INFINITY, f32::max);
        println!("min_motion_part:  {}", min_motion_part);
        println!("mean_motion_part: {}", mean_motion_part);
        println!("max_motion_part:  {}\n", max_motion_part);
        let min_output_part: f32 = output_parts.iter().cloned().fold(f32::INFINITY, f32::min);
        let mean_output_part: f32 = output_parts.iter().sum::<f32>() / output_parts.len() as f32;
        let max_output_part: f32 = output_parts.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        println!("min_output_part:  {}", min_output_part);
        println!("mean_output_part: {}", mean_output_part);
        println!("max_output_part:  {}\n\n", max_output_part);

        // convert fitnesses to &[f64]
        let fitnesses: Vec<f64> = fitnesses.iter().map(|&v| v as f64).collect();

        let optimizer = self.optimizer.as_mut().unwrap();
        optimizer.tell(&self.population, &fitnesses);

        self.population = optimizer.ask();

        // update the brains
        for (i, brain_stuff) in self.population.iter().enumerate() {
            let brain = &mut self.brains[i];
            brain.set_weights_and_biases(brain_stuff.iter().map(|&v| v as f64).collect());
        }

        // reset the robots
        for i in 0..self.brains.len() {
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

        for i in 0..self.robots.len() {
            let robot = &self.robots[i];
            let robot_physics = &mut self.robots_physics[i];
            let observations = get_observations(
                &all_positions_velocities_angles,
                &robot_physics.get_capsule_handles(),
            );
            let outputs_mat: nalgebra::Matrix<
                f64,
                nalgebra::Dyn,
                nalgebra::Dyn,
                nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Dyn>,
            > = self.brains[i].forward(&observations);
            // convert outputs_mat to Vec<f32>
            let outputs: Vec<f32> = outputs_mat.iter().map(|&v| v as f32).collect();
            for ((joint_id, _joint), output) in robot.get_joints().iter().zip(outputs) {
                robot_physics.update_evaluation_data_output(output);
                self.physics_world.set_impulse_joint_motor_direction(
                    robot_physics.get_impulse_joint_handle(*joint_id),
                    output,
                );
            }
        }

        self.physics_world.step();

        for (robot, robot_physics) in self.robots.iter_mut().zip(&mut self.robots_physics) {
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
