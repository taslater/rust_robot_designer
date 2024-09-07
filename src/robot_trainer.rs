use crate::constants::{DISTANCE_WEIGHT, MOTION_WEIGHT, OUTPUT_WEIGHT, SIGMA_INITIAL};

use std::collections::{HashMap, HashSet};

use rapier2d::dynamics::RigidBodyHandle;

use crate::brain::{get_brain, Sequential};
use crate::cma_es::CMAES;
use crate::model::robot::Robot;
use crate::physics_world::{flat_ground_collider, PhysicsWorld, RigidBodyObservation};
use crate::robot_physics::RobotPhysics;
use nalgebra::DVector;

use crate::constants::STEPS_PER_GENERATION;

fn get_observations(
    physics_observations: &HashMap<RigidBodyHandle, RigidBodyObservation>,
    capsule_handles: &HashMap<usize, RigidBodyHandle>,
) -> Vec<f32> {
    capsule_handles
        .iter()
        .map(|(_capsule_id, rigid_body_handle)| {
            let rigid_body_observation: &RigidBodyObservation =
                match physics_observations.get(rigid_body_handle) {
                    // thread 'main' panicked at src/robot_trainer.rs:45:61:
                    // called `Option::unwrap()` on a `None` value
                    Some(observation) => observation,
                    None => panic!("Failed to get rigid body observation"),
                };
            vec![
                rigid_body_observation.y,
                rigid_body_observation.sin,
                rigid_body_observation.cos,
                rigid_body_observation.vel_x,
                rigid_body_observation.vel_y,
                rigid_body_observation.angvel,
            ]
        })
        .flatten()
        .collect()
}

fn get_evaluations(
    physics_evaluations: &HashMap<RigidBodyHandle, f32>,
    capsule_handles: &HashMap<usize, RigidBodyHandle>,
) -> Vec<f32> {
    capsule_handles
        .iter()
        .map(|(_capsule_id, rigid_body_handle)| {
            let rigid_body_evaluation = physics_evaluations.get(rigid_body_handle).unwrap();
            *rigid_body_evaluation
        })
        .collect()
}

pub struct RobotTrainer {
    physics_worlds: Vec<PhysicsWorld>,
    robots: Vec<Robot>,
    robots_physics: Vec<RobotPhysics>,
    brains: Vec<Sequential>,
    use_default_population_size: bool,
    custom_population_size: usize,

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
            physics_worlds: Vec::new(),
            robots: Vec::new(),
            robots_physics: Vec::new(),
            brains: Vec::new(),
            is_playing: false,
            generation_step: 0,
            optimizer: None,
            population: Vec::new(),
            use_default_population_size: true,
            custom_population_size: 64,
        }
    }

    fn clear(&mut self) {
        self.physics_worlds.iter_mut().for_each(|physics_world| {
            physics_world.clear();
        });
        self.physics_worlds.clear();
        self.robots.clear();
        self.robots_physics.clear();
    }

    pub fn init_physics(&mut self, robot: &Robot) {
        println!("Initializing physics");
        self.clear();

        let mut test_robot = robot.clone();
        let mut test_robot_physics = RobotPhysics::new();
        let mut test_world: PhysicsWorld = PhysicsWorld::new();
        test_robot_physics.build_robot(&mut test_robot, &mut test_world);

        let n_outputs: usize = test_robot.joints_count();
        let all_positions_velocities_angles: std::collections::HashMap<
            RigidBodyHandle,
            RigidBodyObservation,
        > = test_world.get_all_rigid_body_observations();
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

        let population_size: Option<usize> = if self.use_default_population_size {
            None
        } else {
            Some(self.custom_population_size)
        };

        let mut optimizer = CMAES::new(mean, SIGMA_INITIAL, population_size);
        self.population = optimizer.ask();

        let pop_size: usize = self.population.len();
        println!("pop_size: {}", pop_size);

        self.clear();

        self.population.iter().for_each(|brain_stuff| {
            let mut brain = test_brain.clone();
            brain.set_weights_and_biases(brain_stuff.iter().map(|&v| v as f64).collect());
            let mut robot = robot.clone();
            let mut robot_physics = RobotPhysics::new();
            let mut physics_world: PhysicsWorld = PhysicsWorld::new();
            let _ = physics_world.add_collider(flat_ground_collider());
            robot_physics.build_robot(&mut robot, &mut physics_world);
            self.robots.push(robot);
            self.robots_physics.push(robot_physics);
            self.brains.push(brain);
            self.physics_worlds.push(physics_world);
        });

        self.optimizer = Some(optimizer);
        println!(
            "Capsule handles after init: {:?}",
            self.robots_physics
                .iter()
                .map(|rp| rp.get_capsule_handles())
                .collect::<Vec<_>>()
        );
    }

    pub fn evaluate_and_update(&mut self) {
        let mut fitnesses: Vec<f32> = Vec::with_capacity(self.brains.len());
        let mut dist_parts: Vec<f32> = Vec::with_capacity(self.brains.len());
        let mut motion_parts: Vec<f32> = Vec::with_capacity(self.brains.len());
        let mut output_parts: Vec<f32> = Vec::with_capacity(self.brains.len());
        (0..self.brains.len()).for_each(|i| {
            let physics_world: &PhysicsWorld = &self.physics_worlds[i];
            let all_rigid_body_evaluations: HashMap<RigidBodyHandle, f32> =
                physics_world.get_all_rigid_body_evaluations();
            let robot_physics = &self.robots_physics[i];
            let evaluations = get_evaluations(
                &all_rigid_body_evaluations,
                &robot_physics.get_capsule_handles(),
            );
            let mut evaluation: f32 =
                DISTANCE_WEIGHT * evaluations.iter().sum::<f32>() / evaluations.len() as f32;
            evaluation *= evaluation * evaluation;
            dist_parts.push(evaluation);
            let output_part: f32 = OUTPUT_WEIGHT * robot_physics.get_evaluation_data().output_sum;
            output_parts.push(output_part);
            evaluation -= output_part;
            let motion_part: f32 = MOTION_WEIGHT * robot_physics.get_evaluation_data().motion_sum;
            motion_parts.push(motion_part);
            evaluation += motion_part;
            fitnesses.push(-1.0 * evaluation);
        });
        // print min, mean, max of dist_parts
        let min_dist_part: f32 = dist_parts.iter().cloned().fold(f32::INFINITY, f32::min);
        let mean_dist_part: f32 = dist_parts.iter().sum::<f32>() / dist_parts.len() as f32;
        let max_dist_part: f32 = dist_parts.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        println!("min_dist_part:  {}", min_dist_part);
        println!("mean_dist_part: {}", mean_dist_part);
        println!("max_dist_part:  {}\n", max_dist_part);
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
        let max_output_part: f32 = output_parts
            .iter()
            .cloned()
            .fold(f32::NEG_INFINITY, f32::max);
        println!("min_output_part:  {}", min_output_part);
        println!("mean_output_part: {}", mean_output_part);
        println!("max_output_part:  {}\n\n", max_output_part);

        // convert fitnesses to &[f64]
        let fitnesses: Vec<f64> = fitnesses.iter().map(|&v| v as f64).collect();

        let optimizer = self.optimizer.as_mut().unwrap();
        optimizer.tell(&self.population, &fitnesses);

        self.population = optimizer.ask();

        self.population
            .iter()
            .enumerate()
            .for_each(|(i, brain_stuff)| {
                self.brains
                    .get_mut(i)
                    .unwrap()
                    .set_weights_and_biases(brain_stuff.iter().map(|&v| v as f64).collect());
            });

        // reset the robots
        (0..self.brains.len()).for_each(|i| {
            let robot_physics = &mut self.robots_physics[i];
            let mut physics_world = &mut self.physics_worlds[i];
            robot_physics.reset_robot(&mut physics_world);
        });
    }

    fn update(&mut self) {
        if !self.is_playing {
            return;
        }

        (0..self.robots.len()).for_each(|i| {
            let physics_world = &mut self.physics_worlds[i];
            let all_positions_velocities_angles: std::collections::HashMap<
                RigidBodyHandle,
                RigidBodyObservation,
            > = physics_world.get_all_rigid_body_observations();
            let robot = &mut self.robots[i];
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
                physics_world.set_impulse_joint_motor_direction(
                    robot_physics.get_impulse_joint_handle(*joint_id),
                    output,
                );
            }
            physics_world.step();
            robot_physics.update_robot_physics(robot, &physics_world);
        });

        self.generation_step += 1;

        if self.generation_step >= STEPS_PER_GENERATION {
            self.generation_step = 0;
            self.evaluate_and_update();
        }
    }

    fn toggle_playback(&mut self, robot: &Robot) {
        self.is_playing = !self.is_playing;
        if self.is_playing {
            self.generation_step = 0;
            self.init_physics(robot);
        }
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
                self.toggle_playback(robot);
            }
            if ui.button("Reset").clicked() {
                self.reset(robot);
            }
        });

        egui::Frame::canvas(ui.style()).show(ui, |ui| {
            self.update();
            for robot in self.robots.iter() {
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

        // checkbox for using default population size or custom population size
        ui.horizontal(|ui| {
            ui.checkbox(
                &mut self.use_default_population_size,
                "Use default population size",
            );
            ui.add(
                egui::Slider::new(&mut self.custom_population_size, 1..=1000)
                    .text("Population size"),
            );
        });
    }
}
