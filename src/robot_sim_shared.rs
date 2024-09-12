use crate::brain::Sequential;
use crate::model::robot::Robot;
use crate::physics_world::{PhysicsWorld, RigidBodyObservation};
use crate::robot_physics::RobotPhysics;
use crate::shared_config::SharedConfig;
use rapier2d::dynamics::RigidBodyHandle;
use std::collections::HashMap;

pub fn add_shared_ui(ui: &mut egui::Ui, shared_config: &mut SharedConfig) {
    // let shared_config: &mut shared_config::SharedConfig = &mut self.shared_config.lock().unwrap();

    ui.add(egui::Slider::new(&mut shared_config.output_weight, -1.0..=1.0).text("Output weight"));
    ui.add(egui::Slider::new(&mut shared_config.motion_weight, -1.0..=1.0).text("Motion weight"));
    ui.add(
        egui::Slider::new(&mut shared_config.distance_weight, -1.0..=1.0).text("Distance weight"),
    );
    ui.add(
        egui::Slider::new(&mut shared_config.steps_per_generation, 1..=4000)
            .text("Steps per generation"),
    );
}

pub fn get_robot_io_size(robot: &Robot) -> (usize, usize) {
    let mut test_robot = robot.clone();
    let mut test_robot_physics = RobotPhysics::new();
    let mut test_world = PhysicsWorld::new();
    test_robot_physics.build_robot(&mut test_robot, &mut test_world);

    let n_outputs = test_robot.joints_count();
    let all_observations = test_world.get_all_rigid_body_observations();
    let observations = test_robot_physics
        .get_capsule_handles()
        .iter()
        .flat_map(|(_capsule_id, rigid_body_handle)| {
            let observation = all_observations.get(rigid_body_handle).unwrap();
            vec![
                observation.y,
                observation.sin,
                observation.cos,
                observation.vel_x,
                observation.vel_y,
                observation.angvel,
            ]
        })
        .collect::<Vec<f32>>();

    let n_inputs = observations.len();

    (n_inputs, n_outputs)
}
pub struct Simulation {
    pub robot: Robot,
    pub robot_physics: RobotPhysics,
    pub physics_world: PhysicsWorld,
    pub brain: Sequential,
    shared_config: SharedConfig,
}

impl Simulation {
    pub fn new(mut robot: Robot, brain: Sequential, shared_config: SharedConfig) -> Self {
        let mut physics_world = PhysicsWorld::new();
        let mut robot_physics = RobotPhysics::new();
        robot_physics.build_robot(&mut robot, &mut physics_world);

        Simulation {
            robot,
            robot_physics,
            physics_world,
            brain,
            shared_config,
        }
    }

    pub fn step(&mut self) {
        let observations = self.get_observations();
        let outputs = self.brain.forward(&observations);
        self.apply_outputs(outputs.as_slice());
        self.physics_world.step();
        self.robot_physics
            .update_robot_physics(&mut self.robot, &self.physics_world);
    }

    pub fn get_observations(&mut self) -> Vec<f32> {
        let all_positions_velocities_angles: HashMap<RigidBodyHandle, RigidBodyObservation> =
            self.physics_world.get_all_rigid_body_observations();

        self.robot_physics
            .get_capsule_handles()
            .iter()
            .flat_map(|(_capsule_id, rigid_body_handle)| {
                let rigid_body_observation = all_positions_velocities_angles
                    .get(rigid_body_handle)
                    .expect("Failed to get rigid body observation");
                vec![
                    rigid_body_observation.y,
                    rigid_body_observation.sin,
                    rigid_body_observation.cos,
                    rigid_body_observation.vel_x,
                    rigid_body_observation.vel_y,
                    rigid_body_observation.angvel,
                ]
            })
            .collect()
    }

    fn apply_outputs(&mut self, outputs: &[f64]) {
        for ((_joint_id, joint), &output) in self.robot.get_joints().iter().zip(outputs) {
            if let Some(joint_handle) = joint.impulse_joint_handle {
                self.physics_world
                    .set_impulse_joint_motor_direction(joint_handle, output as f32);
            }
        }
    }
}

pub trait Evaluator {
    fn evaluate(&self, simulation: &Simulation) -> f32;
}

pub struct RobotEvaluator;

fn get_evaluations(
    physics_evaluations: &HashMap<RigidBodyHandle, f32>,
    capsule_handles: &HashMap<usize, RigidBodyHandle>,
) -> Vec<f32> {
    capsule_handles
        // .par_iter()
        .iter()
        .map(|(_capsule_id, rigid_body_handle)| {
            let rigid_body_evaluation = physics_evaluations.get(rigid_body_handle).unwrap();
            *rigid_body_evaluation
        })
        .collect()
}

impl Evaluator for RobotEvaluator {
    fn evaluate(&self, simulation: &Simulation) -> f32 {
        let shared_config: &SharedConfig = &simulation.shared_config;
        let all_rigid_body_evaluations: HashMap<RigidBodyHandle, f32> =
            simulation.physics_world.get_all_rigid_body_evaluations();
        // let all_rigid_body_evaluations: HashMap<RigidBodyHandle, f32> =
        //     physics_world.get_all_rigid_body_evaluations();
        // let robot_physics = simulation.robot_physics;
        let evaluations = get_evaluations(
            &all_rigid_body_evaluations,
            &simulation.robot_physics.get_capsule_handles(),
        );
        // let evaluations = simulation
        //     .robot_physics
        //     .get_capsule_handles()
        //     .iter()
        //     .map(|(_capsule_id, rigid_body_handle)| {
        //         *all_rigid_body_evaluations.get(rigid_body_handle).unwrap()
        //     })
        //     .collect::<Vec<f32>>();

        let mut evaluation: f32 = shared_config.distance_weight * evaluations.iter().sum::<f32>()
            / evaluations.len() as f32;

        // evaluation *= evaluation * evaluation;

        let output_part =
            shared_config.output_weight * simulation.robot_physics.get_evaluation_data().output_sum;
        evaluation += output_part;

        let motion_part =
            shared_config.motion_weight * simulation.robot_physics.get_evaluation_data().motion_sum;
        evaluation -= motion_part;

        -evaluation // Negative because we want to maximize fitness
    }
}

pub fn run_simulation_steps(
    simulation: &mut Simulation,
    steps: usize,
    evaluator: &dyn Evaluator,
) -> f32 {
    // let mut total_fitness = 0.0;

    for _ in 0..steps {
        simulation.step();
        // total_fitness += evaluator.evaluate(simulation);
    }

    // total_fitness / steps as f32
    evaluator.evaluate(simulation)
}

pub struct SearchResult {
    pub brain: Sequential,
    pub fitness: f32,
}
