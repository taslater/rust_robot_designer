use crate::brain::Sequential;
use crate::evaluation::CompositeEvaluator;
use crate::model::robot::Robot;
use crate::physics_world::{PhysicsWorld, RigidBodyObservation};
use crate::robot_physics::RobotPhysics;
use crate::shared_config::{SharedConfig, SharedConfigRef};
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
    shared_config: SharedConfigRef,
}

impl Simulation {
    pub fn new(mut robot: Robot, brain: Sequential, shared_config: SharedConfigRef) -> Self {
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
                self.robot_physics.update_evaluation_data_output(output as f32);
            }
        }
    }
}

pub fn run_simulation_steps(
    simulation: &mut Simulation,
    steps: usize,
    evaluator: &CompositeEvaluator,
) -> f32 {
    for _ in 0..steps {
        simulation.step();
    }

    // Evaluate the final state
    evaluator.evaluate(
        &simulation.physics_world,
        &simulation.robot_physics,
        &simulation.shared_config.lock().unwrap(),
    )
}

pub struct SearchResult {
    pub brain: Sequential,
    pub fitness: f32,
}
