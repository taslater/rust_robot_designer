use crate::constants::GRAVITY;
use crate::model::robot::Robot;
use egui::Pos2;
use nalgebra::Point2;
use rapier2d::prelude::*;
use std::collections::HashMap;

const GROUND_WIDTH: f32 = 1000.0;
const GROUND_HEIGHT: f32 = 10.0;
const GROUND_RESTITUTION: f32 = 0.0;
const GROUND_X: f32 = 0.0;
const GROUND_Y: f32 = 400.0 - GROUND_HEIGHT / 2.0;

const STEP_COUNT: usize = 300; // Number of steps before flipping direction
const TARGET_VELOCITY: f32 = 1e3; // 1e6; // Velocity of the motor
const DAMPING_FACTOR: f32 = 1.0; // Damping factor for the motor

// const MOTOR_MAX_TORQUE: f32 = 1e6; // Max torque of the motor

pub(crate) struct RobotSimulator {
    robot: Robot,
    robot_physics_map: RobotPhysicsMap,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    event_handler: (),
    is_playing: bool,
    step_counter: usize,
    motor_direction: f32,
}

impl RobotSimulator {
    pub fn new() -> Self {
        let integration_parameters = IntegrationParameters {
            dt: 1.0 / 60.0,
            ..Default::default()
        };
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = BroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();
        let event_handler = ();

        RobotSimulator {
            robot: Robot::new(),
            robot_physics_map: RobotPhysicsMap::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            ccd_solver,
            query_pipeline,
            event_handler,
            is_playing: false,
            step_counter: 0,
            motor_direction: 1.0,
        }
    }

    fn get_pt_offset(&self, capsule_id: usize, pt: &Point2<f32>) -> Option<Point2<f32>> {
        let capsule_center_option: Option<Pos2> = self.robot.capsule_center(capsule_id);
        if let Some(capsule_center) = capsule_center_option {
            let offset_x: f32 = pt.x - capsule_center.x;
            let offset_y: f32 = pt.y - capsule_center.y;
            Some(Point2::new(offset_x, offset_y))
        } else {
            None
        }
    }

    fn clear(&mut self) {
        // delete all existing physics objects
        let impulse_joint_handles: Vec<ImpulseJointHandle> = self
            .impulse_joint_set
            .iter()
            .map(|(handle, _)| handle)
            .collect();
        for impulse_joint_handle in impulse_joint_handles {
            self.impulse_joint_set.remove(impulse_joint_handle, true);
        }
        let multibody_joint_handles: Vec<MultibodyJointHandle> = self
            .multibody_joint_set
            .iter()
            .map(|(handle, _, _, _)| handle)
            .collect();
        for multibody_joint_handle in multibody_joint_handles {
            self.multibody_joint_set
                .remove(multibody_joint_handle, true);
        }
        let collider_handles: Vec<ColliderHandle> =
            self.collider_set.iter().map(|(handle, _)| handle).collect();
        for collider_handle in collider_handles {
            self.collider_set.remove(
                collider_handle,
                &mut self.island_manager,
                &mut self.rigid_body_set,
                true,
            );
        }
        let rigid_body_handles: Vec<RigidBodyHandle> = self
            .rigid_body_set
            .iter()
            .map(|(handle, _)| handle)
            .collect();
        for rigid_body_handle in rigid_body_handles {
            self.rigid_body_set.remove(
                rigid_body_handle,
                &mut self.island_manager,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true,
            );
        }

        // clear the rest of the physics objects
        self.island_manager
            .cleanup_removed_rigid_bodies(&mut self.rigid_body_set);
        // clear the ccd solver
        self.ccd_solver.update_ccd_active_flags(
            // islands,
            &self.island_manager,
            // bodies,
            &mut self.rigid_body_set,
            // dt,
            self.integration_parameters.dt,
            // include_forces
            true,
        );

        // self.query_pipeline = QueryPipeline::new();
        // self.ccd_solver = CCDSolver::new();
        // self.narrow_phase = NarrowPhase::new();
        // self.broad_phase = BroadPhase::new();
        // self.island_manager = IslandManager::new();
        // self.physics_pipeline = PhysicsPipeline::new();

        // Clear existing physics objects
        // self.rigid_body_set = RigidBodySet::new();
        // self.collider_set = ColliderSet::new();
        // self.impulse_joint_set = ImpulseJointSet::new();
        // self.multibody_joint_set = MultibodyJointSet::new();
        self.robot_physics_map.clear();
    }

    pub fn init_physics(&mut self, robot: &Robot) -> &RobotPhysicsMap {
        self.clear();
        self.robot = robot.clone();

        // Create the capsules and populate the robot state
        for capsule in self.robot.get_capsules() {
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![capsule.center().x, capsule.center().y])
                .build();
            let (offset_x, offset_y) = capsule.offset_points();
            let pt_a: nalgebra::OPoint<f32, nalgebra::Const<2>> = point![offset_x, offset_y];
            let pt_b: nalgebra::OPoint<f32, nalgebra::Const<2>> = point![-offset_x, -offset_y];
            let collider =
                ColliderBuilder::new(SharedShape::capsule(pt_a, pt_b, capsule.radius / 2.0))
                    .collision_groups(InteractionGroups::new(0b0010.into(), 0b0001.into()))
                    .build();
            let body_handle: RigidBodyHandle = self.rigid_body_set.insert(rigid_body);
            self.collider_set
                .insert_with_parent(collider, body_handle, &mut self.rigid_body_set);
            self.robot_physics_map
                .insert_capsule(capsule.id, body_handle);
        }

        // Create the joints
        for joint in self.robot.get_joints() {
            let (capsule1_id, capsule2_id) = (joint.capsule1_id, joint.capsule2_id);
            let body_handle1: RigidBodyHandle =
                self.robot_physics_map.get_capsule(capsule1_id).unwrap();
            let body_handle2: RigidBodyHandle =
                self.robot_physics_map.get_capsule(capsule2_id).unwrap();
            let joint_pos: Point2<f32> = joint.position();
            let offset1_opt: Option<nalgebra::OPoint<f32, nalgebra::Const<2>>> =
                self.get_pt_offset(capsule1_id, &joint_pos);
            // early exit if the offset is None
            if offset1_opt.is_none() {
                continue;
            }
            let offset2_opt: Option<nalgebra::OPoint<f32, nalgebra::Const<2>>> =
                self.get_pt_offset(capsule2_id, &joint_pos);
            // early exit if the offset is None
            if offset2_opt.is_none() {
                continue;
            }
            let offset1: nalgebra::OPoint<f32, nalgebra::Const<2>> = offset1_opt.unwrap();
            let offset2: nalgebra::OPoint<f32, nalgebra::Const<2>> = offset2_opt.unwrap();
            if body_handle1 != body_handle2 {
                let revolute_joint: RevoluteJoint = RevoluteJointBuilder::new()
                    .local_anchor1(offset1)
                    .local_anchor2(offset2)
                    .motor_model(MotorModel::AccelerationBased)
                    .motor_max_force(f32::MAX)
                    .motor_velocity(TARGET_VELOCITY, DAMPING_FACTOR)
                    .limits([joint.min, joint.max])
                    .build();
                let impulse_joint_handle =
                    self.impulse_joint_set
                        .insert(body_handle1, body_handle2, revolute_joint, true);
                self.robot_physics_map
                    .insert_joint(joint.id, impulse_joint_handle);
            }
        }

        // Create the ground
        let ground_collider = ColliderBuilder::cuboid(GROUND_WIDTH, GROUND_HEIGHT)
            .translation(vector![GROUND_X, GROUND_Y])
            .restitution(GROUND_RESTITUTION)
            .collision_groups(InteractionGroups::new(0b0001.into(), 0b1111.into()))
            .build();
        self.collider_set.insert(ground_collider);

        &self.robot_physics_map
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
        for (_joint_handle, impulse_joint) in self.impulse_joint_set.iter_mut() {
            impulse_joint.data.set_motor_velocity(
                JointAxis::AngX,
                TARGET_VELOCITY * self.motor_direction,
                DAMPING_FACTOR,
            );
            // self.rigid_body_set
            //     .get_mut(impulse_joint.body1)
            //     .unwrap()
            //     .apply_torque_impulse(self.motor_direction * MOTOR_MAX_TORQUE, true);
            // self.rigid_body_set
            //     .get_mut(impulse_joint.body2)
            //     .unwrap()
            //     .apply_torque_impulse(-self.motor_direction * MOTOR_MAX_TORQUE, true);
        }

        self.physics_pipeline.step(
            &vector![0.0, GRAVITY],
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &self.event_handler,
        );

        // Update the robot's capsule positions based on the simulation
        for capsule in self.robot.get_capsules_mut() {
            if let Some(body_handle) = self.robot_physics_map.get_capsule(capsule.id) {
                if let Some(body) = self.rigid_body_set.get(body_handle) {
                    let position = body.position().translation;
                    // extremely important to add the initial rotation offset here
                    let rotation =
                        body.position().rotation.angle() + capsule.get_initial_rotation_offset();
                    let center = Pos2::new(position.x, position.y);
                    // let half_length = capsule.half_length();
                    // capsule.update_endpoints(center, half_length, rotation);
                    let half_length = capsule.half_length();
                    capsule.update_endpoints(center, half_length, rotation);
                }
            }
        }
        // Update the robot's joint positions based on the simulation
        for joint in self.robot.get_joints_mut() {
            if let Some(impulse_joint_handle) = self.robot_physics_map.get_joint(joint.id) {
                if let Some(impulse_joint) = self.impulse_joint_set.get(impulse_joint_handle) {
                    let body1: &RigidBody = self.rigid_body_set.get(impulse_joint.body1).unwrap();
                    let local_anchor1: nalgebra::OPoint<f32, nalgebra::Const<2>> =
                        impulse_joint.data.local_anchor1();
                    let trans1 = body1.position().translation;
                    joint.set_position(trans1.x + local_anchor1.x, trans1.y + local_anchor1.y);
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

    fn insert_capsule(&mut self, capsule_id: usize, handle: RigidBodyHandle) {
        self.capsule_id_to_handle.insert(capsule_id, handle);
    }

    fn insert_joint(&mut self, joint_id: usize, handle: ImpulseJointHandle) {
        self.joint_id_to_handle.insert(joint_id, handle);
    }

    fn get_capsule(&self, capsule_id: usize) -> Option<RigidBodyHandle> {
        self.capsule_id_to_handle.get(&capsule_id).copied()
    }

    fn get_joint(&self, joint_id: usize) -> Option<ImpulseJointHandle> {
        self.joint_id_to_handle.get(&joint_id).copied()
    }
}
