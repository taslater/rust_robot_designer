use crate::model::{capsule, robot::Robot};
use egui::Pos2;
use nalgebra::Point2;
use rapier2d::prelude::*;
use std::collections::HashMap;

const GROUND_WIDTH: f32 = 1000.0;
const GROUND_HEIGHT: f32 = 10.0;
const GROUND_RESTITUTION: f32 = 0.0;
const GROUND_X: f32 = 0.0;
const GROUND_Y: f32 = 400.0 - GROUND_HEIGHT / 2.0;

pub(crate) struct RobotSimulator {
    robot: Robot,
    robot_state: RobotPhysicsMap,
    // capsule_data: Vec<CapsuleData>,
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
}

impl RobotSimulator {
    pub fn new() -> Self {
        println!("Creating RobotSimulator");
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
            robot_state: RobotPhysicsMap::new(),
            // capsule_data: Vec::new(),
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

    // fn create_capsule(
    //     hinge: &mut Hinge,
    //     capsule_index: usize,
    //     group: u32,
    //     rigid_body_set: &mut RigidBodySet,
    //     collider_set: &mut ColliderSet,
    // ) {
    //     let body = RigidBodyBuilder::dynamic()
    //         .translation(vector![hinge.origin.x, hinge.origin.y])
    //         .build();
    //     let offset_x: f32 =
    //         -hinge.capsules[capsule_index].angle.sin() * hinge.capsules[capsule_index].length / 2.0;
    //     let offset_y: f32 =
    //         hinge.capsules[capsule_index].angle.cos() * hinge.capsules[capsule_index].length / 2.0;
    //     let pt_a: OPoint<f32, nalgebra::Const<2>> = point!(offset_x, offset_y);
    //     let pt_b: OPoint<f32, nalgebra::Const<2>> = point!(-offset_x, -offset_y);
    //     let collider = ColliderBuilder::new(SharedShape::capsule(
    //         pt_a,
    //         pt_b,
    //         hinge.capsules[capsule_index].radius / 2.0,
    //     ))
    //     .collision_groups(InteractionGroups::new(group.into(), 0b0001.into()))
    //     .build();
    //     hinge.capsules[capsule_index].body_handle = rigid_body_set.insert(body);
    //     hinge.capsules[capsule_index].collider_handle = collider_set.insert_with_parent(
    //         collider,
    //         hinge.capsules[capsule_index].body_handle,
    //         rigid_body_set,
    //     );
    // }

    pub fn init_physics(&mut self, robot: &Robot) -> &RobotPhysicsMap {
        self.robot = robot.clone();

        // Clear existing physics objects
        self.rigid_body_set = RigidBodySet::new();
        self.collider_set = ColliderSet::new();
        self.impulse_joint_set = ImpulseJointSet::new();
        self.multibody_joint_set = MultibodyJointSet::new();
        self.robot_state.clear();

        // Create the capsules and populate the robot state
        for capsule in self.robot.get_capsules() {
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![capsule.center().x, capsule.center().y])
                .build();
            let (offset_x, offset_y) = capsule.offset_points();
            let pt_a = point![offset_x, offset_y];
            let pt_b = point![-offset_x, -offset_y];
            let collider =
                ColliderBuilder::new(SharedShape::capsule(pt_a, pt_b, capsule.radius / 2.0))
                    .collision_groups(InteractionGroups::new(0b0010.into(), 0b0001.into()))
                    .build();
            let body_handle: RigidBodyHandle = self.rigid_body_set.insert(rigid_body);
            self.collider_set
                .insert_with_parent(collider, body_handle, &mut self.rigid_body_set);
            self.robot_state.insert(capsule.id, body_handle);
        }

        // Create the joints
        for joint in self.robot.get_joints() {
            let (capsule1_id, capsule2_id) = (joint.capsule1_id, joint.capsule2_id);
            let body_handle1: RigidBodyHandle = self.robot_state.get(capsule1_id).unwrap();
            let body_handle2: RigidBodyHandle = self.robot_state.get(capsule2_id).unwrap();
            let joint_pos: Point2<f32> = joint.center();
            let offset1_opt: Option<nalgebra::OPoint<f32, nalgebra::Const<2>>> =
                self.get_pt_offset(capsule1_id, &joint_pos);
            // early exit if the offset is None
            if offset1_opt.is_none() {
                println!("Offset1 is None");
                continue;
            }
            let offset2_opt: Option<nalgebra::OPoint<f32, nalgebra::Const<2>>> =
                self.get_pt_offset(capsule2_id, &joint_pos);
            // early exit if the offset is None
            if offset2_opt.is_none() {
                println!("Offset2 is None");
                continue;
            }
            let offset1: nalgebra::OPoint<f32, nalgebra::Const<2>> = offset1_opt.unwrap();
            let offset2: nalgebra::OPoint<f32, nalgebra::Const<2>> = offset2_opt.unwrap();

            let revolute_joint: RevoluteJoint = RevoluteJointBuilder::new()
                .local_anchor1(offset1)
                .local_anchor2(offset2)
                .limits([joint.min, joint.max])
                .build();
            self.impulse_joint_set
                .insert(body_handle1, body_handle2, revolute_joint, true);
        }

        // Create the ground
        let ground_collider = ColliderBuilder::cuboid(GROUND_WIDTH, GROUND_HEIGHT)
            .translation(vector![GROUND_X, GROUND_Y])
            .restitution(GROUND_RESTITUTION)
            .collision_groups(InteractionGroups::new(0b0001.into(), 0b1111.into()))
            .build();
        self.collider_set.insert(ground_collider);

        &self.robot_state
    }

    fn update(&mut self) {
        println!("Updating robot, is_playing: {}", self.is_playing);
        if !self.is_playing {
            println!("Not playing");
            return;
        }
        self.physics_pipeline.step(
            &vector![0.0, 9.81 * 10.0],
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
            if let Some(body_handle) = self.robot_state.get(capsule.id) {
                if let Some(body) = self.rigid_body_set.get(body_handle) {
                    let position = body.position().translation;
                    println!("Updating capsule position: {:?}", position);
                    // extremely important to add the initial rotation offset here
                    let rotation =
                        body.position().rotation.angle() + capsule.get_initial_rotation_offset();
                    let center = Pos2::new(position.x, position.y);
                    let half_length = capsule.half_length();
                    capsule.update_endpoints(center, half_length, rotation);
                }
            }
        }
    }

    pub fn toggle_playback(&mut self) {
        println!("Toggling playback, is_playing: {}", self.is_playing);
        self.is_playing = !self.is_playing;
        println!("Toggled playback, is_playing: {}", self.is_playing);
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
            println!("Drawing robot, is_playing: {}", self.is_playing);
            self.update();
            self.robot.draw(
                ui.painter(),
                &Vec::new(),
                &Vec::new(),
                &Vec::new(),
                &Vec::new(),
            );
        });
    }
}

pub struct RobotPhysicsMap {
    capsule_id_to_handle: HashMap<usize, RigidBodyHandle>,
}

impl RobotPhysicsMap {
    fn new() -> Self {
        RobotPhysicsMap {
            capsule_id_to_handle: HashMap::new(),
        }
    }

    fn clear(&mut self) {
        self.capsule_id_to_handle.clear();
    }

    fn insert(&mut self, capsule_id: usize, handle: RigidBodyHandle) {
        self.capsule_id_to_handle.insert(capsule_id, handle);
    }

    fn get(&self, capsule_id: usize) -> Option<RigidBodyHandle> {
        self.capsule_id_to_handle.get(&capsule_id).copied()
    }
}
