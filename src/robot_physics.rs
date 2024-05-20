use crate::constants::{
    CAPSULE_FRICTION, CAPSULE_RESTITUTION, MOTOR_DAMPING, MOTOR_MAX_FORCE, PHYSICS_SCALE,
    TARGET_VELOCITY,
};
use crate::model::robot::Robot;
use crate::physics_world::{to_physics_coords, to_rendering_coords, PhysicsWorld};
use egui::{pos2, Pos2};
use nalgebra::{point, vector};
use rapier2d::prelude::*;
use std::collections::HashMap;

struct RobotPhysicsHandles {
    pub capsule_handles: HashMap<usize, RigidBodyHandle>,
    pub joint_handles: HashMap<usize, ImpulseJointHandle>,
    pub initial_positions: HashMap<usize, Isometry<f32>>,
}

impl RobotPhysicsHandles {
    pub fn new() -> Self {
        RobotPhysicsHandles {
            capsule_handles: HashMap::new(),
            joint_handles: HashMap::new(),
            initial_positions: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.capsule_handles.clear();
        self.joint_handles.clear();
    }
}

pub struct RobotPhysics {
    handles: RobotPhysicsHandles,
}

impl RobotPhysics {
    pub fn new() -> Self {
        RobotPhysics {
            handles: RobotPhysicsHandles::new(),
        }
    }

    pub fn get_capsule_handles(&self) -> &HashMap<usize, RigidBodyHandle> {
        &self.handles.capsule_handles
    }

    // pub fn get_impulse_joint(&self, joint_id: usize) -> Option<&ImpulseJointHandle> {
    //     self.handles.joint_handles.get(&joint_id)
    // }

    pub fn get_impulse_joint_handle(&self, joint_id: usize) -> ImpulseJointHandle {
        self.handles.joint_handles.get(&joint_id).unwrap().clone()
    }

    pub fn clear(&mut self) {
        self.handles.clear();
    }

    pub fn build_robot(&mut self, robot: &mut Robot, physics_world: &mut PhysicsWorld) {
        let mut capsule_handles = HashMap::new();
        let mut joint_handles = HashMap::new();
        let mut initial_positions = HashMap::new();

        // Create the capsules and populate the robot state
        for capsule in robot.get_capsules() {
            let physics_center = to_physics_coords(capsule.center());
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![physics_center.x, physics_center.y])
                .can_sleep(false)
                .build();
            let initial_position = rigid_body.position().clone();
            initial_positions.insert(capsule.id, initial_position);
            let (physics_offset_x, physics_offset_y) = capsule.offset_points_physics();
            let pt_a: nalgebra::OPoint<f32, nalgebra::Const<2>> =
                point![physics_offset_x, physics_offset_y];
            let pt_b: nalgebra::OPoint<f32, nalgebra::Const<2>> =
                point![-physics_offset_x, -physics_offset_y];
            let collider = ColliderBuilder::new(SharedShape::capsule(
                pt_a,
                pt_b,
                capsule.radius * PHYSICS_SCALE / 2.0,
            ))
            .collision_groups(InteractionGroups::new(0b0010.into(), 0b0001.into()))
            .restitution(CAPSULE_RESTITUTION)
            .friction(CAPSULE_FRICTION)
            .build();
            let body_handle = physics_world.add_rigid_body(rigid_body);
            physics_world.add_collider_w_parent(collider, body_handle);
            capsule_handles.insert(capsule.id, body_handle);
        }

        // Collect the joint positions and capsule centers before the loop
        let joint_positions: Vec<(
            usize,
            usize,
            usize,
            nalgebra::OPoint<f32, nalgebra::Const<2>>,
            f32,
            f32,
        )> = robot
            .get_joints()
            .iter()
            .map(|joint| {
                (
                    joint.id,
                    joint.capsule1_id,
                    joint.capsule2_id,
                    joint.position(),
                    joint.min,
                    joint.max,
                )
            })
            .collect();

        let capsule_centers: Vec<(usize, Option<Pos2>)> = robot
            .get_capsules()
            .iter()
            .map(|capsule| (capsule.id, Some(capsule.center())))
            .collect();

        // Create the joints
        for (joint, joint_props) in robot
            .get_joints_mut()
            .iter_mut()
            .zip(joint_positions.iter())
        {
            let (_, capsule1_id, capsule2_id, position, joint_min, joint_max) = joint_props;
            let body_handle1 = capsule_handles.get(&capsule1_id).unwrap();
            let body_handle2 = capsule_handles.get(&capsule2_id).unwrap();
            let physics_joint_pos: Pos2 = to_physics_coords(pos2(position.x, position.y));

            let capsule1_center_option: Option<Pos2> = capsule_centers
                .iter()
                .find(|(id, _)| *id == *capsule1_id)
                .map(|(_, center)| center.clone())
                .unwrap_or(None);

            if capsule1_center_option.is_none() {
                continue;
            }
            let capsule1_center: Pos2 = capsule1_center_option.unwrap();
            let physics_capsule1_center: Pos2 = to_physics_coords(capsule1_center);
            let offset_x1: f32 = physics_joint_pos.x - physics_capsule1_center.x;
            let offset_y1: f32 = physics_joint_pos.y - physics_capsule1_center.y;
            let offset1: nalgebra::OPoint<f32, nalgebra::Const<2>> = point![offset_x1, offset_y1];

            // ... (repeat the same process for capsule2)
            let capsule2_center_option: Option<Pos2> = capsule_centers
                .iter()
                .find(|(id, _)| *id == *capsule2_id)
                .map(|(_, center)| center.clone())
                .unwrap_or(None);

            if capsule2_center_option.is_none() {
                continue;
            }
            let capsule2_center: Pos2 = capsule2_center_option.unwrap();
            let physics_capsule2_center: Pos2 = to_physics_coords(capsule2_center);
            let offset_x2: f32 = physics_joint_pos.x - physics_capsule2_center.x;
            let offset_y2: f32 = physics_joint_pos.y - physics_capsule2_center.y;
            let offset2: nalgebra::OPoint<f32, nalgebra::Const<2>> = point![offset_x2, offset_y2];

            if body_handle1 != body_handle2 {
                let revolute_joint: RevoluteJoint = RevoluteJointBuilder::new()
                    .local_anchor1(offset1)
                    .local_anchor2(offset2)
                    .motor_model(MotorModel::AccelerationBased)
                    .motor_max_force(MOTOR_MAX_FORCE)
                    .motor_velocity(TARGET_VELOCITY, MOTOR_DAMPING)
                    .limits([*joint_min, *joint_max])
                    .build();
                let impulse_joint_handle =
                    physics_world.add_impulse_joint(*body_handle1, *body_handle2, revolute_joint);
                joint_handles.insert(joint_props.0, impulse_joint_handle);
                joint.set_handle(impulse_joint_handle);
            }
        }
        self.handles = RobotPhysicsHandles {
            capsule_handles,
            joint_handles,
            initial_positions,
        }
    }

    pub fn reset_robot(&mut self, physics_world: &mut PhysicsWorld) {
        for (capsule_id, body_handle) in &self.handles.capsule_handles {
            if let Some(rigid_body) = physics_world.get_rigid_body_mut(*body_handle) {
                if let Some(initial_position) = self.handles.initial_positions.get(capsule_id) {
                    rigid_body.set_position(*initial_position, true);
                    rigid_body.set_linvel(vector![0.0, 0.0], true);
                    rigid_body.set_angvel(0.0, true);
                    rigid_body.reset_forces(true);
                }
            }
        }
    }

    pub fn update_robot_physics(
        &self,
        robot: &mut Robot,
        physics_world: &PhysicsWorld,
    ) {
        // Update the robot's capsule positions and rotations
        for capsule in robot.get_capsules_mut() {
            if let Some(body_handle) = self.handles.capsule_handles.get(&capsule.id) {
                if let Some(body) = physics_world.get_rigid_body(*body_handle) {
                    let physics_position = body.position().translation;
                    let rotation =
                        body.position().rotation.angle() + capsule.get_initial_rotation_offset();
                    let rendering_position =
                        to_rendering_coords(pos2(physics_position.x, physics_position.y));
                    let rendering_half_length = capsule.half_length();
                    capsule.update_endpoints(rendering_position, rendering_half_length, rotation);
                }
            }
        }

        // Update the robot's joint positions
        for joint in robot.get_joints_mut() {
            if let Some(impulse_joint_handle) = self.handles.joint_handles.get(&joint.id) {
                if let Some(impulse_joint) = physics_world.get_impulse_joint(*impulse_joint_handle)
                {
                    let body1_pos = physics_world
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
