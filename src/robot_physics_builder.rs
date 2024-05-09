// robot_physics_builder.rs
use crate::constants::{
    CAPSULE_FRICTION, CAPSULE_RESTITUTION, MOTOR_DAMPING, MOTOR_MAX_FORCE, PHYSICS_SCALE,
    TARGET_VELOCITY,
};
use crate::model::robot::Robot;
use crate::physics_world::{to_physics_coords, PhysicsWorld};
use egui::{pos2, Pos2};
use nalgebra::point;
use rapier2d::prelude::*;
use std::collections::HashMap;

pub struct RobotPhysicsBuilder;

impl RobotPhysicsBuilder {
    pub fn build_robot(robot: &mut Robot, physics_world: &mut PhysicsWorld) -> RobotPhysicsHandles {
        let mut capsule_handles = HashMap::new();
        let mut joint_handles = HashMap::new();

        // Create the capsules and populate the robot state
        for capsule in robot.get_capsules() {
            let physics_center = to_physics_coords(capsule.center());
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![physics_center.x, physics_center.y])
                .can_sleep(false)
                .build();
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

        RobotPhysicsHandles {
            capsule_handles,
            joint_handles,
        }
    }
}

pub struct RobotPhysicsHandles {
    pub capsule_handles: HashMap<usize, RigidBodyHandle>,
    pub joint_handles: HashMap<usize, ImpulseJointHandle>,
}

impl RobotPhysicsHandles {
    pub fn new() -> Self {
        RobotPhysicsHandles {
            capsule_handles: HashMap::new(),
            joint_handles: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.capsule_handles.clear();
        self.joint_handles.clear();
    }
}