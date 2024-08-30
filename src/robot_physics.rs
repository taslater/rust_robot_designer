use crate::constants::{
    CAPSULE_FRICTION, CAPSULE_RESTITUTION, MOTOR_DAMPING, MOTOR_MAX_FORCE, PHYSICS_SCALE,
    TARGET_VELOCITY, CAPSULE_DENSITY,
};
use crate::model::robot::Robot;
use crate::physics_world::{to_physics_coords, to_rendering_coords, PhysicsWorld};
use egui::{pos2, Pos2};
use nalgebra::{point, vector};
use rapier2d::prelude::*;
use std::collections::HashMap;

pub(crate) struct CapsuleData {
    rigid_body_handle: RigidBodyHandle,
    initial_position: Isometry<f32>,
}

struct RobotPhysicsHandles {
    capsule_data: HashMap<usize, CapsuleData>,
    joint_map: HashMap<usize, ImpulseJointHandle>,
}

impl RobotPhysicsHandles {
    pub fn new() -> Self {
        RobotPhysicsHandles {
            capsule_data: HashMap::new(),
            joint_map: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.capsule_data.clear();
        self.joint_map.clear();
    }
}

#[derive(Clone)]
pub struct EvaluationData {
    pub motion_sum: f32,
    pub output_sum: f32,
}

pub struct RobotPhysics {
    handles: RobotPhysicsHandles,
    evaluation_data: EvaluationData
}

impl RobotPhysics {
    pub fn new() -> Self {
        RobotPhysics {
            handles: RobotPhysicsHandles::new(),
            evaluation_data: EvaluationData {
                motion_sum: 0.0,
                output_sum: 0.0,
            },
        }
    }

    pub fn get_evaluation_data(&self) -> EvaluationData {
        self.evaluation_data.clone()
    }

    pub fn get_capsule_handles(&self) -> HashMap<usize, RigidBodyHandle> {
        self
            .handles
            .capsule_data
            .iter()
            .map(|(id, data)| (*id, data.rigid_body_handle))
            .collect()
    }

    pub fn get_impulse_joint_handle(&self, joint_id: usize) -> ImpulseJointHandle {
        *self.handles
            .joint_map
            .get(&joint_id)
            .unwrap()
    }

    pub fn clear(&mut self) {
        self.handles.clear();
    }

    pub fn build_robot(&mut self, robot: &mut Robot, physics_world: &mut PhysicsWorld) {
        let mut capsule_data: HashMap<usize, CapsuleData> = HashMap::new();
        let mut joint_map: HashMap<usize, ImpulseJointHandle> = HashMap::new();

        // Create the capsules and populate the robot state
        for (capsule_id, capsule) in robot.get_capsules() {
            let physics_center = to_physics_coords(capsule.center());
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![physics_center.x, physics_center.y])
                .can_sleep(false)
                .build();
            let initial_position = rigid_body.position().clone();
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
            .density(CAPSULE_DENSITY)
            .build();
            let body_handle = physics_world.add_rigid_body(rigid_body);
            physics_world.add_collider_w_parent(collider, body_handle);
            capsule_data.insert(
                *capsule_id,
                CapsuleData {
                    rigid_body_handle: body_handle,
                    initial_position,
                },
            );
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
            .map(|(joint_id, joint)| {
                (
                    *joint_id,
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
            .map(|(capsule_id, capsule)| (*capsule_id, Some(capsule.center())))
            .collect();

        // Create the joints
        for ((_joint_id, joint), joint_props) in robot
            .get_joints_mut()
            .iter_mut()
            .zip(joint_positions.iter())
        {
            let (_, capsule1_id, capsule2_id, position, joint_min, joint_max) = joint_props;
            let body_handle1 = capsule_data.get(&capsule1_id).unwrap().rigid_body_handle;
            let body_handle2 = capsule_data.get(&capsule2_id).unwrap().rigid_body_handle;
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
                    // .motor_model(MotorModel::ForceBased)
                    .motor_max_force(MOTOR_MAX_FORCE)
                    .motor_velocity(TARGET_VELOCITY, MOTOR_DAMPING)
                    .limits([*joint_min, *joint_max])
                    .build();
                let impulse_joint_handle =
                    physics_world.add_impulse_joint(body_handle1, body_handle2, revolute_joint);
                joint_map.insert(
                    joint_props.0,
                    impulse_joint_handle,
                );
                joint.set_handle(impulse_joint_handle);
            }
        }
        self.handles = RobotPhysicsHandles {
            capsule_data,
            joint_map,
        };
    }

    pub fn reset_robot(&mut self, physics_world: &mut PhysicsWorld) {
        for (_capsule_id, capsule_data) in self.handles.capsule_data.iter() {
            if let Some(rigid_body) =
                physics_world.get_rigid_body_mut(capsule_data.rigid_body_handle)
            {                rigid_body.set_position(capsule_data.initial_position, true);
                rigid_body.set_linvel(vector![0.0, 0.0], true);
                rigid_body.set_angvel(0.0, true);
                rigid_body.reset_forces(true);
                
            }
        }
        self.evaluation_data.motion_sum = 0.0;
        self.evaluation_data.output_sum = 0.0;
    }

    pub fn update_evaluation_data_output(&mut self, output: f32) {
        self.evaluation_data.output_sum += output.abs();
    }

    pub fn update_robot_physics(&mut self, robot: &mut Robot, physics_world: &PhysicsWorld) {
        // Update the robot's capsule positions and rotations
        for (capsule_id, capsule_data) in self.handles.capsule_data.iter_mut() {
            if let Some(capsule) = robot.get_capsule_mut(*capsule_id) {
                if let Some(body) = physics_world.get_rigid_body(capsule_data.rigid_body_handle) {
                    let physics_position = body.position().translation;
                    let rotation =
                        body.position().rotation.angle() + capsule.get_initial_rotation_offset();
                    let rendering_position =
                        to_rendering_coords(pos2(physics_position.x, physics_position.y));
                    let rendering_half_length = capsule.half_length();
                    capsule.update_endpoints(rendering_position, rendering_half_length, rotation);
                    let linvel = body.linvel();
                    let angvel = body.angvel();
                    self.evaluation_data.motion_sum += linvel.norm() + angvel.abs();
                }
            }
        }

        // Update the robot's joint positions
        for (joint_id, impulse_joint_handle) in self.handles.joint_map.iter() {
            if let Some(impulse_joint) =
                physics_world.get_impulse_joint(*impulse_joint_handle)
            {
                if let Some(joint) = robot.get_joint_mut(*joint_id) {
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
