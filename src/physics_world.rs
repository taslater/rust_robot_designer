use crate::constants::{
    GRAVITY, GROUND_FRICTION, GROUND_HEIGHT, GROUND_RESTITUTION, GROUND_WIDTH, GROUND_X, GROUND_Y,
    MOTOR_DAMPING, PHYSICS_SCALE, TARGET_VELOCITY,
};
use egui::{pos2, Pos2};
use rapier2d::prelude::*;
use std::collections::HashMap;

pub fn to_physics_coords(rendering_coords: Pos2) -> Pos2 {
    pos2(
        rendering_coords.x * PHYSICS_SCALE,
        rendering_coords.y * PHYSICS_SCALE,
    )
}

pub fn to_rendering_coords(physics_coords: Pos2) -> Pos2 {
    pos2(
        physics_coords.x / PHYSICS_SCALE,
        physics_coords.y / PHYSICS_SCALE,
    )
}

pub fn flat_ground_collider() -> Collider {
    ColliderBuilder::cuboid(GROUND_WIDTH * PHYSICS_SCALE, GROUND_HEIGHT * PHYSICS_SCALE)
        .translation(vector![GROUND_X * PHYSICS_SCALE, GROUND_Y * PHYSICS_SCALE])
        .restitution(GROUND_RESTITUTION)
        .friction(GROUND_FRICTION)
        .collision_groups(InteractionGroups::new(0b0001.into(), 0b1111.into()))
        .build()
}

pub struct RigidBodyObservation {
    pub y: f32,
    pub sin: f32,
    pub cos: f32,
    pub vel_x: f32,
    pub vel_y: f32,
    pub angvel: f32,
    pub d_vel_x: f32,
    pub d_vel_y: f32,
    pub d_angvel: f32,
}

pub struct RigidBodyVelocityObservation {
    pub vel_x: f32,
    pub vel_y: f32,
    pub angvel: f32,
}

pub struct PhysicsWorld {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub event_handler: (),
    pub prev_rigid_body_velocities: HashMap<RigidBodyHandle, RigidBodyVelocityObservation>,
}

// debug trait
impl std::fmt::Debug for PhysicsWorld {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PhysicsWorld")
            // .field("rigid_body_set", &self.rigid_body_set)
            // .field("collider_set", &self.collider_set)
            // .field("impulse_joint_set", &self.impulse_joint_set)
            // .field("multibody_joint_set", &self.multibody_joint_set)
            // .field("integration_parameters", &self.integration_parameters)
            // .field("physics_pipeline", &self.physics_pipeline)
            // .field("island_manager", &self.island_manager)
            // .field("broad_phase", &self.broad_phase)
            // .field("narrow_phase", &self.narrow_phase)
            // .field("ccd_solver", &self.ccd_solver)
            // .field("query_pipeline", &self.query_pipeline)
            // .field("event_handler", &self.event_handler)
            // .field("prev_rigid_body_velocities", &self.prev_rigid_body_velocities)
            // none of the preceding fields implement Debug and it would be too verbose to implement them all
            // so we'll just say that the struct is named PhysicsWorld
            .field("PhysicsWorld", &"PhysicsWorld")
            .finish()
    }
}

impl PhysicsWorld {
    pub fn new() -> Self {
        let integration_parameters = IntegrationParameters {
            dt: 1.0 / 60.0,
            ..Default::default()
        };
        // integration_parameters.switch_to_small_steps_pgs_solver();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = BroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();
        let event_handler = ();
        let prev_rigid_body_velocities = HashMap::new();

        PhysicsWorld {
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
            prev_rigid_body_velocities,
        }
    }

    pub(crate) fn clear(&mut self) {
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
        self.prev_rigid_body_velocities.clear();
    }

    pub fn add_rigid_body(&mut self, rigid_body: RigidBody) -> RigidBodyHandle {
        self.rigid_body_set.insert(rigid_body)
    }

    pub fn add_collider_w_parent(&mut self, collider: Collider, body_handle: RigidBodyHandle) {
        self.collider_set
            .insert_with_parent(collider, body_handle, &mut self.rigid_body_set);
    }

    pub fn add_collider(&mut self, collider: Collider) -> ColliderHandle {
        self.collider_set.insert(collider)
    }

    pub fn add_impulse_joint(
        &mut self,
        body_handle1: RigidBodyHandle,
        body_handle2: RigidBodyHandle,
        joint: RevoluteJoint,
    ) -> ImpulseJointHandle {
        self.impulse_joint_set
            .insert(body_handle1, body_handle2, joint, true)
    }

    pub fn get_rigid_body(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.rigid_body_set.get(handle)
    }

    pub fn get_rigid_body_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.rigid_body_set.get_mut(handle)
    }

    pub fn get_impulse_joint(&self, handle: ImpulseJointHandle) -> Option<&ImpulseJoint> {
        self.impulse_joint_set.get(handle)
    }

    pub fn set_impulse_joint_motor_direction(
        &mut self,
        impulse_joint_handle: ImpulseJointHandle,
        motor_direction: f32,
    ) {
        if let Some(impulse_joint) = self.impulse_joint_set.get_mut(impulse_joint_handle) {
            impulse_joint.data.set_motor_velocity(
                JointAxis::AngX,
                TARGET_VELOCITY * motor_direction,
                MOTOR_DAMPING,
            );
        }
    }

    pub fn step(&mut self) {
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
    }

    pub fn get_all_rigid_body_observations(
        &mut self,
    ) -> HashMap<RigidBodyHandle, RigidBodyObservation> {
        self.rigid_body_set
            .iter()
            .map(|(rigid_body_handle, rigid_body)| {
                let mut new_rigid_body_velocity = RigidBodyVelocityObservation {
                    vel_x: 0.0,
                    vel_y: 0.0,
                    angvel: 0.0,
                };
                let prev_rigid_body_velocity = self
                    .prev_rigid_body_velocities
                    .get(&rigid_body_handle)
                    .unwrap_or(&RigidBodyVelocityObservation {
                        vel_x: 0.0,
                        vel_y: 0.0,
                        angvel: 0.0,
                    });
                let pos: &nalgebra::Isometry<f32, nalgebra::Unit<nalgebra::Complex<f32>>, 2> =
                    rigid_body.position();
                let translation = pos.translation.vector;
                let trans_y: f32 = translation[1] / 10.0;
                let rotation: nalgebra::Unit<nalgebra::Complex<f32>> = pos.rotation;
                let sin: f32 = rotation.im * 10.0;
                let cos: f32 = rotation.re * 10.0;
                let linvel: &nalgebra::Matrix<
                    f32,
                    nalgebra::Const<2>,
                    nalgebra::Const<1>,
                    nalgebra::ArrayStorage<f32, 2, 1>,
                > = rigid_body.linvel();
                let vel_x: f32 = linvel[(0, 0)] * 10.0;
                let vel_y: f32 = linvel[(1, 0)] * 10.0;
                let angvel: f32 = rigid_body.angvel() * 100.0;
                // set previous velocities
                new_rigid_body_velocity.vel_x = vel_x;
                new_rigid_body_velocity.vel_y = vel_y;
                new_rigid_body_velocity.angvel = angvel;
                // calculate the change in velocities
                let d_vel_x: f32 = 100.0 * (vel_x - prev_rigid_body_velocity.vel_x);
                let d_vel_y: f32 = 100.0 * (vel_y - prev_rigid_body_velocity.vel_y);
                let d_angvel: f32 = 100.0 * (angvel - prev_rigid_body_velocity.angvel);
                // release the lock on the previous velocities
                self.prev_rigid_body_velocities
                    .insert(rigid_body_handle, new_rigid_body_velocity);
                (
                    rigid_body_handle,
                    RigidBodyObservation {
                        y: trans_y,
                        sin,
                        cos,
                        vel_x,
                        vel_y,
                        angvel,
                        d_vel_x,
                        d_vel_y,
                        d_angvel,
                    },
                )
            })
            .collect()
    }

    pub fn get_all_rigid_body_evaluations(&self) -> HashMap<RigidBodyHandle, f32> {
        self.rigid_body_set
            .iter()
            .map(|(rigid_body_handle, rigid_body)| {
                (rigid_body_handle, rigid_body.position().translation.x)
            })
            .collect()
    }
}
