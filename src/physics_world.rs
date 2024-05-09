use crate::constants::{
    GRAVITY, GROUND_FRICTION, GROUND_RESTITUTION, 
    GROUND_WIDTH, GROUND_HEIGHT, GROUND_X, GROUND_Y,
    MOTOR_DAMPING, PHYSICS_SCALE, TARGET_VELOCITY,
};
use egui::{pos2, Pos2};
use rapier2d::prelude::*;

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

    pub fn get_impulse_joint(&self, handle: ImpulseJointHandle) -> Option<&ImpulseJoint> {
        self.impulse_joint_set.get(handle)
    }

    // pub fn update_impulse_joints(&mut self, motor_directions: &[f32]) {
    //     for (index, (_joint_handle, impulse_joint)) in self.impulse_joint_set.iter_mut().enumerate() {
    //         if let Some(motor_direction) = motor_directions.get(index) {
    //             impulse_joint.data.set_motor_velocity(
    //                 JointAxis::AngX,
    //                 TARGET_VELOCITY * motor_direction,
    //                 MOTOR_DAMPING,
    //             );
    //         }
    //     }
    // }

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
}
