use crate::model::robot::Robot;
use rapier2d::prelude::*;

pub(crate) struct RobotSimulator {
    capsule_data: Vec<CapsuleData>,
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

#[derive(Debug)]
struct CapsuleData {
    body_handle: RigidBodyHandle,
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
            capsule_data: Vec::new(),
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

    pub fn init_physics(&mut self, robot: &Robot) {
        println!("Initializing physics");
        let rigid_body_handles: Vec<_> = self
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

        // Iterate over all the joints and remove them
        let joint_handles: Vec<_> = self
            .impulse_joint_set
            .iter()
            .map(|(handle, _)| handle)
            .collect();
        for joint_handle in joint_handles {
            self.impulse_joint_set.remove(joint_handle, true);
        }

        // Create the capsules
        for capsule in robot.get_capsules() {
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![capsule.get_center().x, capsule.get_center().y])
                .build();
            let collider = ColliderBuilder::capsule_y(
                capsule.point1.y - capsule.point2.y,
                capsule.radius,
            )
            .build();
            let body_handle = self.rigid_body_set.insert(rigid_body);
            let _collider_handle = self.collider_set.insert_with_parent(
                collider,
                body_handle,
                &mut self.rigid_body_set,
            );
            self.capsule_data.push(CapsuleData {
                body_handle,
            });
        }

        println!("capsule_data: {:?}", self.capsule_data);

        // Create the joints
        for joint in robot.get_joints() {
            // let (capsule1, capsule2) = (
            //     robot.get_capsule(joint.capsule1_id).unwrap(),
            //     robot.get_capsule(joint.capsule2_id).unwrap(),
            // );
            let (body1, body2) = (
                self.capsule_data[joint.capsule1_id].body_handle,
                self.capsule_data[joint.capsule2_id].body_handle,
            );
            let revolute_joint = RevoluteJointBuilder::new()
                .local_anchor1(point![0.0, 0.0])
                .local_anchor2(point![0.0, 0.0])
                .limits([joint.min, joint.max])
                .build();
            let _joint_handle = self.impulse_joint_set.insert(
                body1,
                body2,
                revolute_joint,
                true,
            );
        }
    }

    fn update(&mut self, robot: &mut Robot) {
        println!("Updating robot, is_playing: {}", self.is_playing);
        if !self.is_playing {
            println!("Not playing");
            return;
        }
        self.physics_pipeline.step(
            &vector![0.0, -9.81],
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
        println!("Physics step done");

// Update the robot's capsule positions based on the simulation
for (capsule, capsule_data) in robot.get_capsules_mut().iter_mut().zip(&self.capsule_data) {
    println!("Updating capsule: {:?}", capsule_data.body_handle);
    if let Some(body) = self.rigid_body_set.get(capsule_data.body_handle) {
        println!("Found capsule: {:?}", capsule_data.body_handle);
        let position = body.position().translation;
        let rotation = body.position().rotation.angle() + capsule.rotation_offset;
        let half_length = capsule.length / 2.0;
        
        let center_x = position.x;
        let center_y = position.y;
        
        let dx = half_length * rotation.sin();
        let dy = half_length * rotation.cos();
        
        capsule.point1.x = center_x - dx;
        capsule.point1.y = center_y - dy;
        capsule.point2.x = center_x + dx;
        capsule.point2.y = center_y + dy;
    } else {
        println!("Could not find capsule: {:?}", capsule_data.body_handle);
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

    pub fn ui(&mut self, ui: &mut egui::Ui, robot: &mut Robot) {
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
            self.update(robot);
            robot.draw(
                ui.painter(),
                &Vec::new(),
                &Vec::new(),
                &Vec::new(),
                &Vec::new(),
            );
        });
    }
}