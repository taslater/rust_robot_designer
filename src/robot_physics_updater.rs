use crate::model::robot::Robot;
use crate::physics_world::{to_rendering_coords, PhysicsWorld};
use crate::robot_physics_builder::RobotPhysicsHandles;
use egui::pos2;

pub struct RobotPhysicsUpdater;

impl RobotPhysicsUpdater {
    pub fn update_robot_physics(robot: &mut Robot, physics_world: &PhysicsWorld, robot_handles: &RobotPhysicsHandles) {
        // Update the robot's capsule positions and rotations
        for capsule in robot.get_capsules_mut() {
            if let Some(body_handle) = robot_handles.capsule_handles.get(&capsule.id) {
                if let Some(body) = physics_world.get_rigid_body(*body_handle) {
                    let physics_position = body.position().translation;
                    let rotation = body.position().rotation.angle() + capsule.get_initial_rotation_offset();
                    let rendering_position = to_rendering_coords(pos2(physics_position.x, physics_position.y));
                    let rendering_half_length = capsule.half_length();
                    capsule.update_endpoints(rendering_position, rendering_half_length, rotation);
                }
            }
        }

        // Update the robot's joint positions
        for joint in robot.get_joints_mut() {
            if let Some(impulse_joint_handle) = robot_handles.joint_handles.get(&joint.id) {
                if let Some(impulse_joint) = physics_world.get_impulse_joint(*impulse_joint_handle) {
                    let body1_pos = physics_world.get_rigid_body(impulse_joint.body1).unwrap().position();
                    let local_frame1 = impulse_joint.data.local_frame1;
                    let local_combined1 = body1_pos * local_frame1;
                    let rendering_position1 = to_rendering_coords(pos2(local_combined1.translation.vector.x, local_combined1.translation.vector.y));
                    joint.set_position(rendering_position1.x, rendering_position1.y);
                }
            }
        }
    }
}