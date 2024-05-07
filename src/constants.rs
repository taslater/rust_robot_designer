pub const PHYSICS_SCALE: f32 = 1e-1;
pub const GRAVITY: f32 = 10.0 * 9.81 * PHYSICS_SCALE;
pub const MOTOR_DAMPING: f32 = 1.0 * PHYSICS_SCALE; // Damping factor for the motor
pub const MOTOR_MAX_FORCE: f32 = 1e6 * PHYSICS_SCALE * PHYSICS_SCALE; // Maximum force applied by the motor
pub const TARGET_VELOCITY: f32 = 10_000.0 * PHYSICS_SCALE; // 1e6; // Velocity of the motor

pub const GROUND_RESTITUTION: f32 = 0.0;
pub const CAPSULE_RESTITUTION: f32 = 0.0;
pub const CAPSULE_FRICTION: f32 = 0.0;
