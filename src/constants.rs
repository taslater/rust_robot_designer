pub const PHYSICS_SCALE: f32 = 1e-1;
pub const GRAVITY: f32 = 20.0 * 9.81 * PHYSICS_SCALE;
pub const MOTOR_DAMPING: f32 = 2.0e0 * PHYSICS_SCALE; // Damping factor for the motor
pub const MOTOR_MAX_FORCE: f32 = 1.0e7 * PHYSICS_SCALE * PHYSICS_SCALE; // Maximum force applied by the motor
pub const TARGET_VELOCITY: f32 =  10_000.0 * PHYSICS_SCALE; // 1e6; // Velocity of the motor

pub const GROUND_RESTITUTION: f32 = 0.0;
pub const CAPSULE_RESTITUTION: f32 = 0.0;
pub const GROUND_FRICTION: f32 = 0.7;
pub const CAPSULE_FRICTION: f32 = 0.7;
pub const CAPSULE_DENSITY: f32 = 0.3;


pub const GROUND_WIDTH: f32 = 1e6;
pub const GROUND_HEIGHT: f32 = 100.0;
pub const GROUND_X: f32 = 0.0;
pub const GROUND_Y: f32 = 400.0 + GROUND_HEIGHT / 2.0;


// pub const POPULATION_SIZE: usize = 150;
pub const STEPS_PER_GENERATION: usize = 3000;
pub const BEST_RATIO: f32 = 0.1;
pub const MUTATION_RATE: f64 = 0.05;
pub const MUTATION_AMOUNT: f64 = 0.3;