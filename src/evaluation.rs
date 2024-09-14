use crate::physics_world::PhysicsWorld;
use crate::robot_physics::RobotPhysics;
use crate::shared_config::SharedConfig;

pub trait Evaluator {
    fn evaluate(&self, world: &PhysicsWorld, robot: &RobotPhysics, config: &SharedConfig) -> f32;
}

pub struct RolloverEvaluator;

impl Evaluator for RolloverEvaluator {
    fn evaluate(&self, _world: &PhysicsWorld, robot: &RobotPhysics, _config: &SharedConfig) -> f32 {
        let result = robot.get_evaluation_data().max_relative_rotation;
        println!("DEBUG: RolloverEvaluator - End: {}", result);
        result
    }
}

pub struct MotionEvaluator;
impl Evaluator for MotionEvaluator {
    fn evaluate(&self, _world: &PhysicsWorld, robot: &RobotPhysics, config: &SharedConfig) -> f32 {
        // println!("DEBUG: MotionEvaluator - Start");
        let result = config.motion_weight * robot.get_evaluation_data().motion_sum
            / config.steps_per_generation as f32;
        // println!("DEBUG: MotionEvaluator - End: {}", result);
        result
    }
}

pub struct OutputEvaluator;
impl Evaluator for OutputEvaluator {
    fn evaluate(&self, _world: &PhysicsWorld, robot: &RobotPhysics, config: &SharedConfig) -> f32 {
        // println!("DEBUG: OutputEvaluator - Start");
        let result = config.output_weight * robot.get_evaluation_data().output_sum
            / config.steps_per_generation as f32;
        println!("DEBUG: OutputEvaluator - End: {}", result);
        result
    }
}

pub struct DistanceEvaluator;
impl Evaluator for DistanceEvaluator {
    fn evaluate(&self, world: &PhysicsWorld, robot: &RobotPhysics, config: &SharedConfig) -> f32 {
        // println!("DEBUG: DistanceEvaluator - Start");
        let distance = robot
            .get_capsule_handles()
            .iter()
            .map(|(_capsule_id, rigid_body_handle)| {
                world
                    .get_rigid_body(*rigid_body_handle)
                    .map(|rb| rb.position().translation.x)
                    .unwrap_or(0.0)
            })
            .sum::<f32>()
            / robot.get_capsule_handles().len() as f32
            / config.steps_per_generation as f32;

        let result = config.distance_weight * distance;
        // println!("DEBUG: DistanceEvaluator - End: {}", result);
        result
    }
}

pub struct CompositeEvaluator {
    evaluators: Vec<Box<dyn Evaluator + Send + Sync>>,
}

impl CompositeEvaluator {
    pub fn new() -> Self {
        CompositeEvaluator {
            evaluators: vec![
                Box::new(MotionEvaluator),
                Box::new(OutputEvaluator),
                Box::new(DistanceEvaluator),
                Box::new(RolloverEvaluator),
            ],
        }
    }

    pub fn evaluate(
        &self,
        world: &PhysicsWorld,
        robot: &RobotPhysics,
        config: &SharedConfig,
    ) -> f32 {
        // println!("DEBUG: CompositeEvaluator - Start");
        let result = self
            .evaluators
            .iter()
            .map(|evaluator| evaluator.evaluate(world, robot, config))
            .sum();
        // println!("DEBUG: CompositeEvaluator - End: {}", result);
        result
    }
}
