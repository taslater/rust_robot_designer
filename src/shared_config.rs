use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub struct SharedConfig {
    pub output_weight: f32,
    pub motion_weight: f32,
    pub distance_weight: f32,
    pub steps_per_generation: usize,
}

impl Default for SharedConfig {
    fn default() -> Self {
        SharedConfig {
            output_weight: -0.2,
            motion_weight: 0.5,
            distance_weight: 1.0,
            steps_per_generation: 1000,
        }
    }
}

pub type SharedConfigRef = Arc<Mutex<SharedConfig>>;

pub fn create_shared_config() -> SharedConfigRef {
    Arc::new(Mutex::new(SharedConfig::default()))
}