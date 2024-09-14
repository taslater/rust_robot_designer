use crate::brain::get_brain;
use crate::cma_es::CMAES;
use crate::evaluation::CompositeEvaluator;
use crate::model::robot::Robot;
use crate::physics_world::flat_ground_collider;
use crate::robot_sim_shared::{add_shared_ui, run_simulation_steps, Simulation};
use crate::shared_config::SharedConfigRef;
use egui::Ui;
use nalgebra::DVector;
use std::collections::HashSet;
use std::sync::Arc;

pub struct RobotTrainer {
    simulations: Vec<Simulation>,
    optimizer: Option<CMAES>,
    population: Vec<DVector<f64>>,
    is_playing: bool,
    generation_step: usize,
    use_default_population_size: bool,
    custom_population_size: usize,
    sigma_initial: f64,
    shared_config: SharedConfigRef,
    evaluator: CompositeEvaluator,
}

impl RobotTrainer {
    pub fn new(shared_config: SharedConfigRef) -> Self {
        RobotTrainer {
            simulations: Vec::new(),
            optimizer: None,
            population: Vec::new(),
            is_playing: false,
            generation_step: 0,
            use_default_population_size: true,
            custom_population_size: 64,
            sigma_initial: 2.0,
            shared_config,
            evaluator: CompositeEvaluator::new(),
        }
    }

    pub fn init_physics(&mut self, robot: &Robot) {
        println!("DEBUG: Initializing physics");
        self.simulations.clear();

        let test_robot = robot.clone();
        let mut test_simulation = Simulation::new(
            test_robot,
            get_brain(0, vec![], 0),
            self.shared_config.clone(),
        );
        let n_inputs = test_simulation.get_observations().len();
        let n_outputs = robot.joints_count();

        let test_brain = get_brain(n_inputs, vec![1 * n_inputs, 1 * n_inputs], n_outputs);
        let test_flat: Vec<f64> = test_brain.get_flat_weights_and_biases();

        let mean = DVector::from_element(test_flat.len(), 0.0);
        let population_size = if self.use_default_population_size {
            None
        } else {
            Some(self.custom_population_size)
        };

        let mut optimizer = CMAES::new(mean, self.sigma_initial, population_size);
        self.population = optimizer.ask();

        for brain_params in &self.population {
            let mut brain = test_brain.clone();
            brain.set_weights_and_biases(brain_params.iter().map(|&v| v as f64).collect());
            let mut simulation = Simulation::new(
                robot.clone(),
                brain,
                // self.shared_config.lock().unwrap().clone(),
                Arc::clone(&self.shared_config),
            );
            simulation
                .physics_world
                .add_collider(flat_ground_collider());
            self.simulations.push(simulation);
        }

        self.optimizer = Some(optimizer);
        println!("DEBUG: Physics initialization complete");
        println!("DEBUG: Population size: {}", self.population.len());
        println!("DEBUG: Number of simulations: {}", self.simulations.len());
    }

    pub fn evaluate_and_update(&mut self) {
        let steps_per_generation = self.shared_config.lock().unwrap().steps_per_generation;
        println!(
            "DEBUG: Starting evaluation and update, steps per generation: {}",
            steps_per_generation
        );
        let fitnesses: Vec<f64> = self
            .simulations
            .iter_mut()
            .map(|sim| {
                run_simulation_steps(sim, steps_per_generation, &self.evaluator) as f64
            })
            .collect();
        println!("DEBUG: All evaluations complete");

        let optimizer = self.optimizer.as_mut().unwrap();
        optimizer.tell(&self.population, &fitnesses);

        self.population = optimizer.ask();

        for (simulation, brain_params) in self.simulations.iter_mut().zip(&self.population) {
            simulation
                .brain
                .set_weights_and_biases(brain_params.iter().map(|&v| v as f64).collect());
            simulation
                .robot_physics
                .reset_robot(&mut simulation.physics_world);
        }

        println!("DEBUG: Evaluation and update complete");
    }

    fn update(&mut self) {
        if !self.is_playing {
            return;
        }

        for simulation in &mut self.simulations {
            simulation.step();
        }

        self.generation_step += 1;

        if self.generation_step >= self.shared_config.lock().unwrap().steps_per_generation {
            println!("DEBUG: Generation complete, evaluating and updating");
            self.generation_step = 0;
            self.evaluate_and_update();
        }
    }

    pub fn toggle_playback(&mut self, robot: &Robot) {
        println!("DEBUG: Toggling playback");
        self.is_playing = !self.is_playing;
        if self.is_playing {
            println!("DEBUG: Playback started, initializing physics");
            self.generation_step = 0;
            self.init_physics(robot);
        } else {
            println!("DEBUG: Playback paused");
        }
    }

    pub fn reset(&mut self, robot: &Robot) {
        println!("DEBUG: Resetting simulation");
        self.init_physics(robot);
        self.is_playing = false;
        println!("DEBUG: Reset complete");
    }

    pub fn ui(&mut self, ui: &mut Ui, robot: &Robot) {
        ui.horizontal(|ui| {
            if ui
                .button(if self.is_playing { "Pause" } else { "Play" })
                .clicked()
            {
                println!("DEBUG: Play/Pause button clicked");
                self.toggle_playback(robot);
            }
            if ui.button("Reset").clicked() {
                println!("DEBUG: Reset button clicked");
                self.reset(robot);
            }
        });

        egui::Frame::canvas(ui.style()).show(ui, |ui| {
            self.update();
            for simulation in &self.simulations {
                simulation.robot.draw(
                    ui.painter(),
                    &Vec::new(),
                    &Vec::new(),
                    &HashSet::new(),
                    &Vec::new(),
                );
            }
            ui.ctx().request_repaint();
        });

        ui.checkbox(
            &mut self.use_default_population_size,
            "Use default population size",
        );
        ui.add(
            egui::Slider::new(&mut self.custom_population_size, 1..=1000).text("Population size"),
        );
        // initial sigma
        ui.add(egui::Slider::new(&mut self.sigma_initial, 0.0..=4.0).text("Initial sigma"));

        {
            let mut shared_config = self.shared_config.lock().unwrap();
            add_shared_ui(ui, &mut shared_config);
        }
    }
}
