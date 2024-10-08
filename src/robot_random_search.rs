use crate::brain::get_brain;
// use crate::constants::STEPS_PER_GENERATION;
use crate::evaluation::CompositeEvaluator;
use crate::model::robot::Robot;
use crate::physics_world::flat_ground_collider;
use crate::robot_sim_shared::{
    add_shared_ui, get_robot_io_size, run_simulation_steps, SearchResult, Simulation,
};
use crate::shared_config::SharedConfigRef;
use egui::{Frame, Ui};
use std::collections::HashSet;

pub struct RobotRandomSearcher {
    robot: Option<Robot>,
    best_result: Option<SearchResult>,
    current_simulation: Option<Simulation>,
    show_best: bool,
    n_inputs: Option<usize>,
    n_outputs: Option<usize>,
    shared_config: SharedConfigRef,
    evaluator: CompositeEvaluator,
}

impl RobotRandomSearcher {
    pub fn new(shared_config: SharedConfigRef) -> Self {
        RobotRandomSearcher {
            robot: None,
            best_result: None,
            current_simulation: None,
            show_best: false,
            n_inputs: None,
            n_outputs: None,
            shared_config,
            evaluator: CompositeEvaluator::new(),
        }
    }

    pub fn init(&mut self, robot: &Robot) {
        let (n_inputs, n_outputs) = get_robot_io_size(&robot);
        self.n_inputs = Some(n_inputs);
        self.n_outputs = Some(n_outputs);
        self.robot = Some(robot.clone());
    }

    pub fn generate_and_evaluate(&mut self) {
        if self.n_inputs.is_none() || self.n_outputs.is_none() || self.robot.is_none() {
            // print what's missing
            if self.n_inputs.is_none() {
                println!("n_inputs is None");
            }
            if self.n_outputs.is_none() {
                println!("n_outputs is None");
            }
            if self.robot.is_none() {
                println!("robot is None");
            }
            return;
        }
        let n_inputs = self.n_inputs.unwrap();
        let n_outputs = self.n_outputs.unwrap();

        let new_brain = get_brain(n_inputs, vec![2 * n_inputs, 2 * n_inputs], n_outputs);

        let mut simulation = Simulation::new(
            self.robot.clone().unwrap(),
            new_brain,
            self.shared_config.clone(),
        );
        simulation
            .physics_world
            .add_collider(flat_ground_collider());

        // let evaluator = RobotEvaluator;
        let fitness = run_simulation_steps(
            &mut simulation,
            self.shared_config.lock().unwrap().steps_per_generation,
            &self.evaluator,
        );

        if self.best_result.is_none() || fitness < self.best_result.as_ref().unwrap().fitness {
            self.best_result = Some(SearchResult {
                brain: simulation.brain.clone(),
                fitness,
            });
        }

        self.current_simulation = Some(simulation);
    }

    pub fn show_best(&mut self) {
        if let Some(best_result) = &self.best_result {
            let mut simulation = Simulation::new(
                self.robot.clone().unwrap(),
                best_result.brain.clone(),
                self.shared_config.clone(),
            );
            simulation
                .physics_world
                .add_collider(flat_ground_collider());
            self.current_simulation = Some(simulation);
            self.show_best = true;
        }
    }

    pub fn update(&mut self) {
        if self.show_best {
            if let Some(simulation) = &mut self.current_simulation {
                simulation.step();
            }
        }
    }

    pub fn ui(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            if ui.button("Generate Brain").clicked() {
                self.generate_and_evaluate();
            }
            if ui.button("Show Best").clicked() {
                self.show_best();
            }
        });

        if let Some(best_result) = &self.best_result {
            ui.label(format!("Best Fitness: {:.4}", best_result.fitness));
        } else {
            ui.label("No brains generated yet");
        }

        Frame::canvas(ui.style()).show(ui, |ui| {
            self.update();
            if let Some(simulation) = &self.current_simulation {
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

        add_shared_ui(ui, &mut self.shared_config.lock().unwrap());
    }
}
