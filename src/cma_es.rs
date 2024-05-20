use itertools::Itertools;
use nalgebra::{DMatrix, DVector, RowDVector};
use rand::prelude::*;
use rand_distr::StandardNormal;

struct CMAESParameters {
    pub dimension: usize,
    pub population_size: usize,
    pub mu: usize,
    pub weights: Vec<f64>,
    pub mu_eff: f64,
    pub c_c: f64,
    pub c_s: f64,
    pub c_1: f64,
    pub c_mu: f64,
    pub d_amps: f64,
    pub chi_n: f64,
}

impl CMAESParameters {
    pub fn new(dimension: usize, population_size: usize) -> Self {
        let mu = population_size / 2;
        let mu_eff = (population_size as f64).ln();
        // let alpha_mu = 1.0 + (mu_eff / 2.0).floor().max(1.0);

        let weights: Vec<f64> = (0..mu)
            .map(|i| (population_size as f64 / 2.0 + 0.5 - i as f64).ln())
            .collect();
        let weights_sum: f64 = weights.iter().sum();
        let weights: Vec<f64> = weights.iter().map(|w| w / weights_sum).collect();

        let c_s = (mu_eff + 2.0) / (dimension as f64 + mu_eff + 5.0);
        let c_c = 4.0 / (dimension as f64 + 4.0);
        let c_1 = 2.0 / ((dimension as f64 + 1.3).powi(2) + mu_eff);
        let c_mu = (2.0 * (mu_eff - 2.0 + 1.0 / mu_eff)).min(1.0 - c_1);
        let d_amps =
            1.0 + 2.0 * (0.0f64).max(((mu_eff - 1.0) / (dimension as f64 + 1.0)).sqrt()) + c_s;
        let chi_n = (dimension as f64).sqrt()
            * (1.0 - 1.0 / (4.0 * dimension as f64) + 1.0 / (21.0 * (dimension as f64).powi(2)));

        CMAESParameters {
            dimension,
            population_size,
            mu,
            weights,
            mu_eff,
            c_c,
            c_s,
            c_1,
            c_mu,
            d_amps,
            chi_n,
        }
    }
}

pub struct CMAES {
    params: CMAESParameters,
    mean: DVector<f64>,
    sigma: f64,
    c: DMatrix<f64>,
    p_c: DVector<f64>,
    p_s: DVector<f64>,
    eigen_decompositon_updated: usize,
    eigen_basis: DMatrix<f64>,
    eigen_values: DVector<f64>,
    invsqrt_c: DMatrix<f64>,
    rng: ThreadRng,
}

impl CMAES {
    pub fn new(mean: DVector<f64>, sigma: f64) -> Self {
        let dimension = mean.len();
        let params = CMAESParameters::new(
            dimension,
            (4.0 + (3.0 * (dimension as f64).ln()).floor()) as usize,
        );

        let c = DMatrix::<f64>::identity(dimension, dimension);
        let p_c = DVector::<f64>::zeros(dimension);
        let p_s = DVector::<f64>::zeros(dimension);

        let eigen_decompositon_updated = 0;
        let eigen_basis = DMatrix::<f64>::identity(dimension, dimension);
        let eigen_values = DVector::<f64>::from_element(dimension, 1.0);
        let invsqrt_c = DMatrix::<f64>::identity(dimension, dimension);

        let rng = thread_rng();

        CMAES {
            params,
            mean,
            sigma,
            c,
            p_c,
            p_s,
            eigen_decompositon_updated,
            eigen_basis,
            eigen_values,
            invsqrt_c,
            rng,
        }
    }

    pub fn ask(&mut self) -> Vec<DVector<f64>> {
        self.update_eigen_decomposition(self.eigen_decompositon_updated);

        let mut population = Vec::with_capacity(self.params.population_size);
        for _ in 0..self.params.population_size {
            let z: DVector<f64> = DVector::from_iterator(
                self.params.dimension,
                self.eigen_values
                    .iter()
                    // .map(|&ev| self.sigma * ev.sqrt() * self.rng.sample(rand_distr::StandardNormal)),
                    .map(|&ev| self.sigma * ev.sqrt() * self.rng.sample::<f64, _>(StandardNormal)),
            );
            let x = self.mean.clone() + self.eigen_basis.clone() * z;
            population.push(x);
        }

        population
    }

    pub fn tell(&mut self, population: &[DVector<f64>], fitness_values: &[f64]) {
        let sorted_indices = fitness_values
            .iter()
            .enumerate()
            .sorted_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        let best_solution = population[sorted_indices[0]].clone();

        self.update_mean(population, &sorted_indices);
        self.update_evolution_paths(&best_solution);
        self.update_covariance_matrix(population);
        self.update_sigma();
    }

    fn update_eigen_decomposition(&mut self, current_generation: usize) {
        if current_generation > self.eigen_decompositon_updated + self.params.dimension / 10 {
            let eigen_decomp = self.c.clone().symmetric_eigen();
            self.eigen_values = eigen_decomp.eigenvalues;
            self.eigen_basis = eigen_decomp.eigenvectors.transpose();
            self.invsqrt_c = self
                .eigen_basis
                .component_mul(&self.eigen_values.map(|ev| ev.sqrt().recip()))
                * self.eigen_basis.transpose();
            self.eigen_decompositon_updated = current_generation;
        }
    }

    fn update_mean(&mut self, population: &[DVector<f64>], sorted_indices: &[usize]) {
        assert_eq!(population.len(), sorted_indices.len());
        assert!(self.params.mu <= sorted_indices.len());
        assert_eq!(self.params.mu, self.params.weights.len());
        // check ncols (self.params.dimension) and nrows (self.params.mu) of population
        assert_eq!(population[0].len(), self.params.dimension);

        let iter = sorted_indices[..self.params.mu]
            .iter()
            .zip(self.params.weights.iter())
            .flat_map(|(idx, w)| population[*idx].iter().map(move |x| w * x));

        // Check that the iterator will produce the correct number of elements
        let iter_len = iter.clone().count();
        println!("iter_len: {}", iter_len);
        println!("self.params.dimension: {}", self.params.dimension);
        assert_eq!(iter_len, self.params.dimension);

        let new_mean = RowDVector::from_iterator(self.params.dimension, iter);
        let mean_diff = new_mean.clone() - self.mean.clone();
        self.mean = DVector::from_iterator(new_mean.len(), new_mean.iter().cloned());

        let h_sigma = (self.p_s.norm_squared() / self.params.dimension as f64 / self.params.mu_eff
            * self.params.c_s
            * (2.0 - self.params.c_s))
            .sqrt();
        let delta_h_sigma = (1.0 - h_sigma.powi(2)).sqrt();

        self.p_c = (1.0 - self.params.c_c) * self.p_c.clone()
            + delta_h_sigma
                * (self.params.c_c * (2.0 - self.params.c_c) * self.params.mu_eff).sqrt()
                * self.invsqrt_c.clone()
                * mean_diff;
        let p_s: nalgebra::Matrix<
            f64,
            nalgebra::Const<1>,
            nalgebra::Const<1>,
            nalgebra::ArrayStorage<f64, 1, 1>,
        > = (1.0 - self.params.c_s) * self.p_s.clone()
            + (self.params.c_s * (2.0 - self.params.c_s) * self.params.mu_eff).sqrt()
                * mean_diff.normalize()
                / self.sigma;
        self.p_s = DVector::from_column_slice(p_s.as_slice());
    }

    fn update_evolution_paths(&mut self, best_solution: &DVector<f64>) {
        let best_diff = best_solution - self.mean.clone();
        let h_sigma = (self.p_s.norm_squared() / self.params.dimension as f64 / self.params.mu_eff
            * self.params.c_s
            * (2.0 - self.params.c_s))
            .sqrt();
        let delta_h_sigma = (1.0 - h_sigma.powi(2)).sqrt();

        self.p_c = (1.0 - self.params.c_c) * self.p_c.clone()
            + delta_h_sigma
                * (self.params.c_c * (2.0 - self.params.c_c) * self.params.mu_eff).sqrt()
                * self.invsqrt_c.clone()
                * best_diff.clone();
        self.p_s = (1.0 - self.params.c_s) * self.p_s.clone()
            + (self.params.c_s * (2.0 - self.params.c_s) * self.params.mu_eff).sqrt() * best_diff
                / self.sigma;
    }

    fn update_covariance_matrix(&mut self, population: &[DVector<f64>]) {
        let c_mu = self.params.c_mu * (self.params.dimension as f64 + 2.0)
            / (self.params.dimension as f64 + 2.0);

        self.c *= 1.0 - self.params.c_1 - c_mu;

        let c_mu_eff_sqrt = (self.params.c_mu * self.params.mu_eff).sqrt();
        let c_mu_eff_sqrt_inv = 1.0 / c_mu_eff_sqrt;
        let rank_one_update = self.p_c.clone() * self.p_c.transpose() * self.params.c_1;
        self.c += rank_one_update;

        let rank_mu_update: DMatrix<f64> = population[..self.params.mu]
            .iter()
            .zip(self.params.weights.iter())
            .map(|(x_i, w_i)| {
                let z_i = self.invsqrt_c.clone() * (x_i - self.mean.clone()) * c_mu_eff_sqrt_inv;
                z_i.clone() * z_i.transpose() * *w_i
            })
            .sum();
        self.c += rank_mu_update;
    }

    fn update_sigma(&mut self) {
        let sigma_exp = ((self.p_s.norm_squared() / self.params.dimension as f64).sqrt() - 1.0)
            * (self.params.c_s / self.params.d_amps);
        self.sigma *= (1.0 + sigma_exp).exp();
    }
}

// fn main() {
//     let objective_function = |x: &DVector<f64>| {
//         let n = x.len();
//         let mut sum = 0.0;
//         for i in 0..n {
//             sum += 10.0_f64.powf(6.0 * i as f64 / (n - 1) as f64) * x[i].powi(2);
//         }
//         sum
//     };

//     let dimension = 10;
//     let mean = DVector::from_element(dimension, 0.5);
//     let sigma = 0.2;

//     let mut optimizer = CMAES::new(mean, sigma);
//     let mut best_fitness = f64::INFINITY;
//     let mut best_solution = None;

//     loop {
//         let population = optimizer.ask();
//         let fitness_values: Vec<f64> = population.iter().map(|x| objective_function(x)).collect();
//         optimizer.tell(&population, &fitness_values);

//         let current_best_fitness = fitness_values[0];
//         if current_best_fitness < best_fitness {
//             best_fitness = current_best_fitness;
//             best_solution = Some(population[0].clone());
//         }

//         if best_fitness < 1e-10 {
//             break;
//         }
//     }

//     println!("Best solution found: {:?}", best_solution);
//     println!("Best fitness: {}", best_fitness);
// }
