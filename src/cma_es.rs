use crate::cma_es_fns::*;
use nalgebra::{DMatrix, DVector};
use rand::prelude::*;
use rand_distr::StandardNormal;

pub struct CMAES {
    dimension: usize,
    population_size: usize,
    // mu: usize,
    weights: DVector<f64>,
    mu_eff: f64,
    cc: f64,
    cs: f64,
    c1: f64,
    c_mu: f64,
    damps: f64,
    chi_n: f64,
    mean: DVector<f64>,
    sigma: f64,
    c: DMatrix<f64>,
    p_c: DVector<f64>,
    p_s: DVector<f64>,
    b: DMatrix<f64>,
    d: DMatrix<f64>,
    invsqrt_c: DMatrix<f64>,
    eigen_eval: usize,
    counteval: usize,
    rng: ThreadRng,
}

impl CMAES {
    pub fn new(
        initial_mean: DVector<f64>,
        initial_sigma: f64,
        population_size: Option<usize>,
    ) -> Self {
        let dimension = initial_mean.len();
        let lambda = get_λ_rs(dimension, population_size);
        let mu = get_μ_rs(lambda);
        let weights = get_weights_rs(mu, lambda);
        let mu_eff = get_μeff_rs(&weights);
        let cc = get_cc_rs(mu_eff, dimension, None);
        let cs = get_cσ_rs(mu_eff, dimension, None);
        let c1 = get_c1_rs(dimension, mu_eff, None);
        let c_mu = get_cμ_rs(dimension, mu_eff, None);
        let damps = get_damps_rs(mu_eff, dimension, cs, None);
        let chi_n = get_chi_n_rs(dimension);

        let c = DMatrix::identity(dimension, dimension);
        let b = DMatrix::identity(dimension, dimension);
        let d = DMatrix::identity(dimension, dimension);
        let invsqrt_c = DMatrix::identity(dimension, dimension);

        CMAES {
            dimension,
            population_size: lambda,
            // mu,
            weights,
            mu_eff,
            cc,
            cs,
            c1,
            c_mu,
            damps,
            chi_n,
            mean: initial_mean,
            sigma: initial_sigma,
            c,
            p_c: DVector::zeros(dimension),
            p_s: DVector::zeros(dimension),
            b,
            d,
            invsqrt_c,
            eigen_eval: 0,
            counteval: 0,
            rng: thread_rng(),
        }
    }

    pub fn ask(&mut self) -> Vec<DVector<f64>> {
        println!("Asking...");
        let z: DMatrix<f64> = DMatrix::from_fn(self.population_size, self.dimension, |_, _| {
            self.rng.sample::<f64, _>(StandardNormal)
        });
        let x: nalgebra::Matrix<
            f64,
            nalgebra::Dyn,
            nalgebra::Dyn,
            nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Dyn>,
        > = sample_population_rs(&z, &self.mean, self.sigma, &self.b, &self.d);
        // x.column_iter().map(|col| col.into_owned()).collect()
        // return a Vec of rows from x instead of columns
        let mut population = Vec::with_capacity(self.population_size);
        for i in 0..self.population_size {
            let row: nalgebra::Matrix<
                f64,
                nalgebra::Const<1>,
                nalgebra::Dyn,
                nalgebra::ViewStorage<
                    '_,
                    f64,
                    nalgebra::Const<1>,
                    nalgebra::Dyn,
                    nalgebra::Const<1>,
                    nalgebra::Dyn,
                >,
            > = x.row(i);
            // convert row Matrix to owned DVector
            population.push(
                DVector::from_iterator(x.ncols(), row.iter().cloned()),
            );
        }
        population
    }

    pub fn tell(&mut self, solutions: &[DVector<f64>], fitnesses: &[f64]) {
        println!("Telling...");
        self.counteval += self.population_size;

        let x_sorted = sort_population_rs(solutions, fitnesses, 0.0);
        let xold = self.mean.clone();

        let (m_new, y_mean) = update_mean_rs(&x_sorted, &self.mean, &self.weights, self.sigma);
        self.mean = m_new;

        self.p_c = update_evolution_path_p_c_rs(&self.p_c, self.cc, self.mu_eff, &y_mean);

        // let x_diff = &x_sorted - &xold;
        let mut x_diff = x_sorted.clone();
        for i in 0..x_sorted.ncols() {
            x_diff.set_column(i, &(x_sorted.column(i) - &xold));
        }
        // println!("x_diff shape: {:?}", x_diff.shape());
        // let x_diff_matrix = DMatrix::from_column_slice(x_diff.len(), 1, x_diff.as_slice());
        // println!("x_diff_matrix shape: {:?}", x_diff_matrix.shape());

        self.c = update_covariance_matrix_rs(
            &self.c,
            self.c1,
            self.c_mu,
            &self.p_c,
            &x_diff,
            &self.weights,
            self.sigma,
        );

        self.p_s = update_evolution_path_p_σ_rs(
            &self.p_s,
            self.cs,
            self.mu_eff,
            &self.b,
            &self.d,
            &y_mean,
        );

        self.sigma = update_step_size_rs(self.sigma, self.cs, self.damps, &self.p_s, self.chi_n);

        self.update_eigen_decomposition();
    }

    fn update_eigen_decomposition(&mut self) {
        if self.counteval - self.eigen_eval
            > (self.population_size as f64 / ((self.c1 + self.c_mu) * self.dimension as f64) / 10.0)
                as usize
        {
            self.eigen_eval = self.counteval;
            let (diag_d, b_new, d_new) = eigen_decomposition_rs(&self.c);
            self.d = d_new;
            self.b = b_new;
            self.invsqrt_c =
                &self.b * DMatrix::from_diagonal(&diag_d.map(|x| 1.0 / x)) * self.b.transpose();
        }
    }

    pub fn optimize<F>(
        &mut self,
        objective_function: F,
        max_iterations: usize,
    ) -> (DVector<f64>, f64)
    where
        F: Fn(&DVector<f64>) -> f64,
    {
        println!("Optimizing...");
        for _ in 0..max_iterations {
            let solutions = self.ask();
            let fitnesses: Vec<f64> = solutions.iter().map(|s| objective_function(s)).collect();
            self.tell(&solutions, &fitnesses);

            if self.termination_criterion_met() {
                break;
            }
        }

        (self.mean.clone(), self.sigma)
    }

    fn termination_criterion_met(&self) -> bool {
        // Implement termination criteria here
        // For example, you could check if the step size (sigma) is below a certain threshold
        self.sigma < 1e-8
    }
}
