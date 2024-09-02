// use itertools::Itertools;
// use nalgebra::{Const, DMatrix, DVector, Dyn, Matrix, SymmetricEigen, VecStorage};
// use rand::prelude::*;
// use rand_distr::StandardNormal;

// #[derive(Debug)]
// struct CMAESParameters {
//     pub dimension: usize,
//     pub population_size: usize,
//     pub mu: usize,
//     pub weights: Vec<f64>,
//     pub mu_eff: f64,
//     pub c_c: f64,
//     pub c_s: f64,
//     pub c_1: f64,
//     pub c_mu: f64,
//     pub d_amps: f64,
//     pub chi_n: f64,
// }

// impl CMAESParameters {
//     pub fn new(dimension: usize, population_size: usize) -> Self {
//         let mu: usize = population_size / 2;
//         // weights = log(mu+1/2)-log(1:mu)'; % muXone array for weighted recombination
//         let weights: Vec<f64> = (0..mu).map(|i| (mu as f64 + 0.5 - i as f64).ln()).collect();
//         // weights = weights/sum(weights);  % normalize recombination weights array
//         let weights_sum: f64 = weights.iter().sum();
//         let weights: Vec<f64> = weights.iter().map(|w| w / weights_sum).collect();
//         // mueff=sum(weights)^2/sum(weights.^2); % variance-effectiveness of sum w_i x_i
//         let mu_eff: f64 =
//             weights.iter().sum::<f64>().powi(2) / weights.iter().map(|w| w.powi(2)).sum::<f64>();

//         // cc = (4+mueff/N) / (N+4 + 2*mueff/N);  % time constant for cumulation for C
//         let c_c: f64 = (4.0 + mu_eff / dimension as f64)
//             / (dimension as f64 + 4.0 + 2.0 * mu_eff / dimension as f64);
//         // cs = (mueff+2) / (N+mueff+5);  % t-const for cumulation for sigma control
//         let c_s: f64 = (mu_eff + 2.0) / (dimension as f64 + mu_eff + 5.0);
//         // c1 = 2 / ((N+1.3)^2+mueff);    % learning rate for rank-one update of C
//         let c_1: f64 = 2.0 / ((dimension as f64 + 1.3).powi(2) + mu_eff);
//         // cmu = min(1-c1, 2 * (mueff-2+1/mueff) / ((N+2)^2+mueff));  % and for rank-mu update
//         let c_mu: f64 = (1.0 - c_1)
//             .min(2.0 * (mu_eff - 2.0 + 1.0 / mu_eff) / ((dimension as f64 + 2.0).powi(2) + mu_eff));
//         // damps = 1 + 2*max(0, sqrt((mueff-1)/(N+1))-1) + cs; % damping for sigma
//         let d_amps: f64 = 1.0
//             + 2.0 * (0.0f64).max(((mu_eff - 1.0) / (dimension as f64 + 1.0)).sqrt() - 1.0)
//             + c_s;
//         // chiN=N^0.5*(1-1/(4*N)+1/(21*N^2));  % expectation of ||N(0,I)||
//         let chi_n: f64 = (dimension as f64).sqrt()
//             * (1.0 - 1.0 / (4.0 * dimension as f64) + 1.0 / (21.0 * (dimension as f64).powi(2)));

//         let params = CMAESParameters {
//             dimension,
//             population_size,
//             mu,
//             weights,
//             mu_eff,
//             c_c,
//             c_s,
//             c_1,
//             c_mu,
//             d_amps,
//             chi_n,
//         };
//         // println!("{:?}", params);
//         params
//     }
// }

// pub struct CMAES {
//     params: CMAESParameters,
//     mean: DVector<f64>,
//     sigma: f64,
//     c: DMatrix<f64>,
//     p_c: DVector<f64>,
//     p_s: DVector<f64>,
//     eigen_decomposition_updated: usize,
//     eigen_basis: DMatrix<f64>,
//     eigen_values: DVector<f64>,
//     invsqrt_c: DMatrix<f64>,
//     rng: ThreadRng,
//     counteval: usize,
// }

// impl CMAES {
//     pub fn new(mean: DVector<f64>, sigma: f64, pop_multiplier: usize) -> Self {
//         let dimension: usize = mean.len();
//         // lambda = 4+floor(3*log(N));  % population size, offspring number
//         let params: CMAESParameters = CMAESParameters::new(
//             dimension,
//             pop_multiplier * (4.0 + (3.0 * (dimension as f64).ln()).floor()) as usize,
//         );

//         // B = eye(N,N);                       % B defines the coordinate system
//         // D = ones(N,1);                      % diagonal D defines the scaling
//         // C = B * diag(D.^2) * B';            % covariance matrix C
//         let c: Matrix<f64, Dyn, Dyn, VecStorage<f64, Dyn, Dyn>> =
//             DMatrix::<f64>::identity(dimension, dimension);
//         let p_c: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> =
//             DVector::<f64>::zeros(dimension);
//         let p_s: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> =
//             DVector::<f64>::zeros(dimension);

//         let eigen_decomposition_updated: usize = 0;
//         let eigen_basis: Matrix<f64, Dyn, Dyn, VecStorage<f64, Dyn, Dyn>> =
//             DMatrix::<f64>::identity(dimension, dimension);
//         let eigen_values: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> =
//             DVector::<f64>::from_element(dimension, 1.0);
//         let invsqrt_c: Matrix<f64, Dyn, Dyn, VecStorage<f64, Dyn, Dyn>> =
//             DMatrix::<f64>::identity(dimension, dimension);

//         let rng: ThreadRng = thread_rng();

//         CMAES {
//             params,
//             mean,
//             sigma,
//             c,
//             p_c,
//             p_s,
//             eigen_decomposition_updated,
//             eigen_basis,
//             eigen_values,
//             invsqrt_c,
//             rng,
//             counteval: 0,
//         }
//     }

//     pub fn ask(&mut self) -> Vec<DVector<f64>> {
//         self.update_eigen_decomposition();

//         let mut population = Vec::with_capacity(self.params.population_size);
//         for _ in 0..self.params.population_size {
//             // arx(:,k) = xmean + sigma * B * (D .* randn(N,1)); % m + sig * Normal(0,C)
//             let z: DVector<f64> = DVector::from_iterator(
//                 self.params.dimension,
//                 self.eigen_values
//                     .iter()
//                     .map(|&ev| self.sigma * ev.sqrt() * self.rng.sample::<f64, _>(StandardNormal)),
//             );
//             let x = self.mean.clone() + self.eigen_basis.clone() * z;
//             population.push(x);
//         }
//         population
//     }

//     fn update_eigen_decomposition(&mut self) {
//         // println!("counteval: {:?}", self.counteval);
//         // if counteval - eigeneval > lambda/(c1+cmu)/N/10  % to achieve O(N^2)
//         if self.counteval as f64
//             > self.eigen_decomposition_updated as f64
//                 + self.params.population_size as f64
//                     / ((self.params.c_1 + self.params.c_mu) * self.params.dimension as f64)
//                     / 10.0
//         {
//             // println!("Updating eigen decomposition");
//             // eigeneval = counteval;
//             self.eigen_decomposition_updated = self.counteval;
//             // C = triu(C) + triu(C,1)'; % enforce symmetry
//             // self.c = self.c.clone().symmetric_eigen().eigenvectors;
//             self.c = 0.5 * (self.c.clone() + self.c.transpose());
//             // [B,D] = eig(C);           % eigen decomposition, B==normalized eigenvectors
//             // D = sqrt(diag(D));        % D is a vector of standard deviations now
//             let eigen_decomp: SymmetricEigen<f64, Dyn> = self.c.clone().symmetric_eigen();
//             // println!("eigen_decomp: {:?}", eigen_decomp);
//             self.eigen_values = eigen_decomp.eigenvalues.map(|ev| ev.sqrt());
//             self.eigen_basis = eigen_decomp.eigenvectors.transpose();
//             // invsqrtC = B * diag(D.^-1) * B';
//             // print out the dimensions for debugging
//             // println!("eigen_basis: {:?}", self.eigen_basis);
//             // println!("eigen_values: {:?}", self.eigen_values);
//             // println!("invsqrtC: {:?}", self.invsqrt_c);
//             let diag_inv_sqrt_eigen_values: DMatrix<f64> =
//                 DMatrix::from_diagonal(&self.eigen_values.map(|ev| ev.recip()));
//             // println!(
//             //     "diag_inv_sqrt_eigen_values: {:?}",
//             //     diag_inv_sqrt_eigen_values
//             // );
//             assert_eq!(self.eigen_basis.shape(), self.invsqrt_c.shape());
//             let new_invsqrt_c = self.eigen_basis.clone()
//                 * diag_inv_sqrt_eigen_values
//                 * self.eigen_basis.transpose();
//             // println!("new_invsqrtC: {:?}", new_invsqrt_c);
//             // println!("new_invsqrtC shape: {:?}", new_invsqrt_c.shape());
//             assert_eq!(self.invsqrt_c.shape(), new_invsqrt_c.shape());
//             self.invsqrt_c = new_invsqrt_c;
//         }
//     }

//     pub fn tell(&mut self, population: &[DVector<f64>], fitness_values: &[f64]) {
//         // [arfitness, arindex] = sort(arfitness); % minimization
//         // println!("fitness_values: {:?}", fitness_values);
//         let sorted_indices: Vec<usize> = fitness_values
//             .iter()
//             .enumerate()
//             .sorted_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
//             .map(|(index, _)| index)
//             .collect::<Vec<_>>();
//         let best_solution: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> =
//             population[sorted_indices[0]].clone();
//         // print the top fitness values
//         let top_fitness_values: Vec<f64> = sorted_indices
//             .iter()
//             .take(3)
//             .map(|&idx| fitness_values[idx])
//             .collect();
//         println!("top_fitness_values: {:?}", top_fitness_values);

//         self.counteval += self.params.population_size;

//         // xold = xmean;
//         let xold: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> = self.mean.clone();
//         // xmean = arx(:,arindex(1:mu))*weights;   % recombination, new mean value
//         self.update_mean(population, &sorted_indices);

//         // ps = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC * (xmean-xold) / sigma;
//         // hsig = norm(ps)/sqrt(1-(1-cs)^(2*counteval/lambda))/chiN < 1.4 + 2/(N+1);
//         // pc = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * (xmean-xold) / sigma;
//         self.update_evolution_paths(&xold, &best_solution);

//         // artmp = (1/sigma) * (arx(:,arindex(1:mu))-repmat(xold,1,mu));
//         // C = (1-c1-cmu) * C ...                  % regard old matrix
//         //     + c1 * (pc*pc' ...                 % plus rank one update
//         //             + (1-hsig) * cc*(2-cc) * C) ... % minor correction if hsig==0
//         //     + cmu * artmp * diag(weights) * artmp'; % plus rank mu update
//         self.update_covariance_matrix(population, &xold);

//         // sigma = sigma * exp((cs/damps)*(norm(ps)/chiN - 1));
//         self.update_sigma();
//     }

//     fn update_mean(&mut self, population: &[DVector<f64>], sorted_indices: &[usize]) {
//         let new_mean: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> =
//             DVector::from_iterator(
//                 self.params.dimension,
//                 (0..self.params.dimension).map(|i| {
//                     sorted_indices[..self.params.mu]
//                         .iter()
//                         .zip(self.params.weights.iter())
//                         .map(|(&idx, &w)| w * population[idx][i])
//                         .sum::<f64>()
//                         / self.params.mu as f64
//                 }),
//             );
//         self.mean = new_mean;
//     }

//     fn update_evolution_paths(&mut self, xold: &DVector<f64>, _best_solution: &DVector<f64>) {
//         // ps = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC * (xmean-xold) / sigma;
//         // Compute the conjugate evolution path ps
//         let ps: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> =
//             (1.0 - self.params.c_s) * self.p_s.clone()
//                 + (self.params.c_s * (2.0 - self.params.c_s) * self.params.mu_eff).sqrt()
//                     * self.invsqrt_c.clone()
//                     * (self.mean.clone() - xold)
//                     / self.sigma;

//         // hsig = norm(ps)/sqrt(1-(1-cs)^(2*counteval/lambda))/chiN < 1.4 + 2/(N+1);
//         // Check if the conjugate evolution path ps is larger than the threshold
//         let hsig: bool = ((ps.norm()
//             / (1.0
//                 - (1.0 - self.params.c_s)
//                     .powi(2 * self.counteval as i32 / self.params.population_size as i32))
//             .sqrt())
//             / self.params.chi_n)
//             < (1.4 + 2.0 / (self.params.dimension as f64 + 1.0));

//         // pc = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * (xmean-xold) / sigma;
//         // Update the evolution path pc
//         let pc: Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> =
//             (1.0 - self.params.c_c) * self.p_c.clone()
//                 + hsig as i32 as f64
//                     * (self.params.c_c * (2.0 - self.params.c_c) * self.params.mu_eff).sqrt()
//                     * (self.mean.clone() - xold)
//                     / self.sigma;

//         self.p_s = ps;
//         self.p_c = pc;
//     }

//     fn update_covariance_matrix(&mut self, population: &[DVector<f64>], xold: &DVector<f64>) {
//         // artmp = (1/sigma) * (arx(:,arindex(1:mu))-repmat(xold,1,mu));
//         // artmp represents the scaled difference between the best mu individuals and the old mean
//         let artmp: Vec<DVector<f64>> = population[..self.params.mu]
//             .iter()
//             .map(|x| (x - xold) / self.sigma)
//             .collect();

//         // C = (1-c1-cmu) * C ...                  % regard old matrix
//         //     + c1 * (pc*pc' ...                 % plus rank one update
//         //             + (1-hsig) * cc*(2-cc) * C) ... % minor correction if hsig==0
//         //     + cmu * artmp * diag(weights) * artmp'; % plus rank mu update

//         // Regard the old covariance matrix
//         self.c *= 1.0 - self.params.c_1 - self.params.c_mu;

//         // Add the rank one update
//         self.c += self.params.c_1 * (self.p_c.clone() * self.p_c.transpose());

//         // Add the minor correction if hsig is zero
//         // Note: hsig is not defined in this function, so you need to pass it as an argument or calculate it here
//         // self.c += (1.0 - hsig as i32 as f64) * self.params.c_c * (2.0 - self.params.c_c) * self.c.clone();

//         // Add the rank mu update
//         // This part is causing the panic, so let's break it down further
//         // let rank_mu_update: DMatrix<f64> = artmp
//         //     .iter()
//         //     .zip(self.params.weights.iter())
//         //     .map(|(z, &w)| z.kronecker(z) * w)
//         //     .sum();
//         // let rank_mu_update: DMatrix<f64> = DMatrix::from_iterator(
//         //     self.params.dimension,
//         //     self.params.dimension,
//         //     artmp
//         //         .iter()
//         //         .zip(self.params.weights.iter())
//         //         .flat_map(|(z, &w)| {
//         //             let outer_product = z * z.transpose();
//         //             outer_product
//         //                 .iter()
//         //                 .map(move |&x| x * w)
//         //                 .collect::<Vec<f64>>()
//         //         }),
//         // );

//         let mut rank_mu_update = DMatrix::zeros(self.params.dimension, self.params.dimension);
//         for (z, &w) in artmp.iter().zip(self.params.weights.iter()) {
//             for i in 0..self.params.dimension {
//                 for j in 0..self.params.dimension {
//                     rank_mu_update[(i, j)] += z[i] * z[j] * w;
//                 }
//             }
//         }
//         // assert that the dimensions are correct
//         // println!("rank_mu_update shape: {:?}", rank_mu_update.shape());
//         // println!("C shape: {:?}", self.c.shape());
//         assert_eq!(rank_mu_update.shape(), self.c.shape(),);
//         self.c += rank_mu_update;
//     }

//     fn update_sigma(&mut self) {
//         // sigma = sigma * exp((cs/damps)*(norm(ps)/chiN - 1));
//         let sigma_exp =
//             (self.params.c_s / self.params.d_amps) * (self.p_s.norm() / self.params.chi_n - 1.0);
//         self.sigma *= sigma_exp.exp();
//     }
// }