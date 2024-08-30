use nalgebra::{DMatrix, DVector};
// use rand::distributions::{Distribution, Standard};
// use rand::prelude::*;

pub fn get_λ_rs(n: usize, population_size: Option<usize>) -> usize {
    population_size.unwrap_or_else(|| ((n as f64).ln() * 3.0 + 8.0).floor() as usize)
}

pub fn get_μ_rs(λ: usize) -> usize {
    (λ as f64 / 2.0).floor() as usize
}

pub fn get_weights_rs(μ: usize, λ: usize) -> DVector<f64> {
    let mut weights = DVector::zeros(λ);
    for i in 0..μ {
        weights[i] = ((μ as f64 + 0.5).ln() - (i as f64 + 1.0).ln()).max(0.0);
    }
    weights /= weights.sum();
    weights
}

pub fn get_μeff_rs(weights: &DVector<f64>) -> f64 {
    weights.sum().powi(2) / weights.dot(weights)
}

pub fn get_cc_rs(μeff: f64, n: usize, cc: Option<f64>) -> f64 {
    cc.unwrap_or_else(|| (4.0 + μeff / n as f64) / (n as f64 + 4.0 + 2.0 * μeff / n as f64))
}

pub fn get_cσ_rs(μeff: f64, n: usize, cσ: Option<f64>) -> f64 {
    cσ.unwrap_or_else(|| (μeff + 2.0) / (n as f64 + μeff + 5.0))
}

pub fn get_c1_rs(n: usize, μeff: f64, c1: Option<f64>) -> f64 {
    c1.unwrap_or_else(|| 2.0 / ((n as f64 + 1.3).powi(2) + μeff))
}

pub fn get_cμ_rs(n: usize, μeff: f64, cμ: Option<f64>) -> f64 {
    cμ.unwrap_or_else(|| {
        2.0 * (μeff - 2.0 + 1.0 / μeff) / ((n as f64 + 2.0).powi(2) + 2.0 * μeff / 2.0)
    })
}

pub fn get_damps_rs(μeff: f64, n: usize, cσ: f64, damps: Option<f64>) -> f64 {
    damps.unwrap_or_else(|| {
        1.0 + 2.0 * (0.0f64).max(((μeff - 1.0) / (n as f64 + 1.0)).sqrt() - 1.0) + cσ
    })
}

pub fn get_chi_n_rs(n: usize) -> f64 {
    (n as f64).sqrt() * (1.0 - 1.0 / (4.0 * n as f64) + 1.0 / (21.0 * (n as f64).powi(2)))
}

// sample_population_rs(&z, &self.mean, self.sigma, &self.b, &self.d)
pub fn sample_population_rs(
    z: &DMatrix<f64>,
    m: &DVector<f64>,
    σ: f64,
    b: &DMatrix<f64>,
    d: &DMatrix<f64>,
) -> DMatrix<f64> {
    // println!("z shape: {:?}", z.shape());
    // println!("m shape: {:?}", m.shape());
    // println!("b shape: {:?}", b.shape());
    // println!("d shape: {:?}", d.shape());
    let y = z * (b * d);
    // println!("y shape: {:?}", y.shape());
    // z shape: (28, 937)
    // m shape: (937, 1)
    // b shape: (937, 937)
    // d shape: (937, 937)
    // y shape: (28, 937)
    // let x = m_vector + σ * &y;
    // let x = m + &y.add_scalar(σ);
    // println!("y_row shape: {:?}", y.row(0).shape());
    // println!("y_col shape: {:?}", y.column(0).shape());
    // println!("m_row shape: {:?}", m.row(0).shape());
    // println!("m_col shape: {:?}", m.column(0).shape());
    // y_row shape: (1, 937)
    // y_col shape: (28, 1)
    // m_row shape: (1, 1)
    // m_col shape: (937, 1)
    let m_t = m.transpose();
    let mut x = DMatrix::zeros(y.nrows(), y.ncols());
    // println!("m_t shape: {:?}", m_t.shape());
    for i in 0..y.nrows() {
        x.set_row(i, &(&m_t + &y.row(i) * σ));
    }
    // println!("x shape: {:?}", x.shape());
    x
}

pub fn sort_population_rs(
    solutions: &[DVector<f64>],
    fitnesses: &[f64],
    penalty: f64,
) -> DMatrix<f64> {
    let mut sorted_indices: Vec<usize> = (0..solutions.len()).collect();
    sorted_indices.sort_by(|&i, &j| {
        (fitnesses[i] + penalty)
            .partial_cmp(&(fitnesses[j] + penalty))
            .unwrap()
    });

    let sorted_solutions: Vec<DVector<f64>> = sorted_indices
        .iter()
        .map(|&i| solutions[i].clone())
        .collect();
    DMatrix::from_columns(&sorted_solutions)
}

pub fn update_mean_rs(
    x_sorted: &DMatrix<f64>, // Nxλ
    m: &DVector<f64>, // Nx1
    weights: &DVector<f64>, // λx1
    σ: f64, // scalar
) -> (DVector<f64>, DVector<f64>) {
    // println!("x_sorted shape: {:?}", x_sorted.shape());
    // println!("m shape: {:?}", m.shape());
    // println!("weights shape: {:?}", weights.shape());
    // let x_diff = x_sorted - m;
    let mut x_diff = x_sorted.clone();
    // println!("x_diff rows: {:?}", x_diff.nrows());
    // println!("x_diff cols: {:?}", x_diff.ncols());
    // println!("x_diff row shape: {:?}", x_diff.row(0).shape());
    // println!("x_diff col shape: {:?}", x_diff.column(0).shape());
    for i in 0..x_sorted.ncols() {
        x_diff.set_column(i, &(x_sorted.column(i) - m));
    }
    // println!("x_diff shape: {:?}", x_diff.shape());
    let x_mean = x_diff * weights;
    // println!("x_mean shape: {:?}", x_mean.shape());
    let m_new = m + &x_mean;
    // println!("m_new shape: {:?}", m_new.shape());
    let y_mean = &x_mean / σ;
    // println!("y_mean shape: {:?}", y_mean.shape());
    (m_new, y_mean)
}

pub fn update_evolution_path_p_c_rs(
    p_c: &DVector<f64>,
    cc: f64,
    μeff: f64,
    y_mean: &DVector<f64>,
) -> DVector<f64> {
    (1.0 - cc) * p_c + (cc * (2.0 - cc) * μeff).sqrt() * y_mean
}

pub fn update_covariance_matrix_rs(
    c: &DMatrix<f64>,
    c1: f64,
    cμ: f64,
    p_c_new: &DVector<f64>,
    x_diff: &DMatrix<f64>,
    weights: &DVector<f64>,
    σ: f64,
) -> DMatrix<f64> {
    // println!("c shape: {:?}", c.shape());
    // println!("p_c_new shape: {:?}", p_c_new.shape());
    // println!("x_diff shape: {:?}", x_diff.shape());
    // println!("weights shape: {:?}", weights.shape());
    let p_c_matrix = p_c_new * p_c_new.transpose();
    let c_m: Vec<DMatrix<f64>> = x_diff
        .column_iter()
        .map(|col| {
            let e = col / σ;
            let e_t = e.clone().transpose();
            e * e_t
        })
        .collect();
    let y_s = c_m
        .iter()
        .zip(weights.iter())
        .fold(DMatrix::zeros(c.nrows(), c.ncols()), |acc, (cm, &w)| {
            acc + cm * w
        });

    let mut c_new = (1.0 - c1 - cμ) * c + c1 * &p_c_matrix + cμ * &y_s;

    // Enforce symmetry
    for i in 0..c_new.nrows() {
        for j in (i + 1)..c_new.ncols() {
            c_new[(j, i)] = c_new[(i, j)];
        }
    }

    c_new
}

pub fn update_evolution_path_p_σ_rs(
    p_σ: &DVector<f64>,
    cσ: f64,
    μeff: f64,
    b: &DMatrix<f64>,
    d: &DMatrix<f64>,
    y_mean: &DVector<f64>,
) -> DVector<f64> {
    let d_diag = d.diagonal();
    let d_inv_diag = d_diag.map(|x| 1.0 / x);
    let d_inv = DMatrix::from_diagonal(&d_inv_diag);
    let c_inv_squared = b * &d_inv * b.transpose();
    let c_inv_squared_y = &c_inv_squared * y_mean;
    (1.0 - cσ) * p_σ + (cσ * (2.0 - cσ) * μeff).sqrt() * c_inv_squared_y
}

pub fn update_step_size_rs(
    σ: f64, cσ: f64, damps: f64, p_σ_new: &DVector<f64>, chi_n: f64
) -> f64 {
    σ * ((cσ / damps) * (p_σ_new.norm() / chi_n - 1.0)).exp()
}

pub fn eigen_decomposition_rs(c_new: &DMatrix<f64>) -> (DVector<f64>, DMatrix<f64>, DMatrix<f64>) {
    let eigen = c_new.clone().symmetric_eigen();
    let mut u = eigen.eigenvalues;
    let mut b_new = eigen.eigenvectors;

    // Sort eigenvalues and eigenvectors in descending order
    let mut indices: Vec<usize> = (0..u.len()).collect();
    indices.sort_unstable_by(|&i, &j| u[j].partial_cmp(&u[i]).unwrap());

    u = DVector::from_iterator(u.len(), indices.iter().map(|&i| u[i]));
    b_new = DMatrix::from_columns(
        &indices
            .iter()
            .map(|&i| b_new.column(i).into_owned())
            .collect::<Vec<_>>(),
    );

    let diag_d = u.map(|x| x.sqrt());
    let d_new = DMatrix::from_diagonal(&diag_d);

    (diag_d, b_new, d_new)
}
