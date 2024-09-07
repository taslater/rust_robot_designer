use nalgebra::DMatrix;
// rand gaussian
// use rand::distributions::Standard;
// use meansd::MeanSD;
use rand::prelude::*;
use rand_distr::{Distribution, Normal};
use std::any::Any;

trait Layer: Any {
    fn forward(&self, input: DMatrix<f64>) -> DMatrix<f64>;
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

trait LayerClone {
    fn clone_box(&self) -> Box<dyn LayerClone>;
}

impl<T> LayerClone for T
where
    T: 'static + Layer + Clone,
{
    fn clone_box(&self) -> Box<dyn LayerClone> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn LayerClone> {
    fn clone(&self) -> Box<dyn LayerClone> {
        self.clone_box()
    }
}

#[derive(Clone)]
struct DenseLayer {
    weights: DMatrix<f64>,
    bias: DMatrix<f64>,
    activation: fn(DMatrix<f64>) -> DMatrix<f64>,
}

impl std::fmt::Debug for DenseLayer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "DenseLayer")
    }
}

impl DenseLayer {
    fn new(
        in_size: usize,
        out_size: usize,
        activation: fn(DMatrix<f64>) -> DMatrix<f64>,
        dist: Normal<f64>,
    ) -> Self {
        // get random weights from dist
        let weights =
            DMatrix::<f64>::zeros(out_size, in_size).map(|_| dist.sample(&mut thread_rng()));
        // println!("{:?}", test_weights);
        // let weights = DMatrix::<f64>::new_random(out_size, in_size);
        // println!("{:?}", weights);
        let bias = DMatrix::<f64>::zeros(out_size, 1);
        DenseLayer {
            weights,
            bias,
            activation,
        }
    }

    pub fn get_flat_weights_and_biases(&self) -> Vec<f64> {
        let mut flat: Vec<f64> = Vec::new();
        for i in 0..self.weights.nrows() {
            for j in 0..self.weights.ncols() {
                flat.push(self.weights[(i, j)]);
            }
        }
        for i in 0..self.bias.nrows() {
            flat.push(self.bias[(i, 0)]);
        }
        flat
    }

    pub fn set_weights_and_biases(&mut self, flat: Vec<f64>) {
        let mut flat_index = 0;
        for i in 0..self.weights.nrows() {
            for j in 0..self.weights.ncols() {
                self.weights[(i, j)] = flat[flat_index];
                flat_index += 1;
            }
        }
        for i in 0..self.bias.nrows() {
            self.bias[(i, 0)] = flat[flat_index];
            flat_index += 1;
        }
    }
}

impl Layer for DenseLayer {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    fn forward(&self, input: DMatrix<f64>) -> DMatrix<f64> {
        // println!("{:?}", input);
        (self.activation)(self.weights.clone() * input + &self.bias)
    }
}

#[derive(Clone)]
pub(crate) struct Sequential {
    // layers: Vec<Box<dyn Layer>>,
    layers: Vec<DenseLayer>,
}

impl std::fmt::Debug for Sequential {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Sequential")
    }
}

impl Sequential {
    // fn new(layers: Vec<Box<dyn Layer>>) -> Self {
    fn new(layers: Vec<DenseLayer>) -> Self {
        Sequential { layers }
    }

    // pub fn forward(&self, input: DMatrix<f64>) -> DMatrix<f64> {
    pub fn forward(&self, input: &Vec<f32>) -> DMatrix<f64> {
        let input = DMatrix::<f64>::from_iterator(input.len(), 1, input.iter().map(|&v| v as f64));
        self.layers
            .iter()
            .fold(input, |acc, layer| layer.forward(acc))
    }

    pub fn get_flat_weights_and_biases(&self) -> Vec<f64> {
        self.layers
            .iter()
            .map(|dense_layer| dense_layer.get_flat_weights_and_biases())
            .flatten()
            .collect()
    }

    pub fn set_weights_and_biases(&mut self, flat: Vec<f64>) {
        let mut flat_index = 0;
        for dense_layer in &mut self.layers {
            let layer_size = dense_layer.weights.nrows() * dense_layer.weights.ncols()
                + dense_layer.bias.nrows();
            dense_layer
                .set_weights_and_biases(flat[flat_index..flat_index + layer_size].to_vec());
            flat_index += layer_size;
        }
    }

}

// Activation functions
// fn relu(x: DMatrix<f64>) -> DMatrix<f64> {
//     x.map(|v| v.max(0.0))
// }

fn leaky_relu(x: DMatrix<f64>) -> DMatrix<f64> {
    x.map(|v| v.max(0.1 * v))
}

fn sigmoid(x: DMatrix<f64>) -> DMatrix<f64> {
    x.map(|v| 1.0 / (1.0 + (-v).exp()))
}

pub(crate) fn get_brain(in_size: usize, hidden_sizes: Vec<usize>, out_size: usize) -> Sequential {
    let mut layers: Vec<DenseLayer> = Vec::new();
    let mut prev_size = in_size;
    let normal_relu: Normal<f64> = Normal::new(0.0, 2.0).unwrap();
    let normal_sigmoid: Normal<f64> = Normal::new(0.0, 1.0).unwrap();
    for &size in &hidden_sizes {
        layers.push(DenseLayer::new(prev_size, size, leaky_relu, normal_relu));
        prev_size = size;
    }
    layers.push(DenseLayer::new(
        prev_size,
        out_size,
        sigmoid,
        normal_sigmoid,
    ));

    Sequential::new(layers)
}
