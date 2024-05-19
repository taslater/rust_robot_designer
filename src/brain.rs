use nalgebra::DMatrix;

trait Layer {
    fn forward(&self, input: DMatrix<f64>) -> DMatrix<f64>;
}

struct DenseLayer {
    weights: DMatrix<f64>,
    bias: DMatrix<f64>,
    activation: fn(DMatrix<f64>) -> DMatrix<f64>,
}

impl DenseLayer {
    fn new(in_size: usize, out_size: usize, activation: fn(DMatrix<f64>) -> DMatrix<f64>) -> Self {
        let weights = DMatrix::<f64>::new_random(out_size, in_size);
        let bias = DMatrix::<f64>::zeros(out_size, 1);
        DenseLayer {
            weights,
            bias,
            activation,
        }
    }
}

impl Layer for DenseLayer {
    fn forward(&self, input: DMatrix<f64>) -> DMatrix<f64> {
        (self.activation)(self.weights.clone() * input + &self.bias)
    }
}

pub(crate) struct Sequential {
    layers: Vec<Box<dyn Layer>>,
}

impl Sequential {
    fn new(layers: Vec<Box<dyn Layer>>) -> Self {
        Sequential { layers }
    }

    fn forward(&self, input: DMatrix<f64>) -> DMatrix<f64> {
        self.layers.iter().fold(input, |acc, layer| layer.forward(acc))
    }
}

// Activation functions
fn relu(x: DMatrix<f64>) -> DMatrix<f64> {
    x.map(|v| v.max(0.0))
}

fn leaky_relu(x: DMatrix<f64>) -> DMatrix<f64> {
    x.map(|v| v.max(0.01 * v))
}

fn sigmoid(x: DMatrix<f64>) -> DMatrix<f64> {
    x.map(|v| 1.0 / (1.0 + (-v).exp()))
}

pub fn get_brain(in_size: usize, hidden_sizes: Vec<usize>, out_size: usize) -> Sequential {
    let mut layers: Vec<Box<dyn Layer>> = Vec::new();
    let mut prev_size = in_size;
    for &size in &hidden_sizes {
        layers.push(Box::new(DenseLayer::new(prev_size, size, leaky_relu)));
        prev_size = size;
    }
    layers.push(Box::new(DenseLayer::new(prev_size, out_size, sigmoid)));

    Sequential::new(layers)
}