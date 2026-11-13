// Libraries
#include <algorithm> // To use std::clamp
#include <cmath>     // To use std::sqrt and std::pow
#include <iomanip>   // To use std::setprecision
#include <istream>   // To use std::istream
#include <ostream>   // To use std::ostream
#include <random>    // To use std::mt19937
// Headers
#include "../headers/NeuralNetwork.h"
#include "../headers/Random.h"

namespace {
   constexpr double leak = 0.01; // Leaky ReLU: avoids dead neurons

   double activation(const double x) {
      return x > 0 ? x : leak * x;
   }

   double activationDerivative(const double y) { // From the output: same sign as the input
      return y > 0 ? 1.0 : leak;
   }
}

Layer::Layer(const int in, const int out)
    : inputSize(in),
      outputSize(out),
      weights(static_cast<size_t>(in) * out, 0.0),
      biases(out, 0.0),
      weightGradients(weights.size(), 0.0),
      biasGradients(out, 0.0),
      weightMean(weights.size(), 0.0),
      weightVariance(weights.size(), 0.0),
      biasMean(out, 0.0),
      biasVariance(out, 0.0) {
}

NeuralNetwork::NeuralNetwork(const std::vector<int>& topology, const double lr, const unsigned int seed)
    : learningRate(lr) {
   std::mt19937 rng(seed);
   for (size_t i = 0; i + 1 < topology.size(); ++i) {
      layers.emplace_back(topology[i], topology[i + 1]);
      // He initialization (uniform), suited to ReLU
      const double scale = std::sqrt(6.0 / topology[i]);
      for (double& w : layers.back().weights)
         w = (rnd::uniform(rng) * 2.0 - 1.0) * scale;
   }
}

void NeuralNetwork::forward(const std::vector<double>& input,
                            std::vector<std::vector<double>>& activations) const {
   activations.assign(layers.size() + 1, {});
   activations[0] = input;
   for (size_t i = 0; i < layers.size(); ++i) {
      const Layer& layer = layers[i];
      const std::vector<double>& in = activations[i];
      std::vector<double>& out = activations[i + 1];
      out.resize(layer.outputSize);
      const bool isOutputLayer = i + 1 == layers.size();
      for (int n = 0; n < layer.outputSize; ++n) {
         const double* w = &layer.weights[static_cast<size_t>(n) * layer.inputSize];
         double sum = layer.biases[n];
         for (int k = 0; k < layer.inputSize; ++k)
            sum += w[k] * in[k];
         out[n] = isOutputLayer ? sum : activation(sum);
      }
   }
}

std::vector<double> NeuralNetwork::predict(const std::vector<double>& input) const {
   std::vector<std::vector<double>> activations;
   forward(input, activations);
   return activations.back();
}

void NeuralNetwork::accumulateGradient(const std::vector<double>& input,
                                       const std::vector<double>& target) {
   std::vector<std::vector<double>> activations;
   forward(input, activations);

   // Derivative of the Huber loss: the error, clipped to [-1, 1]
   std::vector<double> errors(activations.back().size());
   for (size_t i = 0; i < errors.size(); ++i)
      errors[i] = std::clamp(activations.back()[i] - target[i], -1.0, 1.0);

   // Backpropagate
   for (int i = static_cast<int>(layers.size()) - 1; i >= 0; --i) {
      Layer& layer = layers[i];
      const std::vector<double>& in = activations[i];
      const std::vector<double>& out = activations[i + 1];
      const bool isOutputLayer = i + 1 == static_cast<int>(layers.size());
      std::vector<double> previousErrors(layer.inputSize, 0.0);
      for (int n = 0; n < layer.outputSize; ++n) {
         const double delta = errors[n] * (isOutputLayer ? 1.0 : activationDerivative(out[n]));
         const size_t row = static_cast<size_t>(n) * layer.inputSize;
         for (int k = 0; k < layer.inputSize; ++k) {
            previousErrors[k] += delta * layer.weights[row + k];
            layer.weightGradients[row + k] += delta * in[k];
         }
         layer.biasGradients[n] += delta;
      }
      errors = std::move(previousErrors);
   }
   accumulatedSamples++;
}

void NeuralNetwork::applyGradients() {
   if (accumulatedSamples == 0)
      return;
   constexpr double beta1 = 0.9, beta2 = 0.999, epsilon = 1e-8;
   updates++;
   const double stepSize = learningRate * std::sqrt(1.0 - std::pow(beta2, updates)) /
                           (1.0 - std::pow(beta1, updates));
   auto adam = [&](std::vector<double>& values, std::vector<double>& gradients,
                   std::vector<double>& mean, std::vector<double>& variance) {
      for (size_t k = 0; k < values.size(); ++k) {
         const double g = gradients[k] / accumulatedSamples;
         mean[k] = beta1 * mean[k] + (1.0 - beta1) * g;
         variance[k] = beta2 * variance[k] + (1.0 - beta2) * g * g;
         values[k] -= stepSize * mean[k] / (std::sqrt(variance[k]) + epsilon);
         gradients[k] = 0.0;
      }
   };
   for (Layer& layer : layers) {
      adam(layer.weights, layer.weightGradients, layer.weightMean, layer.weightVariance);
      adam(layer.biases, layer.biasGradients, layer.biasMean, layer.biasVariance);
   }
   accumulatedSamples = 0;
}

void NeuralNetwork::train(const std::vector<double>& input, const std::vector<double>& target) {
   accumulateGradient(input, target);
   applyGradients();
}

std::vector<int> NeuralNetwork::getTopology() const {
   std::vector<int> topology;
   for (const Layer& layer : layers) {
      if (topology.empty())
         topology.push_back(layer.inputSize);
      topology.push_back(layer.outputSize);
   }
   return topology;
}

void NeuralNetwork::save(std::ostream& out) const {
   const std::vector<int> topology = getTopology();
   out << topology.size();
   for (const int n : topology)
      out << ' ' << n;
   out << '\n' << std::setprecision(17);
   for (const Layer& layer : layers) {
      for (const double w : layer.weights)
         out << w << ' ';
      out << '\n';
      for (const double b : layer.biases)
         out << b << ' ';
      out << '\n';
   }
}

bool NeuralNetwork::load(std::istream& in) {
   size_t size = 0;
   constexpr size_t maxLayers = 16;
   constexpr int maxNeurons = 4096;
   if (!(in >> size) || size < 2 || size > maxLayers)
      return false;
   std::vector<int> topology(size);
   for (int& n : topology)
      if (!(in >> n) || n < 1 || n > maxNeurons)
         return false;
   NeuralNetwork network(topology, learningRate);
   for (Layer& layer : network.layers) {
      for (double& w : layer.weights)
         if (!(in >> w))
            return false;
      for (double& b : layer.biases)
         if (!(in >> b))
            return false;
   }
   *this = std::move(network);
   return true;
}
