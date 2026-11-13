/**
 * @brief Implementation of a neural network (more precisely a MultiLayer Perceptron - MLP) to approximate Q-values.
 */

#pragma once
// Libraries
#include <iosfwd> // To use std::istream and std::ostream
#include <vector> // To use std::vector

/** @brief Parameters of a layer, with what the Adam optimizer needs to update them. */
struct Layer {
   int inputSize = 0;
   int outputSize = 0;
   /** Weights, row by row: weights[n * inputSize + w] links input w to neuron n. */
   std::vector<double> weights, biases;
   /** Gradients accumulated since the last update. */
   std::vector<double> weightGradients, biasGradients;
   /** Adam moments (mean and uncentered variance of the gradients). */
   std::vector<double> weightMean, weightVariance, biasMean, biasVariance;

   Layer(int in, int out);
};

class NeuralNetwork {
 public:
   /** @brief Default constructor (empty network). */
   NeuralNetwork() = default;
   /** @brief Constructor. Hidden layers use a leaky ReLU, the output layer is linear.
    * @param topology Vector of integers representing the number of neurons in each layer.
    * @param learningRate Learning rate for the network.
    * @param seed Random seed for the initial weights. */
   explicit NeuralNetwork(const std::vector<int>& topology,
                          double learningRate = 0.001,
                          unsigned int seed = 0);
   /** @brief Make a prediction.
    * @param input Input vector.
    * @return std::vector<double> Output vector. */
   std::vector<double> predict(const std::vector<double>& input) const;
   /** @brief Add the gradient of the error on one sample (Huber loss: squared error for small
    * errors, absolute error for large ones, so that an outlier cannot destabilize the network).
    * @param input Input vector.
    * @param target Target vector. */
   void accumulateGradient(const std::vector<double>& input, const std::vector<double>& target);
   /** @brief Update the weights with the average of the accumulated gradients (Adam optimizer). */
   void applyGradients();
   /** @brief Train the network on one sample.
    * @param input Input vector.
    * @param target Target vector. */
   void train(const std::vector<double>& input, const std::vector<double>& target);
   /** @brief Write the topology and the weights as text. */
   void save(std::ostream& out) const;
   /** @brief Read a network written by save().
    * @returns Success? The network is unchanged otherwise. */
   bool load(std::istream& in);
   /** @brief Getter.
    * @returns Number of neurons in each layer (empty for an empty network). */
   std::vector<int> getTopology() const;

 private:
   /** @brief Forward pass keeping the output of every layer (activations[0] is the input). */
   void forward(const std::vector<double>& input, std::vector<std::vector<double>>& activations) const;
   /** @brief Vector of layers. */
   std::vector<Layer> layers;
   /** @brief Learning rate. */
   double learningRate = 0.001;
   /** @brief Number of samples accumulated since the last update. */
   int accumulatedSamples = 0;
   /** @brief Number of updates (Adam bias correction). */
   long updates = 0;
};
