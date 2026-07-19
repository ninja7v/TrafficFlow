/**
 * @brief Deep Reinforcement Learning Operator (DQN Agent).
 */
 
#pragma once
// Libraries
#include <vector> // To use vectors
#include <random> // To use std::mt19937
// Headers
#include "IntersectionOperator.h"
#include "NeuralNetwork.h"

/**
 * @brief DeepRLOperator implements a Deep Q-Network (DQN) agent.
 * It uses a neural network to approximate the Q-value function Q(s, a),
 * which estimates the expected future reward for taking action 'a' in state 's'.
 */
class DeepRLOperator : public IntersectionOperator {
public:
    /**
     * @brief Default constructor.
     */
    DeepRLOperator();
    /**
     * @brief Decide the next action to take.
     * @param state Current state of the environment.
     * @param availableActions List of available actions.
     * @return int Chosen action.
     */
    int decide(const std::vector<int>& state,
               const std::vector<int>& availableActions) override;
    /**
     * @brief Learn from the environment.
     * @param state Current state of the environment.
     * @param action Chosen action.
     * @param reward Reward received.
     * @param nextState Resulting state.
     * @param availableActions List of available actions.
     */
    void learn(const std::vector<int>& state,
               const int action,
               const double reward,
               const std::vector<int>& nextState,
               const std::vector<int>& availableActions) override;

private:
    NeuralNetwork policyNetwork;
    NeuralNetwork targetNetwork;
    bool isInitialized = false;

    // Structure to store a single transition (experience)
    struct Experience {
        /** Current traffic state */
        std::vector<double> state;
        /** Traffic light phase chosen */
        int action;
        /** Reward received (negative waiting time) */
        double reward;
        /** Resulting state */
        std::vector<double> nextState;
        /** Actions valid in the resulting state */
        std::vector<int> nextActions;
    };

    /** Memory to store past experiences */
    std::vector<Experience> replayBuffer;
    /** Maximum number of experiences to keep */
    const size_t maxBufferSize = 10000;
    /** Number of experiences to train on at each step */
    const size_t batchSize = 32;
    /** Train the network every N steps */
    const int trainingFrequency = 1;
    /** Copy policy weights to the stable bootstrap network every N transitions. */
    const int targetUpdateFrequency = 250;
    int stepCount = 0;

    // Hyperparameters
    /** Learning Rate: How fast the network updates its weights */
    static constexpr inline double alpha = 0.001;
    /** Discount Factor: Importance of future rewards (0 = short-sighted, 1 = long-sighted) */
    static constexpr inline double gamma = 0.95;
    /** Exploration Rate: Probability of choosing a random action */
    double epsilon = 1.0;
    /** Minimum Exploration Rate: The agent will always explore a little bit */
    static constexpr inline double epsilonMin = 0.01;
    /** Decay Rate: How fast epsilon decreases (multiplied at each step) */
    static constexpr inline double epsilonDecay = 0.995;
    /** Random number generator */
    std::mt19937 rng;
};
