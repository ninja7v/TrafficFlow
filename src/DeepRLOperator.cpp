// Libraries
#include <algorithm>
// Headers
#include "../headers/DeepRLOperator.h"
#include "../headers/Constants.h"


DeepRLOperator::DeepRLOperator() {
    std::random_device rd;
    rng.seed(rd());
}

int DeepRLOperator::decide(const std::vector<int>& state,
                           const std::vector<int>& availableActions) {
    if (availableActions.empty()) return -1;

    // Lazy initialization of the neural network
    // We wait for the first state to know the input size (state dimension)
    if (!isInitialized) {
        const int inputSize = static_cast<int>(state.size());
        const int outputSize = constants::maxConnectedInputRoads;
        // Topology: Input Layer -> 64 Hidden -> 64 Hidden -> Output Layer
        policyNetwork = NeuralNetwork({inputSize, 64, 64, outputSize}, alpha);
        targetNetwork = policyNetwork;
        isInitialized = true;
    }

    // Exploration (epsilon-greedy strategy)
    // With probability 'epsilon', we choose a random action to explore the environment.
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng) < epsilon) {
        std::uniform_int_distribution<size_t> actionDist(0, availableActions.size() - 1);
        return availableActions[actionDist(rng)];
    }

    // Exploitation 
    // Otherwise, we ask the Neural Network to predict Q-values for all actions
    // and we choose the action with the highest Q-value.
    std::vector<double> inputState(state.begin(), state.end());
    std::vector<double> qValues = policyNetwork.predict(inputState);

    int bestAction = availableActions[0];
    double maxQ = -1e9;

    for (int action : availableActions) {
        if (action >= 0 && action < static_cast<int>(qValues.size())) {
            if (qValues[action] > maxQ) {
                maxQ = qValues[action];
                bestAction = action;
            }
        }
    }
    return bestAction;
}

void DeepRLOperator::learn(const std::vector<int>& state,
                           const int action,
                           const double reward,
                           const std::vector<int>& nextState,
                           const std::vector<int>& availableActions) {
    if (!isInitialized) return;

    std::vector<double> s(state.begin(), state.end());
    std::vector<double> ns(nextState.begin(), nextState.end());
    
    // 1. Store experience in Replay Buffer
    // We store the transition (s, a, r, s') to learn from it later.
    // This breaks temporal correlations between consecutive samples.
    if (replayBuffer.size() >= maxBufferSize) {
        replayBuffer.erase(replayBuffer.begin());
    }
    replayBuffer.push_back({s, action, reward, ns, availableActions});

    stepCount++;
    if (stepCount % trainingFrequency != 0) return;

    if (replayBuffer.size() < batchSize) return;

    // 2. Train on a random batch of experiences
    for (size_t i = 0; i < batchSize; ++i) {
        std::uniform_int_distribution<size_t> dist(0, replayBuffer.size() - 1);
        const auto& exp = replayBuffer[dist(rng)];

        std::vector<double> targetQ = policyNetwork.predict(exp.state);
        std::vector<double> nextQ = targetNetwork.predict(exp.nextState);

        double maxNextQ = -1e9;
        bool foundNext = false;
        
        for (int nextAction : exp.nextActions) {
             if (nextAction >= 0 && nextAction < static_cast<int>(nextQ.size())) {
                 if (nextQ[nextAction] > maxNextQ) {
                     maxNextQ = nextQ[nextAction];
                     foundNext = true;
                 }
             }
        }
        if (!foundNext) maxNextQ = 0.0;
        
        // Update the Q-value for the specific action taken
        if (exp.action >= 0 && exp.action < static_cast<int>(targetQ.size())) {
            // Bellman Equation: Target = Reward + Gamma * max(Q(next_state))
            // We want the current Q-value to move towards this target.
            targetQ[exp.action] = exp.reward + gamma * maxNextQ;
            // Backpropagation: Train the network to predict this new target Q-value
            policyNetwork.train(exp.state, targetQ);
        }
    }

    // 3. Decay Epsilon
    // Reduce exploration rate over time to favor exploitation as the agent learns.
    if (epsilon > epsilonMin) {
        epsilon = std::max(epsilonMin, epsilon * epsilonDecay);
    }

    if (stepCount % targetUpdateFrequency == 0) {
        targetNetwork = policyNetwork;
    }
}
