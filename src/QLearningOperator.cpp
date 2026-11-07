// Libraries
#include <algorithm> // To use std::max
#include <random>    // To use std::random_device and std::mt19937
// Headers
#include "../headers/QLearningOperator.h"

QLearningOperator::QLearningOperator() {
    std::random_device rd;
    rng.seed(rd());
}

int QLearningOperator::decide(const std::vector<int>& state, const std::vector<int>& availableActions) {
    if (availableActions.empty()) return -1;

    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng) < epsilon) {
        std::uniform_int_distribution<size_t> actionDist(0, availableActions.size() - 1);
        return availableActions[actionDist(rng)];
    }

    if (!qTable.contains(state)) {
        std::uniform_int_distribution<size_t> actionDist(0, availableActions.size() - 1);
        return availableActions[actionDist(rng)];
    }

    int bestAction = availableActions[0];
    double maxQ = -1e9;
    auto& actionQValues = qTable[state];
    for (int action : availableActions) {
        double q = 0.0;
        if (actionQValues.count(action)) {
            q = actionQValues[action];
        }
        if (q > maxQ) {
            maxQ = q;
            bestAction = action;
        }
    }
    return bestAction;
}

void QLearningOperator::learn(const std::vector<int>& state, int action, double reward, const std::vector<int>& nextState, const std::vector<int>& availableActions) {
    const double currentQ = (qTable[state].count(action)) ? qTable[state][action] : 0.0;

    double maxNextQ = 0.0;
    if (!availableActions.empty()) {
        if (qTable.count(nextState)) {
             auto& nextActionQValues = qTable[nextState];
             double localMax = -1e9;
             bool found = false;
             for (int nextAction : availableActions) {
                 const double q = (nextActionQValues.count(nextAction)) ? nextActionQValues[nextAction] : 0.0;
                 if (q > localMax) {
                     localMax = q;
                     found = true;
                 }
             }
             if (found) maxNextQ = localMax;
        }
    }

    // new Q-value
    qTable[state][action] = currentQ + alpha * (reward + gamma * maxNextQ - currentQ);

    if (epsilon > epsilonMin) {
        epsilon = std::max(epsilonMin, epsilon * epsilonDecay);
    }
}
