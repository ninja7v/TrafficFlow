// Libraries
#include <algorithm> // To use std::max
#include <fstream>   // To use files
#include <utility>   // To use std::move
// Headers
#include "../headers/DeepRLOperator.h"
#include "../headers/Features.h"
#include "../headers/Random.h"

namespace {
   /** Description of each useful decision, in the order of features::usefulActions(). */
   std::vector<std::vector<double>> describeDecisions(const Observation& observation,
                                                      const std::vector<int>& actions) {
      std::vector<std::vector<double>> decisions;
      decisions.reserve(actions.size());
      for (const int action : actions)
         decisions.push_back(features::describe(observation, action));
      return decisions;
   }
}

DeepRLOperator::DeepRLOperator(const unsigned int seed)
    : policyNetwork({features::count, hiddenSize, hiddenSize, 1}, alpha, seed),
      targetNetwork(policyNetwork),
      rng(seed + 1) {
}

size_t DeepRLOperator::bestDecision(const NeuralNetwork& network,
                                    const std::vector<std::vector<double>>& decisions) {
   size_t best = 0;
   double maxQ = network.predict(decisions[0])[0];
   for (size_t k = 1; k < decisions.size(); ++k) {
      const double q = network.predict(decisions[k])[0];
      if (q > maxQ) {
         maxQ = q;
         best = k;
      }
   }
   return best;
}

int DeepRLOperator::decide(const Observation& observation) {
   const std::vector<int> actions = features::usefulActions(observation);
   // Exploration (epsilon-greedy strategy)
   // With probability 'epsilon', we choose a random action to explore the environment.
   if (training && rnd::uniform(rng) < epsilon)
      return actions[rnd::index(rng, static_cast<int>(actions.size()))];
   // Exploitation
   // Otherwise, we choose the action with the highest Q-value.
   return actions[bestDecision(policyNetwork, describeDecisions(observation, actions))];
}

void DeepRLOperator::learn(const Observation& observation,
                           const int action,
                           const double reward,
                           const Observation& nextObservation) {
   if (!training)
      return;

   // 1. Store experience in Replay Buffer
   // We store the transition (s, a, r, s') to learn from it later.
   // This breaks temporal correlations between consecutive samples.
   Experience experience{
       features::describe(observation, action), reward,
       describeDecisions(nextObservation, features::usefulActions(nextObservation))};
   if (replayBuffer.size() < maxBufferSize) {
      replayBuffer.push_back(std::move(experience));
   } else {
      replayBuffer[replayBufferNext] = std::move(experience);
      replayBufferNext = (replayBufferNext + 1) % maxBufferSize;
   }
   if (replayBuffer.size() < minBufferSize)
      return;

   // 2. Train on a random batch of experiences
   for (size_t i = 0; i < batchSize; ++i) {
      const Experience& e = replayBuffer[rnd::index(rng, static_cast<int>(replayBuffer.size()))];
      // Bellman Equation: Target = Reward + Gamma * max(Q(next_state))
      // Double DQN: the policy network chooses the next action, the target network gives its
      // value. Taking both from the same network overestimates the Q-values.
      const size_t next = bestDecision(policyNetwork, e.nextDecisions);
      const double nextQ = targetNetwork.predict(e.nextDecisions[next])[0];
      policyNetwork.accumulateGradient(e.decision, {e.reward + gamma * nextQ});
   }
   policyNetwork.applyGradients();

   // 3. Decay Epsilon
   // Reduce exploration rate over time to favor exploitation as the agent learns.
   epsilon = std::max(epsilonMin, epsilon * epsilonDecay);

   if (++stepCount % targetUpdateFrequency == 0)
      targetNetwork = policyNetwork;
}

bool DeepRLOperator::isLearning() const {
   return training;
}

void DeepRLOperator::setTraining(const bool enabled) {
   training = enabled;
}

void DeepRLOperator::setExploration(const double eps, const double minimum, const double decay) {
   epsilon = eps;
   epsilonMin = minimum;
   epsilonDecay = decay;
}

double DeepRLOperator::getEpsilon() const {
   return epsilon;
}

bool DeepRLOperator::save(const std::string& path) const {
   std::ofstream out(path);
   if (!out)
      return false;
   policyNetwork.save(out);
   return static_cast<bool>(out);
}

bool DeepRLOperator::load(const std::string& path) {
   std::ifstream in(path);
   NeuralNetwork network({features::count, hiddenSize, hiddenSize, 1}, alpha);
   if (!in || !network.load(in))
      return false;
   const std::vector<int> topology = network.getTopology();
   if (topology.front() != features::count || topology.back() != 1)
      return false;
   policyNetwork = network;
   targetNetwork = std::move(network);
   return true;
}
