// Libraries
#include <algorithm> // To use std::max
#include <fstream>   // To use files
#include <iomanip>   // To use std::setprecision
// Headers
#include "../headers/Features.h"
#include "../headers/QLearningOperator.h"
#include "../headers/Random.h"

QLearningOperator::QLearningOperator(const unsigned int seed)
    : qTable(features::tableSize, 0.0),
      visits(features::tableSize, 0),
      rng(seed) {
}

int QLearningOperator::bestAction(const Observation& observation) const {
   // A Q-value that was never updated says nothing: such a decision is only taken when no
   // decision is known, and then the first one is taken (the green light is kept if useful).
   const std::vector<int> actions = features::usefulActions(observation);
   int best = actions.front();
   double maxQ = 0.0;
   bool found = false;
   for (const int action : actions) {
      const int key = features::discretize(observation, action);
      if (visits[key] > 0 && (!found || qTable[key] > maxQ)) {
         maxQ = qTable[key];
         best = action;
         found = true;
      }
   }
   return best;
}

int QLearningOperator::decide(const Observation& observation) {
   if (training && rnd::uniform(rng) < epsilon) {
      const std::vector<int> actions = features::usefulActions(observation);
      return actions[rnd::index(rng, static_cast<int>(actions.size()))];
   }
   return bestAction(observation);
}

void QLearningOperator::learn(const Observation& observation,
                              const int action,
                              const double reward,
                              const Observation& nextObservation) {
   if (!training)
      return;
   const int key = features::discretize(observation, action);
   // Value of the next situation (0 while nothing is known about it)
   const double maxNextQ =
       qTable[features::discretize(nextObservation, bestAction(nextObservation))];
   // The first updates of a Q-value average what was seen, the next ones follow slow changes
   const double alpha = std::max(alphaMin, 1.0 / ++visits[key]);
   qTable[key] += alpha * (reward + gamma * maxNextQ - qTable[key]);
   epsilon = std::max(epsilonMin, epsilon * epsilonDecay);
}

bool QLearningOperator::isLearning() const {
   return training;
}

void QLearningOperator::setTraining(const bool enabled) {
   training = enabled;
}

void QLearningOperator::setExploration(const double eps, const double minimum, const double decay) {
   epsilon = eps;
   epsilonMin = minimum;
   epsilonDecay = decay;
}

double QLearningOperator::getEpsilon() const {
   return epsilon;
}

bool QLearningOperator::save(const std::string& path) const {
   std::ofstream out(path);
   if (!out)
      return false;
   out << qTable.size() << '\n' << std::setprecision(17);
   for (size_t k = 0; k < qTable.size(); ++k)
      out << qTable[k] << ' ' << visits[k] << '\n';
   return static_cast<bool>(out);
}

bool QLearningOperator::load(const std::string& path) {
   std::ifstream in(path);
   size_t size = 0;
   if (!in || !(in >> size) || size != qTable.size())
      return false;
   std::vector<double> values(size);
   std::vector<int> counts(size);
   for (size_t k = 0; k < size; ++k)
      if (!(in >> values[k] >> counts[k]))
         return false;
   qTable = std::move(values);
   visits = std::move(counts);
   return true;
}
