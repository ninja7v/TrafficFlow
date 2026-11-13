// Libraries
#include <algorithm> // To use max
#include <string>    // To use strings
// Headers
#include "../headers/Controllers.h"
#include "../headers/DeepRLOperator.h"
#include "../headers/QLearningOperator.h"
#include "../headers/Random.h"

RandomOperator::RandomOperator(const unsigned int seed) : rng(seed) {
}

int RandomOperator::decide(const Observation& observation) {
   return rnd::index(rng, static_cast<int>(observation.approaches.size()));
}

FixedCycleOperator::FixedCycleOperator(const double greenTime) : greenTime(greenTime) {
}

int FixedCycleOperator::decide(const Observation& observation) {
   constexpr double tolerance = 1e-6;
   if (observation.greenDuration + tolerance < greenTime)
      return observation.currentGreen;
   return (observation.currentGreen + 1) % static_cast<int>(observation.approaches.size());
}

int HeuristicOperator::decide(const Observation& observation) {
   // Keep the current light green while its first vehicle is approaching and can go through.
   // A blocked vehicle must not hold the green: that starves the other roads (gridlock).
   const ApproachObservation& green = observation.approaches[observation.currentGreen];
   constexpr double approachingDistance = 2.0; // Grid units
   if (green.canDischarge && green.headDistance < approachingDistance)
      return observation.currentGreen;

   // Otherwise, among the roads whose traffic can go through: a full road first, then the
   // road with the most vehicles.
   int chosenRoad = observation.currentGreen;
   int bestScore = 0;
   for (size_t i = 0; i < observation.approaches.size(); ++i) {
      const ApproachObservation& a = observation.approaches[i];
      if (!a.canDischarge)
         continue;
      const int score = a.vehicles + (a.full ? 1000 : 0);
      if (score > bestScore) {
         bestScore = score;
         chosenRoad = static_cast<int>(i);
      }
   }
   return chosenRoad;
}

int MaxPressureOperator::decide(const Observation& observation) {
   // Pressure of a road: its vehicles minus the vehicles on the roads they are going to.
   // A road whose first vehicle is blocked has nothing to gain from a green light.
   // The margin avoids switching (and losing time restarting vehicles) for a small gain.
   constexpr double switchMargin = 1.0;
   auto pressure = [&](const int i) {
      const ApproachObservation& a = observation.approaches[i];
      return a.canDischarge ? a.vehicles - a.downstreamVehicles : -1e9;
   };
   int chosenRoad = observation.currentGreen;
   double bestPressure = pressure(chosenRoad) + switchMargin;
   for (int i = 0; i < static_cast<int>(observation.approaches.size()); ++i) {
      const double p = pressure(i);
      if (p > bestPressure) {
         bestPressure = p;
         chosenRoad = i;
      }
   }
   return chosenRoad;
}

namespace {
   /** Load a trained model, looked for from the working directory and from its parent (build folder). */
   template <typename Operator>
   std::shared_ptr<IntersectionOperator> makeLearningOperator(const unsigned int seed) {
      auto op = std::make_shared<Operator>(seed);
      const std::string path = Operator::defaultModelPath;
      op->setTraining(!op->load(path) && !op->load("../" + path));
      return op;
   }
}

std::shared_ptr<IntersectionOperator> makeOperator(const ControllerType type,
                                                   const unsigned int seed) {
   switch (type) {
   case ControllerType::RANDOM:
      return std::make_shared<RandomOperator>(seed);
   case ControllerType::FIXED_CYCLE:
      return std::make_shared<FixedCycleOperator>();
   case ControllerType::HEURISTIC:
      return std::make_shared<HeuristicOperator>();
   case ControllerType::MAX_PRESSURE:
      return std::make_shared<MaxPressureOperator>();
   case ControllerType::Q_LEARNING:
      return makeLearningOperator<QLearningOperator>(seed);
   case ControllerType::DQN:
      return makeLearningOperator<DeepRLOperator>(seed);
   }
   return nullptr;
}

const char* controllerName(const ControllerType type) {
   switch (type) {
   case ControllerType::RANDOM:
      return "random";
   case ControllerType::FIXED_CYCLE:
      return "fixed";
   case ControllerType::HEURISTIC:
      return "heuristic";
   case ControllerType::MAX_PRESSURE:
      return "maxpressure";
   case ControllerType::Q_LEARNING:
      return "qlearning";
   case ControllerType::DQN:
      return "dqn";
   }
   return "";
}
