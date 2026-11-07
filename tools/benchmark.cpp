// Headless benchmark: flow (completed trips per simulated minute) over time, per controller.
// Usage: benchmark <fixed|random|heuristic|qlearning|dqn> [seed=1] [minutes=60] [window=5]
// Prints, for each window: trips per minute (average number of vehicles in the network).
// The same seed gives the same network and traffic demand, so controllers can be compared.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../headers/Constants.h"
#include "../headers/IntersectionOperator.h"
#include "../headers/Network.h"

/** Baseline: next road in turn every period. */
struct FixedCycle : IntersectionOperator {
   int decide(const std::vector<int>& s, const std::vector<int>& a) override {
      return a.empty() ? -1 : (s[0] + 1) % static_cast<int>(a.size());
   }
   void learn(const std::vector<int>&, int, double, const std::vector<int>&, const std::vector<int>&) override {}
};

/** Baseline: random road every period. */
struct RandomOp : IntersectionOperator {
   std::mt19937 rng{123};
   int decide(const std::vector<int>&, const std::vector<int>& a) override {
      if (a.empty()) return -1;
      return a[std::uniform_int_distribution<size_t>(0, a.size() - 1)(rng)];
   }
   void learn(const std::vector<int>&, int, double, const std::vector<int>&, const std::vector<int>&) override {}
};

int main(int argc, char** argv) {
   const std::string method = argc > 1 ? argv[1] : "heuristic";
   const unsigned seed = argc > 2 ? static_cast<unsigned>(atoi(argv[2])) : 1u;
   const int minutes = argc > 3 ? atoi(argv[3]) : 60;
   const int window = argc > 4 ? atoi(argv[4]) : 5;

   constants::boost = 1.0;
   constants::updateBoostDependentConstants();
   constants::learningType = method == "heuristic" ? LearningType::HEURISTIC
                             : method == "dqn"     ? LearningType::DQN
                                                   : LearningType::Q_LEARNING;
   Network n(seed);
   if (method == "fixed") n.setOperator(std::make_shared<FixedCycle>());
   if (method == "random") n.setOperator(std::make_shared<RandomOp>());

   const int stepsPerWindow = static_cast<int>(window * 60 / Network::timeStep);
   int lastCompleted = 0;
   printf("%s seed=%u:", method.c_str(), seed);
   for (int w = 0; w < minutes / window; ++w) {
      long vehicles = 0;
      for (int k = 0; k < stepsPerWindow; ++k) {
         n.step();
         vehicles += n.getVehicleCount();
      }
      const int done = n.getCompletedVehicles() - lastCompleted;
      lastCompleted = n.getCompletedVehicles();
      printf(" %5.1f(%2ld)", done / static_cast<double>(window), vehicles / stepsPerWindow);
      fflush(stdout);
   }
   printf("\n");
}
