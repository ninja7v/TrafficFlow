// Headless training of the learning controllers.
// Usage: train <dqn|qlearning> [--episodes 300] [--minutes 10] [--out path] [--seed 1]
//                               [--validate-every 10] [--log path]
// An episode is one random network with a random demand, simulated for --minutes with the
// controller learning and exploring. Every --validate-every episodes, the controller is
// evaluated without exploration on validation networks, and the best model is written to --out.
// Training networks (seeds 100000+), validation networks (seeds 50000+) and the networks of the
// benchmark (seeds 1+) are different.
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "../headers/DeepRLOperator.h"
#include "../headers/QLearningOperator.h"
#include "../headers/Random.h"
#include "Experiment.h"

namespace {
   constexpr unsigned int validationSeed = 50000, trainingSeed = 100000;
   constexpr int validationNetworks = 4;

   const char* const validationScenarios[] = {"light", "medium", "heavy", "saturated", "overload"};
   /** A gridlocked run would hide all the others in the average. */
   constexpr double maxValidationRatio = 3.0;
   constexpr double validationWarmup = 2.0, validationMinutes = 8.0;

   /** Time spent per vehicle with the hand-written heuristic, for each validation run. */
   std::vector<double> validationReference() {
      std::vector<double> reference;
      for (const char* name : validationScenarios)
         for (int k = 0; k < validationNetworks; ++k)
            reference.push_back(run(std::make_shared<HeuristicOperator>(), validationSeed + k,
                                    *findScenario(name), validationWarmup, validationMinutes)
                                    .timeSpentPerVehicle());
      return reference;
   }

   /** Time spent per vehicle on the validation networks, without exploration, relative to the
    * heuristic (below 1: better than the heuristic). Each run weighs the same. */
   template <typename Operator>
   double validate(const std::shared_ptr<Operator>& op, const std::vector<double>& reference) {
      op->setTraining(false);
      double total = 0.0;
      size_t runs = 0;
      for (const char* name : validationScenarios) {
         for (int k = 0; k < validationNetworks; ++k) {
            const TrafficStats stats = run(op, validationSeed + k, *findScenario(name),
                                           validationWarmup, validationMinutes);
            total += std::min(stats.timeSpentPerVehicle() / reference[runs++], maxValidationRatio);
         }
      }
      op->setTraining(true);
      return total / static_cast<double>(runs);
   }

   template <typename Operator>
   int train(const int episodes, const double minutes, const int validateEvery,
             const unsigned int seed, const std::string& out, const std::string& logPath) {
      auto op = std::make_shared<Operator>(seed);
      std::mt19937 rng(seed);
      std::ofstream log;
      if (!logPath.empty()) {
         log.open(logPath);
         log << "episode,epsilon,validation_score\n";
      }
      const std::vector<double> reference = validationReference();
      // Exploration: from 1 to 0.05 over the first 60 % of the training
      const double decisionsPerMinute = 60.0 / constants::trafficLightPeriod * constants::nbIntersections;
      const double decaySteps = 0.6 * episodes * minutes * decisionsPerMinute;
      op->setExploration(1.0, 0.05, std::pow(0.05, 1.0 / decaySteps));
      double best = 1e18;
      for (int episode = 1; episode <= episodes; ++episode) {
         // Random traffic conditions: from light traffic to more than the network can take
         const bool saturated = rnd::uniform(rng) < 0.3;
         const Scenario scenario{"training", saturated ? 600.0 : 40.0 + 260.0 * rnd::uniform(rng),
                                 saturated ? 50 + rnd::index(rng, 151) : 150};
         run(op, trainingSeed + seed * 1000 + episode, scenario, 0.0, minutes, true);
         if (episode % validateEvery == 0 || episode == episodes) {
            const double score = validate(op, reference);
            const bool improved = score < best;
            if (improved) {
               best = score;
               if (!op->save(out)) {
                  fprintf(stderr, "Cannot write %s\n", out.c_str());
                  return 1;
               }
            }
            printf("episode %4d  epsilon %.3f  time spent / heuristic %.3f%s\n", episode,
                   op->getEpsilon(), score, improved ? "  (saved)" : "");
            fflush(stdout);
            if (log.is_open())
               log << episode << ',' << op->getEpsilon() << ',' << score << std::endl;
         }
      }
      printf("Best time spent / heuristic: %.3f, model written to %s\n", best, out.c_str());
      return 0;
   }
}

int main(int argc, char** argv) {
   if (argc < 2) {
      fprintf(stderr, "Usage: train <dqn|qlearning> [--episodes N] [--minutes M] [--out path] "
                      "[--seed S] [--validate-every K] [--log path]\n");
      return 1;
   }
   const std::string method = argv[1];
   int episodes = 300, validateEvery = 10;
   double minutes = 10.0;
   unsigned int seed = 1;
   std::string out, logPath;
   for (int i = 2; i + 1 < argc; i += 2) {
      const std::string option = argv[i], value = argv[i + 1];
      if (option == "--episodes")
         episodes = atoi(value.c_str());
      else if (option == "--minutes")
         minutes = atof(value.c_str());
      else if (option == "--out")
         out = value;
      else if (option == "--seed")
         seed = static_cast<unsigned int>(atoi(value.c_str()));
      else if (option == "--validate-every")
         validateEvery = atoi(value.c_str());
      else if (option == "--log")
         logPath = value;
      else {
         fprintf(stderr, "Unknown option: %s\n", option.c_str());
         return 1;
      }
   }
   if (method == "dqn")
      return train<DeepRLOperator>(episodes, minutes, validateEvery, seed,
                                   out.empty() ? DeepRLOperator::defaultModelPath : out, logPath);
   if (method == "qlearning")
      return train<QLearningOperator>(episodes, minutes, validateEvery, seed,
                                      out.empty() ? QLearningOperator::defaultModelPath : out, logPath);
   fprintf(stderr, "Unknown method: %s\n", method.c_str());
   return 1;
}
