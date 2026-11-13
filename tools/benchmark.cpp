// Headless benchmark: compares the traffic light controllers on the same networks and demand.
// Usage: benchmark [--controllers random,fixed,heuristic,maxpressure,qlearning,dqn]
//                  [--scenarios light,medium,heavy,saturated,overload] [--demand D --max-vehicles N]
//                  [--seeds 20] [--first-seed 1] [--minutes 20] [--warmup 5] [--csv results.csv]
//                  [--dqn-model path] [--qlearning-model path]
//                  [--one-way-ratio 0.3] [--light-period 2]
// Each seed is a different random network. A seed and a scenario give the same network and the
// same demand to every controller. The learning controllers are not trained here: they apply
// the models written by the train tool.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "../headers/DeepRLOperator.h"
#include "../headers/QLearningOperator.h"
#include "Experiment.h"

namespace {
   struct Metric {
      const char* name;
      const char* unit;
      bool higherIsBetter;
      double (*get)(const TrafficStats&);
   };

   const std::vector<Metric> metrics = {
       {"time_spent", "s/veh", false, [](const TrafficStats& s) { return s.timeSpentPerVehicle(); }},
       {"delay", "s", false, [](const TrafficStats& s) { return s.meanDelay(); }},
       {"travel_time", "s", false, [](const TrafficStats& s) { return s.meanTravelTime(); }},
       {"p95_travel", "s", false, [](const TrafficStats& s) { return s.travelTimePercentile(0.95); }},
       {"max_travel", "s", false, [](const TrafficStats& s) { return s.travelTimePercentile(1.0); }},
       {"flow", "/min", true, [](const TrafficStats& s) { return s.flow(); }},
       {"queue", "veh", false, [](const TrafficStats& s) { return s.meanQueue(); }},
       {"max_queue", "veh", false, [](const TrafficStats& s) { return static_cast<double>(s.maxQueue); }},
       {"speed", "", true, [](const TrafficStats& s) { return s.meanRelativeSpeed(); }},
   };

   double mean(const std::vector<double>& x) {
      double sum = 0.0;
      for (const double v : x)
         sum += v;
      return x.empty() ? 0.0 : sum / static_cast<double>(x.size());
   }

   double deviation(const std::vector<double>& x) {
      if (x.size() < 2)
         return 0.0;
      const double m = mean(x);
      double sum = 0.0;
      for (const double v : x)
         sum += (v - m) * (v - m);
      return std::sqrt(sum / static_cast<double>(x.size() - 1));
   }
}

int main(int argc, char** argv) {
   std::vector<std::string> controllerNames = {"random", "fixed", "heuristic", "maxpressure", "qlearning", "dqn"};
   std::vector<Scenario> selected = scenarios();
   int seeds = 20, firstSeed = 1;
   double minutes = 20.0, warmup = 5.0, demand = -1.0;
   int maxVehicles = 150;
   std::string csvPath, dqnModel = DeepRLOperator::defaultModelPath,
                        qlearningModel = QLearningOperator::defaultModelPath;
   for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (i + 1 >= argc) {
         fprintf(stderr, "Missing value for %s\n", option.c_str());
         return 1;
      }
      const std::string value = argv[++i];
      if (option == "--controllers") {
         controllerNames = split(value, ',');
      } else if (option == "--scenarios") {
         selected.clear();
         for (const std::string& name : split(value, ',')) {
            const Scenario* s = findScenario(name);
            if (!s) {
               fprintf(stderr, "Unknown scenario: %s\n", name.c_str());
               return 1;
            }
            selected.push_back(*s);
         }
      } else if (option == "--demand") {
         demand = atof(value.c_str());
      } else if (option == "--max-vehicles") {
         maxVehicles = atoi(value.c_str());
      } else if (option == "--seeds") {
         seeds = atoi(value.c_str());
      } else if (option == "--first-seed") {
         firstSeed = atoi(value.c_str());
      } else if (option == "--minutes") {
         minutes = atof(value.c_str());
      } else if (option == "--warmup") {
         warmup = atof(value.c_str());
      } else if (option == "--csv") {
         csvPath = value;
      } else if (option == "--one-way-ratio") {
         constants::oneWayRoadRatio = atof(value.c_str());
      } else if (option == "--light-period") {
         constants::trafficLightPeriod = atof(value.c_str());
      } else if (option == "--dqn-model") {
         dqnModel = value;
      } else if (option == "--qlearning-model") {
         qlearningModel = value;
      } else {
         fprintf(stderr, "Unknown option: %s (see the top of tools/benchmark.cpp)\n", option.c_str());
         return 1;
      }
   }
   if (demand >= 0.0)
      selected = {{"custom", demand, maxVehicles}};

   std::ofstream csv;
   if (!csvPath.empty()) {
      csv.open(csvPath);
      csv << "scenario,demand,max_vehicles,seed,controller,generated,rejected,completed";
      for (const Metric& m : metrics)
         csv << ',' << m.name;
      csv << '\n';
   }

   for (const Scenario& scenario : selected) {
      printf("\nScenario %s: %.0f vehicles/min, at most %d vehicles, %d networks, %.0f min (+%.0f warm-up)\n",
             scenario.name.c_str(), scenario.demand, scenario.maxVehicles, seeds, minutes, warmup);
      printf("%-12s", "controller");
      for (const Metric& m : metrics)
         printf(" %16s", (std::string(m.name) + (m.unit[0] ? std::string("(") + m.unit + ")" : "")).c_str());
      printf("  rejected\n");
      std::map<std::string, std::vector<double>> timeSpent;
      for (const std::string& name : controllerNames) {
         ControllerType type;
         if (!parseController(name, type)) {
            fprintf(stderr, "Unknown controller: %s\n", name.c_str());
            return 1;
         }
         std::vector<std::vector<double>> values(metrics.size());
         long rejected = 0;
         for (int seed = firstSeed; seed < firstSeed + seeds; ++seed) {
            // A new controller for each run: no state is carried from a network to the next
            std::shared_ptr<IntersectionOperator> controller;
            if (type == ControllerType::DQN) {
               auto op = std::make_shared<DeepRLOperator>(seed);
               if (!op->load(dqnModel)) {
                  fprintf(stderr, "Cannot load %s: run the train tool first\n", dqnModel.c_str());
                  return 1;
               }
               op->setTraining(false);
               controller = op;
            } else if (type == ControllerType::Q_LEARNING) {
               auto op = std::make_shared<QLearningOperator>(seed);
               if (!op->load(qlearningModel)) {
                  fprintf(stderr, "Cannot load %s: run the train tool first\n", qlearningModel.c_str());
                  return 1;
               }
               op->setTraining(false);
               controller = op;
            } else {
               controller = makeOperator(type, seed);
            }
            const TrafficStats stats = run(controller, seed, scenario, warmup, minutes);
            rejected += stats.rejected;
            for (size_t k = 0; k < metrics.size(); ++k)
               values[k].push_back(metrics[k].get(stats));
            if (csv.is_open()) {
               csv << scenario.name << ',' << scenario.demand << ',' << scenario.maxVehicles << ','
                   << seed << ',' << name << ',' << stats.generated << ',' << stats.rejected << ','
                   << stats.completed;
               for (size_t k = 0; k < metrics.size(); ++k)
                  csv << ',' << values[k].back();
               csv << '\n';
            }
         }
         printf("%-12s", name.c_str());
         for (size_t k = 0; k < metrics.size(); ++k)
            printf(" %8.1f ± %-5.1f", mean(values[k]), deviation(values[k]));
         printf("  %ld\n", rejected);
         fflush(stdout);
         timeSpent[name] = values[0];
      }
      // Paired comparison with the random baseline: same networks, same demand
      if (timeSpent.count("random")) {
         const std::vector<double>& base = timeSpent["random"];
         printf("Time spent per vehicle compared to random, network by network:\n");
         for (const std::string& name : controllerNames) {
            if (name == "random")
               continue;
            std::vector<double> gain;
            int wins = 0;
            for (size_t k = 0; k < base.size(); ++k) {
               gain.push_back(100.0 * (base[k] - timeSpent[name][k]) / base[k]);
               wins += timeSpent[name][k] < base[k];
            }
            printf("  %-12s %+6.1f %% ± %-5.1f better on %d/%zu networks\n", name.c_str(), mean(gain),
                   deviation(gain), wins, base.size());
         }
      }
   }
   return 0;
}
