// Helpers shared by the headless tools: traffic scenarios and measured runs.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../headers/Constants.h"
#include "../headers/Controllers.h"
#include "../headers/Network.h"

/** Traffic conditions of a run. */
struct Scenario {
   std::string name;
   /** Vehicles generated per simulated minute. */
   double demand;
   /** Maximum number of vehicles in the network. */
   int maxVehicles;
};

/** Representative scenarios, from free-flowing to more demand than any controller can serve. */
inline const std::vector<Scenario>& scenarios() {
   static const std::vector<Scenario> all = {
       {"light", 60.0, 150},
       {"medium", 120.0, 150},
       {"heavy", 180.0, 150},
       {"saturated", 600.0, 70}, // The network is always full: measures its capacity
       {"overload", 600.0, 150}, // Stress test: enough vehicles to block the network (gridlock)
   };
   return all;
}

inline const Scenario* findScenario(const std::string& name) {
   for (const Scenario& s : scenarios())
      if (s.name == name)
         return &s;
   return nullptr;
}

/** Run a controller on the random network 'seed': 'warmup' minutes to fill the network, then
 * 'minutes' measured. The same seed and scenario give the same network and the same demand.
 * With 'stopIfGridlocked', the run ends when no vehicle arrived for a minute (used for
 * training: nothing more can be learned from a blocked network). */
inline TrafficStats run(const std::shared_ptr<IntersectionOperator>& controller,
                        const unsigned int seed,
                        const Scenario& scenario,
                        const double warmup,
                        const double minutes,
                        const bool stopIfGridlocked = false) {
   constants::demand = scenario.demand;
   constants::nbVehicleMax = scenario.maxVehicles;
   constants::controllerType = ControllerType::FIXED_CYCLE; // Placeholder, replaced below
   Network network(seed);
   network.setOperator(controller);
   const int stepsPerMinute = static_cast<int>(60.0 / Network::timeStep);
   for (int k = 0; k < static_cast<int>(warmup * stepsPerMinute); ++k)
      network.step();
   network.resetStats();
   int completed = 0;
   for (int k = 0; k < static_cast<int>(minutes * stepsPerMinute); ++k) {
      network.step();
      if (stopIfGridlocked && (k + 1) % stepsPerMinute == 0) {
         if (network.getCompletedVehicles() == completed && network.getVehicleCount() > 0)
            break;
         completed = network.getCompletedVehicles();
      }
   }
   return network.getStats();
}

inline bool parseController(const std::string& name, ControllerType& type) {
   for (const ControllerType t :
        {ControllerType::RANDOM, ControllerType::FIXED_CYCLE, ControllerType::HEURISTIC,
         ControllerType::MAX_PRESSURE, ControllerType::Q_LEARNING, ControllerType::DQN}) {
      if (name == controllerName(t)) {
         type = t;
         return true;
      }
   }
   return false;
}

inline std::vector<std::string> split(const std::string& text, const char separator) {
   std::vector<std::string> parts;
   size_t begin = 0;
   while (begin <= text.size()) {
      const size_t end = std::min(text.find(separator, begin), text.size());
      if (end > begin)
         parts.push_back(text.substr(begin, end - begin));
      begin = end + 1;
   }
   return parts;
}
