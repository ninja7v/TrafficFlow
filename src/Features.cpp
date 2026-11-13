// Libraries
#include <algorithm> // To use min and max
#include <initializer_list>
// Headers
#include "../headers/Features.h"

namespace {
   /** Traffic of the roads other than 'action'. */
   struct Others {
      int maxQueue = 0;
      int maxVehicles = 0;
      int totalVehicles = 0;
      double maxWait = 0.0;
   };

   Others summarizeOthers(const Observation& observation, const int action) {
      Others others;
      for (int i = 0; i < static_cast<int>(observation.approaches.size()); ++i) {
         if (i == action)
            continue;
         const ApproachObservation& a = observation.approaches[i];
         // A road whose vehicles cannot leave does not need the green light
         const int queue = a.canDischarge ? a.queue : 0;
         others.maxQueue = std::max(others.maxQueue, queue);
         others.maxVehicles = std::max(others.maxVehicles, a.canDischarge ? a.vehicles : 0);
         others.totalVehicles += a.vehicles;
         if (a.canDischarge)
            others.maxWait = std::max(others.maxWait, a.headWaitingTime);
      }
      return others;
   }

   /** Number of thresholds reached by a value: 0 below the first one. */
   int level(const double value, const std::initializer_list<double> thresholds) {
      int result = 0;
      for (const double threshold : thresholds)
         result += value >= threshold;
      return result;
   }

   /** 1 when the first vehicle is at the intersection, 0 when it is far (or no vehicle). */
   double closeness(const ApproachObservation& a) {
      constexpr double farDistance = 10.0; // Grid units: about half a second at full speed
      return a.vehicles > 0 ? std::max(0.0, 1.0 - a.headDistance / farDistance) : 0.0;
   }
}

namespace features {

std::vector<int> usefulActions(const Observation& observation) {
   // A full road blocks the intersection it comes from: if some can be emptied, only they are
   // considered. Otherwise, the roads whose first vehicle can leave.
   for (const bool fullOnly : {true, false}) {
      std::vector<int> actions;
      auto isUseful = [&](const int i) {
         const ApproachObservation& a = observation.approaches[i];
         return a.canDischarge && (a.full || !fullOnly);
      };
      if (isUseful(observation.currentGreen))
         actions.push_back(observation.currentGreen);
      for (int i = 0; i < static_cast<int>(observation.approaches.size()); ++i) {
         if (i != observation.currentGreen && isUseful(i))
            actions.push_back(i);
      }
      if (!actions.empty())
         return actions;
   }
   return {observation.currentGreen};
}

std::vector<double> describe(const Observation& observation, const int action) {
   const ApproachObservation& a = observation.approaches[action];
   const ApproachObservation& green = observation.approaches[observation.currentGreen];
   const bool isGreen = action == observation.currentGreen;
   const Others others = summarizeOthers(observation, action);
   // Scales: typical large values, so that the features stay around [0, 1]
   constexpr double vehicleScale = 10.0, waitScale = 30.0, greenScale = 20.0;
   return {
       // The road that would get the green light
       isGreen ? 1.0 : 0.0,
       std::min(a.queue / vehicleScale, 2.0),
       std::min(a.vehicles / vehicleScale, 2.0),
       std::min(a.headWaitingTime / waitScale, 3.0),
       a.canDischarge ? 1.0 : 0.0,
       closeness(a),
       std::min(a.downstreamVehicles / vehicleScale, 2.0),
       a.full ? 1.0 : 0.0,
       // The road that is green now (the same one when the decision is to keep it)
       std::min(green.queue / vehicleScale, 2.0),
       std::min(green.vehicles / vehicleScale, 2.0),
       green.canDischarge ? 1.0 : 0.0,
       closeness(green),
       std::min(observation.greenDuration / greenScale, 1.0),
       // The other roads
       std::min(others.maxQueue / vehicleScale, 2.0),
       std::min(others.totalVehicles / (2.0 * vehicleScale), 2.0),
       std::min(others.maxWait / waitScale, 3.0),
   };
}

int discretize(const Observation& observation, const int action) {
   const ApproachObservation& a = observation.approaches[action];
   const ApproachObservation& green = observation.approaches[observation.currentGreen];
   const bool isGreen = action == observation.currentGreen;
   const Others others = summarizeOthers(observation, action);

   const int vehicleLevel = level(a.vehicles, {1, 3, 6});            // 4: 0, 1-2, 3-5, 6+
   const int waitLevel = level(a.headWaitingTime, {10.0, 30.0});     // 3
   // Is traffic going through the green road? 0: no vehicle or blocked, 1: far, 2: at the light
   const int greenLevel = !green.canDischarge ? 0 : closeness(green) > 0.5 ? 2 : 1; // 3
   // Compared to the busiest other road: 0: fewer vehicles, 1: about the same, 2: more
   const int relativeLevel = level(a.vehicles - others.maxVehicles, {-1, 2}); // 3
   // Load of the whole intersection: what the future holds whatever the decision
   const int totalLevel = level(a.vehicles + others.totalVehicles, {3, 7, 13}); // 4

   int key = isGreen ? 1 : 0;
   key = key * 4 + vehicleLevel;
   key = key * 2 + (a.canDischarge ? 1 : 0);
   key = key * 2 + (a.full ? 1 : 0);
   key = key * 3 + waitLevel;
   key = key * 3 + greenLevel;
   key = key * 3 + relativeLevel;
   key = key * 4 + totalLevel;
   return key;
}

} // namespace features
