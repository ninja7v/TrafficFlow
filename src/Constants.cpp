#include "../headers/Constants.h"

namespace constants {
   ControllerType controllerType = ControllerType::DQN;
   double boost{ 1.0 };
   double demand{ 150.0 };
   int nbVehicleMax{ 150 };
   double oneWayRoadRatio{ 0.3 };
   double trafficLightPeriod{ 2.0 };
}
