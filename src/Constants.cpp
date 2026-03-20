#include "../headers/Constants.h"

namespace constants {
    LearningType learningType = LearningType::Q_LEARNING;
    double boost{ 1.0 };
    int flow{ 10 };
    int nbVehicleMax{ 70 };
    int maxConnectedInputRoads{ 8 };
    double trafficLightPeriod{ 2.0 / boost };
    int stateSize{ 1 + 3 * maxConnectedInputRoads };
    double speedMaxCar  { 0.3  * boost };
    double speedMaxBike { 0.4 * boost };
    double speedMaxTruck{ 0.2 * boost };
    double accelerationCar  { 0.003 * boost };
    double accelerationBike { 0.004 * boost };
    double accelerationTruck{ 0.002 * boost };
}
