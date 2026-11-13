/**
 * @brief Define relevant constants of the simulator.
 */

#pragma once
// Libraries
#include<algorithm> // To use max()

/** Traffic light controller: 2 baselines, 2 hand-written rules and 2 learning methods. */
enum class ControllerType {
   RANDOM,       // Baseline: random road every decision
   FIXED_CYCLE,  // Baseline: each road in turn
   HEURISTIC,    // Hand-written rules
   MAX_PRESSURE, // Hand-written rule from the literature: serve the highest pressure
   Q_LEARNING,   // Tabular reinforcement learning
   DQN           // Deep reinforcement learning
};

namespace constants {
/** @defgroup Network parameters */
/**@{*/
   /** Number of possible positions on the X axis for the intersections. It must be a positive number. */
   constexpr int sizeX{ 110 };
   /** Number of possible positions on the Y axis for the intersections. It must be a positive number. */
   constexpr int sizeY{ 70 };
   /** Duration of one simulation step in seconds. The model always advances with this step,
    * so its behavior does not depend on the frame rate nor on the boost. */
   constexpr double timeStep{ 1.0 / 60.0 };
   /** Speed of the simulation in the window: simulated seconds per real second. It only changes
    * how many steps are computed per frame, never the model. It must be a positive number. */
   extern double boost;
   /** Scale of the representation. It must be a positive number. */
   constexpr double zoom{ 1.0 };
/**@}*/
/** @defgroup Road parameters */
/**@{*/
   /** Width Road in pixel. It must be a positive number. */
   constexpr double widthRoad{ 10.0 * zoom };
   /** Half width Road in pixel. It must be a positive number. */
   constexpr double halfWidthRoad{ widthRoad / 2.0 };
/**@}*/
/** @defgroup Vehicle parameters */
/**@{*/
   /** Traffic demand: vehicles generated per simulated minute (Poisson arrivals). It must be >= 0. */
   extern double demand;
   /** Maximum number of vehicle simultaneously in the network. The vehicles generated while the
    * network is full, or while their first road is full, wait outside. It must be a positive number. */
   extern int nbVehicleMax;
   /** Maximum number of vehicles waiting outside the network. Above it, new vehicles are rejected. */
   constexpr int nbWaitingVehicleMax{ 300 };
   /** Maximum car speed in grid unit per step. It must be a positive number. */
   constexpr double speedMaxCar{ 0.3 };
   /** Maximum Bike speed in grid unit per step. It must be a positive number. */
   constexpr double speedMaxBike{ 0.4 };
   /** Maximum Truck speed in grid unit per step. It must be a positive number. */
   constexpr double speedMaxTruck{ 0.2 };
   /** Car acceleration in grid unit per step, per step. It must be a positive number. */
   constexpr double accelerationCar{ 0.003 };
   /** Bike acceleration in grid unit per step, per step. It must be a positive number. */
   constexpr double accelerationBike{ 0.004 };
   /** Truck acceleration in grid unit per step, per step. It must be a positive number. */
   constexpr double accelerationTruck{ 0.002 };
   /** A vehicle slower than this fraction of its maximum speed is waiting (in a queue). */
   constexpr double waitingSpeedRatio{ 0.1 };
   /** Car width in pixel. It must be a positive number. */
   constexpr double widthCar  { 2.0 * zoom };
   /** Bike width in pixel. It must be a positive number. */
   constexpr double widthBike { 1.0 * zoom };
   /** Truck width in pixel. It must be a positive number. */
   constexpr double widthTruck{ 3.0 * zoom };
   /** Car height in pixel. It must be a positive number. */
   constexpr double heightCar  { 5.0 * zoom };
   /** Bike height in pixel. It must be a positive number. */
   constexpr double heightBike {  3.0 * zoom };
   /** Truck height in pixel. It must be a positive number. */
   constexpr double heightTruck{ 10.0 * zoom };
   /** Vehicle light diameter. It must be a positive number. */
   constexpr double diameterHeadlight{ 1.0 * zoom };
   /** Security gab between a vehicle and an obstacle. It must be a positive number. */
   constexpr double distanceSecurity{ 0.5 * zoom };
/**@}*/
/** @defgroup Intersection parameters */
/**@{*/
   /** Number of Intersection in the network. It must be a positive number. */
   constexpr int nbIntersections{ std::max(sizeX, sizeY) / 10 }; // To have something visible < 15
   /** Distance between intersections. It must be a positive number. */
   constexpr int minGap{ static_cast<int>(10.0 * zoom) }; // pi*minGap^2*nbIntersections<sizeX*sizeY
   /** Intersection diameter in pixel. It must be a positive number. */
   constexpr double diameterIntersection{ 20.0 * zoom };
   /** Probability for a road to be one-way (the others are two-way). It must be in [0, 1].
    * Some one-way roads are made two-way if needed so that every intersection is reachable. */
   extern double oneWayRoadRatio;
   /** Time between two traffic light decisions in simulated seconds. It must be a positive number. */
   extern double trafficLightPeriod;
/**@}*/
/** @defgroup Window parameters */
/**@{*/
   /** Step for the sizeX and sizeY parameters. It must be a positive number. */
   constexpr int interval{ static_cast<int>(8.0 * zoom) };
   /** Distance between the network and the frame of the window. It must be a positive number. */
   constexpr int margin  { static_cast<int>(50.0 * zoom) };
   /** Window width. */
   constexpr int SCREEN_WIDTH { sizeX * interval + 2*margin };
   /** Window height. */
   constexpr int SCREEN_HEIGHT{ sizeY * interval + 2*margin };
   /** Ratio for the X axis. */
   constexpr int ratioX{ /*1*/(SCREEN_WIDTH - 2*margin) / (sizeX - 1) };  // If sizeX>1
   /** Ratio for the Y axis. */
   constexpr int ratioY{ /*1*/(SCREEN_HEIGHT - 2*margin) / (sizeY - 1) }; // If sizeY>1
/**@}*/
/** @defgroup Controller parameters */
/**@{*/
   /** Traffic light controller. */
   extern ControllerType controllerType;
/**@}*/
}
