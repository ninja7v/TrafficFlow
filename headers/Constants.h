/**
 * @brief Define relevant constants of the simulator.
 */

#pragma once
// Libraries
#include<algorithm> // To use max()

enum LearningType {
      Q_LEARNING,
      DQN,
      HEURISTIC
   };

namespace constants {
/** @defgroup Network parameters */
/**@{*/
   /** Number of possible positions on the X axis for the intersections. It must be a positive number. */
   constexpr int sizeX{ 110 };
   /** Number of possible positions on the Y axis for the intersections. It must be a positive number. */
   constexpr int sizeY{ 70 };
   /** Overall speed of the simulation. It must be a positive number. */
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
   /** Frequency of appearance of vehicles. It must be in [0, 99]. */
   extern int flow;
   /** Maximum number of vehicle simultaneously. It must be a positive number. */
   extern int nbVehicleMax;
   /** Maximum car speed. It must be a positive number. */
   extern double speedMaxCar;
   /** Maximum Bike speed. It must be a positive number. */
   extern double speedMaxBike;
   /** Maximum Truck speed. It must be a positive number. */
   extern double speedMaxTruck;
   /** Car acceleration per frame. It must be a positive number. */
   extern double accelerationCar;
   /** Bike acceleration per frame. It must be a positive number. */
   extern double accelerationBike;
   /** Truck acceleration per frame. It must be a positive number. */
   extern double accelerationTruck;
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
   /** Maximum number of connected input roads (for state standardization). */
   extern int maxConnectedInputRoads;
   /** Duration of a state in seconds. */
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
/** @defgroup Learning parameters */
/**@{*/
   /** Learning type. */
   extern LearningType learningType;
   /** Size of the standardized state vector. */
   extern int stateSize;
/**@}*/

   /** function to adjust constants when boost is changed */
   inline void updateBoostDependentConstants()
   {
      flow = static_cast<int>(10.0 * boost);
      trafficLightPeriod = 0.4 / boost;
      speedMaxCar   = 0.3 * boost;
      speedMaxBike  = 0.4 * boost;
      speedMaxTruck = 0.2 * boost;
      accelerationCar   = 0.003 * boost;
      accelerationBike  = 0.004 * boost;
      accelerationTruck = 0.002 * boost;
   }
}
