/**
 * @brief Define roads.
 */

#pragma once
// Libraries
#include <array>  // To use arrays
#include <list>   // To use lists
#include <memory> // To use smart pointers
#include <tuple>  // To use tuples
// Headers
#include "Intersection.h"
#include "Vehicle.h"

class Road {
public:
   Road() = delete;
/** @brief Constructor.
  * @param id Road ID
  * @param begin Start
  * @param end Arrival
  * @returns Road */
   Road(const int id, Intersection* begin, Intersection* end);
/** @brief Destructor. */
   ~Road();
/** @brief Check if the Road contain vehicles.
  * @returns Road */
   bool containVehicle() const;
/** @brief Add vehicle to the Road.
  * @param v Vehicle to add */
   void addVehicle(std::shared_ptr<Vehicle> v);
/** @brief Remove the first vehicle of the Road. */
   void removeVehicle();
/** @brief Advance the vehicles for the next frame. */
   void moveVehicles();
/** @brief Display the Road without the Vehicle on it. */
   void displayRoad() const;
/** @brief Display the traffic light at the end of the Road. */
   void displayLight() const;
/** @brief Getter.
  * @return Road ID*/
   int getID() const;
/** @brief Getter.
  * @return Road lenght */
   double getLength() const;
/** @brief Getter.
 * @return Start Intersection */
   Intersection* getStart() const;
/** @brief Getter.
  * @return End Intersection */
   Intersection* getEnd() const;
/** @brief Getter.
  * @return Get vehicles */
   const std::list<std::shared_ptr<Vehicle>>& getVehicles() const;
/** @brief Getter.
  * @return Orientation of the Road */
   std::array<double, 2> getDirection() const;
/** @brief Get number of vehicle arriving on the road since the beginning of the simulation.
  * @returns Number of vehicle */
   int getTotalNumberOfArringVehicles() const;
/** @brief Get statistics for RL state.
  * @param averageNewVehicles Average number of new vehicles since the beginning of the simulation to determine usage
  * @returns tuple(occupancy, first_car_speed, usage) */
   std::tuple<int, int, int> getVehicleStats(double averageNewVehicles) const;


protected:

private:
/** Road identifier */
   const int idRoad;
/** Start Intersection */
   Intersection* i1;
/** End Intersection */
   Intersection* i2;
/** Road length */
   const double length;
/** Vehicle on the Road */
   std::list<std::shared_ptr<Vehicle>> Vehicles;
/** Number of vehicles that have arrived on this road since the start of the simulation */
   int totalNumberOfArringVehicles;
/** Orientation of the Road */
   std::array<double, 2> direction;
/** Road coordinates */
   const double roadCoordinates[6];
/** Roadside left coordinates */
   const double sideLeft[6];
/** Roadside right coordinates */
   const double sideRight[6];
/** Traffic light coordinates */
   const double lightCoordinates[2];
};
