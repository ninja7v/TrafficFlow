/**
 * @brief Define roads.
 */

#pragma once
// Libraries
#include <array>  // To use arrays
#include <list>   // To use lists
#include <memory> // To use smart pointers
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
/** @brief Free length at the start of the road, behind the last vehicle (grid units).
  * @returns Free length (the road length if empty) */
   double getFreeLengthAtStart() const;
/** @brief Check that a vehicle can enter the road without overlapping the last one.
  * Uses the same spacing as the car-following model (Vehicle::distance).
  * @param v Vehicle entering
  * @returns Enough space? */
   bool canEnter(const Vehicle& v) const;
/** @brief Check that a car could not enter (the queue reaches the start of the road).
  * @returns Full? */
   bool isFull() const;
/** @brief Get number of vehicles that left the road (crossed its end intersection) since the start.
  * @returns Number of vehicles */
   int getNumberOfDepartedVehicles() const;
/** @brief Get number of vehicles queuing (almost stopped).
  * @returns Number of vehicles */
   int getQueueLength() const;
/** @brief Get the time the first vehicle has been waiting on this road.
  * @returns Time in seconds (0 if the road is empty) */
   double getHeadWaitingTime() const;
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
/** Number of vehicles that left this road since the start of the simulation */
   int departedVehicles = 0;
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
