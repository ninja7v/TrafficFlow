/**
 * @brief Define vehicles.
 */

#pragma once
// Libraries
#include <array>  // To use arrays
#include <list>   // To use lists
#include <vector> // To use vectors
#include <memory> // To use smart pointers
// Headers
#include "Intersection.h"

class Road;
/**
 * @brief Define vehicles.
 */
class Vehicle {
public:
   /** @brief Constructor.
    * @param i1 Start Intersection
    * @param i2 End Intersection of the starting road to compute the direction
    * @param id Vehicle ID
    * @param target Destination
    * @param track Itinerary
    * @returns Vehicle*/
   Vehicle(const Intersection* i1,
           const Intersection* i2,
           const int id,
           Intersection* target,
           const std::list<Road*> track);
   /** @brief Destructor (virtual: vehicles are owned through base pointers). */
   virtual ~Vehicle() = default;
   /** @brief Theoritical speed function.
    * @param dist Distance with the front obstacle
    * @return Speed */
   double breakingSpeed(const double dist) const;
   /** @brief Adjust the speed to approach the theoritical speed function.
    * @param v Vehicle in front */
   void moveToVehicle(const std::shared_ptr<Vehicle> v);
   /** @brief Adjust the speed to approach the theoritical speed function.
    * @param i Intersection in front
    * @param idRoad Road ID of the Road the Vehicle is*/
   void moveToIntersection(const Intersection* i,
                           const int idRoad);
   /** @brief Update Vehicle itilerary when arriving at the end of the Road. */
   void updateItinerary();
   /** @brief Display the Vehicle. */
   void displayVehicle();
   /** @brief GPS.
    * @return Next Road fromt the itinerary */
   Road* nextRoad();
   /** @brief Calculate the distance with the front Vehicle.
    * @param v Vehicle in front
    * @return Distance */
   double distance(const std::shared_ptr<Vehicle> v) const;
   /** @brief Calculate the distance with the front Intersection.
    * @param i Intersection in front
    * @return Distance */
   double distance(const Intersection* i) const;
   /** @brief Getter.
    * @return Vehicle ID */
   int getID() const;
   /** @brief Getter.
    * @return Arrived at destination? */
   bool getStatus() const;
   /** @brief Getter.
    * @return Vehicle Speed */
   double getSpeed() const;
   /** @brief Getter.
    * @return Vehicle destination */
   Intersection* getDestination() const;
   /** @brief Getter.
    * @return Vehicle position */
   const std::vector<double>& getPosition() const;
   /** @brief Getter.
    * @return Vehicle speed max */
   virtual double getSpeedMax() const = 0;
   /** @brief Getter.
    * @return Vehicle acceleration */
   virtual double getAcceleration() const = 0;
   /** @brief Getter.
    * @return Vehicle width */
   virtual double getWidth() const = 0;
   /** @brief Getter.
    * @return Vehicle height */
   virtual double getHeight() const = 0;
   /** @brief Getter.
    * @return Vehicle color */
   virtual std::array<double, 3> getColor() const = 0;
   /** @brief Getter.
    * @return Vehicle type */
   virtual bool is2Wheeler() const = 0;
   /** @brief Is the vehicle waiting (almost stopped)?
    * @return Waiting? */
   bool isWaiting() const;
   /** @brief Count one simulation step in the waiting times if the vehicle is waiting. */
   void updateWaitingTime();
   /** @brief Stop the vehicle (blocked). */
   void stop();
   /** @brief Getter.
    * @return Time spent waiting since the vehicle entered the network, in seconds */
   double getWaitingTime() const;
   /** @brief Getter.
    * @return Time spent waiting on the current road, in seconds */
   double getRoadWaitingTime() const;
   /** @brief Forget the time spent waiting on the previous road. */
   void resetRoadWaitingTime();
   /** @brief Getter.
    * @return Simulation time at which the vehicle was generated, in seconds */
   double getCreationTime() const;
   /** @brief Getter.
    * @return Duration of the trip without any traffic nor traffic light, in seconds */
   double getFreeFlowTime() const;
   /** @brief Setter. Describe the trip for the statistics.
    * @param creationTime Simulation time at which the vehicle was generated, in seconds
    * @param freeFlowTime Duration of the trip without any traffic nor traffic light, in seconds */
   void setTrip(double creationTime, double freeFlowTime);
   /** @brief Setter. */
   void setDirection(const Intersection* i);
   /** @brief Setter. */
   void setPosition(const std::vector<double>);
   /** @brief Setter. */
   void setNewItinerary(const std::list<Road*> track);
   /** @brief Setter. */
   void setStatus(const bool arrived);

private:
   /** Vehicle ID. */
   const int idVehicle;
   /** Arrived at destination? */
   bool isArrived;
   /** Vehicle is braking */
   bool isBraking;
   /** Vehicle speed in grid unit per step. */
   double speed;
   /** Vehicle destination. */
   Intersection* destination;
   /** Vehicle position. */
   std::vector<double> position;
   /** Vehicle direction. */
   std::array<double, 2> direction;
   /** Vehicle itinerary. */
   std::list<Road*> itinerary;
   /** Time spent waiting since the vehicle entered the network, in seconds. */
   double waitingTime = 0.0;
   /** Time spent waiting on the current road, in seconds. */
   double roadWaitingTime = 0.0;
   /** Simulation time at which the vehicle was generated, in seconds. */
   double creationTime = 0.0;
   /** Duration of the trip without any traffic nor traffic light, in seconds. */
   double freeFlowTime = 0.0;
};
