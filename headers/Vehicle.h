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
   /** @brief Getter.
    * @return Time when vehicle entered current road */
   clock_t getEnterRoadTime() const;
   /** @brief Setter. */
   void setDirection(const Intersection* i);
   /** @brief Setter. */
   void setPosition(const std::vector<double>);
   /** @brief Setter. */
   void setNewItinerary(const std::list<Road*> track);
   /** @brief Setter. */
   void setStatus(const bool arrived);
   /** @brief Setter. */
   void setEnterRoadTime(clock_t t);

private:
   /** Vehicle ID. */
   const int idVehicle;
   /** Arrived at destination? */
   bool isArrived;
   /** Vehicle is braking */
   bool isBraking;
   /** Vehicle speed in pixel per second. */
   double speed;
   /** Vehicle destination. */
   Intersection* destination;
   /** Vehicle position. */
   std::vector<double> position;
   /** Vehicle direction. */
   std::array<double, 2> direction;
   /** Vehicle itinerary. */
   std::list<Road*> itinerary;
   /** Time when vehicle entered current road. */
   clock_t enterRoadTime;
};
