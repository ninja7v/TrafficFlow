// Libraries
#include <array>     // To use arrays
#include <list>      // To use lists
#include <GLFW/glfw3.h> // To display
#include <iostream>  // To debug
#include <ctime>     // To use clock_t
#include <vector>    // To use vectors
#include <memory>    // To use smart pointers
#include <algorithm> // To use max
#include <cmath>     // To use sqrt, pow, exp
// Headers
#include "../headers/Constants.h"
#include "../headers/Vehicle.h"

Vehicle::Vehicle(const Intersection* i1,
                 const Intersection* i2,
                 const int id,
                 Intersection* target,
                 const std::list<Road*> track)
   : idVehicle(id),
     isArrived(false),
     isBraking(false),
     speed(0.0),
     destination(target),
     itinerary(track),
     enterRoadTime(clock()) {
   if (i1)
      position = i1->getPosition();
   setDirection(i2);
}

double Vehicle::breakingSpeed(const double d) const {
   //// Breaking speed function choice:
   // - Constant (order 1 -> not smoth)
   //return constants::speedMax;
   // - Sigmoid (warning: f(0) != 0)
   //return this->getSpeedMax() / (1.0 + exp(-d + 2*constants::distanceSecurity));
   // - Underwood
   //return constants::speedMax * (1.0 - exp(-pow(d - constants::distanceSecurity, 2.0)));
   // - Squared
   return getSpeedMax() * pow(d, 2) / (pow(d, 2) + 5.0 * constants::distanceSecurity);
}

void Vehicle::moveToVehicle(const std::shared_ptr<Vehicle> v) {
   if (v)
   {
      const double previousSpeed = speed;
      // Distance with the intersection ahead
      const double d = distance(v);
      // Update speed
      if (d < constants::distanceSecurity)
         speed = 0.0; // Pound!
      else
         speed = (speed < breakingSpeed(d)) ? speed + getAcceleration() : (speed + breakingSpeed(d)) / 2.0;
      // Move
      const std::array<double, 2> move {direction[0] * speed, direction[1] * speed};
      const double move_dist = std::sqrt(std::pow(move.at(0), 2) + std::pow(move.at(1), 2)) - (getHeight() + v->getHeight()) / 12.0;
      if (move_dist < d) {
         position[0] += move.at(0);
         position[1] += move.at(1);
      }
      else { // Snap to vehicle to avoid overshooting
         position[0] = v->position[0] - direction[0] * (getHeight() + v->getHeight()) / 7.0;
         position[1] = v->position[1] - direction[1] * (getHeight() + v->getHeight()) / 7.0;
      }
      // Braking ?
      isBraking = previousSpeed * 0.95 > speed; // Braking if speed reduced by more than 5%
   }
}

void Vehicle::moveToIntersection(const Intersection* i,
                                 const int idRoad) {
   if (i)
   {
      const double previousSpeed = speed;
      // Distance with the intersection ahead
      const double d = distance(i);
      // Update speed
      const bool isRed = i->isRed(idRoad);
      if (isRed)
         if (d < constants::distanceSecurity)
            speed = 0.0; // Pound!
         else
            speed = (speed < breakingSpeed(d)) ? speed + getAcceleration() : (speed + breakingSpeed(d)) / 2.0;
      else
         speed = (speed < getSpeedMax()) ? speed + getAcceleration() : getSpeedMax();
      // Move
      const std::array<double, 2> move {direction[0] * speed, direction[1] * speed};
      const double stepLength = std::sqrt(std::pow(move.at(0), 2) + std::pow(move.at(1), 2));
      const double move_dist = stepLength - (getHeight() + constants::diameterIntersection) / 12.0;
      const double stopDistance = constants::distanceSecurity / 2.0;
      if (isRed && stepLength > d - stopDistance) {
         // Never run a red light: stop in front of the intersection instead of snapping into it.
         const double allowed = std::max(d - stopDistance, 0.0);
         position[0] += direction[0] * allowed;
         position[1] += direction[1] * allowed;
         speed = 0.0;
      } else if (move_dist < d) {
         position[0] += move.at(0);
         position[1] += move.at(1);
         if (distance(i) > d) { // double check
            position[0] = i->getPosition()[0];
            position[1] = i->getPosition()[1];
         }
      } else { // Snap to intersection to avoid overshooting
         position[0] = i->getPosition()[0];
         position[1] = i->getPosition()[1];
      }
      // Braking ?
      isBraking = previousSpeed * 0.95 > speed; // Braking if speed reduced by more than 5%
   }
}

void Vehicle::updateItinerary() {
   if (!itinerary.empty()) {
      itinerary.pop_front();
   }
}

void Vehicle::displayVehicle() {
   const double center[2] = { direction[1] * constants::halfWidthRoad + position[0] * constants::ratioX + constants::margin,
                             -direction[0] * constants::halfWidthRoad + position[1] * constants::ratioY + constants::margin };
   const std::array<double, 2> dh = { direction[0] * getHeight(), direction[1] * getHeight() };
   const std::array<double, 2> dw = { direction[0] * getWidth(), direction[1] * getWidth() };
   // Body
   const std::array<double, 2> frontLeft  = {center[0] + dh[0] - dw[1], center[1] + dw[0] + dh[1]};
   const std::array<double, 2> frontRight = {center[0] + dh[0] + dw[1], center[1] - dw[0] + dh[1]};
   const std::array<double, 2> rearLeft   = {center[0] - dh[0] - dw[1], center[1] + dw[0] - dh[1]};
   const std::array<double, 2> rearRight  = {center[0] - dh[0] + dw[1], center[1] - dw[0] - dh[1]};
   const std::array<double, 3> color = getColor();
   glColor3d(color[0], color[1], color[2]);
   glBegin(GL_QUADS);
   glVertex2d(frontLeft[0], frontLeft[1]);
   glVertex2d(frontRight[0], frontRight[1]);
   glVertex2d(rearRight[0], rearRight[1]);
   glVertex2d(rearLeft[0], rearLeft[1]);
   glEnd();
   // Lights
   glPointSize(static_cast<float>(constants::diameterHeadlight));
   glBegin(GL_POINTS);
   // Front lights
   glColor3d(1.0, 1.0, 0.0); // Yellow
   if (is2Wheeler()) {
      glVertex2d(center[0] + dh[0], center[1] + dh[1]);
   }
   else {
      glVertex2d(frontLeft[0], frontLeft[1]);
      glVertex2d(frontRight[0], frontRight[1]);
   }
   // Rear lights
   if (isBraking)
      glColor3d(1.0, 0.0, 0.0); // Red
   else
      glColor3d(0.5, 0.0, 0.0); // Dark Red
   if (is2Wheeler()) {
      glVertex2d(center[0] - dh[0], center[1] - dh[1]);
   }
   else {
      glVertex2d(rearLeft[0], rearLeft[1]);
      glVertex2d(rearRight[0], rearRight[1]);
   }
   glEnd();
}

Road* Vehicle::nextRoad() {
   return (itinerary.empty()) ? nullptr : itinerary.front();
}

double Vehicle::distance(const std::shared_ptr<Vehicle> v) const {
   return v ? std::max(std::sqrt(std::pow(position[0] - v->position[0], 2) + std::pow(position[1] - v->position[1], 2)) -
                       (getHeight() + v->getHeight()) / 7.0,
                       0.0) :
              0.0;
}

double Vehicle::distance(const Intersection* i) const {
   return i ? std::max(std::sqrt(std::pow(position[0] - i->getPosition()[0], 2) + std::pow(position[1] - i->getPosition()[1], 2)) -
                       (getHeight() + constants::diameterIntersection) / 12.0,
                       0.0) :
              0.0;
}

int Vehicle::getID() const {
   return idVehicle;
}

bool Vehicle::getStatus() const {
   return isArrived;
}

double Vehicle::getSpeed() const {
   return speed;
}

Intersection* Vehicle::getDestination() const {
   return destination;
}

std::vector<double> Vehicle::getPosition() const {
   return position;
}

clock_t Vehicle::getEnterRoadTime() const {
   return enterRoadTime;
}

void Vehicle::setDirection(const Intersection* i) {
   if (i)
   {
      // We don't take the direction from road to avoid circular dependencies
      const std::array<double, 2> delta { i->getPosition()[0] - position[0],
                                          i->getPosition()[1] - position[1] };
      if (delta[0] != 0.0 || delta[1] != 0.0) {
         const double d = sqrt(pow(delta[0], 2) + pow(delta[1], 2)); // cannot be zero here
         direction = { delta[0] / d, delta[1] / d };
      }
   }
}

void Vehicle::setPosition(const std::vector<double> pos) {
   position = pos;
}

void Vehicle::setNewItinerary(const std::list<Road*> track) {
   itinerary = track;
}

void Vehicle::setStatus(const bool arrived) {
   isArrived = arrived;
}

void Vehicle::setEnterRoadTime(clock_t t) {
   enterRoadTime = t;
}
