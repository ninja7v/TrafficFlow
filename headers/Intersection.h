/**
 * @brief Define intersections.
 */

#pragma once
// Libraries
#include <array>  // To use arrays
#include <memory> // To use smart pointers
#include <string> // To use strings
#include <vector> // To use vectors

// Headers
#include "IntersectionOperator.h"

class Road;

class Intersection {
 public:
   Intersection() = delete;
   /** @brief Constructor.
    * @param id Road ID
    * @param pos Position coordinates
    * @returns Car acceleration */
   Intersection(const int id,
                const std::vector<double> pos,
                std::shared_ptr<IntersectionOperator> op = nullptr);
   /** @brief Get trafic light color.
    * @param id Road ID
    * @returns true:Red / false:Green */
   bool isRed(const int id) const;
   /** @brief Display the intersection as black point.*/
   void displayIntersection() const;
   /** @brief Add an input road in the input vector.
    * @param r Road pointer */
   void addInputRoad(Road* r);
   /** @brief Getter.
    * @returns Intersection ID */
   int getID() const;
   /** @brief Getter.
    * @returns Position on the grid */
   const std::vector<double>& getPosition() const;
   /** @brief Getter.
    * @returns Number of input roads */
   int getNumberInputRoads() const;
   /** @brief Comparison operator.
    * @returns Equal? */
   bool operator==(const Intersection& i) const;
   /** @brief Advance the intersection by one simulation step: measure the traffic, and take a
    * traffic light decision when the traffic light period is over.
    * @param simulationTime Current simulation time in seconds. */
   void update(double simulationTime);
   /** @brief Set a new controller for this intersection. */
   void setOperator(std::shared_ptr<IntersectionOperator> newOp);
   /** @brief Describe the current situation of the intersection for the controllers.
    * @param simulationTime Current simulation time in seconds.
    * @returns Observation */
   Observation observe(double simulationTime) const;
   /** @brief Reward of a decision: the opposite of the time lost by the vehicles on the input
    * roads until the next decision, plus a bonus for the vehicles that left them.
    * @param waitingVehicles Average number of vehicles waiting on the input roads
    * @param departures Number of vehicles that crossed the intersection
    * @param longestWait Longest time a first vehicle has been waiting, in seconds
    * @returns Reward (higher is better) */
   static double reward(double waitingVehicles, int departures, double longestWait);
   /** @brief Getter.
    * @returns Index of the input road with the green light */
   int getGreenRoadIndex() const;

 protected:
 private:
   /** @brief Reward since the last decision. Starts a new measure, so call it once per decision. */
   double collectReward();

   /** Intersection identifier. */
   const int idIntersection;
   /** Position on the grid. */
   const std::vector<double> position;
   /** Position coordinates. */
   const std::array<double, 2> coordinates;
   /** Input / output road identifiers. */
   std::vector<int> input, output;
   /** Input / output roads. */
   std::vector<Road*> inputRoads;

   /** @defgroup Controller */
   /**@{*/
   /** Controller deciding the traffic lights */
   std::shared_ptr<IntersectionOperator> op;
   /** Index of the current green road in 'input' vector */
   int currentGreenRoadIndex;
   /** Situation when the last decision was taken (no input road: not observed yet) */
   Observation lastObservation;
   /** Last decision */
   int lastAction;
   /** Simulation time (s) of the last traffic light decision */
   double lastDecisionTime;
   /** Simulation time (s) since which the green road has the green light */
   double greenSince;
   /** Vehicles waiting on the input roads, summed over the steps since the last decision */
   long waitingVehicleSteps = 0;
   /** Number of steps since the last decision */
   int stepsSinceDecision = 0;
   /** Departures from the input roads counted at the last decision */
   int lastDepartures = 0;
   /**@}*/
};
