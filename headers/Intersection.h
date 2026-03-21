/**
 * @brief Define intersections.
 */

#pragma once
// Libraries
#include <array>  // To use arrays
#include <ctime>  // To use clock_t
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
   const std::vector<double> getPosition() const;
   /** @brief Getter.
    * @returns Number of input roads */
   int getNumberInputRoads() const;
   /** @brief Comparison operator.
    * @returns Equal? */
   bool operator==(const Intersection i);
   /** @brief Update the intersection lights using RL. */
   void update();
   /** @brief Set a new operator for this intersection. */
   void setOperator(std::shared_ptr<IntersectionOperator> newOp);
   /** @brief Evaluate heuristic fallback. */
   int computeHeuristicAction() const;

 protected:
 private:
   /** @brief Construct the dynamic learning state matrix. */
   std::vector<int> constructState() const;
   /** @brief Calculate the current reward metrics. */
   double calculateReward(double& currentDelay) const;
   /** @brief Pass metrics to operator for mathematical learning. */
   void performLearning(const std::vector<int>& state,
                        double reward,
                        const std::vector<int>& availableActions);

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

   /** @defgroup RL components */
   /**@{*/
   /** operator to decide traffic light actions */
   std::shared_ptr<IntersectionOperator> op;
   /** Index of the current green road in 'input' vector */
   int currentGreenRoadIndex;
   /** Last state observed */
   std::vector<int> lastState;
   /** Last action taken */
   int lastAction;
   /** Last cumulative delay */
   double lastDelay;
   /** Time of the last traffic light switch */
   clock_t lastSwitchTime;
   /**@}*/
};
