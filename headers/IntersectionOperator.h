/**
 * @brief Abstract base class for the traffic light controllers, and what they observe.
 */

#pragma once
// Libraries
#include <vector> // To use vectors

/** What a controller knows about one input road of its intersection. */
struct ApproachObservation {
   /** Number of vehicles on the road. */
   int vehicles = 0;
   /** Number of vehicles waiting (almost stopped) on the road. */
   int queue = 0;
   /** Time the first vehicle has been waiting on the road, in seconds. */
   double headWaitingTime = 0.0;
   /** Distance between the first vehicle and the intersection in grid units (road length if empty). */
   double headDistance = 0.0;
   /** The first vehicle would leave the road if the light were green (its next road is not full). */
   bool canDischarge = false;
   /** The queue of waiting vehicles reaches the start of the road: no more vehicle can enter it. */
   bool full = false;
   /** Average number of vehicles on the roads the vehicles of this road are going to. */
   double downstreamVehicles = 0.0;
   /** Road length in grid units. */
   double length = 0.0;
};

/** What a controller knows about its intersection when it takes a decision. */
struct Observation {
   /** Index of the input road with the green light. */
   int currentGreen = 0;
   /** Time since the green light was given to this road, in seconds. */
   double greenDuration = 0.0;
   /** Input roads. The actions are their indices. */
   std::vector<ApproachObservation> approaches;
};

class IntersectionOperator {
 public:
   virtual ~IntersectionOperator() = default;

   /** @brief Decide which input road gets the green light until the next decision.
    * @param observation Current situation of the intersection (at least one input road).
    * @return Index of the road to turn green. */
   virtual int decide(const Observation& observation) = 0;

   /** @brief Learn from the last decision. Does nothing for the controllers that do not learn.
    * @param observation Situation when the decision was taken.
    * @param action Decision taken (road index).
    * @param reward Reward received since then.
    * @param nextObservation Current situation. */
   virtual void learn(const Observation& observation,
                      int action,
                      double reward,
                      const Observation& nextObservation);

   /** @brief Does the controller learn (see learn())?
    * @return Learning? */
   virtual bool isLearning() const;
};
