/**
 * @brief Traffic light controllers that do not learn: 2 baselines and 2 hand-written rules.
 */

#pragma once
// Libraries
#include <memory> // To use smart pointers
#include <random> // To use std::mt19937
// Headers
#include "Constants.h"
#include "IntersectionOperator.h"

/** @brief Baseline: random road at every decision. */
class RandomOperator : public IntersectionOperator {
 public:
   /** @param seed Random seed */
   explicit RandomOperator(unsigned int seed = 0);
   int decide(const Observation& observation) override;

 private:
   std::mt19937 rng;
};

/** @brief Baseline: each road in turn, for the same duration, whatever the traffic. */
class FixedCycleOperator : public IntersectionOperator {
 public:
   /** @param greenTime Duration of each green light in seconds (at least one traffic light period) */
   explicit FixedCycleOperator(double greenTime = 0.0);
   int decide(const Observation& observation) override;

 private:
   const double greenTime;
};

/** @brief Hand-written rules: keep the green while traffic flows, otherwise serve a full road
 * first, then the road with the most vehicles, among the roads whose vehicles can leave. */
class HeuristicOperator : public IntersectionOperator {
 public:
   int decide(const Observation& observation) override;
};

/** @brief Max-pressure rule (Varaiya, 2013): serve the road with the highest pressure, i.e. the
 * most vehicles compared to the roads they are going to. */
class MaxPressureOperator : public IntersectionOperator {
 public:
   int decide(const Observation& observation) override;
};

/** @brief Build a controller.
 * The learning controllers load their trained model from the 'models' folder when it exists
 * (they then only apply it), and learn from scratch otherwise.
 * @param type Controller type
 * @param seed Random seed
 * @returns Controller */
std::shared_ptr<IntersectionOperator> makeOperator(ControllerType type, unsigned int seed);

/** @brief Name of a controller, as used by the tools.
 * @returns Name, e.g. "dqn" */
const char* controllerName(ControllerType type);
