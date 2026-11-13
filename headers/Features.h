/**
 * @brief What the learning controllers see of a decision.
 *
 * A decision is "give the green light to road a". It is described from the point of view of
 * road a: its own traffic, the traffic of the road that is currently green, and the traffic of
 * the other roads. This description does not depend on the order of the roads nor on their
 * number, so one model is shared by all the intersections, and applies to any network.
 */

#pragma once
// Libraries
#include <vector> // To use vectors
// Headers
#include "IntersectionOperator.h"

namespace features {
   /** Number of values describing a decision for the neural network. */
   constexpr int count{ 16 };
   /** Number of situations distinguished by the Q-learning table. */
   constexpr int tableSize{ 2 * 4 * 2 * 2 * 3 * 3 * 3 * 4 };

   /** @brief Decisions worth considering: the roads whose first vehicle would leave if the light
    * were green. Giving the green light to an empty or blocked road while another one could
    * move only loses time, and can freeze a congested network, so the learning controllers
    * never do it (not even to explore).
    * @param observation Situation of the intersection
    * @returns Road indices, the current green road first if it is one of them. Only the current
    * green road if no vehicle can move. */
   std::vector<int> usefulActions(const Observation& observation);

   /** @brief Describe a decision with real numbers (around [0, 1]) for the neural network.
    * @param observation Situation of the intersection
    * @param action Road that would get the green light
    * @returns 'count' values */
   std::vector<double> describe(const Observation& observation, int action);

   /** @brief Describe a decision with a few levels for the Q-learning table.
    * @param observation Situation of the intersection
    * @param action Road that would get the green light
    * @returns Index in [0, tableSize - 1] */
   int discretize(const Observation& observation, int action);
}
