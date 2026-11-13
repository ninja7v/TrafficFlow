/**
 * @brief Q-learning controller (table).
 */

#pragma once
// Libraries
#include <random> // To use std::mt19937
#include <string> // To use std::string
#include <vector> // To use std::vector
// Headers
#include "IntersectionOperator.h"

/**
 * @brief QLearningOperator learns a table Q(situation, decision): the expected sum of future
 * rewards of the decision "give the green light to this road" in a situation. Situations are
 * described by features::discretize(), from the point of view of the road, so the same table
 * serves every intersection.
 */
class QLearningOperator : public IntersectionOperator {
 public:
   /** Trained table loaded by makeOperator(), relative to the working directory. */
   static constexpr const char* defaultModelPath = "models/qlearning.txt";

   /** @param seed Random seed (exploration) */
   explicit QLearningOperator(unsigned int seed = 0);
   int decide(const Observation& observation) override;
   void learn(const Observation& observation,
              int action,
              double reward,
              const Observation& nextObservation) override;
   bool isLearning() const override;
   /** @brief Learn and explore (true), or only apply the table (false). */
   void setTraining(bool enabled);
   /** @brief Set the exploration: probability of a random decision, multiplied by 'decay' at
    * each learning step down to 'minimum'. */
   void setExploration(double epsilon, double minimum, double decay);
   /** @brief Getter.
    * @returns Current probability of a random decision */
   double getEpsilon() const;
   /** @brief Write the table in a text file.
    * @returns Success? */
   bool save(const std::string& path) const;
   /** @brief Read a table written by save().
    * @returns Success? The table is unchanged otherwise. */
   bool load(const std::string& path);

 private:
   /** @brief Best useful decision according to the table. */
   int bestAction(const Observation& observation) const;
   /** Q-Table: expected future reward (Q-value) of each (situation, decision) */
   std::vector<double> qTable;
   /** Number of updates of each Q-value */
   std::vector<int> visits;
   bool training = true;

   /** @defgroup hyperparameters */
   /**@{*/
   /** Learning Rate: at least this value, more for the first updates of a Q-value (1 / visits) */
   static constexpr inline double alphaMin = 0.01;
   /** Discount Factor: Importance of future rewards (0 = short-sighted, 1 = long-sighted).
    * Lower than for the neural network: the few levels of the table cannot tell what the future
    * holds, so a long horizon mostly adds noise (measured: more unstable trainings). */
   static constexpr inline double gamma = 0.5;
   /** Exploration Rate: Probability of choosing a random action */
   double epsilon = 1.0;
   double epsilonMin = 0.05;
   double epsilonDecay = 0.999;
   /** Random number generator */
   std::mt19937 rng;
   /**@}*/
};
