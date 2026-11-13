/**
 * @brief Deep Reinforcement Learning Operator (DQN Agent).
 */

#pragma once
// Libraries
#include <random> // To use std::mt19937
#include <string> // To use std::string
#include <vector> // To use vectors
// Headers
#include "IntersectionOperator.h"
#include "NeuralNetwork.h"

/**
 * @brief DeepRLOperator implements a Deep Q-Network (DQN) agent.
 * A neural network approximates Q(situation, decision): the expected sum of future rewards of
 * the decision "give the green light to this road". The network takes the description of one
 * decision (features::describe()) and gives one value; it is evaluated for each input road.
 * The same network therefore serves every intersection, whatever its number of roads.
 */
class DeepRLOperator : public IntersectionOperator {
 public:
   /** Trained network loaded by makeOperator(), relative to the working directory. */
   static constexpr const char* defaultModelPath = "models/dqn.txt";

   /** @param seed Random seed (initial weights, exploration, replay) */
   explicit DeepRLOperator(unsigned int seed = 0);
   int decide(const Observation& observation) override;
   void learn(const Observation& observation,
              int action,
              double reward,
              const Observation& nextObservation) override;
   bool isLearning() const override;
   /** @brief Learn and explore (true), or only apply the network (false). */
   void setTraining(bool enabled);
   /** @brief Set the exploration: probability of a random decision, multiplied by 'decay' at
    * each learning step down to 'minimum'. */
   void setExploration(double epsilon, double minimum, double decay);
   /** @brief Getter.
    * @returns Current probability of a random decision */
   double getEpsilon() const;
   /** @brief Write the network in a text file.
    * @returns Success? */
   bool save(const std::string& path) const;
   /** @brief Read a network written by save().
    * @returns Success? The network is unchanged otherwise. */
   bool load(const std::string& path);

 private:
   /** @brief Best decision according to a network (the first one if equal).
    * @param network Network to use
    * @param decisions Description of each possible decision
    * @returns Index in 'decisions' */
   static size_t bestDecision(const NeuralNetwork& network,
                              const std::vector<std::vector<double>>& decisions);

   /** Network being trained, used to decide */
   NeuralNetwork policyNetwork;
   /** Copy updated from time to time, giving stable learning targets */
   NeuralNetwork targetNetwork;
   bool training = true;

   /** A single transition (experience) */
   struct Experience {
      /** Description of the decision taken */
      std::vector<double> decision;
      /** Reward received */
      double reward;
      /** Description of each useful decision in the resulting situation */
      std::vector<std::vector<double>> nextDecisions;
   };

   /** Memory to store past experiences (ring buffer once full) */
   std::vector<Experience> replayBuffer;
   /** Slot overwritten by the next experience once the buffer is full */
   size_t replayBufferNext = 0;
   /** Number of learning steps */
   long stepCount = 0;

   /** @defgroup hyperparameters */
   /**@{*/
   /** Maximum number of experiences to keep */
   static constexpr inline size_t maxBufferSize = 50000;
   /** Number of experiences needed before learning */
   static constexpr inline size_t minBufferSize = 500;
   /** Number of experiences to train on at each step */
   static constexpr inline size_t batchSize = 32;
   /** Copy policy weights to the stable bootstrap network every N transitions. */
   static constexpr inline int targetUpdateFrequency = 500;
   /** Number of neurons of the 2 hidden layers */
   static constexpr inline int hiddenSize = 32;
   /** Learning Rate: How fast the network updates its weights */
   static constexpr inline double alpha = 0.0005;
   /** Discount Factor: Importance of future rewards (0 = short-sighted, 1 = long-sighted) */
   static constexpr inline double gamma = 0.95;
   /** Exploration Rate: Probability of choosing a random action */
   double epsilon = 1.0;
   /** Minimum Exploration Rate: The agent will always explore a little bit */
   double epsilonMin = 0.05;
   /** Decay Rate: How fast epsilon decreases (multiplied at each step) */
   double epsilonDecay = 0.999;
   /** Random number generator */
   std::mt19937 rng;
   /**@}*/
};
