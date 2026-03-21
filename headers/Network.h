/**
 * @brief Central class managing all components of the traffic network.
 */

#pragma once
// Libraries
#include <list>   // To use lists
#include <vector> // To use vectors
#include <memory> // To use smart pointers
// Header files
#include "Intersection.h"
#include "IntersectionOperator.h"
#include "Road.h"
#include "Map.h"

// Forward declaration
struct GLFWwindow;

class Network {
public:
   /** @brief Default constructor.
    * @returns Network */
   Network();
   /** @brief Network display.
    * @param maxFrames Optional parameter restricting total frames to process for the tests (0 = infinite). */
   void displayNetwork(int maxFrames = 0);

protected:

private:
   /** @brief Helper to initialize the window and ImGui. */
   GLFWwindow* initWindowAndImGui();
   /** @brief Process and update the metrics. */
   void processMetrics(double currentTime, double& lastTime, int& lastCompletedVehicles, double& smoothedFlowPerMin, bool& isFirstFlowMeas, int& nbFrames, int& lastFPS);
   /** @brief Render the control panel. */
   void renderControlPanel(double smoothedFlowPerMin, double& smoothedAvgSpeed, int lastFPS, double& lastPrintTime, double elapsedSimulationMinutes, double currentTime);
   /** @brief Process one step of the simulation (vehicle movements, RL/Heuristic updates). */
   void processSimulationStep();
   /** Operators */
   std::shared_ptr<IntersectionOperator> qLearningOp;
   std::shared_ptr<IntersectionOperator> deepRLOp;
   /** Global Intersection Operator. */
   std::shared_ptr<IntersectionOperator> globalOperator;
   /** Is the simulation paused? */
   bool isPaused = false;
   /** Road Map. */
   Map map;
   /** Intersections. */
   std::vector<std::unique_ptr<Intersection>> Intersections;
   /** Roads. */
   std::vector<std::unique_ptr<Road>> Roads;
   /** Vehicles */
   std::list<std::shared_ptr<Vehicle>> Vehicles;
   /** @brief Add vehicle to the simulation. */
   void addVehicle();
   /** @brief Move the Vehicle for the next frame. */
   void updateVehiclesPosition();
   /** Number of completed trips. */
   int completedVehicles = 0;
   /** Start time of the simulation. */
   clock_t simulationStartTime;
};
