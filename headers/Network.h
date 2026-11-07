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
   /** @brief Network display (opens a window and runs the simulation in real time).
    * @param maxFrames Optional parameter restricting total frames to process for the tests (0 = infinite). */
   void displayNetwork(int maxFrames = 0);
   /** @brief Advance the simulation by one fixed time step. Does not need a window / OpenGL context. */
   void step();
   /** @brief Getter.
    * @returns Elapsed simulation time in seconds */
   double getSimulationTime() const;
   /** @brief Getter.
    * @returns Number of vehicles that reached their destination */
   int getCompletedVehicles() const;
   /** @brief Getter.
    * @returns Number of vehicles currently in the network */
   int getVehicleCount() const;
   /** @brief Getter.
    * @returns Number of roads */
   int getRoadCount() const;
   /** @brief Getter.
    * @returns Number of intersections */
   int getIntersectionCount() const;
   /** Duration of one simulation step in seconds (the simulation runs at a fixed rate, independent of the FPS). */
   static constexpr double timeStep{ 1.0 / 60.0 };

protected:

private:
   /** @brief Helper to initialize the window and ImGui. */
   GLFWwindow* initWindowAndImGui();
   /** @brief Process and update the metrics. */
   void processMetrics(double currentTime, double& lastTime, int& lastCompletedVehicles, double& smoothedFlowPerMin, bool& isFirstFlowMeas, int& nbFrames, int& lastFPS);
   /** @brief Render the control panel. */
   void renderControlPanel(double smoothedFlowPerMin, double& smoothedAvgSpeed, int lastFPS, double& lastPrintTime, double elapsedSimulationMinutes, double currentTime);
   /** @brief Draw roads, vehicles, intersections and traffic lights. */
   void render() const;
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
   /** Identifier given to the next spawned vehicle. */
   int nextVehicleId = 0;
   /** Elapsed simulation time in seconds. */
   double simulationTime = 0.0;
};
