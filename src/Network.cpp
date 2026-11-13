// Libraries
#define STB_IMAGE_IMPLEMENTATION
#include <GLFW/glfw3.h>                  // To display
#include <algorithm>                     // To use max
#include <backends/imgui_impl_glfw.h>    // Platform backend: handles GLFW inputs and window events
#include <backends/imgui_impl_opengl2.h> // Renderer backend: draws ImGui with fixed-function OpenGL, like the simulator
#include <cmath>                         // To use sqrt and pow
#include <cstdlib>                       // To use std::getenv
#include <ctime>                         // To use time()
#include <delaunator.hpp>                // To compute the Delaunay triangulation
#include <imgui.h>                       // Core ImGui functionality (panel)
#include <iostream>                      // To use std::cout
#include <memory>                        // To use smart pointers
#include <stb/stb_image.h>               // To use stbi_load()
#include <random>                        // To use std::mt19937
#include <set>                           // To use sets
#include <vector>                        // To use vectors

// Headers
#include "../headers/Bike.h"
#include "../headers/Car.h"
#include "../headers/Constants.h"
#include "../headers/Controllers.h"
#include "../headers/DeepRLOperator.h"
#include "../headers/Graph.h"
#include "../headers/Network.h"
#include "../headers/QLearningOperator.h"
#include "../headers/Random.h"
#include "../headers/Truck.h"

double TrafficStats::flow() const {
   return duration > 0.0 ? completed / (duration / 60.0) : 0.0;
}

double TrafficStats::meanTravelTime() const {
   double sum = 0.0;
   for (const float t : travelTimes)
      sum += t;
   return travelTimes.empty() ? 0.0 : sum / static_cast<double>(travelTimes.size());
}

double TrafficStats::meanDelay() const {
   return travelTimes.empty() ? 0.0
                              : meanTravelTime() - freeFlowTime / static_cast<double>(travelTimes.size());
}

double TrafficStats::travelTimePercentile(const double ratio) const {
   if (travelTimes.empty())
      return 0.0;
   std::vector<float> sorted = travelTimes;
   const size_t k = std::min(sorted.size() - 1, static_cast<size_t>(ratio * static_cast<double>(sorted.size())));
   std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(k), sorted.end());
   return sorted[k];
}

double TrafficStats::timeSpentPerVehicle() const {
   const int vehicles = generated - rejected; // The time of a rejected vehicle is not counted
   return vehicles > 0 ? timeSpent / vehicles : 0.0;
}

double TrafficStats::meanQueue() const {
   return duration > 0.0 ? timeWaiting / duration : 0.0;
}

double TrafficStats::meanVehicles() const {
   return duration > 0.0 ? timeInNetwork / duration : 0.0;
}

double TrafficStats::meanRelativeSpeed() const {
   return timeInNetwork > 0.0 ? relativeSpeedTime / timeInNetwork : 0.0;
}

Network::Network() : Network(static_cast<unsigned int>(time(nullptr))) {}

void Network::setController(const ControllerType type) {
   constants::controllerType = type;
   auto& controller = controllers[static_cast<size_t>(type)];
   if (!controller)
      controller = makeOperator(type, seed);
   setOperator(controller);
}

void Network::setOperator(std::shared_ptr<IntersectionOperator> op) {
   globalOperator = op;
   for (const auto& i : Intersections) {
      if (i)
         i->setOperator(globalOperator);
   }
}

Network::Network(const unsigned int seed) : seed(seed), demandRng(seed ^ 0x9E3779B9u) {
   std::mt19937 rng(seed); // Network layout
   // Construct card as preparation
   auto distance = [&](std::vector<double> p1, std::vector<double> p2) {
      return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
   };
   auto isPositionValid = [&](const std::vector<double>& p) {
      for (const auto& i : Intersections) {
         if (i && distance(p, i->getPosition()) < constants::minGap) {
            return false;
         }
      }
      return true;
   };
   std::vector<double> positions; // x0, y0, x1, y1, ...
   for (int k = 0; k < constants::nbIntersections; k++) {
      std::vector<double> position(2);
      do {
         position[0] = rnd::index(rng, constants::sizeX);
         position[1] = rnd::index(rng, constants::sizeY);
      } while (!isPositionValid(position));
      positions.push_back(position[0]);
      positions.push_back(position[1]);
      Intersections.push_back(std::make_unique<Intersection>(k, position));
   }
   // Roads: the Delaunay triangulation gives a connected planar graph, where point k is
   // Intersections[k]. Informations here: https://github.com/delfrrr/delaunator-cpp
   const delaunator::Delaunator d(positions);
   std::set<graph::Edge> undirectedEdges; // Each triangle side once (shared sides appear twice)
   for (size_t i = 0; i < d.triangles.size(); i += 3) {
      for (size_t j = 0; j < 3; j++) {
         const int a = static_cast<int>(d.triangles[i + j]);
         const int b = static_cast<int>(d.triangles[i + (j + 1) % 3]);
         undirectedEdges.insert({std::min(a, b), std::max(a, b)});
      }
   }
   const std::vector<graph::Edge> roads = graph::orientEdges(
       constants::nbIntersections, {undirectedEdges.begin(), undirectedEdges.end()},
       constants::oneWayRoadRatio, rng);
   int id = 0;
   for (const auto& [a, b] : roads) {
      Intersection* begin = Intersections[a].get();
      Intersection* end = Intersections[b].get();
      Roads.push_back(std::make_unique<Road>(id++, begin, end));
      map.setConnection(a, b, Roads.back().get());
      end->addInputRoad(Roads.back().get());
   }
   setController(constants::controllerType);
}

GLFWwindow* Network::initWindowAndImGui() {
#if defined(__linux__) && defined(GLFW_PLATFORM_X11)
   // GLFW's Wayland backend has no title bar when libdecor is unavailable.
   if (std::getenv("DISPLAY") != nullptr && glfwPlatformSupported(GLFW_PLATFORM_X11)) {
      glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
   }
#endif
   // Initialize GLFW
   if (!glfwInit()) {
      return nullptr;
   }
   glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
   glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
   // Create window
   GLFWwindow* window = glfwCreateWindow(constants::SCREEN_WIDTH, constants::SCREEN_HEIGHT,
                                          "Traffic Simulator", nullptr, nullptr);
   if (!window) {
      glfwTerminate();
      return nullptr;
   }
   // Add icon
   int width, height, channels; // Doesn't need to be initialized
   unsigned char* pixels = stbi_load("Graphics/logo_TS.png", &width, &height, &channels, 4);
   if (pixels) {
      GLFWimage logo;
      logo.width = width;
      logo.height = height;
      logo.pixels = pixels;
      glfwSetWindowIcon(window, 1, &logo);
      stbi_image_free(pixels);
   }
   // Make the window's context current
   glfwMakeContextCurrent(window);
   glfwSwapInterval(1); // V-Sync: no need to render faster than the screen refresh rate
   glfwSetFramebufferSizeCallback(window, [](GLFWwindow*, int width, int height) {
      glViewport(0, 0, width, height);
   });
   glViewport(0, 0, constants::SCREEN_WIDTH,
              constants::SCREEN_HEIGHT); // specifies the part of the window to
                                         // which OpenGL will draw (in pixels)
   glMatrixMode(GL_PROJECTION);          // Projection matrix defines the properties of the camera
                                         // that views the objects in the world coordinate frame
   glLoadIdentity();                     // Replace the current matrix with the identity matrix and
                                         // starts us a fresh one
   glOrtho(0.0, static_cast<double>(constants::SCREEN_WIDTH), 0.0,
           static_cast<double>(constants::SCREEN_HEIGHT), 0.0,
           1.0);               // Set coordinate system
   glMatrixMode(GL_MODELVIEW); // (default matrix mode) modelview matrix defines
                               // how objects are transformed (meaning
                               // translation, rotation and scaling)
   glLoadIdentity();
   glClearColor(0.1f, 0.5f, 0.1f, 0.0f); // Set background color as green
   glEnable(GL_BLEND);                   // Make every component look smoother
   // Setup Dear ImGui context
   IMGUI_CHECKVERSION();
   ImGui::CreateContext();
   ImGuiIO& io = ImGui::GetIO();
   (void)io;
   ImGui::StyleColorsDark();
   if (!ImGui_ImplGlfw_InitForOpenGL(window, true) || !ImGui_ImplOpenGL2_Init()) {
      std::cerr << "Failed to initialize the ImGui backends" << std::endl;
      ImGui_ImplGlfw_Shutdown();
      ImGui::DestroyContext();
      glfwDestroyWindow(window);
      glfwTerminate();
      return nullptr;
   }
   return window;
}

void Network::setTerminalOutput(const bool enabled) {
   terminalOutput = enabled;
}

void Network::printTerminalOutput(const double smoothedFlowPerMin,
                                  const double smoothedAvgSpeed,
                                  const int lastFPS,
                                  const double elapsedSimulationMinutes) const {
   if (!terminalOutput) {
      std::cout << "Time: " << elapsedSimulationMinutes << "m | Flow/min: " << smoothedFlowPerMin
                << " | AvgSpeed: " << smoothedAvgSpeed << std::endl;
      return;
   }
   std::cout << "========== Control Panel ==========" << '\n'
             << "Time: " << elapsedSimulationMinutes << " min (simulation)" << '\n'
             << "FPS: " << lastFPS << '\n'
             << "Intersections: " << constants::nbIntersections << '\n'
             << "Roads: " << Roads.size() << '\n'
             << "Vehicles: " << Vehicles.size() << " (+" << waitingTrips.size() << " waiting to enter)" << '\n'
             << "Completed trips: " << completedVehicles << '\n'
             << "Flow: " << smoothedFlowPerMin << " Trips/min" << '\n'
             << "Avg Speed: " << smoothedAvgSpeed << " px/s" << '\n'
             << "Avg travel time: " << stats.meanTravelTime() << " s" << '\n'
             << "Avg delay: " << stats.meanDelay() << " s" << '\n'
             << "Boost: " << constants::boost << '\n'
             << "Demand: " << constants::demand << " Vehicles/min" << '\n'
             << "Max Vehicles: " << constants::nbVehicleMax << '\n'
             << "Method: " << controllerName(constants::controllerType) << '\n'
             << "State: " << (isPaused ? "PAUSED" : "PLAYING") << '\n'
             << "===================================" << std::endl;
}

void Network::processMetrics(double currentTime, double& lastTime, int& nbFrames, int& lastFPS) {
   if (currentTime - lastTime >= 1.0) { // If last print was more than 1 sec ago
      lastFPS = nbFrames;
      nbFrames = 0;
      lastTime = currentTime;
      // Flow over the simulated time, so it does not depend on the boost nor on the frame rate
      const double deltaTimeMinutes = (simulationTime - lastFlowTime) / 60.0;
      if (deltaTimeMinutes > 0.0) {
         const double instantFlowPerMin = (completedVehicles - lastFlowCompleted) / deltaTimeMinutes;
         smoothedFlowPerMin = lastFlowTime == 0.0 ? instantFlowPerMin
                                                  : 0.8 * smoothedFlowPerMin + 0.2 * instantFlowPerMin;
         lastFlowCompleted = completedVehicles;
         lastFlowTime = simulationTime;
      }
   }
}

void Network::renderControlPanel(int lastFPS, double& lastPrintTime, double currentTime) {
   ImGui_ImplOpenGL2_NewFrame();
   ImGui_ImplGlfw_NewFrame();
   ImGui::NewFrame();

   // Metrics
   ImGui::Begin("Simulation Control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
   ImGui::Text("Metrics");
   ImGui::Text("%d Intersections ", constants::nbIntersections);
   ImGui::Text("%d Vehicles (+%d waiting to enter)", getVehicleCount(), getWaitingVehicleCount());
   ImGui::Text("%.2f Flow (Trips/min)", smoothedFlowPerMin);

   double totalSpeed = 0.0;
   for (const auto& v : Vehicles) {
      totalSpeed += v->getSpeed();
   }

   const double numVehicles = std::max(1.0, static_cast<double>(Vehicles.size()));
   const double avgSpeed = totalSpeed / numVehicles; // Grid units per simulation step
   const double instantAvgSpeedPxPerSec = avgSpeed / timeStep * constants::ratioX;

   if (Vehicles.empty()) {
      smoothedAvgSpeed = 0.0;
   } else if (smoothedAvgSpeed == 0.0) {
      smoothedAvgSpeed = instantAvgSpeedPxPerSec;
   } else if (!isPaused) {
      smoothedAvgSpeed = 0.98 * smoothedAvgSpeed + 0.02 * instantAvgSpeedPxPerSec;
   }

   ImGui::Text("%.2f Average Speed (px/s)", smoothedAvgSpeed);
   ImGui::Text("%.1f Average travel time (s)", stats.meanTravelTime());
   ImGui::Text("%.1f Average delay (s)", stats.meanDelay());
   ImGui::Text("%.1f Vehicles waiting", stats.meanQueue());
   ImGui::Text("%.1f Simulated minutes", simulationTime / 60.0);

   if (currentTime - lastPrintTime >= 5.0) {
      printTerminalOutput(smoothedFlowPerMin, smoothedAvgSpeed, lastFPS, simulationTime / 60.0);
      lastPrintTime = currentTime;
   }

   ImGui::Text("%d FPS", lastFPS);

   // Parameters
   ImGui::Separator();
   ImGui::Text("Parameters");
   float boost = static_cast<float>(constants::boost);
   if (ImGui::SliderFloat("Boost", &boost, 0.1f, 100.0f, "%.1f", ImGuiSliderFlags_Logarithmic)) {
      constants::boost = static_cast<double>(boost);
   }
   float demand = static_cast<float>(constants::demand);
   if (ImGui::SliderFloat("Demand (vehicles/min)", &demand, 0.0f, 600.0f, "%.0f")) {
      constants::demand = static_cast<double>(demand);
   }
   ImGui::SliderInt("Max number vehicle", &constants::nbVehicleMax, 0, 300);

   int currentMethodIndex = static_cast<int>(constants::controllerType);
   if (ImGui::Combo("Traffic lights", &currentMethodIndex,
                    "Random\0Fixed cycle\0Heuristic\0Max pressure\0Q-Learning\0DQN (Neural Network)\0")) {
      setController(static_cast<ControllerType>(currentMethodIndex));
      resetStats(); // The averages describe the selected controller
   }
   if (globalOperator && (constants::controllerType == ControllerType::Q_LEARNING ||
                          constants::controllerType == ControllerType::DQN)) {
      ImGui::Text(globalOperator->isLearning() ? "Learning from scratch (no trained model found)"
                                               : "Trained model");
   }

   ImGui::Separator();
   const char* playPauseLabels[] = {"Pause", "Play"};
   if (ImGui::Button(playPauseLabels[isPaused])) {
      isPaused = !isPaused;
   }
   ImGui::SameLine();
   if (ImGui::Button("Reset metrics")) {
      resetStats();
   }
   ImGui::End();
}

void Network::step() {
   // Live road costs used by the vehicles' GPS
   for (const auto& r : Roads) {
      if (r)
         map.updateConnection(r);
   }
   generateDemand();
   addVehicles();
   updateVehiclesPosition();
   Vehicles.remove_if([this](const std::shared_ptr<Vehicle>& v) {
      if (v->getStatus()) {
         completedVehicles++;
         stats.completed++;
         stats.travelTimes.push_back(
             static_cast<float>(simulationTime + timeStep - v->getCreationTime()));
         stats.freeFlowTime += v->getFreeFlowTime();
         return true; // Remove the vehicle
      }
      return false; // Keep the vehicle
   });
   simulationTime += timeStep;
   // Intersections
   for (const auto& i : Intersections) {
      if (i)
         i->update(simulationTime); // Traffic light decisions
   }
   updateStats();
}

void Network::updateStats() {
   stats.duration += timeStep;
   stats.timeSpent += timeStep * static_cast<double>(Vehicles.size() + waitingTrips.size());
   stats.timeInNetwork += timeStep * static_cast<double>(Vehicles.size());
   for (const auto& v : Vehicles) {
      stats.relativeSpeedTime += timeStep * v->getSpeed() / v->getSpeedMax();
   }
   for (const auto& r : Roads) {
      const int queue = r->getQueueLength();
      stats.timeWaiting += timeStep * queue;
      stats.maxQueue = std::max(stats.maxQueue, queue);
   }
}

const TrafficStats& Network::getStats() const {
   return stats;
}

void Network::resetStats() {
   stats = TrafficStats();
}

int Network::getWaitingVehicleCount() const {
   return static_cast<int>(waitingTrips.size());
}

void Network::render() const {
   for (const auto& r : Roads) {
      if (r)
         r->displayRoad();
   }
   for (const auto& v : Vehicles) {
      if (v)
         v->displayVehicle();
   }
   for (const auto& i : Intersections) {
      if (i)
         i->displayIntersection();
   }
   // Traffic lights
   for (const auto& r : Roads) {
      if (r)
         r->displayLight();
   }
}

double Network::getSimulationTime() const {
   return simulationTime;
}

int Network::getCompletedVehicles() const {
   return completedVehicles;
}

int Network::getVehicleCount() const {
   return static_cast<int>(Vehicles.size());
}

int Network::getRoadCount() const {
   return static_cast<int>(Roads.size());
}

int Network::getIntersectionCount() const {
   return static_cast<int>(Intersections.size());
}

const std::vector<std::unique_ptr<Road>>& Network::getRoads() const {
   return Roads;
}

const Intersection* Network::getIntersection(const int id) const {
   return (id >= 0 && id < getIntersectionCount()) ? Intersections[id].get() : nullptr;
}

void Network::displayNetwork(int maxFrames /*Used by tests*/) {
   GLFWwindow* window = initWindowAndImGui();
   if (!window) {
      return;
   }
   double lastTime = glfwGetTime();
   double lastPrintTime = lastTime;
   double previousFrameTime = lastTime;
   double accumulator = 0.0;
   int nbFrames = 0;
   int lastFPS = 0;
   int totalLoopFrames = 0;
   // Main loop (1 frame)
   while (!glfwWindowShouldClose(window) && (maxFrames == 0 || totalLoopFrames < maxFrames)) {
      totalLoopFrames++;
      // FPS Counter
      const double currentTime = glfwGetTime();
      nbFrames++;

      processMetrics(currentTime, lastTime, nbFrames, lastFPS);

      renderControlPanel(lastFPS, lastPrintTime, currentTime);

      // Fixed time step: the simulation advances at the same pace whatever the frame rate.
      // The boost only changes the number of steps per frame, so the model (and what the
      // controllers learn) is the same at any boost.
      // The frame time is capped so a slow frame (window dragged, breakpoint...) does not
      // trigger a burst of catch-up steps.
      accumulator += std::min(currentTime - previousFrameTime, 0.25) * constants::boost;
      previousFrameTime = currentTime;
      if (isPaused) {
         accumulator = 0.0;
      }
      // In test mode (maxFrames > 0) run at least one step per frame so the logic is exercised.
      if (maxFrames > 0 && !isPaused) {
         accumulator = std::max(accumulator, timeStep);
      }
      constexpr double maxComputeTime = 0.05; // Seconds per frame: keep the window responsive
      while (accumulator >= timeStep) {
         step();
         accumulator -= timeStep;
         if (maxFrames == 0 && glfwGetTime() - currentTime > maxComputeTime) {
            accumulator = 0.0; // Too slow for this boost: run as fast as possible
         }
      }

      // Clear the screen
      glClear(GL_COLOR_BUFFER_BIT);
      render();
      // Rendering panel
      ImGui::Render();
      ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
      // Swap front and back buffers
      glfwSwapBuffers(window);
      // Poll for and process events
      glfwPollEvents();
   }
   // Cleanup
   ImGui_ImplOpenGL2_Shutdown();
   ImGui_ImplGlfw_Shutdown();
   ImGui::DestroyContext();
   glfwDestroyWindow(window);
   glfwTerminate();
}

void Network::generateDemand() {
   if (constants::demand <= 0.0) {
      nextArrivalTime = -1.0;
      return;
   }
   // Poisson arrivals: the time between 2 vehicles follows an exponential law.
   const double rate = constants::demand / 60.0; // Vehicles per second
   if (nextArrivalTime < 0.0 || arrivalDemand != constants::demand) {
      nextArrivalTime = simulationTime + rnd::exponential(demandRng, rate);
      arrivalDemand = constants::demand;
   }
   while (nextArrivalTime <= simulationTime) {
      // Random trip: from the start of a road to another intersection (not the end of this
      // road either: the trip would be this road only... or a U-turn back to its start)
      Road* origin = Roads[rnd::index(demandRng, static_cast<int>(Roads.size()))].get();
      const int first = std::min(origin->getStart()->getID(), origin->getEnd()->getID());
      const int second = std::max(origin->getStart()->getID(), origin->getEnd()->getID());
      int destination = rnd::index(demandRng, constants::nbIntersections - 2);
      if (destination >= first)
         destination++;
      if (destination >= second)
         destination++;
      const int vehicleType = rnd::index(demandRng, 3);
      stats.generated++;
      if (static_cast<int>(waitingTrips.size()) < constants::nbWaitingVehicleMax)
         waitingTrips.push_back({nextArrivalTime, origin, Intersections[destination].get(), vehicleType});
      else
         stats.rejected++;
      nextArrivalTime += rnd::exponential(demandRng, rate);
   }
}

void Network::addVehicles() {
   for (auto trip = waitingTrips.begin(); trip != waitingTrips.end();) {
      if (static_cast<int>(Vehicles.size()) >= constants::nbVehicleMax)
         break;
      Road* r = trip->origin;
      const Intersection* start = r->getStart();
      const Intersection* end = r->getEnd();
      std::shared_ptr<Vehicle> v;
      switch (trip->vehicleType) {
      case 0:
         v = std::make_shared<Car>(start, end, nextVehicleId, trip->destination, std::list<Road*>{});
         break;
      case 1:
         v = std::make_shared<Bike>(start, end, nextVehicleId, trip->destination, std::list<Road*>{});
         break;
      default:
         v = std::make_shared<Truck>(start, end, nextVehicleId, trip->destination, std::list<Road*>{});
         break;
      }
      if (!r->canEnter(*v)) {
         ++trip; // No room at the start of the road: keep waiting
         continue;
      }
      const std::list<Road*> itinerary = map.track(end, trip->destination, r);
      double tripLength = r->getLength();
      for (const Road* next : itinerary)
         tripLength += next->getLength();
      v->setNewItinerary(itinerary);
      v->setTrip(trip->creationTime, tripLength / v->getSpeedMax() * timeStep);
      nextVehicleId++;
      Vehicles.push_back(v);
      r->addVehicle(v);
      stats.entered++;
      trip = waitingTrips.erase(trip);
   }
}

void Network::updateVehiclesPosition() {
   for (const auto& r : Roads) {
      if (r)
         r->moveVehicles();
   }
}
