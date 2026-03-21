// Libraries
#include <GLFW/glfw3.h> // To display
#include <ctime>        // To use clock() and clock_t
#include <memory>       // To use smart pointers
#include <numeric>      // To use accumulate
// Headers
#include "../headers/Constants.h"
#include "../headers/DeepRLOperator.h"
#include "../headers/Global.h"
#include "../headers/Intersection.h"
#include "../headers/QLearningOperator.h"
#include "../headers/Road.h"

Intersection::Intersection(const int n,
                           const std::vector<double> pos,
                           std::shared_ptr<IntersectionOperator> op)
    : idIntersection(n),
      position(pos),
      coordinates{position[0] * constants::ratioX + constants::margin,
                  position[1] * constants::ratioY + constants::margin},
      op(op),
      currentGreenRoadIndex(0),
      lastAction(0),
      lastDelay(0.0),
      lastSwitchTime(clock()) {
}

bool Intersection::isRed(const int id) const {
   return !input.empty() ? input.at(currentGreenRoadIndex) != id : true;
}

void Intersection::displayIntersection() const {
   glPointSize(constants::diameterIntersection);
   glColor3d(0.0, 0.0, 0.0); // Black
   glEnable(GL_POINT_SMOOTH);
   glBegin(GL_POINTS);
   glVertex2d(coordinates[0], coordinates[1]);
   glEnd();
   glDisable(GL_POINT_SMOOTH);
}

void Intersection::addInputRoad(Road* r) {
   if (r) {
      input.push_back(r->getID());
      inputRoads.push_back(r);
   }
}

int Intersection::getID() const {
   return idIntersection;
}

const std::vector<double> Intersection::getPosition() const {
   return position;
}

int Intersection::getNumberInputRoads() const {
   return static_cast<int>(inputRoads.size());
}

bool Intersection::operator==(const Intersection i) {
   return idIntersection == i.idIntersection;
}

void Intersection::setOperator(std::shared_ptr<IntersectionOperator> newOp) {
   op = newOp;
}

int Intersection::computeHeuristicAction() const {
   if (inputRoads.empty())
      return -1;

   // Check if we should keep the current light green
   if (currentGreenRoadIndex >= 0 && currentGreenRoadIndex < static_cast<int>(inputRoads.size())) {
      const Road* currentGreen = inputRoads[currentGreenRoadIndex];
      if (!currentGreen->getVehicles().empty()) {
         auto firstCar = currentGreen->getVehicles().front();
         double distToIntersection =
             currentGreen->getLength() - firstCar->distance(currentGreen->getStart());
         // If the first car is within an approaching threshold, keep it green
         if (distToIntersection < constants::heightCar * 3.0) {
            return currentGreenRoadIndex;
         }
      }
   }

   // 1. Check for "full road"
   int maxCars = -1;
   int choosenRoad = -1;

   for (size_t i = 0; i < inputRoads.size(); ++i) {
      const Road* r = inputRoads[i];
      if (!r->getVehicles().empty()) {
         auto lastCar = r->getVehicles().back();
         // A road is "full" if the last car is very close to the start of the road
         if (lastCar->distance(r->getStart()) <=
             constants::heightCar + 2.0 * constants::distanceSecurity) {
            return static_cast<int>(i); // Priority 1: full road
         }
      }

      int numCars = static_cast<int>(r->getVehicles().size());
      if (numCars > maxCars) {
         maxCars = numCars;
         choosenRoad = static_cast<int>(i);
      }
   }

   // 2. Road containing the most cars
   if (choosenRoad != -1 && maxCars > 0) {
      return choosenRoad;
   }

   // Default to keeping the current green or picking arbitrarily
   return currentGreenRoadIndex;
}

std::vector<int> Intersection::constructState() const {
   std::vector<int> state;
   state.reserve(constants::stateSize);
   state.push_back(currentGreenRoadIndex);

   const int totalNumberOfArrivingVehicle =
       std::accumulate(inputRoads.begin(), inputRoads.end(), 0, [](int sum, const Road* r) {
          return sum + r->getTotalNumberOfArringVehicles();
       });
   const double averageNewVehicles =
       static_cast<double>(totalNumberOfArrivingVehicle) / static_cast<double>(inputRoads.size());
   for (const Road* r : inputRoads) {
      auto stats = r->getVehicleStats(averageNewVehicles);
      state.push_back(std::get<0>(stats)); // Occupancy
      state.push_back(std::get<1>(stats)); // Speed
      state.push_back(std::get<2>(stats)); // Usage
   }

   // Pad to standardized size
   while (state.size() < static_cast<size_t>(constants::stateSize)) {
      state.push_back(0);
   }
   return state;
}

double Intersection::calculateReward(double& currentDelay) const {
   double totalDelay = 0.0;
   int numVehicles = 0;
   for (const Road* r : inputRoads) {
      for (const auto& v : r->getVehicles()) {
         const double timeOnRoad = static_cast<double>(clock() - v->getEnterRoadTime()) /
                                   static_cast<double>(CLOCKS_PER_SEC); // in seconds
         const double dist = v->distance(r->getStart());
         const double ideal = dist / v->getSpeedMax();
         totalDelay += std::max(timeOnRoad - ideal, 0.0);
         numVehicles++;
      }
   }

   currentDelay = numVehicles > 0 ? (totalDelay / static_cast<double>(numVehicles)) : 0.0;

   double rawDelta = lastDelay - currentDelay;
   double reward = rawDelta / 5.0; 
   if (reward > 1.0) reward = 1.0;
   if (reward < -1.0) reward = -1.0;
      
   return reward;
}

void Intersection::performLearning(const std::vector<int>& state, double reward, const std::vector<int>& availableActions) {
   if (!lastState.empty()) {
      op->learn(lastState, lastAction, reward, state, availableActions);
   }
}

void Intersection::update() {
   if (inputRoads.empty())
      return;

   if (constants::learningType == LearningType::HEURISTIC) {
      const clock_t now = clock();
      const double timeSinceSwitch = static_cast<double>(now - lastSwitchTime) /
                                     static_cast<double>(CLOCKS_PER_SEC); 
      if (timeSinceSwitch > constants::trafficLightPeriod) {
         int action = computeHeuristicAction();
         if (action != -1) {
            currentGreenRoadIndex = action;
            lastAction = action;
            lastSwitchTime = now;
         }
      }
      return;
   }

   std::vector<int> state = constructState();

   double currentDelay = 0.0;
   double reward = calculateReward(currentDelay);

   std::vector<int> availableActions;
   for (size_t i = 0; i < inputRoads.size(); ++i) {
      availableActions.push_back(static_cast<int>(i));
   }

   performLearning(state, reward, availableActions);

   const clock_t now = clock();
   const double timeSinceSwitch = static_cast<double>(now - lastSwitchTime) /
                                  static_cast<double>(CLOCKS_PER_SEC); 
   if (timeSinceSwitch > constants::trafficLightPeriod) {
      const int action = op->decide(state, availableActions);

      if (action != -1) {
         currentGreenRoadIndex = action;
         lastAction = action;
         lastState = state;
         lastDelay = currentDelay;
         lastSwitchTime = now;
      }
   }
}
