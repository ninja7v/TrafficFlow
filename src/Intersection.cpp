// Libraries
#include <GLFW/glfw3.h> // To display
#include <algorithm>    // To use clamp
#include <memory>       // To use smart pointers
#include <numeric>      // To use accumulate
// Headers
#include "../headers/Constants.h"
#include "../headers/Global.h"
#include "../headers/Intersection.h"
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
      lastSwitchTime(0.0) {
}

bool Intersection::isRed(const int id) const {
   return !input.empty() ? input.at(currentGreenRoadIndex) != id : true;
}

void Intersection::displayIntersection() const {
   glPointSize(static_cast<float>(constants::diameterIntersection));
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

const std::vector<double>& Intersection::getPosition() const {
   return position;
}

int Intersection::getNumberInputRoads() const {
   return static_cast<int>(inputRoads.size());
}

bool Intersection::operator==(const Intersection& i) const {
   return idIntersection == i.idIntersection;
}

void Intersection::setOperator(std::shared_ptr<IntersectionOperator> newOp) {
   op = newOp;
   lastState.clear();
}

namespace {
   /** The front vehicle of the road would leave it if the light were green (not blocked downstream). */
   bool canDischarge(const Road* r) {
      if (!r->containVehicle())
         return false;
      const auto& front = r->getVehicles().front();
      const Road* next = front->nextRoad();
      return !next || next->canEnter(*front);
   }
}

int Intersection::computeHeuristicAction() const {
   if (inputRoads.empty())
      return -1;

   // Keep the current light green while its first vehicle is approaching and can go through.
   // A blocked vehicle must not hold the green: that starves the other roads (gridlock).
   if (currentGreenRoadIndex >= 0 && currentGreenRoadIndex < static_cast<int>(inputRoads.size())) {
      const Road* currentGreen = inputRoads[currentGreenRoadIndex];
      constexpr double approachingDistance = 2.0; // Grid units
      if (canDischarge(currentGreen) &&
          currentGreen->getVehicles().front()->distance(currentGreen->getEnd()) < approachingDistance) {
         return currentGreenRoadIndex;
      }
   }

   // Otherwise, among the roads whose traffic can go through: a full road first, then the
   // road with the most vehicles.
   int choosenRoad = -1;
   size_t bestScore = 0;
   for (size_t i = 0; i < inputRoads.size(); ++i) {
      const Road* r = inputRoads[i];
      if (!canDischarge(r))
         continue;
      const size_t score = r->getVehicles().size() + (r->isFull() ? 1000 : 0);
      if (score > bestScore) {
         bestScore = score;
         choosenRoad = static_cast<int>(i);
      }
   }
   return choosenRoad != -1 ? choosenRoad : currentGreenRoadIndex;
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

double Intersection::calculateReward() {
   int departures = 0;
   int queue = 0;
   for (const Road* r : inputRoads) {
      departures += r->getNumberOfDepartedVehicles();
      queue += r->getQueueLength();
   }
   const int throughput = departures - lastDepartures;
   lastDepartures = departures;
   // Throughput is what the flow metric measures. The queue penalty makes the agent care about
   // how many vehicles wait (a mean over vehicles, as before, cannot tell 1 from 15 stopped cars).
   // Scaled to stay around [-1, 1] for typical intersections.
   return throughput / 5.0 - queue / 20.0;
}

void Intersection::performLearning(const std::vector<int>& state, double reward, const std::vector<int>& availableActions) {
   if (!lastState.empty()) {
      op->learn(lastState, lastAction, reward, state, availableActions);
   }
}

void Intersection::update(const double simulationTime) {
   if (inputRoads.empty() || simulationTime - lastSwitchTime < constants::trafficLightPeriod)
      return;
   lastSwitchTime = simulationTime;

   if (constants::learningType == LearningType::HEURISTIC || !op) {
      const int action = computeHeuristicAction();
      if (action != -1) {
         currentGreenRoadIndex = action;
         lastAction = action;
      }
      return;
   }

   const std::vector<int> state = constructState();
   std::vector<int> availableActions;
   for (size_t i = 0; i < inputRoads.size(); ++i) {
      availableActions.push_back(static_cast<int>(i));
   }

   // One learning update represents one complete action interval.
   performLearning(state, calculateReward(), availableActions);
   const int action = op->decide(state, availableActions);

   if (action >= 0 && action < static_cast<int>(inputRoads.size())) {
      currentGreenRoadIndex = action;
      lastAction = action;
      lastState = state;
   }
}
