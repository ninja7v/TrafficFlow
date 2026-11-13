// Libraries
#include <GLFW/glfw3.h> // To display
#include <algorithm>    // To use max
#include <memory>       // To use smart pointers
// Headers
#include "../headers/Constants.h"
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
      lastDecisionTime(0.0),
      greenSince(0.0) {
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
   lastObservation.approaches.clear(); // The next decision starts a new learning transition
}

int Intersection::getGreenRoadIndex() const {
   return currentGreenRoadIndex;
}

Observation Intersection::observe(const double simulationTime) const {
   Observation observation;
   observation.currentGreen = currentGreenRoadIndex;
   observation.greenDuration = simulationTime - greenSince;
   observation.approaches.reserve(inputRoads.size());
   for (const Road* r : inputRoads) {
      ApproachObservation a;
      a.length = r->getLength();
      a.headDistance = a.length;
      const auto& vehicles = r->getVehicles();
      if (!vehicles.empty()) {
         const auto& front = vehicles.front();
         a.vehicles = static_cast<int>(vehicles.size());
         a.queue = r->getQueueLength();
         a.headWaitingTime = r->getHeadWaitingTime();
         a.headDistance = front->distance(r->getEnd());
         // Full: the queue reaches the start of the road (not a vehicle that has just entered)
         a.full = r->isFull() && vehicles.back()->isWaiting();
         const Road* next = front->nextRoad();
         a.canDischarge = !next || next->canEnter(*front);
         for (const auto& v : vehicles) {
            if (const Road* n = v->nextRoad())
               a.downstreamVehicles += static_cast<double>(n->getVehicles().size());
         }
         a.downstreamVehicles /= a.vehicles;
      }
      observation.approaches.push_back(a);
   }
   return observation;
}

double Intersection::reward(const double waitingVehicles,
                            const int departures,
                            const double longestWait) {
   // The network objective is the total time lost by the vehicles. Its local, per decision, part
   // is the number of vehicles waiting at this intersection: this is the main term.
   // - Departures: small bonus, so that discharging a queue is rewarded as soon as it starts.
   // - Longest wait: a vehicle alone on a quiet road costs little, so the queue term alone lets
   //   it wait for minutes. This term grows until the vehicle is served.
   // Scaled to stay around [-1, 0] in usual traffic.
   constexpr double longWait = 30.0; // Seconds
   return -waitingVehicles / 10.0 + departures / 20.0 - std::max(longestWait - longWait, 0.0) / 60.0;
}

double Intersection::collectReward() {
   int departures = 0;
   double longestWait = 0.0;
   for (const Road* r : inputRoads) {
      departures += r->getNumberOfDepartedVehicles();
      longestWait = std::max(longestWait, r->getHeadWaitingTime());
   }
   const double waitingVehicles =
       stepsSinceDecision > 0 ? static_cast<double>(waitingVehicleSteps) / stepsSinceDecision : 0.0;
   const int throughput = departures - lastDepartures;
   lastDepartures = departures;
   waitingVehicleSteps = 0;
   stepsSinceDecision = 0;
   return reward(waitingVehicles, throughput, longestWait);
}

void Intersection::update(const double simulationTime) {
   if (inputRoads.empty())
      return;
   // Measure of the traffic for the reward
   for (const Road* r : inputRoads)
      waitingVehicleSteps += r->getQueueLength();
   stepsSinceDecision++;

   if (simulationTime - lastDecisionTime < constants::trafficLightPeriod)
      return;
   lastDecisionTime = simulationTime;
   // Nothing to decide with one road (always green) or without controller
   if (!op || inputRoads.size() < 2)
      return;

   const Observation observation = observe(simulationTime);
   const double reward = collectReward();
   if (!lastObservation.approaches.empty() && op->isLearning())
      op->learn(lastObservation, lastAction, reward, observation);

   const int action = op->decide(observation);
   if (action >= 0 && action < static_cast<int>(inputRoads.size())) {
      if (action != currentGreenRoadIndex) {
         currentGreenRoadIndex = action;
         greenSince = simulationTime;
      }
      lastAction = action;
      lastObservation = observation;
   }
}
