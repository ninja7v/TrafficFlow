// Libraries
#include <list>   // To use lists
#include <vector> // To use lists
#include <memory> // To use smart pointers
#include <limits> // To use infinity
#include <algorithm> // To use max
// Header files
#include "../headers/Map.h"

namespace {
   const double INF_VAL = std::numeric_limits<double>::infinity();

   /** Unvisited intersection with the smallest distance ('fallback' if they are all visited). */
   int closestUnvisited(const double* distance, const int* visited, const int fallback) {
      int closest = fallback;
      double mindistance = INF_VAL;
      for (int i = 0; i < constants::nbIntersections; i++)
         if (distance[i] <= mindistance && !visited[i]) {
            mindistance = distance[i];
            closest = i;
         }
      return closest;
   }

   /** Intersection a vehicle arriving at 'begin' by 'arrivedBy' must not go straight back to. */
   int uTurnNode(const Intersection* begin, const Road* arrivedBy) {
      return (arrivedBy && arrivedBy->getEnd() == begin) ? arrivedBy->getStart()->getID() : -1;
   }
}

decltype(Map::connections) Map::connections{};
std::vector<std::vector <double>> Map::cost = { constants::nbIntersections, std::vector<double>(constants::nbIntersections, INF_VAL) };
std::vector<std::vector <double>> Map::live_cost = { constants::nbIntersections, std::vector<double>(constants::nbIntersections, INF_VAL) };

Map::Map() {
   connections = {};
   cost.assign(constants::nbIntersections,
               std::vector<double>(constants::nbIntersections, INF_VAL));
   live_cost = cost;
}

std::list<Road*> Map::track(const Intersection* begin,
                            const Intersection* end,
                            const Road* arrivedBy) {
   std::list<Road*> path;
   if (!begin || !end)
      return path;
   const int uTurn = uTurnNode(begin, arrivedBy);

   double distance[constants::nbIntersections];
   int pred[constants::nbIntersections];
   int visited[constants::nbIntersections];

   // Parameters initialization from start
   for (int i = 0; i < constants::nbIntersections; i++) {
      distance[i] = i == uTurn ? INF_VAL : live_cost[begin->getID()][i];
      pred[i] = begin->getID();
      visited[i] = 0;
   }
   distance[begin->getID()] = 0;
   visited[begin->getID()] = 1;
   // Find shortest path
   int nextnode = -1;
   double mindistance;
   for (int j = 1; j < constants::nbIntersections; j ++) {
      nextnode = closestUnvisited(distance, visited, nextnode);
      if (nextnode == -1) break;
      mindistance = distance[nextnode];
      visited[nextnode] = 1;
      for (int i = 0; i < constants::nbIntersections; i++)
         if (!visited[i])
            if (mindistance + live_cost[nextnode][i] < distance[i]) {
               distance[i] = mindistance + live_cost[nextnode][i];
               pred[i] = nextnode;
            }
   }
   // Affect result
   if (distance[end->getID()] == INF_VAL) // Not reachable
      return uTurn != -1 ? track(begin, end) /* Dead end: the U-turn is the only way */ : path;
   for (int i = end->getID(); i != begin->getID(); i = pred[i])
      path.push_front(connections[pred[i]][i]);
   return path;
}

Road* Map::getConnection(const int a,
                         const int b) const {
   return ((a >= 0) && (b >= 0) && (a < constants::nbIntersections) && (b < constants::nbIntersections)) ? connections[a][b] : nullptr;
}

void Map::setConnection(const int a,
                        const int b,
                        Road* r) {
   if (r && (a >= 0) && (b >= 0) && (a < constants::nbIntersections) && (b < constants::nbIntersections)) {
      connections[a][b] = r;
      cost[a][b] = r->getLength();
      live_cost[a][b] = cost[a][b]; // Usable by track() before the first live update
   }
}

void Map::updateConnection(const std::unique_ptr<Road>& r) {
   if (r) {
      const int a = r->getStart()->getID();
      const int b = r->getEnd()->getID();
      // The closer the last vehicle is to the start, the fuller the road. The free distance is
      // clamped so a vehicle that just entered does not make the road cost infinite (unreachable).
      constexpr double minFreeDistance = 0.1;
      const double penalty = !r->containVehicle() ? 0.0 :
         std::max(cost[a][b] / std::max(r->getVehicles().back()->distance(r->getStart()), minFreeDistance) - 1.0, 0.0);
      live_cost[a][b] = cost[a][b] + penalty;
   }
}
