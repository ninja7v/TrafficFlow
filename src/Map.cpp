// Libraries
#include <list>   // To use lists
#include <vector> // To use lists
#include <memory> // To use smart pointers
#include <limits> // To use infinity
// Header files
#include "../headers/Map.h"

namespace {
   const double INF_VAL = std::numeric_limits<double>::infinity();
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
                            const Intersection* end) {
   std::list<Road*> path;
   if (!begin || !end)
      return path;

   double distance[constants::nbIntersections];
   int pred[constants::nbIntersections];
   int visited[constants::nbIntersections];

   // Parameters initialization from start
   for (int i = 0; i < constants::nbIntersections; i++) {
      distance[i] = live_cost[begin->getID()][i];
      pred[i] = begin->getID();
      visited[i] = 0;
   }
   distance[begin->getID()] = 0;
   visited[begin->getID()] = 1;
   // Find shortest path
   int nextnode = -1;
   double mindistance;
   for (int j = 1; j < constants::nbIntersections; j ++) {
      mindistance = INF_VAL;
      for (int i = 0; i < constants::nbIntersections; i++)
         if (distance[i] <= mindistance && !visited[i]) {
            mindistance = distance[i];
            nextnode = i;
         }
      if (nextnode == -1) break;
      visited[nextnode] = 1;
      for (int i = 0; i < constants::nbIntersections; i++)
         if (!visited[i])
            if (mindistance + live_cost[nextnode][i] < distance[i]) {
               distance[i] = mindistance + live_cost[nextnode][i];
               pred[i] = nextnode;
            }
   }
   // Affect result
   if (distance[end->getID()] != INF_VAL) { // Check if reachable
      int i = end->getID();
      while (i != begin->getID()) {
         path.push_front(connections[pred[i]][i]);
         i = pred[i];
      }
   }
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
   }
}

void Map::updateConnection(const std::unique_ptr<Road>& r) {
   if (r) {
      const double penalty = !r->containVehicle() ? 0.0 :
         cost[r->getStart()->getID()][r->getEnd()->getID()] / r->getVehicles().back()->distance(r->getStart()) - 1.0;
      live_cost[r->getStart()->getID()][r->getEnd()->getID()] = cost[r->getStart()->getID()][r->getEnd()->getID()] + penalty;
   }
}
