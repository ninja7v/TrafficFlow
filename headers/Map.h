/**
 * @brief Map of the Network.
 */

#pragma once
// Libraries
#include <list>   // To use lists
#include <vector> // To use vectors
// Header files
#include "Intersection.h"
#include "Constants.h"
#include "Road.h"

class Map {
public:
   /** @brief Default constructor.
    * @returns Map*/
   Map();
   /** @brief Obtain the optimal itinerary between 2 intersections using the Dijksra Algorithm.
    * Source: https://www.tutorialspoint.com/cplusplus-program-for-dijkstra-s-shortest-path-algorithm
    * Condition: the graph must be strongly connected to ensure the existence of a solution.
    * @param begin Start Intersection
    * @param end Arrival Intersection
    * @param arrivedBy Road the vehicle is leaving (it ends at 'begin'), nullptr if none. The
    * itinerary does not start with a U-turn on this road, unless there is no other way:
    * vehicles sent back where they come from block each other on a congested two-way road.
    * @returns Road */
   static std::list<Road*> track(const Intersection* begin,
                                 const Intersection* end,
                                 const Road* arrivedBy = nullptr);
   /** @brief Getter to obtain the road between 2 intersections.
    * @param a Start Intersection ID
    * @param b Arrival Intersection ID
    * @returns Road */
   Road* getConnection(const int a,
                       const int b) const;
   /** @brief Setter to build the road between 2 intersections.
    * @param a Start Intersection ID
    * @param b Arrival Intersection ID
    * @param r New Road */
   void setConnection(const int a,
                      const int b,
                      Road* r);
   /** @brief Update the live connection cost.
    * @param r Road */
   void updateConnection(const std::unique_ptr<Road>& r);

protected:

private:
   /** Adjacency matrix for the roads. Filled in the Network contructor. */
   static std::array<std::array<Road*, constants::nbIntersections>, constants::nbIntersections> connections;
   /** Adjacency matrix for the distances. Filled in the Network contructor. */
   static std::vector<std::vector <double>> cost;
   /** Adjacency matrix for the global cost (distance + traffic). Updated in Network::displayNetwork() */
   static std::vector<std::vector <double>> live_cost;
};
