/**
 * @brief Graph helpers used to build the road network.
 */

#pragma once
// Libraries
#include <random>  // To use std::mt19937
#include <utility> // To use std::pair
#include <vector>  // To use vectors

namespace graph {
   /** Edge between two nodes (directed: first -> second). */
   using Edge = std::pair<int, int>;

   /** @brief Strongly connected components (Kosaraju).
    * @param nbNodes Number of nodes, identified by 0..nbNodes-1
    * @param edges Directed edges
    * @returns Component index of each node */
   std::vector<int> stronglyConnectedComponents(int nbNodes, const std::vector<Edge>& edges);

   /** @brief Check that every node can reach every other node.
    * @param nbNodes Number of nodes
    * @param edges Directed edges
    * @returns Strongly connected? */
   bool isStronglyConnected(int nbNodes, const std::vector<Edge>& edges);

   /** @brief Turn undirected edges into one-way and two-way roads.
    * Each edge is one-way (random direction) with probability oneWayRatio, two-way otherwise.
    * One-way edges are then turned into two-way ones, only where needed, until the graph is
    * strongly connected, so a path exists between any 2 nodes.
    * Condition: the undirected graph must be connected.
    * @param nbNodes Number of nodes
    * @param undirectedEdges Edges, each listed once
    * @param oneWayRatio Probability for an edge to be one-way, in [0, 1]
    * @param rng Random number generator
    * @returns Directed edges (a two-way edge gives 2 directed edges) */
   std::vector<Edge> orientEdges(int nbNodes,
                                 const std::vector<Edge>& undirectedEdges,
                                 double oneWayRatio,
                                 std::mt19937& rng);
}
