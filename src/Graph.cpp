// Libraries
#include <functional> // To use std::function
#include <random>     // To use distributions
// Headers
#include "../headers/Graph.h"

namespace graph {

std::vector<int> stronglyConnectedComponents(const int nbNodes, const std::vector<Edge>& edges) {
   std::vector<std::vector<int>> forward(nbNodes), backward(nbNodes);
   for (const auto& [a, b] : edges) {
      forward[a].push_back(b);
      backward[b].push_back(a);
   }
   // 1st pass: order nodes by finishing time
   std::vector<bool> visited(nbNodes, false);
   std::vector<int> order;
   std::function<void(int)> visit = [&](const int u) {
      visited[u] = true;
      for (const int v : forward[u])
         if (!visited[v])
            visit(v);
      order.push_back(u);
   };
   for (int u = 0; u < nbNodes; ++u)
      if (!visited[u])
         visit(u);
   // 2nd pass: on the transposed graph, in reverse finishing order
   std::vector<int> component(nbNodes, -1);
   std::function<void(int, int)> assign = [&](const int u, const int c) {
      component[u] = c;
      for (const int v : backward[u])
         if (component[v] == -1)
            assign(v, c);
   };
   int nbComponents = 0;
   for (auto it = order.rbegin(); it != order.rend(); ++it)
      if (component[*it] == -1)
         assign(*it, nbComponents++);
   return component;
}

bool isStronglyConnected(const int nbNodes, const std::vector<Edge>& edges) {
   const std::vector<int> component = stronglyConnectedComponents(nbNodes, edges);
   for (const int c : component)
      if (c != 0)
         return false;
   return true;
}

std::vector<Edge> orientEdges(const int nbNodes,
                              const std::vector<Edge>& undirectedEdges,
                              const double oneWayRatio,
                              std::mt19937& rng) {
   std::bernoulli_distribution isOneWay(oneWayRatio);
   std::bernoulli_distribution isReversed(0.5);
   // twoWay[k]: edge k is two-way; otherwise oneWay[k] is its only direction
   std::vector<bool> twoWay(undirectedEdges.size());
   std::vector<Edge> oneWay(undirectedEdges.size());
   for (size_t k = 0; k < undirectedEdges.size(); ++k) {
      const auto& [a, b] = undirectedEdges[k];
      twoWay[k] = !isOneWay(rng);
      oneWay[k] = isReversed(rng) ? Edge{b, a} : Edge{a, b};
   }
   auto directedEdges = [&]() {
      std::vector<Edge> edges;
      for (size_t k = 0; k < undirectedEdges.size(); ++k) {
         edges.push_back(oneWay[k]);
         if (twoWay[k])
            edges.push_back({oneWay[k].second, oneWay[k].first});
      }
      return edges;
   };
   // Repair: a one-way edge a->b between 2 components becomes two-way. As a reaches b,
   // adding b->a merges both components, so this ends after at most nbNodes-1 iterations.
   std::vector<Edge> edges = directedEdges();
   std::vector<int> component = stronglyConnectedComponents(nbNodes, edges);
   bool changed = true;
   while (changed) {
      changed = false;
      std::vector<size_t> candidates;
      for (size_t k = 0; k < undirectedEdges.size(); ++k)
         if (!twoWay[k] && component[oneWay[k].first] != component[oneWay[k].second])
            candidates.push_back(k);
      if (!candidates.empty()) {
         std::uniform_int_distribution<size_t> pick(0, candidates.size() - 1);
         twoWay[candidates[pick(rng)]] = true;
         edges = directedEdges();
         component = stronglyConnectedComponents(nbNodes, edges);
         changed = true;
      }
   }
   return edges;
}

} // namespace graph
