////////////////////////////////
/// usage : 1.	
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_MAX_MATCHING_BY_LEMON
#define SMART_SZX_GOAL_MAX_MATCHING_BY_LEMON

#include <type_traits>

#include "lemon/network_simplex.h"
#include "lemon/cost_scaling.h"
#include "lemon/capacity_scaling.h"
#include "lemon/cycle_canceling.h"

#include "lemon/matching.h"

#include "lemon/smart_graph.h"
#include "lemon/full_graph.h"

#include "Arr.h"


namespace szx {

// In general, NetworkSimplex and CostScaling are the most efficient implementations.
// NetworkSimplex is usually the fastest on relatively small graphs (up to several thousands of nodes) and on dense graphs,
// while CostScaling is typically more efficient on large graphs (e.g. hundreds of thousands of nodes or above), especially if they are sparse.
// However, other algorithms could be faster in special cases. For example, if the total supply and/or capacities are rather small, CapacityScaling is usually the fastest algorithm (without effective scaling).
//
// These classes are intended to be used with integer - valued input data (capacities, supply values, and costs),
// except for CapacityScaling, which is capable of handling real - valued arc costs(other numerical data are required to be integer).
struct MaxMatchingByLemon { // http://lemon.cs.elte.hu/trac/lemon
    MaxMatchingByLemon(const Arr2D<int> &costMat)
        : cost(costMat), assignment(costMat.size1()) {}

    template<template<typename GR, typename WM> typename Algorithm>
    const Arr<int>& solveOnSmartBpGraph() {
        using Graph = typename lemon::SmartBpGraph;
        using EdgeMap = typename Graph::template EdgeMap<int>;

        int rowNum = cost.size1();
        int colNum = cost.size2();

        if (std::is_same<lemon::MaxWeightedPerfectMatching<Graph, EdgeMap>, Algorithm<Graph, EdgeMap>>::value) {
            if (rowNum != colNum) { return assignment; }
        }
        //if (rowNum > colNum) { return assignment; }

        Graph g;
        g.reserveNode(rowNum + colNum);
        for (int i = 0; i < rowNum; ++i) { g.addRedNode(); }
        for (int j = 0; j < colNum; ++j) { g.addBlueNode(); }
        g.reserveEdge(rowNum * colNum);
        for (int i = 0; i < rowNum; ++i) {
            for (int j = 0; j < colNum; ++j) {
                g.addEdge(g.asRedNodeUnsafe(g.nodeFromId(i)), g.asBlueNodeUnsafe(g.nodeFromId(rowNum + j)));
            }
        }

        EdgeMap weights(g);
        auto c = cost.begin();
        for (int i = 0; i < rowNum; ++i) {
            for (int j = 0; j < colNum; ++j, ++c) {
                weights[g.edgeFromId(i* colNum + j)] = *c;
            }
        }

        Algorithm<Graph, EdgeMap> alg(g, weights);
        alg.run();

        for (int i = 0; i < rowNum; ++i) {
            assignment[i] = g.id(alg.mate(g.asRedNodeUnsafe(g.nodeFromId(i)))) - rowNum;
        }

        return assignment;
    }

    template<template <typename G, typename M> class Algorithm>
    const Arr<int>& solveMwmOnFullBpGraph() {
        using Graph = typename lemon::FullBpGraph;
        using EdgeMap = typename Graph::template EdgeMap<int>;

        int rowNum = cost.size1();
        int colNum = cost.size2();

        if (std::is_same<lemon::MaxWeightedPerfectMatching<Graph, EdgeMap>, Algorithm<Graph, EdgeMap>>::value) {
            if (rowNum != colNum) { return assignment; }
        }
        //if (rowNum > colNum) { return assignment; }

        Graph g(rowNum, colNum);
        EdgeMap weights(g);
        auto c = cost.begin();
        for (int i = 0; i < rowNum; ++i) {
            for (int j = 0; j < colNum; ++j, ++c) {
                weights[g.edge(g.nodeFromId(i), g.nodeFromId(rowNum + j))] = *c;
            }
        }

        Algorithm<Graph, EdgeMap> alg(g, weights);
        alg.run();

        for (int i = 0; i < rowNum; ++i) {
            assignment[i] = g.id(alg.mate(g.redNode(i))) - rowNum;
        }

        return assignment;
    }

    template<template <typename G, typename C, typename W, typename ...> class Algorithm>
    const Arr<int>& solveMcfOnSmartDigraph() {
        using Weight = int;
        using Capacity = int;
        using Graph = typename lemon::SmartDigraph;
        using NodeMap = typename Graph::template NodeMap<int>;
        using ArcMap = typename Graph::template ArcMap<int>;
        using Alg = Algorithm<Graph, Capacity, Weight>;

        int rowNum = cost.size1();
        int colNum = cost.size2();

        //if (rowNum > colNum) { return assignment; }

        Graph g;
        auto l = [&](int i) { return g.nodeFromId(i); };
        auto r = [&](int j) { return g.nodeFromId(rowNum + j); };
        auto arc = [&](int src, int dst) { return g.arcFromId(src * colNum + dst); };
        g.reserveNode(rowNum + colNum);
        for (int i = 0; i < rowNum; ++i) { g.addNode(); }
        for (int j = 0; j < colNum; ++j) { g.addNode(); }
        g.reserveArc(rowNum * colNum);
        for (int i = 0; i < rowNum; ++i) {
            for (int j = 0; j < colNum; ++j) { g.addArc(l(i), r(j)); }
        }

        NodeMap supplies(g);
        for (int i = 0; i < rowNum; ++i) { supplies[g.nodeFromId(i)] = 1; }
        for (int j = rowNum; j < rowNum + colNum; ++j) { supplies[g.nodeFromId(j)] = -1; }

        ArcMap weights(g);
        ArcMap capacities(g);
        for (int i = 0; i < rowNum; ++i) {
            for (int j = 0; j < colNum; ++j) {
                weights[arc(i, j)] = cost[i][j];
                capacities[arc(i, j)] = 1;
            }
        }

        Alg alg(g);
        alg.costMap(weights).upperMap(capacities).supplyMap(supplies);

        if (alg.run() == Alg::OPTIMAL) {
            for (int i = 0; i < rowNum; ++i) {
                for (int j = 0; j < colNum; ++j) {
                    if (alg.flow(arc(i, j)) > 0) { assignment[i] = j; break; }
                }
            }
        }

        return assignment;
    }

    virtual const Arr<int>& solve() {
        return solveOnSmartBpGraph<lemon::MaxWeightedPerfectMatching>();
        //return solveOnSmartBpGraph<lemon::MaxWeightedMatching>();

        //return solveMwmOnFullBpGraph<lemon::MaxWeightedPerfectMatching>();
        //return solveMwmOnFullBpGraph<lemon::MaxWeightedMatching>();

        //return solveMcfOnSmartDigraph<lemon::NetworkSimplex>();
        //return solveMcfOnSmartDigraph<lemon::CostScaling>();
        //return solveMcfOnSmartDigraph<lemon::CapacityScaling>();
        //return solveMcfOnSmartDigraph<lemon::CycleCanceling>();
    }


protected:
    Arr2D<int> cost;
    Arr<int> assignment;
};

struct MaxMatchingByLemonPerfectMatchingOnSmartBpGraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveOnSmartBpGraph<lemon::MaxWeightedPerfectMatching>();
    }
};

struct MaxMatchingByLemonOnSmartBpGraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveOnSmartBpGraph<lemon::MaxWeightedMatching>();
    }
};

struct MaxMatchingByLemonPerfectMatchingOnFullBpGraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveMwmOnFullBpGraph<lemon::MaxWeightedPerfectMatching>();
    }
};

struct MaxMatchingByLemonOnFullBpGraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveMwmOnFullBpGraph<lemon::MaxWeightedMatching>();
    }
};

struct MinMatchingByLemonNetworkSimplexOnSmartDigraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveMcfOnSmartDigraph<lemon::NetworkSimplex>();
    }
};

struct MinMatchingByLemonCostScalingOnSmartDigraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveMcfOnSmartDigraph<lemon::CostScaling>();
    }
};

struct MinMatchingByLemonCapacityScalingOnSmartDigraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveMcfOnSmartDigraph<lemon::CapacityScaling>();
    }
};

struct MinMatchingByLemonCycleCancelingOnSmartDigraph : public MaxMatchingByLemon {
    using MaxMatchingByLemon::MaxMatchingByLemon;

    virtual const Arr<int>& solve() {
        return solveMcfOnSmartDigraph<lemon::CycleCanceling>();
    }
};

}


#endif // SMART_SZX_GOAL_MAX_MATCHING_BY_LEMON
