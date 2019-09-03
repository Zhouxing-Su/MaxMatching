////////////////////////////////
/// usage : 1.	
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_MAX_MATCHING_LIB_MATCHING_SOLVER_H
#define SMART_SZX_GOAL_MAX_MATCHING_LIB_MATCHING_SOLVER_H


#include <algorithm>

#include "Arr.h"

#include "MaxMatchingByTopCoder.h"
#include "MinMatchingBySamHocevar.h"
#include "MinMatchingByRobertPilgrim.h"
#include "MinMatchingByJohnWeaver.h"
#include "MaxMatchingByDlib.h"
#include "MaxMatchingByLemon.h"
#include "MaxMatchingByGurobi.h"


namespace szx {

struct MatchingSolver {
    // toggle between maximization and minimization.
    static void toggleOrientationInPlace(Arr2D<int> &costMat, int maxCost) {
        for (auto i = costMat.begin(); i != costMat.end(); ++i) { *i = maxCost - *i; }
    }
    static void toggleOrientationInPlace(Arr2D<int> &costMat) {
        int maxCost = *std::max_element(costMat.begin(), costMat.end());
        toggleOrientationInPlace(costMat, maxCost);
    }
    static Arr2D<int> toggleOrientation(const Arr2D<int> &costMat, int maxCost) {
        Arr2D<int> cost(costMat);
        toggleOrientationInPlace(cost, maxCost);
        return cost;
    }
    static Arr2D<int> toggleOrientation(const Arr2D<int> &costMat) {
        int maxCost = *std::max_element(costMat.begin(), costMat.end());
        return toggleOrientation(costMat, maxCost);
    }
};

}


#endif // SMART_SZX_GOAL_MAX_MATCHING_LIB_MATCHING_SOLVER_H
