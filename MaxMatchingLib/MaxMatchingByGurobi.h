////////////////////////////////
/// usage : 1.	
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_MAX_MATCHING_BY_GUROBI_H
#define SMART_SZX_GOAL_MAX_MATCHING_BY_GUROBI_H


#include "gurobi/gurobi_c++.h"

#include "Arr.h"


// if there is "#define x  y", VERBATIM_STRINGIFY(x) will get "x".
#define VERBATIM_STRINGIFY(x)  #x
// if there is "#define x  y", RESOLVED_STRINGIFY(x) will get "y".
#define RESOLVED_STRINGIFY(x)  VERBATIM_STRINGIFY(x)

#define VERBATIM_CONCAT(a, b)  a##b
#define VERBATIM_CONCAT2(a, b, c)  a##b##c
#define VERBATIM_CONCAT3(a, b, c, d)  a##b##c##d
#define RESOLVED_CONCAT(a, b)  VERBATIM_CONCAT(a, b)
#define RESOLVED_CONCAT2(a, b, c)  VERBATIM_CONCAT2(a, b, c)
#define RESOLVED_CONCAT3(a, b, c, d)  VERBATIM_CONCAT3(a, b, c, d)


#pragma region AutoLinking
#pragma region LinkLibraryCheck
#if (_DEBUG || DEBUG) && !(NDEBUG || _NDEBUG || RELEASE || _RELEASE) // prefer release when both are (un)defined.
    #if (_DLL || _SHARED) && !(_STATIC) // prefer static when both are (un)defined.
        #define LINK_TYPE  "mdd"
    #else
        #define LINK_TYPE  "mtd"
    #endif // _DLL
#else
    #if (_DLL || _SHARED) && !(_STATIC) // prefer static when both are (un)defined.
        #define LINK_TYPE  "md"
    #else
        #define LINK_TYPE  "mt"
    #endif // _DLL
#endif // _DEBUG
#pragma endregion LinkLibraryCheck

#pragma comment(lib, "gurobi/gurobi80")
#pragma comment(lib, "gurobi/gurobi_c++" LINK_TYPE "2017")
#pragma endregion AutoLinking


namespace szx {

struct MaxMatchingByGurobi { // http://www.gurobi.com
    MaxMatchingByGurobi(const Arr2D<int> &costMat)
        : cost(costMat), assignment(costMat.size1()) {}


    const Arr<int>& solve() { // maximize the total cost.
        thread_local static GRBEnv env;

        int rowNum = cost.size1();
        int colNum = cost.size2();

        GRBModel model(env);

        Arr2D<GRBVar> x(rowNum, colNum);
        for (int i = 0; i < rowNum; ++i) {
            for (int j = 0; j < colNum; ++j) {
                x.at(i, j) = model.addVar(0, 1, cost.at(i, j), GRB_BINARY);
            }
        }

        for (int i = 0; i < rowNum; ++i) {
            GRBLinExpr sum;
            for (int j = 0; j < colNum; ++j) { sum += x.at(i, j); }
            model.addConstr(sum <= 1);
        }
        for (int j = 0; j < colNum; ++j) {
            GRBLinExpr sum;
            for (int i = 0; i < rowNum; ++i) { sum += x.at(i, j); }
            model.addConstr(sum <= 1);
        }

        model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
        model.set(GRB_IntParam_OutputFlag, false);

        model.optimize();

        if (model.get(GRB_IntAttr_SolCount) > 0) {
            for (int i = 0; i < rowNum; ++i) {
                for (int j = 0; j < colNum; ++j) {
                    if (x.at(i, j).get(GRB_DoubleAttr_X) > 0.5) {
                        assignment[i] = j;
                        break;;
                    }
                }
            }
        }
        
        return assignment;
    }


protected:
    Arr2D<int> cost;
    Arr<int> assignment;
};

}


#endif // SMART_SZX_GOAL_MAX_MATCHING_BY_GUROBI_H
