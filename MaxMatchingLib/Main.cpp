#include <iostream>
#include <iomanip>
#include <chrono>
#include <typeinfo>

#include <cstdlib>

#include "Arr.h"
#include "ColorStr.h"

#include "MatchingSolver.h"


using namespace std;
using namespace szx;


enum { MaxCost = 1000 };

void printSln(const Arr2D<int> &cost, const Arr<int> &assignment) {
    int obj = 0;
    for (int i = 0; i < cost.size1(); ++i) {
        //for (int j = 0; j < cost.size2(); ++j) {
        //    if (assignment[i] == j) { cerr << ColorStr::CmdColor::BrightRedFG; }
        //    cerr << cost[i][j] << " " << ColorStr::CmdColor::Reset;
        //}
        //cerr << endl;
        if (assignment[i] > 0) { obj += cost[i][assignment[i]]; }
    }
    cout << " | " << setw(7) << obj << " |" << endl;
}

Arr2D<int> generateSquareInstance(int n, int m, int seed = 0) {
    if (n < m) { throw "n < m is not allowed."; }
    srand(seed);
    Arr2D<int> cost(n, n); cost.reset(Arr2D<int>::ResetOption::AllBits0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            cost[i][j] = rand() % MaxCost;
        }
    }
    return cost;
}

template<typename MaxMatching>
void test(int n, int m, bool toggle = false) {
    Arr2D<int> cost(generateSquareInstance(n, m));

    chrono::steady_clock::time_point start = chrono::steady_clock::now();

    MaxMatching mm(toggle ? MatchingSolver::toggleOrientation(cost, MaxCost) : cost);
    const Arr<int> &assignment(mm.solve());

    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "| " << setw(50) << (typeid(MaxMatching).name() + 12) << " | "
        << setw(6) << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms";

    printSln(cost, assignment);
}

int main() {
    int n = 1000;
    int m = 1000;

    cout << "| MaxMatching test                                   | time      | obj     |" << endl;
    cout << "| -------------------------------------------------- | --------- | ------- |" << endl;

    test<MaxMatchingByTopCoder>(n, m);
    test<MinMatchingBySamHocevar>(n, m, true);
    //test<MinMatchingByRobertPilgrim>(n, m, true); // too slow.
    //test<MinMatchingByJohnWeaver>(n, m, true); // too slow.
    test<MaxMatchingByDlib>(n, m);
    //test<MaxMatchingByLemon>(n, m);
    test<MaxMatchingByLemonPerfectMatchingOnSmartBpGraph>(n, m);
    test<MaxMatchingByLemonOnSmartBpGraph>(n, m);
    test<MaxMatchingByLemonPerfectMatchingOnFullBpGraph>(n, m);
    test<MaxMatchingByLemonOnFullBpGraph>(n, m);
    test<MinMatchingByLemonNetworkSimplexOnSmartDigraph>(n, m, true);
    test<MinMatchingByLemonCostScalingOnSmartDigraph>(n, m, true);
    test<MinMatchingByLemonCapacityScalingOnSmartDigraph>(n, m, true);
    test<MinMatchingByLemonCycleCancelingOnSmartDigraph>(n, m, true);
    test<MaxMatchingByGurobi>(n, m);

    cout << endl;
    cout << "| MinMatching test                                   | time      | obj     |" << endl;
    cout << "| -------------------------------------------------- | --------- | ------- |" << endl;

    test<MaxMatchingByTopCoder>(n, m, true);
    test<MinMatchingBySamHocevar>(n, m);
    //test<MinMatchingByRobertPilgrim>(n, m); // too slow.
    //test<MinMatchingByJohnWeaver>(n, m); // too slow.
    test<MaxMatchingByDlib>(n, m, true);
    //test<MaxMatchingByLemon>(n, m, true);
    test<MaxMatchingByLemonPerfectMatchingOnSmartBpGraph>(n, m, true);
    test<MaxMatchingByLemonOnSmartBpGraph>(n, m, true);
    test<MaxMatchingByLemonPerfectMatchingOnFullBpGraph>(n, m, true);
    test<MaxMatchingByLemonOnFullBpGraph>(n, m, true);
    test<MinMatchingByLemonNetworkSimplexOnSmartDigraph>(n, m);
    test<MinMatchingByLemonCostScalingOnSmartDigraph>(n, m);
    test<MinMatchingByLemonCapacityScalingOnSmartDigraph>(n, m);
    test<MinMatchingByLemonCycleCancelingOnSmartDigraph>(n, m);
    test<MaxMatchingByGurobi>(n, m, true);

    return 0;
}
