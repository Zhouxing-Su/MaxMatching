# GOAL.MaxMatching

## Introduction

a C++ wrapper for efficient maximum matching solvers.


## Directory Structure

- **MaxMatchingLib/** the C++ wrapper for TSM.
  - **MatchingSolver.h** includes all implementations and provides common utilities.
  - **Main.cpp** the sample code for invoking the library.
  - **MaxMatchingByXXX.cpp** the refactored C++ versions of different algorithm implementations.
- **Doc/** the relating papers.


## Instruction

- add the source files except `Main.cpp` under `MaxMatchingLib/` into your project.
- include the `MatchingSolver.h` in your code.


## Benchmark Results

### N=1000, M=1000, MaxCost=1000, x86

| MaxMatching test                                   | time      | obj     |
| -------------------------------------------------- | --------- | ------- |
|                              MaxMatchingByTopCoder |     98 ms |  996824 |
|                            MinMatchingBySamHocevar |   1974 ms |  996824 |
|                         MinMatchingByRobertPilgrim |  20374 ms |  996824 |
|                            MinMatchingByJohnWeaver |  18072 ms |  996824 |
|                                  MaxMatchingByDlib |     76 ms |  996824 |
|    MaxMatchingByLemonPerfectMatchingOnSmartBpGraph |   1771 ms |  996824 |
|                   MaxMatchingByLemonOnSmartBpGraph |   1780 ms |  996824 |
|     MaxMatchingByLemonPerfectMatchingOnFullBpGraph |    588 ms |  996824 |
|                    MaxMatchingByLemonOnFullBpGraph |    576 ms |  996824 |
|     MinMatchingByLemonNetworkSimplexOnSmartDigraph |    194 ms |  996824 |
|        MinMatchingByLemonCostScalingOnSmartDigraph |    750 ms |  996824 |
|    MinMatchingByLemonCapacityScalingOnSmartDigraph |    252 ms |  996824 |
|     MinMatchingByLemonCycleCancelingOnSmartDigraph |   7486 ms |  996824 |

| MinMatching test                                   | time      | obj     |
| -------------------------------------------------- | --------- | ------- |
|                              MaxMatchingByTopCoder |     61 ms |    1129 |
|                            MinMatchingBySamHocevar |   1931 ms |    1129 |
|                         MinMatchingByRobertPilgrim |  21314 ms |    1129 |
|                            MinMatchingByJohnWeaver |  18710 ms |    1129 |
|                                  MaxMatchingByDlib |     69 ms |    1129 |
|    MaxMatchingByLemonPerfectMatchingOnSmartBpGraph |   1729 ms |    1129 |
|                   MaxMatchingByLemonOnSmartBpGraph |   1701 ms |    1129 |
|     MaxMatchingByLemonPerfectMatchingOnFullBpGraph |    599 ms |    1129 |
|                    MaxMatchingByLemonOnFullBpGraph |    568 ms |    1129 |
|     MinMatchingByLemonNetworkSimplexOnSmartDigraph |    204 ms |    1129 |
|        MinMatchingByLemonCostScalingOnSmartDigraph |    723 ms |    1129 |
|    MinMatchingByLemonCapacityScalingOnSmartDigraph |    250 ms |    1129 |
|     MinMatchingByLemonCycleCancelingOnSmartDigraph |   8881 ms |    1129 |

### N=1000, M=1000, MaxCost=1000, x64

| MaxMatching test                                   | time      | obj     |
| -------------------------------------------------- | --------- | ------- |
|                              MaxMatchingByTopCoder |     73 ms |  996824 |
|                            MinMatchingBySamHocevar |   2018 ms |  996824 |
|                         MinMatchingByRobertPilgrim |  16534 ms |  996824 |
|                            MinMatchingByJohnWeaver |  17672 ms |  996824 |
|                                  MaxMatchingByDlib |     58 ms |  996824 |
|    MaxMatchingByLemonPerfectMatchingOnSmartBpGraph |   1751 ms |  996824 |
|                   MaxMatchingByLemonOnSmartBpGraph |   1741 ms |  996824 |
|     MaxMatchingByLemonPerfectMatchingOnFullBpGraph |    555 ms |  996824 |
|                    MaxMatchingByLemonOnFullBpGraph |    526 ms |  996824 |
|     MinMatchingByLemonNetworkSimplexOnSmartDigraph |    218 ms |  996824 |
|        MinMatchingByLemonCostScalingOnSmartDigraph |    539 ms |  996824 |
|    MinMatchingByLemonCapacityScalingOnSmartDigraph |    232 ms |  996824 |
|     MinMatchingByLemonCycleCancelingOnSmartDigraph |   7368 ms |  996824 |
|                                MaxMatchingByGurobi |   7534 ms |  996824 |

| MinMatching test                                   | time      | obj     |
| -------------------------------------------------- | --------- | ------- |
|                              MaxMatchingByTopCoder |     53 ms |    1129 |
|                            MinMatchingBySamHocevar |   2420 ms |    1129 |
|                         MinMatchingByRobertPilgrim |  17754 ms |    1129 |
|                            MinMatchingByJohnWeaver |  18405 ms |    1129 |
|                                  MaxMatchingByDlib |     60 ms |    1129 |
|    MaxMatchingByLemonPerfectMatchingOnSmartBpGraph |   1720 ms |    1129 |
|                   MaxMatchingByLemonOnSmartBpGraph |   1689 ms |    1129 |
|     MaxMatchingByLemonPerfectMatchingOnFullBpGraph |    564 ms |    1129 |
|                    MaxMatchingByLemonOnFullBpGraph |    598 ms |    1129 |
|     MinMatchingByLemonNetworkSimplexOnSmartDigraph |    233 ms |    1129 |
|        MinMatchingByLemonCostScalingOnSmartDigraph |    529 ms |    1129 |
|    MinMatchingByLemonCapacityScalingOnSmartDigraph |    219 ms |    1129 |
|     MinMatchingByLemonCycleCancelingOnSmartDigraph |   8044 ms |    1129 |
|                                MaxMatchingByGurobi |   6738 ms |    1129 |


## Reference

- https://www.topcoder.com/community/competitive-programming/tutorials/assignment-problem-and-hungarian-algorithm/
- https://github.com/maandree/hungarian-algorithm-n3
- http://csclab.murraystate.edu/~bob.pilgrim/445/munkres.html
- https://github.com/saebyn/munkres-cpp
- http://dlib.net/optimization.html#max_cost_assignment
- http://lemon.cs.elte.hu/trac/lemon
- http://www.gurobi.com
