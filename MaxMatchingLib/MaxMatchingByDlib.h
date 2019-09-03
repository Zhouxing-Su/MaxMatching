////////////////////////////////
/// usage : 1.	
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_MAX_MATCHING_BY_DLIB_H
#define SMART_SZX_GOAL_MAX_MATCHING_BY_DLIB_H


#include <vector>
#include <deque>

#include "Arr.h"


namespace szx {

struct MaxMatchingByDlib {
    MaxMatchingByDlib(const Arr2D<int> &costMat) : cost(costMat),
        lx(cost.nc()), ly(cost.nc()), xy(cost.nc()), yx(cost.nc()), S(cost.nc()), T(cost.nc()),
        slack(cost.nc()), slackx(cost.nc()), aug_path(cost.nc()) {}

    const Arr<int>& solve() { // maximize the total cost.
        if (cost.size1() != cost.size2()) { return xy; }
        max_cost_assignment();
        return xy;
    }

    Arr2D<int>& costs() { return cost; }
    int& costs(int r, int c) { return cost[r][c]; }


protected:
    template<typename T, typename IndexType = int>
    struct matrix : public Arr2D<T, IndexType> {
        using type = T;

        using Arr2D<T, IndexType>::Arr2D;
        matrix(const Arr2D<T, IndexType> &mat) : Arr2D<T, IndexType>(mat) {}

        T& operator()(IndexType i1, IndexType i2) { return Arr2D<T, IndexType>::at(i1, i2); }
        const T& operator()(IndexType i1, IndexType i2) const { return Arr2D<T, IndexType>::at(i1, i2); }

        int nr() const { return Arr2D<T, IndexType>::size1(); }
        int nc() const { return Arr2D<T, IndexType>::size2(); }
    };

    using EXP = matrix<int>;

    template<typename T>
    using matrix_exp = matrix<typename T::type>;

    using type = int;


    matrix<int> cost;

    Arr<type> lx, ly;
    Arr<int> xy;
    Arr<int> yx;
    Arr<char> S, T;
    Arr<type> slack;
    Arr<int> slackx;
    Arr<int> aug_path;

    //template <typename EXP>
    //typename EXP::type assignment_cost(
    //    const matrix_exp<EXP>& cost,
    //    const std::vector<long>& assignment
    //) {
    //    DLIB_ASSERT(cost.nr() == cost.nc(),
    //        "\t type assignment_cost(cost,assignment)"
    //        << "\n\t cost.nr(): " << cost.nr()
    //        << "\n\t cost.nc(): " << cost.nc()
    //    );
    //    #ifdef ENABLE_ASSERTS
    //            // can't call max on an empty vector. So put an if here to guard against it.
    //    if (assignment.size() > 0) {
    //        DLIB_ASSERT(0 <= min(mat(assignment)) && max(mat(assignment)) < cost.nr(),
    //            "\t type assignment_cost(cost,assignment)"
    //            << "\n\t cost.nr(): " << cost.nr()
    //            << "\n\t cost.nc(): " << cost.nc()
    //            << "\n\t min(assignment): " << min(mat(assignment))
    //            << "\n\t max(assignment): " << max(mat(assignment))
    //        );
    //    }
    //    #endif

    //    typename EXP::type temp = 0;
    //    for (unsigned long i = 0; i < assignment.size(); ++i) {
    //        temp += cost(i, assignment[i]);
    //    }
    //    return temp;
    //}

// ----------------------------------------------------------------------------------------

    //namespace impl {
    //template <typename EXP>
    inline void compute_slack(
        const long x,
        Arr<typename EXP::type>& slack,
        Arr<int>& slackx,
        const matrix_exp<EXP>& cost,
        const Arr<typename EXP::type>& lx,
        const Arr<typename EXP::type>& ly
    ) {
        for (long y = 0; y < cost.nc(); ++y) {
            if (lx[x] + ly[y] - cost(x, y) < slack[y]) {
                slack[y] = lx[x] + ly[y] - cost(x, y);
                slackx[y] = x;
            }
        }
    }
    //}

// ----------------------------------------------------------------------------------------

    //template <typename EXP>
    void max_cost_assignment() {
        //const_temp_matrix<EXP> cost(cost_);
        typedef typename EXP::type type;
        // This algorithm only works if the elements of the cost matrix can be reliably 
        // compared using operator==. However, comparing for equality with floating point
        // numbers is not a stable operation. So you need to use an integer cost matrix.
        //COMPILE_TIME_ASSERT(std::numeric_limits<type>::is_integer);
        //DLIB_ASSERT(cost.nr() == cost.nc(),
        //    "\t std::vector<long> max_cost_assignment(cost)"
        //    << "\n\t cost.nr(): " << cost.nr()
        //    << "\n\t cost.nc(): " << cost.nc()
        //);

        //using namespace dlib::impl;
        /*
            I based the implementation of this algorithm on the description of the
            Hungarian algorithm on the following websites:
                http://www.math.uwo.ca/~mdawes/courses/344/kuhn-munkres.pdf
                http://www.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

            Note that this is the fast O(n^3) version of the algorithm.
        */




        // Initially, nothing is matched. 
        //xy.assign(cost.nc(), -1);
        xy.reset(Arr<int>::ResetOption::AllBits1);
        //yx.assign(cost.nc(), -1);
        yx.reset(Arr<int>::ResetOption::AllBits1);
        /*
            We maintain the following invariant:
                Vertex x is matched to vertex xy[x] and
                vertex y is matched to vertex yx[y].

                A value of -1 means a vertex isn't matched to anything.  Moreover,
                x corresponds to rows of the cost matrix and y corresponds to the
                columns of the cost matrix.  So we are matching X to Y.
        */


        // Create an initial feasible labeling.  Moreover, in the following
        // code we will always have: 
        //     for all valid x and y:  lx[x] + ly[y] >= cost(x,y)
        //lx.resize(cost.nc());
        //ly.assign(cost.nc(), 0);
        ly.reset(Arr<type>::ResetOption::AllBits0);
        for (long x = 0; x < cost.nr(); ++x)
            lx[x] = *std::max_element(cost.begin(x), cost.end(x));

        // Now grow the match set by picking edges from the equality subgraph until
        // we have a complete matching.
        for (long match_size = 0; match_size < cost.nc(); ++match_size) {
            std::deque<long> q;

            // Empty out the S and T sets
            //S.assign(cost.nc(), false);
            //T.assign(cost.nc(), false);
            S.reset(Arr<char>::ResetOption::AllBits0);
            T.reset(Arr<char>::ResetOption::AllBits0);

            // clear out old slack values
            //slack.assign(cost.nc(), std::numeric_limits<type>::max());
            std::fill(slack.begin(), slack.end(), (std::numeric_limits<type>::max)());
            //slackx.resize(cost.nc());
            /*
                slack and slackx are maintained such that we always
                have the following (once they get initialized by compute_slack() below):
                    - for all y:
                        - let x == slackx[y]
                        - slack[y] == lx[x] + ly[y] - cost(x,y)
            */

            //aug_path.assign(cost.nc(), -1);
            aug_path.reset(Arr<int>::ResetOption::AllBits1);

            for (long x = 0; x < cost.nc(); ++x) {
                // If x is not matched to anything
                if (xy[x] == -1) {
                    q.push_back(x);
                    S[x] = true;

                    compute_slack(x, slack, slackx, cost, lx, ly);
                    break;
                }
            }


            long x_start = 0;
            long y_start = 0;

            // Find an augmenting path.  
            bool found_augmenting_path = false;
            while (!found_augmenting_path) {
                while (q.size() > 0 && !found_augmenting_path) {
                    const long x = q.front();
                    q.pop_front();
                    for (long y = 0; y < cost.nc(); ++y) {
                        if (cost(x, y) == lx[x] + ly[y] && !T[y]) {
                            // if vertex y isn't matched with anything
                            if (yx[y] == -1) {
                                y_start = y;
                                x_start = x;
                                found_augmenting_path = true;
                                break;
                            }

                            T[y] = true;
                            q.push_back(yx[y]);

                            aug_path[yx[y]] = x;
                            S[yx[y]] = true;
                            compute_slack(yx[y], slack, slackx, cost, lx, ly);
                        }
                    }
                }

                if (found_augmenting_path)
                    break;


                // Since we didn't find an augmenting path we need to improve the 
                // feasible labeling stored in lx and ly.  We also need to keep the
                // slack updated accordingly.
                type delta = std::numeric_limits<type>::max();
                for (long i = 0; i < T.size(); ++i) {
                    if (!T[i])
                        delta = std::min(delta, slack[i]);
                }
                for (long i = 0; i < T.size(); ++i) {
                    if (S[i])
                        lx[i] -= delta;

                    if (T[i])
                        ly[i] += delta;
                    else
                        slack[i] -= delta;
                }



                q.clear();
                for (long y = 0; y < cost.nc(); ++y) {
                    if (!T[y] && slack[y] == 0) {
                        // if vertex y isn't matched with anything
                        if (yx[y] == -1) {
                            x_start = slackx[y];
                            y_start = y;
                            found_augmenting_path = true;
                            break;
                        } else {
                            T[y] = true;
                            if (!S[yx[y]]) {
                                q.push_back(yx[y]);

                                aug_path[yx[y]] = slackx[y];
                                S[yx[y]] = true;
                                compute_slack(yx[y], slack, slackx, cost, lx, ly);
                            }
                        }
                    }
                }
            } // end while (!found_augmenting_path)

            // Flip the edges along the augmenting path.  This means we will add one more
            // item to our matching.
            for (long cx = x_start, cy = y_start, ty;
                cx != -1;
                cx = aug_path[cx], cy = ty) {
                ty = xy[cx];
                yx[cy] = cx;
                xy[cx] = cy;
            }

        }
    }
};

}


#endif // SMART_SZX_GOAL_MAX_MATCHING_BY_DLIB_H
