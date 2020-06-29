////////////////////////////////
/// usage : 1.	
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_MIN_MATCHING_BY_ROBERT_PILGRIM_H
#define SMART_SZX_GOAL_MIN_MATCHING_BY_ROBERT_PILGRIM_H


#include <limits>

#include "Arr.h"


namespace szx {

struct MinMatchingByRobertPilgrim { // http://csclab.murraystate.edu/~bob.pilgrim/445/munkres.html
    MinMatchingByRobertPilgrim(const Arr2D<int> &costMat)
        : nrow(costMat.size1()), ncol(costMat.size2()), C(costMat),
        M(nrow, ncol), path(nrow + ncol + 1, 2), RowCover(nrow), ColCover(ncol), assignment(nrow) {
        resetMaskandCovers();
        step = 1;
    }


    const Arr<int>& solve() { // maximize the total cost.
        if (nrow > ncol) { return assignment; }

        RunMunkres();
        retrieveSln();
        return assignment;
    }


protected:
    int nrow;
    int ncol;
    int path_count = 0;
    int path_row_0;
    int path_col_0;
    int asgn = 0;
    int step;
    Arr2D<int> C;
    Arr2D<int> M;
    Arr2D<int> path;
    Arr<int> RowCover;
    Arr<int> ColCover;

    Arr<int> assignment;

    void retrieveSln() {
        for (int r = 0; r < nrow; ++r) {
            for (int c = 0; c < ncol; ++c) {
                if (M[r][c]) { assignment[r] = c; break; }
            }
        }
    }

    void resetMaskandCovers() {
        for (int r = 0; r < nrow; r++) {
            RowCover[r] = 0;
            for (int c = 0; c < ncol; c++) {
                M[r][c] = 0;
            }
        }
        for (int c = 0; c < ncol; c++)
            ColCover[c] = 0;
    }

    //For each row of the cost matrix, find the smallest element and subtract
    //it from every element in its row.  When finished, Go to Step 2.
    void step_one(int &step) {
        int min_in_row;

        for (int r = 0; r < nrow; r++) {
            min_in_row = C[r][0];
            for (int c = 0; c < ncol; c++)
                if (C[r][c] < min_in_row)
                    min_in_row = C[r][c];
            for (int c = 0; c < ncol; c++)
                C[r][c] -= min_in_row;
        }
        step = 2;
    }

    //Find a zero (Z) in the resulting matrix.  If there is no starred 
    //zero in its row or column, star Z. Repeat for each element in the 
    //matrix. Go to Step 3.
    void step_two(int &step) {
        for (int r = 0; r < nrow; r++)
            for (int c = 0; c < ncol; c++) {
                if (C[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0) {
                    M[r][c] = 1;
                    RowCover[r] = 1;
                    ColCover[c] = 1;
                }
            }
        for (int r = 0; r < nrow; r++)
            RowCover[r] = 0;
        for (int c = 0; c < ncol; c++)
            ColCover[c] = 0;
        step = 3;
    }

    //Cover each column containing a starred zero.  If K columns are covered, 
    //the starred zeros describe a complete set of unique assignments.  In this 
    //case, Go to DONE, otherwise, Go to Step 4.
    void step_three(int &step) {
        int colcount;
        for (int r = 0; r < nrow; r++)
            for (int c = 0; c < ncol; c++)
                if (M[r][c] == 1)
                    ColCover[c] = 1;

        colcount = 0;
        for (int c = 0; c < ncol; c++)
            if (ColCover[c] == 1)
                colcount += 1;
        if (colcount >= ncol || colcount >= nrow)
            step = 7;
        else
            step = 4;
    }

    //methods to support step 4
    void find_a_zero(int &row, int &col) {
        int r = 0;
        int c;
        bool done;
        row = -1;
        col = -1;
        done = false;
        while (!done) {
            c = 0;
            while (true) {
                if (C[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0) {
                    row = r;
                    col = c;
                    done = true;
                }
                c += 1;
                if (c >= ncol || done)
                    break;
            }
            r += 1;
            if (r >= nrow)
                done = true;
        }
    }

    bool star_in_row(int row) {
        bool tmp = false;
        for (int c = 0; c < ncol; c++)
            if (M[row][c] == 1)
                tmp = true;
        return tmp;
    }

    void find_star_in_row(int row, int &col) {
        col = -1;
        for (int c = 0; c < ncol; c++)
            if (M[row][c] == 1)
                col = c;
    }

    //Find a noncovered zero and prime it.  If there is no starred zero 
    //in the row containing this primed zero, Go to Step 5.  Otherwise, 
    //cover this row and uncover the column containing the starred zero. 
    //Continue in this manner until there are no uncovered zeros left. 
    //Save the smallest uncovered value and Go to Step 6.
    void step_four(int &step) {
        int row = -1;
        int col = -1;
        bool done;

        done = false;
        while (!done) {
            find_a_zero(row, col);
            if (row == -1) {
                done = true;
                step = 6;
            } else {
                M[row][col] = 2;
                if (star_in_row(row)) {
                    find_star_in_row(row, col);
                    RowCover[row] = 1;
                    ColCover[col] = 0;
                } else {
                    done = true;
                    step = 5;
                    path_row_0 = row;
                    path_col_0 = col;
                }
            }
        }
    }

    // methods to support step 5
    void find_star_in_col(int c, int &r) {
        r = -1;
        for (int i = 0; i < nrow; i++)
            if (M[i][c] == 1)
                r = i;
    }

    void find_prime_in_row(int r, int &c) {
        for (int j = 0; j < ncol; j++)
            if (M[r][j] == 2)
                c = j;
    }

    void augment_path() {
        for (int p = 0; p < path_count; p++)
            if (M[path[p][0]][path[p][1]] == 1)
                M[path[p][0]][path[p][1]] = 0;
            else
                M[path[p][0]][path[p][1]] = 1;
    }

    void clear_covers() {
        for (int r = 0; r < nrow; r++)
            RowCover[r] = 0;
        for (int c = 0; c < ncol; c++)
            ColCover[c] = 0;
    }

    void erase_primes() {
        for (int r = 0; r < nrow; r++)
            for (int c = 0; c < ncol; c++)
                if (M[r][c] == 2)
                    M[r][c] = 0;
    }


    //Construct a series of alternating primed and starred zeros as follows.  
    //Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote 
    //the starred zero in the column of Z0 (if any). Let Z2 denote the primed zero 
    //in the row of Z1 (there will always be one).  Continue until the series 
    //terminates at a primed zero that has no starred zero in its column.  
    //Unstar each starred zero of the series, star each primed zero of the series, 
    //erase all primes and uncover every line in the matrix.  Return to Step 3.
    void step_five(int &step) {
        bool done;
        int r = -1;
        int c = -1;

        path_count = 1;
        path[path_count - 1][0] = path_row_0;
        path[path_count - 1][1] = path_col_0;
        done = false;
        while (!done) {
            find_star_in_col(path[path_count - 1][1], r);
            if (r > -1) {
                path_count += 1;
                path[path_count - 1][0] = r;
                path[path_count - 1][1] = path[path_count - 2][1];
            } else
                done = true;
            if (!done) {
                find_prime_in_row(path[path_count - 1][0], c);
                path_count += 1;
                path[path_count - 1][0] = path[path_count - 2][0];
                path[path_count - 1][1] = c;
            }
        }
        augment_path();
        clear_covers();
        erase_primes();
        step = 3;
    }

    //methods to support step 6
    void find_smallest(int &minval) {
        for (int r = 0; r < nrow; r++)
            for (int c = 0; c < ncol; c++)
                if (RowCover[r] == 0 && ColCover[c] == 0)
                    if (minval > C[r][c])
                        minval = C[r][c];
    }

    //Add the value found in Step 4 to every element of each covered row, and subtract 
    //it from every element of each uncovered column.  Return to Step 4 without 
    //altering any stars, primes, or covered lines.
    void step_six(int &step) {
        int minval = (std::numeric_limits<int>::max)();
        find_smallest(minval);
        for (int r = 0; r < nrow; r++)
            for (int c = 0; c < ncol; c++) {
                if (RowCover[r] == 1)
                    C[r][c] += minval;
                if (ColCover[c] == 0)
                    C[r][c] -= minval;
            }
        step = 4;
    }

    void RunMunkres() {
        while (step < 7) {
            switch (step) {
            case 1:
                step_one(step);
                break;
            case 2:
                step_two(step);
                break;
            case 3:
                step_three(step);
                break;
            case 4:
                step_four(step);
                break;
            case 5:
                step_five(step);
                break;
            case 6:
                step_six(step);
                break;
            }
        }
    }
};

}


#endif // SMART_SZX_GOAL_MIN_MATCHING_BY_ROBERT_PILGRIM_H
