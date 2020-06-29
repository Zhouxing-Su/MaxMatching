////////////////////////////////
/// usage : 1.	
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_SZX_GOAL_MIN_MATCHING_BY_SAM_HOCEVAR_H
#define SMART_SZX_GOAL_MIN_MATCHING_BY_SAM_HOCEVAR_H


#include <cstdlib>
#include <cstdint>

#include "Arr.h"


namespace szx {

struct MinMatchingBySamHocevar { // https://github.com/maandree/hungarian-algorithm-n3
    // make sure n <= m.
    MinMatchingBySamHocevar(const Arr2D<int> &costMat)
        : cost(costMat), n(cost.size1()), m(cost.size2()),
        table(n, m), marks(n, m), assignment(n) {}


    const Arr<int>& solve() {
        if (n > m) { return assignment; }

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                table[i][j] = cost[i][j];
            }
        }

        kuhn_match();
        retrieveSln();

        return assignment;
    }


protected:
    using cell = long;

    using size_t = int;
    using ssize_t = int;
    using llong = int_fast64_t;
    using byte = int_fast8_t;
    using boolean = int_fast8_t;


    Arr2D<int> cost;

    size_t n;
    size_t m;
    Arr2D<cell> table;
    Arr2D<byte> marks;
    Arr<int> assignment;

    void retrieveSln() {
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < m; ++j) {
                if (marks[i][j] == MARKED) { assignment[i] = j; break; }
            }
        }
    }

    /**
     * Cell marking:  none
     */
    static constexpr int UNMARKED = 0L;

    /**
     * Cell marking:  marked
     */
    static constexpr int MARKED = 1L;

    /**
     * Cell marking:  prime
     */
    static constexpr int PRIME = 2L;



    /**
     * Bit set, a set of fixed number of bits/booleans
     */
    typedef struct {
        /**
         * The set of all limbs, a limb consist of 64 bits
         */
        llong* limbs;

        /**
         * Singleton array with the index of the first non-zero limb
         */
        size_t* first;

        /**
         * Array the the index of the previous non-zero limb for each limb
         */
        size_t* prev;

        /**
         * Array the the index of the next non-zero limb for each limb
         */
        size_t* next;

    } BitSet;



    //ssize_t** kuhn_match(cell** table);
    //void kuhn_reduceRows(cell** t);
    //byte** kuhn_mark(cell** t);
    //boolean kuhn_isDone(byte** marks, boolean* colCovered);
    //size_t* kuhn_findPrime(cell** t, byte** marks, boolean* rowCovered, boolean* colCovered);
    //void kuhn_altMarks(byte** marks, size_t* altRow, size_t* altCol, ssize_t* colMarks, ssize_t* rowPrimes, size_t* prime);
    //void kuhn_addAndSubtract(cell** t, boolean* rowCovered, boolean* colCovered);
    //ssize_t** kuhn_assign(byte** marks);

    //BitSet new_BitSet(size_t size);
    //void BitSet_set(BitSet bitset, size_t i);
    //void BitSet_unset(BitSet bitset, size_t i);
    //ssize_t BitSet_any(BitSet bitset);

    //size_t lb(llong x);


    /**
     * Calculates an optimal bipartite minimum weight matching using an
     * O(n^3)-time implementation of The Hungarian Algorithm, also known
     * as Kuhn's Algorithm.
     *
     * @param   table  The table in which to perform the matching
     * @param   n      The height of the table
     * @param   m      The width of the table
     * @return         The optimal assignment, an array of row¨Ccoloumn pairs
     */
    void kuhn_match() {
        size_t i;

        /* not copying table since it will only be used once */

        kuhn_reduceRows();
        kuhn_mark();

        boolean* rowCovered = (boolean*)malloc(n * sizeof(boolean));
        boolean* colCovered = (boolean*)malloc(m * sizeof(boolean));
        for (i = 0; i < n; i++) {
            rowCovered[i] = false;
            colCovered[i] = false;
        }
        for (i = n; i < m; i++)
            colCovered[i] = false;

        size_t* altRow = (size_t*)malloc(n * m * sizeof(size_t));
        size_t* altCol = (size_t*)malloc(n * m * sizeof(size_t));

        ssize_t* rowPrimes = (ssize_t*)malloc(n * sizeof(ssize_t));
        ssize_t* colMarks = (ssize_t*)malloc(m * sizeof(ssize_t));

        size_t* prime;

        for (;;) {
            if (kuhn_isDone(colCovered))
                break;

            for (;;) {
                prime = kuhn_findPrime(rowCovered, colCovered);
                if (prime != NULL) {
                    kuhn_altMarks(altRow, altCol, colMarks, rowPrimes, prime);
                    for (i = 0; i < n; i++) {
                        rowCovered[i] = false;
                        colCovered[i] = false;
                    }
                    for (i = n; i < m; i++)
                        colCovered[i] = false;
                    free(prime);
                    break;
                }
                kuhn_addAndSubtract(rowCovered, colCovered);
            }
        }

        free(rowCovered);
        free(colCovered);
        free(altRow);
        free(altCol);
        free(rowPrimes);
        free(colMarks);
    }

    /**
     * Reduces the values on each rows so that, for each row, the
     * lowest cells value is zero, and all cells' values is decrease
     * with the same value [the minium value in the row].
     *
     * @param  t  The table in which to perform the reduction
     * @param  n  The table's height
     * @param  m  The table's width
     */
    void kuhn_reduceRows() {
        size_t i, j;
        cell min;
        cell* ti;
        for (i = 0; i < n; i++) {
            ti = table[i];
            min = *ti;
            for (j = 1; j < m; j++)
                if (min > ti[j])
                    min = ti[j];

            for (j = 0; j < m; j++)
                ti[j] -= min;
        }
    }


    /**
     * Create a matrix with marking of cells in the table whose
     * value is zero [minimal for the row]. Each marking will
     * be on an unique row and an unique column.
     *
     * @param   t  The table in which to perform the reduction
     * @param   n  The table's height
     * @param   m  The table's width
     * @return     A matrix of markings as described in the summary
     */
    void kuhn_mark() {
        size_t i, j;
        byte* marksi;
        for (i = 0; i < n; i++) {
            marksi = marks[i];
            for (j = 0; j < m; j++)
                marksi[j] = UNMARKED;
        }

        boolean* rowCovered = (boolean*)malloc(n * sizeof(boolean));
        boolean* colCovered = (boolean*)malloc(m * sizeof(boolean));
        for (i = 0; i < n; i++) {
            rowCovered[i] = false;
            colCovered[i] = false;
        }
        for (i = 0; i < m; i++)
            colCovered[i] = false;

        for (i = 0; i < n; i++)
            for (j = 0; j < m; j++)
                if ((!rowCovered[i]) && (!colCovered[j]) && (table[i][j] == 0)) {
                    marks[i][j] = MARKED;
                    rowCovered[i] = true;
                    colCovered[j] = true;
                }

        free(rowCovered);
        free(colCovered);
    }


    /**
     * Determines whether the marking is complete, that is
     * if each row has a marking which is on a unique column.
     *
     * @param   marks       The marking matrix
     * @param   colCovered  An array which tells whether a column is covered
     * @param   n           The table's height
     * @param   m           The table's width
     * @return              Whether the marking is complete
     */
    boolean kuhn_isDone(boolean* colCovered) {
        size_t i, j;
        for (j = 0; j < m; j++)
            for (i = 0; i < n; i++)
                if (marks[i][j] == MARKED) {
                    colCovered[j] = true;
                    break;
                }

        size_t count = 0;
        for (j = 0; j < m; j++)
            if (colCovered[j])
                count++;

        return count == n;
    }


    /**
     * Finds a prime
     *
     * @param   t           The table
     * @param   marks       The marking matrix
     * @param   rowCovered  Row cover array
     * @param   colCovered  Column cover array
     * @param   n           The table's height
     * @param   m           The table's width
     * @return              The row and column of the found print, <code>NULL</code> will be returned if none can be found
     */
    size_t* kuhn_findPrime(boolean* rowCovered, boolean* colCovered) {
        size_t i, j;
        BitSet zeroes = new_BitSet(n * m);

        for (i = 0; i < n; i++)
            if (!rowCovered[i])
                for (j = 0; j < m; j++)
                    if ((!colCovered[j]) && (table[i][j] == 0))
                        BitSet_set(zeroes, i * m + j);

        ssize_t p;
        size_t row, col;
        boolean markInRow;

        for (;;) {
            p = BitSet_any(zeroes);
            if (p < 0) {
                free(zeroes.limbs);
                free(zeroes.first);
                free(zeroes.next);
                free(zeroes.prev);
                return NULL;
            }

            row = (size_t)p / m;
            col = (size_t)p % m;

            marks[row][col] = PRIME;

            markInRow = false;
            for (j = 0; j < m; j++)
                if (marks[row][j] == MARKED) {
                    markInRow = true;
                    col = j;
                }

            if (markInRow) {
                rowCovered[row] = true;
                colCovered[col] = false;

                for (i = 0; i < n; i++)
                    if ((table[i][col] == 0) && (row != i)) {
                        if ((!rowCovered[i]) && (!colCovered[col]))
                            BitSet_set(zeroes, i * m + col);
                        else
                            BitSet_unset(zeroes, i * m + col);
                    }

                for (j = 0; j < m; j++)
                    if ((table[row][j] == 0) && (col != j)) {
                        if ((!rowCovered[row]) && (!colCovered[j]))
                            BitSet_set(zeroes, row * m + j);
                        else
                            BitSet_unset(zeroes, row * m + j);
                    }

                if ((!rowCovered[row]) && (!colCovered[col]))
                    BitSet_set(zeroes, row * m + col);
                else
                    BitSet_unset(zeroes, row * m + col);
            } else {
                size_t* rc = (size_t*)malloc(2 * sizeof(size_t));
                *rc = row;
                rc[1] = col;
                free(zeroes.limbs);
                free(zeroes.first);
                free(zeroes.next);
                free(zeroes.prev);
                return rc;
            }
        }
    }


    /**
     * Removes all prime marks and modifies the marking
     *
     * @param  marks      The marking matrix
     * @param  altRow     Marking modification path rows
     * @param  altCol     Marking modification path columns
     * @param  colMarks   Markings in the columns
     * @param  rowPrimes  Primes in the rows
     * @param  prime      The last found prime
     * @param  n          The table's height
     * @param  m          The table's width
     */
    void kuhn_altMarks(size_t* altRow, size_t* altCol, ssize_t* colMarks, ssize_t* rowPrimes, size_t* prime) {
        size_t index = 0, i, j;
        *altRow = *prime;
        *altCol = prime[1];

        for (i = 0; i < n; i++) {
            rowPrimes[i] = -1;
            colMarks[i] = -1;
        }
        for (i = n; i < m; i++)
            colMarks[i] = -1;

        for (i = 0; i < n; i++)
            for (j = 0; j < m; j++)
                if (marks[i][j] == MARKED)
                    colMarks[j] = (ssize_t)i;
                else if (marks[i][j] == PRIME)
                    rowPrimes[i] = (ssize_t)j;

        ssize_t row, col;
        for (;;) {
            row = colMarks[altCol[index]];
            if (row < 0)
                break;

            index++;
            altRow[index] = (size_t)row;
            altCol[index] = *(altCol + index - 1);

            col = rowPrimes[altRow[index]];

            index++;
            altRow[index] = *(altRow + index - 1);
            altCol[index] = (size_t)col;
        }

        byte* markx;
        for (i = 0; i <= index; i++) {
            markx = marks[altRow[i]] + altCol[i];
            if (*markx == MARKED)
                *markx = UNMARKED;
            else
                *markx = MARKED;
        }

        byte* marksi;
        for (i = 0; i < n; i++) {
            marksi = marks[i];
            for (j = 0; j < m; j++)
                if (marksi[j] == PRIME)
                    marksi[j] = UNMARKED;
        }
    }


    /**
     * Depending on whether the cells' rows and columns are covered,
     * the the minimum value in the table is added, subtracted or
     * neither from the cells.
     *
     * @param  t           The table to manipulate
     * @param  rowCovered  Array that tell whether the rows are covered
     * @param  colCovered  Array that tell whether the columns are covered
     * @param  n           The table's height
     * @param  m           The table's width
     */
    void kuhn_addAndSubtract(boolean* rowCovered, boolean* colCovered) {
        size_t i, j;
        cell min = 0x7FFFffffL;
        for (i = 0; i < n; i++)
            if (!rowCovered[i])
                for (j = 0; j < m; j++)
                    if ((!colCovered[j]) && (min > table[i][j]))
                        min = table[i][j];

        for (i = 0; i < n; i++)
            for (j = 0; j < m; j++) {
                if (rowCovered[i])
                    table[i][j] += min;
                if (colCovered[j] == false)
                    table[i][j] -= min;
            }
    }


    /**
     * Creates a list of the assignment cells
     *
     * @param   marks  Matrix markings
     * @param   n      The table's height
     * @param   m      The table's width
     * @return         The assignment, an array of row¨Ccoloumn pairs
     */
    ssize_t** kuhn_assign(byte** marks) {
        ssize_t** assignment = (ssize_t**)malloc(n * sizeof(ssize_t*));

        size_t i, j;
        for (i = 0; i < n; i++) {
            assignment[i] = (ssize_t*)malloc(2 * sizeof(ssize_t));
            for (j = 0; j < m; j++)
                if (marks[i][j] == MARKED) {
                    *assignment[i] = (ssize_t)i;
                    assignment[i][1] = (ssize_t)j;
                }
        }

        return assignment;
    }


    /**
     * Constructor for BitSet
     *
     * @param   size  The (fixed) number of bits to bit set should contain
     * @return        The a unique BitSet instance with the specified size
     */
    static BitSet new_BitSet(size_t size) {
        BitSet bitset;

        size_t c = size >> 6L;
        if (size & 63L)
            c++;

        bitset.limbs = (llong*)malloc(c * sizeof(llong));
        bitset.prev = (size_t*)malloc((c + 1) * sizeof(size_t));
        bitset.next = (size_t*)malloc((c + 1) * sizeof(size_t));
        *(bitset.first = (size_t*)malloc(sizeof(size_t))) = 0;

        size_t i;
        for (i = 0; i < c; i++) {
            *(bitset.limbs + i) = 0LL;
            *(bitset.prev + i) = *(bitset.next + i) = 0L;
        }
        *(bitset.prev + c) = *(bitset.next + c) = 0L;

        return bitset;
    }

    /**
     * Turns on a bit in a bit set
     *
     * @param  bitset  The bit set
     * @param  i     The index of the bit to turn on
     */
    static void BitSet_set(BitSet bitset, size_t i) {
        size_t j = i >> 6L;
        llong old = *(bitset.limbs + j);

        *(bitset.limbs + j) |= 1LL << (llong)(i & 63L);

        if ((!*(bitset.limbs + j)) ^ (!old)) {
            j++;
            *(bitset.prev + *(bitset.first)) = j;
            *(bitset.prev + j) = 0;
            *(bitset.next + j) = *(bitset.first);
            *(bitset.first) = j;
        }
    }

    /**
     * Turns off a bit in a bit set
     *
     * @param  bitset  The bit set
     * @param  i     The index of the bit to turn off
     */
    static void BitSet_unset(BitSet bitset, size_t i) {
        size_t j = i >> 6L;
        llong old = *(bitset.limbs + j);

        *(bitset.limbs + j) &= ~(1LL << (llong)(i & 63L));

        if ((!*(bitset.limbs + j)) ^ (!old)) {
            j++;
            size_t p = *(bitset.prev + j);
            size_t n = *(bitset.next + j);
            *(bitset.prev + n) = p;
            *(bitset.next + p) = n;
            if (*(bitset.first) == j)
                *(bitset.first) = n;
        }
    }

    /**
     * Gets the index of any set bit in a bit set
     *
     * @param   bitset  The bit set
     * @return        The index of any set bit
     */
    static ssize_t BitSet_any(BitSet bitset) {
        if (*(bitset.first) == 0L)
            return -1;

        size_t i = *(bitset.first) - 1;
        return (ssize_t)(lb(*(bitset.limbs + i) & -*(bitset.limbs + i)) + (i << 6L));
    }


    /**
     * Calculates the floored binary logarithm of a positive integer
     *
     * @param   value  The integer whose logarithm to calculate
     * @return         The floored binary logarithm of the integer
     */
    static size_t lb(llong value) {
        size_t rc = 0;
        llong v = value;

        if (v & (int_fast64_t)0xFFFFFFFF00000000LL) { rc |= 32L;  v >>= 32LL; }
        if (v & (int_fast64_t)0x00000000FFFF0000LL) { rc |= 16L;  v >>= 16LL; }
        if (v & (int_fast64_t)0x000000000000FF00LL) { rc |= 8L;  v >>= 8LL; }
        if (v & (int_fast64_t)0x00000000000000F0LL) { rc |= 4L;  v >>= 4LL; }
        if (v & (int_fast64_t)0x000000000000000CLL) { rc |= 2L;  v >>= 2LL; }
        if (v & (int_fast64_t)0x0000000000000002LL)     rc |= 1L;

        return rc;
    }
};

}


#endif // SMART_SZX_GOAL_MIN_MATCHING_BY_SAM_HOCEVAR_H
