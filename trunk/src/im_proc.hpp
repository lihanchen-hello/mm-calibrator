/*! \file	im_proc.hpp
 *  \brief	Header file for useful tools.
 *
 * These tools should have no OpenCV dependencies.
 */

#ifndef IM_PROC_HPP
#define IM_PROC_HPP

#include "im_proc.hpp"

#include <vector>

using namespace std;

/// \brief		Calculates Factorial of an integer
long long int factorial(int num);

/// \brief      Gets next possible combination for an exhaustive combinatorial search
void getNextCombo(vector<unsigned int>& currentIndices, int r, int n);

/// \brief      Selects the score which is minimally better than a specified proportion of all scores
double findEquivalentProbabilityScore(double* values, int quantity, double prob);

#endif
