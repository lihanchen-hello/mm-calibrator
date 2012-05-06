#ifndef _THERMALVIS_TOOLS_H_
#define _THERMALVIS_TOOLS_H_

#include "general_resources.h"
#include "opencv_resources.h"
#include "sys/time.h"

/// \brief      Calculates perpendicular distance between two "parallel" lines
double calcLinePerpDistance(double *line1, double *line2);

/// \brief      Calculates time elapsed since the last time the timer was reset
double timeElapsedMS(struct timeval& timer, bool reset = true);

bool matricesAreEqual(Mat& mat1, Mat& mat2);
Scalar getRandomColour();

void addUniqueToVector(vector<unsigned int>& dst, vector<unsigned int>& src);

double asymmetricGaussianValue(double score, double mean, double min, double max);

void randomSelection(vector<unsigned int>& src, vector<unsigned int>& dst, unsigned int max);

/// \brief      Redistorts points using an established distortion model
void redistortPoints(const vector<Point2f>& src, vector<Point2f>& dst, const Mat& cameraMatrix, const Mat& distCoeffs, const Mat& newCamMat=Mat::eye(3,3,CV_64FC1));

/// \brief		Calculates Factorial of an integer
long long int factorial(int num);

/// \brief      Gets next possible combination for an exhaustive combinatorial search
void getNextCombo(vector<unsigned int>& currentIndices, int r, int n);

/// \brief      Selects the score which is minimally better than a specified proportion of all scores
double findEquivalentProbabilityScore(double* values, int quantity, double prob);

#endif
