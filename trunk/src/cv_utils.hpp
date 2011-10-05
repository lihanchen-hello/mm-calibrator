/*! \file	cv_utils.hpp
 *  \brief	Header file for useful functions involving OpenCV structures.
 *
 * Functions which would ideally already exist (in some form) in OpenCV.
 */

#ifndef CV_UTILS_HPP
#define CV_UTILS_HPP

#include "im_proc.hpp"

#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

/// \brief      Very basic image cropping
void cropImage(Mat& image, Point tl, Point br);

/// \brief 		Convert a vector from 'Point' format to 'Point2f' format
void convertVectorToPoint2f(vector<Point>& input, vector<Point2f>& output);

/// \brief      Convert a vector from 'Point2f' format to 'Point' format
void convertVectorToPoint(vector<Point2f>& input, vector<Point>& output);

/// \brief      Finds centroid of a contour
Point findCentroid(vector<Point>& contour);

/// \brief      Finds centroid of a contour
Point2f findCentroid2f(vector<Point>& contour);

/// \brief      Resizes an image without interpolation
void simpleResize(Mat& src, Mat& dst, Size size);

/// \brief      Creates a gaussian intensity distribution in a given matrix
void createGaussianMatrix(Mat& gaussianMat, double sigmaFactor);

/// \brief      Makes a copy of a contour
void copyContour(vector<Point>& src, vector<Point>& dst);

/// \brief      Inverts the pixel intensities of a matrix
void invertMatIntensities(Mat& src, Mat& dst);

/// \brief 		Swaps the position of two elements in a point vector
void swapElements(vector<Point2f>& corners, int index1, int index2);

/// \brief      Determine the width and height of a contour (in x and y directions at this stage)
void contourDimensions(vector<Point> contour, double& width, double& height);

/// \brief 		Generates a point that's half way in between the two input points
Point2f meanPoint(Point2f& P1, Point2f& P2);

/// \brief      Finds the minimum separation between any two points in a set
double findMinimumSeparation(vector<Point2f>& pts);

/// \brief      Draws lines between initial and corrected points
void drawLinesBetweenPoints(Mat& image, const vector<Point2f>& src, const vector<Point2f>& dst);

/// \brief 		Calculates the perpindicular distance between a line (P1 to P2) and a point (P3)
double perpDist(Point2f& P1, Point2f& P2, Point2f& P3);

/// \brief 		Calculates the distance between two "Point" points
double distBetweenPts(Point& P1, Point& P2);
/// \brief      Calculates the distance between two "Point2f" points
double distBetweenPts2f(Point2f& P1, Point2f& P2);

/// \brief 		Move a point from one vector to another, resize and shift old vector
void transferElement(vector<Point2f>& dst, vector<Point2f>& src, int index);

#endif
