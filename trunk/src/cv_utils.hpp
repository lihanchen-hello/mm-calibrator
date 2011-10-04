/*! \file	cv_utils.hpp
 *  \brief	Header file for useful functions involving OpenCV structures.
 *
 * Self-written functions that I think should really be in OpenCV (in some form or another).
 */

#ifndef CV_UTILS_HPP
#define CV_UTILS_HPP

#include "im_proc.hpp"

// =============================================
// INCLUDES
// =============================================
// Standard
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
//#include <iomanip> // for setprecision()


// OpenCV
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

// Linux
#ifndef _WIN32
#include <unistd.h>
#include <getopt.h>
// Miscellaneous
#include <sys/stat.h>
#include <dirent.h>
#endif



// =============================================
// NAMESPACES
// =============================================
using namespace cv;
using namespace std;


// =============================================
// CONSTANTS
// =============================================

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

namespace svLib {
	// =============================================
	// FUNCTION PROTOTYPES
	// =============================================
    /// \brief      Generates a random colour
    Scalar getRandomColour();

	/// \brief 		Copy corner co-ordinates from vector to Mat
	void copy_corner_coords(Mat& dst, vector<Point2f> src, int x, int y);

	/// \brief 		Scale two stereo frames and display them side by side
	void display_combined_stereo(Mat& stereo1, Mat& stereo2);

	bool rectangle_contains_centroid(cv::Rect mainRectangle, cv::Rect innerRectangle);

	void clusterRectangles(vector<Rect>& rectangles, double minOverlap);

	double rectangleOverlap(Rect rectangle1, Rect rectangle2);

    /// \brief 		Calculates the distance between two "Point" points
    double distBetweenPts(Point& P1, Point& P2);
    /// \brief      Calculates the distance between two "Point2f" points
    double distBetweenPts2f(Point2f& P1, Point2f& P2);

    /// \brief      Inverts the pixel intensities of a matrix
    void invertMatIntensities(Mat& src, Mat& dst);

	Rect meanRectangle(vector<Rect>& rectangles);

	/// \brief      Trims the top/bottom or sides of the image to get the correct ratio
	void trimToDimensions(Mat& image, int width, int height);

    /// \brief      Very basic image cropping
    void cropImage(Mat& image, Point tl, Point br);

	Point rectangle_center(cv::Rect input);

	int getAVIFrames(char * fname);

	/// \brief      Returns true if matrices are identical
	bool compareMatrices(Mat& mat1, Mat& mat2);

	/// \brief      Copy a smaller image to the center of a larger image
	void copyToCentre(const Mat& src, Mat& dst);

    /// \brief      Resizes an image without interpolation
    void simpleResize(Mat& src, Mat& dst, Size size);

    /// \brief      Convert feature points from C++ to C-style
    void convertPointsToCvMat(vector<Point2f>& points, CvMat *cvmat_pts);

    /// \brief      Convert feature points from C-style to C++
    void convertPointsFromCvMat(CvMat *cvmat_pts, vector<Point2f>& points);

    /// \brief      Wrapper function for old OpenCV function
    void correctMatches(Mat& F, vector<Point2f>& points1, vector<Point2f>& points2);

    // JUNK BELOW HERE



	/// \brief 		Print new camera matrices
	void print_new_cams(Mat& newCam1, Mat& newCam2);

	/// \brief 		Print intrinsic data to screen
	void print_calib_data(Mat& cam, Mat& dist);

	/// \brief 		Print extrinsic data to screen
	void print_stereo_data(Mat& R, Mat& T, Mat& E, Mat& F);

	/// \brief 		Print rectification data to screen
	void print_rectification_data(Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q);

	/// \brief 		Hopefully a temporary function since OpenCV YML Parsing fails.. :P
	void assign_stereo_intrinsic_data(Mat& camMatrix1, Mat& distVector1, Mat& camMatrix2, Mat& distVector2);
	void assign_stereo_extrinsic_data(Mat& R, Mat& T, Mat& E, Mat& F, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q);
}

#endif
