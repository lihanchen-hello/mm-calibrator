/*! \file	intrinsics.hpp
 *  \brief	Header file for functions associated with intrinsic calibration
 *
 * Ideally this file should only contain functions needed for intrinsic calibration, and not extrinsic calibration.
 * Functions required by both should be included in the "calibration.hpp/cpp" files.
 */

#ifndef INTRINSICS_HPP
#define INTRINSICS_HPP

// Restrictive: CV_CALIB_FIX_K5 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K3 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST
// Rational: CV_CALIB_RATIONAL_MODEL
// Conventional: 0
#define DEFAULT_INTRINSICS_FLAGS 					0
#define SEARCH_ONLY_FOR_BASIC_PARAMETERS 			14569 // 14569 allows freely moving focal point, 14573 fixes focal point

#define ABSOLUTE_MAX_FRAMES_TO_STORE 		1000
#define RADIAL_LENGTH 						1000
#define DEFAULT_NUM							10

#define INTRINSICS_HPP_DEBUG_MODE 			0

//#include "cv_utils.hpp"
#include "improc.h"
#include "calibration.hpp"
#include "tools.h"

using namespace std;
using namespace cv;

/// \brief      Cut down the given vector of pointsets to those optimal for calibration
void optimizeCalibrationSet(Size imSize,
                            cv::vector< cv::vector<Point2f> >& candidatePatterns,
                            cv::vector< cv::vector<Point2f> >& testPatterns,
                            cv::vector<Point3f> row,
                            cv::vector<int>& selectedTags,
                            int selection = ENHANCED_MCM_OPTIMIZATION_CODE,
                            int num = DEFAULT_NUM,
                            bool debugMode = false,
                            int intrinsicsFlags = DEFAULT_INTRINSICS_FLAGS);

/// \brief      Calculate the Extended Reprojection Error: The reprojection error over a desired set of frames.
double calculateERE(Size imSize,
                    cv::vector<Point3f>& physicalPoints,
                    cv::vector< cv::vector<Point2f> >& corners,
                    const Mat& cameraMatrix,
                    const Mat& distCoeffs,
                    double errValues[] = NULL);


#endif
