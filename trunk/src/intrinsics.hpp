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
#define INTRINSICS_FLAGS                    CV_CALIB_RATIONAL_MODEL

#define ABSOLUTE_MAX_FRAMES_TO_STORE 1000

#define RADIAL_LENGTH       1000

#include "cv_utils.hpp"
#include "im_proc.hpp"
#include "calibration.hpp"

// OpenCV
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"




using namespace std;
using namespace cv;

/// \brief      Cut down the given vector of pointsets to those optimal for calibration
void optimizeCalibrationSet(const Mat& image,
                            Mat& distributionMap,
                            cv::vector< cv::vector<Point2f> >& candidateCorners,
                            cv::vector< cv::vector<Point2f> >& testCorners,
                            cv::vector<Point3f> row,
                            int selection,
                            int num,
                            double *radialDistribution,
                            cv::vector<int>& tagNames,
                            cv::vector<int>& selectedTags);

/// \brief      Calculate the Extended Reprojection Error: The reprojection error over a desired set of frames.
    double calculateERE(const Mat& image,
                        cv::vector<Point3f>& physicalPoints,
                        cv::vector< cv::vector<Point2f> >& corners,
                        const Mat& cameraMatrix,
                        const Mat& distCoeffs,
                        double errValues[] = NULL);





#endif
