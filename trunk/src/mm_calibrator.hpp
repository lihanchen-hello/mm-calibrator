/*! \file	mm_calibrator.hpp
 *  \brief	Header file for main calibration program.
 *
 * This program should enable easy and accurate camera calibration.
 * Example (debug) input:
 * LINUX: -d /home/steve/calibration/data/mmcal_test -i -e -n 3 -t 1 -x 12 -y 8 -s -u
 * WINDOWS: -d C:\Users\Steve\Data\calibration\mmcal_test -i -e -n 3 -t 1 -x 12 -y 8 -s -u
 */

#ifndef MM_CALIBRATOR_HPP
#define MM_CALIBRATOR_HPP

#include "calibration.hpp"
#include "intrinsics.hpp"
#include "extrinsics.hpp"

#include "cv_utils.hpp"
#include "im_proc.hpp"

#include <opencv2/opencv.hpp>

#include <dirent.h>
#include <stdio.h>

#if defined(WIN32)
#include "XGetopt.h"
#include <windows.h>
#else
const mode_t DEFAULT_MKDIR_PERMISSIONS = S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH;
#endif


#define DEFAULT_CAM_COUNT 1

#define DEFAULT_FRAMES_TO_LOAD 100 // should be at least 1000
#define DEFAULT_PATTERNS_TO_KEEP 100
#define DEFAULT_MAX_PATTERNS_PER_SET 10

#define DEFAULT_GRID_SIZE 10
#define DEFAULT_GRID_X_DIM 10
#define DEFAULT_GRID_Y_DIM 6

#define DEFAULT_ALPHA 0.00

#define RANDOM_CONSTANT 1
#define PARENT_DEBUG 1

using namespace cv;
using namespace std;

static void usage(const char *argv0)
{
    printf("Usage: %s [options]\n", argv0);
    printf("Supported options:\n");
    printf("-d, --directory                             Parent directory or video address.\n");
    printf("-n, --number                                Number of cameras to calibrate.\n");
    printf("-i, --intrinsics                            Option to calculate intrinsics.\n");
    printf("-e, --extrinsics                            Option to calculate extrinsics.\n");
    printf("-t, --type                                  Pattern type:\n\
                                                        [0] Regular chessboard\n\
                                                        [1] Mask\n\
                                                        [2] Thermal chessboard\n");
    printf("-a, --patterns                              Max number of patterns to keep.\n");
    printf("-b, --setsize                               Max number of patterns per set.\n");
    printf("-g, --gridsize                              Side length of square in mm.\n");
    printf("-x, --xcount                                Number of squares in x direction.\n");
    printf("-y, --ycount                                Number of squares in y direction.\n");
    printf("-s, --show                                  Show intermediate results.\n");
    printf("-o, --optimization                          Optimization method:\n\
                                                        [0] All found patterns\n\
                                                        [1] Random set of patterns\n\
                                                        [2] First N patterns\n\
                                                        [3] Enhanced MCM\n\
                                                        [4] Best of random trials\n\
                                                        [5] Exhaustive search\n");
    printf("-q, --video                                 Input (and output) is video.\n");
    printf("-u, --undistort                             Undistort images.\n");
    printf("-w, --write                                 Write undistorted images.\n");
    printf("\nIf parameters are missing, defaults are used. However, if no parameters are provided, the user will be prompted.\n");
}

#endif
