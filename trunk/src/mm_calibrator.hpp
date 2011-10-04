/*! \file	mm_calibrator.hpp
 *  \brief	Header file for main calibration program.
 *
 * This program should enable easy and accurate camera calibration.
 */

#ifndef MM_CALIBRATOR_HPP
#define MM_CALIBRATOR_HPP

#include "intrinsics.hpp"
#include "extrinsics.hpp"
#include "cv_utils.hpp"
#include "im_proc.hpp"
#include "calibration.hpp"




#define DEFAULT_CAM_COUNT 1

#define DEFAULT_FRAMES_TO_LOAD 10 // should be at least 1000
#define DEFAULT_PATTERNS_TO_KEEP 100
#define DEFAULT_MAX_PATTERNS_PER_SET 5

#define DEFAULT_GRID_SIZE 10
#define DEFAULT_GRID_X_DIM 10
#define DEFAULT_GRID_Y_DIM 6

#define DEFAULT_ALPHA 0.00



// ==========================================================================================
//                                      EXAMPLE INPUT
// ==========================================================================================
// Current debug:
// -i /home/steve/calibration/data/chessboard-100/subset/ -e jpg -m 0 -g 30 -x 10 -y 6 -w -a 0.00 -d -o 4 -c -u -w -f -s -b 0 -q 80 -h 20 -j 5 -l 3 -t 2

// ==============================
// INCLUDES
// ==============================
// Standard
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>					// Linux
#include <getopt.h>
#include <sys/stat.h>				// Miscellaneous
#include <dirent.h>

// ==========================================================================================
//                                       CONSTANTS
// ==========================================================================================
#define RANDOM_CONSTANT 1
#define PARENT_DEBUG 1

// =============================================
// NAMESPACES
// =============================================
using namespace cv;
using namespace std;

static void usage(const char *argv0)
{
	printf("Usage: %s [options]\n", argv0);
	printf("Supported options:\n");
	printf("-d, --directory                             Must contain folder/s of images.\n");
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
    printf("-u, --undistort                             Undistort images.\n");
    printf("-w, --write                                 Write undistorted images.\n");
    printf("\nIf parameters are missing, defaults are used. However, if no parameters are provided, the user will be prompted.\n");
}

#endif



