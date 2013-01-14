/*! \file	mm_calibrator.hpp
 *  \brief	Header file for main calibration program.
 *
 * This program should enable easy and accurate camera calibration.
 * Example (debug) input:
 * LINUX: -d /home/steve/calibration/data/mmcal_test -i -n 1 -t 1 -x 12 -y 8 -s -u
 * WINDOWS: -d C:\Users\Steve\Data\calibration\mmcal_test -i -e -n 3 -t 1 -x 12 -y 8 -s -u
 */

#ifndef MM_CALIBRATOR_HPP
#define MM_CALIBRATOR_HPP

#include "calibration.hpp"
#include "intrinsics.hpp"
#include "extrinsics.hpp"

//#include "cv_utils.hpp"
#include "improc.h"

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

#define DEFAULT_FRAMES_TO_LOAD 1000 // should be at least 1000
#define DEFAULT_PATTERNS_TO_KEEP 100
#define DEFAULT_MAX_PATTERNS_PER_SET 10

#define DEFAULT_GRID_SIZE 10
#define DEFAULT_GRID_X_DIM 12
#define DEFAULT_GRID_Y_DIM 8

#define DEFAULT_ALPHA 0.00

#define RANDOM_CONSTANT 		1
#define DEFAULT_DEBUG_MODE 		0

using namespace cv;
using namespace std;

static void usage(const char *argv0)
{
    printf("Usage: %s [options]\n", argv0);
    printf("Supported options:\n");
    printf("	-d	Parent directory or video address.\n");
    printf("	-n	Number of cameras to calibrate.\n");
    printf("	-i	Option to calculate intrinsics.\n");
    printf("	-e	Option to calculate extrinsics.\n");
    printf("	-t	Pattern type:\n\
			[0] Regular chessboard\n\
			[1] Mask\n\
			[2] Thermal chessboard\n");
	printf("	-r	Use rational (8-parameter) rather than plumb-bob (5-parameter) distortion model.\n");
    printf("	-a	Max number of patterns to keep.\n");
    printf("	-b	Max number of patterns per set.\n");
    printf("	-g	Side length of square in mm.\n");
    printf("	-x	Number of squares in x direction.\n");
    printf("	-y	Number of squares in y direction.\n");
    printf("	-s	Show intermediate results.\n");
    printf("	-o	Optimization method:\n\
			[0] All found patterns\n\
			[1] Random set of patterns\n\
			[2] First N patterns\n\
			[3] Enhanced MCM\n\
			[4] Best of random trials\n\
			[5] Exhaustive search\n");
    printf("	-q	Input (and output) is video.\n");
    printf("	-u	Undistort images.\n");
    printf("	-z	Write original images with pattern overlayed.\n");
    printf("	-w	Write undistorted images.\n");
    printf("	-v	Display more debug info.\n");
    printf("	-p	File containing MSER parameters.\n");
    printf("	-c	Fraction of separation between squares to form corner search radius.\n");
    printf("If parameters are missing, defaults are used. However, if no parameters are provided, the user will be prompted.\n\n");
}

bool promptUserForParameters(char * directory, int numCams, bool wantsIntrinsics, bool wantsExtrinsics, int patternFinderCode, int maxPatternsToKeep, int maxPatternsPerSet, double gridSize, int x, int y, bool wantsToDisplay, int optimizationCode, bool wantsToUndistort, bool wantsToWrite, bool inputIsFolder, bool verboseMode) {

    int optionSelected = -1;

    cout << "Do you wish to [0] use default values or [1] specify values (recommended)?" << endl;

    while ((optionSelected != 0) && (optionSelected != 1)) {
        cin >> optionSelected;

        if ((optionSelected != 0) && (optionSelected != 1)) {
            cout << "Please enter either 0 or 1..." << endl;
        }
    }

    if (optionSelected == 0) {

        // Assign some default values (only those that aren't hard-coded...)

        #ifdef _WIN32
             sprintf(directory, "%s", "C:\Users\Steve\Data\calibration\mmcal_test");
        #else
            sprintf(directory, "%s", "/home/steve/calibration/data/mmcal_test");
        #endif

        return false;
    }

    // ... Get other values

    return true;


}

void obtainMSERparameters(char *filename, mserParameterGroup &mserParams) {
	
	fstream file_io;
	
	file_io.open(filename);
	
	file_io >> mserParams.delta;
	file_io >> mserParams.max_variation;
	file_io >> mserParams.min_diversity;
	file_io >> mserParams.max_evolution;
	file_io >> mserParams.area_threshold;
	file_io >> mserParams.min_margin;
	file_io >> mserParams.edge_blur_size;
	
	
	file_io.close();
	
}

#endif
