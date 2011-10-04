/*! \file	im_proc.hpp
 *  \brief	Header file for useful image processing functions.
 *
 * Self-written functions...
 */

#ifndef IM_PROC_HPP
#define IM_PROC_HPP

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

// OpenCV
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#ifndef _WIN32
// Linux
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



// /// \brief      Randomly cuts down a list of input files and corresponding vector of patterns
//void randomCulling(vector<std::string>& inputList, int maxSearch);

/// \brief		Calculates Factorial
long long int factorial(int num);

/// \brief      Gets next possible combination
	void getNextCombo(vector<unsigned int>& currentIndices, int r, int n);

/// \brief      From a list of scores, selects one which represents that of being better than a specified proportion of all possible scores
    double findEquivalentProbabilityScore(double* values, int quantity, double prob);

// =============================================
// CONSTANTS
// =============================================

namespace svLib {
	// =============================================
	// CLASSES
	// =============================================
	class pt {
	public:
		int i;
		int j;
		//double val;		// this isn't really used at present
	};

	class hillock {
	public:
		int maxIndex;
		std::vector<svLib::pt> points;			// Vector storing points

		int getMaxIndex(Mat& input) {
			// get maxIndex
			double maxVal = 0.0;
			double curVal = 0.0;
			maxIndex = 0;

			for (unsigned int x = 0; x < points.size(); x++) {

			    if (input.depth() == CV_8UC1) {
                    curVal = double(input.at<unsigned char>(points.at(x).i, points.at(x).j));
			    } else if (input.depth() == CV_16UC1) {
                    curVal = double(input.at<unsigned short>(points.at(x).i, points.at(x).j));
			    }

				if (curVal >= maxVal) {			/*points.at(x).val*/
					maxVal = curVal;
					maxIndex = x;
				}
			}

			//printf("%s << maxVal = %f\n", __FUNCTION__, maxVal);
			//cin.get();

			return maxIndex;
		};

	};

	// =============================================
	// FUNCTION PROTOTYPES
	// =============================================
	/// \brief 		Stretches the histogram to span the whole 16-bit scale (16-bit to 16-bit)
	void mixImages(Mat& dst, cv::vector<Mat>& images);

	/// \brief 		Stretches the histogram to span the whole 16-bit scale (16-bit to 16-bit)
	void normalize_16(Mat& dst, Mat& src, double dblmin = -1.0, double dblmax = -1.0);
	void normalize_16_old(Mat& dst, Mat& src);

	void invertIntensities64(Mat& src, Mat& dst);

	void addBorder(Mat& inputMat, int borderSize);

	void normalize_64(Mat& dst, Mat&src);

	void normalize_64_vec(Mat& dst, Mat&src);

	/// \brief      Thresholds a top and bottom fraction of the image
	void dynamic_threshold(Mat& dst, Mat& src, double low, double upp);

	/// \brief      Thresholds image to specified levels
	void fixed_threshold(Mat& dst, Mat& src, double low, double upp);

	/// \brief      Obtains histogram and other image statistics
	void generateHistogram(Mat& src, Mat& dst, double* im_hist, double* im_summ, double* im_stat);

 	/// \brief      Calculates the mean of the matrix
    double matrixMean(const Mat&src);

    /// \brief      Averages a sequence of matrices
    void averageMatrix(const vector<Mat>& matrixList, Mat& dst);

    /// \brief      Determine average gradient over an image using Scharr operator
    double matrixGradient(const Mat& src);

    /// \brief      Determine statistical scores
    void determineHeirarchicalScores(const Mat& src, double* scores);

	/// \brief      Gets a score for how much edge content is in an image
	double obtainEdgeScore(const Mat& src);

	/// \brief      Applies DFT and generates an image for display
	void applyDFT(Mat& src, Mat& dst);

    /// \brief 		Equalizes the histogram to enhance the apparent contrast (16-bit to 16-bit)
	void equalize_16(Mat& dst, Mat& src);

	/// \brief 		16-bit to 8-bit
    void down_level(Mat& dst, Mat& src);

	/// \brief 		8-bit to 16-bit
    void up_level(Mat& dst, Mat& src);

    /// \brief      Adds gaussian noise to an image
    void addGaussianNoise(const Mat& src, Mat& dst, double stdDev = 1.0);

	/// \brief 		16-bit to 16-bit
	void local_enhance(Mat& outputIm, Mat& inputIm, double controlParameter = 3.0);

	/// \brief 		Used by local_enhance
	void SweepAndPush(Mat& outputIm, Mat& inputIm, std::vector<double> grayValues, double controlParameter);

	/// \brief 		Used by local_enhance
	void FindHillocks(Mat& boolMat, std::vector<hillock>* hillockList, int level);
}

#endif
