#ifndef _THERMALVIS_IMPROC_H_
#define _THERMALVIS_IMPROC_H_

#include "general_resources.h"
#include "opencv_resources.h"

#include "tools.h"

/// \brief		ID for custom mapping
#define CUSTOM			3
#define MAP_LENGTH		1024

#define RAINBOW			100
#define IRON			110
#define JET				120
#define BLUERED			130
#define BLUERED2		132
#define ICE				140
#define ICEIRON			150
#define BLACKBODY		300
#define CIECOMP			310
#define IRON2			320
#define RAINBOW2		330
#define CIECOMP2		340
#define GRAYSCALE		900

void obtainEightBitRepresentation(Mat& src, Mat& dst);
void obtainColorRepresentation(Mat& src, Mat& dst);

/// \brief      Very basic image cropping
void cropImage(Mat& image, cv::Point tl, cv::Point br);

/// \brief 		Convert a vector from 'Point' format to 'Point2f' format
void convertVectorToPoint2f(vector<cv::Point>& input, vector<Point2f>& output);

/// \brief      Convert a vector from 'Point2f' format to 'Point' format
void convertVectorToPoint(vector<Point2f>& input, vector<cv::Point>& output);

/// \brief      Finds centroid of a contour
cv::Point findCentroid(vector<cv::Point>& contour);

/// \brief      Finds centroid of a contour
Point2f findCentroid2f(vector<cv::Point>& contour);

/// \brief      Resizes an image without interpolation
void simpleResize(Mat& src, Mat& dst, Size size);

/// \brief      Creates a gaussian intensity distribution in a given matrix
void createGaussianMatrix(Mat& gaussianMat, double sigmaFactor);

/// \brief      Makes a copy of a contour
void copyContour(vector<cv::Point>& src, vector<cv::Point>& dst);

/// \brief      Inverts the pixel intensities of a matrix
void invertMatIntensities(Mat& src, Mat& dst);

/// \brief 		Swaps the position of two elements in a point vector
void swapElements(vector<Point2f>& corners, int index1, int index2);

/// \brief      Determine the width and height of a contour (in x and y directions at this stage)
void contourDimensions(vector<cv::Point> contour, double& width, double& height);

/// \brief 		Generates a point that's half way in between the two input points
Point2f meanPoint(Point2f& P1, Point2f& P2);

/// \brief      Finds the minimum separation between any two points in a set
double findMinimumSeparation(vector<Point2f>& pts);

/// \brief      Draws lines between initial and corrected points
void drawLinesBetweenPoints(Mat& image, const vector<Point2f>& src, const vector<Point2f>& dst);

/// \brief 		Calculates the perpindicular distance between a line (P1 to P2) and a point (P3)
double perpDist(Point2f& P1, Point2f& P2, Point2f& P3);

/// \brief 		Calculates the distance between two "Point" points
double distBetweenPts(cv::Point& P1, cv::Point& P2);
/// \brief      Calculates the distance between two "Point2f" points
double distBetweenPts2f(Point2f& P1, Point2f& P2);

/// \brief 		Move a point from one vector to another, resize and shift old vector
void transferElement(vector<Point2f>& dst, vector<Point2f>& src, int index);

/// \brief 		Stretches the histogram to span the whole 16-bit scale (16-bit to 16-bit)
void normalize_16(Mat& dst, Mat& src, double dblmin = -1.0, double dblmax = -1.0);
void reduceToPureImage(cv::Mat& dst, cv::Mat& src);
void fix_bottom_right(Mat& mat);

bool checkIfActuallyGray(const Mat& im);

void findIntensityValues(double *vals, Mat& im, Mat& mask);
void shiftIntensities(Mat& im, double scaler, double shifter, double downer);
void findPercentiles(const Mat& img, double *vals, double *percentiles, unsigned int num);

/// \brief      Obtains histogram and other image statistics
void generateHistogram(Mat& src, Mat& dst, double* im_hist, double* im_summ, double* im_stat);
	
/// \brief 		16-bit to 8-bit
void down_level(Mat& dst, Mat& src);

Mat normForDisplay(Mat origMat);

class cScheme {
protected:
	/// \brief		Short array of red-weightings of intensity mappings
	double* rx;
	/// \brief		Long array of red-weightings of intensity mappings
	double red[MAP_LENGTH];
	/// \brief		Short array of green-weightings of intensity mappings
	double* gx;
	/// \brief		Long array of green-weightings of intensity mappings
	double green[MAP_LENGTH];
	/// \brief		Short array of blue-weightings of intensity mappings
	double* bx;
	/// \brief		Long of blue-weightings of intensity mappings
	double blue[MAP_LENGTH];
	/// \brief		Number of elements in shortened colour arrays
	int		length;
	/// \brief		Code identifying which map is being used [see load_standard() function]
	int		code;
	
	/// \brief		Lookup table which shows what colour intensities correspond to what raw intensities
	unsigned char	lookupTable_1[256][3];
	unsigned short	lookupTable_2[65536][3];

public:
	/// \brief 		Constructor.
	cScheme();
	


	/// \brief 		Constructor.
	/// \param 		r		Pointer to red weightings
	/// \param 		g		Pointer to green weightings
	/// \param 		b		Pointer to blue weightings
	/// \param 		len		Length of all three arrays
	cScheme(double *r, double *g, double *b, int len);

	/// \brief 		Destructor.
	~cScheme();

	/// \brief 		Load a standard colour map in as the scheme.
	/// \param 		mapCode...
	void load_standard(int mapCode, int mapParam = 0);

	/// \brief 		Creates a long map using the shortened map.
	void create_long_map();

	/// \brief		Recreates lookup table
	void setupLookupTable(unsigned int depth = 65536);
	
	/// \brief 		Create and apply a new colour map as the scheme.
	/// \param 		r		Pointer to red weightings
	/// \param 		g		Pointer to green weightings
	/// \param 		b		Pointer to blue weightings
	/// \param 		len		Length of all three arrays
	void customize(double* r, double* g, double* b, int len);

	/// \brief 		Create a false colour version of a monochrocv::Matic thermal image.
	/// \param 		thermIm		Monochrocv::Matic Thermal Image
	/// \param 		outputIm	Colour output Image
	/// \param 		param		Integer value dictating safe or unsafe mode:
	///							[0] : unsafe mode - includes black and white
	///							[1] : safe mode - no black or white (default)
	void falsify_image(const cv::Mat& thermIm, cv::Mat& outputIm, int param = 1);

	/// \brief 		Create a false colour combination of a colour or monochrome visual image and a monochrome thermal image.
	/// \param 		thermIm		Monochromatic Thermal Image
	/// \param 		visualIm	Colour or monochromatic Visual Image
	/// \param 		outputIm	Colour output Image
	/// \param 		params		Pointer to double values dictating the following parameters:
	///							[0] : minimum lightness (default = 0.2)
	///							[1] : maximum lightness (default = 0.8)
	void fuse_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double* params);

	/// \brief 		Returns code for current scheme.
	/// \param 		output		0 = rainbow, 1 = iron, 2 = jet, 3 = custom
	int current_scheme();

	/// \brief 		Resizes the given image, preserving the underlying data (to some extent).
	/// \param 		inputIm		Input image
	/// \param 		dim_i		Width
	/// \param 		dim_j		Height
	void image_resize(cv::Mat& inputIm, int dim_i, int dim_j);
	
	
};

#endif
