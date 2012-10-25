#ifndef _THERMALVIS_IMPROC_H_
#define _THERMALVIS_IMPROC_H_

#include "general_resources.h"
#include "opencv_resources.h"

#include <opencv2/video/video.hpp>

#include "tools.h"

#ifdef __CLAHE_HEADER__
    #include "clahe.h"
#endif

/// \brief		ID for custom mapping
#define CUSTOM			3
#define MAP_LENGTH		1024

#define GRAYSCALE		0

#define CIECOMP			100
#define CIECOMP_ALT_1	110
#define CIECOMP_ALT_2	120
#define CIECOMP_ALT_3	130

#define BLACKBODY		150

#define RAINBOW			200
#define RAINBOW_ALT_1	210
#define RAINBOW_ALT_2	220
#define RAINBOW_ALT_3	230
#define RAINBOW_ALT_4	240

#define IRON			300
#define IRON_ALT_1		310
#define IRON_ALT_2		320
#define IRON_ALT_3		330

#define BLUERED			400
#define BLUERED_ALT_1	410
#define BLUERED_ALT_2	420

#define JET				500
#define JET_ALT_1		510

#define ICE				600
#define ICE_ALT_1		610
#define ICE_ALT_2		620
#define ICE_ALT_3		630

#define ICEIRON			700
#define ICEIRON_ALT_1	710
#define ICEIRON_ALT_2	720

#define ICEFIRE			800
#define ICEFIRE_ALT_1	810
#define ICEFIRE_ALT_2	820
#define ICEFIRE_ALT_3	830

#define REPEATED		900
#define REPEATED_ALT_1	910
#define REPEATED_ALT_2	920
#define REPEATED_ALT_3	930
#define REPEATED_ALT_4	940
#define REPEATED_ALT_5	950
#define REPEATED_ALT_6	960

#define STANDARD_NORM_MODE 				0
#define LOW_CONTRAST_NORM_MODE 			1
#define CLAHE							2
#define ADAPTIVE_CONTRAST_ENHANCEMENT 	3

#define NO_FILTERING 					0
#define GAUSSIAN_FILTERING 				1
#define BILATERAL_FILTERING 			2

#define MIN_PROP_THRESHOLD 0.002

void weightedMixture(Mat& dst, const cv::vector<Mat>& srcs, const std::vector<double>& weightings);

void addBorder(Mat& inputMat, int borderSize);

void normalize_64_vec(Mat& dst, Mat& src);

void mixImages(Mat& dst, cv::vector<Mat>& images);

bool rectangle_contains_centroid(cv::Rect mainRectangle, cv::Rect innerRectangle);
Rect meanRectangle(vector<Rect>& rectangles);
void clusterRectangles(vector<Rect>& rectangles, double minOverlap);

cv::Point rectangle_center(cv::Rect input);

double rectangleOverlap(Rect rectangle1, Rect rectangle2);

void obtainEightBitRepresentation(Mat& src, Mat& dst);
void obtainColorRepresentation(Mat& src, Mat& dst);

/// \brief      Trims the top/bottom or sides of the image to get the correct ratio
void trimToDimensions(Mat& image, int width, int height);

/// \brief      Very basic image cropping
void cropImage(Mat& image, cv::Point tl, cv::Point br);

void drawGrid(const Mat& src, Mat& dst, int mode = 0);

/// \brief 		Convert a vector from 'Point' format to 'Point2f' format
void convertVectorToPoint2f(vector<cv::Point>& input, vector<Point2f>& output);

/// \brief      Convert a vector from 'Point2f' format to 'Point' format
void convertVectorToPoint(vector<Point2f>& input, vector<cv::Point>& output);

void splitMultimodalImage(const Mat& src, Mat& therm, Mat& vis);

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
void normalize_16(Mat& dst, const Mat& src, double dblmin = -1.0, double dblmax = -1.0);
void histExpand8(const Mat& src, Mat& dst);
void reduceToPureImage(cv::Mat& dst, cv::Mat& src);
void fix_bottom_right(Mat& mat);

void fixedDownsample(const Mat& src, Mat& dst, double center, double range);

void adaptiveContrastEnhancement(const Mat& src, Mat& dst, double factor = 0.0, int filter = NO_FILTERING);
void downsampleCLAHE(const Mat& src, Mat& dst, double factor, int filter = NO_FILTERING);
void adaptiveDownsample(const Mat& src, Mat& dst, int code = STANDARD_NORM_MODE, double factor = 0.0, int filter = NO_FILTERING);

bool checkIfActuallyGray(const Mat& im);

void findIntensityValues(double *vals, Mat& im, Mat& mask);
void shiftIntensities(Mat& im, double scaler, double shifter, double downer);
void findPercentiles(const Mat& img, double *vals, double *percentiles, unsigned int num);

/// \brief      Obtains histogram and other image statistics
void generateHistogram(Mat& src, Mat& dst, double* im_hist, double* im_summ, double* im_stat);

/// \brief 		16-bit to 8-bit
void down_level(Mat& dst, Mat& src);

void applyIntensityShift(const Mat& src1, Mat& dst1, const Mat& src2, Mat& dst2, double grad, double shift);

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
	void fuse_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double *params = NULL);

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
