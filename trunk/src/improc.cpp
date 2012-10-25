#include "improc.h"

void splitMultimodalImage(const Mat& src, Mat& therm, Mat& vis) {

	therm = Mat::zeros(src.size(), CV_8UC1);
	vis = Mat::zeros(src.size(), CV_8UC1);

	for (unsigned int iii = 0; iii < src.rows; iii++) {
		for (unsigned int jjj = 0; jjj < src.cols; jjj++) {
			therm.at<unsigned char>(iii,jjj) = src.at<Vec3b>(iii,jjj)[2];
			vis.at<unsigned char>(iii,jjj) = src.at<Vec3b>(iii,jjj)[1];
		}
	}

}

void applyFilter(const Mat& src, Mat& dst, int filter) {

	double bilateralVal = 5;

	if (filter == GAUSSIAN_FILTERING) {
		//ksize = Size(kernel_width, kernel_width);
		//blurSigma = (2.0 / factor) * pow(2.0, 0.5) / 2.0;
		//ROS_INFO("kernel: (%d) (%f)", kernel_width, blurSigma);
		//GaussianBlur(tmpIm1, tmpIm1b, ksize, blurSigma);
		GaussianBlur(src, dst, Size(5, 5), sqrt(2));
	} else if (filter == BILATERAL_FILTERING) {
		bilateralFilter(src, dst, (int)bilateralVal, bilateralVal * 2.0, bilateralVal * 2.0); //, double sigmaColor, double sigmaSpace, int borderType=BORDER_DEFAULT );
	} else {
		src.copyTo(dst);
	}

}

void downsampleCLAHE(const Mat& src, Mat& dst, double factor, int filter) {

	Mat tmp;

	if (factor == 0.0) {
		//printf("%s << Factor = 0.0...\n", __FUNCTION__);

		if (filter == NO_FILTERING) {
			adaptiveDownsample(src, dst, STANDARD_NORM_MODE);
		} else {
			adaptiveDownsample(src, tmp, STANDARD_NORM_MODE);
			applyFilter(tmp, dst, filter);
		}

		return;
	}

	// from: https://github.com/joshdoe/opencv-clahe

	// ...

	//printf("%s << entered...\n", __FUNCTION__);

	Mat tmp_2;

	if (filter == NO_FILTERING) {
		adaptiveDownsample(src, tmp_2, STANDARD_NORM_MODE);
	} else {
		adaptiveDownsample(src, tmp, STANDARD_NORM_MODE);
		applyFilter(tmp, tmp_2, filter);
	}

	int xdivs = 2;
	int ydivs = 2;
	int bins = 256;
	//int limit_counter = 255.0 * factor;

	IplImage tmp_ipl(tmp_2);


	IplImage* dst_ipl = cvCreateImage(cvSize(tmp_ipl.width,tmp_ipl.height), tmp_ipl.depth, 1);
	dst_ipl->origin = tmp_ipl.origin;

	//IplImage dst_ipl;

	#ifdef __CLAHE_HEADER__
        cvCLAdaptEqualize(&tmp_ipl, dst_ipl, (unsigned int) xdivs, (unsigned int) ydivs,
					(unsigned int) bins, (1.0 + factor * 14.0), CV_CLAHE_RANGE_FULL);
	#endif


	// (float) limit_counter * 0.1


	dst = Mat(dst_ipl);

	//tmp.copyTo(dst);

}

void adaptiveContrastEnhancement(const Mat& src, Mat& dst, double factor, int filter) {

	Mat tmp, tmp_2;

	if (factor == 0.0) {
		//printf("%s << Factor = 0.0...\n", __FUNCTION__);

		if (filter == NO_FILTERING) {
			adaptiveDownsample(src, dst, STANDARD_NORM_MODE);
		} else {
			adaptiveDownsample(src, tmp, STANDARD_NORM_MODE);
			applyFilter(tmp, dst, filter);
		}

		return;
	}

	if (filter == NO_FILTERING) {
		adaptiveDownsample(src, tmp_2, STANDARD_NORM_MODE);
	} else {
		adaptiveDownsample(src, tmp, STANDARD_NORM_MODE);
		applyFilter(tmp, tmp_2, filter);
	}

	int xdivs = 2;
	int ydivs = 2;
	int bins = 256;
	//int limit_counter = 255.0 * factor;

	IplImage tmp_ipl(tmp_2);


	IplImage* dst_ipl = cvCreateImage(cvSize(tmp_ipl.width,tmp_ipl.height), tmp_ipl.depth, 1);
	dst_ipl->origin = tmp_ipl.origin;

	//IplImage dst_ipl;

	printf("%s << Implementing enhancement algorithm...\n", __FUNCTION__);

    #ifdef __CLAHE_HEADER__
    cvCLAdaptEnhance(&tmp_ipl, dst_ipl, (unsigned int) xdivs, (unsigned int) ydivs,
					(unsigned int) bins, (1.0 + factor * 24.0), CV_CLAHE_RANGE_FULL);
    #endif

	// (float) limit_counter * 0.1


	dst = Mat(dst_ipl);


}

void fixedDownsample(const Mat& src, Mat& dst, double center, double range) {

	dst = Mat::zeros(src.rows, src.cols, CV_8UC1);

	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {

			dst.at<unsigned char>(iii,jjj) = std::min(255.0, std::max(0.0, 127.5 + 127.5*(double(src.at<unsigned short>(iii,jjj)) - center)/(0.5 * range)));

		}
	}

}

void applyIntensityShift(const Mat& src1, Mat& dst1, const Mat& src2, Mat& dst2, double grad, double shift) {

	double center = 8000.0, range = 1.0;

	double percentileVals[3] = { 0.001, 0.500, 0.999 };
	double intensityVals[3];
	findPercentiles(src1, intensityVals, percentileVals, 3);

	center = intensityVals[1];
	range = std::max(abs(intensityVals[2] - intensityVals[1]), abs(intensityVals[0] - intensityVals[1]));

	printf("%s << normalizing with center = (%f) and range = (%f)\n", __FUNCTION__, center, range);

	//adaptiveDownsample(src1, dst1);
	fixedDownsample(src1, dst1, center, 2.0*range);

	Mat src2_shifted(src2.rows, src2.cols, src2.type());
	// src2.copyTo(src2_shifted);

	for (int iii = 0; iii < src2_shifted.rows; iii++) {
		for (int jjj = 0; jjj < src2_shifted.cols; jjj++) {

			src2_shifted.at<unsigned short>(iii,jjj) = grad * double(src2.at<unsigned short>(iii,jjj)) + shift;

		}
	}

	//adaptiveDownsample(src2_shifted, dst2);
	fixedDownsample(src2_shifted, dst2, center, range);

}

void drawGrid(const Mat& src, Mat& dst, int mode) {

	src.copyTo(dst);

	Scalar col;

	int shift;

	if (mode == 0) {
		shift = 0;
		col = CV_RGB(255, 0, 0);
	} else {
		shift = 1;
		col = CV_RGB(0, 0, 255);
	}

	Point2f startPt, endPt;

	double amt = double(src.cols) / 8.0;

	// Vertical lines
	for (int iii = shift; iii <= 8-shift; iii++) {
		startPt = Point2f(16.0 * double(iii)*amt, 16.0 * double(shift)*amt);
		endPt = Point2f(16.0 * double(iii)*amt, 16.0 * (double(src.rows) - double(shift)*amt));

		line(dst, startPt, endPt, col, 1, CV_AA, 4);
	}

	// Horizontal lines
	for (int iii = shift; iii <= 6-shift; iii++) {

		startPt = Point2f(16.0 * double(shift)*amt, 16.0 * double(iii)*amt);
		endPt = Point2f(16.0 * (double(src.cols) - double(shift)*amt), 16.0 * double(iii)*amt);

		line(dst, startPt, endPt, col, 1, CV_AA, 4);

	}


}

void histExpand8(const Mat& src, Mat& dst) {

	double minVal, maxVal;
	minMaxLoc(src, &minVal, &maxVal);

	dst = Mat::zeros(src.size(), src.type());

	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {

			unsigned char val = (unsigned char) (((double)src.at<unsigned char>(iii,jjj)) - minVal) * 255.0 / (maxVal - minVal);

			dst.at<unsigned char>(iii,jjj) = val;
		}
	}

}

void clusterRectangles(vector<Rect>& rectangles, double minOverlap) {

    if (rectangles.size() == 0) {
        return;
    }

    unsigned int j,k;

    bool hasBeenClustered;

    vector<Rect> falseVector;

    Rect tempRect;

    vector<vector<Rect> > clusteredRectangles;

    clusteredRectangles.push_back(falseVector);

    clusteredRectangles.at(0).push_back(rectangles.at(0));

    // For each remaining rectangle
    for (unsigned int i = 1; i < rectangles.size(); i++) {

        hasBeenClustered = false;

        j = 0;
        k = 0;

        while (!hasBeenClustered) {
            if (rectangleOverlap(rectangles.at(i), clusteredRectangles.at(j).at(k)) > minOverlap) {
                clusteredRectangles.at(j).push_back(rectangles.at(i));
                hasBeenClustered = true;
            } else if (k < clusteredRectangles.at(j).size()-1) {
                k++;
            } else if (j < clusteredRectangles.size()-1) {
                j++;
                k = 0;
            } else {
                clusteredRectangles.push_back(falseVector);
                clusteredRectangles.at(j+1).push_back(rectangles.at(i));
                hasBeenClustered = true;
            }
        }

        //printf("%s << overlapProp = %f\n", __FUNCTION__, overlapProp);

    }

    rectangles.clear();

    for (unsigned int i = 0; i < clusteredRectangles.size(); i++) {
        tempRect = meanRectangle(clusteredRectangles.at(i));
        rectangles.push_back(tempRect);
    }

}

Rect meanRectangle(vector<Rect>& rectangles) {

    Rect retVal;
    double xSum = 0.0, ySum = 0.0, wSum = 0.0, hSum = 0.0;

    for (unsigned int i = 0; i < rectangles.size(); i++) {
        xSum += rectangles.at(i).x;
        ySum += rectangles.at(i).y;
        wSum += rectangles.at(i).width;
        hSum += rectangles.at(i).height;
    }

    xSum /= rectangles.size();
    ySum /= rectangles.size();
    wSum /= rectangles.size();
    hSum /= rectangles.size();

    retVal = Rect(int(xSum), int(ySum), int(wSum), int(hSum));

    return retVal;
}

void weightedMixture(Mat& dst, const cv::vector<Mat>& srcs, const std::vector<double>& weightings) {

    double totalWeighting = 0.0;
    vector<double> newWeightings;
    newWeightings.insert(newWeightings.end(), weightings.begin(), weightings.end());

    for (unsigned int iii = 0; iii < weightings.size(); iii++) {
        totalWeighting += weightings.at(iii);
    }

    for (unsigned int iii = 0; iii < weightings.size(); iii++) {
        newWeightings.at(iii) /= totalWeighting;
    }

    Mat tmpDst = Mat::zeros(srcs.at(0).size(), CV_64FC3);

    for (unsigned int i = 0; i < srcs.size(); i++) {

        for (int m = 0; m < srcs.at(i).rows; m++) {

            for (int n = 0; n < srcs.at(i).cols; n++) {

                for (int k = 0; k < 3; k++) {
                    tmpDst.at<Vec3d>(m,n)[k] += double((srcs.at(i)).at<Vec3b>(m,n)[k]) * newWeightings.at(i);
                }

            }

        }

    }

    Mat normMat;
    normalize_64_vec(normMat, tmpDst);

    dst = Mat(normMat.size(), CV_8UC3);
    convertScaleAbs(normMat, dst, 255);

}

void mixImages(Mat& dst, cv::vector<Mat>& images) {
    // No checks at present...

    dst = Mat::zeros(images.at(0).size(), CV_64FC3);
    Mat tmp;
    dst.copyTo(tmp);

    for (unsigned int i = 0; i < images.size(); i++) {

        for (int m = 0; m < images.at(i).rows; m++) {

            for (int n = 0; n < images.at(i).cols; n++) {

                for (int k = 0; k < 3; k++) {
                    dst.at<Vec3d>(m,n)[k] += double((images.at(i)).at<Vec3b>(m,n)[k]) / images.size();
                }



            }


        }

        normalize_64_vec(tmp, dst);

        //imshow("tmp", tmp);
        //waitKey(0);

    }

}

bool rectangle_contains_centroid(cv::Rect mainRectangle, cv::Rect innerRectangle) {
    bool retVal = false;

    //printf("%s << Main Rectangle: [%d, %d] : [%d, %d]\n", __FUNCTION__, mainRectangle.x, mainRectangle.y, mainRectangle.x+mainRectangle.width, mainRectangle.y+mainRectangle.height);
    //printf("%s << Centroid: [%d, %d]\n", __FUNCTION__, innerRectangle.x + innerRectangle.width/2, innerRectangle.y + innerRectangle.height/2);

    retVal = mainRectangle.contains(Point(innerRectangle.x + innerRectangle.width/2, innerRectangle.y + innerRectangle.height/2));

    return retVal;
}

Point rectangle_center(cv::Rect input) {
    Point center;

    center.x = input.x + input.width/2;
    center.y = input.y + input.height/2;


    return center;
}

double rectangleOverlap(Rect rectangle1, Rect rectangle2) {

    double area1, area2, exLeft, exRight, exTop, exBot, overlapArea, overlapProp;

    area1 = rectangle1.width*rectangle1.height;
    area2 = rectangle2.width*rectangle2.height;

    exLeft = max(rectangle1.x, rectangle2.x);
    exRight = min(rectangle1.x+rectangle1.width, rectangle2.x+rectangle2.width);
    exTop = max(rectangle1.y, rectangle2.y);
    exBot = min(rectangle1.y+rectangle1.height, rectangle2.y+rectangle2.height);

    if ((exLeft > exRight) || (exTop > exBot)) {
        return 0.0;
    }

    overlapArea = (exRight-exLeft)*(exBot-exTop);

    overlapProp = overlapArea / (max(area1, area2));

    return overlapProp;

}

void trimToDimensions(Mat& image, int width, int height) {

    Mat dst;

    int imWidth = image.cols;
    int imHeight = image.rows;

    double actualRatio = double(imWidth)/double(imHeight);
    double wantedRatio = double(width)/double(height);

    int initialWidth, initialHeight;

    //imshow("cropping", image);
    //waitKey(0);

    printf("%s << image dimensions = %d x %d.\n", __FUNCTION__, imWidth, imHeight);
    printf("%s << desired dimensions = %d x %d.\n", __FUNCTION__, width, height);

    printf("%s << actualRatio = %f; wantedRatio = %f\n", __FUNCTION__, actualRatio, wantedRatio);

    if (actualRatio < wantedRatio) {
        printf("%s << taller than we want.\n", __FUNCTION__);

        initialWidth = width;
        initialHeight = int(double(width)/actualRatio);

        // resize to get width to exactly desired width
        resize(image, dst, Size(initialWidth, initialHeight));

        //imshow("cropping", dst);
        //waitKey(0);

        // cut down height to desired height
        int topY = (dst.rows - height)/2;
        int botY = topY + height;

        printf("%s << topY = %d; botY = %d\n", __FUNCTION__, topY, botY);

        cropImage(dst, Point(0, topY), Point(width, botY));

        //imshow("cropping", dst);
        //waitKey(0);

    } else if (actualRatio >= wantedRatio) {
        printf("%s << wider than we want.\n", __FUNCTION__);

        initialWidth = int(double(height)*actualRatio);
        initialHeight = height;

        // resize to get width to exactly desired width
        resize(image, dst, Size(initialWidth, initialHeight));

        //imshow("cropping", dst);
        //waitKey(0);

        // cut down height to desired height
        int leftX = (dst.cols - width)/2;
        int rightX = leftX + width;

        printf("%s << leftX  = %d; rightX = %d\n", __FUNCTION__, leftX , rightX);

        cropImage(dst, Point(leftX, 0), Point(rightX, height));

        //imshow("cropping", dst);
        //waitKey(0);

    }

    dst.copyTo(image);


}

void adaptiveDownsample(const Mat& src, Mat& dst, int code, double factor, int filter) {

	double minVal, maxVal;
	minMaxLoc(src, &minVal, &maxVal);

	//printf("%s << min/max = (%f / %f)\n", __FUNCTION__, minVal, maxVal);

	double percentileVals[3] = { MIN_PROP_THRESHOLD, 0.500, 1 - MIN_PROP_THRESHOLD };
	double intensityVals[3];

	findPercentiles(src, intensityVals, percentileVals, 3);

	double midVal = intensityVals[1]; // (intensityVals[0] + intensityVals[2]) / 2.0;
	double halfRange = max(abs(intensityVals[2] - midVal), abs(intensityVals[0] - midVal));
	double compressionFactor = 1.0;

	if (code == STANDARD_NORM_MODE) {

		if (halfRange > 128.0) {
			compressionFactor = halfRange / 128.0;
		}

		minVal = midVal - 128.0*compressionFactor;
		maxVal = midVal + 128.0*compressionFactor;

	} else if (code == LOW_CONTRAST_NORM_MODE) {

		minVal = midVal - halfRange;
		maxVal = midVal + halfRange;

	} else if (code == CLAHE) {

		downsampleCLAHE(src, dst, factor, filter);
		return;

	} else if (code == ADAPTIVE_CONTRAST_ENHANCEMENT) {

		adaptiveContrastEnhancement(src, dst, factor, filter);
		return;

	} else {

		if (halfRange > 128.0) {
			compressionFactor = halfRange / 128.0;
		}

		minVal = midVal - 128.0*compressionFactor;
		maxVal = midVal + 128.0*compressionFactor;

	}

	Mat normMat;
	normalize_16(normMat, src, minVal, maxVal);

	if (filter == NO_FILTERING) {
		down_level(dst, normMat);
	} else {
		applyFilter(normMat, dst, filter);
	}


}

double distBetweenPts2f(Point2f& P1, Point2f& P2)
{
    /*
    double retVal;
    retVal = pow((pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y),2)), 0.5);
    return retVal;
    */

    return pow((pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y),2)), 0.5);
}

double distBetweenPts(Point& P1, Point& P2)
{
    // TODO:
    // Possible issue.. see above ^

    double retVal;
    retVal = pow(double(pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y), 2.0)), 0.5);
    return retVal;
}

void drawLinesBetweenPoints(Mat& image, const vector<Point2f>& src, const vector<Point2f>& dst)
{

    Point p1, p2;

    for (unsigned int i = 0; i < src.size(); i++)
    {
        p1 = Point(src.at(i).x*16, src.at(i).y*16);
        p2 = Point(dst.at(i).x*16, dst.at(i).y*16);

        //line(image, p1, p2, CV_RGB(0,0,255), 1, CV_AA, 4);
        circle(image, p2, 4*16, CV_RGB(0,0,255), 1, CV_AA, 4);
    }

}

Point findCentroid(vector<Point>& contour)
{
    Moments momentSet;

    double x,y;

    momentSet = moments(Mat(contour));
    x = momentSet.m10/momentSet.m00;
    y = momentSet.m01/momentSet.m00;

    return Point((int)x,(int)y);
}

Point2f findCentroid2f(vector<Point>& contour)
{
    Moments momentSet;

    double x,y;

    momentSet = moments(Mat(contour));
    x = momentSet.m10/momentSet.m00;
    y = momentSet.m01/momentSet.m00;

    return Point2f(x,y);
}

void createGaussianMatrix(Mat& gaussianMat, double sigmaFactor)
{

    Mat distributionDisplay(Size(640, 480), CV_8UC1);
    Mat binTemp(gaussianMat.size(), CV_8UC1);

    // sigmaFactor says how many standard deviations should span from the center to the nearest edge
    // (i.e. along the shortest axis)


    // What about an elliptical gaussian function?

    Point2f center((float)((double(gaussianMat.size().height-1))/2), (float)((double(gaussianMat.size().width-1))/2));
    Point2f tmpPt;
    double dist = 0.0, average = 0.0, maxVal = 0.0;
    // double sigma = min(gaussianMat.size().width, gaussianMat.size().height)/(2*sigmaFactor);

    // double A = (gaussianMat.size().width*gaussianMat.size().height) / (sigma * pow(2*3.142, 0.5));


    for (int i = 0; i < gaussianMat.size().width; i++)
    {
        for (int j = 0; j < gaussianMat.size().height; j++)
        {
            tmpPt = Point2f(float(j), float(i));
            dist = distBetweenPts2f(center, tmpPt);


            //gaussianMat.at<double>(j,i) = A*exp(-(pow(dist,2)/(2*pow(sigma,2))));
            //gaussianMat.at<double>(j,i) = dist;
            if (dist < max(double(gaussianMat.size().width)/2, double(gaussianMat.size().height)/2))
            {
                gaussianMat.at<double>(j,i) = 1.0;
            }
            else
            {
                gaussianMat.at<double>(j,i) = 0.0;
            }

            average += gaussianMat.at<double>(j,i);

            if (gaussianMat.at<double>(j,i) > maxVal)
            {
                maxVal = gaussianMat.at<double>(j,i);
            }

            //printf("val = %f\n", gaussianMat.at<double>(j,i));

            //gaussianMat.at<double>(j,i) = double(rand()) / RAND_MAX;
        }
    }

    average /= gaussianMat.size().width*gaussianMat.size().height;

    gaussianMat /= average;
    average = 0.0;
    maxVal = 0.0;

    for (int i = 0; i < gaussianMat.size().width; i++)
    {
        for (int j = 0; j < gaussianMat.size().height; j++)
        {
            average += gaussianMat.at<double>(j,i);
            if (gaussianMat.at<double>(j,i) > maxVal)
            {
                maxVal = gaussianMat.at<double>(j,i);
            }
        }
    }

    average /= gaussianMat.size().width*gaussianMat.size().height;

    //printf("%s << average val = %f\n", __FUNCTION__, average);
    //cin.get();

    /*
    convertScaleAbs(gaussianMat, binTemp, (255.0/maxVal), 0);
    svLib::simpleResize(binTemp, distributionDisplay, Size(480, 640));
    //equalizeHist(distributionDisplay, distributionDisplay);
    imshow("gaussianMat", distributionDisplay);
    waitKey(40);
    */
}

void cropImage(Mat& image, Point tl, Point br)
{
    // not compatible with all image types yet...

    int width, height, xOff, yOff;

    //printf("%s << Starting function.\n", __FUNCTION__);

    //printf("%s << TL = (%d, %d); BR = (%d, %d)\n", __FUNCTION__, tl.x, tl.y, br.x, br.y);

    width = abs(br.x-tl.x);
    height = abs(br.y-tl.y);

    xOff = min(tl.x, br.x);
    yOff = min(tl.y, br.y);

    //printf("%s << width = %d, height = %d, xOff = %d, yOff = %d\n", __FUNCTION__, width, height, xOff, yOff);

    //cin.get();

    Mat tmpMat;

    if (image.channels() == 3)
    {
        tmpMat = Mat(height, width, CV_8UC3);
    }
    else if (image.channels() == 1)
    {
        tmpMat = Mat(height, width, CV_8UC1);
    }



    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            //printf("%s << %d / %d\n", __FUNCTION__, i, j);

            if (image.channels() == 3)
            {
                if ((j+yOff < 0) || (j+yOff > image.rows-1) || (i+xOff < 0) || (i+xOff > image.cols-1))
                {
                    tmpMat.at<Vec3b>(j,i)[0] = 0;
                    tmpMat.at<Vec3b>(j,i)[1] = 0;
                    tmpMat.at<Vec3b>(j,i)[2] = 0;
                }
                else
                {
                    tmpMat.at<Vec3b>(j,i) = image.at<Vec3b>(j+yOff,i+xOff);
                }
            }
            else if (image.channels() == 1)
            {
                if ((j+yOff < 0) || (j+yOff > image.rows-1) || (i+xOff < 0) || (i+xOff > image.cols-1))
                {
                    tmpMat.at<unsigned char>(j,i) = 0;
                }
                else
                {
                    tmpMat.at<unsigned char>(j,i) = image.at<unsigned char>(j+yOff,i+xOff);
                }


            }




        }
    }

    //tmpMat.copyTo(image);
    resize(tmpMat, image, Size(width, height)); // working

    //printf("%s << Completing function.\n", __FUNCTION__);

}

void convertVectorToPoint(vector<Point2f>& input, vector<Point>& output)
{
    output.clear();

    for (unsigned int i = 0; i < input.size(); i++)
    {
        output.push_back(Point((int)input.at(i).x, (int)input.at(i).y));
    }
}

void convertVectorToPoint2f(vector<Point>& input, vector<Point2f>& output)
{
    // TODO:
    // Nothing much.

    output.clear();

    for (unsigned int i = 0; i < input.size(); i++)
    {
        output.push_back(Point2f((float)input.at(i).x, (float)input.at(i).y));
    }
}

void simpleResize(Mat& src, Mat& dst, Size size)
{

    dst = Mat::zeros(size, src.type());

    for (int i = 0; i < dst.size().width; i++)
    {
        for (int j = 0; j < dst.size().height; j++)
        {
            if (src.depth() == 1)
            {
                dst.at<unsigned char>(j,i) = src.at<unsigned char>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            }
            else if (src.depth() == 8)
            {
                dst.at<double>(j,i) = src.at<double>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            }
            else if (src.depth() == CV_16U)
            {
                dst.at<unsigned short>(j,i) = src.at<unsigned short>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            }

        }
    }
}

void copyContour(vector<Point>& src, vector<Point>& dst)
{
    // TODO:
    // Make safer.

    dst.clear();

    for (unsigned int i = 0; i < src.size(); i++)
    {
        dst.push_back(src.at(i));
    }
}

void swapElements(vector<Point2f>& corners, int index1, int index2)
{
    // TODO:
    // Nothing much.

    Point2f tempPt;

    tempPt = corners.at(index1);    // copy first element to temp
    corners.at(index1) = corners.at(index2);  // put best element in first
    corners.at(index2) = tempPt;  // copy temp to where best element was
}

void invertMatIntensities(Mat& src, Mat& dst)
{
    dst.release();
    dst = Mat(src.size(), src.type());

    if (src.type() == CV_8UC1)
    {



        for (int iii = 0; iii < src.rows; iii++)
        {
            for (int jjj = 0; jjj < src.cols; jjj++)
            {
                dst.at<unsigned char>(iii,jjj) = 255 - src.at<unsigned char>(iii, jjj);
            }
        }

    }
    else if (src.type() == CV_8UC3)
    {

        // printf("%s << here.\n", __FUNCTION__);

        for (int iii = 0; iii < src.rows; iii++)
        {
            for (int jjj = 0; jjj < src.cols; jjj++)
            {
                //a = src.at<Vec3b>(iii, jjj)[0];
                //z = std::max(std::min(255, int(255 - (1.5*(a - 128)+128))),0);
                //dst.at<Vec3b>(iii, jjj)[0] = z;
                //dst.at<Vec3b>(iii, jjj)[1] = z;
                //dst.at<Vec3b>(iii, jjj)[2] = z;

                dst.at<Vec3b>(iii, jjj)[0] = 255 - src.at<Vec3b>(iii, jjj)[0];
                dst.at<Vec3b>(iii, jjj)[1] = 255 - src.at<Vec3b>(iii, jjj)[1];
                dst.at<Vec3b>(iii, jjj)[2] = 255 - src.at<Vec3b>(iii, jjj)[2];
            }
        }
    }

    //imshow("dst", dst);
    //waitKey();

}

void contourDimensions(vector<Point> contour, double& width, double& height)
{
    // TODO:
    // May want to replace this with something that finds the longest and shortest distances across
    // Because the idea of this is to help determine if it's a square or not.

    // new methodology
    RotatedRect wrapperRectangle;
    Size size;
    vector<Point> contourCpy;
    Point meanPoint;

    // Interpolate contour to get it to an adequate size for "fitEllipse" function
    if (contour.size() < 6)
    {
        for (unsigned int i = 0; i < contour.size()-1; i++)
        {
            contourCpy.push_back(contour.at(i));
            meanPoint.x = (2*contour.at(i).x + 1*contour.at(i+1).x) / 3;
            meanPoint.y = (2*contour.at(i).y + 1*contour.at(i+1).y) / 3;
            contourCpy.push_back(meanPoint);
            meanPoint.x = (1*contour.at(i).x + 2*contour.at(i+1).x) / 3;
            meanPoint.y = (1*contour.at(i).y + 2*contour.at(i+1).y) / 3;
            contourCpy.push_back(meanPoint);
        }

        contourCpy.push_back(contour.at(contour.size()-1));
        meanPoint.x = (2*contour.at(contour.size()-1).x + 1*contour.at(0).x) / 3;
        meanPoint.y = (2*contour.at(contour.size()-1).y + 1*contour.at(0).y) / 3;
        contourCpy.push_back(meanPoint);
        meanPoint.x = (1*contour.at(contour.size()-1).x + 2*contour.at(0).x) / 3;
        meanPoint.y = (1*contour.at(contour.size()-1).y + 2*contour.at(0).y) / 3;
        contourCpy.push_back(meanPoint);

    }
    else
    {
        contourCpy.assign(contour.begin(), contour.end());
    }

    wrapperRectangle = fitEllipse(Mat(contourCpy));
    size = wrapperRectangle.size;
    width = size.width;
    height = size.height;

    // old methodology... (simply using highest and lowest X and Y values..)
    /*
    double xMax = 0.0, yMax = 0.0, xMin = 99999.0, yMin = 99999.0;

    for (unsigned int i = 0; i < contour.size(); i++) {

        if (contour.at(i).x > xMax) {
            xMax = contour.at(i).x;
        }

        if (contour.at(i).y > yMax) {
            yMax = contour.at(i).y;
        }

        if (contour.at(i).x < xMin) {
            xMin = contour.at(i).x;
        }

        if (contour.at(i).y < yMin) {
            yMin = contour.at(i).y;
        }

    }

    width = xMax - xMin;
    height = yMax - yMin;
    */
}

double perpDist(Point2f& P1, Point2f& P2, Point2f& P3)
{
    // TODO:
    // There may be some kind of issue occuring here... check the bug list at the top of this file

    // P3 is the test point
    double u, x, y, d;

    u = ((P3.x - P1.x)*(P2.x - P1.x) + (P3.y - P1.y)*(P2.y - P1.y)) / (pow(double(P2.x - P1.x), 2.0) + pow(double(P2.y - P1.y), 2.0));

    /*
        printf("denominator = %f\n", pow(double(P2.x - P1.x), 2.0) - pow(double(P2.y - P1.y), 2.0));
        printf("P1 = %f,%f\n", P1.x, P1.y);
        printf("P2 = %f,%f\n", P2.x, P2.y);
        printf("P3 = %f,%f\n", P3.x, P3.y);
        printf("u = %f\n", u);
    */

    x = P1.x + u*(P2.x - P1.x);
    y = P1.y + u*(P2.y - P1.y);

    d = pow(pow(P3.x - x, 2.0) + pow(P3.y - y, 2.0),0.5);

    return d;
}

double findMinimumSeparation(vector<Point2f>& pts)
{
    double minSep = 9e50;
    double val = 0.0;

    for (unsigned int i = 0; i < pts.size(); i++)
    {
        for (unsigned int j = i+1; j < pts.size(); j++)
        {
            val = norm(pts.at(i)-pts.at(j));
            if (val < minSep)
            {
                minSep = val;
            }
        }
    }

    return minSep;
}

Point2f meanPoint(Point2f& P1, Point2f& P2)
{
    return Point2f((P1.x+P2.x)/2, (P1.y+P2.y)/2);
}

void transferElement(vector<Point2f>& dst, vector<Point2f>& src, int index)
{
    Point2f pointCpy;

    pointCpy = src.at(index);

    // Move from old one to new one
    dst.push_back(pointCpy);

    // Replace and shift points in old one
    for (unsigned int i = index; i < src.size()-1; i++)
    {
        src.at(i) = src.at(i+1);
    }

    // Truncate the original vector (effectively discarding old point)
    src.pop_back();
}


bool checkIfActuallyGray(const Mat& im) {

	bool retVal = true;

	for (int iii = 0; iii < im.rows; iii++) {
		for (int jjj = 0; jjj < im.cols; jjj++) {

			if (im.at<Vec3b>(iii,jjj)[0] != im.at<Vec3b>(iii,jjj)[1]) {
				return false;
			}

			if (im.at<Vec3b>(iii,jjj)[2] != im.at<Vec3b>(iii,jjj)[1]) {
				return false;
			}

		}
	}

	return retVal;

}

void findPercentiles(const Mat& img, double *vals, double *percentiles, unsigned int num) {

	//double median = 0.0;

	Mat mx;

	unsigned int *aimedPixelCounts;
	aimedPixelCounts = new unsigned int[num];

	for (unsigned int iii = 0; iii < num; iii++) {
		aimedPixelCounts[iii] = (unsigned int) (((double) img.rows) * ((double) img.cols) * percentiles[iii]);
		//printf("%s << aimedPixelCounts[%d] = %d (%f)\n", __FUNCTION__, iii, aimedPixelCounts[iii], percentiles[iii]);
	}

	//cin.get();

	if (img.type() == CV_16UC1) {
		img.convertTo(mx, CV_32FC1);
	}

	MatND hist;
	int channels[] = {0};
	int ibins = 65536;
	int histSize[] = {ibins};
	float iranges[] = { 0, 65535 };
	const float* ranges[] = { iranges };

	calcHist(&mx, 1, channels, Mat(), hist, 1, histSize, ranges);

	unsigned int pointsTally = 0;

	bool allCountsReached = false;

	for (int i = 0; i < ibins; i++) {

		pointsTally += (unsigned int)(hist.at<float>(i));

		if (pointsTally > 0) {
			//printf("%s << hist(%d) = %f\n", __FUNCTION__, i, hist.at<float>(i));
		}

		allCountsReached = true;

		for (unsigned int iii = 0; iii < num; iii++) {
			if (pointsTally <= aimedPixelCounts[iii]) {
				vals[iii] = i;
				//printf("%s << vals[%d] = %f\n", __FUNCTION__, iii, vals[iii]);
				allCountsReached = false;
			}
		}

		if (allCountsReached) {
			return;
		}


		//printf("%s << histval(%d) = %f\n", __FUNCTION__, i, hist.at<float>(i));
	}

	//cin.get();

}

void shiftIntensities(Mat& im, double scaler, double shifter, double downer) {

	double val;
	for (int iii = 0; iii < im.rows; iii++) {
		for (int jjj = 0; jjj < im.cols; jjj++) {

			val = ((double) im.at<unsigned char>(iii,jjj));

			val -= downer;

			val *= scaler;

			val += (downer + shifter);

			im.at<unsigned char>(iii,jjj) = ((unsigned char) val);

		}
	}

}

void findIntensityValues(double *vals, Mat& im, Mat& mask) {

	vals[0] = 9e99;
	vals[1] = -9e99;
	vals[2] = 0.0;

	unsigned int activeCount = countNonZero(mask);

	if (activeCount == 0) {
		return;
	}

	unsigned int hist[256];

	for (unsigned int iii = 0; iii < 256; iii++) {
		hist[iii] = 0;
	}

	for (int iii = 0; iii < im.rows; iii++) {
		for (int jjj = 0; jjj < im.cols; jjj++) {

			if (mask.at<unsigned char>(iii,jjj) != 0) {
				vals[2] += ((double) im.at<unsigned char>(iii,jjj)) / ((double) activeCount);

				if (((double) im.at<unsigned char>(iii,jjj)) < vals[0]) {
					vals[0] = ((double) im.at<unsigned char>(iii,jjj));
				}

				if (((double) im.at<unsigned char>(iii,jjj)) > vals[1]) {
					vals[1] = ((double) im.at<unsigned char>(iii,jjj));
				}

				hist[im.at<unsigned char>(iii,jjj)]++;

			}

		}
	}

	unsigned int intensityCount = 0;
	int intensityPtr = -1;
	unsigned int medianCount = countNonZero(mask);

	while (intensityCount < (medianCount/2)) {
		intensityPtr++;
		intensityCount += hist[intensityPtr];
	}

	vals[3] = intensityPtr;


}

void cScheme::setupLookupTable(unsigned int depth) {

	unsigned int maxIndex, level;

	if (depth == 2) {
		maxIndex = 65536;
	} else if (depth == 1) {
		maxIndex = 256;
	} else {
		maxIndex = 256;
	}

	for (unsigned int iii = 0; iii < maxIndex; iii++) {

		level = (int) floor(double((iii*(MAP_LENGTH-1))/((double) (maxIndex-1)))); // for 16 bits/pixel

		if (depth == 2) {
			lookupTable_2[iii][2] = (unsigned short) (255.0 * red[level]);
			lookupTable_2[iii][1] = (unsigned short) (255.0 * green[level]);
			lookupTable_2[iii][0] = (unsigned short) (255.0 * blue[level]);

			//printf("%s << LT(%d) = (%d, %d, %d)\n", __FUNCTION__, iii, lookupTable_2[iii][2], lookupTable_2[iii][1], lookupTable_2[iii][0]);
		} else if (depth == 1) {
			lookupTable_1[iii][2] = (unsigned char) (255.0 * red[level]);
			lookupTable_1[iii][1] = (unsigned char) (255.0 * green[level]);
			lookupTable_1[iii][0] = (unsigned char) (255.0 * blue[level]);
			//printf("%s << LT(%d) = (%d, %d, %d)\n", __FUNCTION__, iii, lookupTable_1[iii][2], lookupTable_1[iii][1], lookupTable_1[iii][0]);
		}


	}

}

void obtainEightBitRepresentation(Mat& src, Mat& dst) {
	if (src.type() == CV_8UC1) {
		dst = src;
	} else if (src.type() == CV_8UC3) {
		cvtColor(src, dst, CV_RGB2GRAY);
	} else if (src.type() == CV_16UC1) {
		Mat nMat;
		double currMin, currMax;
		minMaxLoc(src, &currMin, &currMax);
		normalize_16(nMat, src, currMin, currMax);
		down_level(dst, nMat);
	} else if (src.type() == CV_16UC3) {
		Mat nMat, tMat;
		cvtColor(src, tMat, CV_RGB2GRAY);
		double currMin, currMax;
		minMaxLoc(tMat, &currMin, &currMax);
		normalize_16(nMat, tMat, currMin, currMax);
		down_level(dst, nMat);
	}
}

void obtainColorRepresentation(Mat& src, Mat& dst) {
	if (src.type() == CV_8UC1) {
		cvtColor(src, dst, CV_GRAY2RGB);
	} else if (src.type() == CV_8UC3) {
		dst = src;
	} else if (src.type() == CV_16UC1) {
		Mat nMat, tMat;
		double currMin, currMax;
		minMaxLoc(src, &currMin, &currMax);
		normalize_16(nMat, src, currMin, currMax);
		down_level(tMat, nMat);
		cvtColor(tMat, dst, CV_GRAY2RGB);
	} else if (src.type() == CV_16UC3) {
		Mat nMat, tMat, tMat2;
		double currMin, currMax;
		cvtColor(src, tMat, CV_RGB2GRAY);
		minMaxLoc(tMat, &currMin, &currMax);
		normalize_16(nMat, tMat, currMin, currMax);
		down_level(tMat2, nMat);
		cvtColor(tMat2, dst, CV_GRAY2RGB);
	}
}



void fix_bottom_right(Mat& mat) {

	for (int iii = 634; iii < 640; iii++) {
		mat.at<unsigned short>(479, iii) = std::max( std::min(( (unsigned int) ((mat.at<unsigned short>(478, iii) + mat.at<unsigned short>(479, iii-1)) / 2.0) ), (unsigned int) 65535) , (unsigned int) 0);
	}

}

void down_level(Mat& dst, Mat& src) {

    // Currently only works for a single channel image i.e. CV_16UC1
	dst = Mat(src.rows, src.cols, CV_8UC1);

    //unsigned int val;
    for (int i = 0; i < dst.size().height; i++) {
        for (int j = 0; j < dst.size().width; j++) {
           dst.at<unsigned char>(i,j) = (src.at<unsigned short>(i,j)) / 256;
        }

    }
}

void reduceToPureImage(cv::Mat& dst, cv::Mat& src) {


	if (dst.cols == 0) {
		if (src.type() == CV_16UC3) {
			dst = Mat(src.size(), CV_16UC1);
		} else if (src.type() == CV_8UC3) {
			dst = Mat(src.size(), CV_8UC1);
		}

	}

		for (int iii = 0; iii < src.cols; iii++) {
			for (int jjj = 0; jjj < src.rows; jjj++) {
				if (src.type() == CV_16UC3) {
					if (((src.at<Vec3s>(jjj,iii)[0]) == (src.at<Vec3s>(jjj,iii)[1])) && ((src.at<Vec3s>(jjj,iii)[2]) == (src.at<Vec3s>(jjj,iii)[1]))) {
						dst.at<unsigned short>(jjj,iii) = src.at<Vec3s>(jjj,iii)[0];
					} else {
						dst.at<unsigned short>(jjj,iii) = (unsigned short) (0.299 * (double) src.at<Vec3s>(jjj,iii)[0] + 0.587 * (double) src.at<Vec3s>(jjj,iii)[1] + 0.114 * (double) src.at<Vec3s>(jjj,iii)[2]);
					}

				} else if (src.type() == CV_8UC3) {
					if (((src.at<Vec3b>(jjj,iii)[0]) == (src.at<Vec3b>(jjj,iii)[1])) && ((src.at<Vec3b>(jjj,iii)[2]) == (src.at<Vec3b>(jjj,iii)[1]))) {
						dst.at<unsigned char>(jjj,iii) = src.at<Vec3b>(jjj,iii)[0];
					} else {
						dst.at<unsigned char>(jjj,iii) = (unsigned char) (0.299 * (double) src.at<Vec3b>(jjj,iii)[0] + 0.587 * (double) src.at<Vec3b>(jjj,iii)[1] + 0.114 * (double) src.at<Vec3b>(jjj,iii)[2]);
					}

				}

			}
		}
}

void addBorder(Mat& inputMat, int borderSize) {
    Mat newMat = Mat::zeros(inputMat.rows + 2*borderSize, inputMat.cols + 2*borderSize, CV_8UC3);

    for (int i = borderSize; i < newMat.rows-borderSize; i++) {
        for (int j = borderSize; j < newMat.cols-borderSize; j++) {
            newMat.at<Vec3b>(i,j) = inputMat.at<Vec3b>(i-borderSize, j-borderSize);
        }
    }

    newMat.copyTo(inputMat);

}

void normalize_64_vec(Mat& dst, Mat& src) {

    //printf("%s << Entered...\n", __FUNCTION__);

    dst = Mat(src.size(), src.type());

    double minVal = 9e50, maxVal = -9e50;

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            for (int k = 0; k < src.channels(); k++) {
                if (src.at<Vec3d>(i,j)[k] > maxVal) {
                    maxVal = src.at<Vec3d>(i,j)[k];
                }

                if (src.at<Vec3d>(i,j)[k] < minVal) {
                    minVal = src.at<Vec3d>(i,j)[k];
                }
            }


        }
    }

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            for (int k = 0; k < src.channels(); k++) {
                dst.at<Vec3d>(i,j)[k] = max(min(((src.at<Vec3d>(i,j)[k] - minVal) / (maxVal - minVal)), 1.0),0.0);
            }


        }
    }
}

void normalize_16(Mat& dst, const Mat& src, double dblmin, double dblmax) {

    unsigned short minv, maxv;
    bool presetLimits;

    double lower_bound = 0.0, upper_bound = 65535.0;

    if ((dblmin > -1) && (dblmax > -1)) {
        presetLimits = true;
    } else {
		presetLimits = false;
	}

    minv = 65535;
	maxv = 0;

    //printf("%s << min = %d; max = %d\n", __FUNCTION__, min, max);

    unsigned short val; //, new_val;
    //double limitTester;

    Size matSize = src.size();
    src.copyTo(dst);

    // Safety checks
	if ((src.size().height != dst.size().height) || (src.size().width != dst.size().width)) {
		printf("svLib::normalize_16() << Image dimensions don't match.\n");
		return;
	}

    if (1) { // (!presetLimits) {

        for (int i = 0; i < matSize.height; i++) {
            for (int j = 0; j < matSize.width; j++) {
                val = src.at<unsigned short>(i,j);             // how do I incorporate channels?

                if (val < minv) {
                    minv = val;
                }

                if (val > maxv) {
                    maxv = val;
                }
            }
        }

	}

	//unsigned short abs_min = std::max(minv, (unsigned short) dblmin);
	//unsigned short abs_max = std::min(maxv, (unsigned short) dblmax);

	#pragma omp parallel for
	for (int i = 0; i < matSize.height; i++) {
        for (int j = 0; j < matSize.width; j++) {
            dst.at<unsigned short>(i,j) = std::max(minv, std::min(maxv, src.at<unsigned short>(i,j)));
		}
	}

	// Compare actual min with the desired min in order to determine lower bound...



	double total_range = abs(std::min(dblmin, double(minv)) - std::max(dblmax, double(maxv)));

	//printf("%s << (minv / dblmin) = (%f, %f); (maxv / dblmax) = (%f, %f)\n", __FUNCTION__, double(minv), dblmin, double(maxv), dblmax);

	if (double(minv) > dblmin) {
		lower_bound = 65535.0 * (abs(double(minv) - dblmin) / total_range);
	}

	if (double(maxv) < dblmax) {
		upper_bound = 65535.0 - 65535.0 * (abs(double(maxv) - dblmax) / total_range);
	}

	//printf("%s << lower_bound = (%f); upper_bound = (%f)\n", __FUNCTION__, lower_bound, upper_bound);


	normalize(dst, dst, lower_bound, upper_bound, NORM_MINMAX);

	return;

	/*
    //printf("PURPLE MONKEY!\n");

    //printf("%s << Debug %d.\n", __FUNCTION__, -1);

    // Perform image processing
    for (int i = 0; i < matSize.height; i++) {
        for (int j = 0; j < matSize.width; j++) {
            for (int k = 0; k < src.channels(); k++) {
                val = src.at<unsigned short>(i,j);             // how do I incorporate channels?

                if (val < min) {
                    val = min;
                } else if (val > max) {
                    val = max;
                }
                // attempt at normalization

                limitTester = std::max(double(val - min), 0.0);
                limitTester *= 65535.0/(max-min);

                if (limitTester < 0) {
                    //printf("%s << limitTester too low: %f\n", __FUNCTION__, limitTester);
                    //cin.get();
			limitTester = 0;
                } else if (limitTester > 65535) {
                    //printf("%s << limitTester too high: %f\n", __FUNCTION__, limitTester);
                    //cin.get();
			limitTester = 65535;
                }

                new_val = (unsigned short) limitTester;

                //new_val = (unsigned short) (65535*((double(val - min))/(double(max - min))));   // 255 for VXL..

                //printf("%s << val = %d; new_val = %d\n", __FUNCTION__, val, new_val);

                if (new_val == 0) {
                    if (1) {
                        //printf("%s << val = %d; new_val = %d\n", __FUNCTION__, val, new_val);
                        //cin.get();
                    }
                }

                dst.at<unsigned short>(i,j) = new_val;
            }
        }
    }

    //printf("%s << min = %d; max = %d\n", __FUNCTION__, min, max);
    //cin.get();
	*/
}

cScheme::cScheme() {
	load_standard(100);
}

cScheme::cScheme(double *r, double *g, double *b, int len) {
	code = CUSTOM;
	rx = r;
	gx = g;
	bx = b;
	length = len;
}

cScheme::~cScheme() { }

void cScheme::create_long_map() {

	int element, i;

	double e, segs, iy, M, fact;

	segs = (double) (length-1);
	M = (double) MAP_LENGTH;

	// Fill in initial elements
	red[0] = rx[0];
	green[0] = gx[0];
	blue[0] = bx[0];

	// Interpolate
	for (i = 1; i < MAP_LENGTH-1; i++)
	{
		element = (int) floor((i*segs)/MAP_LENGTH);

		e = (double) element;
		iy = (double) i;
		fact = segs*iy/M - e;

		red[i] = rx[element] + fact*(rx[element+1] - rx[element]);
		green[i] = gx[element] + fact*(gx[element+1] - gx[element]);
		blue[i] = bx[element] + fact*(bx[element+1] - bx[element]);

		//printf("red[%d] = %f\n", i, red[i]);

	}

	// Fill in final elements
	red[MAP_LENGTH-1] = rx[length-1];
	green[MAP_LENGTH-1] = gx[length-1];
	blue[MAP_LENGTH-1] = bx[length-1];

}

// Should be:
// Every ten units from 100 - 990 corresponds to a new colour scheme
// 100*N + 2*v + a: v corresponds to a different 'variation'; a can equal 1 or 0 - if 1, can include black
// and/or white (a = 0 is basically a safe version [if needed] for image fusion)

void cScheme::load_standard(int mapCode, int mapParam) {

	code = mapCode + mapParam;

	if (code == GRAYSCALE) {
		length = 2;
		double rr [] = { 0, 1 };
		double gg [] = { 0, 1 };
		double bb [] = { 0, 1 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (GRAYSCALE + 1)) {
		length = 2;
		double rr [] = { 0.2, 0.8 };
		double gg [] = { 0.2, 0.8 };
		double bb [] = { 0.2, 0.8 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == CIECOMP) {
		length = 33;
		double rr [] = { 1.000000, 0.765154, 0.514649, 0.264229, 0.082532, 0.004867, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000541, 0.062517, 0.246214, 0.493080, 0.743824, 0.931596, 0.996408, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg [] = { 1.000000, 1.000000, 1.000000, 1.000000, 0.964432, 0.859914, 0.717154, 0.572201, 0.435801, 0.319831, 0.214279, 0.109202, 0.031115, 0.002438, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000889, 0.020277, 0.077613, 0.155367, 0.234768, 0.334034, 0.475222, 0.637346, 0.798860, 0.922554, 0.971514, 0.982362, 0.991155, 0.999981 };
		double bb [] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.938789, 0.756126, 0.508812, 0.258234, 0.070001, 0.004301, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001793, 0.069474, 0.251770, 0.498992, 0.749491, 1.000000 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (CIECOMP + 1)) {
		length = 27;
		double rr [] = { 0.264229, 0.082532, 0.004867, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000541, 0.062517, 0.246214, 0.493080, 0.743824, 0.931596, 0.996408, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg [] = { 1.000000, 0.964432, 0.859914, 0.717154, 0.572201, 0.435801, 0.319831, 0.214279, 0.109202, 0.031115, 0.002438, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000889, 0.020277, 0.077613, 0.155367, 0.234768, 0.334034, 0.475222, 0.637346, 0.798860, 0.922554, 0.971514 };
		double bb [] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.938789, 0.756126, 0.508812, 0.258234, 0.070001, 0.004301, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001793, 0.069474, 0.251770 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == CIECOMP_ALT_1) {
		length = 33;
		double rr [] = { 0.999951, 0.764763, 0.513833, 0.262280, 0.070539, 0.004449, 0.000982, 0.000000, 0.014764, 0.079511, 0.164042, 0.247835, 0.342405, 0.474597, 0.620949, 0.766563, 0.888141, 0.936537, 0.955818, 0.977795, 0.996042, 1.000000, 0.999773, 0.999860, 1.000000, 0.999750, 0.999432, 0.999660, 0.999868, 0.999382, 0.999179, 0.999381, 0.999959 };
		double gg [] = { 1.000000, 1.000000, 1.000000, 0.997055, 0.970402, 0.863627, 0.716516, 0.570543, 0.432488, 0.318623, 0.214564, 0.108254, 0.027442, 0.002708, 0.000136, 0.000000, 0.000011, 0.000033, 0.000094, 0.000746, 0.017982, 0.076261, 0.155089, 0.233998, 0.330934, 0.473361, 0.636642, 0.802257, 0.931872, 0.974355, 0.982178, 0.991650, 1.000000 };
		double bb [] = { 1.000000, 1.000000, 1.000000, 1.000000, 0.999997, 0.999995, 0.999977, 0.999922, 0.999902, 0.999551, 0.999534, 0.998992, 0.999522, 0.999971, 0.999664, 0.999221, 0.950204, 0.758209, 0.505805, 0.252367, 0.056835, 0.003052, 0.000875, 0.000378, 0.000067, 0.000073, 0.000477, 0.004040, 0.061700, 0.249519, 0.498347, 0.748001, 0.997975 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (CIECOMP_ALT_1 + 1)) {
		length = 33;
		double rr [] = { 0.000021, 0.000002, 0.000016, 0.000234, 0.000162, 0.003354, 0.038452, 0.102054, 0.165385, 0.228699, 0.291374, 0.369664, 0.475506, 0.586630, 0.695216, 0.803475, 0.893515, 0.930827, 0.944857, 0.961729, 0.978009, 0.993506, 0.999136, 0.998879, 0.999137, 0.999905, 0.999888, 0.999957, 0.999987, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg [] = { 1.000000, 0.898136, 0.788027, 0.679369, 0.570642, 0.461780, 0.370245, 0.292206, 0.213912, 0.134749, 0.055599, 0.007772, 0.000272, 0.000008, 0.000004, 0.000009, 0.000007, 0.000014, 0.000011, 0.000027, 0.000088, 0.004334, 0.037984, 0.096488, 0.155866, 0.214581, 0.274564, 0.358132, 0.474443, 0.596959, 0.719264, 0.841713, 0.964617 };
		double bb [] = { 0.999914, 1.000000, 1.000000, 0.999790, 0.998744, 0.998603, 0.999497, 0.999707, 0.999911, 0.999704, 0.999322, 0.999702, 1.000000, 0.999979, 0.999586, 0.998111, 0.956023, 0.819800, 0.630929, 0.440522, 0.253413, 0.086470, 0.011451, 0.000127, 0.000003, 0.000001, 0.000015, 0.000453, 0.001038, 0.000450, 0.000225, 0.000199, 0.000161 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == CIECOMP_ALT_2) {
		length = 33;
		double rr [] = { 0.999911, 0.764767, 0.513903, 0.262540, 0.070481, 0.004493, 0.000952, 0.000000, 0.014727, 0.079215, 0.164105, 0.250787, 0.300361, 0.256821, 0.171949, 0.088134, 0.087815, 0.263848, 0.518803, 0.760556, 0.943220, 0.993926, 0.999249, 0.999896, 1.000000, 0.999897, 0.999731, 0.999881, 0.999938, 0.999386, 0.999076, 0.999067, 0.999460 };
		double gg [] = { 1.000000, 0.999999, 0.999994, 0.997249, 0.969474, 0.863789, 0.716632, 0.570567, 0.432580, 0.318934, 0.214508, 0.108219, 0.027005, 0.001377, 0.000081, 0.000009, 0.000000, 0.000031, 0.000254, 0.000000, 0.018056, 0.077171, 0.155264, 0.234004, 0.331232, 0.473361, 0.636852, 0.802178, 0.931685, 0.974284, 0.982440, 0.991755, 1.000000 };
		double bb [] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.999990, 0.999945, 0.999878, 0.999520, 0.999207, 0.994201, 0.946503, 0.762066, 0.509103, 0.254220, 0.060146, 0.006524, 0.000870, 0.000094, 0.000032, 0.000035, 0.000022, 0.000005, 0.000019, 0.000073, 0.000473, 0.005081, 0.060686, 0.248644, 0.498482, 0.748191, 0.998655 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (CIECOMP_ALT_2 + 1)) {
		length = 26;
		double rr [] = { 0.513903, 0.262540, 0.070481, 0.004493, 0.000952, 0.000000, 0.014727, 0.079215, 0.164105, 0.250787, 0.300361, 0.256821, 0.171949, 0.518803, 0.760556, 0.943220, 0.993926, 0.999249, 0.999896, 1.000000, 0.999897, 0.999731, 0.999881, 0.999938, 0.999386, 0.999076 };
		double gg [] = { 0.999994, 0.997249, 0.969474, 0.863789, 0.716632, 0.570567, 0.432580, 0.318934, 0.214508, 0.108219, 0.027005, 0.001377, 0.000081, 0.000254, 0.000000, 0.018056, 0.077171, 0.155264, 0.234004, 0.331232, 0.473361, 0.636852, 0.802178, 0.931685, 0.974284, 0.982440 };
		double bb [] = { 1.000000, 1.000000, 1.000000, 1.000000, 0.999990, 0.999945, 0.999878, 0.999520, 0.999207, 0.994201, 0.946503, 0.762066, 0.509103, 0.000870, 0.000094, 0.000032, 0.000035, 0.000022, 0.000005, 0.000019, 0.000073, 0.000473, 0.005081, 0.060686, 0.248644, 0.498482 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == CIECOMP_ALT_3) {
		length = 33;
		double rr [] = { 0.000021, 0.000002, 0.000016, 0.000234, 0.000162, 0.003354, 0.038452, 0.102054, 0.165385, 0.228699, 0.291374, 0.369664, 0.475506, 0.586630, 0.695216, 0.803475, 0.893515, 0.930827, 0.944857, 0.961729, 0.978009, 0.993506, 0.999136, 0.998879, 0.999137, 0.999905, 0.999888, 0.999957, 0.999987, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg [] = { 1.000000, 0.898136, 0.788027, 0.679369, 0.570642, 0.461780, 0.370245, 0.292206, 0.213912, 0.134749, 0.055599, 0.007772, 0.000272, 0.000008, 0.000004, 0.000009, 0.000007, 0.000014, 0.000011, 0.000027, 0.000088, 0.004334, 0.037984, 0.096488, 0.155866, 0.214581, 0.274564, 0.358132, 0.474443, 0.596959, 0.719264, 0.841713, 0.964617 };
		double bb [] = { 0.999914, 1.000000, 1.000000, 0.999790, 0.998744, 0.998603, 0.999497, 0.999707, 0.999911, 0.999704, 0.999322, 0.999702, 1.000000, 0.999979, 0.999586, 0.998111, 0.956023, 0.819800, 0.630929, 0.440522, 0.253413, 0.086470, 0.011451, 0.000127, 0.000003, 0.000001, 0.000015, 0.000453, 0.001038, 0.000450, 0.000225, 0.000199, 0.000161 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (CIECOMP_ALT_3 + 1)) {
		length = 27;
		double rr [] = { 0.000234, 0.000162, 0.003354, 0.038452, 0.102054, 0.165385, 0.228699, 0.291374, 0.369664, 0.475506, 0.586630, 0.695216, 0.803475, 0.893515, 0.930827, 0.944857, 0.961729, 0.978009, 0.993506, 0.999136, 0.998879, 0.999137, 0.999905, 0.999888, 0.999957, 0.999987, 1.000000 };
		double gg [] = { 0.679369, 0.570642, 0.461780, 0.370245, 0.292206, 0.213912, 0.134749, 0.055599, 0.007772, 0.000272, 0.000008, 0.000004, 0.000009, 0.000007, 0.000014, 0.000011, 0.000027, 0.000088, 0.004334, 0.037984, 0.096488, 0.155866, 0.214581, 0.274564, 0.358132, 0.474443, 0.596959 };
		double bb [] = { 0.999790, 0.998744, 0.998603, 0.999497, 0.999707, 0.999911, 0.999704, 0.999322, 0.999702, 1.000000, 0.999979, 0.999586, 0.998111, 0.956023, 0.819800, 0.630929, 0.440522, 0.253413, 0.086470, 0.011451, 0.000127, 0.000003, 0.000001, 0.000015, 0.000453, 0.001038, 0.000450 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == BLACKBODY) {
		length = 33;
		double rr [] = { 1.000000, 1.000000, 1.000000, 0.999999, 0.999996, 0.999675, 0.999319, 0.999516, 0.999489, 0.999604, 0.999995, 1.000000, 1.000000, 0.999997, 0.999865, 1.000000, 0.983105, 0.909532, 0.813694, 0.720501, 0.626450, 0.532048, 0.438575, 0.343988, 0.250932, 0.157250, 0.062752, 0.007126, 0.000412, 0.000324, 0.000000, 0.000046, 0.000258 };
		double gg [] = { 0.000000, 0.086618, 0.180586, 0.274778, 0.368677, 0.462417, 0.555966, 0.649320, 0.743804, 0.838791, 0.930766, 0.984766, 1.000000, 0.999999, 0.999942, 0.996948, 0.982010, 0.911528, 0.815235, 0.720867, 0.626425, 0.531084, 0.437229, 0.344078, 0.250634, 0.156937, 0.066379, 0.010972, 0.000155, 0.000066, 0.000106, 0.000057, 0.000028 };
		double bb [] = { 0.000000, 0.000028, 0.000047, 0.000017, 0.000006, 0.000011, 0.000034, 0.000020, 0.000028, 0.000211, 0.011424, 0.076038, 0.239387, 0.427148, 0.616124, 0.803833, 0.950133, 0.997841, 0.997676, 0.997875, 0.997774, 0.997078, 0.997007, 0.996407, 0.996357, 0.997493, 0.996829, 0.944413, 0.813917, 0.672345, 0.531919, 0.390918, 0.251743 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (BLACKBODY + 1)) {
		length = 33;
		double rr [] = { 1.000000, 0.999906, 0.999708, 0.999539, 0.999552, 0.999235, 0.999463, 0.999773, 0.999862, 0.999950, 0.999895, 0.999494, 0.999077, 0.999141, 0.998829, 1.000000, 0.986819, 0.938395, 0.875514, 0.812872, 0.750901, 0.688290, 0.625432, 0.562228, 0.499214, 0.436045, 0.374025, 0.311043, 0.250516, 0.188187, 0.125189, 0.062544, 0.000095 };
		double gg [] = { 0.000000, 0.058804, 0.121131, 0.183065, 0.245874, 0.308954, 0.371402, 0.433194, 0.494831, 0.557439, 0.619737, 0.682929, 0.746057, 0.808921, 0.872426, 0.935061, 0.969323, 0.940452, 0.876631, 0.813228, 0.749850, 0.687075, 0.624668, 0.561989, 0.498919, 0.436440, 0.374136, 0.311701, 0.249296, 0.187608, 0.125145, 0.062669, 0.000080 };
		double bb [] = { 0.000012, 0.000028, 0.000042, 0.000152, 0.000239, 0.000069, 0.000387, 0.001237, 0.001583, 0.001240, 0.000291, 0.000238, 0.000232, 0.000184, 0.000516, 0.002223, 0.027364, 0.120060, 0.246693, 0.371988, 0.497032, 0.622486, 0.747998, 0.875008, 0.971356, 0.998454, 0.997949, 0.997694, 0.997953, 0.998069, 0.998122, 0.998256, 0.998504 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == RAINBOW) {
		length = 33;
		double rr [] = { 0.000000, 0.086173, 0.179998, 0.275159, 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.000213, 0.000960, 0.058315, 0.229594, 0.461803, 0.696663, 0.901663, 0.985886, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086, 0.999625, 0.999794, 0.999880, 1.000000 };
		double gg [] = { 0.000129, 0.000054, 0.000053, 0.000064, 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999975, 1.000000, 0.999995, 0.999990, 0.999973, 0.999093, 0.970101, 0.819314, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121, 0.436213, 0.623769, 0.811851, 1.000000 };
		double bb [] = { 0.000000, 0.173973, 0.362471, 0.550963, 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.478570, 0.243157, 0.060479, 0.004108, 0.000011, 0.000015, 0.000009, 0.000124, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726, 0.717459, 0.810571, 0.904202, 0.998013 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (RAINBOW + 1)) {
		length = 25;
		double rr [] = { 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.000213, 0.000960, 0.058315, 0.229594, 0.461803, 0.696663, 0.901663, 0.985886, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086 };
		double gg [] = { 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999975, 1.000000, 0.999995, 0.999990, 0.999973, 0.999093, 0.970101, 0.819314, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121 };
		double bb [] = { 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.478570, 0.243157, 0.060479, 0.004108, 0.000011, 0.000015, 0.000009, 0.000124, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == RAINBOW_ALT_1) {
		length = 11;
		double rr [] = { 0.00, 0.50, 0.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.50, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 1.00 };
		double bb [] = { 0.00, 1.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (RAINBOW_ALT_1 + 1)) {
		length = 9;
		double rr [] = { 0.50, 0.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.50, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.50 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == RAINBOW_ALT_2) {
		length = 13;
		double rr [] = { 0.00, 0.50, 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00, 0.50, 1.00 };
		double bb [] = { 0.00, 0.50, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (RAINBOW_ALT_2 + 1)) {
		length = 9;
		double rr [] = { 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == RAINBOW_ALT_3) {
		length = 13;
		double rr [] = { 0.00, 0.50, 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00, 0.50, 1.00 };
		double bb [] = { 0.00, 0.50, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (RAINBOW_ALT_3 + 1)) {
		length = 9;
		double rr [] = { 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == RAINBOW_ALT_4) {
		length = 33;
		double rr [] = { 0.000000, 0.086173, 0.179998, 0.275159, 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.000213, 0.000960, 0.058315, 0.229594, 0.461803, 0.696663, 0.901663, 0.985886, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086, 0.999625, 0.999794, 0.999880, 1.000000 };
		double gg [] = { 0.000129, 0.000054, 0.000053, 0.000064, 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999975, 1.000000, 0.999995, 0.999990, 0.999973, 0.999093, 0.970101, 0.819314, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121, 0.436213, 0.623769, 0.811851, 1.000000 };
		double bb [] = { 0.000000, 0.173973, 0.362471, 0.550963, 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.478570, 0.243157, 0.060479, 0.004108, 0.000011, 0.000015, 0.000009, 0.000124, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726, 0.717459, 0.810571, 0.904202, 0.998013 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (RAINBOW_ALT_4 + 1)) {
		length = 22;
		double rr [] = { 0.275159, 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.461803, 0.696663, 0.901663, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086, 0.999625 };
		double gg [] = { 0.000064, 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999973, 0.999093, 0.970101, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121, 0.436213 };
		double bb [] = { 0.550963, 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.000011, 0.000015, 0.000009, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726, 0.717459 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == IRON) {
		length = 33;
		double rr [] = { 0.000137, 0.044733, 0.091636, 0.138374, 0.185449, 0.235309, 0.310152, 0.409810, 0.509259, 0.608080, 0.707532, 0.784229, 0.832226, 0.874605, 0.914548, 0.957131, 0.990374, 0.999789, 0.999559, 0.999753, 0.999893, 0.999713, 0.999243, 0.999138, 0.998799, 0.998982, 0.999794, 0.999992, 0.999997, 0.999947, 0.998754, 0.998419, 0.998206 };
		double gg [] = { 0.000218, 0.000655, 0.001318, 0.002240, 0.002696, 0.004508, 0.015631, 0.033312, 0.051963, 0.070414, 0.088750, 0.129074, 0.197559, 0.271724, 0.346554, 0.421253, 0.486973, 0.528391, 0.559335, 0.591508, 0.623114, 0.657761, 0.707739, 0.770047, 0.832696, 0.895595, 0.958334, 0.994681, 0.999982, 1.000000, 1.000000, 1.000000, 1.000000 };
		double bb [] = { 0.000000, 0.087421, 0.182004, 0.276794, 0.372322, 0.463058, 0.506880, 0.513951, 0.520935, 0.528776, 0.535606, 0.503864, 0.411574, 0.308960, 0.206618, 0.103307, 0.024739, 0.001474, 0.003034, 0.003601, 0.005707, 0.007447, 0.006697, 0.005752, 0.004690, 0.002699, 0.007463, 0.077716, 0.244948, 0.434054, 0.622376, 0.810788, 0.998917 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (IRON + 1)) {
		length = 25;
		double rr [] = { 0.185449, 0.235309, 0.310152, 0.409810, 0.509259, 0.608080, 0.707532, 0.784229, 0.832226, 0.874605, 0.914548, 0.957131, 0.990374, 0.999789, 0.999559, 0.999753, 0.999893, 0.999713, 0.999243, 0.999138, 0.998799, 0.998982, 0.999794, 0.999992, 0.999997 };
		double gg [] = { 0.002696, 0.004508, 0.015631, 0.033312, 0.051963, 0.070414, 0.088750, 0.129074, 0.197559, 0.271724, 0.346554, 0.421253, 0.486973, 0.528391, 0.559335, 0.591508, 0.623114, 0.657761, 0.707739, 0.770047, 0.832696, 0.895595, 0.958334, 0.994681, 0.999982 };
		double bb [] = { 0.372322, 0.463058, 0.506880, 0.513951, 0.520935, 0.528776, 0.535606, 0.503864, 0.411574, 0.308960, 0.206618, 0.103307, 0.024739, 0.001474, 0.003034, 0.003601, 0.005707, 0.007447, 0.006697, 0.005752, 0.004690, 0.002699, 0.007463, 0.077716, 0.244948 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == IRON_ALT_1) {
		length = 12;
		double rr [] = { 0.00, 0.05, 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95, 1.00 };
		double bb [] = { 0.00, 0.25, 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (IRON_ALT_1 + 1)) {
		length = 10;
		double rr [] = { 0.05, 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb [] = { 0.25, 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == IRON_ALT_2) {
		length = 11;
		double rr [] = { 0.00, 0.05, 0.10,  0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00,  0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00 };
		double bb [] = { 0.00, 0.25, 0.50,  0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (IRON_ALT_2 + 1)) {
		length = 5;
		double rr [] = { 0.40, 0.80, 0.90, 0.95, 1.00 };
		double gg [] = { 0.60, 0.50, 0.05, 0.00, 0.05 };
		double bb [] = { 0.00, 0.10, 0.25, 0.45, 0.80 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (IRON_ALT_3)) {
		length = 11;
		double rr [] = { 0.00, 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95, 1.00 };
		double bb [] = { 0.00, 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (IRON_ALT_3 + 1)) {
		length = 9;
		double rr [] = { 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb [] = { 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == BLUERED) {
		length = 3;
		double rr [] = { 0.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (BLUERED + 1)) {
		length = 2;
		double rr [] = { 0.00, 1.00 };
		double gg [] = { 0.00, 0.00 };
		double bb [] = { 1.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == BLUERED_ALT_1) {
		length = 4;
		double rr [] = { 0.00, 0.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 1.00 };
		double bb [] = { 0.00, 1.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (BLUERED_ALT_1 + 1)) {
		length = 2;
		double rr [] = { 0.00, 1.00 };
		double gg [] = { 0.00, 0.00 };
		double bb [] = { 1.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == BLUERED_ALT_2) {
		length = 7;
		double rr [] = { 0.00, 0.50, 0.80, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.50, 0.80, 1.00, 0.80, 0.50, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.80, 0.50, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (BLUERED_ALT_2 + 1)) {
		length = 4;
		double rr [] = { 0.00, 0.50, 1.00, 1.00 };
		double gg [] = { 0.00, 0.50, 0.50, 0.00 };
		double bb [] = { 1.00, 1.00, 0.50, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == JET) {
		length = 13;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00, 0.50, 1.00 };
		double bb [] = { 0.00, 0.50, 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (JET + 1)) {
		length = 11;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00, 0.50 };
		double bb [] = { 0.50, 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00, 0.50 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == JET_ALT_1) {
		length = 11;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00, 1.00 };
		double bb [] = { 0.00, 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (JET_ALT_1 + 1)) {
		length = 9;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICE) {
		length = 9;
		double rr [] = { 1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg [] = { 1.00, 0.88, 0.75, 0.68, 0.50, 0.38, 0.25, 0.13, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 1.00, 0.75, 0.50, 0.25, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICE + 1)) {
		length = 7;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg [] = { 0.88, 0.75, 0.68, 0.50, 0.38, 0.25 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.75, 0.50 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICE_ALT_1) {
		length = 11;
		double rr [] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00, 0.00 };
		double gg [] = { 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICE_ALT_1 + 1)) {
		length = 7;
		double rr [] = { 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00 };
		double gg [] = { 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00 };
		double bb [] = { 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICE_ALT_2) {
		length = 11;
		double rr [] = { 1.00, 0.64, 0.36, 0.16, 0.04, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg [] = { 1.00, 0.90, 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20, 0.10, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 1.00, 0.99, 0.89, 0.78, 0.63, 0.48, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICE_ALT_2 + 1)) {
		length = 7;
		double rr [] = { 0.36, 0.16, 0.04, 0.01, 0.00, 0.00, 0.00 };
		double gg [] = { 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20 };
		double bb [] = { 1.00, 1.00, 1.00, 0.99, 0.89, 0.78, 0.63 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICE_ALT_3) {
		length = 9;
		double rr [] = { 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg [] = { 1.00, 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.50, 0.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICE_ALT_3 + 1)) {
		length = 7;
		double rr [] = { 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg [] = { 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.50 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICEIRON) {
		length = 19;
		double rr [] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.33, 0.67,     0.67, 0.86, 0.93, 1.00,   1.00, 1.00, 1.00 };
		double gg [] = { 1.00, 1.00, 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.15, 0.40, 0.80,   0.95, 1.00, 1.00 };
		double bb [] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.00, 0.00,     0.33, 0.40, 0.10, 0.05,   0.45, 0.70, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICEIRON + 1)) {
		length = 12;
		double rr [] = { 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.67,     0.67, 0.86, 0.93, 1.00,   1.00 };
		double gg [] = { 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00, 0.15, 0.40, 0.80,   0.95 };
		double bb [] = { 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.00,     0.33, 0.40, 0.10, 0.05,   0.45 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICEIRON_ALT_1) {
		length = 21;
		double rr [] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.05, 0.10,     0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00 };
		double gg [] = { 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00 };
		double bb [] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.25, 0.50,     0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICEIRON_ALT_1 + 1)) {
		length = 10;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00,          0.40, 0.80, 0.90, 0.95, 1.00 };
		double gg [] = { 1.00, 0.75, 0.50, 0.25, 0.00,          0.00, 0.10, 0.25, 0.45, 0.80 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 1.00,          0.60, 0.50, 0.05, 0.00, 0.05 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICEIRON_ALT_2) {
		length = 19;
		double rr [] = { 1.00,      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,      0.00,        0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00,   1.00 };
		double gg [] = { 1.00,      0.88, 0.75, 0.68, 0.50, 0.38, 0.25, 0.13,      0.00,        0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95,   1.00 };
		double bb [] = { 1.00,      1.00, 1.00, 1.00, 1.00, 0.75, 0.50, 0.25,      0.00,        0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45,   1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICEIRON_ALT_2 + 1)) {
		length = 17;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,      0.00,        0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg [] = { 0.88, 0.75, 0.68, 0.50, 0.38, 0.25, 0.13,      0.00,        0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.75, 0.50, 0.25,      0.00,        0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICEFIRE) {
		length = 19;
		double rr [] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.33, 0.67,     1.00, 1.00, 1.00, 1.00,   1.00, 1.00, 1.00 };
		double gg [] = { 1.00, 1.00, 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.33, 0.67, 1.00,   1.00, 1.00, 1.00 };
		double bb [] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.00, 0.00,     0.00, 0.00, 0.00, 0.00,   0.33, 0.67, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICEFIRE + 1)) {
		length = 12;
		double rr [] = { 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.67,     1.00, 1.00, 1.00, 1.00,   1.00 };
		double gg [] = { 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00, 0.33, 0.67, 1.00,   1.00 };
		double bb [] = { 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.00,     0.00, 0.00, 0.00, 0.00,   0.33 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICEFIRE_ALT_1) {
		length = 17;
		double rr [] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.33, 0.67,     1.00, 1.00, 1.00,   1.00, 1.00, 1.00 };
		double gg [] = { 1.00, 1.00, 1.00,     1.00, 0.50, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.50, 1.00,   1.00, 1.00, 1.00 };
		double bb [] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.00, 0.00,     0.00, 0.00, 0.00,   0.33, 0.67, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICEFIRE_ALT_1 + 1)) {
		length = 10;
		double rr [] = { 0.33,     0.00, 0.00, 0.00,    0.00, 0.67,     1.00, 1.00, 1.00,   1.00 };
		double gg [] = { 1.00,     1.00, 0.50, 0.00,    0.00, 0.00,     0.00, 0.50, 1.00,   1.00 };
		double bb [] = { 1.00,     1.00, 1.00, 1.00,    0.67, 0.00,     0.00, 0.00, 0.00,   0.33 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICEFIRE_ALT_2) {
		length = 17;
		double rr [] = { 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,    0.00,   0.25, 0.50, 0.75, 1.00, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 1.00, 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.50,    0.00,   0.00, 0.00, 0.00, 0.00, 0.50, 1.00, 1.00, 1.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00,    0.00,   0.00, 0.00, 0.50, 0.00, 0.00, 0.00, 0.50, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICEFIRE_ALT_2 + 1)) {
		length = 12;
		double rr [] = { 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.50, 0.75, 1.00, 1.00, 1.00, 1.00 };
		double gg [] = { 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.00, 0.00, 0.00, 0.50, 1.00, 1.00 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.50, 0.00, 0.00, 0.00, 0.50 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == ICEFIRE_ALT_3) {
		length = 17;
		//                  WHITE           CYAN            LBLUE           DBLUE           PURPLE          RED             ORANGE          YELLOW          WHITE
		double rr [] = {    1.00, 0.50,     0.00, 0.00,     0.50, 0.25,     0.00, 0.25,     0.50, 0.75,     1.00, 1.00,     1.00, 1.00,     1.00, 1.00,     1.00 };
		double gg [] = {    1.00, 1.00,     1.00, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00, 0.25,     0.50, 0.75,     1.00, 1.00,     1.00 };
		double bb [] = {    1.00, 1.00,     1.00, 1.00,     1.00, 0.75,     0.50, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00, 0.50,     1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (ICEFIRE_ALT_3 + 1)) {
		length = 13;
		//                  CYAN            LBLUE           DBLUE           PURPLE          RED             ORANGE          YELLOW
		double rr [] = {    0.00, 0.00,     0.50, 0.25,     0.00, 0.25,     0.50, 0.75,     1.00, 1.00,     1.00, 1.00,     1.00 };
		double gg [] = {    1.00, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00, 0.25,     0.50, 0.75,     1.00 };
		double bb [] = {    1.00, 1.00,     1.00, 0.75,     0.50, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == REPEATED) {
		length = 16;
		double rr [] = {    0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = {    0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, };
		double bb [] = {    1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (REPEATED + 1)) {
		length = 12;
		double rr [] = {    0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
		double gg [] = {    0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double bb [] = {    1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == REPEATED_ALT_1) {
		length = 6;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (REPEATED_ALT_1 + 1)) {
		length = 6;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == REPEATED_ALT_2) {
		length = 48;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (REPEATED_ALT_2 + 1)) {
		length = 48;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == REPEATED_ALT_3) {
		length = 41;
		double rr [] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.05, 0.10,     0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.05, 0.10,     0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00 };
		double gg [] = { 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00 };
		double bb [] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.25, 0.50,     0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.25, 0.50,     0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (REPEATED_ALT_3 + 1)) {
		length = 28;
		double rr [] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,    0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,    0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg [] = { 0.88, 0.75, 0.68, 0.50, 0.38, 0.25,    0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95, 0.88, 0.75, 0.68, 0.50, 0.38, 0.25,    0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb [] = { 1.00, 1.00, 1.00, 1.00, 0.75, 0.50,    0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45, 1.00, 1.00, 1.00, 1.00, 0.75, 0.50,    0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == REPEATED_ALT_4) {
		length = 12;
        double rr [] = { 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
        double gg [] = { 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
        double bb [] = { 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
        rx = rr;
        gx = gg;
        bx = bb;
	} else if (code == (REPEATED_ALT_4 + 1)) {
		length = 12;
        double rr [] = { 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 };
        double gg [] = { 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 };
        double bb [] = { 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 };
        rx = rr;
        gx = gg;
        bx = bb;
	} else if (code == REPEATED_ALT_5) {
		length = 6;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (REPEATED_ALT_5 + 1)) {
		length = 6;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == REPEATED_ALT_6) {
		length = 48;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	} else if (code == (REPEATED_ALT_6 + 1)) {
		length = 48;
		double rr [] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg [] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb [] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		rx = rr;
		gx = gg;
		bx = bb;
	}

	create_long_map();

	setupLookupTable(1);
	setupLookupTable(2);

}

int cScheme::current_scheme() {
	return code;
}

void cScheme::customize(double* r, double* g, double* b, int len) {
	code = CUSTOM;
	rx = r;
	gx = g;
	bx = b;
	length = len;
}

void cScheme::falsify_image(const Mat& thermIm, Mat& outputIm, int param) {

	if ((outputIm.size() != thermIm.size()) && (outputIm.type() != CV_8UC3)) {
		outputIm = Mat::zeros(thermIm.size(), CV_8UC3);
	}

	//int level;
	// inputs can be 8 or 16 bits/pixel
	// output is 8 bits/pixel
	//int maxVal = 255;

	// some kind of check to figure out if input image is actual 16-bits/pixel
	if (thermIm.depth() == 2) {
        //maxVal = 65535;
	}

	// store the current standard, and then modify it according to params
	//int newCode, oldCode;

	if (param == 1) {
		//oldCode = code;
		//newCode = code + 1;
	}

	//printf("%s << code = %d; newCode = %d\n", __FUNCTION__, code, newCode);

	/*
	if (params[0] == 0) {   // if a variation hasn't been selected

	    // start to deconstruct code...
        newCode = code;

        if (((newCode % 2) == 0) && (params[1] == 1)) {   // if it's unsafe but safety has been selected
            newCode += 1;
        }


	} else {    // otherwise, ignore last digit
        newCode = code - (code % 10);

        newCode += (params[0]-1)*2;    // add variation code

        if (params[1] == 1) {
            newCode += 1;              // add safety index
        }

	}
	*/

	if (param == 1) {
		//load_standard(newCode);
	}

	//setupLookupTable();

	//double sr, sg, sb;

    /*
	printf("%s << thermIm = %dx%d, %d, %d\n", __FUNCTION__, thermIm.size().width, thermIm.size().height, thermIm.depth(), thermIm.channels());
	printf("%s << outputIm = %dx%d, %d, %d\n", __FUNCTION__, outputIm.size().width, outputIm.size().height, outputIm.depth(), outputIm.channels());
    */

	// Create RGB output image
	//if (outputIm.empty()) {
        //outputIm = Mat(thermIm.size(), CV_8UC3);
	//}

	//struct timeval timer;
	//double elapsedTime;
	//elapsedTime = timeElapsedMS(timer);


	//level = 0;

	int lookupIndex = 0;

	for (int i = 0; i < thermIm.size().height; i++)	{
		for (int j = 0; j < thermIm.size().width; j++) {

			// find closest element...

			//if ((*thermIm)(i, j) > 255

			//level = (thermIm.at<unsigned short>(i,j)*(MAP_LENGTH-1))/65536;
            //level = (int) floor((thermIm.at<unsigned short>(i,j)*(MAP_LENGTH-1))/65535); // or 65536?
			//printf("level = %d\n", level);

			// removing this halves from 15 - 7
			/*
			if (thermIm.depth() == 2) {
                level = (int) floor(double((thermIm.at<unsigned short>(i,j)*(MAP_LENGTH-1))/maxVal)); // for 16 bits/pixel
			} else {
                level = (int) floor(double((thermIm.at<unsigned char>(i,j)*(MAP_LENGTH-1))/maxVal)); // for standard 8bits/pixel
			}
			*/
			// removing this takes it down effectively to zero...
			/*
            outputIm.at<Vec3b>(i,j)[2] = (unsigned char) (255.0 * red[level]);
            outputIm.at<Vec3b>(i,j)[1] = (unsigned char) (255.0 * green[level]);
            outputIm.at<Vec3b>(i,j)[0] = (unsigned char) (255.0 * blue[level]);
			*/

			//printf("%s << depth (%d) (%d, %d)\n", __FUNCTION__, thermIm.depth(), thermIm.cols, thermIm.rows);

			//imshow("asda", thermIm);
			//waitKey();

			if (thermIm.depth() == 2) {
				lookupIndex = thermIm.at<unsigned short>(i,j);
				outputIm.at<Vec3b>(i,j)[2] = lookupTable_2[lookupIndex][2];
				outputIm.at<Vec3b>(i,j)[1] = lookupTable_2[lookupIndex][1];
				outputIm.at<Vec3b>(i,j)[0] = lookupTable_2[lookupIndex][0];
			} else if (thermIm.depth() == 0) {

				lookupIndex = thermIm.at<unsigned char>(i,j);
				//printf("%s << here (%d, %d, %d)\n", __FUNCTION__, lookupTable_1[lookupIndex][2], lookupTable_1[lookupIndex][1], lookupTable_1[lookupIndex][0]);
				outputIm.at<Vec3b>(i,j)[2] = lookupTable_1[lookupIndex][2];
				outputIm.at<Vec3b>(i,j)[1] = lookupTable_1[lookupIndex][1];
				outputIm.at<Vec3b>(i,j)[0] = lookupTable_1[lookupIndex][0];
			}


			/*
			sr = 255.0 * red[level];
			sg = 255.0 * green[level];
			sb = 255.0 * blue[level];

            outputIm.at<Vec3b>(i,j)[2] = (unsigned char) sr;
            outputIm.at<Vec3b>(i,j)[1] = (unsigned char) sg;
            outputIm.at<Vec3b>(i,j)[0] = (unsigned char) sb;
			*/

			//printf("raw = %f\n", thermIm(i, j));
			//printf("lvl = %d\n", level);
			//printf("red = %f\n", red[level]);
			//printf("grn = %f\n", green[level]);
			//printf("blu = %f\n", blue[level]);
			//printf("/n");

		}
		//std::cin.get();
	}

	//elapsedTime = timeElapsedMS(timer);
	//printf("%s << Time elapsed [%d] = %f\n", __FUNCTION__, 0, elapsedTime);

	if (param == 1) {
		//load_standard(oldCode);
	}

}

void cScheme::image_resize(Mat& inputIm, int dim_i, int dim_j) {
	Mat newIm;
	newIm = Mat(dim_i,dim_j,inputIm.type());

	double val;

	int ni, nj;

	ni = inputIm.size().height;
	nj = inputIm.size().width;

	for (int i = 0; i < dim_i; i++) {
		for (int j = 0; j < dim_j; j++) {
			//val = (*inputIm)(((int)((i*inputIm->ni())/dim_i)),((int)((j*inputIm->nj())/dim_j)));
			val = inputIm.at<unsigned char>(((int)((i*ni)/dim_i)),((int)((j*nj)/dim_j)));

			newIm.at<unsigned char>(i,j) = val;
		}
	}

	//inputIm->set_size(dim_i, dim_j);
	//inputIm->deep_copy(*newIm);
    inputIm = Mat(newIm);

}

void cScheme::fuse_image(Mat& thermIm, Mat& visualIm, Mat& outputIm, double* params) {

	double working_params[2];

	if (params == NULL) {
		working_params[0] = 0.2;
		working_params[1] = 0.8;
	} else {
		working_params[0] = params[0];
		working_params[1] = params[1];
	}

	//vil_image_view<double>* greyVisual;
	int falseParam = 1;
	double lumChange; //, temp1;
	//double greyVal;

	unsigned char sr, sg, sb;

	Mat newTherm;

	if (thermIm.channels() > 1) {
        cvtColor(thermIm, newTherm, CV_RGB2GRAY);
    } else {
        thermIm.copyTo(newTherm);
    }

	Mat newVisual;

    if (visualIm.channels() > 1) {
        cvtColor(visualIm, newVisual, CV_RGB2GRAY);
    } else {
        visualIm.copyTo(newVisual);
    }



	// If images aren't the same size, resize						// AARGH!! THE RESIZING SEEMS TO SET TO !!!
	if (thermIm.size() != visualIm.size()) {
	    //printf("%s << Image sizes mismatch.\n");
		// Determine which one is bigger and resize accordingly
		if (thermIm.size().height > thermIm.size().width) {
			//newVisual->set_size();
			image_resize(newVisual, thermIm.size().height, thermIm.size().width);
			//newVisual->fill(0);
		} else {
			//newTherm->set_size(visualIm->ni(), visualIm->nj());
			//newVisual->fill(0);
			image_resize(newTherm, visualIm.size().height, visualIm.size().width);
		}
	}

	// Equalize input images

	//vil_histogram_equalise(*newTherm);
	//vil_histogram_equalise(*newVisual);


	// Create RGB output image
	//outputIm->clear();
	outputIm = Mat(newTherm.size(), CV_8UC3);

	// NONPRIOR: Figure out a way to prevent calculating false image twice
	falsify_image(newTherm, outputIm, falseParam);

	//imshow("tempWin", outputIm);
	//waitKey(0);

	//imshow("grayVisual", newVisual);
	//waitKey(0);

	// PRIOR: Convert colour visual image to grayscale in one hit...

	/*
	vil_image_view<vxl_byte> lim;
	vil_convert_cast(visualIm, lim);
	if (!lim.size()) {
		// FAILED
	}
	if (lim.nplanes() == 3) {
		vil_convert_planes_to_grey(lim, *greyVisual, 0.333333, 0.333333, 0.333333);
	} else {
		vil_convert_cast(lim, *greyVisual);
	}
	*/

	//printf("%s << params = [%f, %f]\n", __FUNCTION__, params[0], params[1]);

	// For each pixel
	for (int i = 0; i < newTherm.size().height; i++)	{
	    //printf("%s << thermalVal[%d][%d] = \n", __FUNCTION__, i, j, );


		for (int j = 0; j < newTherm.size().width; j++) {
			// Calculate change in luminance
			//greyVal = greyVisual(i, j, 0)



			//lumChange = 2 * ((*newVisual)(i, j) - 0.5) * (params[1] - params[0]);
			lumChange = 2.0 * (((double) newVisual.at<unsigned char>(i,j))/255.0 - 0.5) * (working_params[1] - working_params[0]);
			//printf("lumChange = %f\n", lumChange);
			//temp1 = newVisual.at<unsigned char>(i,j);

			if (((i % 10) == 0) && ((j % 10) == 0)) {
                //printf("%s << newVisual[%d][%d] = %f\n", __FUNCTION__, i, j, newVisual.at<unsigned char>(i,j));
                //printf("%s << lumChange[%d][%d] = %f\n", __FUNCTION__, i, j, lumChange);
			}

			//printf("newTherm(%d,%d) = %f\n", i, j, (*newTherm)(i, j));

			if (lumChange > 0) {
				sr = outputIm.at<Vec3b>(i, j)[0] + (255.0 - outputIm.at<Vec3b>(i, j)[0]) * lumChange;
				sg = outputIm.at<Vec3b>(i, j)[1] + (255.0 - outputIm.at<Vec3b>(i, j)[1]) * lumChange;
				sb = outputIm.at<Vec3b>(i, j)[2] + (255.0 - outputIm.at<Vec3b>(i, j)[2]) * lumChange;
			} else {
				sr = outputIm.at<Vec3b>(i, j)[0] + (outputIm.at<Vec3b>(i, j)[0] * lumChange);
				sg = outputIm.at<Vec3b>(i, j)[1] + (outputIm.at<Vec3b>(i, j)[1] * lumChange);
				sb = outputIm.at<Vec3b>(i, j)[2] + (outputIm.at<Vec3b>(i, j)[2] * lumChange);
			}

			//sr = ((lumChange/(params[1] - params[0])) + 0.5) * 255;
			//sg = ((lumChange/(params[1] - params[0])) + 0.5) * 255;
			//sb = ((lumChange/(params[1] - params[0])) + 0.5) * 255;

			if (((i % 10) == 0) && ((j % 10) == 0)) {
                //printf("%s << newVisual[%d][%d] = %f\n", __FUNCTION__, i, j, newVisual.at<unsigned char>(i,j));
                //printf("%s << lumChange[%d][%d] = %f\n", __FUNCTION__, i, j, lumChange);
			}

			outputIm.at<Vec3b>(i,j)[0] = sr;
			outputIm.at<Vec3b>(i,j)[1] = sg;
			outputIm.at<Vec3b>(i,j)[2] = sb;
		}
	}

	//imshow("tempWin", outputIm);
	//waitKey(0);

	//cin.get();

}

void generateHistogram(Mat& src, Mat& dst, double* im_hist, double* im_summ, double* im_stat) {

    MatND hist;
    Mat img;

    int nPixels = src.cols*src.rows;

    //printf("%s << src.depth() = %d\n", __FUNCTION__, src.depth());

    src.convertTo(img, CV_32FC1);

    //svLib::normalize_16(src, img);  // no longer old

    //imshow("converted", img);
    //waitKey(0);

    double minIntensity = 9e50, maxIntensity=0;
    minMaxLoc(img, &minIntensity, &maxIntensity, 0, 0);

    double midIntensity = (maxIntensity + minIntensity) / 2.0;
    double intensityRange = maxIntensity-minIntensity;

    if (intensityRange < 64) {
        minIntensity = midIntensity - 32;
        maxIntensity = midIntensity + 32;
    }

    //double newIntensityRange = maxIntensity-minIntensity;

    printf("%s << intensity range = %f (%f, %f)\n", __FUNCTION__, intensityRange, minIntensity, maxIntensity);

    int intensityBins = 64;
    int histSize[] = {intensityBins};
    float intensityRanges[] = {minIntensity, maxIntensity};
    const float* ranges[] = {intensityRanges};
    int channels[] = {0};

    calcHist(&img, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);

    double minVal = 9e50, maxVal=0.0;
    minMaxLoc(hist, &minVal, &maxVal, 0, 0);

    //printf("%s << minVal = %f; maxVal = %f\n", __FUNCTION__, minVal, maxVal);

    int horizontalScale = 640 / intensityBins;

    //printf("%s << horizontalScale = %d\n", __FUNCTION__, horizontalScale);
    int verticalScale = 480;
    Mat histImg = Mat::zeros(verticalScale, intensityBins*horizontalScale, CV_8UC3); //, Vec3b::all(255));

    Scalar col = CV_RGB(255, 255, 255);
    histImg.setTo(col);

    //int quant01, quant05, quant50, quant95, quant99;

    double quantileCount = 0.0;

    for(int iii = 0; iii < intensityBins; iii++ ) {
        float binVal = hist.at<float>(iii, 0);
        float count = binVal/maxVal;

        quantileCount += binVal;

        /*
        printf("%s << iii = %d\n", __FUNCTION__, iii);
        printf("%s << binVal = %f\n", __FUNCTION__, binVal);
        printf("%s << fullCount = %f\n", __FUNCTION__, fullCount);
        */

        if (quantileCount < 0.01*double(nPixels)) {
            //quant01 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.05*double(nPixels)) {
            //quant05 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.50*double(nPixels)) {
            //quant50 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.95*double(nPixels)) {
            //quant95 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.99*double(nPixels)) {
            //quant99 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }

        //printf("%s << count = %f\n", __FUNCTION__, count);

        rectangle(histImg, Point(iii*horizontalScale+1, verticalScale), Point((iii+1)*horizontalScale-2, (verticalScale-1)*(1-count)+1), CV_RGB(0, 64, 192), CV_FILLED);
    }

    histImg.copyTo(dst);

    double histMean = 0, histDev = 0, histRMS = 0, histSkew = 0;

    double pixelCount = img.rows * img.cols;

    // Calculate histogram statistics:
    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histMean += img.at<float>(iii,jjj) / pixelCount;
        }
    }

    //printf("%s << histMean = %f\n", __FUNCTION__, histMean);
    im_stat[0] = histMean;

    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histDev += pow((img.at<float>(iii,jjj) - histMean), 2) / pixelCount;
        }
    }

    histDev = pow(histDev, 0.5);

    //printf("%s << histDev = %f\n", __FUNCTION__, histDev);
    im_stat[1] = histDev;

    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histRMS += pow(img.at<float>(iii,jjj), 2) / pixelCount;
        }
    }

    histRMS = pow(histRMS, 0.5);

    //printf("%s << histRMS = %f\n", __FUNCTION__, histRMS);

    im_stat[2] = histRMS;

    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histSkew += pow((img.at<float>(iii,jjj)- histMean) / histDev, 3) / pixelCount;
        }
    }

    //printf("%s << histSkew = %f\n", __FUNCTION__, histSkew);

    im_stat[3] = histSkew;

    //printf("%s << qrange_0_100 = %d\n", __FUNCTION__, int(intensityRange));

    //printf("%s << qrange_1_99 = %d\n", __FUNCTION__, quant99-quant01);

    //printf("%s << qrange_5_95 = %d\n", __FUNCTION__, quant95-quant05);

}

Mat normForDisplay(Mat origMat) {
    double minVal = 9e99, maxVal = -9e99;

    for (int iii = 0; iii < origMat.rows; iii++) {
        for (int jjj = 0; jjj < origMat.cols; jjj++) {

            // printf("%s << origMat.val = %f\n", __FUNCTION__, origMat.at<double>(iii,jjj));


            if (origMat.at<double>(iii,jjj) > maxVal) {
                maxVal = origMat.at<double>(iii,jjj);
            }

            if (origMat.at<double>(iii,jjj) < minVal) {
                minVal = origMat.at<double>(iii,jjj);
            }
        }
    }

    Mat displayMat(origMat.size(), CV_8UC1);

    for (int iii = 0; iii < origMat.rows; iii++) {
        for (int jjj = 0; jjj < origMat.cols; jjj++) {
            displayMat.at<unsigned char>(iii,jjj) = (unsigned char) ((origMat.at<double>(iii,jjj) - minVal) * 255.0 / (maxVal - minVal));
        }
    }

    Mat largerMat;

    resize(displayMat, largerMat, Size(origMat.rows*1, origMat.cols*1), 0, 0, INTER_NEAREST);

    return largerMat;
}
