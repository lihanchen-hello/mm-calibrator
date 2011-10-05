#include "cv_utils.hpp"

double distBetweenPts2f(Point2f& P1, Point2f& P2) {
    /*
    double retVal;
    retVal = pow((pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y),2)), 0.5);
    return retVal;
    */

    return pow((pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y),2)), 0.5);
}

double distBetweenPts(Point& P1, Point& P2) {
    // TODO:
    // Possible issue.. see above ^

    double retVal;
    retVal = pow(double(pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y), 2.0)), 0.5);
    return retVal;
}

void drawLinesBetweenPoints(Mat& image, const vector<Point2f>& src, const vector<Point2f>& dst) {

    Point p1, p2;

    for (int i = 0; i < src.size(); i++) {
        p1 = Point(src.at(i).x*16, src.at(i).y*16);
        p2 = Point(dst.at(i).x*16, dst.at(i).y*16);

        //line(image, p1, p2, CV_RGB(0,0,255), 1, CV_AA, 4);
        circle(image, p2, 4*16, CV_RGB(0,0,255), 1, CV_AA, 4);
    }

}

Point findCentroid(vector<Point>& contour) {
    Moments momentSet;

    double x,y;

    momentSet = moments(Mat(contour));
    x = momentSet.m10/momentSet.m00;
    y = momentSet.m01/momentSet.m00;

    return Point((int)x,(int)y);
}

Point2f findCentroid2f(vector<Point>& contour) {
    Moments momentSet;

    double x,y;

    momentSet = moments(Mat(contour));
    x = momentSet.m10/momentSet.m00;
    y = momentSet.m01/momentSet.m00;

    return Point2f(x,y);
}

void createGaussianMatrix(Mat& gaussianMat, double sigmaFactor) {

    Mat distributionDisplay(Size(640, 480), CV_8UC1);
    Mat binTemp(gaussianMat.size(), CV_8UC1);

    // sigmaFactor says how many standard deviations should span from the center to the nearest edge
            // (i.e. along the shortest axis)


    // What about an elliptical gaussian function?

    Point2f center((float)((double(gaussianMat.size().height-1))/2), (float)((double(gaussianMat.size().width-1))/2));
    Point2f tmpPt;
    double dist = 0.0, average = 0.0, maxVal = 0.0;
    double sigma = min(gaussianMat.size().width, gaussianMat.size().height)/(2*sigmaFactor);

    double A = (gaussianMat.size().width*gaussianMat.size().height) / (sigma * pow(2*3.142, 0.5));


    for (int i = 0; i < gaussianMat.size().width; i++) {
        for (int j = 0; j < gaussianMat.size().height; j++) {
            tmpPt = Point2f(float(j), float(i));
            dist = distBetweenPts2f(center, tmpPt);


            //gaussianMat.at<double>(j,i) = A*exp(-(pow(dist,2)/(2*pow(sigma,2))));
            //gaussianMat.at<double>(j,i) = dist;
            if (dist < max(double(gaussianMat.size().width)/2, double(gaussianMat.size().height)/2)) {
                gaussianMat.at<double>(j,i) = 1.0;
            } else {
                gaussianMat.at<double>(j,i) = 0.0;
            }

            average += gaussianMat.at<double>(j,i);

            if (gaussianMat.at<double>(j,i) > maxVal) {
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

    for (int i = 0; i < gaussianMat.size().width; i++) {
        for (int j = 0; j < gaussianMat.size().height; j++) {
            average += gaussianMat.at<double>(j,i);
            if (gaussianMat.at<double>(j,i) > maxVal) {
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

void cropImage(Mat& image, Point tl, Point br) {
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

    if (image.channels() == 3) {
        tmpMat = Mat(height, width, CV_8UC3);
    } else if (image.channels() == 1) {
        tmpMat = Mat(height, width, CV_8UC1);
    }



    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            //printf("%s << %d / %d\n", __FUNCTION__, i, j);

            if (image.channels() == 3) {
                if ((j+yOff < 0) || (j+yOff > image.rows-1) || (i+xOff < 0) || (i+xOff > image.cols-1)) {
                    tmpMat.at<Vec3b>(j,i)[0] = 0;
                    tmpMat.at<Vec3b>(j,i)[1] = 0;
                    tmpMat.at<Vec3b>(j,i)[2] = 0;
                } else {
                    tmpMat.at<Vec3b>(j,i) = image.at<Vec3b>(j+yOff,i+xOff);
                }
            } else if (image.channels() == 1) {
                if ((j+yOff < 0) || (j+yOff > image.rows-1) || (i+xOff < 0) || (i+xOff > image.cols-1)) {
                    tmpMat.at<unsigned char>(j,i) = 0;
                } else {
                    tmpMat.at<unsigned char>(j,i) = image.at<unsigned char>(j+yOff,i+xOff);
                }


            }




        }
    }

    //tmpMat.copyTo(image);
    resize(tmpMat, image, Size(width, height)); // working

    //printf("%s << Completing function.\n", __FUNCTION__);

}

void convertVectorToPoint(vector<Point2f>& input, vector<Point>& output) {
    output.clear();

    for (unsigned int i = 0; i < input.size(); i++) {
        output.push_back(Point((int)input.at(i).x, (int)input.at(i).y));
    }
}

void convertVectorToPoint2f(vector<Point>& input, vector<Point2f>& output) {
    // TODO:
    // Nothing much.

    output.clear();

    for (unsigned int i = 0; i < input.size(); i++) {
        output.push_back(Point2f((float)input.at(i).x, (float)input.at(i).y));
    }
}

void simpleResize(Mat& src, Mat& dst, Size size) {

    dst = Mat::zeros(size, src.type());

    for (int i = 0; i < dst.size().width; i++) {
        for (int j = 0; j < dst.size().height; j++) {
            if (src.depth() == 1) {
                dst.at<unsigned char>(j,i) = src.at<unsigned char>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            } else if (src.depth() == 8) {
                dst.at<double>(j,i) = src.at<double>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            } else if (src.depth() == CV_16U) {
                dst.at<unsigned short>(j,i) = src.at<unsigned short>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            }

        }
    }
}

void copyContour(vector<Point>& src, vector<Point>& dst) {
    // TODO:
    // Make safer.

    dst.clear();

    for (unsigned int i = 0; i < src.size(); i++) {
        dst.push_back(src.at(i));
    }
}

void swapElements(vector<Point2f>& corners, int index1, int index2) {
    // TODO:
    // Nothing much.

    Point2f tempPt;

    tempPt = corners.at(index1);    // copy first element to temp
    corners.at(index1) = corners.at(index2);  // put best element in first
    corners.at(index2) = tempPt;  // copy temp to where best element was
}

void invertMatIntensities(Mat& src, Mat& dst) {
    dst.release();
    dst = Mat(src.size(), src.type());

    double a;
    unsigned int z;

    if (src.type() == CV_8UC1) {



        for (int iii = 0; iii < src.rows; iii++) {
            for (int jjj = 0; jjj < src.cols; jjj++) {
                dst.at<unsigned char>(iii,jjj) = 255 - src.at<unsigned char>(iii, jjj);
            }
        }

    } else if (src.type() == CV_8UC3) {

        // printf("%s << here.\n", __FUNCTION__);

        for (int iii = 0; iii < src.rows; iii++) {
            for (int jjj = 0; jjj < src.cols; jjj++) {
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

void contourDimensions(vector<Point> contour, double& width, double& height) {
    // TODO:
    // May want to replace this with something that finds the longest and shortest distances across
    // Because the idea of this is to help determine if it's a square or not.

    // new methodology
    RotatedRect wrapperRectangle;
    Size size;
    vector<Point> contourCpy;
    Point meanPoint;

    // Interpolate contour to get it to an adequate size for "fitEllipse" function
    if (contour.size() < 6) {
        for (unsigned int i = 0; i < contour.size()-1; i++) {
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

    } else {
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

double perpDist(Point2f& P1, Point2f& P2, Point2f& P3) {
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

double findMinimumSeparation(vector<Point2f>& pts) {
    double minSep = 9e50;
    double val = 0.0;

    for (int i = 0; i < pts.size(); i++) {
        for (int j = i+1; j < pts.size(); j++) {
            val = norm(pts.at(i)-pts.at(j));
            if (val < minSep) {
                minSep = val;
            }
        }
    }

    return minSep;
}

Point2f meanPoint(Point2f& P1, Point2f& P2) {
    return Point2f((P1.x+P2.x)/2, (P1.y+P2.y)/2);
}

void transferElement(vector<Point2f>& dst, vector<Point2f>& src, int index) {
    Point2f pointCpy;

    pointCpy = src.at(index);

    // Move from old one to new one
    dst.push_back(pointCpy);

    // Replace and shift points in old one
    for (unsigned int i = index; i < src.size()-1; i++) {
        src.at(i) = src.at(i+1);
    }

    // Truncate the original vector (effectively discarding old point)
    src.pop_back();
}
