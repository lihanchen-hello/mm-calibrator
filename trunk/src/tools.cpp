#include "tools.h"

void convertUcharToBinary(unsigned char val, int* binaryArray) {
    for (int iii = 0; iii < 8; iii++) {
        if ((int) val >= (int) pow(2, 7-iii)) {
            binaryArray[iii] = 1;
            val -= (int) pow(2, 7-iii);
        } else {
            binaryArray[iii] = 0;
        }
    }
}

void redistortPoints(const vector<Point2f>& src, vector<Point2f>& dst, const Mat& cameraMatrix, const Mat& distCoeffs, const Mat& newCamMat) {

    double fx, fy, ifx, ify, cx, cy;
	double fx0, fy0, ifx0, ify0, cx0, cy0;
    double k[8]={0,0,0,0,0,0,0,0}, RR[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    double r2, icdist, deltaX, deltaY;
    double x, y, x0, y0, x1, y1, xx, yy, ww;

	Mat optimalMat(3, 3, CV_64FC1);

	// optimalMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640,48), 1.0);

    //printf("%s << entered function.\n", __FUNCTION__);

    // will probably crash if it receives the identity matrix etc...

    fx0 = cameraMatrix.at<double>(0, 0);
    fy0 = cameraMatrix.at<double>(1, 1);
    ifx0 = 1./fx0;
    ify0 = 1./fy0;
    cx0 = cameraMatrix.at<double>(0, 2);
    cy0 = cameraMatrix.at<double>(1, 2);

    fx = newCamMat.at<double>(0, 0);
    fy = newCamMat.at<double>(1, 1);
    ifx = 1./fx;
    ify = 1./fy;
    cx = newCamMat.at<double>(0, 2);
    cy = newCamMat.at<double>(1, 2);

    for (unsigned int i = 0; i < 8; i++){
        k[i] = distCoeffs.at<double>(0, i);
    }

    //printf("%s << cx = %f; cy = %f\n", __FUNCTION__, cx, cy);

    //cin.get();

    dst.clear();

    for (unsigned int i = 0; i < src.size(); i++) {
        // Points in undistorted image
        x = src.at(i).x;
        y = src.at(i).y;

        // printf("%s << undistorted points at [%d] = (%f, %f)\n", __FUNCTION__, i, x, y);

        // Apply cameraMatrix parameters (normalization)
        x0 = (x - cx)*ifx;
        y0 = (y - cy)*ify;

		x = x0;
		y = y0;

        // Determine radial and tangential distances/factors
        r2 = x*x + y*y;
        icdist = (1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2)/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
        deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
        deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;

		//icdist *= 0.75;

        // Redistort
        ///*
        //x = (x0 - deltaX)/icdist;
        //y = (y0 - deltaY)/icdist;
        //*/
        ///*
        x = (x0/icdist) + deltaX;
        y = (y0/icdist) + deltaY;
        //*/
		//x = x0/icdist;
		//y = y0/icdist;

        // Do something...
        /*
        xx = RR[0][0]*x + RR[0][1]*y + RR[0][2];
        yy = RR[1][0]*x + RR[1][1]*y + RR[1][2];
        ww = 1./(RR[2][0]*x + RR[2][1]*y + RR[2][2]);
        x = xx*ww;
        y = yy*ww;
        */

		x1 = x;
		y1 = y;

        // Reverse cameraMatrix parameters (denormalization)
        x = (x1/ifx0) + cx0;
        y = (y1/ify0) + cy0;

        // printf("%s << redistorted points at [%d] = (%f, %f)\n", __FUNCTION__, i, x, y);

        //cin.get();

        dst.push_back(Point2f(x, y));
    }

    // Temporary...
    // dst.assign(src.begin(), src.end());
}

double timeElapsedMS(struct timeval& timer, bool reset) {
	
	struct timeval new_time;
	
	long seconds, useconds;
	
	gettimeofday(&new_time, NULL);

    seconds  = new_time.tv_sec  - timer.tv_sec;
    useconds = new_time.tv_usec - timer.tv_usec;

	double retVal = ((double) seconds) * 1000.0 + ((double) useconds) * 0.001;
	
	if (reset) {
		timer = new_time;
	}
    
    return retVal;
    
 
}

bool matricesAreEqual(Mat& mat1, Mat& mat2) {
	
	if (mat1.rows != mat2.rows) {
		//printf("%s << Row mismatch\n", __FUNCTION__);
		return false;
	}
	
	if (mat1.cols != mat2.cols) {
		//printf("%s << Col mismatch\n", __FUNCTION__);
		return false;
	}
	
	if (mat1.type() != mat2.type()) {
		//printf("%s << Type mismatch\n", __FUNCTION__);
		return false;
	}
	
	// printf("%s << type = %d\n", __FUNCTION__, mat1.type());
	
	switch (mat1.type()) {
		case CV_16UC1:
			//printf("%s << type: CV_16UC1\n", __FUNCTION__);
			break;
		case CV_16SC1:
			//printf("%s << type: CV_16SC1\n", __FUNCTION__);
			break;
		case CV_8UC1:
			//printf("%s << type: CV_8UC1\n", __FUNCTION__);
			break;
		case CV_8UC3:
			//printf("%s << type: CV_8UC3\n", __FUNCTION__);
			break;
		case CV_8SC1:
			//printf("%s << type: CV_8SC1\n", __FUNCTION__);
			break;
		case CV_16UC3:
			//printf("%s << type: CV_16UC3\n", __FUNCTION__);
			break;
		case CV_16SC3:
			//printf("%s << type: CV_16SC3\n", __FUNCTION__);
			break;
		case CV_64FC1:
			//printf("%s << type: CV_64FC1\n", __FUNCTION__);
			break;
		case CV_32FC1:
			//printf("%s << type: CV_32FC1\n", __FUNCTION__);
			break;
		default:
			printf("%s << ERROR! Equality check for this type (%d) has not been implemented!\n", __FUNCTION__, mat1.type());
			return false;
	}

	
	for (int iii = 0; iii < mat1.rows; iii++) {
		for (int jjj = 0; jjj < mat1.cols; jjj++) {
			
			switch (mat1.type()) {
				case CV_64FC1:
					//printf("%s << type: CV_64FC1\n", __FUNCTION__);
					if (mat1.at<double>(iii,jjj) != mat2.at<double>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<double>(iii,jjj), mat2.at<double>(iii,jjj));
						return false;
					}
					break;
				case CV_32FC1:
					//printf("%s << type: CV_32FC1\n", __FUNCTION__);
					if (mat1.at<float>(iii,jjj) != mat2.at<float>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<float>(iii,jjj), mat2.at<float>(iii,jjj));
						return false;
					}
					break;
				case CV_16UC1:
					//printf("%s << type: CV_16UC1\n", __FUNCTION__);
					if (mat1.at<unsigned short>(iii,jjj) != mat2.at<unsigned short>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<unsigned short>(iii,jjj), mat2.at<unsigned short>(iii,jjj));
						return false;
					}
					break;
				case CV_16SC1:
					//printf("%s << type: CV_16SC1\n", __FUNCTION__);
					if (mat1.at<short>(iii,jjj) != mat2.at<short>(iii,jjj)) {
						return false;
					}
					break;
				case CV_8UC1:
					//printf("%s << type: CV_8UC1\n", __FUNCTION__);
					if (mat1.at<unsigned char>(iii,jjj) != mat2.at<unsigned char>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<unsigned char>(iii,jjj), mat2.at<unsigned char>(iii,jjj));
						return false;
					}
					break;
				case CV_8UC3:
					//printf("%s << type: CV_8UC3\n", __FUNCTION__);
					if ((mat1.at<Vec3b>(iii,jjj)[0] != mat2.at<Vec3b>(iii,jjj)[0]) || (mat1.at<Vec3b>(iii,jjj)[1] != mat2.at<Vec3b>(iii,jjj)[1]) || (mat1.at<Vec3b>(iii,jjj)[2] != mat2.at<Vec3b>(iii,jjj)[2])) {
						return false;
					}
					break;
				case CV_8SC1:
					//printf("%s << type: CV_8SC1\n", __FUNCTION__);
					if (mat1.at<char>(iii,jjj) != mat2.at<char>(iii,jjj)) {
						return false;
					}
					break;
				case CV_16UC3:
					//printf("%s << type: CV_16UC3\n", __FUNCTION__);
					if ((mat1.at<Vec3s>(iii,jjj)[0] != mat2.at<Vec3s>(iii,jjj)[0]) || (mat1.at<Vec3s>(iii,jjj)[1] != mat2.at<Vec3s>(iii,jjj)[1]) || (mat1.at<Vec3s>(iii,jjj)[2] != mat2.at<Vec3s>(iii,jjj)[2])) {
						return false;
					}
					break;
				case CV_16SC3:
					//printf("%s << type: CV_16SC3\n", __FUNCTION__);
					if ((mat1.at<Vec3s>(iii,jjj)[0] != mat2.at<Vec3s>(iii,jjj)[0]) || (mat1.at<Vec3s>(iii,jjj)[1] != mat2.at<Vec3s>(iii,jjj)[1]) || (mat1.at<Vec3s>(iii,jjj)[2] != mat2.at<Vec3s>(iii,jjj)[2])) {
						return false;
					}
					break;
				default:
					printf("%s << ERROR! Equality check for this type has not been implemented!\n", __FUNCTION__);
					return false;
				}
			}
		}
	
	return true;
	
}

void randomSelection(vector<unsigned int>& src, vector<unsigned int>& dst, unsigned int max) {

	dst.clear();
	dst.insert(dst.end(), src.begin(), src.end());
	
	if (dst.size() <= max) {
		return;
	}
	
	while (dst.size() > max) {
		dst.erase(dst.begin() + (rand() % dst.size()));
	}
	
}

double asymmetricGaussianValue(double score, double mean, double min, double max) {

	double zScore, sigma, retVal;
	
	if (score == mean) {
		return 1.00;
	} else if (score > max) {
		return 0.00;
	} else if (score < min) {
		return 0.00;
	} else if (score > mean) {
		sigma = abs(max - mean);
	} else if (score < mean) {
		sigma = abs(min - mean);
	}
	
	zScore = (score - mean) / sigma;
	retVal = exp(-pow(zScore, 2.0)/2.0);
	
	return retVal;
	
}

void addUniqueToVector(vector<unsigned int>& dst, vector<unsigned int>& src) {

	for (unsigned int iii = 0; iii < src.size(); iii++) {
		
		bool alreadyAdded = false;
		
		for (unsigned int jjj = 0; jjj < dst.size(); jjj++) {
			if (dst.at(jjj) == src.at(iii)) {
				alreadyAdded = true;
			}
			
			
		}
		
		if (!alreadyAdded) {
			dst.push_back(src.at(iii));	
		}
		
	}
	
}

double calcLinePerpDistance(double *line1, double *line2) {
    double retVal = 0.0;

    retVal = abs(line2[2] - line1[2]) / sqrt(pow(line1[0], 2) + pow(line1[1], 2));

    return retVal;
}

Scalar getRandomColour() {
    Scalar color( rand()&255, rand()&255, rand()&255 );

    return color;
}

long long int factorial(int num)
{

    long long int result=1;
    for (int i=1; i<=num; ++i)
        result=result*=i;
    return result;

}

void getNextCombo(vector<unsigned int>& currentIndices, int r, int n)
{

    bool maxed = false;
    bool valid = true;

    //printf("%s << Entered function.\n", __FUNCTION__);

    // If no indices tested, use default (0, 1, 2 etc)
    if (currentIndices.size() == 0)
    {
        for (int i = 0; i < r; i++)
        {
            currentIndices.push_back(i);
        }
    }
    else
    {

        // Going back each digit
        int i = 0;

        while (valid && (i < r))
        {
            //printf("%s << i = %d / %d\n", __FUNCTION__, i, r);
            // If current index is about to go over its maximum...
            if (currentIndices.at(currentIndices.size()-i-1) > (n-2-i))
            {
                //printf("%s << digit #(%d) is valid; less than %d\n", __FUNCTION__, currentIndices.size()-i-1, n-2-i);
                i++;    // check out next index
            }
            else        // Otherwise, just increment it, fill in trailing digits and exit while loop
            {
                currentIndices.at(currentIndices.size()-i-1) = currentIndices.at(currentIndices.size()-i-1) + 1;
                for (int j = 0; j < i; j++)
                {
                    currentIndices.at(currentIndices.size()-i+j) = currentIndices.at(currentIndices.size()-i+j-1) + 1;
                }
                valid = false;
            }
        }


    }
}

double findEquivalentProbabilityScore(double* values, int quantity, double prob)
{

    int i = 0;
    vector<double> listVector;
    double min;
    int minIndex;

    // Push values into vector
    for (int j = 0; j < quantity; j++)
    {
        listVector.push_back(values[j]);
    }

    // Pop minimum values off vector until you've reached sufficient depth for the probability
    while (listVector.size() >= (unsigned int)(std::max(int(prob*quantity), 1)))
    {
        //printf("%s << listVector.size() = %d, prob*quantity = %d\n", __FUNCTION__, listVector.size(), int(prob*quantity));
        //cin.get();
        min = 9e99;

        for (unsigned int j = 0; j < listVector.size(); j++)
        {
            if (listVector.at(j) < min)
            {
                min = listVector.at(j);
                minIndex = j;
            }
        }

        listVector.erase(listVector.begin() + minIndex);

    }

    return min;

}
