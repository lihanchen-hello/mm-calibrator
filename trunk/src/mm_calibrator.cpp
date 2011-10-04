#include "mm_calibrator.hpp"

int main(int argc, char* argv[]) {

    char *directory;
    int numCams = DEFAULT_CAM_COUNT;
    bool wantsIntrinsics = false;
    bool wantsExtrinsics = false;
    int patternFinderCode = CHESSBOARD_FINDER_CODE;
    int maxPatternsToKeep = DEFAULT_PATTERNS_TO_KEEP;
    int maxPatternsPerSet = DEFAULT_MAX_PATTERNS_PER_SET;
    double gridSize = DEFAULT_GRID_SIZE;
    int x = DEFAULT_GRID_X_DIM, y = DEFAULT_GRID_Y_DIM;
    int optimizationCode = ENHANCED_MCM_OPTIMIZATION_CODE;
    bool wantsToDisplay = false;
    bool wantsToUndistort = false;
    bool wantsToWrite = false;

    // --------------------------------------------- PARSING
    printf("%s << Parsing arguments...\n", __FUNCTION__);

    opterr = 0;
    int c;

    if (argc == 1) {
        printf("%s << User should be prompted for parameters...\n", __FUNCTION__);
    } else {
        while ((c = getopt(argc, argv, "d:n:iet:a:b:g:x:y:so:uh")) != -1) {

            switch (c) {
                case 'd':
                    directory = optarg;
                    break;
                case 'n':
                    numCams = atoi(optarg);
                    break;
                case 'i':
                    wantsIntrinsics = true;
                    break;
                case 'e':
                    wantsExtrinsics = true;
                    break;
                case 't':
                    patternFinderCode = atoi(optarg);
                    break;
                case 'a':
                    maxPatternsToKeep = atoi(optarg);
                    break;
                case 'b':
                    maxPatternsPerSet = atoi(optarg);
                    break;
                case 'g':
                    gridSize = atof(optarg);
                    break;
                case 'x':
                    x = atoi(optarg);
                    break;
                case 'y':
                    y = atoi(optarg);
                    break;
                case 's':
                    wantsToDisplay = true;
                    break;
                case 'o':
                    optimizationCode = atoi(optarg);
                    break;
                case 'u':
                    wantsToUndistort = true;
                    break;
                case 'w':
                    wantsToWrite = true;
                    break;
                case 'h':
                    usage(argv[0]);
                    break;
                default:
                    printf("Invalid option -%c\n", c);
                    printf("Run %s -h for help.\n", argv[0]);
                    //usage(argv[0]);
                    return 1;
            }

        }
    }

    int maxFramesToLoad = DEFAULT_FRAMES_TO_LOAD;   // DANGER!! THIS HAS BEEN SET LOW FOR TESTING / DEVELOPMENT

    maxPatternsToKeep = min(maxPatternsToKeep, maxFramesToLoad);
    maxPatternsPerSet = min(maxPatternsPerSet, maxPatternsToKeep);

    printf("%s << Parent directory is: %s\n", __FUNCTION__, directory);
    printf("%s << Number of cameras to calibrate: %d\n", __FUNCTION__, numCams);
    printf("%s << Calculating intrinsics? %d\n", __FUNCTION__, wantsIntrinsics);
    printf("%s << Calculating extrinsics? %d\n", __FUNCTION__, wantsExtrinsics);
    printf("%s << Pattern Finder Code = %d\n", __FUNCTION__, patternFinderCode);
    printf("%s << Max frames to load = %d; Max patterns to keep = %d; Max patterns per set = %d\n", __FUNCTION__, maxFramesToLoad, maxPatternsToKeep, maxPatternsPerSet);
    printf("%s << Pattern dimensions: Squares are %fmm wide, with [%d x %d] calibration points\n", __FUNCTION__, gridSize, x, y);
    printf("%s << Wants to display? %d\n", __FUNCTION__, wantsToDisplay);
    printf("%s << Optimization Code = %d\n", __FUNCTION__, optimizationCode);
    printf("%s << Wants to undistort? %d\n", __FUNCTION__, wantsToUndistort);
    printf("%s << Wants to write? %d\n", __FUNCTION__, wantsToWrite);

    // Currently only contains support for input being in the form of a folder
    bool inputIsFolder = true;

    char *inStream[MAX_CAMS];
    char *outStream[MAX_CAMS];
    char *intrinsicParams[MAX_CAMS];
    char *extrinsicParams;

    DIR * dirp;
    struct dirent * entry;

    vector<string> inputList[MAX_CAMS];

    bool sameNum = true;

    if (wantsExtrinsics) {
        extrinsicParams = (char*) malloc(strlen(directory) + 128);
        sprintf(extrinsicParams, "%s/%s%d%s", directory, "extrinsics-", numCams, ".yml");
        printf("%s << extrinsicParams = %s\n", __FUNCTION__, extrinsicParams);
    }

    for (unsigned int nnn = 0; nnn < numCams; nnn++) {
        inStream[nnn] = (char*) malloc(strlen(directory) + 128);
        outStream[nnn] = (char*) malloc(strlen(directory) + 128);

        intrinsicParams[nnn] = (char*) malloc(strlen(directory) + 128);
        sprintf(intrinsicParams[nnn], "%s/%s%d%s", directory, "intrinsics-", nnn, ".yml");

        printf("%s << intrinsicParams[%d] = %s\n", __FUNCTION__, nnn, intrinsicParams[nnn]);
    }

    if (inputIsFolder) {

        for (unsigned int nnn = 0; nnn < numCams; nnn++) {

            sprintf(inStream[nnn], "%s/%d/", directory, nnn);

            printf("%s << inStream[%d] = %s\n", __FUNCTION__, nnn, inStream[nnn]);

            sprintf(outStream[nnn], "%s/%d-r/", directory, nnn);

            dirp = opendir(inStream[nnn]);

			while ((entry = readdir(dirp)) != NULL) {
				//printf("DEBUG C.\n");
				if (entry->d_type == DT_REG) { // If the entry is a regular file

				    inputList[nnn].push_back(string(entry->d_name));

				}
			}

			closedir(dirp);

            printf("%s << inputList[%d] = %d\n", __FUNCTION__, nnn, inputList[nnn].size());

        }

        for (int nnn = 0; nnn < numCams-1; nnn++) {
            if (inputList[nnn].size() != inputList[nnn+1].size()) {
                sameNum = false;
            }
        }

        if (sameNum) {
            maxFramesToLoad = std::min((int)inputList[0].size(), (int)maxFramesToLoad);
            printf("%s << maxFramesToLoad = %d\n", __FUNCTION__, maxFramesToLoad);

            randomCulling(inputList[0], maxFramesToLoad);

            sort(inputList[0].begin(), inputList[0].end());

        } else {
            printf("%s << Frame count mismatch.\n", __FUNCTION__);
            return -1;
        }
    }

    FileStorage fs;

    Mat imageSize_mat[MAX_CAMS], cameraMatrix[MAX_CAMS], newCamMat[MAX_CAMS], distCoeffs[MAX_CAMS];
    Size imageSize_size[MAX_CAMS];

    for (unsigned int nnn = 0; nnn < numCams; nnn++) {
        imageSize_mat[nnn] = Mat(1, 2, CV_16UC1);
    }

    if (wantsExtrinsics && (!wantsIntrinsics)) {


        for (unsigned int nnn = 0; nnn < numCams; nnn++) {


            printf("%s << intrinsicParams[%d] = %s\n", __FUNCTION__, nnn, intrinsicParams[nnn]);


            fs = FileStorage(intrinsicParams[nnn], FileStorage::READ);
            fs["imageSize"] >> imageSize_mat[nnn];

            imageSize_size[nnn] = Size(imageSize_mat[nnn].at<unsigned short>(0), imageSize_mat[nnn].at<unsigned short>(1));

            fs["cameraMatrix"] >> cameraMatrix[nnn];
            fs["distCoeffs"] >> distCoeffs[nnn];
            fs.release();

            printf("%s << cameraMatrix[%d] = ", __FUNCTION__, nnn);
            cout << cameraMatrix[nnn] << endl;

            printf("%s << distCoeffs[%d] = ", __FUNCTION__, nnn);
            cout << distCoeffs[nnn] << endl;

        }
    }

    //  --------------------------------------------- CREATE ARBITRARY 2D CO-ORDINATE VECTOR
    cv::vector<Point3f> row;
    // Pattern Corner Co-ordinates
    cv::vector<Point2f> cornerSet;
    cv::vector<cv::vector<Point2f> > cornersList[MAX_CAMS];

    for (int i = 0; i < y; i++) {
        for (int j = 0; j < x; j++) {
            row.push_back(Point3f(i*gridSize, j*gridSize, 0.0));
        }
    }

    char filename[128];

    Mat inputMat[MAX_CAMS];

    vector<Mat> allImages[MAX_CAMS];

    Mat tmpMat, dispMat;

    bool patternFound = false;

    vector<bool> foundRecord[MAX_CAMS];



	int index = 0;
    // --------------------------------------------- THE PATTERN SEARCH

    // Run through each camera separately to find the patterns
    for (unsigned int nnn = 0; nnn < numCams; nnn++) {

        index = 0;

        // For each frame for each camera
        while (index < min((int)inputList[0].size(), maxFramesToLoad)) {

            sprintf(filename, "%s%s", inStream[nnn], (inputList[0].at(index)).c_str());

            printf("%s << filename = %s\n", __FUNCTION__, filename);
            inputMat[nnn] = imread(filename);
            allImages[nnn].push_back(inputMat[nnn]);

            /*
            if (wantsToDisplay) {
                imshow("displayWindow", inputMat[nnn]);
                waitKey(40);
            }
            */

            patternFound = false;

            cornerSet.clear();

            switch (patternFinderCode) {
                case CHESSBOARD_FINDER_CODE:
                    patternFound = findChessboardCorners(inputMat[nnn], cvSize(x,y), cornerSet);
                    break;
                case MASK_FINDER_CODE:
                    patternFound = findMaskCorners_1(inputMat[nnn], cvSize(x,y), cornerSet, PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG);
                    break;
                case HEATED_CHESSBOARD_FINDER_CODE:
                    invertMatIntensities(inputMat[nnn], tmpMat);
                    tmpMat.copyTo(inputMat[nnn]);
                    patternFound = findChessboardCorners(inputMat[nnn], cvSize(x,y), cornerSet);
                    break;
                default:
                    patternFound = findChessboardCorners(inputMat[nnn], cvSize(x,y), cornerSet);
                    break;
            }

            printf("%s << Pattern found? %d\n", __FUNCTION__, patternFound);

            inputMat[nnn].copyTo(dispMat);

            drawChessboardCorners(dispMat, cvSize(x, y), Mat(cornerSet), patternFound);

            if (wantsToDisplay) {
                imshow("displayWindow", dispMat);
                waitKey(40);
            }

            index++;

            foundRecord[nnn].push_back(patternFound);
            cornersList[nnn].push_back(cornerSet);

        }

    }

    cv::vector<Mat> distributionMap;

    double radialDistribution[RADIAL_LENGTH];

    vector<int> tagNames[MAX_CAMS], selectedTags[MAX_CAMS];
    vector<vector<int> > extrinsicTagNames, extrinsicSelectedTags;

    cv::vector<cv::vector<Point2f> > candidatesList[MAX_CAMS];

    if (wantsIntrinsics) {

        distributionMap.resize(numCams);

        for (unsigned int nnn = 0; nnn < numCams; nnn++) {

            imageSize_size[nnn] = inputMat[nnn].size();

            Mat tmpMat = Mat(inputMat[nnn].size(), CV_8UC1);

            distributionMap.at(nnn) = Mat(inputMat[nnn].size(), CV_8UC1);

            cv::vector<cv::vector<Point2f> > intrinsicsList;
            vector<string> extractedList;

            for (unsigned int iii = 0; iii < cornersList[nnn].size(); iii++) {
                if (foundRecord[nnn][iii] == true) {
                    intrinsicsList.push_back(cornersList[nnn].at(iii));
                    extractedList.push_back(inputList[nnn].at(iii));
                    tagNames[nnn].push_back(iii);

                    //printf("%s << tagNames[%d].at(%d) = %d\n", __FUNCTION__, nnn, iii, tagNames[nnn].at(iii));
                }
            }


            if (intrinsicsList.size() > maxPatternsToKeep) {
                //randomCulling(extractedList, maxPatternsToKeep, intrinsicsList);
                randomCulling(extractedList, maxPatternsToKeep, intrinsicsList);
            }

            for (int iii = 0; iii < intrinsicsList.size(); iii++) {
                candidatesList[nnn].push_back(intrinsicsList[iii]);
            }

            printf("%s << Optimizing Pattern Set...\n", __FUNCTION__);
            // Optimize which frames to use here, replacing the corners vector and other vectors with new set
            optimizeCalibrationSet(inputMat[nnn], distributionMap.at(nnn), candidatesList[nnn], intrinsicsList, row, optimizationCode, min((int)maxPatternsPerSet, (int)intrinsicsList.size()), radialDistribution, tagNames[nnn], selectedTags[nnn]);

            cv::vector< cv::vector<Point3f> > objectPoints;
            cv::vector<Mat> rvecs, tvecs;

            for (int iii = 0; iii < candidatesList[nnn].size(); iii++) {
                objectPoints.push_back(row);
            }

            rvecs.resize(candidatesList[nnn].size());
            tvecs.resize(candidatesList[nnn].size());

            printf("%s << Optimization Complete.\n", __FUNCTION__);

            if (candidatesList[nnn].size() == 0) {
                printf("%s << No patterns remaining - cannot calibrate. Returning.\n", __FUNCTION__);
                return 1;
            } else {
                printf("%s << Total number of patterns remaining after optimization: %d\n", __FUNCTION__, candidatesList[nnn].size());
            }

            double reprojError, extendedReprojError;

            reprojError = calibrateCamera(objectPoints, candidatesList[nnn], inputMat[nnn].size(), cameraMatrix[nnn], distCoeffs[nnn], rvecs, tvecs, INTRINSICS_FLAGS);

            printf("%s << Calibration DONE.\n", __FUNCTION__);

            printf("%s << OpenCV subsequence MRE = %f\n", __FUNCTION__, reprojError);

            double *errValues;
            errValues = new double[intrinsicsList.size() * intrinsicsList.at(0).size()];

            extendedReprojError = calculateERE(inputMat[nnn], objectPoints.at(0), intrinsicsList, cameraMatrix[nnn], distCoeffs[nnn], errValues);

            printf("%s << Full-sequence MRE = %f\n", __FUNCTION__, extendedReprojError);

            cout << endl << "cameraMatrix = \n" << cameraMatrix[nnn] << endl;
            cout << "distCoeffs = \n" << distCoeffs[nnn] << endl << endl;

            // Undistortion Parameters
            double alpha = DEFAULT_ALPHA;
            Rect validROI;

            newCamMat[nnn] = getOptimalNewCameraMatrix(cameraMatrix[nnn], distCoeffs[nnn], inputMat[nnn].size(), alpha, inputMat[nnn].size(), &validROI);

            cout << endl << "newCamMat = \n" << newCamMat[nnn] << endl;

            string outputString(intrinsicParams[nnn]);

            FileStorage fs(outputString, FileStorage::WRITE);

            imageSize_mat[nnn].at<unsigned short>(0) = imageSize_size[nnn].width;
            imageSize_mat[nnn].at<unsigned short>(1) = imageSize_size[nnn].height;

            fs << "imageSize" << imageSize_mat[nnn];
            fs << "cameraMatrix" << cameraMatrix[nnn];
            fs << "distCoeffs" << distCoeffs[nnn];
            fs << "newCamMat" << newCamMat[nnn];

            fs << "reprojectionError" << reprojError;
            fs << "generalisedError" << extendedReprojError;

            fs << "patternsUsed" << (int)candidatesList[nnn].size();

            fs.release();

            printf("%s << Writing to file...DONE.\n", __FUNCTION__);

        }


	}

	if (wantsExtrinsics) {

	    cv::vector<Mat> extrinsicsDistributionMap;
	    extrinsicsDistributionMap.resize(numCams);

	    printf("%s << Calculating extrinsics...\n", __FUNCTION__);

	    cv::vector<cv::vector<Point2f> > emptyPointSetVector;
	    vector<int> emptyIntVector;

	    cv::vector<cv::vector<cv::vector<Point2f> > > extrinsicsList, extrinsicsCandidates;
        vector<string> extractedList;

        for (unsigned int nnn = 0; nnn < numCams; nnn++) {
            extrinsicsList.push_back(emptyPointSetVector);
            extrinsicsCandidates.push_back(emptyPointSetVector);
            extrinsicTagNames.push_back(emptyIntVector);

            extrinsicsDistributionMap.at(nnn) = Mat(inputMat[nnn].size(), CV_8UC1);
        }

	    for (unsigned int iii = 0; iii < cornersList[0].size(); iii++) {

	        bool allPatternsFound = true;

	        for (unsigned int nnn = 0; nnn < numCams; nnn++) {
                if (foundRecord[nnn][iii] == false) {
                    allPatternsFound = false;
                }
	        }

	        if (allPatternsFound) {
                for (unsigned int nnn = 0; nnn < numCams; nnn++) {
                    extrinsicsList.at(nnn).push_back(cornersList[nnn].at(iii));
                    extrinsicsCandidates.at(nnn).push_back(cornersList[nnn].at(iii));

                    extrinsicTagNames.at(nnn).push_back(iii);


                }



	        }

        }


        vector<Size> extrinsicsSizes;

        for (unsigned int nnn = 0; nnn < numCams; nnn++) {
            extrinsicsSizes.push_back(imageSize_size[nnn]);
        }

        optimizeCalibrationSets(extrinsicsSizes, numCams, cameraMatrix, distCoeffs, extrinsicsDistributionMap, extrinsicsCandidates, extrinsicsList, row, optimizationCode, maxPatternsPerSet, extrinsicTagNames, extrinsicSelectedTags);

        // UNCHECKED

        TermCriteria term_crit;
        term_crit = TermCriteria(TermCriteria::COUNT+ TermCriteria::EPS, 30, 1e-6);

        printf("%s << Calibrating Cameras...\n", __FUNCTION__);

        Mat E[MAX_CAMS], F[MAX_CAMS], Q;      // Between first camera and all other cameras
        Mat R[MAX_CAMS], Rv[MAX_CAMS], T[MAX_CAMS];         // Rotations/translations between first camera and all other cameras
        Mat R2[MAX_CAMS], T2[MAX_CAMS];       // Rotations/translations between all other cameras
        Mat R_[MAX_CAMS], P_[MAX_CAMS];

        // Should make R[0] and T[0] the "identity" matrices
        R[0] = Mat::eye(3, 3, CV_64FC1);
        T[0] = Mat::zeros(3, 1, CV_64FC1);

        cv::vector< cv::vector<Point3f> > objectPoints;

        for (int iii = 0; iii < extrinsicsCandidates[0].size(); iii++) {
            objectPoints.push_back(row);
        }

        for (unsigned int nnn = 0; nnn < numCams; nnn++) {
            for (int k = 0; k < numCams-1; k++) {
                stereoCalibrate(objectPoints,
                        extrinsicsCandidates.at(0), extrinsicsCandidates.at(k+1),
                        cameraMatrix[0], distCoeffs[0],
                        cameraMatrix[k+1], distCoeffs[k+1],
                        imageSize_size[0],                      // hopefully multiple cameras allow multiple image sizes
                        R[k+1], T[k+1], E[k+1], F[k+1],
                        term_crit,
                        EXTRINSICS_FLAGS); //
            }
        }

        double extendedExtrinsicReprojectionError;
        extendedExtrinsicReprojectionError = calculateExtrinsicERE(numCams, objectPoints.at(0), extrinsicsList, cameraMatrix, distCoeffs, R, T);

        printf("%s << eERE = %f\n", __FUNCTION__, extendedExtrinsicReprojectionError);
        // Stereo Calibration between pairs
        printf("%s << Cameras calibrated.\n", __FUNCTION__);

        fs = FileStorage(extrinsicParams, FileStorage::WRITE);

        string tmpString;

        Mat R_Vec;

        char tmp[64];

        sprintf(tmp, "generalisedMRE");
        tmpString = string(tmp);
        fs << tmpString << extendedExtrinsicReprojectionError;

        for (int i = 0; i < numCams; i++) {

            sprintf(tmp, "R%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << R[i];

            sprintf(tmp, "Rvec%d", i);
            tmpString = string(tmp);
            fs << tmpString << Rv[i];

            sprintf(tmp, "T%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << T[i];

            sprintf(tmp, "R_%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << R_[i];

            sprintf(tmp, "P_%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << P_[i];

            sprintf(tmp, "E%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << E[i];

            sprintf(tmp, "F%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << F[i];

            sprintf(tmp, "cameraMatrix%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << cameraMatrix[i];

            sprintf(tmp, "distCoeffs%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << distCoeffs[i];
        }

        fs << "Q" << Q;

        fs.release();

    }

//            grandTotalTime = getTickCount() - grandTotalTime;
//
//            printf("%s << Absolute total calibration time = %f\n", __FUNCTION__, grandTotalTime*1000/getTickFrequency());
//
//            for (int k = 0; k < nCams; k++) {
//
//                Rodrigues(R[k], Rv[k]);
//
//                /*
//                cout << "cameraMatrix[" << k << "] = \n" << cameraMatrix[k] << endl;
//                cout << "distCoeffs[" << k << "] = \n" << distCoeffs[k] << endl;
//                */
//
//                Mat Rdeg;
//
//                Rv[k].copyTo(Rdeg);
//
//                for (int z = 0; z < Rdeg.cols; z++) {
//                    Rdeg.at<double>(0,z) *= (180 / PI);
//                }
//
//                cout << "T[" << k << "] = " << T[k] << endl;
//                cout << "Rdeg[" << k << "] = " << Rdeg << endl << endl;
//            }
//
//            Rect roi1, roi2;
//
//            printf("%s << Before ERE Calculation.\n", __FUNCTION__);
//
//            // Calculate ERE
//            double tMean, tDev;
//
//            printf("%s << DEBUG %d\n", __FUNCTION__, 0);
//
//            tMean = svLib::calculateExtrinsicERE(nCams, objectPoints.at(0), vvvTestingPatterns, cameraMatrix, distCoeffs, R, T);
//
//            printf("%s << DEBUG %d\n", __FUNCTION__, 1);
//
//            tDev = pow(tMean/vvvTestingPatterns.at(0).size(), 0.5);
//
//            printf("%s << ERE : Extended Reprojection Error = %f\n", __FUNCTION__, tMean);
//
//            // Obtain Rectification Maps
//            if (nCams == 2) {
//                printf("%s << 2 Cameras.\n", __FUNCTION__);
//
//                stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
//                              imSize.at(0),
//                              R[1], T[1],
//                              R_[0], R_[1], P_[0], P_[1],
//                              Q,
//                              CALIB_ZERO_DISPARITY,
//                              alpha, imSize.at(0), &roi1, &roi2);
//
//                /*
//                stereoRectify(cameraMatrix[0], distCoeffs[0],
//                              cameraMatrix[1], distCoeffs[1],
//                              imSize.at(0),
//                              R[1], T[1],
//                              R_[0], R_[1], P_[0], P_[1],
//                              Q,
//                              alpha, imSize.at(0), &roi1, &roi2,
//                              CALIB_ZERO_DISPARITY);
//                  */
//
//            } else if (nCams == 3) {
//                printf("%s << 3 Camera rectification commencing. (alpha = %f)\n", __FUNCTION__, alpha);
//
//                double ratio =  rectify3Collinear(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
//                                                    distCoeffs[1], cameraMatrix[2], distCoeffs[2],
//                                                    vvvCandidatePatterns.at(0), vvvCandidatePatterns.at(2),
//                                                    imSize.at(0), R[1], T[1], R[2], T[2],
//                                                    R_[0], R_[1], R_[2], P_[0], P_[1], P_[2], Q, alpha,
//                                                    imSize.at(0), 0, 0, CV_CALIB_ZERO_DISPARITY);
//
//                printf("%s << 3 Camera rectification complete.\n", __FUNCTION__);
//            }
//




    // Depends on intrinsics / extrinsics
    // objectPoints.push_back(row);

    return 0;
}
