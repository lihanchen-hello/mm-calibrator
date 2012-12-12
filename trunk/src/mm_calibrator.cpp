#include "mm_calibrator.hpp"

int main(int argc, char* argv[])
{

    // Undistortion Parameters
    double alpha = DEFAULT_ALPHA;

    char *directory, *parametersFile;
    int numCams = DEFAULT_CAM_COUNT;
    bool wantsIntrinsics = false;
    bool wantsExtrinsics = false;
    int patternFinderCode = MASK_FINDER_CODE;
    int maxPatternsToKeep = DEFAULT_PATTERNS_TO_KEEP;
    int maxPatternsPerSet = DEFAULT_MAX_PATTERNS_PER_SET;
    double gridSize = DEFAULT_GRID_SIZE;
    int x = DEFAULT_GRID_X_DIM, y = DEFAULT_GRID_Y_DIM;
    int optimizationCode = ENHANCED_MCM_OPTIMIZATION_CODE;
    bool wantsToDisplay = true;
    bool wantsToUndistort = false;
    bool wantsToWrite = false;
    double correctionFactor = DEFAULT_CORRECTION_FACTOR;
    bool outputFoundPatterns = false;

    // Currently only contains support for input being in the form of a folder (and maybe AVI video)
    bool inputIsFolder = true;
    
    bool verboseMode = DEFAULT_DEBUG_MODE;
    
    bool providedMSERparams = false;
    
    int intrinsicsFlags = DEFAULT_INTRINSICS_FLAGS;

    // --------------------------------------------- PARSING
    //printf("%s << Parsing arguments...\n", __FUNCTION__);

    opterr = 0;
    int c;

    printf("\n%s << Parsing arguments...\n", __FUNCTION__);

    if (argc == 1)
    {
        bool parameterSupply = false;
        directory = (char*) malloc(256);
        parameterSupply = promptUserForParameters(directory, numCams, wantsIntrinsics, wantsExtrinsics, patternFinderCode, maxPatternsToKeep, maxPatternsPerSet, gridSize, x, y, wantsToDisplay, optimizationCode, wantsToUndistort, wantsToWrite, inputIsFolder, verboseMode);

        if (parameterSupply == false) {
            printf("%s << Using default parameters...\n", __FUNCTION__);
        }

    }
    else
    {


        while ((c = getopt(argc, argv, "a:b:c:d:eg:hin:o:p:qrst:uvx:y:z")) != -1)
        {

            switch (c)
            {
            case 'd':
                directory = optarg;
                break;
			case 'r':
                intrinsicsFlags += CV_CALIB_RATIONAL_MODEL;
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
			case 'z':
                outputFoundPatterns = true;
                break;
            case 'g':
                gridSize = atof(optarg);
                break;
			case 'c':
				correctionFactor = atof(optarg);
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
            case 'q':
                inputIsFolder = false;
                break;
            case 'h':
                usage(argv[0]);
                return 1;
            case 'v':
                verboseMode = true;
                break;  
			case 'p':
                parametersFile = optarg;
                providedMSERparams = true;
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

    if (!inputIsFolder)
    {
        printf("%s << Input in video format.\n", __FUNCTION__);
    }
    else
    {
        printf("%s << Input in image format.\n", __FUNCTION__);
    }

    printf("%s << Number of cameras to calibrate: %d\n", __FUNCTION__, numCams);
    printf("%s << Calculating intrinsics? %d\n", __FUNCTION__, wantsIntrinsics);
    printf("%s << Calculating extrinsics? %d\n", __FUNCTION__, wantsExtrinsics);
    printf("%s << Pattern Finder Code = %d\n", __FUNCTION__, patternFinderCode);
    printf("%s << Max frames to load = %d; Max patterns to keep = %d; Max patterns per set = %d\n", __FUNCTION__, maxFramesToLoad, maxPatternsToKeep, maxPatternsPerSet);
    printf("%s << Pattern dimensions: Squares are %fmm wide, with [%d x %d] squares\n", __FUNCTION__, gridSize, x, y);
    printf("%s << Wants to display? %d\n", __FUNCTION__, wantsToDisplay);
    printf("%s << Optimization Code = %d\n", __FUNCTION__, optimizationCode);
    printf("%s << Wants to undistort? %d\n", __FUNCTION__, wantsToUndistort);
    printf("%s << Wants to write? %d\n", __FUNCTION__, wantsToWrite);

    char *inStream[MAX_CAMS];
    char *outStream[MAX_CAMS];
    char *intrinsicParams[MAX_CAMS];
    char *extrinsicParams;

    DIR * dirp;
    struct dirent * entry;

    VideoCapture cap[MAX_CAMS];
    int videoFrameCount[MAX_CAMS];

    vector<string> inputList[MAX_CAMS], culledList;
    vector<string> outputList[MAX_CAMS];

    bool sameNum = true;

    if (wantsExtrinsics)
    {
        extrinsicParams = (char*) malloc(strlen(directory) + 128);
        sprintf(extrinsicParams, "%s/%s%d%s", directory, "extrinsics-", numCams, ".yml");
        printf("%s << extrinsicParams = %s\n", __FUNCTION__, extrinsicParams);
    }

    for (unsigned int nnn = 0; nnn < numCams; nnn++)
    {
        inStream[nnn] = (char*) malloc(strlen(directory) + 128);
        outStream[nnn] = (char*) malloc(strlen(directory) + 128);

        intrinsicParams[nnn] = (char*) malloc(strlen(directory) + 128);
        sprintf(intrinsicParams[nnn], "%s/%s%d%s", directory, "intrinsics-", nnn, ".yml");

        printf("%s << intrinsicParams[%d] = %s\n", __FUNCTION__, nnn, intrinsicParams[nnn]);
    }

    int * randomIndexArray;
    randomIndexArray = new int[maxFramesToLoad];

    if (inputIsFolder)
    {

        for (unsigned int nnn = 0; nnn < numCams; nnn++)
        {

#if defined(WIN32)
            UINT counter(0);
            bool working(true);
            string buffer;
            string fileName[1000];

            WIN32_FIND_DATA myimage;
            HANDLE myHandle;

            sprintf(inStream[nnn], "%s\\%d\\", directory, nnn);
            sprintf(outStream[nnn], "%s\\%d-r\\", directory, nnn);
#else
            sprintf(inStream[nnn], "%s/%d/", directory, nnn);
            sprintf(outStream[nnn], "%s/%d-r/", directory, nnn);
#endif



            printf("%s << inStream[%d] = %s\n", __FUNCTION__, nnn, inStream[nnn]);





#if defined(WIN32)

            //printf("%s << DEBUG /%d/%d/\n", __FUNCTION__, 0, -2);

            counter = 0;

            char * imageSearcher;

            imageSearcher = (char*) malloc(strlen(directory) + 128);


            sprintf(imageSearcher, "%s/*", inStream[nnn]);

            myHandle=FindFirstFile(imageSearcher,&myimage);

            if(myHandle!=INVALID_HANDLE_VALUE)
            {

                buffer=myimage.cFileName;

                if (buffer.length() > 4)
                {
                    inputList[nnn].push_back(buffer);
                }


                while(working)
                {
                    FindNextFile(myHandle,&myimage);
                    if(myimage.cFileName!=buffer)
                    {
                        buffer=myimage.cFileName;

                        //printf("%s << %s (%c)\n", __FUNCTION__, buffer.c_str(), buffer.at(buffer.length()-1));
                        //cin.get();

                        if (buffer.length() > 4)
                        {
                            //if ((buffer != "..") && (buffer != ".")) {
                            ++counter;
                            inputList[nnn].push_back(buffer);
                            // printf("%s << Files counted: %d (%s)\n", __FUNCTION__, counter, buffer.c_str());
                            // printf("%s << inputList[%d].at(%d) = %s\n", __FUNCTION__, nnn, counter-1, (inputList[nnn].at(counter-1)).c_str());
                        }

                    }
                    else
                    {
                        //end of files reached
                        working=false;
                    }

                }

                sort(inputList[nnn].begin(), inputList[nnn].end());

                for (int qrw = 0; qrw < inputList[nnn].size(); qrw++)
                {
                    //printf("%s << inputList[%d].at(%d) = %s\n", __FUNCTION__, nnn, qrw, (inputList[nnn].at(qrw).c_str()));
                }

            }

#else
            dirp = opendir(inStream[nnn]);

            while ((entry = readdir(dirp)) != NULL)
            {
                //printf("DEBUG C.\n");
                if (entry->d_type == DT_REG)   // If the entry is a regular file
                {

                    inputList[nnn].push_back(string(entry->d_name));

                }
            }

            closedir(dirp);
#endif




            printf("%s << inputList[%d].size() = %d\n", __FUNCTION__, nnn, inputList[nnn].size());



        }

        for (int nnn = 0; nnn < numCams-1; nnn++)
        {
            if (inputList[nnn].size() != inputList[nnn+1].size())
            {
                sameNum = false;
            }
        }

        if (sameNum)
        {
            maxFramesToLoad = std::min((int)inputList[0].size(), (int)maxFramesToLoad);
            printf("%s << maxFramesToLoad = %d\n", __FUNCTION__, maxFramesToLoad);

            culledList.assign(inputList[0].begin(), inputList[0].end());

            //culledList.swap(inputList[0]);
			//copy(inputList[0].begin(), inputList[0].begin() + inputList[0].size(), culledList.begin());

            randomCulling(culledList, maxFramesToLoad);

            sort(culledList.begin(), culledList.end());
            
            outputList[0].assign(culledList.begin(), culledList.end());

            for (int qrw = 0; qrw < culledList.size(); qrw++)
            {
                //printf("%s << culledList.at(%d) = %s\n", __FUNCTION__, qrw, (culledList.at(qrw).c_str()));
            }

        }
        else
        {
            printf("%s << Frame count mismatch.\n", __FUNCTION__);
            return -1;
        }
    }
    else
    {
        // Input is a video!!

        for (unsigned int nnn = 0; nnn < numCams; nnn++)
        {

            sprintf(inStream[nnn], "%s/%d.avi", directory, nnn);

            //printf("%s << inStream[%d] = %s\n", __FUNCTION__, nnn, inStream[nnn]);

            sprintf(outStream[nnn], "%s/%d.avi", directory, nnn);


            cap[nnn].open(inStream[nnn]);

            if(!cap[nnn].isOpened())
            {
                printf("%s << Failed to open capture device...\n", __FUNCTION__);
                return -1;
            }
            else
            {
                printf("%s << Successfully opened capture device!\n", __FUNCTION__);
            }

            printf("%s << Counting frames...\n", __FUNCTION__);

            double frameCountDbl = 0.0;

            Mat frame;
            cap[nnn] >> frame;
            frameCountDbl = cap[nnn].get(CV_CAP_PROP_FRAME_COUNT);

            videoFrameCount[nnn] = (int) frameCountDbl;

            printf("%s << Estimated frame count = %d\n", __FUNCTION__, videoFrameCount[nnn]);

            cap[nnn].release();

        }

        for (int nnn = 0; nnn < numCams-1; nnn++)
        {
            if (videoFrameCount[nnn] != videoFrameCount[nnn+1])
            {
                sameNum = false;
            }
        }

        if (sameNum)
        {
            maxFramesToLoad = std::min(videoFrameCount[0], (int)maxFramesToLoad);
            printf("%s << maxFramesToLoad = %d\n", __FUNCTION__, maxFramesToLoad);

            // Need to determine some kind of random sequence so that the correct number of frames are extracted...
            // Can store this in an array and have a check when actual frames are extracted..

            generateRandomIndexArray(randomIndexArray, maxFramesToLoad, videoFrameCount[0]);

        }
        else
        {
            printf("%s << Frame count mismatch.\n", __FUNCTION__);
            return -1;
        }

    }

    // printf("%s << Q: %d / %d \n", __FUNCTION__, inputList[0].size(), culledList.size());

    FileStorage fs;

    Mat imageSize_mat[MAX_CAMS], cameraMatrix[MAX_CAMS], newCamMat[MAX_CAMS], distCoeffs[MAX_CAMS], rectCamMat[MAX_CAMS];
    Size imageSize_size[MAX_CAMS];

    for (unsigned int nnn = 0; nnn < numCams; nnn++)
    {
        imageSize_mat[nnn] = Mat(1, 2, CV_16UC1);
    }

    Rect validROI[MAX_CAMS];

    if (wantsExtrinsics && (!wantsIntrinsics))
    {


        for (unsigned int nnn = 0; nnn < numCams; nnn++)
        {


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

            newCamMat[nnn] = getOptimalNewCameraMatrix(cameraMatrix[nnn], distCoeffs[nnn], imageSize_size[nnn], alpha, imageSize_size[nnn], &validROI[nnn]);

            //printf("%s << validROI[%d] = (%d, %d) / (%d, %d)\n", __FUNCTION__, nnn, validROI[nnn].x, validROI[nnn].y, validROI[nnn].width, validROI[nnn].y);



            printf("%s << newCamMat[%d] = ", __FUNCTION__, nnn);



            cout << newCamMat[nnn] << endl;

            rectCamMat[nnn] = getOptimalNewCameraMatrix(cameraMatrix[nnn], distCoeffs[nnn], imageSize_size[nnn], 0.5, imageSize_size[nnn], &validROI[nnn]);

            //printf("%s << validROI[%d] = (%d, %d) / (%d, %d)\n", __FUNCTION__, nnn, validROI[nnn].x, validROI[nnn].y, validROI[nnn].width, validROI[nnn].y);
        }
    }

    //  --------------------------------------------- CREATE ARBITRARY 2D CO-ORDINATE VECTOR
    cv::vector<Point3f> row;
    // Pattern Corner Co-ordinates
    cv::vector<Point2f> cornerSet;
    cv::vector<cv::vector<Point2f> > cornersList[MAX_CAMS];

	if (patternFinderCode == MASK_FINDER_CODE) {
		for (int i = 0; i < 2*y; i++) {
			for (int j = 0; j < 2*x; j++) {
				row.push_back(Point3f(i*gridSize, j*gridSize, 0.0));
			}
		}
	} else {
		for (int i = 0; i <= y; i++) {
			for (int j = 0; j <= x; j++) {
				row.push_back(Point3f(i*gridSize, j*gridSize, 0.0));
			}
		}
	}

    

    char filename[128];

    Mat inputMat[MAX_CAMS];

    vector<Mat> allImages[MAX_CAMS];

    Mat tmpMat, dispMat;

    bool patternFound = false;

    vector<bool> foundRecord[MAX_CAMS];

    int index = 0, frameIndex = 0;
    
    mserParameterGroup mserParams;
    
    if (providedMSERparams) {
		obtainMSERparameters(parametersFile, mserParams);
	}
    
    // --------------------------------------------- THE PATTERN SEARCH

    // Run through each camera separately to find the patterns
    for (unsigned int nnn = 0; nnn < numCams; nnn++)
    {
		
		char outputFilename[256];
		
		char newDirectoryPath[256], patternDirectoryPath[256];

		sprintf(newDirectoryPath, "%s/%d-u", directory, nnn);
		sprintf(patternDirectoryPath, "%s/%d-a", directory, nnn);
		
		if (outputFoundPatterns) {

			#if defined(WIN32)
			CreateDirectory(patternDirectoryPath, NULL);
			#else
			mkdir(patternDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);
			#endif
		}

        index = 0;
        frameIndex = 0;

        int numFramesToCapture = 0;

        if (inputIsFolder)
        {
            numFramesToCapture = min((int)culledList.size(), maxFramesToLoad);
        }
        else
        {
            numFramesToCapture = maxFramesToLoad;
            cap[nnn].open(inStream[nnn]);
        }



        // For each frame for each camera
        while (index < numFramesToCapture)
        {


            if (inputIsFolder)
            {
                //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 3);
                
                //printf("%s << CL = %s\n", __FUNCTION__, (culledList.at(index)).c_str());

                sprintf(filename, "%s%s", inStream[nnn], (culledList.at(index)).c_str());
                //printf("%s << filename = %s\n", __FUNCTION__, filename);
                //cin.get();
                
                if (verboseMode) printf("%s << reading [%d] = %s\n", __FUNCTION__, nnn, filename);
                
                inputMat[nnn] = imread(filename);
                
                if (verboseMode) printf("%s << File read.\n", __FUNCTION__);
            }
            else
            {
                while (frameIndex <= randomIndexArray[index])
                {
                    cap[nnn] >> inputMat[nnn];
                    frameIndex++;
                }


            }
            
            if (outputFoundPatterns) {
				sprintf(outputFilename, "%s/%s", patternDirectoryPath, (culledList.at(index)).c_str());
			}

            //printf("%s << filename = %s\n", __FUNCTION__, filename);

            allImages[nnn].push_back(inputMat[nnn]);
            
            if (verboseMode) printf("%s << Image pushed back.\n", __FUNCTION__);

            //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, 0, 5);

            //printf("%s << inputMat[nnn].size() = (%d, %d)\n", __FUNCTION__, inputMat[nnn].cols, inputMat[nnn].rows);

            //imshow("displayWindow", inputMat[nnn]);
            //waitKey(0);

            /*
            if (wantsToDisplay) {
                imshow("displayWindow", inputMat[nnn]);
                waitKey(40);
            }
            */

            patternFound = false;

            cornerSet.clear();

            switch (patternFinderCode)
            {
            case CHESSBOARD_FINDER_CODE:
                patternFound = findChessboardCorners(inputMat[nnn], cvSize(x,y), cornerSet);
                break;
            case MASK_FINDER_CODE:
                //printf("%s << DEBUG {%d}{%d}\n", __FUNCTION__, x, y);
                patternFound = findMaskCorners_1(inputMat[nnn], cvSize(x,y), cornerSet, mserParams, correctionFactor, PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG);
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
            
             if (verboseMode) printf("%s << Pattern searched for. Result = (%d); cornerSet.size() = (%d)\n", __FUNCTION__, patternFound, cornerSet.size());

			

            //printf("%s << Pattern found? %d\n", __FUNCTION__, patternFound);

            inputMat[nnn].copyTo(dispMat);

			if (patternFinderCode == MASK_FINDER_CODE) {
				drawChessboardCorners(dispMat, cvSize(2*x, 2*y), Mat(cornerSet), patternFound);
			} else {
				drawChessboardCorners(dispMat, cvSize(x, y), Mat(cornerSet), patternFound);
			}
            

            if (wantsToDisplay)
            {
                imshow("displayWindow", dispMat);
                waitKey(40);
            }
            
            if (outputFoundPatterns) {
				imwrite(outputFilename, dispMat);
			}

            index++;

            foundRecord[nnn].push_back(patternFound);
            cornersList[nnn].push_back(cornerSet);

        }

        if (!inputIsFolder)
        {
            cap[nnn].release();
        }

    }

    cv::vector<Mat> distributionMap;

    double radialDistribution[RADIAL_LENGTH];

    vector<int> tagNames[MAX_CAMS], selectedTags[MAX_CAMS];
    vector<vector<int> > extrinsicTagNames, extrinsicSelectedTags;

    cv::vector<cv::vector<Point2f> > candidatesList[MAX_CAMS];

    if (wantsIntrinsics)
    {

        distributionMap.resize(numCams);

        for (unsigned int nnn = 0; nnn < numCams; nnn++)
        {

            imageSize_size[nnn] = inputMat[nnn].size();

            Mat tmpMat = Mat(inputMat[nnn].size(), CV_8UC1);

            distributionMap.at(nnn) = Mat(inputMat[nnn].size(), CV_8UC1);

            cv::vector<cv::vector<Point2f> > intrinsicsList;
            vector<string> extractedList;

            for (unsigned int iii = 0; iii < cornersList[nnn].size(); iii++)
            {
                if (foundRecord[nnn][iii] == true)
                {
                    intrinsicsList.push_back(cornersList[nnn].at(iii));
                    extractedList.push_back(culledList.at(iii));
                    tagNames[nnn].push_back(iii);

                    //printf("%s << tagNames[%d].at(%d) = %d\n", __FUNCTION__, nnn, iii, tagNames[nnn].at(iii));
                }
            }


            if (intrinsicsList.size() > maxPatternsToKeep)
            {
                //randomCulling(extractedList, maxPatternsToKeep, intrinsicsList);
                randomCulling(extractedList, maxPatternsToKeep, intrinsicsList);
            }

            for (int iii = 0; iii < intrinsicsList.size(); iii++)
            {
                candidatesList[nnn].push_back(intrinsicsList[iii]);
            }

            printf("%s << Optimizing Pattern Set...\n", __FUNCTION__);
            // Optimize which frames to use here, replacing the corners vector and other vectors with new set
            //optimizeCalibrationSet(inputMat[nnn], distributionMap.at(nnn), candidatesList[nnn], intrinsicsList, row, optimizationCode, min((int)maxPatternsPerSet, (int)intrinsicsList.size()), radialDistribution, tagNames[nnn], selectedTags[nnn]);
			optimizeCalibrationSet(inputMat[nnn].size(), candidatesList[nnn], candidatesList[nnn], row, selectedTags[nnn], ENHANCED_MCM_OPTIMIZATION_CODE, DEFAULT_NUM, false, intrinsicsFlags);

            cv::vector< cv::vector<Point3f> > objectPoints;
            cv::vector<Mat> rvecs, tvecs;

            for (int iii = 0; iii < candidatesList[nnn].size(); iii++)
            {
                objectPoints.push_back(row);
            }

            rvecs.resize(candidatesList[nnn].size());
            tvecs.resize(candidatesList[nnn].size());

            printf("%s << Optimization Complete.\n", __FUNCTION__);

            if (candidatesList[nnn].size() == 0)
            {
                printf("%s << No patterns remaining - cannot calibrate. Returning.\n", __FUNCTION__);
                return 1;
            }
            else
            {
                printf("%s << Total number of patterns remaining after optimization: %d\n", __FUNCTION__, candidatesList[nnn].size());
            }

            double reprojError, extendedReprojError;

            reprojError = calibrateCamera(objectPoints, candidatesList[nnn], inputMat[nnn].size(), cameraMatrix[nnn], distCoeffs[nnn], rvecs, tvecs, intrinsicsFlags);

            printf("%s << Calibration DONE.\n", __FUNCTION__);

            printf("%s << OpenCV subsequence MRE = %f\n", __FUNCTION__, reprojError);

            double *errValues;
            errValues = new double[intrinsicsList.size() * intrinsicsList.at(0).size()];

            //extendedReprojError = calculateERE(inputMat[nnn], objectPoints.at(0), intrinsicsList, cameraMatrix[nnn], distCoeffs[nnn], errValues);
			extendedReprojError = calculateERE(inputMat[nnn].size(), objectPoints.at(0), candidatesList[nnn], cameraMatrix[nnn], distCoeffs[nnn], errValues);

            printf("%s << Full-sequence MRE = %f\n", __FUNCTION__, extendedReprojError);

            cout << endl << "cameraMatrix = \n" << cameraMatrix[nnn] << endl;
            cout << "distCoeffs = \n" << distCoeffs[nnn] << endl << endl;



            newCamMat[nnn] = getOptimalNewCameraMatrix(cameraMatrix[nnn], distCoeffs[nnn], inputMat[nnn].size(), alpha, inputMat[nnn].size(), &validROI[nnn]);
            rectCamMat[nnn] = getOptimalNewCameraMatrix(cameraMatrix[nnn], distCoeffs[nnn], inputMat[nnn].size(), 0.5, inputMat[nnn].size(), &validROI[nnn]);

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

            if (wantsToUndistort)
            {


                // Create directories

                char newDirectoryPath[256];

                sprintf(newDirectoryPath, "%s/%d-u", directory, nnn);

#if defined(WIN32)
                //boost::filesystem::create_directory(newDirectoryPath);

                CreateDirectory(newDirectoryPath, NULL);
#else
                mkdir(newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);
#endif


                printf("%s << Undistorting Images... (%d)\n", __FUNCTION__, inputList[nnn].size());
                Mat undistortedMat(inputMat[nnn].size(), CV_8UC3);

                //videoReader.set(CV_CAP_PROP_POS_AVI_RATIO, 0.00);

                char inputFilename[256], outputFilename[256];


                for (int i = 0; i < inputList[nnn].size(); i++)
                {
					
					

                    if (inputIsFolder)
                    {
                        //sprintf(inputFilename, "%s%d.%s", input, i+1, "jpg");
                        sprintf(inputFilename, "%s%s", inStream[nnn], (inputList[nnn].at(i)).c_str());
                        inputMat[nnn] = imread(inputFilename);

                    }
                    else
                    {
                        //videoReader >> inputMat;
                    }
                    
                    //printf("%s << Undistorting (%s)...\n", __FUNCTION__, inputFilename);

                    undistort(inputMat[nnn], undistortedMat, cameraMatrix[nnn], distCoeffs[nnn], newCamMat[nnn]);

                    imshow("undistortedWin", undistortedMat);
                    waitKey(40);

                    if (inputIsFolder)
                    {
                        //sprintf(outputFilename, "%s/%d.jpg", newDirectoryPath, i);
                        sprintf(outputFilename, "%s/%s", newDirectoryPath, (inputList[nnn].at(i)).c_str());
                    }
                    else
                    {
                        //sprintf(outputFilename, "%s%s%s", inputFolderString, "undistorted/", (inputList.at(i)).c_str());
                    }

                    //printf("%s << writing to file: %s\n", __FUNCTION__, outputFilename);

                    imwrite(outputFilename, undistortedMat);

                }
            }
        }


    }

    if (wantsExtrinsics)
    {

        cv::vector<Mat> extrinsicsDistributionMap;
        extrinsicsDistributionMap.resize(numCams);

        printf("%s << Calculating extrinsics...\n", __FUNCTION__);

        cv::vector<cv::vector<Point2f> > emptyPointSetVector;
        vector<int> emptyIntVector;

        cv::vector<cv::vector<cv::vector<Point2f> > > extrinsicsList, extrinsicsCandidates;
        vector<string> extractedList;

        for (unsigned int nnn = 0; nnn < numCams; nnn++)
        {
            extrinsicsList.push_back(emptyPointSetVector);
            extrinsicsCandidates.push_back(emptyPointSetVector);
            extrinsicTagNames.push_back(emptyIntVector);

            extrinsicsDistributionMap.at(nnn) = Mat(inputMat[nnn].size(), CV_8UC1);
        }

        for (unsigned int iii = 0; iii < cornersList[0].size(); iii++)
        {

            bool allPatternsFound = true;

            for (unsigned int nnn = 0; nnn < numCams; nnn++)
            {
                if (foundRecord[nnn][iii] == false)
                {
                    allPatternsFound = false;
                }
            }

            if (allPatternsFound)
            {
                for (unsigned int nnn = 0; nnn < numCams; nnn++)
                {
                    extrinsicsList.at(nnn).push_back(cornersList[nnn].at(iii));
                    extrinsicsCandidates.at(nnn).push_back(cornersList[nnn].at(iii));

                    extrinsicTagNames.at(nnn).push_back(iii);


                }



            }

        }


        vector<Size> extrinsicsSizes;

        for (unsigned int nnn = 0; nnn < numCams; nnn++)
        {
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

        for (int iii = 0; iii < extrinsicsCandidates[0].size(); iii++)
        {
            objectPoints.push_back(row);
        }

        for (unsigned int nnn = 0; nnn < numCams; nnn++)
        {
            for (int k = 0; k < numCams-1; k++)
            {
                stereoCalibrate(objectPoints,
                                extrinsicsCandidates.at(0), extrinsicsCandidates.at(k+1),
                                cameraMatrix[0], distCoeffs[0],
                                cameraMatrix[k+1], distCoeffs[k+1],
                                imageSize_size[0],                      // hopefully multiple cameras allow multiple image sizes
                                R[k+1], T[k+1], E[k+1], F[k+1],
                                term_crit,
                                EXTRINSICS_FLAGS); //


            }

            cout << "T[" << nnn << "] = " << endl << T[nnn] << endl;
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

        for (int i = 0; i < numCams; i++)
        {

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

        if (wantsToUndistort)
        {

            // Obtain Rectification Maps
            if (numCams == 2)
            {
                printf("%s << 2 Cameras.\n", __FUNCTION__);

                Rect roi1, roi2;

                stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
                              imageSize_size[0],
                              R[1], T[1],
                              R_[0], R_[1], P_[0], P_[1],
                              Q,
                              CALIB_ZERO_DISPARITY,
                              alpha, imageSize_size[0], &roi1, &roi2);

                /*
                stereoRectify(cameraMatrix[0], distCoeffs[0],
                              cameraMatrix[1], distCoeffs[1],
                              imSize.at(0),
                              R[1], T[1],
                              R_[0], R_[1], P_[0], P_[1],
                              Q,
                              alpha, imSize.at(0), &roi1, &roi2,
                              CALIB_ZERO_DISPARITY);
                  */

            }
            else if (numCams == 3)
            {
                printf("%s << 3 Camera rectification commencing. (alpha = %f)\n", __FUNCTION__, alpha);

                double ratio =  rectify3Collinear(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
                                                  distCoeffs[1], cameraMatrix[2], distCoeffs[2],
                                                  extrinsicsCandidates.at(0), extrinsicsCandidates.at(2),
                                                  imageSize_size[0], R[1], T[1], R[2], T[2],
                                                  R_[0], R_[1], R_[2], P_[0], P_[1], P_[2], Q, alpha,
                                                  imageSize_size[0], 0, 0, CV_CALIB_ZERO_DISPARITY);

                printf("%s << 3 Camera rectification complete.\n", __FUNCTION__);
            }

            Point pt1, pt2;
            vector<Point2f> rectangleBounds, newRecBounds;

            printf("%s << Undistorting\n", __FUNCTION__);

            Mat mapx[MAX_CAMS], mapy[MAX_CAMS];

            int topValidHeight = 0, botValidHeight = 65535, leftValid[MAX_CAMS], rightValid[MAX_CAMS];

            Mat blankCoeffs(1, 8, CV_64F);

            for (int i = 0; i < numCams; i++)
            {

                initUndistortRectifyMap(cameraMatrix[i],
                                        distCoeffs[i],
                                        R_[i],
                                        P_[i],  // newCamMat[i]
                                        imageSize_size[i],
                                        CV_32F,     // CV_16SC2
                                        mapx[i],    // map1[i]
                                        mapy[i]);   // map2[i]

                pt1 = Point(validROI[i].x, validROI[i].y);
                pt2 = Point(validROI[i].x + validROI[i].width, validROI[i].y + validROI[i].height);

                printf("%s << (und) pt1 = (%d, %d); pt2 = (%d, %d)\n", __FUNCTION__, pt1.x, pt1.y, pt2.x, pt2.y);

                rectangleBounds.push_back(Point2f(pt1.x, pt1.y));
                rectangleBounds.push_back(Point2f(pt2.x, pt2.y));

                cout << "rectCamMat[i] = " << endl << rectCamMat[i] << endl;

                undistortPoints(Mat(rectangleBounds), newRecBounds, rectCamMat[i], blankCoeffs, R_[i], P_[i]);

                //printf("%s << Original rectangle points: = (%d, %d) & (%d, %d)\n", __FUNCTION__, pt1.x, pt1.y, pt2.x, pt2.y);

                pt1 = Point(int(newRecBounds.at(0).x), int(newRecBounds.at(0).y));
                pt2 = Point(int(newRecBounds.at(1).x), int(newRecBounds.at(1).y));

                printf("%s << pt1 = (%d, %d); pt2 = (%d, %d)\n", __FUNCTION__, pt1.x, pt1.y, pt2.x, pt2.y);

                if (pt1.y > topValidHeight)
                {
                    topValidHeight = pt1.y;
                }

                if (pt2.y < botValidHeight)
                {
                    botValidHeight = pt2.y;
                }

                leftValid[i] = pt1.x;
                rightValid[i] = pt2.x;
            }

            printf("%s << topValidHeight = %d; botValidHeight = %d\n", __FUNCTION__, topValidHeight, botValidHeight);

            vector<Point2f> leftLinePoints, rightLinePoints;

            // Prepare epipolar lines etc:
            for (int k = 1; k < 32; k++)
            {
                // should try to center it on the final (thermal) image
                leftLinePoints.push_back(Point2f(0, k*(botValidHeight - topValidHeight)/32 - 1));
                rightLinePoints.push_back(Point2f(imageSize_size[0].width, k*(botValidHeight - topValidHeight)/32 - 1));
            }

            // Read in images again
            for (int i = 0; i < numCams; i++)
            {

                for (int index = 0; index < inputList[i].size(); index++)
                {

                    char newDirectoryPath[256];
                    sprintf(newDirectoryPath, "%s/%d-r", directory, i);

#if defined(WIN32)
                    CreateDirectory(newDirectoryPath, NULL);
#else
                    mkdir(newDirectoryPath, DEFAULT_MKDIR_PERMISSIONS);
#endif



                    sprintf(filename, "%s%s", inStream[i], (inputList[i].at(index)).c_str());

                    //printf("%s << Reading in image: %s\n", __FUNCTION__, filename);
                    inputMat[i] = imread(filename);

                    //printf("%s << Remapping...\n", __FUNCTION__);

                    Mat undistortedMat;

                    remap(inputMat[i], undistortedMat, mapx[i], mapy[i], INTER_LINEAR);


                    Point x_1 = Point(leftValid[i], topValidHeight);
                    Point x_2 = Point(rightValid[i], botValidHeight);

                    //printf("%s << undistortedMat[%d].size() = (%d, %d); x = (%d, %d); y = (%d, %d)\n", __FUNCTION__, i, undistortedMat[i].size().width, undistortedMat[i].size().height, x_1.x, x_1.y, x_2.x, x_2.y);

                    cropImage(undistortedMat, x_1, x_2);

                    //printf("%s << Resizing...\n", __FUNCTION__);

                    Mat tmpMat;
                    resize(undistortedMat, tmpMat, Size(), 2.0, 2.0);
                    tmpMat.copyTo(undistortedMat);

                    //printf("%s << DEBUG __%d:%d\n", __FUNCTION__, index, 0);

                    //printf("%s << Drawing...\n", __FUNCTION__);

                    // Draw lines
                    Point2f left, right;
                    /*
                    for (int k = 0; k < 8; k++) {
                        left = Point2f(0, (k+1)*(undistortedMat[i].rows)/8 - 1);
                        right = Point2f(undistortedMat[i].cols, (k+1)*(undistortedMat[i].rows)/8 - 1);
                        line(undistortedMat[i], left, right, color);
                    }
                    */

                    //printf("%s << DEBUG __%d:%d\n", __FUNCTION__, index, 1);


                    // Flip the undistorted mat (this is annoying but may be required...)
                    //flip(undistortedMat[i], inputMat[i], -1);

                    //printf("%s << Displaying...\n", __FUNCTION__);

                    imshow("displayWindow", undistortedMat);
                    waitKey(40);

                    sprintf(filename, "%s/%d.jpg", newDirectoryPath, index);
                    imwrite(filename, undistortedMat);
                }

            }

            printf("%s << Finished undistorting.\n", __FUNCTION__);


        }

    }

    return 0;

}
