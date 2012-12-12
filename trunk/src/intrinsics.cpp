#include "intrinsics.hpp"

double calculateERE( Size imSize,
                     cv::vector<Point3f>& physicalPoints,
                     cv::vector< cv::vector<Point2f> >& corners,
                     const Mat& cameraMatrix,
                     const Mat& distCoeffs,
                     double errValues[])
{

    double *errors_in_x, *errors_in_y;

    bool tmpErrorValuesOnly = false;


    if (!errValues)
    {
        tmpErrorValuesOnly = true;
        errValues = new double[corners.size() * corners.at(0).size()];
    }


    Mat fsRvec, fsTvec;
    cv::vector<Point2f> cornerSet;
    cornerSet.resize(physicalPoints.size());

    double err = 0.0, xSum = 0, ySum = 0, tSum = 0, xMean = 0, yMean = 0, tMean = 0, xDev = 0, yDev = 0, tDev = 0;

    Point2f imLoc, imageDec, predictedDec;

    for (unsigned int i = 0; i < corners.size(); i++)
    {
        // Estimate pose of board
        solvePnP(Mat(physicalPoints), Mat(corners.at(i)), cameraMatrix, distCoeffs, fsRvec, fsTvec, false);

        // Reproject the object points using estimated rvec/tvec
        projectPoints(Mat(physicalPoints), fsRvec, fsTvec, cameraMatrix, distCoeffs, cornerSet);
        // do we want distortion vector or not? (noDistVector) it would make undistorted comparison better..

        //image.copyTo(debugImage);

        // Find the mean and standard deviations for differences between results (conventional)
        for (unsigned int j = 0; j < cornerSet.size(); j++)
        {

            imageDec = Point2f(corners.at(i).at(j).x, corners.at(i).at(j).y);
            predictedDec = Point2f(cornerSet.at(j).x, cornerSet.at(j).y);

            if (errValues)
            {
                errValues[i*cornerSet.size()+j] = pow(pow(imageDec.x-predictedDec.x, 2)+pow(imageDec.y-predictedDec.y, 2), 0.5);
                //printf("%s << errValues[%d] = %f\n", __FUNCTION__, i*cornerSet.size()+j, errValues[i*cornerSet.size()+j]);
            }


            // Expand the vector between the actual and predicted points
            predictedDec = imageDec + 3*(predictedDec - imageDec);
        }


    }

    // Calculate means
    for (unsigned int i = 0; i < corners.size(); i++)
    {
        for (unsigned int j = 0; j < cornerSet.size(); j++)
        {
            //xSum += errors_in_x[i*cornerSet.size()+j];
            //ySum += errors_in_y[i*cornerSet.size()+j];
            if (errValues)
            {
                tSum += errValues[i*cornerSet.size()+j];
            }

        }
    }

    //xMean = xSum / (fullSetCorners.size()*cornerSet.size());
    //yMean = ySum / (fullSetCorners.size()*cornerSet.size());
    err = tSum / (corners.size()*cornerSet.size());

    /*
    errTotal = pow(pow(xMean, 2)+pow(yMean, 2), 0.5);

    xDev = pow(xMean/cornerSet.size(), 0.5); // /= (fullSetCorners.size()*cornerSet.size());
    yDev = pow(yMean/cornerSet.size(), 0.5);  // /= (fullSetCorners.size()*cornerSet.size());
    tDev = pow(tMean/cornerSet.size(), 0.5);
    */

    // Blur and normalize fovMat for display:

    //Mat fovMat2;

    // calculate std dev and window size based on no. of points
    //Size winSize;
    //double areaPerPt = double(imSize.width) * double(imSize.height) / double(cornerSet.size());

    //double stdDev = pow(areaPerPt, 0.5);

    // Initially use PURE fovMat to calculate fov score..? Then imagify it...


    //winSize = Size(int(stdDev*3), int(stdDev*3));

    //GaussianBlur(fovMat, fovMat2, winSize, stdDev);
    //normalize_64(fovMat, fovMat2);  // dst, src

    //imshow("errMat", errMat);
    //waitKey(0);


    if (tmpErrorValuesOnly)
    {
        //printf("%s << deleting temporary errValues array...\n", __FUNCTION__);
        delete[] errValues;
    }

    return err;
}

void optimizeCalibrationSet(Size imSize,
                            cv::vector< cv::vector<Point2f> >& candidatePatterns,
                            cv::vector< cv::vector<Point2f> >& testPatterns,
                            cv::vector<Point3f> row,
                            vector<int>& selectedTags,
                            int selection,
                            int num,
                            bool debugMode,
                            int intrinsicsFlags) 
{
	
	if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d].\n", __FUNCTION__, 0);
							
    selectedTags.clear();
    
    Mat distributionMap;

    // If no optimization is desired
    if (selection == 0) {
        return;
    }

    // Initialize Random Number Generator
    srand ( (unsigned int)(time(NULL)) );

    // Calibration Variables
    cv::vector< cv::vector<Point3f> > objectPoints;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat(1, 8, CV_64F);
    cv::vector<Mat> rvecs, tvecs;

    // Pointset Variables
    cv::vector< cv::vector<Point2f> > candidatePatternsCpy;
    candidatePatternsCpy.assign(candidatePatterns.begin(), candidatePatterns.end());     // So all corners are preserved for ERE calculation/s
    cv::vector< cv::vector<Point2f> > fullSetCorners;
    fullSetCorners.assign(testPatterns.begin(), testPatterns.end());     // So all corners are preserved for ERE calculation/s
    cv::vector< cv::vector<Point2f> > selectedFrames;
    cv::vector< cv::vector<Point2f> > tempFrameTester;
    cv::vector< cv::vector<Point2f> > newCorners;

    // Error Measurement Variables
    double err;
    Mat optimalResults(10, ABSOLUTE_MAX_FRAMES_TO_STORE, CV_64FC1);  // Raw results
    Mat rankedScores(10, ABSOLUTE_MAX_FRAMES_TO_STORE, CV_64FC1);    // Reordered results
    Mat rankedIndices(10, ABSOLUTE_MAX_FRAMES_TO_STORE, CV_8UC1);    // Indices in order - to correspond with 'rankedScores'

    // Display and Debugging Variables
    Mat distributionDisplay(distributionMap.size(), CV_8UC1);

    // Optimization Variables
    Mat binMap(30, 40, CV_32SC1);
    Mat binTemp(binMap.size(), CV_8UC1);
    binMap.setTo(0);
    
    if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d].\n", __FUNCTION__, 1);

    // Scoring Variables
    double score, maxScore = 0.0;
    int maxIndex = 0;
    int rankedFrames[ABSOLUTE_MAX_FRAMES_TO_STORE];
    double rankedScoreVector[ABSOLUTE_MAX_FRAMES_TO_STORE];
    double *unrankedScores; // [MAX_FRAMES_TO_LOAD];
    //int addedIndices[MAX_FRAMES_TO_LOAD];
    vector<int> addedIndices;
    double bestScore = 0.0, topScore = 0.0;
    int bestIndex;

    vector<unsigned int> currentIndices, topIndices, bestIndices;

    // For optimum number of frames
    double prevBestScore = 9e99;
    int optimumNum = 0;
    unsigned long int possibleCombos;

    // Gaussian Variables
    Mat gaussianMat(binMap.size().height, binMap.size().width, CV_64FC1);
    createGaussianMatrix(gaussianMat, 0.3);

    // Other Variables
    int randomNum = 0;
    double testingProbability = 1.00;

    double bestErr = 9e99;

    // Random trials code
    int nTrials = 20;

    double median, p90, p99;

    double *values;
    
    //printf("%s << ENTERED. (%d)\n", __FUNCTION__, 0);

    values = new double[num*nTrials];

    /*
    for (int i = 0; i < num; i++) {
    	values[i] = new double[nTrials];
    }
    */

    num = min((int)num, (int)candidatePatterns.size());

    // SEED VARIABLES
    int nSeeds = 5;
    int nSeedTrials = 500;

    int *bestSeedSet, *currentSeedSet;

    double bestSeedScore = 9e50, currentSeedScore;

    bool alreadyUsed;

    int newRandomNum;
    
    double radialDistribution[RADIAL_LENGTH];
    
    vector<int> tagNames;
    
    for (unsigned int iii = 0; iii < candidatePatterns.size(); iii++) {
		tagNames.push_back(iii);
	}

    // Clear radial distribution array
    for (int i = 0; i < RADIAL_LENGTH; i++)
    {
        radialDistribution[i] = 0.0;
    }
    
    //printf("%s << ENTERED. (%d)\n", __FUNCTION__, 1);
    
    if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d] (%d)\n", __FUNCTION__, 2, selection);

    switch (selection)
    {
        // ==================================================
    case SCORE_BASED_OPTIMIZATION_CODE:     //        SCORE-BASED OPTIMAL FRAME SELECTION
        // ==================================================
        // Until you've sufficiently filled the newCorners vector
        while (newCorners.size() < (unsigned int)(num))
        {
            maxScore = 0.0;
            maxIndex = 0;

            // For each corner remaining
            for (unsigned int i = 0; i < candidatePatterns.size(); i++)
            {
                score =  obtainSetScore(distributionMap, binMap, gaussianMat, candidatePatterns.at(i), radialDistribution);
                //printf("%s << Frame [%d] scores %f\n", __FUNCTION__, i, score);
                if (score > maxScore)
                {
                    maxScore = score;
                    maxIndex = i;
                }
                else if (score < 0)
                {
                    printf("%s << ERROR. Negative score. Returning.\n", __FUNCTION__);
                    return;
                }
            }

            //printf("%s << Top scoring frame #%d gets %f\n", __FUNCTION__, maxIndex, maxScore);
            //cin.get();

            newCorners.push_back(candidatePatterns.at(maxIndex));    // Push highest scorer onto new vector

            addToDistributionMap(distributionMap, newCorners.at(newCorners.size()-1));  // update distribution
            addToRadialDistribution(radialDistribution, newCorners.at(newCorners.size()-1), distributionMap.size());

            prepForDisplay(distributionMap, distributionDisplay);
            imshow("distributionMap", distributionDisplay);

            addToBinMap(binMap, newCorners.at(newCorners.size()-1), distributionMap.size()); // update binned mat
            convertScaleAbs(binMap, binTemp);
            simpleResize(binTemp, distributionDisplay, Size(480, 640));
            equalizeHist(distributionDisplay, distributionDisplay);
            //imshow("binMap", distributionDisplay);

            waitKey( 0 );

            candidatePatterns.erase(candidatePatterns.begin()+maxIndex);    // Erase it from original vector
        }

        candidatePatterns.clear();
        newCorners.swap(candidatePatterns);
        break;
        // ==================================================
    case RANDOM_SET_OPTIMIZATION_CODE:     //              RANDOM FRAME SELECTION
        // ==================================================
        for (int i = 0; i < num; i++)
        {
            randomNum = rand() % int(candidatePatterns.size());
            newCorners.push_back(candidatePatterns.at(randomNum));
            candidatePatterns.erase(candidatePatterns.begin()+randomNum);
            selectedTags.push_back(tagNames.at(randomNum));

            addToDistributionMap(distributionMap, newCorners.at(i));
            prepForDisplay(distributionMap, distributionDisplay);
            imshow("distributionMap", distributionDisplay);
            waitKey(40);
        }

        candidatePatterns.clear();
        newCorners.swap(candidatePatterns);
        break;
        // ==================================================
    case FIRST_N_PATTERNS_OPTIMIZATION_CODE:     //              FIRST N FRAMES SELECTION
        // ==================================================
        while (candidatePatterns.size() > (unsigned int)(num))
        {
            candidatePatterns.pop_back();
        }

        for (int i = 0; i < num; i++)
        {
            addToDistributionMap(distributionMap, candidatePatterns.at(i));
            selectedTags.push_back(tagNames.at(i));
            prepForDisplay(distributionMap, distributionDisplay);
            imshow("distributionMap", distributionDisplay);
            waitKey(40);
        }

        delete[] values;

        return;
        // ==================================================
    case ENHANCED_MCM_OPTIMIZATION_CODE:     //        MULTIPLE-TRIAL OPTIMAL FRAME SELECTION
        // ==================================================
        
        if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d] \n", __FUNCTION__, 10);

		//printf("%s << ENTERED. (%d)\n", __FUNCTION__, 2);
        selectedFrames.clear();

        unrankedScores = new double[candidatePatternsCpy.size()];

        prevBestScore = 9e50;

        //printf("%s << num = %d\n", __FUNCTION__, num);
        
        if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d] \n", __FUNCTION__, 11);

        for (int N = 0; N < num; N++)
        {
			
			if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d]\n", __FUNCTION__, 11, N, 0);

            objectPoints.push_back(row);
            
            if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, 11, N, 1);

            //printf("%s << candidatePatternsCpy.size() = %d\n", __FUNCTION__, candidatePatternsCpy.size());

            for (unsigned int i = 0; i < candidatePatternsCpy.size(); i++)
            {
				
				if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 0);

                tempFrameTester.clear();
                tempFrameTester.assign(selectedFrames.begin(), selectedFrames.end());
                tempFrameTester.push_back(candidatePatternsCpy.at(i));
                
                if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 1);

                bool alreadyAdded = false;

                for (int k = 0; k < addedIndices.size(); k++)
                {
                    if (i == addedIndices.at(k))       // this was addedIndices[N] before - but that doesn't make sense...
                    {
                        alreadyAdded = true;
                        //printf("%s << WTF?\n", __FUNCTION__);
                    }
                }
                
                if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 2);

                if (alreadyAdded == true)
                {
                    err = -1.0;
                }
                else
                {
					
					if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 30);

                    randomNum = rand() % 1000 + 1;  // random number between 1 and 1000 (inclusive)

                    Mat fovMat, errMat;
                    double fovScore, errScore;
                    
                    if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 31);

                    if (randomNum > (1 - testingProbability)*1000.0)
                    {
						
						if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 32);
						
                        if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << Calibrating pattern #%d\n", __FUNCTION__, i);

                        if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << objectPoints.size() = %d; tempFrameTester.size() = %d\n", __FUNCTION__, objectPoints.size(), tempFrameTester.size());

						if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << imSize = (%d, %d); objectPoints.at(0).size() = %d; tempFrameTester.at(0).size() = %d\n", __FUNCTION__, imSize.height, imSize.width, objectPoints.at(0).size(), tempFrameTester.at(0).size());

                        calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

						if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 33);

                        //printf("%s << objectPoints.at(0).size() = %d; fullSetCorners.size() = %d\n", __FUNCTION__, objectPoints.at(0).size(), fullSetCorners.size());

                        err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs);
                        
                        if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 34);
                        //printf("%s << err = %f\n", __FUNCTION__, err);
                    }
                    else
                    {
						
						if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 35);
						
                        // If the frame is not to be tested (more likely with lower testingProbability)
                        err = -1.0;
                    }

                }
                
                if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d][%d][%d]\n", __FUNCTION__, 11, N, i, 3);

                unrankedScores[i] = err;
                //printf("%s << score = %f\n", __FUNCTION__, err);
                
                

            }
            
            if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, 11, N, 2);

            bestScore = 9e50;
            bestIndex = 0;

            for (unsigned int j = 0; j < candidatePatternsCpy.size(); j++)
            {

                if ((unrankedScores[j] < bestScore) && (unrankedScores[j] > 0))
                {
                    bestScore = unrankedScores[j];
                    bestIndex = j;
                }
            }
            
            if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, 11, N, 3);

            unrankedScores[bestIndex] = 9e50;

            //printf("%s << Best score for %d frame calibration: %f\n", __FUNCTION__, N+1, bestScore);

            selectedFrames.push_back(candidatePatternsCpy.at(bestIndex));

            // Corrupt best frame in 'originalFramesCpy'
            for (unsigned int i = 0; i < candidatePatternsCpy.at(bestIndex).size(); i++)
            {
                candidatePatternsCpy.at(bestIndex).at(i) = Point2f(0.0,0.0);
            }

            addedIndices.push_back(bestIndex);

            if (bestScore < prevBestScore)
            {
                prevBestScore = bestScore;
                optimumNum = N;
            }
            
            if (debugMode) {
				printf("%s << (%d) frames considered; best generalized error = (%f)\n", __FUNCTION__, N, prevBestScore);
			}
			
			if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, 11, N, 4);

        }

        delete[] unrankedScores;
        
        if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d] \n", __FUNCTION__, 12);

        //printf("%s << Optimum number of frames for calibration = %d\n", __FUNCTION__, optimumNum+1);

        candidatePatterns.clear();
        candidatePatterns.assign(selectedFrames.begin(), selectedFrames.begin() + optimumNum+1);

        for (int i = 0; i < optimumNum+1; i++)
        {
            selectedTags.push_back(tagNames.at(addedIndices.at(i)));
        }
        
        if (INTRINSICS_HPP_DEBUG_MODE > 0) printf("%s << DEBUG [%d] \n", __FUNCTION__, 13);
        
        //printf("%s << ENTERED. (%d)\n", __FUNCTION__, 3);

        break;
        // ==================================================
    case RANDOM_SEED_OPTIMIZATION_CODE:     //        Random N-seed accumulative search
        // ==================================================

        selectedFrames.clear();

        unrankedScores = new double[candidatePatternsCpy.size()];

        prevBestScore = 9e50;

        //printf("%s << num = %d\n", __FUNCTION__, num);



        bestSeedSet = new int[nSeeds];
        currentSeedSet = new int[nSeeds];



        for (int iii = 0; iii < nSeedTrials; iii++)
        {

            objectPoints.clear();
            tempFrameTester.clear();

            randomNum = rand() % candidatePatternsCpy.size();

            currentSeedSet[0] = randomNum;
            objectPoints.push_back(row);


            tempFrameTester.push_back(candidatePatternsCpy.at(currentSeedSet[0]));

            for (int jjj = 1; jjj < nSeeds; jjj++)
            {
                do
                {
                    alreadyUsed = false;

                    randomNum = rand() % candidatePatternsCpy.size();

                    for (int kkk = 0; kkk < jjj; kkk++)
                    {
                        if (randomNum == currentSeedSet[kkk])
                        {
                            alreadyUsed = true;
                        }
                    }
                }
                while (alreadyUsed);

                currentSeedSet[jjj] = randomNum;

                objectPoints.push_back(row);
                tempFrameTester.push_back(candidatePatternsCpy.at(currentSeedSet[jjj]));
            }

            calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

            currentSeedScore = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs);

            if (currentSeedScore < bestSeedScore)
            {
                bestSeedScore = currentSeedScore;

                printf("%s << Best seed score [trial = %d]: %f\n", __FUNCTION__, iii, bestSeedScore);

                for (int jjj = 0; jjj < nSeeds; jjj++)
                {
                    bestSeedSet[jjj] = currentSeedSet[jjj];
                }

            }

        }

        for (int jjj = 0; jjj < nSeeds; jjj++)
        {
            selectedFrames.push_back(candidatePatternsCpy.at(bestSeedSet[jjj]));
            unrankedScores[bestSeedSet[jjj]] = 9e50;

            // Corrupt seed frames
            for (unsigned int kkk = 0; kkk < candidatePatternsCpy.at(bestSeedSet[jjj]).size(); kkk++)
            {
                candidatePatternsCpy.at(bestSeedSet[jjj]).at(kkk) = Point2f(0.0,0.0);
            }

            addedIndices.push_back(bestSeedSet[jjj]);
        }

        bestScore = bestSeedScore;

        // Subtract 1 because later code is dodgy... :P
        optimumNum = nSeeds-1;

        for (int N = nSeeds; N < num; N++)
        {

            objectPoints.push_back(row);

            //printf("%s << candidatePatternsCpy.size() = %d\n", __FUNCTION__, candidatePatternsCpy.size());

            for (unsigned int i = 0; i < candidatePatternsCpy.size(); i++)
            {

                tempFrameTester.clear();
                tempFrameTester.assign(selectedFrames.begin(), selectedFrames.end());
                tempFrameTester.push_back(candidatePatternsCpy.at(i));

                bool alreadyAdded = false;

                for (int k = 0; k < addedIndices.size(); k++)
                {
                    if (i == addedIndices.at(k))       // this was addedIndices[N] before - but that doesn't make sense...
                    {
                        alreadyAdded = true;
                        //printf("%s << WTF?\n", __FUNCTION__);
                    }
                }

                if (alreadyAdded == true)
                {
                    err = -1.0;
                }
                else
                {

                    randomNum = rand() % 1000 + 1;  // random number between 1 and 1000 (inclusive)

                    Mat fovMat, errMat;
                    double fovScore, errScore;

                    if (randomNum > (1 - testingProbability)*1000.0)
                    {
                        //printf("%s << Calibrating pattern #%d\n", __FUNCTION__, i);

                        //printf("%s << objectPoints.size() = %d; tempFrameTester.size() = %d\n", __FUNCTION__, objectPoints.size(), tempFrameTester.size());

                        calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

                        //printf("%s << objectPoints.at(0).size() = %d; fullSetCorners.size() = %d\n", __FUNCTION__, objectPoints.at(0).size(), fullSetCorners.size());

                        err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs);
                        //printf("%s << err = %f\n", __FUNCTION__, err);
                    }
                    else
                    {
                        // If the frame is not to be tested (more likely with lower testingProbability)
                        err = -1.0;
                    }

                }

                unrankedScores[i] = err;

            }

            bestScore = 9e50;
            bestIndex = 0;

            for (unsigned int j = 0; j < candidatePatternsCpy.size(); j++)
            {

                if ((unrankedScores[j] < bestScore) && (unrankedScores[j] > 0))
                {
                    bestScore = unrankedScores[j];
                    bestIndex = j;
                }
            }

            unrankedScores[bestIndex] = 9e50;

            printf("%s << Best score for %d frame calibration: %f\n", __FUNCTION__, N+1, bestScore);

            selectedFrames.push_back(candidatePatternsCpy.at(bestIndex));

            // Corrupt best frame in 'originalFramesCpy'
            for (unsigned int i = 0; i < candidatePatternsCpy.at(bestIndex).size(); i++)
            {
                candidatePatternsCpy.at(bestIndex).at(i) = Point2f(0.0,0.0);
            }

            addedIndices.push_back(bestIndex);

            if (bestScore < prevBestScore)
            {
                prevBestScore = bestScore;
                optimumNum = N;
            }

        }

        delete[] unrankedScores;

        printf("%s << Optimum number of frames for calibration = %d\n", __FUNCTION__, optimumNum+1);

        candidatePatterns.clear();
        candidatePatterns.assign(selectedFrames.begin(), selectedFrames.begin() + optimumNum+1);

        for (int i = 0; i < optimumNum+1; i++)
        {
            selectedTags.push_back(tagNames.at(addedIndices.at(i)));
        }

        break;
        // ==================================================
    case EXHAUSTIVE_SEARCH_OPTIMIZATION_CODE:     //        EXHAUSTIVE TRUE-OPTIMAL SELECTION
        // ==================================================

        if (candidatePatternsCpy.size() > 20)
        {
            printf("%s << Too many frames for exhaustive approach.\n", __FUNCTION__);
            break;
        }
        else
        {
            printf("%s << Searching for absolute optimum.\n", __FUNCTION__);
        }

        bestScore = 9e99;

        // For each different value of N
        for (int N = 0; N < num; N++)
        {

            topScore = 9e99;

            printf("%s << N(+1) = %d\n", __FUNCTION__, N+1);

            objectPoints.push_back(row);

            possibleCombos = 1;

            possibleCombos = factorial(candidatePatternsCpy.size()) / (factorial(N+1) * factorial(candidatePatternsCpy.size() - N - 1));

            printf("%s << possibleCombos = %u\n", __FUNCTION__, (unsigned int) possibleCombos);

            currentIndices.clear();

            // For each possible combo
            for (unsigned int i = 0; i < possibleCombos; i++)
            {

                tempFrameTester.clear();
                getNextCombo(currentIndices, N+1, candidatePatternsCpy.size());

                for (int j = 0; j < currentIndices.size(); j++)
                {
                    //printf("%s << currentIndices.at(%d) = %d\n", __FUNCTION__, j, currentIndices.at(j));
                    tempFrameTester.push_back(candidatePatternsCpy.at(currentIndices.at(j)));
                }

                err = calibrateCamera(objectPoints, tempFrameTester, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

                Mat fovMat, errMat;
                double fovScore, errScore;

                err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs);

                if (err < topScore)
                {
                    topScore = err;
                    topIndices.clear();
                    topIndices.assign(currentIndices.begin(), currentIndices.end());
                }

                if (err < bestScore)
                {
                    bestScore = err;
                    bestIndices.clear();
                    bestIndices.assign(currentIndices.begin(), currentIndices.end());
                }

            }

            printf("%s << topScore [(N+1) = %d] = %f\n", __FUNCTION__, N+1, topScore);

            for (int j = 0; j < topIndices.size(); j++)
            {
                printf("%s << topIndices.at(%d) = %d\n", __FUNCTION__, j, topIndices.at(j));
            }

        }

        candidatePatterns.clear();

        printf("%s << Optimum number of frames for calibration = %d\n", __FUNCTION__, bestIndices.size());
        printf("%s << bestScore = %f\n", __FUNCTION__, bestScore);

        for (unsigned int i = 0; i < bestIndices.size(); i++)
        {
            printf("%s << bestIndices.at(%d) = %d\n", __FUNCTION__, i, bestIndices.at(i));
            candidatePatterns.push_back(candidatePatternsCpy.at(bestIndices.at(i)));
        }

        break;
        // ==================================================
    case BEST_OF_RANDOM_PATTERNS_OPTIMIZATION_CODE:     //              MANY RANDOM TRIALS FRAME SELECTION
        // ==================================================

        bestErr = 9e99;

        printf("%s << Random trial selection\n", __FUNCTION__);

        for (unsigned int k = 0; k < nTrials; k++)
        {

            //printf("%s << Trial #%d.\n", __FUNCTION__, k);

            objectPoints.clear();
            candidatePatterns.clear();
            currentIndices.clear();
            newCorners.clear();

            candidatePatterns.assign(candidatePatternsCpy.begin(), candidatePatternsCpy.end());

            for (int N = 0; N < num; N++)
            {

                objectPoints.push_back(row);
                randomNum = rand() % int(candidatePatterns.size());
                currentIndices.push_back(randomNum);
                newCorners.push_back(candidatePatterns.at(randomNum));
                candidatePatterns.erase(candidatePatterns.begin()+randomNum);
                //printf("%s << oP.size() = %d; nC.size() = %d\n", __FUNCTION__, objectPoints.size(), newCorners.size());

                err = calibrateCamera(objectPoints, newCorners, imSize, cameraMatrix, distCoeffs, rvecs, tvecs, intrinsicsFlags);

                Mat fovMat, errMat;
                double fovScore, errScore;
                err = calculateERE(imSize, objectPoints.at(0), fullSetCorners, cameraMatrix, distCoeffs);

                values[N*nTrials+k] = err;

                //printf("%s << trial #%d, N = %d, score = %f\n", __FUNCTION__, k, N, err);

                if (err < bestErr)
                {
                    bestErr = err;
                    bestIndices.clear();

                    selectedFrames.clear();

                    bestIndices.assign(currentIndices.begin(), currentIndices.end());

                    selectedFrames.assign(newCorners.begin(), newCorners.end());

                    //printf("%s << new bestErr[N = %d] = %f\n", __FUNCTION__, N, bestErr);
                }
            }
        }

        candidatePatterns.clear();

        for (int N = 0; N < num; N++)
        {
            median = findEquivalentProbabilityScore(&values[N*nTrials], nTrials, 0.5);
            p90 = findEquivalentProbabilityScore(&values[N*nTrials], nTrials, 0.9);
            p99 = findEquivalentProbabilityScore(&values[N*nTrials], nTrials, 0.99);

            //printf("%s << Random results for %d frames: median = %f; p90 = %f; p99 = %f\n", __FUNCTION__, N, median, p90, p99);
        }

        candidatePatterns.assign(selectedFrames.begin(), selectedFrames.end());

        break;
    default:

        delete[] values;

        return;
    }
    
    if (INTRINSICS_HPP_DEBUG_MODE > 0)  printf("%s << DEBUG [%d].\n", __FUNCTION__, 99);


    delete[] values;

}
