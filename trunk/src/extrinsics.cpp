#include "extrinsics.hpp"

double calculateExtrinsicERE(int nCams,
                             cv::vector<Point3f>& physicalPoints,
                             cv::vector< cv::vector< cv::vector<Point2f> > > corners,
                             Mat *cameraMatrix,
                             Mat *distCoeffs,
                             Mat *R,
                             Mat *T)
{

    int ptsPerSet = physicalPoints.size();
    int numFrames = corners.at(0).size();

    Mat *fsRvec, *fsTvec, *esRvec, *esTvec;
    fsRvec = new Mat[nCams];
    fsTvec = new Mat[nCams];
    esRvec = new Mat[nCams];
    esTvec = new Mat[nCams];

    cv::vector<Point2f> *estimatedPattern, *projectedPattern;
    estimatedPattern = new cv::vector<Point2f>[nCams];
    projectedPattern = new cv::vector<Point2f>[nCams];

    for (int k = 0; k < nCams; k++)
    {
        estimatedPattern[k].resize(physicalPoints.size());
        projectedPattern[k].resize(physicalPoints.size());
    }

    double *errors;

    double tSum = 0, tMean = 0, tDev = 0;

    double xError, yError;

    int maxCapacity = numFrames*ptsPerSet*nCams;

    errors = new double[maxCapacity];

    Mat physPtsMat = Mat(physicalPoints);
    Mat cornersMat;

    int index = 0;

    for (int i = 0; i < numFrames; i++)
    {

        for (int k = 0; k < nCams; k++)
        {

            cornersMat = Mat(corners.at(k).at(i));

            solvePnP(physPtsMat, cornersMat, cameraMatrix[k], distCoeffs[k], fsRvec[k], fsTvec[k], false);

            projectPoints(Mat(physicalPoints), fsRvec[k], fsTvec[k], cameraMatrix[k], distCoeffs[k], estimatedPattern[k]);

            esRvec[k] = R[k] * fsRvec[0];

            esTvec[k] = fsTvec[0] + T[k];

            projectPoints(Mat(physicalPoints), esRvec[k], esTvec[k], cameraMatrix[k], distCoeffs[k], projectedPattern[k]);

            for (unsigned int j = 0; j < estimatedPattern[k].size(); j++)
            {

                xError = abs(projectedPattern[k].at(j).x - estimatedPattern[k].at(j).x);
                yError = abs(projectedPattern[k].at(j).y - estimatedPattern[k].at(j).y);

                errors[index] = pow(pow(xError, 2)+pow(yError, 2), 0.5);

                tSum += (errors[index] / (double(ptsPerSet) * double(nCams)));

                index++;
            }

        }
    }

    double err = tSum / double(numFrames);

    delete[] fsRvec;
    delete[] fsTvec;
    delete[] esRvec;
    delete[] esTvec;

    delete[] estimatedPattern;
    delete[] projectedPattern;

    delete[] errors;

    //printf("%s << Current EMRE = %f\n", __FUNCTION__, err);

    return err;

}

double obtainMultisetScore(int nCams, vector<Mat>& distributionMap, vector<Mat>& binMap, vector<vector<double> >& distances, cv::vector<cv::vector<cv::vector<Point2f> > >& corners, int index)
{
    double score = 0.0;
    double *viewScore;
    viewScore = new double[nCams];

    cv::vector<Point> hull;
    cv::vector<Point2f> hull2;
    double area;
    cv::vector<cv::vector<double> > distancesCpy;

    //distances.copyTo(distancesCpy);   // why the fuck won't this work??

    Point centroid, center;

    double *distFromCenter, *proportionOfView;
    distFromCenter = new double[nCams];         // delete
    proportionOfView = new double[nCams];

    printf("KHAAAN!!! 2\n");

    // for each view
    for (int k = 0; k < nCams; k++)
    {
        printf("K00\n");
        center = Point((distributionMap.at(k).size().width-1)/2, (distributionMap.at(k).size().height-1)/2);

        printf("K01\n");
        // obtain a convex hull around the points
        convexHull(Mat(corners.at(k).at(index)), hull2);

        convertVectorToPoint(hull2, hull);

        printf("K02\n");
        // measure the convex hull's distance from the centre of the image
        centroid = findCentroid(hull);
        distFromCenter[k] = distBetweenPts(centroid, center);

        printf("K03\n");
        // measure the convex hull's proportional coverage from the centre of the image
        area = contourArea(Mat(hull));
        proportionOfView[k] = area;

        // this shit won't work either :P
        //distancesCpy.push_back((distributionMap.at(k).size().width*distributionMap.at(k).size().height)/area);

        // it hardly adds any points when the pattern is far from the center of a view
        viewScore[k] = 1/(distBetweenPts(centroid, center));
        //viewScore[k] = distBetweenPts(centroid, center);

        score += viewScore[k];
    }

    delete[] viewScore;

    delete[] distFromCenter;
    delete[] proportionOfView;

    // Measure get mean and variance of the distances
    // if distances are scored rather than areas, further away points can be emphasized
    // figure out a better cost function

    // Current configuration works OK, but doesn't utilize distances

    return score;
}

void optimizeCalibrationSets(cv::vector<Size> imSize,
                             int nCams,
                             Mat *cameraMatrix,
                             Mat *distCoeffs,
                             cv::vector<Mat>& distributionMap,
                             cv::vector< cv::vector< cv::vector<Point2f> > >& candidateCorners,
                             cv::vector< cv::vector< cv::vector<Point2f> > >& testCorners,
                             cv::vector<Point3f> row,
                             int selection, int num,
                             cv::vector<cv::vector<int> >& tagNames,
                             cv::vector<cv::vector<int> >& selectedTags)
{

    srand ( time(NULL) );

    //printf("%s << Entered.\n", __FUNCTION__);

    if (selection == 0)
    {
        return;
    }

    char windowName[20];

    int randomNum = 0;
    cv::vector<Mat> distributionDisplay(nCams);
    cv::vector<Mat> binMap(nCams);
    cv::vector<Mat> binTemp(nCams);
    cv::vector<cv::vector<double> > distances(nCams);

    // Scoring Variables
    double score, maxScore = 0.0;
    int maxIndex = 0;
    int rankedFrames[MAX_FRAMES_TO_LOAD];
    double rankedScoreVector[MAX_FRAMES_TO_LOAD];
    double *unrankedScores; //[MAX_FRAMES_TO_LOAD];
    unrankedScores = new double[candidateCorners.at(0).size()];

    vector<int> addedIndices;//[MAX_FRAMES_TO_LOAD];
    double bestScore = 0.0;
    int bestIndex;
    double err;

    // For optimum number of frames
    double prevBestScore = 9e99;
    int optimumNum = 0;

    double testingProbability = 1.00;

    cv::vector< cv::vector<Point3f> > objectPoints;

    cv::vector< cv::vector< cv::vector<Point2f> > > originalFramesCpy;
    cv::vector< cv::vector< cv::vector<Point2f> > > selectedFrames;
    cv::vector< cv::vector< cv::vector<Point2f> > > tempFrameTester;
    cv::vector< cv::vector< cv::vector<Point2f> > > newCorners;

    originalFramesCpy.resize(nCams);
    selectedFrames.resize(nCams);
    tempFrameTester.resize(nCams);
    newCorners.resize(nCams);

    //printf("%s << Half variables initialized.\n", __FUNCTION__);

    for (int i = 0; i < nCams; i++)
    {

        imSize.at(i) = distributionMap.at(i).size();

        distributionDisplay.at(i) = Mat(distributionMap.at(i).size(), CV_8UC1);
        binMap.at(i) = Mat(30, 40, CV_32SC1);
        binTemp.at(i) = Mat(binMap.at(i).size(), CV_8UC1);
        binMap.at(i).setTo(0);


        originalFramesCpy.at(i).assign(candidateCorners.at(i).begin(), candidateCorners.at(i).end());     // So all corners are preserved for ERE calculation/s
        //fullSetCorners.at(i).assign(testCorners.at(i).begin(), testCorners.at(i).end());     // So all corners are preserved for ERE calculation/s

    }

    Mat E[MAX_CAMS], F[MAX_CAMS], Q;      // Between first camera and all other cameras
    Mat R[MAX_CAMS], T[MAX_CAMS];         // Rotations/translations between first camera and all other cameras
    Mat R2[MAX_CAMS], T2[MAX_CAMS];       // Rotations/translations between all other cameras
    Mat R_[MAX_CAMS], P_[MAX_CAMS];

    TermCriteria term_crit;
    term_crit = TermCriteria(TermCriteria::COUNT+ TermCriteria::EPS, 30, 1e-6);

    bool alreadyAdded = false;

    //printf("%s << Al variables initialized.\n", __FUNCTION__);

    // SEED VARIABLES
    int nSeeds = 3;
    int nSeedTrials = 50;

    int *bestSeedSet, *currentSeedSet;

    double bestSeedScore = 9e50, currentSeedScore;

    bool alreadyUsed;

    int newRandomNum;

    switch (selection)
    {
        // ==================================================
    case SCORE_BASED_OPTIMIZATION_CODE:     //         SCORE-BASED OPTIMAL FRAME SELECTION
        // ==================================================
        // Until you've sufficiently filled the newCorners vector
        while (newCorners.at(0).size() < (unsigned int)(num))
        {

            printf("%s << newCorners.at(0).size() = %d; num = %d\n", __FUNCTION__, (int)newCorners.at(0).size(), num);

            maxIndex = 0;
            maxScore = 0;

            // For each corner set-set
            for (unsigned int i = 0; i < candidateCorners.at(0).size(); i++)
            {
                score =  obtainMultisetScore(nCams, distributionMap, binMap, distances, candidateCorners, i);
                printf("%s << Frame [%d] score %f\n", __FUNCTION__, i, score);
                if (score > maxScore)
                {
                    maxScore = score;
                    maxIndex = i;
                }
                else if (score < 0)
                {
                    printf("%s << ERROR. Negative score. Returning.\n", __FUNCTION__);
                    delete[] unrankedScores;
                    return;
                }
            }

            printf("%s << Top scoring frame-set #%d gets %f\n", __FUNCTION__, maxIndex, maxScore);

            for (int k = 0; k < nCams; k++)
            {

                //printf("DEBUG Q_001\n");
                newCorners.at(k).push_back(candidateCorners.at(k).at(maxIndex));    // Push highest scorer onto new vector

                //printf("DEBUG Q_002\n");
                addToDistributionMap(distributionMap.at(k), newCorners.at(k).at(newCorners.at(k).size()-1));  // update distribution

                //printf("DEBUG Q_003\n");
                equalizeHist(distributionMap.at(k), distributionDisplay.at(k));

                //printf("DEBUG Q_004\n");
                sprintf(windowName, "distributionMap-%d", k);

                //printf("DEBUG Q_005\n");
                //imshow(windowName, distributionDisplay.at(k));

                //printf("DEBUG Q_006\n");
                //waitKey(5);

                //printf("DEBUG Q_007\n");
                addToBinMap(binMap.at(k), newCorners.at(k).at(newCorners.at(k).size()-1), distributionMap.at(k).size()); // update binned mat

                //printf("DEBUG Q_008\n");
                convertScaleAbs(binMap.at(k), binTemp.at(k));

                //printf("DEBUG Q_009\n");


                simpleResize(binTemp.at(k), distributionDisplay.at(k), Size(480, 640));

                //printf("DEBUG Q_010\n");
                equalizeHist(distributionDisplay.at(k), distributionDisplay.at(k));

                // why isn't this displaying???
                //sprintf(windowName, "binMap-%d", k);

                //printf("DEBUG Q_011\n");
                //imshow(windowName, distributionDisplay.at(k));

                //printf("DEBUG Q_012\n");
                //waitKey(5);

                //printf("DEBUG Q_013\n");

                candidateCorners.at(k).erase(candidateCorners.at(k).begin()+maxIndex);    // Erase it from original vector

                //printf("DEBUG Q_014\n");
            }

            //printf("DEBUG Q_999\n");
            //waitKey(40);

        }

        candidateCorners.clear();
        newCorners.swap(candidateCorners);
        break;
        // ==================================================
    case RANDOM_SET_OPTIMIZATION_CODE:     //              RANDOM FRAME SELECTION
        // ==================================================

        for (int i = 0; i < num; i++)
        {
            randomNum = rand() % candidateCorners.at(0).size();

            for (int k = 0; k < nCams; k++)
            {
                newCorners.at(k).push_back(candidateCorners.at(k).at(randomNum));

                addToDistributionMap(distributionMap.at(k), newCorners.at(k).at(i));
                equalizeHist(distributionMap.at(k), distributionDisplay.at(k));
                sprintf(windowName, "distributionMap-%d", k);
                //imshow(windowName, distributionDisplay.at(k));

            }

            //waitKey(40);

        }

        candidateCorners.clear();
        newCorners.swap(candidateCorners);

        break;
        // ==================================================
    case FIRST_N_PATTERNS_OPTIMIZATION_CODE:     //              FIRST N FRAMES SELECTION
        // ==================================================
        while (candidateCorners.at(0).size() > (unsigned int)(num))
        {
            for (int k = 0; k < nCams; k++)
            {
                candidateCorners.at(k).pop_back();
            }
        }

        for (int i = 0; i < num; i++)
        {

            for (int k = 0; k < nCams; k++)
            {

                addToDistributionMap(distributionMap.at(k), candidateCorners.at(k).at(i));
                equalizeHist(distributionMap.at(k), distributionDisplay.at(k));
                sprintf(windowName, "distributionMap-%d", k);
                imshow(windowName, distributionDisplay.at(k));

            }

            //waitKey(40);

        }

        delete[] unrankedScores;

        return;
        // ==================================================
    case ENHANCED_MCM_OPTIMIZATION_CODE:     //        MULTIPLE-TRIAL OPTIMAL FRAME SELECTION
        // ==================================================

        //printf("%s << Multiple-trial mode\n", __FUNCTION__);

        for (int k = 0; k < nCams; k++)
        {
            selectedFrames.at(k).clear();
        }

        prevBestScore = 9e50;

        for (int N = 0; N < num; N++)
        {

            //printf("%s << DEBUG N = %d\n", __FUNCTION__, N);

            objectPoints.push_back(row);

            for (unsigned int i = 0; i < originalFramesCpy.at(0).size(); i++)
            {

                //printf("%s << DEBUG i = %d\n", __FUNCTION__, i);

                for (int k = 0; k < nCams; k++)
                {
                    //printf("%s << DEBUG tempFrameTester.size() = %d\n", __FUNCTION__, tempFrameTester.size());
                    //printf("%s << DEBUG tempFrameTester.at(%d).size() = %d\n", __FUNCTION__, k, tempFrameTester.at(k).size());
                    //printf("%s << DEBUG selectedFrames.size() = %d\n", __FUNCTION__, selectedFrames.size());
                    //printf("%s << DEBUG selectedFrames.at(%d).size() = %d\n", __FUNCTION__, k, selectedFrames.at(k).size());
                    tempFrameTester.at(k).assign(selectedFrames.at(k).begin(), selectedFrames.at(k).end());
                    //printf("%s << DEBUG X%d\n", __FUNCTION__, 1);
                    tempFrameTester.at(k).push_back(originalFramesCpy.at(k).at(i));
                    //printf("%s << DEBUG X%d\n", __FUNCTION__, 2);
                }

                //printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                alreadyAdded = false;

                // Check if index has already been added
                for (int k = 0; k < addedIndices.size(); k++)
                {
                    if (i == addedIndices.at(k))
                    {

                        alreadyAdded = true;
                    }
                }

                //printf("%s << DEBUG %d\n", __FUNCTION__, 2);

                //printf("%s << DEBUG[i = %d] %d\n", __FUNCTION__, i, 1);

                // This is a better way of corrupting scores for previously added points
                if (alreadyAdded == true)
                {
                    err = -1.0;
                }
                else
                {

                    randomNum = rand() % 1000 + 1;  // random number between 1 and 1000 (inclusive)


                    if (randomNum > (1.0 - testingProbability)*1000.0)
                    {

                        //printf("%s << randomNum = %d\n", __FUNCTION__, randomNum);

                        R[0] = Mat::eye(3, 3, CV_64FC1);
                        T[0] = Mat::zeros(3, 1, CV_64FC1);

                        for (int k = 0; k < nCams-1; k++)
                        {

                            //printf("%s << op.size() = %d, tFT.at(0).size() = %d; tFT.at(%d).size() = %d\n", __FUNCTION__, objectPoints.size(), tempFrameTester.at(0).size(), k+1, tempFrameTester.at(k+1).size());
                            //printf("%s << op.at(0).size() = %d, tFT.at(0).at(0).size() = %d; tFT.at(%d).at(0).size() = %d\n", __FUNCTION__, objectPoints.at(0).size(), tempFrameTester.at(0).at(0).size(), k+1, tempFrameTester.at(k+1).at(0).size());

                            stereoCalibrate(objectPoints,
                                            tempFrameTester.at(0), tempFrameTester.at(k+1),
                                            cameraMatrix[0], distCoeffs[0],
                                            cameraMatrix[k+1], distCoeffs[k+1],
                                            imSize[0],                      // hopefully multiple cameras allow multiple image sizes
                                            R[k+1], T[k+1], E[k+1], F[k+1],
                                            term_crit,
                                            EXTRINSICS_FLAGS); //

                            //printf("%s << Stereo Calibration complete.\n", __FUNCTION__);




                        }

                        // Calculate ERE
                        err = calculateExtrinsicERE(nCams, objectPoints.at(0), testCorners, cameraMatrix, distCoeffs, R, T);

                    }
                    else
                    {
                        err = -1.0;
                    }

                }

                unrankedScores[i] = err;

                //printf("%s << DEBUG %d\n", __FUNCTION__, 3);

            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 4);

            bestScore = 9e50;
            bestIndex = 0;

            for (unsigned int j = 0; j < originalFramesCpy.at(0).size(); j++)
            {
                //printf("%s << unrankedScores[%d] = %f\n", __FUNCTION__, j, unrankedScores[j]);

                if ((unrankedScores[j] < bestScore) && (unrankedScores[j] > 0.0))
                {
                    bestScore = unrankedScores[j];
                    bestIndex = j;
                }
            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 5);

            unrankedScores[bestIndex] = 9e50;

            printf("%s << Best score for %d frameset calibration: %f\n", __FUNCTION__, N+1, bestScore);

            for (int k = 0; k < nCams; k++)
            {
                selectedFrames.at(k).push_back(originalFramesCpy.at(k).at(bestIndex));

            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 6);

            addedIndices.push_back(bestIndex);

            if (bestScore < prevBestScore)
            {
                prevBestScore = bestScore;
                optimumNum = N;
            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 7);

        }

        //printf("%s << Attempting to delete unrankedScores...\n", __FUNCTION__);

        //printf("%s << unrankedScores deleted!.\n", __FUNCTION__);

        printf("%s << Optimum number of framesets for calibration = %d\n", __FUNCTION__, optimumNum+1);

        for (int k = 0; k < nCams; k++)
        {
            candidateCorners.at(k).clear();
            candidateCorners.at(k).assign(selectedFrames.at(k).begin(), selectedFrames.at(k).begin() + optimumNum+1);
        }

        break;
        // ==================================================
    case RANDOM_SEED_OPTIMIZATION_CODE:     //        Random N-seed accumulative search
        // ==================================================

        printf("%s << Initiating Random N-seed accumulative search [seeds = %d; seed trials = %d]\n", __FUNCTION__, nSeeds, nSeedTrials);

        for (int k = 0; k < nCams; k++)
        {
            selectedFrames.at(k).clear();
        }


        unrankedScores = new double[originalFramesCpy.at(0).size()];

        prevBestScore = 9e50;

        //printf("%s << num = %d\n", __FUNCTION__, num);



        bestSeedSet = new int[nSeeds];
        currentSeedSet = new int[nSeeds];

        R[0] = Mat::eye(3, 3, CV_64FC1);
        T[0] = Mat::zeros(3, 1, CV_64FC1);

        //printf("%s << DEBUG %d\n", __FUNCTION__, 0);

        for (int iii = 0; iii < nSeedTrials; iii++)
        {

            //printf("%s << DEBUG [%d] %d\n", __FUNCTION__, iii, 0);

            objectPoints.clear();

            randomNum = rand() % originalFramesCpy.at(0).size();

            currentSeedSet[0] = randomNum;
            objectPoints.push_back(row);

            //printf("%s << DEBUG [%d] %d\n", __FUNCTION__, iii, 1);

            for (int k = 0; k < nCams; k++)
            {
                tempFrameTester.at(k).clear();
                tempFrameTester.at(k).push_back(originalFramesCpy.at(k).at(currentSeedSet[0]));
            }

            //printf("%s << DEBUG [%d] %d\n", __FUNCTION__, iii, 2);

            for (int jjj = 1; jjj < nSeeds; jjj++)
            {
                do
                {
                    alreadyUsed = false;

                    randomNum = rand() % originalFramesCpy.at(0).size();

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

                for (int k = 0; k < nCams; k++)
                {
                    tempFrameTester.at(k).push_back(originalFramesCpy.at(k).at(currentSeedSet[jjj]));
                }

            }

            //printf("%s << seed[0] = %d; seed[1] = %d\n", __FUNCTION__, currentSeedSet[0], currentSeedSet[1]);

            //printf("%s << DEBUG [%d] %d\n", __FUNCTION__, iii, 3);

            //printf("%s << objectPoints.size() = %d\n", __FUNCTION__, objectPoints.size());

            //printf("%s << tempFrameTester.at(%d).size() = %d\n", __FUNCTION__, 0, tempFrameTester.at(0).size());

            for (int k = 0; k < nCams-1; k++)
            {


                //printf("%s << tempFrameTester.at(%d).size() = %d\n", __FUNCTION__, k+1, tempFrameTester.at(k+1).size());

                stereoCalibrate(objectPoints,
                                tempFrameTester.at(0), tempFrameTester.at(k+1),
                                cameraMatrix[0], distCoeffs[0],
                                cameraMatrix[k+1], distCoeffs[k+1],
                                imSize[0],                      // hopefully multiple cameras allow multiple image sizes
                                R[k+1], T[k+1], E[k+1], F[k+1],
                                term_crit,
                                EXTRINSICS_FLAGS);
            }

            //printf("%s << DEBUG [%d] %d\n", __FUNCTION__, iii, 4);

            currentSeedScore = calculateExtrinsicERE(nCams, objectPoints.at(0), testCorners, cameraMatrix, distCoeffs, R, T);

            if (currentSeedScore < bestSeedScore)
            {
                bestSeedScore = currentSeedScore;

                printf("%s << Best seed score [trial = %d]: %f\n", __FUNCTION__, iii, bestSeedScore);

                for (int jjj = 0; jjj < nSeeds; jjj++)
                {
                    bestSeedSet[jjj] = currentSeedSet[jjj];
                }

            }
            else
            {
                //printf("%s << Current seed score [trial = %d]: %f\n", __FUNCTION__, iii, currentSeedScore);
            }

        }

        for (int jjj = 0; jjj < nSeeds; jjj++)
        {

            for (int k = 0; k < nCams; k++)
            {
                selectedFrames.at(k).push_back(originalFramesCpy.at(k).at(bestSeedSet[jjj]));
            }

            unrankedScores[bestSeedSet[jjj]] = 9e50;

            // Corrupt seed frames
            // Don't think it's necessary - removed.

            addedIndices.push_back(bestSeedSet[jjj]);
        }

        bestScore = bestSeedScore;

        // Subtract 1 because later code is dodgy... :P
        optimumNum = nSeeds-1;


        prevBestScore = 9e50;

        for (int N = nSeeds; N < num; N++)
        {

            //printf("%s << DEBUG N = %d\n", __FUNCTION__, N);

            objectPoints.push_back(row);

            for (unsigned int i = 0; i < originalFramesCpy.at(0).size(); i++)
            {

                //printf("%s << DEBUG i = %d\n", __FUNCTION__, i);

                for (int k = 0; k < nCams; k++)
                {
                    //printf("%s << DEBUG tempFrameTester.size() = %d\n", __FUNCTION__, tempFrameTester.size());
                    //printf("%s << DEBUG tempFrameTester.at(%d).size() = %d\n", __FUNCTION__, k, tempFrameTester.at(k).size());
                    //printf("%s << DEBUG selectedFrames.size() = %d\n", __FUNCTION__, selectedFrames.size());
                    //printf("%s << DEBUG selectedFrames.at(%d).size() = %d\n", __FUNCTION__, k, selectedFrames.at(k).size());
                    tempFrameTester.at(k).assign(selectedFrames.at(k).begin(), selectedFrames.at(k).end());
                    //printf("%s << DEBUG X%d\n", __FUNCTION__, 1);
                    tempFrameTester.at(k).push_back(originalFramesCpy.at(k).at(i));
                    //printf("%s << DEBUG X%d\n", __FUNCTION__, 2);
                }

                //printf("%s << DEBUG %d\n", __FUNCTION__, 1);

                alreadyAdded = false;

                // Check if index has already been added
                for (int k = 0; k < addedIndices.size(); k++)
                {
                    if (i == addedIndices.at(k))
                    {

                        alreadyAdded = true;
                    }
                }

                //printf("%s << DEBUG %d\n", __FUNCTION__, 2);

                //printf("%s << DEBUG[i = %d] %d\n", __FUNCTION__, i, 1);

                // This is a better way of corrupting scores for previously added points
                if (alreadyAdded == true)
                {
                    err = -1.0;
                }
                else
                {

                    randomNum = rand() % 1000 + 1;  // random number between 1 and 1000 (inclusive)


                    if (randomNum > (1.0 - testingProbability)*1000.0)
                    {

                        //printf("%s << randomNum = %d\n", __FUNCTION__, randomNum);



                        for (int k = 0; k < nCams-1; k++)
                        {

                            //printf("%s << op.size() = %d, tFT.at(0).size() = %d; tFT.at(%d).size() = %d\n", __FUNCTION__, objectPoints.size(), tempFrameTester.at(0).size(), k+1, tempFrameTester.at(k+1).size());
                            //printf("%s << op.at(0).size() = %d, tFT.at(0).at(0).size() = %d; tFT.at(%d).at(0).size() = %d\n", __FUNCTION__, objectPoints.at(0).size(), tempFrameTester.at(0).at(0).size(), k+1, tempFrameTester.at(k+1).at(0).size());

                            stereoCalibrate(objectPoints,
                                            tempFrameTester.at(0), tempFrameTester.at(k+1),
                                            cameraMatrix[0], distCoeffs[0],
                                            cameraMatrix[k+1], distCoeffs[k+1],
                                            imSize[0],                      // hopefully multiple cameras allow multiple image sizes
                                            R[k+1], T[k+1], E[k+1], F[k+1],
                                            term_crit,
                                            EXTRINSICS_FLAGS); //

                            //printf("%s << Stereo Calibration complete.\n", __FUNCTION__);




                        }

                        // Calculate ERE
                        err = calculateExtrinsicERE(nCams, objectPoints.at(0), testCorners, cameraMatrix, distCoeffs, R, T);

                    }
                    else
                    {
                        err = -1.0;
                    }

                }

                unrankedScores[i] = err;

                //printf("%s << DEBUG %d\n", __FUNCTION__, 3);

            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 4);

            bestScore = 9e50;
            bestIndex = 0;

            for (unsigned int j = 0; j < originalFramesCpy.at(0).size(); j++)
            {
                //printf("%s << unrankedScores[%d] = %f\n", __FUNCTION__, j, unrankedScores[j]);

                if ((unrankedScores[j] < bestScore) && (unrankedScores[j] > 0.0))
                {
                    bestScore = unrankedScores[j];
                    bestIndex = j;
                }
            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 5);

            unrankedScores[bestIndex] = 9e50;

            printf("%s << Best score for %d frameset calibration: %f\n", __FUNCTION__, N+1, bestScore);

            for (int k = 0; k < nCams; k++)
            {
                selectedFrames.at(k).push_back(originalFramesCpy.at(k).at(bestIndex));

            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 6);

            addedIndices.push_back(bestIndex);

            if (bestScore < prevBestScore)
            {
                prevBestScore = bestScore;
                optimumNum = N;
            }

            //printf("%s << DEBUG %d\n", __FUNCTION__, 7);

        }

        //printf("%s << Attempting to delete unrankedScores...\n", __FUNCTION__);

        //printf("%s << unrankedScores deleted!.\n", __FUNCTION__);

        printf("%s << Optimum number of framesets for calibration = %d\n", __FUNCTION__, optimumNum+1);

        for (int k = 0; k < nCams; k++)
        {
            candidateCorners.at(k).clear();
            candidateCorners.at(k).assign(selectedFrames.at(k).begin(), selectedFrames.at(k).begin() + optimumNum+1);
        }

        break;
    default:
        delete[] unrankedScores;
        return;
    }

    //printf("%s << Trying to delete unrankedScores x3...\n", __FUNCTION__);
    delete[] unrankedScores;
    //printf("%s << Deleted!\n", __FUNCTION__);
}
