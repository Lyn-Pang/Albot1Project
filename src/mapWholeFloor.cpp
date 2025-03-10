//============================================================================
// Name        : Algorithm3.cpp
// Author      : Hossain
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <iostream>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <string>

#include "readAndwriteASCII.H"
#include "Plotting.H"
#include "GeometricOp.H"

#include "Point.H"
#include "Object.H"
#include "asr.H"
#include "mfisOp.H"
#include "asrOp.H"
#include "PathPlanning.H"
#include "Transporter.H"

#include "Minfo.H"
#include "Map.H"
#include "CompareASR.H"
#include "Mapping.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"
#include "PolygonChoice.h"
#include "PerceptualMapping.H"
#include "Comparison.h"

#include "thesis.H"

#include <cstdlib>
#include <ctime>

#include "clipper.hpp"

//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/point_xy.hpp>
//#include <boost/geometry/geometries/polygon.hpp>


#define PI 3.14159265

using namespace std;
using namespace ClipperLib;


int main() 
{
    int v, w, level, set, saveFrom, saveTo;

    cout << "Which level?? ";
    cin >> level;
    cout << "Which dataset?? ";
    cin >> set;
    cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
    cin >> v;
    cin >> w;


    int referenceNumberTH = 0;
    char mappingInfoFileName[100];


    sprintf(mappingInfoFileName, "%s%d%s", "Maps/Offline/MappingInfo-refTh-", referenceNumberTH, ".txt");
    //variables
    Transporter recognizedTargetObjects, computedOutput;
    vector <double> coordTransInfo;
    vector<Object> targetObjectsInPV, targetObjectsInCV, targetObjectsInMFIS;
    vector<Object> allTargetObjectsInPV;
    vector<Object> referenceObjects, odometricReferenceObject;
    vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
    vector<Object> previousRobotPositionInMFIS;
    vector<Object> routeMap;
    vector<Object> routeMapForOneASR;
    vector<Object> routeMapConnLP; //route map simply connecting limiting points
    Object pathFromLastLimitingPoint;
    vector<vector<Object> > routeMapForallASR;
    Object lastRouteMapNode, tempLastRouteMapNode;
    vector <Object> allRobotPositions, allOdometricRPosition;
    vector< vector<Object> > allRPoseAtOneStep;
    vector<Object> robotPositionsAtLimitingPoints;
    Transporter objectForPlaceRecognition;
    Transporter loopClosingInfo;
    Transporter lastStepInfo;
    vector<Object> refObjectForLoopClosing;
    string environmentType = "unknown";
    vector<int> lostPoints, limitingPoints, exitPoints, badLocalization;
    Object lineOfSitePoint;
    limitingPoints.push_back(1);
    Object lastLocomotion;
    vector<Object> wholeRoute;
    double traveledDistance = 0;
    double robotFacing = 0; //in degree


    int odoLocalizationUsed = 0;
    ofstream outFile("Maps/Offline/LocalizationError.txt", ios::out);

    vector<Exit> exitsFromCV;
    vector<Object> crossedExit;
    
    vector<Object> LocalMap; // local map stock
    vector<Minfo> recognizedObjects;
    vector<Object> ObjectsInPV, ObjectsInCV;
    vector<Object> recognizedObjectsFromPV;
    vector<Object> recognizedObjectsFromCV;
    vector<Object> extendedObjectsFromCV;
    
    double angle = 0;
    double rpx = 0;
    double rpy = 0;
    Point rpos;
    rpos.set(rpx, rpy);
    int recognizedFlag = 0;
    int FirstFlag = 0;
    int start_n = v;
    

    vector<Object> objectOfCurrentASR;
    ASR currentASR;
    ASRNetwork perceptualMap;
    int ASRNumber = 1;

    vector<ASR> places;

    MyRobot myrobot(0, 0);
    currentRobotPositionInMFIS = myrobot.getRobot();
    allRobotPositions = currentRobotPositionInMFIS;
    allOdometricRPosition = currentRobotPositionInMFIS; //just to see odometric robot position
    odometricCRPositionInMFIS = currentRobotPositionInMFIS; //just to see odometric robot position
    robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;

    //for routeMap
    previousRobotPositionInMFIS = currentRobotPositionInMFIS;
    lastRouteMapNode = currentRobotPositionInMFIS[6]; // the first position is robot's position
    tempLastRouteMapNode = lastRouteMapNode;
    routeMap.push_back(lastRouteMapNode);
    routeMapForOneASR.push_back(lastRouteMapNode);
    routeMapConnLP.push_back(Object(0, 0, 0, 0, 1));

    const char* levelName = "inputData/level";
    const char* surfaceName = "/surfaces-";
    char viewFileName[80], mfisFileName[80], ctFileName[80];
    sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);

    //reading the first view
    cout << "........Reading " << viewFileName << endl;
    vector <Object> currentView = readASCII(viewFileName);
    if (currentView.size() == 0) 
    {
        cout << "Need to change the file name" << endl;
        surfaceName = "/surface-";
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        currentView = readASCII(viewFileName);
    }

    //finding target objects from the first view
    targetObjectsInPV = findTargetObjects(currentView);
    referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
    referenceObjects.push_back(targetObjectsInPV[0]);

    //tagging side and view number
    currentView = tagObjectsAsSideAndViewNumber(currentView, 1);

    for (unsigned int i = 0; i < currentView.size(); i++) 
    {
        currentView[i].setASRNo(1);
        currentView[i].setLimitingPoint(1);
        currentView[i].setLocalEnvID(1);
    }

    //initializing MFIS
    vector<Object> MFIS = currentView;
    cout << "..........MFIS.........." << endl;
    displayObjects(MFIS);
    //waitHere();

    //initializing Perceptual Map
    objectOfCurrentASR = currentView;
    currentASR.setASRObjects(objectOfCurrentASR);
    currentASR.setASRExit1(Object(-500, 0, 500, 0));
    currentASR.setASRID(1);
    lineOfSitePoint = currentRobotPositionInMFIS[6];
    currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
    perceptualMap.setCurrentASR(currentASR);

    ObjectsInPV = currentView;
    LocalMap = currentView;
    
    sprintf(viewFileName, "%s%d%s", "Maps/Offline/view-", v, ".png");
    plotObjectsOf3Kinds(viewFileName, currentView, allRobotPositions, targetObjectsInPV);
    
    sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
    plotObjects(mfisFileName, allRobotPositions, LocalMap);
    


    cout << "home: " << currentRobotPositionInMFIS[6].distP1ToP1(currentView[4]) << endl;


    vector<Object> errorMap = currentView; //odometric error map initialization


    vector<Point> updatingPoints;
    updatingPoints.push_back(Point(0, 0));

    bool exitCrossed = false;
    crossedExit.push_back(currentRobotPositionInMFIS[6]); //consider home as a exit
    crossedExit.back().setID(v);

    vector<int> failedToRecognizeRef;
    ofstream outFileForRefError("Maps/Offline/Localization", ios::out);
    outFileForRefError << v << " " << 1 << endl;
    cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    do {
        
        v++;
        cout << "@ step " << v << endl;

        sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", v);
        coordTransInfo = readCoordTrans(ctFileName);

        traveledDistance = traveledDistance + coordTransInfo[0];
        robotFacing = robotFacing + coordTransInfo[1];
        // errorMap = addTwoVectorsOfObjects(errorMap,xformPObjectsIntoCV())

        
        //changing the filenames 
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        
        //reading current view and coordinate transformation info
        cout << endl << endl << "........Reading " << viewFileName << endl;
        currentView = readASCII(viewFileName);
        
        if (currentView.size() == 0) 
        {
            cout << "Need to change the file name" << endl;
            surfaceName = "/surface-";
            sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
            currentView = readASCII(viewFileName);
        }
        
        //recognizeViews(MFIS,currentView);
        cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;
        odometricErrorMap(errorMap, currentView, coordTransInfo[0], coordTransInfo[1]); //computing error map
        
        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);
        //finding all objects
        ObjectsInCV = currentView;
        //tagging sides and view number
        currentView = tagObjectsAsSideAndViewNumber(currentView, v);

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
        referenceObjects = recognizedTargetObjects.getReferenceObjects();


            
        

         angle = (coordTransInfo[1] / 180) * PI;
         rpx = coordTransInfo[0] * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
         rpy = coordTransInfo[0] * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
         rpos.set(rpx, rpy);
            
         ObjectsInPV = xformPObjectsIntoCV(ObjectsInPV, rpos, angle);
            
        if (set == 507 && v == 302) 
        {
            referenceObjects.clear(); //there is a bug. 
        }
        cout << "(After recognition) no of targetObjects for next step " << recognizedTargetObjects.getTargetObjects().size() << endl;
        cout << "(After recognition) ref objects " << recognizedTargetObjects.getReferenceObjects().size() << endl;



        
        if ((recognizedTargetObjects.getTargetObjects().size() < 3)) //the critial point of updating
        {
            // if ((recognizedTargetObjects.getTargetObjects().size() < referenceNumberTH)) {
            cout << endl << "Updating situation " << endl;


            if ((v - limitingPoints.back()) > 1) //update at last step //retriving info to update at last step
            { 
                MFIS = lastStepInfo.getMFIS();
                LocalEnv = lastStepInfo.getView();
                
                //finding Exits
                exitsFromCV = findShortestExits(LocalEnv);

                //finding target objects
                targetObjectsInCV = findTargetObjects(LocalEnv);


                referenceObjects = lastStepInfo.getReferenceObjects();
                currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();
                if (referenceObjects.size() == 0) {
                    cout << "trying to update at " << v - 1 << " but no ref: " << endl;
                    waitHere();
                }
                v = v - 1;
            } else if ((v - limitingPoints.back()) == 1) {//update at this step

                if (referenceObjects.size() == 0) {
                    
                    LocalEnv = currentView;
                    //finding Exits
                    exitsFromCV = findShortestExits(LocalEnv);

                    //finding target objects
                    targetObjectsInCV = findTargetObjects(LocalEnv);
                    
                    //localization using odometer
                    Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
                    aLine.setKP(1);
                    odometricReferenceObject.clear();
                    odometricReferenceObject.push_back(aLine);
                    odometricReferenceObject.push_back(allRobotPositions[6]);
                    odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());
                    referenceObjects = odometricReferenceObject;
                    lostPoints.push_back(v); //just for printing at the end
                }
            } 

            //localization 
            currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);

            //routeMap
            pathFromLastLimitingPoint.set(routeMapConnLP.back().X2(), routeMapConnLP.back().Y2(),
                    currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), v);
            routeMapConnLP.push_back(pathFromLastLimitingPoint);

            robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);
            updatingPoints.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

            computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                    allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
           
            //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            MFIS = computedOutput.getView();
            places = computedOutput.getASRs();
            targetObjectsInPV = computedOutput.getTargetObjects();
            limitingPoints.push_back(v); //limiting/updating points just for printing at the end


	    //to print local and global map
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
            plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS);
            
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalEnv-", v, ".png");
            plotObjects(viewFileName, myrobot.getRobot(), LocalEnv);
            //plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), currentView, targetObjectsInCV);
            


        } else 
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
        
        
        ObjectsInPV = xformPObjectsIntoCV(ObjectsInPV, rpos, angle);
        
        
        recognizedObjectsFromCV.clear(); // clear old inf
        extendedObjectsFromCV.clear();   // clear old inf

        //recognizing Objects
        if (recognizedFlag == 0)
            recognizedObjects = recognizeObjects(ObjectsInCV,ObjectsInPV);
        else 
            recognizedObjects = recognizeObjects(ObjectsInCV,recognizedObjectsFromPV);
        
        if (int(recognizedObjects.size()) == 0) // a new view// a new local map
        {
            recognizedFlag = 0;
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/view-", v, ".png");
            plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), recognizedObjectsFromCV, extendedObjectsFromCV);

            sprintf(viewFileName, "%s", "Maps/Offline/CurrentView.png");
            plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), recognizedObjectsFromCV, extendedObjectsFromCV);
            
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/view-", v++, ".png");
            plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), currentView);
            
            sprintf(viewFileName, "%s", "Maps/Offline/CurrentView.png");
            plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), currentView);
            
            
            
            /* Here, a flag of new local map */
            
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
            plotObjects(mfisFileName, allRobotPositions, LocalMap);
            
            
        }
        else
        {

            for (int a = 0; a < int(recognizedObjects.size()); a++)   
            {
                Object s = getObject(ObjectsInCV, recognizedObjects[a].getCID()); //
                Object p = getObject(ObjectsInPV, recognizedObjects[a].getPID()); //
                Object e = getExtendedObject(p, s);   // computing the extended objects
                
                recognizedObjectsFromCV.push_back(s); // stored for next step/loop
                extendedObjectsFromCV.push_back(e);   
            }
            recognizedFlag = 1;
            recognizedObjectsFromPV = xformPObjectsIntoCV(recognizedObjectsFromCV, rpos, angle);

            sprintf(viewFileName, "%s%d%s", "Maps/Offline/view-", v, ".png");
            plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), recognizedObjectsFromCV, extendedObjectsFromCV);

            
        }
           
        
        Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
        aLine.setKP(1);
        odometricReferenceObject.clear();
        odometricReferenceObject.push_back(aLine);
        odometricReferenceObject.push_back(allRobotPositions[6]);
        odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());
        
        if (referenceObjects.size() > 0) 
            {
            currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            //localization error checking
            angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(currentRobotPositionInMFIS[6]));
            distanceError = odometricCRPositionInMFIS[6].distP1ToP1(currentRobotPositionInMFIS[6]);
            if (distanceError > 400.0 or angleError > 5.0) 
            {
                referenceObjects = odometricReferenceObject;
                currentRobotPositionInMFIS = odometricCRPositionInMFIS;
            }
        } 
        else 
        {
            referenceObjects = odometricReferenceObject;
            currentRobotPositionInMFIS = odometricCRPositionInMFIS;
        }
       allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);
        
                
         
        
        cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;



        //save lastStep information to update at next step(in case)
        lastStepInfo.setMFIS(MFIS);
        lastStepInfo.setView(currentView);
        lastStepInfo.setTargetObjects(targetObjectsInCV);
        lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
        lastStepInfo.setReferenceObjects(referenceObjects);
        lastStepInfo.setExits(exitsFromCV);
        

        ObjectsInPV = currentView;

    } while (v < w);  //  while(n == 'y' || n == 'Y');
    

//    cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

    outFile.close();
    outFileForRefError.close();
    
    
    
    //localization 
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

    vector<Object> firstAndLastRP = myrobot.getRobot();
    for (int i = 0; i < currentRobotPositionInMFIS.size(); i++) {
        firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
    }

    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-refTH-", referenceNumberTH, ".png");
    plotObjects(mfisFileName, robotPositionsAtLimitingPoints, MFIS);
    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/finalPercptualMap-", set, ".png");
    plotObjectsOf3Kinds(mfisFileName, firstAndLastRP, crossedExit, MFIS);

    cout << "Traveled Dist: " << traveledDistance << endl;
    writeASCII(convertObjectToSurface(MFIS), "Maps/Offline/pm"); //for thesis
    plotObjects("Maps/errorMap.png", errorMap, errorMap);      //for thesis
    perceptualMap.setMFIS(MFIS);
     if (computeASR == true) 
    {
    abstractASRs(places,MFIS,routeMapForallASR,refObjectForLoopClosing);
    }

    abstractRouteMap(MFIS, robotPositionsAtLimitingPoints, updatingPoints, currentRobotPositionInMFIS);
    keyInfoOfCompuetedPM(mappingInfoFileName, ASRNumber, exitPoints, lostPoints, limitingPoints,
            badLocalization, failedToRecognizeRef, referenceNumberTH, level, set);
    cout << levelName << level << " set " << set << endl;
    cout << "odo " << odoLocalizationUsed << endl;

    return 0;
}



