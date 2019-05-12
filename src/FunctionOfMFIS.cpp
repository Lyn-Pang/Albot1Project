

//============================================================================
// Name        : Computing MFIS as function and return it 
// Author      : 
// Version     : Modified by Wenwang (Lyn)
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
#include <math.h>

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

#include "GeometryAndExit.h"
#include "GeometryFuncs.H"
#include "GeometricOp.H"
#include "ToolFunctions.h"
#include "thesis.H"
#include "ChunksOp.H"
#include "ConvexHull.h"
#include "Exit.h"
#include "FunctionOfMFIS.h"
#include "StructBoundary.h"

#include <cstdlib>
#include <ctime>

#define PI 3.14159265
using namespace std;

vector<Object> temp_poly_each_step;
vector<Object> each_step_update_poly;

vector<Object> FunctionOfMFIS(int level, int set, int v, int w)
{
    int saveFrom, saveTo;
    int start_Number = 0;
    
    const char* levelName = "inputData/level";
    const char* surfaceName = "/surfaces-";
    char viewFileName[80], mfisFileName[80], ctFileName[80], LocalMFileName[80];
    char exitsFileName[80];
    
    //char testFileName[80]; // this file char array is of test programme and plot

    char Flag_LM;
    LocalMap LMS; // added for chunk & local maps
    Transporter chunks; // added for chunk & local maps
    vector<Chunk> allChunks;// added for chunk & local maps

    vector<Object> NewLM; // a new local map
    vector<Object> PreLM; // previous local map
    vector<Object> ALLlocalMap;
    vector< vector<Object> > ALLlocalMapInGlobal;
    vector<Surface> currentPG; // the polygon of current local map

    vector<Object> LandMarks;

    Object potential_xAxis;

    double crx = 0; // robot position in local map coordinate
    double cry = 0; // 
    double dist_currentLM = 0; // accumulate distance in a local map
    double ang_currentLM = 0; // accumulate angle in a local map
    double Pangle = 0; // accumulating angle

    double dist_rb = 0;

    vector<double> distFromorign;
    vector<double> distFromrbot;

    unsigned char PinLM_Flag = 0; // the flag of previous position of robot is in local map 0 -- not 1 -- yes
    vector<double> odome;

    double traveledDistance = 0; // total distance traversed 
    double robotFacing = 0; //in degree relating to global
    vector<double> facingOrient;

    vector<ASR> pla; // for computing local maps

    vector<Point> RobotPs;
    vector<Point> rbs;
    Point rb; // in current coordinate robot position
    Point Prerb; // in previous coordinate robot position where it moves out

    Point beforeTurning;
    vector<Point> turningPoint; // for generating path
    unsigned char turningFlag = 0; // 0 - not turn  1- turned


    //**********************************//
    vector<Object> GlobalLM;
    vector<ASR> placesOfLM;
    Transporter computeRe, Cmoutput;
    Transporter lastLMInfo;

    //Transporter CombineLMs;
    vector<Object> MapofLMs;

    Object xAxisAtCRP; // x axis angle current robot position in global coordinate
    Object xAxisIncurrentCrd; // x axis angle current robot position in current coordinate
    double PrexAxis;
    Object Pathfromlimiting;
    vector<Object> routeMapCo;

    vector<Object> PreLandmarks;
    vector<Exit> exitsFromcurrentLM;
    vector<Object> odomRef;
    vector<Object> odometricPosition;
    vector<Object> LandMarkCV;
    vector<Object> robotPositionsInLMs;

    vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
    vector<Object> currentTransLM;

    unsigned char firstloop_Flag = 0;  //first loop flag 
    unsigned char lastloop_Flag = 0;   //last loop flag for chunk process

    int CrossedP1 = 0; // when two paths are crossed, return the old segment point1
    int CountOfOutASR = 0;
    int intersection_cnt = 0; //local map process, intersection counter

    vector<int> updatingNumber;
    vector<int> intersection_positions; //all intersection positions
    vector<Object> allASR;
    
    //counters for paper
    int vt_counter = 0; //visual triangulation
    int pi_counter = 0; //path integration
    
    MyRobot robot(0, 0);
    
  
    
    
    updatingNumber.push_back(1);
    rbs.push_back(Point(0, 0));
    facingOrient.push_back(0);
    int referenceNumberTH = 0;
    char mappingInfoFileName[100];


    sprintf(mappingInfoFileName, "%s%d%s", "Maps/Offline/MappingInfo-refTh-", referenceNumberTH, ".txt");
    //variables
    Transporter recognizedTargetObjects, computedOutput, computedLocalMapOutput;
    vector <double> coordTransInfo;
    vector<Object> targetObjectsInPV, targetObjectsInCV, targetObjectsInMFIS;
    vector<Object> allTargetObjectsInPV;
    vector<Object> referenceObjects, odometricReferenceObject;
    //vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
    vector<Object> previousRobotPositionInMFIS;
    vector<Object> recognizedTargetObjectInPV;
    vector<Object> routeMap;
    vector<Object> routeMapForOneASR;
    vector<Object> routeMapConnLP; //route map simply connecting limiting points
    Object pathFromLastLimitingPoint;
    vector<vector<Object> > routeMapForallASR;
    Object lastRouteMapNode, tempLastRouteMapNode;
    vector <Object> allRobotPositions, allOdometricRPosition;
    vector < vector<Object> >AllGlobalRobotPs;
    vector< vector<Object> > allRPoseAtOneStep;
    vector<Object> robotPositionsAtLimitingPoints;
    Transporter objectForPlaceRecognition;
    Transporter loopClosingInfo;
    Transporter lastStepInfo;
    vector<Object> refObjectForLoopClosing;
    string environmentType = "unknown";
    vector<int> lostPoints, limitingPoints, exitPoints, badLocalization;
    Object lineOfSitePoint;
    limitingPoints.push_back(v);
    Object lastLocomotion;
    vector<Object> wholeRoute;

    int odoLocalizationUsed = 0;
    ofstream outFile("Maps/Offline/LocalizationError.txt", ios::out);

    vector<Exit> exitsFromCV;
    vector<Object> crossedExit;

    vector<Object> objectOfCurrentASR;
    ASR currentASR;
    ASRNetwork perceptualMap;
    int ASRNumber = 1;

    //currentLM_ID = potential_ID = 1;
    
    vector<ASR> places;

    MyRobot myrobot(0, 0);
    currentRobotPositionInMFIS = myrobot.getRobot();
    allRobotPositions = currentRobotPositionInMFIS;
    AllGlobalRobotPs.push_back(allRobotPositions);
    allOdometricRPosition = currentRobotPositionInMFIS; //just to see odometric robot position
    odometricCRPositionInMFIS = currentRobotPositionInMFIS; //just to see odometric robot position
    robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;
 
    //ListOfPositions.push_back(currentRobotPositionInMFIS); // push the first position into a list

    RobotPs.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

    //for routeMap
    previousRobotPositionInMFIS = currentRobotPositionInMFIS;
    lastRouteMapNode = currentRobotPositionInMFIS[6];
    tempLastRouteMapNode = lastRouteMapNode;
    routeMap.push_back(lastRouteMapNode);
    routeMapForOneASR.push_back(lastRouteMapNode);
    routeMapConnLP.push_back(Object(0, 0, 0, 0, 1));

    routeMapCo.push_back(Object(0, 0, 0, 0, 1));
    sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);

    //reading the first view
    //cout << "........Reading " << viewFileName << endl;
    vector <Object> currentView = readASCII(viewFileName);
    
    //the first LE
    //allLEs.push_back(currentView);

    if (currentView.size() == 0)
    {
        //cout << "Need to change the file name" << endl;
        surfaceName = "/surface-";
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        currentView = readASCII(viewFileName);
    }

    //finding target objects
    targetObjectsInPV = findTargetObjects(currentView);
    referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
    referenceObjects.push_back(targetObjectsInPV[0]);

    PreLandmarks = targetObjectsInPV;

    //tagging side and view number
    currentView = tagObjectsAsSideAndViewNumber(currentView, v);

    for (unsigned int i = 0; i < currentView.size(); i++)
    {
        currentView[i].setASRNo(v);
        currentView[i].setLimitingPoint(v);
        currentView[i].setLocalEnvID(v);
    }

    //initializing MFIS
    vector<Object> MFIS = currentView;
    
    //added for chunk global map
    vector<Object> ChunkGlobal = currentView;
    
    //cout << "..........MFIS.........." << endl;
    displayObjects(MFIS);
    NewLM = currentView; // first local map
    GlobalLM = currentView;

    allASR = currentView;
    MapofLMs = NewLM; // the first local map in global

    exitsFromcurrentLM = findGapasExits(NewLM); // initial exits in first view/local environment
    
    each_step_update_poly = makePolygonOfView(currentView);
    
    for (int i = 0; i < exitsFromcurrentLM.size(); i++)
        exitsFromcurrentLM[i].display();


    //initializing Perceptual Map
    objectOfCurrentASR = currentView;
    currentASR.setASRObjects(objectOfCurrentASR);
    currentASR.setASRExit1(Object(-500, 0, 500, 0));
    currentASR.setASRID(1);
    lineOfSitePoint = currentRobotPositionInMFIS[6];
    currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
    perceptualMap.setCurrentASR(currentASR);
    
    targetObjectsInMFIS = findTargetObjectsFromMFIS(MFIS);
    
    vector<Object> errorMap = currentView; //odometric error map initialization


    vector<Point> updatingPoints;
    updatingPoints.push_back(Point(0, 0));

    bool exitCrossed = false;
    crossedExit.push_back(currentRobotPositionInMFIS[5]); //consider home as a exit
    crossedExit.back().setID(v);

    vector<int> failedToRecognizeRef;
    ofstream outFileForRefError("Maps/Offline/Localization", ios::out);
    outFileForRefError << v << " " << 1 << endl;
    //cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;


    do
    {
        v++;
        cout << "@ step " << v << endl;
        
        if(v == w)
            lastloop_Flag = 1;
        
        //Read coordTrans Data
        sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", v);
        coordTransInfo = readCoordTrans(ctFileName);

        traveledDistance = traveledDistance + coordTransInfo[0]; //The first element.
        robotFacing = robotFacing + coordTransInfo[1]; //The second element.
        // errorMap = addTwoVectorsOfObjects(errorMap,xformPObjectsIntoCV())

        //changing the filenames 
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);

        currentView = readASCII(viewFileName);
        if (currentView.size() == 0)
        {
            //cout << "Need to change the file name" << endl;
            surfaceName = "/surface-";
            sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
            currentView = readASCII(viewFileName);
        }
        //added begin by arthur
        Point rpos;
        double angle = (coordTransInfo[1] / 180) * PI; //angle in radian
        double rpx = coordTransInfo[0] * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
        double rpy = coordTransInfo[0] * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
        rpos.set(rpx, rpy);

        //vector<Object> cv2pv;
        cout << "angle" << angle << "x:" << rpx << "y:" << rpy << endl;

        odometricErrorMap(errorMap, currentView, coordTransInfo[0], coordTransInfo[1]); //computing error map
        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        //exitsFromCV = PotentialExits(currentView);

        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);

        LandMarkCV = findTargetObjects(currentView);

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);

        currentView = tagObjectsAsSideAndViewNumber(currentView, v);
        referenceObjects = recognizedTargetObjects.getReferenceObjects();

        if (set == 507 && v == 302)
        {
            referenceObjects.clear(); //there is a bug. 
        }      
        
        if ((recognizedTargetObjects.getTargetObjects().size() < 4))
        {
            
            if ((v - limitingPoints.back()) > 1) //update at last step //retriving info to update at last step
            {
                    MFIS = lastStepInfo.getMFIS();
                    //LocalMap = computedLocalMapOutput.getView();
                    currentView = lastStepInfo.getView();
                    referenceObjects = lastStepInfo.getReferenceObjects();
                    currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();

                    if (referenceObjects.size() == 0)
                    {
                        cout << "trying to update at " << v - 1 << " but no ref: " << endl;
                        waitHere();
                    }

                    v = v - 1;
                    vt_counter++;
            } 
            else
                if ((v - limitingPoints.back()) == 1) //update at this step
                {

                        //Check whether these two ref object are quite different        
                        if(abs(referenceObjects[1].length()/referenceObjects[0].length()) > 3)
                        {
                            referenceObjects.clear(); // do not use these two ref object
                        }
                        
                        if (referenceObjects.size() == 0)
                        {
                                //localization using odometer
                                Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
                                aLine.setKP(1);
                                odometricReferenceObject.clear();
                                odometricReferenceObject.push_back(aLine);
                                odometricReferenceObject.push_back(allRobotPositions[6]);
                                odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());
                                referenceObjects = odometricReferenceObject;
                                lostPoints.push_back(v); //just for printing at the end
  
                                pi_counter++;
                        }
                        else
                            vt_counter++;
     
                }
            
            
            //localization 
            currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            AllGlobalRobotPs.push_back(currentRobotPositionInMFIS);
 
            allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS); 
       
            //compute boundary for further task
            temp_poly_each_step = makePolygonOfView(TransformforToGlobalCoordinate(currentView, currentRobotPositionInMFIS[6].getP1(), currentRobotPositionInMFIS, currentRobotPositionInMFIS[7].getAngleWithXaxis()));
            each_step_update_poly = polygon_of_boundary(each_step_update_poly, temp_poly_each_step, currentRobotPositionInMFIS, v);
            
            /*===========================================================================================*/
            //routeMap
            pathFromLastLimitingPoint.set(routeMapConnLP.back().X2(), routeMapConnLP.back().Y2(),
                    currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), v);
            routeMapConnLP.push_back(pathFromLastLimitingPoint);

            robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);
           
            updatingPoints.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

            //old approach
            //computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
            //      allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);

            //modified by Lyn Pang
            //new approach
            computedOutput = updatePerceptualMapATPlaceDeletion(places, MFIS, currentView, currentRobotPositionInMFIS,
                  allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);

            //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            //computedLocalMapOutput = updatePerceptualMapATPlace(places, LocalMap, currentView, currentRobotPositionInMFIS,
            //        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
            MFIS = computedOutput.getView();
            
            //LocalMap = computedLocalMapOutput.getView();
            places = computedOutput.getASRs();
            targetObjectsInPV = computedOutput.getTargetObjects();
            limitingPoints.push_back(v); //limiting/updating points just for printing at the end
          
            recognizedTargetObjectInPV = recognizedTargetObjects.getTargetObjects();
              
        } 
        else
        {
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
            PreLandmarks = computeRe.getTargetObjects();
        }


        //save lastStep information to update at next step(in case)
        lastStepInfo.setMFIS(MFIS);
        lastStepInfo.setView(currentView);
        lastStepInfo.setTargetObjects(targetObjectsInCV);
        lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
        lastStepInfo.setReferenceObjects(referenceObjects);
        lastStepInfo.setExits(exitsFromCV);
        
    } while (v < w); 

    outFile.close();
    outFileForRefError.close();
    //localization 
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

    vector<Object> firstAndLastRP = myrobot.getRobot();
    for (int i = 0; i < currentRobotPositionInMFIS.size(); i++)
    {
        firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
    }

    perceptualMap.setMFIS(MFIS);


    abstractRouteMap(MFIS, robotPositionsAtLimitingPoints, updatingPoints, currentRobotPositionInMFIS);
    keyInfoOfCompuetedPM(mappingInfoFileName, ASRNumber, exitPoints, lostPoints, limitingPoints,
            badLocalization, failedToRecognizeRef, referenceNumberTH, level, set);

    return MFIS;
}



