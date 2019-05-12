/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

//============================================================================
// Name        : Path Simplification and Planning
// Author      : Wenwang
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

#include "GeometryFuncs.H"
#include "GeometricOp.H"
#include "ToolFunctions.h"
#include "thesis.H"
#include "ChunksOp.H"
#include "GeometryAndExit.h"

#include <cstdlib>
#include <ctime>

#define PI 3.14159265
using namespace std;

//***//
void Init_variables();

bool between(double a, double X0, double X1);
bool detectIntersect(vector<Object> CurrentV, Point Pos); // detect the intersection between path and surfaces
bool TwoAdjacentPos(Point A, Point B); // detect nearby position and delete these

// combine all local maps in to one global map
void CombineLocalMaps(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num); //transform and plot
void TransforIntoGloabl(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num); // only transform

// detect the intersection between a new path segment and old path segments
bool TwopathSegment(vector<Point> AllPositions);

// calculate the angle between most recent two adjacent paths
bool TwoPathAngle(vector<Point> AllPositions);

// Transform current position onto old coordinate system
Point TransPointToOldCrd(double Transf_angle, Point Transf_point, Point coord);

void TwoPathsCrossed(int num);

vector<Object> TransPositions(vector<Object> positions, double TransAngle, Point coord);

//void test(vector< vector<Object> > LMS); //for plotting different colour chunks
void TestforAllviews(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num);

const char* levelName = "inputData/level";
const char* surfaceName = "/surfaces-";
char viewFileName[80], mfisFileName[80], ctFileName[80], LocalMFileName[80];
char exitsFileName[80];

char testFileName[80]; // this file char array is of test programme and plot

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


vector< vector<Object> > allLEs; // all local environments 
vector< vector<Object> > Memories; // all local maps as memories
vector< vector<Object> > ListOfPositions; // for storing currentCRPInMFIS variable 
vector< vector<Object> > allRobotPositionsObject;

vector<ASR> pla; // for computing local maps
//vector<Object> TwoLines;
vector<Point> RobotPs;
vector<Point> rbs;

vector<Point> robpositions; //for test
Point rb; // in current coordinate robot position
Point Prerb; // in previous coordinate robot position where it moves out

Point beforeTurning;
vector<Point> turningPoint; // for generating path
unsigned char turningFlag = 0; // 0 - not turn  1- turned


Point PathSegment1P1; // path segment 1 point 1
Point PathSegment1P2; // path segment 1 point 2
Point PathSegment2P1; // path segment 2 point 1
Point PathSegment2P2; // path segment 2 point 2

//**********************************//
vector<Object> GlobalLM;
vector<ASR> placesOfLM;
Transporter computeRe, Cmoutput;
Transporter lastLMInfo;

//Transporter CombineLMs;
vector<Object> MapofLMs;

vector<Object> Refs;
vector<Object> positions; // position in current local map
vector<Object> AP;

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

unsigned char firstloop_Flag = 0;
unsigned char inter_Flag = 0;
unsigned char clear_Flag = 0;
unsigned char pathCrossed_Flag = 0;
unsigned char OutOfASR_Flag = 0;
unsigned char NextOfOutASR_Flag = 0;

int CrossedP1 = 0; // when two paths are crossed, return the old segment point1
int CountOfOutASR = 0;

vector<int> updatingNumber;

MyRobot robot(0, 0);

vector<Object> allASR;

vector<int> intermediate_number;

//counters for paper
int vt_counter = 0; //visual triangulation
int pi_counter = 0; //path integration


int main()
{
    int v, w, r, level, set, saveFrom, saveTo;

    cout << "Which level?? ";
    cin >> level;
    cout << "Which dataset?? ";
    cin >> set;
    cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
    cin >> v;
    cin >> w;

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

    vector<ASR> places;

    MyRobot myrobot(0, 0);
    currentRobotPositionInMFIS = myrobot.getRobot();
    allRobotPositions = currentRobotPositionInMFIS;
    AllGlobalRobotPs.push_back(allRobotPositions);
    allOdometricRPosition = currentRobotPositionInMFIS; //just to see odometric robot position
    odometricCRPositionInMFIS = currentRobotPositionInMFIS; //just to see odometric robot position
    robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;

    ListOfPositions.push_back(currentRobotPositionInMFIS); // push the first position into a list
    allRobotPositionsObject.push_back(currentRobotPositionInMFIS); 

    RobotPs.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));


    AP = positions = robot.getRobot(); // initial the positions.
    odometricPosition = positions;
    //robotPositionsInLMs = robot.getRobot();

    //Arthur's variables;
    int objInCurrentMFIS = 0;
    int objInPreviousMFIS = 0;
    vector<Object> LocalMap;
    char LocalMapFileName[80];
    vector<Object> preMFIS;
    vector<Object> LocalMapLists;
    vector<Object> previousView;
    
    //for test
    robpositions.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));
    //end
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

    if (currentView.size() == 0)
    {
        //cout << "Need to change the file name" << endl;
        surfaceName = "/surface-";
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        currentView = readASCII(viewFileName);
    }
    // previousView = currentView;
    //finding target objects
    targetObjectsInPV = findTargetObjects(currentView);
    referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
    referenceObjects.push_back(targetObjectsInPV[0]);

    PreLandmarks = targetObjectsInPV;
    Refs.push_back(PreLandmarks[0]);
    Refs.push_back(PreLandmarks[0]);

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
    sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", v, ".png");
    plotObjects(LocalMFileName, robot.getRobot(), MapofLMs);
    
    allLEs.push_back(currentView);

    //currentPG = makePolygonOfCV(NewLM); // first local to generate polygon
    Memories.push_back(NewLM); //store the first local map as first memory
    currentTransLM = NewLM; // the first one does not to transform

    exitsFromcurrentLM = findGapasExits(NewLM); // initial exits in first view/local environment

    sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
    plotObjects(viewFileName, myrobot.getRobot(), NewLM);
    
    //---//
    ALLlocalMap = NewLM;
    //initialization of local maps
    LMS = TransViewIntoGloabl(ALLlocalMap, NewLM, Point (0,0), myrobot.getRobot(), 0, 1);
    //---//

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


    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
    sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalEnv-", v, ".png");
    plotObjectsOf3Kinds(viewFileName, allRobotPositions, targetObjectsInPV, currentView);
    plotObjects(mfisFileName, allRobotPositions, MFIS);
    
    targetObjectsInMFIS = findTargetObjectsFromMFIS(MFIS);
    objInCurrentMFIS = targetObjectsInMFIS.size();
    sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");
    plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV, recognizedTargetObjects.getTargetObjects());



    // plotObjects(viewFileName, myrobot.getRobot(), currentView);
    //cout << "home: " << currentRobotPositionInMFIS[6].distP1ToP1(currentView[4]) << endl;


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
        //cout << "@ step " << v << endl;
        //Read coordTrans Data

        sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", v);
        coordTransInfo = readCoordTrans(ctFileName);

        traveledDistance = traveledDistance + coordTransInfo[0]; //The first element.
        robotFacing = robotFacing + coordTransInfo[1]; //The second element.
        // errorMap = addTwoVectorsOfObjects(errorMap,xformPObjectsIntoCV())

        //changing the filenames 
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        //reading current view and coordinate transformation info
        //cout << endl << endl << "........Reading " << viewFileName << endl;
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


        vector<Object> cv2pv;
        cout << "angle" << angle << "x:" << rpx << "y:" << rpy << endl;
        if (previousView.size() != 0)
        {
            cv2pv = xformPObjectsIntoCV(previousView, rpos, angle);
            LocalMap = addTwoVectorsOfObjects(LocalMap, cv2pv);

        }
        previousView = currentView;


        odometricErrorMap(errorMap, currentView, coordTransInfo[0], coordTransInfo[1]); //computing error map
        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        //exitsFromCV = findExits(currentView);


        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);

        LandMarkCV = findTargetObjects(currentView);

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);

        sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");
        //plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV, recognizedTargetObjects.getTargetObjects());
        plotObjects(viewFileName, myrobot.getRobot(), currentView);
        //plotObjectsOf4KindsDottedLandMarks(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV, recognizedTargetObjects.getTargetObjects());
        
        //plot exit and boundary
        //sprintf(exitsFileName, "%s%d%s", "Maps/Offline/exitAndboundary-", v, ".png");
        //plotExitsandBoundary(exitsFileName, exitsFromCV) ;

        currentView = tagObjectsAsSideAndViewNumber(currentView, v);
        referenceObjects = recognizedTargetObjects.getReferenceObjects();

        // for computing local maps
        //computeRe = recognizeTargetObjects(GlobalLM, PreLandmarks, currentView, coordTransInfo, v);
        computeRe = recognizeTargetObjects(GlobalLM, PreLandmarks, LandMarkCV, coordTransInfo, v);
        Refs = computeRe.getReferenceObjects();


        if (set == 507 && v == 302)
        {
            referenceObjects.clear(); //there is a bug. 
        }      
        
        if ((recognizedTargetObjects.getTargetObjects().size() < 4))
        {
            // if ((recognizedTargetObjects.getTargetObjects().size() < referenceNumberTH)) {
            //cout << endl << "Updating situation " << endl;
            if ((v - limitingPoints.back()) > 1) //update at last step //retriving info to update at last step
            {
                MFIS = lastStepInfo.getMFIS();
                LocalMap = computedLocalMapOutput.getView();
                currentView = lastStepInfo.getView();
                referenceObjects = lastStepInfo.getReferenceObjects();
                currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();
                
                Refs = lastLMInfo.getReferenceObjects();
                positions = lastLMInfo.getRobotPosition();

                if (referenceObjects.size() == 0)
                {
                    //cout << "trying to update at " << v - 1 << " but no ref: " << endl;
                    waitHere();
                }
                
                v = v - 1;
                vt_counter++;
            } 
            else
                if ((v - limitingPoints.back()) == 1) //update at this step
                {

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

                                Object ThLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, positions[6], 1);
                                ThLine.setKP(1);
                                odomRef.clear();
                                odomRef.push_back(ThLine);
                                odomRef.push_back(AP[6]);
                                odometricPosition = robot.inMFIS(odomRef[0], odomRef[1], odomRef[0].getKP());
                                Refs = odomRef;
                                
                                pi_counter++;
                        }
                        else
                            vt_counter++;

                        if (Refs.size() == 0)
                        {
                                Object ThLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, positions[6], 1);
                                ThLine.setKP(1);
                                odomRef.clear();
                                odomRef.push_back(ThLine);
                                odomRef.push_back(AP[6]);
                                odometricPosition = robot.inMFIS(odomRef[0], odomRef[1], odomRef[0].getKP());
                                Refs = odomRef;
                        }
                        
                }

            //localization 
            currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            AllGlobalRobotPs.push_back(currentRobotPositionInMFIS);
         
            allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);
            allRobotPositionsObject.push_back(currentRobotPositionInMFIS); 
            
            //localization in current local map
            positions = robot.inMFIS(Refs[0], Refs[1], Refs[0].getKP());
            AP = addTwoVectorsOfObjects(AP, positions);

            //rb.set(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1());
            rb.set(positions[6].X1(), positions[6].Y1());
            xAxisAtCRP = currentRobotPositionInMFIS[7];

            if (detectIntersect(NewLM, rb)) // If the path between starting and current positions is intersected with any surface
            {
                    //cout << "intersection flag: " << (int) inter_Flag << endl;
                    //cout << "number of the view v: " << v << endl;

                    clear_Flag = 1;
                    OutOfASR_Flag = 0;
            } 
            else // should add a special case. When the robot moves out from the behind of starting position.
            {
                    //cout << "No any intersection " << endl << endl;
                    //cout << "It is going to check whether the position is under the X axis" << endl << endl;

                    if (rb.Y() < 0) // If the position appears under X axis, this position has moved out this area
                    {
                        clear_Flag = 1;
                        OutOfASR_Flag = 1;
                    }

            }
            
            /*----------------  Compute for Local Maps ------------------- */
            if (clear_Flag == 0)
            {

                Pathfromlimiting.set(routeMapCo.back().X2(), routeMapCo.back().Y2(),
                        positions[6].X1(), positions[6].Y1(), v);

                routeMapCo.push_back(Pathfromlimiting);

                Cmoutput = updatePerceptualMapATPlace(pla, GlobalLM, currentView, positions,
                        AP, Refs, v, ASRNumber, exitCrossed, crossedExit, routeMapCo);
                //Cmoutput = updatePerceptualMapATPlace(places, LocalMap, currentView, currentRobotPositionInMFIS,
                //allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);

                GlobalLM = Cmoutput.getView();
                pla = Cmoutput.getASRs();
                PreLandmarks = Cmoutput.getTargetObjects();

                /*
                if (detectIntersect(GlobalLM, rb) && turningFlag != 1)
                {
                        if (v != w)
                        {
                                turningPoint.push_back(beforeTurning);
                                turningFlag = 1;
                                //updatingPoints.push_back(v);
                        }
                } 
                else
                    beforeTurning = rb;

                //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/pathTurning-", v, ".png");
                //plotObjectsOf3Kinds(LocalMFileName, robot.getRobot(), positions, GlobalLM);
                */
            } 
            else
            {
                if (clear_Flag == 1)
                {
                    if (OutOfASR_Flag == 0)
                    {
                        sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
                        plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);

                        PreLM = NewLM; // store as previous local map
                        NewLM = currentView; // new local map, new coordinate system

                        //Added for integration//
                        LMS = TransViewIntoGloabl(ALLlocalMap, NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
                        //MoveBackDetection(LMS.getGlobalMaps().back(), LMS.getGlobalPositions(), Point(positions[6].X1(), positions[6].Y1()), v);
                        //---//
                        
                        //intermediate_number.push_back(lastStepInfo.getView()[0].getLocalEnvID()[0]);

                        //Prerb = rb; // save it as old position
                        rbs.push_back(rb);
                        xAxisIncurrentCrd = positions[7];

                        updatingNumber.push_back(v);
                        facingOrient.push_back(robotFacing);

                        //push this position into the list
                        ListOfPositions.push_back(currentRobotPositionInMFIS);

                        //add this position into all
                        robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions

                        // combine each local map
                        CombineLocalMaps(NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);

                        RobotPs.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                        if (TwopathSegment(RobotPs)) // detect two new path is intersect 
                            TwoPathsCrossed(v);
                        else
                        {
                            Prerb = rb; // save it as old position
                            PrexAxis = xAxisIncurrentCrd.getAngleWithXaxis();
                        }

                        Init_variables(); // clear all variables
                    } 
                    else
                    {
                        if (OutOfASR_Flag == 1)
                        {
                            //NewLM = PreLM; // take the pre local map as current local map

                            //coordinate transform the point onto Previous coordinate system
                            rb = TransPointToOldCrd(xAxisIncurrentCrd.getAngleWithXaxis(), rb, Prerb);
                            positions = TransPositions(positions, xAxisIncurrentCrd.getAngleWithXaxis(), Prerb);
                            if (detectIntersect(PreLM, rb))
                            {
                                NewLM = PreLM;
                                sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
                                plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);

                                //PreLM = NewLM; // store as previous local map
                                NewLM = currentView; // new local map, new coordinate system

                               //Added for integration//
                                LMS = TransViewIntoGloabl(ALLlocalMap, NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
                                //MoveBackDetection(LMS.getGlobalMaps().back(), LMS.getGlobalPositions(), Point(positions[6].X1(), positions[6].Y1()), v);
                                //---//

                                rbs.pop_back();
                                rbs.push_back(rb);

                                //push this position into the list
                                ListOfPositions.pop_back();
                                ListOfPositions.push_back(currentRobotPositionInMFIS);

                                //add this position into all
                                //robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions
                                robotPositionsInLMs.clear();
                                for (int i = 0; i < ListOfPositions.size(); i++)
                                    robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, ListOfPositions[i]);

                                Memories.pop_back();
                                // combine each local map
                                TransforIntoGloabl(NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
                                MapofLMs.clear();
                                for (int i = 0; i < Memories.size(); i++)
                                    MapofLMs = addTwoVectorsOfObjects(MapofLMs, Memories[i]);

                                //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", v, ".png");
                                //plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());

                                RobotPs.pop_back();
                                RobotPs.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                                updatingNumber.pop_back();
                                updatingNumber.push_back(v);

                                facingOrient.pop_back();
                                facingOrient.push_back(robotFacing);

                                Prerb = rb; // save it as old position
                                PrexAxis = xAxisIncurrentCrd.getAngleWithXaxis();
                                //if crossed with any surface in this ASR, generate and clear
                                Init_variables();
                            } /*
                            else // no intersection with all surfaces in this ASR -- two cases 1 in ASR 2 back of ASR
                            {
                                if (rb.Y() < 0) // still back of the ASR
                                {
                                    sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
                                    plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);

                                    PreLM = NewLM; // store as previous local map
                                    NewLM = currentView; // new local map, new coordinate system

                                    //Added for integration//
                                    LMS = TransViewIntoGloabl(ALLlocalMap, NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
                                    //MoveBackDetection(LMS.getGlobalMaps().back(), LMS.getGlobalPositions(), Point(positions[6].X1(), positions[6].Y1()), v);
                                    //---//

                                    //Prerb = rb; // save it as old position
                                    xAxisIncurrentCrd = positions[7];

                                    //updatingNumber.pop_back();
                                    updatingNumber.push_back(v);

                                    //facingOrient.pop_back();
                                    facingOrient.push_back(robotFacing);

                                    //rbs.pop_back();
                                    rbs.push_back(rb);

                                    //push this position into the list
                                    //ListOfPositions.pop_back();
                                    ListOfPositions.push_back(currentRobotPositionInMFIS);

                                    //RobotPs.pop_back();
                                    RobotPs.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                                    //add this position into all
                                    //robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions
                                    robotPositionsInLMs.clear();
                                    for (int i = 0; i < ListOfPositions.size(); i++)
                                        robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, ListOfPositions[i]);

                                    Memories.pop_back();
                                    // combine each local map
                                    TransforIntoGloabl(NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);

                                    MapofLMs.clear();
                                    for (int i = 0; i < Memories.size(); i++)
                                        MapofLMs = addTwoVectorsOfObjects(MapofLMs, Memories[i]);

                                    sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", v, ".png");
                                    plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());

                                    if (TwopathSegment(RobotPs)) // detect two new path is intersect 
                                        TwoPathsCrossed(v);
                                    else
                                    {
                                        Prerb = rb; // save it as old position
                                        PrexAxis = xAxisIncurrentCrd.getAngleWithXaxis();
                                    }

                                    //if crossed with any surface in this ASR, generate and clear
                                    Init_variables();
                                } 
                                else
                                    if (rb.Y() > 0) // still in the ASR
                                    {
                                        //positions = TransPositions(positions, xAxisIncurrentCrd.getAngleWithXaxis(), Prerb);
                                        //ListOfPositions.pop_back();
                                        //Memories.pop_back();
                                        clear_Flag = 0;

                                        Pathfromlimiting.set(routeMapCo.back().X2(), routeMapCo.back().Y2(),
                                                positions[6].X1(), positions[6].Y1(), v);

                                        routeMapCo.push_back(Pathfromlimiting);

                                        Cmoutput = updatePerceptualMapATPlace(pla, GlobalLM, currentView, positions,
                                                AP, Refs, v, ASRNumber, exitCrossed, crossedExit, routeMapCo);

                                        GlobalLM = Cmoutput.getView();
                                        pla = Cmoutput.getASRs();
                                        PreLandmarks = Cmoutput.getTargetObjects();

                                    }

                             }*/
                             OutOfASR_Flag = 0;
                        }
                    }
                }
            }

            /*===========================================================================================*/
            //routeMap
            pathFromLastLimitingPoint.set(routeMapConnLP.back().X2(), routeMapConnLP.back().Y2(),
                    currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), v);
            routeMapConnLP.push_back(pathFromLastLimitingPoint);

            robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);
           
            updatingPoints.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

            computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                  allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);


            //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            computedLocalMapOutput = updatePerceptualMapATPlace(places, LocalMap, currentView, currentRobotPositionInMFIS,
                    allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
            MFIS = computedOutput.getView();
            LocalMap = computedLocalMapOutput.getView();
            places = computedOutput.getASRs();
            targetObjectsInPV = computedOutput.getTargetObjects();
            limitingPoints.push_back(v); //limiting/updating points just for printing at the end

            //added for chunk//   
            chunks = updateGlobalMapForChunks(places, ChunkGlobal, currentView, currentRobotPositionInMFIS,
                    allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, LMS, 0);
            ChunkGlobal = chunks.getView();
            
            if(chunks.getChunkFlag() == 1)
                allChunks.push_back(chunks.getChunk());
            
            //to print local and global map
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
            //plotpathandobject(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS, updatingPoints);                      
            plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS);
                     
            //plotting ASRs/Local Environments
            //sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalEnv-", v, ".png");
            //plotObjects(viewFileName, myrobot.getRobot(), currentView);
                     
            
            recognizedTargetObjectInPV = recognizedTargetObjects.getTargetObjects();
            
            //for test
            TestforAllviews(currentView, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
            robpositions.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));
        } 
        else
        {
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
            PreLandmarks = computeRe.getTargetObjects();
        }


        if (v == w) // the last step update still
        {

            cout << "----This is the last step, it is going to process and generate final map here----" << endl;
            //waitHere();
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
            plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);
            
            PreLM = NewLM; // store as previous local map
            NewLM = currentView; // new local map, new coordinate system
            
            //---//
            //ALLlocalMap = TransViewIntoGloabl(ALLlocalMap, NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
            //ALLlocalMapInGlobal.push_back(ALLlocalMap);
            LMS = TransViewIntoGloabl(ALLlocalMap, NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
            chunks = updateGlobalMapForChunks(places, ChunkGlobal, currentView, currentRobotPositionInMFIS,
                    allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, LMS, 1);
           
            if(chunks.getChunkFlag() == 1)
                    allChunks.push_back(chunks.getChunk());
            //GenerateChunks(LMS.getGlobalMaps() , v);
            //---//
            
            Prerb = rb; // save it as old position
            rbs.push_back(rb);
            xAxisIncurrentCrd = positions[7];

            updatingNumber.push_back(v);
            facingOrient.push_back(robotFacing);
            //push this position into the list
            ListOfPositions.push_back(currentRobotPositionInMFIS);

            //add this position into all
            robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions

            // combine each local map
            CombineLocalMaps(NewLM, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);

            RobotPs.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

            if (TwopathSegment(RobotPs)) // detect two new path is intersect 
                TwoPathsCrossed(v);


            Init_variables(); // clear all variables
        }
        
        ////////////


        //save lastStep information to update at next step(in case)
        lastStepInfo.setMFIS(MFIS);
        lastStepInfo.setView(currentView);
        lastStepInfo.setTargetObjects(targetObjectsInCV);
        lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
        lastStepInfo.setReferenceObjects(referenceObjects);
        lastStepInfo.setExits(exitsFromCV);

        if (clear_Flag == 0)
        {
            //for computing local maps
            lastLMInfo.setMFIS(GlobalLM);
            lastLMInfo.setView(currentView);
            lastLMInfo.setTargetObjects(PreLandmarks);
            lastLMInfo.setRobotPosition(positions);
            lastLMInfo.setReferenceObjects(Refs);
            //lastLMInfo.setExits(exitsFromCV);
        } 
        else
            Transporter lastLMInfo;


        firstloop_Flag = 1;
    } while (v < w); //(v != 79);//  //(v != 55); //  while(n == 'y' || n == 'Y');//

    cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

    outFile.close();
    outFileForRefError.close();
    //localization 
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
    allRobotPositionsObject.push_back(currentRobotPositionInMFIS); 
    
    vector<Object> firstAndLastRP = myrobot.getRobot();
    for (int i = 0; i < currentRobotPositionInMFIS.size(); i++)
    {
        firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
    }

    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-refTH-", referenceNumberTH, ".png");
    cout<<"size:----------------"<<allRobotPositions.size()<<endl;
    plotObjects(mfisFileName, robotPositionsAtLimitingPoints, MFIS);
    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/finalPercptualMap-", set, ".png");
    plotObjectsOf3Kinds(mfisFileName, firstAndLastRP, crossedExit, MFIS);

    writeASCII(convertObjectToSurface(MFIS), "Maps/Offline/pm"); //for thesis

    perceptualMap.setMFIS(MFIS);


    abstractRouteMap(MFIS, robotPositionsAtLimitingPoints, updatingPoints, currentRobotPositionInMFIS);
    keyInfoOfCompuetedPM(mappingInfoFileName, ASRNumber, exitPoints, lostPoints, limitingPoints,
            badLocalization, failedToRecognizeRef, referenceNumberTH, level, set);

    
    ////////****************************//////////
    RegionAndBoundary(allLEs, robpositions, LMS, allRobotPositionsObject);
    //combinedChunkGeometry(allLEs, robpositions, LMS);
    ///////*****************************/////////
} 

void Init_variables()
{
            //re-initialization coordinate system and all positions
            GlobalLM.clear();
            pla.clear();
            PreLandmarks.clear();
            routeMapCo.clear();
            positions.clear();
            Refs.clear();
            AP.clear();
            odomRef.clear();
            odometricPosition.clear();
            MyRobot robot(0, 0);
            CrossedP1 = 0;

            //NewLM = currentView; // new local map, new coordinate system
            GlobalLM = NewLM;

            PreLandmarks = findTargetObjects(NewLM);
            Refs.push_back(PreLandmarks[0]);
            Refs.push_back(PreLandmarks[0]);
            AP = positions = robot.getRobot();
            odometricPosition = positions;
            routeMapCo.push_back(Object(0, 0, 0, 0, 1));
            pathCrossed_Flag = 0;
            //OutOfASR_Flag = 0;
            clear_Flag = 0; // clear flag first
            turningFlag = 0;
}

///****************************************************************///
//** detect whether two line segments are intersected **//

bool between(double a, double X0, double X1)
{
           double temp1 = a - X0;
           double temp2 = a - X1;
           if ((temp1 < 1e-8 && temp2 > -1e-8) || (temp2 < 1e-6 && temp1 > -1e-8))
           {
                      return true;
           }
           else
           {
                      return false;
           }
}

bool detectIntersect(vector<Object> CurrentV, Point Pos)
{
    cout << "***** This is going to a process two line segments are intersected *****" << endl << endl;

    Point p1, p2, p3, p4;
    double line_x, line_y; //intersect position  


    p3.set(0.0, 0.0); // origin point (0,0)
    p4.set(Pos.X(), Pos.Y());

    inter_Flag = 0;

    for (int i = 0; i < CurrentV.size(); i++)
    {
        p1.set(CurrentV[i].X1(), CurrentV[i].Y1());
        p2.set(CurrentV[i].X2(), CurrentV[i].Y2());

        if ((fabs(p1.X() - p2.X()) < 1e-6) && (fabs(p3.X() - p4.X()) < 1e-6))
        {
            //return false;  
            inter_Flag = 0;
        } else if ((fabs(p1.X() - p2.X()) < 1e-6))
        {
            if (between(p1.X(), p3.X(), p4.X()))
            {
                double k = (p4.Y() - p3.Y()) / (p4.X() - p3.X());
                line_x = p1.X();
                line_y = k * (line_x - p3.X()) + p3.Y();

                if (between(line_y, p1.Y(), p2.Y()))
                {

                    inter_Flag = 1;
                    return true;
                } else
                {
                    //return false;  
                    inter_Flag = 0;
                }
            } else
            {
                //return false;  
                inter_Flag = 0;
            }
        } else if ((fabs(p3.X() - p4.X()) < 1e-6))
        {
            if (between(p3.X(), p1.X(), p2.X()))
            {
                double k = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
                line_x = p3.X();
                line_y = k * (line_x - p2.X()) + p2.Y();

                if (between(line_y, p3.Y(), p4.Y()))
                {

                    inter_Flag = 1;
                    return true;
                } else
                {
                    //return false;  
                    inter_Flag = 0;
                }
            } else
            {
                //return false;  
                inter_Flag = 0;
            }
        } else
        {
            double k1 = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
            double k2 = (p4.Y() - p3.Y()) / (p4.X() - p3.X());

            if (fabs(k1 - k2) < 1e-6)
            {
                //return false;  
                inter_Flag = 0;
            } else
            {
                line_x = ((p3.Y() - p1.Y()) - (k2 * p3.X() - k1 * p1.X())) / (k1 - k2);
                line_y = k1 * (line_x - p1.X()) + p1.Y();
            }

            if (between(line_x, p1.X(), p2.X()) && between(line_x, p3.X(), p4.X()))
            {

                inter_Flag = 1;
                return true;
            } else
            {
                //return false;  
                inter_Flag = 0;
            }
        }

    }

    if (inter_Flag == 0)
        return false;
    // if(inter_Flag == 1)
    //     return true;
}


//*** detect two positions are in same place ***//

bool TwoAdjacentPos(Point A, Point B)
{
           double distBetween = 0;
           double Xaxis = 0;
           double Yaxis = 0;

           Xaxis = B.X() - A.X();
           Yaxis = B.Y() - A.Y();
           distBetween = sqrt(Xaxis * Xaxis + Yaxis * Yaxis);

           if (distBetween < 200)
                      return false;
           else
                      return true;

}

void CombineLocalMaps(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
{
           MyRobot robot(0, 0);
           vector<Object> temp;
           //combine a new local map into global map                           

           double x1, y1, x2, y2;
           angle = ((angle / 180) * PI);

           for (int i = 0; i<int(addView.size()); i++)
           {
                      x1 = addView[i].X1() * cos(angle) - addView[i].Y1() * sin(angle) + coord.X();
                      y1 = addView[i].X1() * sin(angle) + addView[i].Y1() * cos(angle) + coord.Y();

                      x2 = addView[i].X2() * cos(angle) - addView[i].Y2() * sin(angle) + coord.X();
                      y2 = addView[i].X2() * sin(angle) + addView[i].Y2() * cos(angle) + coord.Y();

                      Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());
                      s.setKP(1);
                      temp.push_back(s);
           }

           //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/test-", num, ".png");            
           //plotObjects(LocalMFileName, currentPos, temp);

           currentTransLM = temp;
           Memories.push_back(currentTransLM); // storing all transformed local maps;

           MapofLMs = addTwoVectorsOfObjects(MapofLMs, temp);
           sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", num, ".png");
           plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());
}

void TransforIntoGloabl(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
{
           MyRobot robot(0, 0);
           vector<Object> temp;
           //combine a new local map into global map                           

           double x1, y1, x2, y2;
           angle = ((angle / 180) * PI);

           for (int i = 0; i<int(addView.size()); i++)
           {
                      x1 = addView[i].X1() * cos(angle) - addView[i].Y1() * sin(angle) + coord.X();
                      y1 = addView[i].X1() * sin(angle) + addView[i].Y1() * cos(angle) + coord.Y();

                      x2 = addView[i].X2() * cos(angle) - addView[i].Y2() * sin(angle) + coord.X();
                      y2 = addView[i].X2() * sin(angle) + addView[i].Y2() * cos(angle) + coord.Y();

                      Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());
                      s.setKP(1);
                      temp.push_back(s);
           }

           //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/test-", num, ".png");            
           //plotObjects(LocalMFileName, currentPos, temp);

           currentTransLM = temp;
           Memories.push_back(currentTransLM); // storing all transformed local maps;

           MapofLMs = addTwoVectorsOfObjects(MapofLMs, temp);
           //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", num, ".png");            
           //plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());
}

bool TwopathSegment(vector<Point> AllPositions)
{
            Point p1, p2, p3, p4;
            double line_x, line_y; //intersect position  
            int num_pos = 0;
            num_pos = AllPositions.size();

            p3 = AllPositions[num_pos - 2];
            p4 = AllPositions[num_pos - 1];

            if (AllPositions.size() > 3)
            {
                for (int i = 1; i < (AllPositions.size() - 2); i++)
                {
                    p1 = AllPositions[i - 1];
                    p2 = AllPositions[i];


                    if ((fabs(p1.X() - p2.X()) < 1e-6) && (fabs(p3.X() - p4.X()) < 1e-6))
                    {
                        //return false;  
                        pathCrossed_Flag = 0;
                    } else if ((fabs(p1.X() - p2.X()) < 1e-6))
                    {
                        if (between(p1.X(), p3.X(), p4.X()))
                        {
                            double k = (p4.Y() - p3.Y()) / (p4.X() - p3.X());
                            line_x = p1.X();
                            line_y = k * (line_x - p3.X()) + p3.Y();

                            if (between(line_y, p1.Y(), p2.Y()))
                            {
                                pathCrossed_Flag = 1;
                                CrossedP1 = i - 1;
                                return true;
                            } else
                            {
                                //return false;  
                                pathCrossed_Flag = 0;
                            }
                        } else
                        {
                            //return false;
                            pathCrossed_Flag = 0;
                        }
                    } else if ((fabs(p3.X() - p4.X()) < 1e-6))
                    {
                        if (between(p3.X(), p1.X(), p2.X()))
                        {
                            double k = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
                            line_x = p3.X();
                            line_y = k * (line_x - p2.X()) + p2.Y();

                            if (between(line_y, p3.Y(), p4.Y()))
                            {
                                pathCrossed_Flag = 1;
                                CrossedP1 = i - 1;
                                return true;
                            } else
                            {
                                //return false;
                                pathCrossed_Flag = 0;
                            }
                        } else
                        {
                            //return false;  
                            pathCrossed_Flag = 0;
                        }
                    } else
                    {
                        double k1 = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
                        double k2 = (p4.Y() - p3.Y()) / (p4.X() - p3.X());

                        if (fabs(k1 - k2) < 1e-6)
                        {
                            //return false;
                            pathCrossed_Flag = 0;
                        } else
                        {
                            line_x = ((p3.Y() - p1.Y()) - (k2 * p3.X() - k1 * p1.X())) / (k1 - k2);
                            line_y = k1 * (line_x - p1.X()) + p1.Y();
                        }

                        if (between(line_x, p1.X(), p2.X()) && between(line_x, p3.X(), p4.X()))
                        {
                            pathCrossed_Flag = 1;
                            CrossedP1 = i - 1;
                            return true;
                        } else
                        {
                            //return false;  
                            pathCrossed_Flag = 0;
                        }
                    }

                }
            }
            if (pathCrossed_Flag == 0)
                return false;

}


//given two slopes of two adjacent paths , tanA=|k1-k2|/|1+k1*k2|
// cos law a^2 = b^2 + c^2 -2*b*c*CosA   cosA=(b^2+c^2-a^2)/2bc

bool TwoPathAngle(vector<Point> AllPositions)
{
    cout << "*****This is going to calculate angle between two adjacent paths*****" << endl << endl;
    double k1, k2;
    double angleTwoPath;
    double a, b, c;
    double cosfi = 0, fi = 0, norm = 0;

    Point p1, p2, p3;
    int num = AllPositions.size();
    if (num >= 3)
    {
        //last three points
        p3 = AllPositions[num - 1];
        p2 = AllPositions[num - 2];
        p1 = AllPositions[num - 3];

        b = sqrt((p1.X() - p2.X()) * (p1.X() - p2.X()) + (p1.Y() - p2.Y()) * (p1.Y() - p2.Y())); // b

        c = sqrt((p3.X() - p2.X()) * (p3.X() - p2.X()) + (p3.Y() - p2.Y()) * (p3.Y() - p2.Y())); // c

        a = sqrt((p3.X() - p1.X()) * (p3.X() - p1.X()) + (p3.Y() - p1.Y()) * (p3.Y() - p1.Y())); // a


        //k1 = (p2.Y()-p1.Y()) / (p2.X() - p1.X());
        //k2 = (p2.Y()-p3.Y()) / (p2.X() - p3.X());

        //angleTwoPath = atan((abs(k1 - k2)) / (abs(1+ k1 * k2)));

        //cosfi = dsx * dex + dsy * dey;
        //norm = (dsx * dsx + dsy * dsy) * (dex * dex + dey * dey);
        //cosfi /= sqrt(norm);

        //if (cosfi >= 1.0) return 0;
        //if (cosfi <= -1.0) return PI;

        cosfi = (b * b + c * c - a * a) / (2 * b * c);

        fi = acos(cosfi);


        if (180 * fi / PI < 180)
        {
            angleTwoPath = 180 * fi / PI;
        } else
        {
            angleTwoPath = 360 - 180 * fi / PI;
        }

        cout << "This is for testing angle calculation" << endl << endl;
        cout << "The parameter angleTwoPath/fi is : " << angleTwoPath << endl << endl;
        //waitHere();
    }



    if (angleTwoPath < 45 && angleTwoPath != 0)
        return true;
    else
        return false;
}

//**************************************************//
// Transform current position onto old coordinate system

Point TransPointToOldCrd(double Transf_angle, Point Transf_point, Point coord)
{
           cout << "This is going to transform current robot position onto pre coordinate" << endl << endl;

           double angle = 0;
           double x, y;
           Point point;
           Point temp;


           angle = Transf_angle;
           point = Transf_point;

           angle = ((angle / 180) * PI);


           x = point.X() * cos(angle) - point.Y() * sin(angle) + coord.X();
           y = point.X() * sin(angle) + point.Y() * cos(angle) + coord.Y();

           temp.set(x, y);

           return temp;
}

void TwoPathsCrossed(int num)
{
           cout << "This is going to process two path segments" << endl;
           //cout << "Path Crossed Flag : " << (int) pathCrossed_Flag << endl;
           //cout << "which step is crossed : " << CrossedP1 << endl;
           //cout << "The number of list : " << ListOfPositions.size() << endl;
           //waitHere();

           // delete useless positions
           robotPositionsInLMs.clear();
           for (int i = RobotPs.size() - 1; i > CrossedP1; i--)
                RobotPs.pop_back();
           RobotPs.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

           for (int i = updatingNumber.size() - 1; i > CrossedP1; i--)
                updatingNumber.pop_back();
           updatingNumber.push_back(num);

           for (int i = facingOrient.size() - 1; i > CrossedP1; i--)
                facingOrient.pop_back();
           facingOrient.push_back(robotFacing);

            //rb = TransPointToOldCrd(PrexAxis, rb, Prerb);
           for (int i = rbs.size() - 1; i > CrossedP1; i--)
                rbs.pop_back();

           rbs.push_back(rb);

           // delete useless positions, which are used to present on map
           for (int i = ListOfPositions.size() - 1; i > CrossedP1; i--)
                ListOfPositions.pop_back();
           ListOfPositions.push_back(currentRobotPositionInMFIS);

           // re-build robot positions
           for (int i = 0; i < ListOfPositions.size(); i++)
                robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, ListOfPositions[i]);

           // this part is to delete ASRs, which are useless
           for (int i = Memories.size() - 1; i > CrossedP1; i--)
                Memories.pop_back();
           Memories.push_back(currentTransLM);

           // re-build global map using local maps
           MapofLMs.clear(); // clear Global map and re-compute it
           for (int i = 0; i < Memories.size(); i++)
                MapofLMs = addTwoVectorsOfObjects(MapofLMs, Memories[i]);


           // re-plot this current local map
           sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", num, ".png");
           plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());
}


//********************************************************************************//
// Test Programme for transforming positions onto pre coordinate

vector<Object> TransPositions(vector<Object> positions, double TransAngle, Point coord)
{
           MyRobot robot(0, 0);
           vector<Object> temp;
           Object result;
           //combine a new local map into global map                           

           double x1, y1, x2, y2;
           double angle = TransAngle;

           angle = ((angle / 180) * PI);

           for (int i = 0; i<int(positions.size()); i++)
           {

                      x1 = positions[i].X1() * cos(angle) - positions[i].Y1() * sin(angle) + coord.X();
                      y1 = positions[i].X1() * sin(angle) + positions[i].Y1() * cos(angle) + coord.Y();

                      x2 = positions[i].X2() * cos(angle) - positions[i].Y2() * sin(angle) + coord.X();
                      y2 = positions[i].X2() * sin(angle) + positions[i].Y2() * cos(angle) + coord.Y();

                      Object result(x1, y1, x2, y2, 1, 0, 0, 0, 1);
                      temp.push_back(result);
           }

           return temp;
}

/*
void test(vector< vector<Object> > LMS)
{
    char testFileName[80];
    vector< vector<Object> > chunk1;
    vector< vector<Object> > chunk2;
    vector< vector<Object> > chunk3;
    for(int i = 0; i < 5; i++)
    {
        chunk1.push_back(LMS[i]);
    }
    sprintf(testFileName, "%s%d%s", "Maps/Offline/ChunksWithCL-", 1, ".png");
    plotObjectsColours(testFileName, chunk1);
    
    for(int i = 5; i < 9; i++)
    {
    chunk2.push_back(LMS[i]);
    }
    sprintf(testFileName, "%s%d%s", "Maps/Offline/ChunksWithCL-", 2, ".png");
    plotObjectsColours(testFileName, chunk2);
    for(int i = 9; i < LMS.size(); i++)
    {
        chunk3.push_back(LMS[i]);
    }
    sprintf(testFileName, "%s%d%s", "Maps/Offline/ChunksWithCL-", 3, ".png");
    plotObjectsColours(testFileName, chunk3);
 * 
 * 
 * 
 * 
 * ********************************************************************
 *  vector<Object> chunk;
    vector<Object> modiLM;
    vector< vector<Object> > chunkWithLMs;
    vector< vector<Object> > allrps;
    chunk = LMS.getGlobalMaps()[0];
    
    modiLM = fillGapOfView(LMS.getGlobalMaps()[0]);
    chunkWithLMs.push_back(modiLM);
   
    for(int i = 0 ; i < 48; i++)
    allrps.push_back(allRobotPositionsObject[i]);
    //cout << "path1 done!!" << endl;
    for(int i = 1; i < 5; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
        modiLM = fillGapOfView(LMS.getGlobalMaps()[i]);
        chunkWithLMs.push_back(modiLM);
        
    }
    cout << "boundary1 done!!" << endl;
    SketchBoundary(chunk, allrps, chunkWithLMs);
    
    waitHere();
    chunk.clear();
    allrps.clear();
    modiLM.clear();
    chunkWithLMs.clear();
    chunk = LMS.getGlobalMaps()[5];
    modiLM = fillGapOfView(LMS.getGlobalMaps()[5]);
    chunkWithLMs.push_back(modiLM);
    for(int i = 48 ; i < 73; i++)
        allrps.push_back(allRobotPositionsObject[i]);
     cout << "path2 done!!" << endl;
     
    for(int i = 6; i < 9; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
                modiLM = fillGapOfView(LMS.getGlobalMaps()[i]);
        chunkWithLMs.push_back(modiLM);
    }
    cout << "boundary2 done!!" << endl;
    SketchBoundary(chunk, allrps, chunkWithLMs);
    waitHere();
      
    chunk.clear();
    allrps.clear();
    modiLM.clear();
    chunkWithLMs.clear();
     chunk = LMS.getGlobalMaps()[9];
         modiLM = fillGapOfView(LMS.getGlobalMaps()[9]);
    chunkWithLMs.push_back(modiLM);
    for(int i = 80 ; i < 105; i++) //73; or 81
        allrps.push_back(allRobotPositionsObject[i]);
     cout << "path3 done!!" << endl;
    for(int i = 10; i < 13; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
                modiLM = fillGapOfView(LMS.getGlobalMaps()[i]);
        chunkWithLMs.push_back(modiLM);
    }
     cout << "boundary3 done!!" << endl;
    SketchBoundary(chunk, allrps, chunkWithLMs);
    
    ***************************************************************
    
    
}
*/


void TestforAllviews(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
{
           MyRobot robot(0, 0);
           vector<Object> temp;
           //combine a new local map into global map                           

           double x1, y1, x2, y2;
           angle = ((angle / 180) * PI);

           for (int i = 0; i<int(addView.size()); i++)
           {
                      x1 = addView[i].X1() * cos(angle) - addView[i].Y1() * sin(angle) + coord.X();
                      y1 = addView[i].X1() * sin(angle) + addView[i].Y1() * cos(angle) + coord.Y();

                      x2 = addView[i].X2() * cos(angle) - addView[i].Y2() * sin(angle) + coord.X();
                      y2 = addView[i].X2() * sin(angle) + addView[i].Y2() * cos(angle) + coord.Y();

                      Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());
                      s.setKP(1);
                      temp.push_back(s);
           }

           currentTransLM = temp;
           allLEs.push_back(temp);
           //cout<<" now the number is : " << num << endl;
           //waitHere();
}



