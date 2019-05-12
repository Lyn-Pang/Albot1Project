

//============================================================================
// Name        : Path Simplification and Chunks 
// Author      : Wenwang (Lyn)
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

#include "GeometryAndExit.h"
#include "GeometryFuncs.H"
#include "GeometricOp.H"
#include "ToolFunctions.h"
#include "thesis.H"
#include "ChunksOp.H"
#include "ConvexHull.h"
#include "Exit.h"
#include "Expectation.h"

#include "ChunkInfor.h"

#include <cstdlib>
#include <ctime>


#define PI 3.14159265
using namespace std;

//***********************************************************************************//

void TransforIntoGloabl(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num); // only transform
vector<Object> TransformforAllviews(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num);


void LocalMapAndChunk(vector< vector<Object> > robotPosition, vector<Point> rps,
                      vector< vector<Object> > views, vector<Object> copyMFIS, 
                      vector<int> id_group, int startNum, int endNum, 
                      int chunk_cnter, unsigned char lastloop_Flag);

vector<Object> chunkMap(vector<Object> copyMFIS, int startNum, int endNum, unsigned char lastloop_Flag);
void LocalMapAndRobot(vector<Object> chunk_map, vector<Point> rps,
                      vector< vector<Object> > robotPosition, vector<int> id_group,
                      vector< vector<Object> > views);
void LocalMapAndRobotUsingView(vector<Object> chunk_map, vector<Point> rps,
                      vector< vector<Object> > robotPosition, vector<int> id_group,
                      vector< vector<Object> > views, int chunk_cnter);
void LocalMapAndRobotUsingCircle(vector<Object> chunk_map, vector<Point> rps,
                      vector< vector<Object> > robotPosition, vector<int> id_group,
                      vector< vector<Object> > views);
vector<Object> ConstructChunk(vector< vector<Object> > LMs);
vector<Object> ConstructPath(vector<Point> rps);

/*************************************************************************************/

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
double robotTurning = 0; //in degree relating to global
vector<double> facingOrient;


vector< vector<Object> > allLEs; // all local environments 
vector< vector<Object> > Memories; // all local maps as memories
vector< vector<Object> > ListOfPositions; // for storing currentCRPInMFIS variable 


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
//unsigned char expect_Flag = 0;     //expectation process flag

int CrossedP1 = 0; // when two paths are crossed, return the old segment point1
int CountOfOutASR = 0;
int intersection_cnt = 0; //local map process, intersection counter

vector<int> updatingNumber;
vector<int> intersection_positions; //all intersection positions

MyRobot robot(0, 0);

vector<Object> allASR;

//---chunk process variable---//
vector<Object> LMprocess_GlobalMap; //the global map for processing local map
vector< vector<Object> > robot_Positions; //global position of robot 
vector<Point> robot_points;               //global point of robot position
vector< vector<Object> > views;           //transformed to uniform coordinate
vector<int> id_group;                     //view ID of a group
vector<Object> Global_ASR;

vector<Object> last_step_MFIS;            //for compute chunk map

vector<Object> last_Chunk_Map;            //last chunk map

int last_Chunk_step;                      //the step ID computing last chunk
int befor_start_number;                   //
//----------------------------//

//--- Expectation process vari---//
vector<Object> fixed_mfis; //after projection, the mfis becomes fixed

vector<ChunkInfo> all_chunk_info;
//counters for paper
int chunk_cnt = 0;  //chunk counter
int vt_counter = 0; //visual triangulation
int pi_counter = 0; //path integration

int main()
{
    int v, w, r, level, set, saveFrom, saveTo;
    int start_Number = 0;
    
    
    cout << "Which level?? ";
    cin >> level;
    cout << "Which dataset?? ";
    cin >> set;
    cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
    cin >> v;
    cin >> w;

    data_level = level; //chunk info process
    data_set = set;     //chunk info process
    
    updatingNumber.push_back(1);
    rbs.push_back(Point(0, 0));
    facingOrient.push_back(0);
    int referenceNumberTH = 0;
    char mappingInfoFileName[100];

     
    sprintf(mappingInfoFileName, "%s%d%s", "Maps/Offline/MappingInfo-refTh-", referenceNumberTH, ".txt");
    //variables
    Transporter recognizedTargetObjects, computedOutput, computedLocalMapOutput;
    vector <double> coordTransInfo;
    vector<Object> targetObjectsInMPV;
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
 
    ListOfPositions.push_back(currentRobotPositionInMFIS); // push the first position into a list

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
    allLEs.push_back(currentView);

    if (currentView.size() == 0)
    {
        //cout << "Need to change the file name" << endl;
        surfaceName = "/surface-";
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        currentView = readASCII(viewFileName);
    }

    //finding target objects
    targetObjectsInMPV = targetObjectsInPV = findTargetObjects(currentView);
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

    //currentPG = makePolygonOfCV(NewLM); // first local to generate polygon
    Memories.push_back(NewLM); //store the first local map as first memory
    currentTransLM = NewLM; // the first one does not to transform

    exitsFromcurrentLM = findGapasExits(NewLM); // initial exits in first view/local environment

    sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
    plotObjects(viewFileName, myrobot.getRobot(), NewLM);
    
    //--- Local map and chunk process ---//
    //ALLlocalMap = NewLM;
 
    //initialization of local maps
    //LMS = TransViewIntoGloabl(ALLlocalMap, NewLM, Point (0,0), myrobot.getRobot(), 0, 1);
    Global_ASR = currentView;
    //robpositions.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));
    robot_Positions.push_back(currentRobotPositionInMFIS);
    robot_points.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));              
    views.push_back(Global_ASR);           
    id_group.push_back(v);  
    
    //-----------------------------------//

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
    
    sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");
    plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV, recognizedTargetObjects.getTargetObjects());

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
        robotFacing = robotFacing + coordTransInfo[1];           //The second element.
        
        robotTurning += coordTransInfo[1]; 
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
        
       
        //odometricErrorMap(errorMap, currentView, coordTransInfo[0], coordTransInfo[1]); //computing error map
        //finding Exits
        exitsFromCV = findShortestExits(currentView);

        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);

        LandMarkCV = findTargetObjects(currentView);

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);

        sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");
        plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV, recognizedTargetObjects.getTargetObjects());
        //plotObjects(viewFileName, myrobot.getRobot(), currentView);
 

        currentView = tagObjectsAsSideAndViewNumber(currentView, v);
        referenceObjects = recognizedTargetObjects.getReferenceObjects();

        //
        targetObjectsInMPV = targetObjectsInPV;
        
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
       
            
                /*===========================================================================================*/
                //routeMap
                pathFromLastLimitingPoint.set(routeMapConnLP.back().X2(), routeMapConnLP.back().Y2(),
                        currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), v);
                routeMapConnLP.push_back(pathFromLastLimitingPoint);

                robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);

                updatingPoints.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                //local map process variables
                xAxisAtCRP = currentRobotPositionInMFIS[7];

                Global_ASR = TransformforAllviews(currentView, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), 
                                                  currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);


                //old approach
                //computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                //      allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);

                //modified by Lyn //new approach

                //computedOutput = updatePerceptualMapATPlaceDeletion(places, MFIS, currentView, currentRobotPositionInMFIS,
                //      allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);

                    computedOutput = updatePerceptualMapATPlaceDeletion_version2(places, MFIS, currentView, currentRobotPositionInMFIS,
                         allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, expect_Flag);
 

                //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
                //computedLocalMapOutput = updatePerceptualMapATPlace(places, LocalMap, currentView, currentRobotPositionInMFIS,
                //        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);

                MFIS = computedOutput.getView();

                /*//project backward 
                        vector<Object> project_ref;
                        double max_leng = 0; 
                        if(abs(coordTransInfo[1]) >= 10 && del_ref_list.size() > 0)
                        {
                            for(int m = 0; m < del_ref_list.size(); m++)
                            {
                                if(m == 0)
                                {
                                    project_ref = del_ref_list[m];
                                    max_leng = del_ref_list[m][0].length();
                                }
                                else
                                {
                                    if(del_ref_list[m][0].length()>max_leng)
                                    {
                                        project_ref = del_ref_list[m];
                                        max_leng = del_ref_list[m][0].length();
                                    }
                                }
                            }
                            MFIS = Project_backward_MFIS(MFIS, lastStepInfo.getMFIS(), project_ref);
                            expect_Flag = 1;
                        }
                        del_ref_list.clear();*/

                //LocalMap = computedLocalMapOutput.getView();
                places = computedOutput.getASRs();
                targetObjectsInPV = computedOutput.getTargetObjects();
                //limitingPoints.push_back(v); //limiting/updating points just for printing at the end


                /*//local map process variables
                xAxisAtCRP = currentRobotPositionInMFIS[7];

                Global_ASR = TransformforAllviews(currentView, Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), 
                                                  currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
*/
                //robpositions.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));
                robot_Positions.push_back(currentRobotPositionInMFIS);
                robot_points.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));              
                views.push_back(Global_ASR);           
                id_group.push_back(v);     

                //added for chunk//   
                //chunks = updateGlobalMapForChunks(places, ChunkGlobal, currentView, currentRobotPositionInMFIS,
                //        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, LMS, lastloop_Flag);

                //chunks = updateGlobalMapForChunksWithFlag(places, ChunkGlobal, currentView, currentRobotPositionInMFIS,
                //        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, LMS, lastloop_Flag);

                chunks = updateGlobalMapForChunksNewDeletion(places, ChunkGlobal, currentView, currentRobotPositionInMFIS,
                        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, LMS, lastloop_Flag);


                ChunkGlobal = chunks.getView();

                //chunk process happens flag
                if(chunks.getChunkFlag() == 1)
                {
                        chunk_cnt++;

                        //allChunks.push_back(chunks.getChunk());
                        //LMprocess_GlobalMap = MFIS;          //full MFIS
                        LMprocess_GlobalMap = last_step_MFIS;
                        //LMprocess_GlobalMap = ChunkGlobal; //MFIS without deletion part

                        LocalMapAndChunk(robot_Positions, robot_points, views, LMprocess_GlobalMap, 
                              id_group, start_Number, v, chunk_cnt, lastloop_Flag);                   

                        start_Number = v;
                        robot_Positions.clear();
                        robot_points.clear();
                        views.clear();
                        id_group.clear();


                        all_chunk_info.push_back(chunk_infomation);

                        //project backward 
                        vector<Object> project_ref;
                        double max_leng = 0; 

                        for(int m = 0; m < del_ref_list.size(); m++)
                        {
                            if(m == 0)
                            {
                                project_ref = del_ref_list[m];
                                max_leng = del_ref_list[m][0].length();
                            }
                            else
                            {
                                if(del_ref_list[m][0].length()>max_leng)
                                {
                                    project_ref = del_ref_list[m];
                                    max_leng = del_ref_list[m][0].length();
                                }
                            }
                        }
                        MFIS = Project_backward_MFIS(MFIS, lastStepInfo.getMFIS(), project_ref);
                        expect_Flag = 1;
                        fixed_mfis = MFIS;
                        del_ref_list.clear();

                        //connect adjacent chunk process//
                        //if(all_chunk_info.size() > 1)
                        //{
                        //    vector<Exit> transfered_exits;
                        //    vector<Object> transfered_chunk;
                        //    pair<double, Point> chunk_connect_parameters;
                        //    chunk_connect_parameters = connect_parameters(all_chunk_info.back(), all_chunk_info[all_chunk_info.size()-2]);

                            //transform pre-one to current coordiante
                            //transfered_chunk = TransformforToGlobalCoordinate(all_chunk_info[all_chunk_info.size()-2].getMFIS(), chunk_connect_parameters.second,
                            //                        all_chunk_info[all_chunk_info.size()-2].getRobots().back(), chunk_connect_parameters.first);

                            //transfered_chunk = TransformforToGlobalCoordinate(all_chunk_info[all_chunk_info.size()-2].getMFIS_region().back(), chunk_connect_parameters.second,
                            //                        all_chunk_info[all_chunk_info.size()-2].getRobots().back(), chunk_connect_parameters.first);

                            /*
                            int size_temp;
                            vector<Object> test_region;
                            size_temp = all_chunk_info[all_chunk_info.size()-2].getMFIS_region().size();
                            test_region = all_chunk_info[all_chunk_info.size()-2].getMFIS_region()[size_temp-1];
                            test_region = addTwoVectorsOfObjects(test_region, all_chunk_info[all_chunk_info.size()-2].getMFIS_region()[size_temp-2]);
                            test_region = addTwoVectorsOfObjects(test_region, all_chunk_info[all_chunk_info.size()-2].getMFIS_region()[size_temp-3]);
                            test_region = addTwoVectorsOfObjects(test_region, all_chunk_info[all_chunk_info.size()-2].getMFIS_region()[size_temp-4]);
                            transfered_chunk = TransformforToGlobalCoordinate(test_region, chunk_connect_parameters.second,
                                                    all_chunk_info[all_chunk_info.size()-2].getRobots().back(), chunk_connect_parameters.first);

                            transfered_exits = Transform_Exit_GlobalCoordinate(all_chunk_info[all_chunk_info.size()-2].getExits(), chunk_connect_parameters.second, 
                                                    all_chunk_info[all_chunk_info.size()-2].getRobots().back(), chunk_connect_parameters.first);

                            be_project_region(all_chunk_info[all_chunk_info.size()-1].getMFIS(), transfered_chunk,
                                     all_chunk_info[all_chunk_info.size()-1].getExits(), transfered_exits);
                            */
                            /*
                            //shift previous chunk informaiton/exits
                            vector<Exit> shift_exits = _correct_exit_info(transfered_exits, all_chunk_info.back().getMFIS_region()[0], transfered_chunk);

                            //store those information
                            if(shift_exits.size() > 0)
                                all_chunk_info.back().setShiftExits(shift_exits);


                            Exit ref_exit;
                            vector<Exit> project_exits;

                            if(all_chunk_info[all_chunk_info.size()-2].getShiftExits().size() > 0)
                            { 

                                for(int m = 0; m < all_chunk_info[all_chunk_info.size()-2].getExits().size(); m++)
                                {
                                    if(abs(all_chunk_info[all_chunk_info.size()-2].getExits()[m].length() - shift_exits[0].length()) < 10e-6)
                                        ref_exit = all_chunk_info[all_chunk_info.size()-2].getExits()[m];
                                }


                                project_exits = projectingTheExit(all_chunk_info[all_chunk_info.size()-2].getShiftExits(), shift_exits[0], ref_exit, 1);
                                project_exits = addTwoExits(project_exits, shift_exits);
                                sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Projecting_key_exits-", v, ".png");                     
                                plotObjectsOf3KindswithExits(mfisFileName, all_chunk_info.back().getMFIS_region()[0], all_chunk_info.back().getMFIS_region()[0], project_exits);
                            }*/
                        //}
                }

                if(expect_Flag == 1)
                {
                    //remove 
                    AllGlobalRobotPs.pop_back();
                    allRobotPositions = deleOneVectorOfObject(allRobotPositions, currentRobotPositionInMFIS);
                    
                    //update
                    currentRobotPositionInMFIS = match_view_with_MFIS(Global_ASR, fixed_mfis, currentRobotPositionInMFIS, MFIS, v);
                    AllGlobalRobotPs.push_back(currentRobotPositionInMFIS);
                    allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS); 

                    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Fixed_MFIS-", v, ".png");                     
                    plotObjectsOf3Kinds(mfisFileName, allRobotPositions, currentRobotPositionInMFIS, fixed_mfis);
                    
                }

                limitingPoints.push_back(v); //limiting/updating points just for printing at the end

                //to print local and global map
                sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
                //plotpathandobject(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS, updatingPoints);                      
                plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS);

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
        
        //firstloop_Flag = 1;
        
        //double cope current step MFIS for chunking process
        last_step_MFIS = MFIS;
        
    
    } while (v < w); 

    cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

    outFile.close();
    outFileForRefError.close();
    //localization 
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

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

    //cout << "Traveled Dist: " << traveledDistance << endl;
    writeASCII(convertObjectToSurface(MFIS), "Maps/Offline/pm"); //for thesis
    
    //plotObjects("Maps/errorMap.png", errorMap, errorMap); //for thesis
    perceptualMap.setMFIS(MFIS);


    abstractRouteMap(MFIS, robotPositionsAtLimitingPoints, updatingPoints, currentRobotPositionInMFIS);
    keyInfoOfCompuetedPM(mappingInfoFileName, ASRNumber, exitPoints, lostPoints, limitingPoints,
            badLocalization, failedToRecognizeRef, referenceNumberTH, level, set);
    //cout << levelName << level << " set " << set << endl;
    //cout << "odo " << odoLocalizationUsed << endl;
    cout << endl << " Visual Triangulation updating: " << vt_counter << " times. Path Integration updating: " << pi_counter << " times." << endl;
    cout << endl << " Traveled distance: " << traveledDistance << endl;
    

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

           currentTransLM = temp;
           Memories.push_back(currentTransLM); // storing all transformed local maps;

           MapofLMs = addTwoVectorsOfObjects(MapofLMs, temp);

}


vector<Object> TransformforAllviews(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
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
                      s.setASRNo(addView[i].getASRNo());
                      s.setChunkFLag(addView[i].getChunkFlag());
                      s.setColorID(addView[i].getColorID());
                      //s.setGID(addView[i].getGID());
                      //s.setID(addView[i].getID());
                      s.setKP(addView[i].getKP());
                      s.setLimitingPoint(addView[i].getLimitingPoint());
                      s.setLocalEnvID(addView[i].getLocalEnvID());
                      //s.setLocalEnvID();
                      //s.setNS();
                      s.setOoPV(addView[i].getOoPV());
                      s.setOrt(addView[i].getOrt());
                      s.setP1OS(addView[i].getP1OS());
                      s.setP2OS(addView[i].getP2OS());
                      s.setPEP1(addView[i].getPEP1());
                      s.setPEP2(addView[i].getPEP2());
                      s.setPO(addView[i].getPO());
                      s.setPos(addView[i].getPos());
                      s.setVN(addView[i].getVN());
                      
                      
                      temp.push_back(s);
           }

           //currentTransLM = temp;
           //allLEs.push_back(temp);
           return temp;
}


void LocalMapAndChunk(vector< vector<Object> > robotPosition, vector<Point> rps,
                      vector< vector<Object> > views, vector<Object> copyMFIS, 
                      vector<int> id_group, int startNum, int endNum, 
                      int chunk_cnter, unsigned char lastloop_Flag)
{
    cout << " Generating chunks using local map/big views " << endl << endl;
    vector<Object> chunk_map;       //corresponding chunk map
    char plotFileName[80];
    
    //generate corresponding chunk map 
    chunk_map = chunkMap(copyMFIS, startNum, endNum, lastloop_Flag);
    
    last_Chunk_step = endNum;
    last_Chunk_Map = chunk_map;
    
    sprintf(plotFileName, "%s%d%s", "Maps/Offline/TestChunkMap-", endNum,".png");
    plotObjects(plotFileName, chunk_map);
    
    //chunk information process inside 
    if((endNum - startNum > 1) && (lastloop_Flag != 1)) //for in case, these are last two steps of dataset
        LocalMapAndRobotUsingView(chunk_map, rps, robotPosition, id_group, views, chunk_cnter);
    
    
    //vector<Point> chunk_points =  ObjectToPoints(chunk_map);
    //sprintf(plotFileName, "%s%d%s", "Maps/Offline/TestChunkMap_AllPoints-", endNum,".png");
    //plotPoints(plotFileName, chunk_points);   
    
    //local map, intersection process
    //LocalMapAndRobot(chunk_map, rps, robotPosition, id_group, views);
    //LocalMapAndRobotUsingCircle(chunk_map, rps, robotPosition, id_group, views);
    //constrainPointsToEllipse(robotPosition, chunk_map);
    //constructEllipseOfASR(robotPosition, views,  chunk_map, chunk_map);
    //PolyBasedASR(views, chunk_map);
}


vector<Object> chunkMap(vector<Object> copyMFIS, int startNum, int endNum, unsigned char lastloop_Flag)
{
    //cout << " Compute a chunk map " << endl;  
    cout << " start number is : " << startNum << endl;
    cout << " end number is : " << endNum << endl;
        vector<Object> local_GlobalMap; //for processing local maps
        
        //copy the part of MFIS for local map process 
        for(int i = 0; i < copyMFIS.size(); i++)
        {
             
                //last step include the end
                if(int(lastloop_Flag) == 1)
                {
                    if((copyMFIS[i].getVN() >= startNum) 
                          && (copyMFIS[i].getVN() <= endNum))
                    {
                         local_GlobalMap.push_back(copyMFIS[i]); //local global map 
                    }
                }
                else
                {
                    
                    if((copyMFIS[i].getVN() >= startNum) 
                          && (copyMFIS[i].getVN() < endNum))
                    {
                        //cout << " the current i : "<< i << endl;
                        local_GlobalMap.push_back(copyMFIS[i]); //local global map 
                    }
                }
        }
         
        return local_GlobalMap;
}

/* using MFIS to generate local maps
 * and corresponding robot positions
 */
void LocalMapAndRobot(vector<Object> chunk_map, vector<Point> rps,
                      vector< vector<Object> > robotPosition, vector<int> id_group,
                      vector< vector<Object> > views)
{
        vector< vector<Object> > robot_group;     //local map robot objects
        vector< vector<Object> > LMs;   //all individual local maps of the chunk
        vector<Point> rbs;    //all robot points of the chunk
        vector<int> LMs_chunk;          //local maps ID of the chunk
        int current_LM = 0;

        rbs.push_back(rps[current_LM]);
        LMs.push_back(views[current_LM]);
        LMs_chunk.push_back(id_group[current_LM]);
        
        //cout << " the id group is : ";
        //for(int i = 0; i < id_group.size(); i++)
        //{
        //    cout << id_group[i] << " "; 
        //}
        //local map process, detecting the intersection 

lb:     for(int i = current_LM + 1; i < rps.size(); i++)
        {
                if(interSectWithLine(chunk_map, rps[current_LM], rps[i]) == true)
                {
                    cout << " the current id : " << id_group[i] << endl << endl;
                    
                    cout << " the ref robot position , x : " << rps[current_LM].X()
                            << " ; y : " << rps[current_LM].Y() << endl << endl;
                    cout << " the current robot position , x : " << rps[i].X()
                            << " ; y : " << rps[i].Y() << endl << endl;
                    
                        //record the current position & become start for sequence
                        current_LM = i;
                        robot_group.push_back(robotPosition[i]);
                        
                        rbs.push_back(rps[i-1]);
                        rbs.push_back(rps[i]);
                        LMs.push_back(views[i]);
                        
                        //record local map ID
                        LMs_chunk.push_back(id_group[i]);
                        /*
                        if(rps.size() - id_group[i] <= 15 ) //add the last view
                        {
                            robot_group.push_back(robotPosition.back());
                            rbs.push_back(rps.back());
                            LMs.push_back(views.back());
                            LMs_chunk.push_back(id_group.back());
                            break;
                        }
                        */
                        goto lb;
                }
                else
                {
                    //the last step, save the robot position but the last view
                    if(i == rps.size() - 1)
                    {
                        rbs.push_back(rps[i]);
                    }
                }

        }
        /*
        cout << " the size of LMs id : " << LMs_chunk.size() << endl<<endl;
        cout << " the ids : ";
        for(int i = 0; i < LMs_chunk.size(); i++)
        {
            cout << LMs_chunk[i] << " "; 
        }
        waitHere();
        */
        //construct chunk using local maps 
        vector<Object> chunk_LMs = ConstructChunk(LMs);
        
        //construct paths for chunk
        vector<Object> path_LMs = ConstructPath(rbs);
        
        char plotFileName[80];
        sprintf(plotFileName, "%s", "Maps/Offline/TestChunkWithPath.png");
        plotObjects(plotFileName, chunk_LMs, path_LMs);
        
        //waitHere();
}

/* using MFIS to generate local maps
 * and corresponding robot positions
 */
void LocalMapAndRobotUsingView(vector<Object> chunk_map, vector<Point> rps,
                      vector< vector<Object> > robotPosition, vector<int> id_group,
                      vector< vector<Object> > views, int chunk_cnter)
{
        
        vector< vector<Object> > robot_group;     //local map robot objects
        vector< vector<Object> > LMs;   //all individual local maps of the chunk
        vector<Point> rbs;    //all robot points of the chunk
        vector<int> LMs_chunk;          //local maps ID of the chunk
        int current_LM = 0;

        rbs.push_back(rps[current_LM]);
        LMs.push_back(views[current_LM]);
        LMs_chunk.push_back(id_group[current_LM]);
    
lb:     for(int i = current_LM + 1; i < rps.size(); i++)
        {
                if(interSectWithLine(views[current_LM], rps[current_LM], rps[i]) == true)
                {
                    //record the current one and 
                    current_LM = i;
                    robot_group.push_back(robotPosition[i]);

                    rbs.push_back(rps[i-1]);
                    rbs.push_back(rps[i]);
                    LMs.push_back(views[i]);

                    //record local map ID
                    LMs_chunk.push_back(id_group[i]);
                    
                    
                    goto lb;
                }
                else
                {
                    //the last step, save the robot position but the last view
                    if(i == rps.size() - 1)
                    {
                        rbs.push_back(rps[i]);
                    }
                }
        }
        
        /*
        cout << " the size of LMs id : " << LMs_chunk.size() << endl<<endl;
        cout << " the ids : ";
        for(int i = 0; i < LMs_chunk.size(); i++)
        {
            cout << LMs_chunk[i] << " "; 
        }
        cout << endl;*/
        //waitHere();
        
        //construct chunk using local maps 
        vector<Object> chunk_LMs = ConstructChunk(LMs);
        
        //construct paths for chunk
        //vector<Object> path_LMs = ConstructPath(rbs);
        
        //construct path segments for chunk using circles
        vector<Object> path_LMs = constructEllipseOfASR(robotPosition, views,  chunk_map, LMs, chunk_cnter);
        /*
        vector<Exit> exits = FindExits(chunk_LMs, path_LMs);
        char plotFileName[80];
        sprintf(plotFileName, "%s", "Maps/Offline/Chunk&Exits.png");
        plotObjectsOf3KindswithExits(plotFileName, chunk_map, path_LMs, exits);
        */
        
}

/* Using circle to determine which robot position is crucial 
 * Criterion: Diameter < threshold  ---> cross a narrow place
 */
void LocalMapAndRobotUsingCircle(vector<Object> chunk_map, vector<Point> rps,
                      vector< vector<Object> > robotPosition, vector<int> id_group,
                      vector< vector<Object> > views)
{ 
        cout << " In finding nearest points process !!! " << endl << endl;
        vector<Object> ref_map;        //reference map
        Point left_point, right_point; //nearest points on two sides
        double start, end;           //circle start point and end point
        double radius = 100;         //radius
        double increment_step = 100; //radius increment step
        double space_threshold = 500;

        unsigned char left_flag = 0; //find the left hand nearest point 
        unsigned char right_flag = 0;//find the right hand nearest point 
        
        Object joint_2_point;
        vector<Object> temp;

        //chunk map
        ref_map = chunk_map;

        for(int i = 0; i < robotPosition.size(); i++)
        {
            
                start = robotPosition[i][7].getAngleWithXaxis();
                end = start + PI;
            
                //left side circle, find nearest point
                do
                {
                    left_point = IntersectedWithCircle(rps[i], radius, 
                                      ref_map, start, end);
                    if(left_point.X() != 0 && left_point.Y() != 0)
                        left_flag = 1;
                    else
                        radius += increment_step;
                            
                }while(left_flag != 1);
                //right side circle, repeat
                    
                do
                {
                    right_point = IntersectedWithCircle(rps[i], radius, 
                                      ref_map, start, end);
                    if(right_point.X() != 0 && right_point.Y() != 0)
                        left_flag = 1;
                    else
                        radius += increment_step; 
                            
                }while(left_flag != 1); 
                    
                //join them as a line
                joint_2_point.set(left_point.X(), left_point.X(), right_point.X(), right_point.Y(), i);
                temp.push_back(joint_2_point);
   
                //check conversion rate
        }
        
                char plotFileName[80];
                sprintf(plotFileName, "%s", "Maps/Offline/TestChunk_Path.png");
                plotObjects(plotFileName, chunk_map, temp);
                //waitHere();
}


vector<Object> ConstructChunk(vector< vector<Object> > LMs)
{
        vector<Object> temp;
        temp = LMs[0];
        for(int i = 1; i < LMs.size(); i++)
             temp = addTwoVectorsOfObjects(temp, LMs[i]);

        
        return temp;
}

vector<Object> ConstructPath(vector<Point> rps)
{
    Object temp_obj;
    vector<Object> paths;
    
    for(int i = 0; i < rps.size()-1; i++)
    {
        temp_obj.set(rps[i].X(), rps[i].Y(), rps[i+1].X(), rps[i+1].Y(),i);
        paths.push_back(temp_obj);
    }
    
    return paths;
}




