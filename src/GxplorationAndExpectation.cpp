

//============================================================================
// Name        : Exploration and Expectation switch each other
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
#include "FunctionOfMFIS.h"
#include "PathIntegrationModule.h"

//#include "concave_hull.h"
#include "StructBoundary.h"

#include <cstdlib>
#include <ctime>


#define PI 3.14159265
using namespace std;

//***********************************************************************************//

void TransforIntoGloabl(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num); // only transform
vector<Object> TransformforAllviews(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num);


/*************************************************************************************/

const char* levelName = "inputData/level";
const char* surfaceName = "/surfaces-";
char viewFileName[80], mfisFileName[80], ctFileName[80], LocalMFileName[80];
char exitsFileName[80];

//char testFileName[80]; // this file char array is of test programme and plot

//char Flag_LM;
LocalMap LMS; // added for chunk & local maps
Transporter chunks; // added for chunk & local maps
vector<Chunk> allChunks;// added for chunk & local maps

vector<Object> NewLM; // a new local map
vector<Object> PreLM; // previous local map
vector<Object> ALLlocalMap, MapofLMs;
vector< vector<Object> > ALLlocalMapInGlobal;

double traveledDistance = 0; // total distance traversed 
double robotFacing = 0; //in degree relating to global
double robotTurning = 0; //in degree relating to global
vector<double> facingOrient;

vector< vector<Object> > allLEs; // all local environments 
vector< vector<Object> > Memories; // all local maps as memories
vector< vector<Object> > ListOfPositions; // for storing currentCRPInMFIS variable 

vector<Point> RobotPs;
vector<Point> rbs;

vector<Point> turningPoint; // for generating path
unsigned char turningFlag = 0; // 0 - not turn  1- turned

//**********************************//
vector<Object> GlobalLM;
vector<ASR> placesOfLM;
Transporter computeRe, Cmoutput;
Transporter lastLMInfo;

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
vector<Object> PI_RobotPosition; ///every step
vector<Object> currentTransLM;

unsigned char firstloop_Flag = 0;  //first loop flag 
unsigned char lastloop_Flag = 0;   //last loop flag for chunk process

unsigned char PI_switch_Flag = 0; //PI switch flag
unsigned char function_switch_Flag = 1; //1 -- method1 (normal process), 2 -- method2 (expectation process)
unsigned char expectation_flag = 0;

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
int fist_step_m2 = 0; // 0 is the first step 
int info_part_flag = 0;
int m1_start_point = 1; //the start point of m1 
vector<Object> robot_orientation_arrow;//arrow based on robot's orientation
vector<Object> switch_robot_position;
vector<Object> transformed_view; 
vector<Object> environment_map;             //after projection, the mfis becomes fixed
vector<Object> last_environment_map;
vector< vector<Object> > corrected_info;
vector< vector<Object> > all_arrows;
pair< vector<Object>,  vector<Object> > adjust_info;

vector<Exit> potential_exits;

//for plotting
vector<Object> test_robot_positions;
vector<Object> enduring_part;
vector<Object> enduring_map_polygon;
vector<Surface> currnet_enduring_poly_area;
vector<Surface> polygon;

vector<ChunkInfo> all_chunk_info;
//counters for paper
int chunk_cnt = 0;  //chunk counter
int vt_counter = 0; //visual triangulation
int pi_counter = 0; //path integration

//vector<Object> temp_env_map;

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
    
    switch_robot_position = currentRobotPositionInMFIS; //switch m1 robot position
    
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
    
    ALLlocalMapInGlobal.push_back(currentView);
    
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
    
    views.push_back(currentView);           
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

    //main loop
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
    
       
        if(function_switch_Flag == 2)
            goto lb_pi;
        
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

        targetObjectsInMPV = targetObjectsInPV;
  
        if ((recognizedTargetObjects.getTargetObjects().size() < 4))
        {
            
            if((v - limitingPoints.back()) > 1) //update at last step //retriving info to update at last step
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
            {

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
lb_pi:                          Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
                                aLine.setKP(1);
                                odometricReferenceObject.clear();
                                odometricReferenceObject.push_back(aLine);
                                odometricReferenceObject.push_back(allRobotPositions[6]);
                                odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());
                                referenceObjects = odometricReferenceObject;
                                lostPoints.push_back(v); //just for printing at the end
                                
                                pi_counter++;
                                PI_switch_Flag = 1; 
                        }
                        else
                            vt_counter++;
     
                }
     
            }
               
            
            //localization 
            PI_RobotPosition = Path_integrate_position(coordTransInfo, currentRobotPositionInMFIS, allRobotPositions);
            currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP()); 
            
            views.push_back(TransformforToGlobalCoordinate(currentView, PI_RobotPosition[6].getP1(), PI_RobotPosition, PI_RobotPosition[7].getAngleWithXaxis()));  
            
            if((abs(distanceOftwoP(currentRobotPositionInMFIS[6].getP1(), PI_RobotPosition[6].getP1())/coordTransInfo[0]) > 10.0)
               && ((v - limitingPoints.back()) == 1))
            {
                currentRobotPositionInMFIS = PI_RobotPosition;
                vt_counter--;
                pi_counter++;

            }
            
//            //for data1000set900 download
//            if(v == 48)
//            {
//                currentRobotPositionInMFIS = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, Point (-200,-300), currentRobotPositionInMFIS, 0);  
//            }
//            if(v == 142)
//            {
//                currentRobotPositionInMFIS = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, Point (-450,-100), currentRobotPositionInMFIS, 6);  
//            }
//            if(v == 147)
//            {
//                currentRobotPositionInMFIS = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, Point (-300,0), currentRobotPositionInMFIS, 5);  
//            }
//            if(v == 148)
//            {
//                currentRobotPositionInMFIS = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, Point (-400,100), currentRobotPositionInMFIS, 5);  
//            }
//            
//            if(v == 213)
//            {
//                currentRobotPositionInMFIS = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, Point (-400,0), currentRobotPositionInMFIS, 0);  
//            }

            currentRobotPositionInMFIS[6].setVN(v); 
            robot_points.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));              

            //transformed_view = TransformforToGlobalCoordinate(currentView, odometricCRPositionInMFIS[6].getP1(), odometricCRPositionInMFIS, odometricCRPositionInMFIS[7].getAngleWithXaxis());   
            transformed_view = TransformforToGlobalCoordinate(currentView, currentRobotPositionInMFIS[6].getP1(), currentRobotPositionInMFIS, currentRobotPositionInMFIS[7].getAngleWithXaxis());   
            
            
            if(PI_switch_Flag == 1)
            {
                pair< vector<Object>, vector<Object> > further_adjust = constrain_path_integration(transformed_view, currentRobotPositionInMFIS,  
                                     MFIS, v);
                currentRobotPositionInMFIS = further_adjust.first;
                transformed_view = further_adjust.second;
                
                PI_switch_Flag = 0;
            }
//            else
//            {
//                pair< vector<Object>, vector<Object> > further_adjust = check_postion_and_env(currentRobotPositionInMFIS, transformed_view, MFIS, v);
//                currentRobotPositionInMFIS = further_adjust.first;
//                transformed_view = further_adjust.second;
//
//            }
            
            AllGlobalRobotPs.push_back(currentRobotPositionInMFIS);
            ALLlocalMapInGlobal.push_back(transformed_view);
            
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS&Adjust_position-", v, ".png");                   
            plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, transformed_view, environment_map);
            
            
            //compute arrow based on robot's orientation
            robot_orientation_arrow = drawArrow(currentRobotPositionInMFIS[6], v);
            
            if(function_switch_Flag == 1)
            {
//                    if((critical_point_switch(currentRobotPositionInMFIS, AllGlobalRobotPs) > 0) 
//                            && (abs(robotTurning) > 100)) //&& (expectation_flag == 0)) //first time to switch m2
                if(((critical_point_switch(currentRobotPositionInMFIS, AllGlobalRobotPs) > 0) 
                            && (abs(robotTurning) > 100)) || (pointInPolygon(PointXY(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currnet_enduring_poly_area) == true))    
                //if(computedOutput.getChunkFlag() == 1)
                {
                        int reach_point = critical_point_switch(currentRobotPositionInMFIS, AllGlobalRobotPs);
                        
                        //reach_point = critical_reach_point;
                        if(m1_start_point > reach_point)
                            reach_point = v-1;
                        
                        environment_map = FunctionOfMFIS(level, set, m1_start_point, reach_point);

                        cout << " the reach point : " << reach_point << endl;
                        //transfer just compute environment onto global cooridnate
                        environment_map = TransformforToGlobalCoordinate(environment_map, switch_robot_position[6].getP1(), switch_robot_position, switch_robot_position[7].getAngleWithXaxis());   
                       
//                        //test//
//                        vector<Object> temp_virtual_boundary, temp_solid_boundary;
//                        vector<Object> test_current_enduring_map_poly = each_step_update_poly;
//                        test_current_enduring_map_poly = TransformforToGlobalCoordinate(test_current_enduring_map_poly, switch_robot_position[6].getP1(), switch_robot_position, switch_robot_position[7].getAngleWithXaxis());   
//
//                        cout << " enduring_map_poly size : " << test_current_enduring_map_poly.size() << endl; 
//                        integrate_virtual_boundary(test_current_enduring_map_poly);
                        
                        
                        
//                        for(int m = 0; m < test_current_enduring_map_poly.size();m++)
//                        {
//                            if((test_current_enduring_map_poly[m].get_imagined_flag() == 1) && (test_current_enduring_map_poly[m].length() > 500))
//                            {
//                                temp_virtual_boundary.push_back(test_current_enduring_map_poly[m]);
//                                test_current_enduring_map_poly.erase(test_current_enduring_map_poly.begin()+m);
//                                m--;
//                            }
//                            else
//                                temp_solid_boundary.push_back(test_current_enduring_map_poly[m]);
//                        }
//                        //temp_virtual_boundary = breakTheLinesInto(temp_virtual_boundary);
//                                
//                        //potential_exits = potential_exit_on_boundary(temp_virtual_boundary, test_current_enduring_map_poly);
//                        //vector<Object> modified_boundaries = boundary_without_far_info(temp_virtual_boundary, test_current_enduring_map_poly);
//                        //vector<Object> ultimate_boundary = ultimate_trim_boundary(test_current_enduring_map_poly, modified_boundaries);
//                        //every time compute an individual section of enduring map
//                        //adjust it with respect to the existing part of enduring map
//                        
//                        //re-compute transformed current view
//                        transformed_view = TransformforToGlobalCoordinate(currentView, currentRobotPositionInMFIS[6].getP1(), currentRobotPositionInMFIS, currentRobotPositionInMFIS[7].getAngleWithXaxis()); 
//                        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS_to_critical_point-", v, ".png");                   
//                        plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), test_robot_positions, environment_map);
                        
                        if(last_environment_map.size() != 0)
                        {
                            environment_map = addTwoVectorsOfObjects(environment_map, last_environment_map);
                        }
                        enduring_part = environment_map;
                        
                        //////  test programme for new updating process for data1000set900//////
                        ClipperLib::Path subj2;
                        Paths solution2;
                        ClipperOffset co2;
                        subj2 = viewConvertPath(enduring_part); //convert view into Path
                        co2.AddPath(subj2, jtMiter, etClosedPolygon);
                        co2.Execute(solution2, 0.0); // outward offset the polygon
                        SimplifyPolygons(solution2, pftEvenOdd);
                        enduring_map_polygon = PathsConvertView(solution2);
                        enduring_map_polygon = remove_noise_surfaces(enduring_map_polygon);
                       
                        polygon = convertObjectToSurface(enduring_map_polygon);
                        currnet_enduring_poly_area = polygon;
                 
                        
                        //test functions
                        //test_combine(AllGlobalRobotPs, ALLlocalMapInGlobal);
                        //test_position(AllGlobalRobotPs, ALLlocalMapInGlobal);
                        //bound_map_modified_version2(AllGlobalRobotPs, ALLlocalMapInGlobal);
                        //bounded_space_version2(AllGlobalRobotPs, ALLlocalMapInGlobal);
                        
                        
                        //for paper plotting
                        test_robot_positions.clear();

                        //waitHere();//////////////////////////////////////////////////////////
                        
                        function_switch_Flag = 2;
                        //expectation_flag = 1;
                        goto m2;
                    }

                    //for plotting in paper
                    if(last_environment_map.size() != 0)
                    {
                        test_robot_positions = addTwoVectorsOfObjects(test_robot_positions, currentRobotPositionInMFIS);
                        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/TEST_TEMP_Robots_ENV-", v, ".png");  
                        plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), test_robot_positions, last_environment_map);

                    }
                    
                    
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

                    //computedOutput = updatePerceptualMapATPlaceDeletion_version2(places, MFIS, currentView, currentRobotPositionInMFIS,
                    computedOutput = updatePerceptualMapATPlaceDeletion_version3(places, MFIS, currentView, currentRobotPositionInMFIS,
                            allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, expect_Flag);

                    MFIS = computedOutput.getView();
                    environment_map = MFIS;

                    //LocalMap = computedLocalMapOutput.getView();
                    places = computedOutput.getASRs();
                    targetObjectsInPV = computedOutput.getTargetObjects();

                    //robpositions.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));
                    robot_Positions.push_back(currentRobotPositionInMFIS);
                    //robot_points.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));              
                    //views.push_back(Global_ASR);           
                    id_group.push_back(v);     


                    limitingPoints.push_back(v); //limiting/updating points just for printing at the end

                    //to print local and global map
                    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");                   
                    plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS);

                    recognizedTargetObjectInPV = recognizedTargetObjects.getTargetObjects();

            }
            
m2:         if(function_switch_Flag == 2)
            {

                if(expectation_flag == 1)
                { 
                    
                    vector<Object> ref_of_match = Expect_reference(currentRobotPositionInMFIS, transformed_view, environment_map);
                    if(ref_of_match.size() > 0)
                    {
                        corrected_info = Expect_function(ref_of_match, currentRobotPositionInMFIS, transformed_view, environment_map, v);
                        
                        if((fist_step_m2 == 0) && (corrected_info[0][6].getP1() == currentRobotPositionInMFIS[6].getP1()))
                             goto re_adjust;   
                        
                        if(distanceOftwoP(currentRobotPositionInMFIS[6].getP1(), corrected_info[0][6].getP1()) < 1000)
                        {
                            currentRobotPositionInMFIS = corrected_info[0];
                            transformed_view = corrected_info[1];
                        }
 
//                        //for data1000set900
//                        if(v == 80)
//                        {
//                            transformed_view = TransformforToGlobalCoordinate(transformed_view, Point (180,-70), currentRobotPositionInMFIS, 0);  
//                        }
//                        if(v == 86)
//                        {
//                            transformed_view = TransformforToGlobalCoordinate(transformed_view, Point (270,-150), currentRobotPositionInMFIS, 0);  
//                        }
//                        if(v == 151)
//                        {
//                            currentRobotPositionInMFIS = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, Point (-300,-100), currentRobotPositionInMFIS, 2);  
//                        }
                        
                        //// test ////
                       // environment_map = clear_and_update_env(environment_map, transformed_view, currentRobotPositionInMFIS);
                        environment_map = clear_and_update_env_with_boundary(environment_map, transformed_view, currentRobotPositionInMFIS, enduring_map_polygon);        
                        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS&Adjust_position-", v, ".png");                   
                        plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, corrected_info[1], environment_map);
                        
                    
                    }
                    else
                    {
                        //if this is the first step in m2,  there must be expected
re_adjust:              if(fist_step_m2 == 0)
                        {
                            Object temp_refer_view = getLargestObject(transformed_view);
                            Object joint_with_robot;
                            joint_with_robot.set(temp_refer_view.midpoint().X(), temp_refer_view.midpoint().Y(), currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), 1);
                            
                            Object temp_refer_map = intersectForObject(environment_map, joint_with_robot.getP1(), joint_with_robot.getP2());
                            
                            if(((temp_refer_view.length() / temp_refer_map.length()) > 0.5)
                                    && ((temp_refer_view.length() / temp_refer_map.length()) < 1.5))
                            {
                                ref_of_match.clear();
                                ref_of_match.push_back(temp_refer_map);
                                ref_of_match.push_back(temp_refer_view);
                                corrected_info = Expect_function(ref_of_match, currentRobotPositionInMFIS, transformed_view, environment_map, v);
                                
                                if(distanceOftwoP(currentRobotPositionInMFIS[6].getP1(), corrected_info[0][6].getP1()) < 1000)
                                {
                                    currentRobotPositionInMFIS = corrected_info[0];
                                    transformed_view = corrected_info[1];
                                }
                                
                                fist_step_m2 = 1;
                                
                                
                                //// test ////
                                //environment_map = clear_and_update_env(environment_map, transformed_view, currentRobotPositionInMFIS);
                                environment_map = clear_and_update_env_with_boundary(environment_map, transformed_view, currentRobotPositionInMFIS, enduring_map_polygon); 
                                sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS&Adjust_position-", v, ".png");                   
                                plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, corrected_info[1], environment_map);
                            }
                        }
                        else
                        {
                            //// test ////
                            //environment_map = clear_and_update_env(environment_map, transformed_view, currentRobotPositionInMFIS);
                            environment_map = clear_and_update_env_with_boundary(environment_map, transformed_view, currentRobotPositionInMFIS, enduring_map_polygon);         
                            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS&Adjust_position-", v, ".png");                   
                            plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, transformed_view, environment_map);
                        }
                    }
                    
                    /* for dataset level4set14
                    if(v == 61)
                    {
                        //currentRobotPositionInMFIS = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, Point (300, -300), currentRobotPositionInMFIS, -20);   
                        char ppp;
                        int zhi = -1, hori = 0, vir = 0;
                        cout << "input ppp y/n : " << endl;
                        cin >> ppp;
                        while(ppp == 'y')
                        {
                            transformed_view = TransformforToGlobalCoordinate(transformed_view, Point (hori, vir), currentRobotPositionInMFIS, zhi);   
                            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS&Adjust_position-", v, ".png");                   
                            plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, transformed_view, environment_map);
                            cout << "input ppp y/n : " << endl;
                            cin >> ppp;
                            cout << "input zhi  : " << endl;
                            cin >> zhi;
                            cout << "input hori  : " << endl;
                            cin >> hori;
                            cout << "input vir  : " << endl;
                            cin >> vir;
                        }
                    }
                    
                    if(v == 62)
                    {
                    
                        char ppp;
                        int zhi = -1, hori = 0, vir = 0;
                        cout << "input ppp y/n : " << endl;
                        cin >> ppp;
                        while(ppp == 'y')
                        {
                            transformed_view = TransformforToGlobalCoordinate(transformed_view, Point (hori, vir), currentRobotPositionInMFIS, zhi);   
                            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS&Adjust_position-", v, ".png");                   
                            plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, transformed_view, environment_map);
                            cout << "input ppp y/n : " << endl;
                            cin >> ppp;
                            cout << "input zhi  : " << endl;
                            cin >> zhi;
                            cout << "input hori  : " << endl;
                            cin >> hori;
                            cout << "input vir  : " << endl;
                            cin >> vir;
                        }
                    }*/
                    
          
                    
                    currentRobotPositionInMFIS[6].setVN(v);
                    
                }
                else
                { 
                    //original function//
                    //adjust_info = info_adjustment(transformed_view, currentRobotPositionInMFIS, environment_map, currentRobotPositionInMFIS, v);
                    
                    //using this function//
                    //adjust_info = info_first_expect(transformed_view, currentRobotPositionInMFIS, environment_map, currentRobotPositionInMFIS, v);

                    //test function for data1000est900//
                    adjust_info = info_first_expect_modify(transformed_view, currentRobotPositionInMFIS,  
                                     environment_map, v);
                            
                    sprintf(viewFileName, "%s%d%s", "Maps/Offline/Transed_and_MFIS", v, ".png");
                    //plotObjectsOf5Kinds(viewFileName, transformed_view, odometricCRPositionInMFIS, adjust_info.first, adjust_info.second, environment_map);
                    //plotObjectsOf4Kinds(viewFileName, myrobot.getRobot(), odometricCRPositionInMFIS, environment_map, transformed_view);
                    currentRobotPositionInMFIS = adjust_info.second;
                    if(expect_process_flag == 1)
                    {
                        environment_map = addTwoVectorsOfObjects(adjust_info.first, remaining);
                        expectation_flag = 1;
                    }

                    fist_step_m2 = 1;
                }
                robot_orientation_arrow = drawArrow(currentRobotPositionInMFIS[6], v);
                allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);
                
                
                
                if(expectation_flag == 1)
                {
                    int threshold_cnt = 0;
                    //compute polygon to check if current view is overlapped view MFIS
                    vector<Object> area_of_new_view; 

                    ClipperLib::Path subj;
                    Paths solution;
                    ClipperOffset co;

                    subj = viewConvertPath(transformed_view); //convert view into Path
                    co.AddPath(subj, jtMiter, etClosedPolygon);
                    co.Execute(solution, 300.0); // outward offset the polygon
                    area_of_new_view = PathsConvertView(solution); // convert polygon into view
                    
                    for(int m = 0; m < environment_map.size(); m++)
                    {
                        if(interSectWithLine(area_of_new_view, environment_map[m].getP1(), environment_map[m].getP2()) == true)
                            threshold_cnt++;
                    }

                    //if(pointInPolygon(PointXY(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), polygon) == false)
                    Object temp_path_seg;
                    temp_path_seg.set(AllGlobalRobotPs[AllGlobalRobotPs.size()-2][6].X1(), AllGlobalRobotPs[AllGlobalRobotPs.size()-2][6].Y1(), currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), 1);
                    if(interSectWithLine(enduring_map_polygon, temp_path_seg.getP1(), temp_path_seg.getP2()) == true)
                    {
//                      currnet_enduring_poly_area = polygon;
                        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/TEST_enduring_map_polygon-", v, ".png");
                        plotObjectsOf3Kinds(mfisFileName, currentRobotPositionInMFIS, enduring_part, enduring_map_polygon);
                        goto method1_swtich;
                    }
                    
                    if((threshold_cnt <= 1) && (currentView.size() > 4))
                    {
                        // switch back to mathod 1 //
method1_swtich:         cout << " ----- Switch back to method 1 ----- " << endl;
                        //waitHere();
                        switch_robot_position = currentRobotPositionInMFIS;
                        expect_process_flag = 0;
                        //expectation_flag = 0; 
                        function_switch_Flag =1;
                        fist_step_m2 = 0;
                        robotTurning = 0;
                        m1_start_point = v; //start of m1

                        //environment_map = deletOneViewbaseLength(environment_map, transformed_view);
                        
                        last_environment_map = MFIS = environment_map;

                        test_robot_positions = currentRobotPositionInMFIS;
                    }
                }
                
                
            }
     
            //robot_orientation_arrow = drawArrow(currentRobotPositionInMFIS);       
            all_arrows.push_back(robot_orientation_arrow);
            //sprintf(mfisFileName, "%s%d%s", "Maps/Offline/PercptualMapWithRobots-", v, ".png");
            //plotObjectsOf3Kinds(mfisFileName, allRobotPositions, myrobot.getRobot(), MFIS);
            

            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Arrows_and_positions_map-", v, ".png");
            //plotObjectsColours(mfisFileName, all_arrows, allRobotPositions, environment_map);
            //plotObjects(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, last_environment_map);
            plotObjects(mfisFileName, allRobotPositions, environment_map);

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
        ////expect_process_flag = 0;
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





