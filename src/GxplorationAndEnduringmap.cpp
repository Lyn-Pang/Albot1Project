
//============================================================================
// Name        : Exploration and Enduring map
// Author      : Wenwang (Lyn)
// Version     :
// Copyright   : Your copyright notice
// Description : 
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

void TransforIntoGloabl(vector<Object> addView, Point coord, vector<Object> currentPos, 
                                double angle, int num); // only transform
vector<Object> TransformforAllviews(vector<Object> addView, Point coord, vector<Object> currentPos,
                                double angle, int num);

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
unsigned char familiar_flag = 0;

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

int last_Chunk_step = 0;                      //the step ID computing last chunk
int before_start_number = 0;                   //
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
vector<Object> geometry_map;
vector<Object> geometry_boundary;
vector<Object> project_memory;

vector<Object> sides_of_geo; //four sides of geometry of local space

vector< vector<Object> > corrected_info;
vector< vector<Object> > all_arrows;
vector< vector<Object> > clusters;
vector< vector<Object> > all_resions;

pair< vector<Object>,  vector<Object> > adjust_info;

vector<Exit> potential_exits; //all potential exits
vector<Exit> exit_in_region;  //potentional exits in a particular region

Region_struct regions_exits; //each chunk regions and exits
vector<Region_struct> all_chunks_region_exit;

Exit current_region_exit; // current region exit
Exit last_region_exit;    // last region exit

Object last_entry_side, last_exit_side;       // important sides in last region
Object current_entry_side, current_exit_side; // important sides in current region

vector<Exit> possible_exits_in_MFIS;

//for structural geometry
vector< vector<Object> > last_chunk_regions;
vector< vector<Object> > current_regions; 
vector< vector<Object> > all_chunks; 

vector< vector<Object> > all_chunk_path;


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

    exitsFromcurrentLM = findGapasExits(NewLM); // initial exits in first view/local environment

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

    possible_exits_in_MFIS = exitsFromCV = findShortestExits(currentView);
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/view&exits-", 1, ".png");                   
            plotObjectsOf3KindswithExits(mfisFileName, currentView, currentRobotPositionInMFIS, exitsFromCV);

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
    
       
        //if(function_switch_Flag == 2)
        //    goto lb_pi;
        
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


            
            //dataset5-3 localise robot first
            /*if(v == 134)
            {
                Object aLine;//
                
                double temp_x1, temp_y1, temp_x2, temp_y2;
                
                temp_x1 = sin(deg2rad(coordTransInfo[1]+14)) * coordTransInfo[0] + lastStepInfo.getRobotPosition()[6].X1();
                temp_y1 = cos(deg2rad(coordTransInfo[1]+14)) * coordTransInfo[0] + lastStepInfo.getRobotPosition()[6].Y1();
                temp_x2 = sin(deg2rad(coordTransInfo[1]+14)) * (coordTransInfo[0]+500) + lastStepInfo.getRobotPosition()[6].X1();
                temp_y2 = cos(deg2rad(coordTransInfo[1]+14)) * (coordTransInfo[0]+500) + lastStepInfo.getRobotPosition()[6].Y1();
                aLine.set(temp_x1, temp_y1, temp_x2, temp_y2,0);
                aLine.setKP(1);
                odometricReferenceObject.clear();
                odometricReferenceObject.push_back(aLine);
                odometricReferenceObject.push_back(allRobotPositions[6]);
                currentRobotPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());
                referenceObjects = odometricReferenceObject;
            }*/
            
            views.push_back(TransformforToGlobalCoordinate(currentView, PI_RobotPosition[6].getP1(), PI_RobotPosition, PI_RobotPosition[7].getAngleWithXaxis())); 
            
            
            if((abs(distanceOftwoP(currentRobotPositionInMFIS[6].getP1(), PI_RobotPosition[6].getP1())/coordTransInfo[0]) > 3.0)
                    //|| ((currentRobotPositionInMFIS[6].Y2() - currentRobotPositionInMFIS[6].Y1()) * (PI_RobotPosition[6].Y2() - PI_RobotPosition[6].Y1())) < 0)
                 && ((v - limitingPoints.back()) == 1))
            {
                //currentRobotPositionInMFIS = PI_RobotPosition;
                vt_counter--;
                //pi_counter++;
                
                currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();
                goto lb_pi;
                
            }
            
      
            currentRobotPositionInMFIS[6].setVN(v); 
            robot_points.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));              

            transformed_view = TransformforToGlobalCoordinate(currentView, currentRobotPositionInMFIS[6].getP1(), currentRobotPositionInMFIS, currentRobotPositionInMFIS[7].getAngleWithXaxis());   
            
            exitsFromCV = findShortestExits(transformed_view);
            possible_exits_in_MFIS = addTwoExits(possible_exits_in_MFIS, exitsFromCV);
            
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/View&Exit-",v,".png");                   
            plotObjectsOf3KindswithExits(mfisFileName, transformed_view, currentRobotPositionInMFIS,exitsFromCV);
            
            if(PI_switch_Flag == 1)
            {
                pair< vector<Object>, vector<Object> > further_adjust = constrain_path_integration(transformed_view, currentRobotPositionInMFIS,  
                                                                                                   MFIS, v);
                if(abs(distanceOftwoP(further_adjust.first[6].getP1(), PI_RobotPosition[6].getP1())/coordTransInfo[0]) < 2.0)
                {
                    currentRobotPositionInMFIS = further_adjust.first;
                    transformed_view = further_adjust.second;
                }
                
                PI_switch_Flag = 0;
                
            }

            

            
            AllGlobalRobotPs.push_back(currentRobotPositionInMFIS);
            ALLlocalMapInGlobal.push_back(transformed_view);

            allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS); 
            
            /*===========================================================================================*/
            //routeMap
            pathFromLastLimitingPoint.set(routeMapConnLP.back().X2(), routeMapConnLP.back().Y2(),
                    currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), v);
            routeMapConnLP.push_back(pathFromLastLimitingPoint);

            robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);

            updatingPoints.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

            computedOutput = updatePerceptualMapATPlaceDeletion_version3(places, MFIS, currentView, currentRobotPositionInMFIS,
                     allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP, expect_Flag);

            MFIS = computedOutput.getView();

            places = computedOutput.getASRs();
            targetObjectsInPV = computedOutput.getTargetObjects();

            robot_Positions.push_back(currentRobotPositionInMFIS);           
            id_group.push_back(v);     


            limitingPoints.push_back(v); //limiting/updating points just for printing at the end

            //to print local and global map
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");                   
            plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS);
            
            if(Global_ASR.size() != 0)
                plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS, Global_ASR);

                

            recognizedTargetObjectInPV = recognizedTargetObjects.getTargetObjects();

            //modification by Lyn dataset 5-3
            /*
            if((function_switch_Flag == 2) && (expectation_flag == 0)) //insdie a familiar space
            {
                sides_of_geo = rotate_on_point(sides_of_geo, sides_of_geo[3].getP1(), -0.8);
                
                if(interSectWithLine(sides_of_geo, currentRobotPositionInMFIS[6].getP1(), lastStepInfo.getRobotPosition()[6].getP1()) == true)
                {
                    cout << " It's moved out from previous space !! " << endl;
                    function_switch_Flag = 1;
                    expectation_flag = 1;
                    
                    Object move_out_side = intersectForObject(current_regions.back(), currentRobotPositionInMFIS[6].getP1(), lastStepInfo.getRobotPosition()[6].getP1());
                    current_exit_side = move_out_side;
                    
                    vector<Object> test;
                    test.push_back(move_out_side);
                    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/robot_and_view-", v, ".png");                   
                    plotObjectsOf4Kinds(mfisFileName, currentRobotPositionInMFIS, transformed_view, current_regions.back(), test);
                    
                    //select info on that side
                    last_region_exit = find_exit_cross(MFIS, move_out_side, lastStepInfo.getRobotPosition(), currentRobotPositionInMFIS);
                    waitHere();

                }
                    
            }
            else
            {
                if((function_switch_Flag == 2) &&(expectation_flag == 1)) // not inside a familiar space
                {
                    //form an intermediate geometry's boundary lines 
                    Object line1, line2;
                    Object boundary1, boundary2;
                    
                    line1 = current_regions.back()[1];
                    line2 = current_regions[current_regions.size()-2][0];
                    
                    Point temp_p = intersectPointTwolineEquations(line1, line2);
                    boundary1.set(line1.X1(), line1.Y1(), temp_p.X(), temp_p.Y(), 0);
                    boundary2.set(line2.X1(), line2.Y1(), temp_p.X(), temp_p.Y(), 1);
                    
                    if(twoLinintersect(boundary1.getP1(), boundary1.getP2(), currentRobotPositionInMFIS[6].getP1(), lastStepInfo.getRobotPosition()[6].getP1()) == true)
                        
                    {
                        vector<Object> group1, group2;
                        
                        for(int m = 0; m < MFIS.size(); m++)
                        {
                            if((boundary2.distP1ToP1(MFIS[m]) < 2000) || (boundary2.distP1ToP2(MFIS[m]) < 2000))
                                group1.push_back(MFIS[m]);
                            else
                            {
                                if((boundary2.distP2ToP1(MFIS[m]) < 2000) || (boundary2.distP2ToP2(MFIS[m]) < 2000))
                                    group2.push_back(MFIS[m]);
                            }
                        }
                        
                        //compute the rough position of the exit
                        last_region_exit = shortestExit(group1, group2);
                        
                        vector<Exit> plot_exit;
                        plot_exit.push_back(last_region_exit);
                        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/robot_and_view_and_exit-", v, ".png");                   
                        plotObjectsOf3KindswithExits(mfisFileName, MFIS, current_regions.back(), plot_exit);
                        
                        //find the gap it is to cross
                        Object path_segment, temp1, temp2;
                        vector<Object> temp3;
                        Exit to_cross_gap, project_gap;
                        
                        path_segment.set(lastStepInfo.getRobotPosition()[6].getP1().X(), lastStepInfo.getRobotPosition()[6].getP1().Y(), 
                                    currentRobotPositionInMFIS[6].getP1().X(), currentRobotPositionInMFIS[6].getP1().Y(), 0);
                        
                        to_cross_gap = most_constrain_exit_cross_path(MFIS, path_segment);
                        temp1.set(to_cross_gap.X1(), to_cross_gap.Y1(), to_cross_gap.X2(), to_cross_gap.Y2(), 0);
                        temp1.reverse();
                        //compute the potential gap on left side
                        temp2 = remakeLineP2(temp1, boundary1 , boundary2, boundary2.getID(), 0,2);
                        temp3.push_back(temp1);
                        temp3.push_back(temp2);
                        group1.clear();
                        group2.clear();
                        plot_exit.clear();
                        
                        for(int m = 0; m < MFIS.size(); m++)
                        {
                            if(shortestDistanceBtwTwoObjects(temp2, MFIS[m]) < 600)
                                group1.push_back(MFIS[m]);
                            
                        }
                        
                        for(int m = 0; m < group1.size()-1; m++)
                        {
                            Object temp_gap;
                            if((shortestDistanceBtwTwoObjects(group1[m], group1[m+1]) > 2000)
                                && (shortestDistanceBtwTwoObjects(group1[m], group1[m+1]) <3000))
                            {
                                temp_gap.set(group1[m].midpoint().X(), group1[m].midpoint().Y(), group1[m+1].midpoint().X(), group1[m+1].midpoint().Y(), m);
                                group2.push_back(temp_gap);
                            }
                        }
                        
                        group2.push_back(temp1);
                        group2 = breakTheLinesInto(group2);
                        
                        //compute the rough position of the exit
                        last_region_exit = shortestExit(group1, group2);
                        plot_exit.push_back(last_region_exit);
                        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/robot_and_view_and_exit-", v, ".png");                   
                        //plotObjectsOf3KindswithExits(mfisFileName, MFIS, current_regions.back(), plot_exit);
                        plotObjectsOf4Kinds(mfisFileName, MFIS, current_regions.back(), group1, group2);
                        
                        familiar_flag = 1;
                        function_switch_Flag = 1;
                        expectation_flag = 0;
                        waitHere();
                    }

                }
            }
            
            if(familiar_flag == 1)
            {
                if(twoLinintersect(last_region_exit.getP1(), last_region_exit.getP2(), currentRobotPositionInMFIS[6].getP1(), lastStepInfo.getRobotPosition()[6].getP1()) == true)
                {
                    cout << " it is coming back to the first geometry ! " << endl;
                    cout << " current step @: " << v << endl;
                    
                    vector<Object> ASR_to_find_geometry = ALLlocalMapInGlobal[ALLlocalMapInGlobal.size() - 2]; // last step ASR
                    Object correspond_ref_line = base_line_for_geometry(ASR_to_find_geometry, AllGlobalRobotPs[AllGlobalRobotPs.size() - 2], last_region_exit);
                    correspond_ref_line.reverse();
                    
                    
                    //find the geometry in current view
                    vector<Object> project_geometry = projectingTheView(current_regions[current_regions.size()-3], correspond_ref_line, current_regions[current_regions.size()-3][3], 1);
                    
                    //find the exit on MFIS
                    vector<Object> group1 = collect_object_base_side(transformed_view, project_geometry[0]);
                    vector<Object> group2 = collect_object_base_side(transformed_view, project_geometry[1]);
                    
                    Exit tracked_exit = most_constrain_gap(group1, group2);
                    
                    Object convert1, convert2;
                    
                    convert1.set(tracked_exit.X1(), tracked_exit.Y1(), tracked_exit.X2(), tracked_exit.Y2(), 0);
                    convert2.set(all_chunks_region_exit[0].get_exits()[all_chunks_region_exit[0].get_exits().size()-3].X1(), all_chunks_region_exit[0].get_exits()[all_chunks_region_exit[0].get_exits().size()-3].Y1(),
                                      all_chunks_region_exit[0].get_exits()[all_chunks_region_exit[0].get_exits().size()-3].X2(), all_chunks_region_exit[0].get_exits()[all_chunks_region_exit[0].get_exits().size()-3].Y2(), 0);
                    
                    convert1.reverse();
                    vector<Object> adjust_early_mfis = projectingTheView(all_chunks[0], convert1, convert2,  1);
                    project_memory = adjust_early_mfis;
                    
                    vector<Exit> temp_plot;
                    temp_plot.push_back(tracked_exit);
                    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/robot_and_view_and_exit-", v, ".png");                   
                        plotObjectsOf3KindswithExits(mfisFileName, transformed_view, project_geometry, temp_plot);
                        waitHere();
                    //project all information of MFIS1 onto current MFIS
                    
                    //reset all flags
                    familiar_flag = 1;
                    function_switch_Flag = 1;
                    expectation_flag = 0;
                    
                }
                
                if(project_memory.size() != 0)
                {
                    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/robot_and_view_and_exit-", v, ".png");                   
                    plotObjectsOf3Kinds(mfisFileName, MFIS, project_memory, currentRobotPositionInMFIS);
                }
            }*/
     
        } 
        else
        {
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
            PreLandmarks = computeRe.getTargetObjects();
        }

                    
        if((computedOutput.getChunkFlag() == 1) && (function_switch_Flag == 1))//&&((v - last_Chunk_step) > 10))    
        {

                
                //int cnt = 0;
                regions_exits = bounded_space_version2(AllGlobalRobotPs, ALLlocalMapInGlobal);
                all_chunks_region_exit.push_back(regions_exits);
                
                environment_map = region_geometry(regions_exits.get_regions_boundary(), regions_exits.get_exits());
                
                //form geometry 
                adjust_info = update_geo_map(ALLlocalMapInGlobal, AllGlobalRobotPs);
                sides_of_geo =  getBoundingBox(adjust_info.first);
                sides_of_geo.pop_back();
                
                current_regions.push_back(sides_of_geo);
                //clusters =  cluster_objects_follow_path(adjust_info.first, sides_of_geo);
                sprintf(mfisFileName, "%s%d%s", "Maps/Offline/EnduringMap-", v, ".png");                   
                //plotObjectsOf3Kinds(mfisFileName, adjust_info.first, adjust_info.second, current_regions.back());  
                plotObjects(mfisFileName, adjust_info.first, adjust_info.second);  
                
                
                //test for download dataset
                Object path1, path2;
                Object s1, s2, s3, s4;
                vector<Object> temp_path1, temp_path2;
                vector<Object> info1, info2, info3;
                vector<Object> shape1, shape2;
                path1.set(robot_points[0].X(), robot_points[0].Y(), robot_points[10].X(), robot_points[10].Y(), 0);
                path2.set(robot_points[11].X(), robot_points[11].Y(), robot_points.back().X(), robot_points.back().Y(), 0);
                
                temp_path1.push_back(path1);
                temp_path2.push_back(path2); 
                
                for(int m = 0; m < ALLlocalMapInGlobal.size(); m++)
                {
                    if(m <= 10)
                        info1 = addTwoVectorsOfObjects(info1, ALLlocalMapInGlobal[m]);
                    else
                        info2 = addTwoVectorsOfObjects(info2, ALLlocalMapInGlobal[m]);
                }
                
                for(int m = 0; m < info2.size(); m++)
                {
                    if(info2[m].length() < 500)
                    {
                        info2.erase(info2.begin()+m);
                        m--;
                    }
                }
                
                for(int m = 0; m < info1.size(); m++)
                {
                    if(info1[m].length() < 500)
                    {
                        info1.erase(info1.begin()+m);
                        m--;
                    }
                }
                
                //shape1 = makePolygon_Clipper(info1, 100);
                //shape2 = makePolygon_Clipper(info2, 100);
                
                s1.set(2300, 5500, 4800, 4600, 0);
                s3 = shift_line_along_perpendicluar(s1, 9000, 1);
                s2.set(s1.X2(), s1.Y2(), s3.X2(), s3.Y2(),0);
                s4.set(s1.X1(), s1.Y1(), s3.X1(), s3.Y1(),0);
                
                shape2.push_back(s1);
                shape2.push_back(s2);
                shape2.push_back(s3);
                shape2.push_back(s4);
                Global_ASR = shape2;
                
                possible_exits_in_MFIS = crossed_exits_along_path(possible_exits_in_MFIS, adjust_info.second);
                        
                sprintf(mfisFileName, "%s%d%s", "Maps/Offline/EnduringMap-", v, ".png");                   
                plotObjectsOf3Kinds(mfisFileName, temp_path1, info1, shape1);  
                sprintf(mfisFileName, "%s%d%s", "Maps/Offline/EnduringMap*-", v, ".png");                   
                plotObjectsOf3KindswithExits(mfisFileName, MFIS, adjust_info.second, possible_exits_in_MFIS); 
                
                possible_exits_in_MFIS.clear();
            
                //storage current MFIS
                all_chunks.push_back(MFIS);
                last_step_MFIS = MFIS;
                last_Chunk_step = v;
                
                vector<Object> path_in_mfis = pointsToObjects(robot_points);
                all_chunk_path.push_back(path_in_mfis);
                //before_start_number++;

                cout << " size of input: " <<ALLlocalMapInGlobal.size() << endl;
                //reset method 1 here
                MFIS = transformed_view;
                AllGlobalRobotPs.clear();
                ALLlocalMapInGlobal.clear();
                allRobotPositions.clear();
                referenceObjects.clear();
                robot_points.clear();        
                Memories.clear();
                
                allRobotPositions = myrobot.getRobot();
                referenceObjects.push_back(targetObjectsInCV[0]);
                referenceObjects.push_back(targetObjectsInCV[0]);
   
                computedOutput.setChunkFlag(0);
                chunk_cnt++;
                //function_switch_Flag = 2;
                waitHere();
        }
        

        //save lastStep information to update at next step(in case)
        lastStepInfo.setMFIS(MFIS);
        lastStepInfo.setView(currentView);
        lastStepInfo.setTargetObjects(targetObjectsInCV);
        lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
        lastStepInfo.setReferenceObjects(referenceObjects);
        lastStepInfo.setExits(exitsFromCV);
       
    
        if(v == w)
        {
            //compute returning route
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/finalPercptualMap&returnRoute-", set, ".png");
            plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, all_chunks[0], all_chunks.back());
        }
        
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

    //sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-refTH-", referenceNumberTH, ".png");
    cout<<"size:----------------"<<allRobotPositions.size()<<endl;
    //plotObjects(mfisFileName, robotPositionsAtLimitingPoints, MFIS);
    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/finalPercptualMap-", set, ".png");
    //plotObjectsOf3Kinds(mfisFileName, firstAndLastRP, crossedExit, MFIS);
    plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, all_chunks[0], MFIS);

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





