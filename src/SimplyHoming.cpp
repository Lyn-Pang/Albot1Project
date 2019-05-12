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
#include <fstream>

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

#include "Aria.h"
#include "Laser2Surface.H"
#include "RobotFuncs.H"
#include "ChunksOp.H"
#include "Returning.h"




#define PI 3.14159265


using namespace std;
vector<double> moveToTheDestination(ArRobot&, double distance, double angle, int viewNumber, vector<int> limitingPoints);
void Init_variables();

//variables
Transporter recognizedTargetObjects, computedOutput;
vector <double> coordTransInfo;
vector<Object> targetObjectsInPV, targetObjectsInCV;
vector<Object> allTargetObjectsInPV;
vector<Object> referenceObjects, odometricReferenceObject;

vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
vector<Object> previousRobotPositionInMFIS;
vector<Object> routeMap;
vector<Object> routeMapForOneASR;
vector<Object> routeMapConnLP; //route map simply connecting limiting points
vector<vector<Object> > routeMapForallASR;
Object lastRouteMapNode, tempLastRouteMapNode;
vector <Object> allRobotPositions;
vector<Object> robotPositionsAtLimitingPoints;
Transporter objectForPlaceRecognition;
Transporter loopClosingInfo;
Transporter lastStepInfo;
vector<Object> refObjectForLoopClosing;
string environmentType = "unknown";
vector<int> lostPoints, limitingPoints, exitPoints;
Object lineOfSitePoint;
Object lastLocomotion;
vector<Object> wholeRoute;
double traveledDistance = 0;
double angleError, distanceError;

vector<Exit> exitsFromCV;
vector<Object> exitsFromCVInMFIS, allExitsInMFIS, crossedExit;

vector<Object> objectOfCurrentASR;
ASR currentASR;
ASRNetwork perceptualMap;
int ASRNumber = 1;

vector<ASR> places;
char coordTransFileName[100];
char viewFileName[80], mfisFileName[80];
double angleToMove;
double distanceToMove;

vector <Object> currentView;
vector<Object> MFIS;

vector<Object> current_LM, last_LM; //current local map & last local map
vector<Object> currentRP_In_LM;     //current robot position in local map
unsigned char localMap_Flag = 0;    //local map generation flag 0 -- no 1 -- yes
Map Local_Map; //individual local map
vector<Map> LocalMaps; //a list of local maps 

int main(int argc, char **argv) 
{

    //CODE ADDED HERE BY PASCAL
    remove("Maps/OdometerData.txt");
	
    srand(getpid());

    cout << "argc: " << argc << " argv: " << argv << endl;

    Aria::init();
    ArSimpleConnector connector(&argc, argv);
    ArRobot robot;
    ArSick sick;
    if (!connector.parseArgs() || argc > 1) 
    {
        Aria::logOptions();
        Aria::shutdown();
        Aria::exit(1);
    }

    robot.addRangeDevice(&sick);
    // Try to connect, if we fail exit
    if (!connector.connectRobot(&robot)) 
    {
        cout << "Could not connect to robot... exiting" << endl;
        Aria::shutdown();
        return 1;
    }
    // Turn on the motors, turn off amigobot sounds
    robot.runAsync(true);
    robot.comInt(ArCommands::ENABLE, 1);
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.lock();
    robot.clearDirectMotion();
    robot.unlock();
    // Set up the laser
    connector.setupLaser(&sick);
    sick.runAsync();
    if (!sick.blockingConnect()) 
    {
        cout << "Could not connect to SICK laser... exiting" << endl;
        robot.disconnect();
        Aria::shutdown();
        return 1;
    }
    ArUtil::sleep(1500);
    cout << "----------------Connected to robot and laser-------------------" << endl;

    int v; 
    v = 1;

    bool computeASR = false;

    MyRobot myrobot(0, 0);
    currentRobotPositionInMFIS = myrobot.getRobot();
    allRobotPositions = currentRobotPositionInMFIS;
    robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;
    limitingPoints.push_back(1);

    //for routeMap
    previousRobotPositionInMFIS = currentRobotPositionInMFIS;
    lastRouteMapNode = currentRobotPositionInMFIS[6];
    tempLastRouteMapNode = lastRouteMapNode;
    routeMap.push_back(lastRouteMapNode);
    routeMapForOneASR.push_back(lastRouteMapNode);

    //save coordinate transformation info
    vector<double> distang;
    distang.push_back(0); //rdist);
    distang.push_back(0); //rangle);

    sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", v);
    writeASCII(distang, 2, coordTransFileName);

    //scanning for the current view
    currentView = scanAndSaveView(sick, v);

    //finding target objects
    targetObjectsInPV = findTargetObjects(currentView);
    referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
    referenceObjects.push_back(targetObjectsInPV[0]);

    //tagging side and view number
    currentView = tagObjectsAsSideAndViewNumber(currentView, 1);

    for (unsigned int i = 0; i < currentView.size(); i++)
    {
        currentView[i].setASRNo(1);
    }

    //initializing MFIS
    MFIS = currentView;
    current_LM = last_LM = currentView; //the first local map
    Local_Map.setMap(current_LM);
    Local_Map.setMapID(v);

    //initializing Perceptual Map
    objectOfCurrentASR = currentView;
    currentASR.setASRObjects(objectOfCurrentASR);
    currentASR.setASRExit1(Object(-500, 0, 500, 0));
    currentASR.setASRID(1);
    lineOfSitePoint = currentRobotPositionInMFIS[6];
    currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
    perceptualMap.setCurrentASR(currentASR);

    plotObjects(mfisFileName, allRobotPositions, MFIS);
    plotObjects(mfisFileName, allRobotPositions, MFIS);
    sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
    plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInPV);

    bool exitCrossed = false;
    crossedExit.push_back(currentRobotPositionInMFIS[6]); //consider home as a exit

    cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    char n = 'y';


	//CHANGES DONE BY PASCAL CROSS
	std::ofstream outfile;
	outfile.open("Maps/OdometerData.txt", std::ios_base::app);
	outfile << 1;
	outfile << endl;
	outfile << 0;
	outfile << endl;
	outfile << 0;
	outfile << endl;
	outfile << "true";
	outfile << endl;
        
        
    ///// Here, this function is to ask whether let the robot carry on working
    while (n != 'n' && n != 'N') 
    {
        v++;

        cout << endl << "How much to turn? ";
        cin >> angleToMove;
        cout << "How much to move? ";
        cin >> distanceToMove;

        cout << "Now @ step " << v << endl;
        coordTransInfo = moveToTheDestination(robot, distanceToMove, angleToMove, v, limitingPoints);
        traveledDistance = traveledDistance + coordTransInfo[0];

        //scanning for the current view
        currentView = scanAndSaveView(sick, v);

        cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;

        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);
        //tagging sides and view number
        currentView = tagObjectsAsSideAndViewNumber(currentView, v);

        //saving current view
        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
        plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);
        sprintf(viewFileName, "%s", "Maps/CurrentView.png");
        plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
        referenceObjects = recognizedTargetObjects.getReferenceObjects();

        //localization using odometer
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

        //update MFIS and ASR if necessary
        if (recognizedTargetObjects.getTargetObjects().size() < 3) 
        {
            cout << endl << "Updating situation " << endl;
            if ((v - limitingPoints.back()) > 1) //update at last step //retriving info to update at last step
            { 
                cout << "Updating at last step. i.e. @ " <<v-1<< endl << endl;
                v = v - 1;

                //updating at last step
                computedOutput = updatePerceptualMapATPlace(places, lastStepInfo.getMFIS(), lastStepInfo.getView(),
                        lastStepInfo.getRobotPosition(), lastStepInfo.getAllRobotPositions(), lastStepInfo.getReferenceObjects(),
                        v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                MFIS = computedOutput.getView();
                places = computedOutput.getASRs();
                targetObjectsInPV = computedOutput.getTargetObjects();
                allRobotPositions = lastStepInfo.getAllRobotPositions();
                limitingPoints.push_back(v); //limiting/updating points just for printing at the end

                v++;
                //now processing current view
                //recognizing target Objects
                recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
                referenceObjects = recognizedTargetObjects.getReferenceObjects();

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

                if (recognizedTargetObjects.getTargetObjects().size() < 3) 
                {
                        //updating at current step
                        computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                                allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                        MFIS = computedOutput.getView();
                        places = computedOutput.getASRs();
                        targetObjectsInPV = computedOutput.getTargetObjects();
                        limitingPoints.push_back(v); //limiting/updating points just for printing at the end
                } 
                else 
                {
                     targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
                }

            } 
            else 
                if ((v - limitingPoints.back()) == 1) //update at this step
                {

                        //updating at current step
                        computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                                allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                        MFIS = computedOutput.getView();
                        places = computedOutput.getASRs();
                        targetObjectsInPV = computedOutput.getTargetObjects();
                        limitingPoints.push_back(v); //limiting/updating points just for printing at the end
                }
        } 
        else
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
                      
        cout << "\n\033[1;34m******************PM is computed at step" << v << "********************\033[0m" << endl << endl;



        //** Local Map processing **//
        
        if(interSectWithPtoOrig(current_LM, Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1())) == true)
        {
            localMap_Flag = 1;
            //record useful information for homing
            Local_Map.setMap(current_LM);
            Local_Map.setMapID(v);
            Local_Map.setExitPositon(currentRP_In_LM);
            Local_Map.setHeading(currentRP_In_LM[7].getAngleWithXaxis());
            LocalMaps.push_back(Local_Map);
            
            //new local map 
            current_LM = currentView;
            //initialise all variables
            Init_variables();
        }
        else
        {
            localMap_Flag = 0;
            
            //save lastStep information to update at next step(in case)
            lastStepInfo.setMFIS(MFIS);
            lastStepInfo.setView(currentView);
            lastStepInfo.setTargetObjects(targetObjectsInCV);
            lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
            lastStepInfo.setAllRobotPositions(allRobotPositions);
            lastStepInfo.setReferenceObjects(referenceObjects);
            lastStepInfo.setExits(exitsFromCV);
        }
        
        cout << "Take another step? (y/n) ";
        cin >> n;
    }

    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
    pointingExperiment(MFIS,allRobotPositions,currentRobotPositionInMFIS);

    if (n == 'n' || n == 'N') 
    {
        setHeading(robot, 180);//turn over for homing
        cout << "\n\033[1;34m****************** Homing Navigation is ready !! ********************\033[0m" << endl << endl;
        int cnt = LocalMaps.size() - 1;
        
        //plan and decision 
        do
        {
            Point pos;
            //pos.set(LocalMaps[cnt].getExitPosition()[6].X1(), LocalMaps[cnt].getExitPosition()[6].Y1());
            pos = LocalMaps[cnt].getOrientation();
            CollisionAvoidandMove(robot, sick, pos, cnt);
            cnt--;
        }while(cnt >= 0);
    }
    return 0;
}

void Init_variables()
{
        //variables
        Transporter recognizedTargetObjects, computedOutput;
        vector <double> coordTransInfo;
        vector<Object> targetObjectsInPV, targetObjectsInCV;
        vector<Object> allTargetObjectsInPV;
        vector<Object> referenceObjects, odometricReferenceObject;

        vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
        vector<Object> previousRobotPositionInMFIS;
        vector<Object> routeMap;
        vector<Object> routeMapForOneASR;
        vector<Object> routeMapConnLP; //route map simply connecting limiting points
        vector<vector<Object> > routeMapForallASR;
        Object lastRouteMapNode, tempLastRouteMapNode;
        vector <Object> allRobotPositions;
        vector<Object> robotPositionsAtLimitingPoints;
        Transporter objectForPlaceRecognition;
        Transporter loopClosingInfo;
        Transporter lastStepInfo;
        vector<Object> refObjectForLoopClosing;

        vector<int> lostPoints, limitingPoints, exitPoints;
        Object lineOfSitePoint;
        Object lastLocomotion;
        vector<Object> wholeRoute;
        double traveledDistance = 0;
        double angleError, distanceError;

        vector<Exit> exitsFromCV;
        vector<Object> exitsFromCVInMFIS, allExitsInMFIS, crossedExit;

        vector<Object> objectOfCurrentASR;
        ASR currentASR;
        ASRNetwork perceptualMap;
        int ASRNumber = 1;
        vector<ASR> places;
        double angleToMove;
        double distanceToMove;

        vector<Object> current_LM, last_LM; //current local map & last local map
        vector<Object> currentRP_In_LM;     //current robot position in local map
        unsigned char localMap_Flag = 0;    //local map generation flag 0 -- no 1 -- yes
        Map Local_Map; //individual local map
}

// movement funciton in Guided mode
vector<double> moveToTheDestination(ArRobot& robot, double distance, double angle, int viewNumber, vector<int> limitingPoints) 
{

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(2000);

    //save obometer and orientation angle before moving       
    robot.lock();
    double oldDistance = robot.getOdometerDistance();
    double oldAngle = robot.getTh();
    robot.unlock();

    //movement execution
    setHeading(robot, angle);
    moveDistance(robot, distance);
    angle = 0;

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(2000);

    //get the actual distance traveled for this step
    robot.lock();
	//DATA HERE your name
    double traveledDistance = robot.getOdometerDistance() - oldDistance;
    double turnedAngle = robot.getTh() - oldAngle;
    robot.unlock();

    vector<double> distang;
    distang.push_back(traveledDistance); //rdist);
    distang.push_back(turnedAngle); //rangle);
    char coordTransFileName[100];
    sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", viewNumber);
    writeASCII(distang, 2, coordTransFileName);

    cout << "Traveled Dist: " << traveledDistance << " turned Angle: " << turnedAngle << endl;
	
	//CHANGES DONE BY PASCAL CROSS
	std::ofstream outfile;
	outfile.open("Maps/OdometerData.txt", std::ios_base::app);
	outfile << viewNumber;
	outfile << endl;
	outfile << traveledDistance;
	outfile << endl;
	outfile << turnedAngle;
	outfile << endl;
	if(std::find(limitingPoints.begin(), limitingPoints.end(), viewNumber) != limitingPoints.end()) {
		outfile << "true";
	} else {
		outfile << "false";
	}
	outfile << endl;
	
    return distang;
}

