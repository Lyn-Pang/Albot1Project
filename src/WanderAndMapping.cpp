//============================================================================
// Name        : wander&mapping.cpp
// Author      : Modified by wenwang
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

#define DEGTORAD 3.1415926/180
#define RADTODEG 180/3.1416926

using namespace std;

//movement function
vector<double> moveToTheDestination(ArRobot& robot, double distance, double angle, int viewNumber, vector<int> limitingPoints);
void wanderAndobstacle();

bool computeASR = false;
int v = 1;

//variables
Transporter recognizedTargetObjects, computedOutput; //compute global map
vector <double> coordTransInfo; //displacement parameters
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
ASR currentASR; //ASR
ASRNetwork perceptualMap;
int ASRNumber = 1;

vector<ASR> places;

MyRobot myrobot(0, 0); //initial myrobot
vector<double> distang;
char coordTransFileName[100];
char viewFileName[80], mfisFileName[80];
vector <Object> currentView;
vector<Object> MFIS;
bool exitCrossed = false;

double angleToMove; 
double distanceToMove;
double lastDistance = 0;
double lastDegree = 0;
double oldDistance = 0;
double oldAngle = 0;

double kalman_dist = 0;  //kalman filter distance
double kalman_angle = 0; //kalman filter angle 

unsigned char visual_triang_flag = 0; //visual triangulation flag
 
ArRobot robot;
ArSick sick;

int main(int argc, char** argv)
{

    Aria::init();
    ArSimpleConnector connector(&argc, argv);
    //ArRobot robot;
    //ArSick sick;
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
    
    // turn on the motors, turn off amigobot sounds
    robot.enableMotors();
    robot.comInt(ArCommands::SOUNDTOG, 0);

    // add a set of actions that combine together to effect the wander behavior
    ArActionStallRecover recover;
    ArActionBumpers bumpers;
    ArActionAvoidFront avoidFrontNear("Avoid Front Near", 225, 0, 15);
    ArActionAvoidFront avoidFrontFar;
    ArActionConstantVelocity constantVelocity("Constant Velocity", 150);
    ArActionDriveDistance driveDist;
    
    
    limitingPoints.push_back(1);
    currentRobotPositionInMFIS = myrobot.getRobot();
    allRobotPositions = currentRobotPositionInMFIS;
    robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;

    //for routeMap
    previousRobotPositionInMFIS = currentRobotPositionInMFIS;
    lastRouteMapNode = currentRobotPositionInMFIS[6];
    tempLastRouteMapNode = lastRouteMapNode;
    routeMap.push_back(lastRouteMapNode);
    routeMapForOneASR.push_back(lastRouteMapNode);

    //save coordinate transformation info
    distang.push_back(0); //rdist);
    distang.push_back(0); //rangle);

    ArUtil::sleep(2000); //delay for connecting the laster and robot hardwares

    //scanning for the current view
    currentView = scanAndSaveView(sick, v);

    //sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
    //plotObjects(viewFileName, currentView);

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
    cout << "..........MFIS.........." << endl;

    //initializing Perceptual Map
    objectOfCurrentASR = currentView;
    currentASR.setASRObjects(objectOfCurrentASR);
    currentASR.setASRExit1(Object(-500, 0, 500, 0));
    currentASR.setASRID(1);
    lineOfSitePoint = currentRobotPositionInMFIS[6];
    currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
    perceptualMap.setCurrentASR(currentASR);

    sprintf(mfisFileName, "%s%d%s", "Maps/PM-", v, ".png");
    plotObjects(mfisFileName, allRobotPositions, MFIS);
    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/GUI-", v, ".png");
    plotObjects(mfisFileName, allRobotPositions, MFIS);
    sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
    plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInPV);

 
    crossedExit.push_back(currentRobotPositionInMFIS[6]); //consider home as a exit

    //cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    

    
    while(1)
    {

        wanderAndobstacle();
        ArUtil::sleep(2500);

        robot.stop();

        robot.lock();
        distanceToMove = robot.getOdometerDistance() - lastDistance;
        angleToMove = robot.getTh() - lastDegree;
        robot.unlock();
        cout << "intermediately traversed dist : " << distanceToMove << " , angle : " << angleToMove;

        v++;

        robot.lock();
        lastDistance = robot.getOdometerDistance();
        lastDegree = robot.getTh();
        robot.unlock();

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
          //for plot on GUI
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");
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
            if(distanceError > 400.0 or angleError > 5.0) 
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
                MFIS = lastStepInfo.getMFIS();
                currentView = lastStepInfo.getView();
                referenceObjects = lastStepInfo.getReferenceObjects();
                currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();
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
            else //adjacent step
                if ((v - limitingPoints.back()) == 1) //update at this step
                {
                        cout << "Updating at this step" << endl;
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

        //save lastStep information to update at next step(in case)
        lastStepInfo.setMFIS(MFIS);
        lastStepInfo.setView(currentView);
        lastStepInfo.setTargetObjects(targetObjectsInCV);
        lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
        lastStepInfo.setAllRobotPositions(allRobotPositions);
        lastStepInfo.setReferenceObjects(referenceObjects);
        lastStepInfo.setExits(exitsFromCV);

        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", v, ".png");
        plotObjects(mfisFileName, allRobotPositions, MFIS);

        cout << " Complete Process and Mapping !!! " << endl;
   }
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

    //get the actual distance traveled for this step
    robot.lock();
    //DATA HERE your name
    double traveledDistance = robot.getOdometerDistance() - oldDistance;
    double turnedAngle = robot.getTh() - oldAngle;
    robot.unlock();
    
    //robot.lock();
    //oldDistance = robot.getOdometerDistance();
    //oldAngle = robot.getTh();
    //robot.unlock();

    vector<double> distang;
    
    distang.push_back(distance);
    distang.push_back(angle);

    cout << "Traveled Dist: " << traveledDistance << " turned Angle: " << turnedAngle << endl;
	
    return distang;
}

void wanderAndobstacle()
{
    double dist, angle, myTurnAmount;
    bool myUseTableIRIfAvail = true;
    int myTurning = 0; // 1 for turning left, 0 for not turning, -1 for turning right

    double  myAvoidVel = 150;
    ArSectors myQuadrants;
 
    dist = (robot.checkRangeDevicesCurrentPolar(-70, 70, &angle) 
            - robot.getRobotRadius());
   
    //  printf("%5.0f %3.0f ", dist, angle);
  
    if (dist > 500 && 
       (!myUseTableIRIfAvail || 
        (myUseTableIRIfAvail && !robot.hasTableSensingIR()) || 
        (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
        !robot.isLeftTableSensingIRTriggered() &&
        !robot.isRightTableSensingIRTriggered())))
    {
        if (myTurning != 0)
        {
            robot.setDeltaHeading(0);
            myTurning = 0;
        }
        else
        {
            myTurning = 0;
        }
    }
   
    //  printf("Avoiding ");
   
    if (myTurning == 0)
    {
        if (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
            robot.isLeftTableSensingIRTriggered())
          myTurning = 1;
        else if (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
                 robot.isRightTableSensingIRTriggered())
          myTurning = -1;
        else if (angle < 0)
          myTurning = 1;
        else
          myTurning = -1;
        myTurnAmount = 20;
        myQuadrants.clear();
    }
 
    myQuadrants.update(robot.getTh());
    if (myTurning && myQuadrants.didAll())
    {
        myQuadrants.clear();
        myTurnAmount /= 2;
        if(myTurnAmount == 0)
          myTurnAmount = 20;
    }
    robot.setDeltaHeading(myTurning * myTurnAmount);
 
    if (dist > 500/2 && 
       (!myUseTableIRIfAvail || 
        (myUseTableIRIfAvail && !robot.hasTableSensingIR()) || 
        (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
        !robot.isLeftTableSensingIRTriggered() &&
        !robot.isRightTableSensingIRTriggered())))
    {
        robot.setVel(myAvoidVel * dist / 500);
    }
    else
    {
        robot.setVel(0);
    }
}



