//============================================================================
// Name        : Route map module
// Author      : Wenwang (Lyn)
// Version     : Route map module, associate to GlobalAndRouteMap.cpp 
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

#include "RouteMapModule.h"
#include "GlobalMapModule.h"
#include "PathIntegrationModule.h"
#include "Mapping.H"
#include "Plotting.H"
#include "Transporter.H"
#include "PathPlanning.H"
#include "mfisOp.H"
#include "Mapping.H"
#include "Object.H"


#define PI 3.14159265
using namespace std;

vector<Exit> exits_FromCV;                   //potential exits from current view

vector<Object> trackedObjects_CV;            //tracked surfaces in current view
vector<Object> trackedObjects_PV;            //tracked surfaces in previous view
vector<Object> reference_Objects;            //reference objects for triangulation
vector<Object> MFIS_Global;                  //Global map/MFIS
Transporter recognizedTrackObjects;          //recognised tracked surfaces  

/* Compute route map using PI module
 * transform all individual views onto global coordinate 
 * also includes abstract trackable surfaces & recognised tracked surfaces
 */
void RouteMap(vector<Object> currentView, vector<double> coordTransInfo, 
              double traveledDistance, double robotFacing, int v)
{
        char plotViewFile[100]; //plotting view buffer
        MyRobot myrobot(0, 0);
     
        pair< vector<Object>, vector<Object> > PI_transform; //first is view, another is robot

        //Compute the error-accumulated map by transforming views onto global coordinate/PI
        PI_transform = IntegrateView(currentView, traveledDistance, robotFacing); //computing error map
        
        //finding Exits
        exits_FromCV = findShortestExits(currentView);
        //exitsFromCV = findExits(currentView);

        //finding target objects
        trackedObjects_CV = findTargetObjects(currentView);
        
        if(v > 1) //initialization 
        {
            //cout << " ------ Recognise tracked surfaces between two adjacent steps ------ " << endl;
            //recognizing target/tracked Objects
            recognizedTrackObjects = recognizeTargetObjects(MFIS_Global, trackedObjects_PV, trackedObjects_CV, coordTransInfo, v);
            reference_Objects = recognizedTrackObjects.getReferenceObjects();
        }


        
        //plotting current view with robot position, tracked surface and recognized tracked surfaces
        sprintf(plotViewFile, "%s%d%s", "Maps/Offline/View-", v, ".png");
        plotObjectsOf4Kinds(plotViewFile, currentView, myrobot.getRobot(), trackedObjects_CV, recognizedTrackObjects.getTargetObjects());
        sprintf(plotViewFile, "%s%d%s", "Maps/Offline/GlobalView-", v, ".png");
        plotObjects(plotViewFile, PI_transform.first, PI_transform.second);
 
        
}

void init_variable(vector<Object> currentView)
{
        cout << " ------ Initialisation all variables ------ " << endl;
        MyRobot myrobot(0, 0);
        char mfisFileName[100];
        
        RobotPosition_In_MFIS = myrobot.getRobot();
        All_RobotPositions = RobotPosition_In_MFIS;
        //AllGlobalRobotPs.push_back(allRobotPositions);
        //allOdometricRPosition = currentRobotPositionInMFIS; //just to see odometric robot position
        odometric_RobotPosition_In_MFIS = RobotPosition_In_MFIS; //just to see odometric robot position
        robotPositions_AT_LimitingPoints = RobotPosition_In_MFIS;

        //initializing MFIS
        MFIS_Global = currentView;
        
        sprintf(mfisFileName, "%s", "Maps/Offline/MFIS-1.png");
        plotObjects(mfisFileName, myrobot.getRobot(), MFIS_Global);
        
        trackedObjects_CV = findTargetObjects(currentView);
        
        //reference objects
        trackedObjects_PV = trackedObjects_CV;
        reference_Objects.push_back(trackedObjects_PV[0]); 
        reference_Objects.push_back(trackedObjects_PV[0]);
        
        limiting_Points.push_back(1);
        routeMap_ConnLP.push_back(Object(0, 0, 0, 0, 1));
        
}
