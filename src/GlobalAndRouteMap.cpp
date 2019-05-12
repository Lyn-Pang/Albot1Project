//============================================================================
// Name        : GlobalAndRouteMap
// Author      : Wenwang (Lyn)
// Version     : Systematiclly module of whole routine 
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

#include "Object.H"
#include "mfisOp.H"
#include "Plotting.H"
#include "readAndwriteASCII.H"

#include "RouteMapModule.h"
#include "GlobalMapModule.h"
#include "PathIntegrationModule.h"


//#define PI 3.14159265
using namespace std;

/* In this programme, this is divided into different modules
 * which inludes, route map/movement process, MFIS/Global map
 * And Chunk process, further probably includes homing and planner
 */

char ctFileName[100];   //displacement file
char viewFileName[100]; //view file
const char* levelName = "inputData/level"; //file name & path
const char* surfaceName = "/surfaces-";    //

int main()
{
        int v, w, r, level, set, saveFrom, saveTo;
        MyRobot myrobot(0, 0);
        
        //input by user to choose dataset
        cout << "Which level?? ";
        cin >> level;
        cout << "Which dataset?? ";
        cin >> set;
        cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
        cin >> v;
        cin >> w;

        /* basic input dataset are views 
         * and displacement information 
         * angle & distance moved every step
         */
        
        //initialization variables
        vector<Object> currentView;    //current view
        vector<double> coordTransInfo; //current step displacement
        double traveledDistance = 0;   //traveled distance
        double robotFacing = 0;        //total angle turned

        //ofstream outFileForRefError("Maps/Offline/Localization", ios::out);
        

        while(v < w)
        { 
            
            cout << "\n\033[1;34m******************Main Process is computed at step @" << v << "********************\033[0m" << endl << endl;
            
                //read displacement information from file 
                sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", v);
                coordTransInfo = readCoordTrans(ctFileName);

                //read view from files  
                sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
                currentView = readASCII(viewFileName);

                traveledDistance = traveledDistance + coordTransInfo[0]; //accumulate distance traversed
                robotFacing = robotFacing + coordTransInfo[1];           //accumulate angle
                
                if(v == 1)
                {
                    init_variable(currentView); //initia variables
                    currentView = tagObjectsAsSideAndViewNumber(currentView, v);
                }
        
                
                //route map process module
                RouteMap(currentView, coordTransInfo, traveledDistance, robotFacing, v);
                
                //MFIS/Global map process module 
                if((recognizedTrackObjects.getTargetObjects().size() < 4)
                    && (v > 1))
                {
                    computeGlobalMap(currentView, coordTransInfo, v);    
                }   
                else
                {
                    if((recognizedTrackObjects.getTargetObjects().size() >= 4)
                    && (v > 1))
                        trackedObjects_PV = recognizedTrackObjects.getTargetObjects();
                }
                
                
                //reset all module after updating previous view
                if(update_previous_view == 1) 
                {
                    v -= 1;
                    update_previous_view = 0;
                    
                    //record last step information
                    Record_lastStepInfo(last_step_view);
                }
                else
                {
                    //record last step information
                    Record_lastStepInfo(currentView);
                }
                
                //Chunk process module
                
                
                
                v++;
        }
        
        /*------------------------ All Maps Computed finish ! ------------------*/
        cout << "MFIS size " << MFIS_Global.size() << " all robot position size " << All_RobotPositions.size() << endl;
        vector<Object> firstAndLastRP = myrobot.getRobot();
        for (int i = 0; i < RobotPosition_In_MFIS.size(); i++)
        {
            firstAndLastRP.push_back(RobotPosition_In_MFIS[i]);
        }

        sprintf(viewFileName, "%s%d%s", "Maps/Offline/MFIS-refTH-", 0, ".png");
        cout<<"size:----------------"<<All_RobotPositions.size()<<endl;
        plotObjects(viewFileName, robotPositions_AT_LimitingPoints, MFIS_Global);
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/finalPercptualMap-", set, ".png");
        plotObjectsOf3Kinds(viewFileName, firstAndLastRP, crossed_Exit, MFIS_Global);

        //abstractRouteMap(MFIS, robotPositionsAtLimitingPoints, updatingPoints, currentRobotPositionInMFIS);
        //keyInfoOfCompuetedPM(mappingInfoFileName, ASRNumber, exitPoints, lostPoints, limitingPoints,
        //        badLocalization, failedToRecognizeRef, referenceNumberTH, level, set);
        //cout << levelName << level << " set " << set << endl;
        //cout << "odo " << odoLocalizationUsed << endl;
        cout << endl << " Visual Triangulation updating: " << Triangulation_counter 
                     << " times. Path Integration updating: " << PI_counter << " times." << endl;
        
        cout << endl << " Traveled distance: " << traveledDistance << endl;
        /*----------------------------------------------------------------------*/
        
        /**
         * after processing exploration
         * planner module and return execution
         */
    
        //planner module

        //return home module 
        
}


