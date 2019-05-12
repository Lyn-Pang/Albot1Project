//============================================================================
// Name        : MFIS/Global map module
// Author      : Wenwang (Lyn)
// Version     : Global map module, associate to GlobalAndRouteMap.cpp 
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
#include "Plotting.H"
#include "Transporter.H"
#include "GeometricOp.H"
#include "PerceptualMapping.H"
#include "GlobalMapModule.h"
#include "RouteMapModule.h"
#include "PathIntegrationModule.h"


#define PI 3.14159265
using namespace std;

bool exit_Crossed = false;                     //flag of crossing an exit

unsigned char update_previous_view = 0;        //flag of updating previous view

int ASR_Number = 1;  
int Triangulation_counter = 0;                 //visual triangulation counter
int PI_counter = 0;                            //path integration counter
    
Object path_From_LimitingPoint;                //a path segment between two adjacent updating positions

vector<int> limiting_Points;                   //ID s of updating view
vector<ASR> places_ASR;                        //ASR places

vector<Object> crossed_Exit;                   //
vector<Object> RobotPosition_In_MFIS;          //Triangulation current robot global position
vector<Object> odometric_RobotPosition_In_MFIS;//odometric/PI robot global position
vector<Object> All_RobotPositions;             //all global robot position(for plotting)
vector<Object> robotPositions_AT_LimitingPoints; //
vector<Object> routeMap_ConnLP;                //whole path segments between updating positions
vector<Object> odometric_ReferenceObject;

vector<Object> last_step_view;                 //last step view from last step information

vector<Point> updatingPoints;                  //updating positions (points)

Transporter lastStep_Info;                     //last step information
Transporter computed_Output;                   //updated global map

/* compute global map using ASR(remembered views)
 * which includes modification of object & deletion process
 */
void computeGlobalMap(vector<Object> currentView, vector<double> coordTransInfo, int v)
{
    cout << "\n\033[1;34m******************Global Map is computed at step @" << v << "********************\033[0m" << endl << endl;
    
            MyRobot myrobot(0,0);
            char mfisFileName[100];
            
            if((v - limiting_Points.back()) > 1)
            {
                    Triangulation_reference(); //get triangulation reference
                    currentView = last_step_view;
                    v = v - 1; 
                    update_previous_view = 1;
                    Triangulation_counter++;
            }
            else
            {
                    if ((v - limiting_Points.back()) == 1) //update at this step
                    {
                        //Check whether these two ref object are quite different        
                        //if(abs(reference_Objects[1].length()/reference_Objects[0].length()) > 3)
                        //{
                        //    reference_Objects.clear(); // do not use these two ref object
                        //}
                        
                        if (reference_Objects.size() == 0)
                        {
                                PathInte_reference(coordTransInfo);   //get PI reference
                                PI_counter++;
                        }
                        else
                            Triangulation_counter++; 
                    }  
            }
            
            //cout << "reference_Objects size : " << reference_Objects.size() << endl;
        
            //localization 
            RobotPosition_In_MFIS = myrobot.inMFIS(reference_Objects[0], reference_Objects[1], reference_Objects[0].getKP());
            //All_RobotPositions.push_back(RobotPosition_In_MFIS);

            All_RobotPositions = addTwoVectorsOfObjects(All_RobotPositions, RobotPosition_In_MFIS); 

            //routeMap
            path_From_LimitingPoint.set(routeMap_ConnLP.back().X2(), routeMap_ConnLP.back().Y2(),
                    RobotPosition_In_MFIS[6].X1(), RobotPosition_In_MFIS[6].Y1(), v);
             
            routeMap_ConnLP.push_back(path_From_LimitingPoint);

           
            //all updating position
            robotPositions_AT_LimitingPoints = addTwoVectorsOfObjects(robotPositions_AT_LimitingPoints, RobotPosition_In_MFIS);
           
            updatingPoints.push_back(Point(RobotPosition_In_MFIS[6].X1(), RobotPosition_In_MFIS[6].Y1()));

            if(v == 2)
            {
                cout << " Test programme " << endl;
                cout << " global map size : " << MFIS_Global.size() << endl;
                cout << " ref object isze : " << reference_Objects.size() << endl;
                reference_Objects[0].display();
                reference_Objects[1].display();
                waitHere();
            
            }
            
            //earlier approach
            computed_Output = updatePerceptualMapATPlace(places_ASR, MFIS_Global, currentView, RobotPosition_In_MFIS,
                  All_RobotPositions, reference_Objects, v, ASR_Number, exit_Crossed, crossed_Exit, routeMap_ConnLP);
            
            //modified by Lyn Pang
            //new approach
            //computed_Output = updatePerceptualMapATPlaceDeletion(places_ASR, MFIS_Global, currentView, RobotPosition_In_MFIS,
            //      All_RobotPositions, reference_Objects, v, ASR_Number, exit_Crossed, crossed_Exit, routeMap_ConnLP);
            
            MFIS_Global = computed_Output.getView();
            
            places_ASR = computed_Output.getASRs();
            trackedObjects_PV = computed_Output.getTargetObjects();
            limiting_Points.push_back(v); //limiting/updating points just for printing at the end      
            
            //to print local and global map
            sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
            plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), RobotPosition_In_MFIS, MFIS_Global);
                         
            //recognizedTargetObjectInPV = recognizedTargetObjects.getTargetObjects();
                    
}

//obtain path integration reference object
void PathInte_reference(vector<double> coordTransInfo)
{
        MyRobot myrobot(0,0);
        //localization using odometer
        Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, RobotPosition_In_MFIS[6], 1);
        aLine.setKP(1);
        odometric_ReferenceObject.clear();
        odometric_ReferenceObject.push_back(aLine);
        odometric_ReferenceObject.push_back(All_RobotPositions[6]);
        odometric_RobotPosition_In_MFIS = myrobot.inMFIS(odometric_ReferenceObject[0], odometric_ReferenceObject[1], odometric_ReferenceObject[0].getKP());
        reference_Objects = odometric_ReferenceObject;
        //lostPoints.push_back(v); //just for printing at the end
}

//obtain triangulation reference objec
void Triangulation_reference()
{
        MFIS_Global = lastStep_Info.getMFIS();
        last_step_view = lastStep_Info.getView();
        reference_Objects = lastStep_Info.getReferenceObjects();
        RobotPosition_In_MFIS = lastStep_Info.getRobotPosition();
}


void Record_lastStepInfo(vector<Object> currentView)
{
        //save lastStep information to update at next step(in case)
        lastStep_Info.setMFIS(MFIS_Global);
        lastStep_Info.setView(currentView);
        lastStep_Info.setTargetObjects(trackedObjects_CV);
        lastStep_Info.setRobotPosition(RobotPosition_In_MFIS);
        lastStep_Info.setReferenceObjects(reference_Objects);
        lastStep_Info.setExits(exits_FromCV);
}


