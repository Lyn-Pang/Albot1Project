/* 
 * File:   GlobalMapModule.h
 * Author: arthur
 *
 * Created on April 11, 2017, 1:27 PM
 */

#include <vector>
#include <stdio.h>

#include "Object.H"
#include "Transporter.H"

using namespace std;
   


#ifndef GLOBALMAPMODULE_H
#define GLOBALMAPMODULE_H


extern bool exit_Crossed;

extern unsigned char update_previous_view;               //update previous view

extern int ASR_Number;
extern int Triangulation_counter;                        //visual triangulation counter
extern int PI_counter;                                   //path integration counter
    
extern Object path_From_LimitingPoint;                   //a path segment between two adjacent updating positions

extern vector<int> limiting_Points;                      //ID s of updating view
extern vector<ASR> places_ASR;                           //ASR places

extern vector<Object> crossed_Exit;                      //
extern vector<Object> RobotPosition_In_MFIS;             //Triangulation current robot global position
extern vector<Object> odometric_RobotPosition_In_MFIS;   //odometric/PI robot global position
extern vector<Object> All_RobotPositions;                //all global robot position(for plotting)
extern vector<Object> robotPositions_AT_LimitingPoints;  //
extern vector<Object> routeMap_ConnLP;                   //whole path segments between updating positions
extern vector<Object> odometric_ReferenceObject;

extern vector<Object> last_step_view;                    //last step view from last step information

extern vector<Point> updating_Points;                    //updating positions (points)

extern Transporter lastStep_Info;                        //last step information
extern Transporter computed_Output;                      //updated global map


/* compute global map using ASR(remembered views)
 * which includes modification of object & deletion process
 */ 
void computeGlobalMap(vector<Object> currentView, vector<double> coordTransInfo, int v);

//obtain path integration reference object
void PathInte_reference(vector<double> coordTransInfo);

//obtain triangulation reference object
void Triangulation_reference();

void Record_lastStepInfo(vector<Object> currentView);

#endif /* GLOBALMAPMODULE_H */

