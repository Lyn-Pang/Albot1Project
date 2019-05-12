
/* 
 * File:   RouteMapModule.h
 * Author: arthur
 *
 * Created on April 11, 2017, 1:23 PM
 */

#include <vector>
#include <stdio.h>

#include "Object.H"
#include "Transporter.H"
#include "PathPlanning.H"

using namespace std;


#ifndef ROUTEMAPMODULE_H
#define ROUTEMAPMODULE_H

                     

extern vector<Exit> exits_FromCV;                  //potential exits from current view

extern vector<Object> trackedObjects_CV;           //tracked surfaces in current view
extern vector<Object> trackedObjects_PV;           //tracked surfaces in previous view
extern vector<Object> reference_Objects;           //reference objects for triangulation
extern vector<Object> MFIS_Global;                        //Global map/MFIS
extern Transporter recognizedTrackObjects;         //recognised tracked surfaces  


/* Compute route map using PI module
 * transform all individual views onto global coordinate 
 * also includes abstract trackable surfaces & recognised tracked surfaces
 */
void RouteMap(vector<Object> currentView, vector<double> coordTransInfo, 
              double traveledDistance, double robotFacing, int v);

void init_variable(vector<Object> currentView);

#endif /* ROUTEMAPMODULE_H */

