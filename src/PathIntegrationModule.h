/* 
 * File:   PathIntegrationModule.h
 * Author: arthur
 *
 * Created on April 11, 2017, 1:27 PM
 */

#include <vector>
#include <stdio.h>

#include "Object.H"

using namespace std;

#ifndef PATHINTEGRATIONMODULE_H
#define PATHINTEGRATIONMODULE_H


//integrate individual view onto global coordiante -- route map 
pair< vector<Object>, vector<Object> > IntegrateView(vector<Object> addView,  
                                                     double traveledDistance, 
                                                     double robotFacing);


//
pair< vector<Object>, vector<Object> > constrain_path_integration(vector<Object> currentview, vector<Object> robot_position,  
                                     vector<Object> environment_map, int v);

//vector<Object> PI_function_Informaiton(vector<Object> view, vector<Object> robotInGlobal, vector<double> coordTransInfo, Object ref_posiiton);

#endif /* PATHINTEGRATIONMODULE_H */

