//============================================================================
// Name        : Path Integration module
// Author      : Wenwang (Lyn)
// Version     : Path Integration module, associate to GlobalAndRouteMap.cpp 
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

#include "Point.H"
#include "Object.H"
#include "RouteMapModule.h"
#include "GlobalMapModule.h"
#include "PathIntegrationModule.h"
#include "GeometricOp.H"
#include "Returning.h"
#include "PerceptualMapping.H"
#include "mfisOp.H"
#include "Plotting.H"

#define PI 3.14159265
using namespace std;



//integrate individual view onto global coordiante -- route map 
pair< vector<Object>, vector<Object> > IntegrateView(vector<Object> addView, 
                                                   double traveledDistance, 
                                                   double robotFacing)
{
            MyRobot robot(0, 0);
    
            //finding robot position first
            Point robotPosition;
            double angle = robotFacing; 
            angle = angle * (PI/180); //angle in radian
            
            double rpx = traveledDistance * sin(-angle); //PI robot global position x
            double rpy = traveledDistance * cos(-angle); //PI robot global position y 
            robotPosition.set(rpx, rpy); //PI global position

           
            vector<Object> temp_view, temp_robot;   //global view & robot position
            pair< vector<Object>, vector<Object> > temp; 
            double x1, y1, x2, y2;                  //temp x, y of each object 

            //PI view on global coordinate
            for(int i = 0; i<int(addView.size()); i++)
            {
                    x1 = addView[i].X1() * cos(angle) - addView[i].Y1() * sin(angle) + robotPosition.X();
                    y1 = addView[i].X1() * sin(angle) + addView[i].Y1() * cos(angle) + robotPosition.Y();

                    x2 = addView[i].X2() * cos(angle) - addView[i].Y2() * sin(angle) + robotPosition.X();
                    y2 = addView[i].X2() * sin(angle) + addView[i].Y2() * cos(angle) + robotPosition.Y();

                    Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());
                    s.setKP(1);
                    temp_view.push_back(s);   
                      
            }
           
            //PI corresponding robot global position
            for(int i = 0; i < robot.getRobot().size(); i++)
            {
                    x1 = robot.getRobot()[i].X1() * cos(angle) - robot.getRobot()[i].Y1() * sin(angle) + robotPosition.X();
                    y1 = robot.getRobot()[i].X1() * sin(angle) + robot.getRobot()[i].Y1() * cos(angle) + robotPosition.Y();

                    x2 = robot.getRobot()[i].X2() * cos(angle) - robot.getRobot()[i].Y2() * sin(angle) + robotPosition.X();
                    y2 = robot.getRobot()[i].X2() * sin(angle) + robot.getRobot()[i].Y2() * cos(angle) + robotPosition.Y();

                    Object s(x1, y1, x2, y2, robot.getRobot()[i].getID());
                    temp_robot.push_back(s); 
            }

           
            temp.first = temp_view;
            temp.second = temp_robot;
           
            return temp;
}



/*modification of pi, when robot keeps using pi, it will cause distortion
 *because of accumulation of error
 */
pair< vector<Object>, vector<Object> > constrain_path_integration(vector<Object> currentview, vector<Object> robot_position,  
                                     vector<Object> MFIS, int v)
{
    cout << " ----- Path Integration restriction function ----- " << endl;
    int flag_left = 0, flag_right = 0;
    double expAngle1, expDist1, expDist2, expDist_mid1;
    double expAngle2, expDist3, expDist4, expDist_mid2;
    
    Object left_side_on_mfis, right_side_on_mfis;
    Object left_ref, right_ref;
    
    vector<Object> reference_obj;
    vector<Object> adjust_robot, adjust_view;
    
    pair< vector<Object>,  vector<Object> > temp; //for return 
    
    
    left_ref = expend_Object(robot_position[7], 1200, 1);
    right_ref = expend_Object(robot_position[7], 1200, 2);
    
    //if left or right 
    left_side_on_mfis = intersectForObject(MFIS, left_ref.getP1(), left_ref.getP2());
    right_side_on_mfis = intersectForObject(MFIS, right_ref.getP1(), right_ref.getP2());
    
//    cout << "ref1 x1 : " << left_side_on_mfis.X1() << " ; y1: " << left_side_on_mfis.Y1() << endl;
//    cout << "ref1 x2 : " << left_side_on_mfis.X2() << " ; y2: " << left_side_on_mfis.Y2() << endl;
//    cout << "ref2 x1 : " << right_side_on_mfis.X1() << " ; y1: " << right_side_on_mfis.Y1() << endl;
//    cout << "ref2 x2 : " << right_side_on_mfis.X2() << " ; y2: " << right_side_on_mfis.Y2() << endl;
    
    //match 
    if((left_side_on_mfis.X1() != 0) && (left_side_on_mfis.X2() != 0))
    {
        expAngle1 = left_side_on_mfis.getAngleWithLine(currentview[0]);
        expDist1 = P_To_ShortestDistance(currentview[0].getP1(), left_side_on_mfis);
        expDist2 = P_To_ShortestDistance(currentview[0].getP2(), left_side_on_mfis);
        //expDist_mid1 = distanceOftwoP(currentview[0].midpoint(), left_side_on_mfis.midpoint());
        expDist_mid1 = P_To_ShortestDistance(currentview[0].midpoint(), left_side_on_mfis);

        if (((abs(expAngle1) < 30.0 || abs(expAngle1) > 330.0) || (abs(expAngle1) > 176.6 && abs(expAngle1) < 189.0)) 
                && (expDist1 < 600.0 || expDist2 < 600.0) && (expDist_mid1 < 500))
            flag_left = 1;
    }
    
    if((right_side_on_mfis.X1() != 0) && (right_side_on_mfis.X2() != 0))
    {
        expAngle2 = right_side_on_mfis.getAngleWithLine(currentview.back());
        expDist3 = P_To_ShortestDistance(currentview.back().getP1(), right_side_on_mfis);
        expDist4 = P_To_ShortestDistance(currentview.back().getP2(), right_side_on_mfis);
        //expDist_mid2 = distanceOftwoP(currentview.back().midpoint(), right_side_on_mfis.midpoint());
        expDist_mid2 = P_To_ShortestDistance(currentview.back().midpoint(), right_side_on_mfis);

        if (((abs(expAngle2) < 30.0 || abs(expAngle2) > 330.0) || (abs(expAngle2) > 176.6 && abs(expAngle2) < 189.0)) 
                && (expDist3 < 600.0 || expDist4 < 600.0) && (expDist_mid2 < 500))
            flag_right = 1;
    }
    
    cout << " flag left  : " << flag_left << endl;
    cout << " flag right : " << flag_right << endl; 
    
    //adjust parameter
    if((flag_left == 1) && (flag_right == 1))
    {
        //compute weight 
        double W_left_ref, W_right_ref;
        
        W_left_ref = (3+4)/(expDist_mid1+expAngle1);
        W_right_ref = (3+4)/(expDist_mid2+expAngle2);
        if(W_left_ref > W_right_ref)
        {
            reference_obj.push_back(left_side_on_mfis);
            reference_obj.push_back(currentview[0]);
        }
        else
        {
            reference_obj.push_back(right_side_on_mfis);
            reference_obj.push_back(currentview.back());
        }
    }
    else
    {
        if(flag_left == 1)
        {
            reference_obj.push_back(left_side_on_mfis);
            reference_obj.push_back(currentview[0]);
        }
        else
        {
            if(flag_right == 1)
            {
                reference_obj.push_back(right_side_on_mfis);
                reference_obj.push_back(currentview.back());
            }
        }
    }
    
    
    //adjust errors (robot & view)
    
    if((flag_left == 1) || (flag_right == 1))
    {
        reference_obj[0].display();
        reference_obj[1].display();
        
        int ref_point;
        if(distanceOftwoP(reference_obj[0].getP1(), reference_obj[1].getP1()) < distanceOftwoP(reference_obj[0].getP2(), reference_obj[1].getP2()))
            ref_point = 1;
        else
            ref_point = 2;

        adjust_robot = projectingTheView(robot_position, reference_obj[0], reference_obj[1], ref_point);
        adjust_view = projectingTheView(currentview, reference_obj[0], reference_obj[1], ref_point);
   
    }
    else
    {
        adjust_robot = robot_position;
        adjust_view = currentview;
    }
    
    temp.first = adjust_robot;
    temp.second = adjust_view;
    
    
    return temp;
    
}


//vector<Object> PI_function_Informaiton(vector<Object> view, vector<Object> robotInGlobal, vector<double> coordTransInfo, Object ref_posiiton)
//{
//    MyRobot myrobot;
//    vector<Object> ReferenceObject;
//    vector<Object> PI_robot_position;
//    vector<Object> PI_view;
//    
//    Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, robotInGlobal[6], 1);
//    aLine.setKP(1);
//    ReferenceObject.clear();
//    ReferenceObject.push_back(aLine);
//    ReferenceObject.push_back(ref_posiiton);
//    PI_robot_position = myrobot.inMFIS(ReferenceObject[0], ReferenceObject[1], ReferenceObject[0].getKP());
//
//    cout << " size : " << PI_robot_position.size() << endl;
//    
//    PI_view = TransformforToGlobalCoordinate(view, PI_robot_position[6].getP1(), PI_robot_position, PI_robot_position[7].getAngleWithXaxis());
//    cout << " test proramme2 " << endl;
//    return PI_view;
//}









