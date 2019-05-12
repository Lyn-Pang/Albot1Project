/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RobotPosition.cpp
 * Author: arthur
 * 
 * Created on January 29, 2016, 9:46 AM
 */

#include "RobotPosition.h"
#include "Object.H"
#include "GeometricOp.H"

RobotPosition::RobotPosition()
{
}

RobotPosition::~RobotPosition()
{
}

    void RobotPosition::setPositionPoint(Point p){
        PositionPoint = p;
    }
    Point RobotPosition::getPositionPoint(){
        return PositionPoint;
    }
    void RobotPosition::setNextPositionPoint(Point p){
        NextPositionPoint = p;
    }
    Point RobotPosition::getNextPositionPoint(){
        return NextPositionPoint;
    }
    void RobotPosition::setPreviousPositionPoint(Point p){
        PreviousPositionPoint = p;
    }
    Point RobotPosition::getPreviousPositionPoint(){
        return PreviousPositionPoint;
    }
    void RobotPosition::setNextTurnOrientation(double angle){
        NextTurnOrientation = angle;
    }
    double RobotPosition::getNextTurnOrientation(){
        return NextTurnOrientation;
    }
        void RobotPosition::setPreviousTurnOrientation(double angle){
        PreviousTurnOrientation = angle;
    }
    double RobotPosition::getPreviousTurnOrientation(){
        return PreviousTurnOrientation;
    }


    vector<Object> Path_integrate_position(vector<double> coordTransInfo, vector<Object> currentRobotPositionInMFIS, vector<Object> allRobotPositions)
    {
        MyRobot myrobot(0, 0);
        
        vector<Object> robot_position, odometricReferenceObject;
        
            //localization using odometer
            Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
            aLine.setKP(1);
            odometricReferenceObject.clear();
            odometricReferenceObject.push_back(aLine);
            odometricReferenceObject.push_back(allRobotPositions[6]);
            robot_position = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());

            return robot_position;
    }
    
    