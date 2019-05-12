/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RobotPosition.h
 * Author: arthur
 *
 * Created on January 29, 2016, 9:46 AM
 */
#include "Point.H"
#include "Object.H"

#include<vector>
#ifndef ROBOTPOSITION_H
#define ROBOTPOSITION_H

class RobotPosition {
public:
    RobotPosition();
  //  RobotPosition(const RobotPosition& orig);
     ~RobotPosition();
    void setPositionPoint(Point p);
    Point getPositionPoint();
    void setNextPositionPoint(Point p);
    Point getNextPositionPoint();
    void setPreviousPositionPoint(Point p);
    Point getPreviousPositionPoint();
    void setNextTurnOrientation(double angle);
    double getNextTurnOrientation();
    void setPreviousTurnOrientation(double angle);
    double getPreviousTurnOrientation();
private:
    Point PositionPoint;
    Point NextPositionPoint;
    Point PreviousPositionPoint;
    double NextTurnOrientation;
    double PreviousTurnOrientation;
};

vector<Object> Path_integrate_position(vector<double> coordTransInfo, vector<Object> currentRobotPositionInMFIS, vector<Object> allRobotPositions);

#endif /* ROBOTPOSITION_H */

