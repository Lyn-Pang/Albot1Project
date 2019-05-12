/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Returning.h
 * Author: Wenwang
 *
 * Created on February 4, 2016, 8:10 AM
 */

#ifndef _RETURNING_H_
#define _RETURNING_H_

Point whereDestination(Point desti, double angle, double distance);
double crossedPoint(Point p1, Point p2, Point q1, Point q2);
int pathSpaceDetect(vector<Object> CV, double pathSeg);
Point ExecutionAndReturn(ArRobot& robot, double angle, double dist, Point pos);
double parallelTheSpace(vector<Object> CV);
Point midOfDoor(vector<Object> CV, Point p1, Point p2);
bool isCorridor(vector<Object> CV);
bool rightPath(vector<Object> CV, Point pos);
void matchThePath(ArRobot& robot, ArSick& sick, Point pos);
void matchPathWithSpace(ArRobot& robot, ArSick& sick, Point pos);
void CollisionAvoidandMove(ArRobot& robot, ArSick& sick, Point pos, int numOfstep);
void reTrace(ArRobot& robot, ArSick& sick, Point pos, double heading);


#endif /* RETURNING_H */

