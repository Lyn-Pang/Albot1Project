/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   newfile.h
 * Author: arthur
 *
 * Created on December 25, 2015, 2:01 PM
 */
#include <vector>
#include "Object.H"
#include "PathPlanning.H"
#ifndef NEWFILE_H
#define NEWFILE_H
vector<Object> findEntry(vector<Object>,int);
double getXAngleOfPoint(Point p);
double getDistance(double x1, double y1, double x2, double y2);
vector<Point> sortPointsByDistance(vector<Point> src);
vector<Object> sortObjectsByDistance(vector<Object> src);
vector<Point> findEntryPoints(vector<Object> view, int orientation);

#endif /* NEWFILE_H */

