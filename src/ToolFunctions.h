/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ToolFunctions.h
 * Author: arthur
 *
 * Created on January 22, 2016, 12:55 PM
 */
#ifndef TOOLFUNCTIONS_H
#define TOOLFUNCTIONS_H
#include "Object.H"
#include "RobotPosition.h"
#include "Memory.h"
#include "ChunksOp.H"


Memory ReadMemory();
vector<PointXY> PointsToPointsXY(vector<Point>);
//long getShortestDistance(Object a, Object b);
vector<RobotPosition> GlobalPointsToLocalPositions(vector<Point>);
void plotPointsAsObjects(char filename[],vector<Point> points);
LocalMap TransViewIntoGloabl(vector<Object> MapofLMs,vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num);

vector<Object> Globalview(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num);
vector<Object> CommonSurface(vector< vector<Object> > views);

int reference_endpoint(Object s1, Object s2);
double reference_endpoint_neardist(Object s1, Object s2);

int common_endpoint(Object ref, Object det);

vector<Object> remove_noise_surfaces(vector<Object> polygon);
void correct_polygon(vector<Object> &poly);

vector<Object> ASRs_combine(vector< vector<Object> > asrs);

Point min_distance(vector<Point> points, Point ref_p);
Point max_distance(vector<Point> points, Point ref_p);

vector<Object> adjust_box_temporary(vector<Object> sides_of_geo);
vector<Object> adjust_box_temporary_2(vector<Object> sides_of_geo, vector<Object> view);


#endif /* TOOLFUNCTIONS_H */

