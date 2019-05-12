/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Exploration.h
 * Author: arthur
 *
 * Created on July 12, 2016, 1:58 AM
 */

#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>

#include "PointAndSurface.H"
#include "PathPlanning.H"
#include "Point.H"
#include "Object.H"

vector<Point> PlanThePath(vector<Object> currentView, Exit destination);


#endif /* EXPLORATION_H */

