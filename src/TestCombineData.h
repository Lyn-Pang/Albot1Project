/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
/*
 * Function: Combine some information for testing
 *           and analysing data
 * 
 * author: Lyn Pang
 */

#ifndef TESTCOMBINEDATA_H
#define TESTCOMBINEDATA_H


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "Point.H"
#include "Plotting.H"
#include "Object.H"
#include "GeometricOp.H"
#include "GeometryFuncs.H"
#include "PointAndSurface.H"
#include "PathPlanning.H"
#include "readAndwriteASCII.H"
#include "GeometryAndExit.h"
#include "Ellipse.h"
#include "Exit.h"


void Test_cominbe_ASR(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles);
void Test_cominbe_ASR2(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles);
void Test_cominbe_ASR3(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles);
void Test_cominbe_ASR4(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles);

void Test_combine_regions(vector< vector<Object> >  region1, vector< vector<Object> >  region2, 
                          vector<Object> mfis1, vector<Object> mfis2, vector<Object> Path, vector<Object> robot1,
                          vector<Object> robot2, Exit imagine_exit1, Exit imagine_exit2, Exit imagine_exit3);

void Test_cross_exit(vector< vector<Object> >  region, vector<Object>  lastStep_view, 
                          vector<Object> mfis, vector<Object> Path, vector<Object> lastStep_robot,
                          vector<Object> nextStep_robot, Exit imagine_exit1, Exit imagine_exit2);




#endif /* TESTCOMBINEDATA_H */

