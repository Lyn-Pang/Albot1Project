
/* 
 * File:   ChunkInfo.h
 * Author: arthur
 *
 * Created on September 27, 2017, 7:07 AM
 */

#ifndef CHUNKINFOR_H
#define CHUNKINFOR_H

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

extern ChunkInfo chunk_infomation;
extern int data_level, data_set; 

void information_process(vector< vector<Object> > views_chunk, vector<Object> Path, 
                         vector< vector<Object> > robot_positions, vector<Object> MFIS, 
                         vector<Ellipse> space_circles, int chunk_num);


#endif /* CHUNKINFOR_H */

