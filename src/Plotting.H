/*
 *  Plotting: contains helper functions for plotting Objects and/or points.
 *  Taken from the old gnuplot code.
 *
 *  Thomas
 */
#ifndef _PLOTTING_H_
#define _PLOTTING_H_
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <cmath>

#include "Object.H"
#include "PathPlanning.H"
#include "asrOp.H"
#include "PointAndSurface.H"
#include "Ellipse.h"


using namespace std;

// Display a combination of points and Objects (good for looking at the Object generation)

//void plotObjects(const char * filename, Object pose, vector<Object> Objects);
//void plotObjects(const char * filename, Object pose, vector<Object> Objects1, vector<Object> Objects2);
void plotObjects(const char * filename, vector<Object> Objects);
void plotObjects(const char * filename, vector<Object> pose, vector<Object> Objects);
void plotObjects(const char * filename, vector<Object> pose, vector<Object> Objects, int refID);
void plotObjects(const char * filename, vector<Object> pose, vector<Object> Objects, vector<int> limitingPoints);
void plotObjects(const char * filename, vector<vector<Object> > pose);
void plotObjects(const char * filename, vector<Object> singlePose, vector<vector<Object> > pose, vector<Object> Objects);
void plotObjects(const char * filename, vector<Object> allPreviousRPosition, vector<Object> singlePose, 
        vector<vector<Object> > pose, vector<Object> Objects);
void plotObjectsAndPoints(const char * filename, vector<Object> pose, vector<Point> points, vector<Object> Objects);
void plotObjectsAndPExits(const char * filename, vector<Object> pose, vector<Exit> Objects2, vector<Object> Objects1);
void plotObjectsOf3Kinds(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Object> Objects2);
void plotObjectsOf3Kinds(const char * filename, vector<Object> pose, vector<Object> Objects1, 
        vector<Object> Objects2, vector<int> limitingPoints);
void plotObjectsOf4Kinds(const char * filename, vector<Object> pose,vector<Object> nodes,  vector<Object> Objects1, vector<Object> Objects2);
void plotObjectsOf5Kinds(const char * filename, vector<Object> pose,vector<Object> nodes,  vector<Object> Objects1, vector<Object> Objects2, vector<Object> Objects3);

void plotAll(const char * filename, vector<Object> pose, vector<Object> tobjects, vector<Exit> Objects2, vector<Object> Objects1);
void plotRobotView(const char * filename, vector<Object> pose, vector<Object> Objects);
void plotASR(const char * filename, vector<Object> Objects,int totalasr);
void plotSingleASR(const char * filename, ASR singleASR);
void plotAllASR(vector<ASR> allasr,vector<Object> currentRobotPositions);
void plotPerceptualMapWithASRs(const char * filename, vector<ASR> ASRs);
//void printASR(vector)

void plotPerceptualMap(const char * filename,  vector<Object> pose, vector<Object> Objects,int ASRNumber);


// Display a combination of points and surfaces (good for looking at the surface generation)
void plotPointsAndSurfaces(const char * filename,
						   const std::vector<PointXY> & points,
						   const std::vector<Surface> & surfaces);
void plotPoints(const char * filename, vector<Point> points);
void plotPoints(const char * filename, vector<PointXY> points);

void plotpathandobject(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Object> Objects2);
//void plotExitsandBoundary(const char * filename, vector<Exit> exits) ;
void plotObjectsofexits(const char * filename, vector<Object> pose, vector<Exit> Objects);
void plotObjectsOf3KindswithExits(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Exit> Objects2) ;
void plotObjectsOf3KindswithArc(const char * filename, vector<Object> pose, vector<Object> Objects1, double rad);

//void drawTwoOverlappingPics(vector<Object> ViewA, Point robotPositionA, vector<Object> ViewB, Point robotPositionB);

void plotObjectsOf3KindswithPaths(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Point> Objects2);
void plotObjectsOf3KindswithSpace(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Point> Objects2);
void plotObjectsOf3KindsDifColor(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Object> Objects2);

void plotObjectsColours(const char * filename, vector<vector<Object> > pose);
void plotObjectsColours(const char * filename, vector<vector<Object> > pose, vector<Object> rbp);
void plotObjectsColours(const char * filename, vector<vector<Object> > pose, 
                                            vector<Object> rbp, vector<Object> Objects2);

void plotObjectsOf4KindsDottedLandMarks(const char * filename, vector<Object> pose,
                                                            vector<Object> nodes,
                                                            vector<Object> Objects1,
                                                            vector<Object> Objects2);


//vector<Object> breakTheLineInto(Object smfis, int gapBetweenDots);
void plotObjectsOf3KindswithEllipse(const char * filename, vector<Object> pose, vector<Object> Objects1,
                                    Point centre, double w, double h, double orientation);

void plotObjectsOf3KindswithEllipse(const char * filename, vector<Object> pose, 
                                    vector<Object> Objects1,vector<Ellipse> Ellipses);

void plotObjectsColourAndPoint(const char * filename, vector<vector<Object> > pose, 
                                            vector<Point> rbp);


void plotObjectsColoursAndExits(const char * filename, vector<vector<Object> > pose, 
                                                       vector<Object> rbp,
                                                       vector<Exit> exits);
#endif
