#ifndef _MAPPING_H_
#define _MAPPING_H_

#include <iostream>
#include <vector>

#include "Minfo.H"
#include "Map.H"
#include "Object.H"
#include "Transporter.H"
#include "PathPlanning.H"



Transporter recognizeTargetObjects(vector<Object> mfis, vector<Object> po_pv, vector<Object> pobject_cv, vector<double> ctda,int viewNumber);
Transporter recognizeTargetExits(vector<Object> mfis, vector<Object> pvExitsAsObjects, vector<Object> cvExitsAsObjects,vector<double> ctda, int viewNumber);
//vector<Object> testingFunction();
Transporter recognizeThisPlace(ASRNetwork pm,vector<Object> mfis, vector<Object> currentView,vector<Object> robotPositions,vector<Object> referenceObjects,int viewNumber);

void recognizeViews(vector<Object> previousView, vector<Object> currentView);
vector<double> findBoundariesOfCV(vector<Object> casr);
vector<Object> findBoundaryLinesOfCV(vector<Object> casr);

void computeRouteMap(ASRNetwork pm,vector<Object> routeMap, vector<Object> crossedExit);

void useTopoMap(ASRNetwork perceptualMap,vector<Object> MFIS, vector<Object> crossedExit, vector<Object> refObjects);

vector<Minfo> recognizeObjects(vector<Object> newASRObjects, vector<Object> oldASRObjectsOnNew);
vector<Minfo> recognizeObjectsFrom2ConsViews(vector<Object> newASRObjects, vector<Object> oldASRObjectsOnNew);
vector<Object> discardObjectsPartBehindRobot(vector<Object> previousViewOnCV, vector<double> distAngle);
vector<Object> shapeBasedViewRecognition(vector<Object> previousView, vector<Object> currentView, vector<double> distAngle);

void keyInfoOfCompuetedPM(char *mappingInfoFileName,int ASRNumber, vector<int> exitPoints, vector<int> lostPoints, 
        vector<int> limitingPoints, vector<int> badLocalization, vector<int> failedToRecognizeRef, int refNumberTh, int level, int set);
void pointingExperiment(vector<Object> MFIS,vector<Object> allRobotPositions,vector<Object> currentRobotPositionInMFIS);

void abstractRouteMap(vector<Object> MFIS, vector<Object> robotPositionsAtLimitingPoints, vector<Point> updatingPoints,vector<Object> lastRP);

vector<Object> makeViewFromPoints(int set, int v);

void TestRANSACLines(vector<Point> points);

#endif
