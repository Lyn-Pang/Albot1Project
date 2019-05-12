/* 
 * File:   PerceptualMapping.h
 * Author: mhossain
 *
 * Created on 4 October 2012, 8:31 PM
 */

#ifndef PERCEPTUALMAPPING_H
#define	PERCEPTUALMAPPING_H

#include <vector>

#include "Transporter.H"

using namespace std;
extern int critical_reach_point;
extern int collect_asr_number; 

extern vector< vector<Object> > del_ref_list;

Transporter updatePerceptualMap(vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS, vector<Object> refobjects,int viewNumber, int ASRNumber);
Transporter updatePerceptualMapATPlace(vector<ASR> places,vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS, 
        vector <Object> allRobotPositions,vector<Object> refobjects, int viewNumber, int ASRNumber, 
        bool exitCrossed, vector<Object> crossedExit,vector<Object> routeMap4CPlace);

void updateMapAtLastStep(Transporter lastStepInfo);
bool isBelongToSameSpace(int limitingPoint, vector<int> spaceIDs);
bool isBelongToSameSpace(vector<int> limitingPoint, vector<int> spaceIDs);

bool isThisIDAdded(int newID, vector<int> previousIDs);

//return true if thisObject close to current view
bool isThisCloseToCurrentView(Object thisObject, vector<Object> currentView);

bool isThisCloseToCVPolygonBoundary(Object thisObject, vector<Object> polygonObjects, double distTh);

bool findCrossedExit(vector<Object> & allCrossedExit, vector<Object> lastView, 
        vector<Object> refObjects, vector<double> distanceAngle, int set);

void updatePerceptualMapUsingPlaceInformation(vector<ASR> & places, vector<Object> & MFIS, 
        vector<Object> cView, vector<Object> currentRobotPositionInMFIS,vector <Object> allRobotPositions, 
        vector<Object> refobjects, int viewNumber, int ASRNumber, bool exitCrossed, 
        vector<Object> crossedExit, vector<Object> & targetObjectsInPV, vector<Object> refObjectForLoopClosing,
        int updatingASRNumber,vector<Object> routeMap4CPlace);

void findBestReferenceObjectsUsingOdometryInfo(vector<Object> & referenceObjects,
        vector< vector<Object> > allRPoseAtOneStep, vector<Object> currentRobotPositionInMFIS,
        vector<double> coordTransInfo);


/* modified by Lyn Pang
 * modified part is deletion process
 * delete surfaces that are in the polygon (must-be-removed surfaces) 
 * & behind those must-be-removed surfaces
 */
Transporter updatePerceptualMapATPlaceDeletion(vector<ASR> places,vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS, 
        vector <Object> allRobotPositions,vector<Object> refobjects, int viewNumber, int ASRNumber, 
        bool exitCrossed, vector<Object> crossedExit,vector<Object> routeMap4CPlace);

Transporter updatePerceptualMapATPlaceDeletion_version2(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, unsigned char expectation_flag);

Transporter updatePerceptualMapATPlaceDeletion_version3(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, unsigned char expectation_flag);

Object similarObeject(vector<Object> MFIS, Object Obj);
Object Compare_similarObeject(vector<Object> MFIS, Object Obj, int v);
Object Compare_similar_near_Obeject_(vector<Object>& MFIS, Object& Obj, int v);
Object Compare_similar_near_Obeject_modify(vector<Object>& MFIS, Object& Obj, int v);
int match_two_views(vector<Object> v1, vector<Object> v2);

Object modifyObject(Object currentObj, Object ref);
vector<Object> DeleteObjectFromCV(vector<Object> CV, Object to_delete);
bool inDeleteASR(vector<Object> removed_info, Object potential_Object);

#endif	/* PERCEPTUALMAPPING_H */

