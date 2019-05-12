#include <vector>
#include <iostream>
#include <valarray>

#include "PerceptualMapping.H"
#include "Transporter.H"
#include "GeometricOp.H"
#include "Plotting.H"
#include "mfisOp.H"
#include "CompareASR.H"
#include "Mapping.H"
#include "PointAndSurface.H"
#include "clipper.hpp"
#include "PathPlanning.H"
#include "Expectation.h"
#include "ToolFunctions.h"
#include "concave_hull.h"

using namespace std;
using namespace ClipperLib;

#define PI 3.14159265
int po = 0;

int critical_reach_point = 0;
int critical_reach_flag = 0;
int collect_asr_number = 0; 

vector<Object> deletion_reference;
vector< vector<Object> > del_ref_list;



Transporter updatePerceptualMap(vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS, vector<Object> refobjects, int viewNumber, int ASRNumber) {
    cout << "\033[1;32m-------------------Inside updatePerceptualMap module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();

    //cout << endl << endl << "PerceptualMap" << endl;
    //displayObjects(perceptualMap);

    //cout << endl << endl << "Current View" << endl;
    //displayObjects(cView);

    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    for (unsigned int i = 0; i < cView.size(); i++) 
    {
        temp = remakeLineP2(refobjects[0], refobjects[1], cView[i], i + 1, 0, refobjects[0].getKP());
        if (cView[i].getPos() == 1)
            temp.setPos(1);
        else
            temp.setPos(-1);
        temp.setVN(currentViewNumber);
        temp.setOoPV(true);
        currentViewInMFIS.push_back(temp);
    }


    //for a fresh ASR
    if (perceptualMap.size() == 0) {
        cout << "Current ASR is empty" << endl;
        //no need to go down
        for (unsigned int i = 0; i < currentViewInMFIS.size(); i++) {
            currentViewInMFIS[i].setASRNo(ASRNumber);
        }
        Transporter package;
        package.setView(currentViewInMFIS);
        return package;
        //waitHere();
    }
    // if(viewNumber == 39)

    //computing all MFIS objects on CV to find objects in Current view
    Point currentRobPoint; //Point for current robot position in MFIS
    currentRobPoint.set(xAxisAtCRP.X1(), xAxisAtCRP.Y1());
    double angle = xAxisAtCRP.getAngleWithXaxis(); //Angle btw current robot x-axis and Original x-axis
    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);
    vector<Object> pMapOnCV = xformPVIntoCV(perceptualMap, currentRobPoint, angle);

    //  plotObjects("Maps/MIFSonCV.png", cRobotPositionInMFIS, pMapOnCV);


    vector<double> boundariesOfCV = findBoundariesOfCV(cView);


    vector<Object> exppandableObjects;

    int expandableOnLeft = 0;
    int expandableOnRight = 0;
    //Finding MFIS objects in CV
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) {
        //changing position tag of those objects which appeared on opposite side but now should be other side
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() < 0 && perceptualMap[i].getPos() == 1 && perceptualMap[i].getOoPV() == true) {
            if (perceptualMap[i - 1].getPos() == -1)//some objects position is be changed when robot takes big turn
                perceptualMap[i].setPos(-1);
        }
        //changing position tag
        if (pMapOnCV[i].X1() > 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) {
            if (perceptualMap[i - 1].distP2ToP1(perceptualMap[i]) > 600 or perceptualMap[i - 1].getPos() == 1)
                perceptualMap[i].setPos(1);
        }

        //finding MFIS objects which are behind current robot position
        //if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && perceptualMap[i].getASRNo() == ASRNumber) {
        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (perceptualMap[i].getOoPV() == true || pMapOnCV[i].isThisInsideCV(boundariesOfCV) == true)) {
            //if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && perceptualMap[i].getOoPV() == true) {
            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV

            if (pMapOnCV[i].length() > 3000 && viewNumber == 128) {//special case for set11 and first corridor 
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
            }

        } else {
            objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
            objectsBehindRobot.back().setOoPV(false);



            //finding expandable objects on LEFT
            if (perceptualMap[i].getPos() == -1) {
                if ((pMapOnCV[i].Y1() < 0 && pMapOnCV[i].Y2() > 0) || (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)) {
                    //if(expandableOnLeft == 0) {
                    expandableOnLeft = perceptualMap[i].getID(); //storing last expandable objectID
                    // }
                    exppandableObjects.push_back(perceptualMap[i]);
                    cout << "LEFT: " << perceptualMap[i].getID() << endl;
                }
            }

            //finding expandable objects on RIGHT
            if (perceptualMap[i].getPos() == 1)
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0) {
                    if (expandableOnRight == 0) {
                        expandableOnRight = perceptualMap[i].getID(); //storing last expandable objectID
                        exppandableObjects.push_back(perceptualMap[i]);
                        cout << "RIGHT: " << perceptualMap[i].getID() << endl;
                    }
                }
        }


    }

    //plotObjects("Maps/expandableObects.png",firstRobotPosition,exppandableObjects);
    cout << "Current view in MFIS" << endl;
    //displayObjects(currentViewInMFIS);
    cout << "Obects behind CRP" << endl;
    //displayObjects(objectsBehindRobot);

    // plotObjects("Maps/objectBehindRnCV.png", objectsBehindRobot, firstRobotPosition);

    int lastObjectID = perceptualMap[perceptualMap.size() - 1].getID();
    vector<Object> targetObjectsInCV = findTargetObjects(cView);
    cout << endl << endl << "Target Objects in CV " << targetObjectsInCV.size() << endl;
    //displayObjects(targetObjectsInCV);
    vector<Object> targetObjectsForNextStep;


    double expAngle;
    double expDist;
    int insertFrom = 0;
    int insertTo = currentViewInMFIS.size() + 1;


    //LEFT expanding objects 
    if (expandableOnLeft != 0) {//finding same object of cv on LEFT
        cout << "(LEFT) Object number " << expandableOnLeft << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnLeft) {
                for (unsigned int j = 0; j < currentViewInMFIS.size(); j++) {//will consider first same object of CV on LEFT
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if ((abs(expAngle) < 6 || abs(expAngle) > 354) && expDist < 500 && currentViewInMFIS[j].getPos() == -1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP2(currentViewInMFIS[j].X2(), currentViewInMFIS[j].Y2());
                        objectsBehindRobot[i].setOoPV(true);
                        insertFrom = currentViewInMFIS[j].getID();

                        //finding whether this object is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                break;
            }
        }
    }

    //RIGHT side expending 
    if (expandableOnRight != 0) {
        cout << "(RIGHT) Object number " << expandableOnRight << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnRight) {
                for (unsigned int j = currentViewInMFIS.size() - 1; j > 0; j--) {//will consider last same object of cv on RIGHT
                    cout << "finding same object from last " << j << endl;
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    cout << "Angle: " << expAngle << " dist: " << expDist << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 6 || abs(expAngle) > 354) && expDist < 500 && currentViewInMFIS[j].getPos() == 1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP1(currentViewInMFIS[j].X1(), currentViewInMFIS[j].Y1());
                        objectsBehindRobot[i].setOoPV(true);
                        insertTo = currentViewInMFIS[j].getID();

                        //checking whether this one is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                cout << "Hello" << endl;
                break;
            }
        }
    }

    //inserting LEFT side's objects(old)
    for (int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == -1) {
            updatedPM.push_back(objectsBehindRobot[i]);
        }
    }

    //inserting NEW objects
    cout << "InsertTo: " << insertTo << endl;
    cout << "size: " << objectsBehindRobot.size() << endl;
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        updatedPM.back().setASRNo(ASRNumber);
        
        
        //making target(i.e changing id as they have in MFIS) object for next step
        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                targetObjectsForNextStep.back().setID(lastObjectID + 1);
                break;
            }
        }

        lastObjectID++;
    }
    cout << endl << endl << "TargetObjects for Next step " << targetObjectsForNextStep.size() << endl;
    //displayObjects(targetObjectsForNextStep);

    //inserting RIGHT side's objects(old)
    for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == 1) {
            updatedPM.push_back(objectsBehindRobot[i]);
        }
    }
    updatedPM.back().setID(lastObjectID + 1);
    cout << endl << endl << "UpdatedMFIS " << endl;
    // displayObjects(updatedPM);
    // plotObjects("Maps/updatedMFIS.png", updatedPM, firstRobotPosition);
    //waitHere();
    

    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    return package;
}

//it's THE new version of updatePerceptualMap
//all surfaces are tagged as their place number
//place is initiated when robot crosses a exit

Transporter updatePerceptualMapATPlace(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace) 
{
    cout << "\033[1;32m-------------------Inside updatePerceptualMap module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();

    //cout << endl << endl << "PerceptualMap" << endl;
    //displayObjects(perceptualMap);

    //cout << endl << endl << "Current View" << endl;
    //displayObjects(cView);
    cout << endl << endl << "Ref objects" << endl;
    //displayObjects(refobjects);
    
    cout << " the size of ref objects : " << refobjects.size() << endl;

    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //********************************************//
    // Variables for showcase interface programme //
    vector<Object> old_Objects_GlobalMap, new_Objects_GlobalMap;
    unsigned char inter_flag = 0; //intersection flag for deletion section
    //********************************************//

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();
    //    if(viewNumber == 87) //bug
    //        cViewSize = cView.size()-1;
    for (unsigned int i = 0; i < cViewSize; i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], cView[i], i + 1, 0, refobjects[0].getKP());
            if (cView[i].getPos() == 1)
                temp.setPos(1);
            else
                temp.setPos(-1);
            temp.setVN(currentViewNumber);
            temp.setOoPV(true);
            temp.setASRNo(ASRNumber);
            temp.setLimitingPoint(viewNumber);
            temp.setLocalEnvID(viewNumber);
            temp.setPO(cView[i].getPO());
            currentViewInMFIS.push_back(temp);
    }
    
    //demonstrating error during updating (only for print)
    if (refobjects[1].getColorID() == 100) 
    {
            vector<Object> dummy;
            char mfisNCVfileName[80];
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/MFISandCurrentView-", viewNumber, ".png");
            dummy.push_back(refobjects[0]);
            //plotObjectsOf4Kinds(mfisNCVfileName, cRobotPositionInMFIS, dummy, currentViewInMFIS, perceptualMap);
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);
            //plotObjectsOf3Kinds(mfisNCVfileName, myrobot.getRobot(), dummy, cView);
            //waitHere();
    }

    
    //constructing place    
    if (exitCrossed == true) //or viewNumber == 73) 
    {
            cout << "crossedExits: " << crossedExit.size() << endl;
            char placeFileName[80];
            vector<Object> place;

            ASR onePlace;
            vector<Object> exits;
            sprintf(placeFileName, "%s%d%s", "Maps/place-", ASRNumber - 1, ".png");
            for (unsigned int i = 0; i < perceptualMap.size(); i++) 
            {
                    if (perceptualMap[i].getASRNo() == ASRNumber - 1)
                        place.push_back(perceptualMap[i]);
            }

            //adding exits
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 2])); //exit1
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 1])); //exit2
            plotObjectsOf3Kinds(placeFileName, exits, routeMap4CPlace, place);

            onePlace.setASRObjects(place);
            onePlace.setASRExit1(crossedExit[crossedExit.size() - 2]);
            onePlace.setASRExit2(crossedExit[crossedExit.size() - 1]);
            onePlace.replaceTheWholeRoute(routeMap4CPlace); //this route just connecting limiting points
            places.push_back(onePlace);
    }


    //for a fresh ASR
    if (perceptualMap.size() == 0)
    {
            cout << "Current ASR is empty" << endl;
            //no need to go down
            Transporter package;
            package.setView(currentViewInMFIS);
            return package;
    }


    //computing all MFIS objects on CV to find objects in Current view
    Point currentRobPoint; //Point for current robot position in MFIS
    currentRobPoint.set(xAxisAtCRP.X1(), xAxisAtCRP.Y1());
    double angle = xAxisAtCRP.getAngleWithXaxis(); //Angle btw current robot x-axis and Original x-axis
    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);

    cout << "test programme angle : " << angle << endl << endl;
    
    vector<Object> pMapOnCV = xformPVIntoCV(perceptualMap, currentRobPoint, angle);

    //  plotObjects("Maps/MIFSonCV.png", cRobotPositionInMFIS, pMapOnCV);    

    vector<double> boundariesOfCV = findBoundariesOfCV(cView);
    
    //for printing only
    vector<Object> boundaryLines, pMapOnCVinsideCV;
    boundaryLines.push_back(Object(boundariesOfCV[0], 0, boundariesOfCV[0], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[0], boundariesOfCV[2], boundariesOfCV[1], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[1], 0, boundariesOfCV[1], boundariesOfCV[2]));

    //making polygon of CV
    vector<Surface> polygon = makePolygonOfCV(cView); //findExactBoudaryFrom(cView);//
    //vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    
    //modified by wenwang, building a polygon corresponding current view
    vector<Object> polygonObjects;
    ClipperLib::Path subj;
    Paths solution;
    ClipperOffset co;

    subj = viewConvertPath(cView); //convert view into Path
    co.AddPath(subj, jtMiter, etClosedPolygon);
    co.Execute(solution, 1000.0); // outward offset the polygon
    polygonObjects = PathsConvertView(solution); // convert polygon into view
    polygon = convertObjectToSurface(polygonObjects); // convert view into surfaces
    
    char polyFile[80];
    sprintf(polyFile,"%s%d%s", "Maps/Offline/MFIS&Poly-", viewNumber, ".png");
    plotObjectsOf3Kinds(polyFile, polygonObjects, pMapOnCV, cView);
    
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }
    

    //plotObjects("Maps/Offline/CV-Boundary.png",cView,polygonObjects);
    //if(viewNumber > 68)
    //{
    //    plotObjectsOf3Kinds("Maps/Offline/Polygon.png",xformPVIntoCV(allRobotPositions, currentRobPoint, angle),breakTheLinesInto(polygonObjects),pMapOnCV);    
    //    waitHere();
    //}
    

    vector<Object> exppandableObjects;
    vector<int> lpToDelete; //limiting points to be deleted.
    vector<Object> objectsBelongToSameSpace;
    vector<Object> deletedObjectsOnCV;

    //     //saving local environment number 
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true ||
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) 
        {

            if (lpToDelete.size() == 0) 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++)
                    lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
            } 
            else 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++) 
                {
                        bool alreadySaved = false;
                        for (unsigned int lp = 0; lp < lpToDelete.size(); lp++) 
                        {
                                if (lpToDelete[lp] == perceptualMap[i].getLocalEnvID()[le]) 
                                {
                                        alreadySaved = true;
                                        break;
                                }
                        }
                        if (alreadySaved == false)
                        {
                            lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
                        }
                }
            }
        }
    }
    
 
    int expandableOnLeft = 0;
    int expandableOnRight = 0;
    //Finding MFIS objects in CV
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        //changing position tag of those objects which appeared on opposite side but now should be other side
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() < 0 && perceptualMap[i].getPos() == 1 && perceptualMap[i].getOoPV() == true) 
        {
            if(i == 0)
                perceptualMap[i].setPos(-1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].getPos() == -1)//some objects position is be changed when robot takes big turn
                        perceptualMap[i].setPos(-1);
                }
        }
        
        //changing position tag
        if (pMapOnCV[i].X1() > 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {   
            if(i == 0)
                perceptualMap[i].setPos(1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].distP2ToP1(perceptualMap[i]) > 600 or perceptualMap[i - 1].getPos() == 1)
                        perceptualMap[i].setPos(1);

                }
        }

        //changing position tag
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {
            if (viewNumber == 84)
                perceptualMap[i].setPos(1);
        }

        //finding MFIS objects which are behind current robot position
        //       if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && perceptualMap[i].getOoPV() == true) {
        //       if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && 
        //                perceptualMap[i].getASRNo() == ASRNumber) {
        //       if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (perceptualMap[i].getOoPV() == true || 
        //            pMapOnCV[i].isThisInsideCV(boundariesOfCV) == true)) {
        //        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
        //                (pointInPolygon(PointXY(pMapOnCV[i].X1(),pMapOnCV[i].Y1()),polygon) == true || 
        //                pointInPolygon(PointXY(pMapOnCV[i].X2(),pMapOnCV[i].Y2()),polygon) == true)) {
        if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true &&
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true))// &&
                //interSectWithLine(polygonObjects, Point(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), Point(pMapOnCV[i].X2(), pMapOnCV[i].Y2())) == true) 
        {

            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
            pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing
            deletedObjectsOnCV.push_back(pMapOnCV[i]);


//            if (pMapOnCV[i].length() > 3000 && viewNumber == 128) {//special case for set11 and first corridor 
//                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
//                objectsBehindRobot.back().setOoPV(false);
//            }

        } 
        else 
        {
            //           if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
            //                   isBelongToSameSpace(perceptualMap[i].getLimitingPoint(),lpToDelete)==true) {

            //            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
            //                    isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true &&
            //                    isThisCloseToCurrentView(pMapOnCV[i],cView) == true) {//condition to delete surfaces which belong to same localEnv
            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) {
//            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true ) {
//            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (checkForIntersection(Object(0,0,0,2000),pMapOnCV[i]) || (isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true && 
//                      isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 2500.0) == true))) {
                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
            } 
            else 
            {
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
            }


            //finding expandable objects on LEFT
            if (perceptualMap[i].getPos() == -1) 
            {
                if ((pMapOnCV[i].Y1() < 0 && pMapOnCV[i].Y2() > 0) || (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)) 
                {
                    //if(expandableOnLeft == 0) {
                    expandableOnLeft = perceptualMap[i].getID(); //storing last expandable objectID
                    // }
                    exppandableObjects.push_back(perceptualMap[i]);
                    cout << "LEFT: " << perceptualMap[i].getID() << endl;
                }
            }

            //finding expandable objects on RIGHT
            if (perceptualMap[i].getPos() == 1)
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                {

                    if (expandableOnRight == 0) 
                    {
                        expandableOnRight = perceptualMap[i].getID(); //storing last expandable objectID
                        exppandableObjects.push_back(perceptualMap[i]);
                        cout << "RIGHT: " << perceptualMap[i].getID() << endl;
                    }
                }
        }
    }

    //just for demonstration
    char mfisFileName[80];
    sprintf(mfisFileName, "%s%d%s", "Maps/ObjectInsideCV-", viewNumber, ".png");
    //plotObjectsOf4Kinds(mfisFileName,polygonObjectsOnMFIS,objectsInSideCV,objectsBelongToSameSpace,objectsBehindRobot);
    if (objectsBelongToSameSpace.size() > 10) 
    {

        sprintf(mfisFileName, "%s%d%s", "Maps/ObjectInsideCV-", viewNumber, ".png");
        //if(viewNumber == 242 or viewNumber == 864 or viewNumber == 1009 or viewNumber == 1425) 
        //plotObjectsOf4Kinds(mfisFileName,pMapOnCV,pMapOnCVinsideCV,objectsBelongToSameSpace,polygonObjects);
        //waitHere();

        cout << "Current view in MFIS" << endl;
        //displayObjects(currentViewInMFIS);
        cout << "Obects behind CRP" << endl;
        //displayObjects(objectsBehindRobot);
        //displayObjects(objectsBelongToSameSpace);

        //cout << "lp " << lpToDelete.size() << " " << lpToDelete[0] << endl;
        //waitHere();
    }
    /*
    if(viewNumber == 74)
    {
        cout<<" ---- test programe ---- "<<endl;
        cout<<"the number of deletion : "<<deletedObjectsOnCV.size()<<endl;
        cout<<"the first deleted surfce local evironment ID : "<<deletedObjectsOnCV[20].getLocalEnvID().size()<<endl;
        for(int i = 0; i < deletedObjectsOnCV.size(); i++)
            for(int j = 0; j < deletedObjectsOnCV[i].getLocalEnvID().size(); j++)
                cout<<" "<<deletedObjectsOnCV[i].getLocalEnvID()[j];
            
            waitHere();
    }
    */
//    //setting local env tag based on deleted objects from mfis.
//    //idea: cv object's local env will be same as deleted same objects from mfis
    //    vector<Surface> triangle;
//    Object tempP;
//    for (unsigned int j = 0; j < deletedObjectsOnCV.size(); j++) {
//        tempP = makeParallelObject(deletedObjectsOnCV[j], 300, 1);
//        triangle.push_back(Surface(PointXY(0, 0), PointXY(tempP.X1(), tempP.Y1())));
//        triangle.push_back(Surface(PointXY(tempP.X1(), tempP.Y1()), PointXY(tempP.X2(), tempP.Y2())));
//        triangle.push_back(Surface(PointXY(tempP.X2(), tempP.Y2()), PointXY(0, 0)));
//        for (unsigned int i = 0; i < cView.size(); i++) {
//            if (pointInPolygon(PointXY(cView[i].X1(), cView[i].Y1()), triangle) == true ||
//                    pointInPolygon(PointXY(cView[i].X2(), cView[i].Y2()), triangle) == true ||
//                    checkForIntersection(convertSurfaceToObject(triangle[0]), cView[i]) == 1 ||
//                    checkForIntersection(convertSurfaceToObject(triangle[2]), cView[i]) == 1)
//                for (unsigned int k = 0; k < deletedObjectsOnCV[j].getLocalEnvID().size(); k++)
//                    if (isThisIDAdded(deletedObjectsOnCV[j].getLocalEnvID()[k], currentViewInMFIS[i].getLocalEnvID()) == false)
//                        currentViewInMFIS[i].setLocalEnvID(deletedObjectsOnCV[j].getLocalEnvID()[k]);
//
//        }
//        //sprintf(mfisFileName, "%s%d%s", "Maps/deletedObjectsOnCV-", viewNumber, ".png");
//        //plotObjectsOf4Kinds(mfisFileName, myrobot.getRobot(), cView, deletedObjectsOnCV, convertSurfaceToObject(triangle));
//        //displayObjects(convertSurfaceToObject(triangle));
//        triangle.clear();
//        //waitHere();
//    }

    cout << "deletedObjects on CV" << endl;
    //displayObjects(deletedObjectsOnCV);

    cout << "CV in MFIS" << endl;
    //displayObjects(currentViewInMFIS);


    // plotObjects("Maps/objectBehindRnCV.png", objectsBehindRobot, firstRobotPosition);

    int lastObjectID = perceptualMap[perceptualMap.size() - 1].getID();
    vector<Object> targetObjectsInCV = findTargetObjects(cView);
    cout << endl << endl << "Target Objects in CV " << targetObjectsInCV.size() << endl;
    //displayObjects(targetObjectsInCV);
    vector<Object> targetObjectsForNextStep;


    double expAngle;
    double expDist;
    double similarity;
    int insertFrom = 0;
    int insertTo = currentViewInMFIS.size() + 1;


    //LEFT expanding objects 
    if (expandableOnLeft != 0) {//finding same object of cv on LEFT
        cout << "(LEFT) Object number " << expandableOnLeft << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) 
        {
            if (objectsBehindRobot[i].getID() == expandableOnLeft) 
            {
                for (unsigned int j = 0; j < currentViewInMFIS.size(); j++) {//will consider first same object of CV on LEFT
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && expDist < 500.0
                            && currentViewInMFIS[j].getPos() == -1) {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 && currentViewInMFIS[j].getPos() == -1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP2(currentViewInMFIS[j].X2(), currentViewInMFIS[j].Y2());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertFrom = currentViewInMFIS[j].getID();

                        //finding whether this object is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
                        {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                break;
            }
        }
    }

    //RIGHT side expending 
    if (expandableOnRight != 0) 
    {
        cout << "(RIGHT) Object number " << expandableOnRight << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnRight) {
                for (unsigned int j = currentViewInMFIS.size() - 1; j > 0; j--) {//will consider last same object of cv on RIGHT
                    cout << "finding same object from last " << j << endl;
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && expDist < 500.0
                            && currentViewInMFIS[j].getPos() == 1) {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 
                        //&& currentViewInMFIS[j].getPos() == 1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP1(currentViewInMFIS[j].X1(), currentViewInMFIS[j].Y1());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertTo = currentViewInMFIS[j].getID();

                        //checking whether this one is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                cout << "Hello" << endl;
                break;
            }
        }
    }

    //inserting LEFT side's objects(old)
    for (int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == -1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }

    //inserting NEW objects
    cout<<"InsertFrom "<<insertFrom<<endl;
    cout << "InsertTo: " << insertTo << endl;
    cout << "size: " << objectsBehindRobot.size() << endl;
    cout << "last id: " << cView[cView.size() - 1].getID() << endl;
    if (viewNumber == 87) {
        // insertTo = 14;
        //waitHere();
    }
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);
        new_Objects_GlobalMap.push_back(currentViewInMFIS[j]);

        //making target(i.e changing id as they have in MFIS) object for next step
        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                targetObjectsForNextStep.back().setID(lastObjectID + 1);
                break;
            }
        }

        lastObjectID++;
    }
    cout << endl << endl << "TargetObjects for Next step " << targetObjectsForNextStep.size() << endl;
    //displayObjects(targetObjectsForNextStep);

    //inserting RIGHT side's objects(old)
    for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == 1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }
    updatedPM.back().setID(lastObjectID + 1);
    cout << endl << endl << "UpdatedMFIS " << endl;
    //displayObjects(updatedPM);
    // plotObjects("Maps/updatedMFIS.png", updatedPM, firstRobotPosition);
    // waitHere();
    
    //char GUIFileName[100];
    //MyRobot InitRB(0,0);
    //sprintf(GUIFileName,"%s%d%s", "Maps/Offline/GUI-", viewNumber, ".png");
    //plotObjectsOf4Kinds(GUIFileName, cRobotPositionInMFIS, InitRB.getRobot(), new_Objects_GlobalMap, old_Objects_GlobalMap);
    //plotObjectsOf4Kinds(GUIFileName, allRobotPositions, InitRB.getRobot(), new_Objects_GlobalMap, old_Objects_GlobalMap);

    
    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    
     //add by wenwang
     //package.setDeletionID(lpToDelete);
    /*
    if(viewNumber == 153 && po != 0)
    {
            cout<<"the all surface ID : ";//<<updatedPM[1].getLocalEnvID().size()<<endl;
           for(int i = 0; i < updatedPM.size(); i++)
            {
               cout<<" "<<updatedPM[i].getLocalEnvID()[0];
           }
          waitHere();
    }
    else
        if(viewNumber == 153)
            po = 1;
   */ 
    return package;
}

void updateMapAtLastStep(Transporter lastStepInfo) {
    //project all target objects of last view onto MFIS to recognize
    vector<Object> targetObjectsInMFIS = projectingTheView(lastStepInfo.getTargetObjects(), lastStepInfo.getReferenceObjects()[0], lastStepInfo.getReferenceObjects()[1], 1);

    vector<Minfo> recognizedObject = recognizeObjects(targetObjectsInMFIS, lastStepInfo.getMFIS());
    cout << "Display Matched info" << endl;
    displayMinfo(recognizedObject);

    vector<Object> recogObjectInCV, recogObjectInMFIS;
    Object s;
    if (recognizedObject.size() > 5) {
        for (unsigned int ii = 0; ii < recognizedObject.size(); ii++) {
            s = getObject(targetObjectsInMFIS, recognizedObject[ii].getCID());
            recogObjectInCV.push_back(s);
            s = getObject(lastStepInfo.getMFIS(), recognizedObject[ii].getPID());
            recogObjectInMFIS.push_back(s);
        }
    }
    if (recogObjectInCV.size() > 5) {
        plotObjects("Maps/RecognizedObjectsInCV.png", recogObjectInCV, recogObjectInCV);
        plotObjects("Maps/RecognizedObjectsInMFIS.png", recogObjectInMFIS, recogObjectInMFIS);
    }

    plotObjects("Maps/MFISatlastStep.png", lastStepInfo.getMFIS(), targetObjectsInMFIS);
    //waitHere();
}

bool isBelongToSameSpace(int limitingPoint, vector<int> spaceIDs) {
    for (unsigned int i = 0; i < spaceIDs.size(); i++) {
        if (limitingPoint == spaceIDs[i]) {
            return true;
        }
    }
    return false;
}

bool isBelongToSameSpace(vector<int> limitingPoint, vector<int> spaceIDs) {
    for (unsigned int j = 0; j < limitingPoint.size(); j++) {
        for (unsigned int i = 0; i < spaceIDs.size(); i++) {
            if (limitingPoint[j] == spaceIDs[i]) {
                return true;
            }
        }
    }
    return false;
}

bool isThisIDAdded(int newID, vector<int> previousIDs) {
    for (unsigned int i = 0; i < previousIDs.size(); i++) {
        if (newID == previousIDs[i]) {
            return true;
        }
    }
    return false;
}

//return true if thisObject close to current view

bool isThisCloseToCurrentView(Object thisObject, vector<Object> currentView) {
    for (unsigned int i = 0; i < currentView.size(); i++) {
        if (thisObject.shortestDistanceWithObject(currentView[i]) < 2000)
            return true;
    }

    return false;
}

bool isThisCloseToCVPolygonBoundary(Object thisObject, vector<Object> polygonObjects, double distTh) {
    double distance;
    bool p1IsClose = false, p2IsClose = false;
    bool bigObject = false;
    for (unsigned int i = 0; i < polygonObjects.size(); i++) {
        distance = polygonObjects[i].shortestDistFrom(thisObject.X1(), thisObject.Y1());
        if (distance <  distTh) {
            p1IsClose = true;
        }
        distance = polygonObjects[i].shortestDistFrom(thisObject.X2(), thisObject.Y2());
        if (distance <  distTh) {
            p2IsClose = true;
        }
       
    }
     if(checkForIntersection(Object(0,0,0,3000),thisObject) == 1) {//to find big surf which maybe across cv
            bigObject = true;
        }

    if (p1IsClose == true && p2IsClose == true)
        return true;
    else if(bigObject == true)//for big across cv type surf
        return true;
    else
        return false;
}

bool findCrossedExit(vector<Object> & allCrossedExit, vector<Object> lastView,
        vector<Object> refObjects, vector<double> distanceAngle, int set) {
    bool exitCrossed = false;
    vector<Exit> exitsInPV = findShortestExits(lastView);
    vector<Object> exitsAsObjects = convertExitToObject(exitsInPV);

    Object robotPath, temp;
    if (set == 501 && lastView.back().getVN() == 164) {//for set 501
        temp.set(lastView[lastView.size() - 5].X1(), lastView[lastView.size() - 5].Y1(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }
    if (set == 501 && lastView.back().getVN() == 198) {//for set 501
        temp.set(lastView[0].X2(), lastView[0].Y2(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }

    if (set == 500 && lastView.back().getVN() == 852) {//for exit6 of set 500 
        temp.set(lastView[0].X2(), lastView[0].Y2(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }
    if (set == 500 && lastView.back().getVN() == 1027) {//for exit8 or kitchen entrance of set 500
        temp.set(lastView[3].X2(), lastView[3].Y2(),
                lastView[lastView.size() - 1].X2(), lastView[lastView.size() - 1].Y2(), 1);
        exitsAsObjects.push_back(temp);
    }
    if (set == 500 && lastView.back().getVN() == 1224) {//for exit10 or return home of set 500 
        temp.set(lastView[0].X2(), lastView[0].Y2(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }

    Point cRobotPositionInPV; //current robot position in previous view
    double angle = (distanceAngle[1] / 180) * PI; //angle in radian
    double rpx = (distanceAngle[0] + 300) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    double rpy = (distanceAngle[0] + 300) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
    cRobotPositionInPV.set(rpx, rpy);
    cout << endl << "v" << 1 + 1 << " dis(v12) " << distanceAngle[0] << " angle(v12) " << distanceAngle[1] << endl;
    cout << "robot position id-";
    cRobotPositionInPV.display();


    robotPath.set(0, 0, rpx, rpy, 1);

    vector<Object> dummy;
    dummy.push_back(robotPath);

    //checking for intersection of last view exits n robotPath
    for (unsigned int i = 0; i < exitsAsObjects.size(); i++) {
        cout << "exit length: " << exitsAsObjects[i].length() << endl;
        if (checkForIntersection(exitsAsObjects[i], robotPath) == 1 &&
                exitsAsObjects[i].length() > 750 && exitsAsObjects[i].length() < 1200) {
            dummy.push_back(exitsAsObjects[i]);
            cout << "I just crossed an Exit " << i << endl;
            temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[i], i + 1, 0, refObjects[0].getKP());
            if (allCrossedExit.back().distMPToPoint(temp.mpX(), temp.mpY()) > 3000) {
                allCrossedExit.push_back(temp);
                exitCrossed = true;
            }
        }
    }

    if (set == 501 && exitCrossed == false && lastView.back().getVN() == 133) {//for set 501
        temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[0], 1, 0, refObjects[0].getKP());
        allCrossedExit.push_back(temp);
        exitCrossed = true;
    }

    if (set == 500 && exitCrossed == false && lastView.back().getVN() == 121) {//for exit1 of set 500
        temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[0], 1, 0, refObjects[0].getKP());
        allCrossedExit.push_back(temp);
        exitCrossed = true;
    }

    if (set == 500 && exitCrossed == false && lastView.back().getVN() == 934) {//for exit7 or exit1repeat of set 500
        temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[0], 1, 0, refObjects[0].getKP());
        allCrossedExit.push_back(temp);
        exitCrossed = true;
    }

    char mfisFileName[100];
    sprintf(mfisFileName, "%s%d%s", "Maps/exitsCrossed-", lastView.back().getVN(), ".png");
    //plotObjectsOf3Kinds(mfisFileName,dummy,exitsAsObjects,lastView);

    return exitCrossed;
}

void updatePerceptualMapUsingPlaceInformation(vector<ASR> & places, vector<Object> & MFIS,
        vector<Object> cView, vector<Object> currentRobotPositionInMFIS, vector <Object> allRobotPositions,
        vector<Object> refobjects, int viewNumber, int ASRNumber, bool exitCrossed,
        vector<Object> crossedExit, vector<Object> & targetObjectsInPV, vector<Object> refObjectForLoopClosing,
        int updatingASRNumber, vector<Object> routeMap4CPlace) {

    plotPerceptualMap("Maps/PerceptualMapBeforeUpdating.png", currentRobotPositionInMFIS, MFIS, ASRNumber);

    vector<Object> oldPlaceOnNew;
    Object temp;
    //projecting old place on Current place
    for (unsigned int i = 0; i < MFIS.size(); i++) {
        if (MFIS[i].getASRNo() == updatingASRNumber) {
            temp = remakeLineP2(refObjectForLoopClosing[1], refObjectForLoopClosing[0], MFIS[i], MFIS[i].getID(), 0, 1);
            //oldPlaceOnNew.push_back(temp);
            MFIS[i].set(temp);

        }
    }
    //oldPlaceOnNew = projectingTheView(oldPlaceOnNew,refObjectForLoopClosing[1], refObjectForLoopClosing[0], 1);
    plotPerceptualMap("Maps/PerceptualMapAfterUpdating.png", currentRobotPositionInMFIS, MFIS, ASRNumber);
    //plotPerceptualMap("Maps/PerceptualMapAfterUpdating.png", oldPlaceOnNew, MFIS, ASRNumber);
    exitCrossed = false;
    Transporter computedOutput;
    computedOutput = updatePerceptualMapATPlace(places, MFIS, cView, currentRobotPositionInMFIS, allRobotPositions, refobjects,
            viewNumber, ASRNumber, exitCrossed, crossedExit, routeMap4CPlace);
    MFIS = computedOutput.getView();
    places = computedOutput.getASRs();
    targetObjectsInPV = computedOutput.getTargetObjects();
    plotObjects("Maps/finalUpdatedMFIS.png", currentRobotPositionInMFIS, MFIS);
    //waitHere();
}

void findBestReferenceObjectsUsingOdometryInfo(vector<Object> & referenceObjects,
        vector< vector<Object> > allRPoseAtOneStep, vector<Object> currentRobotPositionInMFIS,
        vector<double> coordTransInfo) {
    MyRobot myrobot(0, 0);
    double distErrorTh = 100.0;
    double angleErrorTh = 1.0;
    double angleError = 0, distanceError = 0;
    vector<Object> odometricReferenceObject, odometricCRPositionInMFIS;
    //localization using odometer
    Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
    aLine.setKP(1);
    odometricReferenceObject.clear();
    odometricReferenceObject.push_back(aLine);
    odometricReferenceObject.push_back(Object(0, 0, 0, 500, 1));
    odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0],
            odometricReferenceObject[1], odometricReferenceObject[0].getKP());
    
    //by default use odo
    referenceObjects = odometricReferenceObject;


    //this condition true means there are some recognized objects. so find best
    if (allRPoseAtOneStep.size() > 0) {
        distanceError = odometricCRPositionInMFIS[6].distP1ToP1(allRPoseAtOneStep[0][6]);
        angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(allRPoseAtOneStep[0][6]));
        if (distanceError < distErrorTh && angleError < angleErrorTh) {//means first one good enough
            referenceObjects.clear();
            referenceObjects.push_back(allRPoseAtOneStep[0][9]); //ref from mfis
            referenceObjects.push_back(allRPoseAtOneStep[0][10]); //ref from cv
        }
       
        for (unsigned int i = 1; i < allRPoseAtOneStep.size(); i++) {
            distanceError = odometricCRPositionInMFIS[6].distP1ToP1(allRPoseAtOneStep[i][6]);
            angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(allRPoseAtOneStep[i][6]));
            if (angleError > 350.0)
                angleError = 360.0 - angleError;
            //            if (angleError < angleErrorTh && distanceError < distErrorTh &&
            //                    angleError < abs(odometricCRPositionInMFIS[6].getAngleWithLine(allRPoseAtOneStep[i - 1][6]))) {
            if (angleError < angleErrorTh && distanceError < distErrorTh && 
                    distanceError < odometricCRPositionInMFIS[6].distP1ToP1(allRPoseAtOneStep[i - 1][6])) {
                referenceObjects.clear();
                referenceObjects.push_back(allRPoseAtOneStep[i][9]); //ref from mfis
                referenceObjects.push_back(allRPoseAtOneStep[i][10]); //ref from cv

            }
        }
    }   
    
}




/* modified by Lyn Pang
 * modified part is deletion process
 * delete surfaces that are in the polygon (must-be-removed surfaces) 
 * & behind those must-be-removed surfaces
 */

Transporter updatePerceptualMapATPlaceDeletion(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace) 
{
    cout << "\033[1;32m-------------------Inside updatePerceptualMap module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();

    //cout << endl << endl << "PerceptualMap" << endl;
    //displayObjects(perceptualMap);

    //cout << endl << endl << "Current View" << endl;
    //displayObjects(cView);
    cout << endl << endl << "Ref objects" << endl;
    //displayObjects(refobjects);
    
    cout << " the size of ref objects : " << refobjects.size() << endl;

    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //********************************************//
    // Variables for showcase interface programme //
    vector<Object> old_Objects_GlobalMap, new_Objects_GlobalMap;
    unsigned char inter_flag = 0; //intersection flag for deletion section
    //********************************************//

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();
    //    if(viewNumber == 87) //bug
    //        cViewSize = cView.size()-1;
    for (unsigned int i = 0; i < cViewSize; i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], cView[i], i + 1, 0, refobjects[0].getKP());
            if (cView[i].getPos() == 1)
                temp.setPos(1);
            else
                temp.setPos(-1);
            temp.setVN(currentViewNumber);
            temp.setOoPV(true);
            temp.setASRNo(ASRNumber);
            temp.setLimitingPoint(viewNumber);
            temp.setLocalEnvID(viewNumber);
            temp.setPO(cView[i].getPO());
            currentViewInMFIS.push_back(temp);
    }
    
    //demonstrating error during updating (only for print)
    if (refobjects[1].getColorID() == 100) 
    {
            vector<Object> dummy;
            char mfisNCVfileName[80];
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/MFISandCurrentView-", viewNumber, ".png");
            dummy.push_back(refobjects[0]);
            //plotObjectsOf4Kinds(mfisNCVfileName, cRobotPositionInMFIS, dummy, currentViewInMFIS, perceptualMap);
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);
            //plotObjectsOf3Kinds(mfisNCVfileName, myrobot.getRobot(), dummy, cView);
            //waitHere();
    }

    
    //constructing place    
    if (exitCrossed == true) //or viewNumber == 73) 
    {
            cout << "crossedExits: " << crossedExit.size() << endl;
            char placeFileName[80];
            vector<Object> place;

            ASR onePlace;
            vector<Object> exits;
            sprintf(placeFileName, "%s%d%s", "Maps/place-", ASRNumber - 1, ".png");
            for (unsigned int i = 0; i < perceptualMap.size(); i++) 
            {
                    if (perceptualMap[i].getASRNo() == ASRNumber - 1)
                        place.push_back(perceptualMap[i]);
            }

            //adding exits
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 2])); //exit1
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 1])); //exit2
            plotObjectsOf3Kinds(placeFileName, exits, routeMap4CPlace, place);

            onePlace.setASRObjects(place);
            onePlace.setASRExit1(crossedExit[crossedExit.size() - 2]);
            onePlace.setASRExit2(crossedExit[crossedExit.size() - 1]);
            onePlace.replaceTheWholeRoute(routeMap4CPlace); //this route just connecting limiting points
            places.push_back(onePlace);
    }


    //for a fresh ASR
    if (perceptualMap.size() == 0)
    {
            cout << "Current ASR is empty" << endl;
            //no need to go down
            Transporter package;
            package.setView(currentViewInMFIS);
            return package;
    }


    //computing all MFIS objects on CV to find objects in Current view
    Point currentRobPoint; //Point for current robot position in MFIS
    currentRobPoint.set(xAxisAtCRP.X1(), xAxisAtCRP.Y1());
    double angle = xAxisAtCRP.getAngleWithXaxis(); //Angle btw current robot x-axis and Original x-axis
    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);

    cout << "test programme angle : " << angle << endl << endl;
    //if(viewNumber >= 27)
    //    waitHere();
    vector<Object> pMapOnCV = xformPVIntoCV(perceptualMap, currentRobPoint, angle);

    //  plotObjects("Maps/MIFSonCV.png", cRobotPositionInMFIS, pMapOnCV);    

    vector<double> boundariesOfCV = findBoundariesOfCV(cView);
    
    //for printing only
    vector<Object> boundaryLines, pMapOnCVinsideCV;
    boundaryLines.push_back(Object(boundariesOfCV[0], 0, boundariesOfCV[0], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[0], boundariesOfCV[2], boundariesOfCV[1], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[1], 0, boundariesOfCV[1], boundariesOfCV[2]));

    //making polygon of CV
    vector<Surface> polygon = makePolygonOfCV(cView); //findExactBoudaryFrom(cView);//
    //vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    
    //modified by wenwang, building a polygon corresponding current view
    vector<Object> polygonObjects;
    ClipperLib::Path subj;
    Paths solution;
    ClipperOffset co;

    subj = viewConvertPath(cView); //convert view into Path
    co.AddPath(subj, jtMiter, etClosedPolygon);
    co.Execute(solution, 1000.0); // outward offset the polygon
    polygonObjects = PathsConvertView(solution); // convert polygon into view
    polygon = convertObjectToSurface(polygonObjects); // convert view into surfaces
    
    char polyFile[80];
    sprintf(polyFile,"%s%d%s", "Maps/Offline/MFIS&Poly-", viewNumber, ".png");
    plotObjectsOf3Kinds(polyFile, polygonObjects, pMapOnCV, cView);
    
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }
    

    //plotObjects("Maps/Offline/CV-Boundary.png",cView,polygonObjects);
    //if(viewNumber > 68)
    //{
    //    plotObjectsOf3Kinds("Maps/Offline/Polygon.png",xformPVIntoCV(allRobotPositions, currentRobPoint, angle),breakTheLinesInto(polygonObjects),pMapOnCV);    
    //    waitHere();
    //}
    

    vector<Object> exppandableObjects;
    vector<int> lpToDelete; //limiting points to be deleted.
    vector<Object> objectsBelongToSameSpace;
    vector<Object> deletedObjectsOnCV;
    vector<Object> deleted_front; //delete front info but not in polygon
    

    //     //saving local environment number 
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true ||
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) 
        {

            if (lpToDelete.size() == 0) 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++)
                    lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
            } 
            else 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++) 
                {
                        bool alreadySaved = false;
                        for (unsigned int lp = 0; lp < lpToDelete.size(); lp++) 
                        {
                                if (lpToDelete[lp] == perceptualMap[i].getLocalEnvID()[le]) 
                                {
                                        alreadySaved = true;
                                        break;
                                }
                        }
                        if (alreadySaved == false)
                        {
                            lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
                        }
                }
            }
        }
    }
    
 
    int expandableOnLeft = 0;
    int expandableOnRight = 0;
    
    //modified by Lyn Pang
    Object line_detect_behind;  //connect robot p with must-be-removed surface
    vector<Object> ref_lines;
    
    //Finding MFIS objects in CV
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        //changing position tag of those objects which appeared on opposite side but now should be other side
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() < 0 && perceptualMap[i].getPos() == 1 && perceptualMap[i].getOoPV() == true) 
        {
            if(i == 0)
                perceptualMap[i].setPos(-1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].getPos() == -1)//some objects position is be changed when robot takes big turn
                        perceptualMap[i].setPos(-1);
                }
        }
        
        //changing position tag
        if (pMapOnCV[i].X1() > 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {   
            if(i == 0)
                perceptualMap[i].setPos(1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].distP2ToP1(perceptualMap[i]) > 600 or perceptualMap[i - 1].getPos() == 1)
                        perceptualMap[i].setPos(1);

                }
        }

        //changing position tag
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {
            if (viewNumber == 84)
                perceptualMap[i].setPos(1);
        }
 
        //finding MFIS objects which are behind current robot position
        //       if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && perceptualMap[i].getOoPV() == true) {
        //       if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && 
        //                perceptualMap[i].getASRNo() == ASRNumber) {
        //       if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (perceptualMap[i].getOoPV() == true || 
        //            pMapOnCV[i].isThisInsideCV(boundariesOfCV) == true)) {
        //        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
        //                (pointInPolygon(PointXY(pMapOnCV[i].X1(),pMapOnCV[i].Y1()),polygon) == true || 
        //                pointInPolygon(PointXY(pMapOnCV[i].X2(),pMapOnCV[i].Y2()),polygon) == true)) {
        if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true || 
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) //||
                //(interSectWithLine(polygonObjects, Point(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), Point(pMapOnCV[i].X2(), pMapOnCV[i].Y2())) == true) 
                //&& ) 
        {

            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
            pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing
            deletedObjectsOnCV.push_back(pMapOnCV[i]);
            
            line_detect_behind.set(cRobotPositionInMFIS[6].X1(),cRobotPositionInMFIS[6].Y1(), perceptualMap[i].X1(), perceptualMap[i].Y1(),1);
            //potential_removed = intersectAllSurfaces(pMapOnCV, Point (0,0), Point (pMapOnCV[i].X1(), pMapOnCV[i].Y1()));
            ref_lines.push_back(line_detect_behind);

//            if (pMapOnCV[i].length() > 3000 && viewNumber == 128) {//special case for set11 and first corridor 
//                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
//                objectsBehindRobot.back().setOoPV(false);
//            }
            

        } 
        else 
        {
            //           if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
            //                   isBelongToSameSpace(perceptualMap[i].getLimitingPoint(),lpToDelete)==true) {

            //            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
            //                    isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true &&
            //                    isThisCloseToCurrentView(pMapOnCV[i],cView) == true) {//condition to delete surfaces which belong to same localEnv
            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) {
//            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true ) {
//            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (checkForIntersection(Object(0,0,0,2000),pMapOnCV[i]) || (isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true && 
//                      isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 2500.0) == true))) {
                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
              
                //line_detect_behind.set(cRobotPositionInMFIS[6].X1(),cRobotPositionInMFIS[6].Y1(), perceptualMap[i].X1(), perceptualMap[i].Y1(),1);
                //potential_removed = intersectAllSurfaces(pMapOnCV, Point (0,0), Point (pMapOnCV[i].X1(), pMapOnCV[i].Y1()));
                //ref_lines.push_back(line_detect_behind);
            } 
            else 
            {
                
                //if(interSectWithLine(ref_lines, Point (perceptualMap[i].X1(), perceptualMap[i].Y1()), Point (perceptualMap[i].X2(), perceptualMap[i].Y2())) == false)
                if((inDeleteASR(deletedObjectsOnCV, pMapOnCV[i]) == true)
                    && (pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0))
                {
                    objectsBelongToSameSpace.push_back(perceptualMap[i]);
                     deletedObjectsOnCV.push_back(pMapOnCV[i]);
                     
                     deleted_front.push_back(pMapOnCV[i]);
                }
                else
                {
                    objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                    objectsBehindRobot.back().setOoPV(false);
                }
                
            }


            //finding expandable objects on LEFT
            if (perceptualMap[i].getPos() == -1) 
            {
                if ((pMapOnCV[i].Y1() < 0 && pMapOnCV[i].Y2() > 0) || (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)) 
                {
                    //if(expandableOnLeft == 0) {
                    expandableOnLeft = perceptualMap[i].getID(); //storing last expandable objectID
                    // }
                    exppandableObjects.push_back(perceptualMap[i]);
                    cout << "LEFT: " << perceptualMap[i].getID() << endl;
                }
            }
           
            //finding expandable objects on RIGHT
            if (perceptualMap[i].getPos() == 1)
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                {

                    if (expandableOnRight == 0) 
                    {
                        expandableOnRight = perceptualMap[i].getID(); //storing last expandable objectID
                        exppandableObjects.push_back(perceptualMap[i]);
                        cout << "RIGHT: " << perceptualMap[i].getID() << endl;
                    }
                }
            
           
        }
        
    }
    
    //for detecting the surfaces that behind must-be-removed surfaces
    for(int i = 0; i < objectsBehindRobot.size(); i++)
    {
            if(interSectWithLine(ref_lines, Point (objectsBehindRobot[i].X1(), objectsBehindRobot[i].Y1()), 
                                            Point (objectsBehindRobot[i].X2(), objectsBehindRobot[i].Y2())) == true)
            {
                //objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                //objectsBehindRobot.back().setOoPV(false);
                
                objectsBehindRobot.erase(objectsBehindRobot.begin()+i);
                i = 0;
            }
    }

    //just for demonstration
    char mfisFileName[80];
    //sprintf(mfisFileName, "%s%d%s", "Maps/ObjectInsideCV-", viewNumber, ".png");
    //plotObjectsOf4Kinds(mfisFileName,polygonObjectsOnMFIS,objectsInSideCV,objectsBelongToSameSpace,objectsBehindRobot);
    if (objectsBelongToSameSpace.size() > 10) 
    {

        sprintf(mfisFileName, "%s%d%s", "Maps/ObjectInsideCV-", viewNumber, ".png");
        //if(viewNumber == 242 or viewNumber == 864 or viewNumber == 1009 or viewNumber == 1425) 
        //plotObjectsOf4Kinds(mfisFileName,pMapOnCV,pMapOnCVinsideCV,objectsBelongToSameSpace,polygonObjects);
        //waitHere();

        cout << "Current view in MFIS" << endl;
        //displayObjects(currentViewInMFIS);
        cout << "Obects behind CRP" << endl;
        //displayObjects(objectsBehindRobot);
        //displayObjects(objectsBelongToSameSpace);

        //cout << "lp " << lpToDelete.size() << " " << lpToDelete[0] << endl;
        //waitHere();
    }
    

    cout << "deletedObjects on CV" << endl;
    //displayObjects(deletedObjectsOnCV);

    cout << "CV in MFIS" << endl;
    //displayObjects(currentViewInMFIS);


    // plotObjects("Maps/objectBehindRnCV.png", objectsBehindRobot, firstRobotPosition);

    int lastObjectID = perceptualMap[perceptualMap.size() - 1].getID();
    vector<Object> targetObjectsInCV = findTargetObjects(cView);
    cout << endl << endl << "Target Objects in CV " << targetObjectsInCV.size() << endl;
    //displayObjects(targetObjectsInCV);
    vector<Object> targetObjectsForNextStep;


    double expAngle;
    double expDist;
    double similarity;
    int insertFrom = 0;
    int insertTo = currentViewInMFIS.size() + 1;


    //LEFT expanding objects 
    if (expandableOnLeft != 0) {//finding same object of cv on LEFT
        cout << "(LEFT) Object number " << expandableOnLeft << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) 
        {
            if (objectsBehindRobot[i].getID() == expandableOnLeft) 
            {
                for (unsigned int j = 0; j < currentViewInMFIS.size(); j++) {//will consider first same object of CV on LEFT
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && expDist < 500.0
                            && currentViewInMFIS[j].getPos() == -1) {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 && currentViewInMFIS[j].getPos() == -1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP2(currentViewInMFIS[j].X2(), currentViewInMFIS[j].Y2());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertFrom = currentViewInMFIS[j].getID();

                        //finding whether this object is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
                        {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                break;
            }
        }
    }

    //RIGHT side expending 
    if (expandableOnRight != 0) 
    {
        cout << "(RIGHT) Object number " << expandableOnRight << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnRight) {
                for (unsigned int j = currentViewInMFIS.size() - 1; j > 0; j--) {//will consider last same object of cv on RIGHT
                    cout << "finding same object from last " << j << endl;
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && expDist < 500.0
                            && currentViewInMFIS[j].getPos() == 1) {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 
                        //&& currentViewInMFIS[j].getPos() == 1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP1(currentViewInMFIS[j].X1(), currentViewInMFIS[j].Y1());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertTo = currentViewInMFIS[j].getID();

                        //checking whether this one is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                cout << "Hello" << endl;
                break;
            }
        }
    }
    

    //inserting LEFT side's objects(old)
    for (int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == -1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }

    //inserting NEW objects
    cout<<"InsertFrom "<<insertFrom<<endl;
    cout << "InsertTo: " << insertTo << endl;
    cout << "size: " << objectsBehindRobot.size() << endl;
    cout << "last id: " << cView[cView.size() - 1].getID() << endl;

    for (unsigned int j = insertFrom; j < insertTo - 1; j++) 
    {
        
        
        //modified bu Lyn Pang
        Object ref_mod, mod_current_Obj;
        ref_mod = similarObeject(updatedPM, currentViewInMFIS[j]);
        if(ref_mod.X1() != 0 && ref_mod.X2() != 0)
        {
            mod_current_Obj = modifyObject(currentViewInMFIS[j], ref_mod);
            
            //cout << " modified current surface info !!!!" << endl;
            //mod_current_Obj.display();
            
            //delete reference object in MFIS
            updatedPM = DeleteObjectFromCV(updatedPM, ref_mod);

            if(ref_mod.length() > 2000)
                updatedPM.push_back(mod_current_Obj);
            else
                updatedPM.push_back(currentViewInMFIS[j]);
            

        }
        else
        {   
            updatedPM.push_back(currentViewInMFIS[j]);
        }
        
        //updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);
        new_Objects_GlobalMap.push_back(currentViewInMFIS[j]);

        //making target(i.e changing id as they have in MFIS) object for next step
        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
        {
            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
            {
                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                targetObjectsForNextStep.back().setID(lastObjectID + 1);
                break;
            }
        }

        lastObjectID++;
    }
    
   
    
    cout << endl << endl << "TargetObjects for Next step " << targetObjectsForNextStep.size() << endl;
    //displayObjects(targetObjectsForNextStep);

    //inserting RIGHT side's objects(old)
    for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == 1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }
    updatedPM.back().setID(lastObjectID + 1);
    cout << endl << endl << "UpdatedMFIS " << endl;

    
    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    

    //sprintf(mfisFileName, "%s%d%s", "Maps/GlobalMFIS_with_deleting_info-", viewNumber, ".png");
    //plotObjectsOf3Kinds(mfisFileName,pMapOnCV,deletedObjectsOnCV,deleted_front);
    
    return package;
}

/* modified by Lyn Pang
 * modified part is deletion process
 * delete surfaces that are in the polygon (must-be-removed surfaces) 
 * & behind those must-be-removed surfaces
 * 
 * version 2.0
 */

Transporter updatePerceptualMapATPlaceDeletion_version2(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, unsigned char expectation_flag) 
{
    cout << "\033[1;32m-------------------Inside updatePerceptualMap module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();

    //cout << endl << endl << "PerceptualMap" << endl;
    //displayObjects(perceptualMap);

    //cout << endl << endl << "Current View" << endl;
    //displayObjects(cView);
    cout << endl << endl << "Ref objects" << endl;
    //displayObjects(refobjects);
    
    cout << " the size of ref objects : " << refobjects.size() << endl;

    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //********************************************//
    // Variables for showcase interface programme //
    vector<Object> old_Objects_GlobalMap, new_Objects_GlobalMap;
    unsigned char inter_flag = 0; //intersection flag for deletion section
    //********************************************//

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();
    //    if(viewNumber == 87) //bug
    //        cViewSize = cView.size()-1;
    for (unsigned int i = 0; i < cViewSize; i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], cView[i], i + 1, 0, refobjects[0].getKP());
            if (cView[i].getPos() == 1)
                temp.setPos(1);
            else
                temp.setPos(-1);
            temp.setVN(currentViewNumber);
            temp.setOoPV(true);
            temp.setASRNo(ASRNumber);
            temp.setLimitingPoint(viewNumber);
            temp.setLocalEnvID(viewNumber);
            temp.setPO(cView[i].getPO());
            currentViewInMFIS.push_back(temp);
    }
    
    //demonstrating error during updating (only for print)
    if (refobjects[1].getColorID() == 100) 
    {
            vector<Object> dummy;
            char mfisNCVfileName[80];
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/MFISandCurrentView-", viewNumber, ".png");
            dummy.push_back(refobjects[0]);
            //plotObjectsOf4Kinds(mfisNCVfileName, cRobotPositionInMFIS, dummy, currentViewInMFIS, perceptualMap);
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);
            //plotObjectsOf3Kinds(mfisNCVfileName, myrobot.getRobot(), dummy, cView);
            //waitHere();
    }

    
    //constructing place    
    if (exitCrossed == true) //or viewNumber == 73) 
    {
            cout << "crossedExits: " << crossedExit.size() << endl;
            char placeFileName[80];
            vector<Object> place;

            ASR onePlace;
            vector<Object> exits;
            sprintf(placeFileName, "%s%d%s", "Maps/place-", ASRNumber - 1, ".png");
            for (unsigned int i = 0; i < perceptualMap.size(); i++) 
            {
                    if (perceptualMap[i].getASRNo() == ASRNumber - 1)
                        place.push_back(perceptualMap[i]);
            }

            //adding exits
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 2])); //exit1
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 1])); //exit2
            plotObjectsOf3Kinds(placeFileName, exits, routeMap4CPlace, place);

            onePlace.setASRObjects(place);
            onePlace.setASRExit1(crossedExit[crossedExit.size() - 2]);
            onePlace.setASRExit2(crossedExit[crossedExit.size() - 1]);
            onePlace.replaceTheWholeRoute(routeMap4CPlace); //this route just connecting limiting points
            places.push_back(onePlace);
    }


    //for a fresh ASR
    if (perceptualMap.size() == 0)
    {
            cout << "Current ASR is empty" << endl;
            //no need to go down
            Transporter package;
            package.setView(currentViewInMFIS);
            return package;
    }


    //computing all MFIS objects on CV to find objects in Current view
    Point currentRobPoint; //Point for current robot position in MFIS
    currentRobPoint.set(xAxisAtCRP.X1(), xAxisAtCRP.Y1());
    double angle = xAxisAtCRP.getAngleWithXaxis(); //Angle btw current robot x-axis and Original x-axis
    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);

    cout << "test programme angle : " << angle << endl << endl;
    //if(viewNumber >= 27)
    //    waitHere();
    vector<Object> pMapOnCV = xformPVIntoCV(perceptualMap, currentRobPoint, angle);

    //  plotObjects("Maps/MIFSonCV.png", cRobotPositionInMFIS, pMapOnCV);    

    vector<double> boundariesOfCV = findBoundariesOfCV(cView);
    
    //for printing only
    vector<Object> boundaryLines, pMapOnCVinsideCV;
    boundaryLines.push_back(Object(boundariesOfCV[0], 0, boundariesOfCV[0], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[0], boundariesOfCV[2], boundariesOfCV[1], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[1], 0, boundariesOfCV[1], boundariesOfCV[2]));

    //making polygon of CV
    vector<Surface> polygon = makePolygonOfCV(cView); //findExactBoudaryFrom(cView);//
    //vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    
    //modified by wenwang, building a polygon corresponding current view
    vector<Object> polygonObjects;
    ClipperLib::Path subj;
    Paths solution;
    ClipperOffset co;

    subj = viewConvertPath(cView); //convert view into Path
    co.AddPath(subj, jtMiter, etClosedPolygon);
    //co.Execute(solution, 1000.0); // outward offset the polygon
    co.Execute(solution, 0.0);
    polygonObjects = PathsConvertView(solution); // convert polygon into view
    polygon = convertObjectToSurface(polygonObjects); // convert view into surfaces
    
    //char polyFile[80];
    //sprintf(polyFile,"%s%d%s", "Maps/Offline/MFIS&Poly-", viewNumber, ".png");
    //plotObjectsOf3Kinds(polyFile, polygonObjects, pMapOnCV, cView);
    //plotObjectsOf3Kinds(polyFile, myrobot.getRobot(), pMapOnCV, cView);
    
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }
    
    

    vector<Object> exppandableObjects;
    vector<int> lpToDelete; //limiting points to be deleted.
    vector<Object> objectsBelongToSameSpace;
    vector<Object> deletedObjectsOnCV;
    vector<Object> deleted_front; //delete front info but not in polygon
    

    //     //saving local environment number 
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true ||
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) 
        {

            if (lpToDelete.size() == 0) 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++)
                    lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
            } 
            else 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++) 
                {
                        bool alreadySaved = false;
                        for (unsigned int lp = 0; lp < lpToDelete.size(); lp++) 
                        {
                                if (lpToDelete[lp] == perceptualMap[i].getLocalEnvID()[le]) 
                                {
                                        alreadySaved = true;
                                        break;
                                }
                        }
                        if (alreadySaved == false)
                        {
                            lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
                        }
                }
            }
        }
    }
    
 
    int expandableOnLeft = 0;
    int expandableOnRight = 0;
    
    //modified by Lyn Pang
    Object line_detect_behind;  //connect robot p with must-be-removed surface
    vector<Object> ref_lines;
    vector< vector<Object> > plot_list;
    vector<Object> del_list;
    
    //Finding MFIS objects in CV
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        //changing position tag of those objects which appeared on opposite side but now should be other side
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() < 0 && perceptualMap[i].getPos() == 1 && perceptualMap[i].getOoPV() == true) 
        {
            if(i == 0)
                perceptualMap[i].setPos(-1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].getPos() == -1)//some objects position is be changed when robot takes big turn
                        perceptualMap[i].setPos(-1);
                }
        }
        
        //changing position tag
        if (pMapOnCV[i].X1() > 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {   
            if(i == 0)
                perceptualMap[i].setPos(1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].distP2ToP1(perceptualMap[i]) > 600 or perceptualMap[i - 1].getPos() == 1)
                        perceptualMap[i].setPos(1);

                }
        }

        
        Object del_ref, replace_ref, modified_surf;
        
        int ref_flag;
        
        
        //changing position tag
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {
            if (viewNumber == 84)
                perceptualMap[i].setPos(1);
        }
 
        
        
                //finding MFIS objects which are behind current robot position
                if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                        (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true  
                        || pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true
                        || interSectWithLine(polygonObjects, pMapOnCV[i].getP1(), pMapOnCV[i].getP2()) == true))
                        //&& ) 
                {

                    //if(expectation_flag != 1)
                    //{
                        objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
                        pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing

                        del_ref = pMapOnCV[i];
                        replace_ref = Compare_similarObeject(cView, del_ref, viewNumber);

                        if((replace_ref.getP1() != replace_ref.getP2())&&(del_ref.length() > replace_ref.length()) && (abs(del_ref.length()/replace_ref.length())<4.00))
                        {
                            modified_surf = currentViewInMFIS[replace_ref.getID()-1];

                            ref_flag = reference_endpoint(replace_ref, del_ref);
                            if(ref_flag == 2)
                            {
                                modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 1);
                                modified_surf.set(modified_surf.X2(), modified_surf.Y2(), currentViewInMFIS[replace_ref.getID()-1].X2(), currentViewInMFIS[replace_ref.getID()-1].Y2(), replace_ref.getID());
                            }
                            else
                            {
                                modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 2);
                                modified_surf.set(currentViewInMFIS[replace_ref.getID()-1].X1(), currentViewInMFIS[replace_ref.getID()-1].Y1(), modified_surf.X2(), modified_surf.Y2(), replace_ref.getID());   
                            }
                            
                            currentViewInMFIS.erase(currentViewInMFIS.begin()+replace_ref.getID()-1);
                            currentViewInMFIS.insert(currentViewInMFIS.begin()+replace_ref.getID()-1, modified_surf);

                            deletion_reference.push_back(modified_surf);
                            deletion_reference.push_back(perceptualMap[i]);
                            del_ref_list.push_back(deletion_reference);
                            deletion_reference.clear();
                        }

                        deletedObjectsOnCV.push_back(pMapOnCV[i]);
                        del_list.push_back(perceptualMap[i]);
                        line_detect_behind.set(cRobotPositionInMFIS[6].X1(),cRobotPositionInMFIS[6].Y1(), perceptualMap[i].X1(), perceptualMap[i].Y1(),1);
                        //potential_removed = intersectAllSurfaces(pMapOnCV, Point (0,0), Point (pMapOnCV[i].X1(), pMapOnCV[i].Y1()));
                        ref_lines.push_back(line_detect_behind);
                    //}
                    //else
                    //    goto lbb;


                } 
                else 
                {

                    if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) 
                    {
                        //if(expectation_flag != 1)
                        //{
                            del_ref = pMapOnCV[i];
                            replace_ref = Compare_similarObeject(cView, del_ref, viewNumber);

                            if((replace_ref.getP1() != replace_ref.getP2())&&(del_ref.length() > replace_ref.length()) && (abs(del_ref.length()/replace_ref.length())<4.00))
                            {
                                modified_surf = currentViewInMFIS[replace_ref.getID()-1];

                                ref_flag = reference_endpoint(replace_ref, del_ref);
                                if(ref_flag == 2)
                                {
                                    modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 1);
                                    modified_surf.set(modified_surf.X2(), modified_surf.Y2(), currentViewInMFIS[replace_ref.getID()-1].X2(), currentViewInMFIS[replace_ref.getID()-1].Y2(), replace_ref.getID());
                                }
                                else
                                {
                                    modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 2);
                                    modified_surf.set(currentViewInMFIS[replace_ref.getID()-1].X1(), currentViewInMFIS[replace_ref.getID()-1].Y1(), modified_surf.X2(), modified_surf.Y2(), replace_ref.getID());   
                                }
                                currentViewInMFIS.erase(currentViewInMFIS.begin()+replace_ref.getID()-1);
                                currentViewInMFIS.insert(currentViewInMFIS.begin()+replace_ref.getID()-1, modified_surf);

                                deletion_reference.push_back(modified_surf);
                                deletion_reference.push_back(perceptualMap[i]);
                                del_ref_list.push_back(deletion_reference);
                                deletion_reference.clear();
                            }

                            objectsBelongToSameSpace.push_back(perceptualMap[i]);
                            deletedObjectsOnCV.push_back(pMapOnCV[i]);
                            del_list.push_back(perceptualMap[i]);
                        //}
                        //else
                        //    goto lbb;

                    } 
                    else 
                    {

                        //if(interSectWithLine(ref_lines, Point (perceptualMap[i].X1(), perceptualMap[i].Y1()), Point (perceptualMap[i].X2(), perceptualMap[i].Y2())) == false)
                        if((inDeleteASR(deletedObjectsOnCV, pMapOnCV[i]) == true)
                            && (pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0))
                        {
                            //if(expectation_flag != 1)
                            //{
                                del_ref = pMapOnCV[i];
                                replace_ref = Compare_similarObeject(cView, del_ref, viewNumber);


                                if((replace_ref.getP1() != replace_ref.getP2())&&(del_ref.length() > replace_ref.length()) && (abs(del_ref.length()/replace_ref.length())<4.00))
                                {
                                    modified_surf = currentViewInMFIS[replace_ref.getID()-1];

                                    ref_flag = reference_endpoint(replace_ref, del_ref);
                                    if(ref_flag == 2)
                                    {
                                        modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 1);
                                        modified_surf.set(modified_surf.X2(), modified_surf.Y2(), currentViewInMFIS[replace_ref.getID()-1].X2(), currentViewInMFIS[replace_ref.getID()-1].Y2(), replace_ref.getID());
                                    }
                                    else
                                    {
                                        modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 2)  ;
                                        modified_surf.set(currentViewInMFIS[replace_ref.getID()-1].X1(), currentViewInMFIS[replace_ref.getID()-1].Y1(), modified_surf.X2(), modified_surf.Y2(), replace_ref.getID());   
                                    }
                                    currentViewInMFIS.erase(currentViewInMFIS.begin()+replace_ref.getID()-1);
                                    currentViewInMFIS.insert(currentViewInMFIS.begin()+replace_ref.getID()-1, modified_surf);

                                    deletion_reference.push_back(modified_surf);
                                    deletion_reference.push_back(perceptualMap[i]);
                                    del_ref_list.push_back(deletion_reference);
                                    deletion_reference.clear();
                                }
 deletion_reference.push_back(modified_surf);
                                    deletion_reference.push_back(perceptualMap[i]);
                                    del_ref_list.push_back(deletion_reference);
                                    deletion_reference.clear();

                                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                                deleted_front.push_back(pMapOnCV[i]);
                                del_list.push_back(perceptualMap[i]);
                            //}
                            //else
                            //    goto lbb;
                        }
                        else
                        {
                            //if((pMapOnCV[i].length()>3000) && ((pMapOnCV[i].X1() * pMapOnCV[i].X2()) < 0)
                            // && ((pMapOnCV[i].Y1() * pMapOnCV[i].Y2()) < 0) && (shortestDistanceBtwTwoObjects(myrobot.getRobot()[6], pMapOnCV[i]) < 500))
                            //{
                            //    deletedObjectsOnCV.push_back(pMapOnCV[i]);
                            //    del_list.push_back(perceptualMap[i]);
                            //}
                            //else
                            //{
                            
lbb:                            objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                                objectsBehindRobot.back().setOoPV(false);
                            //}
                        }

                    }


                    //finding expandable objects on LEFT
                    if (perceptualMap[i].getPos() == -1) 
                    {
                        if ((pMapOnCV[i].Y1() < 0 && pMapOnCV[i].Y2() > 0) || (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)) 
                        {
                            //if(expandableOnLeft == 0) {
                            expandableOnLeft = perceptualMap[i].getID(); //storing last expandable objectID
                            // }
                            exppandableObjects.push_back(perceptualMap[i]);
                            cout << "LEFT: " << perceptualMap[i].getID() << endl;
                        }
                    }

                    //finding expandable objects on RIGHT
                    if (perceptualMap[i].getPos() == 1)
                        if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                        {

                            if (expandableOnRight == 0) 
                            {
                                expandableOnRight = perceptualMap[i].getID(); //storing last expandable objectID
                                exppandableObjects.push_back(perceptualMap[i]);
                                cout << "RIGHT: " << perceptualMap[i].getID() << endl;
                            }
                        }


                }
        
        
    }

    //just for demonstration
    char mfisFileName[80];

    // plotObjects("Maps/objectBehindRnCV.png", objectsBehindRobot, firstRobotPosition);

    int lastObjectID = perceptualMap[perceptualMap.size() - 1].getID();
    vector<Object> targetObjectsInCV = findTargetObjects(cView);
    cout << endl << endl << "Target Objects in CV " << targetObjectsInCV.size() << endl;
    //displayObjects(targetObjectsInCV);
    vector<Object> targetObjectsForNextStep;


    double expAngle;
    double expDist;
    double similarity;
    int insertFrom = 0;
    int insertTo = currentViewInMFIS.size() + 1;


    //LEFT expanding objects 
    if (expandableOnLeft != 0) {//finding same object of cv on LEFT
        cout << "(LEFT) Object number " << expandableOnLeft << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) 
        {
            if (objectsBehindRobot[i].getID() == expandableOnLeft) 
            {
                for (unsigned int j = 0; j < currentViewInMFIS.size(); j++) {//will consider first same object of CV on LEFT
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && (expDist < 500.0)
                            && (currentViewInMFIS[j].getPos() == -1)) 
                    {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 && currentViewInMFIS[j].getPos() == -1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        
//                        if(currentViewInMFIS[j].getAngleWithXaxis() > objectsBehindRobot[i].getAngleWithXaxis())
                            objectsBehindRobot[i].setP2(currentViewInMFIS[j].X2(), currentViewInMFIS[j].Y2());
//                        else
//                        {
//                        
//                            Object temp_expend = expend_Object(objectsBehindRobot[i], currentViewInMFIS[j].length(), 2);
//                            Point perpend_cross_point = crossPerpend(temp_expend.getP1(), temp_expend.getP2(), currentViewInMFIS[j].getP2());
//
//                            objectsBehindRobot[i].setP2(perpend_cross_point.X(), perpend_cross_point.Y());
//                        }
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertFrom = currentViewInMFIS[j].getID();

                        //finding whether this object is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
                        {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                break;
            }
        }
    }

    //RIGHT side expending 
    if (expandableOnRight != 0) 
    {
        cout << "(RIGHT) Object number " << expandableOnRight << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) 
        {
            if (objectsBehindRobot[i].getID() == expandableOnRight) 
            {
                for (unsigned int j = currentViewInMFIS.size() - 1; j > 0; j--) {//will consider last same object of cv on RIGHT
                    cout << "finding same object from last " << j << endl;
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && (expDist < 500.0)
                            && (currentViewInMFIS[j].getPos() == 1)) 
                    {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 
                        //&& currentViewInMFIS[j].getPos() == 1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        
//                        if(currentViewInMFIS[j].getAngleWithXaxis() > objectsBehindRobot[i].getAngleWithXaxis())
                            objectsBehindRobot[i].setP1(currentViewInMFIS[j].X1(), currentViewInMFIS[j].Y1());
//                        else
//                        {
//                            Object temp_expend = expend_Object(objectsBehindRobot[i], currentViewInMFIS[j].length(), 1);
//                            Point perpend_cross_point = crossPerpend(temp_expend.getP1(), temp_expend.getP2(), currentViewInMFIS[j].getP1());
//
//                            objectsBehindRobot[i].setP1(perpend_cross_point.X(), perpend_cross_point.Y());
//                        }
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertTo = currentViewInMFIS[j].getID();

                        //checking whether this one is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
                        {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                //cout << "Hello" << endl;
                break;
            }
        }
    }
    

    //inserting LEFT side's objects(old)
    for (int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == -1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }

    for (unsigned int j = insertFrom; j < insertTo - 1; j++) 
    {
        
        
        //modified bu Lyn Pang
        Object ref_mod, mod_current_Obj;

            ref_mod = similarObeject(updatedPM, currentViewInMFIS[j]);
            if(ref_mod.X1() != 0 && ref_mod.X2() != 0)
            {
                mod_current_Obj = modifyObject(currentViewInMFIS[j], ref_mod);

                //cout << " modified current surface info !!!!" << endl;
                //mod_current_Obj.display();

                //delete reference object in MFIS
                updatedPM = DeleteObjectFromCV(updatedPM, ref_mod);

                if(ref_mod.length() > 2000)
                    updatedPM.push_back(mod_current_Obj);
                else
                    updatedPM.push_back(currentViewInMFIS[j]);


            }
            else
            {   
                updatedPM.push_back(currentViewInMFIS[j]);
            }

        
        //updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);
        new_Objects_GlobalMap.push_back(currentViewInMFIS[j]);
        
        
        //making target(i.e changing id as they have in MFIS) object for next step
        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
        {
            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
            {
                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                targetObjectsForNextStep.back().setID(lastObjectID + 1);
                break;
            }
        }

        lastObjectID++;
    }
    
   
    
    cout << endl << endl << "TargetObjects for Next step " << targetObjectsForNextStep.size() << endl;
    //displayObjects(targetObjectsForNextStep);

    //inserting RIGHT side's objects(old)
    for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == 1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }
    updatedPM.back().setID(lastObjectID + 1);
    cout << endl << endl << "UpdatedMFIS " << endl;

    
//    for(int i = 0; i < del_list.size(); i++)
//    {
//        if((viewNumber - del_list[i].getVN() > 10) && (critical_reach_flag != 1)&&(del_list[i].getVN() > 0))
//        {
//                critical_reach_point = viewNumber;
//                collect_asr_number = del_list[i].getVN(); 
//                critical_reach_flag = 1;
//                cout << "current view number : " << viewNumber << endl;
//                cout << "collect_asr_number  : " << collect_asr_number << endl;
//                waitHere();
//        }
//    }
    //sprintf(mfisFileName, "%s%d%s", "Maps/Offline/updated_mfis_and_deletion-", viewNumber, ".png");                   
    //plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), updatedPM, del_list);
    
    
    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    
    
    return package;
}

/* modified by Lyn Pang
 * modified part is deletion process
 * delete surfaces that are in the polygon (must-be-removed surfaces) 
 * & behind those must-be-removed surfaces
 * 
 * for computing multi-globalmap
 * 
 * version 3.0
 */

Transporter updatePerceptualMapATPlaceDeletion_version3(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, unsigned char expectation_flag) 
{
    cout << "\033[1;32m-------------------Inside updatePerceptualMap module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();

    //cout << endl << endl << "PerceptualMap" << endl;
    //displayObjects(perceptualMap);

    //cout << endl << endl << "Current View" << endl;
    //displayObjects(cView);
    cout << endl << endl << "Ref objects" << endl;
    //displayObjects(refobjects);
    
    cout << " the size of ref objects : " << refobjects.size() << endl;

    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //********************************************//
    // Variables for showcase interface programme //
    vector<Object> old_Objects_GlobalMap, new_Objects_GlobalMap;
    unsigned char inter_flag = 0; //intersection flag for deletion section
    //********************************************//

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();
    //    if(viewNumber == 87) //bug
    //        cViewSize = cView.size()-1;
    for (unsigned int i = 0; i < cViewSize; i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], cView[i], i + 1, 0, refobjects[0].getKP());
            if (cView[i].getPos() == 1)
                temp.setPos(1);
            else
                temp.setPos(-1);
            temp.setVN(currentViewNumber);
            temp.setOoPV(true);
            temp.setASRNo(ASRNumber);
            temp.setLimitingPoint(viewNumber);
            temp.setLocalEnvID(viewNumber);
            temp.setPO(cView[i].getPO());
            currentViewInMFIS.push_back(temp);
    }
    
    //demonstrating error during updating (only for print)
    if (refobjects[1].getColorID() == 100) 
    {
            vector<Object> dummy;
            char mfisNCVfileName[80];
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/MFISandCurrentView-", viewNumber, ".png");
            dummy.push_back(refobjects[0]);
            //plotObjectsOf4Kinds(mfisNCVfileName, cRobotPositionInMFIS, dummy, currentViewInMFIS, perceptualMap);
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);
            //plotObjectsOf3Kinds(mfisNCVfileName, myrobot.getRobot(), dummy, cView);
            //waitHere();
    }

    
    //constructing place    
    if (exitCrossed == true) //or viewNumber == 73) 
    {
            cout << "crossedExits: " << crossedExit.size() << endl;
            char placeFileName[80];
            vector<Object> place;

            ASR onePlace;
            vector<Object> exits;
            sprintf(placeFileName, "%s%d%s", "Maps/place-", ASRNumber - 1, ".png");
            for (unsigned int i = 0; i < perceptualMap.size(); i++) 
            {
                    if (perceptualMap[i].getASRNo() == ASRNumber - 1)
                        place.push_back(perceptualMap[i]);
            }

            //adding exits
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 2])); //exit1
            exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 1])); //exit2
            plotObjectsOf3Kinds(placeFileName, exits, routeMap4CPlace, place);

            onePlace.setASRObjects(place);
            onePlace.setASRExit1(crossedExit[crossedExit.size() - 2]);
            onePlace.setASRExit2(crossedExit[crossedExit.size() - 1]);
            onePlace.replaceTheWholeRoute(routeMap4CPlace); //this route just connecting limiting points
            places.push_back(onePlace);
    }


    //for a fresh ASR
    if (perceptualMap.size() == 0)
    {
            cout << "Current ASR is empty" << endl;
            //no need to go down
            Transporter package;
            package.setView(currentViewInMFIS);
            return package;
    }


    //computing all MFIS objects on CV to find objects in Current view
    Point currentRobPoint; //Point for current robot position in MFIS
    currentRobPoint.set(xAxisAtCRP.X1(), xAxisAtCRP.Y1());
    double angle = xAxisAtCRP.getAngleWithXaxis(); //Angle btw current robot x-axis and Original x-axis
    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);

    vector<Object> pMapOnCV = xformPVIntoCV(perceptualMap, currentRobPoint, angle);

    //  plotObjects("Maps/MIFSonCV.png", cRobotPositionInMFIS, pMapOnCV);    

    vector<double> boundariesOfCV = findBoundariesOfCV(cView);
    
    //for printing only
    vector<Object> boundaryLines, pMapOnCVinsideCV;
    boundaryLines.push_back(Object(boundariesOfCV[0], 0, boundariesOfCV[0], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[0], boundariesOfCV[2], boundariesOfCV[1], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[1], 0, boundariesOfCV[1], boundariesOfCV[2]));

    //making polygon of CV
    vector<Surface> polygon = makePolygonOfCV(cView); //findExactBoudaryFrom(cView);//
    //vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    
    //modified by wenwang, building a polygon corresponding current view
    vector<Object> polygonObjects;
    ClipperLib::Path subj;
    Paths solution;
    ClipperOffset co;

    subj = viewConvertPath(cView); //convert view into Path
    co.AddPath(subj, jtMiter, etClosedPolygon);
    //co.Execute(solution, 1000.0); // outward offset the polygon
    co.Execute(solution, 0.0);
    polygonObjects = PathsConvertView(solution); // convert polygon into view
    polygon = convertObjectToSurface(polygonObjects); // convert view into surfaces
    
    char polyFile[80];
    //sprintf(polyFile,"%s%d%s", "Maps/Offline/MFIS&Poly-", viewNumber, ".png");
    //plotObjectsOf3Kinds(polyFile, polygonObjects, pMapOnCV, cView);
    //plotObjectsOf3Kinds(polyFile, myrobot.getRobot(), pMapOnCV, cView);
    
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }
    
    

    vector<Object> exppandableObjects;
    vector<int> lpToDelete; //limiting points to be deleted.
    vector<Object> objectsBelongToSameSpace;
    vector<Object> deletedObjectsOnCV;
    vector<Object> deleted_front; //delete front info but not in polygon
    

    //     //saving local environment number 
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true ||
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) 
        {

            if (lpToDelete.size() == 0) 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++)
                    lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
            } 
            else 
            {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++) 
                {
                        bool alreadySaved = false;
                        for (unsigned int lp = 0; lp < lpToDelete.size(); lp++) 
                        {
                                if (lpToDelete[lp] == perceptualMap[i].getLocalEnvID()[le]) 
                                {
                                        alreadySaved = true;
                                        break;
                                }
                        }
                        if (alreadySaved == false)
                        {
                            lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
                        }
                }
            }
        }
    }
    
 
    int expandableOnLeft = 0;
    int expandableOnRight = 0;
    
    //modified by Lyn Pang
    Object line_detect_behind;  //connect robot p with must-be-removed surface
    vector<Object> ref_lines;
    vector< vector<Object> > plot_list;
    vector<Object> del_list;
    
    //Finding MFIS objects in CV
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) 
    {
        //changing position tag of those objects which appeared on opposite side but now should be other side
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() < 0 && perceptualMap[i].getPos() == 1 && perceptualMap[i].getOoPV() == true) 
        {
            if(i == 0)
                perceptualMap[i].setPos(-1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].getPos() == -1)//some objects position is be changed when robot takes big turn
                        perceptualMap[i].setPos(-1);
                }
        }
        
        //changing position tag
        if (pMapOnCV[i].X1() > 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {   
            if(i == 0)
                perceptualMap[i].setPos(1);
            else
                if(i > 0)
                {
                    if (perceptualMap[i - 1].distP2ToP1(perceptualMap[i]) > 600 or perceptualMap[i - 1].getPos() == 1)
                        perceptualMap[i].setPos(1);

                }
        }

        
        Object del_ref, replace_ref, modified_surf;
        
        int ref_flag;
        
        
        //changing position tag
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) 
        {
            if (viewNumber == 84)
                perceptualMap[i].setPos(1);
        }
 
        
        
                //finding MFIS objects which are behind current robot position
                if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                        (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true  
                        || pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true
                        || interSectWithLine(polygonObjects, pMapOnCV[i].getP1(), pMapOnCV[i].getP2()) == true))
                        //&& ) 
                {

                        objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
                        pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing

                        del_ref = pMapOnCV[i];
                        replace_ref = Compare_similarObeject(cView, del_ref, viewNumber);

                        if((replace_ref.getP1() != replace_ref.getP2())&&(del_ref.length() > replace_ref.length()) && (abs(del_ref.length()/replace_ref.length())<4.00))
                        {
                            modified_surf = currentViewInMFIS[replace_ref.getID()-1];

                            ref_flag = reference_endpoint(replace_ref, del_ref);
                            if(ref_flag == 2)
                            {
                                modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 1);
                                modified_surf.set(modified_surf.X2(), modified_surf.Y2(), currentViewInMFIS[replace_ref.getID()-1].X2(), currentViewInMFIS[replace_ref.getID()-1].Y2(), replace_ref.getID());
                            }
                            else
                            {
                                modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 2);
                                modified_surf.set(currentViewInMFIS[replace_ref.getID()-1].X1(), currentViewInMFIS[replace_ref.getID()-1].Y1(), modified_surf.X2(), modified_surf.Y2(), replace_ref.getID());   
                            }
                            modified_surf.setVN(currentViewInMFIS[replace_ref.getID()-1].getVN());
                            currentViewInMFIS.erase(currentViewInMFIS.begin()+replace_ref.getID()-1);
                            currentViewInMFIS.insert(currentViewInMFIS.begin()+replace_ref.getID()-1, modified_surf);

                            deletion_reference.push_back(modified_surf);
                            deletion_reference.push_back(perceptualMap[i]);
                            del_ref_list.push_back(deletion_reference);
                            deletion_reference.clear();
                        }

                        deletedObjectsOnCV.push_back(pMapOnCV[i]);
                        del_list.push_back(perceptualMap[i]);
                        line_detect_behind.set(cRobotPositionInMFIS[6].X1(),cRobotPositionInMFIS[6].Y1(), perceptualMap[i].X1(), perceptualMap[i].Y1(),1);
                        //potential_removed = intersectAllSurfaces(pMapOnCV, Point (0,0), Point (pMapOnCV[i].X1(), pMapOnCV[i].Y1()));
                        ref_lines.push_back(line_detect_behind);

                } 
                else 
                {

                    if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) 
                    {

                            del_ref = pMapOnCV[i];
                            replace_ref = Compare_similarObeject(cView, del_ref, viewNumber);

                            if((replace_ref.getP1() != replace_ref.getP2())&&(del_ref.length() > replace_ref.length()) && (abs(del_ref.length()/replace_ref.length())<4.00))
                            {
                                modified_surf = currentViewInMFIS[replace_ref.getID()-1];

                                ref_flag = reference_endpoint(replace_ref, del_ref);
                                if(ref_flag == 2)
                                {
                                    modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 1);
                                    modified_surf.set(modified_surf.X2(), modified_surf.Y2(), currentViewInMFIS[replace_ref.getID()-1].X2(), currentViewInMFIS[replace_ref.getID()-1].Y2(), replace_ref.getID());
                                }
                                else
                                {
                                    modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 2);
                                    modified_surf.set(currentViewInMFIS[replace_ref.getID()-1].X1(), currentViewInMFIS[replace_ref.getID()-1].Y1(), modified_surf.X2(), modified_surf.Y2(), replace_ref.getID());   
                                }
                                modified_surf.setVN(currentViewInMFIS[replace_ref.getID()-1].getVN());
                                currentViewInMFIS.erase(currentViewInMFIS.begin()+replace_ref.getID()-1);
                                currentViewInMFIS.insert(currentViewInMFIS.begin()+replace_ref.getID()-1, modified_surf);

                                deletion_reference.push_back(modified_surf);
                                deletion_reference.push_back(perceptualMap[i]);
                                del_ref_list.push_back(deletion_reference);
                                deletion_reference.clear();
                            }

                            objectsBelongToSameSpace.push_back(perceptualMap[i]);
                            deletedObjectsOnCV.push_back(pMapOnCV[i]);
                            del_list.push_back(perceptualMap[i]);


                    } 
                    else 
                    {

                        //if(interSectWithLine(ref_lines, Point (perceptualMap[i].X1(), perceptualMap[i].Y1()), Point (perceptualMap[i].X2(), perceptualMap[i].Y2())) == false)
                        if((inDeleteASR(deletedObjectsOnCV, pMapOnCV[i]) == true)
                            && (pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0))
                        {

                                del_ref = pMapOnCV[i];
                                replace_ref = Compare_similarObeject(cView, del_ref, viewNumber);


                                if((replace_ref.getP1() != replace_ref.getP2())&&(del_ref.length() > replace_ref.length()) && (abs(del_ref.length()/replace_ref.length())<4.00))
                                {
                                    modified_surf = currentViewInMFIS[replace_ref.getID()-1];

                                    ref_flag = reference_endpoint(replace_ref, del_ref);
                                    if(ref_flag == 2)
                                    {
                                        modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 1);
                                        modified_surf.set(modified_surf.X2(), modified_surf.Y2(), currentViewInMFIS[replace_ref.getID()-1].X2(), currentViewInMFIS[replace_ref.getID()-1].Y2(), replace_ref.getID());
                                    }
                                    else
                                    {
                                        modified_surf = expend_Object(modified_surf, del_ref.length()-replace_ref.length(), 2)  ;
                                        modified_surf.set(currentViewInMFIS[replace_ref.getID()-1].X1(), currentViewInMFIS[replace_ref.getID()-1].Y1(), modified_surf.X2(), modified_surf.Y2(), replace_ref.getID());   
                                    }
                                    modified_surf.setVN(currentViewInMFIS[replace_ref.getID()-1].getVN());
                                    currentViewInMFIS.erase(currentViewInMFIS.begin()+replace_ref.getID()-1);
                                    currentViewInMFIS.insert(currentViewInMFIS.begin()+replace_ref.getID()-1, modified_surf);

                                    deletion_reference.push_back(modified_surf);
                                    deletion_reference.push_back(perceptualMap[i]);
                                    del_ref_list.push_back(deletion_reference);
                                    deletion_reference.clear();
                                }
                                    deletion_reference.push_back(modified_surf);
                                    deletion_reference.push_back(perceptualMap[i]);
                                    del_ref_list.push_back(deletion_reference);
                                    deletion_reference.clear();

                                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                                deleted_front.push_back(pMapOnCV[i]);
                                del_list.push_back(perceptualMap[i]);

                        }
                        else
                        {
                            if((pMapOnCV[i].length()>3000) && ((pMapOnCV[i].X1() * pMapOnCV[i].X2()) < 0)
                             && ((pMapOnCV[i].Y1() * pMapOnCV[i].Y2()) < 0) && (shortestDistanceBtwTwoObjects(myrobot.getRobot()[6], pMapOnCV[i]) < 500))
                            {
                                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                                del_list.push_back(perceptualMap[i]);
                            }
                            else
                            {
                            
                                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                                objectsBehindRobot.back().setOoPV(false);
                            }
                        }

                    }


                    //finding expandable objects on LEFT
                    if (perceptualMap[i].getPos() == -1) 
                    {
                        if ((pMapOnCV[i].Y1() < 0 && pMapOnCV[i].Y2() > 0) || (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)) 
                        {

                            expandableOnLeft = perceptualMap[i].getID(); //storing last expandable objectID
                     
                            exppandableObjects.push_back(perceptualMap[i]);
                            cout << "LEFT: " << perceptualMap[i].getID() << endl;
                        }
                    }

                    //finding expandable objects on RIGHT
                    if (perceptualMap[i].getPos() == 1)
                    {
                        if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                        {

                            if (expandableOnRight == 0) 
                            {
                                expandableOnRight = perceptualMap[i].getID(); //storing last expandable objectID
                                exppandableObjects.push_back(perceptualMap[i]);
                                cout << "RIGHT: " << perceptualMap[i].getID() << endl;
                            }
                        }
                    }


//                    if((del_list.size() > 0) && (currentViewNumber >= 34))
//                    {
//                        sprintf(polyFile,"%s", "Maps/Offline/Delet_info.png");
//                        plotObjects(polyFile, del_list);
//                        //waitHere();
//                    }
                }
        
        
    }

    //just for demonstration
    char mfisFileName[80];

    // plotObjects("Maps/objectBehindRnCV.png", objectsBehindRobot, firstRobotPosition);

    int lastObjectID = perceptualMap[perceptualMap.size() - 1].getID();
    vector<Object> targetObjectsInCV = findTargetObjects(cView);
    cout << endl << endl << "Target Objects in CV " << targetObjectsInCV.size() << endl;
    //displayObjects(targetObjectsInCV);
    vector<Object> targetObjectsForNextStep;


    double expAngle;
    double expDist;
    double similarity;
    int insertFrom = 0;
    int insertTo = currentViewInMFIS.size() + 1;


    //LEFT expanding objects 
    if (expandableOnLeft != 0) {//finding same object of cv on LEFT
        cout << "(LEFT) Object number " << expandableOnLeft << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) 
        {
            if (objectsBehindRobot[i].getID() == expandableOnLeft) 
            {
                for (unsigned int j = 0; j < currentViewInMFIS.size(); j++) {//will consider first same object of CV on LEFT
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && (expDist < 500.0)
                            && (currentViewInMFIS[j].getPos() == -1)) 
                    {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 && currentViewInMFIS[j].getPos() == -1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        
                        objectsBehindRobot[i].setP2(currentViewInMFIS[j].X2(), currentViewInMFIS[j].Y2());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertFrom = currentViewInMFIS[j].getID();

                        //finding whether this object is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
                        {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                break;
            }
        }
    }

    //RIGHT side expending 
    if (expandableOnRight != 0) 
    {
        cout << "(RIGHT) Object number " << expandableOnRight << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) 
        {
            if (objectsBehindRobot[i].getID() == expandableOnRight) 
            {
                for (unsigned int j = currentViewInMFIS.size() - 1; j > 0; j--) {//will consider last same object of cv on RIGHT
                    cout << "finding same object from last " << j << endl;
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && (expDist < 500.0)
                            && (currentViewInMFIS[j].getPos() == 1)) 
                    {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 
                        //&& currentViewInMFIS[j].getPos() == 1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        
                        objectsBehindRobot[i].setP1(currentViewInMFIS[j].X1(), currentViewInMFIS[j].Y1());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertTo = currentViewInMFIS[j].getID();

                        //checking whether this one is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
                        {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
                            {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                //cout << "Hello" << endl;
                break;
            }
        }
    }
    

    //inserting LEFT side's objects(old)
    for (int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == -1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }

    for (unsigned int j = insertFrom; j < insertTo - 1; j++) 
    {
        
        
        //modified bu Lyn Pang
        Object ref_mod, mod_current_Obj;

            ref_mod = similarObeject(updatedPM, currentViewInMFIS[j]);
            if(ref_mod.X1() != 0 && ref_mod.X2() != 0)
            {
                mod_current_Obj = modifyObject(currentViewInMFIS[j], ref_mod);

                //cout << " modified current surface info !!!!" << endl;
                //mod_current_Obj.display();

                //delete reference object in MFIS
                updatedPM = DeleteObjectFromCV(updatedPM, ref_mod);

                if(ref_mod.length() > 2000)
                    updatedPM.push_back(mod_current_Obj);
                else
                    updatedPM.push_back(currentViewInMFIS[j]);


            }
            else
            {   
                updatedPM.push_back(currentViewInMFIS[j]);
            }

        
        //updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);
        new_Objects_GlobalMap.push_back(currentViewInMFIS[j]);
        
        
        //making target(i.e changing id as they have in MFIS) object for next step
        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) 
        {
            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) 
            {
                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                targetObjectsForNextStep.back().setID(lastObjectID + 1);
                break;
            }
        }

        lastObjectID++;
    }
    
   
    
    cout << endl << endl << "TargetObjects for Next step " << targetObjectsForNextStep.size() << endl;
    //displayObjects(targetObjectsForNextStep);

    //inserting RIGHT side's objects(old)
    for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == 1) {
            updatedPM.push_back(objectsBehindRobot[i]);
            old_Objects_GlobalMap.push_back(objectsBehindRobot[i]);
        }
    }
    updatedPM.back().setID(lastObjectID + 1);
    cout << endl << endl << "UpdatedMFIS " << endl;

    unsigned char chunk_flag = 0;
    for(int i = 0; i < del_list.size(); i++)
    {
        //if((viewNumber - del_list[i].getVN() > 10) && (critical_reach_flag != 1)&&(del_list[i].getVN() > 0))
        if((viewNumber - del_list[i].getVN() > 10) && (del_list[i].getVN() > 0))
        {
//            cout << " current deleted infor vn    : " <<  del_list[i].getVN() << endl;
//            if(del_list[i].getLocalEnvID().size() > 0)
//            cout << " current deleted infor EvnID : " <<  del_list[i].getLocalEnvID()[0] << endl;
//            cout << " current step           v    : " << viewNumber << endl;
//            
//            vector<Object> test;
//            test.push_back(del_list[i]);
//            
//            sprintf(polyFile,"%s%d%s", "Maps/Offline/MFIS&Poly-", viewNumber, ".png");
//            plotObjects(polyFile, perceptualMap, test);
//            waitHere();
                critical_reach_point = viewNumber;
                collect_asr_number = del_list[i].getVN(); 
                //critical_reach_flag = 1;
                chunk_flag = 1;
        }
            
    }

    
    
    
    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    package.setChunkFlag(chunk_flag);
    
    
    return package;
}

Object similarObeject(vector<Object> MFIS, Object Obj)
{
        double expAngle, expDist;
        Object temp;
        temp.set(0,0,0,0,0);

        for(int i = 0; i < MFIS.size(); i++)
        {
            expAngle = MFIS[i].getAngleWithLine(Obj);
            expDist = shortestDistanceBtwTwoObjects(MFIS[i], Obj);
            if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && expDist < 500.0)
            {
                return MFIS[i];
            }
        }
        
        return temp;
}

Object Compare_similarObeject(vector<Object> MFIS, Object Obj, int v)
{
        double expAngle, expDist;
        double slope1, slope2;
        Object temp;
        vector<Object> view;
        temp.set(0,0,0,0,0);
        
        slope2 = (Obj.Y2() - Obj.Y1()) / (Obj.X2() - Obj.X1());
       
        for(int i = 0; i < MFIS.size(); i++)
        {
            slope1 = (MFIS[i].Y2() - MFIS[i].Y1()) / (MFIS[i].X2() - MFIS[i].X1());
            expAngle = MFIS[i].getAngleWithLine(Obj);
            expDist = shortestDistanceBtwTwoObjects(MFIS[i], Obj);
            
            /*if(v >= 27)
            {
                Obj.display();
                MFIS[i].display();
                cout << " Obj length : " << Obj.length() << endl;
                cout << " MFIS[i] length : " << MFIS[i].length() << endl;
                cout<< "  angle : " << abs(expAngle) << endl;
                cout<< "  expDist : " << abs(expDist) << endl;
                view.push_back(MFIS[i]);
                view.push_back(Obj);
                
                char mfisFileName[80];
                sprintf(mfisFileName, "%s%d%s", "Maps/test_pair_information-", v, ".png");
                plotObjects(mfisFileName,MFIS,view);
                view.clear();
                waitHere();

            }*/
            
            if ((abs(expAngle) < 30.0 || abs(expAngle) > 330.0) && (expDist < 500.0))
            {
                //MFIS[i].data()->set_replace_flag(1);
                return MFIS[i];
            }
        } 
        
        return temp;
}

Object Compare_similar_near_Obeject_(vector<Object>& MFIS, Object& Obj, int v)
{
        double expAngle, expDist, expDist1, expDist2;
        double slope1, slope2;
        Object temp;
        vector<Object> view;
        temp.set(0,0,0,0,0);
        
        slope2 = (Obj.Y2() - Obj.Y1()) / (Obj.X2() - Obj.X1());
       
        for(int i = 0; i < MFIS.size(); i++)
        {
            slope1 = (MFIS[i].Y2() - MFIS[i].Y1()) / (MFIS[i].X2() - MFIS[i].X1());
            expAngle = MFIS[i].getAngleWithLine(Obj);
            expDist = shortestDistanceBtwTwoObjects(MFIS[i], Obj);
            expDist1 = P_To_ShortestDistance(Obj.getP1(), MFIS[i]);
            expDist2 = P_To_ShortestDistance(Obj.getP2(), MFIS[i]);
            
            if (((abs(expAngle) < 30.0 || abs(expAngle) > 330.0) || (abs(expAngle) > 176.6 && abs(expAngle) < 189.0)) 
                    && (expDist1 < 600.0 || expDist2 < 600.0) && (Obj.length() < MFIS[i].length()))
            {
                //MFIS[i].data()->set_replace_flag(1);
                MFIS[i].set_match_label(i+10);
                MFIS[i].set_match_flag(1);
                Obj.set_match_label(i+10);
  
                return MFIS[i];
            }
   
        } 
        
        return temp;
}

Object Compare_similar_near_Obeject_modify(vector<Object>& MFIS, Object& Obj, int v)
{
        double expAngle, expDist, expDist1, expDist2, expDist_mid;
        double slope1, slope2;
        Object temp;
        vector<Object> view;
        temp.set(0,0,0,0,0);
        
        slope2 = (Obj.Y2() - Obj.Y1()) / (Obj.X2() - Obj.X1());
       
        for(int i = 0; i < MFIS.size(); i++)
        {
            slope1 = (MFIS[i].Y2() - MFIS[i].Y1()) / (MFIS[i].X2() - MFIS[i].X1());
            expAngle = MFIS[i].getAngleWithLine(Obj);
            expDist = shortestDistanceBtwTwoObjects(MFIS[i], Obj);
            expDist1 = P_To_ShortestDistance(Obj.getP1(), MFIS[i]);
            expDist2 = P_To_ShortestDistance(Obj.getP2(), MFIS[i]);
            expDist_mid = distanceOftwoP(Obj.midpoint(), MFIS[i].midpoint());
            
            if (((abs(expAngle) < 30.0 || abs(expAngle) > 330.0) || (abs(expAngle) > 176.6 && abs(expAngle) < 189.0)) 
                    && (expDist1 < 600.0 || expDist2 < 600.0) && (Obj.length() < MFIS[i].length())
                    && (expDist_mid < 500))
            {

                MFIS[i].set_match_label(i+10);
                MFIS[i].set_match_flag(1);
                Obj.set_match_label(i+10);
  
                return MFIS[i];
            }
   
        } 
        
        return temp;
}

int match_two_views(vector<Object> v1, vector<Object> v2)
{
        double expAngle, expDist, expDist1, expDist2, expDist3, expDist4;
        double slope1, slope2;
        int cnter = 0;
        Object temp;
        vector<Object> view;
        temp.set(0,0,0,0,0);
        
        ////slope2 = (Obj.Y2() - Obj.Y1()) / (Obj.X2() - Obj.X1());
       
        for(int j = 0; j < v1.size(); j++)
        {
            for(int i = 0; i < v2.size(); i++)
            {
                ////slope1 = (MFIS[i].Y2() - MFIS[i].Y1()) / (MFIS[i].X2() - MFIS[i].X1());
                expAngle = v2[i].getAngleWithLine(v1[j]);
                expDist = shortestDistanceBtwTwoObjects(v2[i], v1[j]);
                expDist1 = P_To_ShortestDistance(v1[j].getP1(), v2[i]);
                expDist2 = P_To_ShortestDistance(v1[j].getP2(), v2[i]);
                expDist3 = P_To_ShortestDistance(v2[i].getP1(), v1[j]);
                expDist4 = P_To_ShortestDistance(v2[i].getP2(), v1[j]);

                //cout << "exp angle is : " << abs(expAngle) << endl;
                // << "exp Dist1 is : " << expDist1 << endl;
                //cout << "exp Dist2 is : " << expDist2 << endl;
                //cout << "exp Dist3 is : " << expDist3 << endl;
                //cout << "exp Dist4 is : " << expDist4 << endl;
                
                //cout << endl << endl;

                if ((abs(expAngle) < 30.0 || abs(expAngle) > 330.0) 
                        && (expDist1 < 500.0 || expDist2 < 500.0 || expDist3 < 500.0 || expDist4 < 500.0))
                {
                    cnter++;
                    break;
                }
            } 
        }
        
        return cnter;
}

Object modifyObject(Object currentObj, Object ref)
{
    Object temp;
    
    //currentObj.display();
    //ref.display();
    
    if(ref.X1() > 0 && ref.X2() > 0)
    {
        temp.set(currentObj.X1(), currentObj.Y1(), ref.X2(), ref.Y2(), 1);
    }
    else
    {
        if(ref.X1() < 0 && ref.X2() < 0)
        {
            temp.set(ref.X1(), ref.Y1(),currentObj.X2(), currentObj.Y2(), 1);
        }
        else
        {
            if(ref.X1() > 0 && ref.X2() < 0)
            {
                temp.set(currentObj.X1(), currentObj.Y1(), ref.X2(), ref.Y2(), 1);
            }    
                
            if(ref.X1() < 0 && ref.X2() > 0)
            {
                temp.set(ref.X1(), ref.Y1(),currentObj.X2(), currentObj.Y2(), 1);
                
            }
        }
    }
   
    temp.setVN(currentObj.getVN());
    temp.setLocalEnvID(currentObj.getLocalEnvID());
    
    
    temp.setKP(1);
    temp.setASRNo(currentObj.getASRNo());
    temp.setChunkFLag(currentObj.getChunkFlag());
    temp.setColorID(currentObj.getColorID());

    temp.setKP(currentObj.getKP());
    temp.setLimitingPoint(currentObj.getLimitingPoint());

    temp.setOoPV(currentObj.getOoPV());
    temp.setOrt(currentObj.getOrt());
    temp.setP1OS(currentObj.getP1OS());
    temp.setP2OS(currentObj.getP2OS());
    temp.setPEP1(currentObj.getPEP1());
    temp.setPEP2(currentObj.getPEP2());
    temp.setPO(currentObj.getPO());
    temp.setPos(currentObj.getPos());

    
    return temp;
}

vector<Object> DeleteObjectFromCV(vector<Object> CV, Object to_delete)
{
    for(int i = 0; i < CV.size(); i++)
    {
        if(CV[i].X1() == to_delete.X1() && CV[i].X2() == to_delete.X2())
        {
            CV.erase(CV.begin() + i);
            break;
        }
    }
    
    return CV;
}

bool inDeleteASR(vector<Object> removed_info, Object potential_Object)
{
    
    if(removed_info.size() > 0)
    {
            for(int i = 0; i < removed_info.size(); i++)
            {
                    //cout << "potential_Object id size: " << potential_Object.getVN() << endl;
                    //cout << "current removed id size: " << removed_info[i].getVN()<< endl;    
                    if(potential_Object.getVN() == removed_info[i].getVN())
                    {
                        
                        return true;
                    }
                    
            }
    }
    
    return false;
}
