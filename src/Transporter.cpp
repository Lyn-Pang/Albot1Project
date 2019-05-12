#include <iostream>
#include <vector>

#include "Object.H"
#include "asr.H"
#include "Transporter.H"
#include "ChunksOp.H"
#include "PerceptualMapping.H"
#include "GeometricOp.H"
#include "mfisOp.H"
#include "Plotting.H"
#include "mfisOp.H"
#include "CompareASR.H"
#include "Mapping.H"
#include "PointAndSurface.H"
#include "RobotPosition.h"

using namespace std;

#define PI 3.14159265

int LastLMNUM = 0;
unsigned char expect_Flag = 0;     //expectation process flag

void Transporter::setViews(vector<vector<Object> > moreviews) { views = moreviews;}
vector<vector<Object> > Transporter::getViews() { return views; }

void Transporter::setView(vector<Object> cv) {	view=cv;}
vector<Object> Transporter::getView() {	return view;}
void Transporter::setReferenceObjects(vector<Object> refobs) {	refObjects=refobs;}
vector<Object> Transporter::getReferenceObjects(){ return refObjects;}
void Transporter::setTargetObjects(vector<Object> tobs) { tObjects=tobs;}
vector<Object> Transporter::getTargetObjects(){ return tObjects;}

void Transporter::setASRs(vector<ASR> asrs) {ASRs=asrs;}
vector<ASR> Transporter::getASRs() {return ASRs;}
void Transporter::setASR(ASR oneasr) {asr=oneasr;}
ASR Transporter::getASR() {return asr;}

void Transporter::setLostSituation(bool lost) {lostSituation=lost;}
bool Transporter::getLostSituation() { return lostSituation;}

void Transporter::setReferenceObjectsForLoopClosing(vector<Object> refobs) {
    refObjectsForLoopClosing = refobs;
}
    vector<Object> Transporter::getReferenceObjectsForLoopClosing() {
        return refObjectsForLoopClosing;
    }
void Transporter::setRobotPosition(vector<Object> rp) {
    robotPosition = rp;
}
    vector<Object> Transporter::getRobotPosition() {
        return robotPosition;
    }
    void Transporter::setAllRobotPositions(vector<Object> aRP) {
        allRobotPositions = aRP;
    }
    vector<Object> Transporter::getAllRobotPositions() {
        return allRobotPositions;
    }
    void Transporter::setMFIS(vector<Object> mfis){
        MFIS = mfis;
    }
    vector<Object> Transporter::getMFIS() {
        return MFIS;
    }
    
    void Transporter::setExits(vector<Exit> ex) {
        exits = ex;
    }
    vector<Exit> Transporter::getExits() {
        return exits;
    }
    
    
    //add by wenwang
    void Transporter:: setDeletionID(vector<int> deletion)
    {
            asrDelet = deletion;
    }
    vector<int> Transporter::getDeletionID()
    {
        return asrDelet;
    }
    
    void Transporter::setChunk(Chunk currentChunk)
    {
        combChunk = currentChunk;
    }
    Chunk Transporter::getChunk()
    {
        return combChunk;
    }
    
    void Transporter::setChunkFlag(unsigned char flag)
    {
        chunkFlag = flag;
    }
    unsigned char Transporter::getChunkFlag()
    {
        return chunkFlag;
    }
    
    
/* Update global map for processing chunks
 * once the deletion happens, it is to compute a new chunk
 * and meanwhile deletion all information till the latest local map
 * that belongs this chunk (initial version of chunk process)
 */
/*    
Transporter updateGlobalMapForChunks(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag) 
{
    cout << "\033[1;32m-------------------Inside Chunk Generate module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();


    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //added for chunk
    vector<int> deletionSurfaceLocalEID;

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();

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
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);

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

    //cout << "test programme angle : " << angle << endl << endl;
    
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
    vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }

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
                                            //add by wenwang
                                           //if(viewNumber - perceptualMap[i].getLocalEnvID()[le] > 20)
                                           //     lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
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


        if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true &&
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) {

            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
            pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing
            deletedObjectsOnCV.push_back(pMapOnCV[i]);
            
            //added for chunk
            deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);


        } else {

            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) {

                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                
                //added for chunk
                deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);
            } else {
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
            }


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
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                {

                    if (expandableOnRight == 0) {
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
        cout << "Current view in MFIS" << endl;
        cout << "Obects behind CRP" << endl;
    }
    

    cout << "deletedObjects on CV" << endl;
    //displayObjects(deletedObjectsOnCV);

    cout << "CV in MFIS" << endl;
    //displayObjects(currentViewInMFIS);


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
                //cout << "Hello" << endl;
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
    //cout<<"InsertFrom "<<insertFrom<<endl;
    //cout << "InsertTo: " << insertTo << endl;
    //cout << "size: " << objectsBehindRobot.size() << endl;
    //cout << "last id: " << cView[cView.size() - 1].getID() << endl;
    if (viewNumber == 87) {
        // insertTo = 14;
        //waitHere();
    }
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);

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

    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    

     
    //******* modified by Wenwang ********///
/*    //vector<Object>::iterator Del;
    vector<Object> CkView;
    vector<Object> deletion;
    vector<Object> nonDeletion;
    vector<int> chunkID;
    Object tem;
    Chunk currentChunk;
    char percFileName[80];     
    int cnt = 0;
    int NextLMStID = 0;
    //if(viewNumber - lpToDelete[0] > 20) // for not two neighbour steps
    /*if(viewNumber - deletionSurfaceLocalEID[0] > 20)
    {
            cout<<" Deletion involved ASR surfaces "<<endl;
            cout<<"current view num is : "<<viewNumber<<endl;
            //cout<<"The first ID in deletion surfaces is : "<<deletionSurfaceLocalEID[0]<<endl;
            //cout<<"lpToDelete first ID is : "<<lpToDelete[0] <<endl;
            //waitHere();
            for(int i = 0; i <  LMs.getMapIDs().size(); i++)
            {
                    //if(lpToDelete[0] < LMs.getMapIDs()[i])
                    if(deletionSurfaceLocalEID[0] < LMs.getMapIDs()[i])
                    {
                            //next Local map starting ID
                            NextLMStID = LMs.getMapIDs()[i];
                            break;
                    }
            }
            
            //delete all information in global map
            //which are belonging to the chunk/ASRs
            while(cnt < updatedPM.size())
            {
                //cout<<"surface ID is : "<<perceptualMap[i].getLocalEnvID()[0]<<endl;

                        if(updatedPM[cnt].getLocalEnvID()[0] <= NextLMStID)
                        {
                            
                            tem = updatedPM[cnt];
                            updatedPM.erase(updatedPM.begin()+cnt);
                            
                            deletion.push_back(tem);
                            cnt = 0;
                        }
                        else
                        {
                            nonDeletion.push_back(updatedPM[cnt]);
                            cnt++;
                        }
            }
            
            sprintf(percFileName,"%s%d%s", "Maps/Offline/Perceptual-", viewNumber, ".png");
            //plotObjects(percFileName, deletion);
            plotObjects(percFileName, deletion, nonDeletion);


            cout<<" Combine involved Local Maps as a Chunk "<<endl;
            //build up corresponding chunk
            //Ensure the local maps, and combine them as a chunk
            for(int j = LastLMNUM; j < LMs.getMapIDs().size(); j++)
            {
                        //if(lpToDelete[0] < LMs.getMapIDs()[j])
                        if(deletionSurfaceLocalEID[0] < LMs.getMapIDs()[j])
                        {
                                    CkView = LMs.getGlobalMaps()[LastLMNUM];
                                    for(int m = LastLMNUM+0; m < j; m++)
                                    {
                                            CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                            //LastLMNUM = j;
                                            LastLMNUM = m + 1;
                                    }
                                    currentChunk.setChunkView(CkView);
                                    //chunkID.push_back(LMs.getMapIDs()[m]);

                                    char chunFileName[80];   
                                    sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                                    plotObjects(chunFileName, currentChunk.getChunkView());
                                    break;
                        }
            }
            currentChunk.setIDS(chunkID);
    }
    
    //the last step should combine last few LMs
    if(lastStep_Flag == 1)
    {
                            CkView = LMs.getGlobalMaps()[LastLMNUM+0];
                            for(int m = LastLMNUM; m < LMs.getMapIDs().size(); m++)
                            {
                                CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                //LastLMNUM = j;
                            }
                            currentChunk.setChunkView(CkView);
                            //chunkID.push_back(LMs.getMapIDs()[m]);
                            
                            char chunFileName[80];   
                            sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                            plotObjects(chunFileName, currentChunk.getChunkView());

            //currentChunk.setIDS(chunkID);
    }
    */ 
/*    
    // another way to generate chunks
    //cout << "deletion surface id size: " << deletionSurfaceLocalEID.size() << endl;
    //waitHere();
    if(deletionSurfaceLocalEID.size() > 0)
    {
        if(viewNumber - deletionSurfaceLocalEID[0] > 10)
        {
                        if(deletionSurfaceLocalEID[0] >  LMs.getMapIDs()[LastLMNUM])
                        {
                                        for(int i = 0; i <  LMs.getMapIDs().size(); i++)
                                        {
                                                //if(lpToDelete[0] < LMs.getMapIDs()[i])
                                                if(viewNumber < LMs.getMapIDs()[i])
                                                {
                                                            //next Local map starting ID
                                                            NextLMStID = LMs.getMapIDs()[i];
                                                            break;
                                                }
                                                else
                                                {
                                                            if(i == LMs.getMapIDs().size() - 1)
                                                            {
                                                                NextLMStID = LMs.getMapIDs()[i];
                                                            }
                                                }
                                        }

                                        //delete all information in global map
                                        //which are belonging to the chunk/ASRs
                                        while(cnt < updatedPM.size())
                                        {
                                                    if(updatedPM[cnt].getLocalEnvID()[0] <= NextLMStID)
                                                    {
                                                            tem = updatedPM[cnt];
                                                            updatedPM.erase(updatedPM.begin()+cnt);

                                                            deletion.push_back(tem);
                                                            cnt = 0;
                                                    }
                                                    else
                                                    {
                                                            nonDeletion.push_back(updatedPM[cnt]);
                                                            cnt++;
                                                    }
                                        }

                                        sprintf(percFileName,"%s%d%s", "Maps/Offline/Perceptual-", viewNumber, ".png");
                                        plotObjects(percFileName, deletion, nonDeletion);

                                        cout<<" Combine involved Local Maps as a Chunk "<<endl;
                                        //build up corresponding chunk
                                        //Ensure the local maps, and combine them as a chunk
                                        for(int j = LastLMNUM; j < LMs.getMapIDs().size(); j++)
                                        {
                                                    //if(lpToDelete[0] < LMs.getMapIDs()[j])
                                                    if(viewNumber < LMs.getMapIDs()[j])
                                                    {
                                                                CkView = LMs.getGlobalMaps()[LastLMNUM];
                                                                for(int m = LastLMNUM; m < j; m++)
                                                                {
                                                                        CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                                                        //LastLMNUM = j;
                                                                        //if(LastLMNUM == j)
                                                                        //         LastLMNUM = m;
                                                                        //else
                                                                                LastLMNUM = m + 1;
                                                                }
                                                                currentChunk.setChunkView(CkView);
                                                                //chunkID.push_back(LMs.getMapIDs()[m]);

                                                                char chunFileName[80];   
                                                                sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                                                                plotObjects(chunFileName, currentChunk.getChunkView());
                                                                package.setChunkFlag(1);
                                                                break;
                                                    }
                                                    else
                                                    {
                                                        cout<<"The size of MAP ID : "<<LMs.getMapIDs().size()<<endl;
                                                        cout<<"Last LM number : "<<LastLMNUM<<endl;
                                                                CkView = LMs.getGlobalMaps()[LastLMNUM];
                                                                for(int m = LastLMNUM; m <= LMs.getMapIDs().size() - 1; m++)
                                                                {
                                                                        CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                                                        //LastLMNUM = j;
                                                                        //if(LastLMNUM == LMs.getMapIDs().size() - 1)
                                                                        //         LastLMNUM = m;
                                                                        //else
                                                                                 LastLMNUM = m + 1;

                                                                }
                                                                currentChunk.setChunkView(CkView);
                                                                //chunkID.push_back(LMs.getMapIDs()[m]);

                                                                char chunFileName[80];   
                                                                sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                                                                plotObjects(chunFileName, currentChunk.getChunkView());
                                                                package.setChunkFlag(1);
                                                                break;
                                                    }
                                        }
                                        currentChunk.setIDS(chunkID);

                        }
                        else
                                 package.setChunkFlag(0);
        }
        else
               package.setChunkFlag(0);
    }
    else
        package.setChunkFlag(0);
    
    //the last step should combine last few LMs
    if(lastStep_Flag == 1)
    {
                            sprintf(percFileName,"%s%d%s", "Maps/Offline/Perceptual-", viewNumber, ".png");
                            plotObjects(percFileName, updatedPM);
                            
                            CkView = LMs.getGlobalMaps()[LastLMNUM];
                            for(int m = LastLMNUM; m < LMs.getMapIDs().size(); m++)
                            {
                                CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                //LastLMNUM = j;
                            }
                            currentChunk.setChunkView(CkView);
                            //chunkID.push_back(LMs.getMapIDs()[m]);
                            
                            char chunFileName[80];   
                            sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                            plotObjects(chunFileName, currentChunk.getChunkView());

                            //currentChunk.setIDS(chunkID);
                            package.setChunkFlag(1);
    }

    //add by wenwang
    //package.setDeletionID();
    if(package.getChunkFlag() == 1)
        package.setChunk(currentChunk);
    package.setView(updatedPM);
    package.setDeletionID(deletionSurfaceLocalEID);
     
     
    return package;
}
*/
    
/* Deletion process update
 * Deleting information until current view number-1 (previous view number)
 * Other things keep same as previous version
 */
/*
Transporter updateGlobalMapForChunks(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag) 
{
    cout << "\033[1;32m-------------------Inside Chunk Generate module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();


    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //added for chunk
    vector<int> deletionSurfaceLocalEID;

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();

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
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);

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

    //cout << "test programme angle : " << angle << endl << endl;
    
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
    vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }

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
                                            //add by wenwang
                                           //if(viewNumber - perceptualMap[i].getLocalEnvID()[le] > 20)
                                           //     lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
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


        if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true &&
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) {

            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
            pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing
            deletedObjectsOnCV.push_back(pMapOnCV[i]);
            
            //added for chunk
            deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);


        } else {

            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) {

                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                
                //added for chunk
                deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);
            } else {
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
            }


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
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                {

                    if (expandableOnRight == 0) {
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
        cout << "Current view in MFIS" << endl;
        cout << "Obects behind CRP" << endl;
    }
    

    cout << "deletedObjects on CV" << endl;
    //displayObjects(deletedObjectsOnCV);

    cout << "CV in MFIS" << endl;
    //displayObjects(currentViewInMFIS);


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
                //cout << "Hello" << endl;
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
    //cout<<"InsertFrom "<<insertFrom<<endl;
    //cout << "InsertTo: " << insertTo << endl;
    //cout << "size: " << objectsBehindRobot.size() << endl;
    //cout << "last id: " << cView[cView.size() - 1].getID() << endl;
    if (viewNumber == 87) {
        // insertTo = 14;
        //waitHere();
    }
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);

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

    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    

     
    //******* modified by Lyn ********///
    //vector<Object>::iterator Del;
/*    vector<Object> CkView;
    vector<Object> deletion;
    vector<Object> nonDeletion;
    vector<int> chunkID;
    Object tem;
    Chunk currentChunk;
    char percFileName[80];     
    int cnt = 0;
    int NextLMStID = 0;
    
    if(deletionSurfaceLocalEID.size() > 0)
    {
        if(viewNumber - deletionSurfaceLocalEID[0] > 10)
        {
                        if(deletionSurfaceLocalEID[0] >  LMs.getMapIDs()[LastLMNUM])
                        {
                                        for(int i = 0; i <  LMs.getMapIDs().size(); i++)
                                        {
                                                //if(lpToDelete[0] < LMs.getMapIDs()[i])
                                                if(viewNumber < LMs.getMapIDs()[i])
                                                {
                                                            //next Local map starting ID
                                                            NextLMStID = LMs.getMapIDs()[i];
                                                            break;
                                                }
                                                else
                                                {
                                                            if(i == LMs.getMapIDs().size() - 1)
                                                            {
                                                                NextLMStID = LMs.getMapIDs()[i];
                                                            }
                                                }
                                        }

                                        //delete all information in global map
                                        //which are belonging to the chunk/ASRs
                                        while(cnt < updatedPM.size())
                                        {
                                                    if(updatedPM[cnt].getLocalEnvID()[0] <= viewNumber-1)
                                                    {
                                                            tem = updatedPM[cnt];
                                                            updatedPM.erase(updatedPM.begin()+cnt);

                                                            deletion.push_back(tem);
                                                            cnt = 0;
                                                    }
                                                    else
                                                    {
                                                            nonDeletion.push_back(updatedPM[cnt]);
                                                            cnt++;
                                                    }
                                        }

                                        sprintf(percFileName,"%s%d%s", "Maps/Offline/Perceptual-", viewNumber, ".png");
                                        plotObjects(percFileName, deletion, nonDeletion);

                                        cout<<" Combine involved Local Maps as a Chunk "<<endl;
                                        //build up corresponding chunk
                                        //Ensure the local maps, and combine them as a chunk
                                        for(int j = LastLMNUM; j < LMs.getMapIDs().size(); j++)
                                        {
                                                    //if(lpToDelete[0] < LMs.getMapIDs()[j])
                                                    if(viewNumber < LMs.getMapIDs()[j])
                                                    {
                                                                CkView = LMs.getGlobalMaps()[LastLMNUM];
                                                                for(int m = LastLMNUM; m < j; m++)
                                                                {
                                                                        CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                                                        //LastLMNUM = j;
                                                                        //if(LastLMNUM == j)
                                                                        //         LastLMNUM = m;
                                                                        //else
                                                                                LastLMNUM = m + 1;
                                                                }
                                                                currentChunk.setChunkView(CkView);
                                                                //chunkID.push_back(LMs.getMapIDs()[m]);

                                                                char chunFileName[80];   
                                                                sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                                                                plotObjects(chunFileName, currentChunk.getChunkView());
                                                                package.setChunkFlag(1);
                                                                break;
                                                    }
                                                    else
                                                    {
                                                        cout<<"The size of MAP ID : "<<LMs.getMapIDs().size()<<endl;
                                                        cout<<"Last LM number : "<<LastLMNUM<<endl;
                                                                CkView = LMs.getGlobalMaps()[LastLMNUM];
                                                                for(int m = LastLMNUM; m <= LMs.getMapIDs().size() - 1; m++)
                                                                {
                                                                        CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                                                        //LastLMNUM = j;
                                                                        //if(LastLMNUM == LMs.getMapIDs().size() - 1)
                                                                        //         LastLMNUM = m;
                                                                        //else
                                                                                 LastLMNUM = m + 1;

                                                                }
                                                                currentChunk.setChunkView(CkView);
                                                                //chunkID.push_back(LMs.getMapIDs()[m]);

                                                                char chunFileName[80];   
                                                                sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                                                                plotObjects(chunFileName, currentChunk.getChunkView());
                                                                package.setChunkFlag(1);
                                                                break;
                                                    }
                                        }
                                        currentChunk.setIDS(chunkID);

                        }
                        else
                                 package.setChunkFlag(0);
        }
        else
               package.setChunkFlag(0);
    }
    else
        package.setChunkFlag(0);
    
    //the last step should combine last few LMs
    if(lastStep_Flag == 1)
    {
                            sprintf(percFileName,"%s%d%s", "Maps/Offline/Perceptual-", viewNumber, ".png");
                            plotObjects(percFileName, updatedPM);
                            
                            CkView = LMs.getGlobalMaps()[LastLMNUM];
                            for(int m = LastLMNUM; m < LMs.getMapIDs().size(); m++)
                            {
                                CkView = addTwoVectorsOfObjects(CkView, LMs.getGlobalMaps()[m]);
                                //LastLMNUM = j;
                            }
                            currentChunk.setChunkView(CkView);
                            //chunkID.push_back(LMs.getMapIDs()[m]);
                            
                            char chunFileName[80];   
                            sprintf(chunFileName, "%s%d%s", "Maps/Offline/CurrentChunk-", viewNumber, ".png");
                            plotObjects(chunFileName, currentChunk.getChunkView());

                            //currentChunk.setIDS(chunkID);
                            package.setChunkFlag(1);
    }

    //add by wenwang
    //package.setDeletionID();
    if(package.getChunkFlag() == 1)
        package.setChunk(currentChunk);
    package.setView(updatedPM);
    package.setDeletionID(deletionSurfaceLocalEID);
     
     
    return package;
}
*/

/* The fourth version of Chunk process
 * only output a chunk process flag when deletion happens
 * for <localMapsAndChunk.cpp>
 */  
    
Transporter updateGlobalMapForChunks(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag) 
{
    cout << "\033[1;32m-------------------Inside Chunk Generate module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();


    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //added for chunk
    vector<int> deletionSurfaceLocalEID;

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();

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
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);

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

    //cout << "test programme angle : " << angle << endl << endl;
    
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
    vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }

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
                                            //add by wenwang
                                           //if(viewNumber - perceptualMap[i].getLocalEnvID()[le] > 20)
                                           //     lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
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


        if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true &&
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) {

            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
            pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing
            deletedObjectsOnCV.push_back(pMapOnCV[i]);
            
            //added for chunk
            deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);


        } else {

            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) {

                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                
                //added for chunk
                deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);
            } else {
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
            }


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
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                {

                    if (expandableOnRight == 0) {
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
        cout << "Current view in MFIS" << endl;
        cout << "Obects behind CRP" << endl;
    }
    

    cout << "deletedObjects on CV" << endl;
    //displayObjects(deletedObjectsOnCV);

    cout << "CV in MFIS" << endl;
    //displayObjects(currentViewInMFIS);


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
                //cout << "Hello" << endl;
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
    //cout<<"InsertFrom "<<insertFrom<<endl;
    //cout << "InsertTo: " << insertTo << endl;
    //cout << "size: " << objectsBehindRobot.size() << endl;
    //cout << "last id: " << cView[cView.size() - 1].getID() << endl;
    if (viewNumber == 87) {
        // insertTo = 14;
        //waitHere();
    }
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);

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

    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    

     
    //******* modified by Lyn ********///
    //vector<Object>::iterator Del;
    vector<Object> CkView;
    vector<Object> deletion;
    vector<Object> nonDeletion;
    vector<int> chunkID;
    Object tem;
    Chunk currentChunk;
    char percFileName[80];     
    int cnt = 0;
    int NextLMStID = 0;
    
    if(deletionSurfaceLocalEID.size() > 0)
    {
        if(viewNumber - deletionSurfaceLocalEID[0] > 15)
        {
            //cout << "deletion size : " << deletionSurfaceLocalEID.size() << endl;
            //cout << "deletion ob id : " << deletionSurfaceLocalEID[0] << endl;
            //waitHere();
                        
                                //delete all information in global map
                                //which are belonging to the chunk/ASRs
                                while(cnt < updatedPM.size())
                                {
                                   
                                            if(updatedPM[cnt].getLocalEnvID()[0] <= viewNumber-1)
                                            {
                                                    tem = updatedPM[cnt];
                                                    updatedPM.erase(updatedPM.begin()+cnt);

                                                    deletion.push_back(tem);
                                                    cnt = 0;
                                            }
                                            else
                                            {
                                                    nonDeletion.push_back(updatedPM[cnt]);
                                                    cnt++;
                                            }
                                   
                                }

                                package.setChunkFlag(1);
        }
        else
               package.setChunkFlag(0);
    }
    else
        package.setChunkFlag(0);
    
    //the last step should combine last few LMs
    if(lastStep_Flag == 1)
    {
            package.setChunkFlag(1);
    }

    //add by wenwang
    package.setView(updatedPM);
    package.setDeletionID(deletionSurfaceLocalEID);
     
     
    return package;
}


/* The fourth version of Chunk process
 * only output a chunk process flag when deletion happens
 * for <localMapsAndChunk.cpp>
 * and also later the ASR where generating the chunk using FLAG
 */  
    
Transporter updateGlobalMapForChunksWithFlag(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag) 
{
    cout << "\033[1;32m-------------------Inside Chunk Generate module---------------------\033[0m" << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();


    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;
    
    //added for chunk
    vector<int> deletionSurfaceLocalEID;
    vector<Object> deletionSurfaces;

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();

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
            sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
            dummy.clear();
            dummy.push_back(refobjects[1]);

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

    //cout << "test programme angle : " << angle << endl << endl;
    
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
    
    //char polyFile[80];
    //sprintf(polyFile,"%s%d%s", "Maps/Offline/Global&Poly-", viewNumber, ".png");
    //plotObjectsOf3Kinds(polyFile, polygonObjects, pMapOnCV, cView);
    
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }
    
    

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
                                            //add by wenwang
                                           //if(viewNumber - perceptualMap[i].getLocalEnvID()[le] > 20)
                                           //     lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
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


        if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true &&
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) {

            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
            pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing
            deletedObjectsOnCV.push_back(pMapOnCV[i]);
            
            //added for chunk
            deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);
            //deletionSurfaceLocalEID.push_back(pMapOnCV[i].getVN());
            deletionSurfaces.push_back(pMapOnCV[i]);

        } else {

            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) {

                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                
                //added for chunk
                deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);
                //deletionSurfaceLocalEID.push_back(pMapOnCV[i].getVN());
                deletionSurfaces.push_back(pMapOnCV[i]);
                
            } else {
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
            }


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
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)
                {

                    if (expandableOnRight == 0) {
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
        cout << "Current view in MFIS" << endl;
        cout << "Obects behind CRP" << endl;
    }
    

    cout << "deletedObjects on CV" << endl;
    //displayObjects(deletedObjectsOnCV);

    cout << "CV in MFIS" << endl;
    //displayObjects(currentViewInMFIS);


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
                //cout << "Hello" << endl;
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
    //cout<<"InsertFrom "<<insertFrom<<endl;
    //cout << "InsertTo: " << insertTo << endl;
    //cout << "size: " << objectsBehindRobot.size() << endl;
    //cout << "last id: " << cView[cView.size() - 1].getID() << endl;
    if (viewNumber == 87) {
        // insertTo = 14;
        //waitHere();
    }
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);

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

    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    

     
    //******* modified by Lyn ********///
    //vector<Object>::iterator Del;
    vector<Object> CkView;
    vector<Object> deletion;
    vector<Object> nonDeletion;
    vector<int> chunkID;
    Object tem;
    Chunk currentChunk;
    char percFileName[80];     
    int cnt = 0;
    int NextLMStID = 0;
    int process_flag = 0;
    
    if(deletionSurfaces.size() > 0)
    {
        //cout << " current view num : " << currentViewNumber << endl;
        //cout << " deletion surface view num : " << deletionSurfaces[0].getVN() << endl;
        //waitHere();
        for(int i = 0; i < deletionSurfaces.size(); i++)
        {
            if((deletionSurfaces[i].getChunkFlag() != 1) 
                && (currentViewNumber - deletionSurfaces[0].getLocalEnvID()[0] > 1))
            {
                //cout << "deletion surface id : " << deletionSurfaces[0].getLocalEnvID()[0]<<endl;
                //cout << "deletion ob id : " << deletionSurfaceLocalEID[0] << endl;
                //waitHere();
                
                process_flag = 1;
                break;
            }
        }
        
        if(process_flag == 1)
        {
                        
                                //delete all information in global map
                                //which are belonging to the chunk/ASRs
                                while(cnt < updatedPM.size())
                                {
                                            if(updatedPM[cnt].getLocalEnvID()[0] <= viewNumber-1)
                                            {
                                                    tem = updatedPM[cnt];
                                                    updatedPM.erase(updatedPM.begin()+cnt);

                                                    deletion.push_back(tem);
                                                    cnt = 0;
                                            }
                                            else
                                            {
                                                    nonDeletion.push_back(updatedPM[cnt]);
                                                    cnt++;
                                            }
                                }

                                package.setChunkFlag(1);
                                
                                for(int j = 0; j < updatedPM.size(); j++)
                                {
                                    updatedPM[j].setChunkFLag(1);
                                }
                                
                                sprintf(percFileName,"%s%d%s", "Maps/Offline/Perceptual-", viewNumber, ".png");
                                plotObjects(percFileName, deletion, nonDeletion);
        }
        else
               package.setChunkFlag(0);
    }
    else
        package.setChunkFlag(0);
    
    //the last step should combine last few LMs
    if(lastStep_Flag == 1)
    {
            package.setChunkFlag(1);
    }

    //add by wenwang
    package.setView(updatedPM);
    package.setDeletionID(deletionSurfaceLocalEID);
     
     
    return package;
}

/* Chunking withe new mapping proess
 * delete all earlier information when 
 * object is in polygon
 */
Transporter updateGlobalMapForChunksNewDeletion(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag) 
{
    cout << "\033[1;32m-------------------Inside Chunk Generate module---------------------\033[0m" << endl;

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
    
    //added for chunk
    vector<int> deletionSurfaceLocalEID;
    
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
    
    //modified by Lyn, building a polygon corresponding current view
    vector<Object> polygonObjects;
    ClipperLib::Path subj;
    Paths solution;
    ClipperOffset co;

    subj = viewConvertPath(cView); //convert view into Path
    co.AddPath(subj, jtMiter, etClosedPolygon);
    co.Execute(solution, 1000.0); // outward offset the polygon
    polygonObjects = PathsConvertView(solution); // convert polygon into view
    polygon = convertObjectToSurface(polygonObjects); // convert view into surfaces
    
    for (unsigned int i = 0; i < polygonObjects.size(); i++) 
    {
            temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
            polygonObjectsOnMFIS.push_back(temp);
    }
    

    vector<Object> exppandableObjects;
    vector<int> lpToDelete; //limiting points to be deleted.
    vector<Object> objectsBelongToSameSpace;
    vector<Object> deletedObjectsOnCV;
    //vector<Object> delete_front_surfs;


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
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true &&
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
            
            //added for chunk 
            if(pMapOnCV[i].getLocalEnvID().size() != 0)
                    deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);
                 else
                     deletionSurfaceLocalEID.push_back(pMapOnCV[i].getVN());
            

        } 
        else 
        {
            
            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) 
            {

                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
                
                //added for chunk
                 if(pMapOnCV[i].getLocalEnvID().size() != 0)
                    deletionSurfaceLocalEID.push_back(pMapOnCV[i].getLocalEnvID()[0]);
                 else
                     deletionSurfaceLocalEID.push_back(pMapOnCV[i].getVN());
              
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
                     //delete_front_surfs.push_back(pMapOnCV[i]); //which is not in polygon but deleted as well
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
    
    //******* modified by Lyn ********///
    //vector<Object>::iterator Del;
    vector<Object> CkView;
    vector<Object> deletion;
    vector<Object> nonDeletion;
    vector<int> chunkID;
    Object tem;
    Chunk currentChunk;
    char percFileName[80];     
    int cnt = 0;
    int NextLMStID = 0;
    //cout << " Test Programme " << endl;
    cout << "deleted surface array : " << deletionSurfaceLocalEID.size() << endl;
    if(deletionSurfaceLocalEID.size() > 0)
    {
        if(viewNumber - deletionSurfaceLocalEID[0] > 15)
        {
            //cout << "deletion size : " << deletionSurfaceLocalEID.size() << endl;
            cout << "deletion ob id : " << deletionSurfaceLocalEID[0] << endl;
            //waitHere();
                        
                                //delete all information in global map
                                //which are belonging to the chunk/ASRs
                                while(cnt < updatedPM.size())
                                {
                                   
                                            if(updatedPM[cnt].getLocalEnvID()[0] <= viewNumber-1)
                                            {
                                                    tem = updatedPM[cnt];
                                                    updatedPM.erase(updatedPM.begin()+cnt);

                                                    deletion.push_back(tem);
                                                    cnt = 0;
                                            }
                                            else
                                            {
                                                    nonDeletion.push_back(updatedPM[cnt]);
                                                    cnt++;
                                            }
                                   
                                }

                                package.setChunkFlag(1);
        }
        else
        {
               package.setChunkFlag(0);
               
               if((viewNumber - deletionSurfaceLocalEID[0] > 3) && (viewNumber - deletionSurfaceLocalEID[0] > 10))
               {
                   //trigger expectation process
                   
               }
        }
    }
    else
        package.setChunkFlag(0);
    
    //the last step should combine last few LMs
    if(lastStep_Flag == 1)
    {
            package.setChunkFlag(1);
    }
    
    //add by wenwang
    package.setView(updatedPM);
    package.setDeletionID(deletionSurfaceLocalEID);
    
    
    return package;
}