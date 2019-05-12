/*
    Transporter, it follows three rules
    Rule#1: Deal's the deal.
    Rule#2: No names.
    Rule#3: Never look in the package.

        Designed by........
                        MHossain
                        10.07.2011
                        Inspired by......The Transporter(2002)
 */

#include <iostream>
#include <vector>

#include "Object.H"
#include "asr.H"
#include "ChunksOp.H"
#include "PathPlanning.H"

using namespace std;

#ifndef _TRANSPORTER_H_
#define _TRANSPORTER_H_

extern unsigned char expect_Flag;

class Transporter {
private:
    vector<Object> MFIS;
    vector<vector<Object> > views;
    vector<Object> view; //vector of objects
    vector<Object> refObjects; //vector of ref objects
    vector<Object> tObjects; //vector of target objects--- landmark objects
    vector<Object> refObjectsForLoopClosing;
    vector<Object> robotPosition;
    vector<Object> allRobotPositions;
    vector<Exit> exits;
    vector<ASR> ASRs;
    ASR asr;
    bool lostSituation;
    
    //add by wenwang
    vector<int> asrDelet;
    Chunk combChunk;
    unsigned char chunkFlag;

public:

    Transporter() : lostSituation(false) {
    };

    ~Transporter() {
    };
    
    void setViews(vector<vector<Object> > moreviews);
    vector<vector<Object> > getViews();
    void setView(vector<Object> cv);
    vector<Object> getView();
    void setReferenceObjects(vector<Object> refobs);
    vector<Object> getReferenceObjects();
    void setTargetObjects(vector<Object> tobs);
    vector<Object> getTargetObjects();
    void setASRs(vector<ASR> asrs);
    vector<ASR> getASRs();
    void setExits(vector<Exit> ex);
    vector<Exit> getExits();    
    void setASR(ASR oneasr);
    ASR getASR();
    void setLostSituation(bool lost);
    bool getLostSituation();
    
    void setReferenceObjectsForLoopClosing(vector<Object> refobs);
    vector<Object> getReferenceObjectsForLoopClosing();
    
    void setRobotPosition(vector<Object> rp);
    vector<Object> getRobotPosition();
    
    void setAllRobotPositions(vector<Object> aRP);
    vector<Object> getAllRobotPositions();

    void setMFIS(vector<Object> mfis);
    vector<Object> getMFIS();
    
    //add by wenwang 
    void setDeletionID(vector<int> deletion);
    vector<int> getDeletionID();
    
    void setChunk(Chunk currentChunk);
    Chunk getChunk();
    
    void setChunkFlag(unsigned char flag);
    unsigned char getChunkFlag();
};

Transporter updateGlobalMapForChunks(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag);

/* The fourth version of Chunk process
 * only output a chunk process flag when deletion happens
 * for <localMapsAndChunk.cpp>
 * and also later the ASR where generating the chunk using FLAG
 */  
Transporter updateGlobalMapForChunksWithFlag(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag);


/* Chunking withe new mapping proess
 * delete all earlier information when 
 * object is in polygon
 */
Transporter updateGlobalMapForChunksNewDeletion(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<Object> routeMap4CPlace, LocalMap LMs, unsigned int lastStep_Flag);


#endif
