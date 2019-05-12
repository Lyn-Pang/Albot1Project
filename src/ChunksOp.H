/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ChunksOp.H
 * Author: Wenwang
 *
 * Created on February 22, 2016, 5:58 PM
 */

#include <iostream>
#include <vector>
#include <stdio.h>
#include "Object.H"
#include "Point.H"
#include "PathPlanning.H"
//#include "Transporter.H"
#include "Ellipse.h"

using namespace std;

#ifndef CHUNKSOP_H
#define CHUNKSOP_H

class LocalMap
{
        private:
            vector< vector<Object> > relativeCoordMaps;
            vector< vector<Object> > gobalCoordMaps;
            vector<int> mapIDs;
            vector< vector<Object> > robotPositions; // for plotting
            vector<Point> RBPositions; //for process
            unsigned int seperate;
            
        public:
             LocalMap();
            ~LocalMap();
            
            void setRelativeMaps(vector< vector<Object> > individualMaps);
            vector< vector<Object> > getRelativeMaps();
            
            void setGlobalMaps(vector< vector<Object> > globalMaps);
            vector< vector<Object> > getGlobalMaps();
            
            void setMapIDs(vector<int> mapId);
            vector<int> getMapIDs();
            
            void setRobPosition(vector< vector<Object> > robPosition);
            vector< vector<Object> > getRobPosition();
            
            void setSepflag(unsigned int sepFlag);
            unsigned int getSepflag();
            
            void setGlobalPositions(vector<Point> RbPositions);
            vector<Point> getGlobalPositions();
    
};


class Chunk
{
        private:
            LocalMap localMaps;
            vector<Object> chunkMap; //chunk map
            vector<int> IDs; //all local map View ID that constructing the chunk
            vector< vector<Object> > ASRs;
            Point connectionPoint;
            
            int Numb_LMs; //how many local maps construct the chunk
            //int deletionID; //for triggering to generate this chunk
        public:
            Chunk();
            ~Chunk();
            
            void setChunkView(vector<Object> comMap);
            vector<Object> getChunkView();
            
            void setIDS(vector<int> localMapID);
            vector<int> getIDS();
            
            void setASRs(vector< vector<Object> > allASR);
            vector< vector<Object> > getASRS();
            
            void setConnection(Point connect);
            Point getConnection();
            
            //void setTriggerID(int trigger);
            //int getTriggerID();
};

class Map //individual local map
{
        private:
            vector<Object> local_map; //corresponding map/view
            int map_ID; //map ID, corresponding view ID
            Point origin; //robot original position
            vector<Object> exit_position; //robot end position
            double headingAngle; //robot heading with X axis
        public:
            void setMap(vector<Object> map);
            vector<Object> getMap();
            
            void setMapID(int mapID);
            int getMapID();
            
            void setExitPositon(vector<Object> exitPosition);
            vector<Object> getExitPosition();
            Point getOriginPosition();
            
            void setHeading(double robotHeading);
            double getHeading();
            
            Point getOrientation(); // for homing navigation
            
            
};


class ChunkInfo
{
        private:
            
            vector<Exit> compute_exits; //final computed exits
            
            vector<Exit> shifted_pre_chunk_exits; //select and shift
            
            vector<Object> whole_mfis;  //complete MFIS
            
            vector< vector<Object> > path_space_region;       
            vector< vector<Object> > views, robot, MFISs, region;//views, robots, All_mfis, region
            vector< vector<Object> > structures_region; 

            vector< vector<Ellipse> > circle_region; //all circles in region
            
        public:
            void setChunkInfo(vector<Object> mfis, vector<Exit> exit, vector< vector<Object> > robots, 
                      vector< vector<Object> > views_chunk, vector< vector<Object> > MFIS_regions);
            //~ChunkInfo();
            
            vector<Exit> getExits();
            vector<Object> getMFIS();
            
            vector< vector<Object> > getRobots();
            vector< vector<Object> > getViews();
            vector< vector<Object> > getMFIS_region();
            
            void setShiftExits(vector<Exit> modified_exits);
            vector<Exit> getShiftExits();
                         
};



/* Based on LMs overlapping theory
 * Calculating the polygon of each local map
 * Calculating whether the current local map is intersected
 * with previous local maps
 */
void GenerateChunks(vector< vector<Object> > LMs, int viewNumber);

/*checking whether the previous 5-10 steps, 
 * robot positions, are in current LM
 */
void MoveBackDetection(vector<Object> currentLM, vector<Point> PrePositions, Point currentRP, int stepNum);

/*Cluster the surfaces
 *abstract the structure of All ASRs/ Chunks
 */
vector< vector<Object> > ClusterOfChunks(vector<Object> ASRs);

/*This function is to generate an equal width path
 *following the robot positions for each of Chunk
 */
vector<Object> TraversePath(vector<Object> completePath, Point coord, double angle, int num);

//**** modify the final global map function ****//
pair<Point, double> constructInfo(vector<Object> part1, vector<Object> part2);
void reOrganise(vector<Object> finalGM);

//vector<Object> OrganiseLocalMaps(LocalMap LMs);
void OrganiseLocalMaps(LocalMap LMs);
void reOrganiseChunks(vector<Object> chunk1, vector<Object> chunk2);

//transform view to ellipse shape boundary
void TransformViewToEllipseBoundary(vector<Object> currentView);


//sketch the outline of chunk, along the path
//void SketchMap(vector<Object> map, vector< vector<Object> > listOfRP);

//detect the maximum space along the path segment
void MaxSpaceFollowPath(vector<Object> chunkMap, vector< vector<Object> > listOfRP);

//generate rectangular along the path
vector<Object> RectangularAlongPath(Object pathLine);

//All functions are for test exps
/*establish a fix size boudnary follow
 *the route segments
 */
void SketchBoundary(vector<Object> chunkMap, vector< vector<Object> > listOfRP, vector< vector<Object> > chunkLMs);

/*interpolating the lines to 
 *establish a kind of closure boundary
 * (not finished)
 */
vector<Object> Interpolation(vector<Object> simple, vector<Object> path_segments);


vector<Object> SlopeWithRoute(vector<Object> chunkMap, vector<Object> route);

/*establish reasonable route follow chunk
 *chunk includes intermediate views and robot positions
 *connect all robot positions directly & combine some of them
 */
void RoughRoute(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Point> rps);

/*combine some of views belonging the chunk
 *find the surfaces limiting the corresponding route
 */
//vector< vector<Object> > DetectSpace(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Object> route_segments);

//vector<Object> isBoundary(vector<Object> map, Object route);


pair<Point, Point> outPerpenPoint(Object reference);
pair<Point, Point> PerpenPointToRouteSeg(Object reference);
pair<Point, Point> PerpenPointToRouteEnd(Object reference);

//void RegionAndBoundary(vector< vector<Object> > allLEs, vector<Point> robpositions, LocalMap LMS);

//vector<Object> RouteCombine(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Point> rps);

/*establish a line joining the mid point of route segment
 *collect the front and behind surfaces follow the route 
 */
//vector<Object> FrontAndBehind(vector<Object> chunkMap, Object route, unsigned char start_flag);

/*Compute two regions, one is following the front end point
 *another is following the back end point based on the orientation of route
 *once the surface in the regions, collect them as front or behind surfaces
 */
//vector<Object> FrontAndBehindWithinRegion(vector<Object> chunkMap, Object route, unsigned char start_flag);

/*find exit between two regions
 *using one of common surfaces that is the nearest surface
 *to identify a potential exit
 */
vector<Exit> IdentifyExits(vector< vector<Object> > regions, vector<Object> route_segments);

//vector<Exit> IdentifyExitsFollowRoute(vector< vector<Object> > regions, vector<Object> route_segments);

//vector<Object> RegionDeviation(vector<Object> TargetChunk, vector<Object> KeyRegion);

/***********************/
void TestProgramme(vector<Object> Chunk3, vector< vector<Object> > regions_Chunk2, vector<Object> route_Chunk3);
vector<Object> CombineRegions(vector<Object> region_Chunk3, vector< vector<Object> > regions_Chunk2, vector<Exit> exits_Chunk2);
vector<Object> configReturnView(vector<Object> return_View, vector<Object> chunk, vector< vector<Object> > regionOfChunk, Point exit_positionOnChunk);

/***********************/

vector<Object> RegionBlock(vector<Object> chunkMap, vector< vector<Object> > regions, vector<Exit> exits, 
                                vector<Object> route_segments, int chunkNum);

/*********************/
vector<Object> combinedChunkGeometry(vector< vector<Object> > allLEs, vector<Point> robpositions, LocalMap LMS);
vector<Object> returnPATH(vector<Point> rbs, Point key_exit, unsigned char flag);
vector<Object> consGeometry(vector<Object> chunk, vector<Object> path, unsigned char flag);

vector<Object> BlcokBoundaryView(vector<Object> view);
/*********************/

vector<Exit> ExitPerpendPath(vector<Object> routes);


/* compute connect parameters based upon connecting information
 * 
 */
pair<double, Point> connect_parameters(ChunkInfo current_chunk, ChunkInfo previous_chunk);

void correct_exit_info(vector<Exit> exits, vector<Object> current_region, vector<Object> previous_mfis);
vector<Exit> _correct_exit_info(vector<Exit> exits, vector<Object> current_region, vector<Object> previous_mfis);

//re-compute/re-locate these exits using specific surface information in current region
vector<Exit> reCompute_exit(vector<Object> current_region, vector<Exit> project_exits);

#endif /* CHUNKSOP_H */

