/*
 * Function: split chunk information into region
 *           identify exit using regions
 * author: Wenwang
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Point.H"
#include "PointAndSurface.H"
#include "Plotting.H"
#include "Object.H"
#include "mfisOp.H"
#include "ChunksOp.H"
#include "GeometricOp.H"
#include "GeometryFuncs.H"
#include "PointAndSurface.H"
#include "ToolFunctions.h"
#include "PathPlanning.H"
#include "readAndwriteASCII.H"
#include "GeometryAndExit.h"
#include "Ellipse.h"
#include "Exit.h"
#include "ChunkInfor.h"
#include "ConvexHull.h"

#define PI 3.14159265

vector<int> labels;       //robot step labers
vector<int> view_numbers; //key view/ASR numbers 

void RegionAndBoundary(vector< vector<Object> > allLEs, vector<Point> robpositions, LocalMap LMS,
                       vector< vector<Object> > robots)
{
    vector<Object> chunk;
    vector<Object> modiLM;
    vector<Object> CombinedRegion;
    vector<Point> rps;
    
    //vector<Point> rps_chunk;
    vector<Object> chunk_route; 
    
    vector<Exit> exits;
    vector<Object> route_segments;
    
    vector< vector<Object> > chunkWithLMs;
    vector< vector<Object> > allrps;
    vector< vector<Object> > allRegions;
    vector< vector<Object> > splitSpace;
    chunk = LMS.getGlobalMaps()[0];
    char fileName[80];
    
       
    for(int i = 1; i < 5; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }
    
    for(int i = 0; i < allLEs.size(); i++)
    {

        if(i == 0){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);
        chunk_route = robots[i];};
        if(i == 6){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 7){
            chunkWithLMs.push_back(allLEs[i]);  
        rps.push_back(robpositions[i]);}
        if(i == 11){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 12){
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        ///////////
        //if(i == 21)
        //    chunkWithLMs.push_back(allLEs[i]);
        ///////////
        if(i == 29){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 30)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 36){
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 37){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        
        if(i <= 37)
            chunk_route = addTwoVectorsOfObjects(chunk_route, robots[i]);

    }
    
    //cout << " the size of chunk LMS: " << chunkWithLMs.size() << endl;
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    //RegionBlock(chunk, splitSpace, exits, route_segments, 1);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits.png");
    //plotObjectsofexits(fileName, chunk, exits);
    //plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, chunk_route,exits);
    waitHere();
            
    chunkWithLMs.clear();
    rps.clear();  
    chunk.clear();
    
    chunk_route.clear();

    chunk = LMS.getGlobalMaps()[5];

    for(int i = 6; i < 9; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }    
    
    for(int i = 0; i < allLEs.size(); i++)
    {

        if(i == 48){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);
        chunk_route = robots[i];}
        ////////////////////
        //if(i == 53){
        //    chunkWithLMs.push_back(allLEs[i]);}
        ///////////////////
        if(i == 59){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 60){
            chunkWithLMs.push_back(allLEs[i]);  
        rps.push_back(robpositions[i]);}
        if(i == 63){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 64){
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 70){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 71)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}

        if(i > 48 && i <= 71)
            chunk_route = addTwoVectorsOfObjects(chunk_route, robots[i]);
    }
    //
    //CombinedRegion = RegionDeviation(chunk, splitSpace[splitSpace.size()-1]);
    //
    
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    //RegionBlock(chunk, splitSpace, exits, route_segments, 2);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits2.png");
    //plotObjectsofexits(fileName, chunk, exits);
    //plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, chunk_route,exits);
    waitHere();
 
   
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk_route.clear();
    
    chunk = LMS.getGlobalMaps()[9];

    for(int i = 10; i < 13; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }

    for(int i = 0; i < allLEs.size(); i++)
    {
        //if(i == 78)  {
        //    chunkWithLMs.push_back(allLEs[i]); 
        //rps.push_back(robpositions[i]);}
        if(i == 79)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);
        chunk_route = robots[i];}
        if(i == 90){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 91){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 95){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 96){
            chunkWithLMs.push_back(allLEs[i]);  
        rps.push_back(robpositions[i]);}
        //if(i == 142){
        //    chunkWithLMs.push_back(allLEs[i]);
        //rps.push_back(robpositions[i]);}
        if(i == 102){
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 103){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        
        if(i > 79 && i <= 103)
            chunk_route = addTwoVectorsOfObjects(chunk_route, robots[i]);
        
    }
    
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    
    vector< vector<Object> > splitSpace2 = DetectSpace(chunk, chunkWithLMs, route_segments);
    //CombineRegions(splitSpace2[0], splitSpace, exits);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    
    //RegionBlock(chunk, splitSpace, exits, route_segments, 3);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits3.png");
    //plotObjectsofexits(fileName, chunk, exits);
    //plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, chunk_route,exits);
    //configReturnView(chunk, chunk, splitSpace2, Point (0,0));

/*    
    chunk = LMS.getGlobalMaps()[0];
    for(int i = 1; i < 5; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }
    
    for(int i = 0; i < allLEs.size(); i++)
    {

        if(i == 0){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 5){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 6){
            chunkWithLMs.push_back(allLEs[i]);  
        rps.push_back(robpositions[i]);}
        if(i == 12){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 13){
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}

        if(i == 26){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 27)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 36){
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 37){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}

    }
    //cout << " the size of chunk LMS: " << chunkWithLMs.size() << endl;
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    RegionBlock(chunk, splitSpace, exits, route_segments, 1);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits.png");
    //plotObjectsofexits(fileName, chunk, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    waitHere();
            
    chunkWithLMs.clear();
    rps.clear();  
    chunk.clear();

    chunk = LMS.getGlobalMaps()[5];

    for(int i = 6; i < 7; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }    
    
    for(int i = 0; i < allLEs.size(); i++)
    {

        if(i == 50){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}


        if(i == 62){
            chunkWithLMs.push_back(allLEs[i]);  
        rps.push_back(robpositions[i]);}
        if(i == 63){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}



    }
    //
    //CombinedRegion = RegionDeviation(chunk, splitSpace[splitSpace.size()-1]);
    //
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    RegionBlock(chunk, splitSpace, exits, route_segments, 2);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits2.png");
    //plotObjectsofexits(fileName, chunk, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    waitHere();
 
   
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk = LMS.getGlobalMaps()[7];

    for(int i = 8; i < 9; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }

    for(int i = 0; i < allLEs.size(); i++)
    {
        //if(i == 78)  {
        //    chunkWithLMs.push_back(allLEs[i]); 
        //rps.push_back(robpositions[i]);}
        if(i == 76)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
    }

    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    
   //CombineRegions(splitSpace2[0], splitSpace, exits);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    
    RegionBlock(chunk, splitSpace, exits, route_segments, 3);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits3.png");
    //plotObjectsofexits(fileName, chunk, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    //configReturnView(chunk, chunk, splitSpace2, Point (0,0));        
    
    waitHere();
    
    
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk = LMS.getGlobalMaps()[9];

    for(int i = 10; i < LMS.getGlobalMaps().size(); i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }

    for(int i = 0; i < allLEs.size(); i++)
    {

        if(i == 88)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 95)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 96)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 102)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 103)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
    }

    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    
    RegionBlock(chunk, splitSpace, exits, route_segments, 4);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits4.png");
    //plotObjectsofexits(fileName, chunk, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    //configReturnView(chunk, chunk, splitSpace2, Point (0,0));          
*/            
/*    
    chunk = LMS.getGlobalMaps()[0];   
    for(int i = 1; i < 2; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }
    
    for(int i = 0; i < allLEs.size(); i++)
    {
        
        if(i == 0)
        {
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);
        }
        if(i == 21)
        {
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);
        }
        
        if(i == 22)
        {
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);
        }
        
        //if(i <= 22)
        //{
        //    chunkWithLMs.push_back(allLEs[i]);
        //    rps.push_back(robpositions[i]);
        //}
         
        sprintf(fileName, "%s", "Maps/Offline/TEST_AllASR_Points.png");
        plotObjectsColourAndPoint(fileName, chunkWithLMs, rps);
 
        
    }
    
    //cout << " the size of chunk LMS: " << chunkWithLMs.size() << endl;
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    RegionBlock(chunk, splitSpace, exits, route_segments, 1);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits.png");
    plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
 
    waitHere();
    
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk = LMS.getGlobalMaps()[2];

    for(int i = 3; i < 4; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }

    for(int i = 0; i < allLEs.size(); i++)
    {
       
        if(i == 34)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 39)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 40)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}

    }
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    RegionBlock(chunk, splitSpace, exits, route_segments, 2);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits2.png");
    plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
    waitHere();
    
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk = LMS.getGlobalMaps()[4];

    for(int i = 5; i < 7; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }

    for(int i = 0; i < allLEs.size(); i++)
    {
      
        if(i == 54)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 67){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 68){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 73){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 74){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}

    }

    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    //vector< vector<Object> > splitSpace2 = DetectSpace(chunk, chunkWithLMs, route_segments);
    //CombineRegions(splitSpace2[0], splitSpace, exits);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    
    RegionBlock(chunk, splitSpace, exits, route_segments, 3);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits3.png");
    plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
    
    waitHere();
    
    
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk = LMS.getGlobalMaps()[7];

    for(int i = 8; i < 9; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }

    for(int i = 0; i < allLEs.size(); i++)
    {
      
        if(i == 87)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 106){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);} 
        if(i == 107){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);} 

    }

    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    //vector< vector<Object> > splitSpace2 = DetectSpace(chunk, chunkWithLMs, route_segments);
    //CombineRegions(splitSpace2[0], splitSpace, exits);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    
    RegionBlock(chunk, splitSpace, exits, route_segments, 4);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits4.png");
    plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
*/    

     
}


vector<Object> RouteCombine(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Point> rps)
{
        Object temp_Obj, inters_Obj, path_Obj;
        vector<Object> route_segments; //routes stock
        vector<Point> cluster_points; //cluster stock 
        
        Point temp_Point, inter_Point;
        double offset = 800;
        double route_length_threshold = 10000; //route length threshold
        double cluster_threshold = 1500; //cluster threshold 
        double a, b, slope;
        char fileName[80];
        int type = 0;
        int i = 0;

        //unsigned char cluster_flag = 0;//clustering flag for route
        
        while( i < rps.size()-1)
        {

            if(i == 0 || i == rps.size() - 2)
            {
                temp_Obj.set(rps[i].X(), rps[i].Y(), 
                                rps[i+1].X(), rps[i+1].Y(),i);

                route_segments.push_back(temp_Obj);
                i += 1;
            }
            else
            {
                temp_Obj.set(rps[i].X(), rps[i].Y(), 
                                rps[i+2].X(), rps[i+2].Y(),i);

                route_segments.push_back(temp_Obj);
                i += 2;
            }
            
            //split path segment
            //vector<Point> cluster_robotposition(vector<Point> rbs, Object path_segment)
            //vector<Object> modifyPath(vector<Point> group_rbs, Object path_segment)
            
            //if(.size == 2)
            //{
            //    route_segments.pop_back();
            //    route_segments.push_back();
            //    route_segments.push_back();
            //}
        }
        
  
        sprintf(fileName, "%s", "Maps/Offline/TEST_PATH2.png");
        plotObjectsColours(fileName, LMs, route_segments);
        
        return route_segments;
}

/* cluster robot positions for path modification
 * input : all robot position in the chunk
 *          a path segment
 */
vector<Point> cluster_robotposition(vector<Point> rbs, Object path_segment)
{
    vector<Point> group_rbs;
    int start_flag = 0;
    //int end_flag = 0;
    
    for(int i = 0; i < rbs.size(); i++)
    {
        if(path_segment.X2() == rbs[i].X())
            break;
        
        if(start_flag = 0)
        {
            if(path_segment.X1() == rbs[i].X())
                start_flag = 1;
        }
        else
        {
            group_rbs.push_back(rbs[i]);
        }
    }
    
    return group_rbs;
}

/* modify each path segment using interpolation approach
 * input : group of robot position & a path segment 
 */
vector<Object> modifyPath(vector<Point> group_rbs, Object path_segment)
{
    double threshold = 1000;
    double dist = 0;
    
    Object temp1, temp2;
    vector<Object> modified_route;
    
    
    for(int i = 0; i < group_rbs.size(); i++)
    {
        dist = perpendicularDis(Point (path_segment.X1(), path_segment.Y1()), 
                                Point (path_segment.X2(), path_segment.Y2()), group_rbs[i]);
        
        if(dist > threshold)
        {
            temp1.set(path_segment.X1(), path_segment.Y1(), group_rbs[i].X(), group_rbs[i].Y(), 1);
            temp2.set(group_rbs[i].X(), group_rbs[i].Y(), path_segment.X2(), path_segment.Y2(), 2);
            modified_route.push_back(temp1);
            modified_route.push_back(temp2);
            break;
        }
        else
        {
            modified_route.push_back(path_segment);
        }
    }
    
    return modified_route;
}


vector< vector<Object> > DetectSpace(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Object> route_segments)
{
        cout << " This function is to detect the space along the route !!!" << endl;
        int cnt = 0;
        double slope;
        double max_left = 0;
        double max_right = 0;
        double offset = 5000;
        Point temp_Point, a, b, c, d; //left-bottom right-bottom left-top right-top
        Point left_top, right_top;
        vector<Point> allPoints;
        Object temp_Obj, ref1, ref2;
        vector<Object> temp;
        vector<Object> Multi_LM;
        vector<Object> front_and_behind;
        vector< vector<Object> > all;
        vector< vector<Object> > rnt;
        char fileName[80];

        for(int i = 0; i < route_segments.size(); i++)
        {
                slope =  (route_segments[i].Y2() - route_segments[i].Y1()) / (route_segments[i].X2() - route_segments[i].X1());
                
                //identify the top line
                temp_Point.set(route_segments[i].X2(), route_segments[i].Y2());
                c.set(temp_Point.X() - offset, temp_Point.Y());
                d.set(temp_Point.X() + offset, temp_Point.Y());
                
                //c and d establish the top line

                

                
                if(i % 2 == 0 && i != 0 && cnt < LMs.size())
                //if(i != 0 && cnt < LMs.size())
                {
                    Multi_LM = LMs[cnt];
                    cnt++;
                    Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                    cnt++;
                    //Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                    //cnt++;
                    temp = isBoundary(Multi_LM, route_segments[i]); 
                }
                else
                {
                        if(i == 0) // the first three views
                        {
                            Multi_LM = LMs[cnt];
                            cnt++;
                            Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                            cnt++;
                            //Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                            temp = isBoundary(Multi_LM, route_segments[i]);
                        }
                        
                }
                
                all.push_back(temp);
                temp.clear();
                Multi_LM.clear();
                
                /*
                if(route_segments.size() == 5)
                {
                        //cout << " there are 5 route segments !!" << endl;
                        if(i == 0)
                        {
                                Multi_LM = LMs[cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 1)
                        {
                                cnt = cnt - 1;    
                                Multi_LM = LMs[cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 2)
                        {
                                Multi_LM = LMs[++cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                //cnt++;
                                //Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 3)
                        {
                                Multi_LM = LMs[++cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 4)
                        {
                                Multi_LM = LMs[++cnt];
                                //cnt++;
                                //Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                }
                else
                {
                        //cout << " there are 4 route segments !!" << endl;
                    if(route_segments.size() == 4)
                    {
                        if(i == 0)
                        {
                                Multi_LM = LMs[cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 1)
                        {
                                cnt = cnt - 1;    
                                Multi_LM = LMs[cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                //cnt++;
                                //Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 2)
                        {
                                Multi_LM = LMs[++cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 3)
                        {
                                Multi_LM = LMs[++cnt];
                                //cnt++;
                                //Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                    }
                    else
                    {
                        if(i == 0)
                        {
                                Multi_LM = LMs[cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                        
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 1)
                        {
                                cnt = cnt-1;    
                                Multi_LM = LMs[cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                //cnt++;
                                //Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                        if(i == 2)
                        {
                                Multi_LM = LMs[++cnt];
                                cnt++;
                                Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                                temp = isBoundary(Multi_LM, route_segments[i]);
                                front_and_behind = FrontAndBehindWithinRegion(chunkMap, route_segments[i], (unsigned char) i);
                                temp = addTwoVectorsOfObjects(temp, front_and_behind);
                                all.push_back(temp);
                                temp.clear();
                                Multi_LM.clear();
                        }
                    }
                }
                */
        }
        
        cout << " distribute local maps finished !!" << endl;
        vector<Object> commonObjects = CommonSurface(all);
        //cout << " size of all : " << all.size() << endl;
        //sprintf(fileName, "%s", "Maps/Offline/TEST_PATH.png");
        //plotObjectsColours(fileName, all, route_segments, commonObjects);
        //plotObjectsOf3Kinds(fileName,route_segments, chunkMap, commonObjects);
        
        //plotting all individual region following corresponding route
        for(int i = 0; i < all.size(); i++)
        {
            sprintf(fileName, "%s%d%s", "Maps/Offline/TEST_PATH-", i, ".png");
            plotObjects(fileName,all[i]);
        }
        
        //vector<Exit> exits = IdentifyExits(all, route_segments);
        

        //sprintf(fileName, "%s", "Maps/Offline/TEST_PATH&Exits.png");
        //plotObjectsOf3KindswithExits(fileName,route_segments, chunkMap, exits);
        //plotObjectsofexits(fileName, chunkMap, exits);

        return all;
}

       
vector<Object> isBoundary(vector<Object> map, Object route)
{
    
        Point ref1, ref2, a, b, c, d;
        Object ref_Obj1, ref_Obj2, potential_Obj, per1, per2;
        int i = 0;
        double dt1, dt2, dt3, dt4;
        vector<Object> rnt;
        vector<Object> test;
        char fileName[80];

        //test.push_back(route);
        
        while(i < map.size())
        {
            //two end points perpendicular to the route
            //get two line segments
            ref1 = crossPerpend(Point (route.X1(), route.Y1()), Point (route.X2(), route.Y2()), 
                                        Point (map[i].X1(), map[i].Y1()));
            per1.set(map[i].X1(), map[i].Y1(), ref1.X(), ref1.Y(), 1);
            
            ref2 = crossPerpend(Point (route.X1(), route.Y1()), Point (route.X2(), route.Y2()), 
                                        Point (map[i].X2(), map[i].Y2()));
            per2.set(map[i].X2(), map[i].Y2(), ref2.X(), ref2.Y(), 2);
            //check whether this two lines are intersected with rest of  surfaces
            for(int j = 0; j < map.size(); j++)
            {
                if(j != i)
                {
                    
                        //with not intersection 
                        if((TwoObjectIntersect(map[j], per1) == false) && (TwoObjectIntersect(map[j], per2) == false))
                        {
                            potential_Obj = map[j];

                            //limited space lines top & bottom
                            pair<Point, Point> limit = outPerpenPoint(route);
                            //pair<Point, Point> limit = PerpenPointToRouteEnd(route);
                            ref_Obj1.set(limit.first.X(), limit.first.Y(), route.X1(), route.Y1(), 1);
                            ref_Obj2.set(limit.second.X(), limit.second.Y(), route.X2(), route.Y2(), 2);
                            
                            //test.push_back(ref_Obj1);
                            //test.push_back(ref_Obj2);
                            
                            //the tow end point of picked surface to limited lines , perpendicular distance 
                            dt1 = Perpendiculardistance(Point (ref_Obj1.X1(), ref_Obj1.Y1()), Point (ref_Obj1.X2(), ref_Obj1.Y2()), //x1 to line
                                                                    Point (potential_Obj.X1(), potential_Obj.Y1())) ;
                            dt2 = Perpendiculardistance(Point (ref_Obj2.X1(), ref_Obj2.Y1()), Point (ref_Obj2.X2(), ref_Obj2.Y2()),
                                                                    Point (potential_Obj.X1(), potential_Obj.Y1())) ;

                            dt3 = Perpendiculardistance(Point (ref_Obj1.X1(), ref_Obj1.Y1()), Point (ref_Obj1.X2(), ref_Obj1.Y2()), //x2 to line
                                                                    Point (potential_Obj.X2(), potential_Obj.Y2())) ;
                            dt4 = Perpendiculardistance(Point (ref_Obj2.X1(), ref_Obj2.Y1()), Point (ref_Obj2.X2(), ref_Obj2.Y2()),
                                                                    Point (potential_Obj.X2(), potential_Obj.Y2())) ;


                            //checking whether two end point to limited lines are less than space
                            if((dt1 < route.length() && dt2 < route.length()) || (dt3 < route.length() && dt4 < route.length()))
                                rnt.push_back(potential_Obj);
                        }
                }
            }

            i++;
        }

        
        return rnt;
    
}


/*Compute two regions, one is following the front end point
 *another is following the back end point based on the orientation of route
 *once the surface in the regions, collect them as front or behind surfaces
 */
vector<Object> FrontAndBehindWithinRegion(vector<Object> chunkMap, Object route, unsigned char start_flag)
{
        //Object front, behind;
        Object temp_Obj1, temp_Obj2, temp_Obj3, temp_Obj4;
        Object temp_Obj5, temp_Obj6, temp_Obj7, temp_Obj8;
        
        vector<Object> rnt;
        vector<Object> front, behind;
        vector<Object> front_region, behind_region;
        double slope = 0;
        double weidth = 0;
        double offset = 5000;

        Point mid_point; //mid point of route segment
        slope =  (route.Y2() - route.Y1()) / (route.X2() - route.X1());
        if(abs(slope) > 1) 
            weidth = abs(route.X2() - route.X1());
        else
            weidth = abs(route.Y2() - route.Y1());
        
        if(weidth >= 600)
            weidth = weidth / 2;


            //identify the orientation
            if(route.X1() < route.X2() && abs(slope) < 1) //horizontal
            {
                //front
                temp_Obj1.set(route.X2(), route.Y2() - weidth/2, route.X2(), route.Y2() + weidth/2, 1);
                temp_Obj2.set(route.X2(), route.Y2() + weidth/2, route.X2() + offset, route.Y2() + weidth/2, 2);
                temp_Obj3.set(route.X2() + offset, route.Y2() + weidth/2, route.X2() + offset, route.Y2() - weidth/2, 3);
                temp_Obj4.set(route.X2() + offset, route.Y2() - weidth/2, route.X2() , route.Y2() - weidth/2, 4);
                front_region.push_back(temp_Obj1);
                front_region.push_back(temp_Obj2);
                front_region.push_back(temp_Obj3);
                front_region.push_back(temp_Obj4);
                
                //behind
                temp_Obj5.set(route.X1(), route.Y1() + weidth/2, route.X1(), route.Y1() - weidth/2, 5);
                temp_Obj6.set(route.X1(), route.Y1() - weidth/2, route.X1() - offset, route.Y1() - weidth/2, 6);
                temp_Obj7.set(route.X1() - offset, route.Y1() - weidth/2, route.X1() - offset, route.Y1() + weidth/2, 7);
                temp_Obj8.set(route.X1() - offset, route.Y1() + weidth/2, route.X1(), route.Y1() + weidth/2, 8);
                behind_region.push_back(temp_Obj5);
                behind_region.push_back(temp_Obj6);
                behind_region.push_back(temp_Obj7);
                behind_region.push_back(temp_Obj8);
                
            }
            else
            {
                if(route.X1() > route.X2() && abs(slope) < 1)
                {
                    //front
                    temp_Obj1.set(route.X1(), route.Y1() - weidth/2, route.X1(), route.Y1() + weidth/2, 1);
                    temp_Obj2.set(route.X1(), route.Y1() + weidth/2, route.X1() + offset, route.Y1() + weidth/2, 2);
                    temp_Obj3.set(route.X1() + offset, route.Y1() + weidth/2, route.X1() + offset, route.Y1() - weidth/2, 3);
                    temp_Obj4.set(route.X1() + offset, route.Y1() - weidth/2, route.X1() , route.Y1() - weidth/2, 4);
                    front_region.push_back(temp_Obj1);
                    front_region.push_back(temp_Obj2);
                    front_region.push_back(temp_Obj3);
                    front_region.push_back(temp_Obj4);

                    //behind
                    temp_Obj5.set(route.X2(), route.Y2() + weidth/2, route.X2(), route.Y2() - weidth/2, 5);
                    temp_Obj6.set(route.X2(), route.Y2() - weidth/2, route.X2() - offset, route.Y2() - weidth/2, 6);
                    temp_Obj7.set(route.X2() - offset, route.Y2() - weidth/2, route.X2() - offset, route.Y2() + weidth/2, 7);
                    temp_Obj8.set(route.X2() - offset, route.Y2() + weidth/2, route.X2(), route.Y2() + weidth/2, 8);
                    behind_region.push_back(temp_Obj5);
                    behind_region.push_back(temp_Obj6);
                    behind_region.push_back(temp_Obj7);
                    behind_region.push_back(temp_Obj8);

                }
            }
                
                
            if(route.Y1() < route.Y2() && abs(slope) > 1 )// vertical
            {
                //front
                temp_Obj1.set(route.X2() - weidth/2, route.Y2(), route.X2() + weidth/2, route.Y2(), 1);
                temp_Obj2.set(route.X2() + weidth/2, route.Y2(), route.X2() + weidth/2, route.Y2() + offset, 2);
                temp_Obj3.set(route.X2() + weidth/2, route.Y2() + offset, route.X2() - weidth/2, route.Y2() + offset, 3);
                temp_Obj4.set(route.X2() - weidth/2, route.Y2() + offset, route.X2() - weidth/2, route.Y2(), 4);
                front_region.push_back(temp_Obj1);
                front_region.push_back(temp_Obj2);
                front_region.push_back(temp_Obj3);
                front_region.push_back(temp_Obj4);
                
                //behind
                temp_Obj5.set(route.X1() + weidth/2, route.Y1(), route.X1() - weidth/2, route.Y1(), 1);
                temp_Obj6.set(route.X1() - weidth/2, route.Y1(), route.X1() - weidth/2, route.Y1() - offset, 2);
                temp_Obj7.set(route.X1() - weidth/2, route.Y1() - offset, route.X1() + weidth/2, route.Y1() - offset, 3);
                temp_Obj8.set(route.X1() + weidth/2, route.Y1() - offset, route.X1() + weidth/2, route.Y1(), 4);
                behind_region.push_back(temp_Obj5);
                behind_region.push_back(temp_Obj6);
                behind_region.push_back(temp_Obj7);
                behind_region.push_back(temp_Obj8);
            
            }
            else
            {
                if(route.Y1() > route.Y2() && abs(slope) > 1)
                {
                    //front
                    temp_Obj1.set(route.X1() - weidth/2, route.Y1(), route.X1() + weidth/2, route.Y1(), 1);
                    temp_Obj2.set(route.X1() + weidth/2, route.Y1(), route.X1() + weidth/2, route.Y1() + offset, 2);
                    temp_Obj3.set(route.X1() + weidth/2, route.Y1() + offset, route.X1() - weidth/2, route.Y1() + offset, 3);
                    temp_Obj4.set(route.X1() - weidth/2, route.Y1() + offset, route.X1() - weidth/2, route.Y1(), 4);
                    front_region.push_back(temp_Obj1);
                    front_region.push_back(temp_Obj2);
                    front_region.push_back(temp_Obj3);
                    front_region.push_back(temp_Obj4);

                    //behind
                    temp_Obj5.set(route.X2() + weidth/2, route.Y2(), route.X2() - weidth/2, route.Y2(), 1);
                    temp_Obj6.set(route.X2() - weidth/2, route.Y2(), route.X2() - weidth/2, route.Y2() - offset, 2);
                    temp_Obj7.set(route.X2() - weidth/2, route.Y2() - offset, route.X2() + weidth/2, route.Y2() - offset, 3);
                    temp_Obj8.set(route.X2() + weidth/2, route.Y2() - offset, route.X2() + weidth/2, route.Y2(), 4);
                    behind_region.push_back(temp_Obj5);
                    behind_region.push_back(temp_Obj6);
                    behind_region.push_back(temp_Obj7);
                    behind_region.push_back(temp_Obj8);
                }
            }
        
        //generate polygon
        vector<Surface> polygon_front = makePolygonOfCV(front_region);
        vector<Surface> polygon_behind = makePolygonOfCV(behind_region);
        //collect front & behind surfaces
        for(int i = 0; i < chunkMap.size(); i++)
        {
            if((pointInPolygon(PointXY(chunkMap[i].X1(), chunkMap[i].Y1()), polygon_front) == true) 
                    || (pointInPolygon(PointXY(chunkMap[i].X2(), chunkMap[i].Y2()), polygon_front) == true))
                front.push_back(chunkMap[i]);
                
            if((pointInPolygon(PointXY(chunkMap[i].X1(), chunkMap[i].Y1()), polygon_behind) == true)
                || (pointInPolygon(PointXY(chunkMap[i].X2(), chunkMap[i].Y2()), polygon_behind) == true))
                  behind.push_back(chunkMap[i]) ;
        }
        
        /*
        for(int i = 0; i < chunkMap.size(); i++)
        {
            if((pointInPolygon(front_region, Point (chunkMap[i].X1(), chunkMap[i].Y1())) == true) 
                    || (pointInPolygon(front_region, Point (chunkMap[i].X2(), chunkMap[i].Y2())) == true))
                front.push_back(chunkMap[i]);
                
            if((pointInPolygon(behind_region, Point (chunkMap[i].X1(), chunkMap[i].Y1())) == true)
                || (pointInPolygon(behind_region, Point (chunkMap[i].X2(), chunkMap[i].Y2())) == true))
                  behind.push_back(chunkMap[i]) ;
        }
        */
        
        if(start_flag != 0)
        {
            rnt = front;
            rnt = addTwoVectorsOfObjects(rnt, behind);
        }
        else
        {
            rnt = front;
        }
       
        /*
        char fileName[80];
        vector<Object> testview;
        testview.push_back(route);
        sprintf(fileName, "%s%d%s", "Maps/Offline/TEST_box-", start_flag,".png");
        plotObjectsOf3Kinds(fileName, front_region, behind_region,  testview);
        */
        
        return rnt;
}


vector<Exit> IdentifyExitsFollowRoute(vector< vector<Object> > regions, vector<Object> route_segments)
{
        cout<<" Identifying the exits following routes !!!" << endl<<endl;
    
        vector<Object> Common_Surfaces; //common surfaces 
        int i = 0; //region label
        int j = 0; //route label
        int direction_flag = 0; //1 up 2 down 3 right 4 left
        double slope1 = 0;
        double slope2 = 0;
        double min = 10000;
        double offset = 1200;
        double To_ref_dist = 0;
        
        Object potential_Object;
        Object extend_Object;
        Point ref_Point; //the follow path, first end point
        Point intersect_Point;
        Point temp_point, temp_point2;
        Exit potential_exit; //between two adjacent region
        vector<Exit> exits;
        vector< vector<Object> > Two_adjacent_region;
        
        while(i < regions.size() - 1)
        {
                //slope
                slope1 =  (route_segments[i].Y2() - route_segments[i].Y1()) / (route_segments[i].X2() - route_segments[i].X1());
                slope2 =  (route_segments[i+1].Y2() - route_segments[i+1].Y1()) / (route_segments[i+1].X2() - route_segments[i+1].X1());
                
                if((abs(slope1) > 1)&&(route_segments[i].Y1() > route_segments[i].Y2())) //down
                    direction_flag = 2;
                if((abs(slope1) > 1)&&(route_segments[i].Y1() < route_segments[i].Y2())) //up
                    direction_flag = 1;
                if((abs(slope1) < 1)&&(route_segments[i].X1() < route_segments[i].X2())) //right
                    direction_flag = 3;
                if((abs(slope1) < 1)&& (route_segments[i].X1() > route_segments[i].X2())) //left
                    direction_flag = 4;
                
                //reference point from current route
                ref_Point.set(route_segments[i].X2(), route_segments[i].Y2());
                
                
                //find surface nearest and closest to the reference point 
                for(int j = 0; j < regions[i].size(); j++)
                {
                        //get the closest 
                        To_ref_dist = GetPointDistance(Point (regions[i][j].X1(), regions[i][j].Y1()), ref_Point);

                        if(To_ref_dist < min) 
                        {
                            if(direction_flag == 2) //going down
                            {
                                if(regions[i][j].Y1() > route_segments[i].Y2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X1(), regions[i][j].Y1());
                                }
                            }
                            
                            if(direction_flag == 1) //going up
                            {
                                if(regions[i][j].Y1() < route_segments[i].Y2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X1(), regions[i][j].Y1());
                                }
                            }
                            
                            if(direction_flag == 3) //going right
                            {
                                if(regions[i][j].X1() < route_segments[i].X2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X1(), regions[i][j].Y1());
                                }
                            }
                                
                            if(direction_flag == 4) //going left
                            {
                                if(regions[i][j].X1() > route_segments[i].X2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X1(), regions[i][j].Y1());
                                }
                            }
                        }

                        To_ref_dist = GetPointDistance(Point (regions[i][j].X2(), regions[i][j].Y2()), ref_Point);

                        if(To_ref_dist < min)
                        {
                            if(direction_flag == 2) //going down
                            {
                                if(regions[i][j].Y2() > route_segments[i].Y2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X2(), regions[i][j].Y2());
                                }
                            }
                            
                            if(direction_flag == 1) //going up
                            {
                                if(regions[i][j].Y2() < route_segments[i].Y2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X2(), regions[i][j].Y2());
                                }
                            }
                            
                            if(direction_flag == 3) //going right
                            {
                                if(regions[i][j].X2() < route_segments[i].X2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X2(), regions[i][j].Y2());
                                }
                            }
                                
                            if(direction_flag == 4) //going left
                            {
                                if(regions[i][j].X2() > route_segments[i].X2())
                                {
                                    min = To_ref_dist;
                                    temp_point.set(regions[i][j].X2(), regions[i][j].Y2());
                                }
                            }
                        }

                }
                
                
                if((isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), temp_point) > 0)
                        && (abs(slope1) > 1))
                {
                    if(direction_flag == 1)
                        temp_point2.set(temp_point.X()+offset, temp_point.Y());    //x + offset
                    if(direction_flag == 2)
                        temp_point2.set(temp_point.X()-offset, temp_point.Y());    //x + offset
                }
                                
                                
                if((isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), temp_point) > 0)
                        && (abs(slope1) < 1))
                {
                    if(direction_flag == 3)
                        temp_point2.set(temp_point.X(), temp_point.Y()-offset);    //y - offset
                    if(direction_flag == 4)
                        temp_point2.set(temp_point.X(), temp_point.Y()+offset);    //y - offset
                }
                
                if((isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), temp_point) < 0)
                        && (abs(slope1) > 1))
                {
                    if(direction_flag == 2)
                        temp_point2.set(temp_point.X()+offset, temp_point.Y());    //x - offset
                    if(direction_flag == 1)
                        temp_point2.set(temp_point.X()-offset, temp_point.Y());    //x - offset
                }
                
                if((isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), temp_point) < 0)
                        && (abs(slope1) < 1))
                {
                    if(direction_flag == 3)
                        temp_point2.set(temp_point.X(), temp_point.Y()+offset);    //y + offset
                    if(direction_flag == 4)
                        temp_point2.set(temp_point.X(), temp_point.Y()-offset);    //y + offset
                }
                
                
                potential_exit.set(temp_point.X(), temp_point.Y(), temp_point2.X(), temp_point2.Y());
                exits.push_back(potential_exit);
                i++;
                min = 10000;
        }
        
        
        return exits;
}

/*
 * Through the first surface of new chunk
 * to deviate/project the previous region/
 * find reference surfaces and using it to project on
 * current chunk/view
 */
vector<Object> RegionDeviation(vector<Object> TargetChunk, vector<Object> KeyRegion)
{
    Object ref_On_target, ref_On_region;
    vector<Object> temp;
    int refPoint = 2;
    double dist, slope1, slope2;
    double min = 5000;
    double threshold = 1000;
    
    char fileName[80];
    
    //reference obj from target chunk, left & first one
    ref_On_target = TargetChunk[0];
    
    slope1 =  (ref_On_target.Y2() - ref_On_target.Y1()) / (ref_On_target.X2() - ref_On_target.X1());
    for(int i = 0; i < KeyRegion.size(); i++)
    {
        slope2 =  (KeyRegion[i].Y2() - KeyRegion[i].Y1()) / (KeyRegion[i].X2() - KeyRegion[i].X1());
        dist = GetPointDistance(Point (KeyRegion[i].X2(), KeyRegion[i].Y2()), 
                                        Point (ref_On_target.X2(), ref_On_target.Y2()));
        if((abs(slope2 - slope1) <= 0.5) && dist < min)
        {
            ref_On_region =  KeyRegion[i];    
            min = dist;
        }
    }
    //temp = projectingTheView(KeyRegion, ref_On_region, ref_On_target, refPoint);
    if(min < threshold)
    {
            for(int j = 0; j < KeyRegion.size(); j++)
            {
                Object temp_Obj = remakeLineP2(ref_On_target, ref_On_region, KeyRegion[j], 1, 0, refPoint);
                temp.push_back(temp_Obj);
            }
            temp = addTwoVectorsOfObjects(temp, TargetChunk);
    }
    else
    {
            //// find reference object again
            
    }
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_Region.png");
    plotObjects(fileName, temp);
    
    return temp;
}

vector<Object> pointsToObjects(vector<Point> rps)
{
    Object temp_Obj;
    vector<Object> temp;
    
    for(int i = 0; i < rps.size()-1; i++)
    {
        temp_Obj.set(rps[i].X(), rps[i].Y(), rps[i+1].X(), rps[i+1].Y(), i);
        temp.push_back(temp_Obj);
    }
    
    temp_Obj.set(rps.back().X(), rps.back().Y(), rps[0].X(), rps[0].Y(), rps.size());
    temp.push_back(temp_Obj);
    
    return temp;
}

/* Intersection with a circle
 * inputs: ref_view, centre_circle, radius
 */
Point IntersectedWithCircle(Point cirCenter, double cirR, vector<Object> CV, double start, double end) 
{
    //const double PI=3.1415926;
    Point p1, p2; // line segment P1 & P2
    Point rnt_point;
    unsigned char flag = 0;
    double thete = start;
    double x, y, a, b;
    //double start = s;
    double step = 1 / 180 * PI;
    //double range = n * PI;

    for (int i = 0; i < CV.size(); i++) 
    {

        p1.set(CV[i].X1(), CV[i].Y1());
        p2.set(CV[i].X2(), CV[i].Y2());
        a = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
        b = (p1.X() * p2.Y() - p2.X() * p1.Y()) / (p1.X() - p2.X());

        //for (thete = start; thete <= end; thete = thete + step) 
        do
        {
            cout << "current angle : " << thete << endl;
            x = cirR * cos(thete) + cirCenter.X();
            y = cirR * sin(thete) + cirCenter.Y(); //x,y

            if (fabs(y - a * x - b) < 1e-5) //y=ax+b
            {
                
                if ((p1.X() <= x && x <= p2.X() || p1.X() >= x && x >= p2.X())
                        && (p1.Y() <= y && y <= p2.Y() || p1.Y() >= y && y >= p2.Y())) //
                {
                    //the arc is intesected with a surface       
                    flag = 1;
                    rnt_point.set(x, y);
                    break;
                }

            }
            thete += step;
        }while(thete <= end);
    }

    if(flag != 0)
        return  rnt_point;
    else
    {
        rnt_point.set(0, 0);
        return  rnt_point;
    }
}




/* Through intersection to determine size of ellipse
 * construct ellipse using basic lines
 * input: robot position with orientation
 * output: one or two points
 */

void constrainPointsToEllipse(vector< vector<Object> > robot_positions, vector<Object> ref_map)
{
        Object left_line, right_line; //left and right reference lines

        Point ref_point; //centre point
        Point Left_ref_point, Right_ref_point, centre_mod; //left right ref point
        Point left_near, right_near;
        double offset_length = 2000;
        double heigth = 1000;
        double width, orientation;
        
        char fileName[80];
        
        Ellipse temp_ellipse;
        vector<Ellipse> space_relate_robot;
        
           Object base_line, found_line;
           vector<Object> base_lines;
           vector<Object> foundation;
        
        for(int i = 0; i < robot_positions.size(); i++)
        {
            ref_point.set(robot_positions[i][6].X1(), robot_positions[i][6].Y1());
            //construct lines
            Left_ref_point = outSidePerpendPointWithLength(robot_positions[i][6], offset_length, ref_point, 2);
            Right_ref_point = outSidePerpendPointWithLength(robot_positions[i][6], offset_length, ref_point, 1);
            
            //cout << " left foundation point X : " << Left_ref_point.X() << " , Y : "<<Left_ref_point.Y()<<endl;
            //cout << " right foundation point X : " << Right_ref_point.X() << " , Y : "<<Right_ref_point.Y()<<endl;
            //waitHere();
            found_line.set(Left_ref_point.X(), Left_ref_point.Y(), Right_ref_point.X(), Right_ref_point.Y(),1);
            foundation.push_back(found_line);
            
            //check the nearest intersected points
            left_line.set(Left_ref_point.X(), Left_ref_point.Y(), ref_point.X(), ref_point.Y(), 1);
            right_line.set(Right_ref_point.X(), Right_ref_point.Y(), ref_point.X(), ref_point.Y(), 2);
            
            left_near = intersectedNearPoint(ref_map, Left_ref_point,ref_point);
            right_near = intersectedNearPoint(ref_map, Right_ref_point,ref_point);
            
            //test intersected points//
         
            if(left_near.X() != 0 && right_near.X() != 0)
            {
                base_line.set(left_near.X(), left_near.Y(), right_near.X(), right_near.Y(), 1);
                base_lines.push_back(base_line);
            }
            else
            {
                if(left_near.X() != 0)
                {
                    base_line.set(left_near.X(), left_near.Y(), ref_point.X(), ref_point.Y(), 1);
                    base_lines.push_back(base_line);
                }
                if(right_near.X() != 0)
                {
                    base_line.set(right_near.X(), right_near.Y(), ref_point.X(), ref_point.Y(), 1);
                    base_lines.push_back(base_line);
                }
            }
            ////////////////////////////
            
            if(left_near.X() != 0 && right_near.X() != 0)
            {
                //modify the centre point instead of robot position
                centre_mod.set((left_near.X() + right_near.X() / 2), (left_near.Y() + right_near.Y() / 2));
                width = distanceOftwoP(left_near, centre_mod);
                orientation = robot_positions[i][7].getAngleWithXaxis();
                temp_ellipse.setOneEllipse(ref_point, heigth, width, orientation);
            }
            else
            {
                if(left_near.X() != 0)
                    width = distanceOftwoP(left_near, ref_point)*2;
                if(right_near.X() != 0)
                    width = distanceOftwoP(right_near, ref_point)*2;
                
                orientation = robot_positions[i][7].getAngleWithXaxis();
                temp_ellipse.setOneEllipse(ref_point, heigth, width, orientation);
            }
            //store and return
            
            
            space_relate_robot.push_back(temp_ellipse);

        }
        
        
        sprintf(fileName, "%s", "Maps/Offline/TEST_Ellipse_Space.png");
        plotObjectsOf3KindswithEllipse(fileName, ref_map, robot_positions[0], space_relate_robot);
        sprintf(fileName, "%s", "Maps/Offline/TEST_Ellipse_basic_lines.png");
        plotObjectsOf3Kinds(fileName, ref_map, base_lines, robot_positions[0]);
        waitHere();
}


//void constructEllipseOfASR(vector< vector<Object> > robot_positions, vector< vector<Object> > views,
//                            vector<Object> ref_map, vector<Object> paths)
vector<Object> constructEllipseOfASR(vector< vector<Object> > robot_positions, vector< vector<Object> > views,
                            vector<Object> ref_map, vector< vector<Object> > key_views, int chunk_cnter)
{
        double offset_length = 2000; 
        double heigth = 1000;        //offset height of circle
        double width, orientation;   //width & orientation of circle
        
        char fileName[80];           //plot file
        Point centre_ellipse, mid1, mid2;  //centre point, left end point and right end point
        pair<Point, Point> base_points;    //store left and right end points
        vector< pair<Point, Point> > all_base_points; //store all circles left and right end points
        Ellipse temp_ellipse;              //one circle/ellipse
        vector<Ellipse> space_relate_robot; //all circle/ellipse
        
        Object temp, side;                  //temp base line & boundary line
        vector<Object> base_lines;          //all base line of circle
        vector<Object> geometry;            //boundary 
        
        //compute parameters of ellipse
        for(int i = 0; i < views.size(); i++)
        {
            mid1 = views[i][0].midpoint();
            mid2 = views[i].back().midpoint();
            centre_ellipse.set((mid1.X()+mid2.X())/2, (mid1.Y()+mid2.Y())/2);
            orientation = robot_positions[i][7].getAngleWithXaxis();
            width = distanceOftwoP(mid1, mid2)*2;
            temp_ellipse.setOneEllipse(centre_ellipse, heigth, width, orientation);
            temp_ellipse.setlTwoEndpoints(mid1, mid2);
            temp_ellipse.setVN(views[i][0].getVN());
            space_relate_robot.push_back(temp_ellipse);
            
            temp.set(mid1.X(), mid1.Y(), mid2.X(), mid2.Y(), i);
            base_lines.push_back(temp);
            
            //for reduce the space
            base_points.first = mid1;
            base_points.second = mid2;
            all_base_points.push_back(base_points);
            
        }
        
        
        vector<Ellipse> regulize_circles = RegulizationOfCircleSpace(all_base_points, space_relate_robot, robot_positions);
        pair< vector<Object>, vector< vector<Object> > > route_LMs = pathFollowEllipse(regulize_circles, views); //original algorithm
        //pair< vector<Object>, vector< vector<Object> > > route_LMs = pathFollowEllipseUsingPreStep(space_relate_robot, views); //modified algorihm
        
        //split chunk into regions and find exits between regions
        //vector< vector<Object> > splitSpace = DetectSpace(ref_map, route_LMs.second, route_LMs.first);
        //vector<Exit> exits = IdentifyExitsFollowRoute(splitSpace, route_LMs.first);
        
        //find all potential exits for ASRs
        //vector<Exit> exits = PotentialExitFromAlistOfView(route_LMs.second);
        //vector<Object> combinedViews = CombineAllViews(views);
        vector<Object> combinedViews = CombineAllViews(route_LMs.second);
        
        //RegionBlock(chunk, splitSpace, exits, route_segments, 1);
        ///NarrowSpace(combinedViews, route_LMs.first);


        information_process(views, route_LMs.first, robot_positions, ref_map, space_relate_robot, chunk_cnter);
        labels.clear();
        view_numbers.clear();
        waitHere();
        return route_LMs.first;

}

/* Construct box area/space using the first and last surface
 * from each ASR/View
 * input: ASRs
 * output: polygon/box space
 */
void PolyBasedASR(vector< vector<Object> > views, vector<Object> ref_map)
{
        char fileName[80];
        Object temp_Obj;
        vector<Object> base_lines;
        vector< vector<Object> > polygons;

        for(int i = 0; i < views.size(); i++)
        {
                temp_Obj.set(views[i][0].X2(), views[i][0].Y2(), views[i].back().X1(), views[i].back().Y1(), 2);
                base_lines.push_back(views[i][0]);
                base_lines.push_back(temp_Obj);
                base_lines.push_back(views[i].back());
                temp_Obj.set(views[i].back().X2(), views[i].back().Y2(),views[i][0].X1(), views[i][0].Y1(), 4);
                base_lines.push_back(temp_Obj);
                //construct closure polygon
                
                polygons.push_back(base_lines);
                base_lines.clear();
        }
        
        sprintf(fileName, "%s", "Maps/Offline/TEST_Ellipse_basic_lines.png");
        plotObjectsColours(fileName, polygons, ref_map);
        waitHere();
}

vector<Ellipse> RegulizationOfCircleSpace(vector< pair<Point, Point> > base_points, vector<Ellipse> space_relate_robot,
                                            vector< vector<Object> > robot_positions)
{
    double length, min;
    vector<Ellipse> rnt;
    Point left, right;
    
    rnt = space_relate_robot;
    
    //the smallest width of circle
    for(int i = 0; i < base_points.size(); i++)
    {
        if(i == 0)
            min = distanceOftwoP(base_points[i].first, base_points[i].second);
        else
        {
            length = distanceOftwoP(base_points[i].first, base_points[i].second);
            if(length < min)
                min = length;
        }
    }
    
    for(int i = 0; i < rnt.size(); i++)
    {
        rnt[i].setwidth(min); //reset width
        rnt[i].setCentre(Point (robot_positions[i][6].X1(), robot_positions[i][6].Y1())); //reset centre point
        
        vector<Point> four_point = rnt[i].fourEndPoints();
        Object base_line;
        base_line.set(rnt[i].getCentre().X(), rnt[i].getCentre().Y(), 
                      four_point[2].X(), four_point[2].Y(),  1);
        
        left = outSidePerpendPointWithLength(base_line, min/2, rnt[i].getCentre(), 1);
        right = outSidePerpendPointWithLength(base_line, min/2, rnt[i].getCentre(), 2);
        
        rnt[i].setlTwoEndpoints(left, right); //reset left & right end points
    }
    
    return rnt;
}

/* Compute all path segments follow ellipse space
 * input : a list of circle/ellipse
 * output : a list of path segments 
 */
pair< vector<Object>, vector< vector<Object> > > pathFollowEllipse(vector<Ellipse> robot_space, vector< vector<Object> > views)
{
        cout << "---- Path follow ellipse process ----" << endl;
        vector<Ellipse> space_circle; //all space infor
        vector<Ellipse> temp_list; 
        
        vector<Object> whole_path;
        vector< vector<Object> > local_maps;
        pair< vector<Object>, vector< vector<Object> > > paths_and_LMs;
        
        int cnt = 0;
        
        Object temp_Obj; //temp object variable
        
        local_maps.push_back(views[cnt]); //initialisation

        space_circle = robot_space;

lb:     for(int i = cnt; i < space_circle.size(); i++)
        {
            /*
                temp_list.push_back(space_circle[i]);
                for(int m = 1; (m + i) < space_circle.size(); m++)
                {
                    temp_list.push_back(space_circle[m + i]);
                    if(temp_list.size() > 2)
                    {
                        //join a line from start to end circles
                        temp_Obj.set(temp_list[0].getCentre().X(), temp_list[0].getCentre().Y(),
                                         temp_list.back().getCentre().X(), temp_list.back().getCentre().Y(), j);
                        for(int j = 2; j < temp_list.size(); j++)
                        {
                            
                            
                            //check whether this line is intersected with circles
                            if(intersectEllipse(temp_list[m + i], temp_Obj) == false)
                            {
                                //pop all circles
                                
                                //initial original input
                                
                                
                            }
                            else
                                goto 
                        }
                    }
                }
            */
   
    
            temp_list.push_back(space_circle[i]);
            if(temp_list.size() > 2)
            {
                temp_Obj.set(temp_list[0].getCentre().X(), temp_list[0].getCentre().Y(),
                                         temp_list.back().getCentre().X(), temp_list.back().getCentre().Y(), 0);
                //cout << " test programme " << endl;
                for(int m = 1; m < temp_list.size()-1; m++)
                {
                    //if(intersectEllipse(temp_list[m], temp_Obj) == false)
                    if(getIntersectionWithEllipse(temp_Obj.X1(), temp_Obj.X2(), temp_Obj.Y1(), temp_Obj.Y2(), 
                                temp_list[m].getCentre().X(), temp_list[m].getCentre().Y(), temp_list[m].getHeigth(), temp_list[m].getwidth())
                         == false)
                    {
                        //cout << " A path segment is generated !!!!" << endl;
                        //construct path
                        Object path_seg = ConstructPath(temp_list, m);
                        //cnt = i - temp_list.size() + 1 + m;
                        labels.push_back(cnt); //store the steps 
                        view_numbers.push_back(views[cnt][0].getVN()); //store key view number
                        cnt += m;
                        temp_list.clear();
                        
                        local_maps.push_back(views[cnt]);
                        
                        whole_path.push_back(path_seg);
                        
                        //char fileName[80];
                        //sprintf(fileName, "%s", "Maps/Offline/non-intersected circle.png");
                        //plotObjectsOf3KindswithEllipse(fileName, whole_path, whole_path,
                        //           temp_list[m].getCentre(), temp_list[m].getwidth(), temp_list[m].getHeigth(), temp_list[m].getOrientation());
                        //waitHere();
                        
                        goto lb;
                    }
                    else
                    {
                            if((i == space_circle.size() - 1) && (m == temp_list.size()-2))
                            {
                                Object path_seg = ConstructPath(temp_list, temp_list.size()-1);
                                whole_path.push_back(path_seg);
                                
                                local_maps.push_back(views[cnt]);
                                labels.push_back(cnt); //store the steps 
                                view_numbers.push_back(views[i][0].getVN());
                            }
                    }
                  
                }
                

            }
            

        }
   
        /*
        cout << " robot steps are : ";
        for(int m = 0; m < labels.size(); m++)
            cout << labels[m] << " ";
        
        cout << endl << endl;
        cout << " view numbers are : ";
        for(int m = 0; m < view_numbers.size(); m++)
            cout << view_numbers[m] << " ";
        waitHere();
        */

        paths_and_LMs.first = whole_path;
        paths_and_LMs.second = local_maps;
        return paths_and_LMs;
        
}

/* Compute all path segments follow ellipse space
 * input : a list of circle/ellipse
 * output : a list of path segments 
 */

pair< vector<Object>, vector< vector<Object> > > pathFollowEllipseUsingPreStep(vector<Ellipse> robot_space, vector< vector<Object> > views)
{
        cout << "---- Path follow ellipse process ----" << endl;
        vector<Ellipse> space_circle; //all space infor
        vector<Ellipse> temp_list; 
        
        vector<Object> whole_path;
        vector< vector<Object> > local_maps;
        pair< vector<Object>, vector< vector<Object> > > paths_and_LMs;
        int cnt = 0;
        Object temp_Obj; //temp object variable
        
        local_maps.push_back(views[cnt]); //initialisation

        space_circle = robot_space;

lb:     for(int i = cnt; i < space_circle.size(); i++)
        {
            
            temp_list.push_back(space_circle[i]);
            if(temp_list.size() > 2)
            {
                //joining two position points
                temp_Obj.set(temp_list[0].getCentre().X(), temp_list[0].getCentre().Y(),
                                         temp_list.back().getCentre().X(), temp_list.back().getCentre().Y(), 0);
                
                //cout << " test programme " << endl;
                for(int m = 1; m < temp_list.size()-1; m++)
                {
                    //if(intersectEllipse(temp_list[m], temp_Obj) == false)
                    if(getIntersectionWithEllipse(temp_Obj.X1(), temp_Obj.X2(), temp_Obj.Y1(), temp_Obj.Y2(), 
                                temp_list[m].getCentre().X(), temp_list[m].getCentre().Y(), temp_list[m].getHeigth(), temp_list[m].getwidth())
                         == false)
                    {
                        //cout << " A path segment is generated !!!!" << endl;
                        //construct path
                        //Object path_seg = ConstructPath(temp_list, m); //until the first non-intersected step from start point
                        Object path_seg = ConstructPath(temp_list, temp_list.size()-2); //until the previous step from start point
                        
                        //cnt = i - temp_list.size() + 1 + m;
                        //cnt += m;
                        cnt += temp_list.size()-2; //start from previous step
                        
                        temp_list.clear();
                        
                        local_maps.push_back(views[cnt]);
                        
                        whole_path.push_back(path_seg);
                        
                        goto lb;
                    }
                    else
                    {
                            if((i == space_circle.size() - 1) && (m == temp_list.size()-2))
                            {
                                Object path_seg = ConstructPath(temp_list, temp_list.size()-1);
                                whole_path.push_back(path_seg);
                                
                                local_maps.push_back(views[cnt]);
                            }
                    }
                  
                }
                

            }
            

        }

    paths_and_LMs.first = whole_path;
    paths_and_LMs.second = local_maps;
    return paths_and_LMs;
        
}

/*
public static ArrayList<Point2D> getIntersection(double x1, double x2, double y1, double y2, double midX, double midY, double h, double v) 
{
     ArrayList<Point2D> points = new ArrayList();

     x1 -= midX;
     y1 -= midY;

     x2 -= midX;
     y2 -= midY;

     if (x1 == x2) { 
         double y = (v/h)*Math.sqrt(h*h-x1*x1);
         if (Math.min(y1, y2) <= y && y <= Math.max(y1, y2)) {
             points.add(new Point2D(x1+midX, y+midY);
         }
         if (Math.min(y1, y2) <= -y && -y <= Math.max(y1, y2)) {
             points.add(newPoint2D(x1+midX, -y+midY);
         }
     }
     else {
         double a = (y2 - y1) / (x2 - x1);
         double b = (y1 - a*x1);

         double r = a*a*h*h + v*v;
         double s = 2*a*b*h*h;
         double t = h*h*b*b - h*h*v*v;

         double d = s*s - 4*r*t;

         if (d > 0) {
             double xi1 = (-s+Math.sqrt(d))/(2*r);
             double xi2 = (-s-Math.sqrt(d))/(2*r);

             double yi1 = a*xi1+b;
             double yi2 = a*xi2+b;

             if (isPointInLine(x1, x2, y1, y2, xi1, yi1)) {
                 points.add(new Point2D.Double(xi1+midX, yi1+midY);
             }
             if (isPointInLine(x1, x2, y1, y2, xi2, yi2)) {
                 points.add(new Point2D.Double(xi2+midX, yi2+midY);
             }
         }
         else if (d == 0) {
             double xi = -s/(2*r);
             double yi = a*xi+b;

             if (isPointInLine(x1, x2, y1, y2, xi, yi)) {
                 points.add(new Point2D.Double(xi+midX, yi+midY));
             }
         }
     }

     return points;
 }

 public static boolean isPointInLine(double x1, double x2, double y1, double y2, double px, double py) 
 {
     double xMin = Math.min(x1, x2);
     double xMax = Math.max(x1, x2);

     double yMin = Math.min(y1, y2);
     double yMax = Math.max(y1, y2);

     return (xMin <= px && px <= xMax) && (yMin <= py && py <= yMax);
 }
 */

/* Check whether a line is intersected with a ellipse/circle
 * input : a ellipse space, a line 
 * output : false / true
 */
bool intersectEllipse(Ellipse space, Object line)
{
    //cout << "---- intersection between one line and an ellipse ----" << endl;
    int flag = 0;
    vector<int> flags;
    vector<Point> endPoints;
    
    endPoints = space.fourEndPoints();
    //cout << " the size of four points of ellipse " << endPoints.size() << endl;
    for(int i = 0; i < endPoints.size(); i++)
    {
        if(isLeft(Point (line.X1(), line.Y1()), Point (line.X2(), line.Y2()), endPoints[i]) > 0)
            flag = 1;
        else
            flag = 0;
            
        flags.push_back(flag);
    }
    
    //for(int i = 0; i < flags.size()-1; i++)
    //{
        if(flags[0] == flags[1]
                &&flags[0] == flags[2]
                &&flags[0] == flags[3])
            return false;
        else
            return true;
    //}
    
    
}


Object ConstructPath(vector<Ellipse> list_ellipse, int m)
{
    Object path_segment;
    
    path_segment.set(list_ellipse[0].getCentre().X(), list_ellipse[0].getCentre().Y(),
                     list_ellipse[m].getCentre().X(), list_ellipse[m].getCentre().Y(), 1);
    
    return path_segment;

}


vector<Object> CombineAllViews(vector< vector<Object> > views)
{
    vector<Object> chunk_views;
    
    chunk_views = views[0];
    
    for(int i = 1; i < views.size(); i++)
    {
        chunk_views = addTwoVectorsOfObjects(chunk_views, views[i]);
    }
    
    return chunk_views;
}

/*line segment intersects with ellipse/circle
* if D < 0, there are no intersections
* if D = 0, there is one intersection
* if D > 0, there are two intersections
*/

bool getIntersectionWithEllipse(double x1, double x2, double y1, double y2, 
                                double midX, double midY, double h, double v) 
{
     //ArrayList<Point2D> points = new ArrayList();

     x1 -= midX;
     y1 -= midY;

     x2 -= midX;
     y2 -= midY;

     if (x1 == x2) 
     { 
         double y = (v/h)*sqrt(h*h-x1*x1);
         if (min(y1, y2) <= y && y <= max(y1, y2)) 
         {
             //points.add(new Point2D(x1+midX, y+midY);
         }
         if (min(y1, y2) <= -y && -y <= max(y1, y2)) 
         {
             //points.add(newPoint2D(x1+midX, -y+midY);
         }
     }
     else 
     {
         double a = (y2 - y1) / (x2 - x1);
         double b = (y1 - a*x1);

         double r = a*a*h*h + v*v;
         double s = 2*a*b*h*h;
         double t = h*h*b*b - h*h*v*v;

         double d = s*s - 4*r*t;

         if(d >= 0)
         {
             return true;
         }
         else
             return false;
         
         /*
         if (d > 0) 
         {
             double xi1 = (-s + sqrt(d))/(2*r);
             double xi2 = (-s - sqrt(d))/(2*r);

             double yi1 = a*xi1+b;
             double yi2 = a*xi2+b;

             if (isPointInLine(x1, x2, y1, y2, xi1, yi1)) 
             {
                 points.add(new Point2D.Double(xi1+midX, yi1+midY);
             }
             
             if (isPointInLine(x1, x2, y1, y2, xi2, yi2)) 
             {
                 points.add(new Point2D.Double(xi2+midX, yi2+midY);
             }
         }
         else 
             if (d == 0) 
             {
                double xi = -s/(2*r);
                double yi = a*xi+b;

                if (isPointInLine(x1, x2, y1, y2, xi, yi)) 
                {
                    points.add(new Point2D.Double(xi+midX, yi+midY));
                }
             }
         */
     }

     //return points;
}

bool isPointInLine(double x1, double x2, double y1, double y2, double px, double py)
{
    double xMin = min(x1, x2);
    double xMax = max(x1, x2);

    double yMin = min(y1, y2);
    double yMax = max(y1, y2);

    return (xMin <= px && px <= xMax) && (yMin <= py && py <= yMax);
}



/*  Based on base-line/boundary and overlapping 
 *  to split region into several including isolated and overlapping two parts
 *  input : a pair of space
 */
pair< vector<Object>, vector<Object> > SlpitOverlappingRegion(pair< vector<Object>, vector<Object> > Spaces)
{
        Object imagine_exit;
        
        //vector<Object> rectangle_on_imagExit;
        vector<Object> view1, view2;
        vector<Object> overlap_region, isolate_region;
        pair< vector<Object>, vector<Object> > rnt;
        
        
        view1 = Spaces.first;
        view2 = Spaces.second;

        //robot position where out of current region
        imagine_exit.set(view2[0].X1(), view2[0].Y1(), view2.back().X2(), view2.back().Y2(), 0);
        
        vector<Surface> polygon = makePolygonOfCV(view2);
        //split view1 into two regions by view2
        for(int i = 0; i < view1.size(); i++)
        {
                
                //if((pointInPolygon(PointXY(view1[i].X1(), view1[i].Y1()), polygon) == true ||
                //    pointInPolygon(PointXY(view1[i].X2(), view1[i].Y2()), polygon) == true))
                        //|| (isThisCloseToCVPolygonBoundary(view1[i], polygonObjects, 200.0) == true)) 
            
            if((isLeft(Point (imagine_exit.X1(), imagine_exit.Y1()), Point (imagine_exit.X2(), imagine_exit.Y2()),
                    Point (view1[i].X1(), view1[i].Y1())) > 0)
                || (isLeft(Point (imagine_exit.X1(), imagine_exit.Y1()), Point (imagine_exit.X2(), imagine_exit.Y2()),
                    Point (view1[i].X2(), view1[i].Y2())) > 0))
                {
                    //overlapping region
                    overlap_region.push_back(view1[i]);
                }
                else
                {
                    //isolating region
                    isolate_region.push_back(view1[i]);
                }
                
        }
        
        char fileName[80];
        sprintf(fileName, "%s", "Maps/Offline/splitspace.png");
        plotObjects(fileName, isolate_region, overlap_region);
        waitHere();
        
        rnt.first = isolate_region;
        rnt.second = overlap_region;
        
        return rnt;
}


/* cluster surfaces from a region
 * in order to roughly depict the 
 * structure of environment 
 * input  : region -- vector<Object>
 * output : a list of cluster -- vector<vector<Object>>
 */
vector<Object> Cluster_surface(vector<Object> region)
{
        vector<Object> cluster_group;
        vector< vector<Object> > all_clusters;
        
        int cnt = 0;     //counter for record next group
        vector<double> dist1, dist2; //check distance between two surfaces

        for(int i = cnt; i < region.size(); i++)
        {
                //intersected & close 
                if(cluster_group.size() == 0)
                {
                        cluster_group.push_back(region[i]);
                        cnt = i;
                }
                else
                {
                        //distance to all 'cluster_group' elements
                        //intersected or close
                    if(interSectWithLine(cluster_group, 
                                            Point (region[i].X1(), region[i].Y1()), Point (region[i].X2(), region[i].Y2())) == true)
                    {
                        cluster_group.push_back(region[i]);
                        cnt = i;
                    }
                    else
                    {
                        for(int n = 0; n < cluster_group.size(); n++)
                        {
                            dist1.push_back(P_To_ShortestDistance(Point (region[i].X1(), region[i].Y1()), cluster_group[n]));
                            dist2.push_back(P_To_ShortestDistance(Point (region[i].X2(), region[i].Y2()), cluster_group[n]));
                        }
                         
                        for(int m = 0; m < dist1.size(); m++)
                        {
                            if(dist1[m]<1000 || dist2[m] < 1000) //close case 
                            {
                                cluster_group.push_back(region[i]);
                                cnt = i;
                                break;
                            }
                        }
                    }
                }
        }
        
        
        return cluster_group;
}

/* polygon simplification algorithm
 * based on clipper library
 * input: view/map
 * output: simplified view
 */
vector<Object> Simplify_Clipper(vector<Object> map)
{
    ClipperLib::Path input;
    ClipperLib::Path output;
    Paths solution;
    
    vector<Object> polygonObjects;
    
    input = viewConvertPath(map); //convert view into Path
    
    SimplifyPolygon(input, solution, pftNegative);
    polygonObjects = PathsConvertView(solution); // convert polygon into view
    
    //char testFileName[80];
    //sprintf(testFileName, "%s", "Maps/Offline/Simplified_view.png");
    //plotObjects(testFileName, polygonObjects, map);
    //plotObjects(testFileName, polygonObjects);
    
    return polygonObjects;
    
}


pair< vector<Object>, vector<Object> > LeftAndRightList(vector<Object> region, Object path_segment)
{
    vector<Object> left, right;
    
    Point ref1, ref2;
    Point p1, p2;
    
    pair< vector<Object>, vector<Object> > rnt;
    
    ref1.set(path_segment.X1(), path_segment.Y1());
    ref2.set(path_segment.X2(), path_segment.Y2());
    
    for(int i  = 0; i < region.size(); i++)
    {       
        p1.set(region[i].X1(), region[i].Y1());
        p2.set(region[i].X2(), region[i].Y2());
        
            if(isLeft(ref1, ref2, p1) < 0 && isLeft(ref1, ref2, p2) < 0)
            {
                //left information
                right.push_back(region[i]);
            }
            else
            {
                if(isLeft(ref1, ref2, p1) > 0 && isLeft(ref1, ref2, p2) > 0)
                {
                    //right informiaont 
                    left.push_back(region[i]);
                    
                }
            }
    }
    
    rnt.first = left;
    rnt.second = right;
    
    return rnt;
}

/* Trim left and right list 
 * if the surface is blocked
 * with respect to the path segment
 * remove it.
 */
vector<Object> Trim_sides_group(vector<Object> group, Object path_segment)
{
    vector<Object> trimed_group;
    
    trimed_group = isBoundary(group, path_segment);
    
    return trimed_group;
}


/* Trim a list of surface with
 * respect to the line segment
 */
vector<Object> Trim_group(vector<Object> group, Object path_segment)
{
        vector<Object> trimed_group;
        Object ref_Obj;
        //cout << " the size before trimmed : " << group.size() << endl;

        //Trimming process
        for(int i =0; i < group.size(); i++)
        {
                ref_Obj = group[i];

                for(int j = 0; j < group.size(); j++)
                {
                        if(i != j)
                        {
                                if((TwoObjectIntersect(group[i], group[j]) == true)
                                    && ((group[i].X1() != group[j].X2())
                                        && group[i].X2() != group[j].X1()))
                                {
                                    if(group[i].length() > group[j].length())
                                    {    
                                        i = j;
                                        group.erase(group.begin()+j);
                                        break;
                                    }
                                    else
                                    {
                                        //i = i;
                                        group.erase(group.begin()+i);
                                        break;
                                    }
                                }
                        }
                }
        }

        trimed_group = group;
        //cout << " the size after trimmed  : " << trimed_group.size() << endl;
        
        return trimed_group;
}


/* shortest line from left to right
 * input : two lists of surfaces
 * output : a list of potential exits
 */
vector<Exit> Short_between_groups(vector<Object> left, vector<Object> right)
{
    vector<Object> temp;
    Object temp_Obj; 
    
    Exit temp_exit;
    vector<Exit> exits;
    
    double min = 5000;
    double dist;
    
    int insert_flag = 0;
    
    for(int i = 0; i < left.size(); i++)
    {
            for(int j = 0; j < right.size(); j++)
            {
                    dist = distanceOftwoP(Point (left[i].X1(), left[i].Y1()), Point(right[j].X1(), right[j].Y1()));
                    if(dist < min)
                    {
                        min = dist;
                        //temp_Obj.set(left[i].X1(), left[i].Y1(), right[j].X1(), right[j].Y1(), i);
                        temp_exit.set(left[i].X1(), left[i].Y1(), right[j].X1(), right[j].Y1());
                        
                        if(insert_flag == 1)
                            exits.pop_back();
                            //temp.pop_back();
                        
                        exits.push_back(temp_exit);
                        //temp.push_back(temp_Obj);
                        insert_flag = 1;
                    }

                    dist = distanceOftwoP(Point (left[i].X2(), left[i].Y2()), Point(right[j].X1(), right[j].Y1()));
                    if(dist < min)
                    {
                        min = dist;
                        //temp_Obj.set(left[i].X2(), left[i].Y2(), right[j].X1(), right[j].Y1(), i);
                        temp_exit.set(left[i].X1(), left[i].Y1(), right[j].X1(), right[j].Y1());
                        
                        if(insert_flag == 1)
                            exits.pop_back();
                            //temp.pop_back();

                        exits.push_back(temp_exit);
                        //temp.push_back(temp_Obj);
                        insert_flag = 1;
                    }

                    dist = distanceOftwoP(Point (left[i].X1(), left[i].Y1()), Point(right[j].X2(), right[j].Y2()));
                    if(dist < min)
                    {
                        min = dist;
                        //temp_Obj.set(left[i].X1(), left[i].Y1(), right[j].X2(), right[j].Y2(), i);
                        temp_exit.set(left[i].X1(), left[i].Y1(), right[j].X1(), right[j].Y1());

                        if(insert_flag == 1)
                            exits.pop_back();
                            //temp.pop_back();

                        exits.push_back(temp_exit);
                        //temp.push_back(temp_Obj);
                        insert_flag = 1;
                    }

                    dist = distanceOftwoP(Point (left[i].X2(), left[i].Y2()), Point(right[j].X2(), right[j].Y2()));
                    if(dist < min)
                    {
                        min = dist;
                        //temp_Obj.set(left[i].X2(), left[i].Y2(), right[j].X2(), right[j].Y2(), i);
                        temp_exit.set(left[i].X1(), left[i].Y1(), right[j].X1(), right[j].Y1());

                        if(insert_flag == 1)
                             exits.pop_back();
                            //temp.pop_back();

                        exits.push_back(temp_exit);
                        //temp.push_back(temp_Obj);
                        insert_flag == 1;
                    }
                    
                    //if(j == right.size() - 1)
                        

            }
            insert_flag = 0;
            min = 5000; //reset
    }
    
    
    //return temp;
    return exits;
}

/* compute block area based on region and potential exits
 * expend lines to find behind boundary to construct the block space
 */
vector<Object> BuildAreaOfSpace(vector<Object> region, vector<Exit> exits, 
                                  Object path_segment, Exit imagine_exit, int step)
{
    double length = 0;
    
    Object convert1, convert2;
    Object bottom, left, right, top; //boundary objects
    Object detect_boundary;          //expend one line to find a boundary point
    vector<Object> area;             //return variable
    
    Point exp_p1, exp_p2, cons1, cons2;
    
    //***************************// //part of test programme
    char fileName[80];
    vector<Object> test;
    vector<Exit> test_exit;
    Object test_obj1, test_obj2;
    //***************************//
    
    int exp_flag = 0;   //1 -- expend left  2 -- expend right
    int side_flag = 0; // 1 left 2 right
    int cnt = 1;
    
    if(step == 1)
    {
            bottom.set(exits[0].X1(), 0, exits[0].X2(), 0, 1);
            top.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
            right.set(exits[0].X2(), exits[0].Y2(), exits[0].X2(), 0, 3);
            left.set(exits[0].X1(), 0, exits[0].X1(), exits[0].Y1(), 4);

            right.setPositionFlag(4);
            bottom.setPositionFlag(2);
            left.setPositionFlag(3);
            top.setPositionFlag(1);

            area.push_back(bottom);
            area.push_back(left);
            area.push_back(right);
            area.push_back(top);
    }
    else
    {
        if(exits.size() == 1) 
        {
                cout << " Only one exit in this region !!! " << endl;
                //compute two expend lines on right hand side
                length = 2000;//length
                 
 
                convert1.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 1);
                if(isLeft(convert1.getP1(), convert1.getP2(), imagine_exit.getmidPoint()) > 0)//direction
                    side_flag = 1;
                else
                    side_flag = 2;
                
                exp_p1 = outSidePerpendPointWithLength(convert1, length, exits[0].getP1(), side_flag); //1 to left 2 to right
                exp_p2 = outSidePerpendPointWithLength(convert1, length, exits[0].getP2(), side_flag);
                //intersecting with imagine exit

                if((twoLinintersect(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP1(), exp_p1) == true)
                    && (twoLinintersect(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP2(), exp_p2) == true))
                {
                    cons1 = crossedPointReturnP(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP1(), exp_p1);
                    cons2 = crossedPointReturnP(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP2(), exp_p2);
                
                    //construct area

                    bottom.set(cons2.X(), cons2.Y(), cons1.X(), cons1.Y(), 1);
                    left.set(cons1.X(), cons1.Y(), exits[0].X1(), exits[0].Y1(), 2);
                    right.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 3);
                    top.set(exits[0].X2(), exits[0].Y2(), cons2.X(), cons2.Y(), 4);
                    
                    right.setPositionFlag(1);
                    bottom.setPositionFlag(2);
                    left.setPositionFlag(3);
                    top.setPositionFlag(4);
                    
                }
                else
                {
                        cout << " One of lines is NOT intersection !!! " << endl;
                        //goto lb;
                    
                        if(twoLinintersect(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP1(), exp_p1) == true)
                        {
                            cons1 = crossedPointReturnP(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP1(), exp_p1);

                            //expend another one to find cons2
                            detect_boundary.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(),2);
                            exp_flag = 1;
                            cout << " expend second line to find boundary !!!" << endl;
                        }
                        else
                        {
                            if(twoLinintersect(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP2(), exp_p2) == true)
                            {
                                cons2 = crossedPointReturnP(imagine_exit.getP1(), imagine_exit.getP2(), exits[0].getP2(), exp_p2);

                                //expend another one to find cons2
                                detect_boundary.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(),2);
                                exp_flag = 2;
                                cout << " expend first line to find boundary !!!" << endl;
                            }
                            else
                            {
                                exp_flag = 3;
                                cout << " Nether two expend line are intersected with imagine exit." << endl;
                            }
                        }

                        if(exp_flag == 1) //find boudnary point
                        {
                            //detect whether this intersected object is the boundary
                            Point boundry_point =  BoundaryPoint(region, detect_boundary, side_flag);
                            cout << " boundary point x : " << boundry_point.X() << " ; y : " << boundry_point.Y() << endl;
                            
                            bottom.set(cons1.X(), cons1.Y(), exits[0].X1(), exits[0].Y1(), 1);
                            left.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                            right.set(exits[0].X2(), exits[0].Y2(), boundry_point.X(), boundry_point.Y(), 3);
                            top.set(boundry_point.X(), boundry_point.Y(), cons1.X(), cons1.Y(), 4);
                            
                            right.setPositionFlag(4);
                            bottom.setPositionFlag(3);
                            left.setPositionFlag(1);
                            top.setPositionFlag(2);
                        }
                        else
                        {
                                if(exp_flag == 2)
                                {
                                    //detect whether this intersected object is the boundary
                                    Point boundry_point =  BoundaryPoint(region, detect_boundary, side_flag);
                                    cout << " boundary point x : " << boundry_point.X() << " ; y : " << boundry_point.Y() << endl;
                                    cout << " cons point x: " << cons2.X() << "; y : " << cons2.Y() << endl;
                                    bottom.set(boundry_point.X(), boundry_point.Y(), exits[0].X1(), exits[0].Y1(), 1);
                                    left.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                                    right.set(exits[0].X2(), exits[0].Y2(), cons2.X(), cons2.Y(), 3);
                                    top.set(cons2.X(), cons2.Y(), boundry_point.X(), boundry_point.Y(), 4);

                                    right.setPositionFlag(4);
                                    bottom.setPositionFlag(3);
                                    left.setPositionFlag(1);
                                    top.setPositionFlag(2);
                                }
                                else
                                {
                                    Object detect_line1, detect_line2;
                                    
                                    detect_line2.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(),2);
                                    detect_line1.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(),2);
                                    
                                    Point boundry_point1 =  BoundaryPoint(region, detect_line1, side_flag);
                                    Point boundry_point2 =  BoundaryPoint(region, detect_line2, side_flag);
                                    
                                    if(boundry_point1.X() == 0 && boundry_point2.X() == 0)
                                    {
                                        bottom.set(imagine_exit.X2(), imagine_exit.Y2(), imagine_exit.X1(), imagine_exit.Y1(), 1);
                                        top.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                                        right.set(exits[0].X2(), exits[0].Y2(), imagine_exit.X2(), imagine_exit.Y2(), 3);
                                        left.set(imagine_exit.X1(), imagine_exit.Y1(), exits[0].X1(), exits[0].Y1(), 4);
                                    }
                                    else
                                    {
                                        bottom.set(boundry_point1.X(), boundry_point1.Y(), boundry_point2.X(), boundry_point2.Y(), 1);
                                        top.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                                        right.set(exits[0].X2(), exits[0].Y2(), boundry_point2.X(), boundry_point2.Y(), 3);
                                        left.set(boundry_point1.X(), boundry_point1.Y(), exits[0].X1(), exits[0].Y1(), 4);
                                    }
                                    
                                    
                                    right.setPositionFlag(4);
                                    bottom.setPositionFlag(2);
                                    left.setPositionFlag(3);
                                    top.setPositionFlag(1);
                                }
                        }

                        exp_flag = 0;
                    
                }
                
                
                test_exit.push_back(imagine_exit);
                test_obj1.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(), 1);
                test_obj2.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(), 2);
                test.push_back(test_obj1);
                test.push_back(test_obj2);
                sprintf(fileName, "%s", "Maps/Offline/test_expend_region.png");
                plotObjectsOf3KindswithExits(fileName, test, region, test_exit);
                waitHere();
                


                area.push_back(bottom);
                area.push_back(left);
                area.push_back(right);
                area.push_back(top);
               
        }
        else
        {  
            length = 500;
            cout << " More exits in this region !!! " << endl;
            cout << " Size of exits : " << exits.size() << endl;
            
            //direction
            
            //length
            
                if(exits.size() > 1)
                {
                        //first exit is near -- width of space
                        convert1.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 1);
                        
                        if(isLeft(convert1.getP1(), convert1.getP2(), imagine_exit.getmidPoint()) > 0)
                            side_flag = 1;
                        else
                            side_flag = 2;
                        
                        exp_p1 = outSidePerpendPointWithLength(convert1, length, exits[0].getP1(), side_flag);
                        exp_p2 = outSidePerpendPointWithLength(convert1, length, exits[0].getP2(), side_flag);
                        
                      
                        
                        test_exit.push_back(imagine_exit);
                        test_obj1.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(), 1);
                        test_obj2.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(), 2);
                        test.push_back(test_obj1);
                        test.push_back(test_obj2);
                        sprintf(fileName, "%s", "Maps/Offline/test_expend_region.png");
                        plotObjectsOf3KindswithExits(fileName, test, region, test_exit);
                        waitHere();
                        

                        //intersecting with imagine exit

                        if(twoLinintersect(exits[1].getP1(), exits[1].getP2(), exits[0].getP1(), exp_p1) == true)
                        {
                            cons1 = crossedPointReturnP(exits[1].getP1(), exits[1].getP2(), exits[0].getP1(), exp_p1);

                            //expend another one to find cons2
                            detect_boundary.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(),2);
                            exp_flag = 1;
                            cout << " expend second line to find boundary !!!" << endl;
                        }
                        else
                        {
                            if(twoLinintersect(exits[1].getP1(), exits[1].getP2(), exits[0].getP2(), exp_p2) == true)
                            {
                                cons2 = crossedPointReturnP(exits[1].getP1(), exits[1].getP2(), exits[0].getP2(), exp_p2);

                                //expend another one to find cons2
                                detect_boundary.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(),2);
                                exp_flag = 2;
                                cout << " expend first line to find boundary !!!" << endl;
                            }
                            else
                            {
                                exp_flag = 3;
                                cout << " Nether two expend line are intersected with imagine exit." << endl;
                            }
                        }

                        if(exp_flag == 1) //find boudnary point
                        {
                            //detect whether this intersected object is the boundary
                            Point boundry_point =  BoundaryPoint(region, detect_boundary, side_flag);
                            
                            bottom.set(cons1.X(), cons1.Y(), exits[0].X1(), exits[0].Y1(), 1);
                            left.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                            right.set(exits[0].X2(), exits[0].Y2(), boundry_point.X(), boundry_point.Y(), 3);
                            top.set(boundry_point.X(), boundry_point.Y(), cons1.X(), cons1.Y(), 4);
                            
                            right.setPositionFlag(4);
                            bottom.setPositionFlag(3);
                            left.setPositionFlag(1);
                            top.setPositionFlag(2);
                        }
                        else
                        {
                            if(exp_flag == 2)
                            {
                                //detect whether this intersected object is the boundary
                                Point boundry_point =  BoundaryPoint(region, detect_boundary, side_flag);
                                
                                bottom.set(boundry_point.X(), boundry_point.Y(), exits[0].X1(), exits[0].Y1(), 1);
                                left.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                                right.set(exits[0].X2(), exits[0].Y2(), cons2.X(), cons2.Y(), 3);
                                top.set(cons2.X(), cons2.Y(), boundry_point.X(), boundry_point.Y(), 4);
                                
                                right.setPositionFlag(4);
                                bottom.setPositionFlag(3);
                                left.setPositionFlag(1);
                                top.setPositionFlag(2);
                            }
                        }

                        exp_flag = 0;
                }
            
                area.push_back(bottom);
                area.push_back(left);
                area.push_back(right);
                area.push_back(top);
            
        }
    }
    
    

    
    return area;

}

vector<Object> BuildAreaOfSpace(vector<Object> region, vector<Exit> exits, 
                                Object path_segment, Object entrance, int region_num)
{
    double length = 0;
    
    Object convert1, convert2;
    Object bottom, left, right, top; //boundary objects
    Object detect_boundary;          //expend one line to find a boundary point
    vector<Object> area;             //return variable
    
    Point exp_p1, exp_p2, cons1, cons2;
    
    //***************************// //part of test programme
    char fileName[80];
    vector<Object> test;
    vector<Exit> test_exit;
    Object test_obj1, test_obj2;
    //***************************//
    
    int exp_flag = 0;   //1 -- expend left  2 -- expend right
    int side_flag = 0; // 1 left 2 right
    int cnt = 1;
    
    if(region_num != 1)
    {
        if(exits.size() == 1) 
        {
            cout << " Only one exit in this region !!! " << endl;
                //compute two expend lines on right hand side
                length = 2000;//length
 
                convert1.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 1);
                if(isLeft(convert1.getP1(), convert1.getP2(), path_segment.getP1()) > 0)//direction
                    side_flag = 1;
                else
                    side_flag = 2;
                
                exp_p1 = outSidePerpendPointWithLength(convert1, length, exits[0].getP1(), side_flag); //1 to left 2 to right
                exp_p2 = outSidePerpendPointWithLength(convert1, length, exits[0].getP2(), side_flag);
                //intersecting with imagine exit

                if((twoLinintersect(entrance.getP1(), entrance.getP2(), exits[0].getP1(), exp_p1) == true)
                    && (twoLinintersect(entrance.getP1(), entrance.getP2(), exits[0].getP2(), exp_p2) == true))
                {
                    cons1 = crossedPointReturnP(entrance.getP1(), entrance.getP2(), exits[0].getP1(), exp_p1);
                    cons2 = crossedPointReturnP(entrance.getP1(), entrance.getP2(), exits[0].getP2(), exp_p2);
                
                    //construct area

                    bottom.set(cons2.X(), cons2.Y(), cons1.X(), cons1.Y(), 1);
                    left.set(cons1.X(), cons1.Y(), exits[0].X1(), exits[0].Y1(), 2);
                    right.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 3);
                    top.set(exits[0].X2(), exits[0].Y2(), cons2.X(), cons2.Y(), 4);
                    
                    right.setPositionFlag(1);
                    bottom.setPositionFlag(2);
                    left.setPositionFlag(3);
                    top.setPositionFlag(4);
                    
                }
                else
                {
                    cout << " One of lines is NOT intersection !!! " << endl;
                    //goto lb;
                    
                        if(twoLinintersect(entrance.getP1(), entrance.getP2(), exits[0].getP1(), exp_p1) == true)
                        {
                            cons1 = crossedPointReturnP(entrance.getP1(), entrance.getP2(), exits[0].getP1(), exp_p1);

                            //expend another one to find cons2
                            detect_boundary.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(),2);
                            exp_flag = 1;
                            cout << " expend second line to find boundary !!!" << endl;
                        }
                        else
                        {
                            if(twoLinintersect(entrance.getP1(), entrance.getP2(), exits[0].getP2(), exp_p2) == true)
                            {
                                cons2 = crossedPointReturnP(entrance.getP1(), entrance.getP2(), exits[0].getP2(), exp_p2);

                                //expend another one to find cons2
                                detect_boundary.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(),2);
                                exp_flag = 2;
                                cout << " expend first line to find boundary !!!" << endl;
                            }
                            else
                            {
                                exp_flag = 3;
                                cout << " Nether two expend line are intersected with imagine exit." << endl;
                            }
                        }

                        if(exp_flag == 1) //find boudnary point
                        {
                            //detect whether this intersected object is the boundary
                            Point boundry_point =  BoundaryPoint(region, detect_boundary, side_flag);
                            cout << " boundary point x : " << boundry_point.X() << " ; y : " << boundry_point.Y() << endl;
                            
                            bottom.set(cons1.X(), cons1.Y(), exits[0].X1(), exits[0].Y1(), 1);
                            left.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                            right.set(exits[0].X2(), exits[0].Y2(), boundry_point.X(), boundry_point.Y(), 3);
                            top.set(boundry_point.X(), boundry_point.Y(), cons1.X(), cons1.Y(), 4);
                            
                            right.setPositionFlag(4);
                            bottom.setPositionFlag(3);
                            left.setPositionFlag(1);
                            top.setPositionFlag(2);
                        }
                        else
                        {
                            if(exp_flag == 2)
                            {
                                //detect whether this intersected object is the boundary
                                Point boundry_point =  BoundaryPoint(region, detect_boundary, side_flag);
                                cout << " boundary point x : " << boundry_point.X() << " ; y : " << boundry_point.Y() << endl;
                                cout << " cons point x: " << cons2.X() << "; y : " << cons2.Y() << endl;
                                bottom.set(boundry_point.X(), boundry_point.Y(), exits[0].X1(), exits[0].Y1(), 1);
                                left.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                                right.set(exits[0].X2(), exits[0].Y2(), cons2.X(), cons2.Y(), 3);
                                top.set(cons2.X(), cons2.Y(), boundry_point.X(), boundry_point.Y(), 4);
                                
                                right.setPositionFlag(4);
                                bottom.setPositionFlag(3);
                                left.setPositionFlag(1);
                                top.setPositionFlag(2);
                            }
                            else
                            {
                                {
                                    Object detect_line1, detect_line2;
                                    
                                    detect_line2.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(),2);
                                    detect_line1.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(),2);
                                    
                                    Point boundry_point1 =  BoundaryPoint(region, detect_line1, side_flag);
                                    Point boundry_point2 =  BoundaryPoint(region, detect_line2, side_flag);
                                    
                                    if(boundry_point1.X() == 0 && boundry_point2.X() == 0)
                                    {
                                        bottom.set(entrance.X2(), entrance.Y2(), entrance.X1(), entrance.Y1(), 1);
                                        top.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                                        right.set(exits[0].X2(), exits[0].Y2(), entrance.X2(), entrance.Y2(), 3);
                                        left.set(entrance.X1(), entrance.Y1(), exits[0].X1(), exits[0].Y1(), 4);
                                    }
                                    else
                                    {
                                        bottom.set(boundry_point1.X(), boundry_point1.Y(), boundry_point2.X(), boundry_point2.Y(), 1);
                                        top.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                                        right.set(exits[0].X2(), exits[0].Y2(), boundry_point2.X(), boundry_point2.Y(), 3);
                                        left.set(boundry_point1.X(), boundry_point1.Y(), exits[0].X1(), exits[0].Y1(), 4);
                                    }
                                    
                                    
                                    right.setPositionFlag(4);
                                    bottom.setPositionFlag(2);
                                    left.setPositionFlag(3);
                                    top.setPositionFlag(1);
                                }
                            }
                        }

                        exp_flag = 0;
                    
                }
                
                /*
                test_exit.push_back(imagine_exit);
                test_obj1.set(exits[0].X1(), exits[0].Y1(), exp_p1.X(), exp_p1.Y(), 1);
                test_obj2.set(exits[0].X2(), exits[0].Y2(), exp_p2.X(), exp_p2.Y(), 2);
                test.push_back(test_obj1);
                test.push_back(test_obj2);
                sprintf(fileName, "%s", "Maps/Offline/test_expend_region.png");
                plotObjectsOf3KindswithExits(fileName, test, region, test_exit);
                waitHere();
                */


                area.push_back(bottom);
                area.push_back(left);
                area.push_back(right);
                area.push_back(top);
               
        }
    }
    else
    {
                bottom.set(exits[0].X1(), 0, exits[0].X2(), 0, 1);
                top.set(exits[0].X1(), exits[0].Y1(), exits[0].X2(), exits[0].Y2(), 2);
                right.set(exits[0].X2(), exits[0].Y2(), exits[0].X2(), 0, 3);
                left.set(exits[0].X1(), 0, exits[0].X1(), exits[0].Y1(), 4);

                right.setPositionFlag(4);
                bottom.setPositionFlag(2);
                left.setPositionFlag(3);
                top.setPositionFlag(1);
                                
                area.push_back(bottom);
                area.push_back(left);
                area.push_back(right);
                area.push_back(top);
    }
        
    
    

    
    return area;

}


Point BoundaryPoint(vector<Object> map, Object ref_obj, int side_flag)
{
        Object exp_segment;

        double exp_length = 400;
        int cnt = 1;

        Point boundary_point;
        exp_segment = ref_obj;
        
        //extend direction
/*
vector<Object> test;
char fileName[80];
test.push_back(exp_segment);
sprintf(fileName, "%s", "Maps/Offline/test_expend_line.png");
plotObjects(fileName, test, map);
waitHere();       
*/
lb:     exp_segment = expend_Object(exp_segment, exp_length, side_flag);

/*
test.clear();
test.push_back(exp_segment);
sprintf(fileName, "%s", "Maps/Offline/test_expend_line.png");
plotObjects(fileName, test, map);
waitHere();
*/
        if(interSectWithLine(map, exp_segment.getP1(), exp_segment.getP2()) == true)
        {
            cout << " Expended line is intersected !!! " << endl;
            //record intersected point
            boundary_point = intersectedPoint(map, exp_segment.getP1(), exp_segment.getP2());
            cout << " boundary point x : " << boundary_point.X() << " ; y : " << boundary_point.Y() << endl;
            //cnt++;
            goto lb;
        }
        else
        {
            if(cnt > 6)
            {
                if(boundary_point.X() < 1e-6 && boundary_point.Y() < 1e-6)
                {
                    boundary_point = intersectedPoint(map,ref_obj.getP1(), ref_obj.getP2());
                    return boundary_point;
                }
                else
                    return boundary_point;
                
            }
            else
            {
                cnt++;
                goto lb;
            }
        }

    //return boundary_point;
}


/*
Point BoundaryPointWithFlag(vector<Object> map, Object ref_obj, int flag)
{
        Object exp_segment;

        double exp_length = 500;
        int cnt = 1;

        Point boundary_point;
        exp_segment = ref_obj;
        
        //extend direction

lb:     exp_segment = expend_Object(exp_segment, exp_length, flag);

        if(interSectWithLine(map, exp_segment.getP1(), exp_segment.getP2()) == true)
        {
            //record intersected point
            boundary_point = intersectedPoint(map, exp_segment.getP1(), exp_segment.getP2());
            //cnt++;
            goto lb;
        }
        else
        {
            if(cnt > 5)
                return boundary_point;
            else
            {
                cnt++;
                goto lb;
            }
        }
}
*/  


//check whether the imagine exit is quite wide
//compute block between two adjacent small exits
vector<Object> Block_CrossWideSpace(vector<Object> region1, vector<Object> region2,
                                    Exit exit1, Exit exit2)
{
        Object bottom, top,left, right;
        vector<Object> rnt_block;

        //simply connect exit1 & 2
        bottom.set(exit1.X1(), exit1.Y1(), exit1.X2(), exit1.Y2(), 1);
        top.set(exit2.X1(), exit2.Y1(), exit2.X2(), exit2.Y2(), 3);

        left.set(exit1.X1(), exit1.Y1(),exit2.X1(), exit2.Y1(), 2);
        right.set(exit2.X2(), exit2.Y2(), exit1.X2(), exit1.Y2(),  4);

        if(TwoObjectIntersect(left, right) == true)
        {
            left.set(exit1.X1(), exit1.Y1(),exit2.X2(), exit2.Y2(), 2);
            right.set(exit2.X1(), exit2.Y1(), exit1.X2(), exit1.Y2(), 4);
        }

        rnt_block.push_back(bottom);
        rnt_block.push_back(left);
        rnt_block.push_back(right);
        rnt_block.push_back(top);

        return rnt_block;
}

vector<Object> Two_Space_Overlap(vector<Object> block1, vector<Object> block2)
{
    Object ref_top_line;
    Object ext_line1, ext_line2;
    Object interpolate1, interpolate2;
    vector<Object> rnt;
    
    Point inter_p1, inter_p2;
    Point ref_d1, ref_d2;
    
    double size = 500;
    
    int cnt = 0;
    
    
    //expend based upon the top line 
    ref_top_line = block1[2];
    
    ref_d1 = outSidePerpendPointWithLength(ref_top_line, 500, ref_top_line.getP1(), 1);
    ref_d2 = outSidePerpendPointWithLength(ref_top_line, 500, ref_top_line.getP2(), 1);
    
    ext_line1.set(ref_top_line.X1(), ref_top_line.Y1(), ref_d1.X(), ref_d1.Y(), 1);
    ext_line2.set(ref_top_line.X2(), ref_top_line.Y2(), ref_d2.X(), ref_d1.Y(), 2);
    
    //find intersection
lb1:if(interSectWithLine(block2, ext_line1.getP1(), ext_line1.getP2()) ==false)
    {
        if(cnt < 6)
        {
            ext_line1 =  expend_Object(ext_line1, 500, 2);
            cnt++;
            goto lb1;
        }
        else
            inter_p1 = block2[3].getP2();
    }
    else
    {
            inter_p1 = intersectedPoint(block2, ext_line1.getP1(), ext_line1.getP2());
    }
    
    cnt = 0;
        //find intersection
lb2:if(interSectWithLine(block2, ext_line2.getP1(), ext_line2.getP2()) == false)
    {
        if(cnt < 6)
        {
            ext_line2 =  expend_Object(ext_line2, 500, 2);
            cnt++;
            goto lb2;
        }
        else
            inter_p2 = block2[3].getP1();
    }
    else
    { 
        inter_p2 = intersectedPoint(block2, ext_line1.getP2(), ext_line2.getP2());
    }
    
    interpolate1.set(ref_top_line.X1(), ref_top_line.Y1(), inter_p1.X(), inter_p1.Y(), 1);
    interpolate2.set(ref_top_line.X2(), ref_top_line.Y2(), inter_p2.X(), inter_p2.Y(), 2);
            
    rnt.push_back(interpolate1);
    rnt.push_back(interpolate2);
            
            
    return rnt;
}


vector<Object> LeftAndRight_boundary(vector<Object> boundary_lines, Object path_segment)
{
    int flag;
    Object left_boundary, right_boundary;
    vector<Object> rnt;
    
    //find left right boundary
    for(int i = 0; i < boundary_lines.size(); i++)
    { 
            if((isLeft(path_segment.getP1(), path_segment.getP2(), boundary_lines[i].getP1()) > 0)
                && (isLeft(path_segment.getP1(), path_segment.getP2(), boundary_lines[i].getP2()) > 0)) //left 
            {
                left_boundary = boundary_lines[i];
            }
            else
            {
                if((isLeft(path_segment.getP1(), path_segment.getP2(), boundary_lines[i].getP1()) < 0)
                    && (isLeft(path_segment.getP1(), path_segment.getP2(), boundary_lines[i].getP2()) < 0)) //right
                {
                    right_boundary = boundary_lines[i];
                }
            }
            
    }
    
    rnt.push_back(left_boundary);
    rnt.push_back(right_boundary);
    
    return rnt;
     
}

vector<Object> simplifyBoundary(vector<Object> path_segments, vector<Object> boundary_space)
{
    vector<Object> rnt;
    
    rnt = boundary_space;
    
lb: for(int i = 0; i < rnt.size(); i++)
    {
        for(int j = 0; j < path_segments.size(); j++)
        {
            if(TwoObjectIntersect(rnt[i], path_segments[j]) == true)
            {
                rnt.erase(rnt.begin()+i);
                i = 0;
                j = 0;
                goto lb;
            }
        }
    }
    
    return rnt;
}

vector<Object> ConstrainSpace_Narrow(vector<Object> region, vector<Object> block,
                           pair< vector<Object>, vector<Object> > LeftAndRight, 
                           Object path_segment, int side_flag)
{
        cout << " Modified boundary block !!" << endl;
        
        Object intersect_Obj;                      //intersected object
        Object interpolate_Obj1, interpolate_Obj2; //interpolated objects
        Object modify_Obj; //modified boundary line
        Object reference_Obj;
        
        vector<Object> region_block;
        vector<Object> boundary_lines;             //block boundary lines
        vector<Object> intersect_surface;          //all intersected surfaces
        vector<Object> ref_side;                   //choose left or right boundary line as ref
        vector<Point> points_from_boundary, other_side; //transform to points
        
        double ref_solpe_left, ref_solpe_right; //slopes

        Point ref_p1, ref_p2;  //modified boundary line using points
        Point intersect_point;
        
        //find left and right boundary
        boundary_lines = LeftAndRight_boundary(block, path_segment);
        region_block = block;
        
        if(side_flag == 1)
            reference_Obj = boundary_lines[0];
        else
            reference_Obj = boundary_lines[1];
        
                
lb:     vector<Surface> polygon = makePolygonOfCV(region_block);
  
        
        //intersected surfaces with the boundary line 
        if(side_flag == 1)
        {
            ref_side = LeftAndRight.first;
            ref_solpe_left = (reference_Obj.Y2() - reference_Obj.Y1()) / (reference_Obj.X2() - reference_Obj.X1());
            
            //intersect_surface = intersectAllSurfaces(ref_side, boundary_lines[0].getP1(), boundary_lines[0].getP2());
            //points_from_boundary = ObjectToPoints(intersect_surface);
            //other_side = ObjectToPoints(LeftAndRight.second);
            
            for(int i = 0; i < ref_side.size(); i++)
            {
                if(TwoObjectIntersect(ref_side[i], reference_Obj) == true)
                {
                    intersect_Obj = ref_side[i];
                    intersect_point = crossedPointReturnP(reference_Obj.getP1(), reference_Obj.getP2(), ref_side[i].getP1(), ref_side[i].getP2());
                    
                    //the first interpolate boudnary
                    interpolate_Obj1.set(reference_Obj.X1(), reference_Obj.Y1(), intersect_point.X(), intersect_point.Y(), 0);     
                    
                    if(pointInPolygon(PointXY(intersect_Obj.X1(), intersect_Obj.Y1()), polygon) == true)
                    {
                        interpolate_Obj2.set(intersect_point.X(), intersect_point.Y(), intersect_Obj.X1(), intersect_Obj.Y1(), -1);
                    }
                    else
                    {
                        if(pointInPolygon(PointXY(intersect_Obj.X2(), intersect_Obj.Y2()), polygon) == true)
                        {
                            interpolate_Obj2.set(intersect_point.X(), intersect_point.Y(), intersect_Obj.X2(), intersect_Obj.Y2(), -1);
                        }
                            
                    }
                    
                    modify_Obj.set(intersect_Obj.X1(), intersect_Obj.Y1(), reference_Obj.X2(), reference_Obj.Y2(), -3);
                    reference_Obj = modify_Obj;
                    
                    for(int j = 0; j < region_block.size(); j++) //replace this reference obj
                    {
                        if(region_block[j].X1() == reference_Obj.X1() && region_block[j].X2() == reference_Obj.X2())
                            region_block.erase(region.begin()+j);
                    }
                    
                    region_block.push_back(interpolate_Obj1);
                    region_block.push_back(interpolate_Obj2);
                    region_block.push_back(modify_Obj);
                    
                    goto lb;
                }
                
                //region block rebuild
                
            }
        }
        else
        {
            ref_side = LeftAndRight.second;
            ref_solpe_right = (reference_Obj.Y2() - reference_Obj.Y1()) / (reference_Obj.X2() - reference_Obj.X1());
            //intersect_surface = intersectAllSurfaces(LeftAndRight.second, boundary_lines[1].getP1(), boundary_lines[1].getP2());
            //points_from_boundary = ObjectToPoints(intersect_surface);
            //other_side = ObjectToPoints(LeftAndRight.first);
            
            for(int i = 0; i < ref_side.size(); i++)
            {
                if(TwoObjectIntersect(ref_side[i], reference_Obj) == true)
                {
                    intersect_Obj = ref_side[i];
                    intersect_point = crossedPointReturnP(reference_Obj.getP1(), reference_Obj.getP2(), ref_side[i].getP1(), ref_side[i].getP2());
                    
                    //the first interpolate boudnary
                    interpolate_Obj1.set(reference_Obj.X1(), reference_Obj.Y1(), intersect_point.X(), intersect_point.Y(), 0);  
                    
                    if(pointInPolygon(PointXY(intersect_Obj.X1(), intersect_Obj.Y1()), polygon) == true)
                    {
                        interpolate_Obj2.set(intersect_point.X(), intersect_point.Y(), intersect_Obj.X1(), intersect_Obj.Y1(), -1);
                    }
                    else
                    {
                        if(pointInPolygon(PointXY(intersect_Obj.X2(), intersect_Obj.Y2()), polygon) == true)
                        {
                            interpolate_Obj2.set(intersect_point.X(), intersect_point.Y(), intersect_Obj.X2(), intersect_Obj.Y2(), -1);
                        }
                            
                    }
                    
                    modify_Obj.set(intersect_Obj.X2(), intersect_Obj.Y2(), reference_Obj.X2(), reference_Obj.Y2(), -3);
                    reference_Obj = modify_Obj;
                    
                    for(int j = 0; j < region_block.size(); j++) //replace this reference obj
                    {
                        if(region_block[j].X1() == reference_Obj.X1() && region_block[j].X2() == reference_Obj.X2())
                            region_block.erase(region.begin()+j);
                    }
                    
                    region_block.push_back(interpolate_Obj1);
                    region_block.push_back(interpolate_Obj2);
                    region_block.push_back(modify_Obj);
                    
                    goto lb;
                } 
                
                
            }
            
        }
                
        return region_block;
}


/* find all intersected surfaces 
 * one left and right sides
 * and return them as pair type
 */
pair< vector<Object>, vector<Object> > boundary_surfaces(pair< vector<Object>, vector<Object> > leftAndright,
                                                         vector<Object> block, Object path_segment)
{   
    vector<Object> boundary_lines;
    vector<Object> intersect_surface_left, intersect_surface_right;
    pair< vector<Object>, vector<Object> > rnt;
    
    //boundary_lines = LeftAndRight_boundary(block, path_segment);
    
    do
    {
        for(int i = 0; i < block.size(); i++)
        {
            if((block[i].getPositionFlag() == 3)
                && (boundary_lines.size() == 0))
                boundary_lines.push_back(block[i]);
            if((block[i].getPositionFlag() == 4) 
                       && (boundary_lines.size() > 0))
                boundary_lines.push_back(block[i]);    
        }
    }while(boundary_lines.size() < 2);
    
    intersect_surface_left = intersectAllSurfaces(leftAndright.first, boundary_lines[0].getP1(), boundary_lines[0].getP2());
    intersect_surface_right = intersectAllSurfaces(leftAndright.second, boundary_lines[1].getP1(), boundary_lines[1].getP2());
    
    
    rnt.first = intersect_surface_left;
    rnt.second = intersect_surface_right;
    
    cout << " Size of left  list of surfaces : " << rnt.first.size() << endl;
    cout << " Size of right list of surfaces : " << rnt.second.size() << endl;
    return rnt;
}

vector<Exit> Exits_in_region(pair< vector<Object>, vector<Object> > leftAndRight,
                             vector<Object> region_block, Object path_segment)
{
    vector<Point> left_list, right_list;
    
    Exit temp_exit;
    vector<Exit> exits;
    vector<Object> boundary_lines;
    Point perpend_point;
    ClipperLib::Path polygon;
    ClipperLib::IntPoint Poi; 
    
    //boundary_lines = LeftAndRight_boundary(region_block, path_segment);
    
    do
    {
        for(int i = 0; i < region_block.size(); i++)
        {
            if((region_block[i].getPositionFlag() == 3)
                && (boundary_lines.size() == 0))
                boundary_lines.push_back(region_block[i]);
            if((region_block[i].getPositionFlag() == 4) 
                       && (boundary_lines.size() > 0))
                boundary_lines.push_back(region_block[i]);    
        }
    }while(boundary_lines.size() < 2);
    
            
    
    //vector<Surface> polygon = makePolygonOfCV(region_block);
    polygon = viewConvertPath(region_block);
    
    left_list = ObjectToPoints(leftAndRight.first);
    right_list = ObjectToPoints(leftAndRight.second);
    
    int left_cnt = 0, right_cnt = 0;
    
    
    //left list
    for(int i = 0; i < left_list.size(); i++)
    {
        Poi.X = left_list[i].X();
        Poi.Y = left_list[i].Y();
        
        //if(pointInPolygon(PointXY(left_list[i].X(), left_list[i].Y()), polygon) == true)
        if(PointInPolygon (Poi, polygon) == 1)
        {   left_cnt++;
            perpend_point = crossPerpend(boundary_lines[1].getP1(), boundary_lines[1].getP2(), 
                                         left_list[i]);
            
            if(interSectWithLine(leftAndRight.first, left_list[i], perpend_point) == false)
            {
                temp_exit.set(left_list[i].X(), left_list[i].Y(), perpend_point.X(), perpend_point.Y());
                exits.push_back(temp_exit);
            }
        }
    }
    
    //right list
    for(int i = 0; i < right_list.size(); i++)
    {
        Poi.X = right_list[i].X();
        Poi.Y = right_list[i].Y();
        //if(pointInPolygon(PointXY(right_list[i].X(), right_list[i].Y()), polygon) == true)
        if(PointInPolygon (Poi, polygon) == 1)
        {   right_cnt++;
            perpend_point = crossPerpend(boundary_lines[0].getP1(), boundary_lines[0].getP2(), 
                                         right_list[i]);
            
            if(interSectWithLine(leftAndRight.second, right_list[i], perpend_point) == false)
            {
                temp_exit.set(right_list[i].X(), right_list[i].Y(), perpend_point.X(), perpend_point.Y());
                exits.push_back(temp_exit);
            }
        }
    }
    
    cout << " left list counter : " << left_cnt << endl;
    cout << " right list counter : " << right_cnt << endl;
    return exits;
}


/* compute region block based on imagine exit/line 
 * find narrow space and depict the block 
 * 
 * output: two boundary lines
 */
vector<Object> Boundary_Lines(vector<Object> region, Exit imagine_entry, Exit imagine_exit)
{

    double length = 1000;
    int side_flag = 0;
    
    vector<Object> area;
    Object conver_to_object1, conver_to_object2;
    Object boundary_line1, boundary_line2;
    Object top, bottom, left, right;
    Point boundary_point1, boundary_point2;
    
    if(isLeft(imagine_exit.getP1(), imagine_exit.getP2(), imagine_entry.getP1()) > 0)//direction
        side_flag = 1;
    else
        side_flag = 2;
    
    //conver_to_object1.set(imagine_entry.X1(), imagine_entry.Y1(), imagine_entry.X1(), imagine_entry.Y1(), 0);
    conver_to_object2.set(imagine_exit.X1(), imagine_exit.Y1(), imagine_exit.X2(), imagine_exit.Y2(), 0);
    
    boundary_point1 = outSidePerpendPointWithLength(conver_to_object2, length, conver_to_object2.getP1(), side_flag);
    boundary_point2 = outSidePerpendPointWithLength(conver_to_object2, length, conver_to_object2.getP2(), side_flag);
    
    cout << " boundary point 1 x: " << boundary_point1.X() << " ; y: " << boundary_point1.Y() << endl;
    cout << " boundary point 2 x: " << boundary_point2.X() << " ; y: " << boundary_point2.Y() << endl;
    waitHere();
    
    boundary_line1.set(imagine_exit.X1(), imagine_exit.Y1(), boundary_point1.X(), boundary_point1.Y(), 1);
    boundary_line2.set(imagine_exit.X2(), imagine_exit.Y2(), boundary_point2.X(), boundary_point2.Y(), 2);
    
    
    //top = imagine_exit;
    left = boundary_line1;
    right = boundary_line2;
    
    //bottom.set(boundary_point2.X(), boundary_point2.Y(), boundary_point1.X(), boundary_point1.Y(), 3);
    
    //top.setPositionFlag(1);
    //bottom.setPositionFlag(2);
    left.setPositionFlag(3);
    right.setPositionFlag(4);
    
    //area.push_back(top);
    //area.push_back(bottom);
    area.push_back(left);
    area.push_back(right);
    
    
    return area;
    
}

/* filter left and right lists of surfaces
 * collect those are between two lines
 * both intersected and inside
 */
vector<Object> Select_surfaces(vector<Object> region, vector<Object> boundary_lines)
{
    double ref_dist;
    double potent_dist1, potent_dist2; 
    double potent_dist3, potent_dist4; 
    Point p1, p2, p3, p4;
    
    vector<Object> rnt;
    
    //ref_dist = boundary_lines[0].shortestDistP1ToP1(boundary_lines[1]);
    ref_dist = boundary_lines[0].distP1ToP1(boundary_lines[1]);
    
    cout << " reference distance : " << ref_dist << endl;
    
    for(int i = 0; i < region.size(); i++)
    {
        /*
        potent_dist1 = boundary_lines[0].perpendicularDistOfPoint(region[i].getP1());
        potent_dist2 = boundary_lines[1].perpendicularDistOfPoint(region[i].getP1());
        potent_dist3 = boundary_lines[0].perpendicularDistOfPoint(region[i].getP2());
        potent_dist4 = boundary_lines[1].perpendicularDistOfPoint(region[i].getP2());
        */
        
        p1 = crossPerpend(boundary_lines[0].getP1(), boundary_lines[0].getP2(),  //p1 to left
                                         region[i].getP1());
        p2 = crossPerpend(boundary_lines[0].getP1(), boundary_lines[0].getP2(),  //p2 to left
                                         region[i].getP2());
        p3 = crossPerpend(boundary_lines[1].getP1(), boundary_lines[1].getP2(),  //p1 to right
                                         region[i].getP1());
        p4 = crossPerpend(boundary_lines[1].getP1(), boundary_lines[1].getP2(),  //p2 to right
                                         region[i].getP2());
        
        potent_dist1 = distanceOftwoP(region[i].getP1(), p1);
        potent_dist2 = distanceOftwoP(region[i].getP2(), p2);
        potent_dist3 = distanceOftwoP(region[i].getP1(), p3);
        potent_dist4 = distanceOftwoP(region[i].getP2(), p4);
        
        /*
        cout << " potent_dist1 : " << potent_dist1 << endl;
        cout << " potent_dist2 : " << potent_dist2 << endl;
        cout << " potent_dist3 : " << potent_dist3 << endl;
        cout << " potent_dist4 : " << potent_dist4 << endl;
        */
        
        if((potent_dist1 < ref_dist && potent_dist3 < ref_dist)
            || (potent_dist2 < ref_dist && potent_dist4 < ref_dist))
        {
            rnt.push_back(region[i]);
        }
    }
    
    return rnt;
}

Object Constrain_region(vector<Object> region, Object path_segment, Exit imagine_eixt)
{
    Point near_point, cross_point;
    double distance, min;
    
    Object constrain_line;
    vector<Point> points;
    
    points = ObjectToPoints(region);
    
    for(int i = 0; i < points.size(); i++)
    {
        if(i == 0)
        {
            min = distance = P_To_ShortestDistance(points[i], path_segment);
            near_point = points[i];
        }
        else
        {
            distance = P_To_ShortestDistance(points[i], path_segment);
            
            if(distance < min)
            {
                min = distance;
                near_point = points[i];
            }
        }
    }
    
    cross_point = crossPerpend(imagine_eixt.getP1(), imagine_eixt.getP2(), near_point);
    
    constrain_line.set(cross_point.X(), cross_point.Y(), near_point.X(), near_point.Y(), 1);
            
    return constrain_line;
    
}


/* check if two blocks are intersected (vector<Object>)
 * input: two vector<Object>
 * return bool and 
 */
bool blocksIntersection(vector<Object> region_block1, vector<Object> region_block2)
{
    for(int i = 0; i < region_block1.size(); i++)
        if(interSectWithLine(region_block2, region_block1[i].getP1(), region_block1[i].getP2()) == true)
            return true;
    
    return false;
}


/* identify the position of previous 
 * there are four types of positions
 * with respect to current block.
 * 1 -- inside, 2 -- left, 3 -- right
 * 4 -- bottom
 */
unsigned int modified_blocks_withConnection(vector<Object> region_block, Exit previous_exit)
{
        unsigned int position_flag = 0;
    
        ClipperLib::Path polygon;
        ClipperLib::IntPoint Poi_1, Poi_2; 
        
        polygon = viewConvertPath(region_block);
    
        Poi_1.X = previous_exit.X1();
        Poi_1.Y = previous_exit.Y1();
        
        Poi_2.X = previous_exit.X2();
        Poi_2.Y = previous_exit.Y2();
        
        if((PointInPolygon (Poi_1, polygon) == 1)
            ||(PointInPolygon (Poi_2, polygon) == 1))
        {
            position_flag = 1; //inside block at least one endpoint
            
        }
        else
        {
            if((isLeft(region_block[3].getP1(), region_block[3].getP2(), previous_exit.getP1()) < 0)
                &&(isLeft(region_block[3].getP1(), region_block[3].getP2(), previous_exit.getP2()) < 0))
                position_flag = 2;
            if((isLeft(region_block[4].getP1(), region_block[4].getP2(), previous_exit.getP1()) > 0)
                &&(isLeft(region_block[4].getP1(), region_block[4].getP2(), previous_exit.getP2()) > 0))
                position_flag = 3;
            //if((isLeft(region_block[2].getP1(), region_block[2].getP2(), previous_exit.getP1()) > 0)
            //    ||(isLeft(region_block[2].getP1(), region_block[2].getP2(), previous_exit.getP2()) < 0))
            //    position_flag = 4;
            
        }
        
    
        return position_flag;
}

/* input: two exits
 * output: a block of region
 * simply connect these two exits
 * to compute a block
 */
vector<Object> block_based_twoExits(Exit exit1, Exit exit2)
{
    Object bottom, left, right, top;
    vector<Object> rnt;
    
    bottom.set(exit1.X1(), exit1.Y1(), exit1.X2(), exit1.Y2(), 1);
    left.set(exit1.X1(), exit1.Y1(), exit2.X1(), exit2.Y1(), 2);
    top.set(exit2.X1(), exit2.Y1(), exit2.X2(), exit2.Y2(), 3);
    right.set(exit2.X2(), exit2.Y2(), exit1.X2(), exit1.Y2(), 4);
    
    if(TwoObjectIntersect(left, right) == true)
    {
        left.set(exit1.X1(), exit1.Y1(), exit2.X2(), exit2.Y2(), 2);
        right.set(exit2.X1(), exit2.Y1(), exit1.X2(), exit1.Y2(), 4);
    }
    
    rnt.push_back(bottom);
    rnt.push_back(left);
    rnt.push_back(top);
    rnt.push_back(right);

    return rnt;
}

/*
vector<Object> perpend_lines(Object ref_line, Exit imagine_line)
{
    Point boundary_point1, boundary_point2
    int side_flag = 0;
    
    if(isLeft(ref_line.getP1(), ref_line.getP2(), imagine_line.getP1()) > 0)//direction
        side_flag = 1;
    else
        side_flag = 2;

    
    boundary_point1 = outSidePerpendPointWithLength(ref_line, length, ref_line.getP1(), side_flag);
    boundary_point2 = outSidePerpendPointWithLength(ref_line, length, ref_line.getP2(), side_flag);
}
*/

/*******************************************************************************/
/* compute a path in a region & based upon circles
 * input : a list of circles
 * output : path segment in corresponding region
 *          (which is not depicted by using one line) two lines
 */
vector<Object> pathInRegion_UseCircles(vector<Ellipse> space_circle, vector<Object> region,
                                       Exit imagin_entry, Exit imagin_exit, Object path_segment)
{
    cout << " Path Space is computing " << endl;
        char fileName[80];
        double min;
        double k1, k2, b1, b2; //side lines slope and phase
        double K_entry, K_imagin, B_entry, B_imagin; //imaginary line slope and phase
        double x, y;
        int mark;
        int flag = 0;
        Ellipse temp_circle;
        Object ref_line, ref_paralllel, left, right, l1, l2, r1, r2;
        Point p1, p2, p3, p4;
        vector<Object> path_sapce;

        //get the smallest circle
        for(int i = 0; i < space_circle.size(); i++)
        {
                if(i == 0)
                {
                        min = space_circle[i].getwidth();
                        temp_circle = space_circle[i];
                        mark = i;
                }
                else
                {
                        if(min > space_circle[i].getwidth())
                        {
                            min = space_circle[i].getwidth();
                            temp_circle = space_circle[i];
                            mark = i;
                        }
                }
        }

        ref_line = temp_circle.getHorizonline(); //reference line
        //ref_paralllel.set(space_circle[0].getCentre().X(), space_circle[0].getCentre().Y(), space_circle.back().getCentre().X(), space_circle.back().getCentre().Y(), 1);
        ref_paralllel = path_segment;
        
        K_entry = (imagin_entry.Y2() - imagin_entry.Y1()) / (imagin_entry.X2() - imagin_entry.X1());
        K_imagin = (imagin_exit.Y2() - imagin_exit.Y1()) / (imagin_exit.X2() - imagin_exit.X1());
        B_entry = imagin_entry.Y1() - K_entry * imagin_entry.X1();
        B_imagin = imagin_exit.Y1() - K_imagin * imagin_exit.X1();
        
        // b = y - kx
        //x = (Bside - Bim) / (Kim-Kside)
        k1 = k2 = (ref_paralllel.Y2() - ref_paralllel.Y1()) / (ref_paralllel.X2() - ref_paralllel.X1());       
        b1 = temp_circle.fourEndPoints()[0].Y() - k1 * temp_circle.fourEndPoints()[0].X(); //left side line equation
        b2 = temp_circle.fourEndPoints()[1].Y() - k2 * temp_circle.fourEndPoints()[1].X(); //right side line equation
        
        //compute four points p1 p2 p3 p4
        
        
        
        //compute two sides of path space
        if(mark == 0)
        {
                //p1, p3 are given
                p1 = ref_line.getP1();
                p3 = ref_line.getP2();

                //compute other two points on exit side
                //p2 = space_circle.back().fourEndPoints()[0];
                //p4 = space_circle.back().fourEndPoints()[1];
                x = (b1 - B_imagin) / (K_imagin - k1);
                y = k1 * x + b1;
                p2.set(x, y);
                
                x = (b2 - B_imagin) / (K_imagin - k2);
                y = k2 * x + b2;
                p4.set(x, y);
                
        }
        else
        {
                if(mark == space_circle.size()-1)
                {
                        //p2, p4 are given
                        p2 = ref_line.getP1();
                        p4 = ref_line.getP2();

                        //compute other two points on entry side
                        //p1 = space_circle[0].fourEndPoints()[0];
                        //p3 = space_circle[0].fourEndPoints()[1];
                        
                        x = (b1 - B_entry) / (K_entry - k1);
                        y = k1 * x + b1;
                        p1.set(x, y);

                        x = (b2 - B_entry) / (K_entry - k2);
                        y = k2 * x + b2;
                        p3.set(x, y);
                }
                else
                {
                        //compute four points p1, p2, p3, p4
                        //compute 4 points on both entry and exit sides
                        //p1 = space_circle[0].fourEndPoints()[0];
                        //p2 = space_circle.back().fourEndPoints()[0];
                        //p3 = space_circle[0].fourEndPoints()[1];
                        //p4 = space_circle.back().fourEndPoints()[1];
                        x = (b1 - B_entry) / (K_entry - k1);
                        y = k1 * x + b1;
                        p1.set(x, y);

                        x = (b2 - B_entry) / (K_entry - k2);
                        y = k2 * x + b2;
                        p3.set(x, y);
                        
                        x = (b1 - B_imagin) / (K_imagin - k1);
                        y = k1 * x + b1;
                        p2.set(x, y);

                        x = (b2 - B_imagin) / (K_imagin - k2);
                        y = k2 * x + b2;
                        p4.set(x, y);
                        
                        flag = 1;
                }
        }

        
        //if(flag != 1)
        //{
            left.set(p1.X(), p1.Y(), p2.X(), p2.Y(), 1);  //left side
            right.set(p3.X(), p3.Y(), p4.X(), p4.Y(), 2); //right side

            path_sapce.push_back(left);
            path_sapce.push_back(right);
        /*}
        else
        {
            l1.set(p1.X(), p1.Y(), ref_line.X1(), ref_line.Y1(), 1);  //left side
            l2.set(ref_line.X1(), ref_line.Y1(), p2.X(), p2.Y(), 2);  //left side
            r1.set(p3.X(), p3.Y(), ref_line.X2(), ref_line.Y2(), 3); //right side
            r2.set(ref_line.X2(), ref_line.Y2(), p4.X(), p4.Y(), 4); //right side

            path_sapce.push_back(l1);
            path_sapce.push_back(l2);
            path_sapce.push_back(r1);
            path_sapce.push_back(r2);
        }
        */
        
        sprintf(fileName, "%s", "Maps/Offline/PathSpace_based_on Circles.png");
        plotObjectsOf3KindswithEllipse(fileName, region, path_sapce, space_circle);

        //waitHere();
        return path_sapce;
    
}

/* 
 *
 */
vector<Object> structureRegion(vector<Object> region, vector<Object> path_space)
{
        char fileName[80];
        vector<Object> rnt;
        
        double threshold = 1000;
        double dist1, dist2, dist3, dist4;

        for(int i = 0; i < region.size(); i++)
        {
            dist1 = P_To_ShortestDistance(region[i].getP1(), path_space[0]);
            dist2 = P_To_ShortestDistance(region[i].getP1(), path_space[1]);
            dist3 = P_To_ShortestDistance(region[i].getP2(), path_space[0]);
            dist4 = P_To_ShortestDistance(region[i].getP2(), path_space[1]);
            /*
            if((TwoObjectIntersect(region[i], path_space[0]) == true)
                || (TwoObjectIntersect(region[i], path_space[1]) == true)
                || (dist1 < threshold) || (dist2 < threshold)
                || (dist3 < threshold) || (dist4 < threshold))*/
            if((TwoObjectIntersect(region[i], path_space[0]) == true)
                || (TwoObjectIntersect(region[i], path_space[1]) == true)
                || (dist1 < threshold) && (dist3 < threshold)
                || (dist2 < threshold) && (dist4 < threshold))    
            {
                    rnt.push_back(region[i]);
            }
                
           
        }
        
        sprintf(fileName, "%s", "Maps/Offline/PathSpace_And_Surrounding.png");
        plotObjectsOf3Kinds(fileName, region, rnt, path_space);
        
        return rnt;
}

/* 
 * 
 */
vector<Object> RegionBaseLines(vector<Object> filtered_region, vector<Object> path_space,
                               Object path_segment)
{
        char fileName[80];
        Object trend_depict1, trend_depict2;
        vector<Object> side_list1, side_list2;
        vector<Object> rnt;
        pair<double, double> trendline1, trendline2;
        
        double x, y;
        Point p1, p2, p3, p4;

        //cluster information as left and right two group
        for(int i = 0; i < filtered_region.size(); i++)
        {
                if((isLeft(path_segment.getP1(), path_segment.getP2(), filtered_region[i].getP1()) > 0)
                    && (isLeft(path_segment.getP1(), path_segment.getP2(), filtered_region[i].getP2()) > 0))
                {
                    //left list/ side list 1
                    side_list1.push_back(filtered_region[i]);
                }

                if((isLeft(path_segment.getP1(), path_segment.getP2(), filtered_region[i].getP1()) < 0)
                    && (isLeft(path_segment.getP1(), path_segment.getP2(), filtered_region[i].getP2()) < 0))
                {
                    //right list/ side list 2
                    side_list2.push_back(filtered_region[i]);
                }
        }
        
        /*
        //compute the trendline of boundary of environment
        trendline1 = Trendline_parameter(side_list1);
        trendline2 = Trendline_parameter(side_list2);
        
        
        x = side_list1[0].X1();
        y = trendline1.first * x + trendline1.second;
        p1.set(x, y);

        x = side_list2[0].X1();
        y = trendline2.first * x + trendline2.second;
        p3.set(x, y);

        x = side_list1.back().X1();
        y = trendline1.first * x + trendline1.second;
        p2.set(x, y);

        x = side_list2.back().X1();
        y = trendline2.first * x + trendline2.second;
        p4.set(x, y);
        
        
        //construct two based boundary line
        trend_depict1.set(p1.X(), p1.Y(), p2.X(), p2.Y(), 1);  //left side
        trend_depict2.set(p3.X(), p3.Y(), p4.X(), p4.Y(), 2); //right side

        rnt.push_back(trend_depict1);
        rnt.push_back(trend_depict2);
        */
        
        vector<Object> bline1 = BoundaryByGroup(side_list1);
        vector<Object> bline2 = BoundaryByGroup(side_list2);
        rnt = addTwoVectorsOfObjects(rnt, bline1);
        rnt = addTwoVectorsOfObjects(rnt, bline2);
                
        sprintf(fileName, "%s", "Maps/Offline/Trendline_And_Surrounding.png");
        plotObjectsOf3Kinds(fileName, filtered_region, rnt, path_space);
        
        return rnt;
}

/* base line that
 *
 */
vector<Object> structure_boundary(vector<Object> region, vector<Object> path_space, 
                                  Exit imagine_exit, Object path)
{
    char fileName[80];
    int flag = 0; // 1-- left  2 -- right
    double slope_path;
    
    Object ref_bound, compute_bound;
    vector<Object> left_list, right_list;
    vector<Object> filter_region, left_join, right_join;
    vector<Object> left_gaps, right_gaps;
    vector<Object> left_struct, right_struct;
    
    //slope of path to determine which side will be the ref
    slope_path = (path_space[0].Y2() - path_space[0].Y1()) / (path_space[0].X2() - path_space[0].X1());
       
    if(slope_path > 0)
        flag = 2;
    else
        if(slope_path < 0)
            flag = 1;
    /*
    for(int i = 0; i < region.size(); i++)
    {
        if(((isLeft(path_space[0].getP1(), path_space[0].getP2(), region[i].getP1()) > 0)
                    && (isLeft(path_space[0].getP1(), path_space[0].getP2(), region[i].getP2()) > 0))
            ||(TwoObjectIntersect(region[i], path_space[0]) == true))
        {
            left_list.push_back(region[i]);
        }
        else
        {
            if(((isLeft(path_space[0].getP1(), path_space[0].getP2(), region[i].getP1()) < 0)
                    && (isLeft(path_space[0].getP1(), path_space[0].getP2(), region[i].getP2()) < 0))
                ||(TwoObjectIntersect(region[i], path_space[1]) == true))
            {
                right_list.push_back(region[i]);
            }
        }
    }
    */
    
    //imaginary line split information
    for(int i = 0; i < region.size(); i++)
    {
        
        if((isLeft(imagine_exit.getP1(), imagine_exit.getP2(), region[i].getP1()) > 0)
                && (isLeft(imagine_exit.getP1(), imagine_exit.getP2(), region[i].getP2()) > 0)
                && (isLeft(imagine_exit.getP1(), imagine_exit.getP2(), path.getP1()) > 0))
            filter_region.push_back(region[i]);
        
        if((isLeft(imagine_exit.getP1(), imagine_exit.getP2(), region[i].getP1()) < 0)
                && (isLeft(imagine_exit.getP1(), imagine_exit.getP2(), region[i].getP2()) < 0)
                && (isLeft(imagine_exit.getP1(), imagine_exit.getP2(), path.getP1()) < 0))
            filter_region.push_back(region[i]);
    }
    /*
    for(int i = 0; i < filter_region.size(); i++)
    {
        if((isLeft(path.getP1(), path.getP2(), filter_region[i].getP1()) > 0)
                    && (isLeft(path.getP1(), path.getP2(), filter_region[i].getP2()) > 0))
        {
            left_list.push_back(filter_region[i]);
        }
        else
        {
            if((isLeft(path.getP1(), path.getP2(), filter_region[i].getP1()) < 0)
                    && (isLeft(path.getP1(), path.getP2(), filter_region[i].getP2()) < 0))
            {
                right_list.push_back(filter_region[i]);
            }
        }
    }
    */
    for(int i = 0; i < region.size(); i++)
    {
        if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) > 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) > 0))
        {
            left_list.push_back(region[i]);
        }
        else
        {
            if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) < 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) < 0))
            {
                right_list.push_back(region[i]);
            }
        }
    }
    
    left_join = connectSurfaces(left_list);
    right_join = connectSurfaces(right_list);
    
    //surround area
    //compute a circle -- radius 1m
    Point surround_centre;
    double size_surround = 1000;
    surround_centre.set((path_space[0].X2()+path_space[1].X2())/2, (path_space[0].Y2()+path_space[1].Y2())/2);
    
lb: for(int i = 0; i < left_list.size(); i++)
    {
        if((distanceOftwoP(left_list[i].getP1() ,surround_centre) < size_surround)
            || (distanceOftwoP(left_list[i].getP2() ,surround_centre) < size_surround))
        {
            left_gaps.push_back(left_list[i]);
        }
    }
    
    for(int i = 0; i < right_list.size(); i++)
    {
        if((distanceOftwoP(right_list[i].getP1() ,surround_centre) < size_surround)
            || (distanceOftwoP(right_list[i].getP2() ,surround_centre) < size_surround))
        {
            right_gaps.push_back(right_list[i]);
        }
    }
    
    if(left_gaps.size() == 0 || right_gaps.size() == 0)
    {
        size_surround += 200;
        goto lb;
    }

    left_struct =  collectSurfaces(left_list, left_gaps, 1);
    right_struct = collectSurfaces(right_list, right_gaps, 2);
    
    sprintf(fileName, "%s", "Maps/Offline/PathSpace_And_Surrounding.png");
    //plotObjectsOf3Kinds(fileName, left_list, right_list, path_space);
    plotObjectsOf3Kinds(fileName, left_join, right_join, path_space);
    
    sprintf(fileName, "%s", "Maps/Offline/PathSpace_And_GapSurround.png");
    //plotObjectsOf3Kinds(fileName, left_list, right_list, path_space);
    plotObjectsOf3Kinds(fileName, left_struct, right_struct, path_space);

    //get a ref point to compute another boundary
    
    
}

vector<Object> connectSurfaces(vector<Object> surfaces)
{
    Object insert;
    vector<Object> connect;
    
    connect = surfaces;
    if(surfaces.size() > 1)
    {
        for(int i = 0; i < surfaces.size()-1; i++)
        {
            insert.set(surfaces[i].X2(), surfaces[i].Y2(), surfaces[i+1].X1(), surfaces[i+1].Y1(), i);
            connect.push_back(insert);
        }
    }
        
    
    return connect;
}


vector<Object> collectSurfaces(vector<Object> list, vector<Object> ref_Obj, int flag)
{
    vector<Object> rnt;
    int cnt_flag = 0;
    
    if(flag == 1)
    {
        for(int i = 0; i < list.size(); i++)
        {
            if(list[i].X1() != ref_Obj.back().X1() && list[i].X2() != ref_Obj.back().X2())
                rnt.push_back(list[i]);
            else
                break;
        }
    }
    else
    {
        for(int i = 0; i < list.size(); i++)
        {
            if(list[i].X1() == ref_Obj[0].X1() && list[i].X2() == ref_Obj[0].X2())
                cnt_flag = 1;
            
            if(cnt_flag == 1)
                rnt.push_back(list[i]);
        }
    }
    
    return rnt;
}

/* From start surface to imaginary exit, connect surface clockwise
 * input: region MFIS and path space
 * output: polygon
 */
vector<Object> Struct_Connect(vector<Object> region, vector<Object> path_space)
{
    char fileName[80];
    int cnt = 0;
    int endpoint_flag = 0, ref_tag = 0;
    double dist_p1, dist_p2, min;
    Object image_exit;
    Object temp_obj, start, reference;
    vector<Object> region_list; //modified list & return
    vector<Object> insert_list;
    
    region_list = region;
    
    image_exit.set(path_space[0].X2(), path_space[0].Y2(), path_space[1].X2(), path_space[1].Y2(), 1);
    start = region_list[0];
    
    //while(cnt < region_list.size())
    //{
        //find the closest surface
lb:     for(int i = cnt; i < region_list.size(); i++)
        {
                if(start.X1() != region_list[i].X1() && start.X2() != region_list[i].X2())
                {
                        dist_p1 = distanceOftwoP(start.getP2(), region_list[i].getP1());
                        dist_p2 = distanceOftwoP(start.getP2(), region_list[i].getP2());

                        if(i == 0)
                        {
                                if(dist_p1 < dist_p2)
                                {
                                    min = dist_p1;
                                    endpoint_flag = 1;

                                }
                                else
                                {
                                    min = dist_p2;
                                    endpoint_flag = 2;
                                }

                                ref_tag = i;
                                reference = region_list[i];
                        }
                        else
                        {
                                if(dist_p1 < min)
                                {
                                        min = dist_p1;
                                        endpoint_flag = 1;
                                        reference = region_list[i];
                                        ref_tag = i;
                                }

                                if(dist_p2 < min)
                                {
                                        min = dist_p2;
                                        endpoint_flag = 2;
                                        reference = region_list[i];
                                        ref_tag = i;
                                }
                        }
                }
                else
                    if(cnt > 40)
                        ref_tag = region_list.size();
            
        }
        

        //set the insert surface
        //if(endpoint_flag == 1)
            temp_obj.set(start.X2(), start.Y2(), reference.X1(), reference.Y1(), 1);
        //else
        //    if(endpoint_flag == 2)
        //        temp_obj.set(start.X2(), start.Y2(), reference.X2(), reference.Y2(), 2);
        
        insert_list.push_back(temp_obj);
        
        cout << "nexti step cnt is : " << ref_tag << endl;
        
        cnt = ref_tag;
        start = reference;

        
        if(cnt < region_list.size())
            goto lb;
    
    region_list = addTwoVectorsOfObjects(region_list, insert_list);
    
    
    sprintf(fileName, "%s", "Maps/Offline/PathSpace_Geom_Surrounding.png");
    plotObjectsOf3Kinds(fileName, region_list, insert_list,  path_space);
    
    return region_list;
}

/* convert left and right surface into points
 * 
 *
 */
vector<Object> Struct_Connect(vector<Object> region, vector<Object> path_space, Object path,
                                Exit previous_exit, Exit imagin_exit, int region_num)
{
    char fileName[80];
    int no_exit_flag = 0;
    int left_s, left_e, right_s, right_e;
    double dist_entry, dist_out, min_entry, min_out;
    
    Object temp_obj, exit, entrance;
    Point temp_point;
    Point left_entry, left_out, right_entry, right_out;
    
    Exit compute_exit;
    
    vector<Object> env;
    vector<Object> left_list, right_list, left_side, right_side;
    vector<Point> left_points, right_points, left_potential, right_potential;
    
    //entrance and exit info
    entrance.set(path_space[0].X1(), path_space[0].Y1(), path_space[1].X1(), path_space[1].Y1(), 2);
    //entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
    exit.set(path_space[0].X2(), path_space[0].Y2(), path_space[1].X2(), path_space[1].Y2(), 2);
    
    //classify left and right surfaces based upon path
    //cout << "mfis of region size : " << region.size() << endl;
    for(int i = 0; i < region.size(); i++)
    {
        if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) > 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) > 0))
        {
            left_list.push_back(region[i]);
        }
        else
        {
            if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) < 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) < 0))
            {
                right_list.push_back(region[i]);
            }
        }
    }
    
    //convert to point
    left_points = ObjectToPoints(left_list);
    right_points = ObjectToPoints(right_list);
    
    //cout << " left side points : " << left_points.size() << endl;
    //cout << " right side points : " << right_points.size() << endl;
    
    //left & right potential list of points
    for(int i = 0; i < left_points.size(); i++)
    {
        temp_point = crossPerpend(path.getP1(), path.getP2(), left_points[i]);
        temp_obj.set(left_points[i].X(), left_points[i].Y(), temp_point.X(), temp_point.Y(), 1);
        
        if(interSectWithLine(region, temp_obj.getP1(), temp_obj.getP2()) == false)
            left_potential.push_back(left_points[i]);
    }
    
    for(int i = 0; i < right_points.size(); i++)
    {
        temp_point = crossPerpend(path.getP1(), path.getP2(), right_points[i]);
        temp_obj.set(right_points[i].X(), right_points[i].Y(), temp_point.X(), temp_point.Y(), 2);
        
        if(interSectWithLine(region, temp_obj.getP1(), temp_obj.getP2()) == false)
            right_potential.push_back(right_points[i]);
    }
    //cout << "left potential : " << left_potential.size() << endl;
    //cout << "right potential : " << right_potential.size() << endl;
    //sprintf(fileName, "%s", "Maps/Offline/Before_compute_exit.png");
    //plotObjectsOf3Kinds(fileName, left_list, right_list, region);
    
    if(left_list.size() != 0 && right_list.size() != 0)
    {
        //compute exit
        compute_exit = exitFromRegion(left_list, right_list, path, imagin_exit);
        exit.set(compute_exit.X1(), compute_exit.Y1(), compute_exit.X2(), compute_exit.Y2(), 2);
    }
    else
    {
        //there is no exit in current region
        //compute current region and next region
        //entrance of current region and exit of next region to compute the block
        
        no_exit_flag = 1;
        entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
    }
    
    
    //find points near to entrance and exit
    for(int i = 0; i < left_potential.size(); i++)
    {
            dist_entry = distanceOftwoP(left_potential[i], entrance.getP1());
            dist_out = distanceOftwoP(left_potential[i], exit.getP1());

            if(i == 0)
            {
                    min_entry = dist_entry;
                    min_out = dist_out;

                    left_entry = left_out = left_potential[i];
                    left_e = left_s = i;
            }
            else
            {
                    if(dist_entry < min_entry)
                    {
                            min_entry = dist_entry;
                            left_entry = left_potential[i];
                            left_s = i;
                    }

                    if(dist_out < min_out)
                    {
                            min_out = dist_out;
                            left_out = left_potential[i];
                            left_e = i;
                    }
            }
            
    }

    for(int i = 0; i < right_potential.size(); i++)
    {
            dist_entry = distanceOftwoP(right_potential[i], entrance.getP2());
            dist_out = distanceOftwoP(right_potential[i], exit.getP2());

            if(i == 0)
            {
                    min_entry = dist_entry;
                    min_out = dist_out;

                    right_entry = right_out = right_potential[i];
                    right_e = right_s = i;
            }
            else
            {
                    if(dist_entry < min_entry)
                    {
                            min_entry = dist_entry;
                            right_entry = right_potential[i];
                            right_s  = i;
                    }

                    if(dist_out < min_out)
                    {
                            min_out = dist_out;
                            right_out = right_potential[i];
                            right_e = i;
                    }
            }   
    }
            
    
    for(int i = 0; i < left_potential.size() - 1; i++)
    {
        if(i >= left_s && i < left_e)
        {
            temp_obj.set(left_potential[i].X(), left_potential[i].Y(), left_potential[i+1].X(), left_potential[i+1].Y(),1);
            left_side.push_back(temp_obj);
        }
    }
    
    /*for(int i = 0 ; i < left_side.size(); i++)
    { 
        vector<Object> test;
            test.push_back(left_side[i]);
            //test.push_back(left_side[i+1]);
            sprintf(fileName, "%s", "Maps/Offline/Test_modification_function.png");
            plotObjects(fileName, left_side, test);
            test.clear();
            waitHere();
    }*/
    
    for(int i = 0; i < right_potential.size() - 1; i++)
    {
        if(i <= right_s && i >= right_e)
        {
            temp_obj.set(right_potential[i].X(), right_potential[i].Y(), right_potential[i+1].X(), right_potential[i+1].Y(),2);
            right_side.push_back(temp_obj);
        }
    }
    
    
    if(left_side.size() > 1)
        left_side = removeAcuteAngle(left_side, path);
    
    if(right_side.size() > 1)
        right_side = removeAcuteAngle(right_side, path);
    
    
    
/*    
    //join all lines
    for(int i = 0; i < left_potential.size() - 1; i++)
    {
        temp_obj.set(left_potential[i].X(), left_potential[i].Y(), left_potential[i+1].X(), left_potential[i+1].Y(),1);
        left_side.push_back(temp_obj);
    }
    
    for(int i = 0; i < right_potential.size() - 1; i++)
    {
        temp_obj.set(right_potential[i].X(), right_potential[i].Y(), right_potential[i+1].X(), right_potential[i+1].Y(),2);
        right_side.push_back(temp_obj);
    }
    
    //temp_obj.set(left_side.back().X2(), left_side.back().Y2(), right_side.back().X2(), right_side.back().Y2(), 0);
    //right_side.push_back(temp_obj);
*/    
    //env = left_side;
    //env = addTwoVectorsOfObjects(env, right_side);
    
    //special case
    if(region_num == 8)
        entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
    
    //compute block of region based upon entrance & exit
    vector<Object> block_of_region = _exitAndentry_buildregion(region, path_space, entrance, compute_exit, region_num);

    //form a structure 
    vector<Exit> test;
    //Object ent;
    //ent.set(left_side[0].X1(), left_side[0].Y1(), right_side.back().X2(), right_side.back().Y2(), 0);
    test.push_back(compute_exit);
    sprintf(fileName, "%s%d%s", "Maps/Offline/Crossed_exitsAndrobot -",0,".png");
    plotObjectsOf3KindswithExits(fileName, left_side, right_side, test);
    
    //cout << "left size : " << left_list.size() << endl;
    //cout << "right size : " << right_list.size() << endl;
    //cout << "block_of_region size : " << block_of_region.size() << endl;
    //if(region_num != 1)
    //env = StructureDescription(left_list, right_list, block_of_region, path);
       

    sprintf(fileName, "%s", "Maps/Offline/PathSpace_Geom_Surrounding.png");
    //plotObjectsOf4Kinds(fileName, block_of_region, left_side, right_side,  path_space);
    plotObjectsOf3Kinds(fileName, left_side, right_side, path_space);
    
    
    return block_of_region;
    
}

/*
vector<Object> _Struct_Connect(vector<Object>& pre_region, vector<Object>& region, vector<Object> path_space, Object path,
                                Exit previous_exit, Exit imagin_exit, int region_num)
{
    cout << " Struct block processing " << endl;
    char fileName[80];
    int no_exit_flag = 0;
    int left_s, left_e, right_s, right_e;
    double dist_entry, dist_out, min_entry, min_out;
    
    Object temp_obj, exit, entrance;
    Point temp_point;
    Point left_entry, left_out, right_entry, right_out;
    
    Exit compute_exit;
    
    vector<Object> env;
    vector<Object> left_list, right_list, left_side, right_side;
    vector<Point> left_points, right_points, left_potential, right_potential;
    
    //entrance and exit info
    entrance.set(path_space[0].X1(), path_space[0].Y1(), path_space[1].X1(), path_space[1].Y1(), 2);
    exit.set(path_space[0].X2(), path_space[0].Y2(), path_space[1].X2(), path_space[1].Y2(), 2);
    

    for(int i = 0; i < region.size(); i++)
    {
        if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) > 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) > 0))
        {
            left_list.push_back(region[i]);
        }
        else
        {
            if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) < 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) < 0))
            {
                right_list.push_back(region[i]);
            }
        }
    }
    
    //convert to point
    left_points = ObjectToPoints(left_list);
    right_points = ObjectToPoints(right_list);

    
    //left & right potential list of points
    for(int i = 0; i < left_points.size(); i++)
    {
        temp_point = crossPerpend(path.getP1(), path.getP2(), left_points[i]);
        temp_obj.set(left_points[i].X(), left_points[i].Y(), temp_point.X(), temp_point.Y(), 1);
        
        if(interSectWithLine(region, temp_obj.getP1(), temp_obj.getP2()) == false)
            left_potential.push_back(left_points[i]);
    }
    
    for(int i = 0; i < right_points.size(); i++)
    {
        temp_point = crossPerpend(path.getP1(), path.getP2(), right_points[i]);
        temp_obj.set(right_points[i].X(), right_points[i].Y(), temp_point.X(), temp_point.Y(), 2);
        
        if(interSectWithLine(region, temp_obj.getP1(), temp_obj.getP2()) == false)
            right_potential.push_back(right_points[i]);
    }
    
    if(left_list.size() != 0 && right_list.size() != 0)
    {
        //compute exit
        compute_exit = exitFromRegion(left_list, right_list, path, imagin_exit);
        exit.set(compute_exit.X1(), compute_exit.Y1(), compute_exit.X2(), compute_exit.Y2(), 2);
    }
    else
    {
        //there is no exit in current region
        //compute current region and next region
        //entrance of current region and exit of next region to compute the block
        
        no_exit_flag = 1;
        entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
        region[0].set_region_remove_flag(1);
    }

    
    //compute block of region based upon entrance & exit
    vector<Object> block_of_region;
    
    if(no_exit_flag != 1)
    {
        if(pre_region[0].get_region_remove_flag() == 0)
            block_of_region = _exitAndentry_buildregion(region, path_space, entrance, compute_exit, region_num);
        else
        {
            entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
            block_of_region = _exitAndentry_buildregion(region, path_space, entrance, compute_exit, region_num);
        }
    }
    else
    {
        //modified entrance for computing current block, for covering previous region 
        //entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
        //block_of_region = _exitAndentry_buildregion(region, path_space, entrance, compute_exit, region_num);
    }

    //form a structure 

    sprintf(fileName, "%s", "Maps/Offline/PathSpace_Geom_Surrounding.png");
    //plotObjectsOf4Kinds(fileName, block_of_region, left_side, right_side,  path_space);
    plotObjectsOf3Kinds(fileName, left_side, right_side, path_space);
    
    
    return block_of_region;
    
}
*/

vector<Object> _Struct_Connect(vector<Object>& pre_region, vector<Object>& region, vector<Object> path_space, Object path,
                                Exit previous_exit, Exit imagin_exit, int region_num)
{
    cout << " Struct block processing " << endl;
    char fileName[80];
    int no_exit_flag = 0;
    int left_s, left_e, right_s, right_e;
    double dist_entry, dist_out, min_entry, min_out;
    
    Object temp_obj, exit, entrance;
    Point temp_point;
    Point left_entry, left_out, right_entry, right_out;
    
    Exit compute_exit;
    
    vector<Object> env;
    vector<Object> left_list, right_list, left_side, right_side;
    vector<Point> left_points, right_points, left_potential, right_potential;
    
    //entrance and exit info
    entrance.set(path_space[0].X1(), path_space[0].Y1(), path_space[1].X1(), path_space[1].Y1(), 2);
    exit.set(path_space[0].X2(), path_space[0].Y2(), path_space[1].X2(), path_space[1].Y2(), 2);
    

    for(int i = 0; i < region.size(); i++)
    {
        if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) > 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) > 0))
        {
            left_list.push_back(region[i]);
        }
        else
        {
            if((isLeft(path.getP1(), path.getP2(), region[i].getP1()) < 0)
                    && (isLeft(path.getP1(), path.getP2(), region[i].getP2()) < 0))
            {
                right_list.push_back(region[i]);
            }
        }
    }
    
    //convert to point
    left_points = ObjectToPoints(left_list);
    right_points = ObjectToPoints(right_list);

    
    //left & right potential list of points
    for(int i = 0; i < left_points.size(); i++)
    {
        temp_point = crossPerpend(path.getP1(), path.getP2(), left_points[i]);
        temp_obj.set(left_points[i].X(), left_points[i].Y(), temp_point.X(), temp_point.Y(), 1);
        
        if(interSectWithLine(region, temp_obj.getP1(), temp_obj.getP2()) == false)
            left_potential.push_back(left_points[i]);
    }
    
    for(int i = 0; i < right_points.size(); i++)
    {
        temp_point = crossPerpend(path.getP1(), path.getP2(), right_points[i]);
        temp_obj.set(right_points[i].X(), right_points[i].Y(), temp_point.X(), temp_point.Y(), 2);
        
        if(interSectWithLine(region, temp_obj.getP1(), temp_obj.getP2()) == false)
            right_potential.push_back(right_points[i]);
    }
    
    if(left_list.size() != 0 && right_list.size() != 0)
    {
        //compute exit
        compute_exit = exitFromRegion(left_list, right_list, path, imagin_exit);
        exit.set(compute_exit.X1(), compute_exit.Y1(), compute_exit.X2(), compute_exit.Y2(), 2);
    }
    else
    {
        //there is no exit in current region
        //compute current region and next region
        //entrance of current region and exit of next region to compute the block
        
        no_exit_flag = 1;
        entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
        region[0].set_region_remove_flag(1);
    }

    if((compute_exit.X1() == 0) && (compute_exit.X2() == 0) 
            && (compute_exit.Y1() == 0) && (compute_exit.Y2() == 0))
    {
        no_exit_flag = 1;
        entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
        region[0].set_region_remove_flag(1);
    }
    
    //compute block of region based upon entrance & exit
    vector<Object> block_of_region;
    
    if(no_exit_flag != 1)
    {
        if(pre_region[0].get_region_remove_flag() != 1)
            block_of_region = _exitAndentry_buildregion(region, path_space, entrance, compute_exit, region_num);
        else
        {
            entrance.set(previous_exit.X1(), previous_exit.Y1(), previous_exit.X2(), previous_exit.Y2(), 2);
            block_of_region = _exitAndentry_buildregion(pre_region, path_space, entrance, compute_exit, region_num);
        }
    }

    //modified for test
    //if(block_of_region.size() > 0)
    //    environment_shape(region, path_space, entrance, region_num);

    //form a structure 

    sprintf(fileName, "%s%d%s", "Maps/Offline/PathSpace_Geom_Surrounding", region_num, ".png");
    //plotObjectsOf4Kinds(fileName, block_of_region, left_side, right_side,  path_space);
    plotObjectsOf3Kinds(fileName, block_of_region, region, path_space);
    //plotObjectsOf3Kinds(fileName, block_of_region, ref1, ref2);
    
    return block_of_region;
}


/* remove acute angle and insert a modified surface
 * between two points, and return a whole list of surfaces
 */
vector<Object> removeAcuteAngle(vector<Object> surfaces, Object path)
{
    char fileName[80];
    int size;
    double angle, dist_to_path;
    Object temp_obj;
    vector<Object> rnt;
    vector<Object> test;
    
    rnt = surfaces;
    size = rnt.size() - 1;
    
    if(surfaces.size() - 1 > 2)
    {
        for(int i = 0; i < size; i++)
        {
                //angle between two adjacent surfaces
                //angle = rnt[i].getAngleWithLine(rnt[i+1]);
                angle = includedAngle(rnt[i], rnt[i+1]);
                //cout << " the angle is : " << angle << endl;
                //cout << " the size of current : " << rnt.size() << endl;

                dist_to_path = P_To_ShortestDistance(rnt[i].getP2(), path);


                if((angle < 45) && (dist_to_path > 800))

                {
                        if(i+1 != rnt.size()-1)
                        {
                            if(i != 0)
                            {
                                //remove these two surface and the common point
                                temp_obj.set(rnt[i-1].X2(), rnt[i-1].Y2(), rnt[i+2].X1(), rnt[i+2].Y1(), 1);
                                //rnt.erase(rnt.begin() + i + 1);
                                //rnt.erase(rnt.begin() + i);
                            }
                            else
                                temp_obj.set(rnt[i].X1(), rnt[i].Y1(), rnt[i+2].X1(), rnt[i+2].Y1(), 1);

                        }
                        else
                        {   
                                temp_obj.set(rnt[i-1].X2(), rnt[i-1].Y2(), rnt[i+1].X2(), rnt[i+1].Y2(), 1);
                                //
                        }

                        rnt.erase(rnt.begin() + i + 1);
                        rnt.erase(rnt.begin() + i);
                        rnt.insert(rnt.begin()+i, temp_obj);


                        size = rnt.size() - 1;
                        i = 0;
                }

        }
    }
    else
    {
        //only two surfaces
        if(rnt.size() == 2)
        {
            angle = includedAngle(rnt[0], rnt[1]);
            dist_to_path = P_To_ShortestDistance(rnt[0].getP2(), path);

            if((angle < 45) && (dist_to_path > 800))
            {
                temp_obj.set(rnt[0].X1(), rnt[0].Y1(), rnt[1].X2(), rnt[1].Y2(), 1);
                rnt.clear();
                rnt.push_back(temp_obj);
            }
            
        }
        
    }
    
    return rnt;
}


/* compute min space block based upon the region information
 * input : structure computed from region
 * output: block / square shape minimum free space
 */
/*
vector<Object> minSpaceinRegion(vector<Object> block_space, vector<Object> mfis,
                                vector<Object> leftStruct, vector<Object> rightStruct)
{
    cout << " Shrink the block space to minimum free space " << endl << endl;
    
    int flag = 0;
    double slope;
    Point position;
    
    Object s1, s2, s3, s4;
    vector<Object> block;
    
    Object entry, exit;
    
    
    
    if()
    {
            //left side to shift to right until no intersection
            do
            {
                    if(intersectedPoint() == true)
                    {

                    }
                    else
                    {
                        flag = 1;
                    }

            }while(flag == 0);

            //right side to shift to left until no intersection
            do
            {
                    if(intersectedPoint() == true)
                    {

                    }
                    else
                    {
                        flag = 1;
                    }

            }while(flag == 0);
    }
    
   
    
    return block;
}
*/

/* compute two rectangle region
 * detect surface inside these two region
 * join the surfaces inside each of regions
 */
vector<Object> exitAndentry_buildregion(vector<Object> region, vector<Object> path_space,
                                        Object entrance, Exit exit, int num)
{
        char fileName[80];

        int flag;
        double intersect_x, intersect_y;
        double c1, c2, m1, m2;
        double dist1, dist2, dist3, dist4;
        double length_line = 500;
        double slope_line;
        double rand_x, rand_y;
        double extend_range = 5000;

        Object convert_exit;
        Object side1, side2, side3, side4;
        vector<Object> region1,  region2;

        Point p1, p2, p3;
        Point _p1, _p2, _p3;
        Point parallel_point, rand, boundary_point;

        convert_exit.set(exit.X1(), exit.Y1(), exit.X2(), exit.Y2(), 1);

        //compute intersection within range
        //compute 3 main points

        m1 = (entrance.Y2() - entrance.Y1()) / (entrance.X2() - entrance.X1());
        m2 = (convert_exit.Y2() - convert_exit.Y1()) / (convert_exit.X2() - convert_exit.X1());

        c1 = entrance.Y1() - m1 * entrance.X1();
        c2 = convert_exit.Y1() - m2 * convert_exit.X1();

        intersect_x = (c1 - c2) / (m2 - m1);
        intersect_y = (m1 * c2 - c1 * m2) / (m1 - m2);
        p3.set(intersect_x, intersect_y);

        if((distanceOftwoP(convert_exit.getP1(), p3) > extend_range)
            && (distanceOftwoP(convert_exit.getP2(), p3) > extend_range))
            flag = 0;
        else
            flag = 1;

        if(flag == 1)
        {    
                dist1 = distanceOftwoP(entrance.getP1(), convert_exit.getP1());
                dist2 = distanceOftwoP(entrance.getP2(), convert_exit.getP1());
                dist3 = distanceOftwoP(entrance.getP1(), convert_exit.getP2());
                dist4 = distanceOftwoP(entrance.getP2(), convert_exit.getP2());

                if((dist1 < dist2) && (dist1 < dist3) && (dist1 < dist4))
                {
                    p1 = entrance.getP1();
                    p2 = convert_exit.getP1();
                    _p1 = entrance.getP2();
                    _p2 = convert_exit.getP2();
                }

                if((dist2 < dist1) && (dist2 < dist3) && (dist2 < dist4))
                {
                    p1 = entrance.getP2();
                    p2 = convert_exit.getP1();
                    _p1 = entrance.getP1();
                    _p2 = convert_exit.getP2(); 
                }

                if((dist3 < dist1) && (dist3 < dist2) && (dist3 < dist4))
                {
                    p1 = entrance.getP1();
                    p2 = convert_exit.getP2();
                    _p1 = entrance.getP2();
                    _p2 = convert_exit.getP1();
                }

                if((dist4 < dist1) && (dist4 < dist2) && (dist4 < dist3))
                {
                    p1 = entrance.getP2();
                    p2 = convert_exit.getP2();
                    _p1 = entrance.getP1();
                    _p2 = convert_exit.getP1();
                }


                side1.set(p1.X(), p1.Y(), p2.X(), p2.Y(), 1);
                side2.set(p2.X(), p2.Y(), p3.X(), p3.Y(), 2);
                side3.set(p3.X(), p3.Y(), p1.X(), p1.Y(), 3);
                region1.push_back(side1);
                region1.push_back(side2);
                region1.push_back(side3);

                //compute second part region
                //compute parallel line based one side3
                slope_line = (side3.Y2()- side3.Y1()) / (side3.X2() - side3.X1());
                rand_x = 400;
                rand_y = rand_x * slope_line + _p2.Y() - _p2.X() * slope_line;
                parallel_point.set(rand_x, rand_y);

                Object extend_line;
                extend_line.set(_p2.X(), _p2.Y(), parallel_point.X(), parallel_point.Y(), 0);

                _p3 = boundary_point = BoundaryPoint(region, extend_line, 2);

                side1.set(_p1.X(), _p1.Y(), _p3.X(), _p3.Y(), 1);
                side2.set(_p3.X(), _p3.Y(), _p2.X(), _p2.Y(), 2);
                side3.set(_p2.X(), _p2.Y(), _p1.X(), _p1.Y(), 3);
                region2.push_back(side1);
                region2.push_back(side2);
                region2.push_back(side3);
        }
        else
        {
                int s_flag1, s_flag2;
                double size;
                Point pp1, pp2, pp3, pp4;
                Point bp1, bp2, bp3, bp4;
                Object perpend1, perpend2, perpend3, perpend4;
                
                size = distanceOftwoP(entrance.getP1(), convert_exit.getP1());

                if(isLeft(entrance.getP1(), entrance.getP2(), convert_exit.getP1()) > 0)
                {
                    s_flag1 = 1;
                    s_flag2 = 2;
                }
                else
                {
                    s_flag1 = 2;
                    s_flag2 = 1;
                }
                
                if((entrance.X1() != entrance.X2()) && (entrance.Y1() != entrance.Y2()))
                {


                    //compute four perpendicular lines
                    pp1 = outSidePerpendPointWithLength(entrance, size, entrance.getP1(), s_flag1);
                    pp2 = outSidePerpendPointWithLength(entrance, size, entrance.getP2(), s_flag1);
                    pp3 = outSidePerpendPointWithLength(convert_exit, size, convert_exit.getP1(), s_flag2);
                    pp4 = outSidePerpendPointWithLength(convert_exit, size, convert_exit.getP2(), s_flag2);

                    perpend1.set(entrance.X1(), entrance.Y1(), pp1.X(), pp1.Y(), 1);
                    perpend2.set(entrance.X2(), entrance.Y2(), pp2.X(), pp2.Y(), 2);
                    perpend3.set(convert_exit.X1(), convert_exit.Y1(), pp3.X(), pp3.Y(), 3);
                    perpend4.set(convert_exit.X2(), convert_exit.Y2(), pp4.X(), pp4.Y(), 4);

                    //compute intersection
                    bp1 = intersectPointTwolineEquations(perpend1, convert_exit);
                    bp2 = intersectPointTwolineEquations(perpend2, convert_exit);
                    bp3 = intersectPointTwolineEquations(perpend3, entrance);
                    bp4 = intersectPointTwolineEquations(perpend4, entrance);
                    /*
                    side1.set(entrance.X1(), entrance.Y1(), bp1.X(), bp1.Y(), 1);
                    side2.set(bp1.X(), bp1.Y(), bp2.X(), bp2.Y(), 2);
                    side3.set(bp2.X(), bp2.Y(), entrance.X2(), entrance.Y2(), 3);
                    side4.set(entrance.X2(), entrance.Y2(), entrance.X1(), entrance.Y1(), 4);
                    region1.push_back(side1);
                    region1.push_back(side2);
                    region1.push_back(side3);
                    region1.push_back(side4);

                    side1.set(convert_exit.X1(), convert_exit.Y1(), bp3.X(), bp3.Y(), 1);
                    side2.set(bp3.X(), bp3.Y(), bp4.X(), bp4.Y(), 2);
                    side3.set(bp4.X(), bp4.Y(), convert_exit.X2(), convert_exit.Y2(), 3);
                    side4.set(convert_exit.X2(), convert_exit.Y2(), convert_exit.X1(), convert_exit.Y1(), 4);
                    region2.push_back(side1);
                    region2.push_back(side2);
                    region2.push_back(side3);
                    region2.push_back(side4);
                    */
                }
                else
                {
                    if(entrance.Y1() == entrance.Y2())
                    {
                        
                        double temp_x, temp_y;
                        temp_x = entrance.X1();
                        temp_y = temp_x * m2 + c2;
                        bp1.set(temp_x, temp_y);
                        temp_x = entrance.X2();
                        temp_y = temp_x * m2 + c2;
                        bp2.set(temp_x, temp_y);
                        
                        pp3 = outSidePerpendPointWithLength(convert_exit, size, convert_exit.getP1(), s_flag2);
                        pp4 = outSidePerpendPointWithLength(convert_exit, size, convert_exit.getP2(), s_flag2);

                        perpend3.set(convert_exit.X1(), convert_exit.Y1(), pp3.X(), pp3.Y(), 3);
                        perpend4.set(convert_exit.X2(), convert_exit.Y2(), pp4.X(), pp4.Y(), 4);
                    
                        bp3 = intersectPointTwolineEquations(perpend3, entrance);
                        bp4 = intersectPointTwolineEquations(perpend4, entrance);
                    }
                    

                }
                side1.set(entrance.X1(), entrance.Y1(), bp1.X(), bp1.Y(), 1);
                side2.set(bp1.X(), bp1.Y(), bp2.X(), bp2.Y(), 2);
                side3.set(bp2.X(), bp2.Y(), entrance.X2(), entrance.Y2(), 3);
                side4.set(entrance.X2(), entrance.Y2(), entrance.X1(), entrance.Y1(), 4);
                region1.push_back(side1);
                region1.push_back(side2);
                region1.push_back(side3);
                region1.push_back(side4);

                side1.set(convert_exit.X1(), convert_exit.Y1(), bp3.X(), bp3.Y(), 1);
                side2.set(bp3.X(), bp3.Y(), bp4.X(), bp4.Y(), 2);
                side3.set(bp4.X(), bp4.Y(), convert_exit.X2(), convert_exit.Y2(), 3);
                side4.set(convert_exit.X2(), convert_exit.Y2(), convert_exit.X1(), convert_exit.Y1(), 4);
                region2.push_back(side1);
                region2.push_back(side2);
                region2.push_back(side3);
                region2.push_back(side4);

        }


        sprintf(fileName, "%s%d%s", "Maps/Offline/Rectangle_region_pathSpace-", num,".png");
        plotObjectsOf4Kinds(fileName, region, region1, region2,  path_space);

        return addTwoVectorsOfObjects(region1, region2);
}


vector<Object> _exitAndentry_buildregion(vector<Object> region, vector<Object> path_space,
                                        Object entrance, Exit exit, int num)
{
        char fileName[80];

        int flag;
        double intersect_x, intersect_y;
        double c1, c2, m1, m2;
        double dist1, dist2, dist3, dist4;

        Point p3, p4, mid;
        
        Object convert_exit, diagonal;
        Object side1, side2, side3, side4;
        
        
        vector<Object> region1,  region2;
        region2.push_back(entrance);


        convert_exit.set(exit.X1(), exit.Y1(), exit.X2(), exit.Y2(), 1);

        //compute intersection within range
        //compute 3 main points

        m1 = (entrance.Y2() - entrance.Y1()) / (entrance.X2() - entrance.X1());
        m2 = (convert_exit.Y2() - convert_exit.Y1()) / (convert_exit.X2() - convert_exit.X1());

        c1 = entrance.Y1() - m1 * entrance.X1();
        c2 = convert_exit.Y1() - m2 * convert_exit.X1();

        intersect_x = (c1 - c2) / (m2 - m1);
        intersect_y = (m1 * c2 - c1 * m2) / (m1 - m2);
        p3.set(intersect_x, intersect_y);

        if((distanceOftwoP(convert_exit.getP1(), p3) > 3000)
            && (distanceOftwoP(convert_exit.getP2(), p3) > 3000))
            flag = 0;
        else
            flag = 1;

        if(flag == 1)
        {    
                //two sides consist of intersection 
                if(distanceOftwoP(entrance.getP1(), p3) > distanceOftwoP(entrance.getP2(), p3))
                    side1.set(p3.X(), p3.Y(), entrance.X1(), entrance.Y1(), 1);
                else
                    side1.set(p3.X(), p3.Y(), entrance.X2(), entrance.Y2(), 1);

                if(distanceOftwoP(convert_exit.getP1(), p3) > distanceOftwoP(convert_exit.getP2(), p3))
                    side4.set(convert_exit.X1(), convert_exit.Y1(), p3.X(), p3.Y(), 4);
                else
                    side4.set(convert_exit.X2(), convert_exit.Y2(), p3.X(), p3.Y(), 4);            


                side1.setEntranceExitflag(1); //this is entrance 
                side4.setEntranceExitflag(2); //this is exit
                
                //compute p4
                mid.set((side1.X2() + side4.X1()) / 2, (side1.Y2() + side4.Y1()) / 2);
                diagonal.set(p3.X(), p3.Y(), mid.X(), mid.Y(), 0);
                
                Object ext = expend_Object(diagonal, diagonal.length(), 2);
                
                side2.set(side1.X2(), side1.Y2(), ext.X2(), ext.Y2(), 2);
                side3.set(ext.X2(), ext.Y2(), side4.X1(), side4.Y1(), 3);
                
                side2.setEntranceExitflag(0); //this is entrance 
                side3.setEntranceExitflag(0); //this is exit
                        
        }
        else
        {
                //join the entrance and exit to form a rectangle directly
                side1 = entrance;
                side3 = convert_exit;
                
                side1.setEntranceExitflag(1); //this is entrance 
                side3.setEntranceExitflag(2); //this is exit

                side2.set(entrance.X1(), entrance.Y1(), convert_exit.X1(), convert_exit.Y1(), 3);
                side4.set(entrance.X2(), entrance.Y2(), convert_exit.X2(), convert_exit.Y2(), 4);

                if(TwoObjectIntersect(side2, side4) == true)
                {
                    side2.set(entrance.X1(), entrance.Y1(), convert_exit.X2(), convert_exit.Y2(), 3);
                    side4.set(entrance.X2(), entrance.Y2(), convert_exit.X1(), convert_exit.Y1(), 4);
                }
                
                side2.setEntranceExitflag(0); //this is entrance 
                side4.setEntranceExitflag(0); //this is exit
        }

 
        region1.push_back(side1);
        region1.push_back(side2);
        region1.push_back(side3);
        region1.push_back(side4);


        sprintf(fileName, "%s%d%s", "Maps/Offline/Rectangle_region_pathSpace-", num,".png");
        plotObjectsOf4Kinds(fileName, region, region1, region2,  path_space);

        //return addTwoVectorsOfObjects(region1, region2);
        return region1;
}

/* extend two adjacent region block to find connection 
 * and intersection. 
 * inpout: current MFIS, current block and next block
 * output: two extended blocks
 */
vector<Object> extendRegionBlock(vector<Object> current_mfis, vector<Object> space_block, int flag)
{
        char fileName[80];
        int extend_direction = 0;
        double extend_length = 500;

        Point extend_point1, extend_point2;
        
        Object ref, remain;
        Object to_extend1, to_extend2;
        Object extend1, extend2, last_side;
        vector<Object> modifiedblock;
        vector<Object> extend_lines;
        vector<Object> origin_block;
        
        origin_block = space_block;
   

        //cout << "size of elements of block : "<< space_block.size() << endl;
        
        if(flag == 1)
        {
                for(int i = 0; i < origin_block.size(); i++)
                {
                    if(origin_block[i].getEntanceExitflag() == 2)
                        origin_block.erase(origin_block.begin()+i);
                }
            
                //which two sides need to  be extended
                for(int i = 0; i < space_block.size(); i++)
                {
                        if(space_block[i].getEntanceExitflag() == 2)
                            ref = space_block[i];
                }

                for(int i = 0; i < space_block.size(); i++)
                {
                    if(space_block[i].getP2() == ref.getP1()) 
                        to_extend1 = space_block[i];     //share point1 with exit
                    else
                        if(space_block[i].getP1() == ref.getP2())
                            to_extend2 = space_block[i]; //share point2 with exit 
                    
                    if((space_block[i].getP2() != ref.getP1())
                        && (space_block[i].getP1() != ref.getP2())
                        && (space_block[i].getP1() != ref.getP1())
                        && (space_block[i].getP2() != ref.getP2()))
                    {
                            remain = space_block[i];
                            remain.setID(1);
                    }
                }

                //which direction to extend
                if(isLeft(ref.getP1(), ref.getP2(), remain.getP1()) > 0)
                    extend_direction = 2; //2 is right 
                else
                    extend_direction = 1; //1 is left


                //extend_point1 = outSidePerpendPointWithLength(ref, 500, ref.getP1(), extend_direction);
                //extend_point2 = outSidePerpendPointWithLength(ref, 500, ref.getP2(), extend_direction);
                
                extend_point1 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP1(), extend_direction);
                extend_point2 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP2(), extend_direction);

                extend1.set(ref.X1(), ref.Y1(), extend_point1.X(), extend_point1.Y(), 5);
                extend2.set(ref.X2(), ref.Y2(), extend_point2.X(), extend_point2.Y(), 6);

                extend_lines.push_back(extend1);
                extend_lines.push_back(extend2);


                last_side.set(extend_point1.X(), extend_point1.Y(), extend_point2.X(), extend_point2.Y(), 3);
                //modified the block(exit, two sides)
                //modifiedblock.push_back(remain);
                //modifiedblock.push_back(to_extend1);
                modifiedblock.push_back(extend1);
                modifiedblock.push_back(last_side);
                modifiedblock.push_back(extend2);
                //modifiedblock.push_back(to_extend2);
                modifiedblock.push_back(ref);
                
                //modifiedblock = addTwoVectorsOfObjects(modifiedblock, origin_block);
        }
        else
        {
                for(int i = 0; i < origin_block.size(); i++)
                {
                    if(origin_block[i].getEntanceExitflag() == 1)
                        origin_block.erase(origin_block.begin()+i);
                }
                
                //which two sides need to  be extended
                for(int i = 0; i < space_block.size(); i++)
                {
                        if(space_block[i].getEntanceExitflag() == 1)
                            ref = space_block[i];
                }

                for(int i = 0; i < space_block.size(); i++)
                {
                    if((space_block[i].getP2() == ref.getP1()) 
                        || (space_block[i].getP1() == ref.getP1()))
                        to_extend1 = space_block[i];     //share point1 with exit
                    else
                        if((space_block[i].getP1() == ref.getP2())
                            || (space_block[i].getP2() == ref.getP2()))
                            to_extend2 = space_block[i]; //share point2 with exit 
                    
                    if((space_block[i].getP2() != ref.getP1())
                        && (space_block[i].getP1() != ref.getP2())
                        && (space_block[i].getP1() != ref.getP1())
                        && (space_block[i].getP2() != ref.getP2()))
                    {
                            remain = space_block[i];
                            remain.setID(1);
                    }
                }

                //which direction to extend
                if(isLeft(ref.getP1(), ref.getP2(), remain.getP1()) > 0)
                    extend_direction = 2;
                else
                    extend_direction = 1;


                //extend_point1 = outSidePerpendPointWithLength(ref, 500, ref.getP1(), extend_di                                                                                                                                                                                                                                                        rection);
                //extend_point2 = outSidePerpendPointWithLength(ref, 500, ref.getP2(), extend_direction);
                
                extend_point1 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP1(), extend_direction);
                extend_point2 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP2(), extend_direction);

                extend1.set(ref.X1(), ref.Y1(), extend_point1.X(), extend_point1.Y(), 5);
                extend2.set(ref.X2(), ref.Y2(), extend_point2.X(), extend_point2.Y(), 6);

                extend_lines.push_back(extend1);
                extend_lines.push_back(extend2);


                last_side.set(extend_point1.X(), extend_point1.Y(), extend_point2.X(), extend_point2.Y(), 3);
                //modified the block(exit, two sides)
                //modifiedblock.push_back(remain);
                //modifiedblock.push_back(to_extend1);
                modifiedblock.push_back(extend1);
                modifiedblock.push_back(last_side);
                modifiedblock.push_back(extend2);
                //modifiedblock.push_back(to_extend2);
                modifiedblock.push_back(ref);
                //modifiedblock = addTwoVectorsOfObjects(modifiedblock, origin_block);
        }

        
        sprintf(fileName, "%s%d%s", "Maps/Offline/Extended_Region_block-", 0,".png");
        //plotObjects(fileName, modifiedblock, current_mfis);
        plotObjectsOf3Kinds(fileName, modifiedblock, current_mfis, extend_lines);
        
        return modifiedblock;
      
}

vector<Object> _extendRegionBlock(vector<Object> current_mfis, vector<Object> space_block, 
                                 double extend_length, int flag)
{
        char fileName[80];
        int extend_direction = 0;

        Point extend_point1, extend_point2;
        
        Object ref, remain;
        Object to_extend1, to_extend2;
        Object extend1, extend2, last_side;
        vector<Object> modifiedblock;
        vector<Object> extend_lines;
        vector<Object> origin_block;
        
        origin_block = space_block;
   

        //cout << "size of elements of block : "<< space_block.size() << endl;
        
        if(flag == 1)
        {
                for(int i = 0; i < origin_block.size(); i++)
                {
                    if(origin_block[i].getEntanceExitflag() == 2)
                        origin_block.erase(origin_block.begin()+i);
                }
            
                //which two sides need to  be extended
                for(int i = 0; i < space_block.size(); i++)
                {
                        if(space_block[i].getEntanceExitflag() == 2)
                            ref = space_block[i];
                }

                for(int i = 0; i < space_block.size(); i++)
                {
                    if(space_block[i].getP2() == ref.getP1()) 
                        to_extend1 = space_block[i];     //share point1 with exit
                    else
                        if(space_block[i].getP1() == ref.getP2())
                            to_extend2 = space_block[i]; //share point2 with exit 
                    
                    if((space_block[i].getP2() != ref.getP1())
                        && (space_block[i].getP1() != ref.getP2())
                        && (space_block[i].getP1() != ref.getP1())
                        && (space_block[i].getP2() != ref.getP2()))
                    {
                            remain = space_block[i];
                            remain.setID(1);
                    }
                }

                //which direction to extend
                if(isLeft(ref.getP1(), ref.getP2(), remain.getP1()) > 0)
                    extend_direction = 2; //2 is right 
                else
                    extend_direction = 1; //1 is left


                //extend_point1 = outSidePerpendPointWithLength(ref, 500, ref.getP1(), extend_direction);
                //extend_point2 = outSidePerpendPointWithLength(ref, 500, ref.getP2(), extend_direction);
                
                extend_point1 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP1(), extend_direction);
                extend_point2 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP2(), extend_direction);

                extend1.set(ref.X1(), ref.Y1(), extend_point1.X(), extend_point1.Y(), 5);
                extend2.set(ref.X2(), ref.Y2(), extend_point2.X(), extend_point2.Y(), 6);

                extend_lines.push_back(extend1);
                extend_lines.push_back(extend2);


                last_side.set(extend_point1.X(), extend_point1.Y(), extend_point2.X(), extend_point2.Y(), 3);
                //modified the block(exit, two sides)
                //modifiedblock.push_back(remain);
                //modifiedblock.push_back(to_extend1);
                modifiedblock.push_back(extend1);
                modifiedblock.push_back(last_side);
                modifiedblock.push_back(extend2);
                //modifiedblock.push_back(to_extend2);
                modifiedblock.push_back(ref);
                
                //modifiedblock = addTwoVectorsOfObjects(modifiedblock, origin_block);
        }
        else
        {
                for(int i = 0; i < origin_block.size(); i++)
                {
                    if(origin_block[i].getEntanceExitflag() == 1)
                        origin_block.erase(origin_block.begin()+i);
                }
                
                //which two sides need to  be extended
                for(int i = 0; i < space_block.size(); i++)
                {
                        if(space_block[i].getEntanceExitflag() == 1)
                            ref = space_block[i];
                }

                for(int i = 0; i < space_block.size(); i++)
                {
                    if((space_block[i].getP2() == ref.getP1()) 
                        || (space_block[i].getP1() == ref.getP1()))
                        to_extend1 = space_block[i];     //share point1 with exit
                    else
                        if((space_block[i].getP1() == ref.getP2())
                            || (space_block[i].getP2() == ref.getP2()))
                            to_extend2 = space_block[i]; //share point2 with exit 
                    
                    if((space_block[i].getP2() != ref.getP1())
                        && (space_block[i].getP1() != ref.getP2())
                        && (space_block[i].getP1() != ref.getP1())
                        && (space_block[i].getP2() != ref.getP2()))
                    {
                            remain = space_block[i];
                            remain.setID(1);
                    }
                }

                //which direction to extend
                if(isLeft(ref.getP1(), ref.getP2(), remain.getP1()) > 0)
                    extend_direction = 2;
                else
                    extend_direction = 1;


                //extend_point1 = outSidePerpendPointWithLength(ref, 500, ref.getP1(), extend_di                                                                                                                                                                                                                                                        rection);
                //extend_point2 = outSidePerpendPointWithLength(ref, 500, ref.getP2(), extend_direction);
                
                extend_point1 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP1(), extend_direction);
                extend_point2 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP2(), extend_direction);

                extend1.set(ref.X1(), ref.Y1(), extend_point1.X(), extend_point1.Y(), 5);
                extend2.set(ref.X2(), ref.Y2(), extend_point2.X(), extend_point2.Y(), 6);

                extend_lines.push_back(extend1);
                extend_lines.push_back(extend2);


                last_side.set(extend_point1.X(), extend_point1.Y(), extend_point2.X(), extend_point2.Y(), 3);
                //modified the block(exit, two sides)
                //modifiedblock.push_back(remain);
                //modifiedblock.push_back(to_extend1);
                modifiedblock.push_back(extend1);
                modifiedblock.push_back(last_side);
                modifiedblock.push_back(extend2);
                //modifiedblock.push_back(to_extend2);
                modifiedblock.push_back(ref);
                //modifiedblock = addTwoVectorsOfObjects(modifiedblock, origin_block);
        }

        
        sprintf(fileName, "%s%d%s", "Maps/Offline/Extended_Region_block-", 0,".png");
        //plotObjects(fileName, modifiedblock, current_mfis);
        plotObjectsOf3Kinds(fileName, modifiedblock, current_mfis, extend_lines);
        
        return modifiedblock;
      
}

vector<Object> extendRegionBlockToBoundary(vector<Object> current_mfis, vector<Object> space_block, int flag)
{
        char fileName[80];
        int extend_direction = 0;
        double extend_length = 100;
        
        Point extend_point1, extend_point2;
        
        Object parallel_line; //check if it is up to boundary 
        Object ref, remain;
        Object to_extend1, to_extend2;
        Object extend1, extend2, last_side;
        vector<Object> modifiedblock;
        vector<Object> extend_lines;
        vector<Object> origin_block;
        
        origin_block = space_block;
        
lb:     if(flag == 1)
        {
                for(int i = 0; i < origin_block.size(); i++)
                {
                    if(origin_block[i].getEntanceExitflag() == 2)
                        origin_block.erase(origin_block.begin()+i);
                }
            
                //which two sides need to  be extended
                for(int i = 0; i < space_block.size(); i++)
                {
                        if(space_block[i].getEntanceExitflag() == 2)
                            ref = space_block[i];
                }

                for(int i = 0; i < space_block.size(); i++)
                {
                    if(space_block[i].getP2() == ref.getP1()) 
                        to_extend1 = space_block[i];     //share point1 with exit
                    else
                        if(space_block[i].getP1() == ref.getP2())
                            to_extend2 = space_block[i]; //share point2 with exit 
                    
                    if((space_block[i].getP2() != ref.getP1())
                        && (space_block[i].getP1() != ref.getP2())
                        && (space_block[i].getP1() != ref.getP1())
                        && (space_block[i].getP2() != ref.getP2()))
                    {
                            remain = space_block[i];
                            remain.setID(1);
                    }
                }

                //which direction to extend
                if(isLeft(ref.getP1(), ref.getP2(), remain.getP1()) > 0)
                    extend_direction = 2; //2 is right 
                else
                    extend_direction = 1; //1 is left

                
                extend_point1 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP1(), extend_direction);
                extend_point2 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP2(), extend_direction);
        
        }
        else
        {
                for(int i = 0; i < origin_block.size(); i++)
                {
                    if(origin_block[i].getEntanceExitflag() == 1)
                        origin_block.erase(origin_block.begin()+i);
                }
                
                //which two sides need to  be extended
                for(int i = 0; i < space_block.size(); i++)
                {
                        if(space_block[i].getEntanceExitflag() == 1)
                            ref = space_block[i];
                }

                for(int i = 0; i < space_block.size(); i++)
                {
                    if((space_block[i].getP2() == ref.getP1()) 
                        || (space_block[i].getP1() == ref.getP1()))
                        to_extend1 = space_block[i];     //share point1 with exit
                    else
                        if((space_block[i].getP1() == ref.getP2())
                            || (space_block[i].getP2() == ref.getP2()))
                            to_extend2 = space_block[i]; //share point2 with exit 
                    
                    if((space_block[i].getP2() != ref.getP1())
                        && (space_block[i].getP1() != ref.getP2())
                        && (space_block[i].getP1() != ref.getP1())
                        && (space_block[i].getP2() != ref.getP2()))
                    {
                            remain = space_block[i];
                            remain.setID(1);
                    }
                }

                //which direction to extend
                if(isLeft(ref.getP1(), ref.getP2(), remain.getP1()) > 0)
                    extend_direction = 2;
                else
                    extend_direction = 1;
                
                extend_point1 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP1(), extend_direction);
                extend_point2 = outSidePerpendPointWithLeftRight(ref, extend_length, ref.getP2(), extend_direction);

        }

        extend1.set(ref.X1(), ref.Y1(), extend_point1.X(), extend_point1.Y(), 5);
        extend2.set(ref.X2(), ref.Y2(), extend_point2.X(), extend_point2.Y(), 6);
        extend_lines.push_back(extend1);
        extend_lines.push_back(extend2);
        extend_lines.push_back(ref);
        //sprintf(fileName, "%s%d%s", "Maps/Offline/Extended_Region_block TEST-", 0,".png");
        //plotObjects(fileName, current_mfis, extend_lines);
        //waitHere();
        
        //parallel_line.set(extend_point1.X(), extend_point1.Y(), extend_point2.X(), extend_point2.Y(), 0);
        
        if(interSectWithLine(current_mfis, extend_point1, extend_point2) == true)
        {
            extend1.set(ref.X1(), ref.Y1(), extend_point1.X(), extend_point1.Y(), 5);
            extend2.set(ref.X2(), ref.Y2(), extend_point2.X(), extend_point2.Y(), 6);

            extend_lines.push_back(extend1);
            extend_lines.push_back(extend2);
            last_side.set(extend_point1.X(), extend_point1.Y(), extend_point2.X(), extend_point2.Y(), 3);
            modifiedblock.push_back(extend1);
            modifiedblock.push_back(last_side);
            modifiedblock.push_back(extend2);
            modifiedblock.push_back(ref);

            sprintf(fileName, "%s%d%s", "Maps/Offline/Extended_Region_block-", 0,".png");
            //plotObjects(fileName, modifiedblock, current_mfis);
            plotObjectsOf3Kinds(fileName, modifiedblock, current_mfis, extend_lines);
        }
        else
        {
            if(extend_length < 200000)
            {
                //cout << " It is not blocked yet. " << endl;
                extend_length += 100;
                goto lb;
            }
            else
            {
                modifiedblock = origin_block;
            }
        }
        
        //cout << " loop time : " << extend_length/100 << endl<< endl;
        //waitHere();
        
        return modifiedblock;
      
}


vector<Object> StructureDescription(vector<Object> left_env, vector<Object> right_env, 
                                    vector<Object> region_block, Object path_segment)
{
    cout << " Structure description function!!! " << endl; 
    char fileName[80];
    unsigned char collect_flag = 0;
    double threshold = 600;
    
    ClipperLib::Path subj;
    
    Object temp_obj;
    vector<Object> left_surround, right_surround;
    
    vector<Surface> polygon;
    vector<Point> left_points, right_points, left_collect, right_collect;
    
    left_points = ObjectToPoints(left_env);
    right_points = ObjectToPoints(right_env);
    
    //polygon = makePolygonOfCV(region_block);
    
    subj = viewConvertPath(region_block);
    //cout << " TEST Programe1 " << endl;  
    //cout << " size of region block : " << region_block.size() << endl;
    //waitHere();
    //left process 
    for(int i = 0; i < left_points.size(); i++)
    {/*
        if(//(pointInPolygon(PointXY(left_points[i].X(), left_points[i].Y()), polygon) == true)
            //|| (pointInPolygon(PointXY(left_points[i].X(), left_points[i].Y()), polygon) == true)
             (FindDistanceToSegment(left_points[i], region_block[0]) < threshold)
            || (FindDistanceToSegment(left_points[i], region_block[1]) < threshold)
            || (FindDistanceToSegment(left_points[i], region_block[2]) < threshold)
            || (FindDistanceToSegment(left_points[i], region_block[3]) < threshold))
        {
            left_collect.push_back(left_points[i]);
        }
        */
        for(int m = 0; m < region_block.size(); m++)
        {
            if((FindDistanceToSegment(left_points[i], region_block[m]) < threshold)
                || (PointInPolygon(IntPoint (left_points[i].X(), left_points[i].Y()), subj) != 0))
            {
                collect_flag = 1;
                break;
            }
            //cout << " current cunter m : " << m << endl;
                
        }
        
        if(collect_flag == 1)
            left_collect.push_back(left_points[i]);
        collect_flag = 0;
        //cout << " current cunter i : " << i << endl;
    }
    //cout << " TEST Programe2 " << endl;
    for(int i = 0; i < left_collect.size()-1; i++)
    {
        temp_obj.set(left_collect[i].X(), left_collect[i].Y(), left_collect[i+1].X(), left_collect[i+1].Y(), i);
        left_surround.push_back(temp_obj);
    }
    
    //right process 
    for(int i = 0; i < right_points.size(); i++)
    {/*
        if(//(pointInPolygon(PointXY(right_points[i].X(), right_points[i].Y()), polygon) == true)
            //|| (pointInPolygon(PointXY(right_points[i].X(), right_points[i].Y()), polygon) == true)
             (FindDistanceToSegment(right_points[i], region_block[0]) < threshold)
            || (FindDistanceToSegment(right_points[i], region_block[1]) < threshold)
            || (FindDistanceToSegment(right_points[i], region_block[2]) < threshold)
            || (FindDistanceToSegment(right_points[i], region_block[3]) < threshold))
        {
            right_collect.push_back(right_points[i]);
        }
        */
        for(int m = 0; m < region_block.size(); m++)
        {
            if((FindDistanceToSegment(right_points[i], region_block[m]) < threshold)
                || (PointInPolygon(IntPoint (right_points[i].X(), right_points[i].Y()), subj) != 0))
            {
                collect_flag = 1;
                break;
            }
                
        }
        
        if(collect_flag == 1)
            right_collect.push_back(right_points[i]);
        collect_flag = 0;
        
    }
    
    for(int i = 0; i < right_collect.size()-1; i++)
    {
        temp_obj.set(right_collect[i].X(), right_collect[i].Y(), right_collect[i+1].X(), right_collect[i+1].Y(), i);
        right_surround.push_back(temp_obj);
    }
    
    left_surround = removeAcuteAngle(left_surround, path_segment);
    right_surround = removeAcuteAngle(right_surround, path_segment);
    
    
    //vector<Point> temprory;
    
    //addTwoVectorsOfPoints(left_collect, right_collect);
    //sprintf(fileName, "%s", "Maps/Offline/SorrundingStruct_Region_block.png");
    //plotObjectsOf3Kinds(fileName, left_surround, right_surround, region_block);
    
    //plotPoints(fileName, left_collect);
    //waitHere();
    
    return addTwoVectorsOfObjects(left_surround, right_surround);
}

vector<Object> _StructureDescription(vector<Object> mfis, vector<Object> region_block, Object path_segment)
{
    vector<Object> left_list, right_list;
    
    for(int i = 0; i < mfis.size(); i++)
    {
        if((isLeft(path_segment.getP1(), path_segment.getP2(), mfis[i].getP1()) > 0)
                    && (isLeft(path_segment.getP1(), path_segment.getP2(), mfis[i].getP2()) > 0))
        {
            left_list.push_back(mfis[i]);
        }
        else
        {
            if((isLeft(path_segment.getP1(), path_segment.getP2(), mfis[i].getP1()) < 0)
                    && (isLeft(path_segment.getP1(), path_segment.getP2(), mfis[i].getP2()) < 0))
            {
                right_list.push_back(mfis[i]);
            }
        }
    }
    
    
    return StructureDescription(left_list, right_list, region_block, path_segment);
}

vector< vector<Object> >  StructureDescription(vector< vector<Object> > mfis, vector< vector<Object> > combined_blocks, 
                                               vector<Object> Path)
{
    
    
    char fileName[80];
    int region_counter = 0;
    vector<Object> temp;
    vector<Object> left_env,  right_env;
    vector< vector<Object> > rnt;
    
    for(int i = 0; i < combined_blocks.size(); i++)
    {
        
        if(mfis[region_counter][0].get_region_remove_flag() != 1)
        {
            //classify left and right surfaces based upon path
            for(int j = 0; j < mfis[region_counter].size(); j++)
            {
                if(((isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP1()) > 0)
                    && (isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP2()) > 0))
                    ||((isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP1()) > 0)
                        && (isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP2()) > 0)))
                {
                    left_env.push_back(mfis[region_counter][j]);
                }
                else
                {
                    if(((isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP1()) < 0)
                        && (isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP2()) < 0))
                        || ((isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP1()) < 0)
                            && (isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP2()) < 0)))
                    {
                        right_env.push_back(mfis[region_counter][j]);
                    }
                }
            }
            
            temp = StructureDescription(left_env, right_env, combined_blocks[i], Path[region_counter]);
            rnt.push_back(temp);
            region_counter = i ;
        }
        else
        {
            region_counter = i + 1;
            //classify left and right surfaces based upon path
            for(int j = 0; j < mfis[region_counter].size(); j++)
            {
                if(((isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP1()) > 0)
                    && (isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP2()) > 0))
                    || ((isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP1()) > 0)
                        && (isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP2()) > 0)))
                {
                    left_env.push_back(mfis[region_counter][j]);
                }
                else
                {
                    if(((isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP1()) < 0)
                        && (isLeft(Path[region_counter].getP1(), Path[region_counter].getP2(), mfis[region_counter][j].getP2()) < 0))
                        || ((isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP1()) < 0)
                            && (isLeft(Path[region_counter+1].getP1(), Path[region_counter+1].getP2(), mfis[region_counter][j].getP2()) < 0)))
                    {
                        right_env.push_back(mfis[region_counter][j]);
                    }
                }
            }
            
            temp = StructureDescription(left_env, right_env, combined_blocks[i], Path[region_counter]);
            rnt.push_back(temp);
        }
        
        //sprintf(fileName, "%s%d%s", "Maps/Offline/Final_structure_environment - ", i ,".png");
        //plotObjects(fileName, rnt[rnt.size()-1], combined_blocks[i]);
    }
    
    return rnt;
}



pair< vector<Object>, vector<Object> > Combine_origin_extend_regions(vector<Object> structure, vector<Object> structure2,
                                             vector<Object> mfis_before)
{
    int connect_flag = 0;
    
    vector<Object> temp1, temp2;
    vector<Object> trend_boundary, trend_boundary2;
    pair< vector<Object>, vector<Object> > rnt; 
    
    temp1 = structure;
    temp2 = structure2;
    
    //extend two adjacent region blocks
    for(int i = 0; i < structure.size(); i++)
    {
        if(interSectWithLine(structure2, structure[i].getP1(), structure[i].getP2()) == true)
        {
            connect_flag = 1;
            break;
        }
        else
            connect_flag = 2;
    }
    
    if(connect_flag == 2)
    {
        cout << " Two blocks are NOT intersected ." << endl;
 
        trend_boundary = extendRegionBlockToBoundary(mfis_before, structure, 1); //extend exit
        trend_boundary2 = extendRegionBlockToBoundary(mfis_before, structure2, 2); //extend entrance
        //connection_regions = ConnectionOfRegions(structure, structure2, trend_boundary, trend_boundary2);
        
        temp1 = addTwoVectorsOfObjectsNoduplicate(temp1, trend_boundary);
        temp2 = addTwoVectorsOfObjectsNoduplicate(temp2, trend_boundary2);

        //_StructureDescription(mfis4, structure, Path1);  
        //_StructureDescription(mfis4, structure2, Path2);
    }
    else
    {
        cout << " Two blocks are intersected ." << endl;
        //sprintf(fileName, "%s", "Maps/Offline/Extend_two_regions.png");
        //plotObjectsOf3Kinds(fileName, mfis9, structure, structure2);
        Object line1, line2;
        
        for(int m = 0; m < structure.size(); m++)
        {
            if(structure[m].getEntanceExitflag() == 2)
                line1 = structure[m];
        }
        
        for(int n = 0; n < structure2.size(); n++)
        {
            if(structure2[n].getEntanceExitflag() == 1)
                line2 = structure2[n];
        }   
    }
    
    rnt.first = temp1;
    rnt.second = temp2;
    
    return rnt;
}


vector< vector<Object> > Combine_extension_blocks(vector< pair< vector<Object>, vector<Object> > > extension_blocks)
{
    char fileName[80];
    vector<Object> temp;
    vector< vector<Object> > rnt;
    
    for(int i = 0; i < extension_blocks.size()-1; i++)
    {
       
        if(i == 0)
        {
            rnt.push_back(extension_blocks[i].first);
            temp = extension_blocks[i].second;
            temp = addTwoVectorsOfObjectsNoduplicate(temp, extension_blocks[i+1].first);
            //temp = addTwoVectorsOfObjects(temp, extension_blocks[i+1].first);
            rnt.push_back(temp);
        }
        else
        {
            temp = extension_blocks[i].second;
            temp = addTwoVectorsOfObjectsNoduplicate(temp, extension_blocks[i+1].first);
            //temp = addTwoVectorsOfObjects(temp, extension_blocks[i+1].first);
            rnt.push_back(temp);
        }
        temp.clear();
        
        
        //sprintf(fileName, "%s%d%s", "Maps/Offline/Final_combined_block - ", i ,".png");
        //plotObjects(fileName, rnt[rnt.size()-1]);
    }
    
    //the last one
    rnt.push_back(extension_blocks.back().second);
    
    return rnt;
}

/* based upon the entrance and path space, compute an area
 * surfaces in that area picked for describing the structure 
 */
vector<Object> environment_shape(vector<Object> region_mfis, vector<Object> path_space, Object entrance, int cnt)
{
    cout << " Environmental shape function !!" << endl;
    char fileName[80];
    double radius, distance_p1, distance_p2;
    Point center, ref_point;
    
    Object temp_obj;
    vector<Object> info, shape;
    
    center.set((entrance.X2() + entrance.X1()) / 2, (entrance.Y2() + entrance.Y1()) / 2);
    ref_point.set((path_space[0].X2() + path_space[1].X2()) / 2, (path_space[0].Y2() + path_space[1].Y2()) / 2);
    radius = distanceOftwoP(center, ref_point);
    
    //collect all potential surfaces in the area

    for(int i = 0; i < region_mfis.size(); i++)
    {
        distance_p1 =  distanceOftwoP(center, region_mfis[i].getP1());
        distance_p2 =  distanceOftwoP(center, region_mfis[i].getP2());
        if((distance_p1 < radius) || (distance_p2 < radius))
            info.push_back(region_mfis[i]);
    }

    //sprintf(fileName, "%s%d%s", "Maps/Offline/environment_shape - ", cnt ,".png");
    //plotObjectsOf3KindswithEllipse(fileName, region_mfis, info, center, radius*2, radius*2, 0);
    
    shape = info;
    for(int i = 0; i < info.size()- 1; i++)
    {
        temp_obj.set(info[i].X2(), info[i].Y2(), info[i+1].X1(), info[i+1].Y1(), i);
        if(temp_obj.length() < 4000)
        shape.push_back(temp_obj);
    }
    
    //sprintf(fileName, "%s%d%s", "Maps/Offline/environment_shape - ", cnt ,".png");
    //plotObjectsOf3Kinds(fileName, region_mfis, info, shape);
    
    return info;
}


vector< vector<Object> > environment_shape(vector< vector<Object> > MFISs, vector<Object> path, vector<Exit> exits)
{
    Object modified_path_segment;
    vector<Object> left_env, right_env;
    vector<Object> env_shape;
    vector<Object> path_back;
    vector<Object> region_info;
    vector< vector<Object> > all_region_shapes;
    vector< vector<Object> > mfis;
    
    mfis = MFISs;
    path_back = path;

    for(int i = 0; i < mfis.size(); i++)
    {
        if(mfis[i][0].get_region_remove_flag() == 1)
        {
            mfis.erase(mfis.begin()+i);
            
            modified_path_segment.set(path_back[i].X1(), path_back[i].Y1(), path_back[i+1].X2(), path_back[i+1].Y2(), i);
            path_back.insert(path_back.begin()+i+2, modified_path_segment);
            path_back.erase(path_back.begin()+i);
            path_back.erase(path_back.begin()+i);
            i = 0;
        }
    }

    for(int i = 0; i < mfis.size(); i++)
    {
        
        
        if(i == 0) //the first region, take the first exit only  
        {
            region_info = mfis[i];
            for(int j = 0; j < region_info.size(); j++)
            {
                if(((isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP1()) > 0)
                    && (isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP2()) > 0)))
                {
                    left_env.push_back(region_info[j]);
                }
                else
                {
                    if(((isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP1()) < 0)
                        && (isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP2()) < 0)))
                    {
                        right_env.push_back(region_info[j]);
                    }
                }
            }

            env_shape = StructureDescription(left_env, right_env, region_info, path_back[i]);
        }
        else //take the current exit and pre-exit 
        {
            region_info = mfis[i];
            region_info = addTwoVectorsOfObjects(region_info, mfis[i-1]);
            for(int j = 0; j < region_info.size(); j++)
            {
                if(((isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP1()) > 0)
                    && (isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP2()) > 0)))
                {
                    left_env.push_back(region_info[j]);
                }
                else
                {
                    if(((isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP1()) < 0)
                        && (isLeft(path_back[i].getP1(), path_back[i].getP2(), region_info[j].getP2()) < 0)))
                    {
                        right_env.push_back(region_info[j]);
                    }
                }
            }

            env_shape = StructureDescription(left_env, right_env, region_info, path_back[i]);
        }
        
        char fileName[80];
        vector<Exit> entr_exits;
        if(i == 0)
        {
            entr_exits.push_back(exits[i]);
        }
        else
        {
            entr_exits.push_back(exits[i]);
            entr_exits.push_back(exits[i-1]);
        }
        
        env_shape = clear_redundant(env_shape, entr_exits);
        sprintf(fileName, "%s%d%s", "Maps/Offline/environment_shape - ", i ,".png");
        plotObjectsOf3KindswithExits(fileName, env_shape, env_shape, entr_exits);
        
        left_env.clear();
        right_env.clear();
        region_info.clear();
        
        all_region_shapes.push_back(env_shape);
    }
    
    
    return all_region_shapes;
}

vector<Object> clear_redundant(vector<Object> env, vector<Exit> entr_exit)
{
    Object temp_obj, conver_entr, convert_exit;
    
    vector<Object> env_back;
    env_back = env;
    
    if(entr_exit.size() == 1)
        convert_exit.set(entr_exit[0].X1(), entr_exit[0].Y1(), entr_exit[0].X2(), entr_exit[0].Y2(), 0);
    else
    {
        conver_entr.set(entr_exit[0].X1(), entr_exit[0].Y1(), entr_exit[0].X2(), entr_exit[0].Y2(), 0);
        convert_exit.set(entr_exit[1].X1(), entr_exit[1].Y1(), entr_exit[1].X2(), entr_exit[1].Y2(), 0);
    }
    
    for(int i = 0; i < env_back.size(); i++)
    {
        if(entr_exit.size() > 1)
        {
            if((TwoObjectIntersect(env_back[i], conver_entr) == true)
                || (TwoObjectIntersect(env_back[i], convert_exit) == true)
                //|| (Intersect_number(env_back, env_back[i]) >= 3)
                    ||(interSectWithLine(env_back, env_back[i].getP1(), env_back[i].getP2()) == true))
            {
                    env_back.erase(env_back.begin()+i);
                    i--;
            }
        }
        else
        {
            if((TwoObjectIntersect(env_back[i], convert_exit) == true)
                //|| (Intersect_number(env_back, env_back[i]) >= 3)
                    ||(interSectWithLine(env_back, env_back[i].getP1(), env_back[i].getP2()) == true))
            {
                    env_back.erase(env_back.begin()+i);
                    i--;
            }
        }
    }
    
    return env_back;
}

vector<Object> be_project_region(vector<Object> current_chunk, vector<Object> last_chunk,
                                 vector<Exit> current_chunk_exits, vector<Exit> last_chunk_exits)
{ 
    vector<Object> space_area, overlap_area;
    vector<Object> inside_chunk_info, adjust_info;
    vector<Point_Hull> convex;
    vector<Exit> inside_exits;
    
    Point ref_point;
    double adjust_orient = 0;
    
    //overlapping region
    convex = ObjectToHull(current_chunk);
    space_area = ConvexToObject(convex);
    
    //exits inside overlapping region
    for(int i = 0; i < last_chunk_exits.size(); i++)
    {
        if((pointInPolygon(last_chunk_exits[i].getP1(), space_area) == true)
            || (pointInPolygon(last_chunk_exits[i].getP2(), space_area) == true))
        {
                inside_exits.push_back(last_chunk_exits[i]);
        }
        
    }
    /*
    //surface info inside overlapping region
    for(int i = 0; i < last_chunk.size(); i++)
    {
        if(((pointInPolygon(last_chunk[i].getP1(), space_area) == true)
            || (pointInPolygon(last_chunk[i].getP2(), space_area) == true))
            && (last_chunk[i].length() < 3000))
        {
                inside_chunk_info.push_back(last_chunk[i]);
        }
        
    }
    
    
    for(int i = 0; i < space_area.size(); i++)
    {
        if(space_area[i].length() > 4000)
        {
            space_area.erase(space_area.begin()+i);
            i--;
        }
        
    }
    */
    ref_point =         current_chunk[0].getP1();
    //adjust_info = TransformforToGlobalCoordinate(current_chunk, chunk_connect_parameters.second,
    //                                            current_chunk, chunk_connect_parameters.first);
    
    char fileName[80];
    sprintf(fileName, "%s", "Maps/Offline/Selected_information_for_project .png");
    plotObjectsOf3KindswithExits(fileName, current_chunk, last_chunk, inside_exits);
    //plotObjectsOf3KindswithExits(fileName, space_area, last_chunk, last_chunk_exits);
        
    //adjust each of surfaces from previous chunk, take current one as ref
    
    //adjust exit location 

    
}








