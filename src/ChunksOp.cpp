/*
 * To change this license header, choose License Headers in Project Properties.
 * Chunks Functions
 * By wenwang
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
#include "CompareASR.H"
#include "ConvexHull.h"
#include "GeometryAndExit.h"



#define  PI  3.14159265;

using namespace std;

int LastLNumber = 0;

LocalMap::LocalMap() {

}

LocalMap::~LocalMap() {

}

/*set relative coordinate local maps*/
void LocalMap::setRelativeMaps(vector< vector<Object> > individualMaps) {
    relativeCoordMaps = individualMaps;
}

/*get relative coordinate local maps*/
vector< vector<Object> > LocalMap::getRelativeMaps() 
{
    return relativeCoordMaps;
}

/*set global coordinate local maps*/
void LocalMap::setGlobalMaps(vector< vector<Object> > globalMaps) 
{
    gobalCoordMaps = globalMaps;
}

/*get global coordinate local maps*/
vector< vector<Object> > LocalMap::getGlobalMaps() 
{
    return gobalCoordMaps;
}

/*set Map ID*/
void LocalMap::setMapIDs(vector<int> mapId) 
{
    mapIDs = mapId;
}

/*get Map ID*/
vector<int> LocalMap::getMapIDs() 
{
    return mapIDs;
}

/*set all robot positions, gobal coordinate*/
void LocalMap::setRobPosition(vector< vector<Object> > robPosition) 
{
    robotPositions = robPosition;
}

/*get all robot positions*/
vector< vector<Object> > LocalMap::getRobPosition() 
{
    return robotPositions;
}

void LocalMap::setSepflag(unsigned int sepFlag) 
{
    seperate = sepFlag;
}

unsigned int LocalMap::getSepflag() 
{
    return seperate;
}

void LocalMap::setGlobalPositions(vector<Point> RbPositions) 
{
    RBPositions = RbPositions;
}

vector<Point> LocalMap::getGlobalPositions() 
{
    return RBPositions;
}

Chunk::Chunk() {

}

Chunk::~Chunk() {

}

/*set the view of chunk, combined using multi-local maps*/
void Chunk::setChunkView(vector<Object> comMap) 
{
    chunkMap = comMap;
}

/*get the view of chunk*/
vector<Object> Chunk::getChunkView() 
{
    return chunkMap;
}

/*set IDs corresponding to the local maps constructing the chunk*/
void Chunk::setIDS(vector<int> localMapID) 
{
    IDs = localMapID;
}

/*set IDs*/
vector<int> Chunk::getIDS() 
{
    return IDs;
}

/*set all asrs belonging its chunk*/
void Chunk::setASRs(vector< vector<Object> > allASR) 
{
    ASRs = allASR;
}

/*get all asrs belonging its chunk*/
vector< vector<Object> > Chunk::getASRS() 
{
    return ASRs;
}

/*set the connecting position, robot position*/
void Chunk::setConnection(Point connect) 
{
    connectionPoint = connect;
}

/*get the connecting position, robot position*/
Point Chunk::getConnection() 
{
    return connectionPoint;
}

/*set the trigger ID for generating this chunk*/
//void Chunk::setTriggerID(int trigger)
//{
//    deletionID = trigger;
//}
            
/*get the trigger ID for generating this chunk*/
//int Chunk::getTriggerID()
//{
//    return deletionID;
//}


void Map::setMap(vector<Object> map)
{
    local_map = map;
}
vector<Object> Map::getMap()
{
    return local_map;
}

void Map::setMapID(int mapID)
{
    map_ID = mapID;
}
int Map::getMapID()
{
    return map_ID;
}

void Map::setExitPositon(vector<Object> exitPosition)
{
    exit_position = exitPosition;
}
vector<Object> Map::getExitPosition()
{
    return exit_position;
}
Point Map::getOriginPosition()
{
    origin.set(0,0);
    return origin;
}

void Map::setHeading(double robotHeading)
{
    headingAngle = robotHeading;
}
double Map::getHeading()
{
    return headingAngle;
}

Point Map::getOrientation()
{
    Point rnt;
    
        double x, y, a, b;
        double dist, angle;
        double rad;
    
        rad = headingAngle * 3.1415926 / 180;
        angle = PI + rad;
        
        if(angle > 2 * 3.1415926)
            angle = angle - 2 * 3.1415926;

        dist = GetPointDistance(origin, Point (exit_position[6].X1(), exit_position[6].Y1()));

        a = origin.X() - exit_position[6].X1(); //x-rp
        b = origin.Y() - exit_position[6].Y1(); //y-rp

        x = a * cos(angle) + b * sin(angle);
        y = b * cos(angle) - a * sin(angle);
           
        rnt.set(x,y);
    return rnt;
}


void ChunkInfo::setChunkInfo(vector<Object> mfis, vector<Exit> exit, vector< vector<Object> > robots, 
                      vector< vector<Object> > views_chunk, vector< vector<Object> > MFIS_regions)
{
    this->whole_mfis = mfis;
    this->compute_exits = exit;
    this->views = views_chunk;
    this->robot = robots;
    this->region = MFIS_regions;
}

vector<Exit> ChunkInfo::getExits()
{
    return compute_exits;
}
vector<Object> ChunkInfo::getMFIS()
{
    return whole_mfis;
}
            
vector< vector<Object> > ChunkInfo::getRobots()
{
    return robot;
}
vector< vector<Object> > ChunkInfo::getViews()
{
    return views;
}
vector< vector<Object> > ChunkInfo::getMFIS_region()
{
    return region;
}

void ChunkInfo::setShiftExits(vector<Exit> modified_exits)
{
    shifted_pre_chunk_exits = modified_exits;
}

vector<Exit> ChunkInfo::getShiftExits()
{
    return shifted_pre_chunk_exits;
}


/* Based on LMs overlapping theory
 * Calculating the polygon of each local map
 * Calculating whether the current local map is intersected
 * with previous local maps
 */
void GenerateChunks(vector< vector<Object> > LMs, int viewNumber) 
{
           vector<Surface> polygon1, polygon2;
           vector<Object> CkView;
           Chunk currentChunk;
           char chunkFileName[80];

           polygon1 = makePolygonOfCV(LMs[LMs.size() - 1]);

           for (int i = LastLNumber; i < LMs.size() - 1; i++) 
           {
                      for (int j = 0; j < LMs[i].size(); j++) 
                      {
                                 //temp.set(LMs[i][j].X1(), LMs[i][j].Y1());
                                 if (pointInPolygon(PointXY(LMs[i][j].X1(), LMs[i][j].Y1()), polygon1) == true ||
                                        pointInPolygon(PointXY(LMs[i][j].X2(), LMs[i][j].Y2()), polygon1) == true) 
                                 {
                                            //Combie LM 1 --- (size - 2), next chunk starts from (size - 1)
                                            CkView = LMs[LastLNumber];
                                            for (int m = LastLNumber; m < LMs.size() - 2; m++)
                                            {
                                                CkView = addTwoVectorsOfObjects(CkView, LMs[m]);
                                            }
                                            currentChunk.setChunkView(CkView);
                                            LastLNumber = LMs.size() - 1;
                                            sprintf(chunkFileName, "%s%d%s", "Maps/Offline/LocalMapChunks-", viewNumber, ".png");
                                            plotObjects(chunkFileName, CkView);
                                            i = j = 100;
                                            break;
                                 }
                      }
           }
}

/*Current Local Map is on the global coordinate system
 * checking whether the previous 5-10 steps, robot positions, are in current LM
 * checking whether the previous rp where it obtained old LMs are in current LM
 */
void MoveBackDetection(vector<Object> currentLM, vector<Point> PrePositions, Point currentRP, int stepNum) 
{
           //cout<<"----- This is going to process visited LM detection -----"<<endl;
           //waitHere();
           char polygonFileName[80];
           Point p1, p2, temp;
           double error_angle = 0; //how amount the angle need to adjust

           vector<Surface> polygon = makePolygonOfLM(currentLM);
           vector<Object> polygonOBJ = convertSurfaceToObject(polygon);
           sprintf(polygonFileName, "%s%d%s", "Maps/Offline/Polygon-", stepNum, ".png");
           plotObjects(polygonFileName, polygonOBJ);

           //initialize the p1 & p2
           p1.set(currentRP.X(), currentRP.Y());
           cout << "the size of PrePositions is : " << PrePositions.size() << endl;
           //waitHere();
           for (int i = PrePositions.size() - 1; i >= 0; i--) 
           {
                      //cout<<"----test programme----"<<endl;
                      //waitHere();
                      p2.set(PrePositions[i].X(), PrePositions[i].Y());
                      //if(interSectWithLine(currentLM, p1, p2) == false)
                      if (pointInPolygon(PointXY(p2.X(), p2.Y()), polygon) == true) 
                      {

                                 cout << "----This position is in current LM----" << endl;
                                 cout << " The step number is : " << stepNum << endl;
                                 //waitHere();
                      } 
                      else
                      {
                                 //not in this current LM
                                 cout << "----This position is not in current LM----" << endl;
                                 //waitHere();
                      }
           }

            //return particular information angle and distance(point coordinate)
            //for fitting two chunks together
}

/*Cluster the surfaces
 *abstract the structure of All ASRs/ Chunks
 */
vector< vector<Object> > ClusterOfChunks(vector<Object> ASRs) 
{
           cout << "----- This is to cluster surfaces of a Chunk -----" << endl << endl;
           Object temp_Obj; //temporary object
           vector<Object> group; //group objects
           vector< vector<Object> > Cluster; //all groups
           double slope_threshold = 10; //slope threshold between two objects
           double dist_threshold = 1000; //gap threshold between two object
           double k1, k2, gap_dist;
           Point mid1, mid2; //mid-point
           int i = 0; //start ID of a group
           int nextNum = 0; //for next start ID of a group
           unsigned char flag = 0;
           unsigned char finish_flag = 0; //process finish flag 
           unsigned char PointsNear_flag = 0;

           //for(int i = 0; i < ASRs.size() - 1; i ++)
lb1:    while (finish_flag != 1) 
           {
                      temp_Obj = ASRs[i];
                      group.push_back(temp_Obj); //the first element of the clustering 

                      for (int j = i + 1; j < ASRs.size(); j++) 
                      {
                                 //k2 = (ASRs[j].Y2() - ASRs[j].Y1()) / (ASRs[j].X2() - ASRs[j].X1());
                                 mid2.set((ASRs[j].X2() + ASRs[j].X1()) / 2, (ASRs[j].Y2() + ASRs[j].Y1()) / 2);
 
                                 if (group.size() >= 2) 
                                 {
                                            //cout<<"---- This group has more than 2 objects now ----"<<endl;
                                            //cout<<"Current number of group : "<<group.size()<<endl;
                                            for (int n = 0; n < group.size(); n++) 
                                            {
                                                       //k1 = (group[n].Y2() - group[n].Y1()) / (group[n].X2() - group[n].X1());
                                                       mid1.set((group[n].X2() + group[n].X1()) / 2, (group[n].Y2() + group[n].Y1()) / 2);
                                                       gap_dist = GetPointDistance(mid1, mid2);
                                                       if (TwoObjectIntersect(group[n], ASRs[j]) == true) 
                                                       {
                                                                  group.push_back(ASRs[j]);
                                                                  ASRs.erase(ASRs.begin() + j);
                                                                  j--;
                                                                  if (j == ASRs.size() - 1) 
                                                                  {
                                                                             finish_flag = 1;
                                                                             goto lb1;
                                                                  }
                                                                  break;
                                                                  //goto lb2;
                                                       } 
                                                       else 
                                                       {
                                                                  if (ASRs[j].distP1ToP1(ASRs[i]) < dist_threshold || ASRs[j].distP1ToP2(ASRs[i]) < dist_threshold
                                                                            || ASRs[j].distP2ToP1(ASRs[i]) < dist_threshold || ASRs[j].distP2ToP2(ASRs[i]) < dist_threshold) 
                                                                  {
                                                                             if (PointsNear_flag = 0)
                                                                                        PointsNear_flag = 1;
                                                                    
                                                                             //if(abs(k1 - k2) < slope_threshold  && gap_dist < dist_threshold)
                                                                             if (gap_dist <= dist_threshold || PointsNear_flag == 1) 
                                                                             {
                                                                                        group.push_back(ASRs[j]);
                                                                                        ASRs.erase(ASRs.begin() + j);
                                                                                        j--;
                                                                                        PointsNear_flag = 0;
                                                                                        if (j == ASRs.size() - 1) 
                                                                                        {
                                                                                                   finish_flag = 1;
                                                                                                   goto lb1;
                                                                                        }
                                                                                        break;
                                                                                        //goto lb2;
                                                                             } 
                                                                             else 
                                                                             {
                                                                                        if (flag == 0 && n == group.size() - 1) 
                                                                                        {
                                                                                                  nextNum = j;
                                                                                                  flag = 1;
                                                                                        }
                                                                             }
                                                                  }
                                                       }
                                            }
                                 } 
                                 else
                                 {
                                            //cout<<"---- This group has only one object now ----"<<endl;
                                            //k1 = (group[0].Y2() - group[0].Y1()) / (group[0].X2() - group[0].X1());
                                            mid1.set((group[0].X2() + group[0].X1()) / 2, (group[0].Y2() + group[0].Y1()) / 2);
                                            gap_dist = GetPointDistance(mid1, mid2);
                                            if (TwoObjectIntersect(group[0], ASRs[j]) == true) 
                                            {
                                                       group.push_back(ASRs[j]);
                                                       ASRs.erase(ASRs.begin() + j);
                                                       j--;
                                                       if (j == ASRs.size() - 1)
                                                       {
                                                                  finish_flag = 1;
                                                                  goto lb1;
                                                       }
                                                       break;
                                            } 
                                            else 
                                            {
                                                       if (ASRs[j].distP1ToP1(ASRs[i]) < dist_threshold || ASRs[j].distP1ToP2(ASRs[i]) < dist_threshold
                                                                || ASRs[j].distP2ToP1(ASRs[i]) < dist_threshold || ASRs[j].distP2ToP2(ASRs[i]) < dist_threshold) 
                                                       {
                                                                  if (PointsNear_flag = 0)
                                                                             PointsNear_flag = 1;
                                                       }

                                                       if (gap_dist <= dist_threshold || PointsNear_flag == 1) 
                                                       {
                                                                  group.push_back(ASRs[j]);
                                                                  ASRs.erase(ASRs.begin() + j);
                                                                  j--;
                                                                  PointsNear_flag = 0;
                                                                  if (j == ASRs.size() - 1) 
                                                                  {
                                                                             finish_flag = 1;
                                                                             goto lb1;
                                                                  } 
                                                                  break;
                                                       } 
                                                       else 
                                                       {
                                                                  if (flag == 0) 
                                                                  {
                                                                             nextNum = j;
                                                                             flag = 1;
                                                                  }
                                                       }

                                            }
                                 }

                                 if (j == ASRs.size() - 1) 
                                 {
                                            Cluster.push_back(group);
                                            //i = nextNum;
                                            i = 0;
                                            flag = 0;
                                            ASRs.erase(ASRs.begin());
                                            group.clear();
                                 }
                       }

           }

           return Cluster;
}

/*This function is to generate an equal width path
 *following the robot positions for each of Chunk
 */
vector<Object> TraversePath(vector<Object> completePath, Point coord, double angle, int num) 
{
           Object left_Obj, right_Obj;
           Object temp_Obj;

           vector<Object> temp;
           vector<Object> crb; //useless variable
           int i = 0;

           //initial the two objects
           left_Obj.set(-500, 0, -500, 500, -1);
           right_Obj.set(500, 0, 500, 500, 1);
           temp.push_back(left_Obj);
           temp.push_back(right_Obj);

           temp = Globalview(temp, coord, crb, angle, num);

           while (i < temp.size()) 
           {
                      completePath.push_back(temp[i]);
                      i++;
           }

           return completePath;
}


/////////////////// test function ////////////////////////////////

pair<Point, double> constructInfo(vector<Object> part1, vector<Object> part2) 
{
           Object obj1, obj2, obj3, obj4;
           pair<Point, double> temp;
           int max = 0;
           double ang1, ang2;
           Point mid1, mid2, obj_trans;
           char testFileName[80];

           for (int i = 0; i < part1.size(); i++) 
           {
                      if (part1[i].X1() > max)
                      {
                                 max = part1[i].X1();
                                 obj3 = part1[i];
                      }
                      if (part1[i].X2() > max) 
                      {
                                 max = part1[i].X2();
                                 obj3 = part1[i];
                      }
           }
            mid1.set((obj3.X1() + obj3.X2()) / 2, (obj3.Y1() + obj3.Y2()) / 2);
            ang1 = acos(abs(obj3.X2() - obj3.X1()) / obj3.length());

            for (int i = 0; i < part2.size(); i++)
            {
                      if (part2[i].X1() > max) 
                      {
                                 max = part2[i].X1();
                                 obj4 = part2[i];
                      }
                      if (part2[i].X2() > max) 
                      {
                                 max = part2[i].X2();
                                 obj4 = part2[i];
                      }
            }
            mid2.set((obj4.X1() + obj4.X2()) / 2, (obj4.Y1() + obj4.Y2()) / 2);
            ang2 = acos(abs(obj4.X2() - obj4.X1()) / obj4.length());

            temp.second = ang2 - ang1;

            mid1.set(mid1.X() * cos(temp.second) - mid1.Y() * sin(temp.second), mid1.Y() * cos(temp.second) + mid1.X() * sin(temp.second));
            //obj_trans.set(obj3.X1() * cos(temp.second) - obj3.Y1() * sin(temp.second), obj3.Y1() * cos(temp.second) + obj3.X1() * sin(temp.second));

            temp.first.set(mid2.X() - mid1.X(), mid2.Y() - mid1.Y());
            //temp.first.set(obj4.X1() - obj_trans.X(), obj4.Y1() - obj_trans.Y());

            //ang1 = ang1* 180 / PI;
            //ang2 = ang2 * 180 / PI;
            //cout<<"the angle 1 "<<ang1 <<endl;
            //cout<<"the angle 2 "<<ang2 <<endl;
            //waitHere(); 


            return temp;
}


//vector<Object> reOrganise(vector<Object> finalGM)

void reOrganise(vector<Object> finalGM) 
{
           char GMfileName[80];
           vector<Object> temp1, temp2, final;
           temp1.push_back(finalGM[0]);
           int lastNum = 0;
           int nextNum = 0;
           unsigned char sepFlag = 0;
           pair<Point, double> coordInfo;

           //for the first part of a global map 
           for (int i = 1; i < finalGM.size() - 1; i++) 
           {
                      if (i - lastNum < 2) 
                      {
                                 if (abs(finalGM[i].getLocalEnvID()[0] - temp1.back().getLocalEnvID()[0]) < 20) 
                                 {
                                            temp1.push_back(finalGM[i]);
                                            lastNum = i;
                                 } 
                                 else
                                            if (sepFlag == 0) 
                                            {
                                                       nextNum = i;
                                                       sepFlag = 1;
                                            }
                      } 
                      else 
                      {
                                 for (int j = 0; j < temp1.size(); j++) 
                                 {
                                            if (abs(finalGM[i].getLocalEnvID()[0] - temp1[j].getLocalEnvID()[0]) < 10) 
                                            {
                                                       temp1.push_back(finalGM[i]);
                                                       lastNum = i;
                                                       break;
                                            }
                                 }
                      }
           }


           sprintf(GMfileName, "%s%d%s", "Maps/Chunk/part-", 1, ".png");
           plotObjects(GMfileName, temp1);
           sepFlag = 0;

           //for the second part of a global map 
           temp2.push_back(finalGM[nextNum]);
           for (int i = nextNum + 1; i < finalGM.size() - 1; i++) 
           {
                      if (i - lastNum < 2) 
                      {
                                 if (abs(finalGM[i].getLocalEnvID()[0] - temp2.back().getLocalEnvID()[0]) < 20) 
                                 {
                                            temp2.push_back(finalGM[i]);
                                            lastNum = i;
                                 }
                                 //else 
                                 //           if(sepFlag == 0)
                                 //            {
                                 //                       nextNum = i;
                                 //                      sepFlag = 1;
                                 //           }
                      } 
                      else 
                      {
                                 for (int j = 0; j < temp2.size(); j++) 
                                 {
                                            if (abs(finalGM[i].getLocalEnvID()[0] - temp2[j].getLocalEnvID()[0]) < 20) 
                                            {
                                                       temp2.push_back(finalGM[i]);
                                                       lastNum = i;
                                                       break;
                                            }
                                 }
                      }
           }

            //cout<<"ids : ";
            //for(int n = 0; n < temp2.size(); n++)
            //cout<<" "<<temp2[n].getLocalEnvID()[0];

           sprintf(GMfileName, "%s%d%s", "Maps/Chunk/part-", 2, ".png");
           plotObjects(GMfileName, temp2);
           
/*
           ////modified 
           vector<Object> restOf;
           restOf = deleOneVectorOfObject(finalGM, temp1);
           vector< vector<Object> > color;
           color.push_back(temp1);
           color.push_back(restOf);
           sprintf(GMfileName, "%s", "Maps/Chunk/ColorMFIS.png");
           plotObjectsColours(GMfileName, color);
           ///////////
*/
           //coordinate information for transform
           coordInfo = constructInfo(temp1, temp2);

           //part 1 transform onto part 2
           //temp1 = xformPVIntoCV(temp1, coordInfo.first, coordInfo.second);
           for (int i = 0; i < temp1.size(); i++) 
           {
                      int x1, y1, x2, y2;
                      Object temp_obj;
                      x1 = temp1[i].X1() * cos(coordInfo.second) - temp1[i].Y1() * sin(coordInfo.second) + coordInfo.first.X();
                      y1 = temp1[i].Y1() * cos(coordInfo.second) + temp1[i].X1() * sin(coordInfo.second) + coordInfo.first.Y();

                      x2 = temp1[i].X2() * cos(coordInfo.second) - temp1[i].Y2() * sin(coordInfo.second) + coordInfo.first.X();
                      y2 = temp1[i].Y2() * cos(coordInfo.second) + temp1[i].X2() * sin(coordInfo.second) + coordInfo.first.Y();
                      temp_obj.set(x1, y1, x2, y2, 0);
                      final.push_back(temp_obj);
           }


           final = addTwoVectorsOfObjects(final, temp2);

           sprintf(GMfileName, "%s%d%s", "Maps/Chunk/Final-", 0, ".png");
           plotObjects(GMfileName, final);
}

//cirCenter is the centre of circle, cirR is radius, s this starting of the arc, n is the end of the arc
// s from 0 or < 2    mix of n is 2    n * PI = 360 degree 

void IntersectedWithArc(Point cirCenter, double cirR, vector<Object> CV, double s, double n) {
    //const double PI=3.1415926;
    Point p1, p2; // line segment P1 & P2
    unsigned char flag = 0;
    double thete = 0;
    double x, y, a, b;
    double start = s;
    double step = 1 / 180 * PI;
    double range = n * PI;

    for (int i = 0; i < CV.size(); i++) {

        p1.set(CV[i].X1(), CV[i].Y1());
        p2.set(CV[i].X2(), CV[i].Y2());
        a = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
        b = (p1.X() * p2.Y() - p2.X() * p1.Y()) / (p1.X() - p2.X());

        for (thete = start; thete <= range; thete += step) {
            x = cirR * cos(thete) + cirCenter.X();
            y = cirR * sin(thete) + cirCenter.Y(); //x,y

            if (fabs(y - a * x - b) < 1e-5) //y=ax+b
            {
                if ((p1.X() <= x && x <= p2.X() || p1.X() >= x && x >= p2.X())
                        && (p1.Y() <= y && y <= p2.Y() || p1.Y() >= y && y >= p2.Y())) //
                {
                    //the arc is intesected with a surface       
                    flag = 1;
                }
            }
        }
    }

}


/*Try to sketch box-shape to describe the structure
 *
 */
/*
pair<double, double> scaleOfsketch(vector<Object> view)
{
           double baseValue = 1000;
           double width  = abs(view[0].X1() -view.back().X2());
           double max = 0;
           double length = 0;
           pair<double, double> rnt;
           
           for(int i = 0; i < view.size(); i++)  
           {
                      if(view[i].Y1() > max)
                                 max = view[i].Y1();
                      if(view[i].Y2() > max)
                                 max = view[i].Y2();
           }
           
           length = max;
           
           rnt.first = width;
           rnt.second = length;
           
           return rnt;
}
 */
/*
void SketchMap(vector<Object> view, vector<Object> Gview, Point pos, double heading)
{
           vector<Object> largeObject;
           vector<Object> sketchView;
           double threshold = 1000;
           //double baseValue = 1000;
           Point p1, p2, p3, p4;
           Object temp, baseline;
           unsigned char large_flag = 0;
           unsigned char baseline_flag  = 0; // 0 - horizon 1- vertical

           //establish the baseline, for searching the width
           if(heading > 30 || heading < -30)
                      baseline_flag  = 1; //vertical baseline 
           else
                      baseline_flag = 0; //horizontal baseline
           /*
           if(baseline_flag == 0)
           {
                      if()
                                 baseline.set(pos.X() - baseValue, pos.Y(), pos.X() + baseValue, pos.Y(), 0);
                      else
                            if()
                                 baseline.set(pos.X() - baseValue, pos.Y(), pos.X() + baseValue, pos.Y(), 0);
           }
           else
           {
                      if(heading > 0)
                                 baseline.set(pos.X() - 500, pos.Y() - baseValue, pos.X() - 500, pos.Y() + baseValue, 0);
                      else
                            if(heading < 0)
                                baseline.set(pos.X() + 500, pos.Y() - baseValue, pos.X() + 500, pos.Y() + baseValue, 0);
           }
 */


/*           
           for(int i = 0; i < Gview.size(); i++)
           {
                      //scan the large surfaces, and save them from a view
                      if(Gview[i].length() >= threshold)
                      {
                                 largeObject.push_back(Gview[i]);
                                 large_flag = 1;
                      }
                      
                      //scan four surfaces/point, to identify the boundary
           }
           
           //if there is large surface
           if(large_flag == 1)
           {
                      // establish a T-shape box
                      
           }
           else
           {
                      // establish a squire-shape box
           }
           
         
}
 */

 /*
//input is chunk 2, all local maps
//vector<Object> OrganiseLocalMaps(LocalMap LMs)
void  OrganiseLocalMaps(LocalMap LMs)
{
           Object obj1, obj2, temp_obj;
           double dist = 10000;
           double ang1, ang2, DifAng;
           vector<Object> rtn, part1, part2, final;
           Point pot;
           
            char GMfileName[80];
           
           part1 = LMs.getGlobalMaps()[0];
           
           obj1 = LMs.getGlobalMaps()[1][0]; //the first object of second LM

           //extract the second object from the first LM
           for(int i = 0; i < LMs.getGlobalMaps()[0].size(); i++)
           {
                      if(LMs.getGlobalMaps()[0][i].distP2ToPoint(obj1.X1(), obj1.Y1()) < dist)
                      {
                                 dist = LMs.getGlobalMaps()[0][i].distP2ToPoint(obj1.X1(), obj1.Y1());
                                 obj2 = LMs.getGlobalMaps()[0][i];
                      }
           }
           
           //difference of angle between two objects
           ang1 = acos(abs(obj1.X2() - obj1.X1()) / obj1.length());
           ang2 = acos(abs(obj2.X2() - obj2.X1()) / obj2.length());
           
           DifAng = ang1 - ang2;
           //temp.set(obj1.X1() * cos(DifAng) - obj1.Y1() * sin(DifAng), obj1.Y1() * cos(DifAng) + obj1.X1() * sin(DifAng), obj1.X2() * cos(DifAng) - obj1.Y2() * sin(DifAng), obj1.Y2() * cos(DifAng) + obj1.X2() * sin(DifAng),0);
           
           pot.set(obj2.X1() - obj2.X1(),  obj2.Y1() - obj1.Y1());
           
           part2 = LMs.getGlobalMaps()[1];
           
           for(int j = 2; j < LMs.getGlobalMaps().size(); j++)
           {
               part2 = addTwoVectorsOfObjects(part2, LMs.getGlobalMaps()[j]);
           }
           
           //transform all informaiton onto another coordinate
           for(int i = 0; i < part2.size(); i++)
           {
                      int x1, y1, x2, y2;
                      Object temp_obj;
                      x1=part2[i].X1() * cos(DifAng) - part2[i].Y1() * sin(DifAng) + pot.X();
                      y1=part2[i].Y1() * cos(DifAng) + part2[i].X1() * sin(DifAng) +pot.Y();
                
                      x2=part2[i].X2() * cos(DifAng) - part2[i].Y2() * sin(DifAng) + pot.X();
                      y2=part2[i].Y2() * cos(DifAng) + part2[i].X2() * sin(DifAng) + pot.Y();
                      temp_obj.set(x1,y1,x2,y2,0);
                      final.push_back(temp_obj);
           }
           
           rtn = addTwoVectorsOfObjects(part1, part2);
           
           sprintf(GMfileName,"%s%d%s", "Maps/Chunk/Chunk2Reorgainsed-", 0, ".png");
           plotObjects(GMfileName, rtn);
           
           //return rtn;
        
}


void reOrganiseChunks(vector<Object> chunk1, vector<Object> chunk2)
{
           double Xmax = 0;
           double Ymax = 0;
           double ang1, ang2, DifAng, dist;
           Object obj1, obj2, temp;
           Point pot;
           vector<Object> final, test;
           char GMfileName[80];

           //finding the right bottom object in chunk1
           for(int i = 0; i < chunk1.size(); i++)
           {
                      if(chunk1[i].Y1() <= Ymax)
                      {
                                 obj1 = chunk1[i];
                                 Ymax = chunk1[i].Y1();
                      }
                      if(chunk1[i].Y2() <= Ymax)
                      {
                                 obj1 = chunk1[i];
                                 Ymax = chunk1[i].Y2();
                      }
           }

           //finding the right bottom object in chunk2
           //actually the starting surface should be the same 
           obj2 = chunk2[0];
           
           //test.push_back(obj1);
           //test.push_back(obj2);
           //sprintf(GMfileName,"%s%d%s", "Maps/Chunk/testObjeects-", 1, ".png");
           //plotObjects(GMfileName, test);
           
           //difference of angle between two objects
           ang1 = acos(abs(obj1.X2() - obj1.X1()) / obj1.length());
           ang2 = acos(abs(obj2.X2() - obj2.X1()) / obj2.length());
           
           DifAng = ang1 - ang2;
           temp.set(obj1.X1() * cos(DifAng) - obj1.Y1() * sin(DifAng), obj1.Y1() * cos(DifAng) + obj1.X1() * sin(DifAng), obj1.X2() * cos(DifAng) - obj1.Y2() * sin(DifAng), obj1.Y2() * cos(DifAng) + obj1.X2() * sin(DifAng),0);
           
           pot.set(obj2.X1() - temp.X2(),  obj2.Y1() - temp.Y2());
           
           //transform all informaiton onto another coordinate
            for(int i = 0; i < chunk1.size(); i++)
           {
                      int x1, y1, x2, y2;
                      Object temp_obj;
                      x1=chunk1[i].X1() * cos(DifAng) - chunk1[i].Y1() * sin(DifAng) + pot.X();
                      y1=chunk1[i].Y1() * cos(DifAng) + chunk1[i].X1() * sin(DifAng) +pot.Y();
                
                      x2=chunk1[i].X2() * cos(DifAng) - chunk1[i].Y2() * sin(DifAng) + pot.X();
                      y2=chunk1[i].Y2() * cos(DifAng) + chunk1[i].X2() * sin(DifAng) + pot.Y();
                      temp_obj.set(x1,y1,x2,y2,0);
                      final.push_back(temp_obj);
           }
           
           sprintf(GMfileName,"%s%d%s", "Maps/Chunk/FinalChunk-", 1, ".png");
           plotObjects(GMfileName, final);
           sprintf(GMfileName,"%s%d%s", "Maps/Chunk/FinalChunk-", 2, ".png");
           plotObjects(GMfileName, chunk2);
           sprintf(GMfileName,"%s%d%s", "Maps/Chunk/FinalChunk-", 0, ".png");
           plotObjects(GMfileName, chunk2, final);
}
//*****************************************************************************/



/*Through the deletion to identify 
 *Then split a chunk into two parts for adjusting
 */
void SplitChunk(vector<Object> Chunk1, int DeletionID, vector<Point> robPositions)
{
           vector<Object> part1, part2;
           unsigned char floating_flag = 0;
           int floating_num;
           char sperateFileName[80];

           //split the the chunk view by using the ID of deletion information
           for(int i = 0; i < Chunk1.size(); i++)
           {
                      if(Chunk1[i].getLocalEnvID()[0] < DeletionID)
                                 part1.push_back(Chunk1[i]);
                      else
                      {
                                 part2.push_back(Chunk1[i]); // floating ASRs/LMs
                                 if(floating_flag == 0)
                                 {
                                            floating_flag = 1;
                                            floating_num = i; // for processing the robot positions
                                 }  
                      }
           }
           
           
           //take the floating part2 to connect with chunk2
           
           
           //finally take the floating part 1 to connect the combined map
           
    
}


void TransformViewToEllipseBoundary(vector<Object> currentView)
{
    //the farthest point
    Point temp;
    Point midPoint_Ellipse;
    Object diameter_w;
    double orientation = 0;
    double farthest = 0;
    double heigth = 0;
    char ellipseFileName[80];
    MyRobot myrobot(0,0);
    
    
    for(int i = 0; i < currentView.size(); i++)
    {

            if(currentView[i].Y1() > farthest)
            {
                    farthest = currentView[i].Y1();
                    temp.set(currentView[i].X1(), currentView[i].Y1());
            }
            if(currentView[i].Y2() > farthest )
            {
                    farthest = currentView[i].Y2();
                    temp.set(currentView[i].X2(), currentView[i].Y2());
            }
    }

    
    //line joining the origin and farthest point 
    diameter_w.set(0,0,temp.X(),temp.Y(),1);
    midPoint_Ellipse.set(temp.X() / 2, temp.Y() / 2);
    

    //the height of ellipse
    farthest = 0;
    for(int i = 0; i < currentView.size(); i++)
    {
            farthest = Perpendiculardistance(Point (currentView[i].X1(), currentView[i].Y1()),
                                          Point (diameter_w.X1(), diameter_w.Y1()), Point (diameter_w.X2(), diameter_w.Y2())) ;
        
            if(currentView[i].Y1() > farthest)
            {
                    farthest = currentView[i].Y1();
                    temp.set(currentView[i].X1(), currentView[i].Y1());
            }
            
            farthest = Perpendiculardistance(Point (currentView[i].X2(), currentView[i].Y2()),
                                          Point (diameter_w.X1(), diameter_w.Y1()), Point (diameter_w.X2(), diameter_w.Y2())) ;
        
            if(currentView[i].Y2() > farthest )
            {
                    farthest = currentView[i].Y2();
                    temp.set(currentView[i].X2(), currentView[i].Y2());
            }
    }
    heigth = farthest; 
    //orientation of ellipse
    orientation = diameter_w.getAngleWithXaxis();
    //cout << " the orientation is : " << orientation << endl;
    
    sprintf(ellipseFileName, "%s%d%s", "Maps/Offline/View-", 1, ".png");
    plotObjectsOf3KindswithEllipse(ellipseFileName, currentView, myrobot.getRobot(), 
                  midPoint_Ellipse, diameter_w.length(), heigth, orientation);
   
}


/*
void SketchMap(vector<Object> map, vector< vector<Object> > listOfRP)
{
    Object temp_Obj; //temp object for view
    Object path_segment; //individual path segment
    vector<Object> temp; //final sketch map
    vector<Object> wholePath; //whole simple path 
    
    double deviation = 0;
    double dist_threshold  = 4000;
    Point temp_p;
    vector<Point> path_points;
    unsigned char heading_flag = 0; //0 perpendicular x axis, 1 parallel x axis
    char fileName[80];
    temp_p.set(0,0);
    path_points.push_back(temp_p);
    
    //find turning point so that obtain simply path segments
    for(int i = 0; i < listOfRP.size(); i++)
    {
        if(heading_flag == 0 && listOfRP[i][6].getAngleWithXaxis() < 15)
        {
            if(listOfRP[i][6].getAngleWithXaxis() < 180)
                deviation = 180 - listOfRP[i][6].getAngleWithXaxis();
            else
            {
                if(listOfRP[i][6].getAngleWithXaxis() > 180)
                    deviation = 360 - listOfRP[i][6].getAngleWithXaxis();
            }
            
            if()
            //record this position
            temp_p.set(listOfRP[i][6].X1(), listOfRP[i][6].Y1());
            cout << " the point is : " << listOfRP[i][6].X1()<< " ; and : " << listOfRP[i][6].Y1() << endl;
            path_points.push_back(temp_p); 
            heading_flag = 1;
        }
        else
        {
            if(heading_flag == 1 && listOfRP[i][6].getAngleWithXaxis() > 75)
            {
                //record this position
                temp_p.set(listOfRP[i][6].X1(), listOfRP[i][6].Y1());
                cout << " the point is : " << listOfRP[i][6].X1()<< " ; and : " << listOfRP[i][6].Y1() << endl;
                path_points.push_back(temp_p);    
                heading_flag = 0;
            }
        }
    }
    
    cout << " the size of points : " << path_points.size() << endl;
    
    //establish path lines as objects for plotting 
    for(int i = 0 ; i < path_points.size() - 1; i++)
    {
          temp_Obj.set(path_points[i].X(), path_points[i].Y(),
                        path_points[i+1].X(), path_points[i+1].X(), i);
          
          //split path segment if so long 
          
          temp.push_back(temp_Obj);
    }
    
    //gain the farthest point two sides/ left and right
    
    
    cout << " the size of path lines : " << temp.size() << endl;
    
    //plot result map/chunk map
    sprintf(fileName, "%s", "Maps/Offline/TEST_PATH.png");
    plotObjects(fileName, map, temp);
}
*/

void MaxSpaceFollowPath(vector<Object> chunkMap, vector< vector<Object> > listOfRP)
{
    unsigned char heading_flag = 0; //0 perpendicular x axis, 1 parallel x axis, initially the robot facing is y axis
    double deviation = 0;
    char fileName[80];
    Point temp_p;

    Object temp_Obj; //temp object for view
    vector<Object> temp; 
    vector<Object> cluster_along_path;
    vector<Point> path_points;
    temp_p.set(listOfRP[0][6].X1(), listOfRP[0][6].Y1());
    path_points.push_back(temp_p);
    
    //initial heading_flag
    
        
    //find turning point so that obtain simply path segments
    //cout << " the first position face : " << listOfRP[0][6].getAngleWithXaxis() << endl;
    for(int i = 1; i < listOfRP.size(); i++)
    {
        if(heading_flag == 0)
        {
            if(listOfRP[i][6].getAngleWithXaxis() < 90) //the first quartile
                deviation = listOfRP[i][6].getAngleWithXaxis();
            if(listOfRP[i][6].getAngleWithXaxis() > 90 && listOfRP[i][6].getAngleWithXaxis() < 180) //the second quartile
                deviation = 180 - listOfRP[i][6].getAngleWithXaxis();
            if(listOfRP[i][6].getAngleWithXaxis() > 180 && listOfRP[i][6].getAngleWithXaxis() < 270) //the third quartile
                deviation = abs(listOfRP[i][6].getAngleWithXaxis() - 180);
            if(listOfRP[i][6].getAngleWithXaxis() > 270) //the fourth quartile
                deviation = 360 - listOfRP[i][6].getAngleWithXaxis();

            
            if(deviation < 15)
            {
                //record this position
                temp_p.set(listOfRP[i][6].X1(), listOfRP[i][6].Y1());
                //cout << " the point is : " << listOfRP[i][6].X1()<< " ; and : " << listOfRP[i][6].Y1() << endl;
                path_points.push_back(temp_p); 
                heading_flag = 1;
            }
        }
        else
        {
                if(listOfRP[i][6].getAngleWithXaxis() < 90)
                    deviation = listOfRP[i][6].getAngleWithXaxis();
                if(listOfRP[i][6].getAngleWithXaxis() > 90 && listOfRP[i][6].getAngleWithXaxis() < 180)
                    deviation = 180 - listOfRP[i][6].getAngleWithXaxis();
                if(listOfRP[i][6].getAngleWithXaxis() > 180 && listOfRP[i][6].getAngleWithXaxis() < 270)
                    deviation = abs(listOfRP[i][6].getAngleWithXaxis() - 180);
                if(listOfRP[i][6].getAngleWithXaxis() > 270)
                    deviation = 360 - listOfRP[i][6].getAngleWithXaxis();

                if(deviation > 75)
                {
                    //record this position
                    temp_p.set(listOfRP[i][6].X1(), listOfRP[i][6].Y1());
                    //cout << " the point is : " << listOfRP[i][6].X1()<< " ; and : " << listOfRP[i][6].Y1() << endl;
                    path_points.push_back(temp_p); 
                    heading_flag = 0;
                }
        }
    }
    
    //establish path lines as objects for plotting 
    for(int i = 0 ; i < path_points.size() - 1; i++)
    {
          temp_Obj.set(path_points[i].X(), path_points[i].Y(),
                        path_points[i+1].X(), path_points[i+1].Y(), i);
          temp.push_back(temp_Obj);
    }
    
    //find required surfaces and cluster them
    /*
    vector<Object> temp_group;
    vector< vector<Object> > clusters;
    
    for(int i = 0; temp.size(); i++)
    {
        for(int j = 0; j < chunkMap.size(); j++)
        {
            //slope and distance to the path segment
            double object_slope = abs(temp[i].getAngleWithXaxis() - chunkMap[j].getAngleWithXaxis());
            double object_dist = distanceOftwoP(temp[i].midpoint(), chunkMap[j].midpoint());
            if(object_slope < 30 && object_dist < 4000)
                temp_group.push_back(chunkMap[j]);
        }
        clusters.push_back(temp_group);
    }
    */
    
    //establish boundary which cluster should be or not
    vector<Object> temp_boundary;
    vector<Object> combined_boundary;
    for(int i = 0; i < temp.size(); i++)
    {
            temp_boundary = RectangularAlongPath(temp[i]);
            if(i > 0)
                combined_boundary = addTwoVectorsOfObjects(combined_boundary, temp_boundary);
            else
                combined_boundary = temp_boundary;
    }
    
    
    sprintf(fileName, "%s", "Maps/Offline/TEST_PATH.png");
    //plotObjects(fileName, chunkMap, temp);
    plotObjectsOf3Kinds(fileName, chunkMap, temp, combined_boundary);
    
}

vector<Object> RectangularAlongPath(Object pathLine)
{
        Object left, right, bottom, top;
        Point bl, br, tl, tr;
        vector<Object> rnt;
        double width = 0;
        double height = 0;
        double offset = 1200;
        double slope = 0;

        //identify the slope of the path
        slope =  (pathLine.Y2() - pathLine.Y1()) / (pathLine.X2() - pathLine.X1());
        //cout<< " the deviation x : " << pathLine.X2() - pathLine.X1() << endl;
        //cout<< " the deviation y : " << pathLine.Y2() - pathLine.Y1() << endl;
        cout << " the slope of this path is : " << slope << endl;
        //calculate width and height -- four end points
        if(slope < 1)
        {
                height = abs(pathLine.X1() - pathLine.X2()); //x axis of objects
                bl.set(pathLine.X1(), pathLine.Y1() + offset);
                br.set(pathLine.X1(), pathLine.Y1() - offset);
                tl.set(pathLine.X2(), pathLine.Y1() + offset);
                tr.set(pathLine.X2(), pathLine.Y1() - offset);

                left.set(bl.X(), bl.Y(), tl.X(), tl.Y(), 1);
                right.set(br.X(), br.Y(), tr.X(), tr.Y(), 2);
                bottom.set(bl.X(), bl.Y(), br.X(), br.Y(), 3);
                top.set(tl.X(), tl.Y(), tr.X(), tr.Y(), 4);
        }
        if(slope >= 1)
        {
                height = abs(pathLine.Y1() - pathLine.Y2()); //y axis of objects
                bl.set(pathLine.X1() - offset, pathLine.Y1());
                br.set(pathLine.X1() + offset, pathLine.Y1());
                tl.set(pathLine.X1() - offset, pathLine.Y2());
                tr.set(pathLine.X1() + offset, pathLine.Y2());
                
                //establish four objects
                left.set(bl.X(), bl.Y(), tl.X(), tl.Y(), 1);
                right.set(br.X(), br.Y(), tr.X(), tr.Y(), 2);
                bottom.set(bl.X(), bl.Y(), br.X(), br.Y(), 3);
                top.set(tl.X(), tl.Y(), tr.X(), tr.Y(), 4);

        }
        
        //left.display();
        //right.display();
        //bottom.display();
        //top.display();
        
        //push in the temp view
        rnt.push_back(left);
        rnt.push_back(right);
        rnt.push_back(bottom);
        rnt.push_back(top);

        return rnt;
}

//abstract some surfaces that parallel the path segment
void SketchBoundary(vector<Object> chunkMap, vector< vector<Object> > listOfRP, 
                    vector< vector<Object> > chunkLMs)
{
    double length_threshold = 1000;
    double deviation = 0;
    double slope_path = 0;
    char fileName[80];
    unsigned char heading_flag = 0;
    Object temp_Obj;
    Object reference_heading;
    vector<Object> temp_simple_view;
    vector<Object> filtered;
    vector<Object> temp;
    MyRobot myrobot(0,0);
    Point temp_p;
    vector<Point> path_points;
    
    //initialisation
    temp_p.set(listOfRP[0][6].X1(), listOfRP[0][6].Y1());
    path_points.push_back(temp_p);
     
    cout<<" test, the list of rp size: " << listOfRP.size() << endl;
        //turning amount > 90 degree or < -90 degree
        reference_heading.set(listOfRP[0][6].X1(), listOfRP[0][6].Y1(),
                                listOfRP[0][6].X2(), listOfRP[0][6].Y2(),listOfRP[0][6].getID());
        for(int i = 1; i < listOfRP.size(); i++)
        {
            deviation = IncludedAngleOfTwoObjects(reference_heading, listOfRP[i][6]);
            //cout << " the deviation of angle is : " << deviation << endl;
            if(abs(deviation) > 70)
            {
                //new reference
                reference_heading.set(listOfRP[i][6].X1(), listOfRP[i][6].Y1(),
                                listOfRP[i][6].X2(), listOfRP[i][6].Y2(),listOfRP[i][6].getID());
        
                temp_p.set(listOfRP[i][6].X1(), listOfRP[i][6].Y1());
                path_points.push_back(temp_p);
            }
                
            if(i == listOfRP.size() - 1)
            {
                double dif = GetPointDistance(Point (listOfRP[i][6].X1(), listOfRP[i][6].Y1()), path_points.back());
                
                if((dif > 1000) && (abs(deviation) < 10))
                {
                    temp_p.set(listOfRP[i][6].X1(), listOfRP[i][6].Y1());
                    path_points.push_back(temp_p);
                }
            }

        }
    
        cout << " test & size of path point: " << path_points.size() << endl;
        //establish path lines as objects for plotting 
        for(int i = 0 ; i < path_points.size() - 1; i++)
        {
              temp_Obj.set(path_points[i].X(), path_points[i].Y(),
                            path_points[i+1].X(), path_points[i+1].Y(), i);
              temp.push_back(temp_Obj);
        }
        
        temp_simple_view = SlopeWithRoute(chunkMap, temp);
    
    /*
    temp_simple_view = BoundaryByGroup(chunkMap);
    
    for(int i = 0; i < temp_simple_view.size(); i++)
    {
        if(temp_simple_view[i].length() > 2000)
            filtered.push_back(temp_simple_view[i]);
    }
    */
        

    sprintf(fileName, "%s", "Maps/Offline/TEST_PATH.png");
    //plotObjectsColours(fileName,chunkLMs);
    plotObjectsColours(fileName, chunkLMs, temp);
}


vector<Object> SlopeWithRoute(vector<Object> chunkMap, vector<Object> route)
{
        double slope = 0;
        double deviation = 0;
        double dist_2Lines = 0;
        vector<Object> rnt;
        
        
        for(int j = 0; j < route.size(); j++)
        {
             //slope =  (pathLine.Y2() - pathLine.Y1()) / (pathLine.X2() - pathLine.X1());
            for(int i = 0; i < chunkMap.size(); i++)
            {
                //slope of surfaces are similar 
                deviation = IncludedAngleOfTwoObjects(route[j], chunkMap[i]);
                dist_2Lines = GetPointDistance(chunkMap[i].midpoint(), route[j].midpoint());
                if((abs(deviation) < 15) && (dist_2Lines) < 2000)
                    rnt.push_back(chunkMap[i]);
                else
                    if(chunkMap[i].length() > 1500)
                        rnt.push_back(chunkMap[i]);
            }
        }
        
        return rnt;
}


void RoughRoute(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Point> rps)
{
    Object temp_Obj, inters_Obj, path_Obj;
    vector<Object> route_segments;
    Point temp_Point, inter_Point;
    double offset = 800;
    double a, b, slope;
    char fileName[80];
    int type = 0;
    
    for(int i = 0 ; i < rps.size()-1; i++)
    {
        temp_Obj.set(rps[i].X(), rps[i].Y(), 
                        rps[i+1].X(), rps[i+1].Y(),i);
                     
        inters_Obj = intersectForObject(chunkMap, Point (temp_Obj.X1(), temp_Obj.Y1()), Point (temp_Obj.X2(), temp_Obj.Y2()));
        /*
        if(inters_Obj.getID() == 1)
        {
                //shifting to right or left
                inter_Point = intersectedPoint(chunkMap, Point (temp_Obj.X1(), temp_Obj.Y1()), Point (temp_Obj.X2(), temp_Obj.Y2()));
                //a = GetPointDistance(inter_Point, Point (temp_Obj.X1(), temp_Obj.Y1()));
                //b = GetPointDistance(inter_Point,  Point (temp_Obj.X2(), temp_Obj.Y2()));
                slope =  (temp_Obj.Y2() - temp_Obj.Y1()) / (temp_Obj.X2() - temp_Obj.X1());
                
                if(abs(slope) < 0.5)
                {
                    if(temp_Obj.X2() > temp_Obj.X1())
                        type = 3;
                    else
                        type = 1;
                }
                else
                {
                    if(slope > 0)
                        type = 2;
                    else
                    {
                        if(temp_Obj.X2() < temp_Obj.X1())
                            type = 1;
                        else
                            type = 4;
                    }
                }
            
                //compute an inter point & interpolate
                switch(type)
                {
                    case 1 :    temp_Point.set(inter_Point.X() - offset, inter_Point.Y() - offset);break;
                    case 2 :    temp_Point.set(inter_Point.X() - offset, inter_Point.Y() + offset);break;
                    case 3 :    temp_Point.set(inter_Point.X() + offset, inter_Point.Y() + offset);break;
                    case 4 :    temp_Point.set(inter_Point.X() + offset, inter_Point.Y() - offset);break;
                }
            
                //set path segments
                inters_Obj.set(temp_Obj.X1(), temp_Obj.Y1(), temp_Point.X(), temp_Point.Y(), 1);
                
                
                if((interSectWithLine(chunkMap, Point (temp_Obj.X2(), temp_Obj.Y2()), Point (temp_Point.X(), temp_Point.Y())) == true) )
                    //|| (interSectWithLine(chunkMap, Point (temp_Point.X(), temp_Point.Y()), Point (temp_Obj.X2(), temp_Obj.Y2()) == true)))
                {
                    
                    if(abs(slope) < 1)
                            temp_Point.set(rps[i+1].X(), rps[i].Y());
                    else
                        if(abs(slope) > 1.5)
                            temp_Point.set(rps[i].X(), rps[i+1].Y());
                    
                    inters_Obj.set(temp_Obj.X1(), temp_Obj.Y1(), temp_Point.X(), temp_Point.Y(), 1);
                    
                    if(interSectWithLine(chunkMap, Point (inters_Obj.X2(), inters_Obj.Y2()), Point (inters_Obj.X1(), inters_Obj.Y1())) == false)
                        route_segments.push_back(inters_Obj);
                    else
                    {
                        //go around obstacles situation
                        temp_Point.set(inter_Point.X() - offset*3, inter_Point.Y() - offset*3);
                                            inters_Obj.set(temp_Obj.X1(), temp_Obj.Y1(), temp_Point.X(), temp_Point.Y(), 1);
                                             route_segments.push_back(inters_Obj);
                    }
                }
                else
                    route_segments.push_back(inters_Obj);

                path_Obj.set(temp_Point.X(), temp_Point.Y(), temp_Obj.X2(), temp_Obj.Y2(), 2);
                route_segments.push_back(path_Obj);
        }
        else*/
            route_segments.push_back(temp_Obj);
        
        
        //vector<Object> splitSpace = DetectSpace(chunkMap, LMs, route_segments);

    }
    //cout << " the size of LMS : " << LMs.size() << endl;
    //vector<Object> splitSpace = DetectSpace(chunkMap, LMs, route_segments);
    
    sprintf(fileName, "%s", "Maps/Offline/TEST_PATH2.png");
    plotObjectsColours(fileName, LMs, route_segments);
}

/*
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

                

                /*
                //if(i % 2 == 0 && i != 0 && cnt < LMs.size())
                if(i != 0 && cnt < LMs.size())
                {
                    Multi_LM = LMs[cnt];
                    cnt++;
                    Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                    //cnt++;
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
                            Multi_LM = addTwoVectorsOfObjects(Multi_LM, LMs[cnt]);
                            temp = isBoundary(Multi_LM, route_segments[i]);
                        }
                        
                }
                
                all.push_back(temp);
                temp.clear();
                Multi_LM.clear();*/
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
                    if(route_segments.size() == 5)
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
        }
        
        cout << " distribute local maps finished !!" << endl;
        vector<Object> commonObjects = CommonSurface(all);
        cout << " size of all : " << all.size() << endl;
        sprintf(fileName, "%s", "Maps/Offline/TEST_PATH.png");
        //plotObjectsColours(fileName, all, route_segments, commonObjects);
        plotObjectsOf3Kinds(fileName,route_segments, chunkMap, commonObjects);
        
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
*/

/*To compute two perpendicular lines 
 *which is joining two end points of route segment
 *in order to identify a region within these two perpendicular lines
 */
pair<Point, Point> outPerpenPoint(Object reference)
{
        pair<Point, Point> rnt;
        Point p1, p2;
        double k1, x1, x2;
        double k2, y1, y2;
        double offset = 1000;
        
        k1 = (reference.Y1() - reference.Y2()) / (reference.X1() - reference.X2());
        k2 = -1/k1;
        
        x1 = reference.X1() + k2 * (reference.Y1() - reference.Y2());
        y1 = reference.Y1() - k2 * (reference.X1() - reference.X2());
        p1.set(x1, y1);
        x2 = reference.X2() + k2 * (reference.Y1() - reference.Y2());
        y2 = reference.Y2() - k2 * (reference.X1() - reference.X2());
        p2.set(x2, y2);
        
        if(reference.X1() == reference.X2())
        {
            p1.set(reference.X1(), reference.Y1()+offset);
            p2.set(reference.X2(), reference.Y2()+offset);
        }
        
        if(reference.Y1() == reference.Y2())
        {
            p1.set(reference.X1()+offset, reference.Y1());
            p2.set(reference.X2()+offset, reference.Y2());
        }
        
        
        rnt.first = p1;
        rnt.second = p2;
        
        return rnt;           
}

/* X = ( (f2-f1)/2(xp1-xp2) - k^2xp2 - kyp2  / (1-k^2))
 * f1 = a^2 - xp1^2 - yp1^2
 * f2 = b^2 - xp2^2 - yp2^2
 * y = sqrt( b^2 - x^2 )
 */
pair<Point, Point> PerpenPointToRouteSeg(Object reference)
{
        pair<Point, Point> rnt;
        Point p1, p2;
        double k1, x1, x2;
        double k2, y1, y2;
        double f1, f2, a, b;
        double x, y;
        double offset = 1000;
        
        b = offset;
        a = sqrt(b*b + reference.length()*reference.length());
        x1 = reference.X1();
        y1 = reference.Y1();
        x2 = reference.X2();
        y2 = reference.Y2();
        
        k1 = (y2-y1) / (x2-x1);
        
        f1 = a*a - x1*x1 - y1*y1;
        f2 = b*b - x2*x2 - y2*y2;
        x = ((f2-f1)/2*(x1-x2) - k1*k1*x2 - k1*y2)  / (1-k1*k1);
        y = sqrt(b*b - x*x);
        p1.set(x, y);
        
        
        x2 = reference.X1();
        y2 = reference.Y1();
        x1 = reference.X2();
        y1 = reference.Y2();
        
        k1 = (y2-y1) / (x2-x1);
        
        f1 = a*a - x1*x1 - y1*y1;
        f2 = b*b - x2*x2 - y2*y2;
        x = ((f2-f1)/2*(x1-x2) - k1*k1*x2 - k1*y2)  / (1-k1*k1);
        y = sqrt(b*b - x*x);
        p2.set(x, y);
        
        rnt.first = p1;
        rnt.second = p2;
        
        return rnt;           
}

pair<Point, Point> PerpenPointToRouteEnd(Object reference)
{
    double dx, dy, x1, x2, y1, y2;
    double x3, x4, y3, y4, dist;
    double N = 1000;
    Point p1, p2;
    pair<Point, Point> rnt;

    //p1
    x1 = reference.X1();
    y1 = reference.Y1();
    x2 = reference.X2();
    y2 = reference.Y2();

    dx = x1-x2;
    dy = y1-y2;
    dist = sqrt(dx*dx + dy*dy);
    dx /= dist;
    dy /= dist;
    x3 = x1 + (N/2)*dy;
    y3 = y1 - (N/2)*dx;
    x4 = x1 - (N/2)*dy;
    y4 = y1 + (N/2)*dx;
    p1.set(x3, y3);
    
    //p2
    x2 = reference.X1();
    y2 = reference.Y1();
    x1 = reference.X2();
    y1 = reference.Y2();

    dx = x1-x2;
    dy = y1-y2;
    dist = sqrt(dx*dx + dy*dy);
    dx /= dist;
    dy /= dist;
    x3 = x1 + (N/2)*dy;
    y3 = y1 - (N/2)*dx;
    x4 = x1 - (N/2)*dy;
    y4 = y1 + (N/2)*dx;
    p2.set(x3, y3);
    
           
    rnt.first = p1;
    rnt.second = p2;
    
    return rnt;
}

/*
void RegionAndBoundary(vector< vector<Object> > allLEs, vector<Point> robpositions, LocalMap LMS)
{
    vector<Object> chunk;
    vector<Object> modiLM;
    vector<Object> CombinedRegion;
    vector<Point> rps;
    vector<Exit> exits;
    vector<Object> route_segments;
    vector< vector<Object> > chunkWithLMs;
    vector< vector<Object> > allrps;
    vector< vector<Object> > allRegions;
    vector< vector<Object> > splitSpace;
    chunk = LMS.getGlobalMaps()[0];
    char fileName[80];
*/    
   

/*    
    for(int i = 1; i < 5; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }
    
    for(int i = 0; i < allLEs.size(); i++)
    {

        if(i == 0){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
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

    for(int i = 6; i < 9; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }    
    
    for(int i = 0; i < allLEs.size(); i++)
    {

        if(i == 48){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
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
        rps.push_back(robpositions[i]);}
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

    }

    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    
    //////
    TestProgramme(chunk, splitSpace, route_segments);
    //////
    vector< vector<Object> > splitSpace2 = DetectSpace(chunk, chunkWithLMs, route_segments);
    CombineRegions(splitSpace2[0], splitSpace, exits);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    
    RegionBlock(chunk, splitSpace, exits, route_segments, 3);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits3.png");
    //plotObjectsofexits(fileName, chunk, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    //configReturnView(chunk, chunk, splitSpace2, Point (0,0));
*/
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
        if(i == 14)
        {
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);
        }
        
        if(i == 15)
        {
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);
        }
        
         
        sprintf(fileName, "%s", "Maps/Offline/TEST_AllASR_Points.png");
        plotObjectsColourAndPoint(fileName, chunkWithLMs, rps);
 
        
    }
    
    //cout << " the size of chunk LMS: " << chunkWithLMs.size() << endl;
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    RegionBlock(chunk, splitSpace, exits, route_segments, 1);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits.png");
    //plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments, exits);
 
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
       
        if(i == 31)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 38)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 39)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}

    }
    route_segments = RouteCombine(chunk, chunkWithLMs, rps);
    splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
    exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
    RegionBlock(chunk, splitSpace, exits, route_segments, 2);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits2.png");
    //plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    waitHere();
    
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk = LMS.getGlobalMaps()[4];

    for(int i = 5; i < 6; i++)
    {
        chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
    }

    for(int i = 0; i < allLEs.size(); i++)
    {
      
        if(i == 53)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 69){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 70){
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
    //plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
    waitHere();
    
    
    chunkWithLMs.clear();
    rps.clear();
    chunk.clear();
    
    chunk = LMS.getGlobalMaps()[6];

    for(int i = 7; i < 8; i++)
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
    //plotObjectsOf3KindswithExits(fileName, chunk, route_segments, exits);
    plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
 */  
    //cout << " size of All les: " << allLEs.size() << endl;
    //cout << " size of robpositions: " << robpositions.size() << endl;
    
/*    
    for(int i = 0; i < allLEs.size(); i++)
    {
      
        if(i == 0)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 15){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 16){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 27){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 28){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 29){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}

    }
    
    Object temp_route;
    vector<Object> routes;

    
    for(int i = 0; i < rps.size()-1; i++)
    {
        temp_route.set(rps[i].X(), rps[i].Y(), rps[i+1].X(), rps[i+1].Y(), i);
        routes.push_back(temp_route);
    }
    //sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits1.png");
    //plotObjectsColours(fileName, chunkWithLMs, routes);
    
    exits = ExitPerpendPath(routes);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits1.png");
    plotObjectsColoursAndExits(fileName, chunkWithLMs, routes, exits);
    
    waitHere();
    routes.clear();
    rps.clear();
    exits.clear();
    chunkWithLMs.clear();
    for(int i = 0; i < allLEs.size(); i++)
    {
      
        if(i == 40)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 49){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 50){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 61){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 62){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 72){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 73){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 76){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}


    }
    

    
    for(int i = 0; i < rps.size()-1; i++)
    {
        temp_route.set(rps[i].X(), rps[i].Y(), rps[i+1].X(), rps[i+1].Y(), i);
        routes.push_back(temp_route);
    }
    //sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits1.png");
    //plotObjectsColours(fileName, chunkWithLMs, routes);
    
    exits = ExitPerpendPath(routes);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits2.png");
    plotObjectsColoursAndExits(fileName, chunkWithLMs, routes, exits);
    waitHere();
    
    routes.clear();
    exits.clear();
    rps.clear();
    chunkWithLMs.clear();
    for(int i = 0; i < allLEs.size(); i++)
    {
      
        if(i == 79)  {
            chunkWithLMs.push_back(allLEs[i]); 
        rps.push_back(robpositions[i]);}
        if(i == 87){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 88){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 107){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 108){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}
        if(i == 110){
            chunkWithLMs.push_back(allLEs[i]);
        rps.push_back(robpositions[i]);}


    }
    

    
    for(int i = 0; i < rps.size()-1; i++)
    {
        temp_route.set(rps[i].X(), rps[i].Y(), rps[i+1].X(), rps[i+1].Y(), i);
        routes.push_back(temp_route);
    }
    //sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits1.png");
    //plotObjectsColours(fileName, chunkWithLMs, routes);
    
    exits = ExitPerpendPath(routes);
    sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_And_exits3.png");
    plotObjectsColoursAndExits(fileName, chunkWithLMs, routes, exits);
    
    
}
*/

/*
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

        unsigned char cluster_flag = 0;//clustering flag for route
        
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
            /*
            for(int n = 0; n < allrob; n++)
            {
                if(temp_Obj.X1() = allrob[n].X() && temp_Obj.Y1() = allrob[n].Y())
                    cluster_flag = 1;
                    
                if(temp_Obj.X2() = allrob[n].X() && temp_Obj.Y2() = allrob[n].Y())
                    cluster_flag = 0;
                
                if(cluster_flag == 1)
                    cluster_points.push_back(allrob[n]);
            }
            
            for(int m = 0; m < cluster_points.size(); m++)
            {
                //calculating the distance 
                double cluster_dist = perpendicularDis( , ,cluster_points[m]);
                
                if(cluster_dist >= cluster_threshold)
                {
                    //split current route segment into two segments
                    Object first_part;
                    Object second_part;
                    
                    first_part.set(temp_Obj.X1(), temp_Obj.Y1(),
                                    cluster_points[m].X(), cluster_points[m].Y(), 1);//first part
                    
                    second_part.set(cluster_points[m].X(), cluster_points[m].Y(), 
                                    temp_Obj.X2(), temp_Obj.Y2(), 2); //second part
                    
                    route_segments.pop_back();
                    route_segments.push_back(first_part);
                    route_segments.push_back(second_part);
                }
            }
            */

//        }
        
        /*
        for(int i = 0; i < rps.size()-1; i++)
        {
            temp_Obj.set(rps[i].X(), rps[i].Y(), 
                                rps[i+1].X(), rps[i+1].Y(),i);

                route_segments.push_back(temp_Obj);
        }
        */
        

        /*
        //bring back one view to complete a region
        for(int i = 0; i < route_segments.size(); i++)
        {
            if(route_segments[i].length() > route_length_threshold)
            {
                //read one more view   
            }
        }
        */
        
        //cout << " the size of route : " << route_segments.size() << endl;
        //cout << " the size of LMs : " << LMs.size() << endl;
        
        //attempt to generate boundary
        //vector< vector<Object> > splitSpace = DetectSpace(chunkMap, LMs, route_segments);
        
        //cout << " the size of views : " << LMs.size() << endl;
/*        sprintf(fileName, "%s", "Maps/Offline/TEST_PATH2.png");
        plotObjectsColours(fileName, LMs, route_segments);
        
        return route_segments;
}
*/
        
        
vector<Object> GeneralRouteCombine(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Point> rps)
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

        unsigned char cluster_flag = 0;//clustering flag for route
        
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
            
        }
        
        
        //cout << " the size of views : " << LMs.size() << endl;
        sprintf(fileName, "%s", "Maps/Offline/TEST_PATH2.png");
        plotObjectsColours(fileName, LMs, route_segments);
        
        return route_segments;
}

/* based on two end points of route segment
 * extend two lines , which are horizontal or vertical 
 * in order to find the front and behind surfaces surrounding 
 * the route.
 */
vector<Object> FrontAndBehind(vector<Object> chunkMap, Object route, unsigned char start_flag)
{
        Object front, behind;
        Object temp_Obj1, temp_Obj2;
        vector<Object> rnt;
        double slope = 0;
        double offset = 10000;

        Point mid_point; //mid point of route segment
        mid_point.set((route.X1() + route.X2()) / 2, (route.Y1() + route.Y2()) / 2);
        slope =  (route.Y2() - route.Y1()) / (route.X2() - route.X1());


            //identify the orientation
            if(route.X1() < route.X2() && abs(slope) < 1)
            {
                //temp_Obj1.set(mid_point.X(), mid_point.Y(), mid_point.X() + offset, mid_point.Y(), 1);
                //temp_Obj2.set(mid_point.X(), mid_point.Y(), mid_point.X() - offset, mid_point.Y(), 2);
                temp_Obj1.set(route.X2(), route.Y2(), route.X2() + offset, route.Y2(), 1);
                temp_Obj2.set(route.X1(), route.Y1(), route.X1() - offset, route.Y1(), 2);
 
                front = intersectForObject(chunkMap, Point (temp_Obj1.X1(), temp_Obj1.Y1()), Point (temp_Obj1.X2(), temp_Obj1.Y2())) ;
                behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
            }
            else
            {
                if(route.X1() > route.X2())
                {
                    //temp_Obj1.set(mid_point.X(), mid_point.Y(), mid_point.X() - offset, mid_point.Y(), 1);
                    //temp_Obj2.set(mid_point.X(), mid_point.Y(), mid_point.X() + offset, mid_point.Y(), 2);
                    temp_Obj1.set(route.X1(), route.Y1(), route.X1() + offset, route.Y1(), 1);
                    temp_Obj2.set(route.X2(), route.Y2(), route.X2() - offset, route.Y2(), 2);
  
                    front = intersectForObject(chunkMap, Point (temp_Obj1.X1(), temp_Obj1.Y1()), Point (temp_Obj1.X2(), temp_Obj1.Y2())) ;
                    behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
                }
            }
                
                
            if(route.Y1() < route.Y2() && abs(slope) > 1 )
            {
                  //temp_Obj1.set(mid_point.X(), mid_point.Y(), mid_point.X(), mid_point.Y() + offset, 1);
                  //temp_Obj2.set(mid_point.X(), mid_point.Y(), mid_point.X(), mid_point.Y() - offset, 2);
                
                  temp_Obj1.set(route.X2(), route.Y2(), route.X2(), route.Y2() + offset, 1);
                  temp_Obj2.set(route.X1(), route.Y1(), route.X1(), route.Y1() - offset, 2);
 
                
                  front = intersectForObject(chunkMap, Point (temp_Obj1.X1(), temp_Obj1.Y1()), Point (temp_Obj1.X2(), temp_Obj1.Y2())) ;
                  behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
            }
            else
            {
                if(route.Y1() > route.Y2())
                {
                    //temp_Obj1.set(mid_point.X(), mid_point.Y(), mid_point.X(), mid_point.Y() - offset, 1);
                    //temp_Obj2.set(mid_point.X(), mid_point.Y(), mid_point.X(), mid_point.Y() + offset, 2);
                
                    temp_Obj1.set(route.X1(), route.Y1(), route.X1(), route.Y1() + offset, 1);
                    temp_Obj2.set(route.X2(), route.Y2(), route.X2(), route.Y2() - offset, 2);
                    
                    front = intersectForObject(chunkMap, Point (temp_Obj1.X1(), temp_Obj1.Y1()), Point (temp_Obj1.X2(), temp_Obj1.Y2())) ;
                    behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
                }
            }
        
        if(start_flag != 0)
        {
            rnt.push_back(front);
            rnt.push_back(behind);
        }
        else
        {
            rnt.push_back(front);
        }
    
        
        return rnt;
        
}


/*Compute two regions, one is following the front end point
 *another is following the back end point based on the orientation of route
 *once the surface in the regions, collect them as front or behind surfaces
 */
/*
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
/*        
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
/*        
        return rnt;
}
*/


vector<Exit> IdentifyExits(vector< vector<Object> > regions, vector<Object> route_segments)
{
        vector<Object> Common_Surfaces; //common surfaces 
        int i = 0; //region label
        int j = 0; //route label
        double slope1 = 0;
        double slope2 = 0;
        double min = 10000;
        double offset = 5000;
        double To_ref_dist = 0;
        
        Object potential_Object;
        Object extend_Object;
        Point ref_Point; //the follow path, first end point
        Point intersect_Point;
        Exit potential_exit; //between two adjacent region
        vector<Exit> exits;
        vector< vector<Object> > Two_adjacent_region;
        
        while(i < regions.size() - 1)
        {
                //slope
                slope1 =  (route_segments[i].Y2() - route_segments[i].Y1()) / (route_segments[i].X2() - route_segments[i].X1());
                slope2 =  (route_segments[i+1].Y2() - route_segments[i+1].Y1()) / (route_segments[i+1].X2() - route_segments[i+1].X1());
                
                //find common surfaces between two adjacent regions
                
                Two_adjacent_region.push_back(regions[i]); //first region
                Two_adjacent_region.push_back(regions[i+1]); //second region
                Common_Surfaces = CommonSurface(Two_adjacent_region);
                
                //identify the relevant surfaces 
                ref_Point.set(route_segments[i+1].X1(), route_segments[i+1].Y1()); 
                
                for(int v = 0; v < Common_Surfaces.size(); v++)
                {
                        //the nearest surface 
                        To_ref_dist =  GetPointDistance(Point (Common_Surfaces[v].X1(), Common_Surfaces[v].Y1()), ref_Point);
                        if(To_ref_dist < min)
                        {
                            min = To_ref_dist;
                            //potential_Object = Common_Surfaces[v];
                            potential_Object.set(Common_Surfaces[v].X1(), Common_Surfaces[v].Y1(),
                                                Common_Surfaces[v].X2(), Common_Surfaces[v].Y2(), Common_Surfaces[v].getID());
                        }
                        To_ref_dist =  GetPointDistance(Point (Common_Surfaces[v].X2(), Common_Surfaces[v].Y2()), ref_Point);
                        if(To_ref_dist < min)
                        {
                            min = To_ref_dist;
                            //potential_Object = Common_Surfaces[v];
                            potential_Object.set(Common_Surfaces[v].X1(), Common_Surfaces[v].Y1(),
                                                Common_Surfaces[v].X2(), Common_Surfaces[v].Y2(), Common_Surfaces[v].getID());
                        } 
                }
                
                double dist1 = GetPointDistance(Point (potential_Object.X1(), potential_Object.Y1()), ref_Point);
                double dist2 = GetPointDistance(Point (potential_Object.X2(), potential_Object.Y2()), ref_Point);

                if(dist1 < dist2)
                {
                    //identify extending orientation
                    if(abs(slope2) > 1 && route_segments[i+1].Y1() > route_segments[i+1].Y2()) //top
                        extend_Object.set(potential_Object.X1(),potential_Object.Y1(), potential_Object.X1(),potential_Object.Y1()+offset,1);
                    if(abs(slope2) > 1 && route_segments[i+1].Y1() < route_segments[i+1].Y2()) //down
                        extend_Object.set(potential_Object.X1(),potential_Object.Y1(), potential_Object.X1(),potential_Object.Y1()-offset,1);
                    if(abs(slope2) < 1 && route_segments[i+1].X1() > route_segments[i+1].X2()) //right
                        extend_Object.set(potential_Object.X1(),potential_Object.Y1(), potential_Object.X1()+offset,potential_Object.Y1(),1);    
                    if(abs(slope2) < 1 && route_segments[i+1].X1() < route_segments[i+1].X2()) //left
                        extend_Object.set(potential_Object.X1(),potential_Object.Y1(), potential_Object.X1()-offset,potential_Object.Y1(),1);
                }
                else
                {
                    //identify extending orientation
                    if(abs(slope2) > 1 && route_segments[i+1].Y1() > route_segments[i+1].Y2()) //top
                        extend_Object.set(potential_Object.X2(),potential_Object.Y2(), potential_Object.X2(),potential_Object.Y2()+offset,1);
                    if(abs(slope2) > 1 && route_segments[i+1].Y1() < route_segments[i+1].Y2()) //down
                        extend_Object.set(potential_Object.X2(),potential_Object.Y2(), potential_Object.X2(),potential_Object.Y2()-offset,1);
                    if(abs(slope2) < 1 && route_segments[i+1].X1() > route_segments[i+1].X2()) //right
                        extend_Object.set(potential_Object.X2(),potential_Object.Y2(), potential_Object.X2()+offset,potential_Object.Y2(),1);    
                    if(abs(slope2) < 1 && route_segments[i+1].X1() < route_segments[i+1].X2()) //left
                        extend_Object.set(potential_Object.X2(),potential_Object.Y2(), potential_Object.X2()-offset,potential_Object.Y2(),1);
                }
                
                //find the intersected point with any surface in the previous (i)  region
                intersect_Point = intersectedPoint(regions[i], Point (extend_Object.X1(), extend_Object.Y1()), Point (extend_Object.X2(), extend_Object.Y2()));
                Object temp_Obj = intersectForObject(regions[i], Point (extend_Object.X1(), extend_Object.Y1()), Point (extend_Object.X2(), extend_Object.Y2()));
                if(temp_Obj.getID() != 0)
                    potential_exit.set(extend_Object.X1(), extend_Object.Y1(), intersect_Point.X(), intersect_Point.Y());
                else
                {
                    //no any intersection
                    //change another potential object
                    potential_exit.set(extend_Object.X1(), extend_Object.Y1(), extend_Object.X2(), extend_Object.Y2());
                }
                
                exits.push_back(potential_exit);
                
                Two_adjacent_region.clear();
                min = 10000; //initialise again
                i++;
        }
        
        return exits;
}

/*
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
*/


/**
 * Through the first surface of new chunk
 * to deviate/project the previous region/
 * find reference surfaces and using it to project on
 * current chunk/view
 */
/*
vector<Object> RegionDeviation(vector<Object> TargetChunk, vector<Object> KeyRegion)
{
    Object ref_On_target, ref_On_region;
    vector<Object> temp;
    int refPoint = 2;
    double dist, slope1, slope2;
    double min = 5000;
    double threshold = 1000;
    
    char fileName[80];
    
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
*/

/*******************************  TEST PROGRAMME   *****************************/
void TestProgramme(vector<Object> Chunk3, vector< vector<Object> > regions_Chunk2, vector<Object> route_Chunk3)
{
        char fileName[80];
        vector< vector<Object> > plot;
        
        plot.push_back(Chunk3);
        plot.push_back(regions_Chunk2[regions_Chunk2.size()-1]);
        
        
        sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_Region1.png");
        plotObjectsColours(fileName, plot, route_Chunk3);
        
        plot.clear();
        plot.push_back(Chunk3);
        plot.push_back(regions_Chunk2[regions_Chunk2.size()-2]);
        
        
        sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_Region2.png");
        plotObjectsColours(fileName, plot, route_Chunk3);
        
        plot.clear();        
        plot.push_back(Chunk3);
        plot.push_back(regions_Chunk2[regions_Chunk2.size()-3]);
        
        
        sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_Region3.png");
        plotObjectsColours(fileName, plot, route_Chunk3);
}


vector<Object> CombineRegions(vector<Object> region_Chunk3, vector< vector<Object> > regions_Chunk2, vector<Exit> exits_Chunk2)
{
        char fileName[80];
        vector< vector<Object> > plot;
        
        plot.push_back(region_Chunk3);
        plot.push_back(regions_Chunk2[regions_Chunk2.size()-1]);
        
        
        sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_Region4.png");
        plotObjectsColours(fileName, plot);
        
        plot.push_back(regions_Chunk2[regions_Chunk2.size()-2]);

        sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_Region5.png");
        plotObjectsColours(fileName, plot);
        
        plot.push_back(regions_Chunk2[regions_Chunk2.size()-3]);
        sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_Region6.png");
        plotObjectsColours(fileName, plot);
        
        sprintf(fileName, "%s", "Maps/Offline/TEST_Combined_exits.png");
        plotObjectsofexits(fileName, region_Chunk3, exits_Chunk2);
}

vector<Object> configReturnView(vector<Object> return_View, vector<Object> chunk, vector< vector<Object> > regionOfChunk, Point exit_positionOnChunk)
{
    vector<Object> retView;
    char fileName[80];
    char viewFileName[100];
    MyRobot myrobot(0,0);
    cout << " read @@@@ " << endl;
    //return data level 5 data 10 view22
    sprintf(viewFileName, "%s%d%s%d%s%d", "inputData/level", 5, "set", 10, "/surface-", 22);
    retView = readASCII(viewFileName);
    
    sprintf(fileName, "%s", "Maps/Offline/return_view_22.png");
    plotObjects(fileName, myrobot.getRobot(), retView);
    
    cout << " read successfully " << endl;
}

Point interpolatePoint(vector<Point> realRoute, Point startPosition, Point endPosition)
{
    double threshold = 2000;
    double max = 0;
    double dist = 0;
    Point temp;
    
    for(int i = 0; i < realRoute.size(); i++)
    {
        //dist = Perpendiculardistance();
        if(dist > max)
        {
            max = dist;
            temp = realRoute[i];
        }
    }
    
    if(max > threshold)
        return temp;
    
    //else 
    //    return 0;
}

/*******************************************************************************/

/*****************************************/


/* through exits to identify region block
 * connect all surface in one block in order 
 * to generate polygon region
 */
vector<Object> RegionBlock(vector<Object> chunkMap, vector< vector<Object> > regions, vector<Exit> exits, 
                                vector<Object> route_segments, int chunkNum)
{
        vector<Object> temp_region;
        vector<Object> dotted_back;
        vector< vector<Object> > all_regions;
        Object front, back, left, right;
        Object temp;
        Object temp_Obj2, behind;
        Point inter_p1, inter_p2;
        Point ref_point;

        double offset = 5000;
        int inters_cnt = 0;
        int flag = 0;
        char fileName[60];

        for(int i = 0; i < exits.size(); i++)
        {
            //trim the surfaces into an individual region
            //the front is the current exit self
            front.set(exits[i].X1(), exits[i].Y1(),
                     exits[i].X2(), exits[i].Y2(), 0);
            

            //find suitable back surface
            if(i == 0)
            {
                //no any surface behind
                //compute a line segement
                //temp.set(0,0, -1000, 0, 1);
                //inter_p1 = intersectedPoint(chunkMap, Point (temp.X1(), temp.Y1()), Point (temp.X2(), temp.Y2()));
                //temp.set(0,0, 1000, 0, 1);
                //inter_p2 = intersectedPoint(chunkMap, Point (temp.X1(), temp.Y1()), Point (temp.X2(), temp.Y2()));
                if(chunkNum == 1)
                    back.set(0-500, 0, 0+500, 0, 1);
                else
                {
                    //first route p1 as ref point
                    
                }
            }
            else
            {
                    Point mid_point;
                    //mid_point.set((route_segments[i].X1() + route_segments[i].X2()) / 2, (route_segments[i].Y1() + route_segments[i].Y2()) / 2);
                    double slope =  (route_segments[i].Y2() - route_segments[i].Y1()) / (route_segments[i].X2() - route_segments[i].X1());
                    //identify the orientation
                    if(route_segments[i].X1() < route_segments[i].X2() && abs(slope) < 1)
                    {

                        temp_Obj2.set(route_segments[i].X1(), route_segments[i].Y1(), route_segments[i].X1() - offset, route_segments[i].Y1(), 2);
                        behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
                    }
                    else
                    {
                        if(route_segments[i].X1() > route_segments[i].X2() && abs(slope) < 1)
                        {

                            temp_Obj2.set(route_segments[i].X1(), route_segments[i].Y1(), route_segments[i].X1() + offset, route_segments[i].Y1(), 2);
                            behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
                        }
                    }


                    if(route_segments[i].Y1() < route_segments[i].Y2() && abs(slope) > 1 )
                    {

                          temp_Obj2.set(route_segments[i].X1(), route_segments[i].Y1(), route_segments[i].X1(), route_segments[i].Y1() - offset, 2);
                          behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
                    }
                    else
                    {
                        if(route_segments[i].Y1() > route_segments[i].Y2() && abs(slope) > 1 )
                        {

                            temp_Obj2.set(route_segments[i].X1(), route_segments[i].Y1(), route_segments[i].X1(), route_segments[i].Y1() + offset, 2);
                            behind = intersectForObject(chunkMap, Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2())) ;
                        }
                    }
                    
                    if(behind.getID() == 0)
                    {
                        //behind is empty
                        if(abs(slope) > 1)
                            back.set(route_segments[i].X1()+3000, route_segments[i].Y1(), route_segments[i].X1()-3000, route_segments[i].Y1(), 1);
                        if(abs(slope) < 1)
                            back.set(route_segments[i].X1(), route_segments[i].Y1()+3000, route_segments[i].X1(), route_segments[i].Y1()-3000, 1);
                        
                        back.setDottedBack(false);
                        dotted_back = breakTheLineInto(back);
                    }
                    else
                    {
                        cout<<"intersection object as behind"<<endl;
                        back = behind;
                        back.setDottedBack(true);
                     }
            }


            //connect front and back
            //identify left and right
            if((isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (front.X1(), front.Y1())) > 0) &&
               (isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (back.X1(), back.Y1())) > 0)
                    &&(isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (front.X2(), front.Y2())) < 0)
                    &&(isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (back.X2(), back.Y2())) < 0))
            {
                left.set(back.X1(), back.Y1(), front.X1(), front.Y1(), 2);
                right.set(back.X2(), back.Y2(), front.X2(), front.Y2(), 3);
            }
            else
            {
                if((isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (front.X1(), front.Y1())) < 0) &&
                  (isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (back.X1(), back.Y1())) < 0)
                    &&(isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (front.X2(), front.Y2())) > 0)
                    &&(isLeft(Point (route_segments[i].X1(), route_segments[i].Y1()), Point (route_segments[i].X2(), route_segments[i].Y2()), Point (back.X2(), back.Y2())) > 0))
                {
                        left.set(back.X1(), back.Y1(), front.X1(), front.Y1(), 2);
                        right.set(back.X2(), back.Y2(), front.X2(), front.Y2(), 3);
                }
                else
                {
                    left.set(back.X1(), back.Y1(), front.X2(), front.Y2(), 2);
                    right.set(back.X2(), back.Y2(), front.X1(), front.Y1(), 3);
                }
            }
            
            if(TwoObjectIntersect(left, right) == true)
            {
                Point tp1, tp2;
                tp1.set(left.X2(), left.Y2());
                tp2.set(right.X2(), right.Y2());
                
                //modifiied left & right again
                left.set(left.X1(), left.Y1(), tp2.X(), tp2.Y(), 2);
                right.set(right.X1(), right.Y1(), tp1.X(), tp1.Y(),3);
            }
            
            //back.display();
            //waitHere();
            if(back.getDottedBack() == true)
            {
                temp_region.push_back(front);
                temp_region.push_back(back);
                temp_region.push_back(left);
                temp_region.push_back(right);
            }
            else
            {
                temp_region.push_back(front);
                temp_region.push_back(left);
                temp_region.push_back(right);
                
                temp_region = addTwoVectorsOfObjects(temp_region, dotted_back);
            }
            /*
            //re-modify the front, but not using exit as front
            for(int j = 0; j < temp_region.size(); j++)
            {
                for(int k = 0; k < all_regions.back().size(); k++)
                {
                    //if there is no any intersection
                    //re-modify current shape
                    if(TwoObjectIntersect(temp_region[j], all_regions.back()[k]) == true)
                        inters_cnt++;
                }
            }
            
            if(inters_cnt == 0)
            {
                //front
                for(int j = 0; j < regions[i].size(); j++)
                {
                    
                }
                
                //back
                if(back.getDottedBack() == false)
                {
                    //break the back into dots
                    
                }
            }
            */
            all_regions.push_back(temp_region);
            temp_region.clear();

        }
        
        sprintf(fileName, "%s%d%s", "Maps/Offline/TEST_simplify_Region-", chunkNum,".png");
        plotObjectsColours(fileName, all_regions, route_segments);
        
        return temp_region;
}


/*function : combine two chunks
 *description : generate path block and combine two chunks
 *              based on common exit point
 *input : two different chunks 
 */

vector<Object> combinedChunkGeometry(vector< vector<Object> > allLEs, vector<Point> robpositions, LocalMap LMS)
{
        Point key_exit_chunk1;
        Point key_exit_chunk3;
        
        vector<Object> temp_view;
        vector<Object> chunk1, chunk2, chunk3;
        vector< vector<Object> > chunk1_LM, chunk2_LM, chunk3_LM;
        vector<Point> rps1, rps2, rps3;
        vector<Object> trans_view;
        
        char fileName[80];

        //////////////////////////////////
        key_exit_chunk1.set(4000, -12000);
        key_exit_chunk3.set(6300, -12100);
        //////////////////////////////////
        chunk1 = LMS.getGlobalMaps()[0];
        temp_view = BlcokBoundaryView(LMS.getGlobalMaps()[0]);
        chunk1_LM.push_back(temp_view);
        for(int i = 1; i < 5; i++)
        {
            chunk1 = addTwoVectorsOfObjects(chunk1, LMS.getGlobalMaps()[i]);
            temp_view = BlcokBoundaryView(LMS.getGlobalMaps()[i]);
            chunk1_LM.push_back(temp_view);
        }

        for(int i = 0; i < allLEs.size(); i++)
        {

            if(i == 0){
         
            rps1.push_back(robpositions[i]);}

            if(i == 7){  
            rps1.push_back(robpositions[i]);}
    
            if(i == 12){
            rps1.push_back(robpositions[i]);}  
    
            if(i == 30)  {
 
            rps1.push_back(robpositions[i]);}
  
            if(i == 37){
            rps1.push_back(robpositions[i]);}

        }

        chunk2 = LMS.getGlobalMaps()[5];
        temp_view = BlcokBoundaryView(LMS.getGlobalMaps()[5]);
        chunk2_LM.push_back(temp_view);
            
        for(int i = 6; i < 9; i++)
        {
            chunk2 = addTwoVectorsOfObjects(chunk2, LMS.getGlobalMaps()[i]);
            temp_view = BlcokBoundaryView(LMS.getGlobalMaps()[i]);
            chunk2_LM.push_back(temp_view);
        }    

        for(int i = 0; i < allLEs.size(); i++)
        {

            if(i == 48){
          
            rps2.push_back(robpositions[i]);}

            if(i == 60){ 
            rps2.push_back(robpositions[i]);}
 
            if(i == 64){
            rps2.push_back(robpositions[i]);}
         
            if(i == 71)  {
            rps2.push_back(robpositions[i]);}


        }

        chunk3 = LMS.getGlobalMaps()[9];
        temp_view = BlcokBoundaryView(LMS.getGlobalMaps()[9]);
        chunk3_LM.push_back(temp_view); 
        
        for(int i = 10; i < 13; i++)
        {
            chunk3 = addTwoVectorsOfObjects(chunk3, LMS.getGlobalMaps()[i]);
            temp_view = BlcokBoundaryView(LMS.getGlobalMaps()[i]);
            chunk3_LM.push_back(temp_view); 
        }

        for(int i = 0; i < allLEs.size(); i++)
        {
     
            if(i == 79)  {
                
            rps3.push_back(robpositions[i]);}
          
            if(i == 91){
            rps3.push_back(robpositions[i]);}
         
            if(i == 96){ 
            rps3.push_back(robpositions[i]);}
    
            if(i == 103){
            rps3.push_back(robpositions[i]);}

        }

        cout << " Plotting all chunks !!!!! " << endl << endl;
        /*
        vector<Object> path1 = returnPATH(rps1, key_exit_chunk1, 1);
        vector<Object> path3 = returnPATH(rps3, key_exit_chunk3, 2);
        
        // generate geometry follow chunk path
        vector<Object> Geom_1 =  consGeometry(chunk1, path1, 2);
        // generate another one
        vector<Object> Geom_3 =  consGeometry(chunk3, path3, 1);
        
        sprintf(fileName, "%s%d%s", "Maps/Offline/BlockPath-", 1,".png");
        plotObjects(fileName, Geom_1, chunk1);   
        sprintf(fileName, "%s%d%s", "Maps/Offline/BlockPath-", 3,".png");
        plotObjects(fileName, Geom_3, chunk3);
        
        Geom_1 = addTwoVectorsOfObjects(Geom_1, chunk1);
        Geom_3 = addTwoVectorsOfObjects(Geom_3, chunk3);
        
        // combine two geometries of two chunks based on the key point -- translation
        double dx = key_exit_chunk3.X() - key_exit_chunk1.X();
        double dy = key_exit_chunk3.Y() - key_exit_chunk1.Y();
        for(int i = 0; i < Geom_1.size(); i++)
        {
            Object temp_obj;
            temp_obj.set(Geom_1[i].X1() + dx, Geom_1[i].Y1() + dy,
                            Geom_1[i].X2() + dx, Geom_1[i].Y2() + dy,i);
            trans_view.push_back(temp_obj);
        }
        
        sprintf(fileName, "%s%d%s", "Maps/Offline/BlockPathView-", 0,".png");
        plotObjects(fileName, Geom_3, trans_view);

        // whether these two geometries are intersected
        */
        
        char plotFileName[80];
        sprintf(plotFileName, "%s", "Maps/Offline/Memory_Chunk1.png");
        plotObjectsColours(plotFileName, chunk1_LM);
        sprintf(plotFileName, "%s", "Maps/Offline/Memory_Chunk2.png");
        plotObjectsColours(plotFileName, chunk2_LM);
        sprintf(plotFileName, "%s", "Maps/Offline/Memory_Chunk3.png");
        plotObjectsColours(plotFileName, chunk3_LM);
        /*
        cout << " x1 : " << chunk1[0].X1() << " ; y1 : " << chunk1[0].Y1()
                << " ; x2 : "<< chunk1[0].X2() << " ; y2 : " << chunk1[0].Y2() << endl;
        cout << " x1 : " << chunk2[0].X1() << " ; y1 : " << chunk2[0].Y1()
                << " ; x2 : "<< chunk2[0].X2() << " ; y2 : " << chunk2[0].Y2() << endl;
        cout << " x1 : " << chunk3[0].X1() << " ; y1 : " << chunk3[0].Y1()
                << " ; x2 : "<< chunk3[0].X2() << " ; y2 : " << chunk3[0].Y2() << endl;
        */
        
        char chunkFileName[80];
        char chunk_robots[80];
        char key_exits[80];
        
        //write chunks
        sprintf(chunkFileName, "%s", "inputData/Chunks/Chunk-1");
        writeASCII(chunk1, chunkFileName);
        sprintf(chunkFileName, "%s", "inputData/Chunks/Chunk-2");
        writeASCII(chunk2, chunkFileName);
        sprintf(chunkFileName, "%s", "inputData/Chunks/Chunk-3");
        writeASCII(chunk3, chunkFileName);
         
        //write robot positions corresponding chunks
        sprintf(chunk_robots, "%s", "inputData/Chunks/Chunkrobots-1");
        writePositions(rps1, chunk_robots);
        sprintf(chunk_robots, "%s", "inputData/Chunks/Chunkrobots-2");
        writePositions(rps2, chunk_robots);
        sprintf(chunk_robots, "%s", "inputData/Chunks/Chunkrobots-3");
        writePositions(rps3, chunk_robots);
        
        //
        vector<Point> two_key_points;
        two_key_points.push_back(key_exit_chunk1);
        two_key_points.push_back(key_exit_chunk3);
        sprintf(key_exits, "%s", "inputData/Chunks/KeyExits");
        writePositions(two_key_points, key_exits);
}


vector<Object> returnPATH(vector<Point> rbs, Point key_exit, unsigned char flag)
{
    //cout << " the return paht modified process " << endl << endl;
    Object temp_Obj;
    vector<Object> temp_path;
    
    int Numb = 0;
    double min = 5000;
    double dist_btw_points;
    
    //first chunk connect forward
    if(flag == 1)
    {

        for(int i = 0; i < rbs.size(); i++)
        {
            dist_btw_points = distanceOftwoP(key_exit, rbs[i]);
            if(dist_btw_points < min)
            {
                min = dist_btw_points;
                Numb = i;
            }
        }
        

        for(int i = Numb; i > 0; i--)
        {
            if(i == Numb)
            {
                temp_Obj.set(key_exit.X(), key_exit.Y(), 
                                rbs[i-1].X(), rbs[i-1].Y(), i);
                temp_path.push_back(temp_Obj);
            }
            else
            {
                temp_Obj.set(rbs[i].X(), rbs[i].Y(), 
                              rbs[i-1].X(), rbs[i-1].Y(), i);
                temp_path.push_back(temp_Obj);
            }

        }
    }
    
    //last chunk connect backward
    if(flag == 2)
    {

        for(int i = rbs.size() - 1; i >= 0; i--)
        {
            dist_btw_points = distanceOftwoP(key_exit, rbs[i]);
            if(dist_btw_points < min)
            {
                min = dist_btw_points;
                Numb = i;
            }
        }
        
        for(int i = rbs.size() - 1; i > 0; i--)
        {
            if(i == Numb)
            {
                temp_Obj.set(rbs[i+1].X(), rbs[i+1].Y(), 
                          key_exit.X(), key_exit.Y(), i);
                temp_path.pop_back();
                temp_path.push_back(temp_Obj);
            } 
            else
            {
                temp_Obj.set(rbs[i].X(), rbs[i].Y(), 
                              rbs[i-1].X(), rbs[i-1].Y(), i);
                temp_path.push_back(temp_Obj);
            }
           
        }       
    }
    
    return temp_path;
}

vector<Object> consGeometry(vector<Object> chunk, vector<Object> path, unsigned char flag)
{
    Object s1,s2,s3,s4;
    vector<Object> temp, rnt, LR, FB;
    double slope1, slope2;
    double dist1, dist2;
    double length, weidth;
    
    
    if(flag == 1)
    {   
        /*
        for(int i = 0; i < path.size(); i++)
        {

            //find foundation surfaces/informaiton
            slope1 =  (path[i].Y2() - path[i].Y1()) / (path[i].X2() - path[i].X1());
            for(int j = 0; j < chunk.size(); j++)
            {
                slope2 =  (chunk[j].Y2() - chunk[j].Y1()) / (chunk[j].X2() - chunk[j].X1());
                dist1 = perpendicularDis(Point (path[i].X1(), path[i].Y1()), Point (path[i].X2(), path[i].Y2()),
                                            Point (chunk[j].X1(), chunk[j].Y1()));
                dist2 = perpendicularDis(Point (path[i].X1(), path[i].Y1()), Point (path[i].X2(), path[i].Y2()),
                                            Point (chunk[j].X2(), chunk[j].Y2()));

                //left and right side limitation/information
                if(abs(slope1 - slope2) < 0.5 && dist1 < 2000 && dist2 < 2000)
                {
                    LR.push_back(chunk[j]);
                    
                }

                
                //front and back


                //back
            }

            //establish box/rectangle



            rnt.push_back();
        }
        */
        s1.set(8000,-9000, 10000, -7000, 1);
        s2.set(10000, -7000, 21000, -16000,2);
        s3.set(21000, -16000, 20000, -18000,3);
        s4.set(20000, -18000, 8000,-9000, 4);
        rnt.push_back(s1);
        rnt.push_back(s2);
        rnt.push_back(s3);
        rnt.push_back(s4);
        
        s1.set(8300, -7500, 10000, -8500, 5);
        s2.set(10000, -8500, 7700, -12600,6);
        s3.set(7700, -12600, 6000, -12000,7);
        s4.set(6000, -12000, 8300, -7500,8);
        rnt.push_back(s1);
        rnt.push_back(s2);
        rnt.push_back(s3);
        rnt.push_back(s4);
    }
    else
    {
            /*
            //the last path
            slope1 =  (path.back().Y2() - path.back().Y1()) / (path.back().X2() - path.back().X1());
            for(int j = 0; j < chunk.size(); j++)
            {
                slope2 =  (chunk[j].Y2() - chunk[j].Y1()) / (chunk[j].X2() - chunk[j].X1());
                dist1 = perpendicularDis(Point (path.back().X1(), path.back().Y1()), Point (path.back().X2(), path.back().Y2()),
                                            Point (chunk[j].X1(), chunk[j].Y1()));
                dist2 = perpendicularDis(Point (path.back().X1(), path.back().Y1()), Point (path.back().X2(), path.back().Y2()),
                                            Point (chunk[j].X2(), chunk[j].Y2()));

                //left and right side limitation/information
                if(abs(slope1 - slope2) < 0.5 && dist1 < 2000 && dist2 < 2000)
                {
                    LR.push_back(chunk[j]);
                    //weidth = ;
                }

                //front and back


                //back
            }

            //establish box/rectangle



            rnt.push_back();
            */
        s1.set(5400, 3300, 7500, 2900, 1);
        s2.set(7500, 2900, 5000, -12000,2);
        s3.set(5000, -12000, 3000, -12000,3);
        s4.set(3000, -12000, 5400,3300, 4);
        rnt.push_back(s1);
        rnt.push_back(s2);
        rnt.push_back(s3);
        rnt.push_back(s4);
        
    }
    
    
    return rnt;
}

vector<Object> BlcokBoundaryView(vector<Object> view)
{
    //cout << " block all boundary for a view !!! " << endl;
    Object temp_Obj;
    vector<Object> rnt;
    vector<Object> dot_line;
    
    //rnt = view;
    
    for(int i = 0; i < view.size(); i++)
    {
        if(i == view.size()-1)
        {
            temp_Obj.set(view[i].X2(), view[i].Y2(), view[0].X1(), view[0].Y1(),i);
            rnt.push_back(temp_Obj);
            rnt = breakTheLinesInto(rnt);
        }
        else
        {
            temp_Obj.set(view[i].X2(), view[i].Y2(), view[i+1].X1(), view[i+1].Y1(),i);
            rnt.push_back(temp_Obj);
            rnt = breakTheLinesInto(rnt);
            
        }
    }
    
     rnt = addTwoVectorsOfObjects(rnt, view);
    
    return rnt;
}

/*
void RegionAndExit(vector< vector<Object> > allLEs, vector< vector<Object> > allChunks,
                   vector<Point> robpositions, LocalMap LMS, 
                   vector< vector<int> > LM_iD, vector< vector<int> > LM_num,
                   vector<int> Last_chunk_point, vector<int> flag_points)
{
        vector<Object> chunk;                  //chunk map
        vector<Object> modiLM;
        vector<Object> CombinedRegion;         //combined regions
        vector<Point> rps;                     //all robot position, points
        vector<Exit> exits;                    //exits computed between regions
        vector<Object> route_segments;         //path segments
        vector< vector<Object> > chunkWithLMs; //all local maps belonging this 
        vector< vector<Object> > allrps;       
        vector< vector<Object> > allRegions;   //all regions
        vector< vector<Object> > splitSpace;   //splited space
        
        char fileName[80];                     //plot file name

        int cnt = 0;

        for(int n = 0; n < LM_iD.size(); n++) 
        {
            //construct chunk map
            for(int i = 1; i < LM_num[n].size(); i++)
            {
                chunk = addTwoVectorsOfObjects(chunk, LMS.getGlobalMaps()[i]);
            }

            //individual local map put into 
            for(int i = 0; i < allLEs.size(); i++)
            {

                    if(i == LM_iD[n][cnt++]-1)
                    {
                            if(cnt == 0)
                            {
                                //start point of a chunk
                                chunkWithLMs.push_back(allLEs[i]);
                                rps.push_back(robpositions[i]);
                            }
                            else
                            {
                                //intermediate point of a chunk
                                chunkWithLMs.push_back(allLEs[i-1]);
                                rps.push_back(robpositions[i-1]);

                                //chunk point 
                                chunkWithLMs.push_back(allLEs[i]);
                                rps.push_back(robpositions[i]);
                            }
                    }

                    //add intermediate & end points for depicting route in the chunk
                    if(flag_points[n] == 1)
                    {
                        //intermediate points and also the previous one 

                        //then end points of chunk
                        if(i == Last_chunk_point[n]-1)
                            rps.push_back(robpositions[i]);
                    }
                    else
                    {
                        //the last point of chunk put into only
                        if(i == Last_chunk_point[n]-1)
                            rps.push_back(robpositions[i]);
                    }
            }


            //cout << " the size of chunk LMS: " << chunkWithLMs.size() << endl;
            route_segments = RouteCombine(chunk, chunkWithLMs, rps);
            splitSpace = DetectSpace(chunk, chunkWithLMs, route_segments);
            exits = IdentifyExitsFollowRoute(splitSpace, route_segments);
            RegionBlock(chunk, splitSpace, exits, route_segments, 1);
            //sprintf(fileName, "%s%d%s", "Maps/Offline/TEST_Combined_And_exits-", n,".png");
            //plotObjectsColoursAndExits(fileName, chunkWithLMs, route_segments,exits);
        }
    
    
}
*/

/* Crossing one fixed point that on line segment, 
 * computing another one with offset length, which 
 * is gonna perpendicular the given line segment
 */
vector<Exit> ExitPerpendPath(vector<Object> routes)
{
    double offset = 400; //x axis offset 
    double k;
    double a, b, c;
    double x1, y1, x2, y2;
    
    Exit temp_exit;
    vector<Exit> exits;
    
    for(int i = 1; i < routes.size(); i++)
    {
        k = -(1 / ((routes[i].Y2() - routes[i].Y1())/(routes[i].X2() - routes[i].X1())));
        a = routes[i].X1();
        b = routes[i].Y1();
        
        c = b - a * k;
        x1 = a + offset;
        x2 = a - offset;
        y1 = k * x1 + c;
        y2 = k * x2 + c;
        
        temp_exit.set(x1, y1, x2, y2);
        
        if(temp_exit.length() > 1500)
        {
             y1 = b + offset;
             y2 = b - offset;
             x1 = (y1 - c) / k;
             x2 = (y2 - c) / k;
        
             temp_exit.set(x1, y1, x2, y2);
        }
        
        exits.push_back(temp_exit);
        

    }
    
    return exits;
}

/* compute connect parameters based upon connecting information
 * 
 */
pair<double, Point> connect_parameters(ChunkInfo current_chunk, ChunkInfo previous_chunk)
{
    pair<double, Point> parameters; //first is angle, second is phase
    Point phase;
    
    double dist_to_rb, min;
    double dist1, dist2;
    double expAngle, expDist;
    vector<Object> current_first_region, previous_last_region;
    vector<Object> current_first_robot, previous_last_robot;
    
    pair<Object, Object> temp_objs;
    vector <pair<Object, Object> > reference_list;
    
    //
    current_first_region = current_chunk.getMFIS_region()[0];
    previous_last_region = previous_chunk.getMFIS_region().back();
    //
    current_first_robot = current_chunk.getRobots()[0];
    previous_last_robot = previous_chunk.getRobots().back();
            
    //compute surroundings & find common information
    for(int i = 0; i < current_first_region.size(); i++)
    {
        
        for(int j = previous_last_region.size()-1; j >= 0 ; j--)
        {
            
            expAngle = current_first_region[i].getAngleWithLine(previous_last_region[j]);
            expDist = shortestDistanceBtwTwoObjects(current_first_region[i], previous_last_region[j]);
            if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && expDist < 500.0)
            {
                temp_objs.first = current_first_region[i];
                temp_objs.second = previous_last_region[j];
                reference_list.push_back(temp_objs);
            }
                
        }
    }
    
    //optimum reference to compute parameters
    if(reference_list.size() == 1)
    {
        dist1 = distanceOftwoP(reference_list[0].first.getP1(), reference_list[0].second.getP1());
        dist2 = distanceOftwoP(reference_list[0].first.getP2(), reference_list[0].second.getP2());
        
        parameters.first = reference_list[0].first.getAngleWithXaxis()-reference_list[0].second.getAngleWithXaxis();
                
        
        if(dist1 < dist2)
            phase.set(reference_list[0].first.X1() - reference_list[0].second.X1(), reference_list[0].first.Y1() - reference_list[0].second.Y1());
        else
            phase.set(reference_list[0].first.X2() - reference_list[0].second.X2(), reference_list[0].first.Y2() - reference_list[0].second.Y2());
        
        parameters.second = phase;
    }
    else
    {
        if(reference_list.size() > 1)
        {
            for(int i = 0; i < reference_list.size(); i++)
            {
                dist_to_rb = FindDistanceToSegment(current_first_robot[6].getP1(), reference_list[i].first);
                if(i == 0)
                {
                    min = dist_to_rb;
                    temp_objs = reference_list[i];
                }
                else
                {
                    if(dist_to_rb < min)
                    {
                        min = dist_to_rb;
                        temp_objs = reference_list[i];
                    }
                }
            }
            
            
            dist1 = distanceOftwoP(temp_objs.first.getP1(), temp_objs.second.getP1());
            dist2 = distanceOftwoP(temp_objs.first.getP2(), temp_objs.second.getP2());

            parameters.first = temp_objs.first.getAngleWithXaxis()-temp_objs.second.getAngleWithXaxis();


            if(dist1 < dist2)
                phase.set(temp_objs.first.X1() - temp_objs.second.X1(), temp_objs.first.Y1() - temp_objs.second.Y1());
            else
                phase.set(temp_objs.first.X2() - temp_objs.second.X2(), temp_objs.first.Y2() - temp_objs.second.Y2());

            parameters.second = phase;
        }
        else
            if(reference_list.size() < 1)
            {
                parameters.first = 0;
                phase.set(0,0);
                parameters.second = phase;
                cout << " Cannot find reference informaiton surrouding switch point..."<< endl;
            }
    }
    
    vector< vector<Object> > allrefs;
    for(int i = 0; i < reference_list.size(); i++)
    {
        vector<Object> temp;
        temp.push_back(reference_list[i].first);
        temp.push_back(reference_list[i].second);
        allrefs.push_back(temp);
    }
    
    /*
    char fileName[80];
    sprintf(fileName, "%s", "Maps/Offline/matchinginformation.png");                     
    plotObjectsColours(fileName, allrefs);
    */
    
    return parameters;
}



void correct_exit_info(vector<Exit> exits, vector<Object> current_region, vector<Object> previous_mfis)
{
    char mfisFileName[80];
    Exit ref_exit;  
    vector<Exit> transf_exits;
    Object trans_ref_obj; //transfer reference object
    Object convert_ref_exit;
    vector<Object> ref_list1, ref_list2, ref_list;
    vector<Object> trans_ref_list;
    
    ref_exit  = exits[5];
    convert_ref_exit.set(ref_exit.X1(), ref_exit.Y1(), ref_exit.X2(), ref_exit.Y2(), 0);
    for(int i = 0; i < previous_mfis.size(); i++)
    {
        //if(shortestDistanceBtwTwoObjects(convert_ref_exit, previous_mfis[i]) < 1000)
        //    ref_list.push_back(previous_mfis[i]);
        
        if((FindDistanceToSegment(convert_ref_exit.getP1(), previous_mfis[i]) < 1000)
            || (FindDistanceToSegment(convert_ref_exit.getP1(), previous_mfis[i]) < 1000))
        {
            ref_list1.push_back(previous_mfis[i]);
        }
        
        if((FindDistanceToSegment(convert_ref_exit.getP2(), previous_mfis[i]) < 1000)
            || (FindDistanceToSegment(convert_ref_exit.getP2(), previous_mfis[i]) < 1000))
        {
            ref_list2.push_back(previous_mfis[i]);
        }
    }
    
    //find reference/limitation
    
    if(ref_list1.size() > ref_list2.size())
        ref_list = ref_list1;
    else
        ref_list = ref_list2;
    
    //shift information//parameters
    
    trans_ref_list = TransformforToGlobalCoordinate(ref_list, Point (1000,1000), ref_list, 0);
    transf_exits = Transform_Exit_GlobalCoordinate(exits, Point (1000,1000), ref_list, 0);
    //find a rough description using specific surfaces
    
    
    //vector<Exit> test;
    //test.push_back(ref_exit);
    
    sprintf(mfisFileName, "%s", "Maps/Offline/chunk_connection0.png");                     
    plotObjectsOf3KindswithExits(mfisFileName, trans_ref_list, current_region, transf_exits);
}

vector<Exit> _correct_exit_info(vector<Exit> exits, vector<Object> current_region, vector<Object> previous_mfis)
{
    char mfisFileName[80];
    Exit ref_exit; 
    vector<Exit> exit_inside;
    vector<Exit> transf_exits;
    Object trans_ref_obj; //transfer reference object
    Object convert_ref_exit;

    vector<Object> ref_list1, ref_list2, ref_list;
    vector<Object> trans_ref_list;
    vector< vector<Object> > final_list, fina_trans_list;
    
    
    vector<Point_Hull> inter_hull = ObjectToHull(current_region);
    vector<Object> area = ConvexToObject(inter_hull);
    
    for(int i = 0;i < exits.size(); i++)
    {
        if((pointInPolygon(exits[i].getP1(), area) == true)
            && (pointInPolygon(exits[i].getP2(), area) == true))
            exit_inside.push_back(exits[i]);
    }
    
    
    for(int j = 0; j < exit_inside.size(); j++)
    {
        convert_ref_exit.set(exit_inside[j].X1(), exit_inside[j].Y1(), exit_inside[j].X2(), exit_inside[j].Y2(), 0);
        for(int i = 0; i < previous_mfis.size(); i++)
        {
            if(shortestDistanceBtwTwoObjects(convert_ref_exit, previous_mfis[i]) < 1000)
                ref_list.push_back(previous_mfis[i]);

            final_list.push_back(ref_list);
        }
    }
    

    //transfer parameters 
    Point phase_adjust;
    double angle_hadjust;
    
    //shift information//parameters
    for(int i = 0; i < final_list.size(); i++)
    {
        trans_ref_list = TransformforToGlobalCoordinate(final_list[i], Point (1000,1000), final_list[i], 0);
        fina_trans_list.push_back(trans_ref_list);
    }
    transf_exits = Transform_Exit_GlobalCoordinate(exit_inside, Point (1000,1000), ref_list, 0);
    //find a rough description using specific surfaces
    transf_exits = reCompute_exit(current_region, exit_inside);
    
    vector<Object> combine_shift_info;
    for(int i = 0; i < fina_trans_list.size(); i++)
    {
        if(i == 0)
            combine_shift_info = fina_trans_list[i];
        else
            combine_shift_info = addTwoVectorsOfObjects(combine_shift_info, fina_trans_list[i]);
    }
    
    
    sprintf(mfisFileName, "%s", "Maps/Offline/chunk_connection0.png");                     
    plotObjectsOf3KindswithExits(mfisFileName, combine_shift_info, current_region, transf_exits);
    
    return transf_exits;
}

//re-compute/re-locate these exits using specific surface information in current region
vector<Exit> reCompute_exit(vector<Object> current_region, vector<Exit> project_exits)
{
    double dist1, dist2, dist3, dist4, min;
    Exit temp_exit;
    vector<Exit> relocate_exit, portent_exits;
    
    vector<Object> left_potential, right_potential;
    vector<Point> convert_points;
    
    //convert_points = ObjectToPoints(current_region);
    
    for(int i = 0; i < project_exits.size(); i++)
    {
        for(int j = 0; j < current_region.size(); j++)
        {
            dist1 = distanceOftwoP(project_exits[i].getP1(), current_region[j].getP1());
            dist2 = distanceOftwoP(project_exits[i].getP1(), current_region[j].getP2());
            dist3 = distanceOftwoP(project_exits[i].getP2(), current_region[j].getP1());
            dist4 = distanceOftwoP(project_exits[i].getP2(), current_region[j].getP2());
            
            if((dist1 <= 2000) || (dist2 <= 2000))
                left_potential.push_back(current_region[j]);
            if((dist3 <= 2000) || (dist4 <= 2000))
                right_potential.push_back(current_region[j]);
            
        }
        
        portent_exits = Short_between_groups(left_potential, right_potential);
        
        for(int k = 0; k < portent_exits.size(); k++)
        {
            if(k == 0)
            {
                min = portent_exits[k].length();
                temp_exit = portent_exits[k];
            }
            else
            {
                if(min > portent_exits[k].length())
                {
                    min = portent_exits[k].length();
                    temp_exit = portent_exits[k];
                }
            }
        }
        
        relocate_exit.push_back(temp_exit);
        
        //reset left & right lists
        left_potential.clear();
        right_potential.clear();
    }
    
    return relocate_exit;
}



