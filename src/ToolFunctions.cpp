/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cmath>
#include <dirent.h>
#include<algorithm>
#include "Object.H"
#include "readAndwriteASCII.H"
#include "PointAndSurface.H"
#include "RobotPosition.h"
#include "PathPlanning.H"
#include "Plotting.H"
//#include "View.h"
#include "Memory.h"
#include "Laser2Surface.H"
#include "GeometricOp.H"
#include "ChunksOp.H"
#include "CompareASR.H"


#define  PI  3.14159265;
using namespace std;



// for Chunk & Local Maps
vector<int> LMID; 
vector< vector<Object> > ALLRelative;
vector< vector<Object> > ALLGlobal;
vector<Point> ALLRobotPositions;


Memory ReadMemory()
{
    Memory memory;
    vector<Point> PathPointGlobalRobotPs;
    vector<double>PathPointGlobalRobotXAngle;

    vector<Point> AllGlobalRobotPs;
    vector<double> AllGlobalRobotXAngle;

    vector<Point> LocalRobotPs;
    vector<Point> TurningRobotPs;
    
    vector<Point> ReturningRobotPs;

    vector<int> UpdatingNumber;

    char memFilePath [80];
    char AllGlobalRobotPsFile [80];
    char AllGlobalRobotXAngleFile [80];
    char PathPointGlobalRobotPsFile [80];
    char PathPointGlobalRobotXAngleFile [80];
    char localfile[80];
    char turningfile[80];
    char updatingnumbersfile [80];
    char ReturnLocalPosition[80];
    //GlobalRobotPs
    sprintf(memFilePath, "inputData/mem/");
    sprintf(PathPointGlobalRobotPsFile, "%s%s", memFilePath, "/PathPointGlobalRobotPs.txt");
    PathPointGlobalRobotPs = readPoints(PathPointGlobalRobotPsFile);

    sprintf(PathPointGlobalRobotXAngleFile, "%s%s", memFilePath, "/PathPointGlobalRobotXAngle.txt");
    PathPointGlobalRobotXAngle = readDoubleNumbers(PathPointGlobalRobotXAngleFile);

    sprintf(AllGlobalRobotPsFile, "%s%s", memFilePath, "/AllGlobalRobotPs.txt");
    AllGlobalRobotPs = readPoints(AllGlobalRobotPsFile);

    sprintf(AllGlobalRobotXAngleFile, "%s%s", memFilePath, "/AllGlobalRobotXAngle.txt");
    AllGlobalRobotXAngle = readDoubleNumbers(AllGlobalRobotXAngleFile);

    sprintf(localfile, "%s%s", memFilePath, "/LocalRobotPs.txt");
    LocalRobotPs = readPoints(localfile);

    sprintf(turningfile, "%s%s", memFilePath, "/TurningRobotPs.txt");
    TurningRobotPs = readPoints(turningfile);

    sprintf(updatingnumbersfile, "%s%s", memFilePath, "/UpdatingNumber.txt");
    UpdatingNumber = readNumbers(updatingnumbersfile);
    
    sprintf(ReturnLocalPosition, "%s%s", memFilePath, "/ReturnPositions.txt");
    ReturningRobotPs = readPoints(ReturnLocalPosition);

    memory.setAllGlobalRobotPs(AllGlobalRobotPs);
    memory.setAllGlobalRobotXAngle(AllGlobalRobotXAngle);

    memory.setPathPointGlobalRobotPs(PathPointGlobalRobotPs);
    memory.setPathPointGlobalRobotXAngle(PathPointGlobalRobotXAngle);

    memory.setLocalRobotPs(LocalRobotPs);

    memory.setTurningRobotPs(TurningRobotPs);

    memory.setUpdatingNumber(UpdatingNumber);
    
    memory.setReturningRobotPs(ReturningRobotPs);

    return memory;

}

vector<PointXY> PointsToPointsXY(vector<Point> points)
{

            vector<PointXY> vpxy;
            for (unsigned i = 0; i < points.size(); i++)
            {
                PointXY pxy(points[i].X(), points[i].Y());
                vpxy.push_back(pxy);
            }
            return vpxy;
}

/*
long getShortestDistance(Object a, Object b)
{
    Point a1(a.X1(), a.Y1());
    Point a2(a.X2(), a.Y2());
    Point b1(b.X1(), b.Y1());
    Point b2(b.X2(), b.Y2());
    long a1b1 = distanceOftwoP(a1, b1);
    long a1b2 = distanceOftwoP(a1, b2);
    long a2b1 = distanceOftwoP(a2, b1);
    long a2b2 = distanceOftwoP(a2, b2);
    long alllength[4] = {a1b1, a1b2, a2b1, a2b2};

    sort(alllength, alllength + 4);
    return alllength[0];
}
 */
vector<RobotPosition> GlobalPointsToLocalPositions(vector<Point> RoutePoints)
{
           vector<RobotPosition> RobotPositions;

           Point p, np, pp;
           double forwardDistance, backwardDistance, angle;
           Object a, b;

           for (unsigned i = 0; i < RoutePoints.size(); i++)
           {
                      RobotPosition robotposition;
                      p.set(0, 0);
                      robotposition.setPositionPoint(p);
                      if (i < RoutePoints.size() - 1)
                      {
                                 forwardDistance = distanceOftwoP(RoutePoints[i], RoutePoints[i + 1]);

                                 np.set(0, forwardDistance);
                                 robotposition.setNextPositionPoint(np);
                      }
                      if (i < RoutePoints.size() - 2)
                      {
                                 a.set(RoutePoints[i].X(), RoutePoints[i].Y(), RoutePoints[i + 1].X(), RoutePoints[i + 1].Y(), 0);
                                 b.set(RoutePoints[i + 1].X(), RoutePoints[i + 1].Y(), RoutePoints[i + 2].X(), RoutePoints[i + 2].Y(), 1);
                                 angle = getAngleofTwoObjects(a, b) - 180;
                                 robotposition.setNextTurnOrientation(angle);
                      }
                      if (i > 0)
                      {
                                 backwardDistance = distanceOftwoP(RoutePoints[i - 1], RoutePoints[i]);
                                 pp.set(0, backwardDistance);
                                 robotposition.setPreviousPositionPoint(pp);
                      }
                      if (i > 1)
                      {
                                 a.set(RoutePoints[i - 2].X(), RoutePoints[i - 2].Y(), RoutePoints[i - 1 ].X(), RoutePoints[i - 1].Y(), 0);
                                 b.set(RoutePoints[i - 1].X(), RoutePoints[i - 1].Y(), RoutePoints[i].X(), RoutePoints[i].Y(), 1);
                                 angle = 180 - getAngleofTwoObjects(a, b);
                                 robotposition.setPreviousTurnOrientation(angle);
                      }
                      RobotPositions.push_back(robotposition);

           }
           return RobotPositions;
}

void plotPointsAsObjects(char filename[], vector<Point> points)
{
            vector<Object> objs;
            Object o;
            cout << "size:" << points.size() << endl;
            for (unsigned i = 0; i < points.size() - 1; i++)
            {
                        o.set(points[i].X(), points[i].Y(), points[i + 1].X(), points[i + 1].Y(), i);
                        objs.push_back(o);
            }
            plotObjects(filename, objs);
}


LocalMap TransViewIntoGloabl(vector<Object> MapofLMs, vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
{
           MyRobot robot(0, 0);
           LocalMap rtn;
           vector<Object> temp;
           char LocalMFileName[100];
           double x1, y1, x2, y2;

           //combine a new local map into global map         
           ALLRelative.push_back(addView);
           rtn.setRelativeMaps(ALLRelative);

           angle = (angle / 180) * PI;

           for (int i = 0; i<int(addView.size()); i++)
           {
                      x1 = addView[i].X1() * cos(angle) - addView[i].Y1() * sin(angle) + coord.X();
                      y1 = addView[i].X1() * sin(angle) + addView[i].Y1() * cos(angle) + coord.Y();

                      x2 = addView[i].X2() * cos(angle) - addView[i].Y2() * sin(angle) + coord.X();
                      y2 = addView[i].X2() * sin(angle) + addView[i].Y2() * cos(angle) + coord.Y();

                      Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());
                      s.setKP(1);
                      s.setLocalEnvID(num);
                      temp.push_back(s);
           }

           MapofLMs = addTwoVectorsOfObjects(MapofLMs, temp);

           ALLGlobal.push_back(temp);
           ALLRobotPositions.push_back(coord);
           LMID.push_back(num);
           rtn.setGlobalMaps(ALLGlobal);
           rtn.setMapIDs(LMID);
           rtn.setGlobalPositions(ALLRobotPositions);

           return rtn;
}

vector<Object> Globalview(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
{
           MyRobot robot(0, 0);
           vector<Object> temp;
           char GViewMFileName[100];
           double x1, y1, x2, y2;

           angle = (angle / 180) * PI;

           for (int i = 0; i<int(addView.size()); i++)
           {
                      x1 = addView[i].X1() * cos(angle) - addView[i].Y1() * sin(angle) + coord.X();
                      y1 = addView[i].X1() * sin(angle) + addView[i].Y1() * cos(angle) + coord.Y();

                      x2 = addView[i].X2() * cos(angle) - addView[i].Y2() * sin(angle) + coord.X();
                      y2 = addView[i].X2() * sin(angle) + addView[i].Y2() * cos(angle) + coord.Y();

                      Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());
                      s.setKP(1);
                      temp.push_back(s);
           }

           return temp;
}


/*
void matchPathWithSpaceUPPERVersion(ArRobot& robot, ArSick& sick, Point pos)
{
        Point temp(pos.X(), pos.Y());
        Point near;
        Point Interpoint;
        Point midPOfDoor;
        char viewFilename[100];
        unsigned char flagInter = 0;
        MyRobot myrobot(0,0);
        vector<Object> currentView;
        Object inters;
        Object Obj;
        double angle = 0;
        double dist = 0;
        double approach = 0;
        double offset1 = 800;
        double offset2 = 400;
        double d1, d2;
        double adjust = 0;
        pair<double,double> infor;
        int typeOfMove; //1-- moving forward, 2 -- right block 3 -- left block
         
        double path;
        if(pos.X() > 0)
          path = pos.X();
        else
          if(pos.X() < 0)
            path = 0 - pos.X();
        
        cout<<"----This is going to process path match----"<<endl;
        
        
st:    if(temp.X() > 0)
            angle = -90;
        else
            if(temp.X() < 0)
                angle = 90;
            else
                angle = 0;
 
        temp = ExecutionAndReturn(robot, angle, dist, temp);
        
st2:    currentView = scanAndSaveView(sick, numb);
        sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
        plotObjects(viewFilename, myrobot.getRobot(), currentView); 
        numb++;
        
        //parallel the corridor
        adjust = parallelTheSpace(currentView);
        temp = ExecutionAndReturn(robot, adjust, 0, temp); 
        
        currentView = scanAndSaveView(sick, numb);
        sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
        plotObjects(viewFilename, myrobot.getRobot(), currentView); 
        numb++;
        
        typeOfMove = pathSpaceDetect(currentView, path);
        cout<<"----The type number is : "<<typeOfMove<<endl;
        switch(typeOfMove)//1 front free, 2 left free, 3 right free, 4 both blocked 
        {
                case 1: //temp = ExecutionAndReturn(robot, 0, path/2, temp); 
                        temp = ExecutionAndReturn(robot, 0, temp.Y()/2, temp);
                        //half way and correct its direction
                        currentView = scanAndSaveView(sick, numb);
                        sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
                        plotObjects(viewFilename, myrobot.getRobot(), currentView); 
                        numb++;
                        
                        //parallel the corridor
                        adjust = parallelTheSpace(currentView);
                        temp = ExecutionAndReturn(robot, adjust, 0, temp); 
                        //last half way to reach the destination
                        //temp = ExecutionAndReturn(robot, 0, path/2, temp); 
                        temp = ExecutionAndReturn(robot, 0, temp.Y(), temp);
                        break;
                case 2:if(isCorridor(currentView) == true)
                       {
                         cout<<"----But, this is corridor!----"<<endl;
                         goto st3;
                       }
                
                       Interpoint = intersectedPoint(currentView, Point (300, 0), Point (300, path));
                            //calculate distance from the point to robot
                            approach = sqrt(Interpoint.X() * Interpoint.X() + Interpoint.Y() * Interpoint.Y());

                            //if approach < 2000
                            if(approach < 1000)
                            {
                                    temp = ExecutionAndReturn(robot, -angle, 500, temp);
                                    goto st;
                            }
                            else
                            {
                                if(approach > 1000 && approach < 2000)
                                {
                                    midPOfDoor = midOfDoor(currentView, Point (300, 0), Point (300, path));
                                    if(midPOfDoor.getID() == 0)
                                    {
                                        near.set(Interpoint.X() - offset1, Interpoint.Y());
                                        infor = DistAndAngle(near);
                                        temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                        temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                        goto st2;
                                    }
                                    else
                                    {
                                        infor = DistAndAngle(midPOfDoor);
                                        temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                        temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                        goto st2;
                                    }
                                }
                                else
                                  if(approach > 2000)
                                  {
                                          //approach the obstacle
                                          temp = ExecutionAndReturn(robot, -0, Interpoint.Y() - 1500, temp);
                                          currentView = scanAndSaveView(sick, numb);
                                          sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
                                          plotObjects(viewFilename, myrobot.getRobot(), currentView); 
                                          numb++;
      
                                          if(interSectWithLine(currentView, Point (300, 0), Point (300, temp.Y())) == true) // intersected with a surface
                                          {
                                                          inters = intersectForObject(currentView, Point (300, 0), Point (300, temp.Y()));
                                                          near.set(inters.X1() - offset1, inters.Y1());
      
                                                          infor = DistAndAngle(near);
                                                          temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                                          temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                          }
                                          //reset position
                                          temp.set(0, temp.Y());
                                          //reach the destination 
                                          infor = DistAndAngle(temp);
                                          temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                          temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                          
                                  }
                              }
                            break;
                case 3:if(isCorridor(currentView) == true)
                       {
                         cout<<"----But, this is corridor!----"<<endl;
                         goto st3;
                       }
                
                        Interpoint = intersectedPoint(currentView, Point (-300, 0), Point (-300, path));
                            //calculate distance from the point to robot
                            approach = sqrt(Interpoint.X() * Interpoint.X() + Interpoint.Y() * Interpoint.Y());

                            if(approach < 1000)
                            {
                                    temp = ExecutionAndReturn(robot, -angle, 500, temp);
                                    goto st;
                            }
                            else
                            {
                                if(approach > 1000 && approach < 2000)
                                {
                                  midPOfDoor = midOfDoor(currentView, Point (-300, 0), Point (-300, path));
                                  if(midPOfDoor.getID() == 0)
                                  {
                                    near.set(Interpoint.X() + offset1, Interpoint.Y());
                                    infor = DistAndAngle(near);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                    goto st2;
                                  }
                                  else
                                  {
                                     infor = DistAndAngle(midPOfDoor);
                                     temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                     temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                     goto st2;
                                  }
                                }
                                else
                                  if(approach > 2000)
                                  {
                                          //approach the obstacle 
                                          temp = ExecutionAndReturn(robot, -0, Interpoint.Y() - 1500, temp);
                                          currentView = scanAndSaveView(sick, numb);
                                          sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
                                          plotObjects(viewFilename, myrobot.getRobot(), currentView); 
                                          numb++;
      
                                          if(interSectWithLine(currentView, Point (-300, 0), Point (-300, temp.Y())) == true) // intersected with a surface
                                          {
                                                          inters = intersectForObject(currentView, Point (-300, 0), Point (-300, temp.Y()));
                                                          near.set(inters.X2() + offset1, inters.Y2());
      
                                                          infor = DistAndAngle(near);
                                                          temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                                          temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                          }
                                          
                                          //reset position
                                          temp.set(0, temp.Y());
                                          //reach the destination 
                                          infor = DistAndAngle(temp);
                                          temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                          temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                          
                                  }
                              }
                            break;
                case 4:
                        if(isCorridor(currentView) == true)
                        {
st3:                        cout<<"It is in corridor now !!!!"<<endl;
                            
                            currentView = scanAndSaveView(sick, numb);
                            sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
                            plotObjects(viewFilename, myrobot.getRobot(), currentView); 
                            numb++;
                            //parallel the corridor
                            adjust = parallelTheSpace(currentView);
                            temp = ExecutionAndReturn(robot, adjust, 0, temp); 
                            
                            currentView = scanAndSaveView(sick, numb);
                            sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
                            plotObjects(viewFilename, myrobot.getRobot(), currentView); 
                            numb++;
                            
                                                        //left and right +- 200 lines for checking intersected
                            if(interSectWithLine(currentView, Point (-200, 0), Point (-200, temp.Y())) == true)
                            {
                                    /*
                                    //reach the potential door, then carry on
                                    inters = intersectForObject(currentView, Point (-200, 0), Point (-200, temp.Y()));
                                    near.set(inters.X2() + offset2, inters.Y2());

                                    infor = DistAndAngle(near);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, temp.Y(), temp); 
                                    */
                                    
/*                                   
                                    midPOfDoor = midOfDoor(currentView, Point (200, 0), Point (200, temp.Y()));
                                    infor = DistAndAngle(midPOfDoor);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, temp.Y(), temp); 
                                    
                            }
                            else
                            {
                                if(interSectWithLine(currentView, Point (200, 0), Point (200, temp.Y())) == true)
                                {
                                        /*
                                        //reach the potential door, then carry on
                                        inters = intersectForObject(currentView, Point (200, 0), Point (200, temp.Y()));
                                        near.set(inters.X1() - offset2, inters.Y1());

                                        infor = DistAndAngle(near);
                                        temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                        temp = ExecutionAndReturn(robot, -infor.second, temp.Y(), temp); 
                                        */
/*                                    
                                    midPOfDoor = midOfDoor(currentView, Point (-200, 0), Point (-200, temp.Y()));
                                    infor = DistAndAngle(midPOfDoor);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, temp.Y(), temp); 
                                    
                                }
                                else
                                {
                                        //reset temp position/destination position
                                        temp.set(0, temp.Y());
                                        infor = DistAndAngle(temp);
                                        temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                }
                            }
                            /*
                            //reset temp position/destination position
                            temp.set(0, temp.Y());
                            infor = DistAndAngle(temp);
                            temp = ExecutionAndReturn(robot, infor.second, infor.first, temp);
                            */ 
 /*                       }
                        else
                        {
                          // move to the end of the object angle > 0 -- X2, angle < 0 -- X1
                          
                          
                            temp = ExecutionAndReturn(robot, -angle, 600, temp);
                                goto st;
                        }
                        break;
                default:cout<<"----Sorry, wrong type of movement!!!!----"<<endl; break;
        }
}
*/

vector<Object> CommonSurface(vector< vector<Object> > views)
{
    Object temp_Obj;
    vector<Object> temp;
    //cout << "common surface process " << endl;
    //cout << " size of regions : " << views.size() << endl;
    for(int i = 0; i < views.size()-1; i++)
    {
        for(int a = 0; a < views[i].size(); a++)
        {
            for(int b = 0; b < views[i+1].size();b++)
            {
                if((views[i][a].X1() == views[i+1][b].X1()) 
                           && (views[i][a].X2() == views[i+1][b].X2()))
                    temp.push_back(views[i][a]);
            }
        }
    }
    
    return temp;
}

int reference_endpoint(Object s1, Object s2)
{
    int ref_point = 0;
    double distp1p1, distp1p2, distp2p1, distp2p2;
    
    distp1p1 = distanceOftwoP(s1.getP1(), s2.getP1());
    distp1p2 = distanceOftwoP(s1.getP1(), s2.getP2());
    distp2p1 = distanceOftwoP(s1.getP2(), s2.getP1());
    distp2p2 = distanceOftwoP(s1.getP2(), s2.getP2());
    
    //if ref[0] endpoint1 is ref point -- ref point is 2
    if(((distp1p1 < distp2p1) && (distp1p1 < distp2p2))
            || ((distp1p2 < distp2p1) && (distp1p2 < distp2p2)))
    {
        ref_point = 1;
    }
    else
    {
        //if ref[0] endpoint2 is ref point -- ref point is 1
        if(((distp2p1 < distp1p1) && (distp2p1 < distp1p2))
                || ((distp2p2 < distp1p1) && (distp2p2 < distp1p2)))
        {
            ref_point = 2;
        }
    }
    
    return ref_point;
}

double reference_endpoint_neardist(Object s1, Object s2)
{
    double dist, min;
    double distp1p1, distp1p2, distp2p1, distp2p2;
    
    distp1p1 = distanceOftwoP(s1.getP1(), s2.getP1());
    distp1p2 = distanceOftwoP(s1.getP1(), s2.getP2());
    distp2p1 = distanceOftwoP(s1.getP2(), s2.getP1());
    distp2p2 = distanceOftwoP(s1.getP2(), s2.getP2());
    
    //if ref[0] endpoint1 is ref point -- ref point is 2
    if((distp1p1 < distp1p2) && (distp1p1 < distp2p1) && (distp1p1 < distp2p2))
            min = distp1p1;
    if((distp1p2 < distp1p1) && (distp1p2 < distp2p1) && (distp1p2 < distp2p2))
            min = distp1p2;
    if((distp2p1 < distp1p1) && (distp2p1 < distp1p2) && (distp2p1 < distp2p2))
            min = distp2p1;
    if((distp2p2 < distp1p1) && (distp2p2 < distp1p2) && (distp2p2 < distp2p1))
            min = distp2p2;
    
    return min;
}

int common_endpoint(Object ref, Object det)
{
    
    if((ref.getP1() == det.getP1()) || (ref.getP1() == det.getP2()))
        return 1;
    if((ref.getP2() == det.getP1()) || (ref.getP2() == det.getP2()))
        return 2;
}

vector<Object> remove_noise_surfaces(vector<Object> polygon)
{

    Point ref_p1, ref_p2, ref_p;
    vector<Object> large_surfaces;
    vector<Object> temp;
    
    temp = polygon;
    
    for(int i = 0; i < polygon.size(); i++)
    {
        int cnt1 = 0, cnt2 = 0;
        //ref_p1 = polygon[i].getP1();
        //ref_p2 = polygon[i].getP2();
        
        ref_p1.set(polygon[i].getP1().X(), polygon[i].getP1().Y());
        ref_p2.set(polygon[i].getP2().X(), polygon[i].getP2().Y());
        
        for(int j = i+1; j < polygon.size(); j++)
        {
            
            if((polygon[j].getP1() == ref_p1) || (polygon[j].getP2() == ref_p1))
                cnt1++;
            if((polygon[j].getP2() == ref_p2) || (polygon[j].getP1() == ref_p2))
                cnt2++; 
        }

        if(cnt1 >= 3)
        {
            ref_p = ref_p1;
            break;
        }
        else
        {
             if(cnt2 >= 3)
             {
                ref_p = ref_p2;
                break;
             }
        }
    }
    
    if((ref_p.X() != NULL) && (ref_p.Y() != NULL))
    {
        for(int i = 0; i < temp.size(); i++)
        {
            if(((temp[i].getP1() == ref_p) || (temp[i].getP2() == ref_p))
                && (temp[i].length() > 2000))
            {
                temp.erase(temp.begin()+i);
                i--;
            }
        }
        
        char plotFile[80];
        sprintf(plotFile, "%s", "Maps/Offline/TEST_enduring_map_polygon**.png");
        plotObjects(plotFile, temp);
                        
    }
    
    return temp;
    
}

void correct_polygon(vector<Object> &poly)
{

    Object temp;
    vector<Object> ref_surfaces;
    for(int i = 0; i < poly.size(); i++)
    {
        if(((poly[i].X1() == poly[i].Y1() == 0) || (poly[i].X2() == poly[i].Y2() == 0))
            && (poly[i].length() > 8000))
        {
            if(ref_surfaces.size() >= 2)
                break;
            else
            {
                ref_surfaces.push_back(poly[i]);
                poly.erase(poly.begin()+i);
                i--;
            }
            
        }
    }
    
    if(ref_surfaces.size() == 2)
    {
        double x1, x2, y1, y2;
        
        x1 = ref_surfaces[0].X1() == 0? ref_surfaces[0].X2():ref_surfaces[0].X1();
        y1 = ref_surfaces[0].Y1() == 0? ref_surfaces[0].Y2():ref_surfaces[0].Y1();
        x2 = ref_surfaces[1].X1() == 0? ref_surfaces[1].X2():ref_surfaces[1].X1();
        y2 = ref_surfaces[1].Y1() == 0? ref_surfaces[1].Y2():ref_surfaces[1].Y1();
        
        temp.set(x1,y1, x2, y2, ref_surfaces[0].getID());
        poly.insert(poly.begin()+ref_surfaces[0].getID(), temp);
    }
}




vector<Object> ASRs_combine(vector< vector<Object> > asrs)
{
    vector<Object> temp;
    for(int m = 0; m < asrs.size(); m++)
    {
        if(m == 0)
            temp = asrs[m];
        else
            temp = addTwoVectorsOfObjects(temp, asrs[m]);
    }
    
    return temp;
}

Point min_distance(vector<Point> points, Point ref_p)
{
    double dist, min;
    Point temp;
    
    for(int i = 0; i < points.size(); i++)
    {
        dist = distanceOftwoP(ref_p, points[i]);
        if(i == 0)
        {
            min = dist;
            temp = points[i];
        }
        else
        {
            if(min > dist)
            {
                 min = dist;
                temp = points[i];
            }
        }
    }
    
    return temp;
}

Point max_distance(vector<Point> points, Point ref_p)
{
    double dist, max;
    Point temp;
    
    for(int i = 0; i < points.size(); i++)
    {
        dist = distanceOftwoP(ref_p, points[i]);
        if(i == 0)
        {
            max = dist;
            temp = points[i];
        }
        else
        {
            if(max < dist)
            {
                max = dist;
                temp = points[i];
            }
        }
    }
    
    return temp;
}

//for data5set3
vector<Object> adjust_box_temporary(vector<Object> sides_of_geo)
{
            Object side1, side2, side3, side4;
            vector<Object> test;

            side1 = TransformforToGlobalCoordinate(sides_of_geo[0], Point (800,1000), 0); //1st side
            side2 = TransformforToGlobalCoordinate(sides_of_geo[2], Point (4300,-1300), 0); //2nd side
            side4 = TransformforToGlobalCoordinate(sides_of_geo[3], Point (-600,-1200), 0); //4th side

            test.push_back(side1);
            test.push_back(side2);
            test.push_back(sides_of_geo[1]);
            test.push_back(side4);

            Point p1, p2, p3, p4;
            p1 = intersectPointTwolineEquations(test[0], test[1]);
            p2 = intersectPointTwolineEquations(test[1], test[3]);
            p3 = intersectPointTwolineEquations(test[2], test[3]);
            p4 = intersectPointTwolineEquations(test[2], test[0]);

            side1.set(p1.X(), p1.Y(), p2.X(), p2.Y(), 0);
            side2.set(p2.X(), p2.Y(), p3.X(), p3.Y(), 1);
            side3.set(p3.X(), p3.Y(), p4.X(), p4.Y(), 2);
            side4.set(p4.X(), p4.Y(), p1.X(), p1.Y(), 3);
            test.clear();
            test.push_back(side1);
            test.push_back(side2);
            test.push_back(side3);
            test.push_back(side4);


            //sprintf(mfisFileName, "%s%d%s", "Maps/Offline/robot_and_view-", v, ".png");                   
            //plotObjectsOf3Kinds(mfisFileName, adjust_info.first,adjust_info.second, test);
            //waitHere();

            return test;
}


vector<Object> adjust_box_temporary_2(vector<Object> sides_of_geo, vector<Object> view)
{
            Object side1, side2, side3, side4;
            vector<Object> test;

            //side1 = TransformforToGlobalCoordinate(sides_of_geo[0], Point (800,1000), 0); //1st side
            //side2 = TransformforToGlobalCoordinate(sides_of_geo[2], Point (4300,-1300), 0); //2nd side
            //side4 = TransformforToGlobalCoordinate(sides_of_geo[3], Point (-600,-1200), 0); //4th side
            
            side3 = TransformforToGlobalCoordinate(sides_of_geo[2], Point (-6800,7000), 0); //4th side

            test.push_back(sides_of_geo[0]);
            test.push_back(sides_of_geo[1]);
            test.push_back(side3);
            test.push_back(sides_of_geo[3]);

            Point p1, p2, p3, p4;
            p1 = intersectPointTwolineEquations(test[0], test[1]);
            p2 = intersectPointTwolineEquations(test[1], test[3]);
            p3 = intersectPointTwolineEquations(test[2], test[3]);
            p4 = intersectPointTwolineEquations(test[2], test[0]);

            side1.set(p1.X(), p1.Y(), p2.X(), p2.Y(), 0);
            side2.set(p2.X(), p2.Y(), p3.X(), p3.Y(), 1);
            side3.set(p3.X(), p3.Y(), p4.X(), p4.Y(), 2);
            side4.set(p4.X(), p4.Y(), p1.X(), p1.Y(), 3);
            test.clear();
            test.push_back(side1);
            test.push_back(side2);
            test.push_back(side3);
            test.push_back(side4);


            //sprintf(mfisFileName, "%s%d%s", "Maps/Offline/robot_and_view-", v, ".png");                   
            //plotObjectsOf3Kinds(mfisFileName, adjust_info.first,adjust_info.second, test);
            //waitHere();

            return test;
}

/*
bool is_extend_line(Object line1, Object line2)
{
    int flag = 0;
    
    if(shortestDistanceBtwTwoObjects() < 500)
    {
        
    }
    
    if(flag == 1)
        return true;
    else
        return false;
}
*/

