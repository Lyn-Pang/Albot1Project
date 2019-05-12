/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>
#include <vector>
#include <algorithm>
#include "PathPlanning.H"
#include "Object.H"
#include "GeometricOp.H"
#include "mfisOp.H"
#include "Plotting.H"
#include "CompareASR.H"
#include "readAndwriteASCII.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"
#include "Aria.h"
#include "Laser2Surface.H"
#include "RobotFuncs.H"

using namespace std;
#define PI 3.14159265

int numb = 100;

//Always transform the destination point along traveling
Point whereDestination(Point desti, double angle, double distance)
{
        Point temp;
        double rotat;
        double x0, y0;
        double x, y;
        if(angle < 0)
            angle = 360 + angle;
        else
            angle = angle;
        
        rotat =  PI / 180 * angle;

        y0 = cos(rotat) * distance;
        x0 = sin(rotat) * distance;
        if(angle > 0)
            x0 = -x0;
        
        x = (desti.X() - x0) * cos(rotat) + (desti.Y() - y0) * sin(rotat);
        y = (desti.Y() - y0) * cos(rotat) - (desti.X() - x0) * sin(rotat);

        temp.set(x, y);
        return temp;
}

double crossedPoint(Point p1, Point p2, Point q1, Point q2)
{
            Point temp;
            double tmpLeft,tmpRight;
            double x, y;
            double temp_distTOrig;
            tmpLeft = (q2.X() - q1.X()) * (p1.Y() - p2.Y()) - (p2.X() - p1.X()) * (q1.Y() - q2.Y());
            tmpRight = (p1.Y() - q1.Y()) * (p2.X() - p1.X()) * (q2.X() - q1.X()) + q1.X() * (q2.Y() - q1.Y()) * (p2.X() - p1.X()) - p1.X() * (p2.Y() - p1.Y()) * (q2.X() - q1.X());

            x = tmpRight / tmpLeft;

            tmpLeft = (p1.X() - p2.X()) * (q2.Y() - q1.Y()) - (p2.Y() - p1.Y()) * (q1.X() - q2.X());
            tmpRight = p2.Y() * (p1.X() - p2.X()) * (q2.Y() - q1.Y()) + (q2.X()- p2.X()) * (q2.Y() - q1.Y()) * (p1.Y() - p2.Y()) - q2.Y() * (q1.X() - q2.X()) * (p2.Y() - p1.Y()); 
            y = tmpRight / tmpLeft;
            
            temp.set(x,y);
            temp_distTOrig = sqrt(temp.X() * temp.X() + temp.Y() * temp.Y());
            return temp_distTOrig;
}


int pathSpaceDetect(vector<Object> CV, double pathSeg)
{
        Point pathLeft1, pathLeft2, pathRight1, pathRight2;
        Point LCrossP, RCrossP;

        double Ldist, Rdist;
        double Loffset = -300;
        double Roffset = 300;
        int flag = 0;
        pathLeft1.set(Loffset, 0);
        pathLeft2.set(Loffset, pathSeg);
        pathRight1.set(Roffset, 0);
        pathRight2.set(Roffset, pathSeg);
        
        if(interSectWithLine(CV, pathLeft1, pathLeft2) == false)
        {
                if(interSectWithLine(CV, pathRight1, pathRight2) == false)
                    flag = 1; // front free space
                else
                    flag = 2; // left free, right blocked 
        }
        else
        {
                if(interSectWithLine(CV, pathRight1, pathRight2) == false)
                    flag = 3; // right free, left blocked
                else
                    flag = 4; // both blocked
        }
        
        return flag;
}

Point ExecutionAndReturn(ArRobot& robot, double angle, double dist, Point pos)
{
    Point temp;
    
        // Stop the robot and wait a bit
        robot.lock();
        robot.stop();
        robot.clearDirectMotion();
        robot.unlock();
        ArUtil::sleep(2000);

        //save odometer and orientation angle before moving       
        robot.lock();
        double oldDistance = robot.getOdometerDistance();
        double oldAngle = robot.getTh();
        robot.unlock();

        //movement execution
        setHeading(robot, angle);
        moveDistance(robot, dist);
        
        // Stop the robot and wait a bit
        robot.lock();
        robot.stop();
        robot.clearDirectMotion();
        robot.unlock();
        ArUtil::sleep(2000);
        
        temp = whereDestination(pos, angle, dist);
        
        return temp;
        
}


double parallelTheSpace(vector<Object> CV)
{
        double angle, a1,a2, d1, d2,k1,k2;
        double threshold = 3000;
        int cnt = 0;
        vector<int> id;

        for(int i = 0; i < CV.size(); i++)
        {
                if(CV[i].length() > threshold)
                {
                    cnt++;
                    id.push_back(i);
                }
        }
        cout<< "The large object number is : "<< cnt<<endl;
        switch(cnt)
        {
            case 1:   d1 = CV[id[0]].length();
                           a1 = asin(abs(CV[id[0]].X1() - CV[id[0]].X2()) / d1);
                           k1 = (CV[id[0]].Y2() - CV[id[0]].Y1()) / (CV[id[0]].X2() - CV[id[0]].X1());
                           if(k1 > 0 && abs(CV[id[0]].X1() - CV[id[0]].X2()) < 1500)
                               angle = -(180 / PI * a1);
                           else
                           {
                             if(k1 < 0 && abs(CV[id[0]].X1() - CV[id[0]].X2()) < 1500)
                               angle = (180 / PI * a1);
                             else
                               angle = 0;
                           }
                           break;
            case 2:   d1 = CV[id[0]].length();
                           a1 = asin(abs(CV[id[0]].X1() - CV[id[0]].X2()) / d1);
                           k1 = (CV[id[0]].Y2() - CV[id[0]].Y1()) / (CV[id[0]].X2() - CV[id[0]].X1());
                           d2 = CV[id[1]].length();
                           a2 = asin(abs(CV[id[1]].X1() - CV[id[1]].X2()) / d2);
                           k2 = (CV[id[1]].Y2() - CV[id[1]].Y1()) / (CV[id[1]].X2() - CV[id[1]].X1());
                           
                           if(abs(k1-k2) < 100 && abs(CV[id[0]].X1() - CV[id[0]].X2()) < 1500)
                           {
                                if(k1 > 0)
                                    angle = -(180 / PI * a1);
                                else
                                    angle = (180 / PI * a1);
                           }
                           else
                               angle = 0;
                           break;
            default: angle = 0;break;
        }

        return angle;
}

Point midOfDoor(vector<Object> CV, Point p1, Point p2)
{
        Point rtn;
        Point p3, p4;
        vector<Exit> exits;
        unsigned char flag = 0;
        double x, y;

        exits = findShortestExits(CV);

        for(int i = 0; i < exits.size(); i++)
        {
                p3.set(exits[i].X1(), exits[i].Y1());
                p4.set(exits[i].X2(), exits[i].Y2());
                if(twoLinintersect(p1, p2, p3, p4) == true && exits[i].length() < 1500)
                {
                    x = (exits[i].X1() + exits[i].X2()) / 2;
                    y = (exits[i].Y1() + exits[i].Y2()) / 2;

                    rtn.set(x,y);
                    rtn.setID(1);
                    break;
                }
                else
                {
                    rtn.set(0,0);
                    rtn.setID(0);
                }
        }
        
        return rtn;
}

bool isCorridor(vector<Object> CV)
{
        double angle, a1,a2, d1, d2;
        double k1 = 0;
        double k2 = 0;
        double threshold = 2000;
        int cnt = 0;
        vector<int> id;

        for(int i = 0; i < CV.size(); i++)
        {
                if(CV[i].length() > threshold)
                {
                    cnt++;
                    id.push_back(i);
                }
        }


        if(cnt == 2)
        {
            d1 = CV[id[0]].length();
            a1 = asin(abs(CV[id[0]].X1() - CV[id[0]].X2()) / d1);
            k1 = (CV[id[0]].Y2() - CV[id[0]].Y1()) / (CV[id[0]].X2() - CV[id[0]].X1());
            d2 = CV[id[1]].length();
            a2 = asin(abs(CV[id[1]].X1() - CV[id[1]].X2()) / d2);
            k2 = (CV[id[1]].Y2() - CV[id[1]].Y1()) / (CV[id[1]].X2() - CV[id[1]].X1());
            
            cout<<"the x: "<<CV[id[0]].X1()<<" ; and y: "<<CV[id[0]].Y1();
            cout<<" the k1 : "<< k1 <<" ; and k2: "<< k2 << endl; 
        
        }
        
        if(abs(k1-k2)< 100 && k1 != 0 && k2 != 0 && abs(CV[id[0]].X1() - CV[id[0]].X2()) < 1500)
            return true;
        else
            return false;
}


bool rightPath(vector<Object> CV, Point pos)
{
        double min1 = -10000;
        double min2 = 10000;
        Point widthP1, widthP2;
        double distance;
        for(int i = 0; i < CV.size(); i++)
        {
                // if on the left
                if(CV[i].X1() < 0 && CV[i].X1() > min1 && CV[i].Y1() <= pos.Y())
                {
                    min1 = CV[i].X1();
                    widthP1.set(CV[i].X1(), CV[i].Y1());
                }
                if(CV[i].X2() < 0 && CV[i].X2() > min1 && CV[i].Y2() <= pos.Y())
                {
                    min1 = CV[i].X2();
                    widthP1.set(CV[i].X2(), CV[i].Y2());
                }

                //if on the right
                if(CV[i].X1() > 0 && CV[i].X1() < min2 && CV[i].Y1() <= pos.Y())
                {
                    min2 = CV[i].X1();
                    widthP2.set(CV[i].X1(), CV[i].Y1());
                }
                if(CV[i].X2() > 0 && CV[i].X2() < min2 && CV[i].Y2() <= pos.Y())
                {
                    min2 = CV[i].X2();
                    widthP2.set(CV[i].X2(), CV[i].Y2());
                }   
        }

        distance = GetPointDistance(widthP1, widthP2);    

        if(distance < 600)
            return false;
        else
            return true;
}

//matching the left or right side, whether there is potential path
void matchThePath(ArRobot& robot, ArSick& sick, Point pos)
{
        Point temp(pos.X(), pos.Y());
        Point near;
        char viewFilename[100];
        unsigned char flagInter = 0;
        MyRobot myrobot(0,0);
        vector<Object> currentView;
        Object inters;
        Object Obj;
        double angle = 0;
        double dist = 0;
        double approach = 0;
        double offset = 400;
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
        
        
 lb2:if(temp.X() > 0)
            angle = -90;
        else
            if(temp.X() < 0)
                angle = 90;
            else
                angle = 0;
 
        temp = ExecutionAndReturn(robot, angle, dist, temp);
        
        currentView = scanAndSaveView(sick, numb);
        sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
        plotObjects(viewFilename, myrobot.getRobot(), currentView); 
        numb++;
        
        //parallel the corridor
        adjust = parallelTheSpace(currentView);
        temp = ExecutionAndReturn(robot, adjust, 0, temp); 
        
        //typeOfMove = pathSpaceDetect(currentView, path);
        
        inters = IntersectForReturn(currentView, Point (0, path));
        if(inters.getID() == 0) // correct path
        {
                cout<<"----This is a correct potential path----"<<endl;
                temp.set(0, path);
                if(path >= 4000)
                  near = avoidObstracle(currentView, Point (0, path / 2));
                else
                  near = avoidObstracle(currentView, temp);
                d1 = Perpendiculardistance(Point (0,0), temp, near);
                d2 = sqrt(near.X() * near.X() + near.Y() * near.Y());
                //if(currentView[currentView.size() - 1].length() < 2000 
                                                    //&& abs(currentView[currentView.size() - 1].X1() - currentView[currentView.size() - 1].X2()) > 100)
                if(d1 < 200 && d2 < 1500)//d1 to path, d2 to robot
                {
                        cout<<"----General situation!!!----"<<endl;
                        //cross the exit/door/narrow space to a corridor
                        //near = avoidObstracle(currentView, temp);

                        near.set(near.X() - offset, near.Y());
                        Obj = IntersectForReturn(currentView, near);
                        if(Obj.getID() != 0)//no intersected 
                                near.set(near.X() + offset * 2, near.Y());

                        infor = DistAndAngle(near);
                        temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                        temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 

                        //corrent its orientation in corridor 
                        currentView = scanAndSaveView(sick, numb);
                        sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
                        plotObjects(viewFilename, myrobot.getRobot(), currentView); 
                        numb++;

                        //parallel to the corridor
                        //angle = parallelTheSpace(currentView);
                        //temp = ExecutionAndReturn(robot, angle, 0, temp); 
                        //temp.set(0, temp.Y());
                }
                else
                  if(rightPath(currentView, temp) == false)//noise objects construct a small gap if smaller than a theshold, goto lb3!
                    goto lb3;
                
                //parallel to the corridor
                angle = parallelTheSpace(currentView);
                temp = ExecutionAndReturn(robot, angle, 0, temp); 
                temp.set(0, temp.Y());
                
                //half movement and adjust direction
                infor = DistAndAngle(temp);
                temp = ExecutionAndReturn(robot, infor.second, infor.first / 2, temp);
                
                //parallel the corridor
                angle = parallelTheSpace(currentView);
                temp = ExecutionAndReturn(robot, angle, 0, temp); 
                
                //temp.set(0, infor.first / 2);
                
                //moving forward to the place 
                //infor = DistAndAngle(temp);
                //temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                cout<<"This is to process the last part"<<endl<<endl;
                temp = ExecutionAndReturn(robot, 0, infor.first / 2, temp);
        }
        else
        {
          cout<<"----There is not correct potential path----"<<endl;
                //approach = sqrt((inters.X1() + inters.X2()) * (inters.X1() + inters.X2()) / 4 + (inters.Y1() + inters.Y2()) * (inters.Y1() + inters.Y2()) / 4);
                approach = crossedPoint(Point (inters.X1(), inters.Y1()), Point (inters.X2(), inters.Y2()), Point (0, path), Point (0,0));
                if(approach >= 2000)//distance is far
                {
                         cout<<"----Larger than 2 meters----"<<endl;
                         //set the destination
                         temp.set(0, path);

                         //move half distance to approach the obstacle
                         infor = DistAndAngle(Point ((inters.X1() + inters.X2()) / 2, (inters.Y1() + inters.Y2()) / 2));
                         temp = ExecutionAndReturn(robot, infor.second, infor.first / 2, temp); 
                         path = path - infor.first / 2;
                         flagInter = 1;
                         goto lb2;
                }
                else //distance is not far
                {
                        cout<<"-----Front obstacle is less than 2 meters-----"<<endl;
                        cout<<" the flag o Intersection is: "<<(int)flagInter<<endl;
                        if(flagInter == 0)
                        {
                                //not matched
                                //temp = ExecutionAndReturn(robot, -angle, temp.Y() / 2, temp);
lb3:                            temp = ExecutionAndReturn(robot, -angle, 600, temp);
                                cout<<"-----test programme-----"<<endl;
                                path = path - 600;
                                goto lb2;
                        }
                        else
                        {
                                //avoid the obstacle
                                near.set(inters.X1() - offset, inters.Y1());
                                Obj = IntersectForReturn(currentView, temp);
                                if(Obj.getID() != 0)//no intersected 
                                        near.set(inters.X2() + offset, inters.Y2());

                                //avoid the obstacle
                                infor = DistAndAngle(near);
                                temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                
                                //reach the destination
                                infor = DistAndAngle(temp);
                                temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                        } 
                }
        }
                
}

void matchPathWithSpace(ArRobot& robot, ArSick& sick, Point pos)
{
        Point temp(pos.X(), pos.Y());
        Point near;
        Point Interpoint;
        char viewFilename[100];
        unsigned char flagInter = 0;
        MyRobot myrobot(0,0);
        vector<Object> currentView;
        Object inters;
        Object Obj;
        double angle = 0;
        double dist = 0;
        double approach = 0;
        double offset1 = 700;
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
                                      near.set(Interpoint.X() - offset1, Interpoint.Y());
                                      infor = DistAndAngle(near);
                                      temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                      temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                      goto st2;
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
                                    near.set(Interpoint.X() + offset1, Interpoint.Y());
                                    infor = DistAndAngle(near);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                    goto st2;
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
                                    //reach the potential door, then carry on
                                    inters = intersectForObject(currentView, Point (-200, 0), Point (-200, temp.Y()));
                                    near.set(inters.X2() + offset2, inters.Y2());

                                    infor = DistAndAngle(near);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, temp.Y(), temp); 
                            }
                            else
                            {
                                if(interSectWithLine(currentView, Point (200, 0), Point (200, temp.Y())) == true)
                                {
                                        //reach the potential door, then carry on
                                        inters = intersectForObject(currentView, Point (200, 0), Point (200, temp.Y()));
                                        near.set(inters.X1() - offset2, inters.Y1());

                                        infor = DistAndAngle(near);
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
                        }
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

void CollisionAvoidandMove(ArRobot& robot, ArSick& sick, Point pos, int numOfstep)
{
        double angle, dist;
        double offset = 600;
        double approach = 0;
        int relat_flag = 0; // 1-left 2-right
        pair<double,double> infor;
        char flag = 0;
        
        char viewFilename[100];
        MyRobot myrobot(0,0);
        vector<Object> currentView;
        Object temp_inter;
        Point destination(pos.X(), pos.Y());
        Point temp;
        
        currentView = scanAndSaveView(sick, numOfstep);
        sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numOfstep, ".png");
        plotObjects(viewFilename, myrobot.getRobot(), currentView); 
        
        if(pos.Y() < 0)
        {
            destination = ExecutionAndReturn(robot, 180, 0, destination);  
            pos.set(destination.X(), destination.Y());
        }

        //if(numOfstep == 0)
        //{
        //       infor = DistAndAngle(pos);
        //       destination = ExecutionAndReturn(robot, infor.second, infor.first, destination); 
        //       destination = ExecutionAndReturn(robot, -infor.second, 0, destination); 
        //}
        //else
        do
        {
 lb1:        currentView = scanAndSaveView(sick, numb);
                sprintf(viewFilename, "%s%d%s", "Maps/Offline/Views-", numb, ".png");
                plotObjects(viewFilename, myrobot.getRobot(), currentView); 
                numb++;
                
                temp_inter = IntersectForReturn(currentView, Point (0, destination.Y()));

                if(temp_inter.getID() != 0) // intersected with a surface
                {
                        //approach = sqrt((temp_inter.X1() + temp_inter.X2()) * (temp_inter.X1() + temp_inter.X2()) / 4 + (temp_inter.Y1() + temp_inter.Y2()) * (temp_inter.Y1() + temp_inter.Y2()) / 4);
                        approach = crossedPoint(Point (temp_inter.X1(), temp_inter.Y1()), Point (temp_inter.X2(), temp_inter.Y2()), Point (0, destination.Y()), Point (0,0));
                        // approach the intersected surface
                        if(approach >= 2000)
                        {
                                dist = 500;
                                angle = 0;
                                //ExecutionAndGo(robot, angle, dist);
                                destination = ExecutionAndReturn(robot, angle, dist, destination);   
                                goto lb1;
                        }
                        else
                        {
                                if(pos.X() - temp_inter.X1() < 0)
                                {
                                    temp.set(temp_inter.X1() - offset, temp_inter.Y1());
                                }
                                else
                                    if(pos.X() - temp_inter.X2() > 0)
                                    {
                                        temp.set(temp_inter.X2() + offset, temp_inter.Y2());
                                    }
                                
                                infor = DistAndAngle(temp);
                                destination = ExecutionAndReturn(robot, infor.second, infor.first, destination); 
                                destination = ExecutionAndReturn(robot, -infor.second, 0, destination); 
                                goto lb1;
                        }
                }
                else // non-intersected with a surface
                {
                    cout<<"There is no any intersected!!!!"<<endl;
                        // if the distance is near
                        if(destination.Y() >= 2000)
                             destination = ExecutionAndReturn(robot, 0, destination.Y()/2, destination); 

                        //matchThePath(robot, sick, destination);
                        matchPathWithSpace(robot, sick, destination);
                        flag = 1;        
                }
                
        }while(flag != 1);

}


void reTrace(ArRobot& robot, ArSick& sick, Point pos, double heading)
{
    int reTurn_numb = 1000;     //view counter for homing
    int operation_mode = 0;     //operation mode 1 -- approaching the point, 2 -- adjust heading
    int myTurning = 0;
    Object obstacle_Obj;        //obstacle object
    vector<Object> currentView; //current view
    Point end_point, temp;      //object endpoint to guide robot
    double distance, dist, angle,myTurnAmount, destination_dist, deviation_heading; //displacement factors
    unsigned char accomplish_flag = 0; //if the robot reach the destination, flag = 1
    pair<double, double> displacement; //execution variables, dist & angle
    ArSectors myQuadrants;
    bool myUseTableIRIfAvail = true;
    
    temp = pos;
    myTurnAmount = distance = 0;
    
    while(accomplish_flag != 1) //accomplishment flag
    {
            //calculating the destination and heading after moving one step
            temp = whereDestination(temp, myTurning, distance);
            destination_dist = sqrt(temp.X() * temp.X() + temp.Y() * temp.Y());
            deviation_heading += myTurning;
                    
            if(destination_dist < 1e-6 && deviation_heading < 1e-6)
            {
                accomplish_flag = 1; //displacement completely done
            }
            else
            {
                if(destination_dist < 1e-6)
                    operation_mode = 1; //approaching the position
                else
                    if(deviation_heading < 1e-6)
                         operation_mode = 2; //adjust heading only
            }
            
            switch(operation_mode)
            {
                case 1: //facing to the destination
                        displacement = DistAndAngle(temp);
                        robot.setDeltaHeading(displacement.second);
                         
                        //check whether the front is accessible
                        distance = (robot.checkRangeDevicesCurrentPolar(-50, 50, &angle) 
                                    - robot.getRobotRadius());
                        //avoid obstacle and move
                        if (dist > 500 && 
                            (!myUseTableIRIfAvail || 
                            (myUseTableIRIfAvail && !robot.hasTableSensingIR()) || 
                            (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
                             !robot.isLeftTableSensingIRTriggered() &&
                             !robot.isRightTableSensingIRTriggered())))
                        {
                            if (myTurning != 0)
                            {
                                 robot.setDeltaHeading(0);
                                 myTurning = 0;
                            }
                            else
                                 myTurning = 0;
                            
                            //straight forward to the destination 
                            robot.setMoveDoneDist(displacement.first);
                        }

                        if (myTurning == 0)
                        {
                            if (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
                                 robot.isLeftTableSensingIRTriggered())
                               myTurning = 1;
                            else if (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
                                      robot.isRightTableSensingIRTriggered())
                               myTurning = -1;
                            else if (angle < 0)
                               myTurning = 1;
                            else
                               myTurning = -1;
                            myTurnAmount = 20;
                            myQuadrants.clear();
                        }

                        myQuadrants.update(robot.getTh());
                        if (myTurning && myQuadrants.didAll())
                        {
                            myQuadrants.clear();
                            myTurnAmount /= 2;
                            if(myTurnAmount == 0)
                               myTurnAmount = 20;
                        }
                        robot.setDeltaHeading(myTurning * myTurnAmount);
                        
                        if (dist > 500/2 && 
                            (!myUseTableIRIfAvail || 
                             (myUseTableIRIfAvail && !robot.hasTableSensingIR()) || 
                             (myUseTableIRIfAvail && robot.hasTableSensingIR() && 
                             !robot.isLeftTableSensingIRTriggered() &&
                             !robot.isRightTableSensingIRTriggered())))
                        {
                             //robot.setVel(myAvoidVel * dist / 500);
                            robot.setMoveDoneDist(dist/2);
                        }
                        else
                        {
                             robot.setVel(0);
                        }
                        break;   
                         
                case 2: //adjust the heading finally
                        robot.setDeltaHeading(deviation_heading);
                        break;
                         
                default:break;
            }
    }
}


void matchPathForwardWithSpace(ArRobot& robot, ArSick& sick, Point pos)
{
        Point temp(pos.X(), pos.Y());
        Point near;
        Point Interpoint;
        char viewFilename[100];
        unsigned char flagInter = 0;
        MyRobot myrobot(0,0);
        vector<Object> currentView;
        Object inters;
        Object Obj;
        double angle = 0;
        double dist = 0;
        double approach = 0;
        double offset1 = 700;
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
        
        
st:     infor = DistAndAngle(temp); //facing to the destination
        angle = infor.second;
                    
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
                                      near.set(Interpoint.X() - offset1, Interpoint.Y());
                                      infor = DistAndAngle(near);
                                      temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                      temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                      goto st2;
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
                                    near.set(Interpoint.X() + offset1, Interpoint.Y());
                                    infor = DistAndAngle(near);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, 0, temp); 
                                    goto st2;
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
                                    //reach the potential door, then carry on
                                    inters = intersectForObject(currentView, Point (-200, 0), Point (-200, temp.Y()));
                                    near.set(inters.X2() + offset2, inters.Y2());

                                    infor = DistAndAngle(near);
                                    temp = ExecutionAndReturn(robot, infor.second, infor.first, temp); 
                                    temp = ExecutionAndReturn(robot, -infor.second, temp.Y(), temp); 
                            }
                            else
                            {
                                if(interSectWithLine(currentView, Point (200, 0), Point (200, temp.Y())) == true)
                                {
                                        //reach the potential door, then carry on
                                        inters = intersectForObject(currentView, Point (200, 0), Point (200, temp.Y()));
                                        near.set(inters.X1() - offset2, inters.Y1());

                                        infor = DistAndAngle(near);
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
    
                        }
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






