

/* 
 * File:   ReturnHome.cpp
 * Author: Lyn Pang
 *
 * Created on December 14, 2016, 1:48 PM
 */


#include <iostream>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <fstream>

#include "readAndwriteASCII.H"
#include "Plotting.H"
#include "GeometricOp.H"

#include "Point.H"
#include "Object.H"
#include "asr.H"
#include "mfisOp.H"
#include "asrOp.H"
#include "PathPlanning.H"
#include "Transporter.H"

#include "Minfo.H"
#include "Map.H"
#include "CompareASR.H"
#include "Mapping.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"
#include "PolygonChoice.h"
#include "PerceptualMapping.H"
#include "GeometryFuncs.H"
#include "GeometricOp.H"

#include "Aria.h"
#include "Laser2Surface.H"
#include "RobotFuncs.H"
#include "Memory.h"
#include "ToolFunctions.h"
#include "Returning.h"
//#include "ArAction.h"

#define PI 3.14159265


using namespace std;
//void ReadMemory(int level, int set, int w);
void HomewardWithShortcut(ArRobot& robot, ArSick& sick);
vector<Object> MatchPath(vector<Object> chunk1, vector<Object> chunk2, 
                          vector<Point> rbs1, vector<Point> rbs2,
                            Point key_point1, Point key_point2);
void executionInstruction(vector< vector<Object> > chunks, 
                                vector< vector<Object> > paths,
                                ArRobot& robot, ArSick& sick);
void ExecutionInstAndMove(vector< pair<int, double> > instructions, ArRobot& robot, ArSick& sick);
void frontObstacle(vector< pair<int, double> > instructions, ArRobot& robot, ArSick& sick);
void MoveAndTurn(ArRobot& robot, double angle, double dist);
pair<int, Point> isExit(ArRobot& robot, ArSick& sick, int numb, int flag);



ArRobot robot;
ArSick sick;


using namespace std;


int main(int argc, char** argv) 
{
        char flag;
        int v, w, level, set;
        srand(getpid());

        cout << "argc: " << argc << " argv: " << argv << endl;

        Aria::init();
        ArSimpleConnector connector(&argc, argv);

        if (!connector.parseArgs() || argc > 1) 
        {
            Aria::logOptions();
            Aria::shutdown();
            Aria::exit(1);
        }

        robot.addRangeDevice(&sick);
        // Try to connect, if we fail exit
        if (!connector.connectRobot(&robot)) 
        {
            cout << "Could not connect to robot... exiting" << endl;
            Aria::shutdown();
            return 1;
        }

        // Turn on the motors, turn off amigobot sounds
        robot.runAsync(true);
        robot.comInt(ArCommands::ENABLE, 1);
        robot.comInt(ArCommands::SOUNDTOG, 0);
        robot.lock();
        robot.clearDirectMotion();
        robot.unlock();
        // Set up the laserPoint postion
        connector.setupLaser(&sick);
        sick.runAsync();
        if (!sick.blockingConnect()) 
        {
                cout << "Could not connect to SICK laser... exiting" << endl;
                robot.disconnect();
                Aria::shutdown();
                return 1;
        }
        ArUtil::sleep(1500);
        cout << "----------------Connected to robot and laser-------------------" << endl;

        //main function
        HomewardWithShortcut(robot, sick);
        
    return 0;
}



/***********************************************************
 *function : return home using shortcut in chunks
 *author : Lyn Pang
 *Principle : returning key exit place first
 *then localise its position and orientation using matching
 * designed for dataset 5 (floor 5 student lounge)
 ***********************************************************/
void HomewardWithShortcut(ArRobot& robot, ArSick& sick)
{
    cout << " ----- This process is to read memory & plan path ----- " << endl << endl;
    //read memory
    char chunkFileName[80];
    char chunk_robots[80];
    char exitFileName[80];
    char plotFileName[80];
    
    Object temp_path;
    vector<Object> chunk1, chunk2, chunk3;
    vector<Object> path1, path2, path3;
    vector<Object> ult_path1, ult_path3, planned_path;
    vector<Point> chunk_robots1, chunk_robots2, chunk_robots3;
    vector<Point> key_exits;
    
    unsigned char eliminate_flag = 0;
    
    
    //chunks reading from files
    sprintf(chunkFileName, "%s", "inputData/Chunks/Chunk-1");
    chunk1 = readASCII(chunkFileName);
    sprintf(chunkFileName, "%s", "inputData/Chunks/Chunk-2");
    chunk2 = readASCII(chunkFileName);
    sprintf(chunkFileName, "%s", "inputData/Chunks/Chunk-3");
    chunk3 = readASCII(chunkFileName);
    
    /*
    //plotting memory chunks
    sprintf(plotFileName, "%s", "Maps/Offline/Memory_Chunk1.png");
    plotObjects(plotFileName, chunk1);
    sprintf(plotFileName, "%s", "Maps/Offline/Memory_Chunk2.png");
    plotObjects(plotFileName, chunk2);
    sprintf(plotFileName, "%s", "Maps/Offline/Memory_Chunk3.png");
    plotObjects(plotFileName, chunk3);
    */
    
    //robot positions reading from files
    sprintf(chunk_robots, "%s", "inputData/Chunks/Chunkrobots-1");
    chunk_robots1 = readPoints(chunk_robots);
    sprintf(chunk_robots, "%s", "inputData/Chunks/Chunkrobots-2");
    chunk_robots2 = readPoints(chunk_robots);
    sprintf(chunk_robots, "%s", "inputData/Chunks/Chunkrobots-3");
    chunk_robots3 = readPoints(chunk_robots);
    
    //establish path for chunks
    for(int i = 0; i < chunk_robots1.size()-1; i++)
    {
        temp_path.set(chunk_robots1[i].X(), chunk_robots1[i].Y(),
                        chunk_robots1[i+1].X(), chunk_robots1[i+1].Y(), i);
        path1.push_back(temp_path);     
    }
    
    for(int i = 0; i < chunk_robots2.size()-1; i++)
    {
        temp_path.set(chunk_robots2[i].X(), chunk_robots2[i].Y(),
                        chunk_robots2[i+1].X(), chunk_robots2[i+1].Y(), i);
        path2.push_back(temp_path);     
    }
    
    for(int i = 0; i < chunk_robots3.size()-1; i++)
    {
        temp_path.set(chunk_robots3[i].X(), chunk_robots3[i].Y(),
                        chunk_robots3[i+1].X(), chunk_robots3[i+1].Y(), i);
        path3.push_back(temp_path);     
    }
        
            
    //condition of eliminate intermediate information
    for(int i = 0; i < path3.size(); i++)
    {
        if(interSectWithLine(path1, Point (path3[i].X1(), path3[i].Y1()), Point (path3[i].X2(), path3[i].Y2())) == true)
        {
            //elimination flag set by 1
            eliminate_flag = 1;
            //cout << " Here the intersection between chunk paths " << endl;
        }
    }
    
    if(eliminate_flag == 1)
    {
        //plan paths between chunks left only
        sprintf(exitFileName, "%s", "inputData/Chunks/KeyExits");
        key_exits = readPoints(exitFileName);
        
        //plan a path on last chunk 
        planned_path =  MatchPath(chunk1, chunk3, 
                                    chunk_robots1, chunk_robots3,
                                    key_exits[0], key_exits[1]);
        
        //ulti-paths for homing
        ult_path1 = returnPATH(chunk_robots1, key_exits[0], 1); //next chunk
        ult_path3 = returnPATH(chunk_robots3, key_exits[1], 2); //last chunk
        
        ult_path3 = addTwoVectorsOfObjects(ult_path3, planned_path); //combine planned path with last chunk path
        
    }

    vector< vector<Object> > paths;
    vector< vector<Object> > chunks;
    
    chunks.push_back(chunk3);
    chunks.push_back(chunk1);
    
    paths.push_back(ult_path3);
    paths.push_back(ult_path1);
    
    //instructions from paths in different chunk
    executionInstruction(chunks, paths, robot, sick);
    
    //execution following instructions
    
    
}

vector<Object> MatchPath(vector<Object> chunk1, vector<Object> chunk2, 
                          vector<Point> rbs1, vector<Point> rbs2,
                            Point key_point1, Point key_point2)
{
    char plotFileName[80];
    
    double extd_x, extd_y; //extend point x & y
    double exd_length = 2000;
    
    pair<double, double> ref_info; //reference information,1 is dist 2 is angle
    Point extend_point;
    
    Object ref_object; //reference object for localization
    Object temp_path; 
    Object extend_path; //extend key path for identifying environment
    vector<Object> potential_path; //all potential paths for finding home
    vector<Object> ult_path1, ult_path2; //
    
    ult_path1 = returnPATH(rbs1, key_point1, 1);
    ult_path2 = returnPATH(rbs2, key_point2, 2);
    
    sprintf(plotFileName, "%s", "Maps/Offline/ult_path1.png");
    plotObjects(plotFileName, ult_path1);
    sprintf(plotFileName, "%s", "Maps/Offline/ult_path2.png");
    plotObjects(plotFileName, ult_path2);
     
    //identify the particular path length of path 
    double cirR = ult_path1[0].length();
    
    
    //extend the path line
    extd_x = ult_path1[0].X2() + (ult_path1[0].X2() - ult_path1[0].X1()) / ult_path1[0].length() * exd_length;
    extd_y = ult_path1[0].Y2() + (ult_path1[0].Y2() - ult_path1[0].Y1()) / ult_path1[0].length() * exd_length;
    extend_path.set(extd_x, extd_y, ult_path1[0].X2(), ult_path1[0].Y2(), 1);
    
    ref_object = intersectForObject(chunk1, Point (extend_path.X1(), extend_path.Y1()), 
                                Point (extend_path.X2(), extend_path.Y2()));
    if(ref_object.getID() != 0)
    {
        //relation information of path
        //distance and angle relation
        ref_info.first = perpendicularDis(Point (ref_object.X1(), ref_object.Y1()), 
                                            Point (ref_object.X2(), ref_object.Y2())
                                          , Point (extend_path.X1(), extend_path.Y1()));
        
        ref_info.second = abs(extend_path.getAngleWithLine(ref_object));
    }
    else
    {
        ref_info.first = ref_info.second = 0;
    }
    
    //cout << " the reference informaiton first : " << ref_info.first 
    //     << " ; second : " << ref_info.second << endl;   
    
    //compute all potential path in follow chunk      
    for(double thete = 0;thete <= PI; thete+=0.3) //PI/180
    { 
           double x = cirR * cos(thete) + key_point2.X();
           double y = cirR * sin(thete) + key_point2.Y();   //x,y
           
           temp_path.set(key_point2.X(), key_point2.Y(), x, y, 1);
           
           //restrict path using relation information
           double ed_x, ed_y;
           Object temp_extend;
           
           ed_x = temp_path.X2() + (temp_path.X2() - temp_path.X1()) / temp_path.length() * exd_length;
           ed_y = temp_path.Y2() + (temp_path.Y2() - temp_path.Y1()) / temp_path.length() * exd_length;
           temp_extend.set(ed_x, ed_y, temp_path.X2(), temp_path.Y2(), 1);
   
           Object potential_ref_obj = intersectForObject(chunk2, Point (temp_extend.X1(), temp_extend.Y1()), 
                                Point (temp_extend.X2(), temp_extend.Y2()));
           
           if(potential_ref_obj.getID() != 0)
           {
                cout << " There is intersection object ! " << endl;
                double dist_to_pRef = perpendicularDis(Point (potential_ref_obj.X1(), potential_ref_obj.Y1()), 
                                            Point (potential_ref_obj.X2(), potential_ref_obj.Y2())
                                          , Point (temp_extend.X1(), temp_extend.Y1()));
        
                double angle_to_pRef = abs(temp_extend.getAngleWithLine(potential_ref_obj));
               
                if(abs(ref_info.first - dist_to_pRef) <  500
                        && abs(ref_info.second - angle_to_pRef) < 15)
                {
                     potential_path.push_back(temp_path);
                }
           }
    }

    cout << " the size of potential paths : " << potential_path.size() << endl;
    sprintf(plotFileName, "%s", "Maps/Offline/Potential_path.png");
    plotObjects(plotFileName, potential_path, chunk2);
  
    return potential_path;
}

/*Follow path as instruction 
 *and also the real environment
 *finding exits on left/right side
 */
void executionInstruction(vector< vector<Object> > chunks, 
                                vector< vector<Object> > paths,
                                ArRobot& robot, ArSick& sick)
{
    int cnt = 0;
    double include_angle; //included angle between two adjacent path segment
    double move_dist;
    
    //exit position 1left 2right 3turn around 4find exit left and turn around 5find exit right and turn around
    int turn_flag = 0; 
    
    pair<int, double> temp_inst;
    vector< pair<int, double> > instructions;
    
    while(cnt < paths.size())
    {
        //left or right and check included angle
        //included angle < 20, robot turn around
        
        for(int i = 0; i < paths[cnt].size(); i++)
        {
            //collect instructions
            move_dist = paths[cnt][i].length();
            
            if(i != paths[cnt].size()-1)
            {
                //exit position
                if(isLeft(Point (paths[cnt][i].X1(), paths[cnt][i].Y1()), 
                          Point (paths[cnt][i].X2(), paths[cnt][i].Y2()), 
                          Point (paths[cnt][i+1].X2(), paths[cnt][i+1].Y2())) > 0)
                    turn_flag = 1;
                if(isLeft(Point (paths[cnt][i].X1(), paths[cnt][i].Y1()), 
                          Point (paths[cnt][i].X2(), paths[cnt][i].Y2()), 
                          Point (paths[cnt][i+1].X2(), paths[cnt][i+1].Y2())) < 0)
                    turn_flag = 2;

                //include angle. move dist. left or right exit
                include_angle = paths[cnt][i].getAngleWithLine(paths[cnt][i+1]);          
                if(include_angle < 20)
                    turn_flag = 3;

                temp_inst.first = turn_flag;
                temp_inst.second = move_dist;
                cout << " the current step turn flag : " << turn_flag 
                     << " ; the distance is : " << move_dist << endl << endl;;
            }
            else
            {
                turn_flag = 0;
                temp_inst.first = turn_flag;
                temp_inst.second = move_dist;
                cout << " the current step turn flag : " << turn_flag 
                     << " ; the distance is : " << move_dist << endl << endl;;
            }
            instructions.push_back(temp_inst);
            
        }
        
        cnt++;
    }
    
    //manage instructions first
    for(int i = 0; i < instructions.size()-1; i++)
    {
        if(instructions[i].second == instructions[i+1].second)
        {
            if(instructions[i].first == 0)
            {
                instructions[i].first = instructions[i+1].first;
                instructions.erase(instructions.begin()+i+1);
            }
            if(instructions[i+1].first == 0)
            {
                instructions[i].first;
                instructions.erase(instructions.begin()+i);
            }
        }
        
        //finding exit on left hand side and turn around
        if(instructions[i].first == 1 && instructions[i+1].first == 3)
        {
            instructions[i].first = 4;
            instructions.erase(instructions.begin()+i+1);
        }
        
        //finding exit on right hand side and turn around
        if(instructions[i].first == 2 && instructions[i+1].first == 3)
        {
            instructions[i].first = 5;
            instructions.erase(instructions.begin()+i+1);
        }
    }
    
    cout << " The size of instructions : " << instructions.size() << endl;
    for(int i = 0; i < instructions.size(); i++)
    {cout << " the current step turn flag : " << instructions[i].first 
                     << " ; the distance is : " << instructions[i].second << endl<< endl;}
    ExecutionInstAndMove(instructions, robot, sick);

}


void ExecutionInstAndMove(vector< pair<int, double> > instructions, ArRobot& robot, ArSick& sick)
{
    int cnt = 1; //counter for instructions
    int numb = 1;//number for collected views
    MyRobot myrobot(0,0); //robot start point. 
    char viewFileName[80];
    
    Point inter_point; //intersected point
    Point left_p, right_p;
    double left_d, right_d;
    double parellel_ang = 0;
    pair<int, Point> exitAndPosition;
    pair<double, double> infor;
    vector<Object> currentView; //current view
    
    //for testing 
    instructions[1].second = 1000;
    //instructions[0].second = 2000;
    
    while(cnt < instructions.size())
    {
        cout << " Current instruction flag is : " << instructions[cnt].first << endl;
        //collect view 
lb1:    currentView = scanAndSaveView(sick, numb);
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
        plotObjects(viewFileName, myrobot.getRobot(), currentView); 

        //judge whether is accessable
        if(interSectWithLine(currentView, Point (-200, 0), Point (-200, instructions[cnt].second)) == true)
        {
            cout << " The robot left is intersected !!!" << endl << endl;
            if(interSectWithLine(currentView, Point (200, 0), Point (200, instructions[cnt].second)) == false)
            {
                  cout << " The robot left is intersected BUT right is NOT intersected!!!" << endl << endl;
//lb4:              if(instructions[cnt].first != 1) //the priority should be right hand side
                    //if((instructions[cnt].first != 1) && (instructions[cnt+1].first != 0))
                    MoveAndTurn(robot, -15, 0);

            }   
            else
            {
                  cout << " The robot BOTH left And right are intersected !!!" << endl << endl;
                            left_p = intersectedPoint(currentView, Point (-200, 0), Point (-200, instructions[cnt].second));
                            right_p = intersectedPoint(currentView, Point (200, 0), Point (200, instructions[cnt].second));
                            
                            left_d = distanceOftwoP(left_p, Point (-200, 0));
                            right_d = distanceOftwoP(right_p, Point (200, 0));

                            
                            if((left_d > right_d) && (abs(left_d - right_d) > 1000))
                            {
                                MoveAndTurn(robot, 10, 800);
                                MoveAndTurn(robot, -10, 0); 
                                currentView = scanAndSaveView(sick, numb);
                                sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
                                plotObjects(viewFileName, myrobot.getRobot(), currentView);       
                                parellel_ang = parallelTheSpace(currentView);  
                                MoveAndTurn(robot, parellel_ang, 0);
                                goto lb2;
                            }
                            else
                            {
                                if((left_d < right_d) && (abs(left_d - right_d) > 1000))
                                {
                                  MoveAndTurn(robot, -10, 800);
                                  MoveAndTurn(robot, 10, 0);
                                  currentView = scanAndSaveView(sick, numb);
                                sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
                                plotObjects(viewFileName, myrobot.getRobot(), currentView);       
                                parellel_ang = parallelTheSpace(currentView);  
                                MoveAndTurn(robot, parellel_ang, 0);
                                  goto lb2; 
                                }
                                else 
                                  goto lb2;
                            }
            }
            goto lb1;
        }
        else
        {
            if(interSectWithLine(currentView, Point (200, 0), Point (200, instructions[cnt].second)) == true)
            {
                    cout << " The robot right is intersected !!!" << endl << endl;
                    if(interSectWithLine(currentView, Point (-200, 0), Point (-200, instructions[cnt].second)) == false)
                    {
                        cout << " The robot right is intersected BUT the left is NOT intersected!!!" << endl << endl;
//lb3:                    if(instructions[cnt].first != 2) //the priority should be left hand side
                       MoveAndTurn(robot, 15, 0);
//                        else
//                         goto lb4;
                    }
                    else
                    {
                        cout << " The robot BOTH left And right are intersected !!!" << endl << endl;
                            left_p = intersectedPoint(currentView, Point (-200, 0), Point (-200, instructions[cnt].second));
                            right_p = intersectedPoint(currentView, Point (200, 0), Point (200, instructions[cnt].second));
                            
                            left_d = distanceOftwoP(left_p, Point (-200, 0));
                            right_d = distanceOftwoP(right_p, Point (200, 0));

                            if((left_d > right_d) && (abs(left_d - right_d) > 1000))
                            {
                                pair<double, double> obst;
                                obst = DistAndAngle(Point (right_p.X()-400, right_p.Y()));
                                MoveAndTurn(robot, obst.second, obst.first);  
                                MoveAndTurn(robot, -obst.second, 0);
                                
                                currentView = scanAndSaveView(sick, numb);
                                sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
                                plotObjects(viewFileName, myrobot.getRobot(), currentView);       
                                parellel_ang = parallelTheSpace(currentView);  
                                MoveAndTurn(robot, parellel_ang, 0);
                                
                                goto lb2;
                            } 
                            else
                            {
                                if((left_d < right_d) && (abs(left_d - right_d) > 1000))
                                {
                                  pair<double, double> obst;
                                  obst = DistAndAngle(Point (right_p.X()+400, right_p.Y()));
                                  MoveAndTurn(robot, obst.second, obst.first);  
                                  MoveAndTurn(robot, -obst.second, 0);
                                 // MoveAndTurn(robot, -10, 800); 
                                 // MoveAndTurn(robot, 10, 0);
                                  currentView = scanAndSaveView(sick, numb);
                                  sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
                                  plotObjects(viewFileName, myrobot.getRobot(), currentView);       
                                  parellel_ang = parallelTheSpace(currentView);  
                                  MoveAndTurn(robot, parellel_ang, 0);
                                  goto lb2;
                                }
                                else 
                                  goto lb2;
                            } 
                    }
                    goto lb1;
            }
            else
            {
lb2:            if(interSectWithLine(currentView, Point (0, 0), Point (0, instructions[cnt].second)) == false)
                {        
                    cout << " The robot front is NOT intersected !!!" << endl << endl;
                    if(instructions[cnt].second >= 7000)
                    {
                         MoveAndTurn(robot, 0, instructions[cnt].second/2);
                         MoveAndTurn(robot, -10, 0);
                         MoveAndTurn(robot, 0, instructions[cnt].second/2);
                         MoveAndTurn(robot, -6, 0);
                    }
                    else
                    {
                        MoveAndTurn(robot, 0, instructions[cnt].second);
                        if(instructions[cnt].second >= 4000)
                        MoveAndTurn(robot, -10, 0);
                    }
                }
                else
                {
                    cout << " The robot front is intersected !!!" << endl << endl;
    
                    inter_point = intersectedPoint(currentView, Point (0, 0), Point (0, instructions[cnt].second));
   /*                
                    if(instructions[cnt].second >= 7000)
                    {
                         MoveAndTurn(robot, 0, (inter_point.Y() - 800)/2);
                         MoveAndTurn(robot, -10, 0);
                         MoveAndTurn(robot, 0, (inter_point.Y() - 800)/2);
                         MoveAndTurn(robot, -6, 0);
                    }
                    else
                    {
                        MoveAndTurn(robot, 0, instructions[cnt].second);
                        MoveAndTurn(robot, -10, 0);
                    }
    */               
    
                    if(abs(inter_point.Y()) < 3000)
                    {
                          cout << " The robot front intersected point is less than 3000 !!!" << endl << endl;
                            MoveAndTurn(robot, 15, 0);//search on left hand side
                            currentView = scanAndSaveView(sick, numb);
                            sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
                            plotObjects(viewFileName, myrobot.getRobot(), currentView);
                            if(interSectWithLine(currentView, Point (0, 0), Point (0, instructions[cnt].second/2)) == false)
                            {
                                goto lb2;
                            }
                            else
                            {
                                MoveAndTurn(robot, -(15*2), 0);//search on right hand side
                                currentView = scanAndSaveView(sick, numb);
                                sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
                                plotObjects(viewFileName, myrobot.getRobot(), currentView);
                                
                                if((interSectWithLine(currentView, Point (0, 0), Point (0, instructions[cnt].second+500)) == false)
                                  || ((interSectWithLine(currentView, Point (0, 0), Point (0, instructions[cnt].second)) == false) 
                                      && (instructions[cnt].second>7000)))
                                    goto lb2;
                            }
                    }
                    else
                    {
                            inter_point = intersectedPoint(currentView, Point (0, 0), Point (0, instructions[cnt].second));
                            if(instructions[cnt].second >= 7000)
                            {
                                 MoveAndTurn(robot, 0, (inter_point.Y() - 800)/2);
                                 MoveAndTurn(robot, -10, 0);
                                 MoveAndTurn(robot, 0, (inter_point.Y() - 800)/2);
                                 MoveAndTurn(robot, -6, 0);
                            }
                            else
                            {
                                MoveAndTurn(robot, 0, instructions[cnt].second);
                                if(instructions[cnt].second >= 4000)
                                MoveAndTurn(robot, -6, 0);
                            }
                    }
    
                }
            }
        }
        
        
        //search exit following instruction
        switch(instructions[cnt].first)
        {
            case 1: //check left side and cross the exit
                    exitAndPosition = isExit(robot, sick, numb, instructions[cnt].first);
                    
                    if(abs(exitAndPosition.first) == 1)
                    {
                        infor = DistAndAngle(exitAndPosition.second);
                        MoveAndTurn(robot, infor.second, infor.first); 
                        MoveAndTurn(robot, -infor.second, 0);
                    }
                    break;
            case 2: //check right side and cross the exit
                    exitAndPosition = isExit(robot, sick, numb, instructions[cnt].first);
                    if(abs(exitAndPosition.first) == 1)
                    {
                        infor = DistAndAngle(exitAndPosition.second);
                        MoveAndTurn(robot, infor.second, infor.first); 
                        MoveAndTurn(robot, -infor.second, 0);
                    }
                    break;
            case 3: //turn around directly
                         MoveAndTurn(robot, 180, 0);
                    break;
            case 4: //check left side and turn around
                    exitAndPosition = isExit(robot, sick, numb, instructions[cnt].first);
                    if(exitAndPosition.first > 0)
                        MoveAndTurn(robot, 180, 0);
                        else 
                            if(exitAndPosition.first < 0)
                                MoveAndTurn(robot, -90, 0);
                    break;
            case 5: //check right side and turn around
                    exitAndPosition = isExit(robot, sick, numb, instructions[cnt].first);
                    if(exitAndPosition.first > 0)
                        MoveAndTurn(robot, 180, 0);
                        else
                            if(exitAndPosition.first < 0)
                                MoveAndTurn(robot, 90, 0);
                    break;
            default://the last step normally, flag is 0
                    frontObstacle(instructions, robot, sick);
                    break;
        }
        
       cnt++; 
    }
}

pair<int, Point> isExit(ArRobot& robot, ArSick& sick, int numb, int flag)
{
    pair<int, Point> rnt; //first for flag, second for exit point
    Point near;
    
    char viewFileName[80];
    
    Object inters;
    vector<Object> currentView; //current view
    MyRobot myrobot(0,0);
    int turnCounter = 1;  //turn 2 times at most
    int fowardMove = 600; //if cannot find exit, turn back and move forward dist
    
    double door_offset = 600; //the offset for crossing a door
    double exit_threshold = 3000;
    double adjustAngle;
    
lb: if(flag == 1 || flag == 4)
        adjustAngle = 45;
    if(flag == 2 || flag == 5)
        adjustAngle = -45;
    
    MoveAndTurn(robot, adjustAngle, 0);
    currentView = scanAndSaveView(sick, numb);
    sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
    plotObjects(viewFileName, myrobot.getRobot(), currentView); 
     
    if((interSectWithLine(currentView, Point (200, 0), Point (200, exit_threshold)) == false)
        || (interSectWithLine(currentView, Point (-200, 0), Point (-200, exit_threshold)) == false))
    {
        //there is an exit for crossing 
        if(interSectWithLine(currentView, Point (200, 0), Point (200, exit_threshold)) == true)
        {
            cout << " right hand side intersected when crossing door !!" << endl;
            inters = intersectForObject(currentView, Point (200, 0), Point (200, exit_threshold));
            near.set(inters.X1() - door_offset, inters.Y1());
        }
        else
        {
            if(interSectWithLine(currentView, Point (-200, 0), Point (-200, exit_threshold)) == true)
            {
                cout << " left hand side intersected when crossing door !!" << endl;
                inters = intersectForObject(currentView, Point (-200, 0), Point (-200, exit_threshold));
                near.set(inters.X2() + door_offset, inters.Y2());
            }
            else
            {
                cout << " NEITHER are intersected when crossing door !!" << endl;
                near.set(0, 2000); //cross the door directly
            }
        }
        
    
        if(turnCounter != 2)
        {
           rnt.first = -1;
           if(flag == 4 || flag ==5)
           MoveAndTurn(robot, -(adjustAngle*turnCounter), 0);
        }
        else
           rnt.first = 1;
        rnt.second = near;
    }
    else
    {
            //turn back & move forward 
            if(turnCounter != 2)
            {
                turnCounter++;
                goto lb;
            }
            else
            {
                MoveAndTurn(robot, -(adjustAngle*turnCounter), 0);
                currentView = scanAndSaveView(sick, numb);
                sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
                plotObjects(viewFileName, myrobot.getRobot(), currentView); 
                if(interSectWithLine(currentView, Point (0, 0), Point (0, fowardMove+500)) == false)
                    MoveAndTurn(robot, 0, fowardMove);
                else
                {
                    MoveAndTurn(robot, -(adjustAngle/45*40), 0);
                    MoveAndTurn(robot, 0, fowardMove);
                }
                    
                turnCounter = 1;
                goto lb;
            }
    }
    

    
    return rnt;
}

void frontObstacle(vector< pair<int, double> > instructions, ArRobot& robot, ArSick& sick)
{
    int pre_flag_instruction;
    int numb = 10;//number for collected views
    double distance, angle;

    Point near;
    pair<double, double> disAndAng;
    MyRobot myrobot(0,0); //robot start point. 
    char viewFileName[80];
    
    vector<Object> currentView;
    
    //check previous step from instructions
    pre_flag_instruction = instructions[instructions.size()-2].first; //the last second flag
    
lb: currentView = scanAndSaveView(sick, numb);
    sprintf(viewFileName, "%s%d%s", "Maps/Offline/returnViews-", numb++, ".png");
    plotObjects(viewFileName, myrobot.getRobot(), currentView); 
    
     /*  
    if(((interSectWithLine(currentView, Point (-200, 0), Point (-200, instructions[instructions.size()-1].second)) == true)
      && (interSectWithLine(currentView, Point (0, 0), Point (0, instructions[instructions.size()-1].second)) == true))
      || ((interSectWithLine(currentView, Point (200, 0), Point (200, instructions[instructions.size()-1].second)) == true)
      && (interSectWithLine(currentView, Point (0, 0), Point (0, instructions[instructions.size()-1].second)) == true)))
      */
      if((interSectWithLine(currentView, Point (-200, 0), Point (-200, instructions[instructions.size()-1].second)) == true)
      && (interSectWithLine(currentView, Point (0, 0), Point (0, instructions[instructions.size()-1].second)) == true)
      && (interSectWithLine(currentView, Point (200, 0), Point (200, instructions[instructions.size()-1].second)) == true))
    {
                       //left_p = intersectedPoint(currentView, Point (-200, 0), Point (-200, instructions[cnt].second));
                       //right_p = intersectedPoint(currentView, Point (200, 0), Point (200, instructions[cnt].second));
                            
                       //left_d = distanceOftwoP(left_p, Point (-200, 0));
                       //right_d = distanceOftwoP(right_p, Point (200, 0));

                       if(pre_flag_instruction == 1)
                                MoveAndTurn(robot, 20, 0);   
                       else
                       {
                                if(pre_flag_instruction == 2)
                                  MoveAndTurn(robot, -20, 0); 
                       }
                       goto lb;
    }
    else
    {
    /*
        if(interSectWithLine(currentView, Point (-200, 0), Point (-200, instructions[instructions.size()-1].second)) == true)
        {
            near = intersectedPoint(currentView, Point (-200, 0), Point (-200, instructions[instructions.size()-1].second));
            disAndAng = DistAndAngle(Point (near.X()+400, near.Y()));
        }
        
        if(interSectWithLine(currentView, Point (200, 0), Point (200, instructions[instructions.size()-1].second)) == true)
        {
            near = intersectedPoint(currentView, Point (-200, 0), Point (-200, instructions[instructions.size()-1].second));
            disAndAng = DistAndAngle(Point (near.X()-400, near.Y()));
        }
        
        MoveAndTurn(robot, disAndAng.second, instructions[instructions.size()-1].second); 
        */
        MoveAndTurn(robot, 0, instructions[instructions.size()-1].second); 
    }
}

void MoveAndTurn(ArRobot& robot, double angle, double dist)
{
    
        //Stop the robot and wait a bit
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
}


