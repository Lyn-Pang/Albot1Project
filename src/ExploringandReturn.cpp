/**********************************************
 * File:   ExploringandReturn.cpp
 * Author: Wenwang 
 *Modified by: 
 * Created on November 20, 2015, 6:44 PM
 **********************************************/

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
/*
void ReadMemory(int level, int set, int w);
void HomewardWithShortcut(ArRobot& robot, ArSick& sick);
vector<Object> MatchPath(vector<Object> chunk1, vector<Object> chunk2, 
                          vector<Point> rbs1, vector<Point> rbs2,
                            Point key_point1, Point key_point2);
void executionInstruction(vector< vector<Object> > chunks, 
                                vector< vector<Object> > paths,
                                ArRobot& robot, ArSick& sick);
void ExecutionInstAndMove(vector< pair<int, double> > instructions, ArRobot& robot, ArSick& sick);
void MoveAndTurn(ArRobot& robot, double angle, double dist);
pair<int, Point> isExit(ArRobot& robot, ArSick& sick, int numb, int flag);
*/


ArRobot robot;
ArSick sick;

const char* levelName = "inputData/level";
const char* surfaceName = "/surfaces-";
char viewFileName[80];
char rebuildFileName[80];


vector<Object> memoryView;
vector<Object> currentView;
vector<Object> rebuildView;
vector<Object> MemPathView;
vector<Object> CurPathView;
vector < vector<Object> > Memories;

vector<Point> refPoints;
vector<Point> MemPos;
vector<Point> returnPos;

Object refObject;

MyRobot myrobot(0,0);
 
//Point returnp1( -8550.73, -1898.29);
//Point returnp2(2993.31,  2672.53);
//Point returnp3(7356.99,  1009.36);
//Point returnp4(-8026.61,  1806.77);
//Point returnp5(-2913.02,  7153.39);


Point s(0,0);
Point destination;

int left_flag = 0;
int movemnt_flag = 0;
int num = 1;
double distanceOfPos[6];
double midDist;
vector<double> info;
vector< vector<Point> > allPoints;

Memory memoris;

int main(int argc, char **argv) 
{
            char flag;
            int v, w, level, set;
            srand(getpid());

            cout << "argc: " << argc << " argv: " << argv << endl;
            
/*
            cout << "Which level you would like to map" << endl;

            cin >> level;
            cout << "Which Dataset ??? ";
            cin >> set;
            cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
            cin >> w;

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
*/            
            /*
            //reading memory information
            memoris = ReadMemory();
            
            cout <<  "------- Travelling successsful-------" << endl;
            
            cout<<"Do you want to return and process and matching ??? Yes(Y/y)--No(N/n)" << endl << endl;
            cin >> flag;
            
            if(flag == 'y' || flag == 'Y')
            {
                    num = 0;
                    //turning over for returning
                    //ExecutionAndGo(robot, 180, 0);

                    do
                    {
                            destination = memoris.getReturningRobotPs()[memoris.getReturningRobotPs().size() - 1 - num];
                            CollisionAvoidandMove(robot, sick, destination, num);
                            num++;
                    }while(num < memoris.getReturningRobotPs().size());
            }
            */
                        
            
    return 0;
}

/*
void ReadMemory(int level, int set, int w)
{

            for(int i = 1; i <= w; i++)
            {
                    sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, i);
                    memoryView = readASCII(viewFileName);
                    Memories.push_back(memoryView);

                    sprintf(viewFileName, "%s%d%s", "Maps/Offline/MemoryView-", i, ".png");
                    plotObjects(viewFileName, myrobot.getRobot(), memoryView);    
            }
            
           
           returnPos.push_back(returnp5);
           returnPos.push_back(returnp4);
           returnPos.push_back(returnp3);
           returnPos.push_back(returnp2);
           returnPos.push_back(returnp1);
}
*/


