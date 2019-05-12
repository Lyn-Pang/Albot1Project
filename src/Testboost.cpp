#include <iostream>
#include <cmath>
#include <deque>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "readAndwriteASCII.H"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>

#include "Laser2Surface.H"
#include "GeometricOp.H"

#include "Object.H"
#include "PointAndSurface.H"
#include "clipper.hpp"
#include "Point.H"
#include "PathPlanning.H"
#include "Plotting.H"
#include "RobotPosition.h"
#include "ToolFunctions.h"
#include "Returning.h"
#include "ChunksOp.H"
#include "Exploration.h"
#include "FunctionOfMFIS.h"
#include "StructBoundary.h"

#define PI 3.14159265
#define MYPORT 3490 
#define BACKLOG 10

using namespace std;
using namespace ClipperLib;
using namespace boost::assign;
namespace bg = boost::geometry;


const char* levelName = "inputData/level";
const char* surfaceName = "/surfaces-";

char plotFile[80];
char viewFileName[80];
MyRobot myrobot(0,0);


//template <typename Segment>
/*struct gather_segment_statistics
{
    // Remember that if coordinates are integer, the length might be floating point
    // So use "double" for integers. In other cases, use coordinate type
    typedef typename boost::geometry::select_most_precise
        <
            typename boost::geometry::coordinate_type<Segment>::type,
            double
        >::type type;

    type min_length, max_length;

    // Initialize min and max
    gather_segment_statistics()
        : min_length(1e38)
        , max_length(-1)
    {}

    // This operator is called for each segment
    inline void operator()(Segment const& s)
    {
        type length = boost::geometry::length(s);
        if (length < min_length) min_length = length;
        if (length > max_length) max_length = length;
    }
};
*/

int main() 
{
    int v = 1;
    Exit close_exit;
    
    vector< vector<Object> > regions_outline, maps;

    while(v <= 5)
    {
        sprintf(viewFileName, "%s%d%s", "inputData/Chunks/region_map", v,".txt");
        vector<Object> region_map = readASCII(viewFileName);
        //vector<Object> rough_bound_line = join_boundary_verison2(region_map, close_exit);
        sprintf(plotFile, "%s%d%s", "Maps/Offline/region_map", v, ".png");                   
        plotObjects(plotFile, region_map);
        maps.push_back(region_map);
        //regions_outline.push_back(rough_bound_line);
        v++;
    }
    
    //sprintf(plotFile, "%s", "Maps/Offline/integrated_regions_outlines.png");                   
    //plotObjectsColours(plotFile, regions_outline);
    sprintf(plotFile, "%s", "Maps/Offline/integrated_regions_map.png");                   
    plotObjectsColours(plotFile, maps);

}


/*
bool polygonGen(vector<Object> ViewA, Point robotA, vector<Object> ViewB, Point robotB)
{
         typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;

         char coordinates_1[10000];
         char coordinates_2[10000];

         char para_1[10000];
         char para_2[10000];

         double Varea;
         double Iarea;

         polygon poly1,poly2;
         std::deque<polygon> output;
    
         sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, robotA.X(), " ", robotA.Y(), ",");   
         //std::cout << coordinates_1 << std::endl;
        //read the coordinates of each points, 
        for ( int i = 0; i < ViewA.size(); i++) 
        {
                  sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, ViewA[i].X1(), " ", ViewA[i].Y1(), ",");   
                  sprintf(coordinates_1, "%s%f%s%f", coordinates_1, ViewA[i].X2(), " ", ViewA[i].Y2());
                  if (i != ViewA.size() - 1) 
                 {
                      sprintf(coordinates_1, "%s%s", coordinates_1, ",");
                  }
                  else
                  {
                        //sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", ViewA[0].X1(), " ", ViewA[0].Y1());
                        sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", robotA.X(), " ", robotA.Y());
                  }
                  //cout << coordinates_1 << endl;

         }
    
     
         sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, robotB.X(), " ", robotB.Y(), ",");
         for ( int i = 0; i < ViewB.size(); i++) 
         {
                  sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, ViewB[i].X1(), " ", ViewB[i].Y1(), ",");
                  sprintf(coordinates_2, "%s%f%s%f", coordinates_2, ViewB[i].X2(), " ", ViewB[i].Y2());

                  if (i != ViewB.size() - 1) 
                  {
                        sprintf(coordinates_2, "%s%s", coordinates_2, ",");
                  }
                  else
                 {
                        //sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", ViewB[0].X1(), " ", ViewB[0].Y1());
                        sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", robotB.X(), " ", robotB.Y());
                  }

         }
    
    // std::cout << "coordinates" << coordinates << std::endl;
    sprintf(para_1, "%s%s%s", "POLYGON((", coordinates_1, "))");
    sprintf(para_2, "%s%s%s", "POLYGON((", coordinates_2, "))");
    
    // Calculate the area of a cartesian polygon
    
    bg::read_wkt(para_1, poly1);
    bg::read_wkt(para_2, poly2);
    

    
    bg::intersection(poly1, poly2, output);
    
    BOOST_FOREACH(polygon const& p, output)
    {
        Iarea = bg::area(p) / 100000;
        std::cout <<"Intersection area:" << Iarea << std::endl;
    }
    
    Varea = bg::area(poly1) / 100000;
    std::cout << "View 'A' area:" << Varea <<std::endl;


    double percent = (Iarea / Varea) * 100;
    
    if(percent >= 50)
        return true;
    else
        return false;
}

void drawTwoOverlappingPics(vector<Object> ViewA, Point robotA, vector<Object> ViewB, Point robotB) 
{
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    char coordinates_1[10000] = {};
    char coordinates_2[10000] = {};

    char para_1[10000];
    char para_2[10000];

    polygon poly1, poly2, p;
    std::deque<polygon> output;

    sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, robotA.X(), " ", robotA.Y(), ",");

    //read the coordinates of each points, 
    for (int i = 0; i < ViewA.size(); i++) {
        sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, ViewA[i].X1(), " ", ViewA[i].Y1(), ",");
        sprintf(coordinates_1, "%s%f%s%f", coordinates_1, ViewA[i].X2(), " ", ViewA[i].Y2());
        if (i != ViewA.size() - 1) {
            sprintf(coordinates_1, "%s%s", coordinates_1, ",");
        } else {
            //sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", ViewA[0].X1(), " ", ViewA[0].Y1());
            sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", robotA.X(), " ", robotA.Y());
        }

    }
    cout << "coordinates_1:" << coordinates_1 << endl;

    sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, robotB.X(), " ", robotB.Y(), ",");
    for (int i = 0; i < ViewB.size(); i++) {
        sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, ViewB[i].X1(), " ", ViewB[i].Y1(), ",");
        sprintf(coordinates_2, "%s%f%s%f", coordinates_2, ViewB[i].X2(), " ", ViewB[i].Y2());

        if (i != ViewB.size() - 1) {
            sprintf(coordinates_2, "%s%s", coordinates_2, ",");
        } else {
            //sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", ViewB[0].X1(), " ", ViewB[0].Y1());
            sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", robotB.X(), " ", robotB.Y());
        }

    }

    // std::cout << "coordinates" << coordinates << std::endl;
    sprintf(para_1, "%s%s%s", "POLYGON((", coordinates_1, "))");
    sprintf(para_2, "%s%s%s", "POLYGON((", coordinates_2, "))");

    // Calculate the area of a cartesian polygon

    bg::read_wkt(para_1, poly1);
    bg::read_wkt(para_2, poly2);
    
    sprintf(ImgFileName1, "%s", "intersectionmap1.svg");
    sprintf(ImgFileName2, "%s", "intersectionmap2.svg");
    sprintf(ImgFileName3, "%s", "intersectionmap3.svg");

    std::ofstream svg1(ImgFileName1);
    std::ofstream svg2(ImgFileName2);
    std::ofstream svg3(ImgFileName3);

    boost::geometry::svg_mapper<point_type> mapper1(svg1, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper2(svg2, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper3(svg3, 2000, 2000);
    // Add geometries such that all these geometries fit on the map



    bg::intersection(poly1, poly2, output);

    BOOST_FOREACH(p, output) {

    }

    mapper1.add(poly1);

    mapper2.add(poly1);
    mapper2.add(poly2);

    mapper3.add(poly1);
    mapper3.add(poly2);
    mapper3.add(p);


    mapper1.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(p, "fill-opacity:0.8;fill:rgb(0,255,0);stroke:rgb(0,0,153);stroke-width:2", 3);

}
 * */
//////**************************************************************************/////
/*
bool between(double a, double X0, double X1)  
{  
    double temp1= a-X0;  
    double temp2= a-X1;  
    if ( ( temp1<1e-8 && temp2>-1e-8 ) || ( temp2<1e-6 && temp1>-1e-8 ) )  
    {  
        return true;  
    }  
    else  
    {  
        return false;  
    }  
}  
  
 
bool detectIntersect(Point p1, Point p2, Point p3, Point p4)  
{  
    double line_x,line_y; //交点  
    if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
    {  
        return false;  
    }  
    else if ( (fabs(p1.X()-p2.X())<1e-6) ) 
    {  
        if (between(p1.X(),p3.X(),p4.X()))  
        {  
            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
            line_x = p1.X();  
            line_y = k*(line_x-p3.X())+p3.Y();  
  
            if (between(line_y,p1.Y(),p2.Y()))  
            {  
                return true;  
            }  
            else  
            {  
                return false;  
            }  
        }  
        else   
        {  
            return false;  
        }  
    }  
    else if ( (fabs(p3.X()-p4.X())<1e-6) ) //如果直线段p3p4垂直与y轴  
    {  
        if (between(p3.X(),p1.X(),p2.X()))  
        {  
            double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
            line_x = p3.X();  
            line_y = k*(line_x-p2.X())+p2.Y();  
  
            if (between(line_y,p3.Y(),p4.Y()))  
            {  
                return true;  
            }  
            else  
            {  
                return false;  
            }  
        }  
        else   
        {  
            return false;  
        }  
    }  
    else  
    {  
        double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
        double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
  
        if (fabs(k1-k2)<1e-6)  
        {  
            return false;  
        }  
        else   
        {  
            line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
            line_y = k1*(line_x-p1.X())+p1.Y();  
        }  
  
        if (between(line_x,p1.X(),p2.X())&&between(line_x,p3.X(),p4.X()))  
        {  
            return true;  
        }  
        else   
        {  
            return false;  
        }  
    }  
}


void ConvertLogFile() 
{
            Point singleRobotPosition;
            vector< Point > allRobotPosition;

            Point singleLaserPoint;
            vector<Point> aScan;
            vector< vector<Point> > allScans;


            string dName = "Maps/";
            string addForLogFile = dName + "input.log";
        //            addForLogFile += 

            string addForAlbot1 = dName;


            double time;
            vector<double> times;
            ifstream inputFile(addForLogFile.c_str(), ios::in);
            if (inputFile.is_open()) {
                cout << "Reading logfile...." << endl;
                double x, y, theta;
                string data;



                while (!inputFile.eof())
                {

                    inputFile >> data;
                    if (data.compare("robotGlobal:") == 0) 
                    {
                        //reading odometry information
                        inputFile >> x;
                        inputFile >> y;
                        inputFile >> theta;

                        //cout<<"x: "<<x<<" y: "<<y<<" th: "<<theta<<endl;
                        singleRobotPosition.set(x, y);
                        singleRobotPosition.setOAngle(theta);

                        allRobotPosition.push_back(singleRobotPosition);
                        cout << "x: " << singleRobotPosition.X() << " y: " << singleRobotPosition.Y() << " th: " << theta + 90 << endl;
                        //waitHere();
                    }
                    inputFile >> data;
                    if (data.compare("scan1:") == 0) {

                        for (int i = 0; i < 181; i++) {//181 for 1degree resolution 360 for .5 degree resolution
                            inputFile >> x;
                            inputFile >> y;
                            singleLaserPoint.set(x, y);
                            aScan.push_back(singleLaserPoint); //storing all laser readings for this scan
                            cout << "x " << x << " y " << y;
                        }
                        allScans.push_back(aScan); //saving all scans
                        aScan.clear();
                        //waitHere();
                    }
                }


            } else
                cout << "Error opening " << addForLogFile << " .." << endl;


            inputFile.close();


            double thisLaserRange;
            vector<double> distang;
            vector<PointXY> laserPoints;
            vector<Surface> surfaces;
            char fileName[1000];
            int skipCount = 10;
            int stepCount = 1;
            double lastRX = 0.0, lastRY = 0.0, lastRA = 0.0;
            double travelDist = 0.0, turnAngle = 0.0;


            double clusterThreshold = 600; // ct; //atof(argv[1]);
            int surfaceSize = 200; //ss;// atoi(argv[2]);
            int errorThreshold = 150; //et;//atoi(argv[3]);
            //open a file to write for DPSLAM
            cout << "No of scan " << allRobotPosition.size() << endl;
            cout << "No of time " << times.size() << endl;

            for (unsigned int i = 0; i < allRobotPosition.size(); i++) {
                travelDist = getDistBtw2Points(lastRX, lastRY, allRobotPosition[i].X(), allRobotPosition[i].Y());
                turnAngle = allRobotPosition[i].getOAngle() - lastRA;
                if (travelDist > 1000.0 or abs(turnAngle) > 10.0 or i == 0) {

                    //writing laserScan for albot1
                    sprintf(fileName, "%s%s%d", addForAlbot1.c_str(), "laser-", stepCount);
//                    writeLaserScan(fileName, allScans[i]);


                    //writing for albot1
                    if (i == 0) {
                        distang.push_back(0.0);
                        distang.push_back(0.0);
                    } 
                    else 
                    {
                        //distang.push_back(getDistBtw2Points(allRobotPosition[i-1].X(),allRobotPosition[i-1].Y(),allRobotPosition[i].X(),allRobotPosition[i].Y()));
                        //distang.push_back(allRobotPosition[i].getOAngle()-allRobotPosition[i-1].getOAngle());
                        distang.push_back(getDistBtw2Points(lastRX, lastRY, allRobotPosition[i].X(), allRobotPosition[i].Y()));
                        distang.push_back(allRobotPosition[i].getOAngle() - lastRA);
                    }
                    sprintf(fileName, "%s%s%d", addForAlbot1.c_str(), "coordTrans-", stepCount);
                    writeASCII(distang, 2, fileName);
                    distang.clear();
                    for (unsigned int j = allScans[i].size(); j-- > 0;) 
                    {
                        //for(unsigned int j=0;j<allScans[i].size();j++) {
                        thisLaserRange = sqrt(allScans[i][j].X() * allScans[i][j].X() + allScans[i][j].Y() * allScans[i][j].Y());
                        if (thisLaserRange < 30000.0)
                            laserPoints.push_back(PointXY(-allScans[i][j].Y(), allScans[i][j].X()));
                    }
                    surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
                    sprintf(fileName, "%s%s%d", addForAlbot1.c_str(), "surfaces-", stepCount);
                    writeASCII(surfaces, fileName);

                    lastRX = allRobotPosition[i].X();
                    lastRY = allRobotPosition[i].Y();
                    lastRA = allRobotPosition[i].getOAngle();

                    laserPoints.clear();
                    skipCount = 0;
                    stepCount++;
                    cout << "Step " << stepCount << " completed" << endl;

                }

                skipCount++;

            }
}



//A is P1 of exit, B is P2 of exit
//front object P2 to exit 
vector<double> frontObToExit(vector<Object> MpathView, Point A, Point B)
{
    int Obj_pos;
    double dist1, dist2;
    double avg_dist;
    double exitSize;
    vector<double> rnt;
    
    Point frontPoint;
    
    for(int i =0; i < MpathView.size(); i++)
    {
        Obj_pos = SingleObjectPosition(MpathView[i]);
        if(Obj_pos == 3)
            frontPoint.set(MpathView[i].X2(), MpathView[i].Y2());
    }
    
    dist1 =  GetPointDistance(A, frontPoint);    
    dist2 = GetPointDistance(B, frontPoint);
    avg_dist =  (dist1+dist2) / 2;
    
    exitSize = GetPointDistance(B, A);
     rnt.push_back(dist1);
     rnt.push_back(dist2);
     rnt.push_back(exitSize);
     
    return rnt;
}






vector<Object> pathInView(vector<Object> CV, vector<Point> path)
{
        vector<Object> temp;
        Object temp_path1, temp_path2;
        Object intersect;
        vector<Exit> exits;
        Point mid; // mid point of each potential exit
        Point orig(0,0); // origin view
        Point P1, P2; // for establishing path in view
        double disToRobot = 0;
        double path2Dist = GetPointDistance(path[1], path[2]);
        double path1Dist = GetPointDistance(path[0], path[1]);
        double angle = AngleOfNext(path[0], path[1], path[2]);
        double temp_angle;
        double temp_dist;
        double direction = 0;
        double x,y;
        
        //scanning all potential exits in the view
        exits = findGapasExits(CV);
        // plot the potential path in the view
        for(int i = 0; i < exits.size(); i++)
        {
                mid.set((exits[i].X1() + exits[i].X2() / 2) , (exits[i].Y1() + exits[i].Y2() / 2));
                disToRobot = GetPointDistance(orig, mid);
                if(disToRobot < path2Dist)
                {
                       temp_dist = sqrt(mid.X() * mid.X() + mid.Y() * mid.Y());
                       temp_angle = acos(mid.Y() /   temp_dist); 
                       //temp_angle = 180 / PI * temp_angle;
                        
                       y = cos(temp_angle) * disToRobot;
                       x = sqrt(disToRobot * disToRobot - y * y);
                       
                        P1.set(x, y);
                        //establish path on this direction
                        intersect = IntersectForReturn(CV, P1) ;
                        if(intersect.getID() == 0)
                        {
                                // establishing the second point
                                y = path1Dist * cos(angle) + P1.Y();
                                x = sqrt(path1Dist * path1Dist - y * y);
                                P2.set(x, y);
                                // checking whether the second segment is intersected 
                                intersect = IntersectForReturn(CV, P2);
                                if(intersect.getID() == 0)
                                {
                                        // push two segments
                                        temp_path1.set(orig.X(), orig.Y(), P1.X(), P1.Y(), 1);
                                        temp_path2.set(P1.X(), P1.Y(), P2.X(), P2.Y(), 2);

                                        temp.push_back(temp_path1);
                                        temp.push_back(temp_path2);
                                }
                        }
                        else
                            continue;
                    }
                    else
                        continue;
        }
        // choose which one is the most possible

        // return the possible path direction and distance
        return temp;
}



vector<Object> GroupAndSimp(vector<Object> CV)
{
    vector<Object> rtn;
    vector<Object> temp;
    
    for(int i = 0; i < CV.size() - 1; i++)
    {
        //surfaces should be small
        if(CV[i].length()  < 500)
        {
                //small gap between adjacent surface
                if(CV[i].distP2ToP1(CV[i+1]) < 500 && CV[i+1].length() < 500)
                {
                    temp.push_back(CV[i]);
                    temp.push_back(CV[i+1]);
                }
        }
    }
    
    return temp;
}


void IntersectedWithCircle(Point cirCenter, double cirR, vector<Object> CV)
{
        //const double PI=3.1415926;
        Point p1,p2; // line segment P1 & P2

        //x=r*cos(a)+x0  y=r*sin(a)+y0
        //          0<=a<=2PI
        //cin>>p1.x>>p1.y>>p2.x>>p2.y;
        //y=ax+b
        // a=(y2-y1)/(x2-x1)   b=(x1y2-x2y1)/(x1-x2)
        
        for(int i = 0; i < CV.size(); i++)
        {
            
            p1.set(CV[i].X1(), CV[i].Y1());
            p2.set(CV[i].X2(), CV[i].Y2());
                double a=(p2.Y() - p1.Y()) / (p2.X() - p1.X());
                double b=(p1.X() * p2.Y() - p2.X() * p1.Y()) / (p1.X() - p2.X());
                for(double thete = 0;thete <= 2*PI; thete+=PI/180)
                { 
                       double x=cirR*cos(thete)+cirCenter.X();
                       double y=cirR*sin(thete)+cirCenter.Y();   //x,y
                       if(fabs(y-a*x-b)<1e-5)  //y=ax+b
                       { 
                           if((p1.X()<=x && x<=p2.X() || p1.X()>=x && x>=p2.X()) && (p1.Y()<=y && y<=p2.Y() || p1.Y()>=y && y>=p2.Y())) //
                           cout<<"It is intersected x="<<x<<"，y="<<y<<endl;
                       }
                }
        }
}

//cirCenter is the centre of circle, cirR is radius, s this starting of the arc, n is the end of the arc
// s from 0 or < 2    mix of n is 2    n * PI = 360 degree 
bool IntersectedWithArc(Point cirCenter, double cirR, vector<Object> CV, double s, double n)
{
        //const double PI=3.1415926;
        Point p1,p2; // line segment P1 & P2
        unsigned char flag = 0;
  
        for(int i = 0; i < CV.size(); i++)
        {
            
                p1.set(CV[i].X1(), CV[i].Y1());
                p2.set(CV[i].X2(), CV[i].Y2());
                double a=(p2.Y() - p1.Y()) / (p2.X() - p1.X());
                double b=(p1.X() * p2.Y() - p2.X() * p1.Y()) / (p1.X() - p2.X());
                
                for(double thete = s;thete <= n*PI; thete+=PI/180)
                { 
                       double x=cirR*cos(thete)+cirCenter.X();
                       double y=cirR*sin(thete)+cirCenter.Y();   //x,y
                       
                       cout<<" x is : "<< x<<" y is : "<<y<<endl;
                       cout<<" y = ax + b ---"<< fabs(y-a*x-b)<<endl;
                       //if(fabs(y-a*x-b)<1e-5)  //y=ax+b
                       //{ 
                           //cout <<"test programme!!"<<endl;
                            if((p1.X()<=x && x<=p2.X() || p1.X()>=x && x>=p2.X()) && (p1.Y()<=y && y<=p2.Y() || p1.Y()>=y && y>=p2.Y())) //
                            {
                                 cout<<"It is intersected x="<<x<<"，y="<<y<<endl;
                                 flag = 1;
                                 return true;
                            }
                            else
                                flag = 0;
                       //}
                }
        }
        
        if(flag == 0)
            return false;
}



void generatPathes(vector<Object> CV, vector<Position> cur, double path)
{
        Point interPoint; // for obtain slope of path
        vector<Point> allPath;
        double x ,y; // for path end point
        double angle_a, angle_b, dist_a;
        
        cout<<"-----This is going to process pathes-----"<<endl<<endl;
        
        for(int i = 0; i < cur.size(); i++)
        {
                angle_a = asin(cur[i].getDistoPath() / cur[i].getDist());
                angle_b = acos(cur[i].getPoint().X() / cur[i].getDist());
                cout<<"DistoPath is : "<<cur[i].getDistoPath()<<"   Dist is :"<<cur[i].getDist()<<endl;
                //cout<< "left or right??"<<cur[i].getLeftOrR() <<endl;

                if(cur[i].getLeftOrR() == 0)
                {
                    x = cos(PI - angle_b - angle_a) * path;
                    cout<<"angle a is : "<<angle_a<<"  angle b is : "<<angle_b<<endl;
                //}
                //else
                //{
                    //if(cur[i].getLeftOrR() == 1)
                    //{
                     //       x = cos(angle_a + angle_b) * path;
                    //        cout<<"angle a is : "<<angle_a<<"  angle b is : "<<angle_b<<endl;
                    
                 //}
      
                y = sqrt(path * path - x * x);
                interPoint.set(x, y);
                allPath.push_back(interPoint);
               }
        }
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/ViewWithPaths-", 1, ".png");
        //plotObjectsOf3KindswithPaths(viewFileName, myrobot.getRobot(), CV, allPath);
        
}

void findPath()
{
        MyRobot robot(0, 0);
        char surFileName[80];
        char viewFileName[80];
        vector<Exit> exits;
        vector<Object> tmp;
        Object obj;
        for (unsigned i = 0; i < 53; i++)
        {
                sprintf(surFileName, "%s%d", "inputData/level4set3/surfaces-", 1);
                vector <Object> firstView = readASCII(surFileName);
                //cout << firstView.size() << endl;
                sprintf(viewFileName, "%s%d%s", "Maps/viewandentry/view-", 1, ".png");
                exits = findShortestExits(firstView);
                // cout << "Number of exits:" << exits.size() << endl;
                tmp = convertExitToObject(exits);
                plotObjectsOf3Kinds(viewFileName, firstView, tmp, robot.getRobot());
        }
}  
*/

//*************************************************************************//
//*************************************************************************//
vector<Exit> exitsIdentify(vector<Object> cv)
{

        double distp2top1;
        Exit exit;
        int no_exit;
        int exits_counter = 1;
        vector<Exit> exits;
        vector<Exit> realExits;
       
        
        for (int i = 0; i<int(cv.size() - 1); i++) 
        {
            if (cv[i].getP2OS() == true)// occluding point
            {
                no_exit = 0;
                for (int j = i + 1; j<int(cv.size()); j++) 
                {
                    if(((cv[j].getP1OS() == true) || (cv[j].distP1ToP2(cv[j - 1]) == 0))
                        && j > i + 1)
                    {
                        if (no_exit == 0) 
                        {
                                exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                exit.setP1ID(cv[i].getID());
                                exit.setP2ID(cv[j].getID());
                                no_exit++;
                        } 
                        else 
                        {
                                distp2top1 = cv[i].distP2ToP1(cv[j]);
                                if (exit.length() > distp2top1) //condition to get shortest exit
                                { 
                                    exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                    exit.setP1ID(cv[i].getID());
                                    exit.setP2ID(cv[j].getID());
                                }
                        }
                    }
                }
            }
            else
            {
                if(cv[i].distP2ToP1(cv[i + 1]) == 0)// corner point
                {
                    no_exit = 0;
                    for(int j = i + 1; j<int(cv.size()); j++)
                    {
                        if(cv[j].getP1OS() == true && cv[j].distP1ToP2(cv[j-1]) > 0)//second point of exit should be occluding only
                        {
                            if (no_exit == 0) 
                            {
                                    exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                    exit.setP1ID(cv[i].getID());
                                    exit.setP2ID(cv[j].getID());
                                    no_exit++;
                            } 
                            else 
                            {
                                    distp2top1 = cv[i].distP2ToP1(cv[j]);
                                    if (exit.length() > distp2top1) //condition to get shortest exit
                                    { 
                                        exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                        exit.setP1ID(cv[i].getID());
                                        exit.setP2ID(cv[j].getID());
                                    }
                            }
                        }
                    }
                }
            }
            exits.push_back(exit);
            exits_counter++;
        }

        
        for (int i = 0; i<int(exits.size()); i++) 
        {
            if (exits[i].length() > 800) 
                realExits.push_back(exits[i]);
        }

        return realExits;
}

void isFamiliarplace(vector<Object> view1, vector<Object> view2)
{
    double a = 0; 
    double b = 0;
    //vector<Surface> V_a = makePolygonOfCV(view1);
    //vector<Surface> V_b = makePolygonOfCV(view2);
    
    //vector<Object> polygonObjects;
    vector<Object> land1, land2;
    ClipperLib::Path subj1, subj2;
    Paths solution1, solution2;
    ClipperOffset co1, co2;

    subj1 = viewConvertPath(view1); //convert view into Path
    subj2 = viewConvertPath(view2);
      
    a = ClipperLib::Area(subj1) / 100000;
    b = ClipperLib::Area(subj2) / 100000;
    double deviation = a / b;
    cout<<" the area a view 1: "<< a << endl;
    cout<<" the area b view 2: "<< b << endl;
    cout<<" the deviation : "<< deviation << endl;
    
    //land1 = collectLandObjects(view1);
    //land2 = collectLandObjects(view2);
}

vector<Object> simplfyMap(vector<Object> map)
{
        Object temp_Obj;
        vector<Object> temp;
    
        typedef boost::geometry::model::d2::point_xy<double> xy;
        boost::geometry::model::linestring<xy> line;
        
        for(int i = 0; i < map.size(); i++)
        {
            line += xy(map[i].X1(), map[i].Y1()), xy(map[i].X2(), map[i].Y2());
        }
        
        // Simplify it, using distance of 0.5 units
        bg::model::linestring<xy> simplified;
        bg::simplify(line, simplified, 100);
        cout
        << "  original: " << bg::dsv(line) << endl
        << "simplified: " << bg::dsv(simplified) << endl;
        
        
        
        //sprintf(testFileName, "%s%s%d", "Maps/Offline/", "Simplified_surface-", 39);
        //plotObjects(testFileName, rnt2);
        
    
        return temp;
}


/*
void thresh_callback(int, void* )  
{  
  Mat threshold_output;  
  vector<vector<Point> > contours;    //轮廓数组（非矩形数组），每个轮廓是一个Point型的vector  
  vector<Vec4i> hierarchy;                 //见下面findContours的解释  
   
  /// 使用Threshold二值  
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );  
   
  /// 找到轮廓  
  //contours参数为检测的轮廓数组，每一个轮廓用一个point类型的vector表示  
  //hiararchy参数和轮廓个数相同，每个轮廓contours[ i ]对应4个hierarchy元素hierarchy[ i ][ 0 ] ~hierarchy[ i ][ 3 ]，  
  //分别表示后一个轮廓、前一个轮廓、父轮廓、内嵌轮廓的索引编号，如果没有对应项，该值设置为负数。  
  //CV_RETR_TREE：建立一个等级树结构的轮廓  
  //  
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );  
   
  /// 多边形逼近轮廓 + 获取矩形和圆形边界框  
  vector<vector<Point> > contours_poly( contours.size() );          //近似后的轮廓点集  
  vector<Rect> boundRect( contours.size() );                           //包围点集的最小矩形vector  
  vector<Point2f>center( contours.size() );                               //包围点集的最小圆形vector  
  vector<float>radius( contours.size() );                                   //包围点集的最小圆形半径vector  
   
  for( int i = 0; i < contours.size(); i++ )  
     {  
   approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );      //对多边形曲线做适当近似，contours_poly[i]是输出的近似点集  
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );         //计算并返回包围轮廓点集的最小矩形  
       minEnclosingCircle( contours_poly[i], center[i], radius[i] );     //计算并返回包围轮廓点集的最小圆形及其半径  
     }  
  
  /// 画多边形轮廓 + 包围的矩形框 + 圆形框  
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );  
  for( int i = 0; i< contours.size(); i++ )  
     {  
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );   //随机颜色  
    //   drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );  
       drawContours( drawing, contours_poly, i, color, 1, 8, hierarchy, 0, Point() );         //根据轮廓点集contours_poly和轮廓结构hierarchy画出轮廓  
       rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );              //画矩形，tl矩形左上角，br右上角  
       circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );                                        //画圆形  
     }  
  /// 显示在一个窗口  
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );  
  imshow( "Contours", drawing );  
}  
*/
