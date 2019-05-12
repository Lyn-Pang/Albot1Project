/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FindEntry.cpp
 * Author: arthur
 *
 * Created on December 25, 2015, 2:01 PM
 */

#include <cstdlib>
#include <vector>
#include <cmath>
#include "Object.H"
#include "FindGap.h"
#include "Plotting.H"
#include "RobotFuncs.H"
#include "readAndwriteASCII.H"
using namespace std;

/*
 * 
 */
int main(int argc, char** argv)
{
    char viewFileName[80];
    vector<Object> memView;
    vector<Object> entries;
    MyRobot robot(0, 0);
    vector<Object> boundaries;
    vector<Point> entryPoints;
    vector<Object> pointObj;
    Object obj;
    for (int i = 1; i < 62; i++)
    {
        pointObj.clear();
        
        sprintf(viewFileName, "%s%d", "inputData/level1set2/surfaces-", i);
        memView = readASCII(viewFileName);
        boundaries = BoundaryByExits(memView);
        entries = findEntry(boundaries, -1);
       
         entryPoints = findEntryPoints(boundaries,-1);
         if(entryPoints.size()==2)
         {
             obj.set(entryPoints[0].X(),entryPoints[0].Y(),entryPoints[1].X(),entryPoints[1].Y(),0);
             pointObj.push_back(obj);
         }
         
        sprintf(viewFileName, "%s%d%s", "Maps/viewandentry/viewandentryleft", i, ".png");
        plotObjectsOf4Kinds(viewFileName, memView,boundaries, entries, pointObj);
        
             entries = findEntry(boundaries, 0);
       
         entryPoints = findEntryPoints(boundaries,0);
                 pointObj.clear();
         if(entryPoints.size()==2)
         {
             obj.set(entryPoints[0].X(),entryPoints[0].Y(),entryPoints[1].X(),entryPoints[1].Y(),0);
             pointObj.push_back(obj);
         }
         
        sprintf(viewFileName, "%s%d%s", "Maps/viewandentry/viewandentrymiddle", i, ".png");
        plotObjectsOf4Kinds(viewFileName, memView,boundaries, entries, pointObj);

             entries = findEntry(boundaries, 1);
       
         entryPoints = findEntryPoints(boundaries,1);
                 pointObj.clear();
         if(entryPoints.size()==2)
         {
             obj.set(entryPoints[0].X(),entryPoints[0].Y(),entryPoints[1].X(),entryPoints[1].Y(),0);
             pointObj.push_back(obj);
         }
         
        sprintf(viewFileName, "%s%d%s", "Maps/viewandentry/viewandentryright", i, ".png");
        plotObjectsOf4Kinds(viewFileName, memView,boundaries, entries, pointObj);
        
    }
    return 0;
}

vector<Point> findEntryPoints(vector<Object> view, int orientation)
{
    vector<Object> entries;
    Point p1, p2, p3, p4;
    double op1p3, op1p4, op2p3, op2p4;
    vector<Point> tmp;
    entries = findEntry(view, orientation);
    if (entries.size() == 2)
    {
        p1.set(entries[0].X1(), entries[0].Y1());
        p2. set(entries[0].X2(), entries[0].Y2());
        p3.set(entries[1].X1(), entries[1].Y1());
        p4.set(entries[1].X2(), entries[1].Y2());
        op1p3 = getDistance(0, 0, p1.X(), p1.Y()) + getDistance(0, 0, p3.X(), p3.Y()) + getDistance(p1.X(), p1.Y(), p3.X(), p3.Y());
        op1p4 = getDistance(0, 0, p1.X(), p1.Y()) + getDistance(0, 0, p4.X(), p4.Y()) + getDistance(p1.X(), p1.Y(), p4.X(), p4.Y());
        op2p3 = getDistance(0, 0, p2.X(), p2.Y()) + getDistance(0, 0, p3.X(), p3.Y()) + getDistance(p2.X(), p2.Y(), p3.X(), p3.Y());
        op2p4 = getDistance(0, 0, p2.X(), p2.Y()) + getDistance(0, 0, p4.X(), p4.Y()) + getDistance(p2.X(), p2.Y(), p4.X(), p4.Y());
        if (op1p3 < op1p4 && op1p3 < op2p3 && op1p3 < op2p4)
        {
            
            tmp.push_back(p1);
            tmp.push_back(p3);
        } else if (op1p4 < op1p3 && op1p4 < op2p3 && op1p4 < op2p4)
        {
            tmp.push_back(p1);
            tmp.push_back(p4);
        } else if (op2p3 < op1p3 && op2p3 < op1p4 && op2p3 < op2p4)
        {
            tmp.push_back(p2);
            tmp.push_back(p3);
        } else
        {
            tmp.push_back(p2);
            tmp.push_back(p4);
        }
    }
    return tmp;
}

vector<Object> findEntry(vector<Object> view, int orientation)
{
    vector<Object> entries;
    vector<Object> srcObjs;
    Point leftp, rightp;
    double leftangle, rightangle;
    for (int i = 0; i < view.size(); i++)
    {
        if (orientation < 0)
        {
            if (view[i].X1() < 0 || view[i].X2() < 0)
            {
                srcObjs.push_back(view[i]);
            }
        } else if (orientation > 0)
        {
            if (view[i].X1() > 0 || view[i].X2() > 0)
            {
                srcObjs.push_back(view[i]);
            }
        } else
        {
            leftp.set(view[i].X1(), view[i].Y1());
            rightp.set(view[i].X2(), view[i].Y2());
            leftangle = getXAngleOfPoint(leftp);
            rightangle = getXAngleOfPoint(rightp);
            if ((leftangle > 60 && leftangle < 120) || (rightangle > 60 && rightangle < 120))
            {
                srcObjs.push_back(view[i]);
            }
        }
    }
    srcObjs = sortObjectsByDistance(srcObjs);
    if (srcObjs.size() >= 2)
    {
        entries.push_back(srcObjs[0]);
        entries.push_back(srcObjs[1]);
    } else
    {

        cout << "The Objects are too less." << endl;
    }
    /*
    for (int i = 0; i < srcObjs.size(); i++)
    {
        if (getDistance(0, 0, srcObjs[i].X1(), srcObjs[i].Y1()) < getDistance(0, 0, srcObjs[i].X2(), srcObjs[i].Y2()))
        {
            tmpPoint.set(srcObjs[i].X1(), srcObjs[i].Y1());
        } else
        {
            tmpPoint.set(srcObjs[i].X2(), srcObjs[i].Y2());
        }
        closerPoints.push_back(tmpPoint);
    }
    closerPoints = sortPointsByDistance(closerPoints);
    if (closerPoints.size() >= 2)
    {
        closerPoints[0].display();
        closerPoints[1].display();
        tmpObj.set(closerPoints[0].X(), closerPoints[0].Y(), closerPoints[1].X(), closerPoints[1].Y(), 0);
        entries.push_back(tmpObj);
    } else
    {
        cout << "The points are too less." << endl;
    }
     */

    return entries;

}

double getXAngleOfPoint(Point p)
{

    Object tmp;
    double sinangle;
    tmp.set(0, 0, p.X(), p.Y(), 0);
    sinangle = getAngleofOneObject(tmp);
}

double getDistance(double x1, double y1, double x2, double y2)
{

    return sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));
}

vector<Point> sortPointsByDistance(vector<Point> src)
{
    Point tmp;
    double distance1, distance2;
    for (int i = 0; i < src.size(); i++)
    {
        for (int j = 0; j < src.size() - i - 1; j++)
        {
            distance1 = getDistance(0, 0, src[j].X(), src[j].Y());
            distance2 = getDistance(0, 0, src[j + 1].X(), src[j + 1].Y());
            if (distance1 > distance2)
            {

                tmp = src[j];
                src[j] = src[j + 1];
                src[j + 1] = tmp;
            }
        }
    }
    return src;
}

vector<Object> sortObjectsByDistance(vector<Object> src)
{
    Object tmp;
    double distance1, distance2;
    for (int i = 0; i < src.size(); i++)
    {
        for (int j = 0; j < src.size() - i - 1; j++)
        {
            if (getDistance(0, 0, src[j].X1(), src[j].Y1()) < getDistance(0, 0, src[j].X2(), src[j].Y2()))
            {
                distance1 = getDistance(0, 0, src[j].X1(), src[j].Y1());
            } else
            {
                distance1 = getDistance(0, 0, src[j].X2(), src[j].Y2());
            }
            if (getDistance(0, 0, src[j + 1].X1(), src[j + 1].Y1()) < getDistance(0, 0, src[j + 1].X2(), src[j + 1].Y2()))
            {
                distance2 = getDistance(0, 0, src[j + 1].X1(), src[j + 1].Y1());
            } else
            {
                distance2 = getDistance(0, 0, src[j + 1].X2(), src[j + 1].Y2());
            }
            if (distance1 > distance2)
            {
                tmp = src[j];
                src[j] = src[j + 1];
                src[j + 1] = tmp;
            }
        }
    }
    return src;
}

