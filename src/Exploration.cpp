/*
 * Exploration : Exploring exit in an individual view
 * Author : Wenwang
 * Version : 
 */
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>

#include "PointAndSurface.H"
#include "PathPlanning.H"
#include "Point.H"
#include "Object.H"


vector<Point> PlanThePath(vector<Object> currentView, Exit destination)
{
    Point EndP, temp_point;  // one of the end point of destination
    Object temp; // temp object connecting all other points
    vector<Point> pathSequence; // a set of moving points
    unsigned char leftRight_Flag = 0;
    
    if(destination.distP1ToPoint(0,0) > destination.distP2ToPoint(0,0))
    {
        EndP.set(destination.X2(), destination.Y2());
        leftRight_Flag = 1; //P2 is picked 
    }
    else
        if(destination.distP1ToPoint(0,0) < destination.distP2ToPoint(0,0))
        {
            EndP.set(destination.X1(), destination.Y1());
            leftRight_Flag = 0; //P1 is picked
        }
    
    cout<< " The picked end point is: "<< (int)leftRight_Flag << endl;
    
    //direction to connect other points
    //if(EndP.X() < 0) // anti-clockwise 
    //{
        if(leftRight_Flag == 0)
        {
            for(int i = currentView.size() - 1; i >= destination.getP2ID(); i--)
            {
                temp.set(EndP.X(), EndP.Y(), currentView[i].X2(), currentView[i].Y2(), 1);
                temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                pathSequence.push_back(temp_point);
                temp.set(EndP.X(), EndP.Y(), currentView[i].X1(), currentView[i].Y1(), 1);
                temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                pathSequence.push_back(temp_point);
            }
        }
        else
            if(leftRight_Flag == 1)
            {
                for(int i = 0; i <= destination.getP1ID() - 1; i++)
                {
                    temp.set(EndP.X(), EndP.Y(), currentView[i].X1(), currentView[i].Y1(), 1);
                    temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                    pathSequence.push_back(temp_point);
                    temp.set(EndP.X(), EndP.Y(), currentView[i].X2(), currentView[i].Y2(), 1);
                    temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                    pathSequence.push_back(temp_point);
                }
            }
    //}
    /*
    else
    {
        if(EndP.X() > 0) // clockwise to connect
        {
            if(leftRight_Flag == 0)
            {
                for(int i = 0; i <= destination.getP2ID() + 1; i++)
                {
                    temp.set(EndP.X(), EndP.Y(), currentView[i].X2(), currentView[i].Y2(), 1);
                    temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                    pathSequence.push_back(temp_point);
                    temp.set(EndP.X(), EndP.Y(), currentView[i].X1(), currentView[i].Y1(), 1);
                    temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                    pathSequence.push_back(temp_point);
                }
            }
            else
                if(leftRight_Flag == 1)
                {
                    for(int i = 0; i <= destination.getP1ID() + 1; i++)
                    {
                        temp.set(EndP.X(), EndP.Y(), currentView[i].X1(), currentView[i].Y1(), 1);
                        temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                        pathSequence.push_back(temp_point);
                        temp.set(EndP.X(), EndP.Y(), currentView[i].X2(), currentView[i].Y2(), 1);
                        temp_point.set((temp.X1() + temp.X2()) / 2, (temp.Y1() + temp.Y2()) / 2);
                        pathSequence.push_back(temp_point);
                    }
                }
        }
    }
    */
    
    return pathSequence;
} 







