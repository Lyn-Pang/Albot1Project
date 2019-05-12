
#include <stdio.h>
#include <iostream>
#include <vector>

#include "Object.H"
#include "Point.H"
#include "Ellipse.h"
#include "GeometricOp.H"

void Ellipse ::setCentre(Point centre_point)
{
    centre = centre_point;
}
Point Ellipse ::getCentre()
{
    return centre;
}
void Ellipse ::setHeigth(double negtive_axis)
{
    heigth = negtive_axis;
}
double Ellipse ::getHeigth()
{
    return heigth;
}
void Ellipse ::setwidth(double principle_axis)
{
    width = principle_axis;
}
double Ellipse ::getwidth()
{
    return width;
}
void Ellipse ::setOrientation(double orient)
{
    orientation = orient;
}
double Ellipse ::getOrientation()
{
    return orientation;
}

void Ellipse::setOneEllipse(Point centre_point, double H, double W, double orient)
{
    centre = centre_point;
    heigth = H;
    width = W;
    orientation = orient;
}

void Ellipse::setlTwoEndpoints(Point left, Point right)
{
    left_end = left;
    right_end = right;
}

vector<Point> Ellipse::fourEndPoints()
{
    Point top, bottom;
    vector<Point> fourPoints;
    Object base_line;
    base_line.set(left_end.X(), left_end.Y(), right_end.X(), right_end.Y(), 1);
    
    top = outSidePerpendPointWithLength(base_line, heigth/2, centre, 1);
            
    bottom = outSidePerpendPointWithLength(base_line, heigth/2, centre, 2);

    //cout << "left is : " << left_end.X() << "; " << left_end.Y() << endl;
    //cout << "right is : " << right_end.X() << "; " << right_end.Y() << endl;
    //cout << "top is : " << top.X() << "; " << top.Y() << endl;
    //cout << "bottom is : " << bottom.X() << "; " << bottom.Y() << endl;
    
    fourPoints.push_back(left_end);
    fourPoints.push_back(right_end);
    fourPoints.push_back(top);
    fourPoints.push_back(bottom);
    
    return fourPoints;
}


Object Ellipse::getHorizonline()
{
    vector<Point> points; 
    Object line;
    
    points = Ellipse::fourEndPoints();
    line.set(points[0].X(), points[0].Y(), points[1].X(), points[1].Y(), 1);
    
    return line;
    
}
Object Ellipse::getVirticalline()
{
    vector<Point> points; 
    Object line;
    
    points = Ellipse::fourEndPoints();
    line.set(points[2].X(), points[2].Y(), points[3].X(), points[3].Y(), 1);
    
    return line;
}

void Ellipse::setVN(int viewNumber)
{
    VN =viewNumber;
}
int Ellipse::getVN()
{
    return VN;
}
