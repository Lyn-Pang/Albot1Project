/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   concave_hull.h
 * Author: arthur
 *
 * Created on June 4, 2018, 6:15 PM
 */

#ifndef CONCAVE_HULL_H
#define CONCAVE_HULL_H

#include "Point.H"

#include <math.h>
//#include <QList>
#include <vector>
//#include <QMap>

/*
class Point
{
public:
    double x;
    double y;
    Point();
    Point(double _x, double _y);
    Point & operator=(const Point & other);
    Point operator+(const Point & other) const;
    Point operator-(const Point & other) const;
    Point operator*(double k) const;
    Point operator/(double k) const;
    bool operator==(const Point & other) const;

    double DotProduct(const Point & other) const;
    double DistanceSquared(const Point & to) const;
    double Distance(const Point & to) const;
    double Distance(const Point & segmentStart, const Point & segmentEnd) const;
    double DecisionDistance(const QList<Point> & points) const;
};*/

double DotProduct(Point & p, Point & other);
double DistanceSquared(Point & p, Point & to);
double Distance(Point & p, Point & to);
double Distance(Point & p, Point & segmentStart, Point & segmentEnd);
//double DecisionDistance(const QList<Point> & points) const;

static double IsLeft(Point p0, Point p1, Point p2);
static bool IsPointInsidePolygon(Point v, vector<Point> & polygon);
static bool CheckEdgeIntersection(Point & p1, Point & p2, Point & p3, Point & p4);
static bool CheckEdgeIntersection(vector<Point> & hull, Point curEdgeStart, Point curEdgeEnd, Point checkEdgeStart, Point checkEdgeEnd);
//static Point NearestInnerPoint(Point edgeStart, Point edgeEnd, const vector<Point> & points, const vector<Point> & hull, bool * found);
vector<Point> FindConvexHull(vector<Point> & points);
//static vector<Point> FindConcaveHull(const vector<Point> & points, double N);

#endif /* CONCAVE_HULL_H */

