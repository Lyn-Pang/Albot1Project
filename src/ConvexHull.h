/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <algorithm>
#include <vector>

//#include "Point.H"


using namespace std;

typedef struct Point_Hull 
{
	double x, y;

	bool operator <(const Point_Hull &p) const 
        {
		return x < p.x || (x == p.x && y < p.y);
	}
};

#ifndef CONVEXHULL_H
#define CONVEXHULL_H

extern vector<Point_Hull> convex_points;
extern vector<Point_Hull> convex_shape;

double cross(const Point_Hull &O, const Point_Hull &A, const Point_Hull &B);
vector<Point_Hull> convex_hull(vector<Point_Hull> P);

vector<Point_Hull> PointToHull(vector<Point> P);
vector<Object> ConvexToObject(vector<Point_Hull> convex);
vector<Point_Hull> ObjectToHull(vector<Object> view);

vector<Object> getBoundingBox(vector<Object> objs);

#endif /* CONVEXHULL_H */

