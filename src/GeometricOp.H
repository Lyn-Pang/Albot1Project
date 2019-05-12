
#ifndef GEOMETRICOP_H
#define GEOMETRICOP_H

#include <iostream>
#include <vector>
#include "Point.H"
#include "Object.H"
#include "clipper.hpp"
#include "PathPlanning.H"

using namespace std;
using namespace ClipperLib;


vector<Object> xformPVIntoCV(vector<Object> pview, Point rpose, double angle);
vector<Point> xformPointsIntoCV(vector<Point> pview, Point rpose, double angle);
vector<Object> xformPObjectsIntoCV(vector<Object> pview, Point rpose, double angle);
vector<Object> discardLinesIFoR(vector<Object> pview, Point rpose, double angle, vector<Object> cview, Object rmfis,Object rcv,int rp);
//vector<RfPoint> xformRefPointsIntoCV(vector<RfPoint> pview, Point rpose, double angle);
vector<Object> xformCVIntoPV(vector<Object> cview, Point rpose, double angle);

double checkForIntersection (Object s1, Object s2);
vector<double> getIntersectionPoint(Object s1, Object s2);
double getDistBtw2Points(double x1, double y1, double x2, double y2);
double angleObjectAndPoint(Object s, double x3, double y3);
double angleObjectAndXaxis(Object s);
vector<Point> p2cCoordinate(vector<double> vec); //for robot only
double getX(double angle,double dist);
double getY(double angle,double dist);
bool pointInPolygon(Object a, Object b, double x, double y);
//calling clipper algorithm
bool pointInPolygon(Point p, vector<Object> view);
vector<Object> breakTheLineInto(Object smfis);
vector<Object> breakTheLinesInto(vector<Object> allLines);
Object makeLineAtPointWithObject(double angle, double dist, Object smfis);
Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis);
Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis, int a);
vector<Object> makeSquareAtLoSPoints(vector<Object> lineOfSitePoints);
Object makeLineAtTwoPointsWithObject(double angle1, double dist1, double angle2, double dist2, Object smfis, int referencePoint);
Object CreateLineOnObject(double angle1, double dist1, Object smfis, Object detect, int referencePoint, int side_flag);
void waitHere();

vector<Object> makeSquare(Object oneSide);
vector<Object> makeSquare(Object oneSide, double length, int ref_point);

vector<Object> makeAllSquare(vector<Object> allLines);

vector<Object> makeRectangle(Object oneSide);
vector<Object> makeAllRectangle(vector<Object> allLines);

vector<Object> makeArrow(Object oneSide);
vector<Object> drawArrow(Object oneSide, int view_num);

Object makeParallelObject(Object oneSide, double dist,const char side);
Object makeParallelObject(Object oneSide, double dist, int side);

vector<Surface> makePolygonOfCV(vector<Object> cView);
vector<Surface> makePolygonOfLM(vector<Object> currentLM);
vector<Object> makePolygonOfView(vector<Object> View);
vector<Object> makePolygon_Clipper(vector<Object> View, double adjust_size);
vector<Object> Polygons_intersection_Clipper(vector<Object> View, vector<Object> View2);

vector<Surface> findExactBoudaryFrom(vector<Object> cView);

vector<Object> xformObjectsbyXY(vector<Object> inputview,double x, double y);

Point transformToCV(Point pos, Point near, double angle);


ClipperLib::Path viewConvertPath(vector<Object> view);
vector<Object> PathsConvertView(ClipperLib::Paths solution);

vector<Object> fillGapOfView(vector<Object> view);

Point symmetric_point(Point p1, Point l1, Point l2);

Point outSidePerpendPoint(Object line, double offset_length, Point crossPoint, int positive_flag);
Point outSidePerpendPointWithLength(Object line, double offset_length, Point crossPoint, int positive_flag);
Point outSidePerpendPointWithLeftRight(Object line, double offset_length, Point crossPoint, int leftOrright);

//shortest distance from a point to line segment
double P_To_ShortestDistance(Point P, Object line);
double FindDistanceToSegment(Point p, Object line_segment);
double FindDistanceToSegment(double x1, double y1, double x2, double y2, double pointX, double pointY);

/* expend an object following it
 * with particular length and return 
 * the expended part rather then the
 * complete object expended
 */
Object expend_Object(Object s1, double exp_length, int flag);
Object expend_Object_two_sides(Object s1, double dist1, double dist2);

vector<Object> rotate_on_point(vector<Object> view, Point p, double angle);
Object TransformforToGlobalCoordinate(Object Obj, Point coord_to_shift, double angle);
vector<Object> TransformforToGlobalCoordinate(vector<Object> addView, Point coord, vector<Object> currentPos, double angle);

Exit Transform_Exit_GlobalCoordinate(Exit exit, Point coord, double angle);
vector<Exit> Transform_Exit_GlobalCoordinate(vector<Exit> exits, Point coord, vector<Object> currentPos, double angle);



/* calculate the trendline of a list of surface/points
 * return slope and intercept
 * first is slope, second is intercept
 */
pair<double, double> Trendline_parameter(vector<Object> info);

/* included angle between two vectors
 * sina = y1 / (sqrt(x1^2 + y1^2)) ==> a = arcsin[y1 / (sqrt(x1^2 + y1^2))]
 * sinb = (x2 - x1) / (sqrt((x2 - x1)^2 + (y2 - y1)^2)) ==> b = arcsin[(x2 - x1) / (sqrt((x2 - x1)^2 + (y2 - y1)^2))]
 * c = 90 - a
 * included_angle = b + c
 */
double includedAngle(Object s1, Object s2);
double includedAngle(Point p11, Point p12, Point p21, Point p22);
Point intersectPointTwolineEquations(Object line1, Object line2);

/* check whether two line are parallel 
 */
bool parallel_objects(Object obj1, Object obj2);

//single direction shift 
void shift_objects(vector<Object>& objects, double shift_dist, int flag);

double slope(vector<Object> surfaces);

bool compareInterval(Point i1, Point i2);
#endif