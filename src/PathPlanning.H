
#ifndef _PATHPLANNING_H_
#define _PATHPLANNING_H_

#include <cmath>
#include <iostream>
#include <vector>
#include "Object.H"
#include "asr.H"
#include "Aria.h"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"


//#define M_PI 3.14159265358979323846

class Exit
{
private:
	double x1;
	double y1;
	double x2;
	double y2;
	int id; //exit id
	int p1id; //id of object containg first vertex of exit
	int p2id;//id of object containg second vertex of exit
	double angle; //angle btw mp and first point of exit direction(clearness)
	double p2dist;//dist btw p2 and robot position
        double orient; //orientation between exit and robot heading direction
        bool cross_flag;
        
        int surf_describ_flag;  //has been described using objects --- 1, not -- 0 
        
        //modified by Lyn
        vector<Object> exitFromChunk;
        vector<Object> exitToChunk;
        vector<Object> exit_surround;
public:	
	Exit(){};
	Exit(double a, double b,double c, double d);
	~Exit(){};

	double X1();
	double Y1();
	double X2();
	double Y2();

	void setID(int a);
	int getID();
	void set(double a, double b, double c, double d);
	void set(Object tmp);
	void set(Exit e);
	void display();
	void setP1ID(int a);
	int getP1ID();
	void setP2ID(int a);
	int getP2ID();
	void setP2DistFromRobot(double a);
	double getP2DistFromRobot();
	double length();
	double distP1ToPoint(double a, double b);
	double distP2ToPoint(double a, double b);
	double mpX();
	double mpY();
	void setAngle(double a);
	double getAngle();
        
        double midToPoint(double a, double b);
        double getOrietToRob(Object rb);
        void setCrossFlag(bool crossflag);
        bool getCrossFlag();
        
        void set_surf_describe(int flag);
        int get_surf_describe();
        
        Point getP1();
        Point getP2();
        Point getmidPoint();
        
        void set_exit_surround(vector<Object> region);
        vector<Object> get_exit_surround();
	
};
void displayExits(vector<Exit> exits);
bool sortExitsA2L(Exit d1, Exit d2);//sort exits according to length
bool sortExitsA2A(Exit e1, Exit e2); //sort exits according to angle tag
bool sortDistA2V(double a, double b);//

vector<double> exitsFromorign(vector<Exit> exitsFromCurrent);


class Destination {
private:
    double angle;
    double dist;
    int type; //1 means normal 2 means obs avoidance
    int attemptsToAvoidObstacles; //Num of trying done to avoid obstacle
    vector<Object> destinationExits;
    vector<pair<double,double> > otherExitLocations;
public:

    Destination() : attemptsToAvoidObstacles(0), angle(0), dist(0), destinationExits(0) {
    };
    Destination(double a, double b);

    ~Destination() {
    };

    void setAngle(double a);
    double getAngle();
    void setDist(double a);
    double getDist();
    void setType(int a);
    int getType();
    void display();
    
    void oneAttemptToAO();
    int getAttemptsToAO();
    void setAttemptsToAO(int attempts);
    void setDestinationExits(vector<Object> destExits);
    vector<Object> getDestinationExits();
    
    vector<pair<double,double> > getOtherExitLocations();
    

    void findNextDestination(vector<Object> currentView, vector<Object> referenceObjects, int viewNumber);
    void findNextStepLocation(vector<Object> currentView, vector<Object> referenceObjects, pair<double,double> goal, int viewNumber);
};


vector<Exit> findExits(vector<Object> cv);
vector<Exit> findShortestExits(vector<Object> cv);//modified version of findExits(ie modified version of yeap's theory)
vector<Object> findDestinationExits(vector<Object> cv, vector<Object> refObjects);//modified version of shortest Exits
vector<Object> findAllDestinationExits(vector<Object> cv, vector<Object> refObjects);//modified version of findDestinationExits
vector<Object> tranformCVExitsInMFIS(vector<Object> cvExits, vector<Object> refObjects);
vector<Exit> findGateways(vector<Object> cv);//hossain's gateway theorem
vector<Exit> findGatewaysNew(vector<Object> cv);
Destination findDestinationPoint(vector<Object> cv, vector<Exit> exits);
Destination findDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides);
Destination findNextDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides);
Destination findDestToMaxPath(vector<Object> cv, vector<Object> robsides, int viewno);
vector<double> findMovementDirection(vector<Object> pexits);//obsolete
vector<Object> exitsInMFIS(vector<Exit> exits,Object rmfis, Object rcv,int refpoint);
vector<Object> convertExitToObject(vector<Exit> exits);//convert exit to object type

//developed on Tue Oct 11, 2011
vector<Exit> findNewASRFormingExits(vector<Object> cv);

Destination avoidObstacle(double angleToDest,double distanceToDest, vector<Object> currentView, int attempts);
vector<pair<double, double> > findInvalidatedGoals(vector<Object> currentView, int goalType);
//Destination verifyDestination(vector<pair<double, double> > invalidatedGoals, vector<Object> currentView);
Destination verifyDestination(vector<pair<double, double> > invalidatedExitGoals, vector<pair<double, double> > invalidatedGoals, vector<Object> currentView);
Destination avoidObstacleUsingPathPolygon(double angleToDest,double distanceToDest,vector<Object> currentView);

vector<Object> findExitToReachGoal(vector<Object> currentView,pair<double,double> goal,vector<Object> referenceObjects, int viewNumber);

vector<Object> findGapsForGoalExit(vector<Object> currentView,vector<Object> referenceObjects, Object goalExit,int viewNumber);
pair<double,double> findCurrentGoal(vector<Object> goalGap);
bool isThePathClear(vector<Object> currentView, Object goalExit);
Object recognizeGoalExit(vector<Object> exits, Object goalExit);

vector<Exit> findShortestGap(vector<Object> cv);
vector<Point> findPathToReachGoal(vector<ASR> places) ;

Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno);
Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno);
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno,PointXY goal);
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno,PointXY goal);

Point crossedPointReturnP(Point p1, Point p2, Point q1, Point q2);
bool betweenForReturn(double a, double X0, double X1) ;
Object IntersectForReturn(vector<Object> CurrentV, Point Pos) ;
bool twoLinintersect(Point p1, Point p2, Point p3, Point p4);
bool TwoObjectIntersect(Object obj1, Object obj2);
bool interSectWithPtoOrig(vector<Object> CV, Point pos);

bool interSectWithLine(vector<Object> CV, Point p1, Point p2);
Point intersectedPoint(vector<Object> CV, Point p3, Point p4);
//vector<Point> intersectedPoints(vector<Object> view1, vector<Object> view2);
Point intersectedNearPoint(vector<Object> CV, Point p3, Point p4);
Object intersectForObject(vector<Object> CurrentV, Point p3, Point p4) ;
vector<Object> intersectAllSurfaces(vector<Object> CurrentV, Point p3, Point p4);
int Intersect_number(vector<Object> view, Object line);

double distanceOftwoP(Point a, Point b);
bool ObstacleFront(vector<Object> CurrentV, double distance);

bool comPathView(vector<int> Cu, vector<int> Me);
bool matchObjPos(vector<int> mem, vector<int> cur);
vector<Exit> findGapasExits(vector<Object> cv);  // find suitable exits


vector<Point> returningPositions(vector<Point> global, vector<double> facing);
double GetPointDistance(Point p1, Point p2);    
double GetNearestDistance(Point PA, Point PB, Point P3);

vector<Object> routeInView(vector<Object> CV, Point pos);

vector<Point> CollectPoints(vector<Object> view);
bool isObject(vector<Object> view, Point p1, Point p2);
vector< vector<Point> > GroupByPoint(vector<Object> view);
vector< vector<Point> > GroupByPoint_version2(vector<Object> view);

//vector<Object> BoundaryByGroup(vector< vector<Point> > GroupPoints);
vector<Object> BoundaryByGroup(vector<Object> view);

vector<Object> BoundaryByExits(vector<Object> CurrentView); // find exit and rebuild this view and return it
vector<Object> pathView(vector<Object> view, int relat_flag);
vector<int> ObjectPosition(vector<Object> pathView); 
int SingleObjectPosition(Object pathView);
pair<int, int> exitFromMem(vector<Object> Mem, Point des, int relat_flag);
vector<Object> exitOfObjects(vector<Object> CurPathView, int first, int second);

pair<double,double> DistAndAngle(Point Pos);

int isLeft(Point p0, Point p1, Point p2); // is to judge whether the point is on left or right.
double AngleOfNext(Point p1, Point p2, Point p3);
vector<Point> referenceBoundary(vector<Object> boundaryView);//choose reference boundary
double layoutRelation(Point ref1P1, Point ref1P2, Point ref2P1, Point ref2P2);
double ReferenceLength(Point refP1, Point refP2); 
double perpendicularDis(Point ref1P1, Point ref1P2, Point ref2P1);
double Perpendiculardistance(Point ref1P1, Point ref1P2, Point ref2P1) ;
   
bool Recognision(vector<Object> CurrentView, vector<Object> MemoryView);
bool RecognisionBasedPerpend(vector<Object> CurrentView, vector<Object> MemoryView);
vector<Object> recognisedObject(vector<Object> CurrentView, vector<Object> MemoryView);
Point crossPerpend(Point ref1P1, Point ref1P2, Point ref2P1);

vector<Point> recognisedLRObject(vector<Object> CurrentView, vector<Object> MemoryView);
vector<Object> getrecognisedLRObject(vector<Object> CurrentView, vector<Object> MemoryView);

vector<Point> SearchingReference(vector<Object> Memory, int flag);
Object SearchBoundary(vector<Object> CV, double midDist, int relat_flag);

double getAngleofOneObject(Object a);
double getAngleofTwoObjects(Object a, Object b);
double IncludedAngleOfTwoObjects(Object a, Object b);

vector<double> getAngleAndDistance(vector<Object> Memory, Point postion);
Point getNextPosition(vector<Object> Memory, Point postion, Object curRefObject);
bool isExit(vector<Object> CurrentView, int relat_flag);
vector<double> distancesToNext(vector<Object> Mry, Point aim);
double midPointOfMemo(vector<Object> Mry, Point aim);
double midPointOfObj(Object Obj);
void ExecutionAndGo(ArRobot&, double angle, double dist);
void avoidConvex(ArRobot& robot, vector<Object> CurrentView);
Point farthestPoint(vector<Object> CurrentView, double relat_flag);
Object farthestObject(vector<Object> CurrentView, Point rb);
Point avoidObstracle(vector<Object> CurrentView, Point far);

void PathPlanner(ArRobot& robot, ArSick& sick, vector<Point> Points, Point aim, int relat_flag);
void PathStraight(ArRobot& robot, ArSick& sick, vector<Object> CV, double dist, int relat_flag);
void PathRecognistion(ArRobot& robot, ArSick& sick, vector<Object> CV, vector<double> info, double midDist, int relat_flag);
void PathViewRecognistion(ArRobot& robot, ArSick& sick, vector<Object> MpathView, vector<Object> CpathView,
                                                                            vector<double> info, Point pos, int numb, int relat_flag);

void pathPlanBoundandView(ArRobot& robot, ArSick& sick, vector<Object> mBView, vector<Object> cBView,
                                                                            vector<double> info, Point pos, double midDist, int numb, int relat_flag);

void returnPathPlanOrignalVersion(ArRobot& robot, ArSick& sick, Point pos, int num, int relat_flag);
void returnPathPlan(ArRobot& robot, ArSick& sick, Point pos, int num, int relat_flag);

vector<Object> AreaOfSpatial(vector<Object> CV, Point pos);
vector<Position> ObjectsPosition(vector<Object> CV, Point pos);
vector<Position> matchMemory(vector<Position> mem, vector<Object> CV, int num);

long getShortestDistance(Object a, Object b);
vector<Position> GroupMemory(vector<Object> view, Point pos); 
vector<Position> MemoryProces(int level, int set, int num, Point pos);
void pathAndOrientation(ArRobot& robot,ArSick& sick, vector<Object> CV, Point pos, vector<double> path, vector<Position> memPos);

void CalculatePath(vector<Object> CV, vector<Position> cur, vector<Object> path, int num);

vector<Object> Plan_return_path(vector<Object> current_mfis, vector<Object> old_mfis_inside, 
                      vector<Object> end_position, vector<Object> path_in_old_mfis);

#endif