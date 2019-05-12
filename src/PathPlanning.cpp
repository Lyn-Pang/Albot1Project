#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
//#include <mrpt/base/include/Eigen/src/plugins/BlockMethods.h>  // Needed for sort() method
#include "PathPlanning.H"
#include "Object.H"
#include "GeometricOp.H"
#include "mfisOp.H"
#include "Plotting.H"
#include "CompareASR.H"
#include "readAndwriteASCII.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"
#include "Aria.h"
#include "Laser2Surface.H"
#include "RobotFuncs.H"

#define PI 3.14159265

int number = 200;//for pathplanning fucntion

Exit::Exit(double a, double b,double c, double d) {
	x1=a;	y1=b;
	x2=c;	y2=d;
}
double Exit::X1(){return x1;}
double Exit::Y1(){return y1;}
double Exit::X2(){return x2;}
double Exit::Y2(){return y2;}

void Exit::setID(int a) {
	id=a;
}
int Exit::getID() {
	return id;
}
void Exit::set(double a, double b, double c, double d) {
	x1=a;	y1=b;
	x2=c;	y2=d;
}
void Exit::set(Exit e) {
	x1=e.X1();	y1=e.Y1();
	x2=e.X2();	y2=e.Y2();
	id=e.getID();
	p1id=e.getP1ID();	p2id=e.getP2ID();
	angle=e.getAngle();
	p2dist=e.getP2DistFromRobot();
}
void Exit::set(Object tmp) {
	x1=tmp.X1();	y1=tmp.Y1();
	x2=tmp.X2();	y2=tmp.Y2();
}
void Exit::display() {
	cout<<"ID: "<<id<<" X1: "<<x1<<" Y1: "<<y1<<" ------ X2: "<<x2<<" Y2: "<<y2<<" p1id: "<<p1id<<" p2id: "<<p2id<<" p2dist "<<p2dist<<" length "<<length()<<" angle: "<<angle<<endl;
}
void Exit::setP1ID(int a) {
	p1id=a;
}
int Exit::getP1ID() {
	return p1id;
}
void Exit::setP2ID(int a) {
	p2id=a;
}
int Exit::getP2ID() {
	return p2id;
}
void Exit::setP2DistFromRobot(double a) {
	p2dist=a;
}
double Exit::getP2DistFromRobot() {
	return p2dist;
}

double Exit::length() {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double Exit::distP1ToPoint(double a, double b) {
	return sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));
}

double Exit::distP2ToPoint(double a, double b) {
	return sqrt((x2-a)*(x2-a)+(y2-b)*(y2-b));
}

double Exit::mpX() {
	return (x2+x1)/2;
}
double Exit::mpY() {
	return (y2+y1)/2;
}
void Exit::setAngle(double a) {
	angle=a;
}
double Exit::getAngle() {
	return angle;
}

double Exit::midToPoint(double a, double b)
{
        return sqrt(((x2+x1)/2-a)*((x2+x1)/2-a)+((y2+y1)/2-b)*((y2+y1)/2-b));
}

double Exit::getOrietToRob(Object rb)
{
    double Angle, a, b, c;
    Point mid;
    mid.set((x1 + x2) / 2, (y1 + y2) /2);
    a = sqrt((mid.X() - rb.X2())*(mid.X() - rb.X2())+(mid.Y() - rb.Y2())*(mid.Y() - rb.Y2()));
    b = sqrt((rb.X2() - rb.X1())*(rb.X2() - rb.X1())+(rb.Y2() - rb.Y1())*(rb.Y2() - rb.Y1()));
    c = sqrt((mid.X() - rb.X1())*(mid.X() - rb.X1())+(mid.Y() - rb.Y1())*(mid.Y() - rb.Y1()));
    
    Angle = 180/PI * acos((pow(b,2) + pow(c, 2) - pow(a, 2)) / 2*b*c);
    if(isLeft(Point(rb.X1(), rb.Y1()), Point(rb.X2(), rb.Y2()), mid) < 0) //left or right
        Angle = -Angle;
    orient = Angle;
    
    return orient;
}

void Exit::setCrossFlag(bool crossflag)
{
    cross_flag = crossflag;
}
bool Exit::getCrossFlag()
{
    return cross_flag;
}

void Exit::set_surf_describe(int flag)
{
    surf_describ_flag = flag;
}

int Exit::get_surf_describe()
{
    return surf_describ_flag;
}

Point Exit::getP1()
{
    Point _p1;
    _p1.set(x1, y1);
    return _p1;
}
Point Exit::getP2()
{
    Point _p2;
    _p2.set(x2, y2);
    return _p2;
}

Point Exit::getmidPoint()
{
    Point mid;
    mid.set((x1+x2)/2, (y1+y2)/2);
    return mid;
}

void Exit::set_exit_surround(vector<Object> region)
{
    
    vector<Object> surrounding;
    
    for(int i = 0; i < region.size(); i++)
    {
        if((distanceOftwoP(this->getP1(), region[i].getP1()) < this->length()/2)
            ||(distanceOftwoP(this->getP1(), region[i].getP2()) < this->length()/2)
            ||(distanceOftwoP(this->getP2(), region[i].getP1()) < this->length()/2)
            ||(distanceOftwoP(this->getP2(), region[i].getP2()) < this->length()/2))
            surrounding.push_back(region[i]);
    }
    
    exit_surround = surrounding;
}
vector<Object> Exit::get_exit_surround()
{
    return exit_surround;
}


vector<double> exitsFromorign(vector<Exit> exitsFromCurrent)
{
    vector<double> distancesToexits;
    
    for(int i = 0; i < exitsFromCurrent.size(); i++)
        distancesToexits.push_back(exitsFromCurrent[i].midToPoint(0,0));

    
    return distancesToexits;
}

void displayExits(vector<Exit> exits) {
	//cout<<"no of exits "<<exits.size()<<endl;
	for(int i=0;i<int(exits.size());i++) {
		exits[i].display();
	}
}
bool sortExitsA2L(Exit d1, Exit d2)
{
  return d1.length() > d2.length();
}
bool sortExitsA2A(Exit e1, Exit e2) {
	return e1.getAngle() > e2.getAngle();
}

bool sortDistA2V(double a, double b) {
	return a < b;
}

//****************************************Destination class
Destination::Destination(double a, double b) {
	angle = a;
	dist = b;
}

void Destination::setAngle(double a) {
	angle = a;
}
double Destination::getAngle() {
	return angle;
}
void Destination::setDist(double a) {
	dist=a;
}
double Destination::getDist() {
	return dist;
}
void Destination::setType(int a) {
	type=a;
}
int Destination::getType() {
	return type;
}
void Destination::display() {
	cout<<"Angle: "<<angle<<" Dist: "<<dist<<" type: "<<type<<endl;
}

void Destination::oneAttemptToAO() {
    attemptsToAvoidObstacles++;
}
//it returns how many attempts taken to avoid obstacles 
int Destination::getAttemptsToAO() {
    return attemptsToAvoidObstacles;
}
void Destination::setAttemptsToAO(int attempts) {
    attemptsToAvoidObstacles = attempts;
}

void Destination::setDestinationExits(vector<Object> destExits) {
    destinationExits = destExits;
}
vector<Object> Destination::getDestinationExits() {
    return destinationExits;
}
vector<pair<double,double> > Destination::getOtherExitLocations() {
    return otherExitLocations;
}

void Destination::findNextDestination(vector<Object> currentView, vector<Object> referenceObjects, int viewNumber) {
    cout << "\n\033[1;34m               Looking for NextDestination             \033[0m" << endl << endl;

    //calculating direction to max path
    
    Object robotPose(0, 0, 0, 500, 1);   
    char viewFileName[100];
    //finding exit goals
    vector<pair<double,double> > invalidatedExitGoals;
//    cout<<"destination exits: "<<destinationExits.size()<<endl;
//    if(destinationExits.size() == 0 && viewNumber > 15)
    vector<Object> destinationExitsInCV = findDestinationExits(currentView,referenceObjects);
   
//    cout<<"Destination Exits in CV: "<<destinationExitsInCV.size()<<endl;
    if(destinationExitsInCV.size() > 0) {
        cout<<"Destination Exits found "<<destinationExitsInCV.size()<<endl;
        displayObjects(destinationExitsInCV);
//        currentView[10].display();
//        vector<Object> destinationExitsInCV;
//        for(unsigned int i =0;i<destinationExits.size();i++) {
//            destinationExitsInCV.push_back(remakeLineP2(referenceObjects[1],referenceObjects[0],destinationExits[i],1,0,referenceObjects[0].getKP()));
//        }
       
        invalidatedExitGoals = findInvalidatedGoals(destinationExitsInCV,1);//1 bcz these are exit goals
        destinationExitsInCV.push_back(robotPose);
        sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/DestinationExits-", viewNumber, ".png");
        plotObjects(viewFileName,destinationExitsInCV,currentView);
    }
//     if(viewNumber > 12) {
//    vector<Exit> yeapsExits = findExits(currentView);
//    plotObjects("MFIS/yeapsExits.png",convertExitToObject(yeapsExits),currentView);
////    waitHere();
//    }
//    if(viewNumber > 15)
//    waitHere();
    
//    cout<<"Invalidated Goal Angle: "<<angleToMaxPath<<" distance: "<<distanceAlongMaxPath<<endl;  
    
    vector<pair<double,double> > invalidatedGoals = findInvalidatedGoals(currentView,2);//2 bcz these are farthest path goals
    
    for(unsigned int i =0;i<invalidatedGoals.size();i++){
        cout<<"(From NewFunction) Angle: "<<invalidatedGoals[i].first<<" distance: "<<invalidatedGoals[i].second<<endl;
    }
//    waitHere();
    
    //only for printing
    
    double angleToMaxPath, distanceAlongMaxPath;
    if(invalidatedExitGoals.size() > 0) {
        angleToMaxPath = invalidatedExitGoals[0].first;
        distanceAlongMaxPath = invalidatedExitGoals[0].second;
    } else {
        angleToMaxPath = invalidatedGoals[0].first;
        distanceAlongMaxPath = invalidatedGoals[0].second;
    }
    
    PointXY tentativeGoal(getX(angleToMaxPath,distanceAlongMaxPath),getY(angleToMaxPath,distanceAlongMaxPath));
    Object tentativeRobotPose(tentativeGoal.getX(),tentativeGoal.getY(),getX(angleToMaxPath,distanceAlongMaxPath+500),getY(angleToMaxPath,distanceAlongMaxPath+500));
    MyRobot myrobot(0, 0);
//    vector<Object> tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    vector<Object> robotWholePath = myrobot.robotPathForNextDest(angleToMaxPath,distanceAlongMaxPath); 
    
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/BeforeAvoiding-", viewNumber, ".png");
    if(invalidatedExitGoals.size() > 0)
        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
//    if(viewNumber == 42)
    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
    
//    cout<<"Test Angle: "<<robotPose.getAngleWithLine(Object(0,0,currentView.back().X1(),currentView.back().Y1(),1))<<endl;


    Destination verifiedDestination;
//    verifiedDestination = avoidObstacle(invalidatedGoals,currentView,1);
    verifiedDestination = verifyDestination(invalidatedExitGoals,invalidatedGoals,currentView);
    angle = verifiedDestination.getAngle();
    dist = verifiedDestination.getDist();      
    //storing all exits for future 
    for(unsigned int i = 0; i<invalidatedExitGoals.size();i++) {
        otherExitLocations.push_back(invalidatedExitGoals[i]);
    }
    cout<<"Verified Destination: "<<angle<<" dist "<<dist<<endl;
   
    
    //just for printing
    PointXY newGoal(getX(angle,dist),getY(angle,dist));
    tentativeRobotPose.set(newGoal.getX(),newGoal.getY(),getX(angle,dist+500),getY(angle,dist+500),1);
//    tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    robotWholePath = myrobot.robotPathForNextDest(angle,dist);  
    if(invalidatedExitGoals.size() > 0)
        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/AfterAvoiding-", viewNumber,".png");
//    if(viewNumber == 42)
    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
}

//created on 03-07-12
void Destination::findNextStepLocation(vector<Object> currentView, vector<Object> referenceObjects, pair<double,double> goal, int viewNumber) {
    cout << "\n\033[1;34m               Looking for NextDestination             \033[0m" << endl << endl;

    //calculating direction to max path
    
    Object robotPose(0, 0, 0, 500, 1);   
    char viewFileName[100];
    //finding exit goals
    vector<pair<double,double> > invalidatedExitGoals, invalidatedGoals;
//    cout<<"destination exits: "<<destinationExits.size()<<endl;
//    if(destinationExits.size() == 0 && viewNumber > 15)
//    vector<Object> destinationExitsInCV = findDestinationExits(currentView,referenceObjects);
//   
////    cout<<"Destination Exits in CV: "<<destinationExitsInCV.size()<<endl;
//    if(destinationExitsInCV.size() > 0) {
//        cout<<"Destination Exits found "<<destinationExitsInCV.size()<<endl;
//        displayObjects(destinationExitsInCV);
////        currentView[10].display();
////        vector<Object> destinationExitsInCV;
////        for(unsigned int i =0;i<destinationExits.size();i++) {
////            destinationExitsInCV.push_back(remakeLineP2(referenceObjects[1],referenceObjects[0],destinationExits[i],1,0,referenceObjects[0].getKP()));
////        }
//       
//        invalidatedExitGoals = findInvalidatedGoals(destinationExitsInCV,1);//1 bcz these are exit goals
//        destinationExitsInCV.push_back(robotPose);
//        sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/DestinationExits-", viewNumber, ".png");
////        plotObjects(viewFileName,destinationExitsInCV,currentView);
//    }
//     if(viewNumber > 12) {
//    vector<Exit> yeapsExits = findExits(currentView);
//    plotObjects("MFIS/yeapsExits.png",convertExitToObject(yeapsExits),currentView);
////    waitHere();
//    }
//    if(viewNumber > 15)
//    waitHere();
    
//    cout<<"Invalidated Goal Angle: "<<angleToMaxPath<<" distance: "<<distanceAlongMaxPath<<endl;  
    
//    vector<pair<double,double> > invalidatedGoals = findInvalidatedGoals(currentView,2);//2 bcz these are farthest path goals
//    
//    for(unsigned int i =0;i<invalidatedGoals.size();i++){
//        cout<<"(From NewFunction) Angle: "<<invalidatedGoals[i].first<<" distance: "<<invalidatedGoals[i].second<<endl;
//    }
//    waitHere();
    
    //only for printing
   
    invalidatedExitGoals.push_back(goal);
    double angleToMaxPath, distanceAlongMaxPath;
    if(invalidatedExitGoals.size() > 0) {
        angleToMaxPath = invalidatedExitGoals[0].first;
        distanceAlongMaxPath = invalidatedExitGoals[0].second;
    } else {
        angleToMaxPath = invalidatedGoals[0].first;
        distanceAlongMaxPath = invalidatedGoals[0].second;
    }
    
    PointXY tentativeGoal(getX(angleToMaxPath,distanceAlongMaxPath),getY(angleToMaxPath,distanceAlongMaxPath));
    Object tentativeRobotPose(tentativeGoal.getX(),tentativeGoal.getY(),getX(angleToMaxPath,distanceAlongMaxPath+500),getY(angleToMaxPath,distanceAlongMaxPath+500));
    MyRobot myrobot(0, 0);
//    vector<Object> tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    vector<Object> robotWholePath = myrobot.robotPathForNextDest(angleToMaxPath,distanceAlongMaxPath); 
    
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/BeforeAvoiding-", viewNumber, ".png");
//    if(invalidatedExitGoals.size() > 0)
//        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
//    if(viewNumber == 42)
//    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
    plotObjects(viewFileName,robotWholePath,currentView);
    
//    cout<<"Test Angle: "<<robotPose.getAngleWithLine(Object(0,0,currentView.back().X1(),currentView.back().Y1(),1))<<endl;


    Destination verifiedDestination;
//    verifiedDestination = avoidObstacle(invalidatedGoals,currentView,1);
    verifiedDestination = verifyDestination(invalidatedExitGoals,invalidatedGoals,currentView);
    angle = verifiedDestination.getAngle();
    dist = verifiedDestination.getDist();      
    //storing all exits for future 
    for(unsigned int i = 0; i<invalidatedExitGoals.size();i++) {
        otherExitLocations.push_back(invalidatedExitGoals[i]);
    }
    cout<<"Verified Destination: "<<angle<<" dist "<<dist<<endl;
   
    
    //just for printing
    PointXY newGoal(getX(angle,dist),getY(angle,dist));
    tentativeRobotPose.set(newGoal.getX(),newGoal.getY(),getX(angle,dist+500),getY(angle,dist+500),1);
//    tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    robotWholePath = myrobot.robotPathForNextDest(angle,dist);  
//    if(invalidatedExitGoals.size() > 0)
//        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/AfterAvoiding-", viewNumber,".png");
//    if(viewNumber == 42)
//    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
    plotObjects(viewFileName,robotWholePath,currentView);
}


Destination verifyDestination(vector<pair<double, double> > invalidatedExitGoals, vector<pair<double, double> > invalidatedGoals, vector<Object> currentView) {
    cout << "\n\033[1;34m               verifying NextDestination             \033[0m" << endl << endl;
    int attempts = 0;
    Destination verifiedDestination;
    double worstCaseAngle = 0;

    if (invalidatedExitGoals.size() > 0) {
//        if (abs(invalidatedExitGoals[0].first) > 60) {//saving the first target for worst case to find free path
//            if (invalidatedExitGoals[0].first > 0)
//                worstCaseAngle = 60;
//            else
//                worstCaseAngle = -60;
//        } else if (abs(invalidatedExitGoals[0].first) < 60 && abs(invalidatedExitGoals[0].first) > 25) {
//            worstCaseAngle = invalidatedExitGoals[0].first;
//        } 
        if(abs(invalidatedExitGoals[0].first) > 25)
            worstCaseAngle = invalidatedExitGoals[0].first;
        else {
            if (invalidatedExitGoals[0].first > 0)
                worstCaseAngle = 25;
            else
                worstCaseAngle = -25;
        }
    }else  if (invalidatedGoals.size() > 0) {
//        if (abs(invalidatedGoals[0].first) > 60) {//saving the first target for worst case to find free path
//            if (invalidatedGoals[0].first > 0)
//                worstCaseAngle = 60;
//            else
//                worstCaseAngle = -60;
//        } else if (abs(invalidatedGoals[0].first) < 60 && abs(invalidatedGoals[0].first) > 25) {
//            worstCaseAngle = invalidatedGoals[0].first;
//        } 
        if (abs(invalidatedGoals[0].first) > 25)
            worstCaseAngle = invalidatedGoals[0].first;
        else {
            if (invalidatedGoals[0].first > 0)
                worstCaseAngle = 25;
            else
                worstCaseAngle = -25;
        }
    }

    if (invalidatedExitGoals.size() > 0) {
        for (unsigned int i = 0; i < invalidatedExitGoals.size(); i++) {
            cout << endl << "(Exit) Going to check invalidated goal no. " << i + 1 << endl;
            do {
                attempts++;
                cout << "Attempts Number " << attempts << endl;
                //        verifiedDestination = avoidObstacle(invalidatedGoals[0].first, invalidatedGoals[0].second, currentView, 1);
                cout << "Invalidated Exit Goal Angle:  " << invalidatedExitGoals[i].first << " distance: " << invalidatedExitGoals[i].second << endl;
//                if (abs(invalidatedExitGoals[i].first) > 60) {//to limit the turn angle
//                    if (invalidatedExitGoals[i].first > 0)
//                        invalidatedExitGoals[i].first = 60;
//                    else
//                        invalidatedExitGoals[i].first = -60;
//                }
                //        waitHere();
                verifiedDestination = avoidObstacleUsingPathPolygon(invalidatedExitGoals[i].first, invalidatedExitGoals[i].second, currentView);
                if (verifiedDestination.getType() == 2) {//means got obstacle avoided Destination
                    invalidatedExitGoals[i].first = verifiedDestination.getAngle();
                    invalidatedExitGoals[i].second = verifiedDestination.getDist();
//                    cout<<"once verified angle "<<verifiedDestination.getAngle()<<" dist "<<verifiedDestination.getDist()<<endl;
                    //attempts++;
                }
                if (verifiedDestination.getType() == 1) {
                    //                waitHere();
                    cout<<"Exit Goal is reachable:)"<<endl;
                    return verifiedDestination; //means obstacle free goal. verified.
                }
                if (verifiedDestination.getType() == 3) {
                    break; //too narrow path.not possible to go through 
                }
            } while (attempts < 3);

            attempts = 0;
        }
    }
    
    //here means there is no exit goal or exit goals are not reachable 
    if (invalidatedGoals.size() > 0) {//no else it will be only if condition

        for (unsigned int i = 0; i < invalidatedGoals.size(); i++) {
            cout << endl << "(searching) Going to check next invalidated goal no. " << i + 1 << endl;
            do {
                attempts++;
                cout << "Attempts Number " << attempts << endl;
                //        verifiedDestination = avoidObstacle(invalidatedGoals[0].first, invalidatedGoals[0].second, currentView, 1);
                cout << "Invalidated Goal Angle:  " << invalidatedGoals[i].first << " distance: " << invalidatedGoals[i].second << endl;
//                if (abs(invalidatedGoals[i].first) > 60) {//to limit the turn angle
//                    if (invalidatedGoals[i].first > 0)
//                        invalidatedGoals[i].first = 60;
//                    else
//                        invalidatedGoals[i].first = -60;
//                }
                //        waitHere();
                verifiedDestination = avoidObstacleUsingPathPolygon(invalidatedGoals[i].first, invalidatedGoals[i].second, currentView);
                if (verifiedDestination.getType() == 2) {//means got obstacle avoided Destination
                    invalidatedGoals[i].first = verifiedDestination.getAngle();
                    invalidatedGoals[i].second = verifiedDestination.getDist();
//                    cout<<"once verified angle "<<verifiedDestination.getAngle()<<" dist "<<verifiedDestination.getDist()<<endl;
                    //attempts++;
                }
                if (verifiedDestination.getType() == 1) {
                    //                waitHere();
                    return verifiedDestination; //means obstacle free goal. verified.
                }
                if (verifiedDestination.getType() == 3) {
                    break; //too narrow path.not possible to go through 
                }
            } while (attempts < 3);

            attempts = 0;
        }
    }

    //here means all type of goals are failed 
    //now if there was any exit goal then take a turn towards that or just take a turn of 30 deg to find other goals
//    cout<<"No invalidated goal left. so take a turn to go back from this deadend:)"<<endl;
    if(worstCaseAngle == 0) {
        cout<<"I couldn't find any particular goal..so taking an uncertain turn to find path:))"<<endl;
        worstCaseAngle = 0;
    }
    else
        cout<<"No reachable goals...so taking a turn towards exit/searchPath...."<<endl;
    verifiedDestination.setAngle(0);//just take a turn from this deadEnd
    verifiedDestination.setDist(0);
    cout<<"WorstCaseAngle: "<<worstCaseAngle<<endl;
//    waitHere();
    return verifiedDestination;
}

Destination avoidObstacleUsingPathPolygon(double angleToDest,double distanceToDest,vector<Object> currentView){
    cout << "\n\033[1;34m               avoiding obstacle for NextDestination using whole path polygon            \033[0m" << endl << endl;
    Object currentRobotPose(0, 0, 0, 500, 1);
    PointXY tentativeGoal(getX(angleToDest, distanceToDest), getY(angleToDest, distanceToDest));
    Object tentativeRobotPose(tentativeGoal.getX(), tentativeGoal.getY(), getX(angleToDest, distanceToDest + 500), getY(angleToDest, distanceToDest + 500));
       
    //check whether robot is going to cross an exit
    //if robot comes too close to exit then from CV it can't see exit bcz of robot direction
    //may be have to use exit info from previous view as well.
//    vector<Exit> exitsFromCV = findShortestExits(currentView);
//    if (exitsFromCV.size() > 0) {
//        vector<Object> exitsAsObjects = convertExitToObject(exitsFromCV);
//        Object nextRobotPath(0, 0, tentativeGoal.getX(), tentativeGoal.getY(), 1);
//        double crossedExitIndex = splitToFromNewASR(exitsAsObjects, nextRobotPath);
//        if ( crossedExitIndex > -1) {
//            cout << "Going to cross a exit!!!" << endl;
//            nextRobotPath.setP2(exitsAsObjects[crossedExitIndex].mpX(),exitsAsObjects[crossedExitIndex].mpY());
//            double angle = currentRobotPose.getAngleWithLine(nextRobotPath);
//            double distance = nextRobotPath.length()-200;//if will add (100/200) with length then robot collide at back for small exit
//            Destination verifiedDestination;
//            verifiedDestination.setAngle(angle);
//            verifiedDestination.setDist(distance);
//            verifiedDestination.setType(1);//exit destination
//            cout<<"Destination Angle: "<<angle<<" distance: "<<distance<<endl;
////            waitHere();
//            return verifiedDestination;          
//        }
//    }
    
    MyRobot myrobot(0, 0);
//    vector<Object> tentativeRobot = myrobot.inMFIS(tentativeRobotPose, currentRobotPose, 1);//robot at next Destination
    vector<Object> robotWholePath = myrobot.robotPathForNextDest(angleToDest,distanceToDest);
    
    vector<Surface> robotPolygon(4);
    for (unsigned i = 0; i < 4; i++) {
        robotPolygon[i] = Surface(PointXY(robotWholePath[i].X1(), robotWholePath[i].Y1()), PointXY(robotWholePath[i].X2(), robotWholePath[i].Y2()), true);
    }
    
    Destination obsAvoidedDestination;
    obsAvoidedDestination.setType(1);//Destination is free to go if not then it will be changed later
    
    //looking for obstacles in the whole path
    Point obstacleAvoidEnd;
    bool isP1Inside, isP2Inside;
    bool needToTurnLeft = false;
    bool needToTurnRight = false;
    Object objectOriginToObstacleEnd;
    double angleToObstacleEnd;
    double p1PerpendicularDist, p2PerpendicularDist;
    vector<double> obstacleAndFSXPoint;
    double angleToAvoidForThisObs;
    double shiftAngle = 0;//when there is no obstacle then this 0 value is necessary bcz at the end it will be subtract from angleToDest
    double reduceDistance = 0;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        isP1Inside = pointInPolygon(PointXY(currentView[i].X1(), currentView[i].Y1()), robotPolygon, true);
        isP2Inside = pointInPolygon(PointXY(currentView[i].X2(), currentView[i].Y2()), robotPolygon, true);
        if(isP1Inside == true && isP2Inside == false) { //Obstacle p1 inside
            objectOriginToObstacleEnd.set(0,0,currentView[i].X1(),currentView[i].Y1(),1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            cout<<"P1 inside... angleToObstacleEnd: "<<angleToObstacleEnd<<endl;
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                needToTurnRight = true;
            } else {
                needToTurnLeft = true;
            }           
            obstacleAvoidEnd.set(currentView[i].X1(),currentView[i].Y1());
        } else if(isP1Inside == false && isP2Inside == true) {//Obstacle p2 inside
            objectOriginToObstacleEnd.set(0,0,currentView[i].X2(),currentView[i].Y2(),1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            cout<<"P2 inside... angleToObstacleEnd: "<<angleToObstacleEnd<<endl;
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                needToTurnRight = true;
            } else {
                needToTurnLeft = true;
            }  
            obstacleAvoidEnd.set(currentView[i].X2(),currentView[i].Y2());
        } else if(isP1Inside == true && isP2Inside == true) {//Obstacle's both ends inside
            objectOriginToObstacleEnd.set(0,0,currentView[i].mpX(),currentView[i].mpY(),1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                needToTurnRight = true;
            } else {
                needToTurnLeft = true;
            }  
            
            Object objectOriginToNextRobotPos(0, 0, tentativeGoal.getX(), tentativeGoal.getY(), 1);
            p1PerpendicularDist = objectOriginToNextRobotPos.perpendicularDistOfPoint(currentView[i].X1(),currentView[i].Y1());
            p2PerpendicularDist = objectOriginToNextRobotPos.perpendicularDistOfPoint(currentView[i].X2(),currentView[i].Y2());
            if(p1PerpendicularDist < p2PerpendicularDist) {
                obstacleAvoidEnd.set(currentView[i].X1(),currentView[i].Y1());
            } else
                obstacleAvoidEnd.set(currentView[i].X2(),currentView[i].Y2());
        } else if(checkForIntersection(currentView[i], robotWholePath[1]) == 1) {//both ends are outside and crossed with front side
            obstacleAndFSXPoint = getIntersectionPoint(robotWholePath[1],currentView[i]);
            objectOriginToObstacleEnd.set(0,0,obstacleAndFSXPoint[0],obstacleAndFSXPoint[1],1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            cout<<"Xwith FS...angleToObstacleEnd: "<<angleToObstacleEnd<<endl;
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                reduceDistance = robotWholePath[0].length() - robotWholePath[0].distP1ToPoint(obstacleAndFSXPoint[0],obstacleAndFSXPoint[1]);
                needToTurnRight = true;
            } else {
                reduceDistance = robotWholePath[1].length() - robotWholePath[1].distP1ToPoint(obstacleAndFSXPoint[0],obstacleAndFSXPoint[1]);
                needToTurnLeft = true;
            }  
            obstacleAvoidEnd.set(obstacleAndFSXPoint[0],obstacleAndFSXPoint[1]);
        }
        
        
        if(needToTurnRight == true && needToTurnLeft == true) {
            cout<<"Path is so narrow.....not possible to go through..."<<endl;
            obsAvoidedDestination.setType(3);//means need to change path direction
            return obsAvoidedDestination;
        }else if(needToTurnRight == true) {//amount of turn angle
            Object objectSideEndToObsEnd(robotWholePath[0].X1(),robotWholePath[0].Y1(),obstacleAvoidEnd.X(),obstacleAvoidEnd.Y(),1);
            angleToAvoidForThisObs = robotWholePath[0].getAngleWithLine(objectSideEndToObsEnd);
            if(abs(angleToAvoidForThisObs) > abs(shiftAngle))//to find max shift angle in case of multiple obstacles
                shiftAngle = angleToAvoidForThisObs;
            obsAvoidedDestination.setType(2);//obstacle avoided destination
        }else if(needToTurnLeft == true) {//amount of turn angle
//            cout<<"Need to turn left: "<<endl;
            Object objectSideEndToObsEnd(robotWholePath[2].X1(),robotWholePath[2].Y1(),obstacleAvoidEnd.X(),obstacleAvoidEnd.Y(),1);
            angleToAvoidForThisObs = robotWholePath[2].getAngleWithLine(objectSideEndToObsEnd);
            if(abs(angleToAvoidForThisObs) > abs(shiftAngle))//to find max shift angle in case of multiple obstacles
                shiftAngle = angleToAvoidForThisObs;
            obsAvoidedDestination.setType(2);//obstacle avoided destination
        }
    }    
    cout<<"Shift angle "<<shiftAngle<<endl;
    obsAvoidedDestination.setAngle(angleToDest+shiftAngle);
    obsAvoidedDestination.setDist(distanceToDest-reduceDistance);
    return obsAvoidedDestination;
}

Destination avoidObstacle(double angleToDest,double distanceToDest,vector<Object> currentView,int attempts) {
    cout << "\n\033[1;34m               avoiding for NextDestination             \033[0m" << endl << endl;
    Object currentRobotPose(0, 0, 0, 500, 1);
    PointXY tentativeGoal(getX(angleToDest, distanceToDest), getY(angleToDest, distanceToDest));
    Object tentativeRobotPose(tentativeGoal.getX(), tentativeGoal.getY(), getX(angleToDest, distanceToDest + 500), getY(angleToDest, distanceToDest + 500));

    //check whether robot is going to cross an exit
    vector<Exit> exitsFromCV = findShortestExits(currentView);
//    cout << "No of shortest exits: " << exitsFromCV.size() << endl;
    if (exitsFromCV.size() > 0) {
        vector<Object> exitsAsObjects = convertExitToObject(exitsFromCV);
        Object nextRobotPath(0, 0, tentativeGoal.getX(), tentativeGoal.getY(), 1);
        double crossedExitIndex = splitToFromNewASR(exitsAsObjects, nextRobotPath);
        if ( crossedExitIndex > -1) {
            cout << "Going to cross a exit!!!" << endl;
            nextRobotPath.setP2(exitsAsObjects[crossedExitIndex].mpX(),exitsAsObjects[crossedExitIndex].mpY());
            double angle = currentRobotPose.getAngleWithLine(nextRobotPath);
            double distance = nextRobotPath.length()+100;
            Destination verifiedDestination;
            verifiedDestination.setAngle(angle);
            verifiedDestination.setDist(distance);
            cout<<"Destination Angle: "<<angle<<" distance: "<<distance<<endl;
            return verifiedDestination;          
        }
    }
    
    
    MyRobot myrobot(0, 0);
    vector<Object> tentativeRobot = myrobot.inMFIS(tentativeRobotPose, currentRobotPose, 1);

//    cout << "Angle btw currentRP and robot's left side " << currentRobotPose.getAngleWithLine(tentativeRobot[0]) << endl;

    vector<Surface> robotPolygon(4);
    for (unsigned i = 0; i < 4; i++) {
        robotPolygon[i] = Surface(PointXY(tentativeRobot[i].X1(), tentativeRobot[i].Y1()), PointXY(tentativeRobot[i].X2(), tentativeRobot[i].Y2()), true);
    }

    //    cout<<"Current view"<<endl;
    //    displayObjects(currentView);
            
    Object originToExit,originToNonExit, originToLeftSide, originToRightSide;
    double exitAngle,nonExitAngle, leftSideAngle, rightSideAngle,angleToAvoid;
    bool obstacleFreePath = true;
    //check for left side
    angleToAvoid = 0;//when there is no obstacle then this 0 value is necessary bcz at the end it will be subtract from angleToDest
    for (unsigned int i = 0; i < currentView.size(); i++) {
//        if (currentView[i].getPEP1() == true or currentView[i].getPEP2() == true) {//guess. it will happen unless xtra-ordinary case
            double xWithLS = checkForIntersection(currentView[i], tentativeRobot[0]);
            double xWithFS = checkForIntersection(currentView[i], tentativeRobot[1]);
            double xWithRS = checkForIntersection(currentView[i], tentativeRobot[2]);
            bool isP1Inside = pointInPolygon(PointXY(currentView[i].X1(), currentView[i].Y1()), robotPolygon, true);
            bool isP2Inside = pointInPolygon(PointXY(currentView[i].X2(), currentView[i].Y2()), robotPolygon, true);
            if (xWithLS == 1 or xWithFS == 1 or xWithRS == 1 or isP1Inside == true or isP2Inside == true) {
                cout<<endl<<endl<<"Bloody obstacle on my way!!! going to avoid...."<<endl<<endl;
                obstacleFreePath = false;
                if (currentView[i].getPEP1() == false && currentView[i].getPEP2() == true) {//exit is on p2 side
                    cout<<"P2 end is an exit Point"<<endl;
                    originToExit.set(0, 0, currentView[i].X2(), currentView[i].Y2(), 1);
                    exitAngle = currentRobotPose.getAngleWithLine(originToExit);
                    originToNonExit.set(0,0,currentView[i].X1(), currentView[i].Y1(), 1);
                    nonExitAngle = currentRobotPose.getAngleWithLine(originToNonExit);
                    if(exitAngle < nonExitAngle) {//need to turn right
                        cout<<"Need to turn right"<<endl;
                        originToLeftSide.set(0, 0, tentativeRobot[0].X1(), tentativeRobot[0].Y1(), 1);
                        leftSideAngle = currentRobotPose.getAngleWithLine(originToLeftSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "LeftSide Angle " << leftSideAngle << endl;
                        cout<<"ShiftAngle: "<<leftSideAngle-exitAngle<<endl;
                        angleToAvoid = leftSideAngle-exitAngle;
                    } else {//need to turn left
                        cout<<"Need to turn left"<<endl;
                        originToRightSide.set(0, 0, tentativeRobot[2].X1(), tentativeRobot[2].Y1(), 1);
                        rightSideAngle = currentRobotPose.getAngleWithLine(originToRightSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "rightSide Angle " << rightSideAngle << endl;
                        cout<<"ShiftAngle: "<<rightSideAngle-exitAngle<<endl;
                        angleToAvoid = rightSideAngle-exitAngle;
                    }
                } else if (currentView[i].getPEP1() == true && currentView[i].getPEP2() == false) {//exit is on p1 side
                    cout<<"P1 end is an exit Point"<<endl;
                    originToExit.set(0, 0, currentView[i].X1(), currentView[i].Y1(), 1);
                    exitAngle = currentRobotPose.getAngleWithLine(originToExit);
                    originToNonExit.set(0,0,currentView[i].X2(), currentView[i].Y2(), 1);
                    nonExitAngle = currentRobotPose.getAngleWithLine(originToNonExit);
                    cout<<"ExitAngle: "<<exitAngle<<" nonExitAngle: "<<nonExitAngle<<endl;
                    if(exitAngle < nonExitAngle) {//need to turn right
                        cout<<"Need to turn right"<<endl;
                        originToLeftSide.set(0, 0, tentativeRobot[0].X1(), tentativeRobot[0].Y1(), 1);
                        leftSideAngle = currentRobotPose.getAngleWithLine(originToLeftSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "LeftSide Angle " << leftSideAngle << endl;
                        cout<<"ShiftAngle: "<<leftSideAngle-exitAngle<<endl;
                        angleToAvoid = leftSideAngle-exitAngle;
                    } else {//need to turn left
                        cout<<"Need to turn left"<<endl;
                        originToRightSide.set(0, 0, tentativeRobot[2].X1(), tentativeRobot[2].Y1(), 1);
                        rightSideAngle = currentRobotPose.getAngleWithLine(originToRightSide);
                        if(rightSideAngle > 180)
                            rightSideAngle = rightSideAngle-360;
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "rightSide Angle " << rightSideAngle << endl;
                        cout<<"ShiftAngle: "<<rightSideAngle-exitAngle<<endl;
                        angleToAvoid = rightSideAngle-exitAngle;
                    }
                } else {
                    cout<<"this object doesn't contain exit. it's neighbor. search that object"<<endl;
                    for(unsigned int j=i+1;j<currentView.size();j++) {
                        if(currentView[j].getPEP1() == true or currentView[j].getPEP2() == true) {
                            if(currentView[j].getPEP1() == true) {
                                originToExit.set(0, 0, currentView[j].X1(), currentView[j].Y1(), 1);
                                originToNonExit.set(0,0,currentView[j].X2(), currentView[j].Y2(), 1);
                            } else {
                                originToExit.set(0, 0, currentView[j].X2(), currentView[j].Y2(), 1);
                                originToNonExit.set(0,0,currentView[j].X1(), currentView[j].Y1(), 1);
                            }
                            break;
                        }
                    }
                    exitAngle = currentRobotPose.getAngleWithLine(originToExit);
                    nonExitAngle = currentRobotPose.getAngleWithLine(originToNonExit);
                    cout<<"ExitAngle: "<<exitAngle<<" nonExitAngle: "<<nonExitAngle<<endl;
                    if(exitAngle < nonExitAngle) {//need to turn right
                        cout<<"Need to turn right"<<endl;
                        originToLeftSide.set(0, 0, tentativeRobot[0].X1(), tentativeRobot[0].Y1(), 1);
                        leftSideAngle = currentRobotPose.getAngleWithLine(originToLeftSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "LeftSide Angle " << leftSideAngle << endl;
                        cout<<"ShiftAngle: "<<leftSideAngle-exitAngle<<endl;
                        angleToAvoid = leftSideAngle-exitAngle;
                    } else {//need to turn left
                        cout<<"Need to turn left"<<endl;
                        originToRightSide.set(0, 0, tentativeRobot[2].X1(), tentativeRobot[2].Y1(), 1);
                        rightSideAngle = currentRobotPose.getAngleWithLine(originToRightSide);
                        if(rightSideAngle > 180)
                            rightSideAngle = rightSideAngle-360;
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "rightSide Angle " << rightSideAngle << endl;
                        cout<<"ShiftAngle: "<<rightSideAngle-exitAngle<<endl;
                        angleToAvoid = rightSideAngle-exitAngle;
                    }
                }                
            }
        //}

    }
    
    Destination verifiedDestination;
    double newAngleToDest = angleToDest-angleToAvoid;
    verifiedDestination.setAngle(newAngleToDest);
    verifiedDestination.setDist(distanceToDest);
    
    if(obstacleFreePath == false) {
        verifiedDestination.setType(2);
    }
    else
        verifiedDestination.setType(1);
    
    
    return verifiedDestination;
}

//goalType = 1 for Exit goals
//goalType = 2 for farthest path goals
vector<pair<double, double> > findInvalidatedGoals(vector<Object> currentView, int goalType) {
    vector<pair<double, double> > result;
    pair<double, double> goal; //first - angle, second- distance 
    vector<Object> sortedCurrentView;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        if (currentView[i].length() > 400)//this condition is actually for farthest path goal but it doesn't matter for exit goal
            sortedCurrentView.push_back(currentView[i]);//bcz all exits are longer than 400
    }
    std::sort(sortedCurrentView.begin(), sortedCurrentView.end(), sortA2MPDistanceFromOrigin);

    //calculating angle and distance for all goals
    double angleToMaxPath;
    double distanceAlongMaxPath;
    Object maxPath;
    Object robotPose(0, 0, 0, 500, 1);
    for (unsigned int i = 0; i < sortedCurrentView.size(); i++) {
        //calculating angle
        maxPath.set(0, 0, sortedCurrentView[i].mpX(), sortedCurrentView[i].mpY(), 1); //max path object
        angleToMaxPath = robotPose.getAngleWithLine(maxPath); //angle to max path

        //calculating distance
        if (maxPath.length() > 1400)
            distanceAlongMaxPath = 1000;
        else if (maxPath.length() > 400)
            distanceAlongMaxPath = maxPath.length() - 400;
        else if (goalType == 1) //so when robot close to exit then just pass the exit
            distanceAlongMaxPath = maxPath.length() + 200;
        else
            distanceAlongMaxPath = 0;//need to comment out
        
        goal.first = angleToMaxPath;
        goal.second = distanceAlongMaxPath;
        result.push_back(goal);
    }
    return result;
}

vector<Exit> findExits(vector<Object> cv) 
{
    cout << "\033[1;32m-------------------Inside findExits module---------------------\033[0m" << endl;
    //vector<Exit> result;
    //vector<Object> bObjects;//Objects on asr boundary
    double distp2top1;
    //double mindist=400;
    
    for(unsigned int i=0;i<cv.size();i++) 
    {
        cv[i].setID(i+1);
    }
    
    Exit exit;
    int no_exit;
    vector<Exit> exits;
    cout << endl << "******* Finding exits(yeap's theory) ******* " << endl;
    
//    //first side exit
//    exit.set(0, 0, cv[0].X1(), cv[0].Y1()); 
//    exit.setID(1);
//    exit.setP1ID(0);
//    exit.setP2ID(1);
//    exits.push_back(exit);
    
    int exits_counter = 2;
    
   
    for (int i = 0; i<int(cv.size()-1); i++) 
    {
            if (cv[i].getPEP2() == true) //p2 is a probable exit end(p1)
            { 
                    //cout<<"  "<<i<<" "<<cv[i].getID()<<endl;
                    no_exit = 0;
                    for (int j = i + 1; j<int(cv.size()); j++) 
                    {
                        if (cv[j].getPEP1() == true) 
                        { //p1 is a probable exit end(p2)
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
                                        if (exit.length() > distp2top1)
                                        { //condition to get shortest exit
                                            exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                            exit.setP1ID(cv[i].getID());
                                            exit.setP2ID(cv[j].getID());

                                        }
                                }
                              // i = i-1;//new line for place gaps
                        }//if cv j
                     }//for j
                //bObjects.push_back(cv[i]);//boundary Object having exit vertex
                //i = exit.getP2ID() - 2; //for triming for cv

                //exit.setID(exits.back().getID()+1);
                exit.setID(exits_counter);
                exits.push_back(exit);
                exits_counter++;
                //exit.display();

                //cout<<"jj "<<jj<<" id "<<exit.getP2ID()<<endl;
                //i=j-1;
            }//if cv i
    }// for i

    //Point tmpp=cv[33].shortestDistPointFrom(cv[49].X2(),cv[49].Y2());
    //exits.back().set(cv[49].X2(),cv[49].Y2(),tmpp.X(),tmpp.Y());

    //last side exit
//    exit.set(cv[int(cv.size() - 1)].X2(), cv[int(cv.size() - 1)].Y2(), 0, 0); 
//    exit.setID(exits_counter);
//    exit.setP1ID(cv[int(cv.size() - 1)].getID());
//    exit.setP2ID(0);
//    exits.push_back(exit);
    
    //displayObjects(exits);
    //cout<<"boundary Objects "<<endl;displayObjects(bObjects);



    //result.push_back(bObjects);
    //result.push_back(exits);

    //cout<<" last Object of cv "<<endl;cv.back().display();

    //std::sort(exits.begin(),exits.end(),sortExitsA2L);
    return exits;


}

//modified version of findExits()
//finds exits e(p1,p2)
// where, p1 is the first point of exit(probable exit point 2 of objects)
//		  p2 is the end point of exit(nearest point on the objects which contains the nearest probable exit point 1)
//this module is used in PartialUpdating.cpp which updates the map only before crossing the exits

vector<Exit> findShortestExits(vector<Object> cv) 
{
        //vector<Exit> result;
        //vector<Object> bObjects;//Objects on asr boundary
        double distp2top1;
        //double mindist=400;
        Exit exit;
        int no_exit;
        vector<Exit> exits;
        cout << endl << "******* Finding exits(modified version of Yeap's theory) ******* " << endl;
        cout << "CV" << endl;
        //displayObjects(cv);
        int exits_counter = 1;
        for (int i = 0; i<int(cv.size()); i++) 
        {
            if (cv[i].getPEP2() == true) 
            { //p2 is a probable exit end(p1)
                no_exit = 0;
                for (int j = i + 1; j<int(cv.size()); j++) 
                {
                     // if(cv[j].getPEP1() == true && cv[j].getP1OS() == 1 ) { //p1 is a probable exit end(p2)
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
                             if (exit.length() > distp2top1) { //condition to get shortest exit
                                 exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                 exit.setP1ID(cv[i].getID());
                                 exit.setP2ID(cv[j].getID());
                             }
                     }
                    //}//if cv j
                }//for j

                i = exit.getP2ID() - 2; //for triming
                cout << "got one exit " << exit.getP1ID() << " " << exit.getP2ID() << endl;
                exit.setID(exits_counter);
                //find the nearest point instead of p2
                Point tmpp = cv[exit.getP2ID() - 1].shortestDistPointFrom(exit.X1(), exit.Y1());
                exit.set(exit.X1(), exit.Y1(), tmpp.X(), tmpp.Y());

                exits.push_back(exit);
                exits_counter++;
                //exit.display();

                //i=j-1;
            }//if cv i

        }// for i
        cout << "shortest exits" << endl;
    //    displayExits(exits);

        vector<Exit> realExits;
        for (int i = 0; i<int(exits.size()); i++) 
        {
            if (exits[i].length() > 800 )//&& exits[i].length() < 1200)
                realExits.push_back(exits[i]);
        }

        return realExits;
}

//modified version of findShortestExits()
//Definition of DestinationExits:  when some objects are seen through exits then those exits are called destination Exits
//in other words, some shortest exits are not destination exits bcz those are uncertain gaps on the boundary 
//but destination exits are those shortest exits which are not on the CV boundary 
//implementation: extra condition abs(exits[i].getP2ID()-exits[i].getP1ID()) > 1 is added at the end
//finds exits e(p1,p2)
// where, p1 is the first point of exit(probable exit point 2 of objects)
//		  p2 is the end point of exit(nearest point on the objects which contains the nearest probable exit point 1)
//this module is used in PartialUpdating.cpp which updates the map only before crossing the exits

vector<Object> findDestinationExits(vector<Object> cv, vector<Object> refObjects) {

    vector<Exit> exits = findShortestExits(cv);
    vector<Exit> realExits;
    for (int i = 0; i<int(exits.size()); i++) { //real exits have specific length
        if (exits[i].length() > 600 && exits[i].length() < 1200)
            realExits.push_back(exits[i]);
    }
    cout<<"Shortest Exits from CV "<<endl;
    displayExits(realExits);
    vector<Exit> destExits;
    Object originToObject, exitAsObject;
    bool itsAnExit = true;
    for (unsigned int i = 0; i<realExits.size(); i++) {
        if (abs(realExits[i].getP2ID()-realExits[i].getP1ID()) > 1 ){//two consecutive objects means uncertain exit/gap
            exitAsObject.set(realExits[i].X1(),realExits[i].Y1(),realExits[i].X2(),realExits[i].Y2(),1);
            itsAnExit = true;
            for(unsigned int j = realExits[i].getP1ID(); j < realExits[i].getP2ID()-1;j++) {
                originToObject.set(0,0,cv[j].mpX(),cv[j].mpY(),2);
                if(checkForIntersection(exitAsObject,originToObject) != 1) {//to filter those exits who has obstacle towards origin
//                    cout<<"it's not an exit "<<i+1<<endl;
                    itsAnExit = false;
                    break;
                }
            }
            if(itsAnExit == true)
                destExits.push_back(realExits[i]);
        }
    }
    
    cout<<"Destination Exits: "<<endl;
    displayExits(destExits);
    
    
    vector<Object> destinationExits = convertExitToObject(destExits);    
    
//    plotObjectsAndPExits("MFIS/destExits.png",destinationExits,cv,realExits);
//    waitHere();
//    vector<Object> destinationExitsInMFIS;
//    for(unsigned int i = 0; i<destinationExits.size();i++) {
//        destinationExitsInMFIS.push_back(remakeLineP2(refObjects[0],refObjects[1],destinationExits[i],1,0,refObjects[0].getKP()));
//    }
//
//    cout<<"Destination Exits as Objects"<<endl;
//    displayObjects(destinationExitsInMFIS);

//    return destinationExitsInMFIS;
    return destinationExits;
}

//Difference btw this function and findDestinationExits
//findDestinationExits returns only real Exits(who has specific length)
//this function returns all exits(Gaps) whose length more than 60cm
vector<Object> findAllDestinationExits(vector<Object> cv, vector<Object> refObjects) {

    vector<Exit> exits = findShortestExits(cv);
    vector<Exit> realExits;
    for (int i = 0; i<int(exits.size()); i++) { //real exits have specific length
        if (exits[i].length() > 600 )//&& exits[i].length() < 1200)
            realExits.push_back(exits[i]);
    }
    cout<<"Shortest Exits from CV "<<endl;
    displayExits(realExits);
    vector<Exit> destExits;
    Object originToObject, exitAsObject;
    bool itsAnExit = true;
    for (unsigned int i = 0; i<realExits.size(); i++) {
        if (abs(realExits[i].getP2ID()-realExits[i].getP1ID()) > 1 ){//two consecutive objects means uncertain exit/gap
            exitAsObject.set(realExits[i].X1(),realExits[i].Y1(),realExits[i].X2(),realExits[i].Y2(),1);
            itsAnExit = true;
            for(unsigned int j = realExits[i].getP1ID(); j < realExits[i].getP2ID()-1;j++) {
                originToObject.set(0,0,cv[j].mpX(),cv[j].mpY(),2);
                if(checkForIntersection(exitAsObject,originToObject) != 1) {//to filter those exits who has obstacle towards origin
//                    cout<<"it's not an exit "<<i+1<<endl;
                    itsAnExit = false;
                    break;
                }
            }
            if(itsAnExit == true)
                destExits.push_back(realExits[i]);
        }
    }
    
    cout<<"Destination Exits: "<<endl;
    displayExits(destExits);
    
    
    vector<Object> destinationExits = convertExitToObject(destExits);    
    
//    plotObjectsAndPExits("MFIS/destExits.png",destinationExits,cv,realExits);
//    waitHere();
//    vector<Object> destinationExitsInMFIS;
//    for(unsigned int i = 0; i<destinationExits.size();i++) {
//        destinationExitsInMFIS.push_back(remakeLineP2(refObjects[0],refObjects[1],destinationExits[i],1,0,refObjects[0].getKP()));
//    }
//
//    cout<<"Destination Exits as Objects"<<endl;
//    displayObjects(destinationExitsInMFIS);

//    return destinationExitsInMFIS;
    return destinationExits;
}

vector<Object> tranformCVExitsInMFIS(vector<Object> cvExits, vector<Object> refObjects) {
    vector<Object> destinationExitsInMFIS;
    for (unsigned int i = 0; i < cvExits.size(); i++) {
        destinationExitsInMFIS.push_back(remakeLineP2(refObjects[0], refObjects[1], cvExits[i], 1, 0, refObjects[0].getKP()));
    }
    return destinationExitsInMFIS;
}

//modified version of findExits()
//finds exits e(p1,p2)
// where, p1 is the first point of exit(probable exit point 2 of objects)
//		  p2 is the end point of exit(nearest point on the objects which contains the nearest probable exit point 1)
// vital condition is exit length should be between 800 to 1200 cm.
//this module is used in PartialUpdating.cpp which updates the map only before crossing the exits
//developed on Tuesday Oct 11, 2011
vector<Exit> findNewASRFormingExits(vector<Object> cv) {
    //vector<Exit> result;
    //vector<Object> bObjects;//Objects on asr boundary
    double distp2top1,probableExitLength;
    //double mindist=400;
    Point sDistPoint;
    Exit exit;
    int no_exit;
    vector<Exit> exits;
    cout << endl << "******* Finding exits(modified version yeap's theory) ******* " << endl;
    //cout << "CV" << endl;
    //displayObjects(cv);
    int exits_counter = 2;
    for (int i = 0; i<int(cv.size()); i++) {
        if (cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
            no_exit = 0;
            for (int j = i + 1; j<int(cv.size()); j++) {
                //if(cv[j].getPEP1() == true) { //p1 is a probable exit end(p2)
               /* if (no_exit == 0) {
                    exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                    no_exit++;
                } else {
                    distp2top1 = cv[i].distP2ToP1(cv[j]);
                    if (exit.length() > distp2top1) { //condition to get shortest exit
                        exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                        exit.setP1ID(cv[i].getID());
                        exit.setP2ID(cv[j].getID());
                    }
                }*/
                //}//if cv j
                probableExitLength = cv[j].shortestDistFrom(cv[i].X2(),cv[i].Y2());
                if(probableExitLength > 800 && probableExitLength < 1200) {
                    //find the shortest point on cv[j] from cv[i] p2 
                    sDistPoint = cv[j].shortestDistPointFrom(cv[i].X2(),cv[i].Y2());
                    exit.set(cv[i].X2(), cv[i].Y2(),sDistPoint.X(),sDistPoint.Y());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                    exit.setID(exits_counter);
                    exits.push_back(exit);
                    exits_counter++;
                }
                
            }//for j

//            i = exit.getP2ID() - 2; //for triming
//            cout << "got one exit " << exit.getP1ID() << " " << exit.getP2ID() << endl;
//            
//            //find the nearest point instead of p2
//            Point tmpp = cv[exit.getP2ID() - 1].shortestDistPointFrom(exit.X1(), exit.Y1());
//            exit.set(exit.X1(), exit.Y1(), tmpp.X(), tmpp.Y());

            
           
            //exit.display();

            //i=j-1;
        }//if cv i

    }// for i
//   
    return exits;
}

//modified version of findShortestExits()
//finds exits e(p1,p2)
// where, p1 is the first point of exit(p2 of any objects)
//		  p2 is the end point of exit(nearest point on any other objects except objects which are in the same group
//			of p1 containing object)
//this module is used in Algorithm3.cpp to compute ASR(Algorithm3 split ASR when robot crosses a exit of this kind

vector<Exit> findShortestExitsForASR(vector<Object> cv) {

    double distp2top1, shortestDist;
    Exit exit;
    int no_exit, nextPEP1=0;
    vector<Exit> exits;
    int exits_counter = 2;

    for (int i = 0; i<int(cv.size()); i++) {
        //if(cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
        for (int k = i + 1; k<int(cv.size()); k++)
            if (cv[k].getPEP1() == true)
                nextPEP1 = k;
        no_exit = 0;
        for (int j = nextPEP1; j<int(cv.size()); j++) {
            shortestDist = cv[j].shortestDistFrom(cv[i].X2(), cv[i].Y2());
            if (shortestDist > 600 && shortestDist < 1200) {//p1 is a probable exit end(p2)
                //if(cv[j].getPEP1() == true) { //p1 is a probable exit end(p2)
                if (no_exit == 0) {
                    exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                    no_exit++;
                } else {
                    distp2top1 = cv[i].distP2ToP1(cv[j]);
                    if (exit.length() > distp2top1) { //condition to get shortest exit
                        exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                        exit.setP1ID(cv[i].getID());
                        exit.setP2ID(cv[j].getID());
                    }
                }
                //}//if cv j
            }//for j

            i = exit.getP2ID() - 2; //for triming
            cout << "got one exit " << exit.getP1ID() << " " << exit.getP2ID() << endl;
            exit.setID(exits_counter);
            //find the nearest point instead of p2
            Point tmpp = cv[exit.getP2ID() - 1].shortestDistPointFrom(exit.X1(), exit.Y1());
            exit.set(exit.X1(), exit.Y1(), tmpp.X(), tmpp.Y());

            exits.push_back(exit);
            exits_counter++;
        }//if cv i

    }// for i
    cout << "shortest exits" << endl;
    displayExits(exits);

    return exits;
}

vector<Exit> findGateways(vector<Object> cv) {
	cout<<endl<<"Inside gateway finding module "<<endl;
	//cout<<"cv size "<<cv.size()<<endl;
	//displayObjects(cv);

	double distp2top1;

	Exit exit;
	int no_exit;
	vector<Exit> exits;
	cout<<endl<<"******* Finding Gateways******* "<<endl;
	int exits_counter=1;
	for(int i=0;i<int(cv.size()-1);i++) {
		if(cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
			no_exit=0;
			for(int j=i+1;j<int(cv.size());j++) {
				if(cv[j].getPEP1() == true && cv[j].getP1OS() == 1 ) {//&& cv[i].distP2ToP1(cv[j]) > 600){p1 is a probable exit end(p2)
					if(no_exit == 0) {
						Object tmp;
						tmp.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),1);
						bool obs=false;
						for(int k=cv[i].getID();k<cv[j].getID()-1;k++) {
							if(checkForIntersection(tmp,cv[k]) == 1) {
								obs=true;
							}
						}
						if(obs == true) {
							exit.set(cv[i].X2(),cv[i].Y2(),cv[i+1].X1(),cv[i+1].Y1());
							exit.setP1ID(cv[i].getID());
							exit.setP2ID(cv[i+1].getID());
						}
						else {
							exit.set(tmp);
							exit.setP1ID(cv[i].getID());
							exit.setP2ID(cv[j].getID());
						}//
						no_exit++;
					}
					else {
						distp2top1=cv[i].distP2ToP1(cv[j]);
						double dist_from_robot=cv[j].distP1ToPoint(0,0);
						//condition to get shortest exit
						if(exit.length() > distp2top1 && exit.distP2ToPoint(0,0) > dist_from_robot) {
							Object tmp;
							tmp.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),1);
							bool obs=false;
							for(int k=cv[i].getID();k<cv[j].getID()-1;k++) {
								if(checkForIntersection(tmp,cv[k]) == 1) {
									obs=true;
								}
							}
							if(obs == true) {
								exit.set(cv[i].X2(),cv[i].Y2(),cv[i+1].X1(),cv[i+1].Y1());
								exit.setP1ID(cv[i].getID());
								exit.setP2ID(cv[i+1].getID());
							}
							else {
								exit.set(tmp);
								exit.setP1ID(cv[i].getID());
								exit.setP2ID(cv[j].getID());
							}//
						}
					}
				}//if cv j
			}//for j

			if(no_exit > 0) {
				i=exit.getP2ID()-2;//for triming
				//cout<<"i "<<i<<endl;

				exit.setID(exits_counter);
				exits.push_back(exit);
				exits_counter++;
			}

		}//if cv i

	}// for i

	//setting angle with robot direction
	Object rdirection(0,0,0,300,1);
	for(int i=0;i<int(exits.size());i++) {
		//Object exit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object
		Object edirection(0,0,exits[i].mpX(),exits[i].mpY(),i+1);
		Object exit(0,0,exits[i].X1(),exits[i].Y1(),1);
		exits[i].setAngle(abs(exit.getAngleWithLine(edirection)));//rdirection.getAngleWithLine(edirection));
//		cout<<"angle with x axis "<<exit.getAngleWithXaxis()<<endl;
		//exits[i].display();

	}

	//cout<<"exits size "<<exits.size()<<endl;
	//displayExits(exits);
	std::sort(exits.begin(),exits.end(),sortExitsA2A);
	displayExits(exits);
	//char go;
	//cin>>go;
	return exits;
}

vector<Exit> findGatewaysNew(vector<Object> cv) {
	cout<<endl<<"Inside New gateway finding module "<<endl;
	//cout<<"cv size "<<cv.size()<<endl;
	//displayObjects(cv);

	//double distp2top1;

	vector<Object> rsides;
	Object rside;
	rside.set(-200,0,-200,500,1);
	rsides.push_back(rside);
	rside.set(200,0,200,500,2);
	rsides.push_back(rside);



	Exit exit;
	//int no_exit;
	vector<Exit> exits, tmpexits;
	cout<<endl<<"******* Finding Gateways******* "<<endl;
	int exits_counter=1;
	for(int i=0;i<int(cv.size()-1);i++) {
		if(cv[i].getPEP2() == true) { //p2 is a probable exit end(it's p1 for exit)
			//tmpexits.clear();
			for(int j=i+1;j<int(cv.size());j++) {//looking for p2 end(p1 of objects) for this exit
				if(cv[j].getPEP1() == true && cv[j].getP1OS() == 1 ) {//&& cv[i].distP2ToP1(cv[j]) > 600) {//){p1 is a probable exit end(p2)
					//if(no_exit == 0) {
						Object tmp;
						tmp.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),1);
						rsides[0].setP2(tmp.mpX()-200,tmp.mpY()+500);//robot side 1
						rsides[1].setP2(tmp.mpX()+200,tmp.mpY()+500);//robot side 2
						bool obs=false;
						for(int k=cv[i].getID();k<cv[j].getID()-1;k++) {
							if(checkForIntersection(tmp,cv[k]) == 1) {
								obs=true;
							}
							else if(checkForIntersection(rsides[0],cv[k]) == 1) {
								obs=true;
							}
							else if(checkForIntersection(rsides[1],cv[k]) == 1) {
								obs=true;
							}
						}
						if(obs == false) {

							exit.set(tmp);
							exit.setP1ID(cv[i].getID());
							exit.setP2ID(cv[j].getID());
							exit.setP2DistFromRobot(exit.distP2ToPoint(0,0));
							tmpexits.push_back(exit);
						}

				}//if cv j
			}//for j
			int tmpsize=tmpexits.size();
			if(tmpsize == 0){ //if no p2 for this p1
				exit.set(cv[i].X2(),cv[i].Y2(),cv[i+1].X1(),cv[i+1].Y1());
				if(exit.length() > 600) { //don't push if not wide enough
					exit.setP1ID(cv[i].getID());
					exit.setP2ID(cv[i+1].getID());
					exits.push_back(exit);
					exits.back().setID(exits_counter);
					exits_counter++;
				}
				i=cv[i+1].getID()-2;//for trimming
			}
			else if(tmpsize == 1){//if only one for this p1
				if(tmpexits.back().length() > 600) {//don't push if not wide enough
					exits.push_back(tmpexits.back());
					exits.back().setID(exits_counter);
					exits_counter++;
				}
				tmpexits.clear();
				i=tmpexits[0].getP2ID()-2;//trimming
			}
			else { //if more than one for this p1
				Exit tmpe1;//tmp exit 1
				tmpe1.set(tmpexits[0]);
				double e1length=tmpe1.length();
				double e1dist=tmpe1.getP2DistFromRobot();
				double ellength,eldist;
				for(int l=1;l<int(tmpexits.size());l++) {
					ellength=tmpexits[l].length();
					eldist=tmpexits[l].getP2DistFromRobot();
					if(e1dist > eldist) {//e1length >  ellength &&
						tmpe1.set(tmpexits[l]);
						e1length=ellength;
						e1dist=eldist;
					}
				}
				if(tmpe1.length() > 600) {//don't push if not wide enough
					exits.push_back(tmpe1);
					exits.back().setID(exits_counter);
					exits_counter++;
					cout<<"only once "<<endl;
				}
				tmpexits.clear();
				i=tmpe1.getP2ID()-2;//trimming
			}

			//i=exits.back().getP2ID()-2;//trimming
			//if(exits.back().length() < 600) {

		}//if cv i

	}// for i

	//setting angle with robot direction
	Object rdirection(0,0,0,300,1);
	for(int i=0;i<int(exits.size());i++) {
		Object edirection(0,0,exits[i].mpX(),exits[i].mpY(),i+1);
		Object exit(0,0,exits[i].X1(),exits[i].Y1(),1);
		exits[i].setAngle(abs(exit.getAngleWithLine(edirection)));//rdirection.getAngleWithLine(edirection));
	}

	std::sort(exits.begin(),exits.end(),sortExitsA2A);
	displayExits(exits);

	return exits;
}


//new module(called for every steps) to find  next destination(have to use findGatewaysNew first).
Destination findNextDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides) {
	cout<<endl<<"Inside find Next destination module "<<endl;
	//cout<<"current view "<<endl;
	//displayObjects(cv);

	Destination destination;
	double pdist;

	if(exits.size() == 0) {
		bool notclear=false;
		robsides[7].setP2(getX(0,2000)-200,getY(0,2000));
		for(int i=0;i<int(cv.size());i++) {
			if(checkForIntersection(robsides[7],cv[i]) == 1)
				notclear=true;
		}
		if(notclear == false) {
			destination.setAngle(-45);
			destination.setDist(1000);
		}
		else {
			notclear=false;
			robsides[7].setP2(getX(-45,1000)-200,getY(-45,1000));
			for(int i=0;i<int(cv.size());i++) {
				pdist=robsides[7].perpendicularDistOfPoint(cv[i].X2(),cv[i].Y2());
				if(pdist < 100) {
					notclear=true;
				}
			}
			if(notclear == false) {
				destination.setAngle(-45);
				destination.setDist(1000);
			}
		}

	}//first if
	else {
		Object rdirection(0,0,0,300,1);
		Object edirection(0,0,exits[0].mpX(),exits[0].mpY(),1);
		double angle=rdirection.getAngleWithLine(edirection);

		cout<<"Exits "<<endl;
		displayExits(exits);

		robsides[7].setP2(getX(angle,1000)-200,getY(angle,1000));
		robsides[8].setP2(getX(angle,1000)+200,getY(angle,1000));

		pdist=robsides[7].perpendicularDistOfPoint(exits[0].X1(),exits[0].Y1());
		cout<<"pdist "<<pdist<<endl;

		//plotObjectsAndPExits("MFIS/Nextdestination.png",robsides,cv,exits);



		if(pdist > 200) {
			destination.setAngle(angle);
			destination.setDist(1000);
		}
		else {
			double rdist1=exits[0].distP1ToPoint(0,0);
			double rdist2=exits[0].distP2ToPoint(0,0);

			cout<<"rdist1 "<<rdist1<<" rdist2 "<<rdist2<<endl;
			//which point have to avoid
			if(rdist1 < rdist2) { //true means have to avoid first exit point
				destination.setAngle(-15);
				if(rdist1 < 900)
					destination.setDist(rdist1+100);
				else
					destination.setDist(1000);
			}
			else {
				destination.setAngle(15);
				if(rdist2 < 900)
					destination.setDist(rdist2+100);
				else
					destination.setDist(1000);
			}
		}
	}
	cout<<"Destination: angle "<<destination.getAngle()<<" dist: "<<destination.getDist()<<endl;
	return destination;
}

//new module(called for every steps) to find  next destination(have to use findGatewaysNew first).
Destination findDestToMaxPath(vector<Object> cv, vector<Object> robsides, int viewno) {
	cout<<endl<<"Inside find destination to max path module for view "<<viewno<<endl;
	//cout<<"current view "<<endl;
	//displayObjects(cv);

	Destination destination;

	//finding max path
	double mpdist=0;//cv[0].distMPToPoint(0,0);
	int maxindex=0;
	for(int i=0;i<int(cv.size());i++) {
		double tmpdist=cv[i].distMPToPoint(0,0);
		if(mpdist < tmpdist && cv[i].length() > 400) {
			maxindex=i;
			mpdist=tmpdist;
		}
	}

	//calculating direction to max path
	Object rdirection(0,0,0,300,1);
	Object edirection(0,0,cv[maxindex].mpX(),cv[maxindex].mpY(),1);//max path object
	double angle=rdirection.getAngleWithLine(edirection);//angle to max path

	//if(abs(angle) > 50) {
	if(abs(angle) > 50) {//limitting to recognize objects
		if(angle < 0)
			angle=-50;
		else
			angle = 50;
	}


	double destlength=1100;
	robsides[6].setP2(getX(angle,destlength),getY(angle,destlength));
	robsides[7].setP2(getX(angle,destlength)-350,getY(angle,destlength));//rob sides to max path direction
	robsides[8].setP2(getX(angle,destlength)+350,getY(angle,destlength));

	cout<<"direction angle "<<angle<<endl;

	/*cout<<"robsides "<<endl;
	robsides[6].display();
	robsides[7].display();
	robsides[8].display();
	*/
	//looking for any hindrance
	vector<double> interdist;
	bool notclear=false;
	vector<Obstacle> obstacles;
//	cout<<"pdist "<<robsides[7].perpendicularDistOfPoint(cv[0].X2(),cv[0].Y2());
	for(int i=0;i<int(cv.size());i++) {
		//cout<<"checking"<<endl;
		double x7=checkForIntersection(robsides[7],cv[i]);
		double x6=checkForIntersection(robsides[6],cv[i]);
		double x8=checkForIntersection(robsides[8],cv[i]);
		/*double ppd7=robsides[7].perpendicularDistOfPoint(cv[i].X2(),cv[i].Y2());
		double ppd8=robsides[8].perpendicularDistOfPoint(cv[i].X1(),cv[i].Y1());
		double rd7=robsides[7].distP2ToPoint(cv[i].X2(),cv[i].Y2());
		double rd8=robsides[8].distP2ToPoint(cv[i].X1(),cv[i].Y1());*/
		//if(i < 3)
			//cout<<" 7: "<<x7<<" 8: "<<x8<<" x6 "<<x6<<endl;


		//if(x7 ==0 && 8x ==0) {


		if( (x7 == 1 || x6 == 1) && x8 == 0) {//have to turn right
			vector<double> interpoint=getIntersectionPoint(robsides[7],cv[i]);
			//cout<<"inter sect with "<<cv[i].getID()<<" inter point "<<interpoint[0]<<" "<<interpoint[1]<<endl;

			Object tmp(robsides[7].X1(),robsides[7].Y1(),cv[i].X2(),interpoint[1],1);
			double od=rdirection.distP1ToP2(tmp);
			double oa=robsides[7].getAngleWithLine(tmp);
			Obstacle obs(1,od,oa+angle);
			obstacles.push_back(obs);
			notclear=true;
		}


		if( (x8 == 1 || x6 == 1) && x7 == 0) {//have to turn left
			vector<double> interpoint=getIntersectionPoint(robsides[8],cv[i]);
			//cout<<"inter sect with "<<cv[i].getID()<<" inter point "<<interpoint[0]<<" "<<interpoint[1]<<endl;
			Object tmp(robsides[8].X1(),robsides[8].Y1(),cv[i].X1(),interpoint[1],1);
			double od=rdirection.distP1ToP2(tmp);

			double oa=robsides[8].getAngleWithLine(tmp);
			Obstacle obs(2,od,oa+angle);
			obstacles.push_back(obs);

			notclear=true;
		}

		double dp1=rdirection.distP1ToP1(cv[i]);
		double dp2=rdirection.distP1ToP2(cv[i]);
		if( dp1 < 1500 ||  dp2 < 1500) {
			bool isp1inside=pointInPolygon(robsides[7],robsides[6],cv[i].X1(),cv[i].Y1());
			bool isp2inside=pointInPolygon(robsides[7],robsides[6],cv[i].X2(),cv[i].Y2());
			if(isp1inside == true || isp2inside == true) {
				double od;
				Object tmp;
				double px,py;
					if(isp1inside == true && isp2inside == false) {
						tmp.set(robsides[7].X1(),robsides[7].Y1(),cv[i].X1(),cv[i].Y1(),1);
						od=robsides[6].distP1ToPoint(cv[i].X1(),cv[i].Y1());
					}
					else if(isp1inside == false && isp2inside == true) {
						tmp.set(robsides[7].X1(),robsides[7].Y1(),cv[i].X2(),cv[i].Y2(),1);
						od=robsides[6].distP1ToPoint(cv[i].X2(),cv[i].Y2());
					}
					else {
						if(cv[i].X1() > cv[i].X2())
							px=cv[i].X1();
						else
							px=cv[i].X2();
						if(cv[i].Y1() < cv[i].Y2()){
							tmp.set(robsides[7].X1(),robsides[7].Y1(),px,cv[i].Y1(),1);		//take turn upto mid direct
							py=cv[i].Y1();
						}
						else {
							tmp.set(robsides[7].X1(),robsides[7].Y1(),px,cv[i].Y2(),1);
							py=cv[i].Y2();
						}
						od=robsides[6].distP1ToPoint(px,py);
					}
					robsides.push_back(tmp);

				double oa=robsides[7].getAngleWithLine(tmp);
				Obstacle obs(3,od,oa+angle);
				obstacles.push_back(obs);
				notclear=true;
				cout<<"Got LEFT inside obstacle.id-"<<cv[i].getID()<<endl;
			}
			else {
				isp1inside=pointInPolygon(robsides[6],robsides[8],cv[i].X1(),cv[i].Y1());
				isp2inside=pointInPolygon(robsides[6],robsides[8],cv[i].X2(),cv[i].Y2());
				if(isp1inside == true || isp2inside == true) {
					double od;
					Object tmp;
					double px,py;
					if(isp1inside == true && isp2inside == false) {
						tmp.set(robsides[8].X1(),robsides[8].Y1(),cv[i].X1(),cv[i].Y1(),1);
						od=robsides[6].distP1ToPoint(cv[i].X1(),cv[i].Y1());
					}
					else if(isp1inside == false && isp2inside == true) {
						tmp.set(robsides[8].X1(),robsides[8].Y1(),cv[i].X2(),cv[i].Y2(),1);
						od=robsides[6].distP1ToPoint(cv[i].X2(),cv[i].Y2());
					}
					else {
						if(cv[i].X1() < cv[i].X2())
							px=cv[i].X1();
						else
							px=cv[i].X2();
						if(cv[i].Y1() < cv[i].Y2()){
							tmp.set(robsides[8].X1(),robsides[8].Y1(),px,cv[i].Y1(),1);		//take turn upto mid direct
							py=cv[i].Y1();
						}
						else {
							tmp.set(robsides[8].X1(),robsides[8].Y1(),px,cv[i].Y2(),1);
							py=cv[i].Y2();
						}
						od=robsides[6].distP1ToPoint(px,py);
					}
					robsides.push_back(tmp);

					double oa=robsides[8].getAngleWithLine(tmp);
					cout<<"oa "<<oa<<endl;
					Obstacle obs(3,od,oa+angle);
					obstacles.push_back(obs);
					notclear=true;
					cout<<"Got RIGHT inside obstacle.id-"<<cv[i].getID()<<endl;
				}
			}
			//cout<<" oa "<<oa<<endl;
		}//for inside obstacle

	}

	if(notclear == false) { //means clear

		destination.setAngle(angle);
		destination.setDist(1000);
		destination.setType(1);
	}
	else {
		std::sort(obstacles.begin(),obstacles.end(),sortObsA2D);
		displayObstacles(obstacles);
		double oangle=obstacles[0].getAngle();
		double odist=obstacles[0].getDist();
		if(odist > 1000)//
			odist=1000;
		//else
			//odist=odist+100;

		cout<<"Max path isn't clear "<<endl;
		/*if(obstacles[0].getID() == 1 ) {//have to take right
			if(abs(oangle) < 15) {
				destination.setAngle(-15);
				destination.setDist(odist);
			}
			else {
				destination.setAngle(-abs(oangle));
				destination.setDist(odist);
			}
		}
		else if(obstacles[0].getID() == 2) {
			if(abs(oangle) < 15) {//have to take left
				destination.setAngle(15);
				destination.setDist(odist);
			}
			else {
				destination.setAngle(abs(oangle));
				destination.setDist(odist);
			}
		}
		else {//when obs inside rectangle
		*/	cout<<" angle to turn "<<oangle<<endl;
			//if(abs(oangle) < 15) {
			/*
				if(oangle < 0)
					destination.setAngle(-15);
				else
					destination.setAngle(15);*/
			//}
			//else
				destination.setAngle(oangle);
			destination.setDist(odist);
		//}
		destination.setType(2);

	}
	//cout<<"Destination: angle "<<destination.getAngle()<<" dist: "<<destination.getDist()<<endl;


	//plotting the current view
			char vname[50];
			sprintf(vname, "%s%d%s", "MFIS/view-",viewno,".png");
			plotObjects(vname,robsides,cv);

	//if(robsides.size() > 9) {
	//char wait;
	//cin>>wait;
	//}

	return destination;
}


//new module to find destination(have to use findGatewaysNew first)
Destination findDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides) {
	cout<<endl<<"Inside find destination module "<<endl;
	//cout<<"current view "<<endl;
	//displayObjects(cv);

	robsides[7].setP2(-200,exits[0].mpY());
	robsides[8].setP2(200,exits[0].mpY());

	plotObjectsAndPExits("MFIS/destination-c.png",robsides,exits,cv);

	Destination destination;
	Object rdirection(0,0,0,300,1);

	//cout<<"angle "<<exits[0].getAngle()<<endl;
	if(exits.size() == 0) {	//if there's no gateways
		destination.setAngle(0);
		destination.setDist(500);
	}
	else if(abs(exits[0].getAngle()) < 2) { //means doubtful/oblique/opaque/shady gateway
		std::sort(exits.begin(),exits.end(),sortExitsA2L);
		displayExits(exits);

		//turning to the longest gateway
		Object edirection(0,0,exits[0].mpX(),exits[0].mpY(),1);
		//destination.setAngle(rdirection.getAngleWithLine(edirection));
		double angle=rdirection.getAngleWithLine(edirection);
		cout<<"Actual turning angle for doubtful gateway "<<angle<<endl;
		if(angle < 0 && angle > -15)
			destination.setAngle(-15);
		else if(angle > 0 && angle < 15)
			destination.setAngle(15);
		else
			destination.setAngle(angle);
		destination.setDist(0);

	}
	else {
		double pdist=robsides[7].perpendicularDistOfPoint(exits[0].X1(),exits[0].Y1());
		cout<<"pdist "<<pdist<<endl;

		if(pdist < 100) {//set secondary destination point
			double rdist1=exits[0].distP1ToPoint(0,0);
			double rdist2=exits[0].distP2ToPoint(0,0);

			cout<<"rdist1 "<<rdist1<<" rdist2 "<<rdist2<<endl;
			//which point have to avoid
			if(rdist1 < rdist2) { //true means have to avoid first exit point
				Object edirection(0,0,exits[0].X2(),exits[0].Y2(),1);
				double angle=rdirection.getAngleWithLine(edirection);
				cout<<"Actual turning angle to avoid first exit point "<<angle<<endl;
				if(abs(angle) < 15)
					destination.setAngle(-15);
				else
					destination.setAngle(-abs(angle));//sometimes it becomes +ve(e.g. @result/33

				destination.setDist(rdist1+100);
			}
			else { //have to avoid second exit point
				Object edirection(0,0,exits[0].X1(),exits[0].Y1(),1);
				//destination.setAngle(rdirection.getAngleWithLine(edirection));
				double angle=rdirection.getAngleWithLine(edirection);
				cout<<"Actual turning angle to avoid second exit point "<<angle<<endl;
				if(abs(angle) < 15)
					destination.setAngle(15);
				else
					destination.setAngle(abs(angle));
				destination.setDist(rdist2+100);
			}
			//just for plotting to view the destination point
				double dx=destination.getDist()*sin(-(PI/180)*destination.getAngle());
				double dy=destination.getDist()*cos(-(PI/180)*destination.getAngle());

				cout<<"Obstacle on the robot path "<<endl;

				Object tmp;
				tmp.set(0,0,dx,dy,1);
				robsides.push_back(tmp);
				plotObjectsAndPExits("MFIS/dview.png",robsides,exits,cv);
		}//if pdist
		else { //all clear. there's no hindrance
			Object edirection(0,0,exits[0].mpX(),exits[0].mpY(),1);
			//destination.setAngle(rdirection.getAngleWithLine(edirection));
			double angle=rdirection.getAngleWithLine(edirection);
			cout<<"Actual turning angle for no hindrance exit "<<angle<<endl;
			if(angle < 0 && angle > -15)
				destination.setAngle(-15);
			else if(angle > 0 && angle < 15)
				destination.setAngle(15);
			else
				destination.setAngle(angle);
			destination.setDist(edirection.distP2ToPoint(0,0));
		}
	}//if getAngle < 2

	cout<<"destination "<<destination.getAngle()<<" "<<destination.getDist()<<endl;
	//because of robot's mechanical error
	if(destination.getDist() > 4000) {
		destination.setDist(4000);
	}

	return destination;
}


Destination findDestinationPoint(vector<Object> cv, vector<Exit> exits) {
	//vector<double> result;
	Destination destination;
	Object rdirection(0,0,0,300,1);
	for(int i=0;i<int(exits.size());i++) {
		//cout<<exits[i].getID()<<endl;
		bool clearpath=true;
		Object exit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object
		//exit.display();
		Object edirection(0,0,exits[i].mpX(),exits[i].mpY(),i+1);
		cout<<"Angle with robot direction "<<rdirection.getAngleWithLine(edirection)<<" dist from rp: "<<edirection.distP2ToPoint(0,0)<<endl;
		for(int j=exits[i].getP1ID();j<exits[i].getP2ID()-1;j++) {
			Object object(0,0,cv[j].mpX(),cv[j].mpY(),cv[j].getID());
			double isintersect=checkForIntersection(object,exit);
			//intersected means beyond the exit

			if(isintersect == 0) {
				//cout<<cv[j].getID()<<" intersected"<<endl;
			//	cout<<"obstacle exist"<<endl;
				clearpath=false;
			}
			//cv[j].display();
		}
		// clear path is false when any objects in front of exit
		if(clearpath == true ) {
			cout<<"clear path to exit no "<<exits[i].getID()<<endl;

			destination.setAngle(rdirection.getAngleWithLine(edirection));
			destination.setDist(edirection.distP2ToPoint(0,0));
			break;
		}
		//

	}
	return destination;
}

vector<double> findMovementDirection(vector<Object> pexits) {
	vector<double> result;
	std::sort(pexits.begin(),pexits.end(),sortA2L);

	cout<<"Sorted PExits"<<endl;
	displayObjects(pexits);



	return result;
}

vector<Object> exitsInMFIS(vector<Exit> exits,Object rmfis, Object rcv,int refpoint) {
	vector<Object> result;
	for(int i=0;i<int(exits.size());i++) {
		Object tmpexit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object
		Object exit=remakeLineP2(rmfis,rcv,tmpexit,1,0, refpoint);
		result.push_back(exit);
	}

	return result;
}

//converts vector of exits to vector of objects
vector<Object> convertExitToObject(vector<Exit> exits) {
	vector<Object> result;
	for(int i=0;i<int(exits.size());i++) {
		Object tmpexit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object

		result.push_back(tmpexit);
	}
	return result;
}


Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno) {
    vector<Surface> surfaces(s.size());

    for (unsigned i = 0; i < s.size(); i++) {
        surfaces[i] = Surface(PointXY(s[i].X1(), s[i].Y1()), PointXY(s[i].X2(), s[i].Y2()), true);
    }

    return DestinationToGo(myPathPlanner, surfaces, viewno);
}

Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno) {
    pair<double, double> mydest = myPathPlanner->getNextDestination(s);

    return Destination(mydest.first, mydest.second);
}

////will be used to reach a goal
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno,PointXY goal) {
//    vector<Surface> surfaces(s.size());
//
//    for (unsigned i = 0; i < s.size(); i++) {
//        surfaces[i] = Surface(PointXY(s[i].X1(), s[i].Y1()), PointXY(s[i].X2(), s[i].Y2()), true);
//    }
//
//    return DestinationToGo(myPathPlanner, surfaces, viewno,goal);
//}
//
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno,PointXY goal) {
//    pair<double, double> mydest = myPathPlanner->getNextDestination(s,goal);
//
//    return Destination(mydest.first, mydest.second);
//}
            


//goal.first is the anlge goal.second is the distance of goal
vector<Object> findExitToReachGoal(vector<Object> currentView, pair<double,double> goal, vector<Object> referenceObjects, int viewNumber) {
    cout<<"Goal angle and Distance: ";
//    cin>>goal.first>>goal.second;
    goal.first = -45;
    goal.second = 9000;
    MyRobot myrobot(0, 0);
    vector<Object> currentRobotPosition = myrobot.getRobot();
    vector<Object> destinationExitsInCV;
    destinationExitsInCV = findAllDestinationExits(currentView, referenceObjects);
//    vector<Exit> tempExits = findExits(currentView);
//    destinationExitsInCV = convertExitToObject(tempExits);
    Object robotLocationToExitMP;
    double destinationExitDirection;//angle
    double goalDirection = goal.first;//angle
    double goalX = goal.second * sin(-goal.first);//angle in radian
    double goalY = goal.second * cos(-goal.first);
    double exitToGoalDistance;
    
    Object destinationExit;
    
    if (destinationExitsInCV.size() > 1) {
        vector<Object> exitsOnLeftSide, exitsOnRightSide;
        //sort destination exits according to their orientation angle(angle btw -x axis and rp to exit midPoint object)
        for (unsigned int i = 0; i < destinationExitsInCV.size(); i++) {
            robotLocationToExitMP.set(0, 0, destinationExitsInCV[i].mpX(), destinationExitsInCV[i].mpY(), i);
            destinationExitDirection = currentRobotPosition[8].getAngleWithLine(robotLocationToExitMP);
            destinationExitsInCV[i].setOrt(destinationExitDirection);//setting angle
            destinationExitsInCV[i].setDistance(destinationExitsInCV[i].distMPToPoint(goalX, goalY));
            if(destinationExitDirection > -90) 
                exitsOnLeftSide.push_back(destinationExitsInCV[i]);
            else
                exitsOnRightSide.push_back(destinationExitsInCV[i]);
            cout << "Angle with -x Axis: " << destinationExitDirection << endl;
        }

        if (goalDirection < -90)//true means goal is on right side
            std::sort(destinationExitsInCV.begin(), destinationExitsInCV.end(), sortA2OrtAngleR2L); //sorting from right to left
        else//means goal is on left side
            std::sort(destinationExitsInCV.begin(), destinationExitsInCV.end(), sortA2OrtAngleL2R); //sorting from left to right

        cout << "Sorted Destination Exits" << endl;
        displayObjects(destinationExitsInCV);        
        cout<<"Exits on LEFT"<<endl;
        displayObjects(exitsOnLeftSide);
        cout<<"Exits on RIGHT"<<endl;
        displayObjects(exitsOnRightSide);

        //find the closest exit to reach this goal        
        
        if(goalDirection > 0) {//true means goal is on left side
            if(exitsOnLeftSide.size() > 0) {//true means there is at least one exit on left side
                std::sort(exitsOnLeftSide.begin(),exitsOnLeftSide.end(),sortA2Distance);
                destinationExit = exitsOnLeftSide[0];
            }
            else {//true means there is no exits on left side , so pick the first one from right side 
                std::sort(exitsOnRightSide.begin(),exitsOnRightSide.end(),sortA2OrtAngleL2R);
                destinationExit = exitsOnRightSide[0];
            }
        }
        else {//true means goal is on right side
            if(exitsOnRightSide.size() > 0) {//true means these is at least one exit on right side, pick the closet to goal
                std::sort(exitsOnRightSide.begin(),exitsOnRightSide.end(),sortA2Distance);
                destinationExit = exitsOnRightSide[0];
            }
            else {//true means there is no exits on right side, so pick the first one from left side
                std::sort(exitsOnLeftSide.begin(),exitsOnLeftSide.end(),sortA2OrtAngleR2L);
                destinationExit = exitsOnLeftSide[0];
            }
        }
    } else if(destinationExitsInCV.size() == 1){//true means there is only one exit
        destinationExit = destinationExitsInCV[0];
    }
    
    Object temp;
    temp = makeLineAtPointWithObject(goal.first,goal.second,500,currentRobotPosition[6]);
    currentRobotPosition.push_back(temp);
    currentRobotPosition.push_back(destinationExit);

    char viewFileName[80];
    sprintf(viewFileName, "%s%d%s", "Maps/view-", viewNumber, ".png");
    plotObjectsOf3Kinds(viewFileName,destinationExitsInCV,currentRobotPosition,currentView);
//    waitHere();
    return destinationExitsInCV;
}

vector<Object> findGapsForGoalExit(vector<Object> currentView, 
                                                        vector<Object> referenceObjects, 
                                                        Object goalExit,
                                                        int viewNumber) {
    cout <<endl<< "Finding gaps to reach the Goal" << endl;
    vector<Object> goalGaps;

    vector<Object> finalDestination;
    finalDestination.push_back(goalExit);
    finalDestination.push_back(makeLineAtPointWithObject(-90, 0, 500, Object(goalExit.mpX(), goalExit.mpY(), goalExit.X2(), goalExit.Y2(), 1)));
    Object originToGoal = Object(0, 0, finalDestination[1].X2(), finalDestination[1].Y2());
    
    //finding destination exits    
    vector<Object> destinationExitsInCV;// = convertExitToObject(findShortestExits(currentView));
    destinationExitsInCV = findAllDestinationExits(currentView,referenceObjects);

    if (destinationExitsInCV.size() > 0) {
        cout<<"Finding nearest gap"<<endl;
        double gapToGoalDist = destinationExitsInCV[0].distMPToPoint(finalDestination[1].X2(),finalDestination[1].Y2());
        Object gapToGo = destinationExitsInCV[0];
        for (unsigned int i = 0; i < destinationExitsInCV.size(); i++) {
            if(destinationExitsInCV[i].length() > 1000)
            if(gapToGoalDist > destinationExitsInCV[i].distMPToPoint(finalDestination[1].X2(),finalDestination[1].Y2()))
                gapToGo = destinationExitsInCV[i];
        }
        //its a 90degree line on the gapToGoal
        Object goalObject = makeLineAtPointWithObject(-90, 0, 500, Object(gapToGo.mpX(), gapToGo.mpY(), gapToGo.X2(), gapToGo.Y2(), 1));
        goalObject.reverse();
        goalGaps.push_back(goalObject);
    }
    
    char viewFileName[100];
    sprintf(viewFileName, "%s%d%s", "Maps/nextDestination-", viewNumber, ".png");
    //plotObjectsOf4Kinds(viewFileName, currentView, destinationExitsInCV,goalGaps, finalDestination);

    return goalGaps;
}

pair<double,double> findCurrentGoal(vector<Object> goalGap) {
    cout<<"Finding next GapGoal"<<endl;
    MyRobot robot(0, 0);
    pair<double,double> goal;
    cout<<"angle: "<<goal.first<<" Dist: "<<goal.second<<" goalGap Size: "<<goalGap.size()<<endl;
    Object temp(0,0,goalGap[0].mpX(),goalGap[0].mpY(),1);
    
    goal.first = robot.getRobot()[6].getAngleWithLine(temp);
    goal.second = temp.length();
    
    return goal;
}

bool isThePathClear(vector<Object> currentView, Object goalExit) {
    
    bool clear = true;
    vector<Object> finalDestination;
    finalDestination.push_back(goalExit);
    finalDestination.push_back(makeLineAtPointWithObject(-90, 0, 500, Object(goalExit.mpX(), goalExit.mpY(), goalExit.X2(), goalExit.Y2(), 1)));
    Object originToGoal = Object(0, 0, finalDestination[1].X2(), finalDestination[1].Y2());
    cout<<"HERE"<<endl;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        if (checkForIntersection(originToGoal, currentView[i]) == 1) {
            cout << "There is obstacle(s) on the path." << endl;
            clear = false;
        }
    }
    
    return clear;
}

Object recognizeGoalExit(vector<Object> exits, Object goalExit) {
    double mpTompDistance;
    Object recognizedExitInCV;
    for(unsigned int i=0; i<exits.size(); i++) {
        mpTompDistance = goalExit.distMPToPoint(exits[i].mpX(),exits[i].mpY());
        if(mpTompDistance < 1000) {
            recognizedExitInCV = exits[i];
            recognizedExitInCV.setID(100);
            cout<<"One Exit from CV has been recognized as goalExit"<<endl;
        }
    }
    
    return recognizedExitInCV;
}

vector<Exit> findShortestGap(vector<Object> cv) {
    cout << endl << "******* Finding exits(modified version of Yeap's theory) ******* " << endl;
    for (unsigned int i = 0; i < cv.size(); i++) {
        cv[i].setID(i + 1);
    }

    Exit exit;
    vector<Exit> exits;


    Point gapPoint2;
    int exits_counter = 1;
    for (int i = 0; i<int(cv.size()); i++) {
        //if (cv[i].distP2ToP1(cv[i+1]) > 600) {
        if (cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
            exit.set(cv[i].X2(), cv[i].Y2(), cv[i + 1].X1(), cv[i + 1].Y1());
            for (int j = i+1; j<int(cv.size()); j++) {
               
                gapPoint2 = cv[j].shortestDistPointFrom(cv[i].X2(), cv[i].Y2());
                if (cv[i].distP2ToPoint(gapPoint2.X(), gapPoint2.Y()) < exit.length()) {
                    exit.set(cv[i].X2(), cv[i].Y2(), gapPoint2.X(), gapPoint2.Y());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                }          

            }//for j
            if (exit.length() > 600) {//filter out small gaps
                exit.setID(exits_counter);
                exits.push_back(exit);
                exits_counter++;
            }
        }//if cv i
//        if (cv[i].getPEP1() == true) { //p2 is a probable exit end(p1)
//            exit.set(cv[i].X1(), cv[i].Y1(), cv[i + 1].X1(), cv[i + 1].Y1());
//            for (int j = i + 1; j<int(cv.size()); j++) {
//                gapPoint2 = cv[j].shortestDistPointFrom(cv[i].X2(), cv[i].Y2());
//                if (cv[i].distP2ToPoint(gapPoint2.X(), gapPoint2.Y()) < exit.length()) {
//                    exit.set(cv[i].X2(), cv[i].Y2(), gapPoint2.X(), gapPoint2.Y());
//                    exit.setP1ID(cv[i].getID());
//                    exit.setP2ID(cv[j].getID());
//                }
//
//            }//for j
//            exit.setID(exits_counter);
//            exits.push_back(exit);
//            exits_counter++;
//        }//if cv i
    }// for i


    //    cout << "shortest exits" << endl;
    //    displayExits(exits);
    return exits;
}

vector<Point> findPathToReachGoal(vector<ASR> places) {
    cout << endl << endl << "Finding Route to Reach The Exit Goal" << endl;
    vector<Point> path;
    MyRobot myRobot(0, 0);
    vector<Object> leftSideObjects, rightSideObjects;
    vector<Object> leftBoundary, rightBoundary;
    vector<Object> gapOnLeftBoundary, gapOnRightBoundary;
    vector<Object> exit;
    vector<Object> allObjects;
    vector<Object> gaps;
    vector<Object> allParalleLines;
    vector<Point> shortestPath;
    vector<Object> route;
    vector<Object> initialRoute;
    
    Object tempGap;

    char viewFileName[100];
    Object gap;
    Point gapPoint;
    for (unsigned int p = 0; p < 5; p++) {
        allObjects = places[p].getASRObjects();
        
        exit = makeSquare(places[p].getASRExit1());

        for (unsigned int i = 0; i < allObjects.size(); i++) {
            if (allObjects[i].getPos() == -1) {
                leftSideObjects.push_back(allObjects[i]);
            } else
                rightSideObjects.push_back(allObjects[i]);
        }       
        
        
        
        //two vectors(left and right) for boundary        
        //leftside
        //exit1 to p1 of first object
        leftBoundary.push_back(Object(exit[1].mpX(), exit[1].mpY(), leftSideObjects[0].X1(), leftSideObjects[0].Y1()));
        
        for (unsigned int k = 0; k < leftSideObjects.size() - 1; k++) {
            leftBoundary.push_back(leftSideObjects[k]);
            tempGap = Object(leftSideObjects[k].X2(), leftSideObjects[k].Y2(),leftSideObjects[k + 1].X1(), leftSideObjects[k + 1].Y1());
//            for(unsigned int k2=k+1;k2<leftSideObjects.size();k2++) {
//                gapPoint = leftSideObjects[k2].shortestDistPointFrom(tempGap.X1(),tempGap.Y1());
//                if( tempGap.distP1ToPoint(gapPoint.X(),gapPoint.Y())< tempGap.length()) {
//                    tempGap.setP2(gapPoint.X(),gapPoint.Y());
//                }
//            }
            
            if(tempGap.length() > 600) {      
                tempGap = Object(leftSideObjects[k].X2(), leftSideObjects[k].Y2(),leftSideObjects[k + 1].X1(), leftSideObjects[k + 1].Y1());
                gapOnLeftBoundary.push_back(tempGap);
            }
            else
            leftBoundary.push_back(tempGap);
        }
        leftBoundary.push_back(leftSideObjects[leftSideObjects.size() - 1]);
        
        //rightside
        for (unsigned int k = 0; k < rightSideObjects.size() - 1; k++) {
            rightBoundary.push_back(rightSideObjects[k]);
            tempGap = Object(rightSideObjects[k].X2(), rightSideObjects[k].Y2(),rightSideObjects[k + 1].X1(), rightSideObjects[k + 1].Y1());
//            for(unsigned int k2=k+1;k2<rightSideObjects.size();k2++) {
//                gapPoint = rightSideObjects[k2].shortestDistPointFrom(tempGap.X1(),tempGap.Y1());
//                if( tempGap.distP1ToPoint(gapPoint.X(),gapPoint.Y())< tempGap.length()) {
//                    tempGap.setP2(gapPoint.X(),gapPoint.Y());
//                }
//            }
            
            if(tempGap.length() > 600) {       
                tempGap = Object(rightSideObjects[k].X2(), rightSideObjects[k].Y2(),rightSideObjects[k + 1].X1(), rightSideObjects[k + 1].Y1());
                gapOnRightBoundary.push_back(tempGap);
            }
            else
            rightBoundary.push_back(tempGap);
        }
        rightBoundary.push_back(rightSideObjects[rightSideObjects.size() - 1]);
        
        //mpoint of last object to exit1
        rightBoundary.push_back(Object(rightSideObjects[rightSideObjects.size() - 1].X2(), rightSideObjects[rightSideObjects.size() - 1].Y2(), exit[3].mpX(), exit[3].mpY()));
        
        
        //shortest route/path following left boundary
        for(unsigned int i=0;i<leftSideObjects.size();i++) {
        allParalleLines.push_back(makeParallelObject(leftSideObjects[i],600,'right'));
        allParalleLines.back().setID(i+1);
        }
        for(unsigned int i=0;i<allParalleLines.size();i++) {
            
        }
        
        vector<Object> allObstacles;
        allObstacles = addTwoVectorsOfObjects(allObstacles,leftBoundary);
        allObstacles = addTwoVectorsOfObjects(allObstacles,rightBoundary);
        allObstacles = addTwoVectorsOfObjects(allObstacles,gapOnLeftBoundary);
        allObstacles = addTwoVectorsOfObjects(allObstacles,gapOnRightBoundary);
        
        Point lastPoint,tempPoint,nextPoint;
        shortestPath.push_back(Point(places[p].getASRExit1().mpX(),places[p].getASRExit1().mpY()));
        Object tempPath;
        int clear =1;
        //waitHere();
        lastPoint.set(exit[2].mpX(),exit[2].mpY());
//        tempPoint.set(allParalleLines[0].X1(),allParalleLines[0].Y1());
//        tempPath.set(lastPoint.X(),lastPoint.Y(),tempPoint.X(),tempPoint.Y(),1);
        shortestPath.push_back(lastPoint);
        
        //while(checkForIntersection(places[p].getASRExit2(),route.back()) != 1) {
        int forBug;
        
        
        tempPath.set(lastPoint.X(), lastPoint.Y(),places[p].getASRExit2().mpX(),places[p].getASRExit2().mpY(),1);
        clear = 1;
                for (unsigned int j = 0; j < allObstacles.size(); j++) {
                    if (checkForIntersection(tempPath, allObstacles[j]) == 1) {
                        clear = 2;
                        break;
                    }
                }
        if(clear ==1) {//true means exit is visible. 
            route.push_back(Object(lastPoint.X(),lastPoint.Y(),places[p].getASRExit2().mpX(),places[p].getASRExit2().mpY()));
        }
        else { //need to follow boundry to go to exit
        route.push_back(Object(lastPoint.X(),lastPoint.Y(),lastPoint.X(),lastPoint.Y()));
        while(checkForIntersection(route.back(),places[p].getASRExit2()) != 1 and places[p].getASRExit2().distMPToPoint(lastPoint.X(),lastPoint.Y()) > 2000) {
        //for(int ii=0;ii<10;ii++) {
            for (unsigned int i = 0; i < allParalleLines.size(); i++) {
                tempPoint.set(allParalleLines[i].X1(), allParalleLines[i].Y1());
                tempPath.set(lastPoint.X(), lastPoint.Y(), tempPoint.X(), tempPoint.Y(), 1);
                clear = 1;
                for (unsigned int j = 0; j < allObstacles.size(); j++) {
                    if (checkForIntersection(tempPath, allObstacles[j]) == 1) {
                        //lastPoint.set(allParalleLines[i-1].X1(),allParalleLines[i-1].Y1());
                        //shortestPath.push_back(lastPoint);
                        clear = 2;
                        break;
                    }
                }
                
                if (clear == 1 ) {
                    cout << " " << i;
                    nextPoint.set(allParalleLines[i].X1(), allParalleLines[i].Y1());
                    forBug = i;
                }
                

            }
            if(forBug == 55)
                nextPoint.set(allParalleLines[51].X1(), allParalleLines[51].Y1());
            route.push_back(Object(lastPoint.X(), lastPoint.Y(), nextPoint.X(), nextPoint.Y()));
            lastPoint = nextPoint;
//            cout<<"p: "<<p<<endl;
//        waitHere();
        }
        }
       
//        for(unsigned int i=0;i<shortestPath.size()-1;i++) {
//            route.push_back(Object(shortestPath[i].X(),shortestPath[i].Y(),shortestPath[i+1].X(),shortestPath[i+1].Y()));
//        }
        
        
        sprintf(viewFileName, "%s%d%s", "Maps/placeWithBoundaryAndGaps-", p, ".png");
        plotObjectsOf3Kinds(viewFileName, route, leftBoundary,rightBoundary);
        // plotObjectsOf4Kinds(viewFileName, allParalleLines,route, leftBoundary,rightBoundary);
         
        
        leftBoundary.clear();
        rightBoundary.clear();
        gapOnLeftBoundary.clear();
        gapOnRightBoundary.clear();
        shortestPath.clear();
        allParalleLines.clear();
        route.clear();
        shortestPath.clear();

        //single vector of boundary
        for (unsigned int i = 0; i < leftSideObjects.size(); i++) {
            //if(leftSideObjects[i].getPEP2() == true) {
            gap.set(leftSideObjects[i].X2(), leftSideObjects[i].Y2(), rightSideObjects[0].X1(), rightSideObjects[0].Y1(), 1);
            for (unsigned int j = 0; j < rightSideObjects.size(); j++) {
                gapPoint = rightSideObjects[j].shortestDistPointFrom(leftSideObjects[i].X2(), leftSideObjects[i].Y2());
                if (leftSideObjects[i].distP2ToPoint(gapPoint.X(), gapPoint.Y()) < gap.length()) {
                    gap.setP2(gapPoint.X(), gapPoint.Y());
                }
            }
            
//            for(unsigned int k=0;k<leftSideObjects.size();k++) {
//                gapPoint = leftSideObjects[k].shortestDistPointFrom(gap.X2(),gap.Y2());
//                if(gap.distP2ToPoint(gapPoint.X(),gapPoint.Y()))
//            }

            if (gaps.size() > 0) {
                if (gaps.back().distMPToPoint(gap.mpX(), gap.mpY()) > 3000) //filter out of close gaps
                    gaps.push_back(gap);
            } else
                gaps.push_back(gap);
            //}
        }
        
        
        sprintf(viewFileName, "%s%d%s", "Maps/placeWithGaps-", p, ".png");
        plotObjects(viewFileName, gaps, allObjects);
        gaps.clear();
        leftSideObjects.clear();
        rightSideObjects.clear();
               
    }



    //find the wall

    return path;
}

///****************************************************************///

Point crossedPointReturnP(Point p1, Point p2, Point q1, Point q2)
{
            Point temp;
            double tmpLeft,tmpRight;
            double x, y;
            double temp_distTOrig;
            tmpLeft = (q2.X() - q1.X()) * (p1.Y() - p2.Y()) - (p2.X() - p1.X()) * (q1.Y() - q2.Y());
            tmpRight = (p1.Y() - q1.Y()) * (p2.X() - p1.X()) * (q2.X() - q1.X()) + q1.X() * (q2.Y() - q1.Y()) * (p2.X() - p1.X()) - p1.X() * (p2.Y() - p1.Y()) * (q2.X() - q1.X());

            x = tmpRight / tmpLeft;

            tmpLeft = (p1.X() - p2.X()) * (q2.Y() - q1.Y()) - (p2.Y() - p1.Y()) * (q1.X() - q2.X());
            tmpRight = p2.Y() * (p1.X() - p2.X()) * (q2.Y() - q1.Y()) + (q2.X()- p2.X()) * (q2.Y() - q1.Y()) * (p1.Y() - p2.Y()) - q2.Y() * (q1.X() - q2.X()) * (p2.Y() - p1.Y()); 
            y = tmpRight / tmpLeft;
            
            temp.set(x,y);

            return temp;
}


//** detect whether two line segments are intersected **//
bool betweenForReturn(double a, double X0, double X1)  
{  
        double temp1 = a-X0;  
        double temp2 = a-X1;  
        if ( ( temp1 < 1e-8 && temp2 > -1e-8 ) || ( temp2 < 1e-6 && temp1 > -1e-8 ))  
        {  
            return true;  
        }  
        else  
        {  
            return false;  
        }  
}  
  
 
Object IntersectForReturn(vector<Object> CurrentV, Point Pos)  
{  
    cout << "***** This is going to a process two line segments are intersected *****" << endl << endl;

        Point p1, p2, p3, p4;
        double line_x, line_y; //intersect position  
        Object temp;
 
        
        p3.set(0.0, 0.0); // origin point (0,0)
        p4.set(Pos.X(), Pos.Y());
        
        unsigned char inter_Flag = 0;
       
        for(int i = 0; i < CurrentV.size(); i++)
        {        
                p1.set(CurrentV[i].X1(), CurrentV[i].Y1());
                p2.set(CurrentV[i].X2(), CurrentV[i].Y2());
                
                if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                {  
                    //return false;  
                    inter_Flag = 0;
                }  
                else if ( (fabs(p1.X()-p2.X())<1e-6) )
                {  
                    if (betweenForReturn(p1.X(),p3.X(),p4.X()))  
                    {  
                            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                            line_x = p1.X();  
                            line_y = k*(line_x-p3.X())+p3.Y();  

                            if (betweenForReturn(line_y,p1.Y(),p2.Y()))  
                            {  
                                 
                                inter_Flag = 1;
                                //return true;
                                break;
                            }  
                            else  
                            {  
                                //return false;  
                                inter_Flag = 0;
                            }  
                    }  
                    else   
                    {  
                            //return false;  
                            inter_Flag = 0;
                    }  
                }  
                else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                {  
                    if (betweenForReturn(p3.X(),p1.X(),p2.X()))  
                    {  
                        double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                        line_x = p3.X();  
                        line_y = k*(line_x-p2.X())+p2.Y();  

                        if (betweenForReturn(line_y,p3.Y(),p4.Y()))  
                        {  
                            
                            inter_Flag = 1;
                            //return true;  
                            break;
                        }  
                        else  
                        {  
                            //return false;  
                            inter_Flag = 0;
                        }  
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                }  
                else  
                {  
                    double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                    double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                    if (fabs(k1-k2)<1e-6)  
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                    else   
                    {  
                        line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                        line_y = k1*(line_x-p1.X())+p1.Y();  
                    }  

                    if (betweenForReturn(line_x,p1.X(),p2.X())&&betweenForReturn(line_x,p3.X(),p4.X()))  
                    {  
                        
                        inter_Flag = 1;
                        //return true;
                        break;
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                } 
                
        }
        
        if(inter_Flag == 0)
            temp.set(0,0,0,0,0);
        if(inter_Flag == 1)
            temp.set(p1.X(),p1.Y(),p2.X(),p2.Y(), 1);
             
        return temp;
}

//p1 and p2 are line 1, p3 and p4 are line 2
bool twoLinintersect(Point p1, Point p2, Point p3, Point p4)  
{  
    double line_x,line_y; //交点  
    if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
    {  
        return false;  
    }  
    else if ( (fabs(p1.X()-p2.X())<1e-6) ) 
    {  
        if (betweenForReturn(p1.X(),p3.X(),p4.X()))  
        {  
            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
            line_x = p1.X();  
            line_y = k*(line_x-p3.X())+p3.Y();  
  
            if (betweenForReturn(line_y,p1.Y(),p2.Y()))  
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
        if (betweenForReturn(p3.X(),p1.X(),p2.X()))  
        {  
            double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
            line_x = p3.X();  
            line_y = k*(line_x-p2.X())+p2.Y();  
  
            if (betweenForReturn(line_y,p3.Y(),p4.Y()))  
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
  
        if (betweenForReturn(line_x,p1.X(),p2.X())&&betweenForReturn(line_x,p3.X(),p4.X()))  
        {  
            return true;  
        }  
        else   
        {  
            return false;  
        }  
    }  
}

bool TwoObjectIntersect(Object obj1, Object obj2)
{
           Point p1, p2, p3, p4;
           p1.set(obj1.X1(), obj1.Y1());
           p2.set(obj1.X2(), obj1.Y2());
           p3.set(obj2.X1(), obj2.Y1());
           p4.set(obj2.X2(), obj2.Y2());

           if(twoLinintersect(p1, p2, p3, p4) == true)
                      return true;
           else
                      return false;
}

bool interSectWithPtoOrig(vector<Object> CV, Point pos)
{
    Object temp;
    temp = IntersectForReturn(CV, pos);
    if(temp.getID() == 0)
        return false;
    else
        if(temp.getID() == 1)
            return true;
}

bool interSectWithLine(vector<Object> CV, Point p1, Point p2)
{
    Point p3, p4;
    
    int flag = 0;
    for(int i = 0; i < CV.size(); i++)
    {
        p3.set(CV[i].X1(), CV[i].Y1());
        p4.set(CV[i].X2(), CV[i].Y2());
        
        if(twoLinintersect(p1, p2, p3, p4) == true)
            if((p1.operator ==(p3) || p1.operator ==(p4)) //share common endpoints
                    || (p2.operator ==(p3) || p2.operator ==(p4)))
                flag = 1;
            else
                return true;
        if(twoLinintersect(p1, p2, p3, p4) == false)
            flag = 1;
    }
    
    if(flag == 1)
        return false;
    
}

Point intersectedPoint(vector<Object> CV, Point p3, Point p4)
{
    //Object temp_Obj;
    Point rnt;
    Point p1, p2;
        double line_x, line_y; //intersect position  
        Object temp;
 
        
        //p3.set(0.0, 0.0); // origin point (0,0)
        //p4.set(Pos.X(), Pos.Y());
        
        unsigned char inter_Flag = 0;
       
        for(int i = 0; i < CV.size(); i++)
        {        
                p1.set(CV[i].X1(), CV[i].Y1());
                p2.set(CV[i].X2(), CV[i].Y2());
                
                if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                {  
                    //return false;  
                    inter_Flag = 0;
                }  
                else if ( (fabs(p1.X()-p2.X())<1e-6) )
                {  
                    if (betweenForReturn(p1.X(),p3.X(),p4.X()))  
                    {  
                            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                            line_x = p1.X();  
                            line_y = k*(line_x-p3.X())+p3.Y();  

                            if (betweenForReturn(line_y,p1.Y(),p2.Y()))  
                            {  
                                 
                                inter_Flag = 1;
                                //return true;
                                break;
                            }  
                            else  
                            {  
                                //return false;  
                                inter_Flag = 0;
                            }  
                    }  
                    else   
                    {  
                            //return false;  
                            inter_Flag = 0;
                    }  
                }  
                else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                {  
                    if (betweenForReturn(p3.X(),p1.X(),p2.X()))  
                    {  
                        double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                        line_x = p3.X();  
                        line_y = k*(line_x-p2.X())+p2.Y();  

                        if (betweenForReturn(line_y,p3.Y(),p4.Y()))  
                        {  
                            
                            inter_Flag = 1;
                            //return true;  
                            break;
                        }  
                        else  
                        {  
                            //return false;  
                            inter_Flag = 0;
                        }  
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                }  
                else  
                {  
                    double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                    double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                    if (fabs(k1-k2)<1e-6)  
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                    else   
                    {  
                        line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                        line_y = k1*(line_x-p1.X())+p1.Y();  
                    }  

                    if (betweenForReturn(line_x,p1.X(),p2.X())&&betweenForReturn(line_x,p3.X(),p4.X()))  
                    {  
                        
                        inter_Flag = 1;
                        //return true;
                        break;
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                } 
                
        }
        
        if(inter_Flag == 0)
            temp.set(0,0,0,0,0);
        if(inter_Flag == 1)
            temp.set(p1.X(),p1.Y(),p2.X(),p2.Y(), 1);
    
        
     rnt = crossedPointReturnP(Point (temp.X1(), temp.Y1()), Point (temp.X2(), temp.Y2()), p3, p4);
     
     return rnt;
}

/*
vector<Point> intersectedPoints(vector<Object> view1, vector<Object> view2)
{
    Point temp_point;
    vector<Point> points;
    
    for(int i = 0; i  < view2.size(); i++)
    {
        temp_point = intersectedPoint(view1, view2[i].getP1(), view2[i].getP2());
        if(temp_point.X() != 0 && temp_point.Y() != 0)
            points.push_back(temp_point);
    }
    
    return points;
}
*/
Point intersectedNearPoint(vector<Object> CV, Point p3, Point p4)
{
    //Object temp_Obj;
    Point rnt, temp_crs_point;
    Point p1, p2;
    vector<Point> Points_intersected;
    
        double line_x, line_y; //intersect position  
        Object temp;
 
        
        //cout << "P3 x: " << p3.X() << " y: " << p3.Y() << endl;
        //cout << "P4 x: " << p4.X() << " y: " << p4.Y() << endl;
        
        unsigned char inter_Flag = 0;
       
        for(int i = 0; i < CV.size(); i++)
        {        
                p1.set(CV[i].X1(), CV[i].Y1());
                p2.set(CV[i].X2(), CV[i].Y2());
                
                if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                {  
                    //return false;  
                    //inter_Flag = 0;
                }  
                else if ( (fabs(p1.X()-p2.X())<1e-6) )
                {  
                    if (betweenForReturn(p1.X(),p3.X(),p4.X()))  
                    {  
                            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                            line_x = p1.X();  
                            line_y = k*(line_x-p3.X())+p3.Y();  

                            if (betweenForReturn(line_y,p1.Y(),p2.Y()))  
                            {  
                                 
                                //inter_Flag = 1;
                                
                                temp.set(p1.X(),p1.Y(),p2.X(),p2.Y(), 1);
                                temp_crs_point = crossedPointReturnP(Point (temp.X1(), temp.Y1()), Point (temp.X2(), temp.Y2()), p3, p4);
                                Points_intersected.push_back(temp_crs_point);
                            }  
                            else  
                            {  
                                //return false;  
                                //inter_Flag = 0;
                            }  
                    }  
                    else   
                    {  
                            //return false;  
                            //inter_Flag = 0;
                    }  
                }  
                else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                {  
                    if (betweenForReturn(p3.X(),p1.X(),p2.X()))  
                    {  
                        double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                        line_x = p3.X();  
                        line_y = k*(line_x-p2.X())+p2.Y();  

                        if (betweenForReturn(line_y,p3.Y(),p4.Y()))  
                        {  
                            
                            //inter_Flag = 1;
                            temp.set(p1.X(),p1.Y(),p2.X(),p2.Y(), 1);
                            temp_crs_point = crossedPointReturnP(Point (temp.X1(), temp.Y1()), Point (temp.X2(), temp.Y2()), p3, p4);
                            Points_intersected.push_back(temp_crs_point);
                            
                        }  
                        else  
                        {  
                            //return false;  
                            //inter_Flag = 0;
                        }  
                    }  
                    else   
                    {  
                        //return false;  
                        //inter_Flag = 0;
                    }  
                }  
                else  
                {  
                    double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                    double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                    if (fabs(k1-k2)<1e-6)  
                    {  
                        //return false;  
                        //inter_Flag = 0;
                    }  
                    else   
                    {  
                        line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                        line_y = k1*(line_x-p1.X())+p1.Y();  
                    }  

                    if (betweenForReturn(line_x,p1.X(),p2.X())&&betweenForReturn(line_x,p3.X(),p4.X()))  
                    {  
                        
                        //inter_Flag = 1;
                        temp.set(p1.X(),p1.Y(),p2.X(),p2.Y(), 1);
                        temp_crs_point = crossedPointReturnP(Point (temp.X1(), temp.Y1()), Point (temp.X2(), temp.Y2()), p3, p4);
                        Points_intersected.push_back(temp_crs_point);
                    }  
                    else   
                    {  
                        //return false;  
                        //inter_Flag = 0;
                    }  
                } 
                
        }
        
        //cout << " interseced points size : " << Points_intersected.size() << endl;

        if(Points_intersected.size() != 0)
        {
            //double min = 10000;
            double nearest_dist = distanceOftwoP(Points_intersected[0], p4);
            rnt = Points_intersected[0];
            for(int i = 1; i < Points_intersected.size(); i++)
            {
                if(distanceOftwoP(Points_intersected[i], p4) < nearest_dist)
                {
                    nearest_dist = distanceOftwoP(Points_intersected[i], p4);
                    rnt = Points_intersected[0];
                }
            }
        }
        else
            rnt.set(0,0);

     return rnt;
}

Object intersectForObject(vector<Object> CurrentV, Point p3, Point p4)  
{  

        int id;
        Point p1, p2;
        double line_x, line_y; //intersect position  
        Object temp;
        
        unsigned char inter_Flag = 0;
       
        for(int i = 0; i < CurrentV.size(); i++)
        {        
                p1.set(CurrentV[i].X1(), CurrentV[i].Y1());
                p2.set(CurrentV[i].X2(), CurrentV[i].Y2());
                
                if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                {  
                    //return false;  
                    inter_Flag = 0;
                }  
                else if ( (fabs(p1.X()-p2.X())<1e-6) )
                {  
                    if (betweenForReturn(p1.X(),p3.X(),p4.X()))  
                    {  
                            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                            line_x = p1.X();  
                            line_y = k*(line_x-p3.X())+p3.Y();  

                            if (betweenForReturn(line_y,p1.Y(),p2.Y()))  
                            {  
                                 
                                inter_Flag = 1;
                                id = i;
                                //return true;
                                break;
                            }  
                            else  
                            {  
                                //return false;  
                                inter_Flag = 0;
                            }  
                    }  
                    else   
                    {  
                            //return false;  
                            inter_Flag = 0;
                    }  
                }  
                else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                {  
                    if (betweenForReturn(p3.X(),p1.X(),p2.X()))  
                    {  
                        double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                        line_x = p3.X();  
                        line_y = k*(line_x-p2.X())+p2.Y();  

                        if (betweenForReturn(line_y,p3.Y(),p4.Y()))  
                        {  
                            
                            inter_Flag = 1;
                            id = i;
                            //return true;  
                            break;
                        }  
                        else  
                        {  
                            //return false;  
                            inter_Flag = 0;
                        }  
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                }  
                else  
                {  
                    double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                    double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                    if (fabs(k1-k2)<1e-6)  
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                    else   
                    {  
                        line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                        line_y = k1*(line_x-p1.X())+p1.Y();  
                    }  

                    if (betweenForReturn(line_x,p1.X(),p2.X())&&betweenForReturn(line_x,p3.X(),p4.X()))  
                    {  
                        
                        inter_Flag = 1;
                        id = i;
                        //return true;
                        break;
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                } 
                
        }
        
        if(inter_Flag == 0)
            temp.set(0,0,0,0,0);
        if(inter_Flag == 1)
            temp.set(p1.X(),p1.Y(),p2.X(),p2.Y(), id);
             
        return temp;
}


vector<Object> intersectAllSurfaces(vector<Object> CurrentV, Point p3, Point p4)
{
    vector<Object> rnt;
    
    
    
        Point p1, p2;
        double line_x, line_y; //intersect position  
        Object temp;
        
        unsigned char inter_Flag = 0;
       
        for(int i = 0; i < CurrentV.size(); i++)
        {        
                p1.set(CurrentV[i].X1(), CurrentV[i].Y1());
                p2.set(CurrentV[i].X2(), CurrentV[i].Y2());
                
                if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                {  
                    //return false;  
                    inter_Flag = 0;
                }  
                else if ( (fabs(p1.X()-p2.X())<1e-6) )
                {  
                    if (betweenForReturn(p1.X(),p3.X(),p4.X()))  
                    {  
                            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                            line_x = p1.X();  
                            line_y = k*(line_x-p3.X())+p3.Y();  

                            if (betweenForReturn(line_y,p1.Y(),p2.Y()))  
                            {  
                                 
                                inter_Flag = 1;
                                rnt.push_back(CurrentV[i]);  
                                //break;
                            }  
                            else  
                            {  
                                //return false;  
                                inter_Flag = 0;
                            }  
                    }  
                    else   
                    {  
                            //return false;  
                            inter_Flag = 0;
                    }  
                }  
                else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                {  
                    if (betweenForReturn(p3.X(),p1.X(),p2.X()))  
                    {  
                        double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                        line_x = p3.X();  
                        line_y = k*(line_x-p2.X())+p2.Y();  

                        if (betweenForReturn(line_y,p3.Y(),p4.Y()))  
                        {  
                            
                            inter_Flag = 1;
                            rnt.push_back(CurrentV[i]);  
                            //break;
                        }  
                        else  
                        {  
                            //return false;  
                            inter_Flag = 0;
                        }  
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                }  
                else  
                {  
                    double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                    double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                    if (fabs(k1-k2)<1e-6)  
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                    else   
                    {  
                        line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                        line_y = k1*(line_x-p1.X())+p1.Y();  
                    }  

                    if (betweenForReturn(line_x,p1.X(),p2.X())&&betweenForReturn(line_x,p3.X(),p4.X()))  
                    {  
                        
                        inter_Flag = 1;
                        rnt.push_back(CurrentV[i]);  
                        //break;
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                } 
                
        }
        

             
        return rnt;
    
}

int Intersect_number(vector<Object> view, Object line)
{
    int cnter = 0;
    
    for(int i = 0; i < view.size(); i++)
    {
        if((TwoObjectIntersect(view[i], line) == true)
            &&(view[i].getP1().operator !=(line.getP1()))
            &&(view[i].getP2().operator !=(line.getP2())))
            cnter++;
    }
        
        
    return cnter;
}



double distanceOftwoP(Point a, Point b)
{
    return sqrt((a.X()-b.X()) * (a.X()-b.X()) + (a.Y() - b.Y()) * (a.Y() - b.Y()));
}


bool ObstacleFront(vector<Object> CurrentV, double distance)
{
        Point p1, p2, p3, p4;
        double line_x, line_y; //intersect position  
 
        
        p3.set(0.0, 0.0); // origin point (0,0)
        p4.set(0.0, distance);
        
        unsigned char inter_Flag = 0;
       
        for(int i = 0; i < CurrentV.size(); i++)
        {        
                p1.set(CurrentV[i].X1(), CurrentV[i].Y1());
                p2.set(CurrentV[i].X2(), CurrentV[i].Y2());
                
                if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                {  
                    inter_Flag = 0;
                }  
                else if ( (fabs(p1.X()-p2.X())<1e-6) )
                {  
                    if (betweenForReturn(p1.X(),p3.X(),p4.X()))  
                    {  
                            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                            line_x = p1.X();  
                            line_y = k*(line_x-p3.X())+p3.Y();  

                            if (betweenForReturn(line_y,p1.Y(),p2.Y()))  
                            {  
                                 
                                inter_Flag = 1;
                                return true; 
                            }  
                            else  
                            {   
                                inter_Flag = 0;
                            }  
                    }  
                    else   
                    {  
                            inter_Flag = 0;
                    }  
                }  
                else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                {  
                    if (betweenForReturn(p3.X(),p1.X(),p2.X()))  
                    {  
                        double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                        line_x = p3.X();  
                        line_y = k*(line_x-p2.X())+p2.Y();  

                        if (betweenForReturn(line_y,p3.Y(),p4.Y()))  
                        {  
                            
                            inter_Flag = 1;
                            return true;  
                        }  
                        else  
                        {  
                            inter_Flag = 0;
                        }  
                    }  
                    else   
                    {  
                        inter_Flag = 0;
                    }  
                }  
                else  
                {  
                    double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                    double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                    if (fabs(k1-k2)<1e-6)  
                    {  
                        inter_Flag = 0;
                    }  
                    else   
                    {  
                        line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                        line_y = k1*(line_x-p1.X())+p1.Y();  
                    }  

                    if (betweenForReturn(line_x,p1.X(),p2.X())&&betweenForReturn(line_x,p3.X(),p4.X()))  
                    {  
                        
                        inter_Flag = 1;
                        return true;  
                    }  
                    else   
                    {  
                        inter_Flag = 0;
                    }  
                } 
                
        }
        
        if(inter_Flag == 0)
            return false;
}


bool comPathView(vector<int> Cu, vector<int> Me)
{
    int a = Cu.size();
    int b = Me.size();
    int min;
    
    if(a > b)
    {
        min = a - b;
        Cu.erase(Cu.begin(), Cu.begin() + min);
        if(Cu == Me)
            return true;
        else
            return false;
    }
    if(a < b)
    {
        min = b - a;
        Me.erase(Me.begin(), Me.begin() + min);
        if(Cu == Me)
            return true;
        else
            return false;
    }

}

bool matchObjPos(vector<int> mem, vector<int> cur)
{
    int mem_pos[mem.size()];
    int cur_pos[cur.size()];
    int cnt = 0;
    int num = 0;
    
    for(int i = 0; i < mem.size(); i++)
        mem_pos[i] = mem[i];
    for(int i = 0; i < cur.size(); i++)
        cur_pos[i] = cur[i]; 
    
    if(cur.size() > mem.size())
    {
        for(int i = 0; i < mem.size(); i ++)
        {
            for(int j = 0; j < cur.size(); j++)
            {
                if(mem_pos[i] == cur_pos[j])
                {
                    cnt++;
                    break;
                }
            }
        }
    }
    
    cout << "number of count "<<cnt<<endl;
    
    if(cnt == mem.size())
        return true;
    else
        return false;
}


vector<Exit> findGapasExits(vector<Object> cv) 
{
    Exit exit;
    vector<Exit> exits;
    double threshold = 1000;
    double dist_PtP = 0;
    
    cout << endl << "******* Finding exits(modified version of Yeap's theory) ******* " << endl;
   for (unsigned int i = 0; i < cv.size(); i++) 
    {   
        cv[i].setID(i + 1);  
    }
 
    for (int i = 0; i < cv.size(); i++) 
    {
            if(i != cv.size() - 1)
            {
                    dist_PtP = sqrt((cv[i].X2() - cv[i+1].X1()) * (cv[i].X2() - cv[i+1].X1()) + (cv[i].Y2() - cv[i+1].Y1()) * (cv[i].Y2() - cv[i+1].Y1()));
                   //dist_PtP = cv[i].distP1ToP2(cv[i+1]);
                    if(dist_PtP >= threshold) 
                    {
                        exit.set(cv[i].X2(), cv[i].Y2(), cv[i+1].X1(), cv[i+1].Y1());
                        exit.setP1ID(cv[i].getID());
                        exit.setP2ID(cv[i+1].getID());

                        exits.push_back(exit);
                    }
            }
    }
 
    
    return exits;
}


//transform the global positions into relative positions
vector<Point> returningPositions(vector<Point> global, vector<double> facing)
{
        vector<Point> rnt;
        Point temp;
        double x, y;
        double dist, angle;
        double rad;

        for(int i = 0; i < global.size() - 1; i++)
        {
                rad = facing[i+1] * ( PI / 180);
                angle = PI + rad;
                //angle = rad;
                if(angle > 2 * PI)
                    angle = angle - 2 * PI;
                
                dist = GetPointDistance(global[i], global[i+1]);

                double a = global[i].X() - global[i+1].X(); //x-rp
                double b = global[i].Y() - global[i+1].Y(); //y-rp

                x = a * cos(angle) + b * sin(angle);
                y = b * cos(angle) - a * sin(angle);
    
                temp.set(x, y);
                rnt.push_back(temp);
        }

        return rnt;
}



vector<Object> routeInView(vector<Object> CV, Point pos)
{
        Point inter(0, pos.Y());
        Point a, b, c;
        Object interObj1, interObj2;
        Object part1, part2;
        vector<Object> route;

        interObj1 = IntersectForReturn(CV, pos) ;
        interObj2 = IntersectForReturn(CV, inter) ;
        
        if(interObj2.getID() != 0) // intersected with a boundary
        {
                if(pos.X() < 0 && pos.X() < interObj2.X1())
                {
                    if(pos.Y() > interObj2.Y1())
                        a.set(interObj2.X1() - 400, interObj2.Y1());
                    else
                        a.set(interObj2.X1() - 400, pos.Y());
                }
                else
                    if(pos.X() > 0 && pos.X() > interObj2.X2())
                    {
                        if(pos.Y() > interObj2.Y2())
                            a.set(interObj2.X2() + 400, interObj2.Y2());
                        else
                            a.set(interObj2.X2() + 400, pos.Y());
                    }
        }
        else // not intersected with a boundary
        {
             a.set(0, pos.Y());
        }
        
        part1.set(0, 0, a.X(), a.Y(), 1);
        part2.set(a.X(), a.Y(), pos.X(), pos.Y(), 2);
        
        route.push_back(part1);
        route.push_back(part2);
        
        return route;
}

//***** for new view simplification algorithm *****//
vector<Point> CollectPoints(vector<Object> view)
{
        vector<Point> allPoints;
        Point temp_point;

        temp_point.setWithID(view[0].X1(), view[0].Y1(), 0);
        allPoints.push_back(temp_point);
        temp_point.setWithID(view[0].X2(), view[0].Y2(), 0);
        allPoints.push_back(temp_point);
        
        for(int i = 1; i < view.size(); i++)
        {
                if(view[i].X1() == view[i-1].X2() && view[i].Y1() == view[i-1].Y2())
                {
                        temp_point.setWithID(view[i].X2(), view[i].Y2(), i);
                        allPoints.push_back(temp_point);
                }
                else
                {
                        temp_point.setWithID(view[i].X1(), view[i].Y1(), i);
                        allPoints.push_back(temp_point);
                        temp_point.setWithID(view[i].X2(), view[i].Y2(), i);
                        allPoints.push_back(temp_point);
                }
        }

        return allPoints;
}

bool isObject(vector<Object> view, Point p1, Point p2)
{
  
    Point Op1, Op2;
    unsigned char flag = 0;

    //cout<<"----This is going to matching points----"<<endl;
    //cout<<" P1 x: "<<p1.X()<<"; y: "<<p1.Y()<<endl;
    //cout<<" P2 x: "<<p2.X()<<"; y: "<<p2.Y()<<endl;
    for(int i = 0; i < view.size(); i++)
    {
        Op1.set(view[i].X1(), view[i].Y1());
        Op2.set(view[i].X2(), view[i].Y2());
        //cout<<"Object P1 x: "<<Op1.X()<<"; y: "<<Op1.Y()<<endl;
        //cout<<"Object P2 x: "<<Op2.X()<<"; y: "<<Op2.Y()<<endl;
        if(Op1== p1&& Op2 == p2)
        {
            flag = 1;
            return true;
        }
        else
            flag = 0;
            //return false;
    }
    
    if(flag == 0)
        return false;
}

vector< vector<Point> > GroupByPoint(vector<Object> view)
{
        vector<Point> allPoints;
        vector<Point> temp_group;
        vector< vector<Point> > GroupPoints;

        Point temp_point;
        Object temp_Obj;
        double threshold = 500;
        double dist = 0;
        double min = 0;
        int n = 0;
        int m = 1;
        int temp_num = 0;
        unsigned char flag = 0;

        //collect all points with ID
        allPoints = CollectPoints(view);
        cout<<"the size of all points is : "<< allPoints.size()<<endl;
        
        //the first group first point 
        temp_group.push_back(allPoints[0]);
        
        
        while(n < allPoints.size())
        {
                    min = 0;

                    //calculating the distance between two points to get the nearest one
                    for(m = 1; m < 8 ; m++)
                    {                         
                            //whether n+m is the last point in all points list
                            if(n+m < allPoints.size())
                            {
                                 //cout<<" current n is :"<< n<<": m is : "<< m<<endl;
                                      dist = GetPointDistance(allPoints[n], allPoints[n + m]);
                                      //cout<<"----Test programme----"<<endl;
                                      //cout<<"distance cal : " << dist<<endl;
                                      //if this is the first one store all info
                                      if(min == 0)
                                      {
                                          min = dist;
                                          temp_point = allPoints[n + m];
                                          temp_num = n + m;
                                      }
                                      else // this is the first one, compare two minimum distance
                                              if(dist < min && min != 0)
                                              {
                                                  min = dist;
                                                  temp_point = allPoints[n + m];
                                                  temp_num = n + m;
                                              } 
                            }
                            else
                            {

                                    if(dist < min && min != 0)
                                    {
                                                min = dist;
                                                temp_point = allPoints[n + m];
                                                temp_num = n + m;
                                    } 
                                    flag = 1;
                                    //n = allPoints.size();
                                    break;
                            }
                     }
                    
                    if(flag != 1)
                    {
                            //cout << "----- Got the next point, chech whether it is the surface in view -----"<<endl;
                            //got next point, and check whether it is surface in view
                            if(min < threshold)
                            {
                                    //cout<<"----- The distance is smaller than threshold -----"<<endl;
                                    //cout<<"the distance: "<<min<<endl;
                                    //cout<<"current temp point x: "<<temp_point.X()<<"; y: "<<temp_point.Y()<<endl;
                                    temp_group.push_back(temp_point);
                                    n = temp_num;
                            }
                            else
                                if(min >= threshold)
                                {
                                        //cout<<"----- The distance is greater than threshold -----"<<endl;
                                        //check whether this line segment is an actual surface
                                        //cout<<"temp group point n is: "<< n <<endl;
                                        if(isObject(view, allPoints[n], allPoints[n + 1]))
                                        {
                                                //cout << " ---- same point ID both of them ---- "<<endl;
                                                temp_group.push_back(allPoints[n + 1]);
                                                n = n + 1;
                                        }
                                        else
                                        {
                                                GroupPoints.push_back(temp_group);

                                                //new group first one point                           
                                                temp_group.clear();
                                                n = n +1;
                                                temp_group.push_back(allPoints[n]);
                                        }
                                }
                    }
                    else
                    {
                            //cout << "----- Got the next point, chech whether it is the surface in view -----"<<endl;
                            //got next point, and check whether it is surface in view
                            if(min < threshold)
                            {
                                    //cout<<"----- The distance is smaller than threshold -----"<<endl;
                                    //cout<<"the distance: "<<min<<endl;
                                    //cout<<"current temp point x: "<<temp_point.X()<<"; y: "<<temp_point.Y()<<endl;
                                    temp_group.push_back(temp_point);
                                    n = temp_num;
                            }
                            else
                                if(min >= threshold)
                                {
                                        //cout<<"----- The distance is greater than threshold -----"<<endl;
                                        //check whether this line segment is an actual surface
                                        //cout<<"temp group point n is: "<< n <<endl;
                                        if(isObject(view, allPoints[n], temp_point))
                                        {
                                                //cout << " ---- same point ID both of them ---- "<<endl;
                                                temp_group.push_back(temp_point);
                                                n = temp_num;
                                        }
                                        else
                                        {
                                                if(isObject(view, allPoints[n], allPoints[n + 1]))
                                                {
                                                        temp_group.push_back(allPoints[n + 1]);
                                                        n = n + 1;
                                                }
                                                else
                                                {
                                                        GroupPoints.push_back(temp_group);

                                                        //new group first one point                           
                                                        temp_group.clear();
                                                        //n = temp_num;
                                                        n = n + 1;
                                                        temp_group.push_back(allPoints[n]);
                                                }
                                        }
                                }
                            //cout<< "current temp_ num is : "<<temp_num<<endl; 
                            //cout<<"the current n is : " << n <<endl;
                            if(n == allPoints.size() - 1)
                            {
                                  //temp_group.push_back(temp_point);
                                  temp_group.push_back(allPoints.back());
                                  GroupPoints.push_back(temp_group);
                                  break;
                            }
                    }
        
        }
    
    return GroupPoints;
}

vector< vector<Point> > GroupByPoint_version2(vector<Object> view)
{
        vector<Point> allPoints;
        vector<Point> temp_group;
        vector< vector<Point> > GroupPoints;

        Point temp_point;
        Object temp_Obj;
        
        double threshold = 500;
        double dist = 0;


        //collect all points with ID
        allPoints = CollectPoints(view);
        
        //init
        
        
        while(allPoints.size() > 0)
        {
            temp_group.push_back(allPoints[0]);
            allPoints.erase(allPoints.begin()+0);
            for(int i = 0; i < allPoints.size(); i++)
            {
                dist = GetPointDistance(temp_group.back(), allPoints[i]);
                if(dist < threshold)
                {
                    temp_group.push_back(allPoints[i]);
                    allPoints.erase(allPoints.begin()+i);
                    i--;
                }
            }
            GroupPoints.push_back(temp_group);
            temp_group.clear();
        }
        
        
        
        return GroupPoints;
}


vector<Object> BoundaryByGroup(vector<Object> view)
{
        vector< vector<Point> > GroupPoints;
        vector<Object> boundary;
        Object temp_Obj;

        GroupPoints = GroupByPoint(view);
        //test
        //GroupPoints = GroupByPoint_version2(view);
        
        //get a list of group and connect start point and end point of each group
        for(int i = 0; i < GroupPoints.size(); i++)
        {
            temp_Obj.set(GroupPoints[i][0].X(), GroupPoints[i][0].Y(), GroupPoints[i][GroupPoints[i].size() - 1].X(), GroupPoints[i][GroupPoints[i].size() - 1].Y(), 1);
            boundary.push_back(temp_Obj);
        }
        
        return boundary;
}


vector<Object> BoundaryByExits(vector<Object> CurrentView)
{
            Exit exit;
            vector<Exit> exits;
            
            Object temp;
            vector<Object> temp_view;
            
            double threshold = 800;
            double dist_PtP = 0;
            
            cout << endl << "******* Finding exits and Rebuild view with boundary ******* " << endl;
            for (unsigned int i = 0; i < CurrentView.size(); i++) 
             {   
                 CurrentView[i].setID(i + 1);  
             }
            
            for (int i = 0; i < CurrentView.size(); i++) 
            {
                    if(i != CurrentView.size() - 1)
                    {
                            dist_PtP = sqrt((CurrentView[i].X2() - CurrentView[i+1].X1()) * (CurrentView[i].X2() - CurrentView[i+1].X1()) + (CurrentView[i].Y2() - CurrentView[i+1].Y1()) * (CurrentView[i].Y2() - CurrentView[i+1].Y1()));

                            if(dist_PtP >= threshold) 
                            {
                                exit.set(CurrentView[i].X2(), CurrentView[i].Y2(), CurrentView[i+1].X1(), CurrentView[i+1].Y1());
                                exit.setP1ID(CurrentView[i].getID());
                                exit.setP2ID(CurrentView[i+1].getID());

                                exits.push_back(exit);
                            }
                    }
                 
            }
            
            for(int i = 0; i < exits.size(); i ++)
            {
                    if(i == 0 )
                        temp.set(CurrentView[0].X1(), CurrentView[0].Y1(), exits[i].X1(), exits[i].Y1(), i);
                    else
                    {
                        if(i == exits.size() - 1)
                        {
                                    temp.set(exits[i-1].X2(), exits[i-1].Y2(), exits[i].X1(), exits[i].Y1(), i);  
                                    temp_view.push_back(temp);
                                    temp.set(exits[i].X2(), exits[i].Y2(), CurrentView[CurrentView.size() - 1].X2(), CurrentView[CurrentView.size() - 1].Y2(), i);     
                                    temp_view.push_back(temp);
                        }
                        else
                            temp.set(exits[i-1].X2(), exits[i-1].Y2(), exits[i].X1(), exits[i].Y1(), i);  
                    }
                    
                    if( i != exits.size() - 1)
                         temp_view.push_back(temp);
            }
            

            //if(exits.size() <= 2 && CurrentView.size() < 10)
            //   temp_view = CurrentView;
            if(exits.size() == 1 && CurrentView.size() < 10)
            {
                temp.set(CurrentView[0].X1(), CurrentView[0].Y1(), exits[0].X1(), exits[0].Y1(), 1);
                temp_view.push_back(temp);
                temp.set(exits[0].X2(), exits[0].Y2(),CurrentView[CurrentView.size()-1].X1(), CurrentView[CurrentView.size()-1].Y1(),2);
                temp_view.push_back(temp);
            }
            else
                if(exits.size() == 0 && CurrentView.size() <=10)
                    temp_view = CurrentView;
            
            return temp_view;
}

vector<Object> pathView(vector<Object> view, int relat_flag)
{
    vector<Object> temp_view;
    Object temp_obj;
    double threshold = 1000;
    double distToOrigin;
    
    switch(relat_flag)
    {
        case 1:   for(int i =0 ; i < view.size(); i++)
                       {
                        
                            distToOrigin = view[i].midToOrigin();
                            if(view[i].X1() < 0 || view[i].X2() < 0 || (view[i].X2() < 500 && view[i].Y2() < 500))
                                temp_view.push_back(view[i]);
                       }
                        break;
        case 2:   for(int i =0 ; i < view.size(); i++)
                       {
                            distToOrigin = view[i].midToOrigin();
                            if(view[i].X1() > 0 || view[i].X2() > 0 || (view[i].X1() > -500 && view[i].Y1() < 500))
                                temp_view.push_back(view[i]);
                       }
                        break;
        default: break;
    }

    return temp_view;
}

//detect all objects in simplified path view
//define their positions in the coordinate
//1--left 2--right 3--front 4--top left 5--top right
vector<int> ObjectPosition(vector<Object> pathView)
{
    vector<int> temp;
    for(int i = 0; i < pathView.size(); i++)
    {
        if(pathView[i].X1() < 0 && pathView[i].X2() < 0)
        {
            if(pathView[i].Y1() < 1500 && pathView[i].Y2() < 2500)
                temp.push_back(1);// left only
            if(pathView[i].Y1() > 1500 && pathView[i].Y1() < 3000 && pathView[i].Y2() > 1500 && pathView[i].Y2() < 3000 &&  pathView[i].X2() > -500)
                temp.push_back(3);// front only
        }
        if(pathView[i].X1() > 0 && pathView[i].X2() > 0)// right only
        {   
            if(pathView[i].Y1() < 2500 && pathView[i].Y2() < 1500)
                 temp.push_back(2);//right only
            if(pathView[i].Y1() > 1500 && pathView[i].Y1() < 3000 && pathView[i].Y2() > 1500 && pathView[i].Y2() < 3000 && pathView[i].X1()  < 500)
                temp.push_back(3);// front only
        }
        if((pathView[i].X1() > 0 && pathView[i].X2() < 0) || (pathView[i].X1() < 0 && pathView[i].X2() > 0))// front only
        {
            if(pathView[i].Y1() > 1000 || pathView[i].Y2() > 1000)
            temp.push_back(3);
        }

        if(pathView[i].X1() < 0 && pathView[i].X2() <= 0 && pathView[i].Y1() > 3000 && pathView[i].Y2() > 3000)//top left
            temp.push_back(4);
        if(pathView[i].X1() < 0 && pathView[i].X2() <= 0 && pathView[i].Y1() < 1000 && pathView[i].Y2() > 3000)//top left
            temp.push_back(4);
        if(pathView[i].X1() >= 0 && pathView[i].X2() > 0 && pathView[i].Y1() > 3000 && pathView[i].Y2() > 3000)// top right
            temp.push_back(5);     
        if(pathView[i].X1() >= 0 && pathView[i].X2() > 0 && pathView[i].Y1() > 3000 && pathView[i].Y2() < 1000)// top right
            temp.push_back(5);    
    }
    
    return temp;
}


int SingleObjectPosition(Object pathView)
{
    int temp;

        if(pathView.X1() < 0 && pathView.X2() < 0)
        {
            if(pathView.Y1() < 1500 && pathView.Y2() < 2500)
                temp = 1;// left only
            if(pathView.Y1() > 1500 && pathView.Y1() < 3000 && pathView.Y2() > 1500 && pathView.Y2() < 3000 &&  pathView.X2() > -500)
                temp = 3;// front only
        }
        if(pathView.X1() > 0 && pathView.X2() > 0)// right only
        {   
            if(pathView.Y1() < 2500 && pathView.Y2() < 1500)
                 temp = 2;//right only
            if(pathView.Y1() > 1500 && pathView.Y1() < 3000 && pathView.Y2() > 1500 && pathView.Y2() < 3000 && pathView.X1()  < 500)
                temp = 3;// front only
        }
        if((pathView.X1() > 0 && pathView.X2() < 0) || (pathView.X1() < 0 && pathView.X2() > 0))// front only
        {
            if(pathView.Y1() > 1000 || pathView.Y2() > 1000)
            temp = 3;
        }

        if(pathView.X1() < 0 && pathView.X2() <= 0 && pathView.Y1() > 3000 && pathView.Y2() > 3000)//top left
            temp = 4;
        if(pathView.X1() < 0 && pathView.X2() <= 0 && pathView.Y1() < 1000 && pathView.Y2() > 3000)//top left
            temp = 4;
        if(pathView.X1() >= 0 && pathView.X2() > 0 && pathView.Y1() > 3000 && pathView.Y2() > 3000)// top right
            temp = 5;      
        if(pathView.X1() >= 0 && pathView.X2() > 0 && pathView.Y1() > 3000 && pathView.Y2() < 3000)// top right
            temp = 5;     
    return temp;
}



pair<int, int> exitFromMem(vector<Object> Mem, Point des, int relat_flag)
{
    pair<int, int> temp; // two bounday position
    Object interObj; // intersected object
    Object another; // another object
    Point ObjP1, ObjP2;
    Point p1, p2;
    double distToObjs;
    double minDist = 10000;
    
    interObj = IntersectForReturn(Mem, des) ;// one of these construct the exit
    if(relat_flag == 1)
    {
        // find another object
        ObjP2.set(interObj.X2(), interObj.Y2());
            
        for(int i = 0; i < Mem.size(); i++)
        {
                p1.set(Mem[i].X1(), Mem[i].Y1());
                p2.set(Mem[i].X2(), Mem[i].Y2());
                if((ObjP2.X() != p2.X()) && (ObjP2.Y() != p2.Y()))
                {
                    distToObjs = GetNearestDistance(p1, p2, ObjP2);
                    if( distToObjs < minDist)
                    {
                        minDist = distToObjs;
                        another.set(p1.X(), p1.Y(), p2.X(), p2.Y(), 0);
                    }
                }
        }
       
    }
    
    if(relat_flag == 2)
    {
        // find another object
        ObjP1.set(interObj.X1(), interObj.Y1());
        
        for(int i = 0; i < Mem.size(); i++)
        {
                p1.set(Mem[i].X1(), Mem[i].Y1());
                p2.set(Mem[i].X2(), Mem[i].Y2());
                if((ObjP1.X() != p1.X()) && (ObjP1.Y() != p1.Y()))
                {
                    distToObjs = GetNearestDistance(p1, p2, ObjP1);
                    if( distToObjs < minDist)
                    {
                        minDist = distToObjs;
                        another.set(p1.X(), p1.Y(), p2.X(), p2.Y(), 0);
                    }
                }
        }
    }
    
    
    temp.first = SingleObjectPosition(interObj);
    temp.second = SingleObjectPosition(another);
    
    return temp;
}

//prototype exitPositions
//find the two boundary/object constructing the exit
vector<Object> exitOfObjects(vector<Object> CurPathView, int first, int second)
{
    vector<Object> temp_view;
    vector<Object> temp_O1;
    vector<Object> temp_O2;
    Object temp_Obj1;
    Object temp_Obj2;
    int flag1 = 0;
    int flag2 = 0;
    int pos;
    double min = 10000;
    double dist = 0;

    for(int i = 0; i < CurPathView.size(); i++)
    {
        pos = SingleObjectPosition(CurPathView[i]);
        if(pos == first && flag1 == 0)
        {
            //if(CurPathView[i].length() > temp_Obj1.length())
                    temp_O1.push_back(CurPathView[i]);
            if(first == second)
            {
                flag1 = 1;
                continue;
            }
        }
        if(pos == second)
        {
            if(flag2 == 0)
            {
                temp_Obj2 = CurPathView[i];
                flag2 = 1;
            }
            else
                if(CurPathView[i].length() > temp_Obj2.length())
                {
                    temp_Obj2 = CurPathView[i];
                    //flag2 = 1;
                }
        }
    }
    
    if(temp_O1.size() != 1)
    {
        for(int i = 0; i < temp_O1.size(); i++)
        {
            dist = GetNearestDistance(Point (temp_Obj2.X1(), temp_Obj2.Y1()), Point (temp_Obj2.X2(), temp_Obj2.Y2()), Point (temp_O1[i].X2(), temp_O1[i].Y2()));
            if(dist < min)
                temp_Obj1 = temp_O1[i];
        }
    }
    else
    {
        temp_Obj1 = temp_O1[0];
    }
    
    
    temp_view.push_back(temp_Obj1);
    temp_view.push_back(temp_Obj2);
    
    return temp_view;
}


pair<double,double> DistAndAngle(Point Pos)
{
    pair<double, double> temp;
    
    temp.first = sqrt(Pos.X() * Pos.X() + Pos.Y() * Pos.Y());
    temp.second = acos(Pos.Y() /   temp.first); 
    temp.second = 180 / PI * temp.second;

    // moving to the position, second step 
    if(Pos.X() > 0)
         temp.second = 0 -  temp.second;
    
    return temp;
}


// (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)
// input: three points p0, p1, p2
// return: >0  p2 left of line through p0 to p1
//              =0  p2 on the line
//              <0  p2 right of line throught p0 to p1
int isLeft(Point p0, Point p1, Point p2)
{
        return ((p1.X() - p0.X()) * (p2.Y() - p0.Y()) - (p2.X() - p0.X()) * (p1.Y() - p0.Y()));
}


double AngleOfNext(Point p1, Point p2, Point p3)
{
        cout << "*****This is going to calculate angle between two adjacent paths*****" << endl << endl;
        double k1, k2;
        double angleTwoPath;
        double a, b, c;
        double cosfi = 0, fi = 0, norm = 0;

        b = sqrt((p1.X() - p2.X()) * (p1.X() - p2.X()) + (p1.Y() - p2.Y()) * (p1.Y() - p2.Y())); // b

        c = sqrt((p3.X() - p2.X()) * (p3.X() - p2.X()) + (p3.Y() - p2.Y()) * (p3.Y() - p2.Y())); // c

        a = sqrt((p3.X() - p1.X()) * (p3.X() - p1.X()) + (p3.Y() - p1.Y()) * (p3.Y() - p1.Y())); // a

        cosfi = (b * b + c * c - a * a) / (2 * b * c);

        fi = acos(cosfi);

        if (180 * fi / PI < 180)     
        {
            angleTwoPath =  180 * fi / PI;
        }
        else
        {
            angleTwoPath =  360 - 180 * fi / PI;
        } 
                
        angleTwoPath = 180 - angleTwoPath;
        if(p3.X() - p2.X() > 0)
            angleTwoPath = 0 - angleTwoPath;
        
        return angleTwoPath;
}



//choose reference boundary
vector<Point> referenceBoundary(vector<Object> boundaryView)
{
        int i = 0;
        int ref1ID = boundaryView[0].getID();
        int ref2ID = boundaryView[1].getID();
        int tempID;
        Point ref1P1, ref1P2, ref2P1, ref2P2;
        Point temp_p1, temp_p2;
        
        ref1P1.set(boundaryView[0].X1(), boundaryView[0].Y1());
        ref1P2.set(boundaryView[0].X2(), boundaryView[0].Y2());
        
        ref2P1.set(boundaryView[1].X1(), boundaryView[1].Y1());
        ref2P2.set(boundaryView[1].X2(), boundaryView[1].Y2());

        double ref1length = boundaryView[0].length();
        double ref2length = boundaryView[1].length();
        //double avg = 0;
        vector<double> temp; 
        double temp_length;
        
        vector<Point> BoundaryPoints;
        
        // Ensure the reference 1 is the most, reference 2 is the second most. swap value and ID
        if(ref1length < ref2length) 
        {
                //swap length size
                temp_length = ref2length;
                ref2length = ref1length;
                ref1length = temp_length;
                
                //swap ID
                tempID = ref2ID;
                ref2ID = ref1ID;
                ref1ID = ref2ID;
                
                //swap points
                temp_p1 = ref2P1;
                temp_p2 = ref2P2;
                ref2P1 = ref1P1;
                ref2P2 = ref1P2;
                ref1P1 = temp_p1;
                ref1P2 = temp_p2;
        }

        for(i = 2; i < boundaryView.size(); i++)
        {
                temp.push_back(boundaryView[i].length());
                temp_length = boundaryView[i].length();
                tempID = boundaryView[i].getID();
                temp_p1.set(boundaryView[i].X1(), boundaryView[i].Y1());
                temp_p2.set(boundaryView[i].X2(), boundaryView[i].Y2());

                if(temp_length > ref2length && temp_length < ref1length)
                {
                    ref2length = temp_length;
                    ref2ID = tempID;
                    
                    ref2P1 = temp_p1;
                    ref2P2 = temp_p2;
                }

                if(temp_length > ref1length)
                {
                    ref2length = ref1length;
                    ref2ID = ref1ID;

                    ref1length = temp_length;
                    ref1ID = tempID;
                    
                    ref2P1 = ref1P1;
                    ref2P2 = ref1P2;
                    ref1P1 = temp_p1;
                    ref1P2 = temp_p2;
                }
        }
        
        //get the corresponding points p1 p2 p3 p4
        //BoundaryPoints.push_back(ref1P1);
        //BoundaryPoints.push_back(ref1P2);
        //BoundaryPoints.push_back(ref2P1);
        //BoundaryPoints.push_back(ref2P2);
        
        
        
        //get the corresponding points p1 p2 p3 p4
        if (ref1P2.X() - ref2P1.X() > 0)
        {
            BoundaryPoints.push_back(ref2P1);
            BoundaryPoints.push_back(ref2P2);
            BoundaryPoints.push_back(ref1P1);
            BoundaryPoints.push_back(ref1P2);

        } else
        {
            BoundaryPoints.push_back(ref1P1);
            BoundaryPoints.push_back(ref1P2);
            BoundaryPoints.push_back(ref2P1);
            BoundaryPoints.push_back(ref2P2);

        }
        
        return BoundaryPoints;
        
}

// dist_1 = ref1P1 to ref2P1
// dist_2 = ref1P2 to ref2P2
// avg = (dist_1 + dist_2) / 2
double layoutRelation(Point ref1P1, Point ref1P2, Point ref2P1, Point ref2P2)
{
        double dist_1, dist_2;
        double avg;

        dist_1 = sqrt((ref1P1.X() - ref2P1.X()) * (ref1P1.X() - ref2P1.X())   + (ref1P1.Y() - ref2P1.Y()) * (ref1P1.Y() - ref2P1.Y()));
        dist_2 = sqrt((ref1P2.X() - ref2P2.X()) * (ref1P2.X() - ref2P2.X())   + (ref1P2.Y() - ref2P2.Y()) * (ref1P2.Y() - ref2P2.Y()));

        avg = (dist_1 + dist_2) / 2;

        return avg;
}

//calculating two largest reference boundaries length size
double ReferenceLength(Point refP1, Point refP2)
{
    double length;
    length = sqrt((refP1.X() - refP2.X()) * (refP1.X() - refP2.X()) + (refP1.Y() - refP2.Y()) * (refP1.Y() - refP2.Y()));
    return length;
}

//second reference boundary P1 perpendicularity to the first reference boundary
//line segment joined by ref1P1 and ref1P2
//d = |VL * W| / |VL| = ((y0 - y1) * x +( x1 - x0) * y + (x0 * y1 - x1 * y0))) / sqrt((x0 - x1)^2 + (y0 - y1)^2)
// ref1P2 is (x0, y0), ref1P1 is (x1, y1), ref2P1 is (x, y)
double perpendicularDis(Point ref1P1, Point ref1P2, Point ref2P1)
{
    double dist = 0;
    dist = abs(((ref1P2.Y() - ref1P1.Y()) * ref2P1.X() +( ref1P1.X() - ref1P2.X()) * ref2P1.Y() + (ref1P2.X() * ref1P1.Y() - ref1P1.X() * ref1P2.Y())) / sqrt((ref1P2.X() - ref1P1.X()) * (ref1P2.X() - ref1P1.X()) + (ref1P2.Y() - ref1P1.Y()) * (ref1P2.Y() - ref1P1.Y())));
    
    return dist;
}

//another method to calculate the distance from a point to a reference boundary
double Perpendiculardistance(Point ref1P1, Point ref1P2, Point ref2P1) 
{   
    double x1 = ref1P1.X();  
    double y1 = ref1P1.Y();  
    double x2 = ref1P2.X(); 
    double y2 = ref1P2.Y();  
    double x = ref2P1.X();  
    double y = ref2P1.Y();  
    return  abs(((x-x1)*(y2-y1)-(x2-x1)*(y-y1))/sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))); 
}

double GetPointDistance(Point p1, Point p2)    
{   
     return sqrt((p1.X()-p2.X())*(p1.X()-p2.X())+(p1.Y()-p2.Y())*(p1.Y()-p2.Y()));   
}   
double GetNearestDistance(Point PA, Point PB, Point P3)   
{   
    double a,b,c;   
    a = GetPointDistance(PB,P3);   
    if(a <= 0.00001)   
     return 0.0f;   
    b = GetPointDistance(PA,P3);   
    if(b <= 0.00001)   
     return 0.0f;   
    c = GetPointDistance(PA,PB);   
    if(c <= 0.00001)   
     return a;



    if(a*a >= b*b + c*c)
     return b;   
    if(b*b >= a*a + c*c)
     return a;    


    double l = (a + b + c)/2;     
    double s = sqrt(l * (l - a) * (l - b) * (l - c));  
    return 2*s/c;   
}


// current boundary view is to match the memory
// if the current two reference boundary avg is approximated with memorised avg
// treating this as recognised two reference boundaries
bool Recognision(vector<Object> CurrentView, vector<Object> MemoryView)
{
        double threshold = 500;
        vector<Object> currentBoundary;
        vector<Object> memoryBoundary;
        vector<Point> currentPoints;
        vector<Point> memoryPoints;
        
        double currentAvg;
        double memoryAvg;

        currentBoundary = BoundaryByExits(CurrentView);
        memoryBoundary = BoundaryByExits(MemoryView);
        
        currentPoints = referenceBoundary(currentBoundary);
        memoryPoints = referenceBoundary(memoryBoundary);
        
        currentAvg =  layoutRelation(currentPoints[0], currentPoints[1], currentPoints[2], currentPoints[3]);
        memoryAvg =  layoutRelation(memoryPoints[0], memoryPoints[1], memoryPoints[2], memoryPoints[3]);
        
        if(abs(memoryAvg - currentAvg) <= threshold)
            return true;
        else 
            return false; 
}

bool RecognisionBasedPerpend(vector<Object> CurrentView, vector<Object> MemoryView)
{
        double threshold = 350;
        vector<Object> currentBoundary;
        vector<Object> memoryBoundary;
        vector<Point> currentPoints;
        vector<Point> memoryPoints;
        
        double currentSize;
        double memorySize;
        
        double CP1toRef1;
        double MP1toRef1;
        

        currentBoundary = BoundaryByExits(CurrentView);
        memoryBoundary = BoundaryByExits(MemoryView);
        
        currentPoints = referenceBoundary(currentBoundary);
        memoryPoints = referenceBoundary(memoryBoundary);
        
        //calculating the average size between two reference boundaries
        currentSize = (ReferenceLength(currentPoints[0], currentPoints[1]) + ReferenceLength(currentPoints[2], currentPoints[3])) / 2;
        memorySize = (ReferenceLength(memoryPoints[0], memoryPoints[1]) + ReferenceLength(memoryPoints[2], memoryPoints[3])) / 2;
        
        //calculating the 2rd reference boundary P1 to 1st reference perpendicular distance 
        CP1toRef1 = perpendicularDis(currentPoints[0], currentPoints[1], currentPoints[2]);
        MP1toRef1 = perpendicularDis(memoryPoints[0], memoryPoints[1], memoryPoints[2]);
        
        cout << "current P1 to reference 1: " << CP1toRef1 << endl;
        cout << "memory P1 to reference 1: " << MP1toRef1 << endl;
        
        CP1toRef1 = Perpendiculardistance(currentPoints[0], currentPoints[1], currentPoints[2]);
        MP1toRef1= Perpendiculardistance(memoryPoints[0], memoryPoints[1], memoryPoints[2]);
        
        cout << "current P1 to reference 1: " << CP1toRef1 << endl;
        cout << "memory P1 to reference 1: " << MP1toRef1 << endl;
        
        cout << "average size in current view: " << currentSize << endl;
        cout << "average size in memory view: " << memorySize << endl;
        
        //if(abs(CP1toRef1 - MP1toRef1) < threshold && abs(currentSize - memorySize) < 200)
        if(abs(CP1toRef1 - MP1toRef1) < threshold)
            return true;
        else
        {
            if(abs(currentSize - memorySize) < threshold)
                return true;
            else
                return false;
        }
              
}

vector<Object> recognisedObject(vector<Object> CurrentView, vector<Object> MemoryView)
{
    vector<Object> output;
    vector<Object> currentBoundary;
    vector<Point> currentPoints;
    Object reference1;
    Object reference2;
    
    if(RecognisionBasedPerpend(CurrentView, MemoryView))
    {
        currentBoundary = BoundaryByExits(CurrentView);
        currentPoints = referenceBoundary(currentBoundary);
        //memoryPoints = referenceBoundary(memoryBoundary);
        reference1.set(currentPoints[0].X(), currentPoints[0].Y(), currentPoints[1].X(), currentPoints[1].Y(), 1);
        reference2.set(currentPoints[2].X(), currentPoints[2].Y(), currentPoints[3].X(), currentPoints[3].Y(), 2);
        output.push_back(reference1);
        output.push_back(reference2);
    }
    
    return output;
}

// perpendicular crossed point from one point to a line
// return a point with  (x, y)
Point crossPerpend(Point ref1P1, Point ref1P2, Point ref2P1)  
{  
        double A, B, m;
        double x, y;
        Point ptCross;

        A = (ref1P1.Y()-ref1P2.Y()) / (ref1P1.X()- ref1P2.X());  
        B = (ref1P1.Y()-A * ref1P1.X());  
        /// equation is for  0 = ax +b -y;  -x -ay + m = 0; 
        /// > A = a; B = b;  
        m = ref2P1.X() + A* ref2P1.Y();  


        x = (m - A * B) / (A * A + 1);
        y = A* x + B;
        ptCross.set(x, y);
        //ptCross.setX((m-A*B)/(A*A + 1));  
        //ptCross.setY(A*ptCross.x()+B);  
        return ptCross;  
}



vector<Point> recognisedLRObject(vector<Object> CurrentView, vector<Object> MemoryView)
{
        vector<Object> currentBoundary;
        vector<Object> memoryBoundary;
        vector<Point> currentPoints;
        vector<Point> memoryPoints;
        vector<Point> rtn;
        Point flag;
        currentBoundary = BoundaryByExits(CurrentView);
        memoryBoundary = BoundaryByExits(MemoryView);
        if(currentBoundary.size()==0||memoryBoundary.size()==0)
        {
            flag.set(0,0);
            rtn.push_back(flag);
            return rtn;
        }
        currentPoints = referenceBoundary(currentBoundary);

        memoryPoints = referenceBoundary(memoryBoundary);
        if(currentPoints.size()==0||memoryPoints.size()==0)
        {
            flag.set(0,0);
            rtn.push_back(flag);
             return rtn;
        }
        if (currentPoints[0].X() < 0 && memoryPoints[0].X() < 0 && currentPoints[3].X() > 0 && memoryPoints[3].X() > 0)
        {
            return currentPoints;
        } else if (currentPoints[0].X() > 0 && currentPoints[1].X() > 0 && currentPoints[2].X() > 0 && currentPoints[3].X() > 0)
        {
            flag.set(0, 1);

        } else
        {
            flag.set(1, 0);
        }
        rtn.push_back(flag);
        return rtn;
}

vector<Object> getrecognisedLRObject(vector<Object> CurrentView, vector<Object> MemoryView)
{
        vector<Point> currentPoints = recognisedLRObject(CurrentView, MemoryView);
        vector<Object> rtn;
        Object reference1, reference2;
        if (currentPoints.size() == 4)
        {
            reference1.set(currentPoints[0].X(), currentPoints[0].Y(), currentPoints[1].X(), currentPoints[1].Y(), 1);
            reference2.set(currentPoints[2].X(), currentPoints[2].Y(), currentPoints[3].X(), currentPoints[3].Y(), 2);
            rtn.push_back(reference1);
            rtn.push_back(reference2);
        } 
        else
        {
            reference1.set(0, 0, currentPoints[0].X(), currentPoints[0].Y(), 1);
            rtn.push_back(reference1);
        }

        return rtn;
}


vector<Point> SearchingReference(vector<Object> Memory, int flag)
{
        int orien = 0;
        int num = 100;
        ArRobot robot;
        ArSick sick;
        vector<Object> CurrentView;
        vector<Point> temp_points;
        
        switch(flag)
        {
            //* turning left first, 10-degree *//
            case 1: do
                          {
                                setHeading(robot, 10);
                                CurrentView = scanAndSaveView(sick, num);
                                 num++;
                                 //matching again
                                 temp_points = recognisedLRObject(CurrentView, Memory);
                           }while((temp_points.size() != 4) && (temp_points[0].X() == 1));

                           return temp_points;
                           break;
            //* turning right right, 10-degree *//
            case 2: do
                          {
                                setHeading(robot, -10);
                                CurrentView = scanAndSaveView(sick, num);
                                 num++;
                                 //matching again
                                 temp_points = recognisedLRObject(CurrentView, Memory);
                           }while((temp_points.size() != 4) && (temp_points[0].Y() == 1));
                           
                          return temp_points;
                          break;
             //* if cannot matching yet, moving forward a bit *//           
            case 3:  setHeading(robot, 0);
                          moveDistance(robot, -200);
                          CurrentView = scanAndSaveView(sick, num);
                          num++;
                          temp_points = recognisedLRObject(CurrentView, Memory);
                          return temp_points;
                          break;
            default: break;
        }
        
}

Object SearchBoundary(vector<Object> CV, double midDist, int relat_flag)
{
    Object rtn;
    double distance;
    double threshold = 500;
    
    if(relat_flag == 1)
    {
        for(int i = 0; i < CV.size(); i++)
        {
            if(CV[i].X1() < 0 && CV[i].X2() < 0)
            {
                distance = midPointOfObj(CV[i]);
                if(abs(distance - midDist) <= threshold)
                    rtn = CV[i];
            }
        }
    }
    
    if(relat_flag == 2)
    {
        for(int i = 0; i < CV.size(); i++)
        {
            if(CV[i].X1() > 0 && CV[i].X2() > 0)
            {
                distance = midPointOfObj(CV[i]);
                if(abs(distance - midDist) <= threshold)
                    rtn = CV[i];
            }
        }
    }
    
    return rtn;
}

double getAngleofTwoObjects(Object a, Object b)
{
        double angle = 180 - getAngleofOneObject(a) + getAngleofOneObject(b);
        return angle;
}

double getAngleofOneObject(Object a)
{
        double ax1 = a.X1();
        double ay1 = a.Y1();
        double ax2 = a.X2();
        double ay2 = a.Y2();

        return atan2(ay2 - ay1, ax2 - ax1)*180 / PI;
}

//tanθ=∣(k2 - k1)/(1 + k1 * k2)| this is acute angle
double IncludedAngleOfTwoObjects(Object a, Object b)
{
    double k1, k2;
    double angle;
    
    if(a.X1() == 0 && a.X2() == 0)
    {
        angle = acos((b.Y2() - b.Y1()) / b.length());
    }
    else
    {
        k1 = (a.Y2() - a.Y1()) / (a.X2() - a.X1());
        k2 = (b.Y2() - b.Y1()) / (b.X2() - b.X1());
        angle = atan((k2 - k1) / (1 + k1*k2));
    }
    angle = angle * 180 / PI;
    //cout << " the included angle : " << angle << endl;
    return angle;
}

vector<double> getAngleAndDistance(vector<Object> Memory, Point postion)
{
        vector<Object> memoryBoundary;
        vector<Point> memPoints;
        vector<double> rtn;
        double memNextX = postion.X();
        double memNextY = postion.Y();
        double memAngle, memDistance;
        Object memPositonObj;
        Object memRefObj;

        memoryBoundary = BoundaryByExits(Memory);
        //get 4 reference points from mem.
        memPoints = referenceBoundary(memoryBoundary);
        //generate reference object from the first two reference points.
        memRefObj.set(memPoints[0].X(), memPoints[0].Y(), memPoints[1].X(), memPoints[1].Y(), 0);
        //position object is built by the second point of the reference object and the next robot position in memory.
        memPositonObj.set(memPoints[1].X(), memPoints[1].Y(), memNextX, memNextY, 1);
        //get the angle between the reference object and the position object.
        memAngle = getAngleofTwoObjects(memRefObj, memPositonObj);
        //get the distance of the reference object's second point between the next robot position.
        memDistance = memPositonObj.length();
        rtn.push_back(memAngle);
        rtn.push_back(memDistance);
        return rtn;
}

Point getNextPosition(vector<Object> Memory, Point position, Object curRefObject)
{
        double currNextX;
        double currNextY;
        double memAngle, memDistance, currRefObjAngle, currPostionObjAngle;
        vector<double> AngleAndDistance;
        Point nextPoint;
        AngleAndDistance = getAngleAndDistance(Memory, position);
        memAngle = AngleAndDistance[0];
        memDistance = AngleAndDistance[1];
        //get the angle between the current reference object and x-asix.
        currRefObjAngle = getAngleofOneObject(curRefObject);
        //calculate the angle between the current reference object and the position object.
        currPostionObjAngle = memAngle + currRefObjAngle - 180;
        //calculate the next robot position in current view.
        currPostionObjAngle = (currPostionObjAngle / 180) * PI;
        currNextX = curRefObject.X2() + memDistance * cos(currPostionObjAngle);
        currNextY = curRefObject.Y2() + memDistance * sin(currPostionObjAngle);
        nextPoint.set(currNextX, currNextY);
        return nextPoint;
}


//detecting the exit/gap near the left or right
//input: current boundary view and which item
//item - 1: detecting left  ;-2: detecting right
bool isExit(vector<Object> CurrentView, int relat_flag)
{
    unsigned char flag = 0;
        //detecting the boundary is far away
        switch(relat_flag)
        {
              //case 0: 
              //            break;
              case 1:       if(CurrentView[0].X1() < -4000 && CurrentView[0].Y2() < 2000)
                                    return true;
                                  else
                                  {
                                      if(CurrentView[1].X1()< -4000 && CurrentView[1].Y2() < 2000 )
                                          return true;
                                      else
                                        flag = 0;
                                  }

                                if(flag == 0)
                                    return false;
                                break;
                                
              case 2:      if(CurrentView[CurrentView.size() - 1].X2() > 4000 && CurrentView[CurrentView.size() - 1].Y1() < 2000)
                                  return true;
                                else
                                {
                                    if(CurrentView[CurrentView.size() - 2].X2() > 4000 && CurrentView[CurrentView.size() - 2].Y1() < 2000)
                                        return true;
                                    else
                                      flag = 0;
                                }
                                   
                                if(flag == 0)
                                    return false;
                                break;
          default:break;
        }
}

vector<double> distancesToNext(vector<Object> Mry, Point aim)
{
    double temp_dist1; //the first section distance
    double  temp_dist2;//the second section distance
    double temp_angle;//the intermed position to aim position 
    vector<double> temp;
    Object interObj;
    Point inter;
    
    interObj = IntersectForReturn(Mry, aim);
    
    if(interObj.X1() != 0 && interObj.X2() != 0)
    {
            inter.set(interObj.X2() + 300, interObj.Y2() + 300);
            temp_dist1 = sqrt(inter.X() * inter.X() + inter.Y() * inter.Y());
            temp_dist2 = sqrt((aim.X() - inter.X()) * (aim.X() - inter.X()) + (aim.Y() - inter.Y()) * (aim.Y() - inter.Y()));

            temp.push_back(temp_dist1);
            temp.push_back(temp_dist2);

            temp_angle = AngleOfNext(Point (0,0), inter, aim);
            temp.push_back(temp_angle);
    }
    else
    {
            if(interObj.X1() == 0 && interObj.X2() == 0)
            {
                    temp_dist1 = sqrt(aim.X() * aim.X() + aim.Y() * aim.Y());
                    temp_dist2 = 0;

                    temp.push_back(temp_dist1);
                    temp.push_back(temp_dist2);

                    temp_angle = AngleOfNext(Point (0,0), Point (0,0), aim);
                    temp.push_back(temp_angle);
            }
    }
    
    return temp;
    
}

double midPointOfMemo(vector<Object> Mry, Point aim)
{
    Object interObj;
    Point mid;
    double distFromMid;
    double mid_x;
    double mid_y;
    
    interObj = IntersectForReturn(Mry, aim);
    
    if(interObj.X1() != 0 && interObj.X2() != 0)
    {
        mid_x = (interObj.X1() + interObj.X2()) / 2;
        mid_y = (interObj.Y1() + interObj.Y2()) / 2;
        
        distFromMid = sqrt(mid_x * mid_x + mid_y * mid_y);
    }
    else
        distFromMid = 0;
    
    return distFromMid;
    
}

double midPointOfObj(Object Obj)
{
    Object interObj;
    Point mid;
    double distFromMid;
    double mid_x;
    double mid_y;
    
    interObj = Obj;
    
    mid_x = (interObj.X1() + interObj.X2()) / 2;
    mid_y = (interObj.Y1() + interObj.Y2()) / 2;

    distFromMid = sqrt(mid_x * mid_x + mid_y * mid_y);
    
    return distFromMid;
    
}


void ExecutionAndGo(ArRobot& robot, double angle, double dist)
{

        // Stop the robot and wait a bit
        robot.lock();
        robot.stop();
        robot.clearDirectMotion();
        robot.unlock();
        //ArUtil::sleep(2000);

        //save odometer and orientation angle before moving       
        robot.lock();
        double oldDistance = robot.getOdometerDistance();
        double oldAngle = robot.getTh();
        robot.unlock();

        //movement execution
        setHeading(robot, angle);
        moveDistance(robot, dist);
        angle = 0;
        
        // Stop the robot and wait a bit
        robot.lock();
        robot.stop();
        robot.clearDirectMotion();
        robot.unlock();
        //ArUtil::sleep(2000);
        
}

void avoidConvex(ArRobot& robot, vector<Object> CurrentView)
{
    double dis;
    for(int i = 0; i < CurrentView.size(); i++)
    {
        dis = sqrt(CurrentView[i].X1() * CurrentView[i].X1() + CurrentView[i].Y1() * CurrentView[i].Y1());
        if(dis <= 200 && CurrentView[i].X1() < 0)
        {
                ExecutionAndGo(robot, -90, 400);
                ExecutionAndGo(robot, 90, 0);
        }
        if(dis <= 200 && CurrentView[i].X1() > 0)
        {
                ExecutionAndGo(robot, 90, 400);
                ExecutionAndGo(robot, -90, 0);
        }
        dis = sqrt(CurrentView[i].X2() * CurrentView[i].X2() + CurrentView[i].Y2() * CurrentView[i].Y2());
        if(dis <= 200 && CurrentView[i].X2() < 0)
        {
                ExecutionAndGo(robot, -90, 400);
                ExecutionAndGo(robot, 90, 0);
        }
        if(dis <= 200 && CurrentView[i].X2() > 0)
        {
                ExecutionAndGo(robot, 90, 400);
                ExecutionAndGo(robot, -90, 0);
        }
    }
}

//find the farthest boudnary front of the robot
//mid-point of this boundary to calculate and return an angle
Point farthestPoint(vector<Object> CurrentView, double relat_flag)
{
    Point temp;
    //double angle;
    double farthest = 0;
    for(int i = 0; i < CurrentView.size(); i++)
    {
        if(relat_flag == 1)
        {
            if(CurrentView[i].Y1() > farthest && CurrentView[i].X1() <= 0)
            {
                    farthest = CurrentView[i].Y1();
                    temp.set(CurrentView[i].X1(), CurrentView[i].Y1());
            }
            if(CurrentView[i].Y2() > farthest && CurrentView[i].X2() <= 0)
            {
                    farthest = CurrentView[i].Y2();
                    temp.set(CurrentView[i].X2(), CurrentView[i].Y2());
            }
        }
        
        if(relat_flag == 2)
        {
            if(CurrentView[i].Y1() > farthest && CurrentView[i].X1() > 0)
            {
                    farthest = CurrentView[i].Y1();
                    temp.set(CurrentView[i].X1(), CurrentView[i].Y1());
            }
            if(CurrentView[i].Y2() > farthest && CurrentView[i].X2() > 0)
            {
                    farthest = CurrentView[i].Y2();
                    temp.set(CurrentView[i].X2(), CurrentView[i].Y2());
            }
        }
    }
    
    return temp;
}

Object farthestObject(vector<Object> CurrentView, Point rb)
{
    double max = 0;
    Object farthest_obj;
    
    for(int i = 0; i < CurrentView.size(); i++)
    {
        double distance = distanceOftwoP(CurrentView[i].midpoint(), rb);
        if(i == 0)
        {
            max = distance;
            farthest_obj = CurrentView[i];
        }
        else
        {
            if(max < distance)
            {
                    max = distance;
                    farthest_obj = CurrentView[i];
            }
        }
    }
    
    return farthest_obj;
}

//calculating the a path between origin and farthest point
//detecting any point is too near this path, which i need to avoid 
Point avoidObstracle(vector<Object> CurrentView, Point far)
{
        Point temp;
        double threshold = 300;
        double Perdist = 0;
        double temp_x = far.X();// = 0;
        double temp_y = far.Y();//= 0;

        for(int i =0; i < CurrentView.size(); i++)
        {
                temp.set(CurrentView[i].X1(), CurrentView[i].Y1());

                Perdist = Perpendiculardistance(Point (0,0), far, temp);
                cout<<"The perdist is: " << Perdist <<endl;
                if(i == 0 && Perdist <= threshold)
                {
                    temp_x = temp.X();
                    temp_y = temp.Y();
                }
                if(i !=0 && Perdist <= threshold)
                {
                    if(temp.Y() < temp_y)
                    {
                          temp_x = temp.X();
                          temp_y = temp.Y();
                    }
                }

                temp.set(CurrentView[i].X2(), CurrentView[i].Y2());

                Perdist = Perpendiculardistance(Point (0,0), far, temp);
                cout<<"The perdist is: " << Perdist <<endl;
                if(Perdist <= threshold && temp.Y() < temp_y)
                {
                          temp_x = temp.X();
                          temp_y = temp.Y();
                }
        }

        temp.set(temp_x, temp_y);

        return temp;
}

//input: vector<Object> Points, including four points joining the two reference boundaries
//            Point aim is the nest destination
void PathPlanner(ArRobot& robot, ArSick& sick, vector<Point> Points, Point aim, int relat_flag)
{
        MyRobot myrobot(0,0);
        double angle;
        double distance;
        double temp_angle; // temp position where it moves to first
        double temp_dist;    // temp position distance
        double temp_x;        // temp x
        double temp_y;        // temp y
        
        char viewFileName[100];
        char rebuildFileName[100];
        
        vector<Point> temp_points;
        vector<Object> CurrentView;
        vector<double> planOrient;
        Point ref2P1;
        Point ref1P2; 
        Point near;
        Point temp_aim;
        ref2P1 = Points[2];
        

        //first of all, the robot needs to reach and pass through a door or narrow place      
        //calculating the narrow space position
        near.set(ref2P1.X() - 200, ref2P1.Y());
        temp_dist = sqrt(near.X() * near.X() + near.Y() * near.Y());
        temp_angle = acos(near.Y() / temp_dist); 
        temp_angle = 180 / PI * temp_angle;
        
        if(near.X() > 0)
            temp_angle = 0 - temp_angle;

        cout<<"-----Test Programme for angle and distance-----" << endl;
        cout<<"Temp Angle to the destination is : " << temp_angle << endl;
        cout<<"Temp Distance to the destination is : " << temp_dist << endl;
        //waitHere();
        
         //if(temp_angle > -3 && temp_angle <0)
         //          temp_angle = -5;
        // if(temp_angle < 3 && temp_angle > 0)
         //          temp_angle = 5;   
        
        // this step is to reach the narrow door place
        ExecutionAndGo(robot, temp_angle, temp_dist);
        
        //calculating the position in current view
        CurrentView = scanAndSaveView(sick, number);
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
        plotObjects(viewFileName, myrobot.getRobot(), CurrentView);    
        CurrentView =  BoundaryByExits(CurrentView);
        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);    
        number++;
        //if(temp_angle < 0)
        //    temp_angle = -temp_angle;
        //temp_aim = transformToCV(aim, near, temp_angle);
        
        //then second step is to reach the exact position
        //if no any intersection, moving directly, else moving a near place
        /*if(!IntersectForReturn(CurrentView, temp_aim))
        {
                //moving to the position directly
                //distance = sqrt((aim.X() - near.X()) * (aim.X() - near.X()) + (aim.Y() - near.Y()) * (aim.Y() - near.Y()));
                //angle = acos((aim.Y() - near.Y()) / distance);
                distance = sqrt(temp_aim.X() * temp_aim.X() + temp_aim.Y() * temp_aim.Y());
                angle = acos(temp_aim.Y() / distance);
                angle = 180 / PI * angle;
                
                if(aim.X() - near.X() > 0)
                     angle = 0 - angle;

                cout<<"Test Programme for angle and distane" << endl;
                cout<<"Final Angle to the destination is : " << angle << endl;
                cout<<"Fianl Distance to the destination is : " << distance << endl;
                //waitHere();

                ExecutionAndGo(robot, angle, distance);
        }
        else
        {
                //guided by the boundary that is intersected with the straight path
                CurrentView =  BoundaryByExits(CurrentView);
                temp_points = referenceBoundary(CurrentView);
                
                //ref1P2.set(temp_points[1].X(), temp_points[1].Y());
                near.set(temp_points[1].X() + 200, temp_points[1].Y() + 200);
                temp_dist = sqrt(near.X() * near.X() + near.Y() * near.Y());
                temp_angle = acos(near.Y() / temp_dist); 

                if(near.X() > 0)
                    temp_angle = 0 - temp_angle;
                
                cout<<"Test Programme for angle and distane" << endl;
                cout<<"The second temp Angle to the destination is : " << temp_angle << endl;
                cout<<"The second temp Distance to the destination is : " << temp_dist << endl;
                //waitHere();
                
                ExecutionAndGo(robot, temp_angle, temp_dist);
                
                //then the final step to reach the destination
            
        }
        */
        //once the robot moves to a destination 
        //generate current view and plot it with position
        
        
        //first of all, the robot needs to reach and pass through a door or narrow place      
        //calculating the narrow space position
        near.set(CurrentView[0].X2() + 300, CurrentView[0].Y2() + 300);
        temp_dist = sqrt(near.X() * near.X() + near.Y() * near.Y());
        temp_angle = acos(near.Y() / temp_dist); 
        temp_angle = 180 / PI * temp_angle;
        
        if(near.X() > 0)
            temp_angle = 0 - temp_angle;

        cout<<"-----Test Programme for angle and distance-----" << endl;
        cout<<"Temp Angle to the destination is : " << temp_angle << endl;
        cout<<"Temp Distance to the destination is : " << temp_dist << endl;
        //waitHere();
        
         //if(temp_angle > -3 && temp_angle <0)
         //          temp_angle = -5;
        // if(temp_angle < 3 && temp_angle > 0)
         //          temp_angle = 5;   
        
        // this step is to reach the narrow door place
        ExecutionAndGo(robot, temp_angle, temp_dist);
        
        //get a new current view
        CurrentView = scanAndSaveView(sick, number);
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
        plotObjects(viewFileName, myrobot.getRobot(), CurrentView);   
        CurrentView =  BoundaryByExits(CurrentView);
        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);   
        number++;
        if(isExit(CurrentView, relat_flag))
        {
            if(relat_flag == 1)
                angle = 90;
            if(relat_flag == 2)
                angle = -90;
   
            ExecutionAndGo(robot, angle, 0); // turn left
            CurrentView = scanAndSaveView(sick, number);
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
            plotObjects(viewFileName, myrobot.getRobot(), CurrentView);   
            CurrentView =  BoundaryByExits(CurrentView);
            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
            plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);   
            number++;
            avoidConvex(robot, CurrentView);
        }
        else
        {
            //carry on moving
            do
            { 
                ExecutionAndGo(robot, 0, 200); // turn left
                CurrentView = scanAndSaveView(sick, number);
                sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
                plotObjects(viewFileName, myrobot.getRobot(), CurrentView);   
                CurrentView =  BoundaryByExits(CurrentView);
                sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                 plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);   
                 number++;
            }while(!isExit(CurrentView, relat_flag));
            avoidConvex(robot, CurrentView);
        }
}

void PathStraight(ArRobot& robot, ArSick& sick, vector<Object> CV, double dist, int relat_flag)
{
        MyRobot myrobot(0,0);
        double angle;
        double distance;
        double temp_angle; // temp position where it moves to first
        double temp_dist;    // temp position distance
        //double temp_x;        // temp x
        //double temp_y;        // temp y
        
        char viewFileName[100];
        char rebuildFileName[100];
        char near_flag = 0;
        
        
        vector<Object> CurrentView;
        
        Point temp_point;
        Point ref2P1;
        Point ref1P2; 
        Point near;
        Point temp_aim;
       
        
        
        //first of all, the robot needs to reach and pass through a door or narrow place      
        //calculating the narrow space position
        //the first step is straight forward
        temp_point = farthestPoint(CV, relat_flag);
        near = avoidObstracle(CV, temp_point);
        
        if(near.X() != 0 && near.Y() != 0)
        {
            if(near.X() < 0)
            near.set(near.X()+300, near.Y()+300);
            if(near.X() > 0)
            near.set(near.X()-300, near.Y()+300);  

            temp_dist = sqrt(near.X() * near.X() + near.Y() * near.Y());
            temp_angle = acos(near.Y() / temp_dist); 
            temp_angle = 180 / PI * temp_angle;

            if(near.X() > 0)
                temp_angle = 0 - temp_angle;
            
             // this step is to reach the narrow door place
             ExecutionAndGo(robot, temp_angle, temp_dist);
             near_flag = 1;
        }
        else
        {
            temp_dist = sqrt(temp_point.X() * temp_point.X() + temp_point.Y() * temp_point.Y());
            temp_angle = acos(temp_point.Y() / temp_dist); 
            temp_angle = 180 / PI * temp_angle;
            
            if(temp_point.X() > 0)
                temp_angle = 0 - temp_angle;
            
            // this step is to reach the narrow door place
            ExecutionAndGo(robot, temp_angle, dist/2);
        }

        cout<<"-----Test Programme for angle and distance-----" << endl;
        //cout<<"Temp Angle to the destination is : " << temp_angle << endl;
        //cout<<"Temp Distance to the destination is : " << temp_dist << endl;
        //waitHere();
        
        CurrentView = scanAndSaveView(sick, number);
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
        plotObjects(viewFileName, myrobot.getRobot(), CurrentView);   
        CurrentView =  BoundaryByExits(CurrentView);
        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);   
        number++;
        
        
        //turn a particular angle forward to the next position (left or right)
         if(isExit(CurrentView, relat_flag))
        {
             if(near_flag != 1)
             {
                if(relat_flag == 1)
                    angle = 70;
                if(relat_flag == 2)
                    angle = -70;
             }
             else
             {
                    if(relat_flag == 1)
                        angle = 30;
                    if(relat_flag == 2)
                        angle = -30;
                    near_flag = 0;
             }

            ExecutionAndGo(robot, angle, 0); // turn left
            CurrentView = scanAndSaveView(sick, number);
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
            plotObjects(viewFileName, myrobot.getRobot(), CurrentView);   
            CurrentView =  BoundaryByExits(CurrentView);
            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
            plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);   
            number++;
        }
        else
        {
            //carry on moving
            do
            { 
                ExecutionAndGo(robot, 0, 200); // turn left
                CurrentView = scanAndSaveView(sick, number);
                sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
                plotObjects(viewFileName, myrobot.getRobot(), CurrentView);   
                CurrentView =  BoundaryByExits(CurrentView);
                sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                 plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);   
                 number++;
            }while(!isExit(CurrentView, relat_flag));
        }
        
        /*
        CurrentView = scanAndSaveView(sick, number);
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", number, ".png");
        plotObjects(viewFileName, myrobot.getRobot(), CurrentView);   
        CurrentView =  BoundaryByExits(CurrentView);
        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
         plotObjects(rebuildFileName, myrobot.getRobot(), CurrentView);   
         number++;
         */
            
         temp_point =  farthestPoint(CurrentView, relat_flag);
         temp_dist = sqrt(temp_point.X() * temp_point.X() + temp_point.Y() * temp_point.Y());
         temp_angle = acos(temp_point.Y() / temp_dist); 
         temp_angle = 180 / PI * temp_angle;
        
         // moving to the position, second step 
         if(temp_point.X() > 0)
             temp_angle = 0 - temp_angle;
        
         ExecutionAndGo(robot, temp_angle, dist/2); // turn left
         
}

void PathRecognistion(ArRobot& robot, ArSick& sick, vector<Object> CV, 
                                                                            vector<double> info, double midDist, int relat_flag)
{
     MyRobot myrobot(0,0);
    char viewFileName[100];
    char rebuildFileName[100];
    double temp_angle;
    double temp_dist;
    double midPointDist;
    pair<double,double> distAndAngl; //first is distance, second is angle
    
    int threshold = 500;
    
    Point inter;
    Point temp_point;
    Point near;
    Object refObj;

    vector<Object> currentView;
    vector<Point> refs;
    
    switch(relat_flag)
    {
        case 1: //finding the large one on left
                     refs = referenceBoundary(CV);//choose reference boundary
                     refObj.set(refs[0].X(), refs[0].Y(), refs[1].X(), refs[1].Y(), 1);

                     midPointDist = midPointOfObj(refObj);
                     if(abs(midPointDist - midDist) <= threshold)
                     {
                            //get the P2 and moving forward 
                            inter.set(refs[1].X() + 300, refs[1].Y() + 300);
                            
                            //turning a particular angle
                            distAndAngl = DistAndAngle(inter);
                            //ExecutionAndGo(robot, temp_angle, temp_dist);
                            ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                            ExecutionAndGo(robot, info[2], 0);
                     }
                     else
                     {
                            refObj = SearchBoundary(CV, midDist, relat_flag);
                            inter.set(refObj.X2() + 300, refObj.Y2() + 300);
                            
                            //turning a particular angle
                            distAndAngl = DistAndAngle(inter);
                            //ExecutionAndGo(robot, temp_angle, temp_dist);
                            ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                            ExecutionAndGo(robot, info[2], 0);
                     }
                     
                     currentView = scanAndSaveView(sick, number);
                     currentView = BoundaryByExits(currentView);
                     number++;
                     sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                     plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                     
                     //Judge there is no an exit front of robot

                      temp_point =  farthestPoint(currentView, relat_flag);
                      near = avoidObstracle(currentView, temp_point);
                      if(near.X() > 0)
                          temp_point.set(near.X() - 300, near.Y() + 300);
                      if(near.X() < 0)
                          temp_point.set(near.X() + 300, near.Y() + 300);
                      
                      distAndAngl = DistAndAngle(temp_point);
                      ExecutionAndGo(robot, distAndAngl.second, info[1]);
                      break;
        case 2: //finding the large one on right
                     refs = referenceBoundary(CV);//choose reference boundary
                    refObj.set(refs[2].X(), refs[2].Y(), refs[3].X(), refs[3].Y(), 1);

                     midPointDist = midPointOfObj(refObj);
                      if(abs(midPointDist - midDist) <= threshold)
                      {
                           //get the P2 and moving forward
                           inter.set(refs[2].X() - 300, refs[2].Y() + 300);
                           //turning a particular angle
                           distAndAngl = DistAndAngle(inter);

                           //ExecutionAndGo(robot, temp_angle, temp_dist);
                           ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                           ExecutionAndGo(robot, info[2], 0);
                      }
                      else
                      {
                            refObj = SearchBoundary(CV, midDist, relat_flag);
                            inter.set(refObj.X1() - 300, refObj.Y1() + 300);
                            //turning a particular angle
                            distAndAngl = DistAndAngle(inter);
                            //ExecutionAndGo(robot, temp_angle, temp_dist);
                            //ExecutionAndGo(robot, 0,400);
                            ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                           
                            ExecutionAndGo(robot, info[2], 0);
                      }
                     currentView = scanAndSaveView(sick, number);
                     currentView = BoundaryByExits(currentView);
                     number++;
                     sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                     plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                     
                     
                      temp_point =  farthestPoint(currentView, relat_flag);
                      near = avoidObstracle(currentView, temp_point);
                      if(near.X() > 0)
                          temp_point.set(near.X() - 300, near.Y() + 300);
                      if(near.X() < 0)
                          temp_point.set(near.X() + 300, near.Y() + 300);
                      distAndAngl = DistAndAngle(temp_point);
                     
                      ExecutionAndGo(robot, distAndAngl.second, info[1]);
                      break;
        default:break;
    }
}


void PathViewRecognistion(ArRobot& robot, ArSick& sick, vector<Object> MpathView, vector<Object> CpathView,
                                                                            vector<double> info, Point pos, int numb, int relat_flag)
{
     MyRobot myrobot(0,0);
    char viewFileName[100];
    char rebuildFileName[100];
    double temp_angle;
    double temp_dist;
    unsigned char midFlag = 0;
    unsigned char sr_flag = 0;
    
    pair<double,double> distAndAngl; //first is distance, second is angle
    
    int threshold = 500;
    
    Point inter;
    Point temp_point;
    Point near;
    Object refObj;

    pair<int, int> exitObjsInMem;
    vector<int> Mem;
    vector<int> Cur;
    vector<Object> ExitObjs;
    vector<Object> currentView;
    vector<Object> PathView;
    //vector<Point> refs;
    
    Mem = ObjectPosition(MpathView); 
    Cur = ObjectPosition(CpathView);
    
    
    
    cout<<"This is to show the objects positions" << endl;
    for(int i = 0; i < Mem.size(); i++)
        cout<<"The Memory object position is: " << Mem[i]<<endl;
    for(int i = 0; i < Cur.size(); i++)
        cout<<"The Current object position is: " << Cur[i]<<endl;
    
    
    switch(relat_flag)
    {
        case 1: //matching and get a boundary to guide it move
                    
                     if(Cur == Mem) // if matching
                     {
                            cout << "**** Successfully Matching ****" << endl;
                             
                            exitObjsInMem = exitFromMem(MpathView, pos, relat_flag);
                            ExitObjs = exitOfObjects(CpathView, exitObjsInMem.first, exitObjsInMem.second);
                            
                            //temp_point = farthestPoint(CpathView, relat_flag);
                            //near = avoidObstracle(CpathView, temp_point);
                            if(ExitObjs[0].X1() - ExitObjs[1].X2() < 0)
                                inter.set(ExitObjs[0].X2() + 350, ExitObjs[0].Y2() + 300); //get the P2 and moving forward 
                            else
                                 inter.set(ExitObjs[1].X2() + 350, ExitObjs[1].Y2() + 300); //get the P2 and moving forward 
                            //turning a particular angle
                            distAndAngl = DistAndAngle(inter);
                            //ExecutionAndGo(robot, temp_angle, temp_dist);
                            ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                            ExecutionAndGo(robot, info[2], 0);
                     }
                     else // if not equal
                     {
                            if(matchObjPos(Mem, Cur))// matching a part of elements
                            {
number0:                   cout << "---- still mathcing successfully ----" << endl;
                                    exitObjsInMem = exitFromMem(MpathView, pos, relat_flag);
                                    ExitObjs = exitOfObjects(CpathView, exitObjsInMem.first, exitObjsInMem.second);

                                    //temp_point = farthestPoint(CpathView, relat_flag);
                                    //near = avoidObstracle(CpathView, temp_point);
                                    if(ExitObjs[0].X1() - ExitObjs[1].X2() < 0)
                                        inter.set(ExitObjs[0].X2() + 350, ExitObjs[0].Y2() + 300); //get the P2 and moving forward 
                                    else
                                        inter.set(ExitObjs[1].X2() + 350, ExitObjs[1].Y2() + 300); //get the P2 and moving forward 
                                    //turning a particular angle
                                    distAndAngl = DistAndAngle(inter);
                                    //ExecutionAndGo(robot, temp_angle, temp_dist);
                                    ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                    ExecutionAndGo(robot, info[2], 0);
                            }
                            else // searching
                            {    
                                     ExecutionAndGo(robot, 20, 0);
                                     currentView = scanAndSaveView(sick, number);
                                     currentView = BoundaryByExits(currentView);
                                     PathView = pathView(currentView, relat_flag);
                                     number++;
                                     Cur = ObjectPosition(PathView);
                                     if(Cur == Mem)
                                         goto number0;
                            }
                     }
                     
                     currentView = scanAndSaveView(sick, number);
                     currentView = BoundaryByExits(currentView);
                     number++;
                     sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                     plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                     
                     //Judge there is no an exit front of robot

                      temp_point =  farthestPoint(currentView, relat_flag);
                      near = avoidObstracle(currentView, temp_point);
                      if(near.X() > 0)
                          temp_point.set(near.X() - 350, near.Y() + 300);
                      if(near.X() < 0)
                          temp_point.set(near.X() + 350, near.Y() + 300);
                      
                      distAndAngl = DistAndAngle(temp_point);
                      ExecutionAndGo(robot, distAndAngl.second, info[1]);
                      break;
        case 2:  //matching and get a boundary to guide it move
                    if(numb != 5)
                    {
                                    if(Cur == Mem)
                                    {
                                          cout << "**** Successfully Matching ****" << endl;

                                          exitObjsInMem = exitFromMem(MpathView, pos, relat_flag);
                                          ExitObjs = exitOfObjects(CpathView, exitObjsInMem.first, exitObjsInMem.second);

                                          //temp_point = farthestPoint(CpathView, relat_flag);
                                          //near = avoidObstracle(CpathView, temp_point);

                                          if(ExitObjs[1].X2() - ExitObjs[0].X1()> 0 && ExitObjs[1].X1() > 0)
                                              inter.set(ExitObjs[1].X1() - 350, ExitObjs[1].Y1() + 300); //get the P2 and moving forward 
                                          else
                                               inter.set(ExitObjs[0].X1() - 350, ExitObjs[0].Y1() + 300); //get the P2 and moving forward 
                                         //turning a particular angle
                                         distAndAngl = DistAndAngle(inter);

                                         //ExecutionAndGo(robot, temp_angle, temp_dist);
                                         ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                         ExecutionAndGo(robot, info[2], 0);
                                    }
                                    else
                                    {
                                        if(matchObjPos(Mem, Cur))
                                        {
              number1:                   cout << "---- still mathcing successfully ----" << endl;
                                                  exitObjsInMem = exitFromMem(MpathView, pos, relat_flag);
                                                  ExitObjs = exitOfObjects(CpathView, exitObjsInMem.first, exitObjsInMem.second);

                                                  //temp_point = farthestPoint(CpathView, relat_flag);
                                                  //near = avoidObstracle(CpathView, temp_point);

                                                  if(ExitObjs[1].X2() - ExitObjs[0].X1() > 0)
                                                      inter.set(ExitObjs[1].X1() - 350, ExitObjs[1].Y1() + 300); //get the P2 and moving forward 
                                                  else
                                                       inter.set(ExitObjs[0].X1() - 350, ExitObjs[0].Y1() + 300); //get the P2 and moving forward 
                                                 //turning a particular angle
                                                 distAndAngl = DistAndAngle(inter);

                                                 //ExecutionAndGo(robot, temp_angle, temp_dist);
                                                 ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                                 ExecutionAndGo(robot, info[2], 0);

                                        }
                                        else
                                        {
                                                   //ExecutionAndGo(robot, -20, 0);
                                                   currentView = scanAndSaveView(sick, number);
                                                   currentView = BoundaryByExits(currentView);
                                                   PathView = pathView(currentView, relat_flag);
                                                   number++;
                                                   Cur = ObjectPosition(PathView);
                                                   if(Cur == Mem)
                                                       goto number1;
                                                   else
                                                   {
                                                       if(numb == 3)
                                                       {
                                                            //tracking distance 
                                                            //ExecutionAndGo(robot, 40, 0); //back to the previous orient
                                                            for(int i = 0; i < PathView.size(); i++)
                                                            {
                                                                int Obj_pos = SingleObjectPosition(PathView[i]);
                                                                if(Obj_pos == 3)
                                                                    inter.set(PathView[i].X2() + 300, PathView[i].Y2() + 300);
                                                            }

                                                            distAndAngl = DistAndAngle(inter);
                                                            if(distAndAngl.first < 3500)
                                                            {
                                                                ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                                                midFlag = 1;
                                                            }
                                                            else
                                                            {
                                                                inter.set(PathView.back().X1() - 300, PathView.back().Y1() + 200);
                                                                distAndAngl = DistAndAngle(inter);
                                                                 ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                                            }
                                                            
                                                            if(midFlag == 1)
                                                            {
                                                                currentView = scanAndSaveView(sick, number);
                                                                currentView = BoundaryByExits(currentView);
                                                                PathView = pathView(currentView, relat_flag);
                                                                number++;

                                                                inter.set(currentView.back().X1() - 400, currentView.back().Y1() + 100);
                                                                distAndAngl = DistAndAngle(inter);


                                                                 ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                                            }
                                                            //ExecutionAndGo(robot, 10, 500);
                                                            ExecutionAndGo(robot, info[2], 0);
                                                       }
                                                   }
                                        }
                                    }

                                   currentView = scanAndSaveView(sick, number);
                                   currentView = BoundaryByExits(currentView);
                                   number++;
                                   sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                   plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   


                                    temp_point =  farthestPoint(currentView, relat_flag);
                                    near = avoidObstracle(currentView, temp_point);
                                    if(near.X() > 0)
                                        temp_point.set(near.X() - 350, near.Y() + 300);
                                    if(near.X() < 0)
                                        temp_point.set(near.X() + 350, near.Y() + 300);
                                    distAndAngl = DistAndAngle(temp_point);
                                    ExecutionAndGo(robot, distAndAngl.second, 0);
                                    
                   sr1:          currentView = scanAndSaveView(sick, number);
                                   currentView = BoundaryByExits(currentView);
                                   number++;
                                   sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                   plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                                   
                                   if(!ObstacleFront(currentView, info[1]))//no obstruct
                                        ExecutionAndGo(robot, 0, info[1]);
                                   else // obstructed by surface
                                   {
                                       if(sr_flag == 0)
                                       {
                                          ExecutionAndGo(robot, -35, 0);
                                          sr_flag == 1;
                                       }
                                       else
                                          ExecutionAndGo(robot, 70, 0);
                                        goto sr1;
                                   }
                    }
                    else
                    {
                        //find an exit on the right and go       
                        while(!isExit(currentView, relat_flag))
                        {
                                   ExecutionAndGo(robot, 0, 200);
                                   currentView = scanAndSaveView(sick, number);
                                   currentView = BoundaryByExits(currentView);
                                   number++;
                                   sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                   plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                        }
                        
                        //execution
                        temp_point =  farthestPoint(currentView, relat_flag);
                        near = avoidObstracle(currentView, temp_point);
                        if(near.X() > 0)
                            temp_point.set(near.X() - 350, near.Y() + 300);
                        if(near.X() < 0)
                            temp_point.set(near.X() + 350, near.Y() + 300);
                        distAndAngl = DistAndAngle(temp_point);
                        
                        ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                        ExecutionAndGo(robot, 0, info[0]);
                    }
                      break;
        default:break;
    }
}

void pathPlanBoundandView(ArRobot& robot, ArSick& sick, vector<Object> mBView, vector<Object> CV,
                                                                            vector<double> info, Point pos, double midDist,  int numb, int relat_flag)
{
        MyRobot myrobot(0,0);
        char viewFileName[100];
        char rebuildFileName[100];
        double temp_angle;
        double temp_dist;
        double midPointDist;
        //unsigned char midFlag = 0;

        pair<double,double> distAndAngl; //first is distance, second is angle

        int threshold = 500;

        Point inter;
        Point temp_point;
        Point near;
        Object refObj1,refObj2;

        pair<int, int> exitObjsInMem;
        vector<int> Mem;
        vector<int> Cur;
        vector<Object> currentBoundary;
        vector<Object> currentView;
        vector<Point> refs;

        //vector< vector<Point> > allPoints;
        
        currentBoundary = BoundaryByGroup(CV);
        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/BoundaryView-", numb, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), currentBoundary);   

        switch(numb)
        {
            case 0: //left reference/big boundary
                        refs = referenceBoundary(currentBoundary);
                        inter.set(refs[1].X() + 350, refs[1].Y() + 300);
                        //turning a particular angle
                        distAndAngl = DistAndAngle(inter);

                        ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                        ExecutionAndGo(robot, info[2], 0);
                        
                         //turn and move to destination
                        currentView = scanAndSaveView(sick, number);
                        currentView = BoundaryByGroup(currentView);
                        number++;
                        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                        plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                          
                         temp_point =  farthestPoint(currentView, relat_flag);
                         near = avoidObstracle(currentView, temp_point);
                         
                         if((temp_point.X() - near.X()) > 0)
                             temp_point.set(near.X() - 350, near.Y() + 300);
                         if((temp_point.X() - near.X()) < 0)
                             temp_point.set(near.X() + 350, near.Y() + 300);
                         
                         //temp_point.set(currentView[currentView.size()-1].X1() - 350, currentView[currentView.size()-1].Y1() + 300);
                         distAndAngl = DistAndAngle(temp_point);

                         ExecutionAndGo(robot, distAndAngl.second, info[1]);
                         break;
                
            case 1:     //left reference/big boundary
                          refs = referenceBoundary(currentBoundary);
                          inter.set(refs[1].X()+350, refs[1].Y()+300);
                          /*
                          if(inter.X() < 0)
                            inter.set(refs[1].X() + 350, refs[1].Y() + 300);
                          else
                          {
                              temp_point.set(currentView[0].X2(), currentView[0].Y2());
                              inter.set(temp_point.X() + 350, temp_point.Y() + 300);
                          }
                          */
                          //turning a particular angle
                          distAndAngl = DistAndAngle(inter);
  
                          ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                          ExecutionAndGo(robot, info[2], 0);
                          
                           //turn and move to destination
                          currentView = scanAndSaveView(sick, number);
                          currentView = BoundaryByGroup(currentView);
                          number++;
                          sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                          plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                          /*
                         temp_point =  farthestPoint(currentView, relat_flag);
                         near = avoidObstracle(currentView, temp_point);
                         
                         if(near.X() > 0)
                             temp_point.set(near.X() - 350, near.Y() + 300);
                         if(near.X() < 0)
                             temp_point.set(near.X() + 350, near.Y() + 300);
                         */
                         temp_point.set(currentView[currentView.size()-1].X1() - 350, currentView[currentView.size()-1].Y1() + 300);
                         distAndAngl = DistAndAngle(temp_point);

                         ExecutionAndGo(robot, distAndAngl.second, info[1]);
                         //ExecutionAndGo(robot, 0, info[0]);
                        
                         break;
            case 2:      //right reference/short boundary 
                         inter.set(currentBoundary[currentBoundary.size()-1].X1() - 350, currentBoundary[currentBoundary.size()-1].Y1() + 300);
                         //turning a particular angle
                         distAndAngl = DistAndAngle(inter);
 
                         ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                         ExecutionAndGo(robot, info[2], 0);
                         
                        //turn and move to destination
                        currentView = scanAndSaveView(sick, number);
                        currentView = BoundaryByGroup(currentView);
                        number++;
                        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                        plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   

                         temp_point =  farthestPoint(currentView, relat_flag);
                         near = avoidObstracle(currentView, temp_point);
                         if(near.X() > 0)
                             temp_point.set(near.X() - 350, near.Y() + 300);
                         if(near.X() < 0)
                             temp_point.set(near.X() + 350, near.Y() + 300);
                         distAndAngl = DistAndAngle(temp_point);

                         ExecutionAndGo(robot, distAndAngl.second, info[1]);
                         //ExecutionAndGo(robot, 0, info[1]);
                
                         break;
            case 3:                         
                        refs = referenceBoundary(currentBoundary);
                        refObj1.set(refs[0].X(),refs[0].Y(), refs[1].X(),refs[1].Y(),1);
                        refObj2.set(refs[2].X(),refs[2].Y(), refs[3].X(),refs[3].Y(),2);
                        if(refObj1.X1() < 0 && refObj1.X2() > 0)
                        {
                                inter.set(refObj1.X2() + 350, refObj1.Y2() + 300);
                                //turning a particular angle
                                distAndAngl = DistAndAngle(inter);

                                ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                //ExecutionAndGo(robot, info[2], 0);
                                
                                currentView = scanAndSaveView(sick, number);
                                currentView = BoundaryByGroup(currentView);
                                number++;
                                sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                                
                                inter.set(currentView[currentView.size()-1].X1() - 350, currentView[currentView.size()-1].Y1() + 100);
                                //turning a particular angle
                                distAndAngl = DistAndAngle(inter);

                                ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                ExecutionAndGo(robot, info[2], 0);
                                
                        }
                        else
                        {
                          if(refObj2.X1() < 0 && refObj2.X2() > 0)
                          {
                                  inter.set(refObj2.X2() + 350, refObj2.Y2() + 300);
                                  //turning a particular angle
                                  distAndAngl = DistAndAngle(inter);
  
                                  ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                  //ExecutionAndGo(robot, info[2], 0);
                                  
                                  currentView = scanAndSaveView(sick, number);
                                  currentView = BoundaryByGroup(currentView);
                                  number++;
                                  sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                  plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                                  
                                  inter.set(currentView[currentView.size()-1].X1() - 350, currentView[currentView.size()-1].Y1() + 100);
                                  //turning a particular angle
                                  distAndAngl = DistAndAngle(inter);
  
                                  ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                  ExecutionAndGo(robot, info[2], 0);                                 
                                  
                          }
                          else
                          {
                                  currentView = scanAndSaveView(sick, number);
                                  currentView = BoundaryByGroup(currentView);
                                  number++;
                                  sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                  plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                                  
                                  inter.set(currentView[currentView.size()-1].X1() - 350, currentView[currentView.size()-1].Y1() + 300);
                                  //turning a particular angle
                                  distAndAngl = DistAndAngle(inter);

                                  ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                  if(distAndAngl.first > 450)
                                    ExecutionAndGo(robot, info[2], 0);
                                  else
                                  {
                                      currentView = scanAndSaveView(sick, number);
                                      currentView = BoundaryByGroup(currentView);
                                      number++;
                                      sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                      plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                                      
                                      inter.set(currentView[currentView.size()-1].X1() - 350, currentView[currentView.size()-1].Y1() + 300);
                                      //turning a particular angle
                                      distAndAngl = DistAndAngle(inter);
    
                                      ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                                      ExecutionAndGo(robot, info[2], 0);
                                  }
                          }
                        }
                          
                          currentView = scanAndSaveView(sick, number);
                          currentView = BoundaryByGroup(currentView);
                          number++;
                          sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                          plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                                  
                         temp_point =  farthestPoint(currentView, relat_flag);
                         near = avoidObstracle(currentView, temp_point);
                         if(near.X() > 0)
                             temp_point.set(near.X() - 350, near.Y() + 300);
                         if(near.X() < 0)
                             temp_point.set(near.X() + 350, near.Y() + 300);
                         distAndAngl = DistAndAngle(temp_point);

                         ExecutionAndGo(robot, distAndAngl.second, info[1]);
                         //ExecutionAndGo(robot, 0, info[0]);
                         break;
            case 4: //right reference boundary
                         inter.set(currentBoundary[currentBoundary.size()-1].X1() - 450, currentBoundary[currentBoundary.size()-1].Y1() + 300);
                         //turning a particular angle
                         distAndAngl = DistAndAngle(inter);
 
                         ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                         ExecutionAndGo(robot, info[2], 0);
                         
                        //turn and move to destination
                        currentView = scanAndSaveView(sick, number);
                        currentView = BoundaryByGroup(currentView);
                        number++;
                        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                        plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   

                         //temp_point =  farthestPoint(currentView, relat_flag);
                         //near = avoidObstracle(currentView, temp_point);
                         
                         temp_point.set(currentView[0].X2() - 300, currentView[0].Y2() + 300);
                        
                         distAndAngl = DistAndAngle(temp_point);

                         ExecutionAndGo(robot, distAndAngl.second, info[1]);
                         //ExecutionAndGo(robot, 0, info[0]);
                         break;
            case 5: 
                        //find an exit on the right and go       
                        currentView = currentBoundary;
                        
                        while(!isExit(currentView, relat_flag))
                        {
                                   ExecutionAndGo(robot, 0, 350);
                                   currentView = scanAndSaveView(sick, number);
                                   currentView = BoundaryByGroup(currentView);
                                   number++;
                                   sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                   plotObjects(rebuildFileName, myrobot.getRobot(), currentView);   
                        }
                        ExecutionAndGo(robot, -90, 0);
                                                //execution
                        temp_point =  farthestPoint(currentView, relat_flag);
                        near = avoidObstracle(currentView, temp_point);
                        if(near.X() > 0)
                            temp_point.set(near.X() - 350, near.Y() + 300);
                        if(near.X() < 0)
                            temp_point.set(near.X() + 350, near.Y() + 300);
                        distAndAngl = DistAndAngle(temp_point);
                        
                        ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                        ExecutionAndGo(robot, 0, info[0]);
                         break;      
                        
            case 6:break;   
            default:break;
        }
}


void returnPathPlanOrignalVersion(ArRobot& robot, ArSick& sick, Point pos, int num, int relat_flag)
{
        MyRobot myrobot(0,0);
        char viewFileName[100];
        char rebuildFileName[100];
        double temp_angle;
        double temp_dist;
        
        double Objlength;
        double ObjAngl;
        double k;

        pair<double,double> distAndAngl; //first is distance, second is angle

        Point mid;
        Point temp_point;
        Point near;
        Object refObj;

        vector<Object> currentBoundary;
        vector<Object> currentView;
        vector<Object> Cview;
        vector<Point> refs;
        vector<Object> path;
  
        
        Cview = scanAndSaveView(sick, number);
        currentView = BoundaryByGroup(Cview);

        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", num, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
        //number++;
        
        //calculate the ideal path in current view
        path = routeInView(currentView, pos);
        //find intersected boundary
        refObj = IntersectForReturn(currentView, pos) ;

        if(refObj.getID() != 0)
        {
                if(pos.X() < 0)
                      temp_point.set(refObj.X2() + 800, refObj.Y2());
                else
                    if(pos.X() > 0)
                            temp_point.set(refObj.X1() - 800, refObj.Y1());

                //calculating the angle and distance 
                distAndAngl = DistAndAngle(temp_point);

                //execution and move to the point
                ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);

                //re-calculate the position in current view
                pos.set(pos.X() - temp_point.X(), pos.Y() - temp_point.Y());
        }
        else
        {
                // no intersection 
                distAndAngl = DistAndAngle(pos);

                ExecutionAndGo(robot, distAndAngl.second, 0);
                //Then correct its orientation
                Cview = scanAndSaveView(sick, number);
                currentView = BoundaryByGroup(Cview);

                sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                number++;

                for(int i = 0; i < currentView.size(); i++)
                {
                        if((currentView[i].X1() < 0 && currentView[i].X2() > 0) || (currentView[i].X1() > 0 && currentView[i].X2() < 0))
                            refObj = currentView[i];
                        //mid point of the object
                        mid.set((refObj.X1() + refObj.X2()) / 2, (refObj.Y1() + refObj.Y2()) / 2);
                }

                if(distanceOftwoP(mid, Point (0,0)) >= distAndAngl.first)// no any obstruct
                {
                        ExecutionAndGo(robot, 0, distAndAngl.first / 2);

                        //Then correct its orientation
                        Cview = scanAndSaveView(sick, number);
                        currentView = BoundaryByGroup(Cview);

                        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                        plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                        number++;

                        ExecutionAndGo(robot, 0, distAndAngl.first / 2);

                }
                else
                {
                        Cview = scanAndSaveView(sick, number);
                        currentView = BoundaryByGroup(Cview);

                        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                        plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                        number++;
                        
                        
                }
        }


        //Then, at the place, getting a new view
        Cview = scanAndSaveView(sick, number);
        currentView = BoundaryByGroup(Cview);

        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
        number++;

        //confirm where the exit is
        path = routeInView(currentView, pos);

        temp_point.set(path[0].X2(), path[0].Y2());
        distAndAngl = DistAndAngle(temp_point);
        ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);

        temp_angle = AngleOfNext(Point (0,0), Point (path[1].X1(), path[1].Y1()), Point (path[1].X2(), path[1].Y2()));
        temp_dist = distanceOftwoP(Point (path[1].X1(), path[1].Y1()), Point (path[1].X2(), path[1].Y2()));

        ExecutionAndGo(robot, temp_angle, 0);


        refObj = currentView.back();

        Objlength = distanceOftwoP(Point (refObj.X1(), refObj.Y1()), Point (refObj.X2(), refObj.Y2()));
        ObjAngl = acos(abs(refObj.Y1() - refObj.Y2()) /   Objlength); 
        ObjAngl = 180 / PI * ObjAngl;

        k = (refObj.Y1() - refObj.Y2()) / (refObj.X1() - refObj.X2());
        // moving to the position, second step 
         if(k > 0)
                 ObjAngl = 0 -  ObjAngl;
        ExecutionAndGo(robot, ObjAngl, 0);

        //Then correct its orientation
        Cview = scanAndSaveView(sick, number);
        currentView = BoundaryByGroup(Cview);

        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
        number++;

        for(int i = 0; i < currentView.size(); i++)
        {
                if((currentView[i].X1() < 0 && currentView[i].X2() > 0) || (currentView[i].X1() > 0 && currentView[i].X2() < 0))
                    refObj = currentView[i];
                //mid point of the object
                mid.set((refObj.X1() + refObj.X2()) / 2, (refObj.Y1() + refObj.Y2()) / 2);
        }

        if(distanceOftwoP(mid, Point (0,0)) < temp_dist)
        {
            mid.set((currentView.back().X1() + currentView.back().X2()) / 2, (currentView.back().Y1() + currentView.back().Y2()) / 2);
            distAndAngl = DistAndAngle(mid);
            ExecutionAndGo(robot, distAndAngl.second, 0);
        }

        ExecutionAndGo(robot, 0, temp_dist);
 }


void returnPathPlan(ArRobot& robot, ArSick& sick, Point pos, int num, int relat_flag)
{
        MyRobot myrobot(0,0);
        char viewFileName[100];
        char rebuildFileName[100];
        double temp_angle;
        double temp_dist;
        
        double Objlength;
        double ObjAngl;
        double k;

        pair<double,double> distAndAngl; //first is distance, second is angle

        Point mid;
        Point temp_point;
        Point near;
        Object refObj;

        vector<Object> currentBoundary;
        vector<Object> currentView;
        vector<Object> Cview;
        vector<Point> refs;
        vector<Object> path;
  
        
        currentView = scanAndSaveView(sick, number);
        currentView = BoundaryByGroup(currentView);

        sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", num, ".png");
        plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
        //number++;
        
        switch(num)
        {
            case 1:    //calculate the ideal path in current view
                            path = routeInView(currentView, pos);
                            //find intersected boundary
                            refObj = IntersectForReturn(currentView, pos) ;

                            if(refObj.getID() != 0)
                            {
                                    if(pos.X() < 0)
                                          temp_point.set(refObj.X2() + 800, refObj.Y2());
                                    else
                                        if(pos.X() > 0)
                                                temp_point.set(refObj.X1() - 800, refObj.Y1());

                                    //calculating the angle and distance 
                                    distAndAngl = DistAndAngle(temp_point);

                                    //execution and move to the point
                                    ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);

                                    //re-calculate the position in current view
                                    pos.set(pos.X() - temp_point.X(), pos.Y() - temp_point.Y());
                            }

                            //Then, at the place, getting a new view
                            currentView = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(currentView);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;

                            //confirm where the exit is
                            path = routeInView(currentView, pos);

                            temp_point.set(path[0].X2(), path[0].Y2());
                            distAndAngl = DistAndAngle(temp_point);
                            ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first + 400);

                            temp_angle = AngleOfNext(Point (0,0), Point (path[1].X1(), path[1].Y1()), Point (path[1].X2(), path[1].Y2()));
                            temp_dist = distanceOftwoP(Point (path[1].X1(), path[1].Y1()), Point (path[1].X2(), path[1].Y2()));

                            ExecutionAndGo(robot, temp_angle, 0);


                            refObj = currentView.back();

                            Objlength = distanceOftwoP(Point (refObj.X1(), refObj.Y1()), Point (refObj.X2(), refObj.Y2()));
                            ObjAngl = acos(abs(refObj.Y1() - refObj.Y2()) /   Objlength); 
                            ObjAngl = 180 / PI * ObjAngl;

                            k = (refObj.Y1() - refObj.Y2()) / (refObj.X1() - refObj.X2());
                            // moving to the position, second step 
                             if(k > 0)
                                     ObjAngl = 0 -  ObjAngl;
                            ExecutionAndGo(robot, ObjAngl, 0);

                            //Then correct its orientation
                            currentView = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(currentView);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;

                            for(int i = 0; i < currentView.size(); i++)
                            {
                                    if((currentView[i].X1() < 0 && currentView[i].X2() > 0) || (currentView[i].X1() > 0 && currentView[i].X2() < 0))
                                        refObj = currentView[i];
                                    //mid point of the object
                                    mid.set((refObj.X1() + refObj.X2()) / 2, (refObj.Y1() + refObj.Y2()) / 2);
                            }

                            if(distanceOftwoP(mid, Point (0,0)) < temp_dist)
                            {
                                mid.set((currentView.back().X1() + currentView.back().X2()) / 2, (currentView.back().Y1() + currentView.back().Y2()) / 2);
                                distAndAngl = DistAndAngle(mid);
                                ExecutionAndGo(robot, distAndAngl.second, 0);
                            }

                            ExecutionAndGo(robot, 0, temp_dist);
                            
                            //ExecutionAndGo(robot, 30, 0);
                            break;
            case 2:
                            //calculate the ideal path in current view
                            path = routeInView(currentView, pos);
                            Cview = scanAndSaveView(sick, number);
       
                            refObj = Cview.back();
                            Objlength = distanceOftwoP(Point (refObj.X1(), refObj.Y1()), Point (refObj.X2(), refObj.Y2()));
                            ObjAngl = acos(abs(refObj.Y1() - refObj.Y2()) /   Objlength); 
                            ObjAngl = 180 / PI * ObjAngl;

                            k = (refObj.Y1() - refObj.Y2()) / (refObj.X1() - refObj.X2());
                            // moving to the position, second step 
                             if(k > 0)
                                     ObjAngl = 0 -  ObjAngl;
                            ExecutionAndGo(robot, ObjAngl, 0);
                            
                            Cview = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(Cview);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;
                            
                            refObj = Cview.back();
                            
                            ExecutionAndGo(robot, 0, refObj.Y1());
                            ExecutionAndGo(robot, 80, 0);
                            
                            pos.set(0, abs(pos.X()));
                            
                            Cview = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(Cview);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;
                            
                            refObj = IntersectForReturn(currentView, pos);
                            if(refObj.getID() != 0)
                            {
                                temp_point.set(refObj.X1() - 600, refObj.Y1()); 
                            }
                            else
                            {
                                temp_point = avoidObstracle(currentView, pos);
                                if(temp_point.X() > 0)
                                    temp_point.set(temp_point.X() - 600, temp_point.Y());
                                else
                                    if(temp_point.X() < 0);
                                        temp_point.set(temp_point.X() + 600, temp_point.Y());
                            }
                            
                            distAndAngl = DistAndAngle(temp_point);
                            
                            ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                            
                            
                            
                            ExecutionAndGo(robot, 0, abs(pos.Y()) - distAndAngl.first);
                            break;
            case 3:
                           //calculate the ideal path in current view
                            path = routeInView(currentView, pos);
                            
                            refObj = IntersectForReturn(currentView, pos) ;

                            if(refObj.getID() != 0)
                            {
                                    if(pos.X() < 0)
                                          temp_point.set(refObj.X2() + 800, refObj.Y2() + 300);
                                    else
                                        if(pos.X() > 0)
                                                temp_point.set(refObj.X1() - 800, refObj.Y1() + 300);

                                    //calculating the angle and distance 
                                    distAndAngl = DistAndAngle(temp_point);

                                    //execution and move to the point
                                    ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);

                                    //re-calculate the position in current view
                                    pos.set(pos.X() - temp_point.X(), pos.Y() - temp_point.Y());
                            }
                            
                            //find the exit which is on the right side
                            //Then correct its orientation
                            currentView = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(currentView);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;
                            
                            if(!isExit(currentView, 2))
                            {
                                 ExecutionAndGo(robot, 0, 100);
                                 currentView = scanAndSaveView(sick, number);
                                 currentView = BoundaryByGroup(currentView);

                                 sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                 plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                                 number++;
                            }
                            
                            temp_point = farthestPoint(currentView, 2);
                            distAndAngl = DistAndAngle(temp_point);
                            
                            temp_dist = sqrt(currentView.back().X2() * currentView.back().X2() + currentView.back().Y2() * currentView.back().Y2());
                            
                            ExecutionAndGo(robot, distAndAngl.second, temp_dist);
                            
                            pos.set(0, pos.X());
                            
                            Cview = scanAndSaveView(sick, number);
                             currentView = BoundaryByGroup(Cview);

                             sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                             plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                             number++;
                            
                             refObj = Cview[0];

                            Objlength = distanceOftwoP(Point (refObj.X1(), refObj.Y1()), Point (refObj.X2(), refObj.Y2()));
                            ObjAngl = acos(abs(refObj.Y1() - refObj.Y2()) /   Objlength); 
                            ObjAngl = 180 / PI * ObjAngl;

                            k = (refObj.Y1() - refObj.Y2()) / (refObj.X1() - refObj.X2());
                            // moving to the position, second step 
                             if(k > 0)
                                     ObjAngl = 0 -  ObjAngl;
                             
                                     
                                     
                            ExecutionAndGo(robot, ObjAngl, 0);
                            
                            ExecutionAndGo(robot, 0, abs(pos.Y()));
                            break;
            case 4:    
                            //calculate the ideal path in current view
                            path = routeInView(currentView, pos);
                            //find intersected boundary
                            refObj = IntersectForReturn(currentView, pos) ;
                            
                            
                            ExecutionAndGo(robot, 0, refObj.Y1());
                            
                            currentView = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(currentView);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;
                            
                            
                            while(!isExit(currentView, 2))
                            {
                                 ExecutionAndGo(robot, 0, 300);
                                 currentView = scanAndSaveView(sick, number);
                                 currentView = BoundaryByGroup(currentView);

                                 sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                 plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                                 number++;
                            }
                            ExecutionAndGo(robot, -70, 0);
                            
                            temp_dist = sqrt((path[1].X1() - path[1].X2()) * (path[1].X1() - path[1].X2()) + (path[1].Y1() - path[1].Y2()) * (path[1].Y1() - path[1].Y2()));
                  

                            ExecutionAndGo(robot, 0, temp_dist);
                            break;
            case 5:     
                            if(pos.Y() < 0)
                                 ExecutionAndGo(robot, 180, 0);
                            
                            pos.set(-pos.X(), -pos.Y());
                            
                            currentView = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(currentView);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;
                            
                            //calculate the ideal path in current view
                            path = routeInView(currentView, pos);
                            
                            refObj = IntersectForReturn(currentView, pos) ;
                            temp_point.set(refObj.X1() - 500, refObj.Y1() + 300);
                            distAndAngl = DistAndAngle(temp_point);
                            ExecutionAndGo(robot, distAndAngl.second, distAndAngl.first);
                            
                            currentView = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(currentView);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;
                            
                            while(!isExit(currentView, 2))
                            {
                                 ExecutionAndGo(robot, 0, 300);
                                 currentView = scanAndSaveView(sick, number);
                                 currentView = BoundaryByGroup(currentView);

                                 sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                                 plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                                 number++;
                            }
                            ExecutionAndGo(robot, -90, 0);
                            
                            ExecutionAndGo(robot, 0, 300);
                            
                            Cview = scanAndSaveView(sick, number);
                            currentView = BoundaryByGroup(Cview);

                            sprintf(rebuildFileName, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                            plotObjects(rebuildFileName, myrobot.getRobot(), currentView); 
                            number++;
                            
                            Objlength = distanceOftwoP(Point (Cview[0].X1(), Cview[0].Y1()), Point (Cview[0].X2(), Cview[0].Y2()));
                            ObjAngl = acos(abs(refObj.Y1() - refObj.Y2()) /   Objlength); 
                            ObjAngl = 180 / PI * ObjAngl;

                            k = (refObj.Y1() - refObj.Y2()) / (refObj.X1() - refObj.X2());
                            // moving to the position, second step 
                             if(k > 0)
                                     ObjAngl = 0 -  ObjAngl;
                            ExecutionAndGo(robot, ObjAngl, 0);
                            ExecutionAndGo(robot, 0, pos.X());
                            break;
            default:break;
        }
}

//closest to Y axis points, two points establish width of rectangle 
//lenght is established by traveling distance
vector<Object> AreaOfSpatial(vector<Object> CV, Point pos)
{
    
        Point widthP1, widthP2; //P1 is on left, P2 is on right
        Point temp;
        Point crossed1,crossed2; // for corridor case 
        Point p1, p2, p3, p4; // for calculating two points
        
        double min1 = -10000;
        double min2 = 10000;
        double longth, width;
        double dist;
        double minL = 10000;
        double minR = 10000;
        Object s1, s2, s3, s4; // for the rectangle side 1,2,3,4
        vector<Object> retn;
        
        
        //general situations face-to-face
        if(pos.X() > -200 && pos.X() < 200)
        {
                for(int i = 0; i < CV.size(); i++)
                {
                        // if on the left
                        if(CV[i].X1() < 0 && CV[i].X1() > min1 && CV[i].Y1() <= pos.Y())
                        {
                            min1 = CV[i].X1();
                            widthP1.set(CV[i].X1(), CV[i].Y1());
                        }
                        if(CV[i].X2() < 0 && CV[i].X2() > min1 && CV[i].Y2() <= pos.Y())
                        {
                            min1 = CV[i].X2();
                            widthP1.set(CV[i].X2(), CV[i].Y2());
                        }

                        //if on the right
                        if(CV[i].X1() > 0 && CV[i].X1() < min2 && CV[i].Y1() <= pos.Y())
                        {
                            min2 = CV[i].X1();
                            widthP2.set(CV[i].X1(), CV[i].Y1());
                        }
                        if(CV[i].X2() > 0 && CV[i].X2() < min2 && CV[i].Y2() <= pos.Y())
                        {
                            min2 = CV[i].X2();
                            widthP2.set(CV[i].X2(), CV[i].Y2());
                        }   
                }

                s1.set(widthP1.X(), 0, widthP2.X(), 0, 1);
                s2.set(widthP1.X(), 0, widthP1.X(), pos.Y(), 2);
                s3.set(widthP1.X(), pos.Y(), widthP2.X(), pos.Y(), 3);
                s4.set(widthP2.X(), pos.Y(), widthP2.X(), 0, 4);
        }
        else // not face-to-face situation, and with no any big surface
        {
                for(int i = 0; i < CV.size(); i++)
                {
                        if(isLeft(Point (0,0), pos, Point (CV[i].X1(), CV[i].Y1())) > 0) //on left
                        {
                                dist = Perpendiculardistance(Point (0,0), pos, Point (CV[i].X1(), CV[i].Y1()));
                                if(dist < minL)
                                {
                                    minL = dist;
                                    widthP1.set(CV[i].X1(), CV[i].Y1());
                                }
                        }
                        else
                            if(isLeft(Point (0,0), pos, Point (CV[i].X1(), CV[i].Y1())) < 0) // on right
                            {
                                    dist = Perpendiculardistance(Point (0,0), pos, Point (CV[i].X1(), CV[i].Y1()));
                                    if(dist < minR)
                                    {
                                        minR = dist;
                                        widthP2.set(CV[i].X1(), CV[i].Y1());
                                    }
                            }


                        if(isLeft(Point (0,0), pos, Point (CV[i].X2(), CV[i].Y2())) > 0) //on left
                        {
                                dist = Perpendiculardistance(Point (0,0), pos, Point (CV[i].X2(), CV[i].Y2()));
                                if(dist < minL)
                                {
                                    minL = dist;
                                    widthP1.set(CV[i].X2(), CV[i].Y2());
                                }
                        }
                        else
                            if(isLeft(Point (0,0), pos, Point (CV[i].X2(), CV[i].Y2())) < 0) // on right
                            {
                                    dist = Perpendiculardistance(Point (0,0), pos, Point (CV[i].X2(), CV[i].Y2()));
                                    if(dist < minR)
                                    {
                                        minR = dist;
                                        widthP2.set(CV[i].X2(), CV[i].Y2());
                                    }
                            }
                }

                // slope of path line
                if(pos.Y() / pos.X() > 0)
                {
                    // the P1 on the top, P2 on the bottom 
                    double d1 = Perpendiculardistance(Point (0,0), pos, widthP1);
                    double d2 = Perpendiculardistance(Point (0,0), pos, widthP2);
                    

                }
                else
                    if(pos.Y() / pos.X() < 0)
                    {
                        // the P1 on the bottom, P2 on the top
                        ;
                    }
                
                
                //s1.set(widthP1.X(), 0, widthP2.X(), 0, 1);
                //s2.set(widthP1.X(), 0, widthP1.X(), pos.Y(), 2);
                //s3.set(widthP1.X(), pos.Y(), widthP2.X(), pos.Y(), 3);
                //s4.set(widthP2.X(), pos.Y(), widthP2.X(), 0, 4);
        }
        
       //one big wall on the front of robot situation
        for(int i = 0; i < CV.size(); i++)
        {
                if(CV[i].X1() < 0 && CV[i].X2() > 0 && CV[i].length() >= 2000)
                {
                        if((CV[i].Y2() - CV[i].Y1()) / (CV[i].X2() - CV[i].X1()) > 0) // tilt to right
                        {
                                p1 = crossPerpend(Point (CV[i].X1(), CV[i].Y1()), Point (CV[i].X2(), CV[i].Y2()), Point (0,0));
                                p2 = crossPerpend(Point (CV[i].X1(), CV[i].Y1()), Point (CV[i].X2(), CV[i].Y2()), pos);

                                for(int j = 0; j < CV.size(); j++)
                                {
                                        if(isLeft(Point (0,0), pos, Point (CV[j].X1(), CV[j].Y1())) < 0) // on right
                                        {
                                                dist = Perpendiculardistance(Point (0,0), pos, Point (CV[j].X1(), CV[j].Y1()));
                                                if(dist < minR)
                                                {
                                                    minR = dist;
                                                    widthP2.set(CV[j].X1(), CV[j].Y1());
                                                }
                                        }

                                        if(isLeft(Point (0,0), pos, Point (CV[j].X2(), CV[j].Y2())) < 0) // on right
                                        {
                                                dist = Perpendiculardistance(Point (0,0), pos, Point (CV[j].X2(), CV[j].Y2()));
                                                if(dist < minR)
                                                {
                                                    minR = dist;
                                                    widthP2.set(CV[j].X2(), CV[j].Y2());
                                                }
                                        }
                                }

                                p3 = crossPerpend(p2, pos, widthP2);
                                p4 = crossPerpend(p1, Point (0,0), widthP2);
                        }
                        else // tilt to left
                        {
                                p4 = crossPerpend(Point (CV[i].X1(), CV[i].Y1()), Point (CV[i].X2(), CV[i].Y2()), Point (0,0));
                                p3 = crossPerpend(Point (CV[i].X1(), CV[i].Y1()), Point (CV[i].X2(), CV[i].Y2()), pos);


                                for(int j = 0; j < CV.size(); j++)
                                {
                                        if(isLeft(Point (0,0), pos, Point (CV[j].X1(), CV[j].Y1())) > 0) // on left
                                        {
                                                dist = Perpendiculardistance(Point (0,0), pos, Point (CV[j].X1(), CV[j].Y1()));
                                                if(dist < minR)
                                                {
                                                    minR = dist;
                                                    widthP1.set(CV[j].X1(), CV[j].Y1());
                                                }
                                        }

                                        if(isLeft(Point (0,0), pos, Point (CV[j].X2(), CV[j].Y2())) > 0) // on left
                                        {
                                                dist = Perpendiculardistance(Point (0,0), pos, Point (CV[j].X2(), CV[j].Y2()));
                                                if(dist < minR)
                                                {
                                                    minR = dist;
                                                    widthP1.set(CV[j].X2(), CV[j].Y2());
                                                }
                                        }
                                }

                                p2 = crossPerpend(p3, pos, widthP2);
                                p1 = crossPerpend(p4, Point (0,0), widthP2);
                        }

                        s1.set(p1.X(), p1.Y(), p4.X(),p4.Y(), 1);
                        s2.set(p1.X(), p1.Y(), p2.X(),p2.Y(), 2);
                        s3.set(p2.X(), p2.Y(), p3.X(),p3.Y(), 3);
                        s4.set(p3.X(), p3.Y(), p4.X(),p4.Y(), 4);
                        
                }
        }
        
        
        // corridor situation
        if(CV[0].length() >= 2000 && CV[CV.size() - 1].length() >= 2000)
        {
            //cout<< "----the first object and last one are big----"<<endl;
            
                double k1 = (CV[0].Y2() - CV[0].Y1()) / (CV[0].X2() - CV[0].X1());
                double k2 = (CV[CV.size() - 1].Y2() - CV[CV.size() - 1].Y1()) / (CV[CV.size() - 1].X2() - CV[CV.size() - 1].X1());

                //cout<<"slope k1 is : "<<k1<<"  slope k2 is : "<<k2<<endl;
                if(abs(k1 - k2) < 30)
                {
                        crossed1 = crossPerpend(Point (CV[0].X1(), CV[0].Y1()), Point (CV[0].X2(), CV[0].Y2()), pos) ;
                        crossed2 = crossPerpend(Point (CV[CV.size() - 1].X1(), CV[CV.size() - 1].Y1()), Point (CV[CV.size() - 1].X2(), CV[CV.size() - 1].Y2()), pos) ;

                        s1.set(CV[0].X1(), CV[0].Y1(), CV[CV.size() - 1].X2(), CV[CV.size() - 1].Y2(), 1);
                        s2.set(CV[0].X1(), CV[0].Y1(), crossed1.X(), crossed1.Y(), 2);
                        s3.set(crossed1.X(), crossed1.Y(), crossed2.X(), crossed2.Y(), 3);
                        s4.set(crossed2.X(), crossed2.Y(), CV[CV.size() - 1].X2(), CV[CV.size() - 1].Y2(), 4);
                }
        }
        
        retn.push_back(s1);
        retn.push_back(s2);
        retn.push_back(s3);
        retn.push_back(s4);
        return retn;
}


vector<Position> ObjectsPosition(vector<Object> CV, Point pos)
{
        vector<Position> temp;
        Position temp_Obj;
        vector<Object> path;
        Point mid;
        Point origin(0,0);
        double temp_dist;
        double dist_Topath;

        path = AreaOfSpatial(CV, pos);// calculate path space

        for(int i = 0; i < CV.size(); i++)
        {
                    mid.set((CV[i].X1() + CV[i].X2()) / 2, (CV[i].Y1() + CV[i].Y2()) / 2);
                    if(pos.X() != 0 || pos.Y() != 0)
                    {
                            if(mid.Y() <= pos.Y())
                            {
                                    temp_dist = GetPointDistance(mid, pos);
                                    //dist_Topath = Perpendiculardistance(origin, pos, mid);
                                    //distance to right side of path space ---- path[3]
                                    dist_Topath = Perpendiculardistance(Point (path[3].X1(), path[3].Y1()), Point (path[3].X2(), path[3].Y2()), mid);
                                    //calculating relative distance
                                    
                                    //if(mid.X() - pos.X() > 0) // right
                                    //    temp_Obj.set(temp_dist, 0, dist_Topath, 0, i);
                                    //else // left
                                   //     temp_Obj.set(temp_dist, 0, dist_Topath, 1, i);
                                    
                                    if(isLeft(origin, pos, mid) < 0) //right
                                        temp_Obj.set(temp_dist, 0, dist_Topath, 0, i);
                                    else // left
                                        if(isLeft(origin, pos, mid) > 0)
                                            temp_Obj.set(temp_dist, 0, dist_Topath, 1, i);
                                    
                                    //cout << "current group line mid point distance to origin : "<<temp_dist<<endl;
                                    temp_Obj.setPoint(mid);
                                    temp.push_back(temp_Obj);
                            }
                    }
                    else
                        if(pos.X() == 0 && pos.Y() == 0)
                        {
                                temp_dist = GetPointDistance(mid, pos);

                                if(mid.X() > 0) //left -- 0
                                    temp_Obj.set(temp_dist, 0, 0, i);
                                else // right -- 1
                                    temp_Obj.set(temp_dist, 0, 1, i);
                                //cout << "current group line mid point distance to origin : "<<temp_dist<<endl;
                                temp_Obj.setPoint(mid);
                                temp.push_back(temp_Obj);
                        }
        }

        return temp;
}

vector<Position> matchMemory(vector<Position> mem, vector<Object> CV, int num)
{
    char viewFileName[100];
    MyRobot myrobot(0,0);
        Point origin(0,0);
        Point pnt;
        vector<Point> temp_points;
        vector<Object> rebuild;
        
        vector<Position> temp;
        Position temp_rnt;
        vector<Position> rnt;
        
        temp = ObjectsPosition(CV, origin);
        
        cout<<"-----This is going to match memory information-----"<<endl<<endl;
        
        for(int i = 0; i < mem.size(); i++)
        {
                for(int j = 0; j < temp.size(); j++)
                {
                        if(abs(temp[j].getDist() - mem[i].getDist()) <= 500 && temp[j].getLeftOrR() != mem[i].getLeftOrR()
                                                             && mem[i].getflag() != 1 && temp[j].getflag() != 1) // position is matching (distance and left or right)
                        {   
                                //matchingFlag = 1;
                                cout<<"---- some surfaces are matched already ----"<<endl;
                                mem[i].setflag(1);
                                temp[j].setflag(1);
                                cout<<"memory object id is : "<< mem[i].getID() << "   current object id is: "<< temp[j].getID() <<endl;
                                
                                temp_rnt.set(temp[j].getDist(), 0.0, mem[i].getDistoPath(), temp[j].getLeftOrR(), temp[j].getID());
                                temp_rnt.setPoint(temp[j].getPoint());
                                rnt.push_back(temp_rnt);
                                
                                rebuild.push_back(CV[temp[j].getID()]);
                        }
                }
        }
        
        sprintf(viewFileName, "%s%d%s", "Maps/Offline/ViewWithObject-", num, ".png");
        plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), CV, rebuild);  
        
        return rnt;
}


long getShortestDistance(Object a, Object b)
{
        Point a1(a.X1(), a.Y1());
        Point a2(a.X2(), a.Y2());
        Point b1(b.X1(), b.Y1());
        Point b2(b.X2(), b.Y2());
        long a1b1 = distanceOftwoP(a1, b1);
        long a1b2 = distanceOftwoP(a1, b2);
        long a2b1 = distanceOftwoP(a2, b1);
        long a2b2 = distanceOftwoP(a2, b2);
        long alllength[4] = {a1b1,a1b2,a2b1,a2b2};

        sort(alllength,alllength+4);
      /*
        for(int i=0;i<4;i++){
            cout <<"length"<<i<<":"<<alllength[i]<<endl;
        }
      */
        return alllength[0];
}

vector<Position> GroupMemory(vector<Object> view, Point pos)
{
        //int p;
        //p = isLeft(Point (0,0), pos, Point p2);
        int cntL = 0;
        int cntR = 0;
        int gnum = 0;
        double temp_dist;
        double temp_angle;
        Point mid;
        Point org(0,0);
        vector<double> dists;
        vector<double> angls;
        Object grouplineobj;
        vector<Object> group, groupline,  objingroups;
        vector< vector<Object> > groups;
        vector<Position> temp;
        Position temp_Obj;


        for(int i = 0; i < view.size(); i++)
        {
            if(view[i].X1() < 0 && view[i].X2() < 0)
                cntL++;
            if(view[i].X1() > 0 && view[i].X2() > 0)
                cntR++;
        }

        if(cntL >= 1&& cntR >= 1)
        {
                for (unsigned int j = 0; j < view.size() - 1; j++)
                {
                        if (view[j].length() < 1000 && view[j + 1].length() < 1000 && getShortestDistance(view[j], view[j + 1]) <=600)
                        {
                                //cout<< "i:"<<i<<", distance:" << getShortestDistance(objs[j], objs[j+1])<<endl;
                                if (group.size() == 0)
                                {
                                        group.push_back(view[j]);
                                        objingroups.push_back(view[j]);
                                }
                                group.push_back(view[j + 1]);
                                objingroups.push_back(view[j + 1]);
                                if (j == view.size() - 2)
                                {
                                        groups.push_back(group);
                                        grouplineobj.set(group[0].X1(), group[0].Y1(), group[group.size() - 1].X2(), group[group.size() - 1].Y2(), gnum);
                                        cout << "groupsize:" << group.size() << endl;
                                        //cout<<group[0].X1()<<","<<group[0].Y1()<<","<<group[group.size()-1].X2()<<","<<group[group.size()-1].Y2()<<endl;
                                        groupline.push_back(grouplineobj);
                                }
                        } 
                        else
                            if (group.size() != 0)
                            {
                                    groups.push_back(group);
                                    grouplineobj.set(group[0].X1(), group[0].Y1(), group[group.size() - 1].X2(), group[group.size() - 1].Y2(), gnum);
                                    cout << "groupsize:" << group.size() << endl;
                                    //cout<<group[0].X1()<<","<<group[0].Y1()<<","<<group[group.size()-1].X2()<<","<<group[group.size()-1].Y2()<<endl;
                                    groupline.push_back(grouplineobj);
                                    gnum++;
                                    group.clear();
                            }
                }
                
                //sprintf(viewFileName, "%s%d%s", "Maps/Offline/Grouped-", 52, ".png");
                //plotObjectsOf3Kinds(viewFileName, view, groupline, objingroups);
        }
        
        for(int i = 0; i < groupline.size(); i++)
        {
                mid.set((groupline[i].X1() + groupline[i].X2()) / 2, (groupline[i].Y1() + groupline[i].Y2()) / 2);
                if(mid.Y() <= pos.Y() && pos.X() != 0 && pos.Y() != 0)
                {
                        temp_dist = GetPointDistance(mid, pos);
                        //calculating relative distance
                        dists.push_back(temp_dist);
                        //calculating relative angle
                        //temp_angle = ;
                        //angls.push_back(temp_angle);
                        if(mid.X() > 0)
                            temp_Obj.set(temp_dist, 0, 0, i);
                        else
                            temp_Obj.set(temp_dist, 0, 1, i);
                        //cout << "current group line mid point distance to origin : "<<temp_dist<<endl;
                        temp.push_back(temp_Obj);
                }
                else
                    if(pos.X() == 0 && pos.Y() == 0)
                    {
                            temp_dist = GetPointDistance(mid, org);
                            //calculating relative distance
                            dists.push_back(temp_dist);
                            //calculating relative angle relating to x axis 
                            //temp_angle = acos(mid.X() / temp_dist);
                            //angls.push_back(temp_angle);
                            
                            if(mid.X() > 0)
                                temp_Obj.set(temp_dist, 0, 0, i);
                            else
                                temp_Obj.set(temp_dist, 0, 1, i);
                            //cout << "current group line mid point distance to origin : "<<temp_dist<<endl;
                            temp.push_back(temp_Obj);
                    }
        }
  
        cout<<" how many groups in this view: "<<groups.size()<<endl;
        
        return temp;
}

vector<Position> MemoryProces(int level, int set, int num, Point pos)
{
        char memoFileName[100];
        int n = num - 2;
        vector<Object> view;
        vector<Position> temp;

        sprintf(memoFileName, "%s%d%s%d%s%d", "level", level, "set", set, "surface-", n);
        view = readASCII(memoFileName);
        temp = GroupMemory(view, pos);
        
        return temp;
}

void pathAndOrientation(ArRobot& robot,ArSick& sick, vector<Object> CV, Point pos, vector<double> path, 
                                                                                                                    vector<Position> memPos)
{
        MyRobot myrobot(0,0);
        Point path1, path2;
        Point origin(0,0);
        Object interObj;
        vector<Object> currentView;
        double path1Dist;
        vector<Position> temp;
        unsigned char matchingFlag = 0;
        char cviewFilename[100];

        //for current view, group and obtain the rough distance
        //temp = GroupMemory(CV, origin);
        temp = ObjectsPosition(CV, origin);

        for(int i = 0; i < memPos.size(); i++)
        {
                for(int j = 0; j < temp.size(); j++)
                {
                        if(abs(temp[j].getDist() - memPos[i].getDist()) < 100 && temp[j].getLeftOrR() != memPos[i].getLeftOrR()
                                                                                        && temp[j].getflag() != 1 && memPos[i].getflag() != 1) // position is matching (distance and left or right)
                            matchingFlag = 1;
                        else
                            if(temp[j].getDist() == memPos[i].getDist() && temp[j].getLeftOrR() == memPos[i].getLeftOrR())
                            {
                                    if(temp[j].getLeftOrR() == 0) // same side right
                                        ExecutionAndGo(robot, 35, 0);
                                    else // same side left
                                        ExecutionAndGo(robot, -35, 0);
                            }
                }
        }

        if(matchingFlag == 1)
        {
                //adjust its orientation
                //ExecutionAndGo(robot, double angle, 0);        

                currentView = scanAndSaveView(sick, number);

                sprintf(cviewFilename, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                plotObjects(cviewFilename, myrobot.getRobot(), currentView); 
                number++;
                //plot path on this way to make sure 
                path1.set(0, path[0]);

                interObj = IntersectForReturn(CV, path1);
                if(interObj.getID() == 0)
                {
                        //calculate path2 point
                        double y = path[0] * cos(path[2]) + path1.Y();
                        double x = sqrt(path[0] * path[0] - y * y);
                        path2.set(x, y);
                }

                //execution 
                ExecutionAndGo(robot, 0, path[0]);

                //current view for second movement
                sprintf(cviewFilename, "%s%d%s", "Maps/Offline/RebuildView-", number, ".png");
                plotObjects(cviewFilename, myrobot.getRobot(), currentView); 
                number++;

                ExecutionAndGo(robot, path[2], path[1]);
        }
}


void CalculatePath(vector<Object> CV, vector<Position> cur, vector<Object> path, int num)
{
        char viewFileName[100];
        MyRobot myrobot(0,0);
        
        Point start, end;
        vector<Point> forpath; // for plotting path
        start.set(-path[0].X2(), path[0].Y2());
        
        double counter = 0;
        double x ,y; // for path end point
        double angle_a, angle_b, dist_a;
        double temp_dist;
        double route = path[3].length();

        cout<<"-----This is going to process pathes-----"<<endl<<endl;
        
        temp_dist = GetPointDistance(cur[0].getPoint(), start);

       angle_a = asin(cur[0].getDistoPath() / temp_dist);
       angle_b = acos((cur[0].getPoint().X() - start.X()) / temp_dist);
       //cout<<"DistoPath is : "<<cur[0].getDistoPath()<<"   Dist is :"<<cur[0].getDist()<<endl;
       //cout<< "left or right??"<<cur[i].getLeftOrR() <<endl;

       if(cur[0].getLeftOrR() == 1)
       {
           x = cos(PI - angle_b - angle_a) * route;
           cout<<"angle a is : "<<angle_a<<"  angle b is : "<<angle_b<<endl;
       }

       y = sqrt(route * route - x * x);
       end.set(x, y);
        
       
        for(int i = 0; i < cur.size(); i++)
        {
                if(cur[i].getLeftOrR() == 1)
                {
                        if(isLeft(start, end, cur[i].getPoint())  < 0)
                        {
                                temp_dist = GetPointDistance(cur[i].getPoint(), start);

                               angle_a = asin(cur[i].getDistoPath() / temp_dist);
                               angle_b = acos((cur[i].getPoint().X() - start.X()) / temp_dist);
                               cout<<"DistoPath is : "<<cur[i].getDistoPath()<<"   Dist is :"<<cur[i].getDist()<<endl;
                               //cout<< "left or right??"<<cur[i].getLeftOrR() <<endl;

                               if(cur[i].getLeftOrR() == 1)
                               {
                                   //x = cos(PI - angle_b - angle_a) * route;
                                   x = cos(angle_b - angle_a) * route;
                                   cout<<"angle a is : "<<angle_a<<"  angle b is : "<<angle_b<<endl;
                               }

                               y = sqrt(route * route - x * x);
                               end.set(x, y);
                               //allPath.push_back(interPoint);
                        }
                        else
                            continue;
                }
        }
       

        if(cur[0].getLeftOrR() == true)
        {
            cout<<"-----test programme-----"<<endl;
                start.set(0, path[0].X2());
                x = cos( angle_b + angle_a) * route;
                y = sqrt(route * route - x * x);
                end.set(x, y);
        }

       forpath.push_back(start);
       forpath.push_back(end);
       sprintf(viewFileName, "%s%d%s", "Maps/Offline/ViewWithPaths-", num, ".png");
       plotObjectsOf3KindswithSpace(viewFileName, myrobot.getRobot(), CV, forpath);
}



vector<Object> Plan_return_path(vector<Object> current_mfis, vector<Object> old_mfis_inside, 
                      vector<Object> end_position, vector<Object> path_in_old_mfis)
{
    Object potential_path;//when returning
    Object intersect_path_seg; 
    
    vector<Object> temp_path_in_old_mfis;
    vector<Object> return_path;
    temp_path_in_old_mfis = path_in_old_mfis;

    potential_path = end_position[6];
    potential_path.reverse();
    
    //compute stright path first 
    potential_path = expend_Object(potential_path,50000,2);
    
    //find intersection with paht_in_old_mfis
    intersect_path_seg = intersectForObject(temp_path_in_old_mfis, potential_path.getP1(), potential_path.getP2());
    Point intersect_point = intersectedPoint(temp_path_in_old_mfis, potential_path.getP1(), potential_path.getP2());
    
    //modify the whole path
    for(int i = 0; i < intersect_path_seg.getID(); i++)
        return_path.push_back(temp_path_in_old_mfis[i]);
    //
    intersect_path_seg.setP2(intersect_point.X(), intersect_point.Y());
    potential_path.set(intersect_point.X(), intersect_point.Y(), end_position[6].X1(), end_position[6].Y1(), 0);
    
    return_path.push_back(intersect_path_seg);
    return_path.push_back(potential_path);
    
    return return_path;
}




