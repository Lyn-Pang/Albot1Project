/* 
 * File:   Memory.h
 * Author: arthur
 *
 * Created on January 27, 2016, 10:58 AM
 *
 * 
 */
#include <vector>
#include "Object.H"
#include "Point.H"
#ifndef MEMORY_H
#define MEMORY_H

class Memory {
public:
    Memory();
    Memory(const Memory& orig);
    ~Memory();
    void setPathPointGlobalRobotPs(vector<Point>);
    vector<Point> getPathPointGlobalRobotPs();
    
    void setPathPointGlobalRobotXAngle(vector<double>);
    vector<double> getPathPointGlobalRobotXAngle();
    
    void setAllGlobalRobotPs(vector<Point>);
    vector<Point> getAllGlobalRobotPs();
    
     void setAllGlobalRobotXAngle(vector<double> );
     vector<double> getAllGlobalRobotXAngle();
    
    void setLocalRobotPs(vector<Point>);
    vector<Point> getLocalRobotPs();
    
    void setTurningRobotPs(vector<Point>);
    vector<Point> getTurningRobotPs();
    
    void setUpdatingNumber(vector<int>);
    vector<int> getUpdatingNumber();
    
    void setReturningRobotPs(vector<Point>);
    vector<Point> getReturningRobotPs();
    
private:
    vector<Point> LocalRobotPs;
    vector<Point> TurningRobotPs;
    vector<int> UpdatingNumber;

    vector<Point> PathPointGlobalRobotPs;
    vector<double>PathPointGlobalRobotXAngle;

    vector<Point> AllGlobalRobotPs;
    vector<double> AllGlobalRobotXAngle;
    
    vector<Point> ReturningRobotPs;

};

#endif /* MEMORY_H */

