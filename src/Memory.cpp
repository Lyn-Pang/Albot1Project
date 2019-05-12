/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Memory.cpp
 * Author: arthur
 * Modified: Wenwang
 * Created on January 27, 2016, 10:58 AM
 */

#include "Memory.h"

Memory::Memory()
{
}

Memory::Memory(const Memory& orig)
{
}

Memory::~Memory()
{
}

    void Memory::setAllGlobalRobotPs(vector<Point> points)
{
    AllGlobalRobotPs = points;
}
    vector<Point> Memory::getAllGlobalRobotPs(){
        return AllGlobalRobotPs;
    }
    
     void Memory::setAllGlobalRobotXAngle(vector<double> doubles){
        AllGlobalRobotXAngle = doubles;
    }
    vector<double> Memory::getAllGlobalRobotXAngle(){
        return AllGlobalRobotXAngle;
    }
    
void Memory::setPathPointGlobalRobotPs(vector<Point> points)
{
    PathPointGlobalRobotPs = points;
}
    vector<Point> Memory::getPathPointGlobalRobotPs(){
        return PathPointGlobalRobotPs;
    }
         void Memory::setPathPointGlobalRobotXAngle(vector<double> doubles){
        PathPointGlobalRobotXAngle = doubles;
    }
    vector<double> Memory::getPathPointGlobalRobotXAngle(){
        return PathPointGlobalRobotXAngle;
    }
    
    void Memory::setLocalRobotPs(vector<Point> points){
        LocalRobotPs = points;
    }
    vector<Point> Memory::getLocalRobotPs(){
        return LocalRobotPs;
    }
    void Memory::setTurningRobotPs(vector<Point> points){
        TurningRobotPs = points;
    }
    vector<Point> Memory::getTurningRobotPs(){
        return TurningRobotPs;
    }
    void Memory::setUpdatingNumber(vector<int> ints){
        UpdatingNumber = ints;
    }
    vector<int> Memory::getUpdatingNumber(){
        return UpdatingNumber;
    }

    void Memory::setReturningRobotPs(vector<Point> points)
    {
        ReturningRobotPs = points;
    }
    
    vector<Point> Memory::getReturningRobotPs()
    {
        return ReturningRobotPs;
    }
