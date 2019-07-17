/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Exit.h
 * Author: arthur
 *
 * Created on May 4, 2017, 3:24 AM
 */

#ifndef EXIT_H
#define EXIT_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "Object.H"
#include "Point.H"
#include "PathPlanning.H"

/* identify perpendicular line based on each path
 * segment, which is to depict an exit between 
 * two regions/path segments
 */
vector<Exit> FindExits(vector<Object> ref_map, vector<Object> path);

void NarrowSpace(vector<Object> ref_map, vector<Object> path);

vector<Point> FivePointsOnline(Object line_segment);
vector<Point> NinePointsOnline(Object line_segment);

vector<Object> ShortestBaseLine(vector<Point> base_points, Object path_segment, vector<Object> ref_map);

/* Yeap and Margaret theory about ASR & Exits
 * from local environment.
 * input: ASR 
 * output: all potential exits
 * modified version of findExits()
 * finds exits e(p1,p2)
 * where, p1 is the first point of exit(probable exit point 2 of objects)
 * p2 is the end point of exit(nearest point on the objects which contains the nearest probable exit point 1)
 *this module is used in PartialUpdating.cpp which updates the map only before crossing the exits
 */
vector<Exit> PotentialExits(vector<Object> cv);

vector<Exit> PotentialExitFromAlistOfView(vector< vector<Object> > views);
vector<Exit> addTwoExits(vector<Exit> ref, vector<Exit> exits);

void structualExit(pair< vector<Object>, vector<Object> > boundary_view,
                   pair< vector<Object>, vector<Object> > robots);


/* Based on split space & path segment
 * find all potential exits
 * from isolated space
 */
Exit Potential_Exit(pair< vector<Object>, vector<Object> > spaces, 
                             Object path_segment,
                             Object ref_boundary);


/* Compute all exit following path
 * find the smallest one, or pick some 
 * of them from those exits
 */
vector<Exit> PerpendicularExits(vector<Object> region, Object path_segment);


/* filter the list of exits
 * pick the near and shortest one
 * as outputs
 */
vector<Exit> ConstrainListOfExits(vector<Exit> exits, Point exit_point);

/* based upon left and right list of points
 * compute all potential lines
 * from left to right, find the shortest
 * line.
 */
vector<Exit> Potentialine_basePoints(vector<Point> left, vector<Point> right, vector<Object> left_region);

/* based upon left and right list of points
 * compute all potential lines
 * from left to right, find the shortest
 * line.
 * 
 * input: map/view and a path segment
 */
vector<Exit> Potentialine_basePoints(vector<Object> map, Object path);

/* inputs are imagine exit and left & right lists of surfaces
 * find shortest exit and which neats to the imagine exit
 */
Exit exitBasedImagineLine(pair< vector<Object>, vector<Object> > boundary_view, Exit imagine_exit);

Exit NearexitBasedImagineLine(pair< vector<Object>, vector<Object> > boundary_view, Exit imagine_exit);

/* 
 * 
 */
vector<Object> identify_back(vector<Object> region, vector<Object> boundary_lines,
                             Point robot_position, Exit imagine_exit);

Exit lastStep_narrowExit(vector<Object> lastStep_view, vector<Object> MFIS, 
                         vector<Object> lastStep_robot, vector<Object> nextStep_robot,
                         Object path_segment);


vector<Exit> ExitInbetweenRobots(vector<Object> mfis_region, vector< vector<Object> > robots);

Exit exitFromRegion(vector<Object> left_list, vector<Object> right_list, 
                    Object path_segment, Exit imagine_exit);


/* compute the most constrain gap as exit
 * which is crossed by path 
 */
Exit most_constrain_exit_on_path(vector<Object> view, Object path_segment);
Exit most_constrain_exit_on_path(vector< vector<Object> > views, vector< vector<Object> > robots, int cnt);


Exit ConnectionOfRegions(vector<Object> block1, vector<Object> block2,
                         vector<Object> extend_block1, vector<Object> extend_block2);

vector<Exit> ConnectionOfRegions(vector< vector<Object> > combined_origin_extension, vector< vector<Object> > MFISs);
vector<Exit> _ConnectionOfRegions(vector< vector<Object> > combined_origin_extension, vector< vector<Object> > MFISs,
                                  vector< vector<Object> > path_space_region);

pair< vector<Point>, vector<Point> > select_potential_points(vector<Object> view, vector<Exit> exit_area);
Exit shortestExit(vector<Object> list1, vector<Object>  list2);
Exit shortestExit(pair< vector<Point>, vector<Point> > potential_lists);


vector<Exit> potential_exit_on_boundary(vector<Object> virtual_boundary, vector<Object> real_boundary);

vector<Object> trim_boundary_base_exit(vector<Object> boundary, Exit exit_on_boundary);
vector<Object> trim_boundary_base_exit_region(vector<Object> boundary, Exit exit_on_boundary);


Exit confirm_exit_position(vector<Object> view, Object exit);

Exit most_constrain_exit_cross_path(vector<Object> view, Object path_segment);


vector<Exit> crossed_exits_along_path(vector<Exit> exits, vector<Object> path_in_mfis);

#endif /* EXIT_H */ 

