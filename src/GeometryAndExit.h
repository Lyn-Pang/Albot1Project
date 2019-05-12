/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GeometryAndExit.h
 * Author: Wenwang
 *
 * Created on January 27, 2017, 2:47 PM
 */
#include <iostream>
#include <vector>
#include <stdio.h>
#include "Object.H"
#include "Point.H"
#include "PathPlanning.H"

#ifndef GEOMETRYANDEXIT_H
#define GEOMETRYANDEXIT_H

extern vector<int> labels;       //robot step labers
extern vector<int> view_numbers; //key view/ASR numbers 

void RegionAndBoundary(vector< vector<Object> > allLEs, vector<Point> robpositions, LocalMap LMS,
                       vector< vector<Object> > robots);

vector<Object> RouteCombine(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Point> rps);
vector<Object> modifyPath(vector<Point> group_rbs, Object path_segment);
/*combine some of views belonging the chunk
 *find the surfaces limiting the corresponding route
 */
vector< vector<Object> > DetectSpace(vector<Object> chunkMap, vector< vector<Object> > LMs, vector<Object> route_segments);


vector<Object> isBoundary(vector<Object> map, Object route);

/*Compute two regions, one is following the front end point
 *another is following the back end point based on the orientation of route
 *once the surface in the regions, collect them as front or behind surfaces
 */
vector<Object> FrontAndBehindWithinRegion(vector<Object> chunkMap, Object route, unsigned char start_flag);

vector<Exit> IdentifyExitsFollowRoute(vector< vector<Object> > regions, vector<Object> route_segments);
vector<Object> pointsToObjects(vector<Point> rps);

/* Intersection with a circle
 * inputs: ref_view, centre_circle, radius
 * output: the intersected point
 */
Point IntersectedWithCircle(Point cirCenter, double cirR, vector<Object> CV, double start, double end);

/* Through intersection to determine size of ellipse
 * which is to constrain the path segments
 * input: robot position with orientation
 * output: ellipse space
 */
void constrainPointsToEllipse(vector< vector<Object> > robot_positions, vector<Object> ref_map);

//void constructEllipseOfASR(vector< vector<Object> > robot_positions, vector< vector<Object> > views,
//                            vector<Object> ref_map, vector<Object> paths);

vector<Object> constructEllipseOfASR(vector< vector<Object> > robot_positions, vector< vector<Object> > views,
                            vector<Object> ref_map, vector< vector<Object> > key_views, int chunk_cnter);

void PolyBasedASR(vector< vector<Object> > views, vector<Object> ref_map);

vector<Ellipse> RegulizationOfCircleSpace(vector< pair<Point, Point> > base_points, vector<Ellipse> space_relate_robot,
                                            vector< vector<Object> > robot_positions);

/* Compute all path segments follow ellipse space
 * input : a list of circle/ellipse
 * output : a list of path segments 
 */
pair< vector<Object>, vector< vector<Object> > > pathFollowEllipse(vector<Ellipse> robot_space, vector< vector<Object> > views);

/* Compute all path segments follow ellipse space
 * input : a list of circle/ellipse
 * output : a list of path segments 
 */
pair< vector<Object>, vector< vector<Object> > > pathFollowEllipseUsingPreStep(vector<Ellipse> robot_space, vector< vector<Object> > views);

bool intersectEllipse(Ellipse space, Object line);

Object ConstructPath(vector<Ellipse> list_ellipse, int m);

vector<Object> CombineAllViews(vector< vector<Object> > views);

/* Check whether line segment is intersected with ellipse
* if D < 0, there are no intersections
* if D = 0, there is one intersection
* if D > 0, there are two intersections
*/
bool getIntersectionWithEllipse(double x1, double x2, double y1, double y2, 
                                double midX, double midY, double h, double v);
bool isPointInLine(double x1, double x2, double y1, double y2, double px, double py);

/*  Based on base-line/boundary and overlapping 
 *  to split region into several including isolated and overlapping two parts
 *  input : a pair of space
 */
pair< vector<Object>, vector<Object> >  SlpitOverlappingRegion(pair< vector<Object>, vector<Object> > Spaces);

/* cluster surfaces from a region
 * in order to roughly depict the 
 * structure of environment 
 * input  : region -- vector<Object>
 * output : a list of cluster -- vector<vector<Object>>
 */
vector<Object> Cluster_surface(vector<Object> region);

/* polygon simplification algorithm
 * based on clipper library
 * 
 */
vector<Object> Simplify_Clipper(vector<Object> map);


pair< vector<Object>, vector<Object> > LeftAndRightList(vector<Object> region, Object path_segment);

/* Trim left and right list 
 * if the surface is blocked
 * with respect to the path segment
 * remove it.
 */
vector<Object> Trim_sides_group(vector<Object> group, Object path_segmetn);

/* Trim a list of surface with
 * respect to the line segment
 */
vector<Object> Trim_group(vector<Object> group, Object path_segment);

/* shortest line from left to right
 * 
 */
vector<Exit> Short_between_groups(vector<Object> left, vector<Object> right);


/* compute block area based on region and potential exits
 * expend lines to find behind boundary to construct the block space
 */
vector<Object> BuildAreaOfSpace(vector<Object> region, vector<Exit> exits, 
                                  Object path_segment, Exit imagine_exit, int step);

vector<Object> BuildAreaOfSpace(vector<Object> region, vector<Exit> exits, 
                                Object path_segment, Object entrance, int region_num);

Point BoundaryPoint(vector<Object> map, Object ref_obj, int side_flag);

vector<Object> Two_Space_Overlap(vector<Object> block1, vector<Object> block2);

vector<Object> LeftAndRight_boundary(vector<Object> boundary_lines, Object path_segment);

vector<Object> simplifyBoundary(vector<Object> path_segments, vector<Object> boundary_space);

vector<Object> ConstrainSpace_Narrow(vector<Object> region, vector<Object> block,
                           pair< vector<Object>, vector<Object> > LeftAndRight, 
                           Object path_segment, int side_flag);

/* find all intersected surfaces 
 * one left and right sides
 * and return them as pair type
 */
pair< vector<Object>, vector<Object> > boundary_surfaces(pair< vector<Object>, vector<Object> > leftAndright,
                                                         vector<Object> block, Object path_segment);

vector<Exit> Exits_in_region(pair< vector<Object>, vector<Object> > leftAndRight,
                             vector<Object> region_block, Object path_segment);


/* compute region block based on imagine exit/line 
 * find narrow space and depict the block 
 * 
 * output: two boundary lines
 */
vector<Object> Boundary_Lines(vector<Object> region, Exit imagine_entry, Exit imagine_exit);

/* filter left and right lists of surfaces
 * collect those are between two lines
 * both intersected and inside
 */
vector<Object> Select_surfaces(vector<Object> region, vector<Object> boundary_lines);

Object Constrain_region(vector<Object> region, Object path_segment, Exit imagine_eixt);

/* check if two blocks are intersected (vector<Object>)
 * input: two vector<Object>
 * return bool and 
 */
bool blocksIntersection(vector<Object> region_block1, vector<Object> region_block2);

/* identify the position of previous 
 * there are four types of positions
 * with respect to current block.
 * 1 -- inside, 2 -- left, 3 -- right
 * 4 -- bottom
 */
unsigned int modified_blocks_withConnection(vector<Object> region_block, Exit previous_exit);

/* input: two exits
 * output: a block of region
 * simply connect these two exits
 * to compute a block
 */
vector<Object> block_based_twoExits(Exit exit1, Exit exit2);

/* compute a path in a region & based upon circles
 * input : a list of circles
 * output : path segment in corresponding region
 *          (which is not depicted by using one line) two lines
 */
vector<Object> pathInRegion_UseCircles(vector<Ellipse> space_circle, vector<Object> region,
                                       Exit imagin_entry, Exit imagin_exit, Object path_segment);

vector<Object> structureRegion(vector<Object> region, vector<Object> path_space);

vector<Object> RegionBaseLines(vector<Object> filtered_region, vector<Object> path_space,
                               Object path_segment);


vector<Object> structure_boundary(vector<Object> region, vector<Object> path_space, 
                                  Exit imagine_exit, Object path);

vector<Object> connectSurfaces(vector<Object> surfaces);

vector<Object> collectSurfaces(vector<Object> list, vector<Object> ref_Obj, int flag);

/* From start surface to imaginary exit, connect surface clockwise
 * input: region MFIS and path space
 * output: polygon
 */
vector<Object> Struct_Connect(vector<Object> region, vector<Object> path_space);

vector<Object> Struct_Connect(vector<Object> region, vector<Object> path_space, Object path , Exit previous_exit, Exit imagin_exit, int region_num);
//vector<Object> _Struct_Connect(vector<Object>& pre_region, vector<Object>& region, vector<Object> path_space, Object path,
//                                Exit previous_exit, Exit imagin_exit, int region_num);
vector<Object> _Struct_Connect(vector<Object>& pre_region, vector<Object>& region, vector<Object> path_space, Object path,
                                Exit previous_exit, Exit imagin_exit, int region_num);

/* remove acute angle and insert a modified surface
 * between two points, and return a whole list of surfaces
 */
vector<Object> removeAcuteAngle(vector<Object> surfaces, Object path);

/* compute two rectangle region
 * detect surface inside these two region
 * join the surfaces inside each of regions
 */
vector<Object> exitAndentry_buildregion(vector<Object> region, vector<Object> path_space,
                                        Object entrance, Exit exit, int num);


vector<Object> _exitAndentry_buildregion(vector<Object> region, vector<Object> path_space,
                                        Object entrance, Exit exit, int num);

/* extend two adjacent region block to find connection 
 * and intersection. 
 * inpout: current MFIS, current block and next block
 * output: two extended blocks
 */
vector<Object> extendRegionBlock(vector<Object> current_mfis, vector<Object> space_block, int flag);
vector<Object> _extendRegionBlock(vector<Object> current_mfis, vector<Object> space_block, 
                                 double extend_length, int flag);
vector<Object> extendRegionBlockToBoundary(vector<Object> current_mfis, vector<Object> space_block, int flag);


vector<Object> StructureDescription(vector<Object> left_env, vector<Object> right_env, 
                                    vector<Object> region_block, Object path_segment);
vector<Object> _StructureDescription(vector<Object> mfis, vector<Object> region_block, Object path_segment);

vector< vector<Object> >  StructureDescription(vector< vector<Object> > mfis, vector< vector<Object> > combined_blocks, 
                                               vector<Object> Path);

pair< vector<Object>, vector<Object> > Combine_origin_extend_regions(vector<Object> structure, vector<Object> structure2,
                                             vector<Object> mfis_before);

vector< vector<Object> > Combine_extension_blocks(vector< pair< vector<Object>, vector<Object> > > extension_blocks);


vector<Object> environment_shape(vector<Object> region_mfis, vector<Object> path_space, Object entrance, int cnt);
vector< vector<Object> > environment_shape(vector< vector<Object> > MFISs, vector<Object> path, vector<Exit> exits);
vector<Object> clear_redundant(vector<Object> env, vector<Exit> entr_exit);

vector<Object> be_project_region(vector<Object> current_chunk, vector<Object> last_chunk,
                                 vector<Exit> current_chunk_exits, vector<Exit> last_chunk_exits);

#endif /* GEOMETRYANDEXIT_H */

