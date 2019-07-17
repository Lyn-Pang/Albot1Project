/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   StructBoundary.h
 * Author: arthur
 *
 * Created on July 13, 2018, 3:05 AM
 */

#ifndef STRUCTBOUNDARY_H
#define STRUCTBOUNDARY_H

#include <cmath>
#include <iostream>
#include <vector>
#include "PointAndSurface.H"
#include "Object.H"

class Region_struct {
private:
    vector<Exit> exits;
    vector<vector<Object> > retions;


public:
    
    void set_exits(vector<Exit> potential_exits);
    vector<Exit> get_exits();
    
    void set_regions_boundry(vector<vector<Object> >  regions_boundary);
    vector<vector<Object> > get_regions_boundary();
    
};

vector<Object> polygon_of_boundary(vector<Object> poly1, vector<Object> poly2, vector<Object> pos, int v);
void integrate_virtual_boundary(vector<Object>& boundary);
vector<Object> boundary_without_far_info(vector<Object> virtual_boundary, vector<Object> real_boundary);
vector<Object> ultimate_trim_boundary(vector<Object> complete_boundary, vector<Object> trim_lines);


//test programme section
void test_combine(vector< vector<Object> > robots, vector< vector<Object> > views);
void test_position(vector< vector<Object> > robots, vector< vector<Object> > views);

pair< vector<Object>, vector<Object> > check_postion_and_env(vector<Object> robot, vector<Object> view, vector<Object> MFIS, int v);

vector<Object> bound_map(vector< vector<Object> > robots, vector< vector<Object> > views);
vector<Object> bound_map_modified_version(vector< vector<Object> > robots, vector< vector<Object> > views);
vector<Object> bound_map_modified_version2(vector< vector<Object> > robots, vector< vector<Object> > views);

//vector< vector<Object> > test_boundary(vector<Object> map);

vector<Object> bounded_space(vector< vector<Object> > robots, vector< vector<Object> > views);
Region_struct bounded_space_version2(vector< vector<Object> > robots, vector< vector<Object> > views);

vector<Object> update_region(vector<Object> region_map, vector<Object> view, vector<Object> robot);


vector<Object> connection(vector<Object> view_before_cross, vector<Object> view_after_cross, Exit potential_exit);


//vector<Object> region_base_exits(vector<Object> view, vector<Exit> exits);

vector<Object> update_corridor_region(vector<Object> map, vector<Object> to_update_view);
vector<Object> parallel_lines(vector<Object> view);
vector< vector<double> > corridor_width(vector<Object> parralle_info);

vector<Object> trim_region_map(vector<Object> map, vector<Exit> exits);
vector<Object> join_boundary(vector<Object> regions);
vector<Object> join_boundary_verison2(vector<Object> region, Exit exit);

bool include_point(vector<Point> list, Point p);
bool include_surface(vector<Object> list, Object surface);
bool is_first_endpoint(vector<Object> view, Point p);

Point return_second_endpoint(vector<Object> view, Point p);

vector<Object> bounded_surfaces(vector<Object> map, vector<Object> bound_box);

//void test_function(vector<Object> v, Point p1, Point p2);
void integrate_regions(vector< vector<Object> > regions_outline, vector< vector<Object> > regions_map);
//void region_outline_modify(vector<Object> upcoming_reigon, vector< vector<Object> > integrated_region);

vector<Object> region_geometry(vector< vector<Object> > regions, vector<Exit> exits);
void adjustment_region_map(vector<Object> &region1, vector<Object> &region2);


void find_orientation_at_place(vector<Object> current_MFIS, vector<Object> memory_MFIS, Exit current_exit, Exit memory_exit);
pair< vector<Object>, vector<Object> > find_orientation_at_place(vector<Object> current_MFIS, vector<Object> memory_MFIS, 
                                                                 vector<Object> current_robot, Exit current_exit, Exit memory_exit);


vector<Object> exit_and_region_along_path(vector< vector<Object> > views, vector< vector<Object> > robots, vector<Object> mfis);
pair< vector<Object>, vector<Object> >  update_geo_map(vector< vector<Object> > views, vector< vector<Object> > robots);
vector< vector<Object> > cluster_objects_follow_path(vector<Object> map, vector<Object> block);

vector<Object> geometry_sides(vector< vector<Object> > sides);

Object track_geometry(vector<Object> view, vector<Object> robot, Object ref_line);

vector<Object> shift_to_close_object(vector<Object> view, vector<Object> side);
Exit find_exit_cross(vector<Object> map, Object side_of_geometry, vector<Object> last_step, vector<Object> current_step);
vector<Object> select_info_in_geometry(vector<Object> geometry, vector<Object> view);
Object base_line_for_geometry(vector<Object> view, vector<Object> robot, Exit exit_on_side);


vector<Object> collect_object_base_side(vector<Object> view, Object one_side);
Exit most_constrain_gap(vector<Object> group1, vector<Object> group2);


Object shift_line_along_perpendicluar(Object line, double shift_distance, int side);

#endif /* STRUCTBOUNDARY_H */

