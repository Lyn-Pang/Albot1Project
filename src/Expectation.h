
/* 
 * File:   Expectation.h
 * Author: Lyn
 *
 * Created on November 10, 2017, 6:14 AM
 */

#ifndef EXPECTATION_H
#define EXPECTATION_H


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "Point.H"
#include "Plotting.H"
#include "Object.H"
#include "GeometricOp.H"
#include "mfisOp.H"

extern int expect_process_flag;
extern vector<Object> remaining;

/* if it is turning in place, it will expect what just appeared in a particular direction
 * project earlier information/landmarks onto current view
 */
vector<Object> expectation_process(vector<Object> current_ref, vector<Object> previous_ref, 
                                       vector<Object> landmarkInPV,vector<Object> landmarkInPPV);

vector<Object> Project_backward_MFIS(vector<Object> currentMFIS, vector<Object> oldMFIS, vector<Object> references);

//match current view with projected MFIS
//return position on MFIS without updating MFIS
//vector<Object> match_view_with_MFIS(vector<Object> cv, vector<Object> MFIS, vector<Object> rb, int v);
vector<Object> match_view_with_MFIS(vector<Object> cv, vector<Object> fixMFIS, vector<Object> rb,  
                                    vector<Object>& currentMFIS, int v);

pair< vector<Object>,  vector<Object> > info_adjustment(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> expect_information, int v);

vector< vector<Object> > info_adjustment_next(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> rest_info, int v);

pair< vector<Object>,  vector<Object> > info_first_expect_modify(vector<Object> currentview, vector<Object> robot_position,  
                                     vector<Object> environment_map, int v);

vector< vector<Object> > info_adjustment_next_version(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> rest_info, int v, int& flag);


//vector<Object> Fill_operation(vector<Object> view, vector<Object> cv, vector<Object> global_map, vector<Object> refences);

vector<Object> Fill_operation_with_ASR(vector<Object> view, vector<Object> cv, vector<Object> global_map, 
                                       vector< vector<Object> > match_list, vector<Object> references);

//vector<Object> Fill_operation_with_Circle(vector<Object> view, vector<Object> cv, vector<Object> global_map, 
//                                          vector<Object> position, vector<Object> references);

vector<Object> Correct_detail(vector<Object> view, vector<Object>& global_map);
vector<Object> Correct_detail_blocked(vector<Object> view, vector<Object>& global_map, vector<Object> shift_position);

vector<Object> adjust_base_spatial(vector<Object> cv, vector<Object> rb, vector< vector<Object> > match_list);

vector< vector<Object> > Current_and_expectation(vector<Object> currentView, vector<Object> rb, vector<Object> MFIS, int v);

vector< vector<Object> > structure_align(vector<Object> currentView, vector<Object> rb, vector<Object> MFIS, int v, int& change_flag);


//bool parallel_objects(vector<Object> view);

vector<Object> view_in_theory(vector<Object> MFIS, vector<Object> view, vector<Object> robot_position, int v);
vector< vector<Object> > shift_after_expect(vector<Object> expect_info, vector<Object> view, vector<Object> rb, int v);

vector< vector<Object> > method_two_expectation(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> rest_info, int v, int& flag);


/**********************************************************************************/
vector<Object> Fill_operation_with_ASR_modifiyVersion(vector<Object> view, vector<Object> cv, vector<Object> global_map, 
                                       vector< vector<Object> > match_list, vector<Object> references);

void Correct_detail_modifiyVersion(vector<Object>& view, vector< vector<Object> > match_list);

pair< vector<Object>,  vector<Object> > info_first_expect(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> expect_information, int v);

bool Expecting_with_global_map(vector<Object> robot_position, vector<Object> currentview, vector<Object> MFIS, int v);
vector<Object> Expect_reference(vector<Object> robot_position, vector<Object> currentview, vector<Object> MFIS);
vector< vector<Object> > Expect_function(vector<Object> reference_obj, vector<Object> robot_position, vector<Object> currentview, vector<Object> environment, int v);
bool switch_back_premethod(vector<Object> MFIS, vector<Object> view);
int critical_point_switch(vector<Object> currentRb, vector< vector<Object> > all_robots);

void Make_use_structur_to_align(vector<Object> view, vector<Object> environment);

vector<Object> compute_map_without_deleiton(vector< vector<Object> > views_on_global, vector< vector<Object> > robots_on_global);
vector<Object> compute_map_without_deleiton(vector< vector<Object> > views_on_global, vector< vector<Object> > robots_on_global,
                                            vector<int> numbers, int start, int end);


vector<Object> clear_and_update_env(vector<Object> env, vector<Object> currentview, vector<Object> robot_position);
vector<Object> clear_and_update_env_with_boundary(vector<Object> env, vector<Object> currentview, 
                                                  vector<Object> robot_position, vector<Object> boundary);

vector<Object> clear_map(vector<Object> map);
#endif /* EXPECTATION_H */

