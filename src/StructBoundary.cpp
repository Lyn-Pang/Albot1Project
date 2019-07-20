/*
 *  boundary and structure functions for geometric calculations.
 *
 *  Lyn Pang
 */

#include "GeometryFuncs.H"
#include "Plotting.H"
#include "Point.H"
#include "Object.H"
#include "PathPlanning.H"
#include "PerceptualMapping.H"
#include "GeometricOp.H"
#include "mfisOp.H"
#include "concave_hull.h"
#include "CompareASR.H"
#include "Returning.h"
#include "ToolFunctions.h"
#include "StructBoundary.h"
#include "Laser2Surface.H"
#include "Exit.h"
#include "ConvexHull.h"
#include "Expectation.h"
#include "GeometryAndExit.h"
#include "FindGap.h"

#include <cstdlib>


void Region_struct::set_exits(vector<Exit> potential_exits)
{
    exits = potential_exits;
}

vector<Exit> Region_struct::get_exits()
{
    return exits;
}
    
void Region_struct::set_regions_boundry(vector<vector<Object> >  regions_boundary)
{
    retions = regions_boundary;
}

vector<vector<Object> > Region_struct::get_regions_boundary()
{
    return retions;
}


vector<Object> polygon_of_boundary(vector<Object> poly1, vector<Object> poly2, vector<Object> pos, int v)
{
        cout << " ***** Computing polygon boundary function ***** " << endl;
        unsigned int left_trim_id, right_trim_id;
        
        Point joint_point1(0,0), joint_point2(0,0);
        
        Object trim_ref, trim_ref2;
        Object trim_obj1, trim_obj2, trim_obj3, trim_obj4, trim_obj5;
        
        vector<Object> rnt_boundary;
        vector<Object> trim_poly1_part1, trim_poly1_part2;
        vector<Object> trim_poly2;
        
        //trim reference line
        trim_ref = poly2.back();
        trim_ref2 = expend_Object_two_sides(trim_ref, 800, 800);
        
        //trim to get joint point
        for(int i = 0; i < poly1.size(); i++)
        {
            if(twoLinintersect(poly1[i].getP1(), poly1[i].getP2(), trim_ref2.getP1(), trim_ref2.getP2()) == true)
            {
                if((joint_point1.X() == 0) && (joint_point1.Y() == 0))
                {
                    //trimmed objects
                    joint_point1 = intersectPointTwolineEquations(poly1[i], trim_ref2);
                    left_trim_id = i;
                    trim_obj1.set(poly1[i].X1(), poly1[i].Y1(), joint_point1.X(), joint_point1.Y(), i);
                    
                    //special case for trimming
                    trim_obj5.set(poly1[i].X2(), poly1[i].Y2(), joint_point1.X(), joint_point1.Y(), i);
                    
                    if(poly1[i].get_imagined_flag() == 1)
                        trim_obj1.set_imagined_flag(1);
                }
                else
                {
                    //trimmed objects
                    joint_point2 = intersectPointTwolineEquations(poly1[i], trim_ref2);
                    right_trim_id = i;
                    trim_obj2.set(joint_point2.X(), joint_point2.Y(), poly1[i].X2(), poly1[i].Y2(), i);
                    if(poly1[i].get_imagined_flag() == 1)
                        trim_obj2.set_imagined_flag(1);
                }
            }
        }
        
//        cout << " current step    : "<< v << endl;
//        cout << " joined point1 x : " << joint_point1.X() << " , y : " << joint_point1.Y() << endl;
//        cout << " joined point2 x : " << joint_point2.X() << " , y : " << joint_point2.Y() << endl;
    
        if((isLeft(pos[6].getP1(), pos[6].getP2(), joint_point1) > 0) && (isLeft(pos[6].getP1(), pos[6].getP2(), joint_point2) < 0))
        {

            //trimmed poly1 part1 
            for(int i = 0; i <= left_trim_id; i++)
            {
                if(i != left_trim_id)
                    trim_poly1_part1.push_back(poly1[i]);
                else
                    trim_poly1_part1.push_back(trim_obj1);
            }

            //trimmed parts of boundary 
            trim_poly2 = poly2;
            trim_poly2.pop_back();

            trim_obj3.set(joint_point1.X(), joint_point1.Y(), trim_ref.X2(), trim_ref.Y2(), 1);
            trim_obj4.set(trim_ref.X1(), trim_ref.Y1(), joint_point2.X(), joint_point2.Y(),  0);
            trim_obj3.set_imagined_flag(1);
            trim_obj4.set_imagined_flag(1);
            trim_poly2.insert(trim_poly2.begin(), trim_obj3);
            trim_poly2.push_back(trim_obj4);

            for(int i = 0; i < trim_poly2.size(); i++)
                trim_poly1_part1.push_back(trim_poly2[i]);

            for(int i = right_trim_id; i < poly1.size(); i++)
            {
                if(i == right_trim_id)
                    trim_poly1_part1.push_back(trim_obj2);
                else
                    trim_poly1_part1.push_back(poly1[i]);               
            }
        }
        else
        {
            if((isLeft(pos[6].getP1(), pos[6].getP2(), joint_point1) < 0) && (isLeft(pos[6].getP1(), pos[6].getP2(), joint_point2) > 0))
            {
//                //trimmed poly1 part1 
//                for(int i = left_trim_id ; i <= right_trim_id; i++)
//                {
//                        trim_poly1_part1.push_back(poly1[i]);
//                }
//
//                //trimmed parts of boundary 
//                trim_poly2 = poly2;
//                trim_poly2.pop_back();
//
//                trim_obj3.set(joint_point1.X(), joint_point1.Y(), trim_ref.X2(), trim_ref.Y2(), 1);
//                trim_obj4.set(trim_ref.X1(), trim_ref.Y1(), joint_point2.X(), joint_point2.Y(),  poly2.size());
//                trim_obj3.set_imagined_flag(1);
//                trim_obj4.set_imagined_flag(1);
//                trim_poly2.insert(trim_poly2.begin(), trim_obj3);
//                trim_poly2.push_back(trim_obj4);
//
//                for(int i = 0; i < trim_poly2.size(); i++)
//                    trim_poly1_part1.push_back(trim_poly2[i]);
                
                
                
//                    trim_obj5.reverse();
//                    trim_poly1_part1.push_back(trim_obj5);
//                    for(int i = left_trim_id ; i <= right_trim_id-1; i++)
//  
//                    trim_poly1_part1.push_back(trim_obj2);
//                    //trimmed parts of boundary 
//                    trim_poly2 = poly2;
//                    trim_poly2.pop_back();
//
//                    trim_obj3.set(joint_point1.X(), joint_point1.Y(), trim_ref.X2(), trim_ref.Y2(), 1);
//                    trim_obj4.set(trim_ref.X1(), trim_ref.Y1(), joint_point2.X(), joint_point2.Y(),  poly2.size());
//                    trim_obj3.set_imagined_flag(1);
//                    trim_obj4.set_imagined_flag(1);
//                    trim_poly2.insert(trim_poly2.begin(), trim_obj3);
//                    trim_poly2.push_back(trim_obj4);
//
//                    for(int i = 0; i < trim_poly2.size(); i++)
//                        trim_poly1_part1.push_back(trim_poly2[i]);

            }
            else
            {
                if(((isLeft(pos[6].getP1(), pos[6].getP2(), joint_point1) < 0) && (isLeft(pos[6].getP1(), pos[6].getP2(), joint_point2) < 0))
                  ||  ((isLeft(pos[6].getP1(), pos[6].getP2(), joint_point1) > 0) && (isLeft(pos[6].getP1(), pos[6].getP2(), joint_point2) > 0)))
                {
                        //trimmed poly1 part1 
                        for(int i = left_trim_id ; i <= right_trim_id; i++)
                        {
                                trim_poly1_part1.push_back(poly1[i]);
                        }

                        //trimmed parts of boundary 
                        trim_poly2 = poly2;
                        trim_poly2.pop_back();

                        trim_obj3.set(joint_point1.X(), joint_point1.Y(), trim_ref.X2(), trim_ref.Y2(), 1);
                        trim_obj4.set(trim_ref.X1(), trim_ref.Y1(), joint_point2.X(), joint_point2.Y(),  poly2.size());
                        trim_obj3.set_imagined_flag(1);
                        trim_obj4.set_imagined_flag(1);
                        trim_poly2.insert(trim_poly2.begin(), trim_obj3);
                        trim_poly2.push_back(trim_obj4);

                        for(int i = 0; i < trim_poly2.size(); i++)
                            trim_poly1_part1.push_back(trim_poly2[i]);
                }
            }
        }
        
//        cout << "joint point 1 x: " << joint_point1.X() << " ; y: " << joint_point1.Y() << endl;
//        cout << "joint point 2 x: " << joint_point2.X() << " ; y: " << joint_point2.Y() << endl;
//        cout << " left  id      : " << left_trim_id << endl;
//        cout << " right id      : " << right_trim_id << endl;
        char plotFile[90];
        sprintf(plotFile, "%s%d%s", "Maps/Offline/Trim_part1_and_part2-",v,".png");                   
        plotObjectsOf4Kinds(plotFile, pos, trim_poly1_part1, poly2, poly1);


        return trim_poly1_part1;
}

void integrate_virtual_boundary(vector<Object>& boundary)
{
    for(int i = 0; i < boundary.size()-1; i++)
    {
            if((boundary[i].get_imagined_flag() == 1) && (boundary[i+1].get_imagined_flag() == 1))
            {
                //cout << " boundary line num: " <<  boundary.size() << endl;
                //cout << " current        i : " << i << endl;
                //boundary[i].setP2(boundary[i+1].X2(), boundary[i+1].Y2());
                boundary[i].set(boundary[i].X1(), boundary[i].Y1(), boundary[i+1].X2(), boundary[i+1].Y2(), i);
                boundary.erase(boundary.begin()+i+1);
                i--;
            }
    }
}


vector<Object> boundary_without_far_info(vector<Object> virtual_boundary, vector<Object> real_boundary)
{
    double size_threshold = 1500;
    
    //pair<Object, Object> pair_large_virtual_boundary;
    //vector< pair<Object, Object> > list_of_pair_boundary;
    
    vector<Object> pair_large_virtual_boundary;
    vector< vector<Object> > list_of_pair_boundary;
    
    
    Object temp_modified_obj;
    vector<Object> large_virtual_boundary;
    vector<Object> modified_boundary;
    vector<Object> treat_as_solid_boundary;
    
    virtual_boundary.pop_back();
    
    for(int i =0; i < virtual_boundary.size(); i++)
    {
        if(virtual_boundary[i].length() > size_threshold)
            large_virtual_boundary.push_back(virtual_boundary[i]);
        else
            treat_as_solid_boundary.push_back(virtual_boundary[i]);
    }
    
    
    //paring 
    for(int i = 0; i < large_virtual_boundary.size()-1; i++)
    {
            double dist = 0, min = 0;
            double next_ref_num = 0;
            Object ref_obj;
            //pair_large_virtual_boundary.first = large_virtual_boundary[i];
            pair_large_virtual_boundary.push_back(large_virtual_boundary[i]);
            for(int j = i+1; j < large_virtual_boundary.size(); j++)
            {
                    if(j == i+1)
                    {
                        min = distanceOftwoP(large_virtual_boundary[i].getP1(), large_virtual_boundary[j].getP2());
                        ref_obj = large_virtual_boundary[j];
                        next_ref_num = j;
                    }
                    else
                    {
                        if(min > distanceOftwoP(large_virtual_boundary[i].getP1(), large_virtual_boundary[j].getP2()))
                        {
                            min = distanceOftwoP(large_virtual_boundary[i].getP1(), large_virtual_boundary[j].getP2());
                            ref_obj = large_virtual_boundary[j];
                            
                            next_ref_num = j;
                        }
                    }
            }
                
            if(next_ref_num < large_virtual_boundary.size()-1)
                i = next_ref_num;
            
            pair_large_virtual_boundary.push_back(ref_obj);
            list_of_pair_boundary.push_back(pair_large_virtual_boundary);
            pair_large_virtual_boundary.clear();
    }
    
//    char plotFile[90];
//        sprintf(plotFile, "%s", "Maps/Offline/Pair_informaiton_test.png");                   
//        plotObjectsColours(plotFile, list_of_pair_boundary, real_boundary, real_boundary);
    
    
    //if p2 to p1 is shorter than one of both virtual lines
    //join them as real boundary
    //otherwise compute exit from p1 to p2 
    for(int i = 0; i < list_of_pair_boundary.size(); i++)
    {
//        if((distanceOftwoP(list_of_pair_boundary[i][0].getP2(), list_of_pair_boundary[i][1].getP1()) < list_of_pair_boundary[i][0].length())
//            || (distanceOftwoP(list_of_pair_boundary[i][0].getP2(), list_of_pair_boundary[i][1].getP1()) < list_of_pair_boundary[i][1].length()))
//            temp_modified_obj.set(list_of_pair_boundary[i][0].X2(), list_of_pair_boundary[i][0].Y2(), list_of_pair_boundary[i][1].X1(), list_of_pair_boundary[i][1].Y1(), i);
//        else
        temp_modified_obj.set(list_of_pair_boundary[i][0].X1(), list_of_pair_boundary[i][0].Y1(), list_of_pair_boundary[i][1].X2(), list_of_pair_boundary[i][1].Y2(), i);
        modified_boundary.push_back(temp_modified_obj);
    
    }
    
    
    return modified_boundary;
    
}


vector<Object> ultimate_trim_boundary(vector<Object> complete_boundary, vector<Object> trim_lines)
{
    unsigned int counter = 0;
    unsigned int next_trim_point = 0;
    
    vector<Object> final_boundary;
    
    for(int i = 0; i < complete_boundary.size(); i++)
    {
        if(complete_boundary[i].getP2() == trim_lines[counter].getP1())
        {
            final_boundary.push_back(complete_boundary[i]);
            next_trim_point = i;
            do
            {
                next_trim_point++;
            }while(complete_boundary[next_trim_point].getP1() != trim_lines[counter].getP2());
            
            i = next_trim_point - 1;//share point 2 object - 1
            counter++;
        }
        else
            final_boundary.push_back(complete_boundary[i]);
    }
    
    return final_boundary;
}


//test programme section
void test_combine(vector< vector<Object> > robots, vector< vector<Object> > views)
{
    char plotFile[80];
    
    vector<Object> combine;
    combine = views.back();
    //for(int i = views.size()-2; i > 0 ; i--)
    for(int i = 0; i < views.size()-1; i++)
    {
        //combine = addTwoVectorsOfObjects(combine, views[i]);
        
        sprintf(plotFile, "%s%d%s", "Maps/Offline/combine_last_few_asr-", i, ".png");
        plotObjects(plotFile, views[i], views[i+1]);
    }    
}

void test_position(vector< vector<Object> > robots, vector< vector<Object> > views)
{
    int i = 1;
    char plotFile[80];
    
    vector<Object> current_view_poly, boundary_poly;
    
    boundary_poly = current_view_poly = makePolygonOfView(views[0]);
    do
    {
        if(pointInPolygon(robots[i][6].getP1(), current_view_poly) == false)
        {
            current_view_poly = makePolygonOfView(views[i-1]);
            boundary_poly = polygon_of_boundary(boundary_poly, current_view_poly, robots[i-1], i-1);
            sprintf(plotFile, "%s%d%s", "Maps/Offline/large_environemnt_with_robot-", i-1, ".png");
            plotObjects(plotFile, boundary_poly, robots[i-1]);
        }
        else
        {
            sprintf(plotFile, "%s%d%s", "Maps/Offline/large_environemnt_with_robot-", i, ".png");
            plotObjects(plotFile, boundary_poly, robots[i]);
        }
        i++;
        
    }while(i < views.size());
}


//check 
pair< vector<Object>, vector<Object> > check_postion_and_env(vector<Object> robot, vector<Object> view, vector<Object> MFIS, int v)
{
//    int checker = 0;
//    double dist_threshold = 300;
//    double angle_threshold = 10;
//    double expAngle = 0;
//    
//    vector<Object> match_pair;
//    vector< vector<Object> > match_list;
//    
//    pair< vector<Object>, vector<Object> > temp;
//    
//    //check distance 
//    for(int i = 0; i < global_map.size(); i++)
//    {
//            for(int j = 0; j < view.size(); j++)
//            {
//                    expAngle = view[j].getAngleWithLine(global_map[i]);
//                    //if matched 
//                    if(((abs(expAngle) < 30.0 || abs(expAngle) > 330.0) || (abs(expAngle) > 176.6 && abs(expAngle) < 189.0)) 
//                        && (view[j].length() >= 800) 
//                        && ((P_To_ShortestDistance(view[j].getP1(), global_map[i]) < 800)||(P_To_ShortestDistance(view[j].getP2(), global_map[i]) < 800)))
//                    {
//                            match_pair.push_back(view[j]);
//                            match_pair.push_back(global_map[i]);
//                            break;
//                    }
//                    
//                    if(v >= 12)
//                    {
//                    cout << " angle : " << expAngle << endl;
//                    cout << " dist1 : " << P_To_ShortestDistance(view[j].getP1(), global_map[i]) << endl;
//                    cout << " dist2 : " << P_To_ShortestDistance(view[j].getP2(), global_map[i]) << endl;
//                    waitHere();
//                    }
//            }
//    }
//    
//    cout << " match_list : " << match_list.size() << endl;
//    
//    for(int i = 0; i < match_list.size(); i++)
//    {
//            expAngle = match_list[i][0].getAngleWithLine(match_list[i][1]);
//            cout << "exp angle : " << expAngle << endl;
//            if(abs(expAngle) > angle_threshold)
//                checker++;
//    }
//    cout << " num of checker : " << checker << endl;
//    cout << " size of view   : " << view.size() << endl;
//    vector<Object> adjust_view, adjust_robot;
//    if(double(checker)/double(view.size()) > 0.5)
//    {
//            
//            //re-adjust t
//            //rotate
//            adjust_view = TransformforToGlobalCoordinate(view, Point(0,0), robot, match_list[0][0].getAngleWithLine(match_list[0][1])); 
//
//            //shift
//            double shift_offset;
//            shift_offset = P_To_ShortestDistance(match_list[0][0].getP1(), match_list[0][1]);
//            if(isLeft(match_list[0][1].getP1(), match_list[0][1].getP2(), match_list[0][0].getP1()) > 0)
//                shift_offset = -shift_offset;
//            
//            adjust_view = TransformforToGlobalCoordinate(adjust_view, Point(shift_offset, 0), robot, 0); 
//
//            //adjust view 
//            adjust_robot = projectingTheView(robot, adjust_view[0], view[0], 1);
//
//            temp.first = adjust_robot;
//            temp.second = adjust_view;
//
//    }
//    else
//    {
//            temp.first = robot;
//            temp.second = view;
//    }
    
    char plotFile[80];
    int flag_left = 0, flag_right = 0;
    int correct_ref_flag = 0;
    double expAngle1, expDist1, expDist2, expDist_mid1;
    double expAngle2, expDist3, expDist4, expDist_mid2;
    
    Object left_side_on_mfis, right_side_on_mfis;
    Object left_ref, right_ref;
    
    vector<Object> reference_obj;
    vector<Object> adjust_robot, adjust_view;
    
    pair< vector<Object>,  vector<Object> > temp; //for return 
    
    
    left_ref = expend_Object(robot[7], 1200, 1);
    right_ref = expend_Object(robot[7], 1200, 2);
    
    //if left or right 
    left_side_on_mfis = intersectForObject(MFIS, left_ref.getP1(), left_ref.getP2());
    right_side_on_mfis = intersectForObject(MFIS, right_ref.getP1(), right_ref.getP2());
    
    //match 
    if((left_side_on_mfis.X1() != 0) && (left_side_on_mfis.X2() != 0))
    {
        expAngle1 = left_side_on_mfis.getAngleWithLine(view[0]);
        expDist1 = P_To_ShortestDistance(view[0].getP1(), left_side_on_mfis);
        expDist2 = P_To_ShortestDistance(view[0].getP2(), left_side_on_mfis);

        expDist_mid1 = P_To_ShortestDistance(view[0].midpoint(), left_side_on_mfis);

        if (((abs(expAngle1) < 30.0 || abs(expAngle1) > 330.0) || (abs(expAngle1) > 176.6 && abs(expAngle1) < 189.0)) 
                && (expDist1 < 600.0 || expDist2 < 600.0) && (expDist_mid1 < 500))
            flag_left = 1;
    }
    
    if((right_side_on_mfis.X1() != 0) && (right_side_on_mfis.X2() != 0))
    {
        expAngle2 = right_side_on_mfis.getAngleWithLine(view.back());
        expDist3 = P_To_ShortestDistance(view.back().getP1(), right_side_on_mfis);
        expDist4 = P_To_ShortestDistance(view.back().getP2(), right_side_on_mfis);

        expDist_mid2 = P_To_ShortestDistance(view.back().midpoint(), right_side_on_mfis);

        if (((abs(expAngle2) < 30.0 || abs(expAngle2) > 330.0) || (abs(expAngle2) > 176.6 && abs(expAngle2) < 189.0)) 
                && (expDist3 < 600.0 || expDist4 < 600.0) && (expDist_mid2 < 500))
            flag_right = 1;
    }
    
    cout << " flag left  : " << flag_left << endl;
    cout << " flag right : " << flag_right << endl; 
    
    //adjust parameter
    if((flag_left == 1) && (flag_right == 1))
    {
        //compute weight 
        double W_left_ref, W_right_ref;
        
        W_left_ref = (3+4)/(expDist_mid1+expAngle1);
        W_right_ref = (3+4)/(expDist_mid2+expAngle2);
        if(W_left_ref > W_right_ref)
        {
            reference_obj.push_back(left_side_on_mfis);
            reference_obj.push_back(view[0]);
            flag_right = 0;
        }
        else
        {
            reference_obj.push_back(right_side_on_mfis);
            reference_obj.push_back(view.back());
            flag_left = 0;
        }
    }
    else
    {
        if(flag_left == 1)
        {
            reference_obj.push_back(left_side_on_mfis);
            reference_obj.push_back(view[0]);
        }
        else
        {
            if(flag_right == 1)
            {
                reference_obj.push_back(right_side_on_mfis);
                reference_obj.push_back(view.back());
            }
        }
    }
    

    
    //adjust errors (robot & view)
    
    if((flag_left == 1) || (flag_right == 1))
    {

                //rotate
                double angle = abs(reference_obj[0].getAngleWithLine(reference_obj[1]));
                if(angle > 180)
                    angle -= 180;

                adjust_view = TransformforToGlobalCoordinate(view, Point(0,0), robot, angle); 

                
                
                //shift
                
                double shift_offset;

                Object shift_ref;

                if(flag_left == 1)
                    shift_ref = adjust_view[0];
                else
                    if(flag_right == 1)
                        shift_ref = adjust_view.back();
                
                Point joint_cross_point = crossPerpend(reference_obj[0].getP1(), reference_obj[0].getP2(), shift_ref.getP1());
                Object test_line;
                test_line.set(shift_ref.X1(), shift_ref.Y1(), joint_cross_point.X(), joint_cross_point.Y(),0);
                
                for(int i = 0; i < MFIS.size(); i++)
                {
                    if((TwoObjectIntersect(MFIS[i], test_line) == true) && (MFIS[i].length() != reference_obj[0].length()) 
                        && (parallel_objects(MFIS[i], shift_ref) == true))
                    {
                        correct_ref_flag = 1;
                        reference_obj[0] = MFIS[i];

                        break;
                    }

                }

                shift_offset = P_To_ShortestDistance(shift_ref.getP1(), reference_obj[0]);

                double perpend_dist1 = perpendicularDis(reference_obj[0].getP1(), reference_obj[0].getP2(), shift_ref.getP1());
                double perpend_dist2 = perpendicularDis(reference_obj[0].getP1(), reference_obj[0].getP2(), shift_ref.getP2());
                //if(shift_offset > 1000)
                if(perpend_dist1 != shift_offset)
                {
                    shift_offset = P_To_ShortestDistance(shift_ref.getP2(), reference_obj[0]);

                    if(perpend_dist2 != shift_offset)
                        shift_offset = perpend_dist1;
                }


                if((isLeft(reference_obj[0].getP1(), reference_obj[0].getP2(), shift_ref.getP1()) < 0) && ((reference_obj[0].Y2() > reference_obj[0].Y1()))) //&& (abs(reference_obj[0].Y2() - reference_obj[0].Y1())<300)))
                    shift_offset = -shift_offset;
                if((isLeft(reference_obj[0].getP1(), reference_obj[0].getP2(), shift_ref.getP1()) > 0) && ((reference_obj[0].Y2() < reference_obj[0].Y1()))) //&& (abs(reference_obj[0].Y2() - reference_obj[0].Y1())<300)))
                    shift_offset = -shift_offset;


                
                if(v >= 14)
                {
                    cout << " v is step@ " << v << endl;
                    cout << " left flag  : " << flag_left << endl;
                    cout << " right flag : " << flag_right << endl;
                    cout << " shift off  : " << shift_offset << endl;
                    cout << " diff angle : " << reference_obj[0].getAngleWithLine(reference_obj[1]) << endl;
                    char plotFile[80];
                    sprintf(plotFile, "%s%d%s", "Maps/Offline/TEST_PROGRRAME-", v, ".png");
                    plotObjectsOf3Kinds(plotFile, reference_obj, robot, adjust_view);
                    //waitHere();
                }
                adjust_view = TransformforToGlobalCoordinate(adjust_view, Point(shift_offset, 0), robot, 0); 


            //adjust view 
            adjust_robot = projectingTheView(robot, adjust_view[0], view[0], 1);

            temp.first = adjust_robot;
            temp.second = adjust_view;
   
    }
    else
    {
lb:     adjust_robot = robot;
        adjust_view = view;
    }
    
    temp.first = adjust_robot;
    temp.second = adjust_view;
    
    return temp;
    
}


vector<Object> bound_map(vector< vector<Object> > robots, vector< vector<Object> > views)
{
    cout << " ----- Compute boundary map function ----- " << endl;
    
    vector<Object> boundary_map; //final map to return 
    vector<Object> temp_map;     //temp map
    
    vector<Object> plotting;
    
    Object reliable_ref_old, reliable_ref_new; //for projecting robot position
    
    char plotFile[100];
    
    double angle_threshold = 15;     //degree
    double distance_threshold = 400; //mm
    
    temp_map = views[0];             //first step map, initialization 
    
    
    for(int i = 1; i < views.size(); i++)
    {
            //process each of view        
            int pointer = 0;
            for(int j = 0; j < views[i].size(); j++)
            {
                    //process each of object in view
                    int match_flag = 0;
                    for(int m = pointer; m < temp_map.size(); m++)
                    {
                            //check same object from map
                            double include_angle  = abs(temp_map[m].getAngleWithLine(views[i][j]));
                            double dist_between_mid_point = distanceOftwoP(views[i][j].midpoint(), temp_map[m].midpoint());
                            double perpend_distance_mid_point = Perpendiculardistance(temp_map[m].getP1(), temp_map[m].getP2(), views[i][j].midpoint());
                            double shortest_dist = shortestDistanceBtwTwoObjects(temp_map[m], views[i][j]);
                            double distance;
                            if((dist_between_mid_point > perpend_distance_mid_point) && (perpend_distance_mid_point >= shortest_dist))
                                distance = perpend_distance_mid_point;
                            else
                                distance = dist_between_mid_point;

                            if(include_angle > 180)
                                include_angle -= 180;
                            
                            if(((views[i][j].length() >= 500) || (temp_map[m].length() >= 500))
                                && (abs(include_angle) < angle_threshold) && (distance < distance_threshold)) 
                            {
                                vector<Object> adjust_surface; //only one surface
                                adjust_surface.push_back(views[i][j]);
                      
                                if(abs(temp_map[m].getAngleWithXaxis()) < abs(views[i][j].getAngleWithXaxis()))
                                    include_angle = -include_angle;
                                
                                //rotate
                                adjust_surface = TransformforToGlobalCoordinate(adjust_surface, Point(0,0), robots[i], include_angle);   

                                //shift
                                Point shift_point, shortest_dist_point;
                                shortest_dist_point = crossPerpend(temp_map[m].getP1(), temp_map[m].getP2(), adjust_surface[0].midpoint());
                                double perpend_dist = Perpendiculardistance(temp_map[m].getP1(), temp_map[m].getP2(), adjust_surface[0].midpoint());
                                double shortest_dist = shortestDistanceBtwTwoObjects(temp_map[m], adjust_surface[0]);
                                
                                if(perpend_dist == shortest_dist)
                                    shift_point.set(shortest_dist_point.X() - adjust_surface[0].midpoint().X(), shortest_dist_point.Y() - adjust_surface[0].midpoint().Y());
                                else
                                {
                                    shortest_dist_point = crossPerpend(temp_map[m].getP1(), temp_map[m].getP2(), adjust_surface[0].getP1());
                                    perpend_dist = Perpendiculardistance(temp_map[m].getP1(), temp_map[m].getP2(), adjust_surface[0].getP1());
                                    if(perpend_dist == shortest_dist)
                                        shift_point.set(shortest_dist_point.X() - adjust_surface[0].X1(), shortest_dist_point.Y() - adjust_surface[0].Y1());
                                    else
                                    {
                                        shortest_dist_point = crossPerpend(temp_map[m].getP1(), temp_map[m].getP2(), adjust_surface[0].getP2());
                                        perpend_dist = Perpendiculardistance(temp_map[m].getP1(), temp_map[m].getP2(), adjust_surface[0].getP2());
                                        if(perpend_dist == shortest_dist)
                                            shift_point.set(shortest_dist_point.X() - adjust_surface[0].X2(), shortest_dist_point.Y() - adjust_surface[0].Y2());
                                    }
                                        
                                }
                                
                                adjust_surface = TransformforToGlobalCoordinate(adjust_surface, shift_point, robots[i], 0);
                                plotting.push_back(adjust_surface[0]);
                                //
                                vector<Object> test;
                                test.push_back(temp_map[m]);
                                sprintf(plotFile, "%s%d%s", "Maps/Offline/TEST_MATCH-", j, ".png");
                                plotObjects(plotFile, adjust_surface, test);
                                
                                //modify map
                                if(distanceOftwoP(temp_map[m].getP1(), adjust_surface[0].getP2()) > temp_map[m].length())
                                    if(adjust_surface[0].length() > temp_map[m].length())
                                {
                                    temp_map.erase(temp_map.begin()+m);
                                    temp_map.insert(temp_map.begin()+m, adjust_surface[0]);
                                }
                                
                                pointer = m;
                                match_flag = 1;
                                break;
                            }

                    }
                    
                    if(match_flag == 0)
                    {
                        //add on to map
                        temp_map.push_back(views[i][j]);
                    }
            }
            
            //Adjust/project current robot position
            //projectingTheView()
            
            //project next step robot in case restrict errors from following step
            //projectingTheView()
                    
            //affect next loop
            
            
            sprintf(plotFile, "%s%d%s", "Maps/Offline/bound_map-", i, ".png");
            plotObjects(plotFile, temp_map, plotting);
            waitHere();
            
    }
           
    
    //boundary_map = temp_map;
    return boundary_map;
}


////
vector<Object> bound_map_modified_version(vector< vector<Object> > robots, vector< vector<Object> > views)
{
    cout << " ----- Compute boundary map function ----- " << endl;
    
    vector<Object> boundary_map; //final map to return 
    vector<Object> temp_map;     //temp map
    vector<Object> ref_list;
    
    
    vector< vector<Object> > v1_list, v2_list, new_info_list;
    
    Object adjust_object;        //adjust each of surface when updating
    Object reliable_ref_old, reliable_ref_new; //for projecting robot position
    
    char plotFile[100];
    
    vector<int> to_modif_list;
    double angle_threshold = 15;     //degree
    double distance_threshold = 400; //mm
    
    //temp_map = views[0];             //first step map, initialization   
    
    //find common info between two adjacent views
    for(int i = 0; i < views.size()-1; i++)
    {
        vector<Object> common_info_pre_view;
        vector<Object> common_info_cur_view;
        vector<Object> non_matched_info;
        
        int counter = 0;
        for(int j = 0; j < views[i].size(); j++)
        {
                for(int k = counter; k < views[i+1].size(); k++)
                {
                        double include_angle  = abs(views[i+1][k].getAngleWithLine(views[i][j]));
                        double dist_between_mid_point = distanceOftwoP(views[i][j].midpoint(), views[i+1][k].midpoint());
                        double perpend_distance_mid_point = Perpendiculardistance(views[i+1][k].getP1(), views[i+1][k].getP2(), views[i][j].midpoint());
                        double shortest_dist = shortestDistanceBtwTwoObjects(views[i+1][k], views[i][j]);
                        double distance;

                        if((dist_between_mid_point > perpend_distance_mid_point) && (perpend_distance_mid_point >= shortest_dist))
                            distance = perpend_distance_mid_point;
                        else
                            distance = dist_between_mid_point;

                        if(include_angle > 180)
                            include_angle -= 180;
                        if(((views[i][j].length() >= 500) || (views[i+1][k].length() >= 500))
                                        && (include_angle < angle_threshold) && (distance < distance_threshold))
                        {
                                common_info_pre_view.push_back(views[i][j]);
                                common_info_cur_view.push_back(views[i+1][k]);

                                counter = k+1;
                                views[i+1][k].set_match_flag(3);
                                break;
                        }
   
                }
                   
        }

        for(int k = 0; k < views[i+1].size(); k++)
        {
                //non-matched information 
                if(views[i+1][k].get_match_flag() != 3)
                    non_matched_info.push_back(views[i+1][k]);
        }
        
        v1_list.push_back(common_info_pre_view);
        v2_list.push_back(common_info_cur_view);
        new_info_list.push_back(non_matched_info);
                 
    }
    
    //compute bounded map 
    for(int i = 0; i < v1_list.size(); i++)
    {
            if(i == 0)
            {
                    //initial map -- take the first view as base map
                    temp_map = temp_map = views[0];

                    //update and modify info
                    for(int j = 0; j < v1_list[i].size(); j++)
                    {
                            double diff_length = v2_list[i][j].length() - v1_list[i][j].length();
                            double proportion = diff_length/v1_list[i][j].length();

                            if(proportion > 0.1)
                            {
                                    int modi_surf_id;
                                    //find corresponding surface in map
                                    for(int k = 0; k < temp_map.size(); k++)
                                    {
                                            if(temp_map[k].length() == v1_list[i][j].length())
                                            {    modi_surf_id = k;
                                                 to_modif_list.push_back(modi_surf_id);
                                                 ref_list.push_back(v2_list[i][j]);
                                                 break;
                                            }
                                    }
                                    
                                    int exp_flag;
                                    
                                    if((distanceOftwoP(v1_list[i][j].getP1(), v2_list[i][j].getP1()) < distanceOftwoP(v1_list[i][j].getP2(), v2_list[i][j].getP2()))
                                        && (distanceOftwoP(v1_list[i][j].getP1(), v2_list[i][j].getP1()) < 500))
                                        exp_flag = 2;
                                    else
                                    {
                                        if((distanceOftwoP(v1_list[i][j].getP1(), v2_list[i][j].getP1()) > distanceOftwoP(v1_list[i][j].getP2(), v2_list[i][j].getP2()))
                                        && (distanceOftwoP(v1_list[i][j].getP2(), v2_list[i][j].getP2()) < 500))
                                            exp_flag = 1;
                                        else
                                            exp_flag = 3;

                                    }
                                    
                                    Object expend_section;
                                    //determine how to expend 
                                    if((exp_flag == 1) || (exp_flag == 2))
                                        expend_section = expend_Object(temp_map[modi_surf_id], abs(diff_length),exp_flag);   
                                    else
                                    {
                                        
                                        Point p1, p2;
                                        double d1, d2;
                                        p1 = crossPerpend(v2_list[i][j].getP1(), v2_list[i][j].getP2(), v1_list[i][j].getP1());
                                        p2 = crossPerpend(v2_list[i][j].getP1(), v2_list[i][j].getP2(), v1_list[i][j].getP2());
                                        
                                        d1 = distanceOftwoP(p1, v1_list[i][j].getP1());
                                        d2 = distanceOftwoP(p2, v1_list[i][j].getP2());
                                        
                                        expend_section = expend_Object_two_sides(temp_map[modi_surf_id], d1, d2);
                                    }
                                    
                                    switch(exp_flag)
                                    {
                                        case 1:temp_map[modi_surf_id].setP1(expend_section.X1(), expend_section.Y1());break;
                                        case 2:temp_map[modi_surf_id].setP2(expend_section.X2(), expend_section.Y2());break;
                                        case 3:temp_map.erase(temp_map.begin()+modi_surf_id);
                                               temp_map.insert(temp_map.begin()+modi_surf_id, expend_section);
                                               break;
                                        default:break;
                                    }
                            }
                            
                    }
   
            }
            else
            {
                ////
                for(int j = 0; j < v1_list[i].size(); j++)
                {
                        double diff_length = v2_list[i][j].length() - v1_list[i][j].length();
                        double proportion = diff_length/v1_list[i][j].length();

                        if(proportion > 0.1)
                        {
                                int modi_surf_id;
                                //find corresponding surface in map
                                for(int k = 0; k < temp_map.size(); k++)
                                {
                                        if(temp_map[k].length() == v1_list[i][j].length())
                                        {    modi_surf_id = k;
                                             to_modif_list.push_back(modi_surf_id);
                                             ref_list.push_back(v2_list[i][j]);
                                             break;
                                        }
                                }

                                int exp_flag;

                                if((distanceOftwoP(v1_list[i][j].getP1(), v2_list[i][j].getP1()) < distanceOftwoP(v1_list[i][j].getP2(), v2_list[i][j].getP2()))
                                    && (distanceOftwoP(v1_list[i][j].getP1(), v2_list[i][j].getP1()) < 500))
                                    exp_flag = 1;
                                else
                                {
                                    if((distanceOftwoP(v1_list[i][j].getP1(), v2_list[i][j].getP1()) > distanceOftwoP(v1_list[i][j].getP2(), v2_list[i][j].getP2()))
                                    && (distanceOftwoP(v1_list[i][j].getP2(), v2_list[i][j].getP2()) < 500))
                                        exp_flag = 2;
                                    else
                                        exp_flag = 3;

                                }

                                Object expend_section;
                                //determine how to expend 
                                if((exp_flag == 1) || (exp_flag == 2))
                                    expend_section = expend_Object(temp_map[modi_surf_id], abs(diff_length),exp_flag);   
                                else
                                {

                                    Point p1, p2;
                                    double d1, d2;
                                    p1 = crossPerpend(v2_list[i][j].getP1(), v2_list[i][j].getP2(), v1_list[i][j].getP1());
                                    p2 = crossPerpend(v2_list[i][j].getP1(), v2_list[i][j].getP2(), v1_list[i][j].getP2());

                                    d1 = distanceOftwoP(p1, v1_list[i][j].getP1());
                                    d2 = distanceOftwoP(p2, v1_list[i][j].getP2());

                                    expend_section = expend_Object_two_sides(temp_map[modi_surf_id], d1, d2);
                                }

                                switch(exp_flag)
                                {
                                    case 1:temp_map[modi_surf_id].setP1(expend_section.X1(), expend_section.Y1());break;
                                    case 2:temp_map[modi_surf_id].setP2(expend_section.X2(), expend_section.Y2());break;
                                    case 3:temp_map.erase(temp_map.begin()+modi_surf_id);
                                           temp_map.insert(temp_map.begin()+modi_surf_id, expend_section);
                                           break;
                                    default:break;
                                }
                        }
                }  
            }
            
            //choose one ref from modified info
                    
                    
            //project new information that is from second view
            vector<Object> adjust_rest_info = projectingTheView(new_info_list[i], temp_map[to_modif_list[0]], ref_list[0], 1);
                    adjust_rest_info = TransformforToGlobalCoordinate(adjust_rest_info, Point (0,250), adjust_rest_info, 6);

            //update them onto map
            temp_map = addTwoVectorsOfObjects(temp_map, adjust_rest_info);
                    
            //initail
            to_modif_list.clear();
            
            vector<Object> test;
            
            test.push_back(ref_list[0]);
            test.push_back(temp_map[to_modif_list[0]]);
            
            sprintf(plotFile, "%s%d%s", "Maps/Offline/bound_map-", i, ".png");
            plotObjects(plotFile, temp_map, adjust_rest_info);
            
            sprintf(plotFile, "%s%d%s", "Maps/Offline/combined_info-", i, ".png");
            plotObjectsOf3Kinds(plotFile, v1_list[i], v2_list[i], new_info_list[i]);
            //waitHere();
    }

    
    boundary_map = temp_map;
    return boundary_map;
}

//////
vector<Object> bound_map_modified_version2(vector< vector<Object> > robots, vector< vector<Object> > views)
{
    cout << " ----- Compute boundary map function ----- " << endl;
    
    char plotFile[100];
    vector<Object> boundary_map; //final map to return 
    vector<Object> temp_map;     //temp map
    vector<Object> non_match;
    
    temp_map = views[0];
    
    /*
    //find common info between two adjacent views
    for(int i = 1; i < views.size(); i++)
    {
        int cunter = 0;
        vector<Object> cluster;
        vector< vector<Object> > groups;
        cluster.push_back(temp_map[0]);
        for(int i = cunter+1; i < temp_map.size(); i++)
        {
            if(cluster.back().shortestDistanceWithObject(temp_map[i]) < 500)
            {
                cluster.push_back(temp_map[i]);
            }
            else
            {
                cunter = i;
                groups.push_back(cluster);
                cluster.clear();
                cluster.push_back(temp_map[i]);
            }
        }

        vector<Object> common_info_pre_view;
        vector<Object> common_info_cur_view;
        vector<Object> non_matched_info;
        
        
        int counter = 0;
        for(int j = 0; j < views[i].size(); j++)
        {
                int match_flag = 0;
                
                for(int k = counter; k < views[i-1].size(); k++)
                {
                        double include_angle  = abs(views[i][j].getAngleWithLine(views[i-1][k]));
                        double dist_between_mid_point = distanceOftwoP(views[i][j].midpoint(), views[i-1][k].midpoint());
                        double perpend_distance_mid_point = Perpendiculardistance(views[i-1][k].getP1(), views[i-1][k].getP2(), views[i][j].midpoint());
                        double shortest_dist = shortestDistanceBtwTwoObjects(views[i-1][k], views[i][j]);
                        double distance;

                        if((dist_between_mid_point > perpend_distance_mid_point) && (perpend_distance_mid_point >= shortest_dist))
                            distance = perpend_distance_mid_point;
                        else
                            distance = dist_between_mid_point;

                        if(include_angle > 180)
                            include_angle -= 180;
                        if((include_angle < 20) && (distance < 300))
                        {
                                common_info_pre_view.push_back(views[i][j]);
                                common_info_cur_view.push_back(views[i-1][k]);

                                match_flag = 1;
                                                          
                          
                                break;
                        }
                } 
                
                if(match_flag == 0)
                {
                    
                    if(j > 0)
                    {
                        if(views[i][j].getP1() == views[i][j-1].getP2())
                            non_match.push_back(views[i][j-1]);
                    }
                    
                    non_match.push_back(views[i][j]);
                    
                    if(j < views[i].size()-1)
                    {
                        if(views[i][j].getP2() == views[i][j+1].getP1())
                            non_match.push_back(views[i][j+1]);
                    }
                }
                
        }
        
        vector<Object> matched_info;
        for(int m = 0; m < non_match.size(); m++)
        {
            //Object test_object = TransformforToGlobalCoordinate(non_match[m], Point (0,0), double(m/2));
            
                if(interSectWithLine(temp_map, non_match[m].getP1(), non_match[m].getP2()) == false)
                {
                    Object sight_line, exp;
                    sight_line.set(robots[i][6].X1(), robots[i][6].Y1(), non_match[m].midpoint().X(), non_match[m].midpoint().Y(),0);
                    exp = expend_Object(sight_line, 400, 2);
                    sight_line.setP2(exp.getP2().X(), exp.getP2().Y());
                    
                    if(interSectWithLine(temp_map, sight_line.getP1(), sight_line.getP2()) == false)
                    {
                        //temp_map.push_back(non_match[m]);
                        //temp_map.push_back(test_object);
                    }
                    
                }
                else
                {
                    Object overlap_obj = intersectForObject(temp_map, non_match[m].getP1(), non_match[m].getP2());
                    //matched_info.push_back(overlap_obj);
                    
                    if(non_match[m].length() > overlap_obj.length()) //replace old info at old place
                    {
                        //temp_map.erase(temp_map.begin()+overlap_obj.getID());
                        //temp_map.push_back(non_match[m]);
                        //temp_map.push_back(test_object);
                    }
   
                }
                

        }
        
        
        non_match.clear();
        //update non_matched info onto map
        //temp_map = addTwoVectorsOfObjects(temp_map, non_match);
        sprintf(plotFile, "%s%d%s", "Maps/Offline/bounded_map-", i, ".png");
        plotObjects(plotFile, temp_map);
        
    }
    */
    Object far_ref_obj = farthestObject(temp_map, Point (0,0));
    for(int i = 1; i < views.size(); i++)
    {
        Object tracked_farthes_obj = farthestObject(views[i], robots[i][6].getP1());
        
        double include_angle = far_ref_obj.getAngleWithLine(tracked_farthes_obj);
        
        if(include_angle > 270)
            include_angle = 360-include_angle;
        
        
        vector<Object> adjusted_view = TransformforToGlobalCoordinate(views[i], Point(0,0), robots[i], include_angle);
        vector<Object> adjusted_rb = TransformforToGlobalCoordinate(robots[i], Point(0,0), robots[i], include_angle);
        tracked_farthes_obj = farthestObject(adjusted_view, adjusted_rb[6].getP1());
        Point diff_Point(far_ref_obj.midpoint().X() - tracked_farthes_obj.midpoint().X(), far_ref_obj.midpoint().Y() - tracked_farthes_obj.midpoint().Y());
        adjusted_view = TransformforToGlobalCoordinate(adjusted_view, diff_Point, robots[i], 0);
        
        temp_map = addTwoVectorsOfObjects(temp_map, adjusted_view);
        cout << " include angle : " << include_angle << endl;
        cout << " shift x : " << diff_Point.X() << " ; y : " << diff_Point.Y() << endl;
        sprintf(plotFile, "%s%d%s", "Maps/Offline/bounded_map-", i,".png");
        plotObjects(plotFile, temp_map, adjusted_view);
        //waitHere();
    }
    
    

       
    
    boundary_map = temp_map;
    return boundary_map;
}


vector<Object> bounded_space(vector< vector<Object> > robots, vector< vector<Object> > views)
{
    char plotFile[100];
    vector<Object> path;
    vector<Object> temp_map;
    vector<Object> left_list, right_list;
    vector<Point> left_points, right_points;
    
    temp_map = views[0];
    
    for(int i = 0; i < robots.size()-1; i++)
    {
        Object path_seg;
        
        path_seg.set(robots[i][6].X1(), robots[i][6].Y1(), robots[i+1][6].X1(), robots[i+1][6].Y1(), i);
        path.push_back(path_seg);
    }
    
   
    
    //
    for(int i = 0 ; i < temp_map.size(); i++)
    {
        //left list
        if((isLeft(robots[0][6].getP1(), robots[4][6].getP2(), temp_map[i].getP1()) > 0)
            && (isLeft(robots[0][6].getP1(), robots[4][6].getP2(), temp_map[i].getP2()) > 0))
        {
            left_list.push_back(temp_map[i]);
        }
        else
        {
            //right list
            if((isLeft(robots[0][6].getP1(), robots[4][6].getP2(), temp_map[i].getP1()) < 0)
                && (isLeft(robots[0][6].getP1(), robots[4][6].getP2(), temp_map[i].getP2()) < 0))
            {
                right_list.push_back(temp_map[i]);
            }
        }
  
    }
    
    //
    //pair< vector<Object>, vector<Object> > left_And_right = LeftAndRightList(MFIS, path_segment);
    
    
    //convert to points
    left_points = ObjectToPoints(left_list);
    right_points = ObjectToPoints(right_list);
    
    vector<Exit> potential_exits = Potentialine_basePoints(left_points, right_points, left_list);
   
    //the most constraining one
    Exit constrain_exit;
    double min = 0;
    for(int i = 0; i < potential_exits.size(); i++)
    {
        if(i == 0)
        {
            min = potential_exits[i].length();
            constrain_exit = potential_exits[i];
        }
        else
        {
            if(min > potential_exits[i].length())
            {
                min = potential_exits[i].length();
                constrain_exit = potential_exits[i];
            }
        }
    }
    
    
    int  switch_position;
    
    for(int i = 0; i < robots.size()-1; i++)
    {
        Object robot_path_segment;
        robot_path_segment.set(robots[i][6].X1(), robots[i][6].Y1(), robots[i+1][6].X1(), robots[i+1][6].Y1(), i);
        
        if(twoLinintersect(robot_path_segment.getP1(), robot_path_segment.getP2(), constrain_exit.getP1(), constrain_exit.getP2()) == true)
        {
            switch_position = i+1;
            break;
        }
    }
    
//    for(int i = 0; i < switch_position; i++)
//    {
//        vector<Object> trimmed_part;
//        
//        trimmed_part = trim_boundary_base_exit(views[i], constrain_exit);
//        
//        sprintf(plotFile, "%s%d%s", "Maps/Offline/trimmed_part-", i, ".png");
//        plotObjects(plotFile, trimmed_part);
//    }
    
    temp_map.clear();
    for(int i = switch_position; i < views.size()-1; i++)
    {
        temp_map = addTwoVectorsOfObjects(temp_map, views[i]);
        
        sprintf(plotFile, "%s%d%s", "Maps/Offline/in_corridor-", i, ".png");
        plotObjects(plotFile, views[i], views[i+1]);
    }
    
    


}


Region_struct bounded_space_version2(vector< vector<Object> > robots, vector< vector<Object> > views) 
{
    char plotFile[100];
    Object path_segment;
    vector<Object> region_map; 
    
    Exit constrain_exit;
    vector<Exit> potential_exits;
    vector<Exit> useful_exits;
    
    vector<int> cluster;
    
    vector< vector<Object> > rnt;
    
    unsigned int corridor_flag = 0;
    unsigned int end_corridor_flag = 0;
    unsigned int recompute_exit_flag = 0;
    int counter = 0;
    int path_interval_steps = 4;
    int shift_position;
    int i = counter;
    
    Region_struct regions_exits;
    
    do
    {
        if((isCorridor(views[i]) == false) && (corridor_flag == 0))
        {

                /*if(i < (int)(views.size()) - 4)
                    path_segment.set(robots[i][6].X1(), robots[i][6].Y1(), robots[i+4][6].X1(), robots[i+4][6].Y1(), i);
                else
                    path_segment.set(robots[i][6].X1(), robots[i][6].Y1(), robots.back()[6].X1(), robots.back()[6].Y1(), i);

                
                constrain_exit = most_constrain_exit_on_path(views[i], path_segment);*/
                constrain_exit = most_constrain_exit_on_path(views, robots, i);
                
                Object temp_exit;
                temp_exit.set(constrain_exit.X1(), constrain_exit.Y1(), constrain_exit.X2(), constrain_exit.Y2(), 0);
                double to_exit_distance = P_To_ShortestDistance(robots[i][6].getP1(), temp_exit);
                if(to_exit_distance < 300)
                {
                    if(i < (int)(views.size()) - 4)
                        path_segment.set(robots[i+2][6].X1(), robots[i+2][6].Y1(), robots[i+4][6].X1(), robots[i+4][6].Y1(), i);
                    else
                        path_segment.set(robots[i+2][6].X1(), robots[i+2][6].Y1(), robots.back()[6].X1(), robots.back()[6].Y1(), i);
                    constrain_exit = most_constrain_exit_on_path(views[i], path_segment);
                }
                
                if(constrain_exit.length() >= 600)
                    useful_exits.push_back(constrain_exit);
                
                //potential_exits = Potentialine_basePoints(views[i], path_segment);
                //region_base_exits(views[i], potential_exits);
    //
    //            //find constrain exit
    //            cout << " find constrain exit " << endl;
    //            double min = 0;
    //            for(int j = 0; j < potential_exits.size(); j++)
    //            {
    //                    if(j == 0)
    //                    {
    //                            if((potential_exits[j].getP1() != views[i][0].getP1())
    //                                && (potential_exits[j].getP2() != views[i].back().getP2()))
    //                            {
    //                                min = potential_exits[j].length();
    //                                constrain_exit = potential_exits[j];
    //                            }
    //                    }
    //                    else
    //                    {
    //                            if((min > potential_exits[j].length()) && (min != 0))
    //                            {
    //                                min = potential_exits[j].length();
    //                                constrain_exit = potential_exits[j];
    //                            }
    //                            else
    //                            {
    //                                    if(min == 0)
    //                                    {
    //                                        min = potential_exits[j].length();
    //                                        constrain_exit = potential_exits[j];
    //                                    }
    //                            }
    //                    }
    //            }


                /*vector<Exit> test;
                test.push_back(constrain_exit);
                sprintf(plotFile, "%s%d%s", "Maps/Offline/region&exit-", i, ".png");
                plotObjectsofexits(plotFile, views[i], test);*/

                cout << " find shift position where robot crosses the exit " << endl;
                
                for(int k = counter; k < robots.size()-1; k++)
                {
                        Object robot_path_segment;
                        robot_path_segment.set(robots[k][6].X1(), robots[k][6].Y1(), robots[k+1][6].X1(), robots[k+1][6].Y1(), i);

                        if(twoLinintersect(robot_path_segment.getP1(), robot_path_segment.getP2(), constrain_exit.getP1(), constrain_exit.getP2()) == true)
                        {
                                shift_position = k+1;
                                //recompute_exit_flag = 1;
                                break;
                        }
                }

                if(shift_position > views.size())
                {
                   shift_position = views.size()-1;
                }
                //cout << " current step   : " << i << endl;
                //cout << " shift position : " << shift_position << endl;
                
                if(abs(shift_position - i) < 2)
                {
                    regions_exits.set_exits(useful_exits);
                    regions_exits.set_regions_boundry(rnt);
                    if(shift_position == views.size()-1)
                    {
                        //last view becomes the last region
                        rnt.push_back(views[i]);
                        regions_exits.set_regions_boundry(rnt);
                        return regions_exits;
                    }
                    else
                        shift_position += 1;
                }
                
                //vector<Object> connection_info = connection(views[shift_position-1], views[shift_position], constrain_exit);

                vector<Object> temp_map;
                vector<Object> local_region;
                //region_map = temp_map = trim_boundary_base_exit_region(views[i], constrain_exit);
                
//                if(i < 2)
//                {
                    region_map = views[i];
                    
                    //construct region detail
                    for(int j = i+1; j < shift_position; j++)
                        region_map = update_region(region_map, views[j], robots[j]);
//                }
//                else
//                {
//                    region_map = views[i-1];
//                    
//                    //construct region detail
//                    for(int j = i; j < shift_position; j++)
//                        region_map = update_region(region_map, views[j], robots[j]);
//                }

                
                //sprintf(plotFile, "%s%d%s", "Maps/Offline/bounded_map-", i, ".png");
                //plotObjects(plotFile, region_map, views[i]);
                //plotObjects(plotFile, region_map);

                //vector<Object> test_path;
                //test_path.push_back(path_segment);
                //sprintf(plotFile, "%s%d%s", "Maps/Offline/bounded_Exit-", i, ".png");
                //plotObjectsOf3KindswithExits(plotFile, views[i],test_path, test);

                rnt.push_back(region_map);

                i = counter = shift_position;
        }
        else
        {
                cout << " It is in corridor now.... " << endl;
                //cout << " current step : " << i << endl;
                
                if(corridor_flag == 0)
                {
                    if(i < (int)(views.size()) - 4)
                        path_segment.set(robots[i][6].X1(), robots[i][6].Y1(), robots[i+4][6].X1(), robots[i+4][6].Y1(), i);
                    else
                        path_segment.set(robots[i][6].X1(), robots[i][6].Y1(), robots.back()[6].X1(), robots.back()[6].Y1(), i);
                    constrain_exit = most_constrain_exit_on_path(views[i], path_segment);
                    
                    if(constrain_exit.length() >= 600)
                        useful_exits.push_back(constrain_exit);

                    for(int k = counter; k < robots.size()-1; k++)
                    {
                            Object robot_path_segment;
                            robot_path_segment.set(robots[k][6].X1(), robots[k][6].Y1(), robots[k+1][6].X1(), robots[k+1][6].Y1(), i);

                            if(twoLinintersect(robot_path_segment.getP1(), robot_path_segment.getP2(), constrain_exit.getP1(), constrain_exit.getP2()) == true)
                            {
                                    shift_position = k+1;
                                    break;
                            }
                    }
                    
                          
                    vector<Exit> test;
                    test.push_back(constrain_exit);
                    vector<Object> test_path;
                    test_path.push_back(path_segment);
                    
                    //if the potential exit is not crossed by path
                    //this is end of corridor
                    if(twoLinintersect(constrain_exit.getP1(), constrain_exit.getP2(), path_segment.getP1(), path_segment.getP2()) == false)
                    {
                        shift_position = i+4;
                        useful_exits.pop_back();
                        
                        //end corridor flag
                        end_corridor_flag = 1;
                    }
                    
                    if(i > 1)
                    {
                        region_map = views[i - 2];
                        i = i - 2;
                    }
                    else
                        region_map = views[i];
                    
                    //sprintf(plotFile, "%s%d%s", "Maps/Offline/bounded_Exit-", i, ".png");
                    //plotObjectsofexits(plotFile, views[i], test);
                    //plotObjectsOf3KindswithExits(plotFile, views[i],test_path, test);
                    corridor_flag = 1;
                }
                else
                {
                    corridor_flag = 1;
                    if(i > 3)
                        region_map = update_corridor_region(region_map, views[i]);
                    else
                        update_region(region_map, views[i], robots[i]);
                    
                    //sprintf(plotFile, "%s%d%s", "Maps/Offline/bounded_map-", i, ".png");
                    //plotObjects(plotFile, region_map, views[i]);
                    //plotObjects(plotFile, region_map);
                    if(i == shift_position)
                    {
                        corridor_flag = 0;
                        //if(end_corridor_flag = 0)
                            rnt.push_back(region_map);
                    }
                }
                
                i++;

                if((i == shift_position) && (end_corridor_flag == 1))
                {
                    
                    rnt.push_back(region_map);
                    corridor_flag = 0;
                    
                    regions_exits.set_exits(useful_exits);
                    regions_exits.set_regions_boundry(rnt);
                    return regions_exits;
                }
        }
        
    }while(i < views.size());
    
    
    regions_exits.set_exits(useful_exits);
    regions_exits.set_regions_boundry(rnt);
    
    return regions_exits;
}

vector<Object> update_region(vector<Object> region_map, vector<Object> view, vector<Object> robot)
{
            char plotFile[80];
            vector<Object> local_region;
            vector<Object> update_map;
            vector<Object> rest_of_info;
            
            update_map = region_map;
            local_region = makePolygonOfView(region_map);

            for(int m = 0; m < view.size(); m++)
            {
                    if((isThisCloseToCVPolygonBoundary(view[m], local_region, 200.0) == true)
                        ||((pointInPolygon(view[m].getP1(), local_region) == true 
                            || pointInPolygon(view[m].getP2(), local_region) == true)))
                    {
                            //update this region
                            unsigned int modify_flag = 0;
                            for(int i = 0; i < update_map.size(); i++)
                            {
                                    double include_angle  = abs(view[m].getAngleWithLine(update_map[i]));
                                    double dist_between_mid_point = distanceOftwoP(view[m].midpoint(), update_map[i].midpoint());
                                    double perpend_distance_mid_point = Perpendiculardistance(update_map[i].getP1(), update_map[i].getP2(), view[m].midpoint());
                                    double shortest_dist = shortestDistanceBtwTwoObjects(update_map[i], view[m]);
                                    double distance;

                                    if((dist_between_mid_point > perpend_distance_mid_point) && (perpend_distance_mid_point >= shortest_dist))
                                        distance = perpend_distance_mid_point;
                                    else
                                        distance = dist_between_mid_point;

                                    if(include_angle > 180)
                                        include_angle -= 180;

                                    if((include_angle < 15) && (distance < 400))
                                    {
                                            //mod
                                            if(view[m].length() > update_map[i].length())
                                            {
                                                //expend it 
                                                update_map[i].setP2(view[m].X2(), view[m].Y2());
                                                modify_flag = 1;
                                            }
                                    }
                            }

                            if(modify_flag == 0)
                            {
                                    Object sight_line, exp;
                                    sight_line.set(robot[6].X1(), robot[6].Y1(), view[m].midpoint().X(), view[m].midpoint().Y(),0);
                                    exp = expend_Object(sight_line, 400, 2);
                                    sight_line.setP2(exp.getP2().X(), exp.getP2().Y());

                                    if(interSectWithLine(update_map, sight_line.getP1(), sight_line.getP2()) == false)
                                        update_map.push_back(view[m]);

                            }
                    }
                    else
                        rest_of_info.push_back(view[m]);

            }

            

            //sprintf(plotFile, "%s", "Maps/Offline/bounded_map_in_region.png");
            //plotObjectsOf3Kinds(plotFile, update_map, rest_of_info, local_region);
            
            return update_map;
}




vector<Object> connection(vector<Object> view_before_cross, vector<Object> view_after_cross, Exit potential_exit)
{
            vector<Object> ref_info;
            vector<Object> modification;
            vector<Object> result_connection;
    
            Object temp_convert;
            temp_convert.set(potential_exit.X1(), potential_exit.Y1(), potential_exit.X2(), potential_exit.Y2(),0);
            
            vector<Object> area = makeSquare(temp_convert);
        
            double angle_threshold = 15;     //degree
            double distance_threshold = 400; //mm
        
        
            for(int m = 0; m < view_before_cross.size(); m++)
            {
                if((isThisCloseToCVPolygonBoundary(view_before_cross[m], area, 200.0) == true)
                    ||((pointInPolygon(view_before_cross[m].getP1(), area) == true 
                        || pointInPolygon(view_before_cross[m].getP2(), area) == true)))
                {
                    //update this region
                    ref_info.push_back(view_before_cross[m]);
                }
            }

            vector<Object> enrich_connenct_info;
            for(int i = 0; i < ref_info.size(); i++)
            {
                    for(int j = 0; j < view_after_cross.size(); j++)
                    {
                            double include_angle  = abs(view_after_cross[j].getAngleWithLine(ref_info[i]));
                            double dist_between_mid_point = distanceOftwoP(view_after_cross[j].midpoint(), ref_info[i].midpoint());
                            double perpend_distance_mid_point = Perpendiculardistance(ref_info[i].getP1(), ref_info[i].getP2(), view_after_cross[j].midpoint());
                            double shortest_dist = shortestDistanceBtwTwoObjects(ref_info[i], view_after_cross[j]);
                            double distance;

                            if((dist_between_mid_point > perpend_distance_mid_point) && (perpend_distance_mid_point >= shortest_dist))
                                distance = perpend_distance_mid_point;
                            else
                                distance = dist_between_mid_point;

                            if(include_angle > 180)
                                include_angle -= 180;

                            if((include_angle < angle_threshold) && (distance < distance_threshold))
                            {
                                //mod
                                enrich_connenct_info.push_back(view_after_cross[j]);
                            }
                    }
            }

            ref_info = addTwoVectorsOfObjects(ref_info, enrich_connenct_info);
            return ref_info;
}


//vector<Object> region_base_exits(vector<Object> view, vector<Exit> exits)
//{
//    char plotFile[80];
//    
//    vector<Object> region;
//    vector<Object> region_block;
//    
//    Object temp_virtual_Obj;
//    vector<Object> virtual_lines;
//    
//    for(int i = exits.size()-1; i >= 0; i--)
//    {
//        region = trim_boundary_base_exit_region(view, exits[i]);
//        //region_block = makePolygonOfView(region);
//        
//        for(int i = 0; i < region.size(); i++)
//        {
//            if(i == region.size() - 1)
//                temp_virtual_Obj.set(region.back().X2(), region.back().Y2(), region[0].X1(), region[0].Y1(), i);
//            else
//                 temp_virtual_Obj.set(region[i].X2(), region[i].Y2(), region[i+1].X1(), region[i+1].Y1(), i);
//            
//            
//            virtual_lines.push_back(temp_virtual_Obj);
//        }
//        
//        virtual_lines = breakTheLinesInto(virtual_lines);
//        
//        region_block = addTwoVectorsOfObjects(region, virtual_lines);
//        
//        sprintf(plotFile, "%s%d%s", "Maps/Offline/test_region_with_exit-", i,".png");
//        plotObjectsofexits(plotFile, region_block, exits);
//    }
//    waitHere();
//}

vector<Object> update_corridor_region(vector<Object> map, vector<Object> to_update_view)
{
        cout << " This is to update corridor region map !!! " << endl;
        char plotFile[80];
        vector<Object> matched_info; 
        vector<Object> ref_info;
        vector<Object> new_info;

        vector<Object> parallel_info_view;
        vector<Object> parallel_info_map;
        vector<Object> transform_view;

        int unmatched_counter = 0;
        unsigned int match_flag = 0;
        unsigned int parallel_flag = 0;

        vector< vector<double> > corridor_width_in_view;
        vector< vector<double> > corridor_width_in_map;

        //because this is function is for updating corridor
        //there must be two parallel structure
        parallel_info_view = parallel_lines(to_update_view);
        parallel_info_map = parallel_lines(map);

//        sprintf(plotFile, "%s", "Maps/Offline/parallel_info1.png");
//        plotObjects(plotFile,parallel_info_map);
//        sprintf(plotFile, "%s", "Maps/Offline/parallel_info2.png");
//        plotObjects(plotFile, parallel_info_view);
        
//        cout << " size map  : " << parallel_info_map.size() << endl;
//        cout << " size view : " << parallel_info_view.size() << endl;
        
        corridor_width_in_view = corridor_width(parallel_info_view);
        corridor_width_in_map = corridor_width(parallel_info_map);
        // cout << " test program0 " << endl;

//        cout << " size map  : " << corridor_width_in_map.size() << endl;
//        cout << " size view : " << corridor_width_in_view.size() << endl;
        
        for(int i = 0; i < corridor_width_in_map.size(); i++)
        {

            for(int j = 0; j < corridor_width_in_map[i].size(); j++)
            {

                for(int m = 0; m < corridor_width_in_view.size(); m++)
                {

                    for(int n = 0; n < corridor_width_in_view[m].size(); n++)
                    {
               
                        if(abs(corridor_width_in_map[i][j] - corridor_width_in_view[m][n]) < 200)
                        {
                            
                            ref_info.push_back(parallel_info_map[i]);
                            ref_info.push_back(parallel_info_map[i+j+1]);
                            
                            matched_info.push_back(parallel_info_view[m]);
                            matched_info.push_back(parallel_info_view[m+n+1]);

                            double angle1 = ref_info[0].getAngleWithLine(matched_info[0]);
                            double angle2 = ref_info[1].getAngleWithLine(matched_info[1]);
                            double angle = (angle1 + angle2) / 2; //average include angle 
                            
//                            sprintf(plotFile, "%s", "Maps/Offline/match_parallel_info.png");
//                            plotObjects(plotFile, ref_info, matched_info);

                            if(angle > 50)
                                continue;
                            cout << " adjust angle : " << angle << endl;
                            
                            vector<Object> transform_temp = TransformforToGlobalCoordinate(matched_info, Point (0,0), matched_info, -angle); 
                            
                            //cout << " adjusted include angle : " << ref_info[0].getAngleWithLine(transform_temp[0]) << endl;
                            
                            
                            Point shift_offset(0,0);
                            double shift_dist = perpendicularDis(ref_info[0].getP1(), ref_info[0].getP2(), transform_temp[0].midpoint());
                            if(abs(shift_dist) > 100)
                                shift_offset.set(shift_dist, 0);
                            
                            
                            transform_view = TransformforToGlobalCoordinate(to_update_view, Point (0,0), matched_info, -angle);  
                            transform_view = TransformforToGlobalCoordinate(transform_view, shift_offset, matched_info, 0); 
                            
//                            sprintf(plotFile, "%s", "Maps/Offline/match_and_adjustment.png");
//                            plotObjects(plotFile, map, transform_view);
//                            waitHere();
                            match_flag = 1;
                            goto lb;
                        }
                    }
                }
            }
        }
       
lb:     if(match_flag == 1)
        {
            vector<Object> update_mfis;

            update_mfis = map;
            for(int i = 0; i < transform_view.size(); i++) 
            {
                    int match_flag = 0;
                    for(int j = 0; j < update_mfis.size(); j++)
                    {

                        double include_angle  = abs(update_mfis[j].getAngleWithLine(transform_view[i]));
                        double dist_between_mid_point = distanceOftwoP(update_mfis[j].midpoint(), transform_view[i].midpoint());
                        double perpend_distance_mid_point = Perpendiculardistance(update_mfis[j].getP1(), update_mfis[j].getP2(), transform_view[i].midpoint());
                        double shortest_dist = shortestDistanceBtwTwoObjects(update_mfis[j], transform_view[i]);
                        double distance;

                        if((dist_between_mid_point > perpend_distance_mid_point) && (perpend_distance_mid_point >= shortest_dist))
                            distance = perpend_distance_mid_point;
                        else
                            distance = dist_between_mid_point;

                        if(include_angle > 180)
                            include_angle -= 180;

                        if((include_angle < 15) && (distance < 400))
                        {
    //                        Object modified_surf;
    //                        modified_surf = update_mfis[j];
    //
    //                        
    //                        if(transform_view[i].length() - update_mfis[j].length() > 400)
    //                        {
    //                            double d1 = distanceOftwoP(transform_view[i].getP1(),update_mfis[j].getP1());
    //                            double d2 = distanceOftwoP(transform_view[i].getP2(),update_mfis[j].getP2());
    //
    //                            if(abs(d1) > 100)
    //                            {
    //                                modified_surf = expend_Object(modified_surf, abs(d1), 1);
    //                                update_mfis[j].setP1(modified_surf.X2(), modified_surf.Y2());
    //                            }
    //
    //
    //                            if(abs(d2) > 100)
    //                            {
    //                                modified_surf = expend_Object(modified_surf, abs(d2), 2);
    //                                update_mfis[j].setP2(modified_surf.X2(), modified_surf.Y2());
    //                            }
    //                        }
                            if(update_mfis[j].length() < distanceOftwoP(update_mfis[j].getP1(), transform_view[i].getP2()))
                                update_mfis[j].setP2(transform_view[i].X2(), transform_view[i].Y2());

                            if(update_mfis[j].length() < distanceOftwoP(update_mfis[j].getP2(), transform_view[i].getP1()))
                                update_mfis[j].setP1(transform_view[i].X1(), transform_view[i].Y1());

                            match_flag = 1;
                        }
                    }

                    if(match_flag == 0)
                        new_info.push_back(transform_view[i]);
            }
            
            return update_mfis = addTwoVectorsOfObjects(update_mfis, new_info);
        }
        else
            return map;
  
}


vector<Object> parallel_lines(vector<Object> view)
{
    char plotFile[80];
    
    
    int parallel_flag = 0;
    vector<Object> parallel_info_view;
    
    for(int i = 0; i < view.size()-1; i++)
    {
       
            parallel_info_view.push_back(view[i]);
            for(int j = i + 1; j < view.size(); j++)
            {
//                vector<Object> test1, test2;
//                test1.push_back(view[i]);
//                test2.push_back(view[j]);
//
//                sprintf(plotFile, "%s", "Maps/Offline/TESTTESTTEST.png");
//                plotObjects(plotFile, test1, test2);
                
               
                
                double d1 = Perpendiculardistance(view[i].getP1(), view[i].getP2(), view[j].getP1());
                double d2 = Perpendiculardistance(view[i].getP1(), view[i].getP2(), view[j].getP2());
                
//                cout << " d1 : " << d1 << endl;
//                cout << " d2 : " << d2 << endl;
//                waitHere();
                
                if((abs(d1 - d2) < 180) && (abs(d1) >= 1000) && (abs(d2) >= 1000))
                {
                            parallel_info_view.push_back(view[j]);
                            parallel_flag = 1;
                            
                            

                            
                            
                }
                
            }
            
            if(parallel_flag == 1)
                //break;
                parallel_flag = 0;
            else
                parallel_info_view.pop_back();
    }
    
    //delete redundant surface
    for(int i = 0; i < parallel_info_view.size()-1; i++)
    {
        for(int j = i + 1; j < parallel_info_view.size(); j++)
        {
            if((parallel_info_view[i].getP1() == parallel_info_view[j].getP1())
                && (parallel_info_view[i].getP2() == parallel_info_view[j].getP2()))
            {
                parallel_info_view.erase(parallel_info_view.begin() + j);
                j--;
            }
        }
    }
    
    
    return parallel_info_view;
}

vector< vector<double> > corridor_width(vector<Object> parralle_info)
{
    vector<double> array;
    vector< vector<double> > rnt;
    
    //int ct1 = 0, ct2 = 0;
    
    for(int i = 0; i < parralle_info.size()-1; i++)
    {

            for(int j = i+1; j< parralle_info.size(); j++)
            {

                if((isParallel(parralle_info[i].getP1(), parralle_info[i].getP2(), parralle_info[j].getP1(), parralle_info[j].getP2()) == true)
                    && (parralle_info[i].getP1() != parralle_info[j].getP1()))
                {
                    double distance;
                    distance = perpendicularDis(parralle_info[j].getP1(), parralle_info[j].getP2(), parralle_info[i].midpoint());
                    array.push_back(distance);
                    
//                    char plotFile[80];
//                    vector<Object> test1, test2;
//                    
//                    test1.push_back(parralle_info[i]);
//                    test2.push_back(parralle_info[j]);
//                    
//                    sprintf(plotFile, "%s", "Maps/Offline/TESTTESTTEST.png");
//                    plotObjects(plotFile, test1, test2);
//                    waitHere();
                }

            }

            rnt.push_back(array);
            array.clear();
    }
    
    return rnt;
}

vector<Object> trim_region_map(vector<Object> map, vector<Exit> exits)
{
    int num;
    vector<Object> trim_region;
    num = exits.size();
    
    switch(num)
    {
        case 1: 
                for(int j = 0; j < map.size(); j++)
                {
                    if((isLeft(exits[0].getP1(), exits[0].getP2(), map[j].getP1()) < 0)
                        && (isLeft(exits[0].getP1(), exits[0].getP2(), map[j].getP2()) < 0))
                    {
                        trim_region.push_back(map[j]);
                    }
                }
                return trim_region;
            
        case 2:
                for(int j = 0; j < map.size(); j++)
                {
                    if((isLeft(exits[0].getP1(), exits[0].getP2(), map[j].getP1()) < 0)
                        && (isLeft(exits[0].getP1(), exits[0].getP2(), map[j].getP2()) < 0)
                        && (isLeft(exits[1].getP1(), exits[1].getP2(), map[j].getP1()) > 0)
                        && (isLeft(exits[1].getP1(), exits[1].getP2(), map[j].getP2()) > 0))
                    {
                        trim_region.push_back(map[j]);
                    }
                }
                return trim_region;
        default: return map;
    }
}

vector<Object> join_boundary(vector<Object> regions)
{
    Point potent; 
    Object insert, potential_next;
    vector<Object> rnt;
    vector<Point> convert_points, temp;
    
    int next_id = 0;
    double dist, dist1, dist2, min;
    
    convert_points = ObjectToPoints(regions);
    
    temp.push_back(convert_points[0]);
 lb:for(int i = 0; i < convert_points.size()-1; i++)
    {
//        if(is_first_endpoint(regions, temp.back()) == true)
//        {
//            temp.push_back(convert_points[i+1]);
//            continue;
//        }
        
        for(int j = 0; j < convert_points.size(); j++)
        {
            dist = distanceOftwoP(temp.back(), convert_points[j]);
            //if(dist == 0)
            //    goto lb;
            
            if(j == i+1)
            {
                min = dist;
                potent = convert_points[j];
                next_id = j;
            }
            else
            {
                if((min > dist) && (include_point(temp, convert_points[j]) == false))
                {
                    min = dist;
                    potent = convert_points[j];
                    next_id = j;
                }
            }
        }
        
        if(is_first_endpoint(regions, temp.back()) == false)
        {
              temp.push_back(potent);
              //i = next_id;
        }
        else
              temp.push_back(return_second_endpoint(regions, temp.back()));
        
    }
                 
    for(int i = 0; i < temp.size()-1; i++)
    {
        insert.set(temp[i].X(), temp[i].Y(), temp[i+1].X(), temp[i+1].Y(), i);
        rnt.push_back(insert);
    }
 
 for(int i = 0; i < rnt.size(); i++)
 {
     if(rnt[i].length() > 3000)
     {
         rnt.erase(rnt.begin()+i);
         i--;
     }
 }
    
//    rnt.push_back(regions[0]);
//    
//    for(int i = 1; i < regions.size(); i++)
//    {
//        for(int j = i+1; j < regions.size(); j++)
//        {
//            dist1 = distanceOftwoP(rnt.back().getP2(), regions[j].getP1());
//            dist2 = distanceOftwoP(rnt.back().getP2(), regions[j].getP2());
//            
//            if(dist1 < dist2)
//                dist = dist1;
//            else
//                dist = dist2;
//                   
//            if(j == i+1)
//            {
//                min = dist;
//                potential_next = regions[j];
//                next_id = j;
//            }
//            else
//            {
//                if(min > dist)
//                {
//                    min = dist;
//                    potential_next = regions[j];
//                    next_id = j;
//                }
//            } 
//        }
//        
//        if(min != 0)
//        {
//            if(dist1 < dist2)
//                insert.set(rnt.back().X2(), rnt.back().Y2(), potential_next.X1(), potential_next.Y1(), 0);
//            else
//                insert.set(rnt.back().X2(), rnt.back().Y2(), potential_next.X2(), potential_next.Y2(), 0);
//            
//            rnt.push_back(insert);
//        }
//        
//        rnt.push_back(potential_next);
//        //i = next_id;
//    }
    
    return rnt;
}

vector<Object> join_boundary_verison2(vector<Object> region, Exit exit)
{
    int cnt = 0, counter = 1;
    double dist, dist1, dist2, min;
    
    Object potential_next, ref1, ref2, insert;
    vector<Object> rnt;
    
//    for(int i = 0; i < region.size(); i++)
//    {
////        cout << " surface x : " << region[i].X1() << " ; y : " << region[i].Y1() << endl;
////        cout << " exit    x : " << exit.X1()      << " ; y : " << exit.Y1() << endl;
////        cout << " diff      : " << region[i].X1() - exit.X1() << endl;
////        waitHere();
//        if(abs(region[i].X1() - exit.X1()) <= 0.01)// && (region[i].Y1() == exit.Y1()))
//        {
//            rnt.push_back(region[i]);
//            break;
//        }
//    }

    cout << " size of rnt : " << rnt.size() << endl;
    if(rnt.size() == 0)
        rnt.push_back(region[0]);
    
    do
    {
        //find nearest surface
        for(int j = 0; j < region.size(); j++)
        {
            dist1 = distanceOftwoP(rnt.back().getP2(), region[j].getP1());
            dist2 = distanceOftwoP(rnt.back().getP2(), region[j].getP2());
            
            if(dist1 < dist2)
                dist = dist1;
            else
                dist = dist2;
            
            if(j == 0)
            {
                min = dist;
                potential_next = region[j];
                cnt = j;
            }
            else
            {
                if((min > dist) && (include_surface(rnt, region[j]) == false))
                {
                    min = dist;
                    potential_next = region[j];
                    cnt = j;
                }
            }
        }

        //remove the surface 
        //cout << " map size  : " << region.size() << endl;
        //cout << " erase one : " << cnt << endl;
        if(region.size() > 0)
            region.erase(region.begin()+cnt);

        
        //push it into return
        if(min == 0)
              rnt.push_back(potential_next);
        else
        {
            if(dist1 < dist2)
                insert.set(rnt.back().X2(),rnt.back().Y2(), potential_next.X1(), potential_next.Y1(), 0);
            else
                insert.set(rnt.back().X2(),rnt.back().Y2(), potential_next.X2(), potential_next.Y2(), 0);
                
            insert.set_imagined_flag(1);
            rnt.push_back(insert);
            rnt.push_back(potential_next);
        }
        
        //for plotting
        //char plotFile[80];
        //vector<Object> tes_test;
        //tes_test.push_back(potential_next);
        //sprintf(plotFile, "%s%d%s", "Maps/Offline/TESTTESTTEST-", counter,".png");
        //plotObjects(plotFile, rnt, tes_test);
        //cout << " stop here " << endl;
        
        if((distanceOftwoP(potential_next.getP2(), rnt[0].getP1()) < 300) && (counter > 1))
            break;
        if(region.size() == 0)
        {
            insert.set(rnt.back().X2(),rnt.back().Y2(), rnt[0].X1(), rnt[0].Y1(), 0);
            insert.set_imagined_flag(1);
            
            rnt.push_back(insert);
            break;
        }
            
        counter++;
    }while((rnt.back().getP1() != exit.getP2()) && (rnt.back().getP2() != exit.getP2()));
    
    return rnt;
}

bool include_point(vector<Point> list, Point p)
{
    for(int i = 0; i < list.size(); i++)
    {
        if(list[i] == p)
            return true;
    }
    
    return false;
}

bool include_surface(vector<Object> list, Object surface)
{
    for(int i = 0; i <  list.size(); i++)
    {
        if((list[i].getP1() == surface.getP1())
            && (list[i].getP2() == surface.getP2()))
        {
            return true;
        }
    }
    
    return false;
}

bool is_first_endpoint(vector<Object> view, Point p)
{
    for(int i = 0; i < view.size(); i++)
    {
        if(view[i].getP1() == p)
            return true;
    }
    
    return false;
}

Point return_second_endpoint(vector<Object> view, Point p)
{
    for(int i = 0; i < view.size(); i++)
    {
        if(view[i].getP1() == p)
            return view[i].getP2();
    }
}

/*void test_function(vector<Object> v, Point p1, Point p2)
{
    char plotFile[80];
    vector<Object> test1, test2;
    for(int i = 0; i < v.size(); i++)
    {
        if((distanceOftwoP(v[i].getP1(), p1) < 1000) || (distanceOftwoP(v[i].getP2(), p2) < 1000))
        {
            if(distanceOftwoP(v[i].getP1(), p1) < 1000)
            {
                cout << " match p1: " << endl;
                test1.push_back(v[i]);
            }
            if(distanceOftwoP(v[i].getP2(), p2) < 1000)
            {
                cout << " match p2: " << endl;
                test2.push_back(v[i]);
            }
            
            cout << " exact p1: " << v[i].getP1().X() << " ," << v[i].getP1().Y() << endl;
            cout << " exact p2: " << v[i].getP2().X() << " ," << v[i].getP2().Y() << endl;
            
            
            sprintf(plotFile, "%s", "Maps/Offline/TESTTESTTEST.png");
            plotObjects(plotFile, test1, test2);
            waitHere();
        }
    }
}*/

vector<Object> bounded_surfaces(vector<Object> map, vector<Object> bound_box)
{
    double threshold = 1000;
    double distance = 0;
    
    vector<Object> temp;
    
    for(int i = 0; i < map.size(); i++)
    {
        for(int j = 0; j < bound_box.size(); j++)
        {
            double d1 = Perpendiculardistance(bound_box[j].getP1(), bound_box[j].getP2(), map[i].getP1());
            double d2 = Perpendiculardistance(bound_box[j].getP1(), bound_box[j].getP2(), map[i].getP2());
            
            if(d1 < d2)
                distance = d1;
            else
                distance = d2;
            
            if(distance < threshold)
            {
                temp.push_back(map[i]);
                break;
            }
        }
    }
    
    return temp;
}

void integrate_regions(vector< vector<Object> > regions_outline, vector< vector<Object> > regions_map)
{
    char plotFile[80];
    vector< vector<Object> >  test;
    
    for(int i = regions_outline.size() - 1; i >= 0; i--)
    {
        double x = 0, y = 0, angle = 0;
        char yes;
        
        if(i == regions_outline.size() - 1)
            test.push_back(regions_outline.back());
        else
        {
            char yes;
            double x, y, angle;
            do
            {
                cout << "x : ";
                cin >> x;
                cout << "y : ";
                cin >> y;
                cout << "angle : ";
                cin >> angle;
                
                vector<Object> adjustment = TransformforToGlobalCoordinate(regions_outline[i], Point (x,y), regions_outline[i], angle); 
                test.push_back(adjustment);
                sprintf(plotFile, "%s", "Maps/Offline/TESTTESTTEST.png");
                plotObjectsColours(plotFile, test);
                cout << " yes? : ";
                cin >> yes;
                
                if(yes == 'y')
                    test.pop_back();
//                else
//                {
//                    test.pop_back();
//                    region_outline_modify(adjustment, test);
//                    test.push_back(adjustment);
//                    sprintf(plotFile, "%s", "Maps/Offline/TESTTESTTEST.png");
//                    plotObjectsColours(plotFile, test);
//                }
            
            }while(yes == 'y');
        }
        
//        if(i == regions_outline.size() - 4)
//        {
//            char yes;
//            double x, y, angle;
//            do
//            {
//                    cout << "x : ";
//                    cin >> x;
//                    cout << "y : ";
//                    cin >> y;
//                    cout << "angle : ";
//                    cin >> angle;
//
//                    vector<Object> adjustment = TransformforToGlobalCoordinate(test_view, Point (x,y), regions_outline[i], angle); 
//                    sprintf(plotFile, "%s", "Maps/Offline/TESTTESTTEST.png");
//                    plotObjectsColours(plotFile, test, adjustment);
//                    cout << " yes? : ";
//                    cin >> yes;
//
//            }while(yes == 'y');
//        }

    }
}


vector<Object> region_geometry(vector< vector<Object> > regions, vector<Exit> exits)
{
    char plotFile[80];
    
    Exit entrance;
    vector<Exit> exit_entance;
    vector<Object> trim_region;
    vector<Object> bound_outline;
    vector<Object> bounding_box1, bounding_box2;
    vector< vector<Object> > all_bound_region, all_temp;
    
    //trim region based on two exits 
    for(int i = regions.size() - 1; i > 0; i--)
    {
        /*if(i == regions.size() - 1)
        {
            //first, compute region only, no need to trim
            trim_region = regions[i];
        }
        else
        {
            exit_entance.push_back(exits[i]);
            exit_entance.push_back(exits[i-1]);
            trim_region = trim_region_map(regions[i], exit_entance);
            entrance = exits[i-1];
        }*/
        //bounding_box1 = join_boundary_verison2(regions[i], entrance);

            bounding_box1 = getBoundingBox(regions[i]);
            bounding_box2 = getBoundingBox(regions[i-1]);
            //adjustment_region_map(regions[i], regions[i-1]);
            
            vector<Surface> polygon = convertObjectToSurface(bounding_box1);
            for(int j = 0; j < regions[i-1].size(); j++)
            {
                if(((pointInPolygon(PointXY(regions[i-1][j].X1(), regions[i-1][j].Y1()), polygon) == true)
                    || (pointInPolygon(PointXY(regions[i-1][j].X2(), regions[i-1][j].Y2()), polygon) == true)))

                {
                    //select_info.push_back(region2[i]);
                    regions[i-1].erase(regions[i-1].begin()+j);
                }
            }
            
            if( i == regions.size() - 1)
            {
                vector<Object> temp = addTwoVectorsOfObjects(regions[i], bounding_box1);
                all_bound_region.push_back(temp);
                all_temp.push_back(regions[i]);
                temp.clear();
                temp = addTwoVectorsOfObjects(regions[i-1], bounding_box2);
                all_bound_region.push_back(temp);
                all_temp.push_back(regions[i-1]);
            }
            else
            {
                vector<Object> temp = addTwoVectorsOfObjects(regions[i-1], bounding_box2);
                all_bound_region.push_back(temp);
                all_temp.push_back(regions[i-1]);
            }
            
        //sprintf(plotFile, "%s%d%s", "Maps/Offline/TESTTESTTEST_outline-",i,".png");
        //plotObjectsOf4Kinds(plotFile, bounding_box1,regions[i], bounding_box2, regions[i-1]);

        //vector<Object> test, test_box;
        //test = addTwoVectorsOfObjects(regions[i], regions[i-1]);
        //test_box = getBoundingBox(test);
        
        //sprintf(plotFile, "%s%d%s", "Maps/Offline/TESTTESTTEST_comined-",i,".png");
        //plotObjectsOf4Kinds(plotFile, test, test_box, bounding_box1,regions[i]);
        
        //if(test_box includes 3 points of bouding_box1)
        //adjust some surfaces
        //return test;
    }

       //sprintf(plotFile, "%s", "Maps/Offline/TEST_ALL_REGIONS-1.png");
       //plotObjectsColours(plotFile, all_bound_region);
       //sprintf(plotFile, "%s", "Maps/Offline/TEST_ALL_REGIONS-2.png");
       //plotObjectsColours(plotFile, all_temp);
    
}

/*
void adjustment_region_map(vector<Object> &region1, vector<Object> &region2)
{
    vector<Object> select_info;
        vector<Object>    bounding_box1 = getBoundingBox(region1);
        //vector<Object>    bounding_box2 = getBoundingBox(region2);
        vector<Surface> polygon = convertObjectToSurface(bounding_box1);
        for(int i = 0; i < region2.size(); i++)
        {
            if(((pointInPolygon(PointXY(region2[i].X1(), region2[i].Y1()), polygon) == true)
                || (pointInPolygon(PointXY(region2[i].X2(), region2[i].Y2()), polygon) == true))
              && (region2[i].length() > 3000))
            {
                //select_info.push_back(region2[i]);
                region2.erase(region2.begin()+i);
                waitHere();
                break;
            }
        }
        
}*/

/*
void bounded_free_space(vector<Object> perceive_info, vector< vector<Object> > robots, Exit crossed_exit)
{
    vector<Object> full_path;
    vector<Object> space; 
    
    Object 
    Point temp_left_point, temp_right_point; 
    vector<Point> left_points, right_points;
          
    //compute path
    for(int i = 0; i < robots.size()-1; i++)
    {
        Object temp_line;
        temp_line.set(robots[i][6].X1(), robots[i][6].Y1(), robots[i+1][6].Y1(), robots[i+1][6].Y1(), i);
          full_path.push_back(temp_line);
    }
    
    
    //clear information
    for(int i = 0; i < perceive_info.size(); i++)
    {
        if(interSectWithLine(full_path, perceive_info[i].getP1(), perceive_info[i].getP2()) == true)
        {
            perceive_info.erase(perceive_info.begin()+i);
            i--;
        }
    }
    
    
    //compute region of free space 
    
    
}*/



void find_orientation_at_place(vector<Object> current_MFIS, vector<Object> memory_MFIS, Exit current_exit, Exit memory_exit)
{
    char mfisFileName[80];
    
    vector<Object> adjust_current_mfis, adjust_memory_mfis; 
    vector< vector<Object> > match_pair_info;
           
    //take the position of exit as centre of coordiante
    adjust_current_mfis = TransformforToGlobalCoordinate(current_MFIS, 
                                                                    Point (0-current_exit.getmidPoint().X(), 0-current_exit.getmidPoint().Y()),
                                                                    current_MFIS,0);
    adjust_memory_mfis = TransformforToGlobalCoordinate(memory_MFIS, 
                                                                   Point (0 - memory_exit.getmidPoint().X(), 0 - memory_exit.getmidPoint().Y()),
                                                                   memory_MFIS,0);
    
    int adjust_angle = 1;
    int matchNum = 0;
    do
    {
        match_pair_info.clear();
        vector<Object> match_info, ref_info;
        adjust_current_mfis = TransformforToGlobalCoordinate(adjust_current_mfis, Point (0, 0), current_MFIS,adjust_angle);
        adjust_angle++;
        
        double dist_to_ref = 500;
        
        for(int i = 0; i < adjust_current_mfis.size(); i++)
        {
            for(int j = 0; j < adjust_memory_mfis.size(); j++)
            {
                double expAngle = adjust_memory_mfis[j].getAngleWithLine_Acute(adjust_current_mfis[i]);
                //if matched 
                if(((abs(expAngle) < 30.0))// || abs(expAngle) > 330.0)) 
                    && (adjust_current_mfis[i].length() >= 1000) && (memory_MFIS[j] .length() >= adjust_current_mfis[i].length()) 
                    && (abs(shortestDistanceBtwTwoObjects(adjust_current_mfis[i], adjust_memory_mfis[j]) < dist_to_ref)))//&& abs(P_To_ShortestDistance(adjust_current_mfis[i].getP2(), memory_MFIS[j]) < dist_to_ref)))
                {
                        match_info.push_back(adjust_current_mfis[i]);
                        ref_info.push_back(adjust_memory_mfis[j]);
                        break;
                }
            }
        }
        
        match_pair_info.push_back(match_info);
        match_pair_info.push_back(ref_info);
        
        matchNum = match_info.size();
        //cout << " how many surface macthed : " << matchNum << endl;
        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Adjust_current_mfis_with_old-", adjust_angle, ".png");                   
        //plotObjectsOf3KindswithExits(mfisFileName,adjust_view, temp_adjust_mfis2 , exit_in_region);
        plotObjectsOf3Kinds(mfisFileName, adjust_current_mfis, match_info, adjust_memory_mfis);
        //waitHere();
        
    }while(matchNum < 3 || adjust_angle > 90);
    
    //adjust_current_mfis = TransformforToGlobalCoordinate(adjust_current_mfis, Point (0, 0), current_MFIS,50);

    /*exit_in_region = Transform_Exit_GlobalCoordinate(exit_in_region,
                                         Point (0-exit_in_region[0].getmidPoint().X(), 0-exit_in_region[0].getmidPoint().Y()),
                                                                   all_chunks[0],0);
    exit_in_region = Transform_Exit_GlobalCoordinate(exit_in_region,
                                         Point (0, 0),
                                                                   all_chunks[0],50);*/
    //the largest ref to do further adjustment
    int cnt;
    double max;
    for(int i = 0; i < match_pair_info[0].size(); i++)
    {
        if(i == 0)
        {
            max = match_pair_info[0][i].length();
            cnt = i;
        }
        else
        {
            if(max < match_pair_info[0][i].length())
            {
                max = match_pair_info[0][i].length();
                cnt = i;
            }
        }
    }
    
    //adjust again
    //adjust_current_mfis = projectingTheView(adjust_current_mfis, );
    
    //if two p1 match to p2
    
    match_pair_info[0][cnt].reverse();
    
    
    adjust_current_mfis = projectingTheView_ondiff_point(adjust_current_mfis, match_pair_info[1][cnt], match_pair_info[0][cnt], 2);
    vector<Object> test;
    test.push_back(match_pair_info[0][cnt]);
    test.push_back(match_pair_info[1][cnt]);
    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Adjust_current_mfis_with_old-", 100, ".png");                   
    //plotObjectsOf4Kinds(mfisFileName, adjust_current_mfis, adjust_memory_mfis, match_pair_info[0], match_pair_info[1]);
    plotObjectsOf3Kinds(mfisFileName, adjust_current_mfis, adjust_memory_mfis, test);
    
    /*vector<Object> adjust_view, adjust_robot_position;

        adjust_robot_position = TransformforToGlobalCoordinate(currentRobotPositionInMFIS, 
                                                                   Point (0-exit_in_region[0].getmidPoint().X(), 0-exit_in_region[0].getmidPoint().Y()),
                                                                   all_chunks[0],0);
        adjust_view = TransformforToGlobalCoordinate(transformed_view, 
                                                                   Point (adjust_robot_position[6].X1()-currentRobotPositionInMFIS[6].X1(), adjust_robot_position[6].Y1()-currentRobotPositionInMFIS[6].Y1()),
                                                                   all_chunks[0],0);*/
    //return robot with orientation
    
}

pair< vector<Object>, vector<Object> > find_orientation_at_place(vector<Object> current_MFIS, vector<Object> memory_MFIS, 
                                                                 vector<Object> current_robot, Exit current_exit, Exit memory_exit)
{
    char mfisFileName[80];
    
    vector<Object> adjust_robot, adjust_map;
    vector<Object> adjust_current_mfis, adjust_memory_mfis; 
    vector< vector<Object> > match_pair_info;
    
    pair< vector<Object>, vector<Object> > temp;
           
    //take the position of exit as centre of coordiante
    adjust_current_mfis = TransformforToGlobalCoordinate(current_MFIS, 
                                                                    Point (0-current_exit.getmidPoint().X(), 0-current_exit.getmidPoint().Y()),
                                                                    current_MFIS,0);
    adjust_memory_mfis = TransformforToGlobalCoordinate(memory_MFIS, 
                                                                   Point (0 - memory_exit.getmidPoint().X(), 0 - memory_exit.getmidPoint().Y()),
                                                                   memory_MFIS,0);
    
    int adjust_angle = 1;
    int matchNum = 0;
    do
    {
        match_pair_info.clear();
        vector<Object> match_info, ref_info;
        adjust_current_mfis = TransformforToGlobalCoordinate(adjust_current_mfis, Point (0, 0), current_MFIS,adjust_angle);
        adjust_angle++;
        
        double dist_to_ref = 500;
        
        for(int i = 0; i < adjust_current_mfis.size(); i++)
        {
            for(int j = 0; j < adjust_memory_mfis.size(); j++)
            {
                double expAngle = adjust_memory_mfis[j].getAngleWithLine_Acute(adjust_current_mfis[i]);
                //if matched 
                if(((abs(expAngle) < 30.0))// || abs(expAngle) > 330.0)) 
                    && (adjust_current_mfis[i].length() >= 1000) && (memory_MFIS[j] .length() >= adjust_current_mfis[i].length()) 
                    && (abs(shortestDistanceBtwTwoObjects(adjust_current_mfis[i], adjust_memory_mfis[j]) < dist_to_ref)))//&& abs(P_To_ShortestDistance(adjust_current_mfis[i].getP2(), memory_MFIS[j]) < dist_to_ref)))
                {
                        match_info.push_back(adjust_current_mfis[i]);
                        ref_info.push_back(adjust_memory_mfis[j]);
                        break;
                }
            }
        }
        
        match_pair_info.push_back(match_info);
        match_pair_info.push_back(ref_info);
        
        matchNum = match_info.size();
        //cout << " how many surface macthed : " << matchNum << endl;
        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Adjust_current_mfis_with_old-", adjust_angle, ".png");                   
        //plotObjectsOf3KindswithExits(mfisFileName,adjust_view, temp_adjust_mfis2 , exit_in_region);
        plotObjectsOf3Kinds(mfisFileName, adjust_current_mfis, match_info, adjust_memory_mfis);
        //waitHere();
        
    }while(matchNum < 3 || adjust_angle > 90);
    
    //adjust_current_mfis = TransformforToGlobalCoordinate(adjust_current_mfis, Point (0, 0), current_MFIS,50);

    /*exit_in_region = Transform_Exit_GlobalCoordinate(exit_in_region,
                                         Point (0-exit_in_region[0].getmidPoint().X(), 0-exit_in_region[0].getmidPoint().Y()),
                                                                   all_chunks[0],0);
    exit_in_region = Transform_Exit_GlobalCoordinate(exit_in_region,
                                         Point (0, 0),
                                                                   all_chunks[0],50);*/
    //the largest ref to do further adjustment
    int cnt;
    double max;
    for(int i = 0; i < match_pair_info[0].size(); i++)
    {
        if(i == 0)
        {
            max = match_pair_info[0][i].length();
            cnt = i;
        }
        else
        {
            if(max < match_pair_info[0][i].length())
            {
                max = match_pair_info[0][i].length();
                cnt = i;
            }
        }
    }
    
    //adjust again
    //adjust_current_mfis = projectingTheView(adjust_current_mfis, );
    
    //if two p1 match to p2
    //match_pair_info[0][cnt].reverse();
    
    Object ref_old, ref_new;
    //ref_new = getObject(current_MFIS, match_pair_info[0][cnt].getID());
    //ref_old = getObject(memory_MFIS, match_pair_info[1][cnt].getID());
    
    for(int i = 0; i < current_MFIS.size(); i++)
    {
        if(abs(current_MFIS[i].length() - match_pair_info[0][cnt].length()) < 10e-6)
        {
            ref_new = current_MFIS[i];
            break;
        }
    }
    
    for(int i = 0; i < memory_MFIS.size(); i++)
    {
        if(memory_MFIS[i].length() == match_pair_info[1][cnt].length())
        {
            ref_old = memory_MFIS[i];
            break;
        }
    }
    
    
    ref_new.reverse();
            
    adjust_current_mfis = projectingTheView_ondiff_point(current_MFIS, ref_old, ref_new, 2);
    //adjust_current_mfis = projectingTheView_ondiff_point(adjust_current_mfis, match_pair_info[1][cnt], match_pair_info[0][cnt], 2);
    adjust_robot = projectingTheView_ondiff_point(current_robot, ref_old, ref_new, 2);
    
    //vector<Object> test;
    //test.push_back(ref_new);
    //test.push_back(ref_old);
    //sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Adjust_current_mfis_with_old-", 100, ".png");                   
    //plotObjectsOf4Kinds(mfisFileName, adjust_current_mfis, adjust_memory_mfis, match_pair_info[0], match_pair_info[1]);
    //plotObjectsOf3Kinds(mfisFileName, adjust_current_mfis, memory_MFIS, adjust_robot);
    

    //return robot with orientation
    
    temp.first = adjust_current_mfis;
    temp.second = adjust_robot;
    
    return temp;
}


vector<Object> exit_and_region_along_path(vector< vector<Object> > views, vector< vector<Object> > robots, vector<Object> mfis)
{
    char plotFile[80];
    int start_flag = 0 , i = 0;
    vector<int> cross_points;
    cross_points.push_back(0); //initial step
    
    vector<Exit> remember_exis_in_chunk; 
    vector<Exit> exits_from_view;
    vector<Exit> exits_collect_alone_path;
    
    vector< vector<Object> > maps;
    
    do
    {
            if(start_flag == 0)
            {
                    //compute exits from view
                    exits_from_view = findShortestExits(views[i]);
                    exits_collect_alone_path = exits_from_view;
                    start_flag = 1;
                    //cout << " test 1 " << endl;
            }
            else
            {
                    //compute exits from view
                    exits_from_view = findShortestExits(views[i]);
                    exits_collect_alone_path = addTwoExits(exits_collect_alone_path,exits_from_view);
                    vector<Object> conver_exits = convertExitToObject(exits_collect_alone_path);
                    //cout << " test 2 " << endl;

                    if(interSectWithLine(conver_exits, robots[i][6].getP1(), robots[i-1][6].getP1()) == true)
                    {
                            Object  intersect_line = intersectForObject(conver_exits, robots[i][6].getP1(), robots[i-1][6].getP1());
                 
                            //find the corresponding exit
                            Exit crossed_exit;
                            for(int j = 0; j < exits_collect_alone_path.size(); j++)
                            {
                                if((exits_collect_alone_path[j].getP1() == intersect_line.getP1())
                                    && (exits_collect_alone_path[j].getP2() == intersect_line.getP2()))
                                    crossed_exit = exits_collect_alone_path[j];
                            }

                            //store important exit
                            remember_exis_in_chunk.push_back(crossed_exit);
               
                            //construct region detail
                            vector<Object> region_map = views[cross_points.back()];
                            for(int j = cross_points.back(); j < i-1; j++)
                                region_map = update_region(region_map, views[j], robots[j]);
                            
                            maps.push_back(region_map);
                            
                            //initial this variable again
                            exits_collect_alone_path.clear();
                            exits_collect_alone_path = exits_from_view;

                            cross_points.push_back(i);
                            start_flag = 0;
                    }
                    
                    if(i == views.size()-1)
                    {
                        vector<Object> region_map = views[cross_points.back()];
                        for(int j = cross_points.back(); j <= i; j++)
                            region_map = update_region(region_map, views[j], robots[j]);
                        
                        maps.push_back(region_map);
                        
                        exits_from_view = findShortestExits(views[cross_points.back()]);
                        remember_exis_in_chunk = addTwoExits(remember_exis_in_chunk,exits_from_view);
                    }
            }
            
            i++;
        
    }while(i < views.size());
    
    
    vector<Object> geomtry_map = addTwoVectorsOfObjects(maps.back(), maps[maps.size()-2]);
    
    sprintf(plotFile, "%s", "Maps/Offline/TEST_exit_AND_Maps.png");                   
    plotObjectsColoursAndExits(plotFile, maps, robots[0],remember_exis_in_chunk);
    
    /*
    vector<Object> tempcomb, region_map, geo_shape;
    for(int m = 0; m < views.size()-1; m++)
    {
        tempcomb = addTwoVectorsOfObjects(tempcomb, views[m]);
        
        if(m > 0)
            region_map = update_region(region_map, views[m], robots[m]);
        else
            region_map = views[m];
        
        sprintf(plotFile, "%s%d%s", "Maps/Offline/TEST_exit_AND_Maps-", m,".png");                   
        plotObjectsOf3Kinds(plotFile, region_map, robots[m], views[m]);
    }
    

    for(int m = 0; m < tempcomb.size(); m++)
    {
        if((isLeft(remember_exis_in_chunk[0].getP1(), remember_exis_in_chunk[0].getP2(), tempcomb[m].getP1()) > 0) && (isLeft(remember_exis_in_chunk[0].getP1(), remember_exis_in_chunk[0].getP2(), tempcomb[m].getP2()) > 0)
            && (isLeft(remember_exis_in_chunk[1].getP1(), remember_exis_in_chunk[1].getP2(), tempcomb[m].getP1()) < 0) && (isLeft(remember_exis_in_chunk[1].getP1(), remember_exis_in_chunk[1].getP2(), tempcomb[m].getP2()) < 0))
        {
            trim_geo.push_back(tempcomb[m]);
        }
    }
    //geo_shape = makePolygon_Clipper(trim_geo, 1);
    geo_shape = makePolygon_Clipper(tempcomb, 1);
    
    for(int m = 0 ; m < geo_shape.size(); m++)
    {
        if((geo_shape[m].length() > 10000) || (geo_shape[m].length() < 1))
        {
            geo_shape.erase(geo_shape.begin()+m);
            m--;
        }
    }*/
    

    
    return geomtry_map;
}

pair< vector<Object>, vector<Object> > update_geo_map(vector< vector<Object> > views, vector< vector<Object> > robots)
{
    
    
    int cnt = 0;
    
    vector<Point> points;
    vector<Object> path;
    vector<Object> geo_map;
    vector<Object> poly_of_view;
    
    pair< vector<Object>, vector<Object> > temp;
    
    if(views.size() < 25)
        cnt = 0;
    
    do
    {
        vector<Object> info_in_view;
        vector<Object> info_out_view;
        vector<Object> matched_info_view, matched_info_map;
        
        if(cnt == 25)
        {
            geo_map = views[cnt];
            
            points.push_back(robots[cnt][6].getP1());
        }
        else
        {
            poly_of_view = makePolygonOfView(views[cnt]);
            poly_of_view = makePolygon_Clipper(poly_of_view, 50);

            
            for(int i = 0; i < geo_map.size(); i++)
            {
                if(((pointInPolygon(geo_map[i].getP1(), poly_of_view) == true 
                                || pointInPolygon(geo_map[i].getP2(), poly_of_view) == true)))
                {
                    info_in_view.push_back(geo_map[i]);
                }
                else
                    info_out_view.push_back(geo_map[i]);
            }
        
            for(int i = 0; i < info_in_view.size(); i++)
            {
                for(int j = 0; j < views[cnt].size(); j++)
                {
                    double expAngle = views[cnt][j].getAngleWithLine(info_in_view[i]);
                    double expDist = shortestDistanceBtwTwoObjects(views[cnt][j], info_in_view[i]);

                    //matched information 
                    if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && (expDist < 500.0))
                    {
                        matched_info_map.push_back(info_in_view[i]);
                        matched_info_view.push_back(views[cnt][j]);
                        break;
                    }     
                }
            }
        
            vector<Object> rest_info;
            for(int i = 0; i < views[cnt].size(); i++)
            {
                int flag = 0;
                for(int j = 0; j < matched_info_view.size(); j++)
                {
                    if((views[cnt][i].X1() == matched_info_view[j].X1())
                        || (views[cnt][i].X2() == matched_info_view[j].X2()))
                    {
                        flag = 1;
                    }
                    
                    
                }
                if(flag == 0)
                        rest_info.push_back(views[cnt][i]);
            }
            
            //matched_info_view = deleteSmallObjects(matched_info_view);
            geo_map = info_out_view;
            geo_map = addTwoVectorsOfObjects(geo_map, matched_info_map);
            geo_map = addTwoVectorsOfObjects(geo_map, rest_info);
            
            points.push_back(robots[cnt][6].getP1());
            
        }
        
        //sprintf(plotFile, "%s%d%s", "Maps/Offline/TEST_polygon-", cnt, ".png");                   
        //plotObjects(plotFile, poly_of_view, geo_map);
        cnt++;
        
    }while(cnt < views.size());
    
    for(int i = 0; i < points.size() - 1; i++)
    {
        Object segmen;
        segmen.set(points[i].X(), points[i].Y(), points[i+1].X(), points[i+1].Y(), i);
        path.push_back(segmen);
    }
    
    geo_map = clear_map(geo_map);

    temp.first = geo_map;
    temp.second = path;
    return temp;
}


vector< vector<Object> > cluster_objects_follow_path(vector<Object> map, vector<Object> block)
{
        char plotFile[80];
        vector<Object> area;
        vector< vector<Object> > clusters;

        for(int i = 0; i < block.size(); i++)
        {
            vector<Surface> polygon;
            vector<Object> poly_area;
            vector<Object> inside_info;
            //compute an area
            if(i%2 != 0)
                poly_area = makeSquare(block[i], block[i].length(), 2);
            else
                poly_area = makeSquare(block[i], block[i].length(), 1);
            polygon = convertObjectToSurface(poly_area);
            //check if any surface allocate in the area
            for(int j = 0; j < map.size(); j++)
            {
                //collect these surfaces
                if(pointInPolygon(PointXY(map[j].X1(), map[j].Y1()), polygon) == true 
                               || pointInPolygon(PointXY(map[j].X2(), map[j].Y2()), polygon) == true)
                {
                    
                    inside_info.push_back(map[j]);
                }
            }
            
            clusters.push_back(inside_info);
            area = addTwoVectorsOfObjects(area, poly_area);
        }
        
        
        return clusters;
}

vector<Object> geometry_sides(vector< vector<Object> > sides)
{
    char plotFile[80];
    vector<Object> ref_lines;
    vector<Object> temp;
    
    for(int i = 0; i < sides.size(); i++)
    {
        Object line;
        vector<Point> points;
        points = ObjectToPoints(sides[i]);
        sort(points.begin(), points.end(), compareInterval);
        line.set(points[0].X(), points[0].Y(), points.back().X(), points.back().Y(), 0);
        ref_lines.push_back(line);
                
    }
    
    for(int i = 0; i < ref_lines.size()-1; i++)
    {
        if(ref_lines[i].length() >= 2000)
        {
            Point cross_point;
            Object obj1, obj2;
            
            obj1 = expend_Object_two_sides(ref_lines[i], ref_lines[i].length()/2, ref_lines[i].length()/2);
            
            if(ref_lines[i+1].length() > 2000)
                obj2 = expend_Object_two_sides(ref_lines[i+1], ref_lines[i+1].length()/2, ref_lines[i+1].length()/2);
            else
                if(i+2 < ref_lines.size())
                    obj2 = expend_Object_two_sides(ref_lines[i+2], ref_lines[i+1].length()/2, ref_lines[i+2].length()/2);
            
            cross_point = intersectPointTwolineEquations(obj1, obj2);
            
            if(distanceOftwoP(ref_lines[i].getP1(), cross_point) < distanceOftwoP(ref_lines[i].getP1(), cross_point))
            obj1.set(ref_lines[i].X1(), ref_lines[i].Y1(), cross_point.X(), cross_point.Y(), i);
            obj2.set(cross_point.X(), cross_point.Y(), ref_lines[i+1].X1(), ref_lines[i+1].Y1(), i+1);
            
            temp.push_back(obj1);
            if(i == ref_lines.size()-2)
                temp.push_back(obj2);
        }
    }
    
    

    sprintf(plotFile, "%s", "Maps/Offline/TEST_track_side.png");                   
    plotObjects(plotFile, ref_lines, temp);
    waitHere();
        
    
    for(int i = 0; i < temp.size()-1; i++)
    {
        int flag = 0;
        Object insert;
        if((temp[i].getP2() != temp[i+1].getP1())
            && (i != temp.size()-2))
        {
            insert.set(temp[i].getP2().X(), temp[i].getP2().Y(), temp[i+1].getP1().X(), temp[i+1].getP1().Y(), i+1);
            flag = 1;
        }
        else
        {
            if((i == temp.size()-2)
                && (temp[i].getP2() != temp[0].getP1()))
            {
                insert.set(temp[i].getP2().X(), temp[i].getP2().Y(), temp[0].getP1().X(), temp[0].getP1().Y(), i+1);
                flag = 1;
            }
        }
            
        if(flag == 1)
            temp.insert(temp.begin()+i+1, insert);
    }
        
    return temp;
    
}



Object track_geometry(vector<Object> view, vector<Object> robot, Object ref_line)
{
    vector<Point> collect_info, convert;
    convert = ObjectToPoints(view);
    

    for(int i = 0; i < convert.size(); i++)
    {
        if(Perpendiculardistance(ref_line.getP1(), ref_line.getP2(), convert[i]) < 500)
            collect_info.push_back(convert[i]);
    }
  
    Object temp, expend;
    temp.set(collect_info[0].X(), collect_info[0].Y(), collect_info.back().X(), collect_info.back().Y(), 0);
   
    if(temp.length() < ref_line.length())
    {
        expend = expend_Object(temp, abs(temp.length() - ref_line.length()), 2);
        temp.set(collect_info[0].X(), collect_info[0].Y(), expend.X2(), expend.Y2(), 0);
    }
    
    Point endpoint2;
    vector<Point> potential_end_points;
    for(int i = 0; i < view.size(); i++)
    {
        
        if(Perpendiculardistance(temp.getP1(), temp.getP2(), view[i].midpoint()) < 1500)
        {
            potential_end_points.push_back(view[i].midpoint());
        }
    }
    
    cout << " test 1 : " << potential_end_points.size() << endl;
    double dist, min;
    for(int i = 0; i < potential_end_points.size(); i++)
    {
        dist = distanceOftwoP(potential_end_points[i], temp.getP2());
        if(i == 0)
        {
            min = dist;
            endpoint2 = potential_end_points[i];
        }
        else
        {
            if(min > dist)
            {
                min = dist;
                endpoint2 = potential_end_points[i];
            }
        }
    }
    
    temp.set(collect_info[0].X(), collect_info[0].Y(), endpoint2.X(), endpoint2.Y(), 0);
    
//    char plotFile[80];
//    vector<Object> test;
//    test.push_back(temp);
//    sprintf(plotFile, "%s", "Maps/Offline/TEST_track_side.png");                   
//    plotObjects(plotFile, view, test);
    
    return temp;
}

vector<Object> shift_to_close_object(vector<Object> view, vector<Object> side)
{
    vector<Object> temp;
    
    for(int i = 0; i < side.size()-1; i++)
    {
        Object shift_obj;
        Object close_object;
        Point shift_point;
        
        double min;
        
        
        for(int j = 0; j < view.size(); j++)
        {
            double dist = view[j].shortestDistanceWithObject(side[i]);
            
            if(j == 0)
            {
                min = dist;
                close_object = view[j];
                
            }
            else
            {
                if(min > dist)
                {
                    min = dist;
                    close_object = view[j];
                }
            }
        }
        
        shift_point = crossPerpend(side[i].getP1(), side[i].getP2(), close_object.midpoint());
        shift_obj = TransformforToGlobalCoordinate(side[i], Point (close_object.midpoint().X() - shift_point.X(), close_object.midpoint().Y() - shift_point.Y()), 0);
        
        if(distanceOftwoP(shift_obj.midpoint(), side.back().midpoint()) >= distanceOftwoP(side[i].midpoint(), side.back().midpoint()) / 2)
            temp.push_back(shift_obj);  
        else
            temp.push_back(side[i]);
    }
    
    temp.push_back(side.back());       
    
    return temp;
}

Exit find_exit_cross(vector<Object> map, Object side_of_geometry, vector<Object> last_step, vector<Object> current_step)
{
    double threshold = 300; 
    Object path;
    vector<Object> collect_info;
    
    for(int i = 0; i < map.size(); i++)
    {
        if(side_of_geometry.shortestDistanceWithObject(map[i]) < threshold)
            collect_info.push_back(map[i]);
    }
    
    path.set(last_step[6].getP1().X(), last_step[6].getP1().Y(), current_step[6].getP1().X(), current_step[6].getP1().Y(), 0);
    
    Exit constrain_exit;
    vector<Exit> potential_exits = Potentialine_basePoints(collect_info, path);
    
        double min = 0;
        for(int j = 0; j < potential_exits.size(); j++)
        {
                if(j == 0)
                {
                            min = potential_exits[j].length();
                            constrain_exit = potential_exits[j];
                }
                else
                {
                        if((min > potential_exits[j].length()) && (min != 0))
                        {
                            min = potential_exits[j].length();
                            constrain_exit = potential_exits[j];
                        }
                        else
                        {
                                if(min == 0)
                                {
                                    min = potential_exits[j].length();
                                    constrain_exit = potential_exits[j];
                                }
                        }
                }
        }
    
        return constrain_exit;
}


vector<Object> select_info_in_geometry(vector<Object> geometry, vector<Object> view)
{
    vector<Object> temp;
    

          vector<Surface> polygon;

            polygon = convertObjectToSurface(geometry);
            //check if any surface allocate in the area
            for(int j = 0; j < view.size(); j++)
            {
                //collect these surfaces
                if(pointInPolygon(PointXY(view[j].X1(), view[j].Y1()), polygon) == true 
                               || pointInPolygon(PointXY(view[j].X2(), view[j].Y2()), polygon) == true
                               || isThisCloseToCVPolygonBoundary(view[j], geometry, 300.0) == true)
                {
                    
                    temp.push_back(view[j]);
                }
            }

            return temp;
}


Object base_line_for_geometry(vector<Object> view, vector<Object> robot, Exit exit_on_side)
{
    Object temp;
    //Object ref_line;
    
    vector<Object> group_on_ref_side;
    
    for(int i = view.size() - 1; i >= view.size() - 8; i--)
    {
        group_on_ref_side.push_back(view[i]);
    }
        
    vector<Point> convert_points = ObjectToPoints(group_on_ref_side);
    
    temp.set(convert_points[0].X(), convert_points[0].Y(), convert_points.back().X(), convert_points.back().Y(), 0);
    
    return temp;
}


vector<Object> collect_object_base_side(vector<Object> view, Object one_side)
{
    double threshold = 500;
    vector<Object> collect_info;
    
    for(int i = 0; i < view.size(); i++)
    {
        if(one_side.shortestDistanceWithObject(view[i]) < threshold)
            collect_info.push_back(view[i]);
    }
    
    return collect_info;
}

Exit most_constrain_gap(vector<Object> group1, vector<Object> group2)
{
    Exit rnt;
    Point p1, p2;
    Object obj1, obj2;
    
    for(int i = 0; i < group1.size(); i++)
    {
        obj1 = group1[i];
        
        double dist, min;
        for(int j = 0; j < group2.size(); j++)
        {
            dist = shortestDistanceBtwTwoObjects(obj1, group2[j]);
            
            if(j == 0)
            {
                obj2 = group2[j];
                min =dist;
            }
            else
            {
                if(dist < min)
                {
                    obj2 = group2[j];
                    min =dist;
                }
            }
        }
    }
    
    if(obj1.distP1ToP1(obj2) < obj1.distP2ToP1(obj2))
        p1 = obj1.getP1();
    else
        p1 = obj1.getP2();
    
    if(obj2.distP1ToP1(obj1) < obj2.distP2ToP1(obj1))
        p2 = obj2.getP1();
    else
        p2 = obj2.getP2();
    
    rnt.set(p1.X(), p1.Y(), p2.X(), p2.Y());
    return rnt;
}

/*
vector<Object> geometry_sides(vector<Object> map)
{
    Point p1, p2, p3, p4;
    Object side1, side2, side3, side4;
    
    vector<Object> large_surfaces;
    vector< vector<Object> > clusters; 
    
    //large surface
    for(int i = 0; i < map.size(); i++)
    {
        if(map[i].length() >= 500)
            large_surfaces.push_back(map[i]);
    }
    
    //remove redundant surface
    for(int i = 0; i < large_surfaces.size(); i++)
    {
        for(int j = i + 1; j < large_surfaces.size(); j++)
        {
            double expAngle = large_surfaces[j].getAngleWithLine(large_surfaces[i]);
            double expDist = shortestDistanceBtwTwoObjects(large_surfaces[j], large_surfaces[i]);

            //matched information 
            if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && (expDist < 500.0))
            {
                if(large_surfaces[j].length() > large_surfaces[i].length())
                {
                    large_surfaces.erase(large_surfaces.begin() + i);
                    i--;
                    break;
                }
                else
                {
                    if(large_surfaces[j].length() <= large_surfaces[i].length())
                    {
                        large_surfaces.erase(large_surfaces.begin() + j);
                        j--;
                    }
                }
            }
        }
    }
    
    //cluster 
    int cnt = 0;
    vector<Object> group;
    group.push_back(large_surfaces[cnt]);
    for(int i = 0; i < large_surfaces.size(); i++)
    {
        
        if(shortestDistanceBtwTwoObjects(group[0], large_surfaces[i]) < 500)
        {
            group.push_back(large_surfaces[i]);
            large_surfaces.erase(large_surfaces.begin()+i);
            i--;
        }
        else
        {
            if(cnt == 0)
                cnt = i;
        }
        
        if(i == large_surfaces.size()-1)
        {
            clusters.push_back(group);
            group.clear();
            i = cnt;
            
            //empty set
            if(large_surfaces.size() == 0)
                break;
            else
                group.push_back(group.push_back(large_surfaces[cnt]));
            
            
        }
    }
    
    //form the ref side 
    Object ref_side;
    
    for(int i = 0; i < clusters.size(); i++)
    {
        
    }
    
    //opposite side
    
    
    //one perpendicular side
    
    //the last side
    
}*/

Object shift_line_along_perpendicluar(Object line, double shift_distance, int side)
{
    Point shift_p1, shift_p2;
    Object shift_object;
    
    shift_p1 = outSidePerpendPointWithLength(line, shift_distance, line.getP1(), side);
    shift_p2 = outSidePerpendPointWithLength(line, shift_distance, line.getP2(), side);
            
            
    shift_object.set(shift_p1.X(), shift_p1.Y(), shift_p2.X(), shift_p2.Y(), line.getID());
    return shift_object;
}


