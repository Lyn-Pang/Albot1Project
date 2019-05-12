/*		 Expectation process
                    including landmark surfaces process
			
			by Lyn		
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "Point.H"
#include "Plotting.H"
#include "Object.H"
#include "GeometricOp.H"
#include "GeometryFuncs.H"
#include "mfisOp.H"
#include "PerceptualMapping.H"
#include "ToolFunctions.h"
#include "Transporter.H"
#include "Expectation.h"
#include "GeometryAndExit.h"
#include "CompareASR.H"


int expect_process_flag = 0;
vector<Object> remaining;

/* if it is turning in place, it will expect what just appeared in a particular direction
 * project earlier information/landmarks onto current view
 */
vector<Object> expectation_process(vector<Object> current_ref, vector<Object> previous_ref, 
                                       vector<Object> landmarkInPV,vector<Object> landmarkInPPV)
{

    Object one_ref_surface;
    vector<Object> rnt;
    //vector<Object> current_ref, previous_ref;
    vector<Object> remaked_landmarkInPV;
    
    //recognized current landmark surfaces 

    remaked_landmarkInPV = projectingTheView(landmarkInPV, current_ref[0], current_ref[1], 1);
    
    //any recognised landmark is common landmark
    for(int i = 0; i < remaked_landmarkInPV.size(); i++)
    {
        if(remaked_landmarkInPV[i].length() - previous_ref[1].length() < 10e-6)
            one_ref_surface = remaked_landmarkInPV[i];
    }
                  
    //spatial relaitons
    rnt = projectingTheView(landmarkInPPV, one_ref_surface, previous_ref[1], 1);
   
    return rnt;
}


//if it is overlapping with last two views
//using recognised surface to project a complete global map
vector<Object> Project_backward_MFIS(vector<Object> currentMFIS, vector<Object> oldMFIS, vector<Object> references)
{
    vector<Object> project_info;
    vector<Object> current_and_project;
    
    int ref_point = 1;
    double distp1p1, distp1p2, distp2p1, distp2p2;
    
    distp1p1 = distanceOftwoP(references[0].getP1(), references[1].getP1());
    distp1p2 = distanceOftwoP(references[0].getP1(), references[1].getP2());
    distp2p1 = distanceOftwoP(references[0].getP2(), references[1].getP1());
    distp2p2 = distanceOftwoP(references[0].getP2(), references[1].getP2());
    
    //if ref[0] endpoint1 is ref point -- ref point is 2
    if(((distp1p1 < distp2p1) && (distp1p1 < distp2p2))
            || ((distp1p2 < distp2p1) && (distp1p2 < distp2p2)))
    {
        cout << " using the p1 as ref point" << endl;
        ref_point = 1;
    }
    else
    {
        //if ref[0] endpoint2 is ref point -- ref point is 1
        if(((distp2p1 < distp1p1) && (distp2p1 < distp1p2))
                || ((distp2p2 < distp1p1) && (distp2p2 < distp1p2)))
        {
            cout << " using the p2 as ref point" << endl;
            ref_point = 2;
        }
    }
    
    project_info = projectingTheView(oldMFIS, references[0], references[1], ref_point);
    
    //current_and_project = addTwoVectorsOfObjects(currentMFIS, project_info);
    
    char mfisFileName[80];

    sprintf(mfisFileName, "%s%d%s", "Maps/Offline/Project_backward_MFIS-", 0, ".png");
    //plotObjectsOf3Kinds(mfisFileName,project_info, currentMFIS, oldMFIS);
    plotObjectsOf4Kinds(mfisFileName,project_info, currentMFIS, oldMFIS, references);
    waitHere();
    return project_info;
}

//match current view with projected MFIS
//return position on MFIS without updating MFIS
vector<Object> match_view_with_MFIS(vector<Object> cv, vector<Object> fixMFIS, vector<Object> rb,  
                                    vector<Object>& currentMFIS, int v)
{
        char plotFile[80];
        MyRobot myrobot;
        Object temp_match;
        vector<Object> shift_position;
        vector<Object> matched_info;
        vector<Object> reference_obj;
        vector< vector<Object> > match_list;
        //the first matching process
        //tracking information

        for(int i = 0; i < cv.size(); i++)
        {     
                temp_match = Compare_similar_near_Obeject_(fixMFIS, cv[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(cv[i]);

                    match_list.push_back(matched_info);
                    matched_info.clear();
                }
        }

        //cout << " size of matched info : " << match_list.size() << endl;
        //waitHere();

        if(match_list.size() > 0)
        {
            
            //choose a proper one as reference
            for(int j = 0; j < match_list.size(); j++)
            {
                //same size is the 
                if(abs(match_list[j][0].length()/match_list[j][1].length()) < 1.200)
                {
                    reference_obj = match_list[j];
                    break;
                }
            }
            
            if(reference_obj.size() == 0)
            {
                double dist, min;
                
                for(int j = 0; j < match_list.size(); j++)
                {
                    dist = reference_endpoint_neardist(match_list[j][0],match_list[j][1]);
                    if(j == 0)
                    {
                        min = dist;
                        reference_obj = match_list[j];
                    }
                    else
                    {
                        
                        if(dist < min)
                        {
                            min = dist;
                            reference_obj = match_list[j];
                        }
                    }
                    
                }
            }
            
            
            cout << " size of matched info : " << matched_info.size() << endl;
            //triangulate/shift robot position on MFIS based on global information rather than current
            shift_position = projectingTheView(rb, reference_obj[0], reference_obj[1], 1);
            
            vector<Object> adjust_view;
            adjust_view = projectingTheView(cv, reference_obj[0], reference_obj[1], 1);

            sprintf(plotFile, "%s%d%s", "Maps/Offline/Expect_and_position-", v, ".png");
            //plotObjectsOf3Kinds(plotFile, cv, shift_position, MFIS);
            plotObjectsOf4Kinds(plotFile, reference_obj, adjust_view, shift_position, fixMFIS);
            //plotObjectsColours(plotFile, match_list, shift_position, MFIS);
        }
        else
        {
            //reasoning part to recognize reference surfaces
            shift_position = rb;
            sprintf(plotFile, "%s%d%s", "Maps/Offline/Expect_and_position-", v, ".png");
            plotObjectsOf3Kinds(plotFile, cv, rb, fixMFIS);

        }
        
        
        //rest of surfaces, except matched surfaces
        vector<Object> temp;
        vector<Object> to_update_info;
        temp = cv;
        for(int i = 0; i < match_list.size(); i++)
        {
            for(int j = 0; j < temp.size(); j++)
            {
                if((temp[j].getP1() == match_list[i][1].getP1())
                    && (temp[j].getP2() == match_list[i][1].getP2()))
                {
                    temp.erase(temp.begin() + j);
                    continue;
                }
            }
        }

        to_update_info = temp; //rest of
        
        //adjustment of memory relating to what I see recently
        unsigned char shift_mod_flag = 0;
        Object line_join_position_and_obj;
        
        
        /* //add new info & re-activate normal process
        unsigned char matched_flag = 0;
        unsigned char update_flag = 0;

        if(match_list.size() > 0)
        {


            for(int i = 0; i < to_update_info.size(); i++)
            {
                if(to_update_info[i].length() > 1000)
                    update_flag = 1;
            }
            
            if(update_flag == 1)   
            {
                //switch back to normal process
                expect_Flag = 0;
                //meanwhile, udpate global map/MFIS
                currentMFIS = addTwoVectorsOfObjects(fixMFIS, to_update_info);
            }
            else
            {
                //proportion of matching info
                if((match_list.size()/cv.size()) < 0.2 && cv.size() > 5)
                {
                    //switch back to normal process
                    expect_Flag = 0;
                    //meanwhile, udpate global map/MFIS
                    currentMFIS = addTwoVectorsOfObjects(fixMFIS, to_update_info);
                }
            }
        }*/  

    
    
    return shift_position;
}

pair< vector<Object>,  vector<Object> > info_adjustment(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> expect_information, int v)
{
        char plotFile[80];
        MyRobot myrobot;
        int match_flag = 0;
        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> remain_info;
        vector<Object> global_map;
        vector< vector<Object> > match_list;
        pair< vector<Object>,  vector<Object> > temp; //for return 
        pair<int, int> ref_id;
        //the first matching process
        //tracking information

        for(int i = 0; i < cv.size(); i++)
        {     

                temp_match = Compare_similar_near_Obeject_(MFIS, cv[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(cv[i]);

                    //MFIS[temp_match.getID()].set_match_flag(true);
                    
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }

        } 
 
        if(match_list.size() > 0)
        {
            
            //choose a proper one as reference
            for(int j = 0; j < match_list.size(); j++)
            {
                //same size is the 
                //if(abs(match_list[j][0].length()/match_list[j][1].length()) < 1.200)
                if(abs(match_list[j][0].length() - match_list[j][1].length()) < 300.00)
                {
                    reference_obj = match_list[j];
                    //triangulate/shift robot position on MFIS based on global information rather than current
                    shift_position = projectingTheView(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView(cv, reference_obj[0], reference_obj[1], 2);
                    
                    for(int v = 0; v < adjust_view.size(); v++)
                    {
                        adjust_view[v].set_match_flag(cv[v].get_match_flag());
                        adjust_view[v].set_match_label(cv[v].get_match_label());
                    }
                    //ref_id.first = reference_obj[0].getID();
                    //ref_id.second = reference_obj[1].getID();
                    
                    match_flag = 1;
                    break;
                }
            }
            /*
            if(reference_obj.size() == 0)
            {
                double dist, min;
                
                for(int j = 0; j < match_list.size(); j++)
                {
                    dist = reference_endpoint_neardist(match_list[j][0],match_list[j][1]);
                    if(j == 0)
                    {
                        min = dist;
                        reference_obj = match_list[j];
                    }
                    else
                    {
                        
                        if(dist < min)
                        {
                            min = dist;
                            reference_obj = match_list[j];
                        }
                    }
                    
                }
            }
            
            
            cout << " size of matched info : " << matched_info.size() << endl;
            //triangulate/shift robot position on MFIS based on global information rather than current
            shift_position = projectingTheView(rb, reference_obj[0], reference_obj[1], 1);
            
            adjust_view = projectingTheView(cv, reference_obj[0], reference_obj[1], 1);*/
            

                if(match_flag == 1 && cv.size() > 2)
                {
                    
                    if(expect_process_flag == 0)//first time to activate expectation process
                    {
                        /*vector<Object> remain_info;
                    vector<Object> corresp_info;

                    for(int m = 0; m < MFIS.size(); m++)
                    {
                        if((MFIS[m].get_match_flag() != 1)
                            || (MFIS[m].get_match_label() <= 1))
                        {
                            remain_info.push_back(MFIS[m]);
                        }
                        else 
                        {
                            corresp_info.push_back(MFIS[m]);
                        }
                    }
                    for(int m = 0; m < cv.size(); m++)
                    {
                        if(cv[m].get_match_label() > 1)
                            adjust_view[m].set_match_label(cv[m].get_match_label());
                    }

                    for(int m = 0; m < corresp_info.size(); m++)
                    {
                        for(int n = 0; n < adjust_view.size(); n++)
                        {
                            if(adjust_view[n].get_match_label() == corresp_info[m].get_match_label())
                            {
                                if((abs(adjust_view[n].length() / corresp_info[m].length()) >= 1.10)
                                    || (abs(adjust_view[n].length() / corresp_info[m].length()) <= 0.85))
                                {
                                    //modified 
                                    int ref_flag = reference_endpoint(cv[n], corresp_info[m]);
                                    Object modified_surf;
                                    if(ref_flag == 2)
                                    {
                                        modified_surf = expend_Object(adjust_view[n], corresp_info[m].length()-adjust_view[n].length(), 1);
                                        modified_surf.set(modified_surf.X2(), modified_surf.Y2(), adjust_view[n].X2(),adjust_view[n].Y2(), adjust_view[n].getID());
                                    }
                                    else
                                    {
                                        modified_surf = expend_Object(adjust_view[n], corresp_info[m].length()-adjust_view[n].length(), 2);
                                        modified_surf.set(adjust_view[n].X1(), adjust_view[n].Y1(), modified_surf.X2(), modified_surf.Y2(), adjust_view[n].getID());   
                                    }

                                    adjust_view.erase(adjust_view.begin()+n);
                                    adjust_view.insert(adjust_view.begin()+n, modified_surf);

                                    //if intersect with others & inter-point is not endpoint
                                    //shift it

                                }
                            }
                        }
                    }*/


                    //remain_info = Fill_operation(adjust_view, cv, MFIS, reference_obj); //fill in as connected neighbour 
                        remain_info = Fill_operation_with_ASR(adjust_view, cv, MFIS, match_list, reference_obj);  
                        remain_info = Correct_detail(adjust_view, remain_info);
                        global_map = remain_info;
                        
                        sprintf(plotFile, "%s%d%s", "Maps/Offline/Expectation_info_and_position-", v, ".png");
                        plotObjectsOf3Kinds(plotFile, shift_position, adjust_view, remain_info);
                        
                        expect_process_flag = 1;
                    }
                    else
                    {
                        for(int i = 0; i < adjust_view.size(); i++)
                        {     
                            //match between adjust view and expected global map
                            Compare_similar_near_Obeject_(expect_information, adjust_view[i], v);
                        }
                        
                        int correct_flag = 0;
                        int block_flag = 0;
                        vector<Object> piece_adjusted;
                        for(int i = 0; i < expect_information.size(); i++)
                        {
                            if((expect_information[i].get_match_label() > 1)
                                || (expect_information[i].get_imagined_flag() == 2))
                            {
                                correct_flag = 1;
                            }
                            
                            for(int n = 0; n < adjust_view.size(); n++)
                            {
                                    if((TwoObjectIntersect(adjust_view[n], expect_information[i]) == true)
                                        && (expect_information[i].get_imagined_flag() != 1)
                                        && (shortestDistanceBtwTwoObjects(adjust_view[n], expect_information[i]) < 500))
                                    block_flag = 1;

                            }

                        }
                        
                        
                        //using has been expected map
                        if((correct_flag == 1) && (block_flag == 1))
                        {
                            piece_adjusted = Correct_detail(adjust_view, expect_information);
                            //piece_adjusted = Correct_detail_blocked(adjust_view, piece_adjusted);
                            global_map = piece_adjusted;
                        }
                        else
                        {
                            if(correct_flag == 1)
                            {
                                piece_adjusted = Correct_detail(adjust_view, expect_information);
                                global_map = piece_adjusted;
                            }
                            else
                            {
                                //if(block_flag == 1)
                                //{
                                //    piece_adjusted = Correct_detail_blocked(adjust_view, piece_adjusted);
                                //    global_map = piece_adjusted;
                                //}
                                //else  
                                    global_map = expect_information;
                            }
                        }
                        
                        sprintf(plotFile, "%s%d%s", "Maps/Offline/Exist_expectation-", v, ".png");
                        plotObjects(plotFile, adjust_view, global_map);
                        
                        //global_map = piece_adjusted;
                    }
                }
                else
                {
                    adjust_view = cv;
                    shift_position = rb;
                    global_map = expect_information;
                }
            

        }
        else
        {
            adjust_view = cv;
            shift_position = rb;
            global_map = expect_information;
        }

        
        
        temp.first = global_map;
        temp.second = shift_position;
        
        return temp;
}

vector< vector<Object> > info_adjustment_next(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> rest_info, int v)
{
        char plotFile[80];
        MyRobot myrobot;
        
        
        int correct_flag = 0;
        int block_flag = 0;
        int match_flag = 0;
        int place_cross_flag = 0;
        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> remain_info;
        vector<Object> global_map;
        vector<Object> match_reference_view;
        vector< vector<Object> > match_list;
        vector< vector<Object> > temp_rnt;   //return adjust view, robot position & adjust gloal map
        pair< vector<Object>,  vector<Object> > temp; //for return 
        pair<int, int> ref_id;
        
        
        
        //the first matching process
        //tracking information
        
        for(int i = 0; i < cv.size(); i++)
        {     

                temp_match = Compare_similar_near_Obeject_(MFIS, cv[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(cv[i]);

                    //MFIS[temp_match.getID()].set_match_flag(true);
                    
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }

        }


back:   if(match_list.size() > 0)
        {
                double max = 0;
                //choose a proper one as reference
                for(int j = 0; j < match_list.size(); j++)
                {
                    /*if(v >= 41)
                    {
                        cout << " current      v      : " << v << endl;
                        cout << " different of length : " << abs(match_list[j][0].length() - match_list[j][1].length()) << endl;
                        cout << " length of itself    : " << match_list[j][1].length() << endl;
                        waitHere();
                    }*/
                    
                    
                    //same size is the 
                    if((abs(match_list[j][0].length() - match_list[j][1].length()) <= 650.00)
                        && (match_list[j][1].length() >= 700))
                    {
                        if(j == 0)
                        {
                            max = match_list[j][1].length();
                            reference_obj = match_list[j];
                        }
                        else
                        {
                            if(max < match_list[j][1].length())
                            {
                                max = match_list[j][1].length();
                                reference_obj = match_list[j];
                            }
                        }
                        
                        //triangulate/shift robot position on MFIS based on global information rather than current
                        //shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                        //adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
         
                        //match_flag = 1;

                    }
                }
                
                if(max > 0)
                {
                    //triangulate/shift robot position on MFIS based on global information rather than current
                    shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
         
                    match_flag = 1;
                }
                else
                    match_flag = 0;
                
                cout << " current v  : " << v << endl;
                cout << " match flag : " << match_flag << endl;
                cout << " match list : " << match_list.size() << endl;
                waitHere();
                
                if(match_flag == 1)
                {
                    
                        for(int i = 0; i < MFIS.size(); i++)
                        {
                            if((MFIS[i].get_match_label() > 1)
                                || (MFIS[i].get_imagined_flag() == 2))
                            {
                                correct_flag = 1;
                            }
                            
                            for(int n = 0; n < adjust_view.size(); n++)
                            {
                                Object connect_line;
                                connect_line.set(shift_position[6].X1(), shift_position[6].Y1(), adjust_view[n].midpoint().X(), adjust_view[n].midpoint().Y(), 0);
                                    if((TwoObjectIntersect(connect_line, MFIS[i]) == true)
                                        && (MFIS[i].get_imagined_flag() != 1)
                                        && (shortestDistanceBtwTwoObjects(adjust_view[n], MFIS[i]) < 800))
                                    block_flag = 1;

                            }

                        }
                        
                        //using has been expected map
                        if((correct_flag == 1) && (block_flag == 1))
                        {
                            global_map = Correct_detail(adjust_view, MFIS);
                            global_map = Correct_detail_blocked(adjust_view, global_map, shift_position);
                        }
                        else
                        {
                            if(correct_flag == 1)
                            {
                                global_map = Correct_detail(adjust_view, MFIS);
                            }
                            else
                            {
                                if(block_flag == 1)
                                {
                                    global_map = Correct_detail_blocked(adjust_view, MFIS, shift_position);
                                }
                                else  
                                    global_map = MFIS;
                            }
                        }       
                }
                else
                {
                    //adjust_view = cv;
                    //shift_position = rb;
                    //global_map = MFIS;
                    //
                    match_reference_view = MFIS;
                    goto lb;
                }
            

        }
        else
        {
            
lb:         cout << " place_cross_flag : " << place_cross_flag << endl;
            //waitHere();
            if(place_cross_flag == 1)
            {
                match_reference_view.clear();
                match_reference_view = rest_info;
                //info_adjustment_next(cv, rb, match_reference_view, rest_info, iv);
            }
    
     
            for(int i = 0; i < cv.size(); i++)
            {
                if(cv[i].length() >= 500)
                {
                    for(int t = 0; t < match_reference_view.size(); t++)
                    {

                        vector<Object> test1, test2;
                        test1.push_back(cv[i]);
                        test2.push_back(match_reference_view[t]);

                        /*if(v >= 41)
                        {
                        cout << "      leng 1     : " << cv[i].length() << endl;
                        cout << "      leng 2     : " << match_reference_view[t].length() << endl;
                        cout << "      leng dif   : " << abs(cv[i].length() / match_reference_view[t].length()) << endl;
                        sprintf(plotFile, "%s%d%s", "Maps/test_pair_information-", v, ".png");
                        plotObjects(plotFile,test1,test2);
                        waitHere();
                        }*/
                        if((((abs(cv[i].length() / match_reference_view[t].length()) > 0.60) && (abs(cv[i].length() / match_reference_view[t].length()) <= 1.00))
                            || ((abs(cv[i].length() / match_reference_view[t].length()) >= 1.00) && (abs(cv[i].length() / match_reference_view[t].length()) < 1.38)))
                            && (cv[i].length() > 700))
                        {

                            Object obj1, obj2;
                            obj1.set(rb[6].X1(), rb[6].Y1(), cv[i].midpoint().X(), cv[i].midpoint().Y(), 1);
                            obj2.set(rb[6].X1(), rb[6].Y1(), match_reference_view[t].midpoint().X(), match_reference_view[t].midpoint().Y(), 1);
                            if(((TwoObjectIntersect(obj1, match_reference_view[t]) == true) && (distanceOftwoP(cv[i].midpoint(), intersectPointTwolineEquations(obj1, match_reference_view[t])) < 800))
                                    || ((TwoObjectIntersect(obj2, cv[i]) == true) && (distanceOftwoP(match_reference_view[t].midpoint(), intersectPointTwolineEquations(obj2, cv[i])) < 800)))
                            {
                                /*reference_obj.push_back(MFIS[t]);
                                reference_obj.push_back(cv[i]);

                                shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                                adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
                                match_flag = 1;*/

                               
                                if(place_cross_flag == 1)
                                    match_list.clear();

                                matched_info.push_back(match_reference_view[t]);
                                matched_info.push_back(cv[i]);

                                //MFIS[temp_match.getID()].set_match_flag(true);

                                match_list.push_back(matched_info);
                                matched_info.clear();
                                place_cross_flag = 1;
                                goto back;
                            }
                        }
                    }
                }
            }
            
            if(place_cross_flag == 1)
            {
                adjust_view = cv;
                shift_position = rb;
                global_map = MFIS;
            }
            else
            {
                cout << " Trace back to match rest of part. " << endl;
                place_cross_flag = 1;
                goto lb;
            }
            
        }
        
        //return 2 things
        //temp.first = global_map;
        //temp.second = shift_position;
        
        //return 3 things
        temp_rnt.push_back(global_map);
        temp_rnt.push_back(shift_position);
        temp_rnt.push_back(adjust_view);
        
        //return temp;
        return temp_rnt;
}

vector< vector<Object> > info_adjustment_next_version(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> rest_info, int v, int& flag)
{
        char plotFile[80];
        MyRobot myrobot;
        
        
        int correct_flag = 0;
        int block_flag = 0;
        int match_flag = 0;
        int place_cross_flag = 0;
        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> remain_info;
        vector<Object> global_map;
        vector<Object> match_reference_view;
        vector< vector<Object> > match_list;
        vector< vector<Object> > temp_rnt;   //return adjust view, robot position & adjust gloal map
        pair< vector<Object>,  vector<Object> > temp; //for return 
        pair<int, int> ref_id;
        
        
if(flag == 0)
{
        //the first matching process
        //tracking information
        
        for(int i = 0; i < cv.size(); i++)
        {     

                temp_match = Compare_similar_near_Obeject_(MFIS, cv[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(cv[i]);

                    //MFIS[temp_match.getID()].set_match_flag(true);
                    
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }

        }


back:   if(match_list.size() > 0)
        {
                double max = 0;
                     
                //choose a proper one as reference
                for(int j = 0; j < match_list.size(); j++)
                {
                    if(v >= 39)
                    {
                        cout << " current      v      : " << v << endl;
                        cout << " different of length : " << abs(match_list[j][0].length() - match_list[j][1].length()) << endl;
                        cout << " length of itself    : " << match_list[j][1].length() << endl;
                        waitHere();
                    }
                    
                    
                    //same size is the 
                    if((abs(match_list[j][0].length() - match_list[j][1].length()) <= 650.00)
                        && (match_list[j][1].length() >= 700))
                    {
                        if(j == 0) 
                        {
                            max = match_list[j][1].length();
                            reference_obj = match_list[j];
                        }
                        else
                        {
                            if(max < match_list[j][1].length())
                            {
                                max = match_list[j][1].length();
                                reference_obj = match_list[j];
                            }
                        }
                        
                        //triangulate/shift robot position on MFIS based on global information rather than current
                        //shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                        //adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
         
                        //match_flag = 1;

                    }
                }
                
                
                if(max > 0)
                {
                    //triangulate/shift robot position on MFIS based on global information rather than current
                    shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
         
                    
                    //the adjustment reference is not reliable, which should be corrected
                    if(distanceOftwoP(shift_position[6].getP1(), rb[6].getP1()) > 500)
                    {
                        int end_flag = 0;
                        
                        if(rb[6].Y2() > rb[6].Y1())
                        {
                            if(((rb[6].Y2() - rb[6].Y1()) / (rb[6].X2() - rb[6].X1())) > 0)
                                    end_flag = 2;
                            else
                                if(((rb[6].Y2() - rb[6].Y1()) / (rb[6].X2() - rb[6].X1())) < 0)
                                    end_flag = 1;
                        }
                        else
                            end_flag = 2;
                        
                        shift_position = projectingTheView_with_correctDirection(rb, match_list[0][0], match_list[0][1], end_flag);
                        adjust_view = projectingTheView_with_correctDirection(cv, match_list[0][0], match_list[0][1], end_flag);
                        //modified that one in MFIS
                        if(place_cross_flag == 0)
                        {
                            Object insert;
                            for(int n = 0; n < MFIS.size(); n++)
                            {
                                if((MFIS[n].getP1() == reference_obj[0].getP1())
                                    && (MFIS[n].getP2() == reference_obj[0].getP2()))
                                {
                                    
                                    //insert.set(MFIS[n-2].X2(), MFIS[n-2].Y2(), MFIS[n+1].X2(), MFIS[n+1].Y2(), n);
                                    MFIS[n].set(MFIS[n-2].X2(), MFIS[n-2].Y2(), MFIS[n+1].X2(), MFIS[n+1].Y2(), n);
                                    MFIS.erase(MFIS.begin()+n-1);
                                    break;
                                }
                            }
                            
                            
                        }
                    }
                    
                    
                    match_flag = 1;
                }
                else
                    match_flag = 0;
                
                if(v >= 39)
                {
                    cout << " current v : " << v << endl;
                    cout << "    max    : " << max << endl;
                    cout << " match_flag: " << match_flag << endl;
                    waitHere();
                    
                }
                
                if(match_flag == 1)
                {
                    
                        for(int i = 0; i < MFIS.size(); i++)
                        {
                            if((MFIS[i].get_match_label() > 1)
                                || (MFIS[i].get_imagined_flag() == 2))
                            {
                                correct_flag = 1;
                            }
                            
                            for(int n = 0; n < adjust_view.size(); n++)
                            {
                                Object connect_line;
                                connect_line.set(shift_position[6].X1(), shift_position[6].Y1(), adjust_view[n].midpoint().X(), adjust_view[n].midpoint().Y(), 0);
                                    if((TwoObjectIntersect(connect_line, MFIS[i]) == true)
                                        && (MFIS[i].get_imagined_flag() != 1)
                                        && (shortestDistanceBtwTwoObjects(adjust_view[n], MFIS[i]) < 800))
                                    block_flag = 1;

                            }

                        }
                        
                        //using has been expected map
                        if((correct_flag == 1) && (block_flag == 1))
                        {
                            global_map = Correct_detail(adjust_view, MFIS);
                            global_map = Correct_detail_blocked(adjust_view, global_map, shift_position);
                        }
                        else
                        {
                            if(correct_flag == 1)
                            {
                                global_map = Correct_detail(adjust_view, MFIS);
                            }
                            else
                            {
                                if(block_flag == 1)
                                {
                                    global_map = Correct_detail_blocked(adjust_view, MFIS, shift_position);
                                }
                                else  
                                    global_map = MFIS;
                            }
                        }       
                }
                else
                {
                    
                    if(place_cross_flag == 0)
                    {
                        match_reference_view = MFIS;
                        goto lb;
                    }
                    else
                    {
                        adjust_view = cv;
                        shift_position = rb;
                        global_map = MFIS;
                        
                        vector<Object> theory_view = view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                        vector< vector<Object> > temp_adjust = shift_after_expect(theory_view, cv, rb, v);
                        adjust_view = temp_adjust[1];
                        shift_position = temp_adjust[0];
                    }
                }
            

        }
        else
        {
            
lb:         cout << " place_cross_flag : " << place_cross_flag << endl;
            //waitHere();
            if(place_cross_flag == 1)
            {
                match_reference_view.clear();
                match_reference_view = rest_info;
                //info_adjustment_next(cv, rb, match_reference_view, rest_info, iv);
            }
    
     
            for(int i = 0; i < cv.size(); i++)
            {
                if(cv[i].length() >= 500)
                {
                    for(int t = 0; t < match_reference_view.size(); t++)
                    {

                        vector<Object> test1, test2;
                        test1.push_back(cv[i]);
                        test2.push_back(match_reference_view[t]);

                        if((((abs(cv[i].length() / match_reference_view[t].length()) > 0.60) && (abs(cv[i].length() / match_reference_view[t].length()) <= 1.00))
                            || ((abs(cv[i].length() / match_reference_view[t].length()) >= 1.00) && (abs(cv[i].length() / match_reference_view[t].length()) < 1.38)))
                            && (cv[i].length() > 700))
                        {

                            Object obj1, obj2;
                            obj1.set(rb[6].X1(), rb[6].Y1(), cv[i].midpoint().X(), cv[i].midpoint().Y(), 1);
                            obj2.set(rb[6].X1(), rb[6].Y1(), match_reference_view[t].midpoint().X(), match_reference_view[t].midpoint().Y(), 1);
                            //obj1.set(rb[6].X1(), rb[6].Y1(), cv[i].X2(), cv[i].Y2(), 1);
                            //obj2.set(rb[6].X1(), rb[6].Y1(), match_reference_view[t].X2(), match_reference_view[t].Y2(), 1);
                            
                            if(((TwoObjectIntersect(obj1, match_reference_view[t]) == true) && (distanceOftwoP(cv[i].midpoint(), intersectPointTwolineEquations(obj1, match_reference_view[t])) < 800))
                                    || ((TwoObjectIntersect(obj2, cv[i]) == true) && (distanceOftwoP(match_reference_view[t].midpoint(), intersectPointTwolineEquations(obj2, cv[i])) < 800)))
                            {
                                /*reference_obj.push_back(MFIS[t]);
                                reference_obj.push_back(cv[i]);

                                shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                                adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
                                match_flag = 1;*/

                               
                                if(place_cross_flag == 1)
                                    match_list.clear();

                                matched_info.push_back(match_reference_view[t]);
                                matched_info.push_back(cv[i]);

                                //MFIS[temp_match.getID()].set_match_flag(true);

                                match_list.push_back(matched_info);
                                matched_info.clear();
                                place_cross_flag = 1;
                                //goto back;
                            }
                        }
                    }
                }
            }

            if(match_list.size() > 0)
                goto back;

            if(place_cross_flag == 1)
            {
                adjust_view = cv;
                shift_position = rb;
                global_map = MFIS;
                
                vector<Object> theory_view = view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                vector< vector<Object> > temp_adjust = shift_after_expect(theory_view, cv, rb, v);
                adjust_view = temp_adjust[1];
                shift_position = temp_adjust[0];
            }
            else
            {
                cout << " Trace back to match rest of part. " << endl;
                place_cross_flag = 1;
                goto lb;
            }
            
        }

        if(place_cross_flag == 1)
        {
            //global_map = addTwoVectorsOfObjects(global_map, rest_info);
            flag = 1;
        }
        
        //return 2 things
        //temp.first = global_map;
        //temp.second = shift_position;
        
        if(distanceOftwoP(shift_position[6].getP1(), rb[6].getP1()) > 1000)
            shift_position = rb;

        //return 3 things
        temp_rnt.push_back(global_map);
        temp_rnt.push_back(shift_position);
        temp_rnt.push_back(adjust_view);
        
        //return temp;
        return temp_rnt;
}
else
{
        for(int i = 0; i < cv.size(); i++)
        {     

                temp_match = Compare_similar_near_Obeject_(rest_info, cv[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(cv[i]);

                    //MFIS[temp_match.getID()].set_match_flag(true);
                    
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }

        }


back2:  if(match_list.size() > 0)
        {
                double max = 0;
                //choose a proper one as reference
                for(int j = 0; j < match_list.size(); j++)
                {
                    
                    //same size is the 
                    if((abs(match_list[j][0].length() - match_list[j][1].length()) <= 650.00)
                        && (match_list[j][1].length() >= 700))
                    {
                        if(j == 0)
                        {
                            max = match_list[j][1].length();
                            reference_obj = match_list[j];
                        }
                        else
                        {
                            if(max < match_list[j][1].length())
                            {
                                max = match_list[j][1].length();
                                reference_obj = match_list[j];
                            }
                        }
                        
                    }
                }
                
                if(max > 0)
                {
                    //triangulate/shift robot position on MFIS based on global information rather than current
                    shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
         
                    match_flag = 1;
                }
                else
                    match_flag = 0;
                
                if(match_flag == 1)
                {
                    
                        for(int i = 0; i < rest_info.size(); i++)
                        {
                            if((rest_info[i].get_match_label() > 1)
                                || (rest_info [i].get_imagined_flag() == 2))
                            {
                                correct_flag = 1;
                            }
                            
                            for(int n = 0; n < adjust_view.size(); n++)
                            {
                                Object connect_line;
                                connect_line.set(shift_position[6].X1(), shift_position[6].Y1(), adjust_view[n].midpoint().X(), adjust_view[n].midpoint().Y(), 0);
                                    if((TwoObjectIntersect(connect_line, rest_info[i]) == true)
                                        && (rest_info[i].get_imagined_flag() != 1)
                                        && (shortestDistanceBtwTwoObjects(adjust_view[n], rest_info[i]) < 800))
                                    block_flag = 1;

                            }

                        }
                        
                        /*//using has been expected map
                        if((correct_flag == 1) && (block_flag == 1))
                        {
                            global_map = Correct_detail(adjust_view, rest_info);
                            global_map = Correct_detail_blocked(adjust_view, global_map, shift_position);
                        }
                        else
                        {
                            if(correct_flag == 1)
                            {
                                global_map = Correct_detail(adjust_view, rest_info);
                            }
                            else
                            {
                                if(block_flag == 1)
                                {
                                    global_map = Correct_detail_blocked(adjust_view, rest_info, shift_position);
                                }
                                else  
                                    global_map = rest_info;
                            }
                        }*/       
                }
                else
                {
                    if(place_cross_flag == 0)
                    {
                        match_reference_view = rest_info;
                        goto lb2;
                    }
                    else
                    {
                        if(place_cross_flag == 1)
                        {
                            for(int j = 0; j < match_list.size(); j++)
                            {

                                //same size is the 
                                if(((distanceOftwoP(match_list[j][0].getP1(), match_list[j][1].getP1()) < 600.00) 
                                        || (distanceOftwoP(match_list[j][0].getP2(), match_list[j][1].getP2()) < 600.00))
                                    && (match_list[j][1].length() >= 700))
                                {
                                    if(j == 0)
                                    {
                                        max = match_list[j][1].length();
                                        reference_obj = match_list[j];
                                    }
                                    else
                                    {
                                        if(max < match_list[j][1].length())
                                        {
                                            max = match_list[j][1].length();
                                            reference_obj = match_list[j];
                                        }
                                    }

                                }
                            }
                            
                            if(max > 0)
                            {
                                if(distanceOftwoP(reference_obj[0].getP1(), reference_obj[1].getP1()) < 600.00) 
                                {
                                    //triangulate/shift robot position on MFIS based on global information rather than current
                                    shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 1);
                                    adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 1);
                                }
                                else                                    
                                {
                                    if(distanceOftwoP(reference_obj[0].getP2(), reference_obj[1].getP2()) < 600.00) 
                                    {
                                        //triangulate/shift robot position on MFIS based on global information rather than current
                                        shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                                        adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
                                    }
                                }
                            }
                        }
                    }
                }
            

        }
        else
        {
            
lb2:        cout << " place_cross_flag : " << place_cross_flag << endl;
            //waitHere();
            if(place_cross_flag == 1)
            {
                match_reference_view.clear();
                match_reference_view = MFIS;
                //info_adjustment_next(cv, rb, match_reference_view, rest_info, iv);
            }
    
     
            for(int i = 0; i < cv.size(); i++)
            {
                if(cv[i].length() >= 500)
                {
                    for(int t = 0; t < match_reference_view.size(); t++)
                    {
                        if((((abs(cv[i].length() / match_reference_view[t].length()) > 0.60) && (abs(cv[i].length() / match_reference_view[t].length()) <= 1.00))
                            || ((abs(cv[i].length() / match_reference_view[t].length()) >= 1.00) && (abs(cv[i].length() / match_reference_view[t].length()) < 1.38)))
                            && (cv[i].length() > 700))
                        {

                            Object obj1, obj2;
                            obj1.set(rb[6].X1(), rb[6].Y1(), cv[i].midpoint().X(), cv[i].midpoint().Y(), 1);
                            obj2.set(rb[6].X1(), rb[6].Y1(), match_reference_view[t].midpoint().X(), match_reference_view[t].midpoint().Y(), 1);
                            if(((TwoObjectIntersect(obj1, match_reference_view[t]) == true) && (distanceOftwoP(cv[i].midpoint(), intersectPointTwolineEquations(obj1, match_reference_view[t])) < 800))
                                    || ((TwoObjectIntersect(obj2, cv[i]) == true) && (distanceOftwoP(match_reference_view[t].midpoint(), intersectPointTwolineEquations(obj2, cv[i])) < 800)))
                            {
                                /*reference_obj.push_back(MFIS[t]);
                                reference_obj.push_back(cv[i]);

                                shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                                adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
                                match_flag = 1;*/

                               
                                if(place_cross_flag == 1)
                                    match_list.clear();

                                matched_info.push_back(match_reference_view[t]);
                                matched_info.push_back(cv[i]);

                                //MFIS[temp_match.getID()].set_match_flag(true);

                                match_list.push_back(matched_info);
                                matched_info.clear();
                                place_cross_flag = 1;
                                goto back2;
                            }
                        }
                    }
                }
            }
            
            if(place_cross_flag == 1)
            {
                adjust_view = cv;
                shift_position = rb;
                global_map = MFIS;
                //view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                vector<Object> theory_view = view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                vector< vector<Object> > temp_adjust = shift_after_expect(theory_view, cv, rb, v);
                adjust_view = temp_adjust[1];
                shift_position = temp_adjust[0];
            }
            else
            {
                cout << " Trace back to match rest of part. " << endl;
                place_cross_flag = 1;
                goto lb;
            }
            
        }

        if(place_cross_flag == 1)
        {
            //global_map = addTwoVectorsOfObjects(global_map, MFIS);
            flag = 0;
        }
        
        if(distanceOftwoP(shift_position[6].getP1(), rb[6].getP1()) > 1000)
            shift_position = rb;

        //return 3 things
        temp_rnt.push_back(MFIS);
        temp_rnt.push_back(shift_position);
        temp_rnt.push_back(adjust_view);
        
        //return temp;
        return temp_rnt;
}
        
        
}

vector<Object> Fill_operation(vector<Object> view, vector<Object> cv, vector<Object> global_map, vector<Object> refences)
{
        char plotFile[80];
        int lab_num, cnt = 1;;
        int fill_cnt, fill_temp_cnt; 
        int fill_mode = 0; //1--anti, 2 clockwise, 3 both
        int fill_finish_flag = 0; //finish anti-clockwise first, then clockwise
        
        double angle1 = 0;
        Object fill_obj;
        Object ref_obj;
        Object detect_obj;
        vector<Object> fill_info;
        vector<Object> remaining;
        vector<int> connect_to_ref;
                
        vector<int> filled_id_num; 
        pair<int, int> ref_id;
        ref_id.first  =   refences[0].getID();     
        ref_id.second =   refences[1].getID();    
        
lb:     detect_obj = refences[0];
        
        //clockwise/anti-clockwise/both mode
        for(int i = 0; i < global_map.size(); i++)
        {
            if((twoLinintersect(global_map[i].getP1(), global_map[i].getP2(), refences[0].getP1(), refences[0].getP2()) == true)
                && (common_endpoint(refences[0], global_map[i]) == 1))
                        connect_to_ref.push_back(1);
            
            if((twoLinintersect(global_map[i].getP1(), global_map[i].getP2(), refences[0].getP1(), refences[0].getP2()) == true)
                && (common_endpoint(refences[0], global_map[i]) == 2))
                        connect_to_ref.push_back(2);
        }
        
        if(connect_to_ref.size() == 2)
            fill_mode = 3; //both sides mode
        else
        {
            if(connect_to_ref.size() == 1)
            {
                if(connect_to_ref[0] == 1)
                    fill_mode = 1; //anti-clockwise
                else
                    fill_mode = 2; //clockwise   
            }
        }
            
        ///
        for(int i = 0; i < view.size(); i++)
        {
            if(abs(view[i].length() - refences[1].length()) < 10e-6)
            {
                ref_obj = fill_obj = view[i];
                break;
            }
        }
        fill_info.push_back(fill_obj); //key reference object
        filled_id_num.push_back(refences[0].getID());
        fill_cnt= 1;
        
        //clockwise filling in
        for(int m = 0; m < global_map.size(); m++)
        {
                if((twoLinintersect(global_map[m].getP1(), global_map[m].getP2(), detect_obj.getP1(), detect_obj.getP2()) == true)
                    //&& ((intersectPointTwolineEquations(global_map[m], detect_obj) == detect_obj.getP1()) || (intersectPointTwolineEquations(global_map[m], detect_obj) == detect_obj.getP2()) )
                    && ((global_map[m].get_match_flag() == 1) && (global_map[m].get_match_label() > 1)))
                {
                    int filled_flag = 0;
                    for(int j = 0; j < filled_id_num.size(); j++)
                    {
                        if(filled_id_num[j] == m)
                        {
                            filled_flag = 1;
                            break;
                        }
                    }

                    if(filled_flag != 1)
                    {
                        
                        /*//modify it first based upon current 
                        int exp_label = 0;
                        Object exp_ref;
                        for(int k = 0; k < cv.size(); k++)
                        {
                            if(cv[k].get_match_label() == global_map[m].get_match_label())
                            {
                                exp_ref = cv[k];
                                exp_label = k;
                            }

                        }


                        int ref_flag = reference_endpoint(exp_ref, global_map[m]);
                        Object modified_surf;
                        if(ref_flag == 2)
                        {
                            modified_surf = expend_Object(view[exp_label], global_map[m].length()-view[exp_label].length(), 1);
                            modified_surf.set(modified_surf.X2(), modified_surf.Y2(), view[exp_label].X2(),view[exp_label].Y2(), view[exp_label].getID());
                        }
                        else
                        {
                            modified_surf = expend_Object(view[exp_label], global_map[m].length() - global_map[m].length(), 2);
                            modified_surf.set(view[exp_label].X1(), view[exp_label].Y1(), modified_surf.X2(), modified_surf.Y2(), view[exp_label].getID());   
                        }*/
                        //double angle1 = 0;
                        int label_num = 0;
                        for(int k = 0; k < view.size(); k++)
                        {
                            if(view[k].get_match_label() == global_map[m].get_match_label())
                            {
                                //angle1 = ref_obj.getAngleWithLine(view[k]);
                                angle1 = view[k].getAngleWithXaxis();
                                label_num = k;
                                break;
                            }
                        }
                        
                        if((fill_mode == 1) || (fill_mode == 3 && fill_finish_flag == 0))
                        {
                            if(common_endpoint(detect_obj, global_map[m]) == 1)
                            {
                                cout << "two lines share the endpoint 1" << endl;
                                //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,1);
                                //fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj,1);
                                if(fill_cnt == 1)
                                    fill_obj.set(ref_obj.X1(), ref_obj.Y1(), view[label_num].X1(), view[label_num].Y1(), global_map[m].getID());
                                else
                                    fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 2, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP1()));
                                
  
                                fill_obj.setID(m);
                                fill_info.push_back(fill_obj);
                                ref_obj = fill_obj;
                                detect_obj = global_map[m];
                                fill_cnt++;
                                filled_id_num.push_back(m);

                                vector<Object> test;
                                test.push_back(global_map[m]);
                                sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                waitHere();
                            }
                        }
                        //else
                        if((fill_mode == 2) || (fill_mode == 3 && fill_finish_flag == 1))
                        {
                            if(common_endpoint(detect_obj, global_map[m]) == 2)
                            {
                                cout << "two lines share the endpoint 2" << endl;
                                //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,2);
                                //fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj,2);
                                if(fill_cnt == 1)
                                    fill_obj.set(ref_obj.X1(), ref_obj.Y1(), view[label_num].X1(), view[label_num].Y1(), global_map[m].getID());
                                else
                                    fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 1, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP2()));
                                
                                fill_obj.setID(m);
                                fill_info.push_back(fill_obj);
                                ref_obj = fill_obj;
                                detect_obj = global_map[m];
                                fill_cnt++;
                                filled_id_num.push_back(m);

                                vector<Object> test;
                                test.push_back(global_map[m]);
                                sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                waitHere();
                            }
                        }
                        m = 0;
 
                    }


                }
                else
                {
                    if((twoLinintersect(global_map[m].getP1(), global_map[m].getP2(), detect_obj.getP1(), detect_obj.getP2()) == true)
                    //&& ((intersectPointTwolineEquations(global_map[m], detect_obj) == detect_obj.getP1()) || (intersectPointTwolineEquations(global_map[m], detect_obj) == detect_obj.getP2()) )
                    && ((global_map[m].get_match_flag() != 1) || (global_map[m].get_match_label() <= 1)))
                    {
                        int filled_flag = 0;
                        for(int j = 0; j < filled_id_num.size(); j++)
                        {
                            if(filled_id_num[j] == m)
                            {
                                filled_flag = 1;
                                break;
                            }
                        }

                        if(filled_flag != 1)
                        {
                            
                            //angle1 = global_map[ref_obj.getID()].getAngleWithLine(global_map[m])+3.4115;
                            //angle1 = 3.1415926/1.9 - includedAngle(detect_obj, global_map[m]) - global_map[ref_obj.getID()].getAngleWithXaxis();

                            double k1, k2, _k1, _k2;
                            k1 = (detect_obj.Y2() - detect_obj.Y1()) / (detect_obj.X2() -  detect_obj.X1());
                            k2 = (global_map[m].Y2() - global_map[m].Y1()) / (global_map[m].X2() -  global_map[m].X1());
                            _k1 = (ref_obj.Y2() - ref_obj.Y1()) / (ref_obj.X2() -  ref_obj.X1());
                            _k2 = _k1 - k1 + k2;
                            angle1 = atan(k2);
                            

                            if((fill_mode == 1) || (fill_mode == 3 && fill_finish_flag == 0))
                            {
                                if(common_endpoint(detect_obj, global_map[m]) == 1)
                                {
                                    cout << "two lines share the endpoint 1" << endl;
                                    //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,1);
                                    fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 2, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP1()));
                                    
                                    fill_obj.setID(m);
                                    fill_info.push_back(fill_obj);
                                    ref_obj = fill_obj;
                                    detect_obj = global_map[m];
                                    fill_cnt++;
                                    filled_id_num.push_back(m);
    
                                    vector<Object> test;
                                    test.push_back(global_map[m]);
                                    sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                    plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                    waitHere();
                                }
                                else
                                {
                                    //intersected point is close to ref p2
                                    Point Interspoint = intersectPointTwolineEquations(global_map[m], detect_obj);
                                    if(distanceOftwoP(Interspoint, detect_obj.getP1()) < 300)
                                    {
                                        cout << "two lines share the endpoint 1" << endl;
                                        //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,1);
                                        fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 1, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP1()));

                                        fill_obj.setID(m);
                                        fill_info.push_back(fill_obj);
                                        ref_obj = fill_obj;
                                        detect_obj = global_map[m];
                                        fill_cnt++;
                                        filled_id_num.push_back(m);

                                        vector<Object> test;
                                        test.push_back(global_map[m]);
                                        sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                        plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                        waitHere();
                                    }
                                }
                                
                            }
                            //else
                            if((fill_mode == 2) || (fill_mode == 3 && fill_finish_flag == 1))
                            {
                                if(common_endpoint(detect_obj, global_map[m]) == 2)
                                {
                                    cout << "two lines share the endpoint 2" << endl;
                                    //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,2);
                                    fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 2, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP2()));
                                    
                                    fill_obj.setID(m);
                                    fill_info.push_back(fill_obj);
                                    ref_obj = fill_obj;
                                    detect_obj = global_map[m];
                                    fill_cnt++;
                                    filled_id_num.push_back(m);

                                    vector<Object> test;
                                    test.push_back(global_map[m]);
                                    sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                    plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                    waitHere();
                                }
                                else
                                {
                                    //intersected point is close to ref p2
                                    Point Interspoint = intersectPointTwolineEquations(global_map[m], detect_obj);
                                    if(distanceOftwoP(Interspoint, detect_obj.getP2()) < 300)
                                    {
                                        cout << "two lines share the endpoint 2" << endl;
                                        //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,2);
                                        fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 1, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP2()));

                                        fill_obj.setID(m);
                                        fill_info.push_back(fill_obj);
                                        ref_obj = fill_obj;
                                        detect_obj = global_map[m];
                                        fill_cnt++;
                                        filled_id_num.push_back(m);

                                        vector<Object> test;
                                        test.push_back(global_map[m]);
                                        sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                        plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                        waitHere();
                                    }
                                }
                                
                            }
                            
                            m = 0;
                        }
 
                    }
                    else
                    {
                        if((m == global_map.size()-1) && (fill_finish_flag == 0) && (fill_mode == 3))
                        {
                            fill_finish_flag = 1;
                            m = 0;
                            goto lb;
                        }
                        /*if(fill_mode == 3 && fill_finish_flag == 0)
                        {
                            fill_finish_flag = 1;
                            m = 0;
                        }
                        else
                            if(fill_mode == 3 && fill_finish_flag == 1)
                                fill_finish_flag = 2;*/
                        /*if((fill_mode == 1) || (fill_mode == 3 && fill_finish_flag == 0))
                        {
                            angle1 = global_map[ref_obj.getID()].getAngleWithLine(global_map[m]);
                            if(distanceOftwoP(global_map[m].getP2(), detect_obj.getP1()) < 300)
                            {
                                cout << "two lines share the endpoint 1" << endl;
                                //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,2);
                                fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 2, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP2()));

                                //fill_obj.display();
                                fill_obj.setID(m);
                                fill_info.push_back(fill_obj);
                                ref_obj = fill_obj;
                                detect_obj = global_map[m];
                                fill_cnt++;
                                filled_id_num.push_back(m);
                                m = 0;
                                //goto lb;
                                vector<Object> test;
                                test.push_back(global_map[m]);
                                sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                waitHere();
                            }
                            
                        }
                        
                        if((fill_mode == 2) || (fill_mode == 3 && fill_finish_flag == 1))
                        {
                            angle1 = global_map[ref_obj.getID()].getAngleWithLine(global_map[m]);
                            if(distanceOftwoP(global_map[m].getP1(), detect_obj.getP1()) < 300)
                            {
                                cout << "two lines share the endpoint 2" << endl;
                                //fill_obj = makeLineAtTwoPointsWithObject(angle1, global_map[m].length(), angle1, global_map[m].length(), ref_obj,2);
                                fill_obj = CreateLineOnObject(angle1, global_map[m].length(), ref_obj, detect_obj, 1, isLeft(detect_obj.getP1(), detect_obj.getP2(), global_map[m].getP2()));

                                //fill_obj.display();
                                fill_obj.setID(m);
                                fill_info.push_back(fill_obj);
                                ref_obj = fill_obj;
                                detect_obj = global_map[m];
                                fill_cnt++;
                                filled_id_num.push_back(m);
                                m = 0;
                                //goto lb;
                                vector<Object> test;
                                test.push_back(global_map[m]);
                                sprintf(plotFile, "%s%d%s", "Maps/Offline/Filling_in_process-", cnt++, ".png");
                                plotObjectsOf3Kinds(plotFile, global_map, fill_info, test);
                                waitHere();
                            }
                            
                        }*/

                    }
                }
    
        }

        return fill_info;    
}

vector<Object> Fill_operation_with_ASR(vector<Object> view, vector<Object> cv, vector<Object> global_map, 
                                       vector< vector<Object> > match_list, vector<Object> references)
{
        vector<int> match_ASRs;
        vector<Object> info_collect;

        vector<Object> temp;

        for(int i = 0; i < match_list.size(); i++)
        {
            match_ASRs.push_back(match_list[i][0].getVN());
        }

        for(int i = 0; i < match_ASRs.size(); i++)
        {
            for (int j = 0; j < match_ASRs.size() - i; j++) 
            {
                if (match_ASRs[j] > match_ASRs[j + 1]) 
                {
                    int temp = match_ASRs[j];
                    match_ASRs[j] = match_ASRs[j + 1];
                    match_ASRs[j + 1] = temp;
                }
            }
        }
        
        for(int i = 0; i < global_map.size(); i++)
        {
            for(int j = 0; j < match_ASRs.size(); j++)
            {
                if(global_map[i].getVN() == match_ASRs[j])
                    info_collect.push_back(global_map[i]);     
            }
        }
        
        remaining = deletOneViewbaseLength(global_map, info_collect);
        
        Object ref_project;
        for(int i = 0; i < view.size(); i++)
        {
            if(view[i].length() == references[1].length())
            {
                ref_project = view[i];
                break;
            }
        }
        
        //except the ref
        for(int i = 0; i < info_collect.size(); i++)
        {
            if(info_collect[i].length() == references[0].length())
                info_collect.erase(info_collect.begin()+i);

        }
        
        //temp = projectingTheView(info_collect, references[0], ref_project, 2);
        temp = projectingTheView(info_collect, references[0], references[1], 2);
        
        //----the same one, simply project, others can't project using references----//
        
        
        
        //for(int i = 0; i < view.size(); i++)
        //{
        //    if(view[i].length() == references[1].length())
        //    {
                temp.push_back(references[0]);//insert ref object
                temp.back().set_imagined_flag(1);
        //        break;
        //    }
        //}
        
        
        //mark all those surfaces
        for(int i = 0; i < temp.size(); i++)
        {
            if((info_collect[i].get_match_label() > 1)
                && (info_collect[i].get_imagined_flag() != 1))
            {
                temp[i].set_match_flag(info_collect[i].get_match_flag());
                temp[i].set_match_label(info_collect[i].get_match_label());
                temp[i].set_imagined_flag(2); //not imagined 
            }
            else
            {
                temp[i].set_match_flag(info_collect[i].get_match_flag());
                temp[i].set_match_label(info_collect[i].get_match_label());
                temp[i].set_imagined_flag(2); //imagined
            }
        }
            
        return temp;
}

/*
vector<Object> Fill_operation_with_Circle(vector<Object> view, vector<Object> cv, vector<Object> global_map, 
                                          vector<Object> position, vector<Object> references)
{
    double radius = 5000;
    Point centre;
    vector<Object> info_collect, rnt;
    
    centre = position[6].getP1();
    for(int i = 0; i < global_map.size(); i++)
    {
        vector<Object> temp;
        temp.push_back(global_map[i]);
        
        if((distanceOftwoP(global_map[i].getP1(), centre) <= radius)
            || (distanceOftwoP(global_map[i].getP2(), centre) <= radius)
            || (P_To_ShortestDistance(centre, global_map[i]) <= radius))
        {
                info_collect.push_back(global_map[i]);
        }
    }

    rnt = projectingTheView(info_collect, references[0], references[1], 1);
    return rnt;
}
*/


vector<Object> Correct_detail(vector<Object> view, vector<Object>& global_map)
{
    cout <<" This detail correct position process"  << endl;
    
    Object correspond_obj;
    Object temp_modi;
    vector<Object> temp_global_map;
    
    int flag = 0;
    double dist1, dist2;
    Point p1,p2;
    
    temp_global_map = global_map;
    for(int i = 0; i < temp_global_map.size(); i++)
    {

        if((temp_global_map[i].get_match_label() > 1)
            && (temp_global_map[i].get_imagined_flag() != 1))
        {
            //modified this piece of info
            for(int j = 0; j < view.size(); j++)
            {
                if((view[j].get_match_label() == temp_global_map[i].get_match_label())
                    && (view[j].length() > 1000))
                {
                    correspond_obj = view[j];
                    flag = 1;
                    break;
                }
            }
            
            if(flag == 1)
            {
                //expend correspond object
                if(distanceOftwoP(temp_global_map[i].getP1(), correspond_obj.getP1()) < 300)
                {
                      dist1 = temp_global_map[i].length() - correspond_obj.length();
                      temp_modi = expend_Object(correspond_obj, abs(dist1), 2);
                      temp_modi.set(temp_modi.X2(), temp_modi.Y2(), correspond_obj.X1(), correspond_obj.Y1(), correspond_obj.getID());

                      temp_modi.set_imagined_flag(1);
                      temp_global_map.erase(temp_global_map.begin()+i);
                      temp_global_map.insert(temp_global_map.begin()+i, temp_modi);
                }
                else
                {
                    if(distanceOftwoP(temp_global_map[i].getP2(), correspond_obj.getP2()) < 300)
                    {
                        dist2 =  temp_global_map[i].length() - correspond_obj.length();
                        temp_modi = expend_Object(correspond_obj, abs(dist2), 1);
                        temp_modi.set(correspond_obj.X2(), correspond_obj.Y2(), temp_modi.X1(), temp_modi.Y1(),  correspond_obj.getID());
                        
                        temp_modi.set_imagined_flag(1);
                        temp_global_map.erase(temp_global_map.begin()+i);
                        temp_global_map.insert(temp_global_map.begin()+i, temp_modi);
                    }
                    else 
                    {
                        dist1 = distanceOftwoP(temp_global_map[i].getP1(), correspond_obj.getP1());
                        dist2 =   distanceOftwoP(temp_global_map[i].getP2(), correspond_obj.getP2());
                        temp_modi = expend_Object_two_sides(correspond_obj, dist1, dist2);
                        temp_modi.set_imagined_flag(1);
                        
                        temp_global_map.erase(temp_global_map.begin()+i);
                        temp_global_map.insert(temp_global_map.begin()+i, temp_modi);
                    }
                }
                temp_global_map[i].set_imagined_flag(1);
            }
            
        }
    }
    
    for(int i = 0; i < global_map.size(); i++)
    {
        if(global_map[i].get_imagined_flag() == 1)
            temp_global_map[i].set_imagined_flag(1);
        
        if((global_map[i].get_imagined_flag() == 2) && (temp_global_map[i].get_imagined_flag() != 1))
            temp_global_map[i].set_imagined_flag(2);
    }
    
    return temp_global_map;
}

vector<Object> Correct_detail_blocked(vector<Object> view, vector<Object>& global_map, vector<Object> shift_position)
{
    double dist1, dist2;
    Object temp_modi;
    vector<Object> temp_global_map;
    
    temp_global_map = global_map;
    
    
    for(int i = 0; i < view.size(); i++)
    {
        for(int j = 0; j < temp_global_map.size(); j++)
        {
            Object connect_line;
            connect_line.set(shift_position[6].X1(), shift_position[6].Y1(), view[i].midpoint().X(), view[i].midpoint().Y(), 0);
                                    
            if((TwoObjectIntersect(connect_line, temp_global_map[j]) == true)
                && (temp_global_map[j].get_imagined_flag() != 1)
                && (shortestDistanceBtwTwoObjects(view[i], temp_global_map[j]) < 1000))
            {
                //temp_modi = view[i];
                
                /*if(distanceOftwoP(view[i].getP1(), temp_global_map[j].getP1()) < distanceOftwoP(view[i].getP2(), temp_global_map[j].getP2()))
                {
                    dist1 = temp_global_map[j].length() - view[i].length();
                      temp_modi = expend_Object(view[i], abs(dist1), 2);
                      temp_modi.set(temp_modi.X2(), temp_modi.Y2(), view[i].X1(), view[i].Y1(), i);

                      temp_modi.set_imagined_flag(1);
                      temp_global_map.erase(temp_global_map.begin()+j);
                      temp_global_map.insert(temp_global_map.begin()+j, temp_modi);
                }
                else
                {
                    dist2 =  temp_global_map[j].length() - view[i].length();
                    temp_modi = expend_Object(view[i], abs(dist2), 1);
                    temp_modi.set(view[i].X2(), view[i].Y2(), temp_modi.X1(), temp_modi.Y1(),  i);
                        
                    temp_modi.set_imagined_flag(1);
                    temp_global_map.erase(temp_global_map.begin()+j);
                    temp_global_map.insert(temp_global_map.begin()+j, temp_modi);
                }
                
                temp_global_map[j].set_imagined_flag(1);*/
                
                if(abs(view[i].length()/ temp_global_map[j].length()) <= 0.20)
                {

                    if(distanceOftwoP(view[i].getP1(), temp_global_map[j].getP1()) < distanceOftwoP(view[i].getP2(), temp_global_map[j].getP2()))
                        
                    {
                        temp_modi.set(view[i].X2(), view[i].Y2(), temp_global_map[j].X2(), temp_global_map[j].Y2(), i);
                        temp_modi.set_imagined_flag(1);
                        temp_global_map.erase(temp_global_map.begin()+j);
                        temp_global_map.insert(temp_global_map.begin()+j, temp_modi);
                        view[i].set_imagined_flag(1);
                        temp_global_map.insert(temp_global_map.begin()+j, view[i]);
                    }
                    else
                    {
                        temp_modi.set(temp_global_map[j].X1(), temp_global_map[j].Y1(), view[i].X1(), view[i].Y1(), i);
                        temp_modi.set_imagined_flag(1);
                        temp_global_map.erase(temp_global_map.begin()+j);
                        view[i].set_imagined_flag(1);
                        temp_global_map.insert(temp_global_map.begin()+j, view[i]);
                        temp_global_map.insert(temp_global_map.begin()+j, temp_modi);
                        
                    }

                }
            }
        }
    }
    
    return temp_global_map;
}

vector<Object> adjust_base_spatial(vector<Object> cv, vector<Object> rb, vector< vector<Object> > match_list)
{
    vector<Object> adjust_view;
    int front_match = 0, left_match = 0, right_match = 0;
    vector<Object> front_ref, left_ref, right_ref;
    for(int m = 0; m < match_list.size(); m++)
    {
        if((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP1()) > 0)
            && ((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP2()) > 0)))
        {
            left_ref = match_list[m];
            left_match = 1;
        }

        if((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP1()) < 0)
            && ((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP2()) < 0)))
        {
            right_ref = match_list[m];
            right_match = 1;
        }

        if(((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP1()) > 0)
            && ((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP2()) < 0)))
            || ((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP1()) < 0)
            && ((isLeft(rb[6].getP1(), rb[6].getP2(), match_list[m][1].getP2()) > 0))))
        {
            front_ref = match_list[m];
            front_match = 1;
        }
    }

    cout << " front_match : " << front_match << endl;
    cout << "  left_match : " << left_match << endl;
    cout << " front_match : " << front_match << endl;
    
    if((front_match == 1) && (left_match == 1 || right_match == 1))
    {
        //rotate 
        double rotate_angle = includedAngle(left_ref[0], left_ref[1]);
        if(rotate_angle > 90)
            rotate_angle -= 180;
        cout << " include angle : " << rotate_angle << endl;
        waitHere();
        adjust_view = TransformforToGlobalCoordinate(cv, Point (0,0), rb, abs(rotate_angle));
        
        char plot[80];
        sprintf(plot, "%s", "Maps/test_adjust_step_1.png");
        plotObjects(plot,cv,adjust_view);
        
        //horizontally                            
        Point shift_point;
        /*if(left_ref[0].X1() > left_ref[1].X1())
            shift_point.set(0 + shortestDistanceBtwTwoObjects(left_ref[0], left_ref[1]), 0);
        else
            shift_point.set(0 - shortestDistanceBtwTwoObjects(left_ref[0],left_ref[1]), 0);
        
        adjust_view = TransformforToGlobalCoordinate(adjust_view, shift_point, rb, 0);        
        sprintf(plot, "%s", "Maps/test_adjust_step_2.png");
        plotObjects(plot,cv,adjust_view);*/
        
        //vertically
        if(front_ref[0].Y1() > front_ref[1].Y1())
            shift_point.set(0, 0 + shortestDistanceBtwTwoObjects(front_ref[0], front_ref[1]));
        else
            shift_point.set(0, 0 - shortestDistanceBtwTwoObjects(front_ref[0], front_ref[1]));
        
        adjust_view = TransformforToGlobalCoordinate(adjust_view, shift_point, rb, 0);   
        sprintf(plot, "%s", "Maps/test_adjust_step_3.png");
        plotObjects(plot,cv,adjust_view);
        waitHere();
    }
    
    return adjust_view;
}


vector< vector<Object> > Current_and_expectation(vector<Object> currentView, vector<Object> rb, vector<Object> MFIS, int v)
{
        char plot[80];
        double max = 0;
        int block_flag = 0;
        int match_flag = 0;

        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view



        vector< vector<Object> > match_list;
        vector< vector<Object> > temp_rnt;   //return adjust view, robot position & adjust gloal map

 
    
        for(int i = 0; i < currentView.size(); i++)
        {     

                temp_match = Compare_similar_near_Obeject_(MFIS, currentView[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(currentView[i]);
                    
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }
        }


        if(match_list.size() == 0)
        {
                    for(int i = 0; i < currentView.size(); i++)
                    {
                        if(currentView[i].length() >= 500)
                        {
                            for(int t = 0; t < MFIS.size(); t++)
                            {
                                if((((abs(currentView[i].length() / MFIS[t].length()) > 0.60) && (abs(currentView[i].length() / MFIS[t].length()) <= 1.00))
                                    || ((abs(currentView[i].length() / MFIS[t].length()) >= 1.00) && (abs(currentView[i].length() / MFIS[t].length()) < 1.38)))
                                    && (currentView[i].length() > 700))
                                {
                                        matched_info.push_back(MFIS[t]);
                                        matched_info.push_back(currentView[i]);

                                        match_list.push_back(matched_info);
                                        matched_info.clear();
                                }
                            }
                        }
                    }
            
                    
                    if(match_list.size() > 0)
                    {
                        //choose a proper one as reference
                        for(int j = 0; j < match_list.size(); j++)
                        {
                                if(j == 0)
                                {
                                    max = match_list[j][1].length();
                                    reference_obj = match_list[j];
                                }
                                else
                                {
                                    if(max < match_list[j][1].length())
                                    {
                                        max = match_list[j][1].length();
                                        reference_obj = match_list[j];
                                    }
                                }
                        }
                    }
                    
                    if(max > 0)
                    {
                        //triangulate/shift robot position on MFIS based on global information rather than current
                        shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                        adjust_view = projectingTheView_with_correctDirection(currentView, reference_obj[0], reference_obj[1], 2);

                        temp_rnt.push_back(adjust_view);
                        temp_rnt.push_back(shift_position);
                        sprintf(plot, "%s%d%s", "Maps/Adjust_info-", v,".png");
                        plotObjectsOf5Kinds(plot,adjust_view,shift_position, currentView,rb, MFIS);
                    }
        }
        else
        {
            
                //choose a proper one as reference
                for(int j = 0; j < match_list.size(); j++)
                {
                    //same size is the 
                    if((abs(match_list[j][0].length() - match_list[j][1].length()) <= 650.00)
                        && (match_list[j][1].length() >= 700))
                    {
                        if(j == 0)
                        {
                            max = match_list[j][1].length();
                            reference_obj = match_list[j];
                        }
                        else
                        {
                            if(max < match_list[j][1].length())
                            {
                                max = match_list[j][1].length();
                                reference_obj = match_list[j];
                            }
                        }
                    }
                }
                
                
                if(max > 0)
                {
                    //triangulate/shift robot position on MFIS based on global information rather than current
                    shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView_with_correctDirection(currentView, reference_obj[0], reference_obj[1], 2);
         
                    temp_rnt.push_back(adjust_view);
                    temp_rnt.push_back(shift_position);  
                    sprintf(plot, "%s%d%s", "Maps/Adjust_info-", v,".png");
                    plotObjectsOf5Kinds(plot,adjust_view,shift_position, currentView,rb, MFIS);
                    
                }
                
                
        }
        
        if(max == 0)
        {
                temp_rnt.push_back(currentView);
                temp_rnt.push_back(rb);  
                sprintf(plot, "%s%d%s", "Maps/PathInte_info-", v,".png");
                plotObjectsOf3Kinds(plot,currentView,rb ,MFIS);
        }

        
        return temp_rnt;
}

vector< vector<Object> > structure_align(vector<Object> currentView, vector<Object> rb, vector<Object> MFIS, int v, int& change_flag)
{
        char plot[80];
        double max = 0;
        int block_flag = 0;
        int match_flag = 0;

        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> adjust_global_info;
        vector<Object> collect_info;
        vector<Object> Global_map; 

        vector< vector<Object> > match_list;
        vector< vector<Object> > temp_rnt;   //return adjust view, robot position & adjust gloal map

        Global_map = MFIS;
        if(change_flag == 0)
        {
                
          
                //change & align info
                for(int i = 0; i < currentView.size(); i++)
                {     

                        temp_match = Compare_similar_near_Obeject_(Global_map, currentView[i], v);
                        if(temp_match.X1() != 0 && temp_match.X2() != 0)
                        {
                            matched_info.push_back(temp_match);
                            matched_info.push_back(currentView[i]);

                            match_list.push_back(matched_info);
                            matched_info.clear();
                        }
                }
                
                if(match_list.size() > 0)
                {
                    //choose a proper one as reference
                    for(int j = 0; j < match_list.size(); j++)
                    {
                            if(j == 0)
                            {
                                max = match_list[j][1].length();
                                reference_obj = match_list[j];
                            }
                            else
                            {
                                if(max < match_list[j][1].length())
                                {
                                    max = match_list[j][1].length();
                                    reference_obj = match_list[j];
                                }
                            }
                            
                            collect_info.push_back(reference_obj[0]);
                    }
                }

                if(max > 0 && currentView.size() > 2)
                {
                    int cnt;
                    for(int n = 0; n < MFIS.size(); n++)
                    {
                        if((MFIS[n].getP1() == reference_obj[0].getP1())
                            && (MFIS[n].getP2() == reference_obj[0].getP2()))
                        {
                            cnt = n;
                            break;
                        }
                    }
                    
                    for(int i = cnt-1; i >= 0; i--)
                    {
                        if((collect_info.back().getP2() == Global_map[i].getP1())
                            || (distanceOftwoP(collect_info.back().getP1(), Global_map[i].getP2()) <= 400))
                        {
                            collect_info.push_back(Global_map[i]);
                            Global_map.erase(Global_map.begin()+i);
                        }
 
                    }
        
                    
                    shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_global_info = projectingTheView_with_correctDirection(collect_info, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView_with_correctDirection(currentView, reference_obj[0], reference_obj[1], 2);
                    Global_map = addTwoVectorsOfObjects(Global_map, adjust_global_info);
                    change_flag = 1;
                    sprintf(plot, "%s%d%s", "Maps/Offline/Collect_change_info&adjust", v, ".png");
                    plotObjectsOf4Kinds(plot, collect_info, shift_position, adjust_view, Global_map);
                }
                else
                {
                    //Global_map = MFIS
                    shift_position = rb;
                    adjust_view = currentView;
                }
                
                
        }
        else
        { 
                //align with MFIS
                for(int i = 0; i < currentView.size(); i++)
                {     

                        temp_match = Compare_similar_near_Obeject_(MFIS, currentView[i], v);
                        if(temp_match.X1() != 0 && temp_match.X2() != 0)
                        {
                            matched_info.push_back(temp_match);
                            matched_info.push_back(currentView[i]);

                            match_list.push_back(matched_info);
                            matched_info.clear();
                        }
                }

                if(match_list.size() > 0)
                {
                    //choose a proper one as reference
                    for(int j = 0; j < match_list.size(); j++)
                    {
                            if(j == 0)
                            {
                                max = match_list[j][1].length();
                                reference_obj = match_list[j];
                            }
                            else
                            {
                                if(max < match_list[j][1].length())
                                {
                                    max = match_list[j][1].length();
                                    reference_obj = match_list[j];
                                }
                            }
                            
                            collect_info.push_back(reference_obj[0]);
                    }
                }

                if(max > 0)
                {
                    shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView_with_correctDirection(currentView, reference_obj[0], reference_obj[1], 2);
                }
        }
        
        
        temp_rnt.push_back(Global_map);
        temp_rnt.push_back(shift_position);
        temp_rnt.push_back(adjust_view);
        return temp_rnt;
}


/*vector<Object> position_in_theory(vector<Object> currentRobotPositionInMFIS, vector<Object> allRobotPositions, vector<double> coordTransInfo)
{
    double distancen = 0, angle = 0;
    vector<Object> theoritical_position;
    if(coordTransInfo[0] >= 100)
        distancen = 100;
    if(coordTransInfo[1] >= 9)
        angle = 10;
        
    
    Object aLine = makeLineAtTwoPointsWithObject(angle, distancen, angle, distancen + 500, currentRobotPositionInMFIS[6], 1);
    aLine.setKP(1);

    odometricReferenceObject.push_back(aLine);
    odometricReferenceObject.push_back(allRobotPositions[6]);
    theoritical_position = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());

    return theoritical_position;
}*/

vector<Object> view_in_theory(vector<Object> MFIS, vector<Object> view, vector<Object> robot_position, int v)
{
    //robot_position[7] right hand side /robot_position[8] left hand side 
    char plot[80];
    double max_dist = 0, threshold_range = 400; //range 
    Point left_limit_point, right_limit_point; //limit point
    Object modify_obj; 
    Object expend_left, expend_right;
    Object left_view_limit, right_view_limit;
    vector<Object> global_map, trim_info;
    
    vector<Object> test;
    
    global_map = MFIS;
    
    expend_left = expend_Object(robot_position[8], 2000, 2);
    expend_right = expend_Object(robot_position[7], 2000, 2);
    left_view_limit.set(robot_position[8].X1(),robot_position[8].Y1(), expend_left.X2(), expend_left.Y2(), 1);
    right_view_limit.set(robot_position[7].X1(),robot_position[7].Y1(), expend_right.X2(), expend_right.Y2(), 1);
    
    test.push_back(left_view_limit);
    test.push_back(right_view_limit);

    left_limit_point = intersectedNearPoint(MFIS, left_view_limit.getP1(), left_view_limit.getP2());
    right_limit_point = intersectedNearPoint(MFIS, right_view_limit.getP1(), right_view_limit.getP2());

    //cut information
    for(int i = 0; i < global_map.size(); i++)
    {
 
            if((isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP1()) < 0)  
                    && (isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP2()) < 0) 
                    && (isLeft(robot_position[7].getP2(), robot_position[8].getP2(), robot_position[6].getP2()) < 0))
            {
                    trim_info.push_back(global_map[i]);
           
            }
            else
            {
                if((TwoObjectIntersect(left_view_limit, global_map[i]) == true)
                    || ((isLeft(robot_position[6].getP1(), robot_position[6].getP2(), global_map[i].getP1()) > 0)
                       && (isLeft(robot_position[6].getP1(), robot_position[6].getP2(), global_map[i].getP2()) > 0)))
                {
                    if((isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP1()) < 0) //p1 
                            && (isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP2()) > 0)) //p2 
                    {
                        //modified this surface using left point
                        if(left_limit_point.X() !=0 && left_limit_point.Y() != 0)
                            modify_obj.set(left_limit_point.X(), left_limit_point.Y(), global_map[i].X1(), global_map[i].Y1(), i);
                        else
                            modify_obj = global_map[i];

                    }
                    
                    if((isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP1()) > 0) //p1 
                            && (isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP2()) < 0)) //p2 
                    {
                        //modified this surface using left point
                        if(left_limit_point.X() !=0 && left_limit_point.Y() != 0)
                            modify_obj.set(left_limit_point.X(), left_limit_point.Y(), global_map[i].X2(), global_map[i].Y2(), i);
                        else
                            modify_obj = global_map[i];

                    }
                    if(modify_obj.X1() != 0 && modify_obj.X2() !=0)
                        trim_info.push_back(modify_obj);
                }
                
                if((TwoObjectIntersect(right_view_limit, global_map[i]) == true)
                    || ((isLeft(robot_position[6].getP1(), robot_position[6].getP2(), global_map[i].getP1()) < 0)
                       && (isLeft(robot_position[6].getP1(), robot_position[6].getP2(), global_map[i].getP2()) < 0)))
                {
                    if((isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP2()) > 0)  //p2 left
                            && (isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP1()) < 0)) //p1 right
                    {
                        //modified this surface using right point
                        if(left_limit_point.X() !=0 && left_limit_point.Y() != 0)
                            modify_obj.set(global_map[i].X1(), global_map[i].Y1(), right_limit_point.X(), right_limit_point.Y(), i);
                        else
                            modify_obj = global_map[i];

                    }
                    
                    if((isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP2()) < 0)  //p2 left
                            && (isLeft(robot_position[7].getP2(), robot_position[8].getP2(), global_map[i].getP1()) > 0)) //p1 right
                    {
                        //modified this surface using right point
                        if(left_limit_point.X() !=0 && left_limit_point.Y() != 0)
                            modify_obj.set(global_map[i].X2(), global_map[i].Y2(), right_limit_point.X(), right_limit_point.Y(), i);
                        else
                            modify_obj = global_map[i];
       
                    }
                    if(modify_obj.X1() != 0 && modify_obj.X2() !=0)
                        trim_info.push_back(modify_obj);
                }
   
            }
    }


    for(int i = 0; i < view.size(); i++)
    {
        //furthest distance
        if(i == 0)
        {
            if(distanceOftwoP(view[i].getP1(), robot_position[6].getP1()) > distanceOftwoP(view[i].getP2(), robot_position[6].getP1()))
                max_dist = distanceOftwoP(view[i].getP1(), robot_position[6].getP1());
            else
                max_dist = distanceOftwoP(view[i].getP2(), robot_position[6].getP1());
        }
        else
        {
            if((distanceOftwoP(view[i].getP1(), robot_position[6].getP1())  > max_dist) && (view[i].length() >= 800))
                max_dist = distanceOftwoP(view[i].getP1(), robot_position[6].getP1());
            if((distanceOftwoP(view[i].getP2(), robot_position[6].getP1())  > max_dist) && (view[i].length() >= 800))
                max_dist = distanceOftwoP(view[i].getP2(), robot_position[6].getP1());   
        }
    }

    for(int i = 0; i < trim_info.size(); i++)
    {
        if((distanceOftwoP(trim_info[i].getP1(), robot_position[6].getP1())  >= (max_dist + threshold_range))
            || (distanceOftwoP(trim_info[i].getP2(), robot_position[6].getP1())  >= (max_dist + threshold_range))
            || (trim_info[i].length() < 100))
        {
            trim_info.erase(trim_info.begin()+i);
            i--;
        }
    }
    
    
    /////////////////////////////////////////////////////
    vector<Object> duplicate;
    duplicate = trim_info;
    for(int i = 0; i < trim_info.size(); i++)
    {
        Object line;

        line.set(robot_position[6].X1(), robot_position[6].Y1(), trim_info[i].midpoint().X(), trim_info[i].midpoint().Y(), 0);
        for(int j = 0; j < duplicate.size(); j++)
        {
            //if(j != i)
            if((duplicate[j].X1() != trim_info[i].X1()) && (duplicate[j].X2() != trim_info[i].X2()))
            {

                if(TwoObjectIntersect(duplicate[j], line) == true)
                {
                    if((crossedPointReturnP(duplicate[j].getP1(), duplicate[j].getP2(), line.getP1(), line.getP2()) != duplicate[j].getP1()) 
                            || (crossedPointReturnP(duplicate[j].getP1(), duplicate[j].getP2(), line.getP1(), line.getP2()) != duplicate[j].getP2()))
                    {
                        trim_info.erase(trim_info.begin()+i);
                        i = 0;
                        break;
                    }
                }
            }
            
        }
    }
    
    //sprintf(plot, "%s%d%s", "Maps/Offline/Trim_info_and_view", v, ".png");
    //plotObjectsOf5Kinds(plot, trim_info, view, robot_position, test, global_map);
    
    return trim_info;
}

vector< vector<Object> > shift_after_expect(vector<Object> expect_info, vector<Object> view, vector<Object> rb, int v)
{
    cout << " ----- Using theoretical view to do adjustment process ------ " << endl;
    char plot[80];
    Point exp_p1, exp_p2;
    Object side1, side2, exp_seg1, exp_seg2;
    
    vector<Object> potential_list;
    vector<Object> match_pair, reference_obj;
    
    vector<Object> shift_position;
    vector<Object> adjust_view;
    
    vector< vector<Object> > all_match_pairs, temp_rnt;

    for(int i = 0; i < expect_info.size(); i++)
    {
        side1.set(rb[6].X1(), rb[6].Y1(), expect_info[i].X1(), expect_info[i].Y1(), i);
        side2.set(rb[6].X1(), rb[6].Y1(), expect_info[i].X2(), expect_info[i].Y2(), i);
        
        exp_seg1 = expend_Object(side1, 1000, 2);
        exp_seg2 = expend_Object(side2, 1000, 2);
        
        //final range/sides
        side1.set(side1.X1(), side1.Y1(), exp_seg1.X2(), exp_seg1.Y2(), i);
        side2.set(side2.X1(), side2.Y1(), exp_seg2.X2(), exp_seg2.Y2(), i);

        for(int j = 0; j < view.size(); j++)
        {
            if((TwoObjectIntersect(view[j], side1) == true) || (TwoObjectIntersect(view[j], side2) == true))
            {
                potential_list.push_back(view[j]);
                
                if(j > 0 && j < view.size()-1)
                {
                    potential_list.push_back(view[j+1]);
                    potential_list.push_back(view[j-1]);
                }
            }
            else
            {
                if(((isLeft(side1.getP1(), side1.getP2(),view[j].getP1()) > 0) && (isLeft(side1.getP1(), side1.getP2(),view[j].getP2()) > 0)
                        && (isLeft(side2.getP1(), side2.getP2(),view[j].getP1()) < 0) && (isLeft(side2.getP1(), side2.getP2(),view[j].getP2()) < 0))
                    || ((isLeft(side1.getP1(), side1.getP2(),view[j].getP1()) < 0) && (isLeft(side1.getP1(), side1.getP2(),view[j].getP2()) < 0)
                        && (isLeft(side2.getP1(), side2.getP2(),view[j].getP1()) > 0) && (isLeft(side2.getP1(), side2.getP2(),view[j].getP2()) > 0)))
                {
                    potential_list.push_back(view[j]);
                    if(j > 0 && j < view.size()-1)
                    {
                        potential_list.push_back(view[j+1]);
                        potential_list.push_back(view[j-1]);
                    }
                }
            }
        }

        //sprintf(plot, "%s%d%s", "Maps/Test_potential-", i, ".png");
        //plotObjects(plot, expect_info, potential_list);
        double max_length = 0;
        Object temp_ref;
        if(potential_list.size() == 1) //only one pair
        {
            if(distanceOftwoP(expect_info[i].midpoint(), temp_ref.midpoint()) < 1500)
            {
                match_pair.push_back(expect_info[i]);
                match_pair.push_back(potential_list[0]);
                all_match_pairs.push_back(match_pair);
                
                match_pair.clear();
            }
        }
        else
        {
            int temp_ref_flag = 0;
            for(int k = 0; k < potential_list.size(); k++) //more pairs are potential
            {
                    if((max_length < potential_list[k].length())
                       && (((potential_list[k].length() / expect_info[i].length()) >= 0.40) && ((potential_list[k].length() / expect_info[i].length()) <= 2.2)))
                    {
                        max_length = potential_list[k].length();
                        temp_ref = potential_list[k];
                        temp_ref_flag = 1;
                    }

            }
   
            if((distanceOftwoP(expect_info[i].midpoint(), temp_ref.midpoint()) < 1500) && (temp_ref_flag == 1))
            {
                match_pair.push_back(expect_info[i]);
                match_pair.push_back(temp_ref);
                all_match_pairs.push_back(match_pair);
            }
                match_pair.clear();
            
        }
        potential_list.clear();
    }
    
    sprintf(plot, "%s%d%s", "Maps/Test-", v, ".png");
    plotObjectsColours(plot, all_match_pairs);
    

    double max = 0;

    if(all_match_pairs.size() > 0)
    {
        
        //sort using distance with respect to robot
        //Object temp_exch;
        //vector< vector<Object> > sort_match_list;
        
        
        //choose a proper one as reference
        for(int j = 0; j < all_match_pairs.size(); j++)
        {
                if(j == 0)
                {
                    max = all_match_pairs[j][1].length();
                    reference_obj = all_match_pairs[j];
                }
                else
                {
                    if(max < all_match_pairs[j][1].length())
                    {
                        max = all_match_pairs[j][1].length();
                        reference_obj = all_match_pairs[j];
                    }
                }
        }
    }

    if(max > 0)
    {
        if((reference_obj[0].length() - reference_obj[1].length()) > 800)
        {
            int refer_endpoint;
            //triangulate/shift robot position on MFIS based on global information rather than current
            if(distanceOftwoP(reference_obj[1].getP1(), rb[6].getP1()) > distanceOftwoP(reference_obj[1].getP2(), rb[6].getP1()))
                refer_endpoint = 1;
            else
                refer_endpoint = 2;
            
            shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], refer_endpoint);
            adjust_view = projectingTheView_with_correctDirection(view, reference_obj[0], reference_obj[1], refer_endpoint);
        }
        else
        {
            shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 1);
            adjust_view = projectingTheView_with_correctDirection(view, reference_obj[0], reference_obj[1], 1);
        }
        
        temp_rnt.push_back(shift_position);
        temp_rnt.push_back(adjust_view);

    }
    else
    {
        temp_rnt.push_back(rb);
        temp_rnt.push_back(view);
    }
    //cout << "   max      : " << max << endl;
    //cout << " match list : " << all_match_pairs.size() << endl;
    sprintf(plot, "%s%d%s", "Maps/Match_pair_test-", v, ".png");
    plotObjectsOf4Kinds(plot, shift_position, adjust_view, expect_info, rb);
    
    return temp_rnt;
}


vector< vector<Object> > method_two_expectation(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> rest_info, int v, int& flag)
{
        char plotFile[80];
        MyRobot myrobot;
        
        
        int correct_flag = 0;
        int block_flag = 0;
        int match_flag = 0;
        int place_cross_flag = 0;
        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> remain_info;
        vector<Object> global_map;
        vector<Object> match_reference_view;
        vector< vector<Object> > match_list;
        vector< vector<Object> > temp_rnt;   //return adjust view, robot position & adjust gloal map
        pair< vector<Object>,  vector<Object> > temp; //for return 
        pair<int, int> ref_id;
        
        
        if(flag == 0)
        {
                //the first matching process
                //tracking information

                for(int i = 0; i < cv.size(); i++)
                {     

                        temp_match = Compare_similar_near_Obeject_(MFIS, cv[i], v);
                        if(temp_match.X1() != 0 && temp_match.X2() != 0)
                        {
                            matched_info.push_back(temp_match);
                            matched_info.push_back(cv[i]);

                            //MFIS[temp_match.getID()].set_match_flag(true);

                            match_list.push_back(matched_info);
                            matched_info.clear();
                        }

                }


back:           if(match_list.size() > 0)
                {
                        double max = 0;

                        //choose a proper one as reference
                        for(int j = 0; j < match_list.size(); j++)
                        {

                            //same size is the 
                            if((abs(match_list[j][0].length() - match_list[j][1].length()) <= 650.00)
                                && (match_list[j][1].length() >= 700))
                            {
                                if(j == 0) 
                                {
                                    max = match_list[j][1].length();
                                    reference_obj = match_list[j];
                                }
                                else
                                {
                                    if(max < match_list[j][1].length())
                                    {
                                        max = match_list[j][1].length();
                                        reference_obj = match_list[j];
                                    }
                                }

                            }
                        }


                        if(max > 0)
                        {
                            //triangulate/shift robot position on MFIS based on global information rather than current
                            shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                            adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);


                            //the adjustment reference is not reliable, which should be corrected
                            if(distanceOftwoP(shift_position[6].getP1(), rb[6].getP1()) > 500)
                            {
                                int end_flag = 0;

                                if(rb[6].Y2() > rb[6].Y1())
                                {
                                    if(((rb[6].Y2() - rb[6].Y1()) / (rb[6].X2() - rb[6].X1())) > 0)
                                            end_flag = 2;
                                    else
                                        if(((rb[6].Y2() - rb[6].Y1()) / (rb[6].X2() - rb[6].X1())) < 0)
                                            end_flag = 1;
                                }
                                else
                                    end_flag = 2;

                                shift_position = projectingTheView_with_correctDirection(rb, match_list[0][0], match_list[0][1], end_flag);
                                adjust_view = projectingTheView_with_correctDirection(cv, match_list[0][0], match_list[0][1], end_flag);
                                //modified that one in MFIS
                                if(place_cross_flag == 0)
                                {
                                    Object insert;
                                    for(int n = 0; n < MFIS.size(); n++)
                                    {
                                        if((MFIS[n].getP1() == reference_obj[0].getP1())
                                            && (MFIS[n].getP2() == reference_obj[0].getP2()))
                                        {

                                            //insert.set(MFIS[n-2].X2(), MFIS[n-2].Y2(), MFIS[n+1].X2(), MFIS[n+1].Y2(), n);
                                            MFIS[n].set(MFIS[n-2].X2(), MFIS[n-2].Y2(), MFIS[n+1].X2(), MFIS[n+1].Y2(), n);
                                            MFIS.erase(MFIS.begin()+n-1);
                                            break;
                                        }
                                    }


                                }
                            }


                            match_flag = 1;
                        }
                        else
                            match_flag = 0;


                        if(match_flag == 1)
                        {

                                for(int i = 0; i < MFIS.size(); i++)
                                {
                                    if((MFIS[i].get_match_label() > 1)
                                        || (MFIS[i].get_imagined_flag() == 2))
                                    {
                                        correct_flag = 1;
                                    }

                                    for(int n = 0; n < adjust_view.size(); n++)
                                    {
                                        Object connect_line;
                                        connect_line.set(shift_position[6].X1(), shift_position[6].Y1(), adjust_view[n].midpoint().X(), adjust_view[n].midpoint().Y(), 0);
                                            if((TwoObjectIntersect(connect_line, MFIS[i]) == true)
                                                && (MFIS[i].get_imagined_flag() != 1)
                                                && (shortestDistanceBtwTwoObjects(adjust_view[n], MFIS[i]) < 800))
                                            block_flag = 1;

                                    }

                                }

                                //using has been expected map
                                if((correct_flag == 1) && (block_flag == 1))
                                {
                                    global_map = Correct_detail(adjust_view, MFIS);
                                    global_map = Correct_detail_blocked(adjust_view, global_map, shift_position);
                                }
                                else
                                {
                                    if(correct_flag == 1)
                                    {
                                        global_map = Correct_detail(adjust_view, MFIS);
                                    }
                                    else
                                    {
                                        if(block_flag == 1)
                                        {
                                            global_map = Correct_detail_blocked(adjust_view, MFIS, shift_position);
                                        }
                                        else  
                                            global_map = MFIS;
                                    }
                                }       
                        }
                        else
                        {

                            if(place_cross_flag == 0)
                            {
                                match_reference_view = MFIS;
                                goto lb;
                            }
                            else
                            {
                                adjust_view = cv;
                                shift_position = rb;
                                global_map = MFIS;

                                vector<Object> theory_view = view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                                vector< vector<Object> > temp_adjust = shift_after_expect(theory_view, cv, rb, v);
                                adjust_view = temp_adjust[1];
                                shift_position = temp_adjust[0];
                            }
                        }


                }
                else
                {

lb:                 cout << " 1-place_cross_flag : " << place_cross_flag << endl;

                    if(place_cross_flag == 1)
                    {
                        match_reference_view.clear();
                        match_reference_view = rest_info;
                        //info_adjustment_next(cv, rb, match_reference_view, rest_info, iv);
                    }


                    for(int i = 0; i < cv.size(); i++)
                    {
                        if(cv[i].length() >= 500)
                        {
                            for(int t = 0; t < match_reference_view.size(); t++)
                            {

                                vector<Object> test1, test2;
                                test1.push_back(cv[i]);
                                test2.push_back(match_reference_view[t]);

                                if((((abs(cv[i].length() / match_reference_view[t].length()) > 0.60) && (abs(cv[i].length() / match_reference_view[t].length()) <= 1.00))
                                    || ((abs(cv[i].length() / match_reference_view[t].length()) >= 1.00) && (abs(cv[i].length() / match_reference_view[t].length()) < 1.38)))
                                    && (cv[i].length() > 700))
                                {

                                    Object obj1, obj2;
                                    obj1.set(rb[6].X1(), rb[6].Y1(), cv[i].midpoint().X(), cv[i].midpoint().Y(), 1);
                                    obj2.set(rb[6].X1(), rb[6].Y1(), match_reference_view[t].midpoint().X(), match_reference_view[t].midpoint().Y(), 1);

                                    if(((TwoObjectIntersect(obj1, match_reference_view[t]) == true) && (distanceOftwoP(cv[i].midpoint(), intersectPointTwolineEquations(obj1, match_reference_view[t])) < 800))
                                            || ((TwoObjectIntersect(obj2, cv[i]) == true) && (distanceOftwoP(match_reference_view[t].midpoint(), intersectPointTwolineEquations(obj2, cv[i])) < 800)))
                                    {

                                        if(place_cross_flag == 1)
                                            match_list.clear();

                                        matched_info.push_back(match_reference_view[t]);
                                        matched_info.push_back(cv[i]);

                                        //MFIS[temp_match.getID()].set_match_flag(true);

                                        match_list.push_back(matched_info);
                                        matched_info.clear();
                                        place_cross_flag = 1;
                                        //goto back;
                                    }
                                }
                            }
                        }
                    }

                    if(match_list.size() > 0)
                        goto back;

                    if(place_cross_flag == 1)
                    {
                        adjust_view = cv;
                        shift_position = rb;
                        global_map = MFIS;

                        vector<Object> theory_view = view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                        vector< vector<Object> > temp_adjust = shift_after_expect(theory_view, cv, rb, v);
                        adjust_view = temp_adjust[1];
                        shift_position = temp_adjust[0];
                    }
                    else
                    {
                        cout << " Trace back to match rest of part. " << endl;
                        place_cross_flag = 1;
                        goto lb;
                    }

                }

                if(place_cross_flag == 1)
                {
                    //global_map = addTwoVectorsOfObjects(global_map, rest_info);
                    flag = 1;
                }



                if(distanceOftwoP(shift_position[6].getP1(), rb[6].getP1()) > 1000)
                {
                    shift_position = rb;
                    adjust_view = cv;
                    //expect_process_flag = 0;
                    //global_map = addTwoVectorsOfObjects(MFIS, rest_info);
                }

                //return 3 things
                temp_rnt.push_back(global_map);
                temp_rnt.push_back(shift_position);
                temp_rnt.push_back(adjust_view);

                //return temp;
                return temp_rnt;
        }
        else
        {
                for(int i = 0; i < cv.size(); i++)
                {     

                        temp_match = Compare_similar_near_Obeject_(rest_info, cv[i], v);
                        if(temp_match.X1() != 0 && temp_match.X2() != 0)
                        {
                            matched_info.push_back(temp_match);
                            matched_info.push_back(cv[i]);

                            //MFIS[temp_match.getID()].set_match_flag(true);

                            match_list.push_back(matched_info);
                            matched_info.clear();
                        }

                }


back2:          if(match_list.size() > 0)
                {
                        double max = 0;
                        //choose a proper one as reference
                        for(int j = 0; j < match_list.size(); j++)
                        {

                            //same size is the 
                            if((abs(match_list[j][0].length() - match_list[j][1].length()) <= 650.00)
                                && (match_list[j][1].length() >= 700))
                            {
                                if(j == 0)
                                {
                                    max = match_list[j][1].length();
                                    reference_obj = match_list[j];
                                }
                                else
                                {
                                    if(max < match_list[j][1].length())
                                    {
                                        max = match_list[j][1].length();
                                        reference_obj = match_list[j];
                                    }
                                }

                            }
                        }

                        if(max > 0)
                        {
                            //triangulate/shift robot position on MFIS based on global information rather than current
                            shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                            adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);

                            match_flag = 1;
                        }
                        else
                            match_flag = 0;

                        if(match_flag == 1)
                        {

                                for(int i = 0; i < rest_info.size(); i++)
                                {
                                    if((rest_info[i].get_match_label() > 1)
                                        || (rest_info [i].get_imagined_flag() == 2))
                                    {
                                        correct_flag = 1;
                                    }

                                    for(int n = 0; n < adjust_view.size(); n++)
                                    {
                                        Object connect_line;
                                        connect_line.set(shift_position[6].X1(), shift_position[6].Y1(), adjust_view[n].midpoint().X(), adjust_view[n].midpoint().Y(), 0);
                                            if((TwoObjectIntersect(connect_line, rest_info[i]) == true)
                                                && (rest_info[i].get_imagined_flag() != 1)
                                                && (shortestDistanceBtwTwoObjects(adjust_view[n], rest_info[i]) < 800))
                                            block_flag = 1;

                                    }

                                }

                        }
                        else
                        {
                            if(place_cross_flag == 0)
                            {
                                match_reference_view = rest_info;
                                goto lb2;
                            }
                            else
                            {
                                if(place_cross_flag == 1)
                                {
                                    for(int j = 0; j < match_list.size(); j++)
                                    {

                                        //same size is the 
                                        if(((distanceOftwoP(match_list[j][0].getP1(), match_list[j][1].getP1()) < 600.00) 
                                                || (distanceOftwoP(match_list[j][0].getP2(), match_list[j][1].getP2()) < 600.00))
                                            && (match_list[j][1].length() >= 700))
                                        {
                                            if(j == 0)
                                            {
                                                max = match_list[j][1].length();
                                                reference_obj = match_list[j];
                                            }
                                            else
                                            {
                                                if(max < match_list[j][1].length())
                                                {
                                                    max = match_list[j][1].length();
                                                    reference_obj = match_list[j];
                                                }
                                            }

                                        }
                                    }

                                    if(max > 0)
                                    {
                                        if(distanceOftwoP(reference_obj[0].getP1(), reference_obj[1].getP1()) < 600.00) 
                                        {
                                            //triangulate/shift robot position on MFIS based on global information rather than current
                                            shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 1);
                                            adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 1);
                                        }
                                        else                                    
                                        {
                                            if(distanceOftwoP(reference_obj[0].getP2(), reference_obj[1].getP2()) < 600.00) 
                                            {
                                                //triangulate/shift robot position on MFIS based on global information rather than current
                                                shift_position = projectingTheView_with_correctDirection(rb, reference_obj[0], reference_obj[1], 2);
                                                adjust_view = projectingTheView_with_correctDirection(cv, reference_obj[0], reference_obj[1], 2);
                                            }
                                        }
                                    }
                                }
                            }
                        }


                }
                else
                {

lb2:                cout << " 2-place_cross_flag : " << place_cross_flag << endl;

                    if(place_cross_flag == 1)
                    {
                        match_reference_view.clear();
                        match_reference_view = MFIS;
                    }


                    for(int i = 0; i < cv.size(); i++)
                    {
                        if(cv[i].length() >= 500)
                        {
                            for(int t = 0; t < match_reference_view.size(); t++)
                            {
                                if((((abs(cv[i].length() / match_reference_view[t].length()) > 0.60) && (abs(cv[i].length() / match_reference_view[t].length()) <= 1.00))
                                    || ((abs(cv[i].length() / match_reference_view[t].length()) >= 1.00) && (abs(cv[i].length() / match_reference_view[t].length()) < 1.38)))
                                    && (cv[i].length() > 700))
                                {

                                    Object obj1, obj2;
                                    obj1.set(rb[6].X1(), rb[6].Y1(), cv[i].midpoint().X(), cv[i].midpoint().Y(), 1);
                                    obj2.set(rb[6].X1(), rb[6].Y1(), match_reference_view[t].midpoint().X(), match_reference_view[t].midpoint().Y(), 1);
                                    if(((TwoObjectIntersect(obj1, match_reference_view[t]) == true) && (distanceOftwoP(cv[i].midpoint(), intersectPointTwolineEquations(obj1, match_reference_view[t])) < 800))
                                            || ((TwoObjectIntersect(obj2, cv[i]) == true) && (distanceOftwoP(match_reference_view[t].midpoint(), intersectPointTwolineEquations(obj2, cv[i])) < 800)))
                                    {


                                        if(place_cross_flag == 1)
                                            match_list.clear();

                                        matched_info.push_back(match_reference_view[t]);
                                        matched_info.push_back(cv[i]);

                                        //MFIS[temp_match.getID()].set_match_flag(true);

                                        match_list.push_back(matched_info);
                                        matched_info.clear();
                                        place_cross_flag = 1;
                                        goto back2;
                                    }
                                }
                            }
                        }
                    }

                    if(place_cross_flag == 1)
                    {
                        adjust_view = cv;
                        shift_position = rb;
                        global_map = MFIS;
                        //view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                        vector<Object> theory_view = view_in_theory(addTwoVectorsOfObjects(MFIS, rest_info), cv, rb, v);
                        vector< vector<Object> > temp_adjust = shift_after_expect(theory_view, cv, rb, v);
                        adjust_view = temp_adjust[1];
                        shift_position = temp_adjust[0];
                    }
                    else
                    {
                        cout << " Trace back to match rest of part. " << endl;
                        place_cross_flag = 1;
                        goto lb;
                    }

                }

                if(place_cross_flag == 1)
                {
                    flag = 0;
                }

                if(distanceOftwoP(shift_position[6].getP1(), rb[6].getP1()) > 1000)
                {
                    shift_position = rb;
                    adjust_view = cv;
                    //expect_process_flag = 0;
                    //global_map = addTwoVectorsOfObjects(MFIS, rest_info);
                }

                //return 3 things
                temp_rnt.push_back(MFIS);
                temp_rnt.push_back(shift_position);
                temp_rnt.push_back(adjust_view);

                //return temp;
                return temp_rnt;
        }
        
        
}


bool Expecting_with_global_map(vector<Object> robot_position, vector<Object> currentview, vector<Object> MFIS, int v)
{
    //return if it needs to swith to method2(expectation process)

    vector<Object> match_reference_view;
    
    //compute theoritical view
    match_reference_view = view_in_theory(MFIS, currentview, robot_position, v);
    
    for(int i = 0; i < currentview.size(); i++)
    {
        if(currentview[i].length() >= 500)
        {
            for(int t = 0; t < match_reference_view.size(); t++)
            {

                if((((abs(currentview[i].length() / match_reference_view[t].length()) > 0.60) && (abs(currentview[i].length() / match_reference_view[t].length()) <= 1.00))
                    || ((abs(currentview[i].length() / match_reference_view[t].length()) >= 1.00) && (abs(currentview[i].length() / match_reference_view[t].length()) < 1.38)))
                    && (currentview[i].length() > 700))
                {

                    Object obj1, obj2;
                    obj1.set(robot_position[6].X1(), robot_position[6].Y1(), currentview[i].midpoint().X(), currentview[i].midpoint().Y(), 1);
                    obj2.set(robot_position[6].X1(), robot_position[6].Y1(), match_reference_view[t].midpoint().X(), match_reference_view[t].midpoint().Y(), 1);

                    if(((TwoObjectIntersect(obj1, match_reference_view[t]) == true) && (distanceOftwoP(currentview[i].midpoint(), intersectPointTwolineEquations(obj1, match_reference_view[t])) < 800))
                            || ((TwoObjectIntersect(obj2, currentview[i]) == true) && (distanceOftwoP(match_reference_view[t].midpoint(), intersectPointTwolineEquations(obj2, currentview[i])) < 800)))
                    {
                        //return true;
                        if((v - match_reference_view[t].getVN() > 8) && (match_reference_view[t].getVN() > 0))
                            return true;
                    }
                }
            }
        }
    }
    
    return false;
}

pair< vector<Object>,  vector<Object> > Expecting_and_Modify(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> expect_information, int v)
{
        char plotFile[80];
        MyRobot myrobot;
        int match_flag = 0;
        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> remain_info;
        vector<Object> global_map;
        vector< vector<Object> > match_list;
        pair< vector<Object>,  vector<Object> > temp; //for return 
        pair<int, int> ref_id;


        for(int i = 0; i < cv.size(); i++)
        {     

                temp_match = Compare_similar_near_Obeject_(MFIS, cv[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(cv[i]);
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }
        }
        
        if(match_list.size() > 0)
        {
            
            //choose a proper one as reference
            for(int j = 0; j < match_list.size(); j++)
            {
                //same size is the 
                if(abs(match_list[j][0].length() - match_list[j][1].length()) < 300.00)
                {
                    reference_obj = match_list[j];
                    //triangulate/shift robot position on MFIS based on global information rather than current
                    shift_position = projectingTheView(rb, reference_obj[0], reference_obj[1], 2);
                    adjust_view = projectingTheView(cv, reference_obj[0], reference_obj[1], 2);
                    
                    for(int v = 0; v < adjust_view.size(); v++)
                    {
                        adjust_view[v].set_match_flag(cv[v].get_match_flag());
                        adjust_view[v].set_match_label(cv[v].get_match_label());
                    }

                    
                    match_flag = 1;
                    break;
                }
            }

            if(match_flag == 1 && cv.size() > 2)
            {
                    remain_info = Fill_operation_with_ASR(adjust_view, cv, MFIS, match_list, reference_obj);
                    remain_info = Correct_detail(adjust_view, remain_info);
                    global_map = remain_info;

                    sprintf(plotFile, "%s%d%s", "Maps/Offline/Expectation_info_and_position-", v, ".png");
                    plotObjectsOf3Kinds(plotFile, shift_position, adjust_view, remain_info);
            }
            else
            {
                adjust_view = cv;
                shift_position = rb;
                global_map = expect_information;
            }
            

        }
        else
        {
            adjust_view = cv;
            shift_position = rb;
            global_map = expect_information;
        }

        
        
        temp.first = global_map;
        temp.second = shift_position;
        
        return temp;
}

vector<Object> Expect_reference(vector<Object> robot_position, vector<Object> currentview, vector<Object> MFIS)
{
    
    vector<Object> reference_obj;
    vector<Object> matched_info;
    vector<Object> match_reference_view;
    vector< vector<Object> > match_list;
    //compute theoritical view
    match_reference_view = view_in_theory(MFIS, currentview, robot_position, 0);
    
    for(int i = 0; i < currentview.size(); i++)
    {
        if(currentview[i].length() >= 500)
        {
            for(int t = 0; t < match_reference_view.size(); t++)
            {

                if((((abs(currentview[i].length() / match_reference_view[t].length()) > 0.60) && (abs(currentview[i].length() / match_reference_view[t].length()) <= 1.00))
                    || ((abs(currentview[i].length() / match_reference_view[t].length()) >= 1.00) && (abs(currentview[i].length() / match_reference_view[t].length()) < 1.38)))
                    && (currentview[i].length() > 700))
                {

                    Object obj1, obj2;
                    obj1.set(robot_position[6].X1(), robot_position[6].Y1(), currentview[i].midpoint().X(), currentview[i].midpoint().Y(), 1);
                    obj2.set(robot_position[6].X1(), robot_position[6].Y1(), match_reference_view[t].midpoint().X(), match_reference_view[t].midpoint().Y(), 1);

                    if(((TwoObjectIntersect(obj1, match_reference_view[t]) == true) && (distanceOftwoP(currentview[i].midpoint(), intersectPointTwolineEquations(obj1, match_reference_view[t])) < 800))
                            || ((TwoObjectIntersect(obj2, currentview[i]) == true) && (distanceOftwoP(match_reference_view[t].midpoint(), intersectPointTwolineEquations(obj2, currentview[i])) < 800)))
                    {
                        matched_info.push_back(match_reference_view[t]);
                        matched_info.push_back(currentview[i]);
                        match_list.push_back(matched_info);
                        matched_info.clear();
                    }
                }
            }
        }
    }
    
    double max = 0;
    for(int j = 0; j < match_list.size(); j++)
    {

        //same size is the 
        if((abs(match_list[j][0].length() - match_list[j][1].length()) <= 650.00)
            && (match_list[j][1].length() >= 700))
        {
            if(j == 0)
            {
                max = match_list[j][1].length();
                reference_obj = match_list[j];
            }
            else
            {
                if(max < match_list[j][1].length())
                {
                    max = match_list[j][1].length();
                    reference_obj = match_list[j];
                }
            }
        }
    }

    
    return reference_obj;
}

vector< vector<Object> > Expect_function(vector<Object> reference_obj, vector<Object> robot_position, 
                                         vector<Object> currentview, vector<Object> environment, int v)
{
    int pi_flag = 0;
    vector<Object> shift_position, adjust_view;
    vector< vector<Object> > temp;

    //triangulate/shift robot position on MFIS based on global information rather than current
    shift_position = projectingTheView_with_correctDirection(robot_position, reference_obj[0], reference_obj[1], 2);

    adjust_view = projectingTheView_with_correctDirection(currentview, reference_obj[0], reference_obj[1], 2);
    
    //the adjustment reference is not reliable, which should be corrected
    if(distanceOftwoP(shift_position[6].getP1(), robot_position[6].getP1()) > 500)
    {
        int end_flag = 0;

        if(robot_position[6].Y2() > robot_position[6].Y1())
        {
            if(((robot_position[6].Y2() - robot_position[6].Y1()) / (robot_position[6].X2() - robot_position[6].X1())) > 0)
                    end_flag = 2;
            else
                if(((robot_position[6].Y2() - robot_position[6].Y1()) / (robot_position[6].X2() - robot_position[6].X1())) < 0)
                    end_flag = 1;
        }
        else
            end_flag = 2;
   

        shift_position = projectingTheView_with_correctDirection(robot_position, reference_obj[0], reference_obj[1], end_flag);
        adjust_view = projectingTheView_with_correctDirection(currentview, reference_obj[0], reference_obj[1], end_flag);
        
        if(distanceOftwoP(shift_position[6].getP1(), robot_position[6].getP1()) > 500)
        {
            shift_position = robot_position;
            adjust_view = currentview;
            pi_flag = 1;
        }
    }
    

    if((pi_flag == 0) && (v > 64))
    {
        Object check_ref, correspond_in_map;
        vector<Object> temp_potential_list;
        //if there are other large surfaces, using them to check
        for(int i = 0; i < currentview.size(); i++)
        {
            if((currentview[i].length() > 2000) //there is another large surface
                && ((currentview[i].getP1() != reference_obj[1].getP1())
                || (currentview[i].getP2() != reference_obj[1].getP2())))
            {
                check_ref = adjust_view[currentview[i].getID()-1];
                break;
            }
        }
        
        if((check_ref.getP1().X() != NULL) && (check_ref.getP1().Y() != NULL))
        {

            for(int i = 0; i < environment.size(); i++)
            {
                if((environment[i].length() > 2000)
                    && (parallel_objects(environment[i], check_ref) == true))
                {
                    temp_potential_list.push_back(environment[i]);
                }
            }

            if(temp_potential_list.size() != 0)
            {

                for(int i = 0; i < temp_potential_list.size(); i++)
                {
                    
                    double temp_k1, temp_k2, temp_k3, temp_k4;
                    temp_k1 = (check_ref.Y1() - shift_position[6].Y1()) / (check_ref.X1() - shift_position[6].X1());
                    temp_k2 = (check_ref.Y2() - shift_position[6].Y1()) / (check_ref.X2() - shift_position[6].X1());
                    temp_k3 = (temp_potential_list[i].Y1() - shift_position[6].Y1()) / (temp_potential_list[i].X1() - shift_position[6].X1());
                    temp_k4 = (temp_potential_list[i].Y2() - shift_position[6].Y1()) / (temp_potential_list[i].X2() - shift_position[6].X1());
                    /*cout << "k1 : " << temp_k1 << " ; k2 : " << temp_k2 << endl;
                    cout << "k3 : " << temp_k3 << " ; k4 : " << temp_k4 << endl;
                    cout << "p1 to p1 : " <<distanceOftwoP(check_ref.getP1(), temp_potential_list[i].getP1())<<endl;
                    cout << "p2 to p2 : " <<distanceOftwoP(check_ref.getP2(), temp_potential_list[i].getP2())<<endl;
                    waitHere();*/
                         
                    //if(((isLeft(shift_position[6].getP1(), shift_position[6].getP2(), check_ref.getP1()) * isLeft(shift_position[6].getP1(), shift_position[6].getP2(), temp_potential_list[i].getP1())) > 0)
                    // && ((isLeft(shift_position[6].getP1(), shift_position[6].getP2(), check_ref.getP2()) * isLeft(shift_position[6].getP1(), shift_position[6].getP2(), temp_potential_list[i].getP2())) > 0)
                    //        && ((distanceOftwoP(check_ref.getP1(), temp_potential_list[i].getP1()) < 1500) || (distanceOftwoP(check_ref.getP2(), temp_potential_list[i].getP2()) < 1500)))
                    if((distanceOftwoP(check_ref.getP1(), temp_potential_list[i].getP1()) < 1500) || (distanceOftwoP(check_ref.getP2(), temp_potential_list[i].getP2()) < 1500)
                            && ((temp_k1 * temp_k3) > 0) && ((temp_k2 * temp_k4) > 0))
                    {
                        correspond_in_map = temp_potential_list[i];
                        double diff_dist = P_To_ShortestDistance(check_ref.getP1(),correspond_in_map);
                        int shift_flag = 0;
                        
                        if(isLeft(check_ref.getP1(), check_ref.getP2(), correspond_in_map.getP1()) > 0)
                            shift_flag = 1;
                        else
                            shift_flag = 2;
                        
                        shift_objects(shift_position, diff_dist, shift_flag);
                        shift_objects(adjust_view, diff_dist, shift_flag);

                        break;
                    }

                }
            }
        }
    }
     
    temp.push_back(shift_position);
    temp.push_back(adjust_view);
    
    return temp;
}



vector<Object> Fill_operation_with_ASR_modifiyVersion(vector<Object> view, vector<Object> cv, vector<Object> global_map, 
                                       vector< vector<Object> > match_list, vector<Object> references)
{
        char plotFile[80];
        vector<int> match_ASRs;
        vector<Object> info_collect;
        vector<Object> coll_typ1, coll_typ2, coll_type3;

        vector<Object> temp;

        for(int i = 0; i < match_list.size(); i++)
        {
            match_ASRs.push_back(match_list[i][0].getVN());
        }

        for(int i = 0; i < match_ASRs.size(); i++)
        {
            for (int j = 0; j < match_ASRs.size() - i; j++) 
            {
                if (match_ASRs[j] > match_ASRs[j + 1]) 
                {
                    int temp = match_ASRs[j];
                    match_ASRs[j] = match_ASRs[j + 1];
                    match_ASRs[j + 1] = temp;
                }
            }
        }
        
        
        /* collect info need to re-aligned */
        // 1st type, matched info

        for(int i = 0; i < match_list.size(); i++)
        {
            //coll_typ1.push_back(match_list[i][0]);
            
            if(match_list[i][1].length() != references[1].length())
            {
                temp.push_back(match_list[i][1]);
                coll_typ1.push_back(match_list[i][0]);
            }
        }
        
        
        for(int i = 0; i < global_map.size(); i++)
        {
            if(Intersect_number(coll_typ1,global_map[i]) >= 2)
            {
                temp.push_back(global_map[i]);
                coll_typ1.push_back(global_map[i]);
            }
        }
        

        remaining = deletOneViewbaseLength(global_map, coll_typ1);
        temp = projectingTheView(temp, references[0], references[1], 2);
        
        Correct_detail_modifiyVersion(temp, match_list);
        
        //sprintf(plotFile, "%s", "Maps/Offline/TESTTESTTEST2.png");
        //plotObjectsColours(plotFile, match_list, temp, global_map);

        //mark all those surfaces
        /*for(int i = 0; i < temp.size(); i++)
        {
            if((info_collect[i].get_match_label() > 1)
                && (info_collect[i].get_imagined_flag() != 1))
            {
                temp[i].set_match_flag(info_collect[i].get_match_flag());
                temp[i].set_match_label(info_collect[i].get_match_label());
                temp[i].set_imagined_flag(2); //not imagined 
            }
            else
            {
                temp[i].set_match_flag(info_collect[i].get_match_flag());
                temp[i].set_match_label(info_collect[i].get_match_label());
                temp[i].set_imagined_flag(2); //imagined
            }
        }*/
            
        return temp;
}

void Correct_detail_modifiyVersion(vector<Object>& view, vector< vector<Object> > match_list)
{
    cout <<" This detail correct position process"  << endl;
    
    
    Object temp_modi;
    vector<Object> temp_global_map;
    double dist1, dist2;
    Point p1,p2;
    

    for(int i = 0; i < view.size(); i++)
    {
            int flag = 0;
            Object correspond_obj;
            //modified this piece of info
            for(int j = 0; j < match_list.size(); j++)
            {
                if(abs(view[i].length() - match_list[j][1].length()) < 10e-6)
                {
                    correspond_obj = match_list[j][0];
                    //match_list[j][0].set_imagined_flag(2);
                    correspond_obj.display();
                    match_list[j][0].display();
                    flag = 1;
                    break;
                }
            }
            
            if(flag == 1)
            {
                //expend correspond object
                if(distanceOftwoP(view[i].getP1(), correspond_obj.getP1()) < 300)
                {

                      dist1 = view[i].length() - correspond_obj.length();
                      temp_modi = expend_Object(view[i], abs(dist1), 2);
                      temp_modi.set(temp_modi.X2(), temp_modi.Y2(), view[i].X1(), view[i].Y1(), view[i].getID());

                      temp_modi.set_imagined_flag(1);
                      view.erase(view.begin()+i);
                      view.insert(view.begin()+i, temp_modi);
                }
                else
                {
                    
                    if(distanceOftwoP(view[i].getP2(), correspond_obj.getP2()) < 300)
                    {

                        dist2 =  view[i].length() - correspond_obj.length();
                        temp_modi = expend_Object(view[i], abs(dist2), 1);
                        temp_modi.set(view[i].X2(), view[i].Y2(), temp_modi.X1(), temp_modi.Y1(),  view[i].getID());
                        
                        temp_modi.set_imagined_flag(1);
                        view.erase(view.begin()+i);
                        view.insert(view.begin()+i, temp_modi);
                    }
                    else 
                    {
                        
                        p1 = crossPerpend(correspond_obj.getP1(), correspond_obj.getP2(), view[i].getP1());
                        p2 = crossPerpend(correspond_obj.getP1(), correspond_obj.getP2(), view[i].getP2());
                        dist1 = distanceOftwoP(p1, correspond_obj.getP1());
                        dist2 = distanceOftwoP(p2, correspond_obj.getP2());
                        //dist1 = distanceOftwoP(view[i].getP1(), correspond_obj.getP1());
                        //dist2 =   distanceOftwoP(view[i].getP2(), correspond_obj.getP2());

                        temp_modi = expend_Object_two_sides(view[i], dist1, dist2);
                        temp_modi.set_imagined_flag(1);
                        
                        view.erase(view.begin()+i);
                        view.insert(view.begin()+i, temp_modi);
                    }
                }
                //temp_global_map[i].set_imagined_flag(1);
            }
               // waitHere();

    }
    
    /*for(int i = 0; i < global_map.size(); i++)
    {
        if(global_map[i].get_imagined_flag() == 1)
            temp_global_map[i].set_imagined_flag(1);
        
        if((global_map[i].get_imagined_flag() == 2) && (temp_global_map[i].get_imagined_flag() != 1))
            temp_global_map[i].set_imagined_flag(2);
    }*/
    
    //return temp_global_map;
}

pair< vector<Object>,  vector<Object> > info_first_expect(vector<Object> cv, vector<Object> rb,  
                                    vector<Object> MFIS, vector<Object> expect_information, int v)
{
        char plotFile[80];
        MyRobot myrobot;
        int match_flag = 0;
        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> remain_info;
        vector<Object> global_map;
        vector< vector<Object> > match_list;
        pair< vector<Object>,  vector<Object> > temp; //for return 
        pair<int, int> ref_id;
        //the first matching process
        //tracking information

        for(int i = 0; i < cv.size(); i++)
        {     

                temp_match = Compare_similar_near_Obeject_(MFIS, cv[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(cv[i]);

                    //MFIS[temp_match.getID()].set_match_flag(true);
                    
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }

        }
        sprintf(plotFile, "%s%d%s", "Maps/Offline/Test_matched_list", v, ".png");
        plotObjectsColours(plotFile, match_list, MFIS, rb);
                    
        if(match_list.size() > 0)
        {
            
                //choose a proper one as reference
                for(int j = 0; j < match_list.size(); j++)
                {
                    //same size is the 
                    if(abs(match_list[j][0].length() - match_list[j][1].length()) < 300.00)
                    {
                        reference_obj = match_list[j];
                        //triangulate/shift robot position on MFIS based on global information rather than current
                        shift_position = projectingTheView(rb, reference_obj[0], reference_obj[1], 2);
                        adjust_view = projectingTheView(cv, reference_obj[0], reference_obj[1], 2);

                        for(int v = 0; v < adjust_view.size(); v++)
                        {
                            adjust_view[v].set_match_flag(cv[v].get_match_flag());
                            adjust_view[v].set_match_label(cv[v].get_match_label());
                        }

                        match_flag = 1;
                        break;
                    }
                }

                if(match_flag == 1 && cv.size() > 2)
                {
                    
                    if(expect_process_flag == 0)//first time to activate expectation process
                    {
                       
                        //remain_info = Fill_operation_with_ASR(adjust_view, cv, MFIS, match_list, reference_obj);
                        remain_info = Fill_operation_with_ASR_modifiyVersion(adjust_view, cv, MFIS, match_list, reference_obj);
                        //remain_info = Correct_detail(adjust_view, remain_info);
                        global_map = remain_info;

                        sprintf(plotFile, "%s%d%s", "Maps/Offline/Expectation_info_and_position-", v, ".png");
                        plotObjectsOf3Kinds(plotFile, shift_position, adjust_view, remain_info);
                        
                        expect_process_flag = 1;
                    }
                    /*else
                    {
                        for(int i = 0; i < adjust_view.size(); i++)
                        {     
                            //match between adjust view and expected global map
                            Compare_similar_near_Obeject_(expect_information, adjust_view[i], v);
                        }
                        
                        int correct_flag = 0;
                        int block_flag = 0;
                        vector<Object> piece_adjusted;
                        for(int i = 0; i < expect_information.size(); i++)
                        {
                            if((expect_information[i].get_match_label() > 1)
                                || (expect_information[i].get_imagined_flag() == 2))
                            {
                                correct_flag = 1;
                            }
                            
                            for(int n = 0; n < adjust_view.size(); n++)
                            {
                                    if((TwoObjectIntersect(adjust_view[n], expect_information[i]) == true)
                                        && (expect_information[i].get_imagined_flag() != 1)
                                        && (shortestDistanceBtwTwoObjects(adjust_view[n], expect_information[i]) < 500))
                                    block_flag = 1;

                            }

                        }
  
                        //using has been expected map
                        if((correct_flag == 1) && (block_flag == 1))
                        {
                            piece_adjusted = Correct_detail(adjust_view, expect_information);
                            global_map = piece_adjusted;
                        }
                        else
                        {
                            if(correct_flag == 1)
                            {
                                piece_adjusted = Correct_detail(adjust_view, expect_information);
                                global_map = piece_adjusted;
                            }
                            else
                            {
                                    global_map = expect_information;
                            }
                        }
                        
                        sprintf(plotFile, "%s%d%s", "Maps/Offline/Exist_expectation-", v, ".png");
                        plotObjects(plotFile, adjust_view, global_map);
                    }*/
                }
                else
                {
                    adjust_view = cv;
                    shift_position = rb;
                    global_map = expect_information;
                }

        }
        else
        {
            adjust_view = cv;
            shift_position = rb;
            global_map = expect_information;
        }

        temp.first = global_map;
        temp.second = shift_position;
        
        return temp;
}

pair< vector<Object>,  vector<Object> > info_first_expect_modify(vector<Object> currentview, vector<Object> robot_position,  
                                     vector<Object> environment_map, int v)
{
        char plotFile[80];

        int match_flag = 0;
        Object temp_match;
        vector<Object> shift_position;   //adjust robot position
        vector<Object> matched_info;     //match object
        vector<Object> reference_obj;    //ref object for adjustment
        vector<Object> adjust_view;      //adjust view
        vector<Object> remain_info;
        vector<Object> global_map;
        vector< vector<Object> > match_list;
        pair< vector<Object>,  vector<Object> > temp; //for return 
        pair<int, int> ref_id;
        //the first matching process
        //tracking information
        
        global_map = environment_map;

        for(int i = 0; i < currentview.size(); i++)
        {     

                //temp_match = Compare_similar_near_Obeject_(environment_map, currentview[i], v);
                temp_match = Compare_similar_near_Obeject_modify(environment_map, currentview[i], v);
                if(temp_match.X1() != 0 && temp_match.X2() != 0)
                {
                    matched_info.push_back(temp_match);
                    matched_info.push_back(currentview[i]);

                    //MFIS[temp_match.getID()].set_match_flag(true);
                    
                    match_list.push_back(matched_info);
                    matched_info.clear();
                }

        }
        //sprintf(plotFile, "%s%d%s", "Maps/Offline/Test_matched_list", v, ".png");
        //plotObjectsColours(plotFile, match_list, environment_map, robot_position);
                    
        //if(match_list.size() > 0)
        //{
            
                /*//choose a proper one as reference
                for(int j = 0; j < match_list.size(); j++)
                {
                    //same size is the 
                    if(abs(match_list[j][0].length() - match_list[j][1].length()) < 300.00)
                    {
                        reference_obj = match_list[j];
                        //triangulate/shift robot position on MFIS based on global information rather than current
                        shift_position = projectingTheView(robot_position, reference_obj[0], reference_obj[1], 2);
                        adjust_view = projectingTheView(currentview, reference_obj[0], reference_obj[1], 2);

                        for(int v = 0; v < adjust_view.size(); v++)
                        {
                            adjust_view[v].set_match_flag(currentview[v].get_match_flag());
                            adjust_view[v].set_match_label(currentview[v].get_match_label());
                        }

                        match_flag = 1;
                        break;
                    }
                }

                if(match_flag == 1 && currentview.size() > 2)
                {
                    
                    if(expect_process_flag == 0)//first time to activate expectation process
                    {
                       
                        remain_info = Fill_operation_with_ASR_modifiyVersion(adjust_view, currentview, environment_map, match_list, reference_obj);
                        global_map = remain_info;

                        sprintf(plotFile, "%s%d%s", "Maps/Offline/Expectation_info_and_position-", v, ".png");
                        plotObjectsOf3Kinds(plotFile, shift_position, adjust_view, remain_info);
                        
                        expect_process_flag = 1;
                    }
                }
                else
                {
                    adjust_view = currentview;
                    shift_position = robot_position;
                    global_map = environment_map;
                }*/
                //cout << " test programme " << endl;
                Object join_line;
                vector<Object> polygonObjects;
                vector<Surface> polygon;
                ClipperLib::Path subj;
                Paths solution;
                ClipperOffset co;

                subj = viewConvertPath(currentview); //convert view into Path
                co.AddPath(subj, jtMiter, etClosedPolygon);
                co.Execute(solution, 100.0); // outward offset the polygon
                polygonObjects = PathsConvertView(solution); // convert polygon into view
                polygon = convertObjectToSurface(polygonObjects); // convert view into surfaces
                for(int j = 0; j < global_map.size(); j++)
                {
                    join_line.set(robot_position[6].X1(), robot_position[6].Y1(), global_map[j].midpoint().X(), global_map[j].midpoint().Y(), j);
                    Object temp_detect = intersectForObject(currentview, join_line.getP1(), join_line.getP2());
                    
                    if(((pointInPolygon(PointXY(global_map[j].X1(), global_map[j].Y1()), polygon) == true) 
                            || (pointInPolygon(PointXY(global_map[j].X2(), global_map[j].Y2()), polygon) == true)
                            || (isThisCloseToCVPolygonBoundary(global_map[j], polygonObjects, 600.0) == true))
                        && ((temp_detect.length() < 700) || (temp_detect.X1() == 0)))
                    {
                        global_map.erase(global_map.begin()+j);
                        j--;
                    }
                }
                
                global_map = addTwoVectorsOfObjects(global_map, currentview);
                shift_position = robot_position;
                //sprintf(plotFile, "%s%d%s", "Maps/Offline/Test_matched_list*", v, ".png");
                //plotObjects(plotFile, global_map, robot_position);  
                //waitHere();

        //}
        //else
        //{
            adjust_view = currentview;
        //    shift_position = robot_position;
        //    global_map = environment_map;
        //}
        expect_process_flag = 1;
        temp.first = global_map;
        temp.second = shift_position;
    
    return temp;
}

bool switch_back_premethod(vector<Object> MFIS, vector<Object> view)
{
    int cunter_inter = 0, cunter_inside = 0;
    
    vector<Surface> polygon;
    vector<Object>  _polygon;
    
    polygon = makePolygonOfCV(view);
    _polygon= convertSurfaceToObject(polygon);
    
    for(int i = 0; i < _polygon.size(); i++)
    {
        cunter_inter = cunter_inter + Intersect_number(MFIS, _polygon[i]);
    }
    

    for(int i = 0; i < MFIS.size(); i++)
    {
        if((pointInPolygon(PointXY(MFIS[i].X1(), MFIS[i].Y1()), polygon) == true)
                && (pointInPolygon(PointXY(MFIS[i].X2(), MFIS[i].Y2()), polygon) == true))
        {
            cunter_inside += 1;
        }
    }
    
    if((cunter_inter < 3) && (cunter_inside < 3) && (view.size() > 4))
        return true;
    else
        return false;
}

int critical_point_switch(vector<Object> currentRb, vector< vector<Object> > all_robots)
{
    cout<< "searching critical point"<<endl;
    int rnt;
    double k1, k2;
    double temp, distance;
    vector< vector<Object> > sort;
    
    
    k1 = (currentRb[6].Y2() - currentRb[6].Y1()) / (currentRb[6].X2() - currentRb[6].X1());
    
    for(int i = all_robots.size()-2; i > 0; i--)
    {
        k2 = (all_robots[i][6].Y2() - all_robots[i][6].Y1()) / (all_robots[i][6].X2() - all_robots[i][6].X1());
        //cout << "angles are   : " << includedAngle(currentRb[6], all_robots[i][6]) << endl;
        cout << "angles-2 are : " << includedAngle(currentRb[6].getP1(), currentRb[6].getP2(), all_robots[i][6].getP1(), all_robots[i][6].getP2()) << endl;
        
        //if((includedAngle(currentRb[6], all_robots[i][6]) > 165) && ((k1 * k2) < 0))
        if((includedAngle(currentRb[6].getP1(), currentRb[6].getP2(), all_robots[i][6].getP1(), all_robots[i][6].getP2()) > 165) //&& ((k1 * k2) < 0)
                && (distanceOftwoP(currentRb[6].getP1(), all_robots[i][6].getP1()) < 1000))
        {
            rnt = all_robots[i][6].getVN();
            return rnt;
        }
    }
    
    
    return 0;
}


vector<Object> compute_map_without_deleiton(vector< vector<Object> > views_on_global, vector< vector<Object> > robots_on_global)
{
    char plotFile[90];
    vector<Object> diff_format_map, robot_path;
    
    
    for(int i = 0; i < views_on_global.size(); i++)
    {
        if(i == 0)
        {
            diff_format_map = views_on_global[i];
            robot_path = robots_on_global[i];
        }
        else
        {
            diff_format_map = addTwoVectorsOfObjects(diff_format_map, views_on_global[i]);
            robot_path = addTwoVectorsOfObjects(robot_path, robots_on_global[i]);
        }
        sprintf(plotFile, "%s%d%s", "Maps/Offline/diff_format_map_and_robot_path-", i, ".png");
        plotObjects(plotFile, diff_format_map, robot_path);
    }
}

vector<Object> compute_map_without_deleiton(vector< vector<Object> > views_on_global, vector< vector<Object> > robots_on_global,
                                            vector<int> numbers, int start, int end)
{
    char plotFile[90];
    
    int i = 0; 
    vector<Object> diff_format_map, robot_path;
    
    
    while(numbers[i] >= start && numbers[i] <= end)
    {
        if(i == 0)
        {
            diff_format_map = views_on_global[i];
            robot_path = robots_on_global[i];
        }
        else
        {
            diff_format_map = addTwoVectorsOfObjects(diff_format_map, views_on_global[i]);
            robot_path = addTwoVectorsOfObjects(robot_path, robots_on_global[i]);
        }
        //sprintf(plotFile, "%s%d%s", "Maps/Offline/diff_format_map_and_robot_path-", i, ".png");
        //plotObjects(plotFile, diff_format_map, robot_path);
        
        i++;
    }
    
    vector<Object> map_informaiton;
    vector<Object> each_group;
    vector< vector<Object> > clusters;
    
    for(int i = 0; i < diff_format_map.size(); i++)
    {
        each_group.push_back(diff_format_map[i]);
        for(int j = i; j < diff_format_map.size(); j++)
        {
            if(distanceOftwoP(diff_format_map[i].midpoint(), diff_format_map[j].midpoint()) < 500)
            {
                each_group.push_back(diff_format_map[j]);
            }
        }
        clusters.push_back(each_group);
        each_group.clear();
    }
    
    double max_length = 0;
    Object largest_object;
    for(int i = 0; i < clusters.size(); i++)
    {
        for(int j = 0; j < clusters[i].size(); j++)
        {
            if(j == 0)
            {
                max_length = clusters[i][j].length();
                largest_object = clusters[i][j];
            }
            else
            {
                if(max_length < clusters[i][j].length())
                {
                    max_length = clusters[i][j].length();
                    largest_object = clusters[i][j];
                }
            }
        }
        map_informaiton.push_back(largest_object);
    }
    
    sprintf(plotFile, "%s", "Maps/Offline/diff_format_map_and_robot_path-.png");
    plotObjects(plotFile, map_informaiton, robot_path);
    
    return diff_format_map;
}


vector<Object> clear_and_update_env(vector<Object> env, vector<Object> currentview, vector<Object> robot_position)
{
        
        /*Object join_line;
        vector<Object> global_map;
        vector<Object> polygonObjects;
        vector<Surface> polygon;
        
        global_map = env;
        
        ClipperLib::Path subj;
        Paths solution;
        ClipperOffset co;

        subj = viewConvertPath(currentview); //convert view into Path
        co.AddPath(subj, jtMiter, etClosedPolygon);
        co.Execute(solution, 0.0); // outward offset the polygon
        polygonObjects = PathsConvertView(solution); // convert polygon into view
        polygon = convertObjectToSurface(polygonObjects); // convert view into surfaces
        for(int j = 0; j < global_map.size(); j++)
        {
            join_line.set(robot_position[6].X1(), robot_position[6].Y1(), global_map[j].midpoint().X(), global_map[j].midpoint().Y(), j);
            Object temp_detect = intersectForObject(currentview, join_line.getP1(), join_line.getP2());

            if(((pointInPolygon(PointXY(global_map[j].X1(), global_map[j].Y1()), polygon) == true) 
                    && (pointInPolygon(PointXY(global_map[j].X2(), global_map[j].Y2()), polygon) == true))
                && ((temp_detect.length() < 700) || (temp_detect.X1() == 0)))
            {
                global_map.erase(global_map.begin()+j);
                j--;
            }
        }

        global_map = addTwoVectorsOfObjects(global_map, currentview);
        */
        
        vector<Object> temp_rnt;
        vector<Object> global_map, remain_info;
        vector<Object> match_group;
        vector< vector<Object> > list_of_match_group;
        
        double dist_threshold = 500;
        double distp1, distp2, distmid, avg_dist;
        double include_angle = 0;
        
        global_map = env;
        
        for(int j = 0; j < currentview.size(); j++)
        {
                match_group.push_back(currentview[j]);
            
                for(int i = 0; i < global_map.size(); i++)
                {
                    distp1 = distanceOftwoP(currentview[j].getP1(), global_map[i].getP1());
                    distp2 = distanceOftwoP(currentview[j].getP2(), global_map[i].getP2());
                    distmid = distanceOftwoP(currentview[j].midpoint(), global_map[i].midpoint());
                    avg_dist = (distp1 + distp2 + distmid)/3;
                    
                    include_angle = includedAngle(currentview[j], global_map[i]);
                            
                    if((avg_dist < dist_threshold) && (global_map[i].get_match_flag() != 1))
                    {
                        match_group.push_back(global_map[i]);
                        global_map[i].set_match_flag(1);
                    }
                }
                
                list_of_match_group.push_back(match_group);
                match_group.clear();  
        }
        
        for(int i = 0; i < global_map.size(); i++)
        {
            if(global_map[i].get_match_flag() != 1)
                remain_info.push_back(global_map[i]);
        }
        
        
        double max_length = 0;
        Object max_object;
        
        for(int i = 0; i < list_of_match_group.size(); i++)
        {
            for(int j = 0; j < list_of_match_group[i].size(); j++)
            {
                if(j == 0)
                {
                    max_length = list_of_match_group[i][j].length();
                    max_object = list_of_match_group[i][j];
                }
                else
                {
                    if(max_length < list_of_match_group[i][j].length())
                    {
                        max_length = list_of_match_group[i][j].length();
                        max_object = list_of_match_group[i][j];
                    }
                }
            }
            
            temp_rnt.push_back(max_object);
        }
        
        for(int i = 0; i < temp_rnt.size(); i++)
        {
            for(int j = i + 1; j < temp_rnt.size(); j++)
            {
                double expAngle = temp_rnt[i].getAngleWithLine(temp_rnt[j]);
                double expDist = shortestDistanceBtwTwoObjects(temp_rnt[i], temp_rnt[j]);
                if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && (expDist < 300.0))
                {
                    if(temp_rnt[i].length() > temp_rnt[j].length())
                    {
                        temp_rnt.erase(temp_rnt.begin()+j);
                        j--;
                    }
                    else
                    {
                        temp_rnt.erase(temp_rnt.begin()+i);
                        i--;
                        break;
                    }
                }
            }
        }

//        char plotFile[80];
//        sprintf(plotFile, "%s", "Maps/cominbed_parts.png");                   
//        plotObjectsOf3Kinds(plotFile, robot_position, temp_rnt, remain_info);
//        cout << " test programme !!!!!! " << endl;
//        waitHere();
        temp_rnt = addTwoVectorsOfObjects(temp_rnt, remain_info);
            
        return temp_rnt;
}


vector<Object> clear_and_update_env_with_boundary(vector<Object> env, vector<Object> currentview, 
                                                  vector<Object> robot_position, vector<Object> boundary)
{
        
        Object join_obj1, join_obj2;
        vector<Object> temp_rnt;
        vector<Object> global_map, remain_info;
        vector<Object> match_group;
        vector< vector<Object> > list_of_match_group;
        
        double dist_threshold = 500;
        double distp1, distp2, distmid, avg_dist;
        double include_angle = 0;
        
        global_map = env;
        
        for(int j = 0; j < currentview.size(); j++)
        {
            join_obj1.set(robot_position[6].X1(), robot_position[6].Y1(), currentview[j].X1(), currentview[j].Y1(), 0);
            join_obj2.set(robot_position[6].X1(), robot_position[6].Y1(), currentview[j].X2(), currentview[j].Y2(), 0);
            
            //if(((interSectWithLine(boundary, join_obj1.getP1(), join_obj1.getP2()) == false)
            //    && (interSectWithLine(boundary, join_obj2.getP1(), join_obj2.getP2()) == false)))
                //|| ((distanceOftwoP(intersectedNearPoint(boundary, join_obj1.getP1(), join_obj1.getP2()), currentview[j].getP1()) < 800) && (distanceOftwoP(intersectedNearPoint(boundary, join_obj2.getP1(), join_obj2.getP2()), currentview[j].getP2()) < 800)))
            //{
                match_group.push_back(currentview[j]);
            
                for(int i = 0; i < global_map.size(); i++)
                {
                    distp1 = distanceOftwoP(currentview[j].getP1(), global_map[i].getP1());
                    distp2 = distanceOftwoP(currentview[j].getP2(), global_map[i].getP2());
                    distmid = distanceOftwoP(currentview[j].midpoint(), global_map[i].midpoint());
                    avg_dist = (distp1 + distp2 + distmid)/3;
                    
                    include_angle = includedAngle(currentview[j], global_map[i]);
                            
                    if((avg_dist < dist_threshold) && (global_map[i].get_match_flag() != 1))
                    {
                        match_group.push_back(global_map[i]);
                        global_map[i].set_match_flag(1);
                    }
                }
                
                list_of_match_group.push_back(match_group);
                match_group.clear();
            //}
        }
        
        for(int i = 0; i < global_map.size(); i++)
        {
            if(global_map[i].get_match_flag() != 1)
                remain_info.push_back(global_map[i]);
        }
        
        
        double max_length = 0;
        Object max_object;
        
        for(int i = 0; i < list_of_match_group.size(); i++)
        {
            for(int j = 0; j < list_of_match_group[i].size(); j++)
            {
                if(j == 0)
                {
                    max_length = list_of_match_group[i][j].length();
                    max_object = list_of_match_group[i][j];
                }
                else
                {
                    if(max_length < list_of_match_group[i][j].length())
                    {
                        max_length = list_of_match_group[i][j].length();
                        max_object = list_of_match_group[i][j];
                    }
                }
            }
            
            temp_rnt.push_back(max_object);
        }
        
        for(int i = 0; i < temp_rnt.size(); i++)
        {
            for(int j = i + 1; j < temp_rnt.size(); j++)
            {
                double expAngle = temp_rnt[i].getAngleWithLine(temp_rnt[j]);
                double expDist = shortestDistanceBtwTwoObjects(temp_rnt[i], temp_rnt[j]);
                if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && (expDist < 300.0))
                {
                    if(temp_rnt[i].length() > temp_rnt[j].length())
                    {
                        temp_rnt.erase(temp_rnt.begin()+j);
                        j--;
                    }
                    else
                    {
                        temp_rnt.erase(temp_rnt.begin()+i);
                        i--;
                        break;
                    }
                }
            }
        }

//        char plotFile[80];
//        sprintf(plotFile, "%s", "Maps/cominbed_parts.png");                   
//        plotObjectsOf3Kinds(plotFile, robot_position, temp_rnt, remain_info);
//        cout << " test programme !!!!!! " << endl;
//        waitHere();
        temp_rnt = addTwoVectorsOfObjects(temp_rnt, remain_info);
            
        return temp_rnt;
}


vector<Object> clear_map(vector<Object> map)
{
    vector<Object> temp_rnt; 
    temp_rnt = map;
        for(int i = 0; i < temp_rnt.size(); i++)
        {
            for(int j = i + 1; j < temp_rnt.size(); j++)
            {
                double expAngle = temp_rnt[i].getAngleWithLine(temp_rnt[j]);
                double expDist = shortestDistanceBtwTwoObjects(temp_rnt[i], temp_rnt[j]);
                if ((abs(expAngle) < 11.0 || abs(expAngle) > 349.0) && (expDist < 300.0))
                {
                    if(temp_rnt[i].length() > temp_rnt[j].length())
                    {
                        temp_rnt.erase(temp_rnt.begin()+j);
                        j--;
                    }
                    else
                    {
                        temp_rnt.erase(temp_rnt.begin()+i);
                        i--;
                        break;
                    }
                }
            }
        }
    
    return temp_rnt;
}








