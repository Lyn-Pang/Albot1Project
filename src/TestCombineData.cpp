/*
 * Function: Combine some information for testing
 *           and analysing data
 * 
 * author: Lyn Pang
 */


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Point.H"
#include "Plotting.H"
#include "Object.H"
#include "mfisOp.H"
#include "ChunksOp.H"
#include "GeometricOp.H"
#include "GeometryFuncs.H"
#include "PointAndSurface.H"
#include "ToolFunctions.h"
#include "PathPlanning.H"
#include "readAndwriteASCII.H"
#include "GeometryAndExit.h"
#include "Ellipse.h"
#include "Exit.h"
#include "TestCombineData.h"
#include "FunctionOfMFIS.h"
#include "Returning.h"

using namespace std;


void Test_cominbe_ASR(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles)
{
    vector< vector<Object> > MFISs; 
    vector< vector<Object> > combined1, combined2,combined3,combined4,combined5;
    vector< vector<Object> > combined6, combined7,combined8,combined9,combined10;
    vector< vector<Object> > region1, region2,region3,region4,region5;
    vector< vector<Object> > region6, region7,region8,region9,region10;
    vector< vector<Object> > robot1, robot2,robot3,robot4,robot5;
    vector< vector<Object> > robot6, robot7,robot8,robot9,robot10;
    vector<Object> combined_1,combined_2,combined_3,combined_4,combined_5;
    vector<Object> combined_6,combined_7,combined_8,combined_9,combined_10;
    vector<Object> mfis1, mfis2, mfis3, mfis4, mfis5, mfis6, mfis7, mfis8, mfis9, mfis10;
    vector<Ellipse> circles1, circles2, circles3, circles4, circles5;
    vector<Ellipse> circles6, circles7, circles8, circles9, circles10;
    
    /*
    char testfileName[80];
    for(int i = 0; i < views_chunk.size(); i++)
    {
        sprintf(testfileName, "%s%d%s", "Maps/Offline/Global_ASR", i+1,".png");
        plotObjectsOf3Kinds(testfileName, views_chunk[i], robot_positions[i], robot_positions[i+1]);
    }
    */
    
        /*
        cout << " robot steps are : ";
        for(int m = 0; m < labels.size(); m++)
            cout << labels[m] << " ";
        
        cout << endl << endl;
        cout << " view numbers are : ";
        for(int m = 0; m < view_numbers.size(); m++)
            cout << view_numbers[m] << " ";
        waitHere();
        */
   
    for(int i = 0; i < views_chunk.size(); i++)
    {
        if(i >= 0 && i <= 2)
        {
            combined1.push_back(views_chunk[i]);
            robot1.push_back(robot_positions[i]);
            circles1.push_back(space_circles[i]);
        }
            
        if(i >= 2 && i <= 8)
        {
            combined2.push_back(views_chunk[i]);
            robot2.push_back(robot_positions[i]);
            circles2.push_back(space_circles[i]);
        }
            
        if(i >= 8 && i <= 10)
        {
            combined3.push_back(views_chunk[i]);
            robot3.push_back(robot_positions[i]);
            circles3.push_back(space_circles[i]);
        } 
            
        if(i >= 10 && i <= 13)
        {
            combined4.push_back(views_chunk[i]);
            robot4.push_back(robot_positions[i]);
            circles4.push_back(space_circles[i]);
        }  
        
        if(i >= 13 && i <= 23)
        {
            combined5.push_back(views_chunk[i]);
            robot5.push_back(robot_positions[i]);
            circles5.push_back(space_circles[i]);
        }    
                        
        if(i >= 23 && i <= 27)
        {
            combined6.push_back(views_chunk[i]);
            robot6.push_back(robot_positions[i]);
            circles6.push_back(space_circles[i]);
        }
            
        if(i >= 27 && i <= 29)
        {
            combined7.push_back(views_chunk[i]);
            robot7.push_back(robot_positions[i]);
            circles7.push_back(space_circles[i]);
        }  
            
        if(i >= 29 && i <= 33)
        {
            combined8.push_back(views_chunk[i]);
            robot8.push_back(robot_positions[i]);
            circles8.push_back(space_circles[i]);
        } 
            
        if(i >= 33 && i <= 36)
        {
            combined9.push_back(views_chunk[i]);
            robot9.push_back(robot_positions[i]);
            circles9.push_back(space_circles[i]);
        }   
            
        if(i >= 36 && i <= 40)
        {
            combined10.push_back(views_chunk[i]);
            robot10.push_back(robot_positions[i]);
            circles10.push_back(space_circles[i]);
        }    
            
    }
    //cout << " size of 1 : " << combined1.size() << endl;
    for(int i = 0; i < combined1.size()-1; i++)
    {
        combined_1 = addTwoVectorsOfObjects(combined_1, combined1[i]);
    }
    
    region1.push_back(combined_1);
    region1.push_back(combined1.back());

    //cout << " size of 2 : " << combined2.size() << endl;
    for(int i = 0; i < combined2.size()-1; i++)
    {
        combined_2 = addTwoVectorsOfObjects(combined_2, combined2[i]);
    }
    
    region2.push_back(combined_2);
    region2.push_back(combined2.back());

    //cout << " size of 3 : " << combined3.size() << endl;
    for(int i = 0; i < combined3.size()-1; i++)
    {
        combined_3 = addTwoVectorsOfObjects(combined_3, combined3[i]);
    }
    
    region3.push_back(combined_3);
    region3.push_back(combined3.back());

    //cout << " size of 4 : " << combined4.size() << endl;
    for(int i = 0; i < combined4.size()-1; i++)
    {
        combined_4 = addTwoVectorsOfObjects(combined_4, combined4[i]);
    }
    
    region4.push_back(combined_4);
    region4.push_back(combined4.back());

    //cout << " size of 5 : " << combined5.size() << endl;
    for(int i = 0; i < combined5.size()-1; i++)
    {
        combined_5 = addTwoVectorsOfObjects(combined_5, combined5[i]);
    }
    
    region5.push_back(combined_5);
    region5.push_back(combined5.back());

    //cout << " size of 6 : " << combined6.size() << endl;
    for(int i = 0; i < combined6.size()-1; i++)
    {
        combined_6 = addTwoVectorsOfObjects(combined_6, combined6[i]);
    }
    
    region6.push_back(combined_6);
    region6.push_back(combined6.back());

    //cout << " size of 7 : " << combined7.size() << endl;
    for(int i = 0; i < combined7.size()-1; i++)
    {
        combined_7 = addTwoVectorsOfObjects(combined_7, combined7[i]);
    }
    //cout << " Test Program " << endl;
    region7.push_back(combined_7);
    region7.push_back(combined7.back());

    //cout << " size of 8 : " << combined8.size() << endl;
    for(int i = 0; i < combined8.size()-1; i++)
    {
        combined_8 = addTwoVectorsOfObjects(combined_8, combined8[i]);
    }
    
    region8.push_back(combined_8);
    region8.push_back(combined8.back());

    //cout << " size of 9 : " << combined9.size() << endl;
    for(int i = 0; i < combined9.size()-1; i++)
    {
        combined_9 = addTwoVectorsOfObjects(combined_9, combined9[i]);
    }
    
    region9.push_back(combined_9);
    region9.push_back(combined9.back());

    //cout << " size of 10 : " << combined10.size() << endl;
    for(int i = 0; i < combined10.size()-1; i++)
    {
        combined_10 = addTwoVectorsOfObjects(combined_10, combined10[i]);
    }
    
    region10.push_back(combined_10);
    region10.push_back(combined10.back());    
    
    /*
    for(int i = 0; i < MFIS.size(); i++)
    {
        if(MFIS[i].getVN() <= 3)
            mfis1.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 13)
            mfis2.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 15)
            mfis3.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 22)
            mfis4.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 32)
            mfis5.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 36)
            mfis6.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 39)
            mfis7.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 48)
            mfis8.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 56)
            mfis9.push_back(MFIS[i]);
        if(MFIS[i].getVN() <= 68)
            mfis10.push_back(MFIS[i]);
    }
    */
   
    char fileName[80];
   /* sprintf(fileName, "%s", "Maps/Offline/Test_data1.png");
    plotObjectsColours(fileName, region1);
    sprintf(fileName, "%s", "Maps/Offline/Test_data2.png");
    plotObjectsColours(fileName, region2);
    sprintf(fileName, "%s", "Maps/Offline/Test_data3.png");
    plotObjectsColours(fileName, region3);
    sprintf(fileName, "%s", "Maps/Offline/Test_data4.png");
    plotObjectsColours(fileName, region4);
    sprintf(fileName, "%s", "Maps/Offline/Test_data5.png");
    plotObjectsColours(fileName, region5);
    sprintf(fileName, "%s", "Maps/Offline/Test_data6.png");
    plotObjectsColours(fileName, region6);
    sprintf(fileName, "%s", "Maps/Offline/Test_data7.png");
    plotObjectsColours(fileName, region7);
    sprintf(fileName, "%s", "Maps/Offline/Test_data8.png");
    plotObjectsColours(fileName, region8);
    sprintf(fileName, "%s", "Maps/Offline/Test_data9.png");
    plotObjectsColours(fileName, region9);
    sprintf(fileName, "%s", "Maps/Offline/Test_data10.png");
    plotObjectsColours(fileName, region10);
    */
    
    mfis1 = FunctionOfMFIS(5, 3, 1, 3);
    
    mfis2 = FunctionOfMFIS(5, 3, 3, 13);
    mfis2 = TransformforToGlobalCoordinate(mfis2, Point(robot_positions[2][6].X1(), robot_positions[2][6].Y1()), 
                                              robot_positions[2], robot_positions[2][7].getAngleWithXaxis());
    
    mfis3 = FunctionOfMFIS(5, 3, 13, 15);
    mfis3 = TransformforToGlobalCoordinate(mfis3, Point(robot_positions[8][6].X1(), robot_positions[8][6].Y1()), 
                                              robot_positions[8], robot_positions[8][7].getAngleWithXaxis());
    
    mfis4 = FunctionOfMFIS(5, 3, 15, 22);
    mfis4 = TransformforToGlobalCoordinate(mfis4, Point(robot_positions[10][6].X1(), robot_positions[10][6].Y1()), 
                                              robot_positions[10], robot_positions[10][7].getAngleWithXaxis());
    
    mfis5 = FunctionOfMFIS(5, 3, 22, 33);
    mfis5 = TransformforToGlobalCoordinate(mfis5, Point(robot_positions[13][6].X1(), robot_positions[13][6].Y1()), 
                                              robot_positions[13], robot_positions[13][7].getAngleWithXaxis());

    mfis6 = FunctionOfMFIS(5, 3, 33, 36);
    mfis6 = TransformforToGlobalCoordinate(mfis6, Point(robot_positions[23][6].X1(), robot_positions[23][6].Y1()), 
                                              robot_positions[23], robot_positions[23][7].getAngleWithXaxis());
    mfis7 = FunctionOfMFIS(5, 3, 36, 39);
    mfis7 = TransformforToGlobalCoordinate(mfis7, Point(robot_positions[27][6].X1(), robot_positions[27][6].Y1()), 
                                              robot_positions[27], robot_positions[27][7].getAngleWithXaxis());
    mfis7[0].set_region_remove_flag(1);
    
    mfis8 = FunctionOfMFIS(5, 3, 39, 48);
    /*
    cout << " @ 39 , angle : " << robot_positions[29][7].getAngleWithXaxis() << endl;
    cout << " position x : " << robot_positions[29][6].X1()
            << " ; y : " << robot_positions[29][6].Y1() << endl;
    cout << " @ 40 , angle : " << robot_positions[30][7].getAngleWithXaxis() << endl;
    cout << " position x : " << robot_positions[30][6].X1()
            << " ; y : " << robot_positions[30][6].Y1() << endl;*/
    
    mfis8 = TransformforToGlobalCoordinate(mfis8, Point(robot_positions[29][6].X1(), robot_positions[29][6].Y1()), 
                                              robot_positions[29], robot_positions[29][7].getAngleWithXaxis());
    
    mfis9 = FunctionOfMFIS(5, 3, 48, 56);
    mfis9 = TransformforToGlobalCoordinate(mfis9, Point(robot_positions[33][6].X1(), robot_positions[33][6].Y1()), 
                                              robot_positions[33], robot_positions[33][7].getAngleWithXaxis());
    
    mfis10 = FunctionOfMFIS(5, 3, 56, 67);
    mfis10 = TransformforToGlobalCoordinate(mfis10, Point(robot_positions[36][6].X1(), robot_positions[36][6].Y1()), 
                                                robot_positions[36], robot_positions[36][7].getAngleWithXaxis());
    
    MFISs.push_back(mfis1);
    MFISs.push_back(mfis2);
    MFISs.push_back(mfis3);
    MFISs.push_back(mfis4);
    MFISs.push_back(mfis5);
    MFISs.push_back(mfis6);
    MFISs.push_back(mfis7);
    MFISs.push_back(mfis8);
    MFISs.push_back(mfis9);
    MFISs.push_back(mfis10);
    /*
    vector< vector<Object> > test_part;
    vector<Object> test_rb;
    test_part.push_back(views_chunk[5]);
    test_part.push_back(views_chunk[6]);
    test_part.push_back(views_chunk[7]);
    test_rb= addTwoVectorsOfObjects(test_rb, robot_positions[5]);
    test_rb= addTwoVectorsOfObjects(test_rb, robot_positions[6]);
    test_rb= addTwoVectorsOfObjects(test_rb, robot_positions[7]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis1_withTEST.png");
    plotObjectsColours(fileName, test_part, test_rb, mfis2);
    
    
    sprintf(fileName, "%s", "Maps/Offline/region_mfis1.png");
    plotObjectsOf3Kinds(fileName, mfis1, robot_positions[0], robot_positions[2]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis2.png");
    plotObjectsOf3Kinds(fileName, mfis2, robot_positions[2], robot_positions[8]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis3.png");
    plotObjectsOf3Kinds(fileName, mfis3, robot_positions[8], robot_positions[10]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis4.png");
    plotObjectsOf3Kinds(fileName, mfis4, robot_positions[10], robot_positions[13]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis5.png"); 
    plotObjectsOf3Kinds(fileName, mfis5, robot_positions[13], robot_positions[23]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis6.png");
    plotObjectsOf3Kinds(fileName, mfis6, robot_positions[23], robot_positions[27]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis7.png");
    plotObjectsOf3Kinds(fileName, mfis7, robot_positions[27], robot_positions[29]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis8.png");
    plotObjectsOf3Kinds(fileName, mfis8, robot_positions[29], robot_positions[33]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis9.png");
    plotObjectsOf3Kinds(fileName, mfis9, robot_positions[33], robot_positions[36]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis10.png");
    plotObjectsOf3Kinds(fileName, mfis10, robot_positions[36], robot_positions[37]);
    */
    /*
    pair< vector<Object>, vector<Object> > boundary_view;
    boundary_view.first = region10[0];
    boundary_view.second = region10[1];
    
    int p;
    p = 10 - 1;
    
    //structualExit(boundary_view,boundary_view);
    pair< vector<Object>, vector<Object> >  isolate = SlpitOverlappingRegion(boundary_view);
    //PotentialExits(isolate);
    /*
    Object imagine_exit;
    imagine_exit.set(boundary_view.second[0].X1(), boundary_view.second[0].Y1(), 
                     boundary_view.second.back().X2(), boundary_view.second.back().Y2(), 0);
    
    Potential_Exit(isolate, Path[0], imagine_exit);
    */
    
    //PerpendicularExits(boundary_view.first , Path[0]);
    //vector<Object> simplified_region = BoundaryByGroup(boundary_view.first);
    //vector<Object> simplified_region = BoundaryByExits(boundary_view.first);
    
    //Simplify_Clipper(boundary_view.first);
    //sprintf(fileName, "%s", "Maps/Offline/Simplified_view.png");
    //plotObjects(fileName, simplified_region, boundary_view.first);
    /*
    pair< vector<Object>, vector<Object> > left_And_right;
    vector<Object> left, right;
    vector<Exit> crossed_lines;
    vector<Object>  space_block;
    Exit imagine_exit;
    //imagine_exit.set(-2000, 0, 2000, 0);    //first step 
    imagine_exit.set(region9[1][0].X1(), region9[1][0].Y1(), 
                     region9[1].back().X2(), region9[1].back().Y2());
    
    left_And_right = LeftAndRightList(isolate.first, Path[p]);
    
    left = Trim_group(left_And_right.first, Path[p]);
    right = Trim_group(left_And_right.second, Path[p]);
    
    
    sprintf(fileName, "%s", "Maps/Offline/before_trim.png");
    plotObjects(fileName, left_And_right.first, left_And_right.second);
    
    sprintf(fileName, "%s", "Maps/Offline/trim_region.png");
    plotObjects(fileName, left, right);
    
    //
    crossed_lines = Short_between_groups(left_And_right.first, left_And_right.second);
    
    sprintf(fileName, "%s", "Maps/Offline/crossed_lines.png");
    plotObjectsOf3KindswithExits(fileName, left_And_right.first, left_And_right.second, crossed_lines);
    
    //
    vector<Exit> filter_exits = ConstrainListOfExits(crossed_lines, Point (Path[p].X2(), Path[p].Y2()));
    
    sprintf(fileName, "%s", "Maps/Offline/filtered_exits.png");
    plotObjectsOf3KindswithExits(fileName, left_And_right.first, left_And_right.second, filter_exits);
    
    //
    space_block = BuildAreaOfSpace(isolate.first, filter_exits, Path[p], imagine_exit);
    
    sprintf(fileName, "%s", "Maps/Offline/Space_block.png");
    plotObjects(fileName, space_block, isolate.first);
     * */
    
    Exit imagine_exit1,imagine_exit2, imagine_exit3, imagine_exit4, imagine_exit5;   
    Exit imagine_exit6,imagine_exit7, imagine_exit8, imagine_exit9, imagine_exit10, imagine_exit11;
    
    
    imagine_exit1.set(-2000, 0, 2000, 0);    //first step 
    //imagine_exit1.set(region1[0][0].X1(), region1[0][0].Y1(), 
    //                 region1[0].back().X2(), region1[0].back().Y2());
                     
    imagine_exit2.set(region1[1][0].X1(), region1[1][0].Y1(), 
                     region1[1].back().X2(), region1[1].back().Y2());
    
    imagine_exit3.set(region2[1][0].X1(), region2[1][0].Y1(), 
                     region2[1].back().X2(), region2[1].back().Y2());
    
    imagine_exit4.set(region3[1][0].X1(), region3[1][0].Y1(), 
                     region3[1].back().X2(), region3[1].back().Y2());
    
    imagine_exit5.set(region4[1][0].X1(), region4[1][0].Y1(), 
                     region4[1].back().X2(), region4[1].back().Y2());
    
    imagine_exit6.set(region5[1][0].X1(), region5[1][0].Y1(), 
                     region5[1].back().X2(), region5[1].back().Y2());
    
    imagine_exit7.set(region6[1][0].X1(), region6[1][0].Y1(), 
                     region6[1].back().X2(), region6[1].back().Y2());
    
    imagine_exit8.set(region7[1][0].X1(), region7[1][0].Y1(), 
                     region7[1].back().X2(), region7[1].back().Y2()); 
    
    imagine_exit9.set(region8[1][0].X1(), region8[1][0].Y1(), 
                     region8[1].back().X2(), region8[1].back().Y2());
    
    imagine_exit10.set(region9[1][0].X1(), region9[1][0].Y1(), 
                     region9[1].back().X2(), region9[1].back().Y2());
    
    imagine_exit11.set(region10[1][0].X1(), region10[1][0].Y1(), 
                     region10[1].back().X2(), region10[1].back().Y2());
    
    //--------------------------------------//
    //Test_combine_regions(region5, region6, mfis5, mfis6, Path, robot_positions[23], robot_positions[27], imagine_exit5, imagine_exit6, imagine_exit7);
    
    //--------------------------------------//
    //Test_cross_exit(region9, views_chunk[36], mfis9, Path, robot_positions[36],
    //                      robot_positions[38], imagine_exit1, imagine_exit2);
    
    //ExitInbetweenRobots(mfis10, robot10);
    
    /*--------------------------------------*/
    vector<Object> path_space, structure;
    vector<Object> path_space2, structure2;
    vector<Object> path_space3, structure3;
    vector<Object> path_space4, structure4;
    vector<Object> path_space5, structure5;
    vector<Object> path_space6, structure6;
    vector<Object> path_space7, structure7;
    vector<Object> path_space8, structure8;
    vector<Object> path_space9, structure9;
    vector<Object> path_space10, structure10;
    
    
    pair< vector<Object>, vector<Object> > extension1, extension2, extension3, extension4, extension5;
    pair< vector<Object>, vector<Object> > extension6, extension7, extension8, extension9, extension10;
    
    path_space = pathInRegion_UseCircles(circles1, region1[0], imagine_exit1, imagine_exit2, Path[1-1]);
    //structure = structureRegion(mfis8, path_space);
    //trend_boundary = RegionBaseLines(structure, path_space, Path[8-1]);
    //structure_boundary(mfis6, path_space, imagine_exit7, Path[6-1]); // compute useful information for structure description
    //Struct_Connect(mfis1, path_space);
    //structure = Struct_Connect(mfis8, path_space, Path[8-1], imagine_exit9, 8);
    structure = Struct_Connect(mfis1, path_space, Path[1-1], imagine_exit7, imagine_exit2, 1);
    
    path_space2 = pathInRegion_UseCircles(circles2, region2[0], imagine_exit2, imagine_exit3, Path[2-1]);
    structure2 = Struct_Connect(mfis2, path_space2, Path[2-1], imagine_exit7, imagine_exit3, 2);
    
    extension1 = Combine_origin_extend_regions(structure, structure2, mfis1);
    
    path_space3 = pathInRegion_UseCircles(circles3, region3[0], imagine_exit3, imagine_exit4, Path[3-1]);
    structure3 = Struct_Connect(mfis3, path_space3, Path[3-1], imagine_exit7, imagine_exit4, 3);
    
    extension2 = Combine_origin_extend_regions(structure2, structure3, mfis2);
    
    path_space4 = pathInRegion_UseCircles(circles4, region4[0], imagine_exit4, imagine_exit5, Path[4-1]);
    structure4 = Struct_Connect(mfis4, path_space4, Path[4-1], imagine_exit7, imagine_exit5, 4);
    
    extension3 = Combine_origin_extend_regions(structure3, structure4, mfis3);
    
    path_space5 = pathInRegion_UseCircles(circles5, region5[0], imagine_exit5, imagine_exit6, Path[5-1]);
    structure5 = Struct_Connect(mfis5, path_space5, Path[5-1], imagine_exit7, imagine_exit6, 5);
    
    extension4 = Combine_origin_extend_regions(structure4, structure5, mfis4);
    
    path_space6 = pathInRegion_UseCircles(circles6, region6[0], imagine_exit6, imagine_exit7, Path[6-1]);
    structure6 = Struct_Connect(mfis6, path_space6, Path[6-1], imagine_exit7, imagine_exit7, 6);
    
    extension5 = Combine_origin_extend_regions(structure5, structure6, mfis5);
    
    //path_space7 = pathInRegion_UseCircles(circles7, region7[0], imagine_exit7, imagine_exit8, Path[7-1]);
    //structure7 = Struct_Connect(mfis7, path_space7, Path[7-1], imagine_exit7, imagine_exit8, 7);
    
    //Combine_origin_extend_regions(vector<Object> structure, vector<Object> structure2,
    //                                         vector<Object> mfis1, vector<Object> mfis2)
    
    path_space8 = pathInRegion_UseCircles(circles8, region8[0], imagine_exit8, imagine_exit9, Path[8-1]);
    structure8 = Struct_Connect(mfis8, path_space8, Path[8-1], imagine_exit7, imagine_exit9, 8);
    
    extension6 = Combine_origin_extend_regions(structure6, structure8, mfis6);
    
    path_space9 = pathInRegion_UseCircles(circles9, region9[0], imagine_exit9, imagine_exit10, Path[9-1]);
    structure9 = Struct_Connect(mfis9, path_space9, Path[9-1], imagine_exit7, imagine_exit10, 9);
    
    extension7 = Combine_origin_extend_regions(structure8, structure9, mfis8);
    
    path_space10 = pathInRegion_UseCircles(circles10, region10[0], imagine_exit10, imagine_exit11, Path[10-1]);
    structure10 = Struct_Connect(mfis10, path_space10, Path[10-1], imagine_exit7, imagine_exit11, 10);
    
    extension8 = Combine_origin_extend_regions(structure9, structure10, mfis10);
    
    /*
    int connect_flag = 0;
    int flag_extend = 0;
    double extend_value = 100;
    Exit connection_regions;
    vector<Exit> connections;
    vector<Object> region_block1, region_block2;
    
    //extend two adjacent region blocks
    for(int i = 0; i < structure.size(); i++)
    {
        if(interSectWithLine(structure2, structure[i].getP1(), structure[i].getP2()) == true)
        {
            connect_flag = 1;
            break;
        }
        else
            connect_flag = 2;
    }
    */
    /*
    if(connect_flag == 2)
    {
        cout << " Two blocks are NOT intersected ." << endl;
 
lb:     trend_boundary = _extendRegionBlock(mfis9, structure, extend_value, 1); //extend exit
        trend_boundary2 = _extendRegionBlock(mfis10, structure2, extend_value, 2); //extend entrance
        
        for(int i = 0; i < trend_boundary.size(); i++)
        {
            if(interSectWithLine(trend_boundary2, trend_boundary[i].getP1(), trend_boundary[i].getP2()) == true)
            {
                flag_extend = 1;
                break;
            }
            else
                flag_extend = 0;
                

        }
        
        if(flag_extend == 0)
        {
            extend_value += 200;
            goto lb;
        }
        
        structure = addTwoVectorsOfObjects(structure, structure2);
        sprintf(fileName, "%s", "Maps/Offline/Extend_two_regions.png");
        //plotObjectsOf3Kinds(fileName, mfis8, trend_boundary, trend_boundary2);
        plotObjectsOf4Kinds(fileName, mfis9, trend_boundary, trend_boundary2, structure);
        
    }
    else
    {
        cout << " Two blocks are intersected ." << endl;
        sprintf(fileName, "%s", "Maps/Offline/Extend_two_regions.png");
        plotObjectsOf3Kinds(fileName, mfis9, structure, structure2);
    }
    */
    /*
    if(connect_flag == 2)
    {
        cout << " Two blocks are NOT intersected ." << endl;
 
        trend_boundary = extendRegionBlockToBoundary(mfis4, structure, 1); //extend exit
        trend_boundary2 = extendRegionBlockToBoundary(mfis4, structure2, 2); //extend entrance
        //structure = addTwoVectorsOfObjects(structure, structure2);
        //sprintf(fileName, "%s", "Maps/Offline/Extend_two_regions.png");
        //plotObjectsOf4Kinds(fileName, mfis9, trend_boundary, trend_boundary2, structure);
        
        connection_regions = ConnectionOfRegions(structure, structure2, trend_boundary, trend_boundary2);
        ////
        //structure = addTwoVectorsOfObjects(structure, structure2);
        
        structure = addTwoVectorsOfObjects(structure, trend_boundary);
        structure2 = addTwoVectorsOfObjects(structure2, trend_boundary2);

        _StructureDescription(mfis4, structure, Path[4-1]);
        
        _StructureDescription(mfis4, structure2, Path[5-1]);
    }
    else
    {
        cout << " Two blocks are intersected ." << endl;
        //sprintf(fileName, "%s", "Maps/Offline/Extend_two_regions.png");
        //plotObjectsOf3Kinds(fileName, mfis9, structure, structure2);
        Object line1, line2;
        
        for(int m = 0; m < structure.size(); m++)
        {
            if(structure[m].getEntanceExitflag() == 2)
                line1 = structure[m];
        }
        
        for(int n = 0; n < structure2.size(); n++)
        {
            if(structure2[n].getEntanceExitflag() == 1)
                line2 = structure2[n];
        }
        ////
        structure = addTwoVectorsOfObjects(structure, structure2);
        
        if(line1.length() < line2.length())
            connection_regions.set(line1.X1(), line1.Y1(), line1.X2(), line1.Y2());
        else
            connection_regions.set(line2.X1(), line2.Y1(), line2.X2(), line2.Y2());
            
    }
    connections.push_back(connection_regions);
    sprintf(fileName, "%s", "Maps/Offline/Blocks_and_connection.png");
    //plotObjectsOf3KindswithExits(fileName, mfis9, structure, connections);
    */
    
    vector< vector<Object> > combined_extend_origin;
    vector< pair< vector<Object>, vector<Object> > > extension_blocks;
    extension_blocks.push_back(extension1);
    extension_blocks.push_back(extension2);
    extension_blocks.push_back(extension3);
    extension_blocks.push_back(extension4);
    extension_blocks.push_back(extension5);
    extension_blocks.push_back(extension6);
    extension_blocks.push_back(extension7);
    extension_blocks.push_back(extension8);
    
    combined_extend_origin = Combine_extension_blocks(extension_blocks);
/*
    for(int j = 0; j < combined_extend_origin.size(); j++)
    {
        cout << " combined block size : " << combined_extend_origin[j].size() << endl;
        if(j >= 6)
        {
            sprintf(fileName, "%s%d%s", "Maps/Offline/Blocks_and_Corresond_region - ", j, ".png");
            plotObjects(fileName, combined_extend_origin[j], MFISs[j+1]);
        }
        else
        {
            sprintf(fileName, "%s%d%s", "Maps/Offline/Blocks_and_Corresond_region - ", j, ".png");
            plotObjects(fileName, combined_extend_origin[j], MFISs[j]);    
        }
    }
*/    
    StructureDescription(MFISs, combined_extend_origin, Path);
    ConnectionOfRegions(combined_extend_origin, MFISs);
}

void Test_cominbe_ASR2(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles)
{
    vector< vector<Object> > MFISs; 
    vector< vector<Object> > combined1, combined2,combined3,combined4,combined5;
    vector< vector<Object> > region1, region2,region3,region4,region5;
    vector< vector<Object> > robot1, robot2,robot3,robot4,robot5;
    vector<Object> combined_1,combined_2,combined_3,combined_4,combined_5;
    vector<Object> mfis1, mfis2, mfis3, mfis4, mfis5;
    vector<Ellipse> circles1, circles2, circles3, circles4, circles5;


   
    for(int i = 0; i < views_chunk.size(); i++)
    {
        if(i >= 0 && i <= 10)
        {
            combined1.push_back(views_chunk[i]);
            robot1.push_back(robot_positions[i]);
            circles1.push_back(space_circles[i]);
        }
            
        if(i >= 10 && i <= 15)
        {
            combined2.push_back(views_chunk[i]);
            robot2.push_back(robot_positions[i]);
            circles2.push_back(space_circles[i]);
        }
            
        if(i >= 15 && i <= 18)
        {
            combined3.push_back(views_chunk[i]);
            robot3.push_back(robot_positions[i]);
            circles3.push_back(space_circles[i]);
        } 
     
    }
    
    //cout << " size of 1 : " << combined1.size() << endl;
    for(int i = 0; i < combined1.size()-1; i++)
    {
        combined_1 = addTwoVectorsOfObjects(combined_1, combined1[i]);
    }
    
    region1.push_back(combined_1);
    region1.push_back(combined1.back());

    //cout << " size of 2 : " << combined2.size() << endl;
    for(int i = 0; i < combined2.size()-1; i++)
    {
        combined_2 = addTwoVectorsOfObjects(combined_2, combined2[i]);
    }
    
    region2.push_back(combined_2);
    region2.push_back(combined2.back());

    //cout << " size of 3 : " << combined3.size() << endl;
    for(int i = 0; i < combined3.size()-1; i++)
    {
        combined_3 = addTwoVectorsOfObjects(combined_3, combined3[i]);
    }
    
    region3.push_back(combined_3);
    region3.push_back(combined3.back());


    
    mfis1 = FunctionOfMFIS(5, 3, 69, 80);
    mfis1 = TransformforToGlobalCoordinate(mfis1, Point(robot_positions[0][6].X1(), robot_positions[0][6].Y1()), 
                                              robot_positions[0], robot_positions[0][7].getAngleWithXaxis());
    mfis2 = FunctionOfMFIS(5, 3, 80, 86);
    mfis2 = TransformforToGlobalCoordinate(mfis2, Point(robot_positions[10][6].X1(), robot_positions[10][6].Y1()), 
                                              robot_positions[10], robot_positions[10][7].getAngleWithXaxis());
    
    mfis3 = FunctionOfMFIS(5, 3, 86, 102);
    mfis3 = TransformforToGlobalCoordinate(mfis3, Point(robot_positions[15][6].X1(), robot_positions[15][6].Y1()), 
                                              robot_positions[15], robot_positions[15][7].getAngleWithXaxis());

    char fileName[80];
    sprintf(fileName, "%s", "Maps/Offline/region_mfis1.png");
    plotObjectsOf3Kinds(fileName, mfis1, robot_positions[10], robot_positions[10]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis2.png");
    plotObjectsOf3Kinds(fileName, mfis2, robot_positions[15], robot_positions[15]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis3.png");
    plotObjectsOf3Kinds(fileName, mfis3, robot_positions[18], robot_positions[18]);
    
    MFISs.push_back(mfis1);
    MFISs.push_back(mfis2);
    MFISs.push_back(mfis3);

    
    Exit imagine_exit1,imagine_exit2, imagine_exit3, imagine_exit4;   
    
    imagine_exit1.set(region1[0][0].X1(), region1[0][0].Y1(), 
                     region1[0].back().X2(), region1[0].back().Y2());    
    
    imagine_exit2.set(region1[1][0].X1(), region1[1][0].Y1(), 
                     region1[1].back().X2(), region1[1].back().Y2());    
                     
    imagine_exit3.set(region2[1][0].X1(), region2[1][0].Y1(), 
                     region2[1].back().X2(), region2[1].back().Y2());
    
    imagine_exit4.set(region3[1][0].X1(), region3[1][0].Y1(), 
                     region3[1].back().X2(), region3[1].back().Y2());

    vector<Object> path_space, structure;
    vector<Object> path_space2, structure2;
    vector<Object> path_space3, structure3;
    vector<Object> path_space4, structure4;

    pair< vector<Object>, vector<Object> > extension1, extension2, extension3;

    path_space = pathInRegion_UseCircles(circles1, region1[0], imagine_exit1, imagine_exit2, Path[1-1]);
    structure = Struct_Connect(mfis1, path_space, Path[1-1], imagine_exit1, imagine_exit2, 1);

    path_space2 = pathInRegion_UseCircles(circles2, region2[0], imagine_exit2, imagine_exit3, Path[2-1]);
    structure2 = Struct_Connect(mfis2, path_space2, Path[2-1], imagine_exit2, imagine_exit3, 2);
    sprintf(fileName, "%s", "Maps/Offline/2struct_blocks - 1.png");
    plotObjectsOf3Kinds(fileName, mfis1, structure, structure2);
    extension1 = Combine_origin_extend_regions(structure, structure2, mfis1);

    path_space3 = pathInRegion_UseCircles(circles3, region3[0], imagine_exit3, imagine_exit4, Path[3-1]);
    structure3 = Struct_Connect(mfis3, path_space3, Path[3-1], imagine_exit3, imagine_exit4, 3);
    sprintf(fileName, "%s", "Maps/Offline/2struct_blocks - 2.png");
    plotObjectsOf3Kinds(fileName, mfis2, structure2, structure3);
    extension2 = Combine_origin_extend_regions(structure2, structure3, mfis2);
    
    
    vector< vector<Object> > combined_extend_origin;
    vector< pair< vector<Object>, vector<Object> > > extension_blocks;
    extension_blocks.push_back(extension1);
    extension_blocks.push_back(extension2);
    //extension_blocks.push_back(extension3);
 
    
    combined_extend_origin = Combine_extension_blocks(extension_blocks);   
    StructureDescription(MFISs, combined_extend_origin, Path);
    ConnectionOfRegions(combined_extend_origin, MFISs);
}

void Test_cominbe_ASR3(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles)
{
    vector< vector<Object> > MFISs; 
    vector< vector<Object> > combined1, combined2;
    vector< vector<Object> > region1, region2;
    vector< vector<Object> > robot1, robot2;
    vector<Object> combined_1,combined_2;
    vector<Object> mfis1, mfis2;
    vector<Ellipse> circles1, circles2;


   
    for(int i = 0; i < views_chunk.size(); i++)
    {
        if(i >= 0 && i <= 10)
        {
            combined1.push_back(views_chunk[i]);
            robot1.push_back(robot_positions[i]);
            circles1.push_back(space_circles[i]);
        }
            
    }
    
    //cout << " size of 1 : " << combined1.size() << endl;
    for(int i = 0; i < combined1.size()-1; i++)
    {
        combined_1 = addTwoVectorsOfObjects(combined_1, combined1[i]);
    }
    
    region1.push_back(combined_1);
    region1.push_back(combined1.back());

    mfis1 = FunctionOfMFIS(5, 3, 103, 127);
    mfis1 = TransformforToGlobalCoordinate(mfis1, Point(robot_positions[0][6].X1(), robot_positions[0][6].Y1()), 
                                              robot_positions[0], robot_positions[0][7].getAngleWithXaxis());

    char fileName[80];
    sprintf(fileName, "%s", "Maps/Offline/region_mfis1.png");
    plotObjectsOf3Kinds(fileName, mfis1, robot_positions[0], robot_positions[0]);

    
    MFISs.push_back(mfis1);

    Exit imagine_exit1,imagine_exit2;   
    
    imagine_exit1.set(region1[0][0].X1(), region1[0][0].Y1(), 
                     region1[0].back().X2(), region1[0].back().Y2());    //first step 
    
    imagine_exit2.set(region1[1][0].X1(), region1[1][0].Y1(), 
                     region1[1].back().X2(), region1[1].back().Y2());                 

    vector<Object> path_space, structure;


    pair< vector<Object>, vector<Object> > extension1, extension2, extension3, extension4, extension5;
    
    path_space = pathInRegion_UseCircles(circles1, region1[0], imagine_exit1, imagine_exit2, Path[1-1]);
    structure = Struct_Connect(mfis1, path_space, Path[1-1], imagine_exit1, imagine_exit2, 1);
    sprintf(fileName, "%s", "Maps/Offline/2struct_blocks - 3.png");
    plotObjects(fileName, mfis1, structure);
    //extension1 = Combine_origin_extend_regions(structure, structure2, mfis1);
    /*
    vector< vector<Object> > combined_extend_origin;
    vector< pair< vector<Object>, vector<Object> > > extension_blocks;
    extension_blocks.push_back(extension1);
    //extension_blocks.push_back(extension2);
    //extension_blocks.push_back(extension3);
 
    if(extension_blocks.size() > 1)
    {
        combined_extend_origin = Combine_extension_blocks(extension_blocks);   
        StructureDescription(MFISs, combined_extend_origin, Path);
        ConnectionOfRegions(combined_extend_origin, MFISs);
    }*/
}

void Test_cominbe_ASR4(vector< vector<Object> > views_chunk, vector<Object> Path, 
                      vector< vector<Object> > robot_positions, vector<Object> MFIS, vector<Ellipse> space_circles)
{
    vector< vector<Object> > MFISs; 
    vector< vector<Object> > combined1, combined2,combined3,combined4,combined5;
    vector< vector<Object> > region1, region2,region3,region4,region5;
    vector< vector<Object> > robot1, robot2,robot3,robot4,robot5;
    vector<Object> combined_1,combined_2,combined_3,combined_4,combined_5;
    vector<Object> mfis1, mfis2, mfis3, mfis4, mfis5;
    vector<Ellipse> circles1, circles2, circles3, circles4, circles5;


   
    for(int i = 0; i < views_chunk.size(); i++)
    {
        if(i >= 0 && i <= 5)
        {
            combined1.push_back(views_chunk[i]);
            robot1.push_back(robot_positions[i]);
            circles1.push_back(space_circles[i]);
        }
            
        if(i >= 5 && i <= 7)
        {
            combined2.push_back(views_chunk[i]);
            robot2.push_back(robot_positions[i]);
            circles2.push_back(space_circles[i]);
        }
        
        if(i >= 7 && i <= 8)
        {
            combined3.push_back(views_chunk[i]);
            robot3.push_back(robot_positions[i]);
            circles3.push_back(space_circles[i]);
        }
     
    }
    
    //cout << " size of 1 : " << combined1.size() << endl;
    for(int i = 0; i < combined1.size()-1; i++)
    {
        combined_1 = addTwoVectorsOfObjects(combined_1, combined1[i]);
    }
    
    region1.push_back(combined_1);
    region1.push_back(combined1.back());

    //cout << " size of 2 : " << combined2.size() << endl;
    for(int i = 0; i < combined2.size()-1; i++)
    {
        combined_2 = addTwoVectorsOfObjects(combined_2, combined2[i]);
    }
    
    region2.push_back(combined_2);
    region2.push_back(combined2.back());
    
    for(int i = 0; i < combined3.size()-1; i++)
    {
        combined_3 = addTwoVectorsOfObjects(combined_3, combined3[i]);
    }
    
    region3.push_back(combined_3);
    region3.push_back(combined3.back());

    mfis1 = FunctionOfMFIS(5, 3, 129, 133);
    mfis1 = TransformforToGlobalCoordinate(mfis1, Point(robot_positions[0][6].X1(), robot_positions[0][6].Y1()), 
                                              robot_positions[0], robot_positions[0][7].getAngleWithXaxis());
    mfis2 = FunctionOfMFIS(5, 3, 133, 135);
    mfis2 = TransformforToGlobalCoordinate(mfis2, Point(robot_positions[5][6].X1(), robot_positions[5][6].Y1()), 
                                              robot_positions[5], robot_positions[5][7].getAngleWithXaxis());
    
    mfis3 = FunctionOfMFIS(5, 3, 135, 152);
    mfis3 = TransformforToGlobalCoordinate(mfis3, Point(robot_positions[7][6].X1(), robot_positions[7][6].Y1()), 
                                              robot_positions[7], robot_positions[7][7].getAngleWithXaxis());
    
    char fileName[80];
    sprintf(fileName, "%s", "Maps/Offline/region_mfis1.png");
    plotObjectsOf3Kinds(fileName, mfis1, robot_positions[0], robot_positions[0]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis2.png");
    plotObjectsOf3Kinds(fileName, mfis2, robot_positions[7], robot_positions[7]);
    sprintf(fileName, "%s", "Maps/Offline/region_mfis3.png");
    plotObjectsOf3Kinds(fileName, mfis3, robot_positions[8], robot_positions[8]);
    
    MFISs.push_back(mfis1);
    MFISs.push_back(mfis2);
    MFISs.push_back(mfis3);

    Exit imagine_exit1,imagine_exit2, imagine_exit3, imagine_exit4;   
    
    
    imagine_exit1.set(region1[0][0].X1(), region1[0][0].Y1(), 
                     region1[0].back().X2(), region1[0].back().Y2());    //first step 
                     
    imagine_exit2.set(region1[1][0].X1(), region1[1][0].Y1(), 
                     region1[1].back().X2(), region1[1].back().Y2());
    imagine_exit3.set(region2[1][0].X1(), region2[1][0].Y1(), 
                     region2[1].back().X2(), region2[1].back().Y2());
    imagine_exit4.set(region3[1][0].X1(), region3[1][0].Y1(), 
                     region3[1].back().X2(), region3[1].back().Y2());
    

    vector<Object> path_space, structure;
    vector<Object> path_space2, structure2;
    vector<Object> path_space3, structure3;


    pair< vector<Object>, vector<Object> > extension1, extension2, extension3, extension4, extension5;
    
    path_space = pathInRegion_UseCircles(circles1, region1[0], imagine_exit1, imagine_exit2, Path[1-1]);
    structure = Struct_Connect(mfis1, path_space, Path[1-1], imagine_exit1, imagine_exit2, 1);
    
    path_space2 = pathInRegion_UseCircles(circles2, region2[0], imagine_exit2, imagine_exit3, Path[2-1]);
    structure2 = Struct_Connect(mfis2, path_space2, Path[2-1], imagine_exit1, imagine_exit3, 2);
    sprintf(fileName, "%s", "Maps/Offline/2struct_blocks - 4.png");
    plotObjectsOf3Kinds(fileName, mfis1, structure, structure2);

    //extension1 = Combine_origin_extend_regions(structure, structure2, mfis1);
    
    path_space3 = pathInRegion_UseCircles(circles3, region3[0], imagine_exit3, imagine_exit4, Path[3-1]);
    structure3 = Struct_Connect(mfis3, path_space3, Path[3-1], imagine_exit2, imagine_exit4, 3);
    sprintf(fileName, "%s", "Maps/Offline/2struct_blocks - 5.png");
    plotObjectsOf3Kinds(fileName, mfis2, structure2, structure3);
    extension2 = Combine_origin_extend_regions(structure2, structure3, mfis2);
   
    
    vector< vector<Object> > combined_extend_origin;
    vector< pair< vector<Object>, vector<Object> > > extension_blocks;
    extension_blocks.push_back(extension1);
    extension_blocks.push_back(extension2);
    extension_blocks.push_back(extension3);
 
    
    combined_extend_origin = Combine_extension_blocks(extension_blocks);   
    StructureDescription(MFISs, combined_extend_origin, Path);
    ConnectionOfRegions(combined_extend_origin, MFISs);   
}

void Test_combine_regions(vector< vector<Object> >  region1, vector< vector<Object> >  region2, 
                          vector<Object> mfis1, vector<Object> mfis2, vector<Object> Path, vector<Object> robot1,
                          vector<Object> robot2, Exit imagine_exit1, Exit imagine_exit2, Exit imagine_exit3)
{
    char fileName[80];
    vector<Object> left1, right1, left2, right2;
    vector<Object>  space_block1, space_block2;
    vector<Object> back1, back2;
    vector<Exit> crossed_lines1, crossed_lines2;
    vector<Exit> filter_exits1, filter_exits2;
    vector<Point> pointsA1, pointsA2;
    vector<Point> pointsB1, pointsB2;
    int p1, p2;
    p1 = 5 - 1;
    p2 = 6 - 1;
    
    vector<Exit> all_potential_exits1, all_potential_exits2; //compute using points
    
    
    vector<Exit> imagine_exits;
    imagine_exits.push_back(imagine_exit1);
    imagine_exits.push_back(imagine_exit2);
    imagine_exits.push_back(imagine_exit3);
    
    pair< vector<Object>, vector<Object> > left_And_right1, left_And_right2;
    pair< vector<Object>, vector<Object> >  isolate1, isolate2;
    
    pair< vector<Object>, vector<Object> > boundary_view1, boundary_view2;
    //boundary_view1.first = region1[0];
    boundary_view1.first = mfis1;
    boundary_view1.second = region1[1];
    
    //boundary_view2.first = region2[0];
    boundary_view2.first = mfis2;
    boundary_view2.second = region2[1];
    

    //structualExit(boundary_view,boundary_view);
    isolate1 = SlpitOverlappingRegion(boundary_view1);
    isolate2 = SlpitOverlappingRegion(boundary_view2);

    /*
    sprintf(fileName, "%s%d%s", "Maps/Offline/isolate_with_imagines", p1+1, ".png");
    plotObjectsOf3KindswithExits(fileName, isolate1.first, isolate1.second, imagine_exits);
    sprintf(fileName, "%s%d%s", "Maps/Offline/isolate_with_imagines", p2+1, ".png");
    plotObjectsOf3KindswithExits(fileName, isolate2.first, isolate2.second, imagine_exits);
    */
    /*
    back1 = identify_back(region1[0], vector<Object> boundary_lines,
                             Point robot_position, Exit imagine_exit);
    back2 = identify_back(region2[0], vector<Object> boundary_lines,
                             Point robot_position, Exit imagine_exit);
    */
            
    left_And_right1 = LeftAndRightList(isolate1.first, Path[p1]);
    left_And_right2 = LeftAndRightList(isolate2.first, Path[p2]);
    
    left1 = Trim_group(left_And_right1.first, Path[p1]);
    right1 = Trim_group(left_And_right1.second, Path[p1]);
    
    left2 = Trim_group(left_And_right2.first, Path[p2]);
    right2 = Trim_group(left_And_right2.second, Path[p2]);
    
    //Exit exit_1 = exitBasedImagineLine(left_And_right1, imagine_exit2);
    //Exit exit_2 = exitBasedImagineLine(left_And_right2, imagine_exit3);
    Exit exit_1 = NearexitBasedImagineLine(left_And_right1, imagine_exit2);
    Exit exit_2 = NearexitBasedImagineLine(left_And_right2, imagine_exit3);
    all_potential_exits1.push_back(exit_1);
    all_potential_exits2.push_back(exit_2);
    /*
    pointsA1 = ObjectToPoints(left1);
    pointsA2 = ObjectToPoints(right1);
    
    pointsB1 = ObjectToPoints(left2);
    pointsB2 = ObjectToPoints(right2);
    
    all_potential_exits1 = Potentialine_basePoints(pointsA2, pointsA1, right1);
    all_potential_exits2 = Potentialine_basePoints(pointsB2, pointsB1, right2);*/
    
    /*
    sprintf(fileName, "%s%d%s", "Maps/Offline/all_potential_exit_base_points", p1+1, ".png");
    plotObjectsOf3KindswithExits(fileName, robot1, mfis1, all_potential_exits1);
    //plotObjectsOf3Kinds(fileName, region1[0], mfis1, Path);
    sprintf(fileName, "%s%d%s", "Maps/Offline/all_potential_exit_base_points", p2+1, ".png");
    plotObjectsOf3KindswithExits(fileName, robot2, mfis2, all_potential_exits2);
    */
    /*
    vector<Object> temp_region = block_based_twoExits(exit_1, exit_2);
    cout << " the size of temp region : " << temp_region.size() << endl;
    
    temp_region[0].display();
    temp_region[1].display();
    temp_region[2].display();
    temp_region[3].display();
    waitHere();
    
    sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_block", p1+1, ".png");
    plotObjects(fileName, temp_region, mfis2);
    */
    
    crossed_lines1 = Short_between_groups(left_And_right1.first, left_And_right1.second);
    
    filter_exits1 = ConstrainListOfExits(crossed_lines1, Point (Path[p1].X2(), Path[p1].Y2()));
    
    //space_block1 = BuildAreaOfSpace(isolate1.first, filter_exits1, Path[p1], imagine_exit1, p1+1);
    space_block1 = BuildAreaOfSpace(mfis1, filter_exits1, Path[p1], imagine_exit1, p1+1);
    
    sprintf(fileName, "%s", "Maps/Offline/Space_block1.png");
    plotObjects(fileName, space_block1, isolate1.first);
    
    left_And_right2 = LeftAndRightList(isolate2.first, Path[p2]);
    
    left2 = Trim_group(left_And_right2.first, Path[p2]);
    right2 = Trim_group(left_And_right2.second, Path[p2]);
    
    //
    crossed_lines2 = Short_between_groups(left_And_right2.first, left_And_right2.second);
    
    filter_exits2 = ConstrainListOfExits(crossed_lines2, Point (Path[p2].X2(), Path[p2].Y2()));
    
    //space_block2 = BuildAreaOfSpace(isolate2.first, filter_exits2, Path[p2], imagine_exit2, p2+1);
    space_block2 = BuildAreaOfSpace(mfis2, filter_exits2, Path[p2], imagine_exit2, p2+1);
    
    sprintf(fileName, "%s", "Maps/Offline/Space_block2.png");
    plotObjectsOf4Kinds(fileName, space_block1, space_block2, region1[0], region2[0]);
    
    /*
    vector<Object> interpolate = Two_Space_Overlap(space_block1, space_block2);
    
    vector< vector<Object> > plot;
    plot.push_back(space_block1);
    plot.push_back(space_block2);
    plot.push_back(interpolate);
    sprintf(fileName, "%s", "Maps/Offline/Space_block_interpolation.png");
    plotObjectsColours(fileName, plot, region1[0], region2[0]);
    */
    
    //vector<Object>  boundary_lines;
    //vector<Object> simply_boundary;
    //vector<Object> path_segments;
    
    //boundary_lines = addTwoVectorsOfObjects(boundary_lines, space_block1);
    //boundary_lines = addTwoVectorsOfObjects(boundary_lines, space_block2);
    //boundary_lines = addTwoVectorsOfObjects(boundary_lines, interpolate);
    
    //path_segments = addTwoVectorsOfObjects(path_segments, Path[0]);
    //path_segments = addTwoVectorsOfObjects(path_segments, Path[1]);
     
    //simplify boundary block
    //simply_boundary = simplifyBoundary(Path, boundary_lines);
    
    //sprintf(fileName, "%s", "Maps/Offline/simplified_boundary.png");
    //plotObjectsOf3Kinds(fileName, simply_boundary, region1[0], region2[0]);
    
    /*
    vector<Object> modified_block;       
    modified_block =  ConstrainSpace_Narrow(boundary_view2.first, space_block2,
                                            left_And_right2, Path[1], 1);
    sprintf(fileName, "%s", "Maps/Offline/modified_boundary_block.png");
    plotObjectsOf3Kinds(fileName, modified_block, region1[0], region2[0]);
    */
    
    /*
    pair< vector<Object>, vector<Object> > boundary_intersect_surfaces1;
    pair< vector<Object>, vector<Object> > boundary_intersect_surfaces2;
    vector<Exit> potential_perpend1;
    vector<Exit> potential_perpend2;
    
    boundary_intersect_surfaces1 = boundary_surfaces(left_And_right1, space_block1, Path[p1]);
    boundary_intersect_surfaces2 = boundary_surfaces(left_And_right2, space_block2, Path[p2]);
    
    potential_perpend1 = Exits_in_region(boundary_intersect_surfaces1, space_block1, Path[p1]);
    potential_perpend2 = Exits_in_region(boundary_intersect_surfaces2, space_block2, Path[p2]);
    
    sprintf(fileName, "%s%d%s", "Maps/Offline/boundary_block_Exits-", p1+1,".png");
    plotObjectsOf3KindswithExits(fileName, region1[0], space_block1, potential_perpend1);
    sprintf(fileName, "%s%d%s", "Maps/Offline/boundary_block_Exits-",p2+1,".png");
    plotObjectsOf3KindswithExits(fileName, region2[0], space_block2, potential_perpend2);
    */
    
    /*
    vector<Object> base_lines1, base_lines2;
    vector<Object> filtered_region1, filtered_region2,filtered_region3,filtered_region4;
    
    Object left_line, right_line, left_line2, right_line2;
    vector<Object> constrain_boundary1, constrain_boundary2;
    
    base_lines1 = Boundary_Lines(region1[0], imagine_exit1, imagine_exit2);
    //base_lines2 = Boundary_Lines(region1[0], imagine_exit1, imagine_exit2);
        
    sprintf(fileName, "%s%d%s", "Maps/Offline/extend_base_line -",p1+1,".png");
    plotObjects(fileName, region1[0], base_lines1);
    
    filtered_region1 = Select_surfaces(left_And_right1.first, base_lines1);
    filtered_region2 = Select_surfaces(left_And_right1.second, base_lines1);
  
    left_line = Constrain_region(filtered_region1, Path[p1], imagine_exit2);
    right_line = Constrain_region(filtered_region2, Path[p1], imagine_exit2);
    
    constrain_boundary1.push_back(left_line);
    constrain_boundary1.push_back(right_line);
    
    sprintf(fileName, "%s%d%s", "Maps/Offline/filtered_region -",p1+1,".png");
    plotObjectsOf3Kinds(fileName, filtered_region1, filtered_region2, constrain_boundary1);
    
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    
    base_lines2 = Boundary_Lines(region2[0], imagine_exit2, imagine_exit3);
    //base_lines2 = Boundary_Lines(region1[0], imagine_exit1, imagine_exit2);
        
    sprintf(fileName, "%s%d%s", "Maps/Offline/extend_base_line -",p2+1,".png");
    plotObjects(fileName, region1[0], base_lines2);
    
    filtered_region3 = Select_surfaces(left_And_right2.first, base_lines2);
    filtered_region4 = Select_surfaces(left_And_right2.second, base_lines2);
    
    
    //sprintf(fileName, "%s%d%s", "Maps/Offline/filtered_region -",p1+1,".png");
    //plotObjects(fileName, filtered_region1, filtered_region2);
    
    left_line2 = Constrain_region(filtered_region3, Path[p2], imagine_exit3);
    right_line2 = Constrain_region(filtered_region4, Path[p2], imagine_exit3);
    
    constrain_boundary2.push_back(left_line2);
    constrain_boundary2.push_back(right_line2);
    
    sprintf(fileName, "%s%d%s", "Maps/Offline/filtered_region -",p2+1,".png");
    plotObjectsOf3Kinds(fileName, filtered_region3, filtered_region4, constrain_boundary2);
    */
    
    
    /*----------------------------------------*/
    
}

void Test_cross_exit(vector< vector<Object> >  region, vector<Object>  lastStep_view, 
                          vector<Object> mfis, vector<Object> Path, vector<Object> lastStep_robot,
                          vector<Object> nextStep_robot, Exit imagine_exit1, Exit imagine_exit2)
{
    
    char fileName[80];
    int p;
    
    p = 9 - 1;
    
    Exit exit_Crossed;
    vector<Exit> exit_list;
    
    exit_Crossed = lastStep_narrowExit(lastStep_view, mfis, lastStep_robot, nextStep_robot, Path[p]);
    exit_list.push_back(exit_Crossed);
    sprintf(fileName, "%s%d%s", "Maps/Offline/Crossed_exit_betweenRbs -",p+1,".png");
    plotObjectsOf3KindswithExits(fileName, mfis, lastStep_robot, exit_list);  
    sprintf(fileName, "%s%d%s", "Maps/Offline/Crossed_exit_2Robots -",p+1,".png");
    plotObjectsOf3Kinds(fileName, mfis, lastStep_robot, nextStep_robot);    
}












