/*		 Chunk information process
                    including view, region, circle, path
			
			by Lyn		
*/


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

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

int data_level, data_set; 

ChunkInfo chunk_infomation;

void information_process(vector< vector<Object> > views_chunk, vector<Object> Path, 
                         vector< vector<Object> > robot_positions, vector<Object> MFIS, 
                         vector<Ellipse> space_circles, int chunk_num)
{
        cout << "\n\033[1;34m****************** Information is processing further/reasoning ********************\033[0m" << endl; 
         
        Exit imaginary_exit; //temporary imaginary exit 
        vector<Exit> compute_exits;
        
        vector<Exit> imagine_exits;   //imaginary exits
        vector<Object> mfis, combine; //mfis, combined_views
        vector<Object> path_space, structure; //path space & structure block
        vector< vector<Object> > path_space_region;
        vector< vector<Object> > views, robot, MFISs, region, structures_region; //views, robots, All_mfis, region
        vector< vector< vector<Object> > > view_region, robot_region, regions; //all views & robots in region
        vector< vector<Object> > combined_extend_origin;   //combined blocks
        pair< vector<Object>, vector<Object> > extension;  //block extension
        vector< pair< vector<Object>, vector<Object> > > extension_blocks; //all combined blocks
        vector<Ellipse> circles; //circles
        vector< vector<Ellipse> > circle_region; //all circles in region
        
        
        //regions, robots, circles
        for(int m = 0; m < labels.size()-1; m++)
        {
            for(int i = 0; i < views_chunk.size(); i++)
            {
                if(i >= labels[m] && i <= labels[m+1])
                {
                    views.push_back(views_chunk[i]);
                    robot.push_back(robot_positions[i]);
                    circles.push_back(space_circles[i]);
                }
            }

            if(views.size() > 0)
            {
                
                view_region.push_back(views); //all region views
                robot_region.push_back(robot);//all region robots
                circle_region.push_back(circles); //all region circles

                for(int j = 0; j < views.size()-1; j++)
                {
                    combine = addTwoVectorsOfObjects(combine, views[j]);
                }

                region.push_back(combine);      //each region
                region.push_back(views.back()); //each region
                regions.push_back(region);      //store each of region
            }
     
            //reset for different region
            views.clear();
            robot.clear();
            circles.clear();
            combine.clear();
            region.clear();
            
        }  
        cout << " Fundamental information process done !!" << endl;    
    
        //mfis for each of region
        for(int m = 0; m < view_numbers.size()-1; m++)
        {
            mfis = FunctionOfMFIS(data_level, data_set, view_numbers[m], view_numbers[m+1]);
            mfis = TransformforToGlobalCoordinate(mfis, Point(robot_positions[labels[m]][6].X1(), robot_positions[labels[m]][6].Y1()), 
                                              robot_positions[labels[m]], robot_positions[labels[m]][7].getAngleWithXaxis());
            MFISs.push_back(mfis); //all region mfis
        }
        cout << " All MFIS process done !!" << endl; 
    
        //imaginary exits 
        for(int i = 0; i < regions.size(); i++)
        {
            if(chunk_num == 1)
            {
                if(imagine_exits.size() < 1)
                {
                    imaginary_exit.set(-2000, 0, 2000, 0);    //first step / initialisation
                    i--;
                }
                else
                {
                    imaginary_exit.set(regions[i][1][0].X1(), regions[i][1][0].Y1(), 
                                     regions[i][1].back().X2(), regions[i][1].back().Y2());
                }

                imagine_exits.push_back(imaginary_exit);
            }
            else
            {
                if(imagine_exits.size() < 1)
                {
                    imaginary_exit.set(regions[i][0][0].X1(), regions[i][0][0].Y1(), 
                                 regions[i][0].back().X2(), regions[i][0].back().Y2());
                    i--;
                }
                else
                {
                    imaginary_exit.set(regions[i][1][0].X1(), regions[i][1][0].Y1(), 
                                     regions[i][1].back().X2(), regions[i][1].back().Y2());
                }

                imagine_exits.push_back(imaginary_exit);
            }
        }
        
        cout << " All imaginary exits process done !!" << endl; 
        
        //compute all structures
        int p = 0; //previous step

        for(int i = 0; i < regions.size(); i++)
        {
            //process information section
            if(i == 0)
                p = i;
            else
                p = i - 1;
            
            path_space = pathInRegion_UseCircles(circle_region[i], regions[i][0], imagine_exits[i], imagine_exits[i+1], Path[i]);    
            structure = _Struct_Connect(MFISs[p], MFISs[i], path_space, Path[i], imagine_exits[p], imagine_exits[i+1], i+1);
            if(structure.size() > 0)
            {
                structures_region.push_back(structure); 
                path_space_region.push_back(path_space);
            }
        }
        
        cout << " Structure blocks process done !!" << endl; 
        
        if(structures_region.size() > 1)
        {
            
            p = 0;
            //compute all inter extension between a pair of structures
            for(int i = 0; i < structures_region.size()-1; i++)
            {
                if(MFISs[i][0].get_region_remove_flag() != 1)
                    extension = Combine_origin_extend_regions(structures_region[i], structures_region[i+1], MFISs[p]);
                else
                    extension = Combine_origin_extend_regions(structures_region[i], structures_region[i+1], MFISs[++p]);

                extension_blocks.push_back(extension);
                p++;
            }

            cout << " Structure blocks extension process done !!" << endl; 

            combined_extend_origin = Combine_extension_blocks(extension_blocks);   
            /////StructureDescription(MFISs, combined_extend_origin, Path);
            compute_exits = _ConnectionOfRegions(combined_extend_origin, MFISs, path_space_region);
            //environment_shape(MFISs, Path, compute_exits);
        }
        else
        {
            //only one region in chunk
            cout << " There is only one region !" << endl << endl;
            waitHere();
        }
        
        chunk_infomation.setChunkInfo(MFIS, compute_exits, robot_positions, views_chunk, MFISs);
        //environment_shape(MFISs, Path, compute_exits);
}

