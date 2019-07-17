/*
 * Function: split chunk information into region
 *           identify exit using regions
 * author: Lyn Pang
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "Exit.h"
#include "Object.H"
#include "PathPlanning.H"
#include "GeometricOp.H"
#include "Plotting.H"
#include "GeometryAndExit.h"

/* identify perpendicular line based on each path
 * segment, which is to depict an exit between 
 * two regions/path segments
 */
vector<Exit> FindExits(vector<Object> ref_map, vector<Object> path)
{
      Exit temp_exit;
      vector<Exit> All_exits;
      
      //Object temp_obj;
      Point temp_point1, temp_point2;
    
      //vertical line with respect to single path segment
      for(int i = 0; i < path.size(); i++)
      {
          //perpendicular line
          temp_point1 = outSidePerpendPoint(path[i], 500, Point (path[i].X2(), path[i].Y2()), 1);
          temp_point2 = outSidePerpendPoint(path[i], 500, Point (path[i].X2(), path[i].Y2()), 2);
          
          temp_exit.set(temp_point1.X(), temp_point1.Y(), temp_point2.X(), temp_point2.Y());
          All_exits.push_back(temp_exit);
      }
    
      return All_exits;
}

/*
 *
 */
void NarrowSpace(vector<Object> ref_map, vector<Object> path)
{
        double threshold = 500;
        
        vector<Point> points_on_line;      //five points on 
        vector<Object> gap_path;
        vector<Object> all_gaps;
               
        for(int i = 0; i < path.size(); i++)
        {
            //take one path segment as reference
            

            //points on path segment 
            points_on_line = FivePointsOnline(path[i]);
                            
            //find narrow place and check
            gap_path = ShortestBaseLine(points_on_line, path[i], ref_map);
            
            /*
            for(int m = 0; m < gap_path.size(); m++)
            {
                if(gap_path[m].length() > 1000)
                {
                    gap_path.erase(gap_path.begin()+m);
                    m = 0;
                }
            }
            */
            all_gaps = addTwoVectorsOfObjects(all_gaps, gap_path);
            
            //construct area based on the narrow gap 

                 
        }
        
        char fileName[80];
        sprintf(fileName, "%s", "Maps/Offline/Chunk_Gap_Path.png");
        plotObjectsOf3Kinds(fileName, ref_map, all_gaps, path);
}


vector<Point> FivePointsOnline(Object line_segment)
{
    Point mid, quad1, quad2;
    vector<Point> temp;
    
    if(line_segment.length() > 1000)
    {
        //
        mid.set((line_segment.X1() + line_segment.X2())/2, (line_segment.Y1() + line_segment.Y2())/2);
        quad1.set((line_segment.X1() + mid.X())/2, (line_segment.Y1() + mid.Y())/2);
        quad2.set((line_segment.X2() + mid.X())/2, (line_segment.Y2() + mid.Y())/2);

        temp.push_back(Point (line_segment.X1(), line_segment.Y1()));
        temp.push_back(quad1);
        temp.push_back(mid);
        temp.push_back(quad2);
        temp.push_back(Point (line_segment.X2(), line_segment.Y2()));
    }
    else
    {
        temp.push_back(Point (line_segment.X1(), line_segment.Y1()));
        temp.push_back(Point (line_segment.X2(), line_segment.Y2()));
    }
    
    return temp;
}

vector<Point> NinePointsOnline(Object line_segment)
{
    Point mid, quad1, quad2, quad3, quad4, quad5, quad6;
    vector<Point> temp;
    
    if(line_segment.length() > 1000)
    {
        //
        mid.set((line_segment.X1() + line_segment.X2())/2, (line_segment.Y1() + line_segment.Y2())/2);
        quad1.set((line_segment.X1() + mid.X())/2, (line_segment.Y1() + mid.Y())/2);
        quad2.set((line_segment.X2() + mid.X())/2, (line_segment.Y2() + mid.Y())/2);
        
        quad3.set((line_segment.X1() + quad1.X())/2, (line_segment.Y1() + quad1.Y())/2);
        quad4.set((quad1.X() + mid.X())/2, (quad1.Y() + mid.Y())/2);
        
        quad5.set((quad2.X() + mid.X())/2, (quad2.Y() + mid.Y())/2);
        quad6.set((line_segment.X2() + quad2.X())/2, (line_segment.Y2() + quad2.Y())/2);

        temp.push_back(Point (line_segment.X1(), line_segment.Y1()));
        temp.push_back(quad3);
        temp.push_back(quad1);
        temp.push_back(quad4);
        temp.push_back(mid);
        temp.push_back(quad5);
        temp.push_back(quad2);
        temp.push_back(quad6);
        temp.push_back(Point (line_segment.X2(), line_segment.Y2()));
    }
    else
    {
        temp.push_back(Point (line_segment.X1(), line_segment.Y1()));
        temp.push_back(Point (line_segment.X2(), line_segment.Y2()));
    }
    
    return temp;
}


vector<Object> ShortestBaseLine(vector<Point> base_points, Object path_segment, vector<Object> ref_map)
{
    Point ref_point;
    Point Left_ref_point, Right_ref_point;
    Point left_near, right_near;
    
    Object left_line, right_line;
    Object base_line, found_line;
    vector<Object> base_lines;
    
    
    for(int i = 0 ; i < base_points.size(); i++)
    {
            ref_point.set(base_points[i].X(), base_points[i].Y());
                    
            //construct lines
            Left_ref_point = outSidePerpendPointWithLength(path_segment, 1000, ref_point, 2);
            Right_ref_point = outSidePerpendPointWithLength(path_segment, 1000, ref_point, 1);
            

            found_line.set(Left_ref_point.X(), Left_ref_point.Y(), Right_ref_point.X(), Right_ref_point.Y(),1);
            //foundation.push_back(found_line);
            
            //check the nearest intersected points
            left_line.set(Left_ref_point.X(), Left_ref_point.Y(), ref_point.X(), ref_point.Y(), 1);
            right_line.set(Right_ref_point.X(), Right_ref_point.Y(), ref_point.X(), ref_point.Y(), 2);
            
            left_near = intersectedNearPoint(ref_map, Left_ref_point,ref_point);
            right_near = intersectedNearPoint(ref_map, Right_ref_point,ref_point);
            
            //test intersected points//
         
            if(left_near.X() != 0 && right_near.X() != 0)
            {
                base_line.set(left_near.X(), left_near.Y(), right_near.X(), right_near.Y(), 1);
                if(base_line.length() < 2000)
                    base_lines.push_back(base_line);
            }
            else
            {
                if(left_near.X() != 0)
                {
                    base_line.set(left_near.X(), left_near.Y(), ref_point.X(), ref_point.Y(), 1);
                    
                    if(base_line.length() < 2000)
                        base_lines.push_back(base_line);
                }
                if(right_near.X() != 0)
                {
                    base_line.set(right_near.X(), right_near.Y(), ref_point.X(), ref_point.Y(), 1);
                    
                    if(base_line.length() < 2000)
                        base_lines.push_back(base_line);
                }
            }
            
    }
           
    return base_lines;
}


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
vector<Exit> PotentialExits(vector<Object> cv)
{

        //vector<Exit> result;
        //vector<Object> bObjects;//Objects on asr boundary
        double distp2top1;
        double mindist=400;
        
        int no_exit;
        
        Exit exit;
        vector<Exit> exits;
        vector<Exit> realExits;

        int exits_counter = 1;
        for (int i = 0; i< cv.size(); i++) 
        {
            if (cv[i].getPEP2() == true) 
            { //p2 is a probable exit end(p1)
                no_exit = 0;
                for (int j = i + 1; j<int(cv.size()); j++) 
                {
                     // if(cv[j].getPEP1() == true && cv[j].getP1OS() == 1 ) { //p1 is a probable exit end(p2)
                     if (no_exit == 0) 
                     {
                             exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                             exit.setP1ID(cv[i].getID());
                             exit.setP2ID(cv[j].getID());
                             no_exit++;
                     } 
                     else 
                     {
                             distp2top1 = cv[i].distP2ToP1(cv[j]);
                             if (exit.length() > distp2top1) { //condition to get shortest exit
                                 exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                 exit.setP1ID(cv[i].getID());
                                 exit.setP2ID(cv[j].getID());
                             }
                     }
                    //}//if cv j
                }//for j

                //i = exit.getP2ID() - 2; //for triming

                exit.setID(exits_counter);
                //find the nearest point instead of p2
                Point tmpp = cv[exit.getP2ID() - 1].shortestDistPointFrom(exit.X1(), exit.Y1());
                exit.set(exit.X1(), exit.Y1(), tmpp.X(), tmpp.Y());

                exits.push_back(exit);
                exits_counter++;

                //i--;
                //i=j-1;
            }//if cv i

        }// for i


        
        for (int i = 0; i<int(exits.size()); i++) 
        {
            if (exits[i].length() > 800 )//&& exits[i].length() < 1200)
                realExits.push_back(exits[i]);
        }
        
        char fileName[80];
        sprintf(fileName, "%s", "Maps/Offline/Potential_Exit_View.png");
        plotObjectsOf3KindswithExits(fileName, cv, cv, realExits);

        //waitHere();
        return realExits;
}

/* find a list of views potential exits
 * input: a list of views
 * output: a list of exits -- vector<Exit>
 */
vector<Exit> PotentialExitFromAlistOfView(vector< vector<Object> > views)
{
    Exit temp_exit;
    vector<Exit> exits;
    vector<Exit> rnt;
    
    //initial
    rnt = PotentialExits(views[0]);
    for(int i = 1; i < views.size(); i++)
    {
       exits = PotentialExits(views[i]);
       /*
       cout << " the size of current exit : " << exits.size() << endl;
        char fileName[80];
        sprintf(fileName, "%s", "Maps/Offline/Potential_Exit_View.png");
        plotObjectsOf3KindswithExits(fileName, views[i], views[i], exits);

        waitHere();
        */
       rnt = addTwoExits(rnt, exits);
    }
    
    return rnt;
}

/* add two vector<Exit> variables
 * input: two vector<Exit> variables
 * output: a vector<Exit> 
 */
vector<Exit> addTwoExits(vector<Exit> ref, vector<Exit> exits)
{
    vector<Exit> rnt;
    rnt = ref;
    for(int i = 0; i < exits.size(); i++)
    {
        rnt.push_back(exits[i]);
        //cout << " size of rnt : " << rnt.size() << endl;
        //waitHere();
    }
    
    return rnt;
}

void structualExit(pair< vector<Object>, vector<Object> > boundary_view,
                   pair< vector<Object>, vector<Object> > robots)
{
        Object imagine_exit;
        
        vector<Object> rectangle_on_imagExit;
        vector<Object> view1, view2;
        
        view1 = boundary_view.first;
        view2 = boundary_view.second;

        //robot position where out of current region
        imagine_exit.set(view2[0].X1(), view2[0].Y1(), view2.back().X2(), view2.back().Y2(), 0);
        
        rectangle_on_imagExit = makeRectangle(imagine_exit);
        
        char fileName[80];
        sprintf(fileName, "%s", "Maps/Offline/imagine_rectangle.png");
        plotObjectsOf3Kinds(fileName, view1, view2, rectangle_on_imagExit);

}



/* Based on split space & path segment
 * find all potential exits
 * from isolated space
 */
Exit Potential_Exit(pair< vector<Object>, vector<Object> > spaces, 
                             Object path_segment,
                             Object ref_boundary)
{
        vector<Object> view1, view2;
        
        Point P1, P2; //key point1 & 2
        
        double left_min, right_min;
        double dist_to_ref = 0;
        
        //initialization
        left_min = right_min = 2000;

        view1 = spaces.first;
        view2 = spaces.second;

        
        for(int i = 0; i < view1.size(); i++)
        {
            //left
            dist_to_ref = Perpendiculardistance(Point (ref_boundary.X1(), ref_boundary.Y1()),
                                                Point (ref_boundary.X2(), ref_boundary.Y2()), 
                                                Point (view1[i].X1(), view1[i].Y1()));
            
            if((isLeft(Point (path_segment.X1(), path_segment.Y1()), Point (path_segment.X2(), path_segment.Y2()),
                       Point (view1[i].X1(), view1[i].Y1())) > 0)
                &&(dist_to_ref < left_min))
            {
                left_min = dist_to_ref;
                P1.set(view1[i].X1(), view1[i].Y1());
            }
            
            
            if((isLeft(Point (path_segment.X1(), path_segment.Y1()), Point (path_segment.X2(), path_segment.Y2()),
                    Point (view1[i].X1(), view1[i].Y1())) < 0)
                &&(dist_to_ref < right_min))
            {
                right_min = dist_to_ref;
                P2.set(view1[i].X1(), view1[i].Y1());
            }
            
            
            //left
            dist_to_ref = Perpendiculardistance(Point (ref_boundary.X1(), ref_boundary.Y1()),
                                                Point (ref_boundary.X2(), ref_boundary.Y2()), 
                                                Point (view1[i].X2(), view1[i].Y2()));
            
            if((isLeft(Point (path_segment.X1(), path_segment.Y1()), Point (path_segment.X2(), path_segment.Y2()),
                    Point (view1[i].X2(), view1[i].Y2())) > 0)
                &&(dist_to_ref < left_min))
            {
                left_min = dist_to_ref;
                P1.set(view1[i].X2(), view1[i].Y2());
            }
                        
            
            if((isLeft(Point (path_segment.X1(), path_segment.Y1()), Point (path_segment.X2(), path_segment.Y2()),
                    Point (view1[i].X2(), view1[i].Y2())) < 0)
                &&(dist_to_ref < right_min))
            {
                right_min = dist_to_ref;
                P2.set(view1[i].X2(), view1[i].Y2());
            }
            
            
            
        }
        
        Exit temp;
        vector<Exit> test;
        temp.set(P1.X(), P1.Y(), P2.X(), P2.Y());
        test.push_back(temp);
        
        char fileName[80];
        sprintf(fileName, "%s", "Maps/Offline/Two_space_with_exit.png");
        plotObjectsOf3KindswithExits(fileName, view1, view2, test);
        
        return temp;
    
}


/* Compute all exit following path
 * find the smallest one, or pick some 
 * of them from those exits
 */
vector<Exit> PerpendicularExits(vector<Object> region, Object path_segment)
{
    int para = 1;
        Exit temp_exit;
        vector<Exit> Exits;
        
        Point ref_point;
        Point Left_ref_point, Right_ref_point;
        Point left_near, right_near;
        vector<Point> points;
        
        Object found_line, left_line, right_line;
        vector<Object> foundation;

        
        //points = FivePointsOnline(path_segment);
        points = NinePointsOnline(path_segment);
        cout << " size of pints : " << points.size() << endl;

lb:     for(int i = 0; i < points.size(); i++)
        {
                ref_point.set(points[i].X(), points[i].Y());
                //construct lines
                Left_ref_point = outSidePerpendPointWithLength(path_segment, 1000 * para, ref_point, 2);
                Right_ref_point = outSidePerpendPointWithLength(path_segment, 1000 * para, ref_point, 1);


                found_line.set(Left_ref_point.X(), Left_ref_point.Y(), Right_ref_point.X(), Right_ref_point.Y(),1);
                foundation.push_back(found_line);

                //check the nearest intersected points
                left_line.set(Left_ref_point.X(), Left_ref_point.Y(), ref_point.X(), ref_point.Y(), 1);
                right_line.set(Right_ref_point.X(), Right_ref_point.Y(), ref_point.X(), ref_point.Y(), 2);

                left_near = intersectedNearPoint(region, Left_ref_point,ref_point);
                right_near = intersectedNearPoint(region, Right_ref_point,ref_point);
                
                //cout << " left  x : " << left_near.X() << "  y : " << left_near.Y() << endl;
                //cout << " right x : " << right_near.X() << " y : " << right_near.Y() << endl;
                
                if(left_near.X() != 0 && right_near.X() != 0)
                {
                    
                    temp_exit.set(left_near.X(), left_near.Y(), right_near.X(), right_near.Y());
                    Exits.push_back(temp_exit);
                }
                //else
                //    cout << " Left or right is not intersected !" << endl;
        }
        //out << " size of Exits : " << Exits.size() << endl;
        if(Exits.size() <= 1)
        {
            para += 1;
            if(para < 12)
                goto lb;
        }
        
        
        //
        vector<Object> tem;
        tem.push_back(path_segment);
        
        char fileName[80];
        sprintf(fileName, "%s", "Maps/Offline/Perpendicular_exits.png");
        plotObjectsOf3KindswithExits(fileName, region, tem, Exits);
        //sprintf(fileName, "%s", "Maps/Offline/Perpendicular_base_lines.png");
        //plotObjectsOf3Kinds(fileName, region, tem, foundation);
   
        //waitHere();
        
        return Exits;
}

/* filter the list of exits
 * pick the near and shortest one
 * as outputs
 */
vector<Exit> ConstrainListOfExits(vector<Exit> exits, Point exit_point)
{
        double d1,d2,d3, average;
        double min = 5000;
        Point p1, p2, p3;
        
        Exit near, shortest;
        vector<Exit> rnt;
        //vector<Point> points;


        //near one first, three points average distance
        for(int i =0; i < exits.size(); i++)
        {
            
      
                    p1.set(exits[i].X1(), exits[i].Y1());
                    p2.set((exits[i].X1() + exits[i].X2())/2, (exits[i].Y1() + exits[i].Y2())/2);
                    p2.set(exits[i].X2(), exits[i].Y2());

                    //d1 = P_To_ShortestDistance(p1, Object line);
                    //d2 = P_To_ShortestDistance(p2, Object line);
                    //d3 = P_To_ShortestDistance(p3, Object line);

                    d1 = distanceOftwoP(p1, exit_point);
                    d2 = distanceOftwoP(p2, exit_point);
                    d3 = distanceOftwoP(p3, exit_point);


                    average = (d1 + d2 + d3) / 3;

                    if(average < min)
                    {
                        min = average;
                        near = exits[i];
                    }
        }
    
        min = 5000;
        //shortest one from the list
        for(int i =0; i < exits.size(); i++)
        {
                if(exits[i].length() < min)
                {
                    min = exits[i].length();
                    shortest = exits[i];
                }
        }
        
        if(near.X1() != shortest.X1() && near.X2() != shortest.X2())
        {
            //cout << "Near & short are NOT the same exit " << endl;
            //cout << "the length of near : " << near.length() << endl;
            //cout << "the length of short : " << shortest.length() << endl;
            if(near.length() > 0)
                rnt.push_back(near);
            if(shortest.length() >= 800)
                rnt.push_back(shortest);
        }
        else
        {
            //cout << "Near & short are the same exit " << endl;
            //these two exits are the same one
            rnt.push_back(near); 
        }
        
        
        return rnt;
        
}

/* based upon left and right list of points
 * compute all potential lines
 * from left to right, find the shortest
 * line.
 */
vector<Exit> Potentialine_basePoints(vector<Point> left, vector<Point> right, vector<Object> left_region)
{
    double min, dist;
    Exit temp_Obj;
    vector<Exit> all_potential;
    
    for(int i = 0; i < left.size(); i++)
    {
        for(int j =0; j < right.size(); j++)
        {
            if(j == 0)
            {    
                min = distanceOftwoP(left[i], right[j]);
                temp_Obj.set(left[i].X(), left[i].Y(), right[j].X(), right[j].Y());
            }
            else
            {
                dist = distanceOftwoP(left[i], right[j]);
                if(dist < min)
                {
                    min = dist;
                    temp_Obj.set(left[i].X(), left[i].Y(), right[j].X(), right[j].Y());
                }
            }
        }
        
        //check whether this exit is blocked by any other surface
        if(interSectWithLine(left_region, temp_Obj.getP1(), temp_Obj.getP2()) == false)     
            all_potential.push_back(temp_Obj);
    }
    
    return all_potential;
}

vector<Exit> Potentialine_basePoints(vector<Object> map, Object path)
{

    vector<Object> left_list, right_list;
    vector<Point> left, right;
   
    //
    for(int i = 0 ; i < map.size(); i++)
    {
        //left list
            if((isLeft(path.getP1(), path.getP2(), map[i].getP1()) > 0)
                && (isLeft(path.getP1(), path.getP2(), map[i].getP2()) > 0))
            {
                left_list.push_back(map[i]);
            }
            else
            {
                //right list
                if((isLeft(path.getP1(), path.getP2(), map[i].getP1()) < 0)
                    && (isLeft(path.getP1(), path.getP2(), map[i].getP2()) < 0))
                {
                    right_list.push_back(map[i]);
                }
            }
        
    }
    
    //convert to points
    left = ObjectToPoints(left_list);
    right = ObjectToPoints(right_list);
    
    double min, dist;
    Exit temp_Obj;
    vector<Exit> all_potential;
    
    for(int i = 0; i < left.size(); i++)
    {
        for(int j =0; j < right.size(); j++)
        {
            if(j == 0)
            {    
                min = distanceOftwoP(left[i], right[j]);
                temp_Obj.set(left[i].X(), left[i].Y(), right[j].X(), right[j].Y());
            }
            else
            {
                dist = distanceOftwoP(left[i], right[j]);
                if(dist < min)
                {
                    min = dist;
                    temp_Obj.set(left[i].X(), left[i].Y(), right[j].X(), right[j].Y());
                }
            }
        }
        
        //check whether this exit is blocked by any other surface
        if(interSectWithLine(left_list, temp_Obj.getP1(), temp_Obj.getP2()) == false)     
            all_potential.push_back(temp_Obj);
    }
    
    return all_potential;
}

/* inputs are imagine exit and left & right lists of surfaces
 * find shortest exit and which neats to the imagine exit
 */
Exit exitBasedImagineLine(pair< vector<Object>, vector<Object> > boundary_view, Exit imagine_exit)
{
    Object ref_Obj;
    vector<Object> left_list, right_list;
    
    Point left_p, right_p;
    vector<Point> left_points, right_points;
    double distance, min;
    double slope_ref, slope_temp;
    
    Exit rnt;
    
    ref_Obj.set(imagine_exit.X1(), imagine_exit.Y1(), imagine_exit.X2(), imagine_exit.Y2(), 0);
    
    left_list = boundary_view.first;
    right_list = boundary_view.second;
    
    cout << " size of left list : " << left_list.size() << endl;
    cout << " size of right list : " << right_list.size() << endl;
    
    left_points = ObjectToPoints(left_list);
    right_points = ObjectToPoints(right_list);
    
    
    //get the nearest point with respect to imagine exit 
    for(int i = 0;i, i < left_points.size(); i++)
    {
        distance = P_To_ShortestDistance(left_points[i], ref_Obj);
        
        if(i == 0)
        {
            min = distance;
            left_p = left_points[i];
        }
        else
        {
            if(distance < min)
            {
                min = distance;
                left_p = left_points[i];
            }
        }
    }
    cout << " left potential point x : " << left_p.X() << " ; y : " << left_p.Y() << endl;
    //get the closest point on opposite side
    for(int i = 0; i < right_points.size(); i++)
    {
        distance = distanceOftwoP(left_p, right_points[i]);
        
        if(i == 0)
        {
            min = distance;
            right_p = right_points[i];
        }
        else
        {
            if(distance < min)
            {
                min = distance;
                right_p = right_points[i];
            }
        }
    }
    cout << " right potential point x : " << right_p.X() << " ; y : " << right_p.Y() << endl;
    //check if the line joint left p and right p is the shortest from right
    //if not, replace it
    for(int i = 0 ; i < left_points.size(); i++)
    {
        distance = distanceOftwoP(right_p, left_points[i]);
        

            if(distance < distanceOftwoP(right_p, left_p))
            {
                slope_ref = (right_p.Y() - left_p.Y()) / (right_p.X() - left_p.X());
                slope_temp = (right_p.Y() - left_points[i].Y()) / (right_p.X() - left_points[i].X());
                
                if(abs(slope_temp/slope_ref) < 2)
                {
                    min = distance;
                    left_p = left_points[i];
                }
            }
    }
    
    rnt.set(left_p.X(), left_p.Y(), right_p.X(), right_p.Y());
    
    return rnt;
}

/* 
 * 
 */
Exit NearexitBasedImagineLine(pair< vector<Object>, vector<Object> > boundary_view, Exit imagine_exit)
{
    Exit rnt;
    Object covert;
    Point left_p, right_p;
    vector<Point> left_points, right_points;
    
    double dist, min;
    
    left_points = ObjectToPoints(boundary_view.first);
    right_points = ObjectToPoints(boundary_view.second);
    covert.set(imagine_exit.X1(), imagine_exit.Y1(), imagine_exit.X2(), imagine_exit.Y2(), 0);
    
    //left list
    for(int i = 0; i < left_points.size(); i++)
    {
        dist = P_To_ShortestDistance(left_points[i], covert);
        if(i == 0)
        {
            min = dist;
            left_p = left_points[i];
        }
        else
        {
            if(dist < min)
            {
                min = dist;
                left_p = left_points[i];
            }
        }
    }
    
    //right list
    for(int i = 0; i < right_points.size(); i++)
    {
        dist = P_To_ShortestDistance(right_points[i], covert);
        if(i == 0)
        {
            min = dist;
            right_p = right_points[i];
        }
        else
        {
            if(dist < min)
            {
                min = dist;
                right_p = right_points[i];
            }
        }
    }
    
    rnt.set(left_p.X(), left_p.Y(), right_p.X(), right_p.Y());
    
    //identify the surface holds the points
    
    return rnt;
}


/* 
 * 
 */
vector<Object> identify_back(vector<Object> region, vector<Object> boundary_lines,
                             Point robot_position, Exit imagine_exit)
{
    Point based_point;
    Object ref_line; //reference line for computing back 
    
    vector<Object> back_list;
    
    // compute base line
    based_point = crossPerpend(boundary_lines[0].getP1(), boundary_lines[0].getP2(), robot_position);
    //based_point = crossPerpend(boundary_lines[1].getP1(), boundary_lines[1].getP2(), robot_position);
    
    ref_line.set(robot_position.X(), robot_position.Y(), based_point.X(), based_point.Y(), 0);
    //check side 
    
    for(int i = 0; i < region.size(); i++)
    {
        if((isLeft(ref_line.getP1(), ref_line.getP2(), imagine_exit.getP1()) > 0 && isLeft(ref_line.getP1(), ref_line.getP2(), region[i].getP1()) < 0)
            || (isLeft(ref_line.getP1(), ref_line.getP2(), imagine_exit.getP1()) < 0 && isLeft(ref_line.getP1(), ref_line.getP2(), region[i].getP1()) > 0)
            || (isLeft(ref_line.getP1(), ref_line.getP2(), imagine_exit.getP1()) > 0 && isLeft(ref_line.getP1(), ref_line.getP2(), region[i].getP2()) < 0)
            || (isLeft(ref_line.getP1(), ref_line.getP2(), imagine_exit.getP1()) < 0 && isLeft(ref_line.getP1(), ref_line.getP2(), region[i].getP2()) > 0))
        {
                //remove this surface
                back_list.push_back(region[i]);
        }
    }
    
    //
    return back_list;
}

Exit lastStep_narrowExit(vector<Object> lastStep_view, vector<Object> MFIS, 
                         vector<Object> lastStep_robot, vector<Object> nextStep_robot,
                         Object path_segment)
{
    double min;
    Exit rnt;
    vector<Exit> exits;
    
    Object ref_line; //for classify left & right
    vector<Object> left_list, right_list;
    
    vector<Point> left_points, right_points;
    
    //
    for(int i = 0 ; i < lastStep_view.size(); i++)
    {
        //left list
        if((isLeft(lastStep_robot[6].getP1(), lastStep_robot[6].getP2(), lastStep_view[i].getP1()) > 0)
            && (isLeft(lastStep_robot[6].getP1(), lastStep_robot[6].getP2(), lastStep_view[i].getP2()) > 0))
        {
            left_list.push_back(lastStep_view[i]);
        }
        else
        {
            //right list
            if((isLeft(lastStep_robot[6].getP1(), lastStep_robot[6].getP2(), lastStep_view[i].getP1()) < 0)
                && (isLeft(lastStep_robot[6].getP1(), lastStep_robot[6].getP2(), lastStep_view[i].getP2()) < 0))
            {
                right_list.push_back(lastStep_view[i]);
            }
        }
  
    }
    
    //
    //pair< vector<Object>, vector<Object> > left_And_right = LeftAndRightList(MFIS, path_segment);
    
    
    //convert to points
    left_points = ObjectToPoints(left_list);
    right_points = ObjectToPoints(right_list);
    
    exits = Potentialine_basePoints(left_points, right_points, MFIS);
    
    
    //before this step, check if the exit in between two robot position
    for(int i = 0; i < exits.size(); i++)
    {
        if((distanceOftwoP(exits[i].getP1(), lastStep_robot[6].getP2()) > distanceOftwoP(lastStep_robot[6].getP2(), nextStep_robot[6].getP2()))
            ||(distanceOftwoP(exits[i].getP2(), lastStep_robot[6].getP2()) > distanceOftwoP(lastStep_robot[6].getP2(), nextStep_robot[6].getP2()))
            ||(distanceOftwoP(exits[i].getP1(), nextStep_robot[6].getP2()) > distanceOftwoP(lastStep_robot[6].getP2(), nextStep_robot[6].getP2()))
            ||(distanceOftwoP(exits[i].getP2(), nextStep_robot[6].getP2()) > distanceOftwoP(lastStep_robot[6].getP2(), nextStep_robot[6].getP2())))
        {
            exits.erase(exits.begin()+i);
            i = 0;
        }
    }
    
    
    
    //find the shortest exit 
    for(int i = 0 ; i < exits.size(); i++)
    {
        if(i == 0)
        {
            rnt = exits[i];
            min =  exits[i].length();
        }
        else
        {
            if(min > exits[i].length())
            {
                rnt = exits[i];
                min =  exits[i].length();
            }
        }
    
    }
    
    char fileName[80];
    sprintf(fileName, "%s%d%s", "Maps/Offline/Crossed_exitsAndrobot -",0,".png");
    //plotObjectsOf3KindswithExits(fileName, left_And_right.first, left_And_right.second, exits);  
    plotObjectsOf3KindswithExits(fileName, MFIS, lastStep_robot, exits);  
    
    
    return rnt;
}


vector<Exit> ExitInbetweenRobots(vector<Object> mfis_region, vector< vector<Object> > robots)
{
    char fileName[80];
    Exit temp;
    vector<Exit> exits, potentials, filted_exits;
    vector<Object> robots_in_region;
    Object convert;
    Object path_segment;
    Object orientation_ref;
    
    double dist, min;
    unsigned char inter_flag = 0;
    
    vector<Point> left_points, right_points;
    pair< vector<Object>, vector<Object> > left_And_right;
    
    path_segment.set(robots[0][6].X2(), robots[0][6].Y2(), robots.back()[6].X2(), robots.back()[6].Y2(), 1);
    
    for(int i = 0; i < robots.size()-1; i++)
    {
        orientation_ref.set(robots[i][6].X2(), robots[i][6].Y2(), robots[i+1][6].X2(), robots[i+1][6].Y2(),1);
        left_And_right = LeftAndRightList(mfis_region, orientation_ref);//classify left & right using robot facing
        
        left_points = ObjectToPoints(left_And_right.first);
        right_points = ObjectToPoints(left_And_right.second);
        exits = Potentialine_basePoints(left_points, right_points, left_And_right.first);


        
        /*
        for(int j = 0; j < exits.size(); j++)
        {
            convert.set(exits[j].X1(), exits[j].Y1(), exits[j].X2(), exits[j].Y2(), 0);
            dist = P_To_ShortestDistance(robots[i][6].getP2(), convert);
            if(j == 0)
            {
                min = dist;
                temp = exits[j];
            }
            else
            {
                if(dist < min)
                {
                    min = dist;
                    temp = exits[j];
                }
            }
        }
        
        potentials.push_back(temp);
        */
        
        
    }
    
/*
    //filtered useless potential exits
    for(int n = 0; n < exits.size(); n++)
    {
        for(int m = 0; m < mfis_region.size(); m++)
        {
            if((twoLinintersect(exits[n].getP1(), exits[n].getP2(), mfis_region[m].getP1(), mfis_region[m].getP2()) == true)
                && ((exits[n].X1() != mfis_region[m].X1()) && (exits[n].X2() != mfis_region[m].X2())
                    && (exits[n].X1() != mfis_region[m].X2()) && (exits[n].X2() != mfis_region[m].X1())))
            {
                //filted_exits.push_back(exits[n]);
                inter_flag = 1;
                //continue;
            }
            
            if((inter_flag == 0) && (m = mfis_region.size()-1))
                filted_exits.push_back(exits[n]);
                
        }
    }
*/
    //intersected with path segment of this region
    for(int i = 0; i < exits.size(); i++)
    {
        if(twoLinintersect(exits[i].getP1(), exits[i].getP2(), path_segment.getP1(), path_segment.getP2()) == false)
        {
            exits.erase(exits.begin()+i);
            i--;
        }
    }    
    
    //combine all robots
    for(int j = 0 ; j < robots.size(); j++)
        robots_in_region = addTwoVectorsOfObjects(robots_in_region, robots[j]);
    
    sprintf(fileName, "%s%d%s", "Maps/Offline/Crossed_exitsAndrobot -",0,".png");
    //plotObjectsOf3KindswithExits(fileName, left_And_right.first, left_And_right.second, exits);  
    plotObjectsOf3KindswithExits(fileName, left_And_right.first, left_And_right.second, exits); 
    //waitHere();
    
    return potentials;
}


Exit exitFromRegion(vector<Object> left_list, vector<Object> right_list, 
                    Object path_segment, Exit imagine_exit)
{

    double dist, min;
    
    Exit rnt;
    
    Object covert;
    Object entry_space, exit_space;
    
    vector<Object> overlap_region;
    vector<Object> left_surf, right_surf;
    
    Point left_p, right_p;
    vector<Point> left_points, right_points;
    
    for(int i = 0; i < left_list.size(); i++)
    {
            if(isLeft(imagine_exit.getP1(), imagine_exit.getP2(), path_segment.getP1()) > 0)
            {
                    if((isLeft(imagine_exit.getP1(), imagine_exit.getP2(),left_list[i].getP1()) > 0)
                            || (isLeft(imagine_exit.getP1(), imagine_exit.getP2(),left_list[i].getP2()) > 0))
                    {
                            //overlapping region
                            left_surf.push_back(left_list[i]);
                    }
            }
            else
            {
                    if((isLeft(imagine_exit.getP1(), imagine_exit.getP2(),left_list[i].getP1()) < 0)
                            || (isLeft(imagine_exit.getP1(), imagine_exit.getP2(),left_list[i].getP2()) < 0))
                    {
                            //overlapping region
                            left_surf.push_back(left_list[i]);
                    }
            }
    }   
    
    for(int i = 0; i < right_list.size(); i++)
    {
            if(isLeft(imagine_exit.getP1(), imagine_exit.getP2(), path_segment.getP1()) > 0)
            {
                    if((isLeft(imagine_exit.getP1(), imagine_exit.getP2(),right_list[i].getP1()) > 0)
                            || (isLeft(imagine_exit.getP1(), imagine_exit.getP2(),right_list[i].getP2()) > 0))
                    {
                            //overlapping region
                            right_surf.push_back(right_list[i]);
                    }
            }
            else
            {
                    if((isLeft(imagine_exit.getP1(), imagine_exit.getP2(),right_list[i].getP1()) < 0)
                            || (isLeft(imagine_exit.getP1(), imagine_exit.getP2(),right_list[i].getP2()) < 0))
                    {
                            //overlapping region
                            right_surf.push_back(right_list[i]);
                    }
            }
    } 
        
    left_points = ObjectToPoints(left_surf);
    right_points = ObjectToPoints(right_surf);

    covert.set(imagine_exit.X1(), imagine_exit.Y1(), imagine_exit.X2(), imagine_exit.Y2(), 0);
    
    /*
    //left list
    for(int i = 0; i < left_points.size(); i++)
    {
            dist = P_To_ShortestDistance(left_points[i], covert);
            if(i == 0)
            {
                    min = dist;
                    left_p = left_points[i];
            }
            else
            {
                    if(dist < min)
                    {
                            min = dist;
                            left_p = left_points[i];
                    }
            }
    }
    
    //right list
    for(int i = 0; i < right_points.size(); i++)
    {
            dist = P_To_ShortestDistance(right_points[i], covert);
            if(i == 0)
            {
                    min = dist;
                    right_p = right_points[i];
            }
            else
            {
                    if(dist < min)
                    {
                            min = dist;
                            right_p = right_points[i];
                    }
            }
    }
    */
    
    //vector<Exit> all_potential = Short_between_groups(left_surf, right_surf);
    
    vector<Exit> all_potential = Potentialine_basePoints(left_points, right_points, left_surf);
    
    
    cout << " left size : " << left_surf.size() << endl;
    cout << " right size : " << right_surf.size() << endl;
    cout << " potential exits size : " << all_potential.size() << endl;
    //waitHere();
    
    
    //char fileName[80];
    //sprintf(fileName, "%s%d%s", "Maps/Offline/Crossed_exitsAndrobot -",0,".png");
    //plotObjectsOf3KindswithExits(fileName, left_surf, right_surf, all_potential); 
    
    if(all_potential.size() > 0) 
    {
        for(int i = 0; i < all_potential.size(); i++)
        {
            double check_area;
            check_area = distanceOftwoP(all_potential[i].getmidPoint(), path_segment.getP1());

            if((check_area < 500)
               || (distanceOftwoP(all_potential[i].getP1(), left_points[0]) < 500) 
               ||  (distanceOftwoP(all_potential[i].getP2(), right_points.back()) < 500))   
            {
                all_potential.erase(all_potential.begin() + i);
                i--;
            }
        }


        for(int i = 0; i < all_potential.size(); i++)
        {
            if(i == 0)
            {
                rnt = all_potential[i];
                min = all_potential[i].length();
            }
            else
            {
                if(all_potential[i].length() < min)
                {
                    min = all_potential[i].length();
                    rnt = all_potential[i];
                }
            }
        }

        entry_space.set(left_points[0].X(), left_points[0].Y(), right_points.back().X(), right_points.back().Y(), 1);
        exit_space.set(rnt.X1(), rnt.Y1(), rnt.X2(), rnt.Y2(), 2);
        //check if the exit is close to entrance

        //Path poly;
        double inner_anle, block_area;
        inner_anle = includedAngle(entry_space, exit_space);

        //poly = viewConvertPath(rnt);
        //block_area = Area(poly);

        if(((TwoObjectIntersect(entry_space, exit_space) == true) && inner_anle < 20)
            ||  (((distanceOftwoP(rnt.getP1(), left_points[0]) < 600) 
                ||  (distanceOftwoP(rnt.getP2(), right_points.back()) < 600))
                && (rnt.length() > 2000))
            //||  (distanceOftwoP(rnt.getmidPoint(), path_segment.getP1()) < 500)
            ||  (all_potential.size() == 0))
            rnt.set(left_points.back().X(), left_points.back().Y(), right_points[0].X(), right_points[0].Y());


        //rnt.set(left_p.X(), left_p.Y(), right_p.X(), right_p.Y());
        //rnt.set(left_points.back().X(), left_points.back().Y(), right_points[0].X(), right_points[0].Y());
    }  
    else
        rnt.set(0,0,0,0);
        
    return rnt;    
}

Exit most_constrain_exit_on_path(vector<Object> view, Object path_segment)
{
            Exit constrain_exit;
    
            vector<Exit> potential_exits = Potentialine_basePoints(view, path_segment);

            double min = 0;
            for(int j = 0; j < potential_exits.size(); j++)
            {
                    if(j == 0)
                    {
                            if((potential_exits[j].getP1() != view[0].getP1())
                                && (potential_exits[j].getP2() != view.back().getP2()))
                            {
                                min = potential_exits[j].length();
                                constrain_exit = potential_exits[j];
                            }
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
            
            Object specific_surface;
            for(int i = 0; i < view.size(); i++)
            {
                if((view[i].getP1() == constrain_exit.getP2())
                    || (view[i].getP2() == constrain_exit.getP2()))
                {
                    specific_surface = view[i];
                    break;
                }
            }
            
            if(P_To_ShortestDistance(constrain_exit.getP1(), specific_surface) < constrain_exit.length())
            {
                Point p = crossPerpend(specific_surface.getP1(), specific_surface.getP2(), constrain_exit.getP1());
                constrain_exit.set(constrain_exit.X1(), constrain_exit.Y1(), p.X(), p.Y());
            }
            
            return constrain_exit;
}


Exit most_constrain_exit_on_path(vector< vector<Object> > views, vector< vector<Object> > robots, int cnt)
{
        Object path_segment;
        Exit constrain_exit;
        vector<Exit> potential_exits;
    
lb:     if(cnt < (int)(views.size()) - 4)
            path_segment.set(robots[cnt][6].X1(), robots[cnt][6].Y1(), robots[cnt+4][6].X1(), robots[cnt+4][6].Y1(), 0);
        else
            path_segment.set(robots[cnt][6].X1(), robots[cnt][6].Y1(), robots.back()[6].X1(), robots.back()[6].Y1(), 0);

        potential_exits = Potentialine_basePoints(views[cnt], path_segment);
    
        double min = 0;
        for(int j = 0; j < potential_exits.size(); j++)
        {
                if(j == 0)
                {
                        if((potential_exits[j].getP1() != views[cnt][0].getP1())
                            && (potential_exits[j].getP2() != views[cnt].back().getP2()))
                        {
                            min = potential_exits[j].length();
                            constrain_exit = potential_exits[j];
                        }
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

        

        if((constrain_exit.length() >= 600) && (twoLinintersect(constrain_exit.getP1(), constrain_exit.getP2(), path_segment.getP1(), path_segment.getP2()) == true))
            return constrain_exit;
        else
        {
            if(cnt < views.size()-1)
            {
                cnt += 1;
                goto lb;
            }
            else
            {
                constrain_exit.set(0,0,0,0);
                return constrain_exit;
            }
        }

        
}

Exit ConnectionOfRegions(vector<Object> block1, vector<Object> block2,
                         vector<Object> extend_block1, vector<Object> extend_block2)
{
    int intersect_flag = 0;
    
    Exit exit_connection;
    
    Object overlap_info;
    vector<Object> intersect_objects;
    
    Point temp_point;
    vector<Point> project_points;
    
            
    for(int i = 0; i < block2.size(); i++)
    {
        if(interSectWithLine(extend_block1, block2[i].getP1(), block2[i].getP2()) )
            intersect_objects.push_back(block2[i]);
    }
    
    if(intersect_objects.size() > 1)
    {
        for(int i = 0; i < intersect_objects.size()-1; i++)
        {
            if((intersect_objects[i].getP1() == intersect_objects[i+1].getP1())
                || (intersect_objects[i].getP1() == intersect_objects[i+1].getP2())
                || (intersect_objects[i].getP2() == intersect_objects[i+1].getP1())
                || (intersect_objects[i].getP2() == intersect_objects[i+1].getP2()))
            {
                intersect_flag = 2;
                break;
            }
            else
                intersect_flag = 1;
        } 
    }
    else
        intersect_flag = 1;
    
    
    if(intersect_flag == 1)
    {
        //using the small as connection
        for(int i = 0; i < extend_block1.size(); i++)
        {
            for(int j = 0; j < intersect_objects.size(); j++)
            {
                if(TwoObjectIntersect(extend_block1[i], intersect_objects[j]) == true)
                {
                    temp_point = intersectPointTwolineEquations(extend_block1[i], intersect_objects[j]);
                    project_points.push_back(temp_point);
                }
            }
        }
        
        if(project_points.size() > 2)
        {
            exit_connection.set(project_points[0].X(), project_points[0].Y(), project_points[2].X(), project_points[2].Y());
        }
        else
            exit_connection.set(project_points[0].X(), project_points[0].Y(), project_points[1].X(), project_points[1].Y());
    }
    else
    {
        Object line1, line2;
        //compute a connection between two blocks
        if((interSectWithLine(extend_block2, extend_block1[1].getP1(), extend_block1[1].getP2()) == true)
            && (interSectWithLine(extend_block2, extend_block1[1].getP1(), extend_block1[1].getP2()) == true))
        {
            //choose the small one directly
            for(int m = 0; m < block1.size(); m++)
            {
                if(block1[m].getEntanceExitflag() == 2)
                    line1 = block1[m];
            }

            for(int n = 0; n < block2.size(); n++)
            {
                if(block2[n].getEntanceExitflag() == 1)
                    line2 = block2[n];
            }

            if(line1.length() < line2.length())
                exit_connection.set(line1.X1(), line1.Y1(), line1.X2(), line1.Y2());
            else
                exit_connection.set(line2.X1(), line2.Y1(), line2.X2(), line2.Y2());
            
            
        }
        else
        {
            //left or right
            if((interSectWithLine(block2, extend_block1[0].getP1(), extend_block1[0].getP2()) == true)
                || (interSectWithLine(extend_block2, extend_block1[0].getP1(), extend_block1[0].getP2()) == true))
            {
                //choose the one on the same side as connection
                exit_connection.set(extend_block1[0].X1(), extend_block1[0].Y1(), extend_block1[0].X2(), extend_block1[0].Y2());
            }
            else
            {
                if((interSectWithLine(block2, extend_block1[2].getP1(), extend_block1[2].getP2()) == true)
                || (interSectWithLine(extend_block2, extend_block1[2].getP1(), extend_block1[2].getP2()) == true))
                {
                    exit_connection.set(extend_block1[2].X1(), extend_block1[2].Y1(), extend_block1[2].X2(), extend_block1[2].Y2());
                }
            }
            
            
        }
        
    }
    
    
    return exit_connection;
}

vector<Exit> ConnectionOfRegions(vector< vector<Object> > combined_origin_extension, vector< vector<Object> > MFISs)
{
    char fileName[80];
    Exit connection, temp_exit; 
    vector<Exit> exit_area, exits;
    
    Point temp_point;
    vector<Point> connect_points;
    vector<Point> left_potential, right_potential;
    pair< vector<Point>, vector<Point> > potential_list;
    
            
    for(int i = 0; i < combined_origin_extension.size()-1; i++)
    {
        //connect_points = intersectedPoints(combined_origin_extension[i], combined_origin_extension[i+1]);
        for(int m = 0; m < combined_origin_extension[i].size() ; m++)
        {
            for(int n = 0; n < combined_origin_extension[i+1].size(); n++)
            {
                if(TwoObjectIntersect(combined_origin_extension[i+1][n], combined_origin_extension[i][m]) == true)
                {
                    temp_point = crossedPointReturnP(combined_origin_extension[i+1][n].getP1(), combined_origin_extension[i+1][n].getP2(),
                                                     combined_origin_extension[i][m].getP1(), combined_origin_extension[i][m].getP2());
                    connect_points.push_back(temp_point);
                }
            }
        }
        
        Object ref_obj;
        for(int m = 0; m < combined_origin_extension[i].size() ; m++)
        {
            if(combined_origin_extension[i][m].getEntanceExitflag() == 2) //pick the exit side
                ref_obj = combined_origin_extension[i][m];
        }
        
        for(int m = 0; m < connect_points.size(); m++)
        {
            if(FindDistanceToSegment(connect_points[m], ref_obj) > 2500) //far away from exit/entrance
            {
                connect_points.erase(connect_points.begin()+m);
                m--;
            }
        }
        
        if(connect_points.size() > 1)
        {
            for(int j = 0; j < connect_points.size(); j++)
            {
                if(j != connect_points.size() - 1)
                    connection.set(connect_points[j].X(), connect_points[j].Y(), connect_points[j+1].X(), connect_points[j+1].Y());
                else
                    connection.set(connect_points[j].X(), connect_points[j].Y(), connect_points[0].X(), connect_points[0].Y());

                exit_area.push_back(connection);
            }
        }
        else
        {
            connection.set(ref_obj.X1(), ref_obj.Y1(), ref_obj.X2(), ref_obj.Y2());
            exit_area.push_back(connection);
            
        }
        
        
        if(MFISs[i][0].get_region_remove_flag() != 1)
        {
            //sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_region -",i,".png");
            //plotObjectsOf3KindswithExits(fileName, MFISs[i], MFISs[i], exit_area); 
            
            if(exit_area.size() > 1)
            {
                //select a list of points
                potential_list = select_potential_points(MFISs[i], exit_area);
                temp_exit = shortestExit(potential_list);
                exits.push_back(temp_exit);
            }
            else
            {
                if(exit_area[0].get_surf_describe() != 1)
                {
                    //select a list of points
                    potential_list = select_potential_points(MFISs[i], exit_area);
                    temp_exit = shortestExit(potential_list);
                    exits.push_back(temp_exit);
                }
                else
                    exits.push_back(exit_area[0]);
            }  
        }
        else
        {
            //sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_region -",i,".png");
            //plotObjectsOf3KindswithExits(fileName, MFISs[i+1], MFISs[i+1], exit_area);
            if(exit_area.size() > 1)
            {
                //select a list of points
                potential_list = select_potential_points(MFISs[i+1], exit_area);
                temp_exit = shortestExit(potential_list);
                exits.push_back(temp_exit);
            }
            else
            {
                if(exit_area[0].get_surf_describe() != 1)
                {
                    //select a list of points
                    potential_list = select_potential_points(MFISs[i+1], exit_area);
                    temp_exit = shortestExit(potential_list);
                    exits.push_back(temp_exit);
                }
                else
                    exits.push_back(exit_area[0]);
            }  
        }
        
        
        
        sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_region -",i,".png");
        plotObjectsOf3KindswithExits(fileName, MFISs[i], MFISs[i+1], exits);
        exit_area.clear();
        connect_points.clear();
    }
     
    //the last region
    //combined_origin_extension.back();
   
    
    return exits;
}

vector<Exit> _ConnectionOfRegions(vector< vector<Object> > combined_origin_extension, vector< vector<Object> > MFISs,
                                  vector< vector<Object> > path_space_region)
{
    char fileName[80];
    Exit connection, temp_exit; 
    vector<Exit> exit_area, exits;
    
    Point temp_point;
    vector<Point> connect_points;
    vector<Point> left_potential, right_potential;
    pair< vector<Point>, vector<Point> > potential_list;
    
            
    for(int i = 0; i < combined_origin_extension.size()-1; i++)
    {
        //connect_points = intersectedPoints(combined_origin_extension[i], combined_origin_extension[i+1]);
        for(int m = 0; m < combined_origin_extension[i].size() ; m++)
        {
            for(int n = 0; n < combined_origin_extension[i+1].size(); n++)
            {
                if(TwoObjectIntersect(combined_origin_extension[i+1][n], combined_origin_extension[i][m]) == true)
                {
                    temp_point = crossedPointReturnP(combined_origin_extension[i+1][n].getP1(), combined_origin_extension[i+1][n].getP2(),
                                                     combined_origin_extension[i][m].getP1(), combined_origin_extension[i][m].getP2());
                    connect_points.push_back(temp_point);
                }
            }
        }
        
        Object ref_obj;
        for(int m = 0; m < combined_origin_extension[i].size() ; m++)
        {
            if(combined_origin_extension[i][m].getEntanceExitflag() == 2) //pick the exit side
                ref_obj = combined_origin_extension[i][m];
        }
        
        for(int m = 0; m < connect_points.size(); m++)
        {
            if(FindDistanceToSegment(connect_points[m], ref_obj) > 2500) //far away from exit/entrance
            {
                connect_points.erase(connect_points.begin()+m);
                m--;
            }
        }
        
        if(connect_points.size() > 1)
        {
            for(int j = 0; j < connect_points.size(); j++)
            {
                if(j != connect_points.size() - 1)
                    connection.set(connect_points[j].X(), connect_points[j].Y(), connect_points[j+1].X(), connect_points[j+1].Y());
                else
                    connection.set(connect_points[j].X(), connect_points[j].Y(), connect_points[0].X(), connect_points[0].Y());

                exit_area.push_back(connection);
            }
        }
        else
        {
            connection.set(ref_obj.X1(), ref_obj.Y1(), ref_obj.X2(), ref_obj.Y2());
            exit_area.push_back(connection);
            
        }
        
        
        if(MFISs[i][0].get_region_remove_flag() != 1)
        {
            //sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_region -",i,".png");
            //plotObjectsOf3KindswithExits(fileName, MFISs[i], MFISs[i], exit_area); 
            
            if(exit_area.size() > 1)
            {
                //select a list of points
                potential_list = select_potential_points(MFISs[i], exit_area);
                temp_exit = shortestExit(potential_list);
                temp_exit.set_exit_surround(MFISs[i]);
                exits.push_back(temp_exit);
            }
            else
            {
                if(exit_area[0].get_surf_describe() != 1)
                {
                    //select a list of points
                    potential_list = select_potential_points(MFISs[i], exit_area);
                    temp_exit = shortestExit(potential_list);
                    temp_exit.set_exit_surround(MFISs[i]);
                    exits.push_back(temp_exit);
                }
                else
                {
                    exit_area[0].set_exit_surround(MFISs[i]);
                    exits.push_back(exit_area[0]);
                }
            }  
        }
        else
        {
            //sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_region -",i,".png");
            //plotObjectsOf3KindswithExits(fileName, MFISs[i+1], MFISs[i+1], exit_area);
            if(exit_area.size() > 1)
            {
                //select a list of points
                potential_list = select_potential_points(MFISs[i+1], exit_area);
                temp_exit = shortestExit(potential_list);
                temp_exit.set_exit_surround(MFISs[i+1]);
                exits.push_back(temp_exit);
            }
            else
            {
                if(exit_area[0].get_surf_describe() != 1)
                {
                    //select a list of points
                    potential_list = select_potential_points(MFISs[i+1], exit_area);
                    temp_exit = shortestExit(potential_list);
                    temp_exit.set_exit_surround(MFISs[i+1]);
                    exits.push_back(temp_exit);
                }
                else
                {
                    exit_area[0].set_exit_surround(MFISs[i+1]);
                    exits.push_back(exit_area[0]);
                }
            }  
        }
        
        sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_region -", i,".png");
        plotObjectsOf3KindswithExits(fileName, MFISs[i], MFISs[i+1], exits);
        exit_area.clear();
        connect_points.clear();
    }
     
    //the last region
    connection.set(path_space_region.back()[0].X2(), path_space_region.back()[0].Y2(), 
                        path_space_region.back()[1].X2(), path_space_region.back()[1].Y2());
    exit_area.push_back(connection);
    potential_list = select_potential_points(MFISs.back(), exit_area);
    temp_exit = shortestExit(potential_list);
    temp_exit.set_exit_surround(MFISs.back());
    exits.push_back(temp_exit);
    sprintf(fileName, "%s%d%s", "Maps/Offline/Exit_region -",combined_origin_extension.size(),".png");
    plotObjectsOf3KindswithExits(fileName, MFISs.back(), MFISs.back(), exits);
    //////////////////
    
    
    return exits;
}

pair< vector<Point>, vector<Point> > select_potential_points(vector<Object> view, vector<Exit> exit_area)
{
    double max = 0;
    double radius = 500;
    double exp1 = 0, exp2 = 0;
    Exit large_exit;
    
    vector<Point> temp_list1, temp_list2, convert;
    pair< vector<Point>, vector<Point> > rnt;
    
    //select ref information
    for(int i = 0; i < exit_area.size(); i++)
    {
        if(i == 0)
        {
            max = exit_area[i].length();
            large_exit = exit_area[i];
        }
        else
        {
            if(exit_area[i].length() > max)
            {
                max = exit_area[i].length();
                large_exit = exit_area[i];
            }
        }
    }
    
    //compute circles based on two endpoints
    convert = ObjectToPoints(view);
lb: for(int i = 0; i < convert.size(); i++)
    {
        if(distanceOftwoP(convert[i], large_exit.getP1()) < radius+exp1)
        {
            temp_list1.push_back(convert[i]);
            continue;
        }
        
        if(distanceOftwoP(convert[i], large_exit.getP2()) < radius+exp2)
        {
            temp_list2.push_back(convert[i]);
        }
    }
    
    if((temp_list1.size() < 1) && (temp_list2.size() < 1))
    {
        exp1 += 100;
        exp2 += 100;
        
        temp_list1.clear();
        temp_list2.clear();
        goto lb;
    }
    else
    {
        if(temp_list1.size() < 1)
        {
            exp1 += 100;
            temp_list1.clear();
            temp_list2.clear();
            goto lb;
        }
        
        if(temp_list2.size() < 1)
        {
            exp2 += 100;
            temp_list1.clear();
            temp_list2.clear();
            goto lb;
        }
    }
    
    rnt.first = temp_list1;
    rnt.second = temp_list2;
    
    return rnt;
}

Exit shortestExit(vector<Object> list1, vector<Object>  list2)
{
    vector<Point> point_group1, point_group2;
    pair< vector<Point>, vector<Point> > potential_lists;
    
    point_group1 = ObjectToPoints(list1);
    point_group2 = ObjectToPoints(list2);
    
    potential_lists.first = point_group1;
    potential_lists.second = point_group2;
    
    return shortestExit(potential_lists);
    
}

Exit shortestExit(pair< vector<Point>, vector<Point> > potential_lists)
{
    double min, dist;
    
    Exit temp_exit, shorttest;
    vector<Exit> all_potential_exits;
    vector<Point> temp1, temp2;
    
    temp1 = potential_lists.first;
    temp2 = potential_lists.second;
    
    for(int i = 0; i < temp1.size(); i++)
    {
        for(int j = 0; j < temp2.size(); j++)
        {
            if(j == 0)
            {
                min = distanceOftwoP(temp2[j], temp1[i]);
                temp_exit.set(temp2[j].X(), temp2[j].Y(), temp1[i].X(), temp1[i].Y());
            }
            else
            {
                if(distanceOftwoP(temp2[j], temp1[i]) < min)
                {
                    min = distanceOftwoP(temp2[j], temp1[i]);
                    temp_exit.set(temp2[j].X(), temp2[j].Y(), temp1[i].X(), temp1[i].Y());
                }
            }
        }
        
        all_potential_exits.push_back(temp_exit);
    }
    
    
    for(int i = 0; i < all_potential_exits.size(); i++)
    {
        if((i == 0)
           && (all_potential_exits[i].length() > 700))
        {
            min = all_potential_exits[i].length();
            shorttest = all_potential_exits[i];
        }
        else
        {
            if((all_potential_exits[i].length() < min)
                && (all_potential_exits[i].length() > 700))
            {
                min = all_potential_exits[i].length();
                shorttest = all_potential_exits[i];
            }
        }
    }
    
    return shorttest;
}

/* virtual boundary & real boundary
 * computer potential exits on boundary
 * return vector<Exit>
 */
vector<Exit> potential_exit_on_boundary(vector<Object> virtual_boundary, vector<Object> real_boundary)
{
    
    Exit temp_exit1, temp_exit2, temp_exit3, temp_exit4;
    vector<Exit> all_exit_lines;
    vector<Exit> potential_exits_rnt;
    
    vector<Object> temp_vitural;
    
    temp_vitural = virtual_boundary;
    //temp_vitural.pop_back();   
    
//    for(int i =0; i< virtual_boundary.size()-2; i++)
//    {
//        for(int j = i+1; j < virtual_boundary.size()-1; j++)
//        {
//            if((distanceOftwoP(virtual_boundary[i].getP1(), virtual_boundary[j].getP1())>600)
//                && (distanceOftwoP(virtual_boundary[i].getP1(), virtual_boundary[j].getP1())<1500))
//            {
//                temp_exit1.set(virtual_boundary[i].getP1().X(), virtual_boundary[i].getP1().Y(), virtual_boundary[j].getP1().X(), virtual_boundary[j].getP1().Y());
//            }
//            
//            if((distanceOftwoP(virtual_boundary[i].getP1(), virtual_boundary[j].getP2())>600)
//                && (distanceOftwoP(virtual_boundary[i].getP1(), virtual_boundary[j].getP2())<1500))
//            {
//                temp_exit2.set(virtual_boundary[i].getP1().X(), virtual_boundary[i].getP1().Y(), virtual_boundary[j].getP2().X(), virtual_boundary[j].getP2().Y());
//            }
//            
//            if((distanceOftwoP(virtual_boundary[i].getP2(), virtual_boundary[j].getP1())>600)
//                && (distanceOftwoP(virtual_boundary[i].getP2(), virtual_boundary[j].getP1())<1500))
//            {
//                temp_exit3.set(virtual_boundary[i].getP2().X(), virtual_boundary[i].getP2().Y(), virtual_boundary[j].getP1().X(), virtual_boundary[j].getP1().Y());
//            }
//            
//            if((distanceOftwoP(virtual_boundary[i].getP2(), virtual_boundary[j].getP2())>600)
//                && (distanceOftwoP(virtual_boundary[i].getP2(), virtual_boundary[j].getP2())<1500))
//            {
//                temp_exit4.set(virtual_boundary[i].getP2().X(), virtual_boundary[i].getP2().Y(), virtual_boundary[j].getP2().X(), virtual_boundary[j].getP2().Y());
//            }
//            
//            all_exit_lines.push_back(temp_exit1);
//            all_exit_lines.push_back(temp_exit2);
//            all_exit_lines.push_back(temp_exit3);
//            all_exit_lines.push_back(temp_exit4);
//        }
//    }
//    
//    //vector<Object> temp = convertExitToObject(all_exit_lines);
//    for(int i = 0; i < all_exit_lines.size(); i++)
//    {
//        for(int j = 0; j < real_boundary.size(); j++)
//        {
//                if((all_exit_lines[i].getP1() == real_boundary[j].getP1()) && (all_exit_lines[i].getP2() == real_boundary[j].getP2()))
//                {
//                           all_exit_lines.erase(all_exit_lines.begin()+i);
//                           i--;
//                           break;
//                }           
//        }
//    }
//    
//   
//    for(int i = 0; i < all_exit_lines.size(); i++)
//    {
//            for(int j = 0; j < real_boundary.size(); j++)
//            {
//                    if(twoLinintersect(all_exit_lines[i].getP1(), all_exit_lines[i].getP2(), real_boundary[j].getP1(), real_boundary[j].getP2())==true)
//                    {
//                        if((crossedPointReturnP(all_exit_lines[i].getP1(), all_exit_lines[i].getP2(), real_boundary[j].getP1(), real_boundary[j].getP2()) != real_boundary[j].getP1())
//                            && (crossedPointReturnP(all_exit_lines[i].getP1(), all_exit_lines[i].getP2(), real_boundary[j].getP1(), real_boundary[j].getP2()) != real_boundary[j].getP2()))
//                        {
//                            all_exit_lines.erase(all_exit_lines.begin()+i);
//                            i--;
//                            break;
//                        }
//                    }
//            }
//    }
//    

    
    temp_vitural.pop_back();
    //temp_vitural.erase(temp_vitural.begin());
    Point ref_p1, ref_p2;
    
    for(int i = 0; i < temp_vitural.size() - 1; i++)
    {
        ref_p1.set(temp_vitural[i].X1(), temp_vitural[i].Y1());
        double min = 0, dist = 0;
        for(int j = i+1; j < temp_vitural.size(); j++)
        {
            dist = distanceOftwoP(ref_p1, temp_vitural[j].getP2());
            if(j == i+1)
            {
                min = dist;
                ref_p2 = temp_vitural[j].getP2();
            }
            else
            {
                if(dist < min)
                {
                    min = dist;
                    ref_p2 = temp_vitural[j].getP2();
                }
            }
        }
        
        temp_exit1.set(ref_p1.X(), ref_p1.Y(), ref_p2.X(), ref_p2.Y());
        all_exit_lines.push_back(temp_exit1);
        
    }
    
    vector<Object> polygon = makePolygon_Clipper(real_boundary, 0.0);
 
    for(int i = 0; i < all_exit_lines.size(); i++)
    {

        //if(pointInPolygon(PointXY(all_exit_lines[i].getmidPoint().X(), all_exit_lines[i].getmidPoint().Y()), poly_surface) == false)
        if((pointInPolygon(all_exit_lines[i].getmidPoint(), polygon) == false) || (all_exit_lines[i].length() < 1000) || (all_exit_lines[i].length() > 2000))
        {
            all_exit_lines.erase(all_exit_lines.begin()+i);
            i--;
        }
    }
    
//    vector<Object> temp;
//    temp = convertExitToObject(all_exit_lines);
//    for(int i = 0; i < temp.size(); i++)
//    {
//        if(Intersect_number(real_boundary, temp[i]) > 2)
//        {
//            all_exit_lines.erase(all_exit_lines.begin()+i);
//            i--;
//        }
//    }
    
    potential_exits_rnt = all_exit_lines;
    
    return potential_exits_rnt;
}


vector<Object> trim_boundary_base_exit(vector<Object> boundary, Exit exit_on_boundary)
{
    vector<Object> trimmed_boundary;
    unsigned int begin_flag = 0;
    
    for(int i = 0; i < boundary.size(); i++)
    {
        if(((boundary[i].getP2() == exit_on_boundary.getP1()) 
            || (twoLinintersect(boundary[i].getP1(), boundary[i].getP2(), exit_on_boundary.getP1(), exit_on_boundary.getP2()) == true))
             && (begin_flag == 0))
        {
            trimmed_boundary.push_back(boundary[i+1]);
            begin_flag = 1;
            continue;
        }
        
        if(((boundary[i].getP1() == exit_on_boundary.getP2())
            || (twoLinintersect(boundary[i].getP1(), boundary[i].getP2(), exit_on_boundary.getP1(), exit_on_boundary.getP2()) == true))
            && (begin_flag == 1))
        {
            //trimmed_boundary.push_back(boundary[i]);
            begin_flag = 0;
            break;
        }
        
        if(begin_flag == 1)
            trimmed_boundary.push_back(boundary[i]);
    }
    
    return trimmed_boundary;
}

vector<Object> trim_boundary_base_exit_region(vector<Object> boundary, Exit exit_on_boundary)
{
    vector<Object> trimmed_boundary;
    unsigned int begin_flag = 0;
    
    for(int i = 0; i < boundary.size(); i++)
    {
        if(((boundary[i].getP2() == exit_on_boundary.getP1()) 
            || (twoLinintersect(boundary[i].getP1(), boundary[i].getP2(), exit_on_boundary.getP1(), exit_on_boundary.getP2()) == true))
             && (begin_flag == 0))
        {
            trimmed_boundary.push_back(boundary[i]);
            begin_flag = 1;
            continue;
        }
        
        if(((boundary[i].getP1() == exit_on_boundary.getP2())
            || (twoLinintersect(boundary[i].getP1(), boundary[i].getP2(), exit_on_boundary.getP1(), exit_on_boundary.getP2()) == true))
            && (begin_flag == 1))
        {
            //trimmed_boundary.push_back(boundary[i]);
            begin_flag = 0;
            //break;
        }
        
        if(begin_flag == 0)
            trimmed_boundary.push_back(boundary[i]);
    }
    
    return trimmed_boundary;
}


Exit confirm_exit_position(vector<Object> view, Object exit)
{
    vector<Point> convert_points;
    convert_points = ObjectToPoints(view);
    
    
    Point p1, p2; 
    double min1, min2;
    
    for(int i = 0; i < convert_points.size(); i++)
    {
        double dist1, dist2;
        dist1 = exit.distP1ToPoint(convert_points[i].X(), convert_points[i].Y());
        dist2 = exit.distP2ToPoint(convert_points[i].X(), convert_points[i].Y());
        
        if(i == 0)
        {
            min1 = dist1;
            min2 = dist2;
            p1 = p2 = convert_points[i];
        }
        else
        {
            if(dist1 < min1)
            {
                min1 = dist1;
                p1 = convert_points[i];
            }
            
            if(dist2 < min2)
            {
                min2 = dist2;
                p2 = convert_points[i];
            }
        }
        
    }
    
    Exit rnt;
    rnt.set(p1.X(), p1.Y(), p2.X(), p2.Y());
    
    return rnt;
}


Exit most_constrain_exit_cross_path(vector<Object> view, Object path_segment)
{
            Exit constrain_exit;
    
            vector<Exit> potential_exits = Potentialine_basePoints(view, path_segment);

            double min = 0;
            for(int j = 0; j < potential_exits.size(); j++)
            {
                if(twoLinintersect(potential_exits[j].getP1(), potential_exits[j].getP2(), path_segment.getP1(), path_segment.getP2()) == true)
                {
                    if(j == 0)
                    {
                            if((potential_exits[j].getP1() != view[0].getP1())
                                && (potential_exits[j].getP2() != view.back().getP2()))
                            {
                                min = potential_exits[j].length();
                                constrain_exit = potential_exits[j];
                            }
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
            }
            
            Object specific_surface;
            for(int i = 0; i < view.size(); i++)
            {
                if((view[i].getP1() == constrain_exit.getP2())
                    || (view[i].getP2() == constrain_exit.getP2()))
                {
                    specific_surface = view[i];
                    break;
                }
            }
            
            if(P_To_ShortestDistance(constrain_exit.getP1(), specific_surface) < constrain_exit.length())
            {
                Point p = crossPerpend(specific_surface.getP1(), specific_surface.getP2(), constrain_exit.getP1());
                constrain_exit.set(constrain_exit.X1(), constrain_exit.Y1(), p.X(), p.Y());
            }
            
            return constrain_exit;
}

vector<Exit> crossed_exits_along_path(vector<Exit> exits, vector<Object> path_in_mfis)
{
    
    vector<Exit> temp;
    
    for(int i = 0; i < exits.size(); i++)
    {
        for(int j = 0; j < path_in_mfis.size(); j++)
        {
            if(twoLinintersect(exits[i].getP1(), exits[i].getP2(), path_in_mfis[j].getP1(), path_in_mfis[j].getP2()) == true)
            {
                temp.push_back(exits[i]);
                break;
            }
        }
    }
    
    
    for(int i = 0; i < temp.size(); i++)
    {
        for(int j = i+1; j < temp.size(); j++)
        {
            if((distanceOftwoP(temp[i].getP1(), temp[j].getP1()) < 500 && distanceOftwoP(temp[i].getP2(), temp[j].getP2()) < 500)
                || (distanceOftwoP(temp[i].getP1(), temp[j].getP1()) < 500 && distanceOftwoP(temp[i].getmidPoint(), temp[j].getmidPoint()) < 500)
                || (distanceOftwoP(temp[i].getP2(), temp[j].getP2()) < 500 && distanceOftwoP(temp[i].getmidPoint(), temp[j].getmidPoint()) < 500))
            {
                temp.erase(temp.begin()+j);
                j--;
            }
        }
    }
    
    
    return temp;
}