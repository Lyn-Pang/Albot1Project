// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.

#include <algorithm>
#include <vector>
#include <cfloat>
#include <valarray>

#include "Point.H"
#include "Object.H"
#include "ConvexHull.h"
#include "GeometricOp.H"

using namespace std;

//typedef double coord_t;         // coordinate type
//typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2

vector<Point_Hull> convex_points;
vector<Point_Hull> convex_shape;

//vector<Point_Hull> convex_points;
//vector<Point_Hull> convex;

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double cross(const Point_Hull &O, const Point_Hull &A, const Point_Hull &B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
vector<Point_Hull> convex_hull(vector<Point_Hull> P)
{
	int n = P.size(), k = 0;
	if (n == 1) return P;
	vector<Point_Hull> H(2*n);

	// Sort points lexicographically
	sort(P.begin(), P.end());

	// Build lower hull
	for (int i = 0; i < n; ++i) 
        {
		while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}

	// Build upper hull
	for (int i = n-2, t = k+1; i >= 0; i--) 
        {
		while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}

	H.resize(k-1);
	return H;
}

vector<Point_Hull> PointToHull(vector<Point> P)
{
    vector<Point_Hull> temp;
    Point_Hull temp_p;
    
    for(int i = 0; i < P.size(); i++)
    {
        temp_p.x = P[i].X();
        temp_p.y = P[i].Y();
        
        temp.push_back(temp_p);         
    }
    
    return temp;
}

vector<Object> ConvexToObject(vector<Point_Hull> convex)
{
    Object temp_Obj;
    vector<Object> temp;
    
    for(int i = 0; i < convex.size()-1; i++)
    {
        temp_Obj.set(convex[i].x, convex[i].y, convex[i+1].x, convex[i+1].y, i);
        temp.push_back(temp_Obj);
    }
    
    temp_Obj.set(convex.back().x, convex.back().y, convex[0].x, convex[0].y, convex.size());
    temp.push_back(temp_Obj);
    
    return temp;
}


vector<Point_Hull> ObjectToHull(vector<Object> view)
{
    vector<Point_Hull> temp;
    Point_Hull temp_p;
    
    for(int i = 0; i < view.size(); i++)
    {
        temp_p.x = view[i].X1();
        temp_p.y = view[i].Y1();
        
        temp.push_back(temp_p); 

        temp_p.x = view[i].X2();
        temp_p.y = view[i].Y2();
        
        temp.push_back(temp_p);         
    }
    
    return temp;
}


// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
vector<Point_Hull> convex_hull_modify(vector<Point_Hull> P)
{
	int n = P.size(), k = 0;
	if (n == 1) return P;
	vector<Point_Hull> H(2*n);

	// Sort points lexicographically
	sort(P.begin(), P.end());

	// Build lower hull
	for (int i = 0; i < n; ++i) 
        {
		while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}

	// Build upper hull
	for (int i = n-2, t = k+1; i >= 0; i--) 
        {
		while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}

	H.resize(k-1);
	return H;
}


vector<Object> getBoundingBox(vector<Object> objs) 
{

    vector<Point_Hull> hull = ObjectToHull(objs);
    hull = convex_hull(hull);
    vector<Object> hullObjects = ConvexToObject(hull);

    vector<Object> box;

    double minArea = DBL_MAX;

    for (int i = 0; i < hullObjects.size(); i++) 
    {

        Object o = hullObjects.at(i);

        // point farthest left of surface
        double dist1 = -DBL_MAX;
        Point p1;
        // point farthest right of surface
        double dist2 = DBL_MAX;
        Point p2;
        // point farthest opposite of surface
        double dist3 = -DBL_MAX;
        Point p3;
        for (int j = 0; j < hullObjects.size(); j++) 
        {

            Object p = hullObjects.at(j);

            // check first point of potential surface
            double v1[] = {o.X2() - o.X1(), o.Y2() - o.Y1()}; // first point of ref surface to second point of ref surface
            double v2[] = {p.X1() - o.X1(), p.Y1() - o.Y1()}; // first point of ref surface to first point of potential surface

            double v1_norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
            double v1_dot_v2 = v1[0] * v2[0] + v1[1] * v2[1];

            double scalar_proj = v1_dot_v2 / v1_norm;

            double a1[] = {(v1[0] * scalar_proj) / v1_norm, (v1[1] * scalar_proj) / v1_norm};
            double a2[] = {v2[0] - a1[0], v2[1] - a1[1]};
            double vector_proj = sqrt(a2[0] * a2[0] + a2[1] * a2[1]);

            //            cout << "refSurf: (" << o.X1() << ", " << o.Y1() << ", " << o.X2() << ", " << o.Y2() << ")" << endl;
            //            cout << "potSurf: (" << p.X1() << ", " << p.Y1() << ", " << p.X2() << ", " << p.Y2() << ")" << endl;
            //            cout << "v1: (" << v1[0] << ", " << v1[1] << ")" << endl;
            //            cout << "v2: (" << v2[0] << ", " << v2[1] << ")" << endl;
            //            cout << "scalar_proj: " << scalar_proj << endl;
            //            cout << "vector_proj: " << vector_proj << endl;

            //waitHere2();

            if (dist1 < scalar_proj) 
            {
                dist1 = scalar_proj;
                p1 = p.getP1();
            }
            if (dist2 > scalar_proj) 
            {
                dist2 = scalar_proj;
                p2 = p.getP1();
            }
            if (dist3 < vector_proj) 
            {
                dist3 = vector_proj;
                p3 = p.getP1();
            }

            // check second point of potential surface
            double v3[] = {p.X2() - o.X1(), p.Y2() - o.Y1()}; // first point of ref surface to second point of potential surface

            double v1_dot_v3 = v1[0] * v3[0] + v1[1] * v3[1];

            scalar_proj = v1_dot_v3 / v1_norm;

            a1 = {v1[0] / v1_norm*scalar_proj, v1[1] / v1_norm * scalar_proj};
            a2 = {v3[0] - a1[0], v3[1] - a1[1]};
            vector_proj = sqrt(a2[0] * a2[0] + a2[1] * a2[1]);

            if (dist1 < scalar_proj) 
            {
                dist1 = scalar_proj;
                p1 = p.getP2();
            }
            if (dist2 > scalar_proj) 
            {
                dist2 = scalar_proj;
                p2 = p.getP2();
            }
            if (dist3 < vector_proj) 
            {
                dist3 = vector_proj;
                p3 = p.getP2();
            }
        }

        double v1[] = {o.X2() - o.X1(), o.Y2() - o.Y1()}; // first point of ref surface to second point of ref surface
        double v1_norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
        double v1_unit[] = {v1[0] / v1_norm, v1[1] / v1_norm};

        double vp1[] = {p1.X() - o.X1(), p1.Y() - o.Y1()};
        double v1_dot_vp1 = v1[0] * vp1[0] + v1[1] * vp1[1];
        double scalar_p1 = v1_dot_vp1 / v1_norm;
        double a1[] = {v1_unit[0] * scalar_p1, v1_unit[1] * scalar_p1};
        double box_p1[] = {o.X1() + a1[0], o.Y1() + a1[1]};

        double vp2[] = {p2.X() - o.X1(), p2.Y() - o.Y1()};
        double v1_dot_vp2 = v1[0] * vp2[0] + v1[1] * vp2[1];
        double scalar_p2 = v1_dot_vp2 / v1_norm;
        double a2[] = {v1_unit[0] * scalar_p2, v1_unit[1] * scalar_p2};
        double box_p2[] = {o.X1() + a2[0], o.Y1() + a2[1]};

        double v_p1_bp1[] = {box_p1[0] - p1.X(), box_p1[1] - p1.Y()};
        double norm_p1_bp1 = sqrt(v_p1_bp1[0] * v_p1_bp1[0] + v_p1_bp1[1] * v_p1_bp1[1]);
        double unit_p1_bp1[] = {v_p1_bp1[0] / norm_p1_bp1, v_p1_bp1[1] / norm_p1_bp1};

        double v_p1_p3[] = {p3.X() - p1.X(), p3.Y() - p1.Y()};
        double p1_bp1_dot_p1_p3 = v_p1_bp1[0] * v_p1_p3[0] + v_p1_bp1[1] * v_p1_p3[1];
        double scalar_a13 = p1_bp1_dot_p1_p3 / norm_p1_bp1;
        double a13[] = {unit_p1_bp1[0] * scalar_a13, unit_p1_bp1[1] * scalar_a13};
        double box_p3[] = {p1.X() + a13[0], p1.Y() + a13[1]};

        double v_p2_bp2[] = {box_p2[0] - p2.X(), box_p2[1] - p2.Y()};
        double norm_p2_bp2 = sqrt(v_p2_bp2[0] * v_p2_bp2[0] + v_p2_bp2[1] * v_p2_bp2[1]);
        double unit_p2_bp2[] = {v_p2_bp2[0] / norm_p2_bp2, v_p2_bp2[1] / norm_p2_bp2};

        double v_p2_p3[] = {p3.X() - p2.X(), p3.Y() - p2.Y()};
        double p2_bp2_dot_p2_p3 = v_p2_bp2[0] * v_p2_p3[0] + v_p2_bp2[1] * v_p2_p3[1];
        double scalar_a23 = p2_bp2_dot_p2_p3 / norm_p2_bp2;
        double a23[] = {unit_p2_bp2[0] * scalar_a23, unit_p2_bp2[1] * scalar_a23};
        double box_p4[] = {p2.X() + a23[0], p2.Y() + a23[1]};

        //fix a bug, when box_p4 is NULL , modified by Lyn
        if(norm_p2_bp2 == 0 && p2_bp2_dot_p2_p3 == 0)  
        {
            
            double dist;
            double x_mid, y_mid;
            x_mid  = (box_p3[0] + box_p2[0]) / 2;
            y_mid = (box_p3[1] + box_p2[1]) / 2;
            
            dist = distanceOftwoP(Point (box_p1[0], box_p1[1]), Point (x_mid, y_mid));
            
            Object temp;
            temp.set(box_p1[0], box_p1[1], x_mid, y_mid, 0);
            Object exp = expend_Object(temp, dist, 2);
            
            box_p4[0] = exp.X2();
            box_p4[1] = exp.Y2();
            
        }
        
        double width = sqrt(pow(box_p2[0] - box_p1[0], 2) + pow(box_p2[1] - box_p1[1], 2));
        double length = sqrt(pow(box_p3[0] - box_p1[0], 2) + pow(box_p3[1] - box_p1[1], 2));

        double area = width*length;

        if (area < minArea) 
        {
            minArea = area;
            //draw actual rectangle here
            Object obj1;
            obj1.set(box_p1[0], box_p1[1], box_p2[0], box_p2[1], 0);
            Object obj2;
            obj2.set(box_p1[0], box_p1[1], box_p3[0], box_p3[1], 1);
            Object obj3;
            obj3.set(box_p2[0], box_p2[1], box_p4[0], box_p4[1], 2);
            Object obj4;
            obj4.set(box_p3[0], box_p3[1], box_p4[0], box_p4[1], 3);

            Object refSurf;
            refSurf.set(o.X1(), o.Y1(), o.X2(), o.Y2(), 4);

            vector<Object> tempBox;
            tempBox.push_back(obj1);
            tempBox.push_back(obj2);
            tempBox.push_back(obj3);
            tempBox.push_back(obj4);
            tempBox.push_back(refSurf);

            box = tempBox;
        }

    }

    return box;
}

