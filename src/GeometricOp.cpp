#include <vector>
#include <iostream>
#include <cmath>
#include <numeric>

#include "GeometricOp.H"

#include "Point.H"
#include "Object.H"
#include "mfisOp.H"
#include "Minfo.H"
#include "Plotting.H"
#include "clipper.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace ClipperLib;

#define PI 3.14159265



//transform previous view into current view

vector<Object> xformPVIntoCV(vector<Object> pview, Point rpose, double angle) 
{
        cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
        vector<Object> result;
        double x1, y1, x2, y2;
        //angle=angle*(180/PI);
        for (int i = 0; i<int(pview.size()); i++) 
        {
                double a = pview[i].X1() - rpose.X(); //x-x0
                double b = pview[i].Y1() - rpose.Y(); //y-y0

                x1 = a * cos(angle) + b * sin(angle);
                y1 = b * cos(angle) - a * sin(angle);

                double c = pview[i].X2() - rpose.X(); //x-x0
                double d = pview[i].Y2() - rpose.Y(); //y-y0

                x2 = c * cos(angle) + d * sin(angle);
                y2 = d * cos(angle) - c * sin(angle);

                Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());
                s.setLocalEnvID(pview[i].getLocalEnvID());
                s.setVN(pview[i].getVN());
                result.push_back(s);
        }

        return result;

}

vector<Point> xformPointsIntoCV(vector<Point> pview, Point rpose, double angle) {
    vector<Point> result;
    double x1, y1;
    //angle=angle*(180/PI);
    for (int i = 0; i<int(pview.size()); i++) {

        double a = pview[i].X() - rpose.X(); //x-x0
        double b = pview[i].Y() - rpose.Y(); //y-y0

        x1 = a * cos(angle) + b * sin(angle);
        y1 = b * cos(angle) - a * sin(angle);

        
        //Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());
        result.push_back(Point(x1,y1));
    }
    return result;
}


//transform potential objects of pv into current view
vector<Object> xformPObjectsIntoCV(vector<Object> pview, Point rpose, double angle)
{//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
	vector<Object> result;
	double x1, y1, x2, y2;
//angle=angle*(180/PI);
	for(int i=0;i<int(pview.size());i++)
	{
				
		double a=pview[i].X1()-rpose.X(); //x-x0
		double b=pview[i].Y1()-rpose.Y(); //y-y0

		x1=a*cos(angle)+b*sin(angle);
		y1=b*cos(angle)-a*sin(angle);
                
                                    //x1=a*cos(angle)-b*sin(angle);
		//y1=b*cos(angle)+a*sin(angle);
		
		double c=pview[i].X2()-rpose.X(); //x-x0
		double d=pview[i].Y2()-rpose.Y(); //y-y0
	
		x2=c*cos(angle)+d*sin(angle);
		y2=d*cos(angle)-c*sin(angle);
                
                                    //x2=c*cos(angle)-d*sin(angle);
		//y2=d*cos(angle)+c*sin(angle);
		
		if(y1 > 0) {
			
			
			if(y2 > 0) {
				Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());				s.setKP(pview[i].getKP());
				result.push_back(s);
			}
		}
		//for first object
		if(y1 < 0 && y2 > 500 && pview[i].getKP() > 1) {
			Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());			s.setKP(2);
			result.push_back(s);
		}
		//for last object
		if(y1 > 500 && y2 < 0 && pview[i].getKP() == 1) {
			Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());			s.setKP(1);
			result.push_back(s);
		}
		//cout<<"id "<<pview[i].getID()<<" KP- "<<pview[i].getKP()<<" y1 "<<y1<<" y2 "<<y2<<endl;
	}
	return result;
}



vector<Object> discardLinesIFoR(vector<Object> pview, Point rpose, double angle, vector<Object> cview, Object rmfis,Object rcv,int rp)
{//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
	vector<Object> result;
	double x1, y1, x2, y2;
	//finding left and right boundary from current view
	double leftb=0;
	double rightb=0;
	double max=0;
	for(int i=0;i<int(cview.size());i++) {
		if(cview[i].X1() < leftb) {//have problem
			leftb=cview[i].X1();
		}
		if(cview[i].X2() < leftb) {
			leftb=cview[i].X2();
		}

		if(cview[i].X1() > rightb) {
			rightb=cview[i].X1();
		}
		if(cview[i].X2() > rightb) {
			rightb=cview[i].X2();
		}

		if(cview[i].Y1() > max) {
			max=cview[i].Y1();
		}
		if(cview[i].Y2() > max) {
			max=cview[i].Y2();
		}
	}
	
	//cout<<"cv "<<endl;
	//displayObjects(cview);
	//cout<<endl<<"left b "<<leftb<<" right b"<<rightb<<" max "<<max<<endl;
//angle=angle*(180/PI);
/*
	if(leftb > 10000) {
		leftb = 10000;
	}
	if(rightb > 10000) {
		rightb=10000;
	}
	if(max > 10000) {
		max=10000;
	}*/
leftb=-10000;rightb=10000;max=12000;

	bool insert_first_line_of_cv=true;
	for(int i=0;i<int(pview.size());i++)
	{
				
		double a=pview[i].X1()-rpose.X(); //x-x0
		double b=pview[i].Y1()-rpose.Y(); //y-y0

		x1=a*cos(angle)+b*sin(angle);
		y1=b*cos(angle)-a*sin(angle);

		double c=pview[i].X2()-rpose.X(); //x-x0
		double d=pview[i].Y2()-rpose.Y(); //y-y0

		x2=c*cos(angle)+d*sin(angle);
		y2=d*cos(angle)-c*sin(angle);
		
		bool blines=true;
		/*
			condition (y1>0 or y2>0) is to replace the line which is on both side
			condition (y1>0) and then again (y2>0) is to keep both of them

		*/
		//for replacement
		if(y1 > 0|| y2 > 0) {
			
			
			//if(y2 > 0) {
			if(x1 < 0 && x1 > leftb) {
				//blines=false;
				if(y1 > 0 && y2 > 0) {
					if(y1 < max && y2 < max) 
						blines=false;
				}
			}
			if(x1 > 0 && x1 < rightb) {
				//blines=false;
				if(y1 > 0 && y2 > 0) {
					if(y1 < max && y2 < max) 
						blines=false;
				}
			}

			
			//}
		}

		//for using both
	/*	if(y1 > 0 && y2 > 0) {
			
			
			//if(y2 > 0) {
			if(x1 < 0 && x1 > leftb) {
				blines=false;
			}
			if(x1 > 0 && x1 < rightb) {
				blines=false;
			}
			//}
		}*/


		//for extending
		if(y1 < 0 && y2 > 0) {
			Object line1=remakeLineP2(rmfis,rcv,cview[0],1,0, rp);
			double s_dist=pview[i].perpendicularDistOfPoint(line1.X1(),line1.Y1());
			//cout<<"extending mfis Object as first Object of cview "<<s_dist<<endl;
			if(s_dist < 300) {
				pview[i].setP2(line1.X2(),line1.Y2());
				result.push_back(pview[i]);
				blines = false;
			}
			else
				result.push_back(line1);
			insert_first_line_of_cv=false;
		}

		if(y1 > 0 && y2 < 0) {
			Object line1=remakeLineP2(rmfis,rcv,cview[cview.size()-1],1,0, rp);
			double s_dist=pview[i].perpendicularDistOfPoint(line1.X1(),line1.Y1());
			//cout<<"extending mfis Object as last Object of cview "<<s_dist<<endl;
			if(s_dist < 300) {
				pview[i].setP1(line1.X1(),line1.Y1());
				result.push_back(pview[i]);
				blines = false;
			}
			//else
			//	result.push_back(line1);
		}
		

		
		if(blines==true) {
			result.push_back(pview[i]);
		}

		
	}
	if(insert_first_line_of_cv == true){
		Object line1=remakeLineP2(rmfis,rcv,cview[0],1,0, rp);
		result.push_back(line1);
	}
		
	return result;
}




/*
//transform previous ref points into current view
vector<RfPoint> xformRefPointsIntoCV(vector<RfPoint> pview, Point rpose, double angle)
{
	vector<RfPoint> result;
	double x1, y1;
//angle=angle*(180/PI);
	for(int i=0;i<int(pview.size());i++)
	{
				
		double a=pview[i].X()-rpose.X(); //x-x0
		double b=pview[i].Y()-rpose.Y(); //y-y0

		x1=a*cos(angle)+b*sin(angle);
		y1=b*cos(angle)-a*sin(angle);
		
		if(y1 > 0) {
			RfPoint p(x1, y1);
			p.setYth();
			p.setID(pview[i].getID());
			result.push_back(p);
		}
	}
	return result;
}
*/

//most probably it doesn't work. CHECK
//transform current view into previous view
vector<Object> xformCVIntoPV(vector<Object> cview, Point rpose, double angle)
{//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
	vector<Object> result;
	double x1, y1, x2, y2;
                  //angle=angle*(180/PI);
	for(int i=0;i<int(cview.size());i++)
	{
				
		x1=cview[i].X1()*cos(angle)-cview[i].Y1()*sin(angle)+rpose.X();
		y1=cview[i].X1()*sin(angle)+cview[i].Y1()*cos(angle)+rpose.Y();
		
		x2=cview[i].X2()*cos(angle)-cview[i].Y2()*sin(angle)+rpose.X();
		y2=cview[i].X2()*sin(angle)+cview[i].Y2()*cos(angle)+rpose.Y();
		
		Object s(x1, y1, x2, y2, cview[i].getID(), cview[i].nearness(), 
                                        cview[i].getP1OS(), cview[i].getP2OS(), cview[i].getGID());
		result.push_back(s);
		
	}
	return result;
}


//returns intersection point of two lines
vector<double> getIntersectionPoint(Object s1, Object s2)
{
	
	double x1= s1.X1();	
	double y1= s1.Y1();		
	double x2= s1.X2();	
	double y2= s1.Y2();	

	double x3= s2.X1();
	double y3= s2.Y1();
	double x4= s2.X2();
	double y4= s2.Y2();

	double y43= y4-y3;
	double x21= x2-x1;
	double x43= x4-x3;
	double y21= y2-y1;

	double u_deno=y43*x21-x43*y21;
	
	double y13= y1-y3;
	double x13= x1-x3;
	double ua_num= x43*y13-y43*x13;
	//double ub_num= x21*y13-y21*x13;

	double ua=ua_num/u_deno;
	//double ub=ub_num/u_deno;
	
	double x=x1+ua*x21;
	double y=y1+ua*y21;
	
	vector<double> result;
	result.push_back(x);
	result.push_back(y);

	//cout<<" x "<<x<<" y "<<y<<endl;
	return result;
}

//return 1 if both segment has intersection point
double checkForIntersection (Object s1, Object s2)
{
					
 
	double x1= s1.X1();	
	double y1= s1.Y1();
	double x2= s1.X2();
	double y2= s1.Y2();

	double x3= s2.X1();
	double y3= s2.Y1();
	double x4= s2.X2();
	double y4= s2.Y2();

	double y43= y4-y3;
	double x21= x2-x1;
	double x43= x4-x3;
	double y21= y2-y1;

	double u_deno=y43*x21-x43*y21;
	
	double y13= y1-y3;
	double x13= x1-x3;
	double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	//cout<<"ua "<<ua<<" ub "<<ub<<endl;
	if(ua>=0 && ua<=1)
		if(ub>=0 && ub<=1)
			return 1;
		else
		return 0;
	else
	return 0;
}


//it returns abs distance between two points
double getDistBtw2Points(double x1, double y1, double x2, double y2)
{
	double x=x1-x2;
	double y=y1-y2;
	return sqrt(x*x+y*y);
}

double angleObjectAndPoint(Object s, double x3, double y3)
{
	double x21=s.X2()-s.X1();
	double y21=s.Y2()-s.Y1();
	double x31=x3-s.X1();
	double y31=y3-s.Y1();
	
	double la=s.length();
	double lb=getDistBtw2Points(s.X1(), s.Y1(), x3, y3);

	/*if(lb==0)
	return 0;
	
	double r=(x21*x31+y21*y31)/(la*lb);*/
	double r1=x21/la;
	double angle1=acos(r1);

	double r2=x31/lb;
	double angle2=acos(r2);
	
	if(lb==0)
	return 0;

	if(y21 < 0)
	angle1 = 2*PI - angle1;	
	angle1=((180/PI)*angle1);
	
	if(y31<0)
	angle2=2*PI-angle2;
	angle2=((180/PI)*angle2);
	
	double diff=angle2-angle1;

	if(diff>0)
	return diff;
	else 
	return 360+diff;
}

double angleObjectAndXaxis(Object s)
{
	double x21=s.X2()-s.X1();
	double y21=s.Y2()-s.Y1();
	
	
	double la=s.length();
	
	/*if(lb==0)
	return 0;
	
	double r=(x21*x31+y21*y31)/(la*lb);*/
	double r1=x21/la;
	double angle1=acos(r1);

	

	if(y21 < 0)
	angle1 = 2*PI - angle1;	
	

	return ((180/PI)*angle1);	
}
//not general it's only for robot
//converts polar to cartesian coordinates for robot position 
vector<Point> p2cCoordinate(vector<double> vec)
{
	vector<Point> result;
	for(int i=0;i<int(vec.size());i+=2)
	{
		double angle=vec[i];
		double dist=vec[i+1];

		angle = (angle/180)*PI;
		double x=dist*sin(angle);
		double y=dist*cos(angle);
		
		Point p(x, y);
		result.push_back(p);
	}
	return result;
}

//it returns x coordinate in certesian system
//angle in deg and with respect to y axis
double getX(double angle,double dist) {
	return dist*sin(-(PI/180)*angle);
}

//it returns y coordinate in certesian system
//angle in deg and with respect to y axis
double getY(double angle,double dist) {
	return dist*cos(-(PI/180)*angle);
}


//it return 1 if inside else 0
//Ref: http://paulbourke.net/geometry/insidepoly/
//solution 3
bool pointInPolygon(Object a, Object b, double x, double y) 
{
	//double x0,y0,x1,y1;
	double test;
	vector<double> px, py;
	px.push_back(a.X1());
	px.push_back(a.X2());
	px.push_back(b.X2());
	px.push_back(b.X1());
	px.push_back(a.X1());

	py.push_back(a.Y1());
	py.push_back(a.Y2());
	py.push_back(b.Y2());
	py.push_back(b.Y1());
	py.push_back(a.Y1());

	bool inside=true;
	for(int i=0;i<4;i++) {
		test=((y-py[i])*(px[i+1]-px[i]))-((x-px[i])*(py[i+1]-py[i]));
		if(test > 0) 
                                    {
			inside = false;
			return inside;
		}
	}
	return inside;
			
}

//calling clipper algorithm
bool pointInPolygon(Point p, vector<Object> view) 
{
    IntPoint temp_p;
    Path polygon;
    temp_p.X = p.X();
    temp_p.Y = p.Y();
    
    polygon = viewConvertPath(view);
    
    //0 - false  1/-1  true
    if(PointInPolygon(temp_p, polygon) != 0)
        return true;
    else
        return false;
}

vector<Object> breakTheLineInto(Object smfis)
{
	vector<Object> output;
	double x1, y1, x2, y2;
	double angle=0;
	double dist=0;
	x1=smfis.X1();
	y1=smfis.Y1();

		int id=0;
		while(dist < double(smfis.length()-50)) {
		dist=dist+150;
		x2=((smfis.X2()-smfis.X1())/smfis.length())*cos(angle);
		y2=((smfis.Y2()-smfis.Y1())/smfis.length())*cos(angle);

		x2=x2*dist+smfis.X1();
		y2=y2*dist+smfis.Y1();

		Object oneline(x1,y1,x2,y2, id+1);
		output.push_back(oneline);

		dist=dist+300;
		x2=((smfis.X2()-smfis.X1())/smfis.length())*cos(angle);
		y2=((smfis.Y2()-smfis.Y1())/smfis.length())*cos(angle);

		x2=x2*dist+smfis.X1();
		y2=y2*dist+smfis.Y1();
		x1=x2;
		y1=y2;
		}
	return output;
}

vector<Object> breakTheLinesInto(vector<Object> allLines) {
    vector<Object> result;
    vector<Object> temp;
    for(unsigned int i=0;i<allLines.size();i++) {
        temp = breakTheLineInto(allLines[i]);
        for(unsigned int j=0;j<temp.size();j++)
                result.push_back(temp[j]);
    }
    return result;
}

//it makes a line with respect to object using angle and dist.

Object makeLineAtPointWithObject(double angle, double dist, Object smfis) {
    Object output;
    double x1, y1, x2, y2;
    x1 = smfis.X1();
    y1 = smfis.Y1();

    angle = (angle / 180) * PI; //angle in radian

    x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x1 = x1 * dist + smfis.X1();
    y1 = y1 * dist + smfis.Y1();

    dist = dist + 300;
    x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x2 = x2 * dist + smfis.X1();
    y2 = y2 * dist + smfis.Y1();

    output.set(x1, y1, x2, y2, 1);

    return output;
}

//it makes a line with respect to object using angle and dist.
//length is the length of object

Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis) {
    Object output;
    double x1, y1, x2, y2;
    x1 = smfis.X1();
    y1 = smfis.Y1();

    angle = (angle / 180) * PI; //angle in radian

    x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x1 = x1 * dist + smfis.X1();
    y1 = y1 * dist + smfis.Y1();

    dist = dist + length;
    x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x2 = x2 * dist + smfis.X1();
    y2 = y2 * dist + smfis.Y1();

    output.set(x1, y1, x2, y2, 1);

    return output;
}

//it makes a line with respect to object using angle and dist.
//length is the length of object
//argument angle is in radian

Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis, int a) {
    Object output;
    double x1, y1, x2, y2, x, y;
    x1 = smfis.X1();
    y1 = smfis.Y1();

    //angle = (angle / 180) * PI; //angle in radian
    if(a == 1)
    {
        x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
        y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

        x1 = x1 * dist + smfis.X1();
        y1 = y1 * dist + smfis.Y1();

        dist = dist + length;
        x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
        y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

        x2 = x2 * dist + smfis.X1();
        y2 = y2 * dist + smfis.Y1();
    }
    
    if(a == 2)
    {
        x = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
        y = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

        x2 = x * dist + smfis.X1();
        y2 = y * dist + smfis.Y1();

        dist = dist + length;
        x = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
        y = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

        x1 = x * dist + smfis.X1();
        y1 = y * dist + smfis.Y1();
    }
    

    output.set(x1, y1, x2, y2, 1);

    return output;
}

vector<Object> makeSquareAtLoSPoints(vector<Object> lineOfSitePoints) {
    Object obj1,obj2,side;
    vector<Object> output;
    for(int i=0;i<int(lineOfSitePoints.size());i++) {
        obj1 = makeLineAtPointWithObject(45,0,500,lineOfSitePoints[i]);
        output.push_back(obj1);
        obj1.reverse();
        obj1 = makeLineAtPointWithObject(90,0,500,obj1);
        output.push_back(obj1);
        obj1.reverse();
        obj1 = makeLineAtPointWithObject(90,0,500,obj1);
        output.push_back(obj1);
        obj1.reverse();
        obj1 = makeLineAtPointWithObject(90,0,500,obj1);
        output.push_back(obj1);
        
//        obj2 = makeLineAtPointWithObject(315,0,lineOfSitePoints[i]);
//        
//        
//        side.set(obj1.X2(),obj1.Y2(),lineOfSitePoints[i].X2(),lineOfSitePoints[i].Y2(),2);
//        output.push_back(side);
//        side.set(lineOfSitePoints[i].X2(),lineOfSitePoints[i].Y2(),obj2.X2(),obj2.Y2(),3);
//        output.push_back(side);
//        output.push_back(obj2);
//        output.push_back(lineOfSitePoints[i]);
    }
    return output;
}

//it takes angle and dist of two points with respect to the given point
//make and return the line at those two point
///*
Object makeLineAtTwoPointsWithObject(double angle1, double dist1, double angle2, double dist2, Object smfis,int referencePoint) {
    Object output;
    double x1, y1, x2, y2;


    angle1 = (angle1 / 180) * PI; //angle in radian
    angle2 = (angle2 / 180) * PI; //angle in radian

    if (referencePoint == 1) {
        x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle1)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle1);
        y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle1)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle1);

        x1 = x1 * dist1 + smfis.X1();
        y1 = y1 * dist1 + smfis.Y1();

        x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle2)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle2);
        y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle2)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle2);

        x2 = x2 * dist2 + smfis.X1();
        y2 = y2 * dist2 + smfis.Y1();
    }
    else 
    {
        x1 = ((smfis.X1() - smfis.X2()) / smfis.length()) * cos(angle1)-((smfis.Y1() - smfis.Y2()) / smfis.length()) * sin(angle1);
        y1 = ((smfis.X1() - smfis.X2()) / smfis.length()) * sin(angle1)+((smfis.Y1() - smfis.Y2()) / smfis.length()) * cos(angle1);

        x1 = x1 * dist1 + smfis.X2();
        y1 = y1 * dist1 + smfis.Y2();

        x2 = ((smfis.X1() - smfis.X2()) / smfis.length()) * cos(angle2)-((smfis.Y1() - smfis.Y2()) / smfis.length()) * sin(angle2);
        y2 = ((smfis.X1() - smfis.X2()) / smfis.length()) * sin(angle2)+((smfis.Y1() - smfis.Y2()) / smfis.length()) * cos(angle2);

        x2 = x2 * dist2 + smfis.X2();
        y2 = y2 * dist2 + smfis.Y2();
    }

    output.set(x1, y1, x2, y2, 1);

    return output;
}

Object CreateLineOnObject(double angle1, double dist1, Object smfis, Object detect, int referencePoint, int side_flag) 
{
    Object output;
    double x1, y1, x2, y2;


    //angle1 = (angle1 / 180) * PI; //angle in radian
    //angle2 = (angle2 / 180) * PI; //angle in radian

    if (referencePoint == 1) 
    {

        x1 = smfis.X1();
        y1 = smfis.Y1();

        x2 = x1 + dist1*cos(angle1);
        y2 = y1 + dist1*sin(angle1);
        if(twoLinintersect(smfis.getP1(), detect.getP1(), smfis.getP2(), detect.getP2()) == false)
        {
            if(isLeft(smfis.getP1(), smfis.getP2(), Point (x2, y2)) == side_flag)
                goto lb; 
            else
            {
                x2 = x1 + dist1*cos(angle1+PI);
                y2 = y1 + dist1*sin(angle1+PI);
            }
        }
        else
        {
            if(isLeft(smfis.getP2(), smfis.getP1(), Point (x2, y2)) == side_flag)
                goto lb; 
            else
            {
                x2 = x1 + dist1*cos(angle1+PI);
                y2 = y1 + dist1*sin(angle1+PI);
            }
        }
    }
    else 
    {
        x1 = smfis.X2();
        y1 = smfis.Y2();
        
        x2 = x1 + dist1*cos(angle1);
        y2 = y1 + dist1*sin(angle1);
        
        if(twoLinintersect(smfis.getP1(), detect.getP1(), smfis.getP2(), detect.getP2()) == false)
        {
            if(isLeft(smfis.getP1(), smfis.getP2(), Point (x2, y2)) == side_flag)
                goto lb; 
            else
            {
                x2 = x1 + dist1*cos(angle1+PI);
                y2 = y1 + dist1*sin(angle1+PI);
            }
        }
        else
        {
            if(isLeft(smfis.getP2(), smfis.getP1(), Point (x2, y2)) == side_flag)
                goto lb; 
            else
            {
                x2 = x1 + dist1*cos(angle1+PI);
                y2 = y1 + dist1*sin(angle1+PI);
            }
        }

    }

lb: output.set(x1, y1, x2, y2, 1);

    return output;
}

void waitHere() 
{
    char wait[10];
    cin>>wait;
    return;
}

vector<Object> makeSquare(Object oneSide) {
    vector<Object> exit;
    Object tempObj;
    exit.push_back(oneSide);
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(90, 0, 1000, oneSide); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //right side line
            exit.push_back(tempObj);
            
            return exit;
}

vector<Object> makeSquare(Object oneSide, double length, int ref_point) 
{
    vector<Object> exit;
    Object tempObj;
    exit.push_back(oneSide);
    if(ref_point == 2)
        oneSide.reverse();
    
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(90, 0, length, oneSide); //left side line
            
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, length, tempObj); //parallel line of original exit
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, length, tempObj); //right side line
            exit.push_back(tempObj);
            
            return exit;
}



vector<Object> makeAllSquare(vector<Object> allLines) {
    
    vector<Object> result;
    vector<Object> temp;
    for(unsigned int i=0;i<allLines.size();i++) {
        temp = makeSquare(allLines[i]);
        for(unsigned int j=0;j<temp.size();j++)
                result.push_back(temp[j]);
    }
    return result;
    
}

vector<Object> makeRectangle(Object oneSide) {
    vector<Object> exit;
    Object tempObj;
    exit.push_back(oneSide);
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(90, 0, 500, oneSide); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            tempObj.reverse();
            //tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
            tempObj = makeLineAtPointWithObject(90, 0, oneSide.length(), tempObj);
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 500, tempObj); //right side line
            exit.push_back(tempObj);
            
            return exit;
}

vector<Object> makeAllRectangle(vector<Object> allLines) {
    
    vector<Object> result;
    vector<Object> temp;
    for(unsigned int i=0;i<allLines.size();i++) {
        temp = makeRectangle(allLines[i]);
        for(unsigned int j=0;j<temp.size();j++)
                result.push_back(temp[j]);
    }
    return result;
    
}

Object makeParallelObject(Object oneSide, double dist, const char side) {
    Object tempObj;
    //cout<<side<<endl;
    if(side == 'left') {
        
    tempObj = makeLineAtPointWithObject(90, 0, dist, oneSide);
    tempObj.reverse();
    tempObj = makeLineAtPointWithObject(-90, 0, 1000, tempObj);
    }
    else
      tempObj = makeLineAtPointWithObject(-90, 0, dist, oneSide);  
    
    
    
    return tempObj;
    //waitHere();
}

Object makeParallelObject(Object oneSide, double dist, int side) {
    Object tempObj;
    //cout<<side<<endl;
    if(side == 1) {//left
        
    tempObj = makeLineAtPointWithObject(90, dist, dist, oneSide);
    //tempObj.reverse();
    tempObj = makeLineAtPointWithObject(-90, 0, oneSide.length(), tempObj);
    }
    else
      tempObj = makeLineAtPointWithObject(-90, 0, dist, oneSide);  
    
    
    
    return tempObj;
    //waitHere();
}

vector<Object> makeArrow(Object oneSide) {
    vector<Object> exit;
    Object tempObj;
    exit.push_back(oneSide);
    
    oneSide.reverse();
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(45, 0, 500, oneSide); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            
            tempObj = makeLineAtPointWithObject(-45, 0, 500, oneSide); //parallel line of original exit
            exit.push_back(tempObj);
                        
            return exit;
}

vector<Object> drawArrow(Object oneSide, int view_num) 
{
    Object temp;
    vector<Object> exit;
    Object tempObj;
    
    temp = expend_Object(oneSide, 200, 2);
    temp.set(oneSide.X1(), oneSide.Y1(), temp.X2(), temp.Y2(), oneSide.getID());
    
    exit.push_back(temp);
    //exit.push_back(oneSide);
    
    temp.reverse();
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(45, 0, 150, temp); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            
            tempObj = makeLineAtPointWithObject(-45, 0, 150, temp); //parallel line of original exit
            exit.push_back(tempObj);
                        
            exit[0].setVN(view_num);
            return exit;
}

//it computes a boundary which is 30cm outside from cView
vector<Surface> makePolygonOfCV(vector<Object> cView) 
{
        vector<Surface> polygon;
       
        vector<Object> newCView;
        for(unsigned int i=0;i<cView.size();i++) 
        {
           //newCView.push_back(makeParallelObject(cView[i],3000,1));
            newCView.push_back(makeParallelObject(cView[i],3000,1));
        }
        //plotObjects("Maps/Offline/wideCV.png",newCView,cView);
        cView = newCView;
        //waitHere();
        
        Surface tempSurf;
        //tempSurf = Surface(PointXY(0,0),PointXY(1000,1000));
        tempSurf = Surface(PointXY(0,0),PointXY(cView[0].X1(),cView[0].Y1()));//,1);
        polygon.push_back(tempSurf);//for 0,0 to p1 of first object
        for(unsigned int i=0;i<cView.size()-1;i++) 
        {
            tempSurf = Surface(PointXY(cView[i].X1(),cView[i].Y1()),PointXY(cView[i].X2(),cView[i].Y2()));//,(i*2)+2);
            polygon.push_back(tempSurf);
            tempSurf = Surface(PointXY(cView[i].X2(),cView[i].Y2()),PointXY(cView[i+1].X1(),cView[i+1].Y1()));//,(i*2)+3)
            polygon.push_back(tempSurf);
        }
        tempSurf = Surface(PointXY(cView[cView.size()-1].X1(),cView[cView.size()-1].Y1()),
                                                     PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()));//,(cView[cView.size()-1]*2)+2);
        polygon.push_back(tempSurf);//for last surface
        tempSurf = Surface(PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()),
                                                     PointXY(0,0));//,(cView[cView.size()-1]*2)+3);
        polygon.push_back(tempSurf);//for last surface to 0,0
    
   
    
    return polygon;
    
}

//it computes a boundary which from current Local Map
vector<Surface> makePolygonOfLM(vector<Object> currentLM) 
{
           vector<Surface> polygon;
           vector<Object> cView;
       
           cView = currentLM;
           //waitHere();
           Surface tempSurf;
           //tempSurf = Surface(PointXY(0,0),PointXY(1000,1000));
           tempSurf = Surface(PointXY(0,0),PointXY(cView[0].X1(),cView[0].Y1()));//,1);
           polygon.push_back(tempSurf);//for 0,0 to p1 of first object
           for(unsigned int i=0;i<cView.size()-1;i++) 
           {
                      tempSurf = Surface(PointXY(cView[i].X1(),cView[i].Y1()),PointXY(cView[i].X2(),cView[i].Y2()));//,(i*2)+2);
                      polygon.push_back(tempSurf);
                      tempSurf = Surface(PointXY(cView[i].X2(),cView[i].Y2()),PointXY(cView[i+1].X1(),cView[i+1].Y1()));//,(i*2)+3)
                      polygon.push_back(tempSurf);
           }
           tempSurf = Surface(PointXY(cView[cView.size()-1].X1(),cView[cView.size()-1].Y1()),
                                                         PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()));//,(cView[cView.size()-1]*2)+2);
           polygon.push_back(tempSurf);//for last surface
           tempSurf = Surface(PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()),
                                                         PointXY(0,0));//,(cView[cView.size()-1]*2)+3);
           polygon.push_back(tempSurf);//for last surface to 0,0

           return polygon;
    
}

vector<Object> makePolygonOfView(vector<Object> View)
{
    Object temp_obj;
    vector<Object> rnt;
    
    for(int i = 0; i < View.size(); i++)
    {
        if(i != View.size()-1)
            temp_obj.set(View[i].X2(), View[i].Y2(), View[i+1].X1(), View[i+1].Y1(), i);
        else
            temp_obj.set(View[i].X2(), View[i].Y2(), View[0].X1(), View[0].Y1(), i);
        temp_obj.set_imagined_flag(1);    
        rnt.push_back(View[i]);
        rnt.push_back(temp_obj);
    }
    
    return rnt;
}

vector<Object> makePolygon_Clipper(vector<Object> View, double adjust_size)
{
    
    vector<Object> rnt;
    
    ClipperLib::Path subj2;
    Paths solution2;
    ClipperOffset co2;
    subj2 = viewConvertPath(View); //convert view into Path
    co2.AddPath(subj2, jtMiter, etClosedPolygon);
    co2.Execute(solution2, adjust_size); // outward offset the polygon
    SimplifyPolygons(solution2, pftNonZero);

    rnt = PathsConvertView(solution2);
    
    return rnt;
}

vector<Object> Polygons_intersection_Clipper(vector<Object> View, vector<Object> View2)
{
    
    vector<Object> rnt;
    
    ClipperLib::Path subj2, subj;
    Paths solution2;
    Clipper co2;
    subj2 = viewConvertPath(View); //convert view into Path
    subj = viewConvertPath(View2);
    
    co2.AddPath(subj2, ptSubject, true);
    co2.AddPath(subj, ptClip, true);
    co2.Execute(ctIntersection, solution2, pftNonZero, pftNonZero); // outward offset the polygon
    SimplifyPolygons(solution2, pftNonZero);
  
    rnt = PathsConvertView(solution2);
    
    return rnt;
}

//it computes a exact boundary from cView 
//added on 26 Nov 2013
vector<Surface> findExactBoudaryFrom(vector<Object> cView) {
       vector<Surface> polygon;
       
//       vector<Object> newCView;
//       for(unsigned int i=0;i<cView.size();i++) {
//           newCView.push_back(makeParallelObject(cView[i],300,1));
//           
//       }
//       //plotObjects("Maps/wideCV.png",newCView,cView);
//       cView = newCView;

    Surface tempSurf;
    //tempSurf = Surface(PointXY(0,0),PointXY(1000,1000));
    tempSurf = Surface(PointXY(0,0),PointXY(cView[0].X1(),cView[0].Y1()));//,1);
    polygon.push_back(tempSurf);//for 0,0 to p1 of first object
    for(unsigned int i=0;i<cView.size()-1;i++) {
        tempSurf = Surface(PointXY(cView[i].X1(),cView[i].Y1()),PointXY(cView[i].X2(),cView[i].Y2()));//,(i*2)+2);
        polygon.push_back(tempSurf);
        tempSurf = Surface(PointXY(cView[i].X2(),cView[i].Y2()),PointXY(cView[i+1].X1(),cView[i+1].Y1()));//,(i*2)+3)
        polygon.push_back(tempSurf);
    }
    tempSurf = Surface(PointXY(cView[cView.size()-1].X1(),cView[cView.size()-1].Y1()),
                                                 PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()));//,(cView[cView.size()-1]*2)+2);
    polygon.push_back(tempSurf);//for last surface
    tempSurf = Surface(PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()),
                                                 PointXY(0,0));//,(cView[cView.size()-1]*2)+3);
    polygon.push_back(tempSurf);//for last surface to 0,0
    
   
    
    return polygon;
    
}

vector<Object> xformObjectsbyXY(vector<Object> inputview,double x, double y)
{
    vector<Object> result;
    double x1,y1,x2,y2;
    for(int i=0;i<inputview.size();i++){
        x1 = inputview[i].X1()+x;
        y1 = inputview[i].Y1()+y;
        x2 = inputview[i].X2()+x;
        y2 = inputview[i].Y2()+y;
        Object s(x1, y1, x2, y2, inputview[i].getID(), inputview[i].nearness(), inputview[i].getP1OS(), inputview[i].getP2OS(), inputview[i].getGID());
        s.setKP(inputview[i].getKP());
        result.push_back(s);
    }
    return result;
}


Point transformToCV(Point pos, Point near, double angle)
{
    Point  temp;
    double x, y;
    
    x=(pos.X()-near.X())*cos(angle) + (pos.Y()-near.Y())*sin(angle);
    y=(pos.Y()-near.Y())*cos(angle) - (pos.X()-near.X())*sin(angle);
    
    temp.set(x,y);
    return temp;
}


ClipperLib::Path viewConvertPath(vector<Object> view)
{
    ClipperLib::Path temp;
    temp << IntPoint(0,0);
    for(int i = 0; i < view.size(); i++)
    {
        temp << IntPoint(view[i].X1(), view[i].Y1()) 
             << IntPoint(view[i].X2(), view[i].Y2());
    }
    
    return temp;
}

vector<Object> PathsConvertView(ClipperLib::Paths solution)
{
    vector<Object> temp_rnt;
    Object temp_obj;
    
    for(int i = 0; i < solution.size(); i++)
    {
        for(int j = 0; j < solution[i].size(); j++)
        {
            if(j != solution[i].size() - 1)
                temp_obj.set(solution[i][j].X, solution[i][j].Y
                            ,solution[i][j+1].X, solution[i][j+1].Y, i);
            else
                temp_obj.set(solution[i][j].X, solution[i][j].Y
                            ,solution[0][0].X, solution[0][0].Y, i);
            
            temp_rnt.push_back(temp_obj);
        }
    }
    
    return temp_rnt;
}

vector<Object> fillGapOfView(vector<Object> view)
{
    vector<Object> temp;
    vector<Object> rnt;
    Object temp_Obj;
    
    for(int i = 0; i < view.size()-1; i++)
    {
        temp_Obj.set(view[i].X2(), view[i].Y2(), view[i+1].X1(), view[i+1].Y1(), i);
        temp.push_back(temp_Obj);
    }
    
    //break into dash lines
    rnt = breakTheLinesInto(temp);
    
    //combine together with origin view
    rnt = addTwoVectorsOfObjects(rnt, view);
    
    return rnt;
}

Point symmetric_point(Point p1, Point l1, Point l2)// 
{
    Point ret;
    double eps = 1e-4;
    double x = 0;
    double y = 0;
    
    if (l1.X() > l2.X() - eps && l1.X() < l2.X() + eps)//
    {
        x = (2 * l1.X() - p1.X());
        y = p1.Y();
        
        ret.set(x, y);
    }
    else
    {
        double k = (l1.Y() - l2.Y()) / (l1.X() - l2.X());
        if(k + eps > 0 && k - eps < 0)//
        {
            x = p1.X();
            y = l1.Y() - (p1.Y() - l1.Y());
        }
        else
        {
            x = (2*k*k*l1.X() + 2*k*p1.Y() - 2*k*l1.Y() - k*k*p1.X() + p1.X()) / (1 + k*k);
            y = p1.Y() - (ret.X() - p1.X()) / k;
        }
        
        ret.set(x, y);
    }
    return ret;
}

Point outSidePerpendPoint(Object line, double offset_length, Point crossPoint, int positive_flag)
{
        double offset = 400; //x axis offset 
    double k;
    double a, b, c;
    double x, y;
    
    Point temp;
    

        k = -(1 / ((line.Y2() - line.Y1())/(line.X2() - line.X1())));
        a = crossPoint.X();
        b = crossPoint.Y();
        
        c = b - a * k;
        
        if(positive_flag == 1)
        x = a + offset;
        if(positive_flag == 2)
        x = a - offset;
        y = k * x + c;

    temp.set(x, y);
    return temp;

}

//flag 1--negative  2--positive
Point outSidePerpendPointWithLength(Object line, double offset_length, Point crossPoint, int positive_flag)
{
        double offset = 400; //x axis offset 
    double k;
    double a, b, c, l;
    double x, y, x1, y1;
    
    Point temp;
    
    l = offset_length;
    x = crossPoint.X();
    y = crossPoint.Y();
    

    if((line.X1() != line.X2()) && (line.Y1() != line.Y2()))
    {
        k = -(1 / ((line.Y2() - line.Y1())/(line.X2() - line.X1())));
        a = k;
        b = y - a * x;
        
        //y1 = l / sqrt(a * a + 1) + y;
        //x1 = (y1 - b) / a;
        
        if(positive_flag == 2) //positive 
            y1 = l / sqrt(a * a + 1) + y;
        if(positive_flag == 1) //negative
            y1 =  y - l / sqrt(a * a + 1);

        
        x1 = (y1 - b) / a;
    }
    else
    {
        if(line.X1() == line.X2())
        {
            y1 = 0;
            if(positive_flag == 1) //negative
                x1 = -offset_length;
            if(positive_flag == 2) //positive
                x1 =  offset_length;
        }
        
        if(line.Y1() == line.Y2())
        {
                   x1 = 0;
            if(positive_flag == 1) //positive
                y1 = offset_length;
            if(positive_flag == 2) //negative
                y1 =  -offset_length;
        }
    }

    temp.set(x1, y1);
    return temp;

}

//1 - left ; 2 - right
Point outSidePerpendPointWithLeftRight(Object line, double offset_length, Point crossPoint, int leftOrright)
{
        double offset = 400; //x axis offset 
    double k;
    double a, b, c, l;
    double x, y, x1, y1;
    
    Point temp;
    
    l = offset_length;
    x = crossPoint.X();
    y = crossPoint.Y();
    

    if((line.X1() != line.X2()) && (line.Y1() != line.Y2()))
    {
        k = -(1 / ((line.Y2() - line.Y1())/(line.X2() - line.X1())));
        a = k;
        b = y - a * x;
        
        //y1 = l / sqrt(a * a + 1) + y;
        //x1 = (y1 - b) / a;
        
        
        y1 = l / sqrt(a * a + 1) + y;
        x1 = (y1 - b) / a;
        
        temp.set(x1, y1);
        
        //point is not one the expected side
        if(((isLeft(line.getP1(), line.getP2(), temp) > 0) && (leftOrright == 2))
            || ((isLeft(line.getP1(), line.getP2(), temp) < 0) && (leftOrright == 1)))
        {
            y1 =  y - l / sqrt(a * a + 1);
            x1 = (y1 - b) / a;
        }
         
        
    }
    else
    {
        if(line.X1() == line.X2())
        {
            y1 = 0;
            if(leftOrright == 1) //negative
                x1 = -offset_length;
            if(leftOrright == 2) //positive
                x1 =  offset_length;
        }
        
        if(line.Y1() == line.Y2())
        {
                   x1 = 0;
            if(leftOrright == 1) //positive
                y1 = offset_length;
            if(leftOrright == 2) //negative
                y1 =  -offset_length;
        }
    }

    temp.set(x1, y1);
    return temp;

}


double P_To_ShortestDistance(Point P, Object line) 
{

    double x, y;
    double x1,y1, x2, y2;
    
    x = P.X();
    y = P.Y();
    
    x1 = line.X1();
    x2 = line.X2();
    y1 = line.Y1();
    y2 = line.Y2();
    
    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = -1;
    
    if (len_sq != 0) //in case of 0 length line
        param = dot / len_sq;

    double xx, yy;

    if (param < 0) 
    {
      xx = x1;
      yy = y1;
    }
    else 
    {
        if (param > 1) 
        {
          xx = x2;
          yy = y2;
        }
        else 
        {
          xx = x1 + param * C;
          yy = y1 + param * D;
        }
    }

    double dx = x - xx;
    double dy = y - yy;
    return sqrt(dx * dx + dy * dy);
}

double FindDistanceToSegment(Point p, Object line_segment)
{
    double shortest;
    shortest = FindDistanceToSegment(line_segment.X1(), line_segment.Y1(), line_segment.X2(), line_segment.Y2(), p.X(), p.Y());
    return shortest;
}

double FindDistanceToSegment(double x1, double y1, double x2, double y2, double pointX, double pointY)
{
    double diffX = x2 - x1;
    float diffY = y2 - y1;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = pointX - x1;
        diffY = pointY - y1;
        return sqrt(diffX * diffX + diffY * diffY);
    }

    float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

    if (t < 0)
    {
        //point is nearest to the first point i.e x1 and y1
        diffX = pointX - x1;
        diffY = pointY - y1;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e x2 and y2
        diffX = pointX - x2;
        diffY = pointY - y2;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = pointX - (x1 + t * diffX);
        diffY = pointY - (y1 + t * diffY);
    }

    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}



/* expend an object following it
 * with particular length and return 
 * the expended part rather then the
 * complete object expended
 */
Object expend_Object(Object s1, double exp_length, int flag)
{
    double extd_x, extd_y;
    double ext_x1, ext_y1, ext_x2, ext_y2;
    Object exp_obj;
    if(flag == 1)
    {
        //extend the path line
        extd_x = s1.X1() + (s1.X1() - s1.X2()) / s1.length() * exp_length;
        extd_y = s1.Y1() + (s1.Y1() - s1.Y2()) / s1.length() * exp_length;
        exp_obj.set(extd_x, extd_y, s1.X1(), s1.Y1(), 1);
    }
    
    if(flag == 2)
    {
        //extend the path line
        extd_x = s1.X2() + (s1.X2() - s1.X1()) / s1.length() * exp_length;
        extd_y = s1.Y2() + (s1.Y2() - s1.Y1()) / s1.length() * exp_length;
        exp_obj.set(s1.X2(), s1.Y2(), extd_x, extd_y, 1);
    }
    
    /*if(flag == 3)
    {
        //extend the path line
        ext_x1 = s1.X1() + (s1.X1() - s1.X2()) / s1.length() * exp_length;
        ext_y1 = s1.Y1() + (s1.Y1() - s1.Y2()) / s1.length() * exp_length;
        
        
        //extend the path line
        ext_x2 = s1.X2() + (s1.X2() - s1.X1()) / s1.length() * exp_length;
        ext_y2 = s1.Y2() + (s1.Y2() - s1.Y1()) / s1.length() * exp_length;
        exp_obj.set(ext_x1, ext_y1, ext_x2, ext_y2, 1);
    }*/
    
    return exp_obj;
}

Object expend_Object_two_sides(Object s1, double dist1, double dist2)
{

    double ext_x1, ext_y1, ext_x2, ext_y2;
    Object exp_obj;

        //extend the path line
        ext_x1 = s1.X1() + (s1.X1() - s1.X2()) / s1.length() * dist1;
        ext_y1 = s1.Y1() + (s1.Y1() - s1.Y2()) / s1.length() * dist1;
        
        //extend the path line
        ext_x2 = s1.X2() + (s1.X2() - s1.X1()) / s1.length() * dist2;
        ext_y2 = s1.Y2() + (s1.Y2() - s1.Y1()) / s1.length() * dist2;
        exp_obj.set(ext_x1, ext_y1, ext_x2, ext_y2, 1);

    
    return exp_obj;
}

vector<Object> rotate_on_point(vector<Object> view, Point p, double angle)
{
    vector<Object>  temp;
    temp = TransformforToGlobalCoordinate(view, Point (0-p.X(), 0-p.Y()),view, 0);
    temp = TransformforToGlobalCoordinate(temp, Point (0,0),view, angle);
    temp = TransformforToGlobalCoordinate(temp, Point (p.X(), p.Y()),view, 0);
    
    return temp;
}

Object TransformforToGlobalCoordinate(Object Obj, Point coord_to_shift, double angle)
{

        double x1, y1, x2, y2;
        angle = ((angle / 180) * PI);

         x1 = Obj.X1() * cos(angle) - Obj.Y1() * sin(angle) + coord_to_shift.X();
         y1 = Obj.X1() * sin(angle) + Obj.Y1() * cos(angle) + coord_to_shift.Y();

         x2 = Obj.X2() * cos(angle) - Obj.Y2() * sin(angle) + coord_to_shift.X();
         y2 = Obj.X2() * sin(angle) + Obj.Y2() * cos(angle) + coord_to_shift.Y();

        Object s(x1, y1, x2, y2, Obj.getID());


        return s;
}

vector<Object> TransformforToGlobalCoordinate(vector<Object> addView, Point coord, vector<Object> currentPos, double angle)
{
           MyRobot robot(0, 0);
           vector<Object> temp;
           //combine a new local map into global map                           

           double x1, y1, x2, y2;
           angle = ((angle / 180) * PI);

           for (int i = 0; i<int(addView.size()); i++)
           {
                      x1 = addView[i].X1() * cos(angle) - addView[i].Y1() * sin(angle) + coord.X();
                      y1 = addView[i].X1() * sin(angle) + addView[i].Y1() * cos(angle) + coord.Y();

                      x2 = addView[i].X2() * cos(angle) - addView[i].Y2() * sin(angle) + coord.X();
                      y2 = addView[i].X2() * sin(angle) + addView[i].Y2() * cos(angle) + coord.Y();

                      Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());
                      s.setKP(1);
                      s.setASRNo(addView[i].getASRNo());
                      s.setChunkFLag(addView[i].getChunkFlag());
                      s.setColorID(addView[i].getColorID());
                      //s.setGID(addView[i].getGID());
                      //s.setID(addView[i].getID());
                      s.setKP(addView[i].getKP());
                      s.setLimitingPoint(addView[i].getLimitingPoint());
                      s.setLocalEnvID(addView[i].getLocalEnvID());
                      //s.setLocalEnvID();
                      //s.setNS();
                      s.setOoPV(addView[i].getOoPV());
                      s.setOrt(addView[i].getOrt());
                      s.setP1OS(addView[i].getP1OS());
                      s.setP2OS(addView[i].getP2OS());
                      s.setPEP1(addView[i].getPEP1());
                      s.setPEP2(addView[i].getPEP2());
                      s.setPO(addView[i].getPO());
                      s.setPos(addView[i].getPos());
                      s.setVN(addView[i].getVN());
                      s.set_imagined_flag(addView[i].get_imagined_flag());
                      
                      temp.push_back(s);
           }

           //currentTransLM = temp;
           //allLEs.push_back(temp);
           return temp;
}

Exit Transform_Exit_GlobalCoordinate(Exit exit, Point coord, double angle)
{
    Exit temp_exit;
    vector<Exit> rnt;
    
    Object temp_obj;
    vector<Object> convert, transfered;
    vector<Object> currentPos;
    

    temp_obj.set(exit.X1(), exit.Y1(), exit.X2(), exit.Y2(), 0);
    convert.push_back(temp_obj);
    transfered = TransformforToGlobalCoordinate(convert, coord, currentPos, angle);
    
    temp_exit.set(transfered[0].X1(), transfered[0].Y1(), transfered[0].X2(), transfered[0].Y2());
    return temp_exit;
  
}

vector<Exit> Transform_Exit_GlobalCoordinate(vector<Exit> exits, Point coord, vector<Object> currentPos, double angle)
{
    Exit temp_exit;
    vector<Exit> rnt;
    
    Object temp_obj;
    vector<Object> convert, transfered;
    
    for(int i = 0; i < exits.size(); i++)
    {
        temp_obj.set(exits[i].X1(), exits[i].Y1(), exits[i].X2(), exits[i].Y2(), i);
        convert.push_back(temp_obj);
    }
    
    transfered = TransformforToGlobalCoordinate(convert, coord, currentPos, angle);
    
    for(int i = 0; i < transfered.size(); i++)
    {
        temp_exit.set(transfered[i].X1(), transfered[i].Y1(), transfered[i].X2(), transfered[i].Y2());
        rnt.push_back(temp_exit);
    }
    
    return rnt;
}

/* compute a trendline of a list of points
 * formulas : line y = kx + b
 * slope : s = (E(x-x')*(y-y'))/(E(x-x')^2)
 */
//a = n * E(x*y)
//b = Ex + Ey
//c = n + Ex^2
//d = (Ex)^2
//slope = (a - b)/(c - d)
//e = Ey
//f = slope * Ex
//y-intercept = (e - f) / n
pair<double, double> Trendline_parameter(vector<Object> info)
{
    double a, b, c, d, e, f;
    double num, Ex, Ey, Exy, Ex2;
    double slope, intercept;
    pair<double, double> rnt;
    
    vector<Point> convert_points;
    
    Ex = Ey = Exy = Ex2 = 0; //initial
    
    convert_points = ObjectToPoints(info);
    num = convert_points.size();
    
    for(int i = 0; i < convert_points.size(); i++)
    {
        Ex += convert_points[i].X();
        Ey += convert_points[i].Y();
        
        Exy += convert_points[i].X() * convert_points[i].Y();
        Ex2 += convert_points[i].X() * convert_points[i].X();
    }
   
    a = num * Exy;
    b = Ex + Ey;
    c = num + Ex2;
    d = Ex * Ex;
    slope = (a - b)/(c - d);
    e = Ey;
    f = slope * Ex;
    intercept = (e - f) / num;
            
    rnt.first = slope;
    rnt.second = intercept;
    
    return rnt;
}

/* included angle between two vectors
 * sina = y1 / (sqrt(x1^2 + y1^2)) ==> a = arcsin[y1 / (sqrt(x1^2 + y1^2))]
 * sinb = (x2 - x1) / (sqrt((x2 - x1)^2 + (y2 - y1)^2)) ==> b = arcsin[(x2 - x1) / (sqrt((x2 - x1)^2 + (y2 - y1)^2))]
 * c = 90 - a
 * included_angle = b + c
 * 
 * 
 * cosin law c^2 = a^2 + b^2 - 2abcosC
 */
double includedAngle(Object s1, Object s2)
{
    double a, b, c, include_angle;
    
    /*
    a = asin((s1.Y2() - s1.Y1()) / (sqrt(pow(s1.X2() - s1.X1(), 2) + pow(s1.Y2() - s1.Y1(), 2))));
    b = asin((s2.X2() - s2.X1()) / (sqrt(pow(s2.X2() - s2.X1(), 2) + pow(s2.Y2() - s2.Y1(), 2))));
    
    c = 90 - a;
    include_angle = b + c;
       
    
    a = s2.length();
    b = s1.length();
    c = distanceOftwoP(s2.getP2(), s1.getP1());
    
    include_angle = acos((pow(a,2) + pow(b,2) - pow(c,2)) / 2*a*b);
    */
    
    float dx21 =  s1.X1() - s1.X2(); //x2-x1; 
    float dx31 =  s2.X2() - s2.X1(); //x3-x1;
    float dy21 =  s1.Y1() - s1.Y2(); //y2-y1;
    float dy31 =  s2.Y2() - s2.Y1(); //y3-y1;
    float m12 = sqrt( dx21*dx21 + dy21*dy21 );
    float m13 = sqrt( dx31*dx31 + dy31*dy31 );
    float theta = acos( (dx21*dx31 + dy21*dy31) / (m12 * m13) );
    
    include_angle = rad2deg(theta);
    
    return include_angle;       
}

double includedAngle(Point p11, Point p12, Point p21, Point p22)
{
    double a, b, c, include_angle;
    
    float dx21 =  p11.X() - p12.X(); //x2-x1; 
    float dx31 =  p21.X() - p22.X(); //x3-x1;
    float dy21 =  p11.Y() - p12.Y(); //y2-y1;
    float dy31 =  p21.Y() - p22.Y(); //y3-y1;
    float m12 = sqrt( dx21*dx21 + dy21*dy21 );
    float m13 = sqrt( dx31*dx31 + dy31*dy31 );
    float theta = acos( (dx21*dx31 + dy21*dy31) / (m12 * m13) );
    
    include_angle = rad2deg(theta);
    
    return include_angle;       
}


Point intersectPointTwolineEquations(Object line1, Object line2)
{   
    double m1, m2, c1, c2;
    double intersect_x, intersect_y;
    Point p3;
        
    
        m1 = (line1.Y2() - line1.Y1()) / (line1.X2() - line1.X1());
        m2 = (line2.Y2() - line2.Y1()) / (line2.X2() - line2.X1());

        c1 = line1.Y1() - m1 * line1.X1();
        c2 = line2.Y1() - m2 * line2.X1();

        intersect_x = (c1 - c2) / (m2 - m1);
        intersect_y = (m1 * c2 - c1 * m2) / (m1 - m2);
        p3.set(intersect_x, intersect_y);
        
        return p3;
}

bool parallel_objects(vector<Object> view)
{
    double k1, k2;
    //size must be larger than 3
    for(int i = 0; i < view.size() - 1; i++)
    {
        if(view[i].length() >= 800)
        {
            k1 = (view[i].Y2()-view[i].Y1()) / (view[i].X2()-view[i].X1());
            for(int j = i + 1; j < view.size(); j++)
            {
                k2 = (view[j].Y2()-view[j].Y1()) / (view[j].X2()-view[j].X1());
                if((view[j].length() >= 800)
                    && (abs(k1-k2) <= 0.3))
                {
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool parallel_objects(Object obj1, Object obj2)
{
    /*double k1, k2;
    //size must be larger than 3

    k1 = (obj1.Y2()-obj1.Y1()) / (obj1.X2()-obj1.X1());

    k2 = (obj2.Y2()-obj2.Y1()) / (obj2.X2()-obj2.X1());
    if(abs(k1-k2) <= 0.3)
        return true;
    else
        return false;*/
    double angle = 0;
    angle = includedAngle(obj1.getP1(), obj1.getP2(), obj2.getP1(), obj2.getP2());
    
    if((angle < 30) || (angle > 150))
        return true;
    else
        return false;
}   

//single direction shift 
void shift_objects(vector<Object>& objects, double shift_dist, int flag)
{
    switch(flag)
    {
        case 1: for(int i = 0; i < objects.size(); i++) //left
                {
                    objects[i].set(objects[i].X1()-shift_dist, objects[i].Y1(), objects[i].X2()-shift_dist, objects[i].Y2(), i);
                }
                break;
        case 2: for(int i = 0; i < objects.size(); i++) //right
                {
                    objects[i].set(objects[i].X1()+shift_dist, objects[i].Y1(), objects[i].X2()+shift_dist, objects[i].Y2(), i);
                }
                break;
        case 3: for(int i = 0; i < objects.size(); i++) //up
                {
                    objects[i].set(objects[i].X1(), objects[i].Y1()+shift_dist, objects[i].X2(), objects[i].Y2()+shift_dist, i);
                }
                break;
        case 4: for(int i = 0; i < objects.size(); i++) //down
                {
                    objects[i].set(objects[i].X1(), objects[i].Y1()-shift_dist, objects[i].X2(), objects[i].Y2()-shift_dist, i);
                }
                break;
    }
}



double slope(vector<Object> surfaces)
{
    
    vector<double> x, y;
    
    for(int i = 0; i < surfaces.size(); i++)
    {
        x.push_back(surfaces[i].X1());
        x.push_back(surfaces[i].X2());
        y.push_back(surfaces[i].Y1());
        y.push_back(surfaces[i].Y2());
    }
    
    //if(x.size() != y.size()){
    //    throw exception("...");
    //}
    double n = x.size();

    double avgX = accumulate(x.begin(), x.end(), 0.0) / n;
    double avgY = accumulate(y.begin(), y.end(), 0.0) / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for(int i=0; i<n; ++i){
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }

    //if(denominator == 0){
    //    throw exception("...");
    //}

    return numerator / denominator;
}

bool compareInterval(Point i1, Point i2) 
{ 
    return (i1.X() < i2.X()); 
} 
