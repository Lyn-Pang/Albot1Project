/*		 Object Class
			Object class consists of id, co-od of p1 & p2, nearness, Occluding status of p1 & p2.
			
			by Jhon
			modified by mhossain
*/


#include <iostream>
#include "Object.H"
#include "Point.H"
#include "GeometricOp.H"
#include "mfisOp.H"
#include "PointAndSurface.H"
#include "PathPlanning.H"

#include <cmath>

using namespace std;
#define PI 3.14159265

Object::Object(double X1, double Y1, double X2, double Y2, int id, int ns)
{
    x1 = X1; y1 = Y1;
    x2 = X2; y2 = Y2;
    ID = id;
    NS = ns;
	GID = 0;
    P1OS = 0; P2OS = 0;
	PEP1=false;	PEP2=false;
	Ort=0;
        po = false;
}

Object::Object(double X1, double Y1, double X2, double Y2, int id, int ns, int p1os, int p2os, int gid)
{
    x1 = X1; y1 = Y1;
    x2 = X2; y2 = Y2;
    ID = id;
    NS = ns;
	GID = gid;
    P1OS = p1os; P2OS = p2os;
}

Object::Object(double X1, double Y1, double X2, double Y2)
{
    x1 = X1; y1 = Y1;
    x2 = X2; y2 = Y2;
    ID = 0;
    NS = 0;
	GID = 0;
    P1OS = 0; P2OS = 0;
}

//ultimate function used in othre files 
Object::Object(double X1, double Y1, double X2, double Y2, int id)
{
    x1 = X1; y1 = Y1;
    x2 = X2; y2 = Y2;
    ID = id;
    
}

int Object::getID()
{
    return ID;
}

void Object::setID(int id)
{
  ID = id;
}

int Object::getGID()
{
    return GID;
}



void Object::setGID(int gid)
{
  GID = gid;
}

double Object::X1(){return x1;}
double Object::Y1(){return y1;}
double Object::X2(){return x2;}
double Object::Y2(){return y2;}

double Object::mpX() {
	return (x1+x2)/2;
}
double Object::mpY() {
	return (y1+y2)/2;
}

double Object::length()
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)); // 
}



//updates whole Object
void Object::set(double a1, double b1, double a2, double b2, int id)
{
    x1 = a1; y1 = b1;
    x2 = a2; y2 = b2;
    ID = id;
}

//replaces all
void Object::set(Object s)
{
    x1 = s.X1(); y1 = s.Y1();
    x2 = s.X2(); y2 = s.Y2();
    ID = s.getID();
	P1OS=s.getP1OS();
	P2OS=s.getP2OS();
}/*
void Object::convertExitToObject(Exit e) {
	x1=e.X1();	y1=e.Y1();
	x2=e.X2();	y2=e.Y2();
	ID=e.getID();
}*/

//updates point1
void Object::setP1(double a1, double b1)
{
	x1=a1; y1=b1;
}

//updates point2
void Object::setP2(double a1, double b1)
{
	x2=a1; y2=b1;
}

//returns p1's occluding status
int Object::getP1OS()
{
	return P1OS;
}

//returns p2's occluding status
int Object::getP2OS()
{
	return P2OS;
}

//set p1's occluding status
void Object::setP1OS(int a)
{
	P1OS=a;
}

//set p1's occluding status
void Object::setP2OS(int a)
{
	P2OS=a;
}

//replaces Object by s
void Object::replace(Object s)
{
	x1 = s.X1(); y1 = s.Y1();
    	x2 = s.X2(); y2 = s.Y2();
}

//returns distance from first point of line to another point
double Object::distP1ToPoint(double a, double b)
{

	return sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));

}

//returns distance from second point of line to another point
double Object::distP2ToPoint(double a, double b)
{

	return sqrt((x2-a)*(x2-a)+(y2-b)*(y2-b));

}

double Object::distMPToPoint(double a, double b)
{
	double mpx=(x1+x2)/2;
	double mpy=(y1+y2)/2;
	return sqrt((mpx-a)*(mpx-a)+(mpy-b)*(mpy-b));

}

//returns distance between point1 of two Objects
double Object::distP1ToP1(Object s)
{
	return sqrt((x1-s.X1())*(x1-s.X1())+(y1-s.Y1())*(y1-s.Y1()));
}

//returns distance between point 2 of two Objects
double Object::distP2ToP2(Object s)
{
	return sqrt((x2-s.X2())*(x2-s.X2())+(y2-s.Y2())*(y2-s.Y2()));
}

double Object::distP1ToP2(Object s)
{
	return sqrt((x1-s.X2())*(x1-s.X2())+(y1-s.Y2())*(y1-s.Y2()));
}

//returns distance from p2 to p1 of other Object
double Object::distP2ToP1(Object s)
{
	return sqrt((x2-s.X1())*(x2-s.X1())+(y2-s.Y1())*(y2-s.Y1()));
}

//display x and y values with id on console
void Object::display()
{
	cout<<"id: "<<ID<<" X1: "<<x1<<" Y1: "<<y1<<" ------ X2: "<<x2<<" Y2: "<<y2<<endl;
                cout<<" P1OS: "<<P1OS<<" P2OS: "<<P2OS<<" p1pe: "<<PEP1<<" p2pe: "<<PEP2<<
                        " KP- "<<kp<<" position: "<<pos<<" OoPV: "<<OoPV<<" VN: "<<vn<<" Ort Angle: "
                        <<Ort<<" dist: "<<distance<<" ASRN: "<<asrno<<" LocalEnv: ";
                for(unsigned int i=0; i < localEnvID.size(); i++) 
                    cout<<localEnvID[i]<<" ";
                cout<<" PO- "<<po<<endl<<endl;//" p1pe: "<<PEP1<<" p2pe: "<<PEP2<<" GID-"<<GID<<"  NS: "<<NS<<
}

Point Object::midpoint()
{
	
	double x=(x1+x2)/2;
	double y=(y1+y2)/2;
	Point mid(x,y);
	return mid;

}

int Object::nearness()
{
	
	return NS;
}

void Object::setNS(int a)
{
	NS=a;
}

/********* Probable Exit ********/
void Object::setPEP1(bool p1pe)
{
	PEP1=p1pe;
}

void Object::setPEP2(bool p2pe)
{
	PEP2=p2pe;
}

bool Object::getPEP1()
{
	return PEP1;
}
bool Object::getPEP2()
{
	return PEP2;
}

void Object::setOrt(double ang)
{
	Ort=ang;
}

double Object::getOrt()
{
	return Ort;
}

void Object::setDistance(double dist) {
    distance = dist;
}
    double Object::getDistance() {
        return distance;
    }

void Object::setKP(int a) {
	kp=a;
}

int Object::getKP() {
	return kp;
}


void Object::setPO(bool a) {
	po=a;
}

bool Object::getPO() {
	return po;
}

void Object::setPos(int a) {
	pos=a;
}
int Object::getPos() {
	return pos;
}

void Object::setOoPV(bool a) {
	OoPV=a;
}

bool Object::getOoPV() {
	return OoPV;
}

void Object::setVN(int a) {
	vn=a;
}

int Object::getVN() {
	return vn;
}

void Object::setASRNo(int a) {
	asrno=a;
}
int Object::getASRNo() {
	return asrno;
}

void Object::setLimitingPoint(int a) {
    limitingPoint = a;
}

int Object::getLimitingPoint() {
    return limitingPoint;
}

void Object::setLocalEnvID(int a) {
    localEnvID.push_back(a);
}
void Object::setLocalEnvID(vector<int> a) {
    localEnvID = a;
}
    vector<int> Object::getLocalEnvID() {
        return localEnvID;
    }
    
    void Object::setColorID(int a) {
        colorID = a;
    }
    int Object::getColorID() {
        return colorID;
    }
    
    
void Object::setDottedBack(bool flag)
{
    dotted_back = flag;
}
bool Object::getDottedBack()
{
    return dotted_back;
}

//few Geometric functions

/*it returns location of perpendicular projection point of given point.

 	  r<0      P is on the backward extension of AB
          r>1      P is on the forward extension of AB
          0<r<1    P is interior to AB
	http://www.exaflop.org/docs/cgafaq/cga1.html#Subject%201.02:%20How%20do%20I%20find%20the%20distance%20from%20a%20point%20to%20a%20line?

*/
 
    double Object::midToOrigin()
    {
        double rtn;
        double mid_x;
        double mid_y;
        
        mid_x = (x1 + x2) / 2;
        mid_y = (y1 + y2) / 2;
        rtn = sqrt(mid_x * mid_x + mid_y * mid_y);
        
        return rtn;
    }
    
double Object::ppLocOfPoint(double x3, double y3)
{
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	if(r<0)
	return -1;
	else if(r>1)
	return 1;
	else
	return 0;
}
//it returns coordinates of PP of given point
Point Object::ppCordOfPoint(double x3, double y3)
{
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	double x=x1+(r*x21);
	double y=y1+(r*y21);
	Point p(x,y);
	return p;
}

//it returns perpendicular distance of given point and line
double Object::perpendicularDistOfPoint(double x3, double y3)
{
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	double x=x1+(r*x21);
	double y=y1+(r*y21);
	//if(r>=0 && r <=1)
	return sqrt((x-x3)*(x-x3)+(y-y3)*(y-y3));
	
}

double Object::distP1ToPPOfPoint(double x3, double y3)
{
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	double x=x1+(r*x21);
	double y=y1+(r*y21);

	return sqrt((x1-x)*(x1-x)+(y1-y)*(y1-y));
}

double Object::isP1ToBeExtended(Object s1)
{
	double x3=s1.X1();
	double y3=s1.Y1();
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	
	if(r<0)
	{
		double x=x1+(r*x21);
		double y=y1+(r*y21);

		double d=sqrt((x1-x)*(x1-x)+(y1-y)*(y1-y));
		if(d>200)
		return d;
		else 
		return 0;
	}
	else 
	return 0;
	
}

double Object::isP2ToBeExtended(Object s1)
{
	double x3=s1.X2();
	double y3=s1.Y2();
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	
	if(r>1)
	{
		double x=x1+(r*x21);
		double y=y1+(r*y21);
	
		double d=sqrt((x2-x)*(x2-x)+(y2-y)*(y2-y));
		if(d>200)
		return d;
		else 
		return 0;
	}
	else 
	return 0;
}

double Object::shortestDistP1ToP1(Object s1)
{
	double x3=s1.X1();
	double y3=s1.Y1();
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	if(r<0)
	return ((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
	else if(r>1)
	return ((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
	else
	{
		double x=x1+(r*x21);
		double y=y1+(r*y21);
		return ((x-x1)*(x-x1)+(y-y1)*(y-y1));
	}

}

//new one(2/06/11): it returns the shortest distant point along the object
//from the given point
Point Object::shortestDistPointFrom(double a, double b)
{
	double x3=a;
	double y3=b;
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;

	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

	double r=((x31*x21)+(y31*y21))/(length*length);
//	cout<<endl<<"r "<<r<<endl;
	if(r<0) {
			Point p(x1,y1);
			return p;
		}
		else if(r>1) {
			Point p(x2,y2);
				return p;
		}
		else
		{
			double x=x1+(r*x21);
			double y=y1+(r*y21);
			Point p(x,y);
				return p;
		}
}

//new one(2/06/11): it returns the shortest distant from the given point to this object
//from the given point
//shortest point could be p1 or p2 or any point on this object(not outside of this object)
double Object::shortestDistFrom(double a, double b)
{
	double x3=a;
	double y3=b;
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;

	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

	double r=((x31*x21)+(y31*y21))/(length*length);
	//cout<<endl<<"r "<<r<<endl;
	if(r<0) {
			//Point p(x1,y1);
			//return p;
			return getDistBtw2Points(a,b,x1,y1);
		}
		else if(r>1) {
			//Point p(x2,y2);
				//return p;
			return getDistBtw2Points(a,b,x2,y2);
		}
		else
		{
			double x=x1+(r*x21);
			double y=y1+(r*y21);
			//Point p(x,y);
				//return p;
			return getDistBtw2Points(a,b,x,y);
		}
}


double Object::getAngleWithPoint(double a, double b)
{
	double x21=x2-x1;
	double y21=y2-y1;
	double x31=a-x1;
	double y31=b-y1;
	
	double la=length();
	double lb=getDistBtw2Points(x1, y1, a, b);

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

double Object::getAngleWithPointFP1E(double a, double b)
{
	double x21=x1-x2;
	double y21=y1-y2;
	double x31=a-x2;
	double y31=b-y2;
	
	double la=length();
	double lb=getDistBtw2Points(x2, y2, a, b);

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

double Object::getAngleWithXaxis()
{
	double x21=x2-x1;
	double y21=y2-y1;
	
	double angle1=acos(x21/length());

	if(y21 < 0)
	angle1 = 2*PI - angle1;	

	return ((180/PI)*angle1);
}


double Object::getAngleWithLine(Object s)
{
    
	double angdiff=s.getAngleWithXaxis()-getAngleWithXaxis();
	return angdiff;
}

double Object::getAngleWithLine_Acute(Object s)
{
    
	double angdiff=s.getAngleWithXaxis()-getAngleWithXaxis();
        
        if(angdiff < 0)
            angdiff += 180;
        if(angdiff > 180)
            angdiff -= 180;
        
	return angdiff;
}


double Object::getAngleWithPointForASR(double a, double b)
{
	double x21=x2-x1;
	double y21=y2-y1;
	double x31=a-x1;
	double y31=b-y1;
	
	double la=length();
	double lb=getDistBtw2Points(x1, y1, a, b);

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
	
	double diff=angle1-angle2;

	/*if(diff>0)
	return diff;
	else */
	return diff;
}


//check for line matching using LoS
bool Object::isP1MatchedWith(Object s)
{
	double a=0; //x1
	double b=0; //y1
	double c=x1; //x2
	double d=y1; //y2
	
	double x3= s.X1();
	double y3= s.Y1();
	double x4= s.X2();
	double y4= s.Y2();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/

	bool result;
	//cout<<" haha "<<" ub "<<ub<<" id "<<s.getID()<<endl;
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		double dist=s.distP2ToPoint(x, y);//dist btw p2 of new line and intersection 
		//cout<<"dist "<<dist<<endl;
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<" ms- "<<s.getMS()<<endl;
		if(dist > 200)
		{
			//double mppdist=s.perpendicularDistOfPoint((x1+x2)/2,(y1+y2)/2);
			//double thd=sqrt((c-x)*(c-x)+(d-y)*(d-y));//dist threshold to check mismatch
			//cout<<" pdist: "<<pdist<<" thd "<<thd<<endl;
			//double p1pdist=s.perpendicularDistOfPoint(x1,y1);
			//if(mppdist < 200 || thd < 200)
			result = true;
			//else
			//result = false;
		}
		else 	
		result=false;
		
	}
	else 
	result= false;

	return result;
}

//to delete tmp lines............12.01.11
bool Object::isP1MatchedWith(Object s, Object rpc)
{
	double a=rpc.X1(); //x1
	double b=rpc.Y1(); //y1
	double c=x1; //x2
	double d=y1; //y2
	
	double x3= s.X1();
	double y3= s.Y1();
	double x4= s.X2();
	double y4= s.Y2();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/

	bool result;
	//cout<<" haha "<<" ub "<<ub<<" id "<<s.getID()<<endl;
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		double dist=s.distP2ToPoint(x, y);//dist btw p2 of new line and intersection 
		//cout<<"dist "<<dist<<endl;
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<" ms- "<<s.getMS()<<endl;
		if(dist > 200)
		{
			//double mppdist=s.perpendicularDistOfPoint((x1+x2)/2,(y1+y2)/2);
			double thd=sqrt((c-x)*(c-x)+(d-y)*(d-y));//dist threshold to check mismatch
			//cout<<" pdist: "<<pdist<<" thd "<<thd<<endl;
			//double p1pdist=s.perpendicularDistOfPoint(x1,y1);
			if(thd < 500)
			result = true;
			else
			result = false;
		}
		else 	
		result=false;
		
	}
	else 
	result= false;

	return result;
}

//to delete tmp lines............12.01.11
bool Object::isMidpointMatchedWith(Object s, Object rpc)
{
	double a=rpc.X1(); //x1
	double b=rpc.Y1(); //y1
	double c=(x1+x2)/2; //x2
	double d=(y1+y2)/2; //y2
	
	double x3= s.X1();
	double y3= s.Y1();
	double x4= s.X2();
	double y4= s.Y2();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/

	bool result;
	//cout<<" haha "<<" ub "<<ub<<" id "<<s.getID()<<endl;
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		double thd=sqrt((c-x)*(c-x)+(d-y)*(d-y));//dist threshold to check mismatch
		if(thd < 500)
			result = true;
		else 	
		result=false;
		/*double dist=s.distP2ToPoint(x, y);//dist btw p2 of new line and intersection 
		//cout<<"dist "<<dist<<endl;
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<" ms- "<<s.getMS()<<endl;
		if(dist > 200)
		{
			//double mppdist=s.perpendicularDistOfPoint((x1+x2)/2,(y1+y2)/2);
			//double thd=sqrt((c-x)*(c-x)+(d-y)*(d-y));//dist threshold to check mismatch
			//cout<<" pdist: "<<pdist<<" thd "<<thd<<endl;
			//double p1pdist=s.perpendicularDistOfPoint(x1,y1);
			//if(mppdist < 200 || thd < 200)
			result = true;
			//else
			//result = false;
		}
		else 	
		result=true;*/
		
	}
	else 
	result= false;

	return result;
}

bool Object::isP2MatchedWith(Object s)
{
	double a=0; //x1
	double b=0; //y1
	double c=x2; //x2
	double d=y2; //y2
	
	double x3= s.X1();
	double y3= s.Y1();
	double x4= s.X2();
	double y4= s.Y2();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/
	bool result;
	
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		double dist=s.distP1ToPoint(x, y);
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<endl;
		if(dist > 80 )
		result = true;
		else 	
		result=false;
	}
	else 
	result= false;

	return result;
}

bool Object::isMidpointMatchedWith(Object s)
{
	
	double a=0; //x1
	double b=0; //y1
	double c=(x1+x2)/2; //x2
	double d=(y1+y2)/2; //y2
	
	double x3= s.X1();
	double y3= s.Y1();
	double x4= s.X2();
	double y4= s.Y2();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/
	bool result;
	//cout<<" haha "<<" ub "<<ub<<" id "<<s.getID()<<endl;
	if((ub >= 0) && (ub < 1))
	{
		//double ua_num= x43*y13-y43*x13;
		//double ua=ua_num/u_deno;
		
		//double x=a+ua*x21;
		//double y=b+ua*y21;
		//double dist=s.distP1ToPoint(x, y);
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<endl;
		double pdist=s.perpendicularDistOfPoint((x1+x2)/2,(y1+y2)/2);
		//double thd=sqrt((c-x)*(c-x)+(d-y)*(d-y));//dist threshold to check mismatch
		//cout<<"pdist::: "<<pdist<<" thd::: "<<thd<<endl;
		if(pdist < 300)
		result = true;
		else
		result = false;
		
	}
	else 
	result= false;

	return result;
}

bool Object::isOverlappedWith(Object s, double rx, double ry)
{
	
	double a=rx; //x1
	double b=ry; //y1
	double c=(x1+x2)/2; //x2
	double d=(y1+y2)/2; //y2
	
	double x3= s.X1();
	double y3= s.Y1();
	double x4= s.X2();
	double y4= s.Y2();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/
	bool result;
	
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		//double dist=s.distP1ToPoint(x, y);
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<endl;
		double pdist=s.perpendicularDistOfPoint((x1+x2)/2,(y1+y2)/2);
		double thd=sqrt((c-x)*(c-x)+(d-y)*(d-y));//dist threshold to check mismatch
		cout<<"pdist::: "<<pdist<<" thd::: "<<thd<<endl;
		if(pdist < 200 || thd < 800)
		result = true;
		else
		result = false;
		
	}
	else 
	result= false;

	return result;
}

//for inside Object check
bool Object::isInsideObject(Object s, Object crp)
{
	
	double a=crp.X1();  /*x1*/	double x3= s.X1();
	double b=crp.Y1();  /*y1*/	double y3= s.Y1();
	double c=(x1+x2)/2; /*x2*/	double x4= s.X2();
	double d=(y1+y2)/2; /*y2*/	double y4= s.Y2();	

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	bool result;
	//cout<<" haha "<<" ub "<<ub<<" id "<<s.getID()<<endl;
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		Object tmp;
		tmp.set(a,b,c,d,1);
		double angle=abs(tmp.getAngleWithPoint(x,y));

		if(angle > 179 && angle < 181)
		result = false;
		else
		result = true;
		
	}
	else 
	result= false;

	return result;
}

//same as shortestDistanceBtwTwoObjects
double Object::shortestDistanceWithObject(Object old) {
    vector<double> dists;
    dists.push_back(shortestDistFrom(old.X1(),old.Y1()));
    dists.push_back(shortestDistFrom(old.X2(),old.Y2()));
    dists.push_back(old.shortestDistFrom(x1,y1));
    dists.push_back(old.shortestDistFrom(x2,y2));
    double shortestDistance;
    shortestDistance = dists[0];
    for(int i =1;i<int(dists.size());i++) {
        if(shortestDistance > dists[i])
            shortestDistance = dists[i];
    }
    return shortestDistance;
}

bool Object::isThisInsideCV(vector<double> boundaries) {
    if(x1 > boundaries[0] || x2 > boundaries[0])
        if(x1 < boundaries[1] || x2 < boundaries[1])
            if(y1 < boundaries[2] || y2 < boundaries[2])
                return true;
        return false;
}

bool Object::isThisOverlappingObject(vector<Object> currentView) {
    double angle,shortestDistance;
    Object thisObjectMPToOrigin, cViewObjectToOrigin,thisObject;
    for(int i=0;i<int(currentView.size());i++) {
        angle = getAngleWithLine(currentView[i]);
         if ((abs(angle) < 10 || abs(angle) > 350) || (abs(angle) > 170 && abs(angle) < 190)) {
             shortestDistance = shortestDistanceWithObject(currentView[i]);
             if(shortestDistance < 400)
                 return true;
             else if(shortestDistance < 1000) {
                 thisObject.set(x1,y1,x2,y2,1);
                 thisObjectMPToOrigin.set(mpX(),mpY(),0,0,1);
                 cViewObjectToOrigin.set(currentView[i].mpX(),currentView[i].mpY(),0,0,1);
                 if(checkForIntersection(thisObject, cViewObjectToOrigin) ==1 or checkForIntersection(thisObjectMPToOrigin, currentView[i])==1)
                     return true;
             }
         }
    }
    
    return false;
}


void Object::reverse() {
    double tx = x1;
    double ty = y1;
    x1 = x2;
    y1 = y2;
    x2 = tx;
    y2 = ty;
}



/////////////// for exit confirmation ///////////////////////
double Object::whereIsExit(Object s)
{
	
	double a=0; //x1
	double b=0; //y1
	double c=(x1+x2)/2; //x2
	double d=(y1+y2)/2; //y2
	
	double x3= s.X1();
	double y3= s.Y1();
	double x4= s.X2();
	double y4= s.Y2();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/
	double result;
	//cout<<" haha "<<" ub "<<ub<<" id "<<s.getID()<<endl;
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		//double dist=s.distP1ToPoint(x, y);
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<endl;
		//double pdist=s.perpendicularDistOfPoint((x1+x2)/2,(y1+y2)/2);
		//double thd=sqrt((c-x)*(c-x)+(d-y)*(d-y));//dist threshold to check mismatch
		//cout<<"pdist::: "<<pdist<<" thd::: "<<thd<<endl;
		double thd1=sqrt(x*x+y*y);//distance btw robpos and los point on exit
		double thd2=sqrt(c*c+d*d);//distance btw robpos and Object midpoint 
		if(thd2 > thd1)
		result = 0;
		else
		result = (thd1-thd2);
		
	}
	else 
	result= 1;

	return result;
}


void Object::setChunkFLag(int flag)
{
    chunk_flag = flag;
}
int Object::getChunkFlag()
{
    return chunk_flag;
}

void Object::setPositionFlag(int flag)
{
    position_flag = flag;
}

int Object::getPositionFlag()
{
    return position_flag;
}

void Object::setEntranceExitflag(int flag)
{
    entance_exit_flag = flag;
}
int Object::getEntanceExitflag()
{
    return entance_exit_flag;
}

void Object::set_region_remove_flag(int flag)
{
    region_remove_flag = flag;
}
int Object::get_region_remove_flag()
{
    return region_remove_flag;
}

void Object::set_replace_flag(int flag)
{
    replace_flag = flag;
}
int Object::get_replace_flag()
{
    return replace_flag;
}

void Object::set_match_label(int label_numb)
{
    match_label = label_numb;
}

int Object::get_match_label()
{
    return match_label;
}

void Object::set_match_flag(int match_flag)
{
    track_in_map = match_flag;
}
int Object::get_match_flag()
{
    return track_in_map;
}

void Object::set_imagined_flag(int flag)
{
    imagined_flag = flag;
}
int Object::get_imagined_flag()
{
    return imagined_flag;
}

double Object::get_slope()
{
    double temp;
    
    temp = (y2 - y1) / (x2 - x1);
    return temp;
}

Point Object::getP1()
{
    Point _p1;
    _p1.set(x1, y1);
    return _p1;
}
Point Object::getP2()
{
    Point _p2;
    _p2.set(x2, y2);
    return _p2;
}



bool MyDataSortPredicate(Object d1, Object d2)
{
  return d1.getOrt() > d2.getOrt();
}

bool sortA2L(Object d1, Object d2)
{
  return d1.length() > d2.length();
}
bool sortA2MPDistanceFromOrigin(Object d1, Object d2) {
    return d1.distMPToPoint(0,0) > d2.distMPToPoint(0,0);
}
bool sortA2OrtAngleL2R(Object d1, Object d2) {
    return d1.getOrt() > d2.getOrt();
}
bool sortA2OrtAngleR2L(Object d1, Object d2) {
    return d1.getOrt() < d2.getOrt();
}
bool sortA2Distance(Object d1, Object d2) {
    return d1.getDistance() < d2.getDistance();
}


vector<Object> setOccludingEdges(vector<Object> Objects)
{	double th=100;
	double dp2top1;
	for(int i=0;i<int(Objects.size()-1);i++)
	{
		dp2top1=Objects[i].distP2ToP1(Objects[i+1]);
		if(dp2top1>th)
		{	//check distance from p1 and p2 to robot position(0,0)
			if(Objects[i].distP2ToPoint(0,0)<Objects[i+1].distP1ToPoint(0,0))
			Objects[i].setP2OS(1);
			else
			Objects[i+1].setP1OS(1);
		}
		else
		{
			Objects[i].setP2OS(1);
			Objects[i+1].setP1OS(1);
		}
		
		if(dp2top1 > 600)// for probable exit point
		{
			Objects[i].setPEP2(true);
			Objects[i+1].setPEP1(true);
		}
	}
	return Objects;
}

//returns Object
Object getObject(vector<Object> Objects, int id)
{
	//Object result;
	for(int i=0;i<int(Objects.size());i++)
	{
		if(Objects[i].getID()==id)
			return Objects[i];
		
	}
	cout<<" asking Object not found"<<endl;
	
	Object ss(0,0,0,0);
	return ss;
}

Object getExtendedObject(Object ObjectInPV, Object ObjectInCV)
{
    double threshold = 30;
    double p1d, p2d;
    
    double px1 = ObjectInPV.X1();
    double py1 = ObjectInPV.Y1();
    double px2 = ObjectInPV.X2();
    double py2 = ObjectInPV.Y2();
    
    double cx1 = ObjectInCV.X1();
    double cy1 = ObjectInCV.Y1();
    double cx2 = ObjectInCV.X2();
    double cy2 = ObjectInCV.Y2();
    
    double x3,y3;
    double plength=sqrt((px2-px1)*(px2-px1)+(py2-py1)*(py2-py1));
    double clength=sqrt((cx2-cx1)*(cx2-cx1)+(cy2-cy1)*(cy2-cy1));
    
    p1d = ObjectInPV.distP1ToP1(ObjectInCV);
    p2d = ObjectInPV.distP2ToP2(ObjectInCV);
    if (p1d <= threshold)
    {
        y3 = plength*((cy2-cy1)/clength) + cy1;
        x3 = plength*((cx2-cx1)/clength) + cx1;
        Object o(x3,y3,cx2,cy2);
        return o;
    }
    else if(p2d <= threshold)
    {
        y3 = cy2 - plength*((cy2-cy1)/clength);
        x3 = cx2 - plength*((cx2-cx1)/clength);
        Object o(cx1,cy1,x3,y3);
        return o;
    }
    
    Object ss(0,0,0,0);
    return ss;
}

vector<Object> deleteObject(vector<Object> Objects, int dsur)
{
	for(vector<Object>::iterator it = Objects.begin(); it != Objects.end();)
    	{
        	if((*it).getID() == dsur)
       		{
            		it = Objects.erase(it);
       		}
       		else
       		{
          		++it;
       		}
  	}
	return Objects;
}

//it will delete small Objects
vector<Object> removeSmallObjects(vector<Object> mfis)
{
	vector<Object> Objects=mfis;
	for(vector<Object>::iterator it = Objects.begin(); it != Objects.end();) {
        	if((*it).length() < 500) {
            		it = Objects.erase(it);
       		}
       		else {
          		++it;
       		}
  	}
	return Objects;
}

/*
vector<Object> considerLongObjects(vector<Object> Objects, double per)
{
	Objects=sortAccording2Length(Objects);
	vector<Object> result;
	for(int i=0;i<int(Objects.size()*per);i++) {
		result.push_back(Objects[i]);
	}
	return result;
}*/

int getIndex(vector<Object> Objects, int id)
{
	int index=0;
	for(int i=0;i<int(Objects.size());i++)
	{
		if(Objects[i].getID()==id)
			index =i;	
		
	}
return index;
	//cout<<" asking Object not found"<<endl;
}

void displayObjects(vector<Object> Objects)
{
	for(int i=0;i<int(Objects.size());i++)
		Objects[i].display();
}

vector<Object> addTwoVectorsOfObjects(vector<Object> first, vector<Object> second) {
    for(int i =0;i<int(second.size());i++) {
        first.push_back(second[i]);
    }
    return first; 
}

vector<Object> addTwoVectorsOfObjectsNoduplicate(vector<Object> first, vector<Object> second) 
{
    vector<Object> temp, ref;
    /*
    for(int i =0;i<int(second.size());i++) 
    {
        for(int j = 0; j < first.size(); j++)
        {
            if((first[j].getP1() != second[i].getP1()) && (first[j].getP2() != second[i].getP2()))
                first.push_back(second[i]);
        }
    }
    */
    
    temp = addTwoVectorsOfObjects(first, second);
    
    for(int i = 0; i < temp.size(); i++)
    {
        //ref = temp[i];
        for(int j = 0 ; j < temp.size(); j++)
        {
            if((temp[j].getP1() == temp[i].getP1())
                && (temp[j].getP2() == temp[i].getP2())
                && (i != j))
            {
                temp.erase(temp.begin() + j);
                break;
            }
        }
    }
    
    return temp; 
}

vector<Object> deleOneVectorOfObject(vector<Object> first, vector<Object> second)
{
    vector<Object> temp;
    for(int i = 0; i < first.size(); i++)
    {
            if(isObject(second, Point (first[i].X1(), first[i].Y1()), Point (first[i].X2(), first[i].Y2())) == false)
                temp.push_back(first[i]);
    }
    
    return temp;
}

vector<Object> deletOneViewbaseLength(vector<Object> first, vector<Object> second)
{
    vector<Object> temp;
    temp = first;
    for(int i = 0; i < temp.size(); i++)
    {
        for(int j = 0; j < second.size(); j++)
        {
            if(temp[i].length() == second[j].length())
            {
                temp.erase(temp.begin()+i);
                i--;
                break;
            }
        }
    }
    
    return temp;
}

vector<Object> convertSurfaceToObject(vector<Surface> surfs) {
    vector<Object> objects;
    for(unsigned int i=0;i<surfs.size();i++) {
        objects.push_back(Object(surfs[i].getX1(),surfs[i].getY1(),surfs[i].getX2(),surfs[i].getY2()));
    }
    return objects;
}

vector<Surface> convertObjectToSurface(vector<Object> surfs) {
    vector<Surface> objects;
    for(unsigned int i=0;i<surfs.size();i++) {
        objects.push_back(Surface(PointXY(surfs[i].X1(),surfs[i].Y1()),PointXY(surfs[i].X2(),surfs[i].Y2())));
    }
    return objects;
}

Object convertSurfaceToObject(Surface surf) {
    Object object;
    
        object.set(surf.getX1(),surf.getY1(),surf.getX2(),surf.getY2(),1);
 
    return object;
}

Object getLargestObject(vector<Object> Objects)
{
    Object temp_refer;
    double temp_max = 0;
    for(int i = 0; i < Objects.size(); i++)
    {
        if(i == 0)
        {
            temp_max = Objects[i].length();
            temp_refer = Objects[i];
        }
        else
        {
            if(temp_max < Objects[i].length())
            {
                temp_max = Objects[i].length();
                temp_refer = Objects[i];
            }
        }
    }
    
    return temp_refer;
}


/****************************************myRobot members *****************************/

MyRobot::MyRobot(double a, double b) {
	double rsize=100;
	Object side1(-rsize,-rsize,-rsize,rsize,1);//(-,- to -,+)left side
	sides.push_back(side1);
	Object side2(-rsize,rsize,rsize,rsize,2);//(-,+ to ++)front side
	sides.push_back(side2);
	Object side3(rsize,-rsize,rsize,rsize,3);//(+- to ++)right side
	sides.push_back(side3);
	Object side4(-rsize,-rsize,rsize,-rsize,4);//(-- to +-)bottom side
	sides.push_back(side4);
	Object side5(rsize,rsize,rsize,-rsize,5);
	sides.push_back(side5);
	Object side6(rsize,-rsize,rsize,-rsize,6);
	sides.push_back(side6);
	Object side7(0,0,0,250,7);//+y axis at current robot position
	sides.push_back(side7);
	//Object side8(-350,0,-300,500,8);
	//sides.push_back(side8);
	//Object side9(350,0,300,500,9);
	//sides.push_back(side9);
	Object side8(0,0,rsize,0,10);//x axis at current robot position
	sides.push_back(side8);
                Object side9(0,0,-rsize,0,10);//(-ve)x axis at current robot position
	sides.push_back(side9);
	
	//allrobotpos.push_back(sides);
}
vector<Object> MyRobot::getRobot() {
	return sides;
}

vector<Object> MyRobot::robotPathForNextDest(double angle, double distance) {
    vector<Object> result;
    double rsize = 350;
    if(distance == 0) {
        distance = 350;//happens when it needs to take turn only to come back from deadend.
    }
    Object side;
    side.set(-rsize,-rsize,-rsize,distance+rsize,1);
    result.push_back(side);//(-,- to -,+)left side
    side.set(-rsize,distance+rsize,rsize,distance+rsize,1);
    result.push_back(side);//(-,+ to ++)front side
    side.set(rsize, -rsize, rsize,distance+rsize,3);
    result.push_back(side);//(+- to ++)right side
    side.set(-rsize,-rsize,rsize,-rsize,4);//(-- to +-)bottom side
    result.push_back(side);
    
    
    Object currentRobotPose(0, 0, 0, 500, 1);
    Object originToMaxPath(0,0,getX(angle, distance), getY(angle, distance),1);
    vector<Object> output;
    
    for(unsigned int i = 0; i<result.size();i++) {
        side = remakeLineP2(originToMaxPath,currentRobotPose,result[i],1,0,1);
        output.push_back(side);
    }
    return output;
}

vector<Object> MyRobot::inMFIS(Object rmfis, Object rcv,int refpoint) 
{
	vector<Object> result;
	for(int i=0;i<int(sides.size());i++) 
                  {
		Object side=remakeLineP2(rmfis,rcv,sides[i],1,0, refpoint);
		result.push_back(side);
	}
	//allrobotpos.push_back(result);
	return result;
}

vector<Object> MyRobot::allRobPos(vector<vector<Object> > allrobotpos) {
	vector<Object> tmp;
	vector<Object> result;
	cout<<"size "<<allrobotpos.size()<<endl;
	for(int i=0;i<int(allrobotpos.size());i++) {
		tmp=allrobotpos[i];
		for(int j=0;j<int(tmp.size());j++) {
			result.push_back(tmp[j]);
		}
	}
	return result;
}

/*****************Obstacle ********************/
Obstacle::Obstacle(int i,double d, double a) {
	id=i;
	dist=d;
	angle=a;
}
int Obstacle::getID() {
	return id;
}
void Obstacle::display() {
	cout<<"obs id "<<id<<" dist "<<dist<<" angle "<<angle<<endl;
}
double Obstacle::getAngle() {
	return angle;
}
double Obstacle::getDist() {
	return dist;
}
void displayObstacles(vector<Obstacle> obs) {
	for(int i=0;i<int(obs.size());i++) 
		obs[i].display();
}
bool sortObsA2D(Obstacle a, Obstacle b) {
	return a.getDist() < b.getDist();
}

void Position::setID(int num)
{
    id = num;
}

void Position::set(double distance, double angleToHorizon, bool leftOrR, int num)
{
    dist = distance;
    angle = angleToHorizon;
    isLeft =  leftOrR;
    id = num;
}

void Position::set(double distance, double angleToHorizon, double DistoPath, bool leftOrR, int num)
{
    dist = distance;
    DisToPath = DistoPath;
    angle = angleToHorizon;
    isLeft =  leftOrR;
    id = num;
}


double Position::getDist()
{
    return dist;
}

int Position::getID()
{
    return id;
}


bool Position::getLeftOrR()
{
    return isLeft;
}

void Position::setflag(int flg)
{
    flag = flg;
}

int Position::getflag()
{
    return flag;
}

void Position::setDistoPath(double distance)
{
    DisToPath = distance;
}

double Position::getDistoPath()
{
    return DisToPath;
}

void Position::setPoint(Point pt)
{
    thePoint.set(pt.X(), pt.Y());
}

Point Position::getPoint()
{
    return thePoint;
}

/*
void Path::setCnt(int counter)
{
    SupportCnt = counter;
}
int Path::getCnt()
{
    return SupportCnt;
}
*/


vector<Point> ObjectToPoints(vector<Object> view)
{
    Point temp_point;
    vector<Point> temp;
    
    for(int i = 0; i < view.size(); i++)
    {
        if(i == 0)
        {
            temp_point.set(view[i].X1(), view[i].Y1());
            temp.push_back(temp_point);
            temp_point.set(view[i].X2(), view[i].Y2());
            temp.push_back(temp_point);
        }
        else
        {
            if(view[i].X1() != view[i-1].X2() 
                    && view[i].Y1() != view[i-1].Y2())
            {
                temp_point.set(view[i].X1(), view[i].Y1());
                temp.push_back(temp_point);
            }
            temp_point.set(view[i].X2(), view[i].Y2());
            temp.push_back(temp_point);
        }
    }
    
    return temp;
}


