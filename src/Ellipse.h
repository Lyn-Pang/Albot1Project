
/* Ellipse class 
 *  Lyn Pang
 *  2017-02-20
 */


#ifndef _Ellipse_H
#define	_Ellipse_H

#include <vector>
#include "Point.H"



using namespace std;

class Ellipse 
{
    private:
        
        Point centre;
        Point left_end;
        Point right_end;
        double heigth;
        double width;
        double orientation;
        vector<Point> ellip;
        int VN;
        
    public:	
        void setCentre(Point centre_point);
        Point getCentre();
        void setHeigth(double negtive_axis);
        double getHeigth();
        void setwidth(double principle_axis);
        double getwidth();
        void setOrientation(double orient);
        double getOrientation();
        void setOneEllipse(Point centre_point, double H, double W, double orient);
        void setlTwoEndpoints(Point left, Point right);
        vector<Point> fourEndPoints();
        Object getHorizonline();
        Object getVirticalline();
        
        void setVN(int viewNumber);
        int getVN();
};


#endif