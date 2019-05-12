#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cmath>
#include <dirent.h>
#include "Object.H"
#include "readAndwriteASCII.H"
#include "PointAndSurface.H"

#define PI 3.14159265

using namespace std;

/*
      This program reads in an ASCII file and fill up the data into vector of type Object
      Object class consists of id, co-od of p1 & p2, nearness, Occluding status of p1 & p2.

      Zati

        modified by 
	
        mhossain
        7 September, 2010
 */

vector<Object> readASCII(char *filename)
{
    
    //vector <double> vec;
    vector<Object> Objects;
    ifstream inputFile(filename, ios::in);
    if (!inputFile)
    {
        cout << "Error opening " << filename << " .." << endl;
        return Objects;
    } else
    {

        // start reading from the third data (ignore ASCII header)
        double row, colum, x1, y1, x2, y2;
        //double x, y;
        int ns = 0;
        inputFile >> row; //cout <<" row    = "<<data<<endl;
        inputFile >> colum; //cout <<" column = "<<data<<endl;
        //inputFile >> data;            //cout <<"push this data = "<<data<<endl;

        inputFile >> x1; //cout <<"push this data = "<<data<<endl;
        inputFile >> y1;
        inputFile >> x2;
        inputFile >> y2;

        /*
                x=x1+x2;
                y=y1+y2;
                double d=sqrt(x*x+y*y);
                if(d<15000)
                ns=1;
                else
                ns=0;*/
        int id = 1;
        while (!inputFile.eof())
        {
            Object s(x1, y1 - 20, x2, y2 - 20, id, ns); //0 for group id
            //Object s(x1, y1, x2, y2, id, ns);//0 for group id
            Objects.push_back(s);
            //  vec.push_back(data);
            inputFile >> x1; //cout <<"push this data = "<<data<<endl;
            inputFile >> y1;
            inputFile >> x2;
            inputFile >> y2;

            id++;
        }
    }
    //cout<<"was here"<<endl;
    inputFile.close();
    vector<Object> result = setOccludingEdges(Objects);
    //cout<<"was here"<<endl;
    //double th=600;
    //	int idd=1;
    //	result[0].setGID(idd);
    //	for(int i=0;i<int(result.size());i++)
    //	{	
    //		if(result[i].distP2ToP1(result[i+1])>th)
    //		{
    //			result[i].setGID(idd);
    //			idd++;
    //			result[i+1].setGID(idd);
    //		}
    //		else
    //		{
    //			result[i+1].setGID(idd);
    //		}
    //	}
    //cout<<"finished"<<endl;
    return result;
}

//This program reads in an ASCII file and fill up the data into vector of type double

vector <double> readCoordTrans(char *filename)
{

    vector <double> vec;

    ifstream inputFile(filename, ios::in);
    if (!inputFile)
    {
        cout << "Error opening " << filename << " .." << endl;
        exit(1);
    } else
    {

        // start reading from the third data (ignore ASCII header)
        double data;
        inputFile >> data; //cout <<" row    = "<<data<<endl;
        inputFile >> data; //cout <<" column = "<<data<<endl;
        inputFile >> data; //cout <<"push this data = "<<data<<endl;

        while (!inputFile.eof())
        {
            vec.push_back(data);
            inputFile >> data; //cout <<"push this data = "<<data<<endl;
        }
    }

    return vec;

}

//This program reads in an ASCII file and fill up the data into vector of type double

vector <Point> readPoints(char *filename)
{

    vector <Point> vec;

    ifstream inputFile(filename, ios::in);
    if (!inputFile)
    {
        cout << "Error opening " << filename << " .." << endl;
        exit(1);
    } else
    {

        // start reading from the third data (ignore ASCII header)
        double row, colum, x1, y1;
        //double x, y;

        inputFile >> row; //cout <<" row    = "<<data<<endl;
        inputFile >> colum; //cout <<" column = "<<data<<endl;
        //inputFile >> data;            //cout <<"push this data = "<<data<<endl;

        inputFile >> x1; //cout <<"push this data = "<<data<<endl;
        inputFile >> y1;


        while (!inputFile.eof())
        {
            vec.push_back(Point(x1, y1));

            inputFile >> x1; //cout <<"push this data = "<<data<<endl;
            inputFile >> y1;

        }
    }

    return vec;

}

/* Write a vector of Objects with Object id to a ASCII file
 Author 
        Thomas
        modified by

        mhossain
        7 Sept. 2010
 */

void writeASCII(vector<Object> Objects, char * filename)
{
    ofstream outFile(filename, ios::out);

    // Output ASCII header (row and column)
    outFile << Objects.size() << " " << 5 << endl;

    // 8 digits should be more than enough
    // outFile << fixed;
    //outFile.precision(10);

    for (int i = 0; i<int(Objects.size()); i++)
    {
        outFile << Objects[i].getID() << " ";
        outFile << Objects[i].X1() << " ";
        outFile << Objects[i].Y1() << " ";
        outFile << Objects[i].X2() << " ";
        outFile << Objects[i].Y2() << endl;
    }

    outFile.close();
}

void NoIDwriteASCII(vector<Object> Objects, char * filename)
{
    ofstream outFile(filename, ios::out);

    // Output ASCII header (row and column)
    outFile << Objects.size() << " " << 5 << endl;

    // 8 digits should be more than enough
    // outFile << fixed;
    //outFile.precision(10);

    for (int i = 0; i<int(Objects.size()); i++)
    {
        //outFile << Objects[i].getID() << " ";
        outFile << Objects[i].X1() << " ";
        outFile << Objects[i].Y1() << " ";
        outFile << Objects[i].X2() << " ";
        outFile << Objects[i].Y2() << endl;
    }

    outFile.close();
}


//from thomas
// Write a vector of surfaces to a file

void writeASCII(const vector<Surface> & surfaces, const char *filename)
{
    ofstream outFile(filename, ios::out);

    // Output ASCII header (row and column)
    outFile << surfaces.size() << " " << 4 << endl;

    // 8 digits should be more than enough
    outFile << fixed;
    outFile.precision(10);

    for (vector<Surface>::const_iterator it = surfaces.begin(); it != surfaces.end(); ++it)
    {
        outFile << (*it).getX1() << " ";
        outFile << (*it).getY1() << " ";
        outFile << (*it).getX2() << " ";
        outFile << (*it).getY2() << endl;
    }

    outFile.close();
}

//to write Coordinate Transformation information

void writeASCII(vector <double> vec, int column, char *outputFileName)
{

    ofstream outFile(outputFileName, ios::out);

    // output ASCII header (row and column)
    outFile << int(vec.size()) / column << " " << column << endl;

    int data_counter = 0;
    int column_counter = 1;

    for (int data = 0; data<int(vec.size()); data += column)
    {
        for (int next = (data_counter * column); next < (column_counter * column); next++)
        {
            outFile << vec[next] << " ";
        }
        outFile << endl;

        data_counter++;
        column_counter++;
    }
}
//to write a sequence of umbers

void writeNumbers(vector <int> vecInt, char *outputFileName)
{

    ofstream outFile(outputFileName, ios::out);
    for (int i = 0; i<int(vecInt.size()); i++)
    {
        outFile << vecInt[i];
        if (i != int(vecInt.size()) - 1)
        {
            outFile << " ";
        }
    }
    outFile << endl;
}

void writeNumbers(vector <double> vecDouble, char *outputFileName)
{

    ofstream outFile(outputFileName, ios::out);
    for (int i = 0; i<int(vecDouble.size()); i++)
    {
        outFile << vecDouble[i];
        if (i != int(vecDouble.size()) - 1)
        {
            outFile << " ";
        }
    }
    outFile << endl;
}
//to read a sequence of umbers

vector <int> readNumbers(char *inputFileName)
{
    vector<int> numbers;
    int num;
    ifstream inputFile(inputFileName, ios::in);
    if (!inputFile)
    {
        cout << "Error opening " << inputFileName << " .." << endl;
        return numbers;
    } else
    {
        inputFile >> num;
        while (!inputFile.eof())
        {
            numbers.push_back(num);
            cout << "This is a num:" << num << endl;
            inputFile >> num;
        }
    }
    inputFile.close();
    return numbers;
}

vector <double> readDoubleNumbers(char *inputFileName)
{
    vector<double> numbers;
    double num;
    ifstream inputFile(inputFileName, ios::in);
    if (!inputFile)
    {
        cout << "Error opening " << inputFileName << " .." << endl;
        return numbers;
    } else
    {
        inputFile >> num;
        while (!inputFile.eof())
        {
            numbers.push_back(num);
            inputFile >> num;

        }
    }
    inputFile.close();
    return numbers;
}

/*it receives folder name of current directory and then deletes all files
  from that folder
                by mhossain, 
                21 September, 2010
 */
void deleteFiles(char *dirname)
{
    char path[50];
    sprintf(path, "%s%s", "./", dirname);

    DIR *dir = opendir(path);
    if (dir)
    {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL)
        {
            //puts(ent->d_name);
            char filename[50];
            sprintf(filename, "%s%s%s%s", "./", dirname, "/", ent->d_name);
            //cout<<filename<<endl;
            //if(remove(filename)==-1)
            //cout<<"Error deleting file"<<endl;
            remove(filename);
        }
    } else
    {
        fprintf(stderr, "Error opening directory\n");
    }

}

// For writing all robot positions (along simplified path) into a file

void writePositions(vector<Point> Pos, char *filename)
{
    ofstream outFile(filename, ios::out);

    // Output ASCII header (row and column)
    outFile << Pos.size() << " " << 2 << endl;

    // 8 digits should be more than enough
    // outFile << fixed;
    //outFile.precision(10);

    for (int i = 0; i<int(Pos.size()); i++)
    {
        outFile << Pos[i].X() << " ";
        outFile << Pos[i].Y() << endl;
    }

    outFile.close();
}

void ConverToSLAMlog()
{
        Point singleRobotPosition;
        vector< Point > allRobotPosition;

        Point singleLaserPoint;
        vector<Point> aScan;
        vector< vector<Point> > allScans;


        string dName = "Maps/logfiles/";
        string addForLogFile = dName + "input.log";
        string addForAlbot1 = dName;

        double time;
        vector<double> times;
        ifstream inputFile(addForLogFile.c_str(), ios::in);
        

        
        cout << "Before Reading logfile...." << endl;
        if (inputFile.is_open()) 
        {
            cout << "Reading logfile...." << endl;

            double x, y, theta;
            string data;

            while (!inputFile.eof()) 
            {

                inputFile >> data;
                if (data.compare("robotGlobal:") == 0) 
                {
                    //reading odometry information
                    inputFile >> x;
                    inputFile >> y;
                    inputFile >> theta;
                    

                    //cout<<"x: "<<x<<" y: "<<y<<" th: "<<theta<<endl;
                    singleRobotPosition.set(x, y);
                    singleRobotPosition.setOAngle(theta);

                    allRobotPosition.push_back(singleRobotPosition);
                    //cout << "x: " << singleRobotPosition.X() << " y: " << singleRobotPosition.Y() << " th: " << theta + 90 << endl;
                    //waitHere();
 
                }
                inputFile >> data;
                if (data.compare("scan1:") == 0) 
                {

                    //fprintf(logFile, "Laser %d ", SENSE_NUMBER);

                    for (int i = 0; i < 181; i++) //181 for 1degree resolution 360 for .5 degree resolution
                    {
                        inputFile >> x;
                        inputFile >> y;
                        singleLaserPoint.set(x, y);
                        aScan.push_back(singleLaserPoint); //storing all laser readings for this scan
                        //cout << "x " << x << " y " << y;

                    }
                    allScans.push_back(aScan); //saving all scans
                    aScan.clear();
                    //waitHere();
                }
                
            }   

        } 
        else
        {
            cout << "Error opening " << addForLogFile << " .." << endl;
        }

        
        inputFile.close();


        //convert the format of log file 
        char *logFile;
        char fileName[10000];
        int SENSE_NUMBER = 181;
        int MAP_SCALE = 1000;
        double LaserDistance = 0;
        double last_x = 0, last_y = 0, last_theta = 0;
        
        
        //create directory of SLAM log 
        sprintf(fileName, "%s", "Maps/logfiles/slam.log");
        ofstream outFile(fileName, ios::app);
        for(int a = 0; a < allRobotPosition.size(); a++)
        {
           
            //write in the odometey            

            outFile << "Odometry"<<" "<< allRobotPosition[a].X() / MAP_SCALE 
                    <<" "<< allRobotPosition[a].Y() / MAP_SCALE 
                    <<" "<< (double)((int)((allRobotPosition[a].getOAngle() * PI / 180) * 1000000)) / 1000000;
            
            outFile << endl;
            outFile << "Laser" <<" "<< SENSE_NUMBER<<" ";
            for(int b = 0; b < allScans[a].size(); b++)
            {
                LaserDistance = ((int)(sqrt(allScans[a][b].X() * allScans[a][b].X()
                                              + allScans[a][b].Y() * allScans[a][b].Y()) * 1000)) / 1000;

                //write into the laser distance
                outFile << LaserDistance/MAP_SCALE<<" ";
                
            }
            outFile << endl;
            
            //last step odometric information
            last_x = allRobotPosition[a].X();
            last_y = allRobotPosition[a].Y();
            last_theta = allRobotPosition[a].getOAngle();
        }
        outFile.close();
}
