#ifndef _READWRITEASCII_H_
#define _READWRITEASCII_H_

#include <vector>
#include <cstdlib>
#include "Object.H"
#include "PointAndSurface.H"

using namespace std;

vector <Object> readASCII (char *filename1);
vector <double> readCoordTrans (char *filename1);
vector <Point> readPoints (char *filename);
void writeASCII(vector<Object> Objects, char *filename);
void NoIDwriteASCII(vector<Object> Objects, char *filename);
void writeASCII(const vector<Surface> & surfaces, const char *filename);
void writeASCII (vector <double> vec, int column, char *outputFileName);
vector <int> readNumbers(char *inputFileName);
vector <double> readDoubleNumbers(char *inputFileName);
void writeNumbers(vector <int> vecInt, char *outputFileName);
void writeNumbers(vector <double> vecDouble, char *outputFileName);
void deleteFiles(char *dirname);
void writePositions(vector<Point> Pos, char *filename);
void ConverToSLAMlog();


#endif
