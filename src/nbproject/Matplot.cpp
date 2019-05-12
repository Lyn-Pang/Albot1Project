/*
 * Plotting: plotting information using Matplot/python library .
 *  2D/3D function for plotting
 * 
 *  Wenwang (Lyn) 
 * 
 */
#include <vector>

#include <iostream>
#include <cmath>
#include <deque>
#include "readAndwriteASCII.H"
#include <fstream>
#include <python3.6m/Python.h>



void plotObjects(const char * filename, vector<Object> Objects) 
{
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) 
    {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) 
    {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    
    if (diff > 0) 
    {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } 
    else 
    {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;

    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);

    fprintf(fgnup, "plot \"-\" ti \"Objects\" with linespoints 1 19, \n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) 
    {
        
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

/*
void PlotView(const char * filename, vector<Object> Objects)
{
    Py_Initialize();
    
    Py_Finalize();
}
*/



