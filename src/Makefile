# This makefile builds all programs from PROGSRCS and links them with the source files from LIBSRCS.

CC=g++-4.4
CFLAGS=-c -O1 -Wall -fPIC -I/home/arthur/Lyn -I/usr/local/Aria/include -I/usr/local/include/opencv2 -I/usr/local/include 
LDFLAGS=-L/usr/local/Aria/lib  -O1 -lAria -lpthread -ldl -lrt 
#LDFLAGS = -L/usr/local/Aria/lib \
          -L/usr/local/include/opencv2 \

#LDLIBS =  -lAria -lpthread -ldl -lrt \ 
#	  -lopencv_core -lopencv_videoio -lopencv_highgui \
	  -lopencv_imgproc -lopencv_video -lopencv_objdetect \
	     
#LDFLAGS	+= `pkg-config --libs opencv mrpt-base mrpt-slam mrpt-gui`

LIBSRCS= \
	Object.cpp \
	Point.cpp \
	asr.cpp \
	GeometricOp.cpp \
	asrOp.cpp \
	PerceptualMapping.cpp \
	mfisOp.cpp \
	Mapping.cpp \
	readAndwriteASCII.cpp \
	Minfo.cpp \
	PointAndSurface.cpp \
	GeometryFuncs.cpp \
	GroupS.cpp  \
	Transporter.cpp \
	Plotting.cpp \
	PathPlanning.cpp \
	Map.cpp \
	CompareASR.cpp \
	CompareASRClass.cpp \
	Laser2Surface.cpp \
	Polygon.cpp  \
	PolygonChoice.cpp  \
	space.cpp  \
	display.cpp \
	convexPathPlanner.cpp \
	Comparison.cpp \
	thesis.cpp \
	clipper.cpp \
	RobotPosition.cpp \
	ToolFunctions.cpp \
	Memory.cpp \
	Returning.cpp \
	ChunksOp.cpp \
	Exploration.cpp \
	Kalman_filter.cpp \
	GeometryAndExit.cpp \
	Ellipse.cpp \
	RouteMapModule.cpp \
	GlobalMapModule.cpp \
	PathIntegrationModule.cpp \
	ConvexHull.cpp \
	concave_hull.cpp \
	Exit.cpp \
	TestCombineData.cpp \
	FunctionOfMFIS.cpp \
	ChunkInfor.cpp \
	Expectation.cpp \
	StructBoundary.cpp \
	
	
    
# These are the executable programs with a main() function
PROGSRCS= \
    GXplorationAndMapping.cpp \
    ExploringandReturn.cpp \
    convertlog.cpp \
    localmapAndglobal.cpp \
    localmapGen.cpp \
    localMapsAndChunk.cpp \
    localMapAndExpectation.cpp \
    FindGap.cpp \
    WanderAndMapping.cpp \
    AutoXploreAndMapping.cpp \
    SimplyHoming.cpp \
    ReturnHome.cpp \
    Testboost.cpp \
    GlobalAndRouteMap.cpp \
    GxplorationAndExpectation.cpp \
    GxplorationAndExpectation_DeletionModify.cpp \
    GxplorationAndEnduringmap.cpp \
    
    

LIBOBJS = $(LIBSRCS:%.cpp=../obj/%.o)
PROGOBJS = $(PROGSRCS:%.cpp=../obj/%.o)
LIBDEPS = $(PROGOBJS:.o=.o.d)
PROGDEPS = $(LIBOBJS:.o=.o.d)

PROG = $(PROGSRCS:%.cpp=../bin/%)

all: $(LIBOBJS) $(PROGOBJS) $(PROG)

# Linking: for each executable, link its o file together with all the non-executable o files
../bin/%: $(LIBOBJS) $(PROGOBJS)
	$(CC) $(LDFLAGS) ../obj/$*.o $(LIBOBJS) -o ../bin/$*

# Compiling: first make dependencies, then compile all cpp files
../obj/%.o: %.cpp
	$(CC) $(CFLAGS) $< -MM -MT $@ -MF $@.d
	$(CC) $(CFLAGS) $< -o $@

clean:      ; rm $(LIBOBJS) $(PROGOBJS) $(LIBDEPS) $(PROGDEPS) $(PROG)

# Include the dependencies so the cpp files are recompiled if headers change
-include $(LIBDEPS)
-include $(PROGDEPS)
