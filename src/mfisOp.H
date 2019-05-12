#include <iostream>



#include <vector>

#include "Object.H"
#include "Minfo.H"
#include "PathPlanning.H"

using namespace std;



vector<Object> getLinesIFoR(vector<Object> view);

vector<Object> findPotentialObjects(vector<Object> cv);
vector<Object> findTargetObjects(vector<Object> & cv);
vector<Object> findTargetObjectsFromMFIS(vector<Object>  cv);
bool isUpdateEssential(vector<Object> tobjects, double angle, double dist);
vector<vector<Object> > recognizeTargetObjects(vector<Object> mfis, vector<Object> cv, vector<Object> po_pv, vector<Object> pobjects_cv, Point rpos, double angle);
vector<vector<Object> > updateForTurned(vector<Object> tmpmfis, vector<Object> cv, vector<Object> next_po_pv, vector<Object> pobjects_cv, vector<Object> ref_obj, vector<double> distang);
vector<vector<Object> > update(vector<Object> tmpmfis, vector<Object> cv, vector<Object> pobjects_cv, vector<Object> refobjects);


vector<vector<Object> > updateMFIS(vector<Object> mfis, vector<Object> po_pv, vector<Object> cv, Point rpos, double angle, int v, vector<Object> ud_c);

vector<vector<Object> > newUpdateMFIS(vector<Object> mfis, vector<Object> po_pv, vector<Object> cv, double dist, double angle, int v, vector<Object> ud_c);

Object remakeLineP2(Object smfis, Object scv, Object rem, int id, double ang, int refpoint);
Object remakeLine_ondiff_point(Object smfis, Object scv, Object rem, int id, double ang, int refpoint);

vector<Object> projectingTheView(vector<Object> currentView, Object refObjectOld, Object refObjectNew, int refPoint);
vector<Object> projectingTheView_ondiff_point(vector<Object> currentView, Object refObjectOld, Object refObjectNew, int refPoint);
vector<Object> projectingTheView_with_correctDirection(vector<Object> currentView, Object refObjectOld, Object refObjectNew, int refPoint);
vector<Exit> projectingTheExit(vector<Exit> exits, Exit ref_oldExit, Exit ref_newExit, int refPoint);

//new mfis(algorithm-3)
vector<Object> updateIfNecessary(vector<Object> newMFIS,vector<Object> cview,vector<Object> exits_in_mfis,vector<vector<Object> > allrpos,vector<Object> ref_objects);
vector<vector<Object> > updateUsingRobotPosition(vector<Object> MFIS,vector<Object> cview,vector<Object> pobjects_cv,vector<Object> special_ref, vector<Object> ud_c);
