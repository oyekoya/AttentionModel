/*
 *      LoadAttentionDll.cpp
 *
 *      Copyright 2010 Wole Oyekoya <w.oyekoya@cs.ucl.ac.uk>
 *		University College London
 */

#include <Windows.h>
#include <iostream>


typedef int (__cdecl *STARTPROC)(); 
typedef int (__cdecl *AVATARPROC)(int avatar_part, float x, float y, float z, float rx, float ry, float rz, float w); 
typedef int (__cdecl *UPDATEPROC)(char *id, float x, float y, float z, float rx, float ry, float rz, float w); 
typedef int (__cdecl *REMOVEPROC)(char *id); 
typedef int (__cdecl *EYEMODELPROC)(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w); 
typedef int (__cdecl *HEADMODELPROC)(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w); 
typedef int (__cdecl *QUATDIFFPROC)(float *result, float q1x, float q1y, float q1z, float q1w, float q2x, float q2y, float q2z, float q2w);
typedef int (__cdecl *PRINTPROC)();
typedef int (__cdecl *STOPPROC)(); 

STARTPROC _start; 
AVATARPROC _avatar;
UPDATEPROC _update; 
REMOVEPROC _remove;
EYEMODELPROC _eyemodel; 
HEADMODELPROC _headmodel; 
QUATDIFFPROC _quatdiff;
PRINTPROC _print;
STOPPROC _stop; 
HINSTANCE hinstLib; 

#ifdef WIN32
extern "C" __declspec(dllexport) void start()
#else
void start()
#endif
{
	(_start)();
}

//update global co-ordinates of nodes in-world
#ifdef WIN32
extern "C" __declspec(dllexport) void updateNode(char *id, float x, float y, float z, float rx, float ry, float rz, float w)
#else
void updateNode(char *id, float x, float y, float z, float rx, float ry, float rz, float w)
#endif
{
	(_update)(id,x,y,z,rx,ry,rz,w);
}

//update local co-ordinates of controlling avatar in-world
#ifdef WIN32
extern "C" __declspec(dllexport) void setAvatarCoords(int avatar_part, float x, float y, float z, float rx, float ry, float rz, float w)
#else
void setAvatarCoords(int avatar_part, float x, float y, float z, float rx, float ry, float rz, float w)
#endif
{
	(_avatar)(avatar_part,x,y,z,rx,ry,rz,w);
}
#ifdef WIN32
extern "C" __declspec(dllexport) void getEyeModelTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w)
#else
void getEyeModelTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w)
#endif
{
	(_eyemodel)(idstr, tx, ty, tz, eye_ax, eye_ay, eye_az, eye_w);
}

#ifdef WIN32
extern "C" __declspec(dllexport) void getHeadModelTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w)
#else
void getHeadModelTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w)
#endif
{
	(_headmodel)(idstr, tx, ty, tz, head_ax, head_ay, head_az, head_w);
}


#ifdef WIN32
extern "C" __declspec(dllexport) void removeNode(char *id)
#else
void removeNode(char *id)
#endif
{
	(_remove)(id);
}


#ifdef WIN32
extern "C" __declspec(dllexport) void quatDifference (float *result, float q1x, float q1y, float q1z, float q1w, float q2x, float q2y, float q2z, float q2w)
#else
void quatDifference (float *result, float q1x, float q1y, float q1z, float q1w, float q2x, float q2y, float q2z, float q2w)
#endif
{
	(_quatdiff)(result,q1x,q1y,q1z,q1w,q2x,q2y,q2z,q2w);
}


#ifdef WIN32
extern "C" __declspec(dllexport) void printOutput()
#else
void printOutput()
#endif
{
	(_print)();
}


#ifdef WIN32
extern "C" __declspec(dllexport) void stop()
#else
void stop()
#endif
{
	(_stop)();
}

int main(int argc, char**argv)
{
	char id[20];
	float px,py,pz,ex,ey,ez,ew,hx,hy,hz,hw;
	hinstLib = LoadLibrary(TEXT("attentionmodel.dll"));
	_start = (STARTPROC)GetProcAddress(hinstLib, "start");
	start();

	_avatar = (AVATARPROC)GetProcAddress(hinstLib, "setAvatarCoords");
	setAvatarCoords(1,0,0,0,0,1,0,0);
	setAvatarCoords(2,0,0,0,0,1,0,0);

	_update = (UPDATEPROC)GetProcAddress(hinstLib, "updateNode");
	updateNode("table",0,2,0,0,1,0,0);
	updateNode("chair",0,3,0,0,1,0,0);

	_eyemodel = (EYEMODELPROC)GetProcAddress(hinstLib, "getEyeModelTarget");
	getEyeModelTarget(id,&px,&py,&pz,&ex,&ey,&ez,&ew);
	_headmodel = (HEADMODELPROC)GetProcAddress(hinstLib, "getHeadModelTarget");
	getEyeModelTarget(id,&px,&py,&pz,&hx,&hy,&hz,&hw);

	_remove = (REMOVEPROC)GetProcAddress(hinstLib, "removeNode");
	removeNode("table");
	removeNode("chair");

	_stop =   (STOPPROC)GetProcAddress(hinstLib, "stop");
	stop();

/*
	1: // right eye's current global co-ordinates (in its default look straight ahead state)
	2: // head's current global co-ordinates (in its default look straight ahead state)

	int f=0;
	//float data[3];
	f++;
	updateNode(0,f%10,f%10,f%10,f%10,f%10,f%10,f%10);
*/
}