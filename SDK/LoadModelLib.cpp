/*
 *      LoadAttentionLib.cpp
 *
 *      Copyright 2010 Wole Oyekoya <w.oyekoya@cs.ucl.ac.uk>
 *		University College London
 */

#include <Windows.h>
#include <iostream>
#include "utilMath.h"
#include "model.h"

int main(int argc, char**argv)
{
	char id[20];
	float px,py,pz,ex,ey,ez,ew,hx,hy,hz,hw;
	start();

/*	SetAvatarCoords:
	1: // right eye's current global co-ordinates (in its default look straight ahead state)
	2: // head's current global co-ordinates (in its default look straight ahead state)
*/
	setAvatarCoords(1,0,0,0,0,1,0,0);
	setAvatarCoords(2,0,0,0,0,1,0,0);

	updateNode("table",0,2,0,0,1,0,0);
	updateNode("chair",0,3,0,0,1,0,0);

	getEyeModelTarget(id,&px,&py,&pz,&ex,&ey,&ez,&ew);
	Sleep(2500);
	getHeadModelTarget(id,&px,&py,&pz,&hx,&hy,&hz,&hw);

	removeNode("table");
	removeNode("chair");

	stop();

/*	int f=0;
	//float data[3];
	f++;
	updateNode(0,f%10,f%10,f%10,f%10,f%10,f%10,f%10);

*/
	CMatrix m2(true);
	m2.Translate(CVec3(3.f,4.f,5.f));
	m2.RotateMatrix(-90.f,0,1,0);
	CVec3 f2 = m2 * CVec3(0,0,-5);
	printf("%.3f, %.3f, %.3f\n",f2.x,f2.y,f2.z); 
	printf("\n");
	//NOTE: works fine for translating position along its line of rotation

	float angle_between;
	float hh,hp,hr;
	CVec3 aimV = CVec3(10,9,-5);
	CMatrix m1(true);
	m1.Translate(aimV);
	m1.RotateMatrix(90,0,1,0);
	CVec3 f = m1 * CVec3(0,0,-1);
	f.Normalize();
	//CMatrix m2(true);
	//std::cout<<m1.GetTranslate()[0]<<" "<<m1.GetTranslate()[1]<<" "<<m1.GetTranslate()[2]<<std::endl;
	aimV = aimV - CVec3(10,9,0);
	aimV.Normalize();
	//m2.RotateMatrix(90.f,0,1,0); //rotatematrix resets b4 rotating while rotate() rotates further
	//m2.ToEuler(&hh,&hp,&hr);
	//std::cout<<"euler: "<<hh<<" "<<hp<<" "<<hr<<" "<<std::endl;
	//CVec3 f2 = m2 * aimV;
	angle_between = aimV.angle_between(aimV,f);
	std::cout<<RAD_TO_DEG( angle_between )<<" "<<std::ends;
	float rx, ry, rz;
	CQuat shortest_arc;
	shortest_arc.shortestArc(aimV,f);
	shortest_arc.constrain(DEG_TO_RAD(60.f));
	shortest_arc.getAngleAxis(&angle_between,&rx,&ry,&rz);
	std::cout<<RAD_TO_DEG( angle_between )<<" axis="<<rx<<" "<<ry<<" "<<rz<<std::endl;
	Sleep(5000);


}