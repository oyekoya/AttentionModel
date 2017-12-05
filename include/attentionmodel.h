/*
 *      attentionmodel.h
 *
 *      Copyright 2010 Wole Oyekoya <w.oyekoya@cs.ucl.ac.uk>
 *		University College London
 */
#ifndef ATTENTIONMODEL_H
#define ATTENTIONMODEL_H

#include "utilMath.h"
//#include <stdio.h>
#include <string>
#include <map>
#include <list>
#include <vector>
#include <iostream>
#include <fstream>

class attentionmodel
{
protected:
	typedef std::map<std::string, std::vector<float> >::const_iterator nodeDBIterator;
	double dur; //controls how long random is seeded
	bool head_is_moving;
	float headx, heady, headz;
	std::string headID;
	int model_fov;

public:

	attentionmodel();
	~attentionmodel();

	double getTime(void);
	CVec3 LerpCVec3( CVec3 a, CVec3 b, float f );
	CQuat QuatDifferential(CQuat q1, CQuat q2);
	void aimEyeAtTarget(float tX,float tY,float tZ,float *dstRX,float *dstRY,float *dstRZ,float *dstW);
	void aimHeadAtTarget(float tX,float tY,float tZ,float *dstRX,float *dstRY,float *dstRZ,float *dstW);
	void setAvatarInfo(int i, float x, float y, float z, float rx, float ry, float rz, float w);
	void EyeModel(float *tx, float *ty, float *tz, int *update);
	void HeadModel(float *tx, float *ty, float *tz, int *update);
	void NavModel(float *tx, float *ty, float *tz, int *update);
	void RandomModel(float *tx, float *ty, float *tz, int *update);
	void RandomNavModel(float *tx, float *ty, float *tz, int *update);

	std::map<std::string, std::vector<float> > m_nodeDB;

	std::map<std::string,float> rot_x; //store previous rot quaternions - head model
	std::map<std::string,float> rot_y;
	std::map<std::string,float> rot_z;
	std::map<std::string,float> rot_w;
	std::map<std::string,float> rot_x_eye; //store previous rot quaternions - eye model
	std::map<std::string,float> rot_y_eye;
	std::map<std::string,float> rot_z_eye;
	std::map<std::string,float> rot_w_eye;
	std::map<std::string,float> pos_x; //store previous node position - head model
	std::map<std::string,float> pos_y;
	std::map<std::string,float> pos_z;
	std::map<std::string,float> pos_x_eye; //store previous node position - eye model
	std::map<std::string,float> pos_y_eye;
	std::map<std::string,float> pos_z_eye;

	//store linear interpolation vector (6) for eyeball
	std::list<CVec3> LerpVectors;
	std::list<CVec3> LerpVectors_eye; //for eye

	//matrix for local coordinates of local avatar's head and right eye
	//and global coordinates of their parents respectively
	CMatrix glob_eyeM;	//set 1st param to 1 in setavatarinfo for right eye
	CMatrix glob_headM;	//set to 2 for global head

	char model_target[128];
	char head_target[128];
	char random_target[128];
	char random_nav_target[128];
	char nav_target[128];
};

#ifndef _MAIN_CPP_
extern attentionmodel * a_model;
#endif

#endif // ATTENTIONMODEL_H
