
/*
 *		attentionmodel.cpp
 *
 *      Copyright 2010 Wole Oyekoya <w.oyekoya@cs.ucl.ac.uk>
 *		University College London
 */

/// Includes
#include "model.h"
#include "attentionmodel.h"

attentionmodel *a_model;
float eye_x=0,eye_y=0,eye_z=0,eye_rx=0,eye_ry=0,eye_rz=0,eye_rw=0;
float head_x=0,head_y=0,head_z=0,head_rx=0,head_ry=0,head_rz=0,head_rw=0;

//start library: initialise model
#ifdef WIN32
MODEL_LIB_EXPORT void start()
#else
void start()
#endif
{
	a_model = new attentionmodel();
	std::cout<<"\n attention model initialised \n"<<std::endl;
}

//update global co-ordinates of controlling avatar's head and eye in-world
#ifdef WIN32
MODEL_LIB_EXPORT void setAvatarCoords(int avatar_part, float x, float y, float z, float rx, float ry, float rz, float w)
#else
void setAvatarCoords(int avatar_part, float x, float y, float z, float rx, float ry, float rz, float w)
#endif
{
	if (a_model == NULL) return;
	a_model->setAvatarInfo(avatar_part,x,y,z,rx,ry,rz,RAD_TO_DEG(w));
}

// get model targets on each frame
#ifdef WIN32
MODEL_LIB_EXPORT void getHeadModelTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w)
#else
void getHeadModelTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w)
#endif
{
	if (a_model == NULL) return;
	//std::cout<<"a_model running"<<std::endl;

	int attentionModelType = 1;
	int model_updated = 0;
	float a, b, c = 0;
	static float x3, y3, z3 = 0;
	float ax,ay,az,aw;

	//determine head vector
	a_model->HeadModel(&a, &b, &c, &model_updated);
	if (model_updated) {
		*tx = head_x = x3 = a; 
		*ty = head_y = y3 = b; 
		*tz = head_z = z3 = c;
	}	
	else {	
		*tx = head_x = x3; 
		*ty = head_y = y3; 
		*tz = head_z = z3; 
	}
	a_model->aimHeadAtTarget(x3,y3,z3,&ax,&ay,&az,&aw); //get axis-angle
	*head_ax = head_rx = ax;
	*head_ay = head_ry = ay;
	*head_az = head_rz = az;
	*head_w = head_rw = DEG_TO_RAD(aw);
	strcpy(idstr,a_model->head_target);
/*	CMatrix tmp(true);
	tmp.RotateMatrix(aw,ax,ay,az);
	tmp.ToEuler(&eh,&ep,&er); //convert to euler
	*head_h = eh;
	*head_p = ep;
	*head_r = er;*/
}

// get model targets on each frame
#ifdef WIN32
MODEL_LIB_EXPORT void getEyeModelTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w)
#else
void getEyeModelTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w)
#endif
{
	if (a_model == NULL) return;
	//std::cout<<"a_model running"<<std::endl;

	int attentionModelType = 1;
	int model_updated = 0;
	float a, b, c = 0;
	static float x1, y1, z1, x2, y2, z2 = 0;
	float ax,ay,az,aw;

	switch (attentionModelType){ //get new eye target based on specified model
		case 1: //Eye model
			a_model->EyeModel(&a, &b, &c, &model_updated);
			if (model_updated) {
				*tx = eye_x = x2 = a; 
				*ty = eye_y = y2 = b; 
				*tz = eye_z = z2 = c;
			}
			else {	
				*tx = eye_x = x2; 
				*ty = eye_y = y2; 
				*tz = eye_z = z2; }
			a_model->aimEyeAtTarget(x2,y2,z2,&ax,&ay,&az,&aw); //get quaternion 
			break;
		case 2: //Random model
			a_model->RandomModel(&a, &b, &c, &model_updated);
			if (model_updated) {
				*tx = eye_x = x1 = a; 
				*ty = eye_y = y1 = b; 
				*tz = eye_z = z1 = c;
			}
			else {	
				*tx = eye_x = x1; 
				*ty = eye_y = y1; 
				*tz = eye_z = z1; }
			a_model->aimEyeAtTarget(x1,y1,z1,&ax,&ay,&az,&aw);
			break;
	}//end switch
	*eye_ax = eye_rx = ax;
	*eye_ay = eye_ry = ay;
	*eye_az = eye_rz = az;
	*eye_w = eye_rw = DEG_TO_RAD(aw);
	strcpy(idstr,a_model->model_target);
/*	CMatrix tmp2(true);
	tmp2.RotateMatrix(aw,ax,ay,az);
	tmp2.ToEuler(&eh,&ep,&er); //convert to euler
	*eye_h = eh;
	*eye_p = ep;
	*eye_r = er;*/
}

// get model targets on each frame
#ifdef WIN32
MODEL_LIB_EXPORT void getNavModelTarget(char *idstr, float *tx, float *ty, float *tz)
#else
void getNavModelTarget(char *idstr, float *tx, float *ty, float *tz)
#endif
{
	if (a_model == NULL) return;
	//std::cout<<"a_model running"<<std::endl;
	int model_updated = 0;
	float a, b, c = 0;

	a_model->NavModel(&a, &b, &c, &model_updated);
	if (model_updated) {
		*tx = a; 
		*ty = b; 
		*tz = c;
	}
	strcpy(idstr,a_model->nav_target);
}

// get model targets on each frame
#ifdef WIN32
MODEL_LIB_EXPORT void getRandomNavModelTarget(char *idstr, float *tx, float *ty, float *tz)
#else
void getRandomNavModelTarget(char *idstr, float *tx, float *ty, float *tz)
#endif
{
	if (a_model == NULL) return;
	//std::cout<<"a_model running"<<std::endl;
	int model_updated = 0;
	float a, b, c = 0;

	a_model->RandomNavModel(&a, &b, &c, &model_updated);
	if (model_updated) {
		*tx = a; 
		*ty = b; 
		*tz = c;
	}
	strcpy(idstr,a_model->random_nav_target);
}

// returns calculated head target
#ifdef WIN32
MODEL_LIB_EXPORT void getHeadTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w)
#else
void getHeadTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w)
#endif
{
	if (a_model == NULL) return;
	strcpy(idstr,a_model->head_target);
	*tx = head_x;
	*ty = head_y;
	*tz = head_z; 
	*head_ax = head_rx;
	*head_ay = head_ry;
	*head_az = head_rz;
	*head_w = head_rw;

}

// returns calculated eye target
#ifdef WIN32
MODEL_LIB_EXPORT void getEyeTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w)
#else
void getEyeTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w)
#endif
{
	if (a_model == NULL) return;
	strcpy(idstr,a_model->model_target);
	*tx = eye_x;
	*ty = eye_y;
	*tz = eye_z; 
	*eye_ax = eye_rx;
	*eye_ay = eye_ry;
	*eye_az = eye_rz;
	*eye_w = eye_rw;
}

// Update global co-ordinates of nodes in-world
#ifdef WIN32
MODEL_LIB_EXPORT void updateNode(char *id, float x, float y, float z, float rx, float ry, float rz, float w)
#else
void updateNode(char *id, float x, float y, float z, float rx, float ry, float rz, float w)
#endif
{
	if (a_model == NULL) return;
	if (a_model->m_nodeDB.find(id) == a_model->m_nodeDB.end())
	{
		std::vector<float> myfloats (7,0.f);
		a_model->m_nodeDB[id] = myfloats;
	}
	a_model->m_nodeDB[id][0] = x;
	a_model->m_nodeDB[id][1] = y;
	a_model->m_nodeDB[id][2] = z;
	a_model->m_nodeDB[id][3] = rx;
	a_model->m_nodeDB[id][4] = ry;
	a_model->m_nodeDB[id][5] = rz;
	a_model->m_nodeDB[id][6] = RAD_TO_DEG(w);
	//std::cout<<a_model->m_nodeDB[id][1]<<" "<<std::endl;
}

// Remove node
#ifdef WIN32
MODEL_LIB_EXPORT void removeNode(char *id)
#else
void removeNode(char *id)
#endif
{
	if (a_model == NULL) return;
	a_model->m_nodeDB.erase(id);
	a_model->rot_x.erase(id);
	a_model->rot_y.erase(id);
	a_model->rot_z.erase(id);
	a_model->rot_w.erase(id);
	a_model->rot_x_eye.erase(id);
	a_model->rot_y_eye.erase(id);
	a_model->rot_z_eye.erase(id);
	a_model->rot_w_eye.erase(id);
	a_model->pos_x.erase(id);
	a_model->pos_y.erase(id);
	a_model->pos_z.erase(id);
	a_model->pos_x_eye.erase(id);
	a_model->pos_y_eye.erase(id);
	a_model->pos_z_eye.erase(id);
}

#ifdef WIN32
MODEL_LIB_EXPORT int printOutput()
#else
int printOutput()
#endif
{
	if (a_model == NULL) return 0;
	for ( std::map<std::string, std::vector<float> >::const_iterator it=a_model->m_nodeDB.begin() ; it != a_model->m_nodeDB.end(); it++ )
	{
		printf("%s ",it->first.c_str());
		for ( int i = 0; i < it->second.size(); i++ )
		{
			printf("%.3f ",it->second[i]);
		}
		printf("\n");
	}
	printf("eyepos %.3f %.3f %.3f \n",a_model->glob_eyeM.GetTranslate().x,a_model->glob_eyeM.GetTranslate().y,a_model->glob_eyeM.GetTranslate().z);
	float hh,hp,hr;
	a_model->glob_headM.ToEuler(&hh,&hp,&hr);
	printf("headpos %.3f %.3f %.3f %.3f %.3f %.3f\n",a_model->glob_headM.GetTranslate().x,a_model->glob_headM.GetTranslate().y,a_model->glob_headM.GetTranslate().z,hh,hp,hr);
	printf("model target %s; head target %s\n",a_model->model_target,a_model->head_target);
	printf("size=%i  ",a_model->m_nodeDB.size());
	return a_model->m_nodeDB.size();
}

#ifdef WIN32
MODEL_LIB_EXPORT void quatDifference (float *result, float q1x, float q1y, float q1z, float q1w, float q2x, float q2y, float q2z, float q2w)
#else
void quatDifference (float *result, float q1x, float q1y, float q1z, float q1w, float q2x, float q2y, float q2z, float q2w)
#endif
{
	if (a_model == NULL) return;
	CQuat q1 = CQuat(q1x,q1y,q1z,q1w);
	CQuat q2 = CQuat(q2x,q2y,q2z,q2w);
	CQuat q = a_model->QuatDifferential(q1,q2);
	result[0] = q.x; 
	result[1] = q.y; 
	result[2] = q.z; 
	result[3] = q.w;
}

//clean up and shutdown
#ifdef WIN32
MODEL_LIB_EXPORT void stop()
#else
void stop()
#endif
{
	if (a_model == NULL) return;
	if (a_model!=NULL)
		delete a_model;
	std::cout<<"\n attention model stopped \n"<<std::endl;
}

// this function allows to simplify XVR import of functions using autoexterndll
//but cannot take more than six parameters - useless!
/*extern "C" __declspec(dllexport) const char * _meta(int id)
{
    if (id != 0) return "";
	return "void start() //starts the server \n"
		"void setAvatarCoords(string id, int avatar_part, float x, float y, float z, float rx, float ry, float rz, float w) //set controlling avatar coords \n"
		"void updateNode(string id, float x, float y, float z, float rx, float ry, float rz, float w) // update node\n"
		"void removeNode(string id) //remove node \n"
		"void getEyeModelTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w) // compute eye target\n"
		"void getHeadModelTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w) // compute head target\n"
		"void stop() // stops the server \n";
}*/

