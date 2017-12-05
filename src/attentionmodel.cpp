/*
 *      attentionmodel.cpp
 *
 *      Copyright 2010 Wole Oyekoya <w.oyekoya@cs.ucl.ac.uk>
 *		University College London
 */
#include "attentionmodel.h"
#include <sys/timeb.h>
#include <time.h>
#include <windows.h>

attentionmodel::attentionmodel()
{
	dur = 750; //750ms
	head_is_moving = false;
	headx, heady, headz = 0.f;
	model_fov = 35;
	model_target[0] = '\0';
	random_target[0] = '\0';
	glob_eyeM.Identity();
	glob_headM.Identity();
}

/****************************************************************************/
attentionmodel::~attentionmodel()
{
}

/****************************************************************************/
CQuat attentionmodel::QuatDifferential(CQuat q1, CQuat q2)
{
		q1.Normalize();
		q2.Normalize();
		return (q2 * q1.Invert());
}

/****************************************************************************/
void attentionmodel::setAvatarInfo(int i, float x, float y, float z, float rx, float ry, float rz, float w)
{
	CMatrix M(true);
	M.Translate(CVec3(x,y,z));
	M.RotateMatrix(w,rx,ry,rz);
	switch (i)
	{
		case 1: // right eye's current global co-ordinates (in its default look straight ahead state)
			glob_eyeM = M;
			break;
		case 2: // head's current global co-ordinates (in its default look straight ahead state)
			glob_headM = M;
			//std::cout<<"x,y,z,rx,ry,rz,w: "<<x<<" "<<y<<" "<<z<<" "<<rx<<" "<<ry<<" "<<rz<<" "<<w<<" "<<std::endl;
			//float hh,hp,hr;
			//a_model->glob_headM.ToEuler(&hh,&hp,&hr);
			//printf("headpos %.3f %.3f %.3f %.3f %.3f %.3f\n",a_model->glob_headM.GetTranslate().x,a_model->glob_headM.GetTranslate().y,a_model->glob_headM.GetTranslate().z,hh,hp,hr);
			//M.ToEuler(&hh,&hp,&hr);
			//printf("headposM %.3f %.3f %.3f %.3f %.3f %.3f\n",M.GetTranslate().x,M.GetTranslate().y,M.GetTranslate().z,hh,hp,hr);
			break;
		default:
			std::cout<<"avatar part "<<i<<" does not exist"<<std::endl;
	}
}

/****************************************************************************/
CVec3 attentionmodel::LerpCVec3 ( CVec3 a, CVec3 b, float f )
{
	CVec3 dst;
	dst.x = a[0] + f * ( b[0] - a[0] ) ;
	dst.y = a[1] + f * ( b[1] - a[1] ) ;
	dst.z = a[2] + f * ( b[2] - a[2] ) ;
	//std::cout<<dst<<std::endl;
	return dst;
}
/****************************************************************************/
void attentionmodel::aimEyeAtTarget(float tX,float tY,float tZ,float *dstRX,float *dstRY,float *dstRZ,float *dstW)
{
	CMatrix eyeM(true);

	//get main avatar/participant's eye matrix
	eyeM = glob_eyeM;

	// get eye-to-target vector
	CVec3 aimV = CVec3(tX,tY,tZ) - eyeM.GetTranslate();
	aimV.Normalize();

	// get heading vector
	CVec3 f = eyeM * CVec3(0.f,0.f,-1.f);
	f.Normalize();

	// get rotation angle and axis from heading to aiming vector
	float angle_between, rx, ry, rz;
	CQuat shortest_arc;
	shortest_arc.shortestArc(aimV,f);
	if (shortest_arc.isFinite())
	{
		shortest_arc.constrain(DEG_TO_RAD(60.f)); 
		shortest_arc.getAngleAxis(&angle_between,&rx,&ry,&rz);
		*dstW = RAD_TO_DEG( angle_between );
		CVec3 c(rx,ry,rz);
		c.Normalize();
		*dstRX = c[0];
		*dstRY = c[1];
		*dstRZ = c[2];
	}
	else
	{ 
		*dstRX = 0.f;
		*dstRY = 0.f;
		*dstRZ = 0.f;
		*dstW  = RAD_TO_DEG( 180.f );
	}
}
/****************************************************************************/
void attentionmodel::aimHeadAtTarget(float tX,float tY,float tZ,float *dstRX,float *dstRY,float *dstRZ,float *dstW)
{
	CMatrix headM(true);

	//get main avatar/participant's head matrix
	headM = glob_headM;

	// get head-to-target vector
	CVec3 aimV = CVec3(tX,tY,tZ) - headM.GetTranslate();
	aimV.Normalize();

	// get heading vector
	CVec3 f = headM * CVec3(0.f,0.f,-1.f);
	f.Normalize();

	// get rotation angle and axis from heading to aiming vector
	float angle_between, rx, ry, rz;
	CQuat shortest_arc;
	shortest_arc.shortestArc(aimV,f);
	if (shortest_arc.isFinite())
	{
		shortest_arc.constrain(DEG_TO_RAD(60.f));
		shortest_arc.getAngleAxis(&angle_between,&rx,&ry,&rz);
		*dstW = RAD_TO_DEG( angle_between );
		CVec3 c(rx,ry,rz);
		c.Normalize();
		*dstRX = c[0];
		*dstRY = c[1];
		*dstRZ = c[2];
	}
	else
	{ 
		*dstRX = 0.f;
		*dstRY = 0.f;
		*dstRZ = 0.f;
		*dstW  = RAD_TO_DEG( 180.f );
	}
}

/****************************************************************************/
double attentionmodel::getTime(void)
{
	//SYSTEMTIME tst;
#ifdef WIN32
	struct _timeb timebuffer;
#else
	struct timeb timebuffer;
#endif
	char timeline [40];
	double millisec;

	//GetLocalTime( &tst );
#ifdef WIN32
	_ftime_s( &timebuffer );
#else
	ftime( &timebuffer );
#endif
	//GetTickCount();

	sprintf(timeline, "%lu%hu", timebuffer.time, timebuffer.millitm );
	//printf("%d\n", GetTickCount() );
	millisec = atof(timeline);
	return millisec;
}

/****************************************************************************/
//Eye model - generate eye target
void attentionmodel::EyeModel(float *tx, float *ty, float *tz, int *update)
{
	static float prev_timestamp = 0.f;
	float curr_time = GetTickCount() - prev_timestamp;
	static float cumu_time = 0;
	static int frame_count, lerpvec_size = 0;
	cumu_time += curr_time;
	prev_timestamp = GetTickCount();
	nodeDBIterator it;
	std::string text, node_id;
	int pos;
	//char originID[10];
	//sprintf(originID,"%s",avataroriginID.c_str());
	//std::string tmpID = avataroriginID;
	CMatrix m_globeye(true);
	float x,y,z,rx,ry,rz,w,s;
	std::map<std::string, float> nodeProbability;
	std::vector<std::string> items_fov;
	static CVec3 lastAimV = CVec3(0.f,0.f,0.f);
	static std::string prev_targetID, targetID;
	float dist,_dist,rotation_angle,eccentricity,angular_distance,sac_magnitude = 0;
	//char rightEyestr[10];
	static float pa, pb, pc = 0.f;
	std::vector<float> node;

	//randomly pick from 2 saliency levels (high or low)
	srand((unsigned)time(0)); //seed random every second
	//srand((unsigned)(int)floor(getTime()/dur));
	int saliency_level = (rand()%4); //high 0/1/2 and low 3
	float saliency_average = 0;
	int salient_count = 0;

	if (head_is_moving)
	{
		*tx = pa = headx; *ty = pb = heady; *tz = pc = headz;
		frame_count = lerpvec_size = cumu_time = 0;
		LerpVectors_eye.clear();
		prev_targetID = targetID = headID;
		lastAimV = CVec3(pa,pb,pc);
		*update = 1;
		return;
	}

	float avg_dist = 1.834;

	//get main avatar/participant's eye matrix
	m_globeye = glob_eyeM;

	// get heading vector
	CVec3 f = m_globeye * CVec3(0.f,0.f,-1.f);
	f.Normalize();

	for ( it=m_nodeDB.begin() ; it != m_nodeDB.end(); it++ )
	{
		node = m_nodeDB[it->first]; //access each node in turn
		if (node.empty()) continue;

		text = it->first;
		//need to convert to global-coords,
		//if (text.find(originID) != std::string::npos)
		//{ /*do nothing, skip if it's own node*/	}
		//else {
	    // get eye-to-currentnode vector
		CVec3 aimV = CVec3(node[0],node[1],node[2]) - CVec3(m_globeye.GetTranslate());
		aimV.Normalize();

		// get rotation angle from heading to aiming vector - eccentricity
		float angle_between = aimV.angle_between(aimV,f);
		rotation_angle = RAD_TO_DEG( angle_between );

		//angle between two quaternions, calculates differences in orientation between frames for each node
		CQuat q,q1,q2;
		q = CQuat(node[6],CVec3(node[3],node[4],node[5]));
		q.Normalize();
		q1 = q.Invert() * CQuat(rot_w_eye[text],CVec3(rot_x_eye[text],rot_y_eye[text],rot_z_eye[text]));
		float qw = q1.w;
		//std::cout<<qw<<std::endl;
		//store away as previous quaternion for next loop
		q2 = q;
		rot_x_eye[text]=q2.x; rot_y_eye[text]=q2.y; rot_z_eye[text]=q2.z; rot_w_eye[text]=q2.w;

		//differences in node positions between frames for each node
		float move_dist = sqrtf(powf((node[0]-pos_x_eye[text]),2)+powf((node[1]-pos_y_eye[text]),2)+powf((node[2]-pos_z_eye[text]),2));
		float node_velocity = move_dist / (0.001*curr_time); //convert to feet per second
		node_velocity /= 20; //normalize - assume max of 20 feet per second
		//store away as previous position for next loop
		pos_x_eye[text] = node[0]; pos_y_eye[text] = node[1]; pos_z_eye[text] = node[2];

		if ( rotation_angle < model_fov ) //add to db map if item within specified field of view
		{
			//std::cout<<text<<" "<<rotation_angle<<" ";
			//ensure that vertical angle is btw -25 and 25
			double eyeAngle = fabs((atan(aimV[1]/aimV[2]))*57.29578);
			if (eyeAngle > 25)
				continue;

			//calculate probability of occurence for the euclidean Distance
			// get eye-to-currentnode distance
			_dist = sqrtf(powf((node[0]-m_globeye.GetTranslate()[0]),2)+powf((node[1]-m_globeye.GetTranslate()[1]),2)+powf((node[2]-m_globeye.GetTranslate()[2]),2));
			float a1 = 19.11;  //(7.912, 30.32)
			float b1 = avg_dist;  //1.834(1.729, 1.938)
			float c1 = 0.8704;  //(0.586, 1.155)
			float a2 = 6.68;  //(2.328, 11.03)
			float b2 = avg_dist + 1.4;  //3.271(1.405, 5.136)
			float c2 = 1.7;  //(0.305, 3.095)
			dist = (a1*expf(-(powf(((_dist-b1)/c1),2)))) + (a2*expf(-(powf(((_dist-b2)/c2),2))));
			dist /= a1;
			//std::cout<<dist<<" ";

			// calculate probability of occurence for the eccentricity
			// Angular distance from the centre of gaze (head centric view).
			a1 = 40.13; //(39.28, 40.97)
			b1 = 14.39; //(14.31, 14.47)
			c1 = 4.175; //(4.07, 4.28)
			a2 = 8.089; //(5.318, 10.86)
			b2 = -14.05; //(-36.16, 8.05)
			c2 = 40.5; //(26.8, 54.2)
			eccentricity = (a1*expf(-(powf(((rotation_angle-b1)/c1),2)))) + (a2*expf(-(powf(((rotation_angle-b2)/c2),2))));
			eccentricity /= a1;
			//std::cout<<eccentricity<<" "<<qw/180<<" "<<node_velocity<<std::endl;

			// calculate probability of occurence for the saccade magnitude
			// Angle through which the eyeball rotates as it changes position from one target to another in the scene.
			float angular_distance = aimV.angle_between(aimV,lastAimV);
			angular_distance = RAD_TO_DEG( angular_distance );
			//if (angular_distance < 5) angular_distance = 5;
			a1 = 128.3; //(121.8, 134.8)
			b1 = -0.1729; //(-0.1806, -0.1652)
			sac_magnitude = a1*expf(b1*angular_distance);
			sac_magnitude /= a1;

			// calculate probability of occurence for the saccade velocity
			// Velocity is the magnitude divided by last frame duration
			a1 = 338.8;  //(287.3, 390.3)
			b1 = -0.01569;  //(-0.01711, -0.01427)
			float angular_velocity = angular_distance/(0.001*curr_time);
			float sac_velocity = a1*expf(b1*angular_velocity);
			sac_velocity /= a1;

			nodeProbability[text] = dist + eccentricity + (qw/180) + node_velocity /*+ (sac_magnitude/100) + (sac_velocity/10)*/;
			saliency_average += (dist + eccentricity + (qw/180) + node_velocity /*+ (sac_magnitude/100) + (sac_velocity/10)*/);
			salient_count++;
			items_fov.push_back(text);
		}
		//}//end if
	}//end for

	//if eyeball is currently being interpolated, send off next target vector
	//as long as new target is still within fov
	if ((1 <= frame_count) && (frame_count <= lerpvec_size) && (nodeProbability.find(targetID) != nodeProbability.end()))
	{
		frame_count++;
		CVec3 &pos = LerpVectors_eye.front();
		*tx = pos[0]; *ty = pos[1];	*tz = pos[2];
		LerpVectors_eye.pop_front();
		*update = 1;
		sprintf(model_target,"%s",targetID.c_str());
		//std::cout<<targetID<<" ";
		nodeProbability.clear();
		items_fov.clear();
		return;
	}

	//target is set to minimum 100ms threshold as long as the previous target is still within fov
	if ((cumu_time < 100) && (nodeProbability.find(prev_targetID) != nodeProbability.end()))
	{
	//std::cout<<targetID<<" ";
		*update = 0;
		nodeProbability.clear();
		items_fov.clear();
		return; //attempt to avoid jitters caused by targets moving in/out of FOV too quickly
	}
	else //if (nodeProbability.find(prev_targetID) == nodeProbability.end())
	{ //reset threshold if previous target is out of field of view and continue to get new target
		cumu_time = 0;
	}

	if (salient_count != 0)
		saliency_average /= salient_count;

	//include "NULL" for 'look straight ahead' - may be picked as low saliency_level
	items_fov.push_back("NULL");
	CVec3 str_aheadV;
	str_aheadV = ( m_globeye * CVec3(0,0,-5) ) + m_globeye.GetTranslate();

	_dist = sqrtf(powf((str_aheadV.x-m_globeye.GetTranslate()[1]),2)+powf((str_aheadV.y-m_globeye.GetTranslate()[1]),2)+powf((str_aheadV.z-m_globeye.GetTranslate()[2]),2));
	dist = ((19.11*expf(-(powf(((_dist-avg_dist)/0.8704),2)))) + (6.68*expf(-(powf(((_dist-(avg_dist + 1.4))/1.7),2)))))/19.11;
	rotation_angle = 0;//zero deg for straight-ahead
	eccentricity = ((40.13*expf(-(powf(((rotation_angle-14.39)/4.175),2)))) + (8.089*expf(-(powf(((rotation_angle-(-14.05))/40.5),2)))))/40.13;
	nodeProbability["NULL"] = dist + eccentricity;

	//future work - targets can be added at some arbitrary point (random ANGLE)

	//include "ANGLE" - this picks from upper right 0, lower right 1, lower left 2, or upper left 3
	//based on signing of lastAimV(x,y,z) i.e. if x and y is positive +ve or negative -ve
	//OR pick randomly from all 4 if last aimV is straight ahead (0,0,0)
	//Improves the model if there are limited targets within field of view
	//(-ve,+ve,z)3	|  0(+ve,+ve,z)
	//			____|____
	//				|
	//(-ve,-ve,z)2	|  1(+ve,-ve,z)
/*	float vAngle, hAngle;
	float angle_limit = (rand()%21);
	if (lastAimV == CVec3(0.f,0.f,0.f))
	{ //this chooses one of 4 angles up to 21 degrees - bias towards straight ahead
		int angle_choice = (rand()%4);
		switch (angle_choice)
		{
		case 0:
			hAngle = angle_limit;
			hAngle = angle_limit;
			break;
		case 1:
			hAngle = angle_limit;
			hAngle = -(angle_limit);
			break;
		case 2:
			hAngle = -(angle_limit);
			hAngle = -(angle_limit);
			break;
		case 3:
			hAngle = -(angle_limit);
			hAngle = angle_limit;
			break;
		}
	}
	else { //pick based on signing - nearest of the four to lastAimV
		if ( lastAimV[0] >= 0 ) { hAngle = angle_limit; } else { hAngle = -(angle_limit); }
		if ( lastAimV[1] >= 0 ) { vAngle = angle_limit; } else { vAngle = -(angle_limit); }
	}
	//calculate the offset of the angle picked baased on avg_dist
	CVec3 angleAimV;
	angleAimV[0] = avg_dist * tan(DEG_TO_RAD(hAngle));
	angleAimV[1] = avg_dist * tan(DEG_TO_RAD(vAngle));
	angleAimV[2] = avg_dist;
	CMatrix m_temp,m_tmpnode2; m_tmpnode2.setValue(m_tmpnode);
	m_temp.Translate(angleAimV[0],angleAimV[1],angleAimV[2]); //then translate by angle-offset
	m_tmpnode2.mult(m_temp);
	float ax,ay,az,aw = 0; tmpID = avataroriginID;
	//get eye's axis-angle rotation by aiming eye at the angle-offset
	aimEyeAtTarget(tmpID.append("er"),m_tmpnode2[3][0],m_tmpnode2[3][1],m_tmpnode2[3][2],&ax,&ay,&az,&aw);
	CMatrix m; CQuat q;
	q.setValueAsAxisDeg(ax,ay,az,aw);
	//apply axis-angle values to the eye's glob matrix
	m.setTransform(CVec3(m_tmpnode[3][0],m_tmpnode[3][1],m_tmpnode[3][2]),q);
	px,py,pz = 0;
	//to get the actual hitpoint of the eye (rather than using the angle offset)
	getTargetHitPnt(m,worldscene,&px,&py,&pz,false);
	//enabling accurate calculation of distance and eccentricity
	_dist = sqrtf(powf((px-m_globeye.GetTranslate()[0]),2)+powf((py-m_globeye.GetTranslate()[1]),2)+powf((pz-m_globeye.GetTranslate()[2]),2));
	dist = ((19.11*expf(-(powf(((_dist-avg_dist)/0.8704),2)))) + (6.68*expf(-(powf(((_dist-(avg_dist + 1.4))/1.7),2)))))/19.11;
	rotation_angle = angle_limit;
	eccentricity = ((40.13*expf(-(powf(((rotation_angle-14.39)/4.175),2)))) + (8.089*expf(-(powf(((rotation_angle-(-14.05))/40.5),2)))))/40.13;
	nodeProbability["ANGLE"] = dist + eccentricity;
*/

	if ( (saliency_level != 3) && (nodeProbability.size() != 0) )
	{
		//get highest saliency from the map
		float comp = 0;
		for ( std::map<std::string, float>::const_iterator cIter = nodeProbability.begin(); cIter!=nodeProbability.end(); cIter++ )
		{
			if (cIter == nodeProbability.begin())
			{
				targetID = cIter->first;
				comp = cIter->second;
			}else if (cIter->second > comp)
			{
				targetID = cIter->first;
				comp = cIter->second;
			}
		}
		//std::cout<<" smartEYES:"<<targetID<<" " <<nodeProbability[targetID]<<std::ends;
	} else if ( (saliency_level == 3) && (items_fov.size() != 0))
	{
		targetID = items_fov[rand() % items_fov.size()];
		//std::cout<<" randomEYES:"<<targetID<<std::ends;
	} else targetID = "NULL";
	static float a,b,c;
	float wx,wy,wz;
	//CVec3 aimV = str_aheadV - m_globeye.GetTranslate();
	//aimV.Normalize();
	//float angle_between;
	//aimV.enclosedAngle(f,&angle_between);
	if ( (nodeProbability.size() != 0) || (items_fov.size() != 0) )
	{
		//std::cout<< "Chosen one - " << item->getName() << "; NodeID - " << item->getNodeID() <<std::endl;
		if (targetID.find("NULL") != std::string::npos)
		{
			a = str_aheadV.x; b = str_aheadV.y; c = str_aheadV.z;
			//std::cout<<" straight-aheadEYES="<<RAD_TO_DEG( angle_between )<<std::ends;
			//a = 0; b = 0; c = 0;
		} else if (targetID.find("ANGLE") != std::string::npos)
		{
			std::cout<<"ANGLE";
			//a = px; b = py; c = pz;
		} else {
			node = m_nodeDB[targetID];
			a = node[0]; b = node[1]; c = node[2];
		}
		nodeProbability.clear();
	} else //if no item within field of view
	{
		targetID = "NULL";
		a = str_aheadV.x; b = str_aheadV.y; c = str_aheadV.z;
		//std::cout<<" straight-aheadEYES="<<RAD_TO_DEG( angle_between )<<std::ends;
		//a = 0; b = 0; c = 0;
	}
	//calculate eyeball interpolation for the next 6 frames (i.e. 6 target vectors)
	//when target changes and new target is not equal to NULL
	if ( targetID != prev_targetID )
	{
		frame_count = 1;
		LerpVectors_eye.clear();
		//compute 6 targets using curve fit y=14*exp((-(pi/4)*((x-3)^2)))
		//compute first target and send off
		CVec3 first_target = LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.02145);
		*tx = first_target[0]; *ty = first_target[1]; *tz = first_target[2];
		//compute remaining 5 and store into std::list to be accessed later
		LerpVectors_eye.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.24955));
		LerpVectors_eye.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.75009));
		LerpVectors_eye.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.97819));
		LerpVectors_eye.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.99964));
		LerpVectors_eye.push_back(CVec3(a,b,c));//arrives at new target here
		lerpvec_size = LerpVectors_eye.size();
	} else
	{//send off target as usual when there is no change from previous target
		*tx = a; *ty = b; *tz = c;
	}
	if ((a==0) && (b==0) && (c==0))
		lastAimV = CVec3(a,b,c);
	else {
		lastAimV = CVec3(a,b,c) - m_globeye.GetTranslate();
		lastAimV.Normalize();
	}
	//store current into previous
	prev_targetID = targetID;
	pa = a; pb = b; pc = c;
	*update = 1;
	sprintf(model_target,"%s",targetID.c_str());
}
/****************************************************************************/
//Head model - generate head target
void attentionmodel::HeadModel(float *tx, float *ty, float *tz, int *update)
{
	static float prev_timestamp = 0.f;
	float curr_time = GetTickCount() - prev_timestamp;
	static float cumu_time = 0;
	static int frame_count, lerpvec_size = 0;
	cumu_time += curr_time;
	prev_timestamp = GetTickCount();
	nodeDBIterator it;
	std::string text, node_id;
	int pos;
	//char originID[10];
	//sprintf(originID,"%s",avataroriginID.c_str());
	//std::string tmpID = avataroriginID;
	CMatrix m_globhead(true);
	float x,y,z,rx,ry,rz,w,s;
	std::map<std::string, float> nodeProbability;
	std::vector<std::string> items_fov;
	static CVec3 lastAimV = CVec3(0.f,0.f,0.f);
	static std::string prev_targetID, targetID;
	float dist,_dist,rotation_angle,eccentricity,angular_distance,sac_magnitude = 0;
	static float pa, pb, pc = 0.f;
	std::vector<float> node;

	//randomly pick from 2 saliency levels (high or low)
	srand((unsigned)time(0)); //seed random every second
	//srand((unsigned)(int)floor(getTime()/dur));
	int saliency_level = (rand()%4); //high 0/1/2 and low 3
	float saliency_average = 0;
	int salient_count = 0;
	float avg_dist = 1.834;

	//std::cout<<targetID<<std::endl;

	//get main avatar/participant's head matrix
	m_globhead = glob_headM;

	// get heading vector
	CVec3 f = m_globhead * CVec3(0.f,0.f,-1.f);
	f.Normalize();

	for ( it=m_nodeDB.begin() ; it != m_nodeDB.end(); it++ )
	{
		node = m_nodeDB[it->first]; //access each node in turn
		if (node.empty()) continue;

		text = it->first;
		//need to convert to global-coords,
		//if (text.find(originID) != std::string::npos)
		//{ /*do nothing, skip if it's own node*/	}
		//else {
	   // get head-to-currentnode vector
		CVec3 aimV = CVec3(node[0],node[1],node[2]) - m_globhead.GetTranslate();
		aimV.Normalize();

		// get rotation angle from heading to aiming vector - eccentricity
		float angle_between = aimV.angle_between(aimV,f);
		rotation_angle = RAD_TO_DEG( angle_between );

		//angle between two quaternions, calculates differences in orientation between frames for each node
		CQuat q,q1,q2;
		q = CQuat(node[6],CVec3(node[3],node[4],node[5]));
		q.Normalize();
		q1 = q.Invert() * CQuat(rot_w[text],CVec3(rot_x[text],rot_y[text],rot_z[text]));
		float qw = q1.w;
		//std::cout<<qw<<std::endl;
		//store away quaternion for next loop
		q2 = q;
		rot_x[text]=q2.x; rot_y[text]=q2.y; rot_z[text]=q2.z; rot_w[text]=q2.w;

		//differences in node positions between frames for each node
		float move_dist = sqrtf(powf((node[0]-pos_x[text]),2)+powf((node[1]-pos_y[text]),2)+powf((node[2]-pos_z[text]),2));
		float node_velocity = move_dist / (0.001*curr_time); //convert to feet per second
		node_velocity /= 20; //normalize - assume max of 20 feet per second
		//store away as previous position for next loop
		pos_x[text] = node[0]; pos_y[text] = node[1]; pos_z[text] = node[2];

		if ( rotation_angle < (60/*model_fov*2*/) ) //add to db map if item within specified field of view
		{
			//std::cout<<text<<" "<<rotation_angle<<" ";
			//ensure that vertical angle is btw -25 and 25
			double headAngle = fabs((atan(aimV[1]/aimV[2]))*57.29578);
			if (headAngle > 45)
				continue;

			//calculate probability of occurence for the euclidean Distance
			// get head-to-currentnode distance
			_dist = sqrtf(powf((node[0]-m_globhead.GetTranslate()[0]),2)+powf((node[1]-m_globhead.GetTranslate()[1]),2)+powf((node[2]-m_globhead.GetTranslate()[2]),2));
			float a1 = 19.11;  //(7.912, 30.32)
			float b1 = avg_dist;  //1.834(1.729, 1.938)
			float c1 = 0.8704;  //(0.586, 1.155)
			float a2 = 6.68;  //(2.328, 11.03)
			float b2 = avg_dist + 1.4;  //3.271(1.405, 5.136)
			float c2 = 1.7;  //(0.305, 3.095)
			dist = (a1*expf(-(powf(((_dist-b1)/c1),2)))) + (a2*expf(-(powf(((_dist-b2)/c2),2))));
			dist /= a1;
			//std::cout<<dist<<" ";

			// calculate probability of occurence for the eccentricity
			// Angular distance from the centre of gaze (head centric view).
			a1 = 40.13; //(39.28, 40.97)
			b1 = 14.39; //(14.31, 14.47)
			c1 = 4.175; //(4.07, 4.28)
			a2 = 8.089; //(5.318, 10.86)
			b2 = -14.05; //(-36.16, 8.05)
			c2 = 40.5; //(26.8, 54.2)
			eccentricity = (a1*expf(-(powf(((rotation_angle-b1)/c1),2)))) + (a2*expf(-(powf(((rotation_angle-b2)/c2),2))));
			eccentricity /= a1;
			//std::cout<<eccentricity<<" "<<qw/180<<" "<<node_velocity<<std::endl;

			// calculate probability of occurence for the saccade magnitude
			// Angle through which the head rotates as it changes position from one target to another in the scene.
			float angular_distance = aimV.angle_between(aimV,lastAimV);
			angular_distance = RAD_TO_DEG( angular_distance );
			//if (angular_distance < 5) angular_distance = 5;
			a1 = 128.3; //(121.8, 134.8)
			b1 = -0.1729; //(-0.1806, -0.1652)
			sac_magnitude = a1*expf(b1*angular_distance);
			sac_magnitude /= a1;

			// calculate probability of occurence for the saccade velocity
			// Velocity is the magnitude divided by last frame duration
			a1 = 338.8;  //(287.3, 390.3)
			b1 = -0.01569;  //(-0.01711, -0.01427)
			float angular_velocity = angular_distance/(0.001*curr_time);
			float sac_velocity = a1*expf(b1*angular_velocity);
			sac_velocity /= a1;

			nodeProbability[text] = dist + eccentricity + (qw/180) + node_velocity /*+ (sac_magnitude/100) + (sac_velocity/10)*/;
			saliency_average += (dist + eccentricity + (qw/180) + node_velocity /*+ (sac_magnitude/100) + (sac_velocity/10)*/);
			salient_count++;
			items_fov.push_back(text);
		}
		//}//end if
	}//end for

	//if head is currently being interpolated, send off next target vector
	//as long as new target is still within fov
	if ((1 <= frame_count) && (frame_count <= lerpvec_size) && (nodeProbability.find(targetID) != nodeProbability.end()))
	{
		frame_count++;
		CVec3 &pos = LerpVectors.front();
		*tx = pos[0]; *ty = pos[1];	*tz = pos[2];
		LerpVectors.pop_front();
		//std::cout<<LerpVectors.size()<<std::endl;
		//std::cout<<*tx<<" "<<*ty<<" "<<*tz<<" "<<std::endl;
		//strcpy(rightEyestr,originID); strcat(rightEyestr,"er");
		*update = 1;
		//since head is moving, store current target
		head_is_moving = true;
		headx = pos[0]; heady = pos[1];	headz = pos[2];
		//float ax,ay,az,aw = 0;
		//aimHeadAtTarget(pos[0],pos[1],pos[2],&ax,&ay,&az,&aw);
		nodeProbability.clear();
		items_fov.clear();
		return;
	} else head_is_moving = false;

	//target is set to minimum 1000ms threshold as long as the previous target is still within fov
	if ((cumu_time < 1000) && (nodeProbability.find(prev_targetID) != nodeProbability.end()))
	{
	//std::cout<<targetID<<" ";
		*update = 0;
		nodeProbability.clear();
		items_fov.clear();
		return; //attempt to avoid jitters caused by targets moving in/out of FOV too quickly
	}
	else //if (nodeProbability.find(prev_targetID) == nodeProbability.end())
	{ //reset threshold if previous target is out of field of view and continue to get new target
		cumu_time = 0;
	}

	if (salient_count != 0)
		saliency_average /= salient_count;

	//include "NULL" for 'look straight ahead' - may be picked as low saliency_level
	items_fov.push_back("NULL");
	CVec3 str_aheadV;
	str_aheadV = ( m_globhead * CVec3(0,0,-5) ) + m_globhead.GetTranslate();

	_dist = sqrtf(powf((str_aheadV.x-m_globhead.GetTranslate()[1]),2)+powf((str_aheadV.y-m_globhead.GetTranslate()[1]),2)+powf((str_aheadV.z-m_globhead.GetTranslate()[2]),2));
	dist = ((19.11*expf(-(powf(((_dist-avg_dist)/0.8704),2)))) + (6.68*expf(-(powf(((_dist-(avg_dist + 1.4))/1.7),2)))))/19.11;
	rotation_angle = 0;//zero deg for straight-ahead
	eccentricity = ((40.13*expf(-(powf(((rotation_angle-14.39)/4.175),2)))) + (8.089*expf(-(powf(((rotation_angle-(-14.05))/40.5),2)))))/40.13;
	nodeProbability["NULL"] = dist + eccentricity;

	if ( (saliency_level != 3) && (nodeProbability.size() != 0) )
	{
		//get highest saliency from the map
		float comp = 0;
		for ( std::map<std::string, float>::const_iterator cIter = nodeProbability.begin(); cIter!=nodeProbability.end(); cIter++ )
		{
			if (cIter == nodeProbability.begin())
			{
				targetID = cIter->first;
				comp = cIter->second;
			}else if (cIter->second > comp)
			{
				targetID = cIter->first;
				comp = cIter->second;
			}
		}
		//std::cout<<" smartHEAD:"<<targetID<<" " <<nodeProbability[targetID]<<std::ends;
	} else if ( (saliency_level == 3) && (items_fov.size() != 0))
	{
		targetID = items_fov[rand() % items_fov.size()];
		//std::cout<<" randomHEAD:"<<targetID<<std::ends;
	} else targetID = "NULL";
	static float a,b,c;
	float wx,wy,wz;
	//CVec3 aimV = str_aheadV - m_globhead.GetTranslate();
	//aimV.Normalize();
	//float angle_between;
	//aimV.enclosedAngle(f,&angle_between);
	if ( (nodeProbability.size() != 0) || (items_fov.size() != 0) )
	{
		//std::cout<< "Chosen one - " << item->getName() << "; NodeID - " << item->getNodeID() <<std::endl;
		if (targetID.find("NULL") != std::string::npos)
		{
			a = str_aheadV.x; b = str_aheadV.y; c = str_aheadV.z;
			//std::cout<<" straight-aheadHead="<<RAD_TO_DEG( angle_between )<<std::ends;
			//a = 0; b = 0; c = 0;
		} else if (targetID.find("ANGLE") != std::string::npos)
		{
			std::cout<<"ANGLE";
			//a = px; b = py; c = pz;
		} else {
			node = m_nodeDB[targetID];
			a = node[0]; b = node[1]; c = node[2];
		}
		nodeProbability.clear();
	} else //if no item within field of view
	{
		targetID = "NULL";
		a = str_aheadV.x; b = str_aheadV.y; c = str_aheadV.z;
		//std::cout<<" straight-aheadHead="<<RAD_TO_DEG( angle_between )<<std::ends;
		//std::cout<<" straight-aheadHEAD"<<std::ends;
		//a = 0; b = 0; c = 0;
	}
	//calculate head interpolation for the next 6 frames (i.e. 6 target vectors)
	//when target changes and new target is not equal to NULL
	if ( targetID != prev_targetID )
	{
		frame_count = 1;
		LerpVectors.clear();
		//compute 6 targets using curve fit y=(exp(0.027*t))*((sin((pi/2)*t))^1.90)
		//compute first target and send off
		CVec3 first_target = LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.02954);
		*tx = first_target[0]; *ty = first_target[1]; *tz = first_target[2];
		//since head is moving, store current target
		head_is_moving = true;
		headx = first_target[0]; heady = first_target[1];	headz = first_target[2];
		headID = prev_targetID;
		//std::cout<<" "<<std::endl;
		//std::cout<<*tx<<" "<<*ty<<" "<<*tz<<" "<<std::endl;
		//compute remaining 8 and store into std::list to be accessed later
		LerpVectors.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.10797));
		LerpVectors.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.22486));
		LerpVectors.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.3683));
		LerpVectors.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.52467));
		LerpVectors.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.67945));
		LerpVectors.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.81843));
		LerpVectors.push_back(LerpCVec3(CVec3(pa,pb,pc),CVec3(a,b,c),0.92891));
		LerpVectors.push_back(CVec3(a,b,c));//arrives at new target here
		lerpvec_size = LerpVectors.size();
	} else
	{//send off target as usual when there is no change from previous target
		*tx = a; *ty = b; *tz = c;
	}
	if ((a==0) && (b==0) && (c==0))
		lastAimV = CVec3(a,b,c);
	else {
		lastAimV = CVec3(a,b,c) - m_globhead.GetTranslate();
		lastAimV.Normalize();
	}
	//store current into previous
	prev_targetID = targetID;
	pa = a; pb = b; pc = c;
	*update = 1;
	sprintf(head_target,"%s",targetID.c_str());
}
/****************************************************************************/
//random model - generate eye target
void attentionmodel::RandomModel(float *tx, float *ty, float *tz, int *update)
{
	static float prev_timestamp = 0.f;
	float curr_time = GetTickCount() - prev_timestamp;
	prev_timestamp = GetTickCount();
	nodeDBIterator it;
	std::string text, node_id, targetID;
	int pos;
	//char originID[10];
	//sprintf(originID,"%s",avataroriginID.c_str());
	//std::string tmpID = avataroriginID;
	CMatrix m_globeye(true);
	std::vector<std::string> items_fov;//items within field of view
	srand((unsigned)time(0)); //seed random every second
	//srand((unsigned)(int)floor(getTime()/dur));
	float x,y,z,rx,ry,rz,w,s;
	std::vector<float> node;

	//get main avatar/participant's eye matrix
	m_globeye = glob_eyeM;

	// get heading vector
	CVec3 f = m_globeye * CVec3(0.f,0.f,-1.f);
	f.Normalize();

	for ( it=m_nodeDB.begin() ; it != m_nodeDB.end(); it++ )
	{
		node = m_nodeDB[it->first]; //access each node in turn
		if (node.empty()) continue;

		text = it->first;
		//need to convert to global-coords,
		//if (text.find(originID) != std::string::npos)
		//{ /*do nothing, skip if it's own node*/	}
		//else {
		// get eye-to-currentnode vector
		CVec3 aimV = CVec3(node[0],node[1],node[2]) - m_globeye.GetTranslate();
		aimV.Normalize();

		// get rotation angle from heading to aiming vector - eccentricity
		float angle_between = aimV.angle_between(aimV,f);
		float rotation_angle = RAD_TO_DEG(angle_between );

		if ( rotation_angle < model_fov ) //add to db if item within specified field of view
		{
			//ensure that vertical angle is btw -25 and 25
			double eyeAngle = fabs((atan(aimV[1]/aimV[2]))*57.29578);
			if (eyeAngle > 25)
				continue;
			items_fov.push_back(text);
		}
		//}//end if
	}//end for

	//include "NULL" for 'look straight ahead'
	items_fov.push_back("NULL");
	CVec3 str_aheadV;
	str_aheadV = ( m_globeye * CVec3(0,0,-5) ) + m_globeye.GetTranslate();

	float a,b,c,wx,wy,wz;
	//MatrixConvert mc;
	if (items_fov.size() != 0)
	{
		targetID = items_fov[rand() % items_fov.size()];
		//std::cout<< "Chosen one - " << item->getName() << "; NodeID - " << item->getNodeID() <<std::endl;
		if (targetID.find("NULL") != std::string::npos)
		{
			a = str_aheadV.x; b = str_aheadV.y; c = str_aheadV.z;
			//a = 0; b = 0; c = 0;
		} else {
			node = m_nodeDB[targetID];
			a = node[0]; b = node[1]; c = node[2];
		}
		items_fov.clear();
	} else //if no item within field of view
	{
		a = str_aheadV.x; b = str_aheadV.y; c = str_aheadV.z;
		//a = 0; b = 0; c = 0;
	}
	*tx = a;
	*ty = b;
	*tz = c;
	*update = 1;
	sprintf(random_target,"%s",targetID.c_str());
}

/****************************************************************************/
//Navigation model - generate navigation target
void attentionmodel::NavModel(float *tx, float *ty, float *tz, int *update)
{
	static float prev_timestamp = 0.f;
	float curr_time = GetTickCount() - prev_timestamp;
	prev_timestamp = GetTickCount();
	nodeDBIterator it;
	std::string text;
	CMatrix m_globhead(true);
	float x,y,z,rx,ry,rz,w,s;
	//std::map<std::string, float> nodeProbability;
	std::vector<std::string> items_fov;
	static CVec3 lastAimV = CVec3(0.f,0.f,0.f);
	static std::string prev_targetID, targetID;
	float dist,_dist,rotation_angle,angular_distance = 0.f;
	std::vector<float> node;

	//randomly pick from 2 saliency levels (high or low)
	//srand((unsigned)time(0)); //seed random every second
	//srand((unsigned)(int)floor(getTime()/dur));
	//int saliency_level = (rand()%4); //high 0/1/2 and low 3
	//float saliency_average = 0;
	//int salient_count = 0;
	//float avg_dist = 1.834;

	//std::cout<<targetID<<std::endl;

	//get main avatar/participant's head matrix
	m_globhead = glob_headM;

	for ( it=m_nodeDB.begin() ; it != m_nodeDB.end(); it++ )
	{
		node = m_nodeDB[it->first]; //access each node in turn
		if (node.empty()) continue;

		text = it->first;
	    // get head-to-currentnode vector
		CVec3 aimV = CVec3(node[0],node[1],node[2]) - m_globhead.GetTranslate();
		aimV.Normalize();

		//angle between two quaternions, calculates differences in orientation between frames for each node
		CQuat q,q1,q2;
		q = CQuat(node[6],CVec3(node[3],node[4],node[5]));
		q.Normalize();
		q1 = q.Invert() * CQuat(rot_w[text],CVec3(rot_x[text],rot_y[text],rot_z[text]));
		float qw = q1.w;
		//std::cout<<qw<<std::endl;
		//store away quaternion for next loop
		q2 = q;
		rot_x[text]=q2.x; rot_y[text]=q2.y; rot_z[text]=q2.z; rot_w[text]=q2.w;

		//differences in node positions between frames for each node
		float move_dist = sqrtf(powf((node[0]-pos_x[text]),2)+powf((node[1]-pos_y[text]),2)+powf((node[2]-pos_z[text]),2));
		//float node_velocity = move_dist / (0.001*curr_time); //convert to feet per second
		//node_velocity /= 20; //normalize - assume max of 20 feet per second
		//store away as previous position for next loop
		pos_x[text] = node[0]; pos_y[text] = node[1]; pos_z[text] = node[2];

		//calculate probability of occurence for the euclidean Distance
		// get head-to-currentnode distance
		_dist = sqrtf(powf((node[0]-m_globhead.GetTranslate()[0]),2)+powf((node[1]-m_globhead.GetTranslate()[1]),2)+powf((node[2]-m_globhead.GetTranslate()[2]),2));
		/*float a1 = 19.11;  //(7.912, 30.32)
		float b1 = avg_dist;  //1.834(1.729, 1.938)
		float c1 = 0.8704;  //(0.586, 1.155)
		float a2 = 6.68;  //(2.328, 11.03)
		float b2 = avg_dist + 1.4;  //3.271(1.405, 5.136)
		float c2 = 1.7;  //(0.305, 3.095)
		dist = (a1*expf(-(powf(((_dist-b1)/c1),2)))) + (a2*expf(-(powf(((_dist-b2)/c2),2))));
		dist /= a1;*/
		//std::cout<<dist<<" ";

		//compute the node's probability of interest
		//nodeProbability[text] = move_dist; //dist + (qw/180) + node_velocity;
		//saliency_average += move_dist; //(dist + (qw/180) + node_velocity);
		//salient_count++;
		if ( (move_dist != 0) || (qw != 0) )
			items_fov.push_back(text);
	}//end for

	//if (salient_count != 0)
	//	saliency_average /= salient_count;

	/*if ( (saliency_level != 3) && (nodeProbability.size() != 0) )
	{
		//get highest saliency from the map
		float comp = 0;
		for ( std::map<std::string, float>::const_iterator cIter = nodeProbability.begin(); cIter!=nodeProbability.end(); cIter++ )
		{
			if (cIter == nodeProbability.begin())
			{
				targetID = cIter->first;
				comp = cIter->second;
			}else if (cIter->second > comp)
			{
				targetID = cIter->first;
				comp = cIter->second;
			}
		}
		//std::cout<<" smartNAV:"<<targetID<<" " <<nodeProbability[targetID]<<std::ends;
	} else*/ if ( /*(saliency_level == 3) &&*/ (items_fov.size() != 0))
	{
		targetID = items_fov[rand() % items_fov.size()];
		//std::cout<<" randomNAV:"<<targetID<<std::ends;
	} else targetID = "NULL";
	static float a,b,c;
	//float wx,wy,wz;
	if ( /*(nodeProbability.size() != 0) ||*/ (items_fov.size() != 0) )
	{
		node = m_nodeDB[targetID];
		a = node[0]; b = node[1]; c = node[2];
		//nodeProbability.clear();
	} else //if no item
	{
		targetID = "NULL";
		a = 0; b = 0; c = 0;
	}
	//send off target as usual when there is no change from previous target
	*tx = a; *ty = b; *tz = c;
	*update = 1;
	sprintf(nav_target,"%s",targetID.c_str());
}

/****************************************************************************/
//random model - generate eye target
void attentionmodel::RandomNavModel(float *tx, float *ty, float *tz, int *update)
{
	static float prev_timestamp = 0.f;
	float curr_time = GetTickCount() - prev_timestamp;
	prev_timestamp = GetTickCount();
	nodeDBIterator it;
	std::string text, node_id, navID;
	int pos;
	std::vector<std::string> items_fov;//items within field of view
	srand((unsigned)time(0)); //seed random every second
	//srand((unsigned)(int)floor(getTime()/dur));
	float x,y,z,rx,ry,rz,w,s;
	std::vector<float> node;

	for ( it=m_nodeDB.begin() ; it != m_nodeDB.end(); it++ )
	{
		node = m_nodeDB[it->first]; //access each node in turn
		if (node.empty()) continue;

		text = it->first;
		items_fov.push_back(text);
	}//end for


	float a,b,c,wx,wy,wz;
	//MatrixConvert mc;
	if (items_fov.size() != 0)
	{
		navID = items_fov[rand() % items_fov.size()];
		node = m_nodeDB[navID];
		a = node[0]; b = node[1]; c = node[2];
		items_fov.clear();
	} else navID = "NULL";
	*tx = a;
	*ty = b;
	*tz = c;
	*update = 1;
	sprintf(random_nav_target,"%s",navID.c_str());
}
