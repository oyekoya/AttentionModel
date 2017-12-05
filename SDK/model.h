#include <string>


#ifdef WIN32
#define MODEL_LIB_EXPORT __declspec(dllexport)
#else
#define MODEL_LIB_EXPORT  
#endif


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* Initialise model */
MODEL_LIB_EXPORT void start();


/*	Inform the model about the eye and head position of the attentionmodel-controlled avatar 
	avatar_part:
	1: // right eye's current global co-ordinates (in its default look straight ahead state)
	2: // head's current global co-ordinates (in its default look straight ahead state)*/
MODEL_LIB_EXPORT void setAvatarCoords(int avatar_part, float x, float y, float z, float rx, float ry, float rz, float w);


/* Get eye model targets on each frame*/
MODEL_LIB_EXPORT void getEyeModelTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w);

/* Get head model targets on each frame*/
MODEL_LIB_EXPORT void getHeadModelTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w);

/* Get eye model targets calculated from getEyeModelTarget*/
MODEL_LIB_EXPORT void getEyeTarget(char *idstr, float *tx, float *ty, float *tz, float *eye_ax, float *eye_ay, float *eye_az, float *eye_w);

/* Get head model targets calculated from getHeadModelTarget*/
MODEL_LIB_EXPORT void getHeadTarget(char *idstr, float *tx, float *ty, float *tz, float *head_ax, float *head_ay, float *head_az, float *head_w);

MODEL_LIB_EXPORT void getNavModelTarget(char *idstr, float *tx, float *ty, float *tz);

/* Update global co-ordinates of nodes in-world 
   On first call to update, the node id is added to database */
MODEL_LIB_EXPORT void updateNode(char *id, float x, float y, float z, float rx, float ry, float rz, float w);


/* Remove nodes/object to avoid the eyes being allocated to targets that are no longer in world */
MODEL_LIB_EXPORT void removeNode(char *id);


/* For debugging */
MODEL_LIB_EXPORT int printOutput();

/* Difference between two quaternions */
MODEL_LIB_EXPORT void quatDifference (float *result, float q1x, float q1y, float q1z, float q1w, float q2x, float q2y, float q2z, float q2w);


/* Clean up and shut down*/
MODEL_LIB_EXPORT void stop();


#ifdef __cplusplus
}
#endif /* __cplusplus */
