/*
 * app_types.h
 *
 *  Created on: 30-Jun-2019
 *      Author: hp
 */

#ifndef APP_INC_APP_TYPES_H_
#define APP_INC_APP_TYPES_H_



/* Parameters for task creation */
typedef struct TaskCreationParams_
{
	char * p_name;
	configSTACK_DEPTH_TYPE stackDepth;
	UBaseType_t priority;
}TaskCreationParams;


#ifdef __cplusplus
extern "C"{
#endif





#ifdef __cplusplus
}
#endif


#endif /* APP_INC_APP_TYPES_H_ */
