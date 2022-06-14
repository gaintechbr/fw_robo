#ifndef _QUEUE_ROBO_ROS_H_
#define _QUEUE_ROBO_ROS_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

xQueueHandle queueOdom = NULL;
xQueueHandle queueSPVelROdas = NULL;
xQueueHandle queueVelRodas = NULL;

typedef struct odomQueueData
{
    double poseX;
    double poseY;
    double poseTheta;
    double velLin;
    double velAng;
} odomQueueData_t;


typedef struct velRodasData{
    float rodaDireita;
    float rodaEsquerda;
} velRodasData_t;

#endif