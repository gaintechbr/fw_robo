#ifndef _QUEUE_ROBO_ROS_H_
#define _QUEUE_ROBO_ROS_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

xQueueHandle queueOdom = NULL;

typedef struct odomQueueData
{
    double poseX;
    double poseY;
    double poseTheta;
    double velLin;
    double velAng;
} odomQueueData_t;

#endif