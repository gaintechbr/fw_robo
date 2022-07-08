#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ros.c"
#include "odom.c"
#include "pid.c"
#include "queueRoboROS.h"

#define CONFIG_MICRO_ROS_APP_STACK 16000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 7

static void odomQueueTask(void* arg)
{
    odomQueueData_t odomDataFromQueue;
    for(;;) {
        if(xQueueReceive(queueOdom, &odomDataFromQueue, portMAX_DELAY)) {
			atualizaMsgOdom(&odomDataFromQueue);
        }
    }
}

static void setPointsQueueTask(void* arg)
{
    grobot_interfaces__msg__SetPointsRodas setPoints;
    for(;;) {
        if(xQueueReceive(queueSetPoints, &setPoints, portMAX_DELAY)) {
			atualizaSetPoints(&setPoints);
        }
    }
}

static void velRodasQueueTask(void* arg)
{
    velRodasData_t velRodas;
    for(;;) {
        if(xQueueReceive(queueVelRodas, &velRodas, portMAX_DELAY)) {
			atualizaVelRodas(&velRodas);
        }
    }
}

void appMain(void * arg)
{
	queueOdom = xQueueCreate(10, sizeof(odomQueueData_t));
	queueSetPoints = xQueueCreate(10, sizeof(grobot_interfaces__msg__SetPointsRodas));
	queueVelRodas = xQueueCreate(10, sizeof(velRodasData_t));

	xTaskCreate(odomQueueTask, "odom_queueTask", 2048, NULL, 10, NULL);
	xTaskCreate(setPointsQueueTask, "setpoints_queueTask", 2048, NULL, 10, NULL);
	xTaskCreate(velRodasQueueTask, "velrodas_queueTask", 2048, NULL, 10, NULL);

	xTaskCreatePinnedToCore(rosThreadTask,
							"ros_thread",
							CONFIG_MICRO_ROS_APP_STACK,
							NULL,
							CONFIG_MICRO_ROS_APP_TASK_PRIO,
							NULL,0);

	xTaskCreatePinnedToCore(odomTaskThread,
							"odom_thread",
							CONFIG_MICRO_ROS_APP_STACK,
							NULL,
							CONFIG_MICRO_ROS_APP_TASK_PRIO,
							NULL,1);

	xTaskCreate(pidTaskThread,
				"pid_thread",
				CONFIG_MICRO_ROS_APP_STACK,
				NULL,
				CONFIG_MICRO_ROS_APP_TASK_PRIO,
				NULL);

	while(1){
		sleep(10);
	}

  	vTaskDelete(NULL);
}
