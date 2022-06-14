#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ros.c"
#include "robo.c"
#include "queueRobo.h"

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

void appMain(void * arg)
{
	queueOdom = xQueueCreate(10, sizeof(odomQueueData_t));
	xTaskCreate(odomQueueTask, "odom_queueTask", 2048, NULL, 10, NULL);

	xTaskCreate(rosThreadTask,
							"ros_thread",
							CONFIG_MICRO_ROS_APP_STACK,
							NULL,
							CONFIG_MICRO_ROS_APP_TASK_PRIO,
							NULL);

	xTaskCreate(roboTaskThread,
							"robo_thread",
							CONFIG_MICRO_ROS_APP_STACK*2,
							NULL,
							CONFIG_MICRO_ROS_APP_TASK_PRIO,
							NULL);

	while(1){
		sleep(10);
	}

  	vTaskDelete(NULL);


}
