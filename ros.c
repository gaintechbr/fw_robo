#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <nav_msgs/msg/odometry.h>
#include <grobot_interfaces/msg/set_points_rodas.h>
#include "rosidl_runtime_c/string_functions.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "queueRoboROS.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


rcl_publisher_t publisher;
rcl_subscription_t subscriber;
nav_msgs__msg__Odometry odomMsg;
nav_msgs__msg__Odometry odomData;
grobot_interfaces__msg__SetPointsRodas setPointsMsg;


geometry_msgs__msg__Quaternion RPYToQuat(double roll, double pitch, double yaw){
	double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs__msg__Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

geometry_msgs__msg__PoseWithCovariance setPose(double x, double y, double theta){
	geometry_msgs__msg__PoseWithCovariance pose;

	memset(pose.covariance, 0, 36 * sizeof(double));
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = 0;
	
	pose.pose.orientation = RPYToQuat(0,0,theta);

	return pose;
}

geometry_msgs__msg__TwistWithCovariance setVel(double linear, double angular){
	geometry_msgs__msg__TwistWithCovariance vel;
	memset(vel.covariance, 0, 36 * sizeof(double));
	vel.twist.linear.x = linear;
	vel.twist.linear.y = 0;
	vel.twist.linear.z = 0;
	vel.twist.angular.x = 0;
	vel.twist.angular.y = 0;
	vel.twist.angular.z = angular;
	
	return vel;
}

void zeraOdometria(nav_msgs__msg__Odometry* msg, char * headerframeid, char * childframeid){
	msg->pose = setPose(0, 0, 0);
	rosidl_runtime_c__String__assign(&msg->header.frame_id, headerframeid);
	rosidl_runtime_c__String__assign(&msg->child_frame_id, childframeid);
	msg->twist = setVel(0,0);
}

void atualizaMsgOdom(odomQueueData_t* odomDataFromRobot){
	odomData.pose = setPose(odomDataFromRobot->poseX,odomDataFromRobot->poseY,odomDataFromRobot->poseTheta);
	odomData.twist = setVel(odomDataFromRobot->velLin,odomDataFromRobot->velAng);
	odomMsg = odomData;
}


/* Callback do timer para atualização e publicação da odometria */

void pubOdomTimerCallback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &odomMsg, NULL));
	}
}

/* Callback da subscrição do tópico cmd_vel, atualizando as mensagens de cmd vel */

void setPointsSubscriptionCallback(const void * msgin)
{
	const grobot_interfaces__msg__SetPointsRodas * setPointsMsg = (const grobot_interfaces__msg__SetPointsRodas*)msgin;
	xQueueSend(queueSetPoints, setPointsMsg, NULL);
}

void rosThreadTask(){
    zeraOdometria(&odomData, "odom", "base_link");
	zeraOdometria(&odomMsg, "odom", "base_link");

    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32", "esp32", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"odometry"));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(grobot_interfaces, msg, SetPointsRodas),
		"set_points_rodas"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 40;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		pubOdomTimerCallback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &setPointsMsg, &setPointsSubscriptionCallback, ON_NEW_DATA));

	zeraOdometria(&odomMsg,"odom", "base_link");
	printf("Setup do ROS finalizado\n");
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(40));
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}