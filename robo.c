#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "odom.c"


void roboTaskThread(){
  
  initGPIOEncoders();


  while (1){
    // contPulsos();
    computaOdometria();
    // usleep(dtLoopOdom_us);
    vTaskDelay(20/portTICK_RATE_MS);
  }
}