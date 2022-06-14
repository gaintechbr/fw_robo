#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "odom.c"


void roboTaskThread(){
  gpio_config_t io_conf = {};

  //Configurações comuns para os dois encoders

  // Interrupção na borda de subida
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  // Modo setado como entrada
  io_conf.mode = GPIO_MODE_INPUT;
  // Habilita modo pullup
  io_conf.pull_up_en = 1;

  // Configura pinos encoder direito
  // Bit mask para os pinos, usando GPIOs configuradas para encoder direito
  io_conf.pin_bit_mask = GPIO_ENC_DIR_PIN_SEL;
  gpio_config(&io_conf);
  
  // Configura pinos encoder direito
  // Bit mask para os pinos, usando GPIOs configuradas para encoder direito
  io_conf.pin_bit_mask = GPIO_ENC_ESQ_PIN_SEL;
  gpio_config(&io_conf);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

  gpio_isr_handler_add(ROTARY_ESQUERDA_PIN_A, encoder_esquerdo, (void*) ROTARY_ESQUERDA_PIN_A);
  gpio_isr_handler_add(ROTARY_ESQUERDA_PIN_B, encoder_esquerdo, (void*) ROTARY_ESQUERDA_PIN_B);
  
  gpio_isr_handler_add(ROTARY_DIREITA_PIN_A, encoder_direito, (void*) ROTARY_DIREITA_PIN_A);
  gpio_isr_handler_add(ROTARY_DIREITA_PIN_B, encoder_direito, (void*) ROTARY_DIREITA_PIN_B);
  
  // Inicializa Odometria
  // Inicializa interrupções GPIO para leitura dos encoders
  // encodersGPIOISRInit();
  // Inicializa interrupção do timer para calculo da odometria

  int timer_group = TIMER_GROUP_0;
  int timer_idx = TIMER_0;
  timer_config_t config;
  config.alarm_en = 1;
  config.auto_reload = 1;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = TIMER_DIVIDER;
  config.intr_type = TIMER_INTR_SEL;
  config.counter_en = TIMER_PAUSE;
  /*Configure timer*/
  timer_init(timer_group, timer_idx, &config);
  /*Stop timer counter*/
  timer_pause(timer_group, timer_idx);
  /*Load counter value */
  timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
  /*Set alarm value*/
  timer_set_alarm_value(timer_group, timer_idx, (ODOM_TIMER_INTERVAL_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
  /*Enable timer interrupt*/
  timer_enable_intr(timer_group, timer_idx);
  /*Set ISR handler*/
  timer_isr_register(timer_group, timer_idx, odomTimerCallback, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
  /*Start timer counter*/
  timer_start(timer_group, timer_idx);
  // odomTimerInit();

  while (1){
    // computaOdometria();
    vTaskDelay(10000/portTICK_RATE_MS);
    // sleep(10);
  }

}