#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "queueRoboROS.h"
#include "driver/timer.h"


// =================== CONFIG DO TIMER ==============================

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   80               /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
// #define ODOM_TIMER_INTERVAL_SEC   (0.02)   /*!< test interval for timer 0 */

// =================== CONFIG DOS GPIOs DA ODOM ==============================

#define ESP_INTR_FLAG_DEFAULT 0

#define ROTARY_DIREITA_PIN_A 4
#define ROTARY_DIREITA_PIN_B 5 
#define ROTARY_ESQUERDA_PIN_A 18
#define ROTARY_ESQUERDA_PIN_B 19

#define GPIO_ENC_DIR_PIN_SEL  ((1ULL<<ROTARY_DIREITA_PIN_A) | (1ULL<<ROTARY_DIREITA_PIN_B))
#define GPIO_ENC_ESQ_PIN_SEL  ((1ULL<<ROTARY_ESQUERDA_PIN_A) | (1ULL<<ROTARY_ESQUERDA_PIN_B))

// //################################## Variaveis Leitura Encoder Direita ############################

int64_t contPulsosRodaDireita=0;
int64_t contPulsosRodaDireitaAnterior=0;
uint8_t estadoRodaDireita=0;

// //################################## Variaveis Leitura Encoder Esquerda ############################

int64_t contPulsosRodaEsquerda=0;
int64_t contPulsosRodaEsquerdaAnterior=0;
uint8_t estadoRodaEsquerda=0;

// //################################## Variaveis da Odometria #######################################

const int dtLoopOdom_us = 20000;                                                                     //base em micro segundos
double dtLoopOdom_s = dtLoopOdom_us / 1000000.0;

uint16_t pprRodaDireita = 9110; // PARAMETRO ROS
uint16_t pprRodaEsquerda = 8893; // PARAMETRO ROS

#define distancia_entre_eixos             0.495 // PARAMETRO ROS
#define raio_da_roda_direita              0.13 // PARAMETRO ROS
#define raio_da_roda_esquerda             0.13 // PARAMETRO ROS                                                            //está em cm
#define dist_entre_pulsos_roda_dir_em_cm  2 * M_PI * raio_da_roda_direita/(pprRodaDireita*1.0)
#define dist_entre_pulsos_roda_esq_em_cm  2 * M_PI * raio_da_roda_esquerda/(pprRodaEsquerda*1.0)

double x = 0.0;
double y = 0.0;
double theta = 0;
double xAnterior = 0.0;
double yAnterior = 0.0;
double thetaAnterior = 0.0;

odomQueueData_t odomDataToSend;
velRodasData_t velRodasDataToSend;

portMUX_TYPE muxEncDireito = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxEncEsquerdo = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR encoder_direito(void* arg) {
  
  uint8_t s_d = estadoRodaDireita & 3;
  portENTER_CRITICAL_ISR(&muxEncDireito);
  
  if (gpio_get_level(ROTARY_DIREITA_PIN_A)) s_d |= 4;
  if (gpio_get_level(ROTARY_DIREITA_PIN_B)) s_d |= 8;
  switch (s_d) {
    case 0: case 5: case 10: case 15:
      break;
    case 1: case 7: case 8: case 14:
      contPulsosRodaDireita++; break;
    case 2: case 4: case 11: case 13:
      contPulsosRodaDireita--; break;
    case 3: case 12:
      contPulsosRodaDireita += 2; break;
    default:
      contPulsosRodaDireita -= 2; break;
  }
  estadoRodaDireita = (s_d >> 2);
  
  portEXIT_CRITICAL_ISR(&muxEncDireito);
}


void IRAM_ATTR encoder_esquerdo(void* arg) { 
  
  uint8_t s_e = estadoRodaEsquerda & 3;

  portENTER_CRITICAL_ISR(&muxEncEsquerdo);
  
  if (gpio_get_level(ROTARY_ESQUERDA_PIN_A)) s_e |= 4;
  if (gpio_get_level(ROTARY_ESQUERDA_PIN_B)) s_e |= 8;
  switch (s_e) {
    case 0: case 5: case 10: case 15:
      break;
    case 1: case 7: case 8: case 14:
      contPulsosRodaEsquerda++; break;
    case 2: case 4: case 11: case 13:
      contPulsosRodaEsquerda--; break;
    case 3: case 12:
      contPulsosRodaEsquerda += 2; break;
    default:
      contPulsosRodaEsquerda -= 2; break;
  }
  estadoRodaEsquerda = (s_e >> 2);
  
  portEXIT_CRITICAL_ISR(&muxEncEsquerdo); 
}

void computaOdometria(){
  // Variáveis locais
  int difPulsosRodaDireita=0;
  int difPulsosRodaEsquerda=0;
  double distPercorridaRodaDireita = 0.00;
  double distPercorridaRodaEsquerda = 0.00;
  double deltaTheta = 0.00;
  double deltaDist = 0.00;

  double velRodaDireita = 0.000;
  double velRodaEsquerda = 0.000;
  double velLinear = 0.00;
  double velAngular = 0.00;

  vTaskEnterCritical(&muxEncDireito);
  difPulsosRodaDireita = contPulsosRodaDireita - contPulsosRodaDireitaAnterior;
  contPulsosRodaDireitaAnterior = contPulsosRodaDireita;
  vTaskExitCritical(&muxEncDireito);

  vTaskEnterCritical(&muxEncEsquerdo);
  difPulsosRodaEsquerda = contPulsosRodaEsquerda - contPulsosRodaEsquerdaAnterior;
  contPulsosRodaEsquerdaAnterior = contPulsosRodaEsquerda;
  vTaskExitCritical(&muxEncEsquerdo);

  distPercorridaRodaDireita = dist_entre_pulsos_roda_dir_em_cm * difPulsosRodaDireita;
  distPercorridaRodaEsquerda = dist_entre_pulsos_roda_esq_em_cm * difPulsosRodaEsquerda;

  deltaTheta = (distPercorridaRodaDireita - distPercorridaRodaEsquerda)/distancia_entre_eixos;
  deltaDist = (distPercorridaRodaDireita + distPercorridaRodaEsquerda)/2;

  x = xAnterior + deltaDist*(cos(theta+(deltaTheta/2)));
  y = yAnterior + deltaDist*(sin(theta+(deltaTheta/2)));
  theta = thetaAnterior + deltaTheta;

  velLinear = (deltaDist/dtLoopOdom_s);                                                                       //converte cm to m
  velAngular = deltaTheta/dtLoopOdom_s;

  velRodaEsquerda = ((difPulsosRodaEsquerda*dist_entre_pulsos_roda_esq_em_cm)/dtLoopOdom_s);          //converte em cm to m
  velRodaDireita = ((difPulsosRodaDireita*dist_entre_pulsos_roda_dir_em_cm)/dtLoopOdom_s);            //converte em cm to m

  xAnterior = x;
  yAnterior = y;
  thetaAnterior = theta;

  odomDataToSend.poseX = x;
  odomDataToSend.poseY = y;
  odomDataToSend.poseTheta = theta;
  odomDataToSend.velLin = velLinear;
  odomDataToSend.velAng = velAngular;

  velRodasDataToSend.vRD = velRodaDireita;
  velRodasDataToSend.vRE = velRodaEsquerda;

  xQueueSend(queueVelRodas, &velRodasDataToSend, NULL);
}

void initGPIOEncoders(){
  gpio_config_t io_conf = {};

  //Configurações comuns para os dois encoders

  // Interrupção na borda de subida
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  // Modo setado como entrada
  io_conf.mode = GPIO_MODE_INPUT;
  // Habilita modo pullup
  io_conf.pull_up_en = false;
  io_conf.pull_down_en = false;

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

}


void odomTaskThread(){
  
  initGPIOEncoders();

  const portTickType delay = 20 / portTICK_RATE_MS;
  TickType_t tic = xTaskGetTickCount();
  while (1){
    computaOdometria();
    xQueueSend(queueOdom, &odomDataToSend, NULL);

    vTaskDelayUntil(&tic, delay);
  }
}