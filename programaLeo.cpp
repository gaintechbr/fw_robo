// KY-040 ...  ESP32
// CLK    ...  PIN 4
// DT     ...  PIN 2
// SW     ...  PIN 5
// +      ...  3.3V
// GND    ...  GND

#include <Arduino.h>
#include <math.h>

#define ROTARY_DIREITA_PIN_A 2
#define ROTARY_DIREITA_PIN_B 4 

#define ROTARY_ESQUERDA_PIN_A 18
#define ROTARY_ESQUERDA_PIN_B 19

double converte_tempo_us_to_s(int tempo_base_us){
double tempo_out;
tempo_out = (tempo_base_us)*1.0/1000000.0;
return tempo_out;  
}

//################################## Variaveis do PWM #############################################
const int ledPin = 16;                                                              // Corresponde ao GPIO16
uint16_t dutyCycle;
// DEfinindo as Configurações do PWM
const int freq = 15000;                                                             // Definindo a frequência de PWM em 15KHz
const int ledChannel = 0;                                                           
const int resolution = 12;

//################################## Variaveis loop do PID #######################################
int tempo_loop_PID_us = 50000;
double tempo_loop_PID_s = converte_tempo_us_to_s(tempo_loop_PID_us);

double Kc_v = 0.48;
double Ti_v_s = 0.55;
double Td_v_s = 0.0;
//entradas
double SP_v = 0.5;
double erro_v;
double integral_v;
double ck_v;
double es_v = 0.0;
double Tt_v;
double integral_v_a = 0.0;
double erro_v_a = 0.0;
//saídas
double u_v;
double uf_v;
double up_v;
double up_v_sat;
double uf_v_a = 0.0;

double Kc_w = 45.0;
double Ti_w_s = 0.9909;
double Td_w_s = 0.0;
//entradas
double SP_w = 0.8;
double erro_w;
double integral_w;
double ck_w;
double es_w = 0.0;
double Tt_w;
double integral_w_a = 0.0;
double erro_w_a = 0.0;
//saídas
double u_w;
double uf_w;
double up_w;
double up_w_sat;
double uf_w_a = 0.0;

//################################## Variaveis Leitura Encoder Direita ############################

long int cont_roda_direita=0;
uint8_t estado_roda_direita=0;

long int contPulsosRodaDireitaAnterior=0;
int difPulsosRodaDireita=0;

//################################## Variaveis Leitura Encoder Esquerda ############################

long int cont_roda_esquerda=0;
uint8_t estado_roda_esquerda=0;

long int contPulsosRodaEsquerdaAnterior=0;
int difPulsosRodaEsquerda=0;

//################################## Variaveis da Odometria #######################################

int dtLoopOdom_us = 10000;                                                                     //base em micro segundos
double dtLoopOdom_s = converte_tempo_us_to_s(dtLoopOdom_us);
u_int ppr_direita = 8977;
u_int ppr_esquerda = 9083;

double distancia_entre_eixos = 50; 
double raio_da_roda_direita = 13;
double raio_da_roda_esquerda = 13;                                                                      //está em cm
double dist_entre_pulsos_roda_dir_em_cm = 2*PI*raio_da_roda_direita/(ppr_direita)*1.0;
double dist_entre_pulsos_roda_esq_em_cm = 2*PI*raio_da_roda_esquerda/(ppr_esquerda)*1.0;

double delta_s_roda_direita = 0.00;
double delta_s_roda_esquerda = 0.00;
double delta_teta = 0.00;
double delta_s = 0.00;

double x;
double y;
double teta;
double teta_aux;
double xAnterior = 0.00;
double yAnterior = 0.00;
double thetaAnterior = 0.00;

double velocidade_roda_direita = 0.000;
double velocidade_roda_esquerda = 0.000;
double velocidade_linear = 0.00;
double velocidade_angular = 0.00;

hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;

portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

int sign(int x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}


void stopTimer0() {
    timerEnd(timer0);
    timer0 = NULL; 
}

void stopTimer1() {
    timerEnd(timer1);
    timer1 = NULL; 
}

void cb_timer0(){
 
  difPulsosRodaDireita = cont_roda_direita - contPulsosRodaDireitaAnterior;
  difPulsosRodaEsquerda = cont_roda_esquerda - contPulsosRodaEsquerdaAnterior;

  delta_s_roda_direita = dist_entre_pulsos_roda_dir_em_cm*difPulsosRodaDireita;
  delta_s_roda_esquerda = dist_entre_pulsos_roda_esq_em_cm*difPulsosRodaEsquerda;

  delta_teta = (delta_s_roda_direita - delta_s_roda_esquerda)/distancia_entre_eixos;
  delta_s = (delta_s_roda_direita + delta_s_roda_esquerda)/2;

  x = xAnterior + delta_s*(cos(teta+(delta_teta/2)));
  y = yAnterior + delta_s*(sin(teta+(delta_teta/2)));
  teta = thetaAnterior + delta_teta;

  velocidade_linear = (delta_s/dtLoopOdom_s)/100;                                                                       //converte cm to m
  velocidade_angular = delta_teta/dtLoopOdom_s;

  velocidade_roda_esquerda = ((difPulsosRodaEsquerda*dist_entre_pulsos_roda_esq_em_cm)/dtLoopOdom_s)/100;          //converte em cm to m
  velocidade_roda_direita = ((difPulsosRodaDireita*dist_entre_pulsos_roda_dir_em_cm)/dtLoopOdom_s)/100;            //converte em cm to m



  contPulsosRodaDireitaAnterior = cont_roda_direita;
  contPulsosRodaEsquerdaAnterior = cont_roda_esquerda;
  xAnterior = x;
  yAnterior = y;
  thetaAnterior = teta;
}

void cb_timer1() {

if(SP_v != 0){
erro_v = SP_v-velocidade_linear;
Tt_v = 0.5*Ti_v_s;
integral_v = integral_v_a + ((erro_v*tempo_loop_PID_s*Kc_v)/Ti_v_s)+(es_v*tempo_loop_PID_s)/Tt_v;
ck_v = Kc_v*(erro_v+Td_v_s*(erro_v-erro_v_a)/tempo_loop_PID_s);
if (ck_v > 100){
ck_v = 100;
}
u_v = ck_v + integral_v;
uf_v = 0.2212*u_v+0.7788*uf_v_a;
up_v = uf_v + 15*sign(uf_v);
} else {
up_v = 0.0;
integral_v = 0; }

if(up_v <= 100 && up_v >=-100){
up_v_sat = up_v;
} else if(up_v > 100){
up_v_sat = 100;
} else if (up_v < -100){
up_v_sat = -100;  
}

if(SP_w != 0){
erro_w = SP_w-velocidade_angular;
Tt_w = 0.5*Ti_w_s;
integral_w = integral_w_a + ((erro_w*tempo_loop_PID_s*Kc_w)/Ti_w_s)+(es_w*tempo_loop_PID_s)/Tt_w;
ck_w = Kc_w*(erro_w+Td_w_s*(erro_w-erro_w_a)/tempo_loop_PID_s);
if (ck_w > 100){
ck_w = 100;
}
u_w = ck_w + integral_w;
uf_w = 0.2212*u_w+0.7788*uf_w_a;
up_w = uf_w + 19*sign(uf_w);
} else {
up_w = 0.0;
integral_w = 0; }

if(up_w <= 100 && up_w >=-100){
up_w_sat = up_w;
} else if(up_w > 100){
up_w_sat = 100;
} else if (up_w < -100){
up_w_sat = -100;  
}

es_v = up_v_sat-up_v;
erro_v_a = erro_v;
integral_v_a = integral_v;
uf_v_a = uf_v;

es_w = up_w_sat-up_w;
erro_w_a = erro_w;
integral_w_a = integral_w;
uf_w_a = uf_w;

}

void startTimer0(){
    //inicialização do timer. Parametros:
    /* 0 - seleção do timer a ser usado, de 0 a 3.
      80 - prescaler. O clock principal do ESP32 é 80MHz. Dividimos por 80 para ter 1us por tick.
    true - true para contador progressivo, false para regressivo
    */
    timer0 = timerBegin(0, 80, true);

    /*conecta à interrupção do timer
     - timer é a instância do hw_timer
     - endereço da função a ser chamada pelo timer
     - edge=true gera uma interrupção
    */
    timerAttachInterrupt(timer0, &cb_timer0, true);

    /* - o timer instanciado no inicio
       - o valor em us para 1s
       - auto-reload. true para repetir o alarme
    */
    timerAlarmWrite(timer0, dtLoopOdom_us, true); 

    //ativa o alarme
    timerAlarmEnable(timer0);
}

void startTimer1(){
    //inicialização do timer. Parametros:
    /* 0 - seleção do timer a ser usado, de 0 a 3.
      80 - prescaler. O clock principal do ESP32 é 80MHz. Dividimos por 80 para ter 1us por tick.
    true - true para contador progressivo, false para regressivo
    */
    timer1 = timerBegin(1, 80, true);

    /*conecta à interrupção do timer
     - timer é a instância do hw_timer
     - endereço da função a ser chamada pelo timer
     - edge=true gera uma interrupção
    */
    timerAttachInterrupt(timer1, &cb_timer1, true);

    /* - o timer instanciado no inicio
       - o valor em us para 1s
       - auto-reload. true para repetir o alarme
    */
    timerAlarmWrite(timer1, tempo_loop_PID_us, true); 

    //ativa o alarme
    timerAlarmEnable(timer1);
}

void IRAM_ATTR encoder_direita() {
  
  uint8_t s_d = estado_roda_direita & 3;

  portENTER_CRITICAL_ISR(&timerMux0);
  taskEte
    
    if (digitalRead(ROTARY_DIREITA_PIN_A)) s_d |= 4;
    if (digitalRead(ROTARY_DIREITA_PIN_B)) s_d |= 8;
    switch (s_d) {
      case 0: case 5: case 10: case 15:
        break;
      case 1: case 7: case 8: case 14:
        cont_roda_direita++; break;
      case 2: case 4: case 11: case 13:
        cont_roda_direita--; break;
      case 3: case 12:
        cont_roda_direita += 2; break;
      default:
        cont_roda_direita -= 2; break;
    }
    estado_roda_direita = (s_d >> 2);
  
  portEXIT_CRITICAL_ISR(&timerMux0);
 
}

void IRAM_ATTR encoder_esquerda() { 
  
  uint8_t s_e = estado_roda_esquerda & 3;

  portENTER_CRITICAL_ISR(&timerMux1);
    
    if (digitalRead(ROTARY_ESQUERDA_PIN_A)) s_e |= 4;
    if (digitalRead(ROTARY_ESQUERDA_PIN_B)) s_e |= 8;
    switch (s_e) {
      case 0: case 5: case 10: case 15:
        break;
      case 1: case 7: case 8: case 14:
        cont_roda_esquerda++; break;
      case 2: case 4: case 11: case 13:
        cont_roda_esquerda--; break;
      case 3: case 12:
        cont_roda_esquerda += 2; break;
      default:
        cont_roda_esquerda -= 2; break;
    }
    estado_roda_esquerda = (s_e >> 2);
  
  portEXIT_CRITICAL_ISR(&timerMux1); 
 
}

void setup(){

  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  pinMode(ROTARY_DIREITA_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_DIREITA_PIN_B, INPUT_PULLUP);

  pinMode(ROTARY_ESQUERDA_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_ESQUERDA_PIN_B, INPUT_PULLUP);

  attachInterrupt(ROTARY_DIREITA_PIN_A, encoder_direita, CHANGE);
  attachInterrupt(ROTARY_DIREITA_PIN_B, encoder_direita, CHANGE);
  
  attachInterrupt(ROTARY_ESQUERDA_PIN_A, encoder_esquerda, CHANGE);
  attachInterrupt(ROTARY_ESQUERDA_PIN_B, encoder_esquerda, CHANGE);

  Serial.begin(115200);
  startTimer0();
  startTimer1();
}

void loop(){
  
/* Serial.println("");
 Serial.print(" Encoder roda direita: ");
 Serial.print(cont_roda_direita);
 Serial.print("    ");
 Serial.print(" Encoder roda esquerda: ");
 Serial.print(cont_roda_esquerda);
 Serial.print("    ");
 delay(100); */ 

//dutyCycle = analogRead(A0); 
//Serial.print(dutyCycle);
// changing the LED brightness with PWM
//ledcWrite(ledChannel, dutyCycle);
delay(100);

}