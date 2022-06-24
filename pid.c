#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define PID_TIMER_INTERVAL_SEC   (0.02)

#define PIN_PWM_RD_R 36
#define PIN_PWM_RD_L 36
#define PIN_EN_RD_R 0
#define PIN_EN_RD_L 0
#define CHANNEL_PWM_RD_R LEDC_CHANNEL_0
#define CHANNEL_PWM_RD_L LEDC_CHANNEL_1

#define PIN_PWM_RE_R 39
#define PIN_PWM_RE_L 39
#define PIN_EN_RE_R 0
#define PIN_EN_RE_L 0
#define CHANNEL_PWM_RE_R LEDC_CHANNEL_2
#define CHANNEL_PWM_RE_L LEDC_CHANNEL_3

#define PIN_MOTORES_EN_SELECT ((1ULL<<PIN_EN_RD_R) | (1ULL<<PIN_EN_RD_L) | (1ULL<<PIN_EN_RE_R) | (1ULL<<PIN_EN_RE_L) )

#define BANDA_MORTA_MOTORES 10

typedef struct pidGains
{
    float Kc;
    float Ti;
    float Td;
} pidGains;

int sign(int x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}


float computaPID(float SP, float PV, pidGains ganho, float* erroAnt, float* inteAnt, float* erroSatAnt, float Ts){
    float inte, u, erro, Tt, propDeri, uSat;
    
    erro = SP - PV;
    if (SP == 0){
        u = 0;
        inte = 0;
    }
    else{
        Tt = 0.5*ganho.Ti;

        inte = (*inteAnt) + (erro*Ts*ganho.Kc/ganho.Ti) + ((*erroSatAnt)*Ts/Tt);

        propDeri = ganho.Kc*(erro + ganho.Td*(erro - (*erroAnt))/Ts);
        if (propDeri > 100) propDeri = 100;
        
        u = propDeri + inte + 5*sign(propDeri + inte);
    }
    uSat = u;
    if (u > 100) uSat = 100;
    if (u < -100) uSat = -100;
        
    
    *erroSatAnt = uSat - u;
    *inteAnt = inte;
    *erroAnt = erro;

    return uSat;
}


void inicializaMotores(){
    // RODA DIREITA R
    ledc_channel_config_t pwmChannelRD_R = {  .gpio_num     = PIN_PWM_RD_R,
                                            .speed_mode     = LEDC_HIGH_SPEED_MODE,
                                            .channel        = CHANNEL_PWM_RD_R,
                                            .intr_type      = LEDC_INTR_DISABLE,
                                            .timer_sel      = LEDC_TIMER_0,
                                            .duty           = 0 
                                            };
    // RODA DIREITA L
    ledc_channel_config_t pwmChannelRD_L = {  .gpio_num     = PIN_PWM_RD_L,
                                            .speed_mode     = LEDC_HIGH_SPEED_MODE,
                                            .channel        = CHANNEL_PWM_RD_L,
                                            .intr_type      = LEDC_INTR_DISABLE,
                                            .timer_sel      = LEDC_TIMER_0,
                                            .duty           = 0 
                                            };
    // RODA ESQUERDA R
    ledc_channel_config_t pwmChannelRE_R = {  .gpio_num     = PIN_PWM_RE_R,
                                            .speed_mode     = LEDC_HIGH_SPEED_MODE,
                                            .channel        = CHANNEL_PWM_RE_R,
                                            .intr_type      = LEDC_INTR_DISABLE,
                                            .timer_sel      = LEDC_TIMER_0,
                                            .duty           = 0 
                                            };
    // RODA ESQUERDA L
    ledc_channel_config_t pwmChannelRE_L = {  .gpio_num     = PIN_PWM_RE_L,
                                            .speed_mode     = LEDC_HIGH_SPEED_MODE,
                                            .channel        = CHANNEL_PWM_RE_L,
                                            .intr_type      = LEDC_INTR_DISABLE,
                                            .timer_sel      = LEDC_TIMER_0,
                                            .duty           = 0 
                                            };
    // CONFIGURA PWMs
    ledc_channel_config(&pwmChannelRD_R);
    ledc_channel_config(&pwmChannelRD_L);
    ledc_channel_config(&pwmChannelRE_R);
    ledc_channel_config(&pwmChannelRE_L);

    ledc_timer_config_t timerConfig = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .bit_num = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 10000
    };

    ledc_timer_config(&timerConfig);

    // INICIALIZA GPIOs DE ENABLE MOTORES
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = PIN_MOTORES_EN_SELECT,
        .pull_down_en = 0,
        .pull_up_en = 0,

    };
    gpio_config(&io_conf);
}

void atuaMotores(float uRD, float uRE, bool habiltaMotores){
    if(!habiltaMotores){
        uRD = 0;
        uRE = 0;
        gpio_set_level(PIN_EN_RD_R, 0);
        gpio_set_level(PIN_EN_RD_L, 0);
        gpio_set_level(PIN_EN_RE_R, 0);
        gpio_set_level(PIN_EN_RE_L, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_L, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_R, 0);
    }
    else{
        if((uRD < BANDA_MORTA_MOTORES) & (uRD > (-BANDA_MORTA_MOTORES))){
            // DESLIGA MOTOR
            gpio_set_level(PIN_EN_RD_R, 0);
            gpio_set_level(PIN_EN_RD_L, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_L, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_R, 0);
        }
        else{
            uint32_t dutyCycleRD = abs(uRD * 4095 / 100);

            if(dutyCycleRD < 0)     dutyCycleRD = 0;
            if(dutyCycleRD > 4095)  dutyCycleRD = 4095;

            if(uRD > 0){
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_R, dutyCycleRD);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_L, 0);
                gpio_set_level(PIN_EN_RD_R, 1);
                gpio_set_level(PIN_EN_RD_L, 1);
            }
            else{
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_R, 0);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RD_L, dutyCycleRD);
                gpio_set_level(PIN_EN_RD_R, 1);
                gpio_set_level(PIN_EN_RD_L, 1);
            }
        }

        if((uRE < BANDA_MORTA_MOTORES) & (uRE > (-BANDA_MORTA_MOTORES))){
            // DESLIGA MOTOR
            gpio_set_level(PIN_EN_RE_R, 0);
            gpio_set_level(PIN_EN_RE_L, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RE_L, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RE_R, 0);
        }
        else{
            uint32_t dutyCycleRE = abs(uRE * 4095 / 100);
            
            if(dutyCycleRE < 0)     dutyCycleRE = 0;
            if(dutyCycleRE > 4095)  dutyCycleRE = 4095;

            if(uRE > 0){
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RE_R, dutyCycleRE);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RE_L, 0);
                gpio_set_level(PIN_EN_RE_R, 1);
                gpio_set_level(PIN_EN_RE_L, 1);
            }
            else{
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RE_R, 0);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL_PWM_RE_L, dutyCycleRE);
                gpio_set_level(PIN_EN_RE_R, 1);
                gpio_set_level(PIN_EN_RE_L, 1);
            }
        }
    }
}



void pidTaskThread(){
    // Variáveis PID RD
    float uRD = 0, SPRD = 0, vRD = 0, erroAntRD = 0, inteAntRD = 0, erroSatAntRD = 0;
    pidGains ganhoRD = {.Kc = 25, .Ti = 0.1, .Td = 0};

    // Variáveis PID RE
    float uRE = 0, SPRE = 0, vRE = 0, erroAntRE = 0, inteAntRE = 0, erroSatAntRE = 0;
    pidGains ganhoRE = {.Kc = 25, .Ti = 0.1, .Td = 0};

    float Ts = PID_TIMER_INTERVAL_SEC;

    // Inicializa PWMs
    inicializaMotores();
  
    const portTickType delay = (u_int32_t)(Ts * 1000) / portTICK_RATE_MS;
    TickType_t tic = xTaskGetTickCount();

    while (1){
        // Computa PIDs
        uRD = computaPID(SPRD, vRD, ganhoRD, &erroAntRD, &inteAntRD, &erroSatAntRD, Ts);
        uRE = computaPID(SPRE, vRE, ganhoRE, &erroAntRE, &inteAntRE, &erroSatAntRE, Ts);

        atuaMotores(uRD, uRE, true);

        // vTaskDelay(20/portTICK_RATE_MS);
        vTaskDelayUntil(&tic, delay);
    }
}