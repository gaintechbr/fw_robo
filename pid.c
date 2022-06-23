#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define PID_TIMER_INTERVAL_SEC   (0.02) 
double tempo_loop_PID_s = PID_TIMER_INTERVAL_SEC;
double tempo_loop_PID_us = 1000000 * PID_TIMER_INTERVAL_SEC;

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

void pidTaskThread(){
    // Variáveis PID RD
    float uRD = 0, SPRD = 0, vRD = 0, erroAntRD = 0, inteAntRD = 0, erroSatAntRD = 0;
    pidGains ganhoRD = {.Kc = 25, .Ti = 0.1, .Td = 0};

    // Variáveis PID RE
    float uRE = 0, SPRE = 0, vRE = 0, erroAntRE = 0, inteAntRE = 0, erroSatAntRE = 0;
    pidGains ganhoRE = {.Kc = 25, .Ti = 0.1, .Td = 0};

    
    float Ts = 0.02;
  
    const portTickType delay = (u_int32_t)(Ts * 1000) / portTICK_RATE_MS;
    TickType_t tic = xTaskGetTickCount();

    while (1){
        // Computa PIDs
        uRD = computaPID(SPRD, vRD, ganhoRD, &erroAntRD, &inteAntRD, &erroSatAntRD, Ts);
        uRE = computaPID(SPRE, vRE, ganhoRE, &erroAntRE, &inteAntRE, &erroSatAntRE, Ts);

        // TODO inserir rotinas de escrita do PWM

        // vTaskDelay(20/portTICK_RATE_MS);
        vTaskDelayUntil(&tic, delay);
    }
}