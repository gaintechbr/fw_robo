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