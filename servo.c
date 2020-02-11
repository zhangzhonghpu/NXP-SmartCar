#include "servo.h"

void Servo_Init(void)
{
  tpm_pwm_init(TPM1, TPM_CH1,333,SMID); 
}

void Turn(uint16 SPWM)//SPWM（servo pwm）
{
  if(SPWM<LMAX) //若超出左边最值
  {
    tpm_pwm_duty(TPM1,TPM_CH1,LMAX);
  }
  else if(SPWM>RMAX) //若超出右边最值
  {
    tpm_pwm_duty(TPM1,TPM_CH1,RMAX);
  }
  else //若舵机PWM正常
  {
    tpm_pwm_duty(TPM1,TPM_CH1,SPWM); 
  }
}
