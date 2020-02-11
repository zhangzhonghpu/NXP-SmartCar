#include "motor.h"

#define RPWM_MAX   -500   //reverse max pwm 反向最大PWM,范围是[-1000,0]
#define FPWM_MAX  1000    //forward max pwm 正向最大PWM,范围是[0,1000]
#define   DIR    PTC0_IN  //DIR==1为反转，DIR==0为正转

void Motor_Init(void)
{
  /******电机******/
   tpm_pwm_init(TPM0, TPM_CH1,25*1000,0); //CH1为高时，正转
   tpm_pwm_init(TPM0, TPM_CH0,25*1000,0);
  /******编码器******/
   lptmr_pulse_init(LPT0_ALT2,0xFFFF, LPT_Rising);//初始化脉冲计数器，用LPT0_ALT2，即PTC5输入，每隔0xFFFF产生中断（需要开中断才能产生中断），上升沿触发
   gpio_init (PTC0, GPI,0);     //方向指示引脚
   port_init_NoALT(PTC0, PULLUP); 
}

void Run(int16 MPWM) //MPWM（motor PWM）为电机占空比
{
  /******正转******/
  if(MPWM > 0)
  {
    if(MPWM < FPWM_MAX)//未超出限定幅度
    {
      tpm_pwm_duty(TPM0,TPM_CH1,MPWM);
      tpm_pwm_duty(TPM0,TPM_CH0,0); 
    }
    else               //超出限定幅度
    {
     tpm_pwm_duty(TPM0,TPM_CH1,FPWM_MAX);
     tpm_pwm_duty(TPM0,TPM_CH0,0);
    }
  }
  /******反转******/
  else
  {
    if(MPWM > RPWM_MAX)//未超出限定幅度
    {
      tpm_pwm_duty(TPM0,TPM_CH1,0);
      tpm_pwm_duty(TPM0,TPM_CH0,-MPWM);  
    }
    else               //超出限定幅度
    {
     tpm_pwm_duty(TPM0,TPM_CH1,0);
     tpm_pwm_duty(TPM0,TPM_CH0,-RPWM_MAX);
    } 
  }
}

int8 GetSpeed() //返回值可根据需要改为int16
{
  int8 speed;
  if(!DIR)
  {
    speed = lptmr_pulse_get(); //保存脉冲计数器计算值
  }
  else
  {
    speed = -lptmr_pulse_get(); //保存脉冲计数器计算值
  }
  lptmr_pulse_clean();      //清空脉冲计数器计算值
  
  return speed;
}
