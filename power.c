#include "power.h"
#define AD_CNT 10     //AD采集次数
#define CMAX   10000   //the maximum of Charge PWM
#define CMIN   0      //the minimum of Charge PWM

void Power_Init(void) 
{
  adc_init(ADC0_SE23); //ADC初始化
  //adc_init(ADC0_SE13);
  //adc_init(ADC0_SE12);
  //tpm_pwm_init(TPM2,TPM_CH0,10*1000,0);
}

uint8 Get(uint8 State) //获取电量
{
  uint8 vol = 0;
  uint8 result = 0;
  uint16 voltage = 0;
  uint8 i;
  switch(State)
  {
  case POWER0:
              for(i=0; i<AD_CNT;i++)
              {
                vol = adc_once(ADC0_SE23, ADC_8bit);
                voltage = voltage + vol; 
              } 
              result = (uint8)(voltage/AD_CNT);
              result = result*53/100+1;
              break;
  case CURRENT:
              for(i=0; i<AD_CNT;i++)
              {
                vol = adc_once(ADC0_SE13, ADC_8bit);
                voltage = voltage + vol; 
              } 
              result = (uint8)(voltage/AD_CNT);
              break;
  case VOLTAGE:
              for(i=0; i<AD_CNT;i++)
              {
                vol = adc_once(ADC0_SE12, ADC_8bit);
                voltage = voltage + vol; 
              } 
              result = (uint8)(voltage/AD_CNT);
              break;
  }
               
  return result; 
}

void Charge(uint16 CPWM)//CPWM（charge pwm）
{
  if(CPWM <= CMIN) 
  {
    tpm_pwm_duty(TPM2,TPM_CH0,CMIN);
  }
  else if(CPWM > CMAX) 
  {
    tpm_pwm_duty(TPM2,TPM_CH0,CMAX);
  }
  else 
  {
    tpm_pwm_duty(TPM2,TPM_CH0,CPWM); 
  }
}
