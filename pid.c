#include "pid.h"

//#define PROTECT                     //丢线保护
#define CIRTIME 240                 //在 CIRTIME*50ms 内不检测圆环，200即10秒

#define Tspeed  20                  //控制周期：20ms
extern  uint16 Timecnt;             //计数值，每毫秒加一
extern  uint8  protectflag;
extern  uint8  speed,dir;
extern  uint8  nocirflag;             //不入圆环
extern  uint8  admaxflag;             //AD最大值采集完成标志位
uint8 cirin,cirout,cirR,cirinside,countflag,cirtime;       //圆环相关

PID SpeedPID =       
{
  .SetValue = 0,                    //最大42
  .ActualValue = 0,   
  .PWM_NOW = 0,      
  .PWM_LAST = 0,      
  .Kp = 0,
  .Ki = 10,
  .Kd = 0,
  .ErrorNow = 0,        
  .ErrorLast = 0  
};

PID AnglePID =
{
  .SetValue = MIDPOINT,     
  .ActualValue = MIDPOINT,   
  .PWM_NOW = 0,      
  .PWM_LAST = 0,      
  .Kp = 20,         
  .Ki = 0,
  .Kd = 20,
  .ErrorNow = 0,        
  .ErrorLast = 0    
};


/******速度控制(20ms)：增量式PI******/
void Speed_PI(PID *V)         
{
  if(V->SetValue == 42)
    V->PWM_NOW = 1000;
  else if(V->SetValue == 41)
    V->PWM_NOW = 800;
  else if(V->SetValue == 40)
    V->PWM_NOW = 700;
  else if (V->SetValue == -1)
    V->PWM_NOW = -100;
  else if (V->SetValue == 0)
    V->PWM_NOW = 100;
  else if (V->SetValue == 5)
    V->PWM_NOW = 500;
  
  V->PWM_LAST = V->PWM_NOW;
}

/******PWM计算结果输出******/
int16 Run_Out(PID *V) //motor pwm 输出，1ms
{
  int16 PWM_Error;
  static int16 PWM_OUT = 0;
  PWM_Error =  V->PWM_NOW - V->PWM_LAST;
  PWM_OUT = V->PWM_LAST + PWM_Error*(Timecnt%Tspeed+1)/Tspeed;
  return  PWM_OUT;
}

/******转向控制：位置式PD******/
uint16 Dir_PD(PID *A)
{
  uint16 left = 0;
  uint16 right = 0;
  
  if(admaxflag == 1)
  {
    left = ADC_Collect(ADC0_SE8,adc_L);
    right =  ADC_Collect(ADC0_SE9,adc_R);
  }
  
/******************圆环检测******************/
  if((left+right)>185 && cirin==0 && nocirflag==0 && admaxflag==1)     //到达入环口
  {
    cirin = 1;
    cirR = 1;                          //判断为右圆环
    
    if(countflag == 0)                 //如果在圆环计时时间内，触发圆环标志位，则视为误触发
      SpeedPID.SetValue = 0;
    
    led (LED0,LED_ON);
  }
  
  if(cirin==1 && (left+right)<100)     //已经进入环内，或未入环
  {
    cirinside = 1;
    SpeedPID.SetValue = speed;
   // SpeedPID.SetValue = 5;                            /////////////////////////////////////////////////////////////////////////////////////

    led (LED1,LED_ON);
  }

  if(cirinside==1 && (left+right)>150) //到达出环口
  {
    cirout = 1;
    led (LED2,LED_ON);
  }
  
  if(cirin==1 && cirout==1 && (left+right)<100)  //已经出环
  {
    cirR = 0; 
    cirin = 0;
    cirout = 0;
    cirinside = 0;
    countflag = 1;
    
   //SpeedPID.SetValue = speed;                                         //////////////////////////////////////////////////////////////////
   
    led (LED0,LED_OFF);
    led (LED1,LED_OFF); 
    led (LED2,LED_OFF); 
  } 

  if(cirtime>0 && cirtime<CIRTIME)
  {
    if(cirin == 1)
      cirin = 0;
    if(cirR == 1)
      cirR = 0;
    if(cirout == 1)
      cirout = 0;
    if(cirinside == 1)
      cirinside = 0;
    
    led (LED0,LED_OFF);
    led (LED1,LED_OFF);
    led (LED2,LED_OFF);
  }   
  else if(cirtime == CIRTIME)
  {
     cirtime = 0;
     countflag = 0; 
  }
  
/******************越界保护******************/ 
#ifdef PROTECT
  if(((left+right)<10) && admaxflag==1)
  {
     protectflag = 1;
  }
#endif
  
/******************舵机执行******************/
  A->ErrorNow = A->ActualValue - A->SetValue;
  
  if(cirR==1 && A->ErrorNow<0)
  {
    A->ErrorNow = -A->ErrorNow;
  }
  
  if(!cirin)
  {
    if(A->ErrorNow > 0)
    {
      A->Kp = A->ErrorNow*A->ErrorNow*39/5000 - A->ErrorNow/2 + dir;
    }
    else if(A->ErrorNow < 0)
    {
      A->Kp = A->ErrorNow*A->ErrorNow*39/5000 + A->ErrorNow/2 + dir;
    }
    else if(A->ErrorNow == 0)  
    {
      A->Kp = 0;
      A->Kd = 0;
    }
  }
  else//圆环内
  {
    if(speed == 42)
    {
      if(A->ErrorNow > 0)
      {
        A->Kp = A->ErrorNow*A->ErrorNow*39/5000 - A->ErrorNow/2 + dir - 4;
      }
      else if(A->ErrorNow < 0)
      {
        A->Kp = A->ErrorNow*A->ErrorNow*39/5000 + A->ErrorNow/2 + dir - 4;
      }
      else if(A->ErrorNow == 0)  
      {
        A->Kp = 0;
        A->Kd = 0;
      }
    }
    else if(speed == 41)
    {
      if(A->ErrorNow > 0)
      {
        A->Kp = A->ErrorNow*A->ErrorNow*39/5000 - A->ErrorNow/2 + dir - 5;
      }
      else if(A->ErrorNow < 0)
      {
        A->Kp = A->ErrorNow*A->ErrorNow*39/5000 + A->ErrorNow/2 + dir - 5;
      }
      else if(A->ErrorNow == 0)  
      {
        A->Kp = 0;
        A->Kd = 0;
      }
    }
    else if(speed == 40)
    {
      if(A->ErrorNow > 0)
      {
        A->Kp = A->ErrorNow*A->ErrorNow*39/5000 - A->ErrorNow/2 + dir - 6;
      }
      else if(A->ErrorNow < 0)
      {
        A->Kp = A->ErrorNow*A->ErrorNow*39/5000 + A->ErrorNow/2 + dir - 6;
      }
      else if(A->ErrorNow == 0)  
      {
        A->Kp = 0;
        A->Kd = 0;
      }
    }
  }
 
  A->PWM_NOW  = A->Kp*A->ErrorNow + A->Kd*(A->ErrorNow - A->ErrorLast);
  A->ErrorLast = A->ErrorNow;
  return A->PWM_NOW;
}
