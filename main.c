/* @file       单右环（三档速度）
 * @brief      Smartcar
 * @author     Brunin
 * @date       2018-07-15
 */
/*********************************区赛必胜！*********************************/
#include "common.h"
#include "include.h"     

#define STOPTIME 30000            //发车后延迟 STOPTIME  ms 开始检测干簧管
#define POWERSET 85               //电压设定值
#define ADTIME   300              //MS

/******调试选项******/
#define STOP                      //干簧管停车
//#define ShowImage               //上位机显示图像，分为RUN和DEBUG模式
//void vcan_sendimg(void *imgaddr, uint32_t imgsize);
//#define ShowWave                //上位机显示波形
//void vcan_sendware(void *wareaddr, uint32_t waresize);
/********FLAG********/
uint8  runflag = 0;               //充电完成，起步标志位
uint8  stopflag = 0;              //开始检测起跑线标志位
uint8  servoflag = 0;             //舵机状态标志位，为1时，舵机执行
uint16 Timecnt = 0;               //定时器计数，每ms加1  
uint8  protectflag = 0;           //丢线标志位  
uint8  pipeflag = 0;              //干簧管标志位
uint8  nocirflag = 0;             //不入圆环
uint8  admaxflag = 0;             //AD采集标志位
uint16 adtimecnt = 0;             //开始AD采集计时
/******外部变量******/
extern PID SpeedPID;         
extern PID AnglePID;
extern uint8  countflag,cirtime;       //圆环相关
/******摄像头相关******/
uint8 imgbuff[CAMERA_SIZE];            //定义存储接收图像的数组
uint8 img[CAMERA_H][CAMERA_W];  //60*80//由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
uint8 threshold = 87;                 //阈值
void PORTA_IRQHandler();               //假如需要二维数组，只需要改成 uint8 img[CAMERA_H][CAMERA_W];
void DMA0_IRQHandler();                //imgbuff是采集的缓冲区，img是解压后的缓冲区。imgbuff用于采集图像，img用于图像处理.   
/******系统初始化******/
void System_Init(void);
/******速度设定******/
uint8 speed,dir;



/****************PIT0中断服务函数,定时时间：1ms****************/
void PIT0_IRQHandler(void)
{
  int16  mpwm = 0;
  uint16 spwm = 0; 
  if(PIT_TFLG(PIT0) == 1 )    //判断是否 PIT0 进入中断
  { 
    PIT_Flag_Clear(PIT0);     //清中断标志位
    Timecnt++;
   
/*********PERIOD：1MS*********/ 
      if(servoflag)
      {
        spwm = SMID + Dir_PD(&AnglePID); 
        Turn(spwm);
        servoflag = 0;
      }
      
      if(admaxflag == 0)     //如果未采集AD
      {
        adtimecnt++;
        if(adtimecnt == ADTIME)  //发车后ADTIME，开始采集最大值
        {
          admax_collect();       //AD最大值采集
          admaxflag = 1;         //只会置1一次，不清零
          adtimecnt = 0;
        }
      }
    
/*********PERIOD：20MS*********/
    if(Timecnt%20 == 0)      
    {
#ifdef STOP   
     if(pipeflag==1 || protectflag==1)
     {
        led (LED0,LED_ON);
        led (LED1,LED_ON); 
        led (LED2,LED_ON); 
        led (LED3,LED_ON);    
        SpeedPID.SetValue = -1;  
     }
#endif
      Speed_PI(&SpeedPID);  
    }      
    
/*********PERIOD：1MS*********/  
      mpwm = Run_Out(&SpeedPID);
      Run(mpwm); 
      
/*********PERIOD：50MS*********/ 
    if(Timecnt%50 == 0)      
    {
      if(countflag == 1)       //入环计时标志位
      {
          cirtime++;
      }  
    }      
               
/*********PERIOD：STOPTIME_MS*********/
      if(runflag) 
      {
        if(Timecnt%STOPTIME == 0) 
        {
          stopflag = 1;
           Timecnt = 0;               //这句应该放在最长执行周期内
        }   
      }
      else if(Timecnt == STOPTIME)
        Timecnt = 0;                  //防止溢出出现未知错误
  }
}   
 
/**************************MAIN函数**************************/
void main(void)
{
#ifdef ShowWave
    int16 var[4];                                  //上位机波形缓存
#endif
    uint8 power = 0; 
    
/************************起步前参数设定及电量检测******************/
    DELAY_MS(2000);
    Power_Init();
    led_init (LED_MAX);
    led (LED3,LED_ON);
    
    while(!runflag)         //此flag建议写入flash                         
    {
      power = Get(POWER0);
      if(power > POWERSET)
      {
        runflag = 1;
        led (LED3,LED_OFF);
      }
    }
    
/************************参数配置完成&充电完成！************************/  
    System_Init();                                       //系统初始化必须放在充电完成之后
                                                         //否则充电时电压不稳，将卡死在某个初始化      
  //配置PORTA和DMA中断服务函数
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);   //设置PORTA的中断服务函数为 PORTA_IRQHandler
  set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);     //设置DMA0的中断服务函数为 DMA0_IRQHandler
  //配置定时器PIT0中断函数
  pit_init_ms(PIT0, 1);                             //初始化 PIT0，定时时间为： 1ms
  set_vector_handler(PIT_VECTORn ,PIT0_IRQHandler); // 设置中断服务函数到中断向量表里
  enable_irq(PIT_IRQn);                             // 使能PIT中断
   
  while(runflag)
  {     
     if(!servoflag)//18ms
     {     
        camera_get_img();
        img_extract(img, (uint8 *)imgbuff,CAMERA_SIZE); //解压到二维数组img[CAMERA_H][CAMERA_W]
        Image_Deal();
        servoflag = 1;                                  //图像处理完成，舵机才执行
     }
     
#ifdef STOP 
     if(stopflag)
     {
        if(!PIPE)
        {
          pipeflag = 1;
          led (LED0,LED_ON);
          led (LED1,LED_ON); 
          led (LED2,LED_ON); 
          led (LED3,LED_ON);    
          SpeedPID.SetValue = -1;  
        }
     }
#endif
     
/******上位机显示图象******/   
#ifdef ShowImage      
  #if 1   //Run模式下使用 
    vcan_sendimg(imgbuff, CAMERA_SIZE);             //imgbuff为原始二值化图象      
  #elif 0 //Debug模式下使用,在image.c中开启显示中线
    vcan_sendimg(img, CAMERA_W * CAMERA_H);         //解压后变为灰度格式的图象  
  #endif  
#endif

/******上位机显示波形******/     
#ifdef ShowWave //改变波形显示个数时，要记得更改上面定义的数组
      var[0] = AnglePID.SetValue;
      var[1] = AnglePID.ActualValue;
      var[2] = AnglePID.PWM_NOW;
      var[3] = AnglePID.ErrorNow;
      vcan_sendware(var, sizeof(var)); 
      DELAY_MS(10);
#endif
  }
}



/******************系统初始化选择******************/
void System_Init(void) 
{
  Dip_Init();
  
  if(DIP1==1 && DIP2==1 && DIP3==1 && DIP4==1) //默认状态，run模式 1111
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init(); 
    dir = 31;
    speed = 42;
    SpeedPID.SetValue = speed;  
  }
  else if(DIP1==1 && DIP2==1 && DIP3==1 && DIP4==0) //camera模式  1110
  {
    Site_t V = {0, 0}; 
    
    Key_Init();
    OLED_Init(); 
    camera_init(imgbuff);  
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置PORTA的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置DMA0的中断服务函数为 DMA0_IRQHandler
    
    while(1)
    {
        camera_get_img();
        img_extract(img, (uint8 *)imgbuff,CAMERA_SIZE); //解压到二维数组img[CAMERA_H][CAMERA_W]
        Image_Deal();     
       /******OLED显示******/
        V.x = 0;  V.y = 7;    //设定光标起始位置（左上角）
        OLED_PrintImage((uint8 *)img,CAMERA_H,CAMERA_W); 
        if(!KEYL)
         threshold--;
        else if(!KEYR)
         threshold++;
        OLED_BL(V.x,V.y, threshold);
        SCCB_WriteByte(OV7725_CNST ,threshold);
    }
  }
  else if(DIP1==1 && DIP2==1 && DIP3==0 && DIP4==1) //ADC模式  1101
  {
    uint8 left0,right0;
    uint16 left1,right1;
    uint16 sum;
    Site_t V = {0, 0}; 
    OLED_Init(); 
    ADC_Init();
    
    admax_collect();                                     //AD最大值采集

    while(1)
    {
        left0  =  adc_collect(ADC0_SE8,adc_L);   //绝对值
        right0 =  adc_collect(ADC0_SE9,adc_R);
        left1  =  ADC_Collect(ADC0_SE8,adc_L);   //相对值
        right1 =  ADC_Collect(ADC0_SE9,adc_R);
        
        sum =  left1 + right1;
       
       /******OLED显示******/
        V.x = 0;  V.y = 0; 
        OLED_BL(V.x,V.y,left0);
        V.x = 30;
        OLED_BL(V.x,V.y,right0);
        V.x = 0;  V.y = 2;
        OLED_BL(V.x,V.y,left1); 
        V.x = 30;
        OLED_BL(V.x,V.y,right1);
        V.x = 60;
        OLED_BL(V.x,V.y,sum);
    }
  }
  else if(DIP1==0 && DIP2==0 && DIP3==0 && DIP4==1) //run模式 0001   speed = 41
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init();
    dir = 31;
    speed = 41;
    SpeedPID.SetValue = speed;  
  }
  else if(DIP1==0 && DIP2==0 && DIP3==0 && DIP4==0) //run模式 0000   speed = 40
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init(); 
    dir = 31;
    speed = 40;
    SpeedPID.SetValue = speed;  
  }
  else if(DIP1==0 && DIP2==0 && DIP3==1 && DIP4==0) //弃赛模式 0010  不入圆环
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init(); 
    dir = 31;
    nocirflag = 1;
    speed = 42;
    SpeedPID.SetValue = speed; 
  }
  else if(DIP1==0 && DIP2==0 && DIP3==1 && DIP4==1) //默认状态，run模式 舵机加硬1
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init();
    dir = 32;
    speed = 42;
    SpeedPID.SetValue = speed;  
  }
  else if(DIP1==0 && DIP2==1 && DIP3==0 && DIP4==0) //默认状态，run模式 舵机变软2
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init();
    dir = 29;
    speed = 42;
    SpeedPID.SetValue = speed;  
  }
  else if(DIP1==1 && DIP2==1 && DIP3==0 && DIP4==0) //默认状态，run模式 舵机加硬2
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init();
    dir = 33;
    speed = 42;
    SpeedPID.SetValue = speed;  
  }
  else if(DIP1==0 && DIP2==1 && DIP3==0 && DIP4==1) //默认状态，run模式 舵机变软3
  {
    Motor_Init();
    Servo_Init();
    ADC_Init();
    Pipe_Init();
    camera_init(imgbuff);  
    //Beep_Init();
    dir = 28;
    speed = 42;
    SpeedPID.SetValue = speed;  
  }
}



/***************************************************************************************************/
/*!
 *  @brief      PORTA中断服务函数
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 6;                                              //场中断
    if(flag & (1 << n))                                 //PTA6触发中断
    {
        camera_vsync();  //鹰眼ov7725场中断服务函数
    }
}
/*!
 *  @brief      DMA0中断服务函数
 */
void DMA0_IRQHandler()
{
    camera_dma();  //鹰眼ov7725 DMA中断服务函数
}

/*!
 *  @brief      山外多功能调试助手上位机，摄像头显示函数
 *  @param      imgaddr    图像起始地址
 *  @param      imgsize    图像占用空间的大小
 *  Sample usage:
             具体用法参考这帖子:
            【山外摄像头】鹰眼上位机例程和微焦效果 - 智能车资料区
             http://vcan123.com/forum.php?mod=viewthread&tid=6242&ctid=27
 */
void vcan_sendimg(void *imgaddr, uint32_t imgsize)
{
#define CMD_IMG     1
    uint8_t cmdf[2] = {CMD_IMG, ~CMD_IMG};    //山外上位机 使用的命令
    uint8_t cmdr[2] = {~CMD_IMG, CMD_IMG};    //山外上位机 使用的命令

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送命令
    uart_putbuff(VCAN_PORT, (uint8_t *)imgaddr, imgsize); //再发送图像
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //先发送命令
}
/*!
 *  @brief      山外多功能调试助手上位机，虚拟示波器显示函数
 *  @param      wareaddr    波形数组起始地址
 *  @param      waresize    波形数组占用空间的大小
 *  Sample usage:
             具体用法参考这帖子:
            【山外资料】陀螺仪和加速度 上位机显示例程 - 智能车资料区
             http://vcan123.com/forum.php?mod=viewthread&tid=6253&ctid=27
 */
void vcan_sendware(void *wareaddr, uint32_t waresize)
{
#define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送前命令
    uart_putbuff(VCAN_PORT, (uint8_t *)wareaddr, waresize);    //发送数据
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //发送后命令
}
