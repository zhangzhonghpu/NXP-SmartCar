#include "image.h"
#include "key.h"

#define ShowMidLine             //上位机显示中线
#define ZEBRA                  //斑马线

#define black 0                 //大写的BLACK和WHITE为LCD显示颜色的宏
#define white 255
#define START 29                //最近开始行
#define END   2                 //最远结束行
#define BASE  31                //基准行,31-29

extern PID SpeedPID;
extern PID AnglePID;
extern uint8 stopflag;          //开始检测起跑线标志位
extern uint8 pipeflag;          //干簧管标志位
extern uint8 cirin,cirout,cirR,cirinside;     //圆环相关

void  ImageScan(void);
void  GetMid(void); 
void  GetBaseLine(void);

int8  MidResult[2] = {40,40};    //上一次计算结果
int8  LeftLine[32];              //左边界线
uint8 RightLine[32];             //右边界线
int8  MidLine[32];               //中线
uint8 Width[32];
uint8 L_findflag[32];
uint8 R_findflag[32];
uint8 LAST;                      //终止行
Site_t MidLineSite = {0,0};     //LCD中线坐标
Site_t LeftLineSite = {0,0}; 
Site_t RightLineSite = {0,0}; 
/***图像处理函数***/
void Image_Deal()
{    
  GetBaseLine();
  ImageScan();
  GetMid();
  
 /******ZEBRA&PROTECT******/
#ifdef ZEBRA            
  if(stopflag == 1 && cirin == 0)
  {
    if(Width[29] >3 && Width[29] < 20)
    {
        SpeedPID.SetValue = -1;
        pipeflag = 1;
    }
  }
#endif
}

/******边线扫描-【black=0，white=255】-左上角为(0,0)******/ 
void ImageScan(void)
{
  int8  i,j; 
 
  for(i=START;i>=END;i--)          //从下往上扫，从近往远扫
  {
    if(img[i][MidLine[i+1]]==black)
    {
        LAST = i + 1;
        break;
    }
    
    L_findflag[i] = 0;
    R_findflag[i] = 0;
    
    for(j=MidLine[i+1]; j>=0; j--)  //从上一次中点向左扫
    {
      if(img[i][j] == black)        //若第i行j列为边线
      {
        L_findflag[i] = 1;
        LeftLine[i] = j;            //则将第i行的左边线位置j存放在LeftLine[i]
        break;
      }
    }   
    
    for(j=MidLine[i+1]; j<=79; j++) //从上一次中点向右扫
    {
      if(img[i][j] == black)
      {
        R_findflag[i] = 1;
        RightLine[i] = j;
        break;
      }      
    }  
    
    if(L_findflag[i]==0 && R_findflag[i]==0)
    {   
      if(cirR == 0)                                             ////////////////////////////////////////////////
      {
          LeftLine[i] = 2 * LeftLine[i+1] - LeftLine[i+2];
          RightLine[i] = 2 * RightLine[i+1] - RightLine[i+2]; 
      }
      else if(cirR == 1 && cirinside==0)
       {
           RightLine[i] = RightLine[i+1] + 1;
           LeftLine[i] =  LeftLine[i+1] + 1;
       }
      
//      if(cirR == 1 && cirinside==0)
//      {
//           RightLine[i] = RightLine[i+1] + 1;
//           LeftLine[i] =  LeftLine[i+1] + 1;
//      }
//      else
//      {
//          LeftLine[i] = 2 * LeftLine[i+1] - LeftLine[i+2];
//          RightLine[i] = 2 * RightLine[i+1] - RightLine[i+2]; 
//      }
    }
    else if(L_findflag[i] == 0)
    {
    	if(RightLine[i] > RightLine[i+1])
    	{
            LeftLine[i] = LeftLine[i+1] + RightLine[i+1] - RightLine[i];
            RightLine[i] = 2 * RightLine[i+1] - RightLine[i];//
    	}
    	else
    	{
            LeftLine[i] = LeftLine[i+1] + RightLine[i] - RightLine[i+1];
    	}
    }
    else if(R_findflag[i] == 0)
    {
    	if(LeftLine[i] < LeftLine[i+1])
    	{
            RightLine[i] = RightLine[i+1] + LeftLine[i+1] - LeftLine[i];
            LeftLine[i] = 2 * LeftLine[i+1] - LeftLine[i];//
        }
        else
        {
            RightLine[i] = RightLine[i+1] + LeftLine[i] - LeftLine[i+1];
        }
    }
    
    if(cirout==1 && cirR == 1) //到达出环口
    {
      RightLine[i] = LeftLine[i] + 40;
    }
    
	/******边缘变化率限幅******/
    if(RightLine[i] - RightLine[i+1] > 10)
      RightLine[i] = RightLine[i+1] + 10;
    else if(RightLine[i] - RightLine[i+1] < -10)
      RightLine[i] = RightLine[i+1] - 10;
    
    if(LeftLine[i] - LeftLine[i+1] > 10)
      LeftLine[i] = LeftLine[i+1] + 10;
    else if(LeftLine[i] - LeftLine[i+1] < -10)
      LeftLine[i] = LeftLine[i+1] - 10;

    /******获取中线******/
    MidLine[i] = (LeftLine[i] + RightLine[i])/2;///////////////////////////////////////////////////
    Width[i] = RightLine[i] - LeftLine[i];
    
    if(MidLine[i]-MidLine[i+1] > 3)       //大变化率小限幅法
      MidLine[i] = MidLine[i+1] + 2;
    if(MidLine[i]-MidLine[i+1] < -3)
      MidLine[i] = MidLine[i+1] - 2;
    
    
    if(MidLine[i] < 0)                           //MidLine[i] 限幅
    {
      MidLine[i] = 0;
    }
    else if(MidLine[i] > 79)
    {
      MidLine[i] = 79; 
    }
    
   /******显示中线******/
#ifdef ShowMidLine   
    img[i][MidLine[i]] = black;
#endif
    
  if(i <= END)                                  //如果直接搜到了END行
    LAST = END;
  } 
}

/******中点结果获取******/
void GetMid(void)
{
  int8 i;
  int16 sum = 0;
  
  for(i=START;i>=LAST;i--)
  {
    sum += MidLine[i];
  }
  
  if(sum != 0)
  {
    AnglePID.ActualValue = sum / (START - LAST + 1);
    MidResult[1] = MidResult[0];
    MidResult[0] = AnglePID.ActualValue;
  }
  else                                          //和为0，即一个中点都没找到
  {
    AnglePID.ActualValue =2 * MidResult[0] - MidResult[1];
  }
}

/******首行定位******/ 
void GetBaseLine(void)
{
  int8  i,j; 
  static uint8 LastBase = 40;
   
  for(i=BASE;i>START;i--)
  {
    L_findflag[i] = 0;
    R_findflag[i] = 0;
   
    for(j=LastBase; j>=0; j--)            //中点向左扫 
    {
      if(img[i][j] == black)        //若第i行j列为边线
      {
        L_findflag[i] = 1;
        LeftLine[i] = j;            //则将第i行的左边线位置j存放在LeftLine[i]
        break;
      }
    }   
    
    for(j=LastBase; j<=79; j++)          //从上一次中点向右扫
    {
      if(img[i][j] == black)
      {
        R_findflag[i] = 1;
        RightLine[i] = j;
        break;
      }      
    }  
    
    if(L_findflag[i]==1 && R_findflag[i]==1)
    {
      MidLine[i] = (LeftLine[i] + RightLine[i])/2;
    }
    else if(L_findflag[i]==0 && R_findflag[i]==0)
    {
      MidLine[i] = LastBase;
      LeftLine[i] = 0;
      RightLine[i] = 79;
    }
    else if(L_findflag[i] == 0)
    {
      MidLine[i] = (0 + RightLine[i])/2;
      LeftLine[i] = 0;
    }
    else if(R_findflag[i] == 0)
    {
      MidLine[i] = (LeftLine[i] + 79)/2;
      RightLine[i] = 79;
    }
  }
  
  MidLine[30] = (MidLine[30] + MidLine[31])/2;
  LastBase = MidLine[30];
}
