#include "adc.h"

#define AD_CNT1 5  //在3个采样值中选出一个中间值，这一件事一共做5次
#define AD_CNT2 3
#define AD_NUM 2   //电感个数

uint8  AD_MIN[AD_NUM] = {0,0};       //L,R
uint8  AD_MAX[AD_NUM];

void ADC_Init(void)
{
	adc_init(ADC0_SE8);
	adc_init(ADC0_SE9);
}


/******提取AD采集数据中的中间值******/
uint8 ad_middle_select(uint8 *ad_group)
{
	uint8 i,j;
	uint8 k;
	for(i=0;i<AD_CNT2-1;i++)
	{
		for(j=0;j<AD_CNT2-1;j++)
		{
			if(ad_group[j]<ad_group[j+1])
			{
				k = ad_group[j+1];
				ad_group[j+1] = ad_group[j];
				ad_group[j] = k;
			}
		}
	}
	return ad_group[(AD_CNT2-1)/2];
}

/******ADC采集及归一化处理******/
uint16 ADC_Collect(ADCn_Ch_e my_cn,ADC_POS adc_pos)
{
	uint8  i,j;
	uint8  ad_last = 0;          //当前值
	uint8  chazhi;               
	
	uint8  AD_MIDDLE[AD_CNT2];   //存储采集AD_CNT2次的值
        uint8  AD_AVERAGE[AD_CNT1];  //存储AD_CNT1个中间值
        uint16  AD_LAST[AD_NUM];     //归一化的结果
	
	for(i=0; i<AD_CNT1; i++)
	{
		for(j=0; j<AD_CNT2; j++)
		{
			AD_MIDDLE[j] = adc_once(my_cn,ADC_8bit);
		}
		AD_AVERAGE[i] = ad_middle_select(AD_MIDDLE);
	}
	ad_last = (AD_AVERAGE[0]+AD_AVERAGE[1]+AD_AVERAGE[2]+AD_AVERAGE[3]+AD_AVERAGE[4])/AD_CNT1; 
 
        if(ad_last > 254)  
          ad_last = 255;
         
	//归一化
        chazhi = ad_last - AD_MIN[adc_pos];               //在最小值为0时，chazhi为无符号会溢出，误判为圆环
	AD_LAST[adc_pos] = chazhi*80/(AD_MAX[adc_pos]-AD_MIN[adc_pos]);

        return AD_LAST[adc_pos];
}

/******ADC采集绝对值******/
uint8 adc_collect(ADCn_Ch_e my_cn,ADC_POS adc_pos)
{
	uint8  i,j;
	uint8  ad_last = 0;          //当前值             
	
	uint8  AD_MIDDLE[AD_CNT2];   //存储采集AD_CNT2次的值
        uint8  AD_AVERAGE[AD_CNT1];  //存储AD_CNT1个中间值
   
	for(i=0; i<AD_CNT1; i++)
	{
		for(j=0; j<AD_CNT2; j++)
		{
			AD_MIDDLE[j] = adc_once(my_cn,ADC_8bit);
		}
		AD_AVERAGE[i] = ad_middle_select(AD_MIDDLE);
	}
	ad_last = (AD_AVERAGE[0]+AD_AVERAGE[1]+AD_AVERAGE[2]+AD_AVERAGE[3]+AD_AVERAGE[4])/AD_CNT1; 
 
        if(ad_last > 254)  
          ad_last = 255;

        return ad_last;
}

void admax_collect(void)
{
  uint8 i;
  uint8 left,right;
  uint16 sum[5];
  
  for(i=0; i<5; i++)
  {
     left  =  adc_collect(ADC0_SE8,adc_L);   //绝对值
     right =  adc_collect(ADC0_SE9,adc_R);
     sum[i]   =  left + right;
  }
  AD_MAX[0] = (sum[0] + sum[1] + sum[2] + sum[3] + sum[4])/5;
  AD_MAX[1] = AD_MAX[0];
}

