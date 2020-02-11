#include "key.h"

void Beep_Init(void)
{
   /*Initialize the BEEP*/
   gpio_init(PTC1, GPO, 1);
   port_init_NoALT(PTC1, PULLUP); 
}

void Pipe_Init(void)
{
   /*Initialize the Pipe*/
   gpio_init(PTA2, GPI, 1);
   port_init_NoALT(PTA2, PULLUP); 
}

void Key_Init(void)
{ 
     /*Initialize the KEYU*/
       gpio_init(PTB21, GPI, 0);
       port_init_NoALT(PTB21, PULLUP); 
     /*Initialize the KEYD*/
       gpio_init(PTB23, GPI, 0);
       port_init_NoALT(PTB23, PULLUP);
     /*Initialize the KEYL*/  
       gpio_init(PTB20, GPI, 0);
       port_init_NoALT(PTB20, PULLUP);
     /*Initialize the KEYR*/
       gpio_init(PTB22, GPI, 0);
       port_init_NoALT(PTB22, PULLUP);
     /*Initialize the KEYM*/
       gpio_init(PTB19, GPI, 0);
       port_init_NoALT(PTB19, PULLUP);   
}

void Dip_Init(void)
{
     /*Initialize the DIP1*/
       gpio_init(PTD1, GPI, 0);
       port_init_NoALT(PTD1, PULLUP); 
     /*Initialize the DIP2*/
       gpio_init(PTD0, GPI, 0);
       port_init_NoALT(PTD0, PULLUP); 
     /*Initialize the DIP3*/
       gpio_init(PTD3, GPI, 0);
       port_init_NoALT(PTD3, PULLUP);
     /*Initialize the DIP4*/
       gpio_init(PTD2, GPI, 0);
       port_init_NoALT(PTD2, PULLUP);  
}
