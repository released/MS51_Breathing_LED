/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/* Website: http://www.nuvoton.com                                                                         */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/

/************************************************************************************************************/
/*  File Function: MS51 DEMO project                                                                        */
/************************************************************************************************************/

#include "MS51_16K.h"


//#define ENABLE_16MHz
#define ENABLE_24MHz

#if defined (ENABLE_16MHz)
#define SYS_CLOCK 								(16000000ul)
#elif defined (ENABLE_24MHz)
#define SYS_CLOCK 								(24000000ul)
#endif

#define PWM_FREQ 								(32000ul)
//#define TIMER_LOG_MS							(1000ul)
#define DUTY_MAX								(100ul)
#define DUTY_MIN								(1ul)

uint8_t u8TH0_Tmp = 0;
uint8_t u8TL0_Tmp = 0;
uint8_t u8Duty = 0;

#define LED_ON									(P12 = 1)
#define LED_OFF									(P12 = 0)

//UART 0
bit BIT_TMP;
bit BIT_UART;
bit uart0_receive_flag=0;
unsigned char uart0_receive_data;

volatile uint16_t timer_counter = 0;

typedef enum{
	TIMER_10MS = 10 ,	
	TIMER_100MS = 100 ,
	TIMER_1000MS = 1000 ,
	TIMER_5000MS = 5000 ,

	
	TIMER_DEFAULT	
}TIMER_Define;

typedef enum{
	flag_LED_Reverse = 0 ,

	flag_1ms ,
	flag_10ms ,
	flag_100ms ,

	flag_1000ms ,
	flag_5000ms ,
	
	flag_DEFAULT	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))


void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);		
		#endif
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
}

void GPIO_Config(void)
{
	P11_PUSHPULL_MODE;	

	P13_PUSHPULL_MODE;	
	P14_PUSHPULL_MODE;		
}

void PWMx_CH0_SetDuty(uint16_t d)
{
	uint16_t res = 0 ;
	res = d*(MAKEWORD(PWMPH,PWMPL)+1)/100;

    PWM0H = HIBYTE(res);
    PWM0L = LOBYTE(res);

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void PWMx_Init(uint16_t uFrequency)
{
	uint32_t res = 0;

	P12_PUSHPULL_MODE;	//Add this to enhance MOS output capability
    PWM0_P12_OUTPUT_ENABLE;
	
    PWM_IMDEPENDENT_MODE;
    PWM_CLOCK_DIV_16;

/*
	PWM frequency   = Fpwm/((PWMPH,PWMPL)+1) = (24MHz/2)/(PWMPH,PWMPL)+1) = 20KHz
*/	

	res = (SYS_CLOCK>>4);			// 2 ^ 4 = 16
	res = res/uFrequency;
	res = res - 1;	

    PWMPH = HIBYTE(res);
    PWMPL = LOBYTE(res);
}

void BreathingLight_Handler(void)
{
	static uint16_t counter = 0;
	
	if (++counter >= 15)
	{
		counter = 0;

//		// toggle test
//		P11 = ~P11;
		
		if (is_flag_set(flag_LED_Reverse))
		{
			if (u8Duty++ == DUTY_MAX)
			{
				set_flag(flag_LED_Reverse,Disable);
				u8Duty = DUTY_MAX;

				// toggle test
				P11 = 0;
			}			
		}
		else
		{
			if (u8Duty-- == DUTY_MIN)
			{
				set_flag(flag_LED_Reverse,Enable);
				u8Duty = DUTY_MIN;

				// toggle test
				P11 = 1;				
			}			
		}		
	}
	 
}

void BreathingLight_Config(void)
{
	PWMx_Init(PWM_FREQ);
	set_flag(flag_LED_Reverse , Enable);
}

void loop_5000ms(void)
{	
	if (is_flag_set(flag_5000ms))
	{		
		set_flag(flag_5000ms,Disable);

//		// toggle test
//		P14 = ~P14;
			
	}
}

void loop_1000ms(void)
{	
	if (is_flag_set(flag_1000ms))
	{		
		set_flag(flag_1000ms,Disable);

//		// toggle test
//		P13 = ~P13;
		
	}
}

void loop_100ms(void)
{	
	if (is_flag_set(flag_100ms))
	{		
		set_flag(flag_100ms,Disable);
			
	}
}

void loop_10ms(void)
{	
	if (is_flag_set(flag_10ms))
	{		
		set_flag(flag_10ms,Disable);
			
	}
}

void loop_1ms(void)
{	
	if (is_flag_set(flag_1ms))
	{		
		set_flag(flag_1ms,Disable);
		BreathingLight_Handler();
		PWMx_CH0_SetDuty(u8Duty);	
	}
}

void loop_process(void)
{
	loop_1ms();
	loop_10ms();
	loop_100ms();
	
	loop_1000ms();
	loop_5000ms();
	
}


void timer_process(void)
{
	timer_counter++;

	set_flag(flag_1ms,Enable);
	
	if(!(timer_counter %TIMER_10MS)){
		set_flag(flag_10ms,Enable);}
	
	if(!(timer_counter %TIMER_100MS)){
		set_flag(flag_100ms,Enable);}

	if(!(timer_counter %TIMER_1000MS)){
		set_flag(flag_1000ms,Enable);}

	if(!(timer_counter %TIMER_5000MS)){
		set_flag(flag_5000ms,Enable);}

	if(timer_counter >= 65500){
		timer_counter = 0;}
}

void Timer0_IRQHandler(void)
{
	timer_process();	
	
}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();
}

void TIMER0_Init(void)
{
	uint16_t res = 0;

	ENABLE_TIMER0_MODE1;

	// 1 ms , 1000 Hz
	u8TH0_Tmp = HIBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000);
	u8TL0_Tmp = LOBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000); 

    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;

    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
    set_TCON_TR0;                                  //Timer0 run
}


void Serial_ISR (void) interrupt 4 
{
    if (RI)
    {   
      uart0_receive_flag = 1;
      uart0_receive_data = SBUF;
      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }
}

void UART0_Init(void)
{
	#if 1
	unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3

	#if defined (ENABLE_16MHz)
	RH3    = HIBYTE(65536 - (1000000/u32Baudrate)-1);  
	RL3    = LOBYTE(65536 - (1000000/u32Baudrate)-1);  
	#elif defined (ENABLE_24MHz)
	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	#endif
	
	set_T3CON_TR3;         //Trigger Timer3
	set_IE_ES;

	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else	
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF; 
	#endif
}


#if defined (ENABLE_16MHz)
void MODIFY_HIRC_16(void)
{
    unsigned char data hircmap0,hircmap1;
    set_CHPCON_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;
}

#elif defined (ENABLE_24MHz)
void MODIFY_HIRC_24(void)
{
    unsigned char data hircmap0,hircmap1;
/* Check if power on reset, modify HIRC */
    if (PCON&SET_BIT4)
    {
        set_CHPCON_IAPEN;
        IAPAL = 0x38;
        IAPAH = 0x00;
        IAPCN = READ_UID;
        set_IAPTRG_IAPGO;
        hircmap0 = IAPFD;
        IAPAL = 0x39;
        IAPAH = 0x00;
        set_IAPTRG_IAPGO;
        hircmap1 = IAPFD;
        clr_CHPCON_IAPEN;
        TA=0XAA;
        TA=0X55;
        RCTRIM0 = hircmap0;
        TA=0XAA;
        TA=0X55;
        RCTRIM1 = hircmap1;
        clr_CHPCON_IAPEN;
    }
}

#endif

void SYS_Init(void)
{
    MODIFY_HIRC_24();

    ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;                // global enable bit	
}

void main (void) 
{
    SYS_Init();

    UART0_Init();
	GPIO_Config();

	BreathingLight_Config();					
			
	TIMER0_Init();	// timer interrupt : 200 000
	
    while(1)
    {
		loop_process();
    }
}



