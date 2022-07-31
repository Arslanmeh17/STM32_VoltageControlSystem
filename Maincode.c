#include "stm32f10x.h"
#include "delay.h" 
#include <stdbool.h>
#include <stdio.h>

void UART_Transmit(char *string){  //Our data processing function for sending to PC.
  while(*string)
  {
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  USART_SendData(USART1,*string++);
  }
}

	
double Gz(double uk)    // Difference Equation
{
	static double yk_1=0, yk_2=0, uk_1=0, uk_2=0;
	double yk = 1.822*yk_1-0.8262*yk_2+0.002134*uk_1+0.002002*uk_2;
	uk_2 = uk_1;
	uk_1 = uk;
	yk_2 = yk_1;
	yk_1 = yk;
	return yk;
}

volatile double Kp = 2.5, Ki = 0.0, Kd = 0;

double PID(double r, double y)     // P type Controller
{
static double uk_1 = 0, ek_1 = 0, ek_2 = 0;
double ek = r - y;
double uk = uk_1 + (Kp + Ki + Kd) * ek - (Kp + 2 * Kd) * ek_1 + Kd * ek_2;
uk_1 = uk;
ek_2 = ek_1;
ek_1 = ek;
return uk;
}

volatile double Kp = 2.8, Ki = 0.4, Kd=0;

double PID(double r, double y)     // PI type Controller
{
static double uk_1 = 0, ek_1 = 0;
double ek = r - y;
double uk = uk_1 + (2.8) * ek - (2.72) * ek_1;
uk_1=uk;
ek_1=ek;
return uk;
}

GPIO_InitTypeDef GPIO_InitStructure; // Peripheral libraries
EXTI_InitTypeDef EXTI_InitStructure; //External interrupt library
NVIC_InitTypeDef NVIC_InitStructure; //NVIC Library
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //Timer library
TIM_OCInitTypeDef TIM_OCInitStructure; //Oc library 
ADC_InitTypeDef ADC_InitStructure; //ADC library
USART_InitTypeDef USART_InitStructure; //USART Library

void GPIO_config(void);
void ADC_config(void);
void EXTIconfig(void);
void TIM3_config(void);
void NvicConfig(void);
void USART_config(void);
double Gz(double uk);
double PID(double r, double y);

bool reference = false;
			 
static int inputToSystem = 0;  // System input (step) to be sent if exti triggered

double RealOutputValue;

static double E;

double SimoutputToPC;

bool sendtime = false;    //Sending time value

char data[20];          // The value generated for processing the sending data
		

void TIM3_IRQHandler(void){   //TIMER Function for sending data to pc as period of 0.1 seconds.
      
		 if((TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) ){  // 10 Hz = 0.1 seconds of period.
			 			 
       sendtime=!sendtime; //this variable changes every 0.1 second.
     			 
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);   //we need to clear line pending bit manually
	 }
		}

void EXTI4_IRQHandler(void){   //Our interrupt for B4 (step input)

    if((EXTI_GetITStatus(EXTI_Line4) != RESET)) 
    {			
		 reference = !reference;
		}		 	    
		EXTI_ClearITPendingBit(EXTI_Line4);//we need to clear line pending bit manually
		}

 int main(void) {		 			 
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //A port clock enabled
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //B port clock enabled
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE); //AFIO clock enabled
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Timer clock enabled for send data
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // Setting Adc clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	// ADC clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART CLOCK enabled
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); //I2C Clock enabled

	delayInit(); //delay initialize
  
	GPIO_config(); //Init. of configurations.
  ADC_config();
	EXTIconfig();
	NvicConfig(); 
	TIM3_config();
	USART_config();

	  
     while(1)
     {			
//			 if(reference==true) inputToSystem=1;   //these are for simulating system.
//			 if(reference==false) inputToSystem=0;
//			 			

//            	
//			 if(sendtime==true){
////				 SimoutputToPC = Gz(inputToSystem);     //For only simulate G(s) step response
//				 
//				  E = PID(inputToSystem,Gz(E));           //For simulate P and PI type controlled system.
//				  SimoutputToPC = Gz(E); 
//				
//		 
//			   sprintf(data,"%0.2f\r",SimoutputToPC);
//         UART_Transmit(data);				 
//				 sendtime=false;
//			 }
			 
        if(reference==true)   // these are for no controlled REAL system. when button pressed, 1V reference applied to real system.
				{					
				TIM3->CCR1 = 15151;   // 15101/50k = 0.303. When max period --> 3.3V applied, 3.3*0.303 = 1V applied when pulse is 15151
				}
        if(reference==false) 
				{	
				TIM3->CCR1 = 0;  				
        }
        if(sendtime==true){                              //For only no controlled REAL system step response
				 RealOutputValue = ADC_GetConversionValue(ADC1); // getting the output value of real system.
         SimoutputToPC = RealOutputValue/1100 - 0.190;   // Aligning the output value.  			 
			   sprintf(data,"%0.2f\r",SimoutputToPC);
         UART_Transmit(data);				 
				 sendtime=false;
			 }

//        if(reference==true)   // these are for P-PI TYPE controlled REAL system.
//				{
//				inputToSystem=1;	 
//				}
//        if(reference==false) 
//				{	
//				inputToSystem=0;
//				TIM3->CCR1 = 0;  				
//        }	
//        if(sendtime==true)
//			 {
//				 RealOutputValue = ADC_GetConversionValue(ADC1); // getting the output value of real system.
//				 E = PID(inputToSystem,RealOutputValue);
//				 TIM3->CCR1 = -E*30.06;                       // Aligning the control signal for our purpose.
//         SimoutputToPC = RealOutputValue/1100 - 0.350;     //For only REAL system step response
//				 			 
//			   sprintf(data,"%0.2f\r",SimoutputToPC);
//         UART_Transmit(data);				 
//				 sendtime=false;
//			 }				 
   } //Closing while
 } //Closing main
 
 void GPIO_config(void)     //GPIO configuration
{  
	// Configure analog input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  // Configuring pin A0 for analog input (RealOutputValue).
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configue UART TX - (UART module's RX should be connected to this pin)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 	
	// Configure button for realizing step input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;   //Button on B4 pin 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //Pull-down mode
	GPIO_Init(GPIOB, &GPIO_InitStructure);	 //B port
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);	//Choosing port B4 as an external interrupt.
		
	// configure Vin output
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  //Vin pin
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //clock Speed
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-pull mode
  GPIO_Init(GPIOA, &GPIO_InitStructure); //A port
}

void NvicConfig(void)  //NVIC Configuration
{		
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //Choosing timer2 for NVIC
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; //Choosing Line4 for NVIC for button
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStructure);
}	

void EXTIconfig(void)   // EXTI configuration 
{
	EXTI_InitStructure.EXTI_Line = EXTI_Line4; //Choosing port B4 as line4 of external interrupt.
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //Choosing interrupt mode.
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //for detecting the pressing.
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

void ADC_config(void)   // ADC configuration 
{
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   //For continious conversation of RealOutputValue.
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);	 

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	// Start the conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
}

void TIM3_config(void)    // TIMER configuration for TIM3
{
  TIM_TimeBaseStructure.TIM_Period = 49999; 
	TIM_TimeBaseStructure.TIM_Prescaler = 143;            // 72M /144*50K = 10Hz = 0.1 second period.
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM3, ENABLE); //Enabling the timer
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // Our PWM for Vin to actual system
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OCInitStructure.TIM_Pulse = 0;  
  TIM_OC1Init(TIM3, &TIM_OCInitStructure); // for Vin (pin A6)

}

void USART_config(void)   // USART configuration 
{
	// USART settings
	USART_InitStructure.USART_BaudRate = 9600;     //Our Baud rate.
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx; // We use only Tx for transmitting the data
	USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
}

//  			 newU = PID(inputToSystem,oldSystemOutput);
//  			 SimoutputToPC = Gz(oldU);     //For simulate closed loop
//				 oldSystemOutput = SimoutputToPC;
//				 oldU = newU;
 //static float oldSystemOutput=0;

//static float oldU=0;

//static float newU=0;


////  			 newU = PID(inputToSystem,oldSystemOutput);   //For controlled REAL closed loop
////  			 SimoutputToPC = Gz(oldU);     
////				 oldSystemOutput = SimoutputToPC;
////				 oldU = newU;
