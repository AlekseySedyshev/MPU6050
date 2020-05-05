#include "stm32f0xx.h"        // Device header
#include "SSD1306.h"
#include "mpu6050.h"
#include <math.h>
//#define I2C_100				1	//		100 kHz i2C Freq
#define I2C_400				1		//		400 kHz i2C Freq
TM_MPU6050_t MPU6050_Sensor;
uint8_t i,j,flag=0,BUFF[130];

uint16_t TimingDelay,led_count,sec05_f=0,s=0;
uint16_t counter;


void TimingDelayDec(void) 																													{ //msec - timer
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;sec05_f++;}
 led_count--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void TIM2_IRQHandler(void)																													{
if (TIM2->SR & TIM_SR_TIF) {
	//				counter = TIM2->CNT/469;
  //				TIM2->SR &=(~TIM_SR_TIF);
}
}
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}


//----------------------------------------------------
	
	float XAxisFinal,YAxisFinal,ZAxisFinal,XGyroFinal,YGyroFinal,ZGyroFinal;
	float delta_angle_x,delta_angle_y,pitch,roll,yaw,real_angle = 0,prev_angle = 0;
	
	float ang_x,ang_y,ang_z,Gangle_x,Gangle_y;
	float old_x = 0,old_y = 0,prev_angle_x = 0,prev_angle_y = 0;
	float XAxis_kalman,YAxis_kalman,ZAxis_kalman,XGyro_kalman,YGyro_kalman,ZGyro_kalman;
	float kalman_old = 0,cov_old = 1,kalman_new,cov_new,avg,val1, val2, val3, val4, val5 ;
	float M_PI=3.14159265358979323846;

	int AX_offset = 0,AY_offset = 0,AZ_offset = 0;//1700;
	int GX_offset = 520,GY_offset = 0,GZ_offset = 0;
	
	TM_MPU6050_t MPU6050_Sensor;
	
	
//----------------------------------------------------
	float kalman_filter (float input)										{

	   kalman_new = kalman_old;
		 cov_new = cov_old + 0.50;

	  float kalman_gain = cov_new / (cov_new + 0.9);
	  float kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new));

	  cov_new = (1 - kalman_gain) * cov_old;
	  cov_old = cov_new;

	  kalman_old = kalman_calculated;

	  return kalman_calculated;

	}

	void math_Accel()															{
		XAxisFinal = (float)(MPU6050_Sensor.Accelerometer_X + AX_offset) * MPU6050_Sensor.Acce_Mult;
		YAxisFinal = (float)(MPU6050_Sensor.Accelerometer_Y + AY_offset) * MPU6050_Sensor.Acce_Mult;
		ZAxisFinal = (float)(MPU6050_Sensor.Accelerometer_Z + AZ_offset) * MPU6050_Sensor.Acce_Mult;
/*
		XAxis_kalman = kalman_filter(XAxisFinal);
		YAxis_kalman = kalman_filter(YAxisFinal);
		ZAxis_kalman = kalman_filter(ZAxisFinal);

		if(XAxis_kalman>0.99) XAxis_kalman=1;   
		if(YAxis_kalman>0.99) YAxis_kalman=1;
		if(ZAxis_kalman>0.99) ZAxis_kalman=1;

		if(XAxis_kalman<-0.99) XAxis_kalman=-1; 
		if(YAxis_kalman<-0.99) YAxis_kalman=-1;
		if(ZAxis_kalman<-0.99) ZAxis_kalman=-1;
*/		
	}

	void math_Gyro()																{
		XGyroFinal = (float)((float)(MPU6050_Sensor.Gyroscope_X+GX_offset)* MPU6050_Sensor.Gyro_Mult); 
		YGyroFinal = (float)((float)(MPU6050_Sensor.Gyroscope_Y+GY_offset)* MPU6050_Sensor.Gyro_Mult);                     
		ZGyroFinal = (float)((float)(MPU6050_Sensor.Gyroscope_Z+GZ_offset)* MPU6050_Sensor.Gyro_Mult); 
/*
		XGyro_kalman = kalman_filter(XGyroFinal);
		YGyro_kalman = kalman_filter(YGyroFinal);
		ZGyro_kalman = kalman_filter(ZGyroFinal);

		delta_angle_x = (0.03 * old_x) + ((0.03 * (XGyro_kalman - old_x)) / 2); //a??sal h?z ve ge?en s?reye ba?l? olarak taranan a?? hesaplan?r
		Gangle_x = (prev_angle_x + delta_angle_x);                             //taranan a?? de?eri ile bir ?nceki a?? de?eri hesaplanarak
		prev_angle_x = Gangle_x;                                                     //g?ncel a?? de?eri bir sonraki d?ng?de kullan?lmak ?zere ?nceki a?? de?eri olarak kaydedilir
		old_x = XGyro_kalman;

		delta_angle_y = (0.03 * old_y) + ((0.03 * (YGyro_kalman - old_y)) / 2); //yukar?daki i?lemlerin ayn?s? Y ekseni i?in de yap?l?r
		Gangle_y = (prev_angle_y + delta_angle_y);
		prev_angle_y = Gangle_y;
		old_y = YGyro_kalman;
*/		

	}

//------------------------------LCD ------------------------------

void initial (void)																																	{
//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 								//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;								//Pb0-Out 
//------------I2C1---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOFEN;
	GPIOF->MODER 		|=GPIO_MODER_MODER0_1 		| GPIO_MODER_MODER1_1; 							// Alt -mode /Pf0 - SDA, Pf1- SCL
	GPIOF->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR0 	| GPIO_OSPEEDER_OSPEEDR1;
	GPIOF->OTYPER		|=GPIO_OTYPER_OT_0 				| GPIO_OTYPER_OT_1;
	GPIOF->AFR[0] 	|=(1<<GPIO_AFRL_AFRL0_Pos) |(1<<GPIO_AFRL_AFRL1_Pos);  				// I2C - Alternative

	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
#ifdef I2C_100	
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_PRESC_Pos); 	//100 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x13	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0xF	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x2	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x4	<<I2C_TIMINGR_SCLDEL_Pos);
#endif
#ifdef I2C_400	
	I2C1->TIMINGR |=(0x0	<<I2C_TIMINGR_PRESC_Pos); 	//400 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x9	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLDEL_Pos);
#endif	
	I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
	I2C1->CR1 |=I2C_CR1_PE;
	
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 								//
	GPIOA->MODER |= GPIO_MODER_MODER1_0;							//Output mode;							
	
	GPIOA->MODER &=(~GPIO_MODER_MODER0);							//Input_mode
	GPIOA->PUPDR |=GPIO_PUPDR_PUPDR0_1;								//Pull-Up

} 




int main(void)																																			{

initial();
delay_ms (200);	
LCD_Init();LCD_Clear();
LCD_Gotoxy (5,0);LCD_PrintStr(" TEST MPU6050 ",1);

MPU6050_Init(&MPU6050_Sensor, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_2G, TM_MPU6050_Gyroscope_250s);	

//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
 if (sec05_f)											{// Run - 1 time in second
		sec05_f=0;

	 MPU6050_ReadAll(&MPU6050_Sensor);
	 
	math_Accel();
	math_Gyro();
	pitch = 180 * atan (XAxisFinal/sqrt(YAxisFinal*YAxisFinal + ZAxisFinal*ZAxisFinal))/M_PI;
	roll = 	180 * atan (YAxisFinal/sqrt(XAxisFinal*XAxisFinal + ZAxisFinal*ZAxisFinal))/M_PI;
	yaw = 	180 * atan (ZAxisFinal/sqrt(XAxisFinal*XAxisFinal + ZAxisFinal*ZAxisFinal))/M_PI;
	 
LCD_Gotoxy (1,1);LCD_PrintStr("Temp.: ",0);LCD_PrintDec(((int32_t)MPU6050_Sensor.Temperature),0);LCD_PrintStr(" C   ",0);
LCD_Gotoxy (1,2);LCD_PrintStr("aX ",0);LCD_PrintDec((int32_t)(XAxisFinal*90),0);LCD_PrintStr("    ",0); LCD_Gotoxy (64,2);LCD_PrintStr("gX ",0);LCD_PrintDec(XGyroFinal,0);LCD_PrintStr("    ",0);
LCD_Gotoxy (1,3);LCD_PrintStr("aY ",0);LCD_PrintDec((int32_t)(YAxisFinal*90),0);LCD_PrintStr("    ",0); LCD_Gotoxy (64,3);LCD_PrintStr("gY ",0);LCD_PrintDec(YGyroFinal,0);LCD_PrintStr("    ",0);
LCD_Gotoxy (1,4);LCD_PrintStr("aZ ",0);LCD_PrintDec((int32_t)(ZAxisFinal*90),0);LCD_PrintStr("    ",0); LCD_Gotoxy (64,4);LCD_PrintStr("gZ ",0);LCD_PrintDec(ZGyroFinal,0);LCD_PrintStr("    ",0);
LCD_Gotoxy (1,5);LCD_PrintStr("Pitch ",0);LCD_PrintDec((int32_t)(pitch),0);LCD_PrintStr("  ",0);	
LCD_Gotoxy (1,6);LCD_PrintStr("Roll ",0);LCD_PrintDec((int32_t)(roll),0);LCD_PrintStr("  ",0); 
LCD_Gotoxy (1,7);LCD_PrintStr("Yaw ",0);LCD_PrintDec((int32_t)(yaw ),0);LCD_PrintStr("  ",0); 
	 
 }
	 
	
				
		

} // end - main loop 
} // end - Main  
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
