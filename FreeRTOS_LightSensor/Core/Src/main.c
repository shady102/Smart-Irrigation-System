#include "STM32f103Xb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define MoistureSensor_Priority 2
#define LightSensor_Priority 1
//float xPeriod=2;
void PWM_Init_TIM3(){
	 TIM3->CCER |=(1<<4);
	 TIM3->CCMR1 |=(1<<11)|(0b110<<12);
	 TIM3->CR1 |=(1<<7)|(0b10<<5);
	 TIM3->ARR =4000;
	 TIM3->EGR |=(1<<0);
	 TIM3->CR1 |=(1<<0);
}
void PWM_Init_TIM2(){
	 TIM2->CCER |=(1<<4);
	 TIM2->CCMR1 |=(1<<11)|(0b110<<12);
	 TIM2->CR1 |=(1<<7)|(0b10<<5);
	 TIM2->ARR =4000;
	 TIM2->EGR |=(1<<0);
	 TIM2->CR1 |=(1<<0);
}
void ADC1_Init(){
		ADC1->CR2 |=(1<<0);
		ADC1->CR2 |=(1<<2);
		while(ADC1->CR2 &(1<<2));
		ADC1->CR2 &=~(1<<1) & ~(1<<11);
		ADC1->CR2 |= (0b1111<<17);
		ADC1->SMPR2=0;
		ADC1->SQR1=0;
		ADC1->SQR3=0;
	}
void ADC2_Init(){
		ADC2->CR2 |=(1<<0);
		ADC2->CR2 |=(1<<2);
		while(ADC1->CR2 &(1<<2));
		ADC2->CR2 &=~(1<<1) & ~(1<<11);
		ADC2->CR2 |= (0b1111<<17);
		ADC2->SMPR2=0;
		ADC2->SQR1=0;
		ADC2->SQR3=3;
	}
uint16_t ADC2_Read(){
	ADC2->CR2 |=(1<<22);
	while((ADC2->SR &(1<<1))==0);
	ADC2->SR &=~(1<<1);
	return ADC2->DR;
}
uint16_t ADC1_Read(){
	ADC1->CR2 |=(1<<22);
	while((ADC1->SR &(1<<1))==0);
	ADC1->SR &=~(1<<1);
	return ADC1->DR;
}
void TaskMoisturePump(void* pvParameters){
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(50);

	float desmoisture=50;
	float actmoisture;
	float Error2;
	float prevError2;
	float sum2=0;
	float diff;
	int adcread=0;
	float u2;

	ADC2_Init();
	PWM_Init_TIM3();
	xLastWakeTime= xTaskGetTickCount();
	   while(1){

		   adcread = ADC2_Read();
		   actmoisture=100-((adcread/4095.0)*100);
		   Error2 = desmoisture - actmoisture;
		  	 	//  int u = (int)1000* Error;
		   if (Error2>2){
		  	 	GPIOA->ODR |= (1 << 8);
		  	 	GPIOA->ODR &= ~(1 << 9);
		  	 	sum2=sum2 +(((Error2+prevError2)/2)*xPeriod); // sum= sum +(((Error-prevError)/2)*Δt)
		  	 	diff=(Error2-prevError2)/xPeriod;

		  	 	if(sum2>=2000){
		  	 		sum2=2000;
		  	 	}
		  	  u2= (5*Error2)+(5*sum2)+(10 *diff);
		  		 		TIM3->CCR2 = u2;
		  		 	 }


		   else {
			  GPIOA->ODR |= (1 << 8);
			  GPIOA->ODR &= ~(1 << 9);
			   TIM3->CCR2 = 0;
		   	   }
		   prevError2=Error2;

		   vTaskDelayUntil(&xLastWakeTime, xPeriod);
	   }
}

void TaskLightSensor(void* pvParameters){

		uint16_t desLight;
		float actualLight;
		float Error;
		uint16_t prevError;
		uint16_t U;
		float microamps;
		float Voltage;
		float Current;
		float sum;
		float errorTest;

		TickType_t xLastWakeTime;
		TickType_t xPeriod=pdMS_TO_TICKS(100);
		desLight=0;
		ADC1_Init();
		PWM_Init_TIM2();
		//GPIOA->ODR |=(1<<5);
		//GPIOA->ODR &= ~(1<<6);
		xLastWakeTime=xTaskGetTickCount();
		while(1){
			Voltage=ADC1_Read();
			Voltage=Voltage*(3.3/4095);
			Current=Voltage/10000;
			microamps = Current * 1000000;
			actualLight=2*microamps;
			Error=desLight-actualLight;
			if(Error>0){
				GPIOA->ODR |=(1<<5);
				GPIOA->ODR &= ~(1<<6);
				//GPIOA->ODR |=(1<<7);
				errorTest=Error;
			}
			else{
				GPIOA->ODR &=~(1<<5);
				GPIOA->ODR |= (1<<6);
				//GPIOA->ODR &=~(1<<7);
				errorTest=-Error;
			}
			sum=sum+((prevError+errorTest)/2)*xPeriod;
			if (sum>=2000){
				sum=0;
			}
			U=(10*errorTest);//+(10*sum)+(3*((errorTest-prevError)/xPeriod));
		//	U=(5*Error)+ (10*prevError)+ 3*((prevError+Error)/2);
			TIM2->CCR2=U;
			prevError=errorTest;

			vTaskDelayUntil(&xLastWakeTime,xPeriod);

		}
}
int main(void){

	RCC-> APB2ENR |=RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN|RCC_APB2ENR_AFIOEN|RCC_APB2ENR_ADC2EN;
	RCC-> APB1ENR |=RCC_APB1ENR_TIM2EN |RCC_APB1ENR_TIM3EN;;
	GPIOA->CRL=0xB33404B0;
	GPIOA->CRH=0x44444433;

	xTaskCreate(TaskLightSensor,"Light Sensor & Motor",128,NULL,LightSensor_Priority,NULL);
	xTaskCreate(TaskMoisturePump,"Moisture Task",128,NULL,MoistureSensor_Priority,NULL);

	vTaskStartScheduler();
	while(1);
}
