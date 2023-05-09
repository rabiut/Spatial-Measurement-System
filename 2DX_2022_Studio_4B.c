// 2DX4_Knowledge_Thread_3_Session_1
// This program illustrates the use of SysTick in the C language.
// Note the library headers asscoaited are PLL.h and SysTick.h,
// which define functions and variables used in PLL.c and SysTick.c.
// This program uses code directly from your course textbook.

//  Written by Ama Simons
//  January 18, 2020
//  Last Update: January 18, 2020


#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"



void PortM_Init(void){
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}
void PortE0_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		              // activate the clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	        // allow time for clock to stabilize
  
	GPIO_PORTE_DIR_R = 0b00000000;//enable
	GPIO_PORTE_DEN_R = 0b00000001;                        		// Enabled both as digital outputs
	return;
	}

void PortN0N1_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 //activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R=0b00000011;
	GPIO_PORTN_DEN_R=0b00000011;
	return;
}
	
void PortF0F4_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;		              // activate the clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	        // allow time for clock to stabilize
  
	GPIO_PORTF_DIR_R = 0b00010001;//enable
	GPIO_PORTF_DEN_R = 0b00010001;                        		// Enabled both as digital outputs
	return;
	}

//Flash D1
void FlashLED1(int count) {
		while(count--) {
			GPIO_PORTN_DATA_R ^= 0b00000010; 								//hello world!
			SysTick_Wait10ms(1);														//.1s delay
			GPIO_PORTN_DATA_R ^= 0b00000010;			
		}
}

void spin(){
	for(int i=1; i<=512; i++){//ccw
		
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		
		if (i==64 || i==64*2 || i==64*3 || i==64*4 || i==64*5 || i==64*6 || i==64*7 || i==64*8 ){
			FlashLED1(1);
		}
	}
	
	/*for(int i=0; i<512; i++){//cw
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);

	}*/
}

int main(void){
	PLL_Init();																			// Default Set System Clock to 120MHz
	SysTick_Init();																	// Initialize SysTick configuration
	PortM_Init();
	PortN0N1_Init();
	PortF0F4_Init();
	PortE0_Init();
	volatile int check;
	while(1){
		check = GPIO_PORTE_DATA_R;
		if((GPIO_PORTE_DATA_R&=0b00000001) == 0){//check first bit
				spin();	
		}
		
	}
	return 0;
}






