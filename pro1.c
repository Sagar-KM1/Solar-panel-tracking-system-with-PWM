#include <LPC17xx.h>
#include<stdio.h>
#include "lcd_fun.c"

#define VREF 3.3  //REFERENCE VOLTAGE @VREFP PIN, VREFN=0
#define ADC_CLK_EN() (1<<12)  // POWER ENABLE FOR ADC
#define SEL_AD0_2 (1<<2)  //SEL CHANNEL AD0.1
#define CLKDIV (3<<8) // ADC CLKDIV (ADC_CLK=PCLK/CLKDIV+1)=1MHZ
#define PWRUP (1<<21) // SETTING TO OPERATION MODE
#define START_CNV (1<<24)  // 001 FOR START CONVERSION
#define ADC_DONE (1U<<31) // DEFINE IT AS UNSIGNED VALUE
#define T_COEFF 100.0f

    void PWM_Init(void) {
    LPC_PINCON->PINSEL4 |= (1 << 0); // Configure P2.0 as PWM1.1
    LPC_PWM1->PCR = (1 << 9); // Enable PWM1 output
    LPC_PWM1->MR0 = 20000; // Set period (20ms for servo)
    LPC_PWM1->MR1 = 1500; // Set initial duty cycle (1.5ms for 90 degrees)
    LPC_PWM1->MCR = (1 << 1); // Reset on MR0 match
    LPC_PWM1->LER = (1 << 0) | (1 << 1); // Load new values
    LPC_PWM1->TCR = (1 << 0) | (1 << 3); // Enable PWM and counter
   }

   void setServoAngle(uint16_t angle) {
     uint16_t pulseWidth = (angle * 10) + 1000; // Convert angle to pulse width
    LPC_PWM1->MR1 = pulseWidth;
    LPC_PWM1->LER = (1 << 1); // Load new value
    }

int main(void)
 {
	unsigned int i;
	int result=0;
    float volts=0;
    char svolts[20];
	float temp=0;
    char stemp[20];
    PWM_Init();
	



LPC_PINCON -> PINSEL1 |= (1<<18);//SELECT AD0.2 FOR PO.25
LPC_SC -> PCONP |= ADC_CLK_EN(); // ENABLE ADC CLOCK SC=SYSTEM CLOCK
LPC_ADC -> ADCR = PWRUP | CLKDIV | SEL_AD0_2;
lcd_config();

while(1)
{
LPC_ADC -> ADCR |= START_CNV;// START NEW CONVERSION
while((LPC_ADC -> ADDR2 & ADC_DONE) ==0){} //WAIT UNTIL CONVERSION IS FINISHED
result = (LPC_ADC -> ADDR2 >> 4) & 0XFFF; //12 BIT MASK TO EXTRACT RESULT

volts = (result *VREF)/4096.0;// CONVERT RESULT TO VOLTAGE
sprintf(svolts, "voltage=%.2f v",volts);
lcd_str_write(svolts);
lcd_cmd_write(0Xc0);

//temp=volts*T_COEFF;
//sprintf(stemp, "temp=%.2f 'c",temp);
lcd_str_write(stemp);
delay(500);
lcd_cmd_write(0x01);

  if (volts < 2.5)
	 {
        setServoAngle(90); // Set servo to 90 degrees
		for(i = 0; i<1000000; i++); // Delay
        setServoAngle(0); // Set servo to 0 degrees
        for(i = 0; i< 1000000; i++); // Delay
    }
}
   return 0;
}