/****************************************************************
 * Lab One:     Interfacing with MSP432 using Interrupts
 * Author:      Kasonde Musonda
 * Date:        9/18/2020
 * Professor:   Dr. Brian Krug
 * Description:
 *              To develop a program for the MSP432 microcontroller (MCU) that interfaces
 *              with pushbutton switches to control the sequencing of color lighting using a RGB LED
 *              To develop a program for the MSP432 MCU that uses the SysTick system timer to generate
 *              precise time intervals for controlling duration of LED illumination
 *              To incorporate both SysTick and GPIO interrupts
 *              To develop a program that uses multiple pushbuttons in combination to carry out a function
 ****************************************************************/
#include "msp.h"

enum leds{off,fast,faster,fastest,slow,slower,slowest};  /*Blinking frequency for leds*/
enum leds state = off;

volatile uint8_t flag = 0;             /*Flag for first button*/
volatile uint8_t flag2 = 0;            /*Flag for second button*/
volatile uint32_t load_val=3000000;
volatile uint8_t count=0;              /*Counter for button 1*/
volatile uint8_t count2=0;

/*Function prototypes*/
void setPins();
void sysTick_delay();
void PORT5_IRQHandler(void);
void SysTick_Handler(void);
void execute();

/*
 * Setup Function - Sets the Pushbuttons and LEDs Ports
 * P2.3-P2.5 for LEDs as Output ports.
 * P5.4 and P6.1 as input ports for buttons.
 * @Param:void
 * @Return: void
 */

void setPins(){
    __disable_irq();

    /*White Button- Connected to P5.0*/
    P5->SEL0 &= ~BIT0;
    P5->SEL1 &= ~BIT0;
    P5->DIR &= ~BIT0;
    P5->REN |= BIT0;
    P5->OUT |= BIT0;
    P5->IES |= BIT0;       /* Make interrupt trigger on high to low transition*/
    P5->IFG = 0;           /* Clear pending interrupt flags*/
    P5->IE |= BIT0;        /* Enable interrupt on P5.4*/

    //Black Button - Connected to P3.0*/
    P3->SEL0 &= ~BIT0;
    P3->SEL1 &= ~BIT0;
    P3->DIR &= ~BIT0;
    P3->REN |= BIT0;
    P3->OUT |= BIT0;
    P3->IES |= BIT0;       /*Interrupt setup on the P5.5*/
    P3->IFG = 0;
    P3->IE |= BIT0;

    P3->SEL0 &= ~BIT6;     /*Configures P3.6 as GPIO.*/
    P3->SEL1 &= ~BIT6;
    P3->DIR |= BIT6;        /*Sets P3.6 as output.*/
    P3->OUT &= ~BIT6;

    /*Interrupt Enable*/
    NVIC->ISER[0] = 1 << ((TA0_N_IRQn) & 31); // Enable interrupt in NVIC vector
     __enable_irq();
}

/*  GANSSLE switch bounce routine - SW1
 *  Service routine called at regular timer intervals
 */
uint8_t DebounceSwitch1(){
 static uint16_t State = 0; /*Current debounce status*/
                            /*read switch, upper 5 bits of State are don't cares*/
 State=(State<<1) | (P5IN & 0x1) | 0xf800;
 if(State==0xfc00)return 1; // indicates 0 level is
 // stable for 10 consecutive calls
 return 0;
}

/*  GANSSLE switch bounce routine - SW 2
 *  Service routine called at regular timer intervals
 */
uint8_t DebounceSwitch2(){
 static uint16_t State = 0; /*Current debounce status*/
                            /*read switch, upper 5 bits of State are don't cares*/
 State=(State<<1) | (P3IN & 0x1) | 0xf800;
 if(State==0xfc00)return 1; // indicates 0 level is
 // stable for 10 consecutive calls
 return 0;
}
/*
 * SysTick_Timer with Interrupts.
 */
void SysTick_Handler(){
    P3->OUT ^= BIT6;
}

/*
 * SysTick Custom Time Delay
 */
void sysTick_delay(){ /*delay in milliseconds using SysTick*/
    __disable_irq();                   /*IRQ disable*/

    SysTick->LOAD = load_val-1;
    SysTick->CTRL = 7;                  /*enables SysTick 3MHz no interrupts*/

    /*Interrupt Enable*/
    NVIC_SetPriority(PORT5_IRQn,1);  /*Sets P5.4 priority in NVIC register*/
    NVIC_EnableIRQ(PORT5_IRQn);      /*Enable interrupt in NIVC*/

    NVIC_SetPriority(PORT3_IRQn,2);  /*Sets P3.0 priority in NVIC register*/
    NVIC_EnableIRQ(PORT3_IRQn);      /*Enable interrupt in NIVC*/

    __enable_irq();                      /*enable interrupts*/
}

/*
 * SW1 Handler Connected to P5.0
 */
void PORT5_IRQHandler(void){

   if(P5->IFG & BIT0){  /*Button P5.4 pressed*/
      P5->IE &= ~BIT0; /*Disables interrupts*/
      flag=1;
    }
   P5->IFG &= ~BIT0;     /*Clears the interrupt*/
}

/*
 * SW1 Handler Connected to P3.0
 */
void PORT3_IRQHandler(void){

   if(P3->IFG & BIT0){  /*Button P5.4 pressed*/
      P3->IE &= ~BIT0; /*Disables interrupts*/
      flag=1;
    }
   P3->IFG &= ~BIT0;     /*Clears the interrupt*/
}

/*
 * Button and Interrupt cycles
 * @Param: void
 * @Return: void
 */
void execute(){

while(1){
    if(DebounceSwitch1()){
        if(flag){

           /*State Machine traversing*/
           count++;

           /*State Machine Traverse*/
           if(count==1){state=fast;}
           else if(count==2){state=faster;}
           else if(count==3){state=fastest;}

           flag=0;  /*Disables flags*/

           P5->IE |= BIT0; /*Enables interrupt on P5.0*/
            }
    }else if(DebounceSwitch2()){
        if(flag2){
            count2++;   /*Counter for SW2 - Black button on circuit*/

            /*State Machine Traverse*/
            if(count2==1 && count==3){state=faster;}
            else if(count2==2 && count==2){state=faster;}
            else if(count2==3 && count ==1){state=fast;}
            else{ state=off;}

            flag2=0;        /*Flag Disable on P3.0*/
            P3->IE |= BIT0; /*Enables interrupt on P3.0*/
        }
    }

    switch(state){
       case off:     /*turns off all color LEDs on leds*/
           P3->OUT &= ~BIT6;
           break;
       case fast:   /*Toggles all leds (one second on and one second off).*/
           load_val=150000;
           break;
       case faster:
           load_val=750000;
       case fastest:
           load_val=375000;
       default:
       state = off;  /*Default state of the leds*/
        }
    }
}
void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer.

    /*Main Function Invocation*/
    setPins();
    sysTick_delay();
    execute();
}

