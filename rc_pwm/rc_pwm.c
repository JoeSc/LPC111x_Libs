/*
 *  RC.c
 *  RC_PWM
 *
 *  Created by Joe  Schaack on 12/2/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "rc_pwm.h"





// Connected to 
// 0: CT16B0_MAT0 PIO0_8
// 1: CT16B0_MAT1 PIO0_9
// 2: CT16B0_MAT2 PIO0_10
// 3: CT16B1_MAT1 PIO1_10

// PPM in = C32B0_CAP0  PIO1_5

// Radio                1000 / 2000
// CH 1     =   Roll    L    / R
// CH 2     =   Pitch   Forw / Back
// CH 3     =   Throt   More / Less
// CH 4     =   Yaw     CW   / CCW
// CH 5     =   SW G    3 pos   1000 / 1500 / 2000
// CH 6     =   dial VR 



void rcinit()
{

    /************   TMR16B0   **************/
    TMR_TMR16B0TCR = TMR_TMR16B0TCR_COUNTERENABLE_DISABLED;
    SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_CT16B0;

    IOCON_PIO0_8 = IOCON_PIO0_8_FUNC_CT16B0_MAT0;
    IOCON_PIO0_9 = IOCON_PIO0_9_FUNC_CT16B0_MAT1;
    IOCON_JTAG_TCK_PIO0_10 = IOCON_JTAG_TCK_PIO0_10_FUNC_CT16B0_MAT2;

    TMR_TMR16B0PWMC = TMR_TMR16B0PWMC_PWM3_ENABLED | TMR_TMR16B0PWMC_PWM2_ENABLED
        | TMR_TMR16B0PWMC_PWM1_ENABLED | TMR_TMR16B0PWMC_PWM0_ENABLED;

    TMR_TMR16B0PR = 47;    //set prescaler
    // to 48 to allow 20000 range

    TMR_TMR16B0MR3 = 20000;    //set full range value

    TMR_TMR16B0MR2 = 20000-1500;         //REMEBER the value starts low so val = 20000-val
    TMR_TMR16B0MR1 = 20000-1500;
    TMR_TMR16B0MR0 = 20000-1500;


    TMR_TMR16B0MCR = TMR_TMR16B0MCR_MR3_RESET_ENABLED;//RESET ON MR3


    /************   TMR16B1   **************/
    TMR_TMR16B1TCR = TMR_TMR16B1TCR_COUNTERENABLE_DISABLED;
    SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_CT16B1;

    IOCON_PIO1_10 = IOCON_PIO1_10_FUNC_CT16B1_MAT1;

    TMR_TMR16B1PWMC = TMR_TMR16B1PWMC_PWM3_ENABLED | TMR_TMR16B1PWMC_PWM1_ENABLED;

    TMR_TMR16B1PR = 47;    //set prescaler
    // to 48 to allow 20000 range

    TMR_TMR16B1MR3 = 20000;    //set full range value

    TMR_TMR16B1MR1 = 20000-1500;

    TMR_TMR16B1MCR = TMR_TMR16B1MCR_MR3_RESET_ENABLED;//RESET ON MR3


    /************   TMR32B0   **************/
    timer_old = 0;
    PPM_Counter = 0;
    radio_status = 0;
    TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_DISABLED;
    SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_CT32B0;

    IOCON_PIO1_5 = IOCON_PIO1_5_FUNC_CT32B0_CAP0;

    TMR_TMR32B0PR = 47;

    /* capture on rising edge  Interrupt on event*/
    TMR_TMR32B0CCR = TMR_TMR32B0CCR_CAP0RE_ENABLED | TMR_TMR32B0CCR_CAP0I_ENABLED;
    NVIC_EnableIRQ(TIMER_32_0_IRQn);





    TMR_TMR16B0TCR = TMR_TMR16B0TCR_COUNTERENABLE_ENABLED;
    TMR_TMR16B1TCR = TMR_TMR16B1TCR_COUNTERENABLE_ENABLED;
    TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_ENABLED;

}


/*
 * I need to add in something to set radio_status to 0 when
 * too much time passes
 */
void TIMER32_0_IRQHandler(void)
{
    uint32_t Pulse;
    uint32_t Pulse_Width;
    //printf("Pulse_width = %d \n",TMR_TMR32B0CR0-timer_old);

    Pulse = TMR_TMR32B0CR0;
    Pulse_Width = Pulse - timer_old;

    if (Pulse_Width > 4000) /* Sync Pulse */
        PPM_Counter = 0;
    else
    {
        PPM_Counter &= 0x7;
        PWM_RAW[PPM_Counter++]=Pulse_Width;
        if (PPM_Counter >= NUM_CHANNELS) 
            radio_status = 1;
    }
    timer_old = Pulse;

    TMR_TMR32B0IR = TMR_TMR32B0IR_CR0;
}


void setRC(uint8_t ch , int pwm)
{
    pwm = 20000 - constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
    switch(ch)
    {
        case 0:
            TMR_TMR16B0MR0 = pwm;
            break;

        case 1:
            TMR_TMR16B0MR1 = pwm;
            break;

        case 2:
            TMR_TMR16B0MR2 = pwm;
            break;

        case 3:
            TMR_TMR16B1MR1 = pwm;
            break;         
    }
}


//PPM input on CT32B0_CAP0  PIO1_5
unsigned int inputPPM(uint8_t ch)
{
    return PWM_RAW[ch];
}
int validPPM()
{
    return radio_status;
}




//
