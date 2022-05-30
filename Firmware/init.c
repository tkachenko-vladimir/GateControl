#include "stm8s.h"
#include "iostm8s105c6.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "osa.h"
#include "stm8s_adc1.h"
#include "main.h"

void
init_gpio(void)
{
  PB_DDR_bit.DDR0 = 0;//VBAT_SENSE
  PB_CR1_bit.C10 = 0;
  PB_CR2_bit.C20 = 0;

  PB_DDR_bit.DDR1 = 0;//VCC_SENSE
  PB_CR1_bit.C11 = 0;
  PB_CR2_bit.C21 = 0;

  PB_DDR_bit.DDR2 = 0;//I_SENSE
  PB_CR1_bit.C12 = 0;
  PB_CR2_bit.C22 = 0;

  PB_DDR_bit.DDR3 = 0;//FIRE
  PB_CR1_bit.C13 = 0;
  PB_CR2_bit.C23 = 0;

  PA_DDR_bit.DDR4 = 1;//U_BAT_SW
  PA_CR1_bit.C14 = 1;
  PA_CR2_bit.C24 = 1;

  PA_DDR_bit.DDR5 = 1;//LD_BAT_SW
  PA_CR1_bit.C15 = 1;
  PA_CR2_bit.C25 = 1;

  PA_DDR_bit.DDR6 = 1;//CHRG_SW
  PA_CR1_bit.C16 = 1;
  PA_CR2_bit.C26 = 1;

  PA_DDR_bit.DDR3 = 0;//BTN_OPEN
  PA_CR1_bit.C13 = 0;
  PA_CR2_bit.C23 = 0;

  PB_DDR_bit.DDR4 = 0;//BTN_CLOSE
  PB_CR1_bit.C14 = 0;
  PB_CR2_bit.C24 = 0;

  PB_DDR_bit.DDR5 = 0;//BTN_STOP
  PB_CR1_bit.C15 = 0;
  PB_CR2_bit.C25 = 0;

  PB_DDR_bit.DDR6 = 0;//C_LEFT
  PB_CR1_bit.C16 = 0;
  PB_CR2_bit.C26 = 0;

  PB_DDR_bit.DDR7 = 0;//C_RIGHT
  PB_CR1_bit.C17 = 0;
  PB_CR2_bit.C27 = 0;

  PE_DDR_bit.DDR7 = 0;//BTN_PB
  PE_CR1_bit.C17 = 0;
  PE_CR2_bit.C27 = 0;

  PE_DDR_bit.DDR6 = 0;//PHOTO1
  PE_CR1_bit.C16 = 0;
  PE_CR2_bit.C26 = 0;

  PE_DDR_bit.DDR5 = 0;//PHOTO2
  PE_CR1_bit.C15 = 0;
  PE_CR2_bit.C25 = 0;

  PG_DDR_bit.DDR0 = 0;//M_RIGHT
  PG_CR1_bit.C10 = 0;
  PG_CR2_bit.C20 = 0;

  PG_DDR_bit.DDR1 = 0;//NOFIRECONTROL
  PG_CR1_bit.C11 = 0;
  PG_CR2_bit.C21 = 0;

  PC_DDR_bit.DDR3 = 0;//LOWSPEED
  PC_CR1_bit.C13 = 0;
  PC_CR2_bit.C23 = 0;

  PC_DDR_bit.DDR4 = 0;//DIP4
  PC_CR1_bit.C14 = 0;
  PC_CR2_bit.C24 = 0;

  PC_DDR_bit.DDR5 = 0;//DIP5
  PC_CR1_bit.C15 = 0;
  PC_CR2_bit.C25 = 0;

  PC_DDR_bit.DDR6 = 0;//DIP6
  PC_CR1_bit.C16 = 0;
  PC_CR2_bit.C26 = 0;

  PC_DDR_bit.DDR7 = 1;//K_IN_FLT
  PC_CR1_bit.C17 = 1;
  PC_CR2_bit.C27 = 1;

  PE_DDR_bit.DDR0 = 1;//K_BAT_FLT
  PE_CR1_bit.C10 = 1;
  PE_CR2_bit.C20 = 1;

  PD_DDR_bit.DDR3 = 1;//OUT_FLT_OTHER
  PD_CR1_bit.C13 = 1;
  PD_CR2_bit.C23 = 1;

  PD_DDR_bit.DDR2 = 1;//OUT_CLOSED
  PD_CR1_bit.C12 = 1;
  PD_CR2_bit.C22 = 1;

  PD_DDR_bit.DDR4 = 1;//M_PWM
  PD_CR1_bit.C14 = 1;
  PD_CR2_bit.C24 = 1;

  PC_DDR_bit.DDR1 = 1;//OUT_LIGHT
  PC_CR1_bit.C11 = 1;
  PC_CR2_bit.C21 = 1;

  PC_DDR_bit.DDR2 = 1;//M_FORW
  PC_CR1_bit.C12 = 1;
  PC_CR2_bit.C22 = 1;

  PD_DDR_bit.DDR0 = 1;//M_BACK
  PD_CR1_bit.C10 = 1;
  PD_CR2_bit.C20 = 1;

  PE_DDR_bit.DDR3 = 0;//BTN1
  PE_CR1_bit.C13 = 0;
  PE_CR2_bit.C23 = 0;

  PD_DDR_bit.DDR7 = 1;//HL_BAT_OK
  PD_CR1_bit.C17 = 1;
  PD_CR2_bit.C27 = 1;
}

void
init_timers(void)
{
  TIM2_PSCR = 0x08;       //  Prescaler = 8.
  TIM2_ARRH = 0x01;       //  High byte of 50,000.
  TIM2_ARRL = 0x5E;       //  Low byte of 50,000.
  TIM2_CCR1H = 0x00;      //  High byte of 12,500
  TIM2_CCR1L = 0x00;      //  Low byte of 12,500
  TIM2_CCER1_CC1P = 0;    //  Active high.
  TIM2_CCER1_CC1E = 1;    //  Enable compare mode for channel 1
  TIM2_CCMR1_OC1M = 6;    //  PWM Mode 1 - active if counter < CCR1, inactive otherwise.
  TIM2_CR1_CEN = 1;       //  Finally enable the timer.

  TIM4_SR_bit.UIF = 0;
  TIM4_PSCR = 7;
  TIM4_ARR = 124;
  TIM4_IER_bit.UIE = 1;
  TIM4_CR1_bit.CEN = 1;
}

void
init_adc(void)
{
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
  ADC1_DeInit();
//  ADC1_ScanModeCmd(ENABLE);
//  ADC1_DataBufferCmd(ENABLE);
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
            ADC1_CHANNEL_2,
            ADC1_PRESSEL_FCPU_D18,
            ADC1_EXTTRIG_TIM,
            DISABLE,
            ADC1_ALIGN_RIGHT,
            ADC1_SCHMITTTRIG_CHANNEL0 | ADC1_SCHMITTTRIG_CHANNEL1 | ADC1_SCHMITTTRIG_CHANNEL2 | ADC1_SCHMITTTRIG_CHANNEL3 | ADC1_SCHMITTTRIG_CHANNEL8,
            DISABLE);
   ADC1_Cmd(ENABLE);
   ADC1_ClearITPendingBit(ADC1_IT_EOC);
   ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
}

void
InitialiseIWDG(void)
{
  IWDG_KR = 0xCC;         //  Start the independent watchdog.
  IWDG_KR = 0x55;         //  Allow the IWDG registers to be programmed.
  IWDG_PR = 0x06;         //  Prescaler is 2 => each count is 250uS
  IWDG_RLR = 0xFF;        //  Reload counter.
  IWDG_KR = 0xAA;         //  Reset the counter.
}