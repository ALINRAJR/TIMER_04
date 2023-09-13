


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
#include "powercontrol.h"
#include "clock.h"
#include "timer.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Function pointer object declaration */
timerisrfuncptr vTimerintisr0= NULL;
timerisrfuncptr vTimerintisr1= NULL;
timerisrfuncptr vTimerintisr2= NULL;
timerisrfuncptr vTimerintisr3= NULL;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief     Setting power ON/OFF for Timer Peripheral
  * @param[in] ucUARTNum value can be TIMER_0,TIMER_1,TIMER_2,TIMER_3
  * @param[in] mode value can be either POWERON/ POWEROFF
  * @return    No return value
  **/
void vTimerPowerControl(uint8_t ucTimerNum,uint8_t mode)
{
	switch(ucTimerNum)
	{
	case TIMER_0:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCTIM0;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCTIM0);
		break;
	case TIMER_1:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCTIM1;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCTIM1);
		break;
	case TIMER_2:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCTIM2;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCTIM2);
		break;
	case TIMER_3:
		if (mode == POWERON)
			LPC_SC->PCONP |= 1 << BIT_PCONP_PCTIM3;
		else if (mode == POWEROFF)
			LPC_SC->PCONP &= ~(1 << BIT_PCONP_PCTIM3);
		break;
	default :
		/* DO NOTHING */
		break;
	}
}

/**
 * @brief     Setting clock for Timer Peripheral
 * @param[in] ucUARTNum value can be TIMER_0,TIMER_1,TIMER_2,TIMER_3
 * @param[in] clockmode value can be ONEFOURTH,SAME,HALF,ONEEIGTH
 * @return    No return value
 **/
void vTimerClockControl(uint8_t ucTimerNum,uint8_t clockmode)
{
	switch(ucTimerNum)
	{
	case TIMER_0:
		LPC_SC->PCLKSEL0 &= ~(3 << BIT_PCLKSEL0_PCLK_TIMER0);
		LPC_SC->PCLKSEL0 |= clockmode << BIT_PCLKSEL0_PCLK_TIMER0;
		break;
	case TIMER_1:
		LPC_SC->PCLKSEL0 &= ~(3 << BIT_PCLKSEL0_PCLK_TIMER1);
		LPC_SC->PCLKSEL0 |= clockmode << BIT_PCLKSEL0_PCLK_TIMER1;
		break;
	case TIMER_2:
		LPC_SC->PCLKSEL1 &= ~(3 << BIT_PCLKSEL1_PCLK_TIMER2);
		LPC_SC->PCLKSEL1 |= clockmode << BIT_PCLKSEL1_PCLK_TIMER2;
		break;
	case TIMER_3:
		LPC_SC->PCLKSEL1 &= ~(3 << BIT_PCLKSEL1_PCLK_TIMER3);
		LPC_SC->PCLKSEL1 |= clockmode << BIT_PCLKSEL1_PCLK_TIMER3;
		break;
	default :
		/* DO NOTHING */
		break;
	}
}

/**
 * @brief     Initializing the selected TIMER Peripheral
 * @param[in] ucTIMERNum value can be TIMER_0, TIMER_1, TIMER_2, TIMER_3
 * @return    No return value
 **/

void vTimerInitialize(uint8_t ucTIMERNum)
{

	/* Step 1: Power ON/OFF for the selected Timer Peripheral */
		vTimerPowerControl(TIMER_0,POWERON);
	/* Step 2: Setting up the clock for the selected Timer Peripheral as 25MHz */
		vTimerClockControl(TIMER_0,ONEFOURTH);
	/* Step 3: Pin Initialize */
		/* For this program Timer Pins initialization not required */
	/* Step 4: Interrupt enabling */
		/* For this program we are using interrupt method it is handled in vAppHardwareInit */

}

/**
 * @brief     Selecting timer mode or counter mode for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] mode value can be TIMER_MODE,TC_INCREMENT_RISING_EDGE,TC_INCREMENT_FALLING_EDGE,TC_INCREMENT_BOTH_EDGE
 * @return    No return value
 **/
void vTimerCountControl(LPC_TIM_TypeDef* TIMERx, uint8_t mode)
{
	  TIMERx->CTCR = mode<<BIT_CTCR_COUNTER_TIMER_MODE;
}

/**
 * @brief     Selecting cap pin from which the counting to be done for counter mode for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] countinputsel value can be CAPn_0=0,CAPn_1
 * @return    No return value
 **/
void vTimerCountControlInpSel(LPC_TIM_TypeDef* TIMERx, uint8_t countinputsel)
{
	  TIMERx->CTCR &= ~(3<<BIT_CTCR_COUNTER_INPUT_SELECT);
	  TIMERx->CTCR |= countinputsel<<BIT_CTCR_COUNTER_INPUT_SELECT;
}

/**
 * @brief     Enabling  the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] mode value can be TIMER_COUNTER_DISABLE,TIMER_COUNTER_ENABLE
 * @return    No return value
 **/
void vTimerControl(LPC_TIM_TypeDef* TIMERx,uint8_t mode)
{
	if(mode ==TIMER_COUNTER_ENABLE)
		TIMERx->TCR = 1<<BIT_TCR_COUNTER_ENABLE;
	else if (mode ==TIMER_COUNTER_DISABLE)
		TIMERx->TCR = 0<<BIT_TCR_COUNTER_ENABLE;
}

/**
 * @brief     Resetting the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @return    No return value
 **/
void vTimerControlReset(LPC_TIM_TypeDef* TIMERx)
{
	TIMERx->TCR = 1<<BIT_TCR_COUNTER_RESET;
	TIMERx->TCR = 0<<BIT_TCR_COUNTER_RESET;
}

/**
 * @brief     Setting the prescalar value for  the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] prescale value can be PRESCALAR_US,PRESCALAR_MS,PRESCALAR_SEC
 * @return    No return value
 **/
void vTimerPrescalarSel(LPC_TIM_TypeDef* TIMERx,uint32_t prescale)
{
   TIMERx->PR =prescale-1;
}

/**
 * @brief     Generating delay in milliseconds using polling method for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] delayinms value can be 32 bit integer value
 * @return    No return value
 **/
void vTimerDelayinMs(LPC_TIM_TypeDef* TIMERx,uint32_t delayinms)
{
	 /* Step1: Setting the Timer in TIMER MODE for TC to count every PCLK (internal clock) pulses*/
	 vTimerCountControl(TIMERx,TIMER_MODE);
	 /* Resetting the timer before starting to count from 0*/
	 vTimerControlReset(TIMERx);
	 /* TC will increment for every 1 ms */
	 vTimerPrescalarSel(TIMERx,PRESCALAR_MS);

	 /* Enabling the timer and counting starts */
	 vTimerControl(TIMERx,TIMER_COUNTER_ENABLE);
	 /* Wait until timer count register matches with the delayinms (polling method) */
	 while(TIMERx->TC<delayinms);
	 /* Resetting the timer before starting to count from 0*/
	 vTimerControlReset(TIMERx);
	 /* Disabling the timer and counting stops */
	 vTimerControl(TIMERx,TIMER_COUNTER_DISABLE);

}
/**
 * @brief     Generating delay in microseconds using polling method for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] delayinus value can be 32 bit integer value
 * @return    No return value
 **/
void vTimerDelayinUs(LPC_TIM_TypeDef* TIMERx,uint32_t delayinus)
{
	 /* Step1: Setting the Timer in TIMER MODE for TC to count every PCLK (internal clock) pulses*/
		 vTimerCountControl(TIMERx,TIMER_MODE);
		 /* Resetting the timer before starting to count from 0*/
		 vTimerControlReset(TIMERx);
		 /* TC will increment for every 1 us */
		 vTimerPrescalarSel(TIMERx,PRESCALAR_US);

		 /* Enabling the timer and counting starts */
		 vTimerControl(TIMERx,TIMER_COUNTER_ENABLE);
		 /* Wait until timer count register matches with the delayinms (polling method) */
		 while(TIMERx->TC<delayinus);
		 /* Resetting the timer before starting to count from 0*/
		 vTimerControlReset(TIMERx);
		 /* Disabling the timer and counting stops */
		 vTimerControl(TIMERx,TIMER_COUNTER_DISABLE);

}
/**
 * @brief     Generating delay in seconds using polling method for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] delayinsecs value can be 32 bit integer value
 * @return    No return value
 **/
void vTimerDelayinSecs(LPC_TIM_TypeDef* TIMERx,uint32_t delayinsecs)
{
	 /* Step1: Setting the Timer in TIMER MODE for TC to count every PCLK (internal clock) pulses*/
		 vTimerCountControl(TIMERx,TIMER_MODE);
		 /* Resetting the timer before starting to count from 0*/
		 vTimerControlReset(TIMERx);
		 /* TC will increment for every 1 sec */
		 vTimerPrescalarSel(TIMERx,PRESCALAR_SEC);

		 /* Enabling the timer and counting starts */
		 vTimerControl(TIMERx,TIMER_COUNTER_ENABLE);
		 /* Wait until timer count register matches with the delayinms (polling method) */
		 while(TIMERx->TC<delayinsecs);
		 /* Resetting the timer before starting to count from 0*/
		 vTimerControlReset(TIMERx);
		 /* Disabling the timer and counting stops */
		 vTimerControl(TIMERx,TIMER_COUNTER_DISABLE);

}



/**
  * @brief     Reseting the respective Match Register for the selected timer
  * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
  * @param[in] matchsource value can be MATCH0,MATCH1,MATCH2,MATCH3
  **/
void vTimerMatchReset(LPC_TIM_TypeDef* TIMERx,uint8_t matchsource)
{
	switch (matchsource) {
				case MATCH0:
					TIMERx->MCR |= 1<<BIT_MCR_MR0R;
					break;
				case MATCH1:
					TIMERx->MCR |= 1<<BIT_MCR_MR1R;
					break;
				case MATCH2:
					TIMERx->MCR |= 1<<BIT_MCR_MR2R;
					break;
				case MATCH3:
					TIMERx->MCR |= 1<<BIT_MCR_MR3R;
					break;
	}
}

/**
 * @brief     Stopping the respective Match Register for the selected timer
 * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] matchsource value can be MATCH0,MATCH1,MATCH2,MATCH3
 **/
void vTimerMatchStop(LPC_TIM_TypeDef* TIMERx, uint8_t matchsource) {
	switch (matchsource) {
	case MATCH0:
		TIMERx->MCR |= 1 << BIT_MCR_MR0S;
		break;
	case MATCH1:
		TIMERx->MCR |= 1 << BIT_MCR_MR1S;
		break;
	case MATCH2:
		TIMERx->MCR |= 1 << BIT_MCR_MR2S;
		break;
	case MATCH3:
		TIMERx->MCR |= 1 << BIT_MCR_MR3S;
		break;
	}
}

/**
 * @brief     Loading the respective Match Register for the selected timer
 * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] matchsource value can be MATCH0,MATCH1,MATCH2,MATCH3
 * @param[in] loadvalue value can be 32bit integer value
 **/
void vTimerLoadMatchRegister(LPC_TIM_TypeDef* TIMERx, uint8_t matchsource, uint32_t loadvalue)
{

	switch (matchsource) {
		case MATCH0:
			TIMERx->MR0 = loadvalue;
			break;
		case MATCH1:
			TIMERx->MR1 = loadvalue;
			break;
		case MATCH2:
			TIMERx->MR2 = loadvalue;
			break;
		case MATCH3:
			TIMERx->MR3 = loadvalue;
			break;
		}
}

/**
  * @brief     Enabling Interrupt for the Timer and the source for interrupt
  * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
  * @param[in] interruptsource value can be MATCH0,MATCH1,MATCH2,MATCH3,CAPTURE0,CAPTURE1
  * @return    No return value
  **/
void vTimerInterruptEnable(LPC_TIM_TypeDef* TIMERx,uint8_t matchsource)
{
		switch (matchsource) {
			case MATCH0:
				TIMERx->MCR |= 1<<BIT_MCR_MR0I;
				break;
			case MATCH1:
				TIMERx->MCR |= 1<<BIT_MCR_MR1I;
				break;
			case MATCH2:
				TIMERx->MCR |= 1<<BIT_MCR_MR2I;
				break;
			case MATCH3:
				TIMERx->MCR |= 1<<BIT_MCR_MR3I;
				break;
			case CAPTURE0:
				TIMERx->CCR |= 1<<BIT_CCR_CAP0I;
				break;
			case CAPTURE1:
				TIMERx->CCR |= 1<<BIT_CCR_CAP1I;
				break;
			default:
				break;
		}
}


/**
  * @brief     Disabling Interrupt for the Timer and the source for interrupt
  * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
  * @param[in] interruptsource value can be MATCH0INT,MATCH1INT,MATCH2INT,MATCH3INT,CAPTURE0INT,CAPTURE1INT
  * @return    No return value
  **/
void vTimerInterruptDisable(LPC_TIM_TypeDef* TIMERx,uint8_t interruptsource)
{
	switch (interruptsource) {
				case MATCH0:
					TIMERx->MCR &= ~(1<<BIT_MCR_MR0I);
					break;
				case MATCH1:
					TIMERx->MCR &= ~(1<<BIT_MCR_MR1I);
					break;
				case MATCH2:
					TIMERx->MCR &= ~(1<<BIT_MCR_MR2I);
					break;
				case MATCH3:
					TIMERx->MCR &= ~(1<<BIT_MCR_MR3I);
					break;
				case CAPTURE0:
					TIMERx->CCR &= ~(1<<BIT_CCR_CAP0I);
					break;
				case CAPTURE1:
					TIMERx->CCR &= ~(1<<BIT_CCR_CAP1I);
					break;
				default:
					break;
			}

}



/**
 * @brief     Timer interrupt call back registration
 * @param[in] ucTIMERNum value can be TIMER_0, TIMER_1, TIMER_2, TIMER_3
 * @param[in] vTimerintisrobject address of the function which should be called when the respective interrupt occurs
 * @return    No return value
 **/

void vTimerIntAttachCallback(uint8_t ucTIMERNum, timerisrfuncptr vTimerintisrobject)
{
		switch(ucTIMERNum)
		{
		case TIMER_0:
			vTimerintisr0 = vTimerintisrobject;
			break;
		case TIMER_1:
			vTimerintisr1 = vTimerintisrobject;
			break;
		case TIMER_2:
			vTimerintisr2 = vTimerintisrobject;
			break;
		case TIMER_3:
			vTimerintisr3 = vTimerintisrobject;
			break;
			default :
			/* DO NOTHING */
			break;

		}
}

/**
  * @brief  TiMER0 service routine
  * @retval none
  */
void TIMER0_IRQHandler(void)
{
	uint32_t timerintidentificationreg;
	timerintidentificationreg= LPC_TIM0->IR;
    /* Calling back the attached function to this function pointer */
	vTimerintisr0(timerintidentificationreg);
	/* For re-enabling the timer interrupt for next time*/
	LPC_TIM0->IR=timerintidentificationreg;
}
/**
  * @brief  TiMER1 service routine
  * @retval none
  */
void TIMER1_IRQHandler(void)
{
	uint32_t timerintidentificationreg;
	timerintidentificationreg= LPC_TIM1->IR;
	/* Calling back the attached function to this function pointer */
	vTimerintisr1(timerintidentificationreg);
	/* For re-enabling the timer interrupt for next time*/
	LPC_TIM1->IR=timerintidentificationreg;
}
/**
  * @brief  TiMER2 service routine
  * @retval none
  */
void TIMER2_IRQHandler(void)
{
	uint32_t timerintidentificationreg;
	timerintidentificationreg= LPC_TIM2->IR;
	/* Calling back the attached function to this function pointer */
	vTimerintisr2(timerintidentificationreg);
	/* For re-enabling the timer interrupt for next time*/
		LPC_TIM2->IR=timerintidentificationreg;
}

/**
  * @brief  TiMER3 service routine
  * @retval none
  */
void TIMER3_IRQHandler(void)
{
	uint32_t timerintidentificationreg;
		timerintidentificationreg= LPC_TIM3->IR;
	/* Calling back the attached function to this function pointer */
	vTimerintisr3(timerintidentificationreg);
	/* For re-enabling the timer interrupt for next time*/
		LPC_TIM3->IR=timerintidentificationreg;

}
