

/* Includes ------------------------------------------------------------------*/
#include <LPC17xx.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
#include "extint.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Function pointer object declarations */
extintfuncptr vExtintisr0= NULL;
extintfuncptr vExtintisr1= NULL;
extintfuncptr vExtintisr2= NULL;
extintfuncptr vExtintisr3= NULL;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief     Enabling the interrupt and for clearing/re enabling the interrupt
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @param[in] mode  value can be EXTINTENABLE or EXTINTDISABLE
 * @return    No return value
 **/
void vExtIntSel(uint8_t ucExtintnum, uint8_t mode)
{
	if(mode==EXTINTENABLE)
		LPC_SC->EXTINT |= 1<<ucExtintnum;
	else if(mode==EXTINTDISABLE)
		LPC_SC->EXTINT &= ~(1<<ucExtintnum);

}

/**
 * @brief     Setting the mode for the external interrupt as level or edge
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @param[in] ucExtintmode value can be LEVEL_SENSITIVE, EDGE_SENSITIVE
 * @return    No return value
 **/
void vExtIntMode(uint8_t ucExtintnum, uint8_t ucExtintmode)
{
	if(ucExtintmode==EDGE_SENSITIVE)
		LPC_SC->EXTMODE |= 1<<ucExtintnum;
	else if(ucExtintmode==LEVEL_SENSITIVE)
		LPC_SC->EXTMODE &= ~(1<<ucExtintnum);
}

/**
 * @brief     Setting the polarity for the external interrupt as low/high
 * 			  for level triggered and falling/rising for edge triggered
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @param[in] ucExtintpolarity value can be LOW_LEVEL,HIGH_LEVEL,FALLING_EDGE,RISING_EDGE
 * @return    No return value
 **/
void vExtIntPolarity(uint8_t ucExtintnum, uint8_t ucExtintpolarity)
{
	if(ucExtintpolarity==HIGH_LEVEL || ucExtintpolarity== RISING_EDGE)
		LPC_SC->EXTPOLAR |= 1<<ucExtintnum;
	else if(ucExtintpolarity==LOW_LEVEL ||ucExtintpolarity== FALLING_EDGE)
		LPC_SC->EXTPOLAR &= ~(1<<ucExtintnum);
}

/**
 * @brief     External interrupt Initialization
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @return    No return value
 **/

void vExtIntInitialize(uint8_t ucExtintnum)
{
    /*  Step 1:  Enabling the selected external interrupt */
	vExtIntSel(ucExtintnum,EXTINTENABLE);

#if TESTCASE0
	/* Step 2: Selecting whether the interrupt should occur for level trigger or edge trigger */
	vExtIntMode(ucExtintnum,LEVEL_SENSITIVE);
	/* Step 3: Selecting whether the interrupt should occur for low level or high level if mode is level triggered
	 *         or falling edge or rising edge if mode is edge triggered */
	vExtIntPolarity(ucExtintnum,HIGH_LEVEL);
#endif

#if TESTCASE1
	/* Step 2: Selecting whether the interrupt should occur for level trigger or edge trigger */
	vExtIntMode(ucExtintnum,LEVEL_SENSITIVE);
	/* Step 3: Selecting whether the interrupt should occur for low level or high level if mode is level triggered
	 *         or falling edge or rising edge if mode is edge triggered */
	vExtIntPolarity(ucExtintnum,LOW_LEVEL);
#endif

#if TESTCASE2
	/* Step 2: Selecting whether the interrupt should occur for level trigger or edge trigger */
	vExtIntMode(ucExtintnum,EDGE_SENSITIVE);
	/* Step 3: Selecting whether the interrupt should occur for low level or high level if mode is level triggered
	 *         or falling edge or rising edge if mode is edge triggered */
	vExtIntPolarity(ucExtintnum,RISING_EDGE);
#endif

#if TESTCASE3
	/* Step 2: Selecting whether the interrupt should occur for level trigger or edge trigger */
	vExtIntMode(ucExtintnum,EDGE_SENSITIVE);
	/* Step 3: Selecting whether the interrupt should occur for low level or high level if mode is level triggered
	 *         or falling edge or rising edge if mode is edge triggered */
	vExtIntPolarity(ucExtintnum,FALLING_EDGE);
#endif

	/* Step 4: Enabling the selected external interrupt in NVIC */
	switch(ucExtintnum)
	{
		case EINT0:
			NVIC_EnableIRQ(EINT0_IRQn);
			break;
		case EINT1:
			NVIC_EnableIRQ(EINT1_IRQn);
			break;
		case EINT2:
			NVIC_EnableIRQ(EINT2_IRQn);
			break;
		case EINT3:
			NVIC_EnableIRQ(EINT3_IRQn);
			break;
		default:
			/* DO NOTHING */
			break;
	}

}

/**
 * @brief     External interrupt call back registration
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @param[in] vExtintisrobject address of the function which should be called when the respective interrupt occurs
 * @return    No return value
 **/

void vExtIntAttachCallback(uint8_t ucExtintnum, extintfuncptr vExtintisrobject)
{
	/* Attaching the function address with respective function pointer objects */
	switch(ucExtintnum)
		{
			case EINT0:
				vExtintisr0 = vExtintisrobject;
				break;
			case EINT1:
				vExtintisr1 = vExtintisrobject;
				break;
			case EINT2:
				vExtintisr2 = vExtintisrobject;
				break;
			case EINT3:
				vExtintisr3 = vExtintisrobject;
				break;
			default:
				/* DO NOTHING */
				break;
		}
}

/**
  * @brief  External Interrupt (EINT0) service routine
  * @retval none
  */
void EINT0_IRQHandler(void)
{

	/* For re enabling for next interrupt or clearing the current interrupt */
	vExtIntSel(EINT0,EXTINTENABLE);
	/* Callback function pointer */
	vExtintisr0();

}

/**
  * @brief  External Interrupt (EINT1) service routine
  * @retval none
  */
void EINT1_IRQHandler(void)
{
	/* For re enabling for next interrupt or clearing the current interrupt */
	vExtIntSel(EINT1,EXTINTENABLE);
	/* Callback function pointer */
	vExtintisr1();
}


/**
  * @brief  External Interrupt (EINT2) service routine
  * @retval none
  */
void EINT2_IRQHandler(void)
{
	/* For re enabling for next interrupt or clearing the current interrupt */
	vExtIntSel(EINT2,EXTINTENABLE);
	/* Callback function pointer */
	vExtintisr2();
}

/**
  * @brief  External Interrupt (EINT3) service routine
  * @retval none
  */
void EINT3_IRQHandler(void)
{
	/* For re enabling for next interrupt or clearing the current interrupt */
	vExtIntSel(EINT3,EXTINTENABLE);
	/* Callback function pointer */
	vExtintisr3();
}





