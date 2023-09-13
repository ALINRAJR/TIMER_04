/*
===============================================================================
 Name        : 33_Day70_29112021_TIMER004.c

 Description : Program to demonstrate the state machine using button and timer

	 Input State/function	Inputevent	    Output State/function
	**********************************************************
	LED Off					Timeout			None
	LED Off	                Button Press	Turn on LED
	LED On 					Timeout	        Turn off LED
	LED On	                Button Press	Start blinking LED
	LED Blink				Timeout	        Turn off LED
	LED Blink	       		Button Press	Turn off LED

 
	**********************************************************
 Layered Architecture used for this project
 ************************************
 Application layer-33_Day70_29112021_TIMER004.c
 *******************************************************************
 Board layer -  configboard.h, led.c/.h, buttonint.c/.h
 *******************************************************************
 Low level drivers or chip level - pinmux.c/.h,timer.c/.h, gpio.c/.h
 	 	 	 	 	 	 	 	   extint.c/.h
 *******************************************************************
 Hardware
 *******************************************************************
===============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
#include "pinmux.h"
#include "timer.h"
#include "led.h"
#include "buttonint.h"
#include "swqueue.h"

/* Private typedef -----------------------------------------------------------*/
//typedef enum {LEDOFF=0,LEDON,LEDBLINK}states_t;
typedef enum {NONE=0,TIMEOUT,BUTTONPRESS}inputs_t;
/* Private define ------------------------------------------------------------*/

/* Indicates the Input event's data is one byte */
#define INPUTEVENTSIZE              1

/* Input buffer size for queuing the inputs */
#define INPUTBUFFERSIZE             15

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Buffer for input event queue object*/
uint8_t InputEventBuffer[INPUTBUFFERSIZE];

//volatile states_t currentstate =LEDOFF;
volatile inputs_t currentinput =NONE;

/* Input event object */
swqueue_t inputeventqueueobj;

/* Function pointer to handle the states/output functions */
typedef void (*ptrstatehandler)(inputs_t currentinput);

/* State machine to handle the input events and outputs */
typedef struct statemachine{
	/* Function pointer object to handle the current states/output functions */
	ptrstatehandler currentstatehandler;
	/* Pointer to software queue object for input events*/
	swqueue_t* ptrinputeventqueueobj;
}statemachine_t;


/* Private function prototypes -----------------------------------------------*/
void vLedOffStatefunction(inputs_t currentinput);
void vLedOnStatefunction(inputs_t currentinput);
void vLedBlinkStatefunction(inputs_t currentinput);


/* Private user code ---------------------------------------------------------*/

/* State machine object declaration and initialization */
statemachine_t statemachineobj= {.currentstatehandler    =  vLedOffStatefunction,
								 .ptrinputeventqueueobj  =  &inputeventqueueobj
								};

/**
  * @brief  Timeout Interrupt handler for TIMER0
  * @retval none
  */

void vTimeoutHandler(uint32_t interruptsource)
{
		switch (interruptsource) {
		case MR0INT:
			/*Queuing the inputs */
			ucswqueuewrite(&inputeventqueueobj,TIMEOUT);
			break;
		case MR1INT:
			/* DO NOTHING*/
			break;
		case MR2INT:
			/* DO NOTHING*/
			break;
		case MR3INT:
			/* DO NOTHING*/
			break;
		case CAP0INT:
			/* DO NOTHING*/
			break;
		case CAP1INT:
			/* DO NOTHING*/
			break;
		}
}

/**
  * @brief  Button interrupt Handler
  * @retval none
  */
void vButtonIntHandler(void)
{
	/*Queuing the inputs */
	ucswqueuewrite(&inputeventqueueobj, BUTTONPRESS);
}

/**
  * @brief  Led blink timer handler for TIMER1
  * @retval none
  */

void vLedBlinkHandler(uint32_t interruptsource)
{
		switch (interruptsource) {
		case MR0INT:
			vLedToggle(LED_0);
			break;
		case MR1INT:
			/* DO NOTHING*/
			break;
		case MR2INT:
			/* DO NOTHING*/
			break;
		case MR3INT:
			/* DO NOTHING*/
			break;
		case CAP0INT:
			/* DO NOTHING*/
			break;
		case CAP1INT:
			/* DO NOTHING*/
			break;
		}
}



/**
  * @brief  Initialize all the hardware connected
  * @retval none
  */
void vAppHardwareInit(void)
{
	vPinmuxInitialize();
	vLedInitialize();
	vButtonIntInitialize();

	/*Initializing the Event/input handler queue object */
	vswqueueinitialize(&inputeventqueueobj, InputEventBuffer, INPUTEVENTSIZE,INPUTBUFFERSIZE);

	/********************Configuring TIMER0 for Timeout Handler**************/
	/* Basic configurations of Timer */
	vTimerInitialize(TIMER_0);
	/* Configuring the timer in Timer Mode */
	vTimerCountControl(TIMER0,TIMER_MODE);
	/* Setting the prescalar for making TC count every 1ms */
	vTimerPrescalarSel(TIMER0,PRESCALAR_MS);
	/* Load the respective match registers with required delay in ms */
	vTimerLoadMatchRegister(TIMER0,MATCH0,10000);
	/* Enabling to Reset the timer when TC matches with MR0 register value */
	vTimerMatchReset(TIMER0,MATCH0);

	/* Call back registration of vTimeoutHandler with Timer IRQ Handler */
	vTimerIntAttachCallback(TIMER_0,vTimeoutHandler);
	/* Enabling interrupt for all match register 0 */
	vTimerInterruptEnable(TIMER0,MATCH0);
	/* Enabling the Timer0 interrupt in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);
	/********************************************************************/
	/********************Configuring TIMER1 for Led Blink Handler**************/
	/* Basic configurations of Timer */
	vTimerInitialize(TIMER_1);
	/* Configuring the timer in Timer Mode */
	vTimerCountControl(TIMER1,TIMER_MODE);
	/* Setting the prescalar for making TC count every 1ms */
	vTimerPrescalarSel(TIMER1,PRESCALAR_MS);
	/* Load the respective match registers with required delay in ms */
	vTimerLoadMatchRegister(TIMER1,MATCH0,500);
	/* Enabling to Stop the timer when TC matches with MR0 register value */
	vTimerMatchReset(TIMER1,MATCH0);

	/* Call back registration of vLedBlinkHandler with Timer IRQ Handler */
	vTimerIntAttachCallback(TIMER_1,vLedBlinkHandler);
	/* Enabling interrupt for all match register 0 */
	vTimerInterruptEnable(TIMER1,MATCH0);
	/* Enabling the Timer0 interrupt in NVIC */
	NVIC_EnableIRQ(TIMER1_IRQn);
	/********************************************************************/
}

/**
  * @brief  Starting the timer for Timeout input
  * @retval none
  */
void vTimeoutTimerStart(void)
{
	/* Resetting the Timer */
    vTimerControlReset(TIMER0);
	/* Enabling the timer and counting starts */
	vTimerControl(TIMER0,TIMER_COUNTER_ENABLE);
}
/**
  * @brief  Stopping the timer of Timeout
  * @retval none
  */
void vTimeoutTimerStop(void)
{
	/* Resetting the Timer */
	vTimerControlReset(TIMER0);
	/* Disabling the timer and counting starts */
	vTimerControl(TIMER0,TIMER_COUNTER_DISABLE);
}


/**
  * @brief  Starting the timer for Led Blinking
  * @retval none
  */
void vLedBlinkTimerStart(void)
{
	/* Resetting the Timer */
	vTimerControlReset(TIMER1);
    /* Enabling the timer and counting starts */
	vTimerControl(TIMER1,TIMER_COUNTER_ENABLE);
}
/**
  * @brief  Starting the timer for Led Blinking
  * @retval none
  */
void vLedBlinkTimerStop(void)
{
	/* Resetting the Timer */
	vTimerControlReset(TIMER1);
    /* Disabling the timer and counting stops */
	vTimerControl(TIMER1,TIMER_COUNTER_DISABLE);
}

/**
  * @brief  Led Off State function
  * @param[in] currentinput current input to this state,
  * 		   based on which it will move to the next state
  * @retval none
  */
void vLedOffStatefunction(inputs_t currentinput)
{
	switch (currentinput) {
	case NONE:
		/* DO NOTHING */
		break;
	case TIMEOUT:
		/* DO NOTHING */
		break;
	case BUTTONPRESS:
		vLedOn(LED_0);
		vTimeoutTimerStart();
		/* Assigning the function pointer to point to the next state/output function */
		statemachineobj.currentstatehandler = vLedOnStatefunction;
		break;
	default:
		/* DO NOTHING */
		break;
	}
}

/**
  * @brief  Led On State function
  * @param[in] currentinput current input to this state,
  * 		   based on which it will move to the next state
  * @retval none
  */

void vLedOnStatefunction(inputs_t currentinput)
{
	 switch(currentinput)
				  {
				  case NONE:
					  /* DO NOTHING */
					  break;
				  case TIMEOUT:
					  vLedOff(LED_0);
					  /* Assigning the function pointer to point to the next state/output function */
					  statemachineobj.currentstatehandler = vLedOffStatefunction;
					  break;
				  case BUTTONPRESS:
					  vTimeoutTimerStart();
					  vLedBlinkTimerStart();
					  /* Assigning the function pointer to point to the next state/output function */
					  statemachineobj.currentstatehandler = vLedBlinkStatefunction;
					  break;
				  default:
					  /* DO NOTHING */
					  break;
				  }

}

/**
  * @brief  Blink State function
  * @param[in] currentinput current input to this state,
  * 		   based on which it will move to the next state
  * @retval none
  */
void vLedBlinkStatefunction(inputs_t currentinput)
{
	switch (currentinput) {
	case NONE:
		/* DO NOTHING */
		break;
	case TIMEOUT:
		vLedBlinkTimerStop();
		vLedOff(LED_0);
		/* Assigning the function pointer to point to the next state/output function */
		statemachineobj.currentstatehandler = vLedOffStatefunction;
		break;
	case BUTTONPRESS:
		vTimeoutTimerStop();
		vLedBlinkTimerStop();
		vLedOff(LED_0);
		/* Assigning the function pointer to point to the next state/output function */
		statemachineobj.currentstatehandler = vLedOffStatefunction;
		break;
	default:
		/* DO NOTHING */
		break;
	}
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Variable declarations and initializations */
	 inputs_t currentinput = NONE;
  /* MCU Configuration--------------------------------------------------------*/
  /* Initialize all configured peripherals */
   vAppHardwareInit();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // for(;;)
  {
		if (ucswqueueread(statemachineobj.ptrinputeventqueueobj, &currentinput)
				== TRUE) {
			/* Calling the respective state functions based on the input */
			statemachineobj.currentstatehandler(currentinput);
			currentinput = NONE;
		}

  }
  /* End of Application entry point */
}



