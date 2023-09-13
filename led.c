

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "led.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

 /**
  * @brief Setting direction for LEDs connected on board
  * @return None
  **/
void vLedInitialize(void)
{
	    vGpioSetDir(LED0PORT,LED0PIN,OUTPUT);
		vGpioSetDir(LED1PORT,LED1PIN,OUTPUT);
		vGpioSetDir(LED2PORT,LED2PIN,OUTPUT);
}

/**
 * @brief Making selected LED on board to ON state
 * @param[in] lednum specify the LED number
 * @return None
 **/
void vLedOn(uint8_t lednum)
{
	switch(lednum)
	{
	case LED_0:
		vGpioSetPin(LED0PORT,LED0PIN);
		break;
	case LED_1:
		vGpioSetPin(LED1PORT,LED1PIN);
		break;
	case LED_2:
		vGpioSetPin(LED2PORT,LED2PIN);
		break;
	default:
		/* DO NOTHING */
		break;
	}

}
/**
 * @brief Making selected LED on board to OFF state
 * @param[in] lednum specify the LED number
 * @return None
 **/
void vLedOff(uint8_t lednum)
{
	switch(lednum)
	{
	case LED_0:
		vGpioClrPin(LED0PORT,LED0PIN);
		break;
	case LED_1:
		vGpioClrPin(LED1PORT,LED1PIN);
		break;
	case LED_2:
		vGpioClrPin(LED2PORT,LED2PIN);
		break;
	default:
		/* DO NOTHING */
		break;
	}

}

/**
 * @brief Making selected LED on board to OFF state
 * @param[in] value specify the 8bit value to display
 * @return None
 **/
void vLedDisplayNum(uint8_t value)
{

	    /* TODO  */


}


/**
 * @brief Making selected LED on board to toggle
 * @param[in] lednum specify the LED number
 * @return None
 **/
void vLedToggle(uint8_t lednum)
{

	switch(lednum)
		{
		case LED_0:
			vGpioToggle(LED0PORT,LED0PIN);
			break;
		case LED_1:
			vGpioToggle(LED1PORT,LED1PIN);
			break;
		case LED_2:
			vGpioToggle(LED2PORT,LED2PIN);
			break;
		default:
			/* DO NOTHING */
			break;
		}


}


