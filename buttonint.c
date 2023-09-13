

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "buttonint.h"
#include "led.h"
#include "extint.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/


/**
  * @brief Initializing the button as external interrupt
  * @return None
  **/
void vButtonIntInitialize(void)
{

	vExtIntAttachCallback(EINT0,vButtonIntHandler);
	vExtIntInitialize(EINT0);


}
