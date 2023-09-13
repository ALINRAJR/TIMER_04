
  **/
#ifndef BUTTONINT_H_
#define BUTTONINT_H_

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Initializing the button as external interrupt
  * @return None
  **/
void vButtonIntInitialize(void);

/**
  * @brief  Button interrupt Handler
  * @retval none
  */
void vButtonIntHandler(void);

#endif /* BUTTONINT_H_ */
