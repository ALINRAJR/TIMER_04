
#ifndef LED_H_
#define LED_H_


/* Private includes ----------------------------------------------------------*/
#include "configboard.h"
#include "gpio.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum {LED_0=0,LED_1,LED_2};

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief Setting direction for LEDs connected on board
  * @return None
  **/
void vLedInitialize(void);

/**
 * @brief Making selected LED on board to ON state
 * @param[in] lednum specify the LED number
 * @return None
 **/
void vLedOn(uint8_t lednum);

/**
 * @brief Making selected LED on board to OFF state
 * @param[in] lednum specify the LED number
 * @return None
 **/
void vLedOff(uint8_t lednum);

/**
 * @brief Making selected LED on board to OFF state
 * @param[in] value specify the 8bit value to display
 * @return None
 **/
void vLedDisplayNum(uint8_t value);

/**
 * @brief Making selected LED on board to toggle
 * @param[in] lednum specify the LED number
 * @return None
 **/
void vLedToggle(uint8_t lednum);



#endif /* LED_H_ */
