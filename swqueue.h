
#ifndef SWQUEUE_H_
#define SWQUEUE_H_

#include <stdint.h>
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

typedef struct {
	uint8_t *swqueuebuffer; // pointer to a buffer of as the application writer's design
	uint16_t writeindex; // Also called as Head
	uint16_t readindex;  // Also called as Tail
	uint16_t itemsize;   // No of bytes an item in a location
	uint16_t count;      // No of locations
}swqueue_t;

/* Private define ------------------------------------------------------------*/
enum {FALSE=0,TRUE};
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


/**
 * @brief     Initializing the queue object with the initial values
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @param[in] buffer base address of the array passed
 * @param[in] itemsize Size in bytes of the item in a location
 * @param[in] count No of items in a location
 * @return    No return value
 **/
void vswqueueinitialize(swqueue_t *ptrtoswqueueobj,uint8_t *buffer, uint16_t itemsize, uint16_t count);

/**
 * @brief     checking whether queue is full
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @return    TRUE or FALSE
 **/
uint8_t isswqueuefull(swqueue_t *ptrtoswqueueobj);


/**
 * @brief    checking whether queue is empty
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @return    TRUE or FALSE
 **/

uint8_t isswqueueempty(swqueue_t *ptrtoswqueueobj);

/**
 * @brief     Writing data from the queue object
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @param[in] writedata data to be written into the respective queue object
 * @return    TRUE or FALSE
 **/
uint8_t ucswqueuewrite(swqueue_t *ptrtoswqueueobj, uint8_t writedata);

/**
 * @brief     Reading data from the queue object
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @param[in] ptrtoreaddata reading the data from the queue and putting into the application buffer
 * @return    TRUE or FALSE
 **/

uint8_t ucswqueueread(swqueue_t *ptrtoswqueueobj, uint8_t* ptrtoreaddata);


#endif /* SWQUEUE_H_ */
