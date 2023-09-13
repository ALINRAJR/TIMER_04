
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "swqueue.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

 /**
  * @brief     Initializing the queue object with the initial values
  * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
  * @param[in] buffer base address of the array passed
  * @param[in] itemsize Size in bytes of the item in a location
  * @param[in] count No of items in a location
  * @return    No return value
  **/
void vswqueueinitialize(swqueue_t *ptrtoswqueueobj,uint8_t *buffer, uint16_t itemsize, uint16_t count)
{
	ptrtoswqueueobj->writeindex=0;
	ptrtoswqueueobj->readindex=0;
	ptrtoswqueueobj->itemsize= itemsize;
	ptrtoswqueueobj->count= count;
	ptrtoswqueueobj->swqueuebuffer = buffer;

}

/**
 * @brief     checking whether queue is full
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @return    TRUE or FALSE
 **/
uint8_t isswqueuefull(swqueue_t *ptrtoswqueueobj)
{

	 if(ptrtoswqueueobj->writeindex+1==ptrtoswqueueobj->readindex)
		 return TRUE;
	 else
		 return FALSE;
}

/**
 * @brief    checking whether queue is empty
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @return    TRUE or FALSE
 **/

uint8_t isswqueueempty(swqueue_t *ptrtoswqueueobj)
{
	 if(ptrtoswqueueobj->writeindex==ptrtoswqueueobj->readindex)
		 return TRUE;
	 else
		 return FALSE;
}

/**
 * @brief     Writing data from the queue object
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @param[in] writedata data to be written into the respective queue object
 * @return    TRUE or FALSE
 **/
uint8_t ucswqueuewrite(swqueue_t *ptrtoswqueueobj, uint8_t writedata)
{

	if(isswqueuefull(ptrtoswqueueobj)==TRUE)
		return FALSE;

	/* Writing the value into the buffer inside the respective queue object */
	ptrtoswqueueobj->swqueuebuffer[ptrtoswqueueobj->writeindex]=writedata;
	/* Incrementing the write index for next cycle operation */
	ptrtoswqueueobj->writeindex++;
	/* Overflow management for the write index */
	ptrtoswqueueobj->writeindex = ptrtoswqueueobj->writeindex%ptrtoswqueueobj->count;

	/* Returning TRUE if successfully written in the queue location */
  return TRUE;
}

/**
 * @brief     Reading data from the queue object
 * @param[in] ptrtoswqueueobj pointer to the swqeueue object passed
 * @param[in] ptrtoreaddata reading the data from the queue and putting into the application buffer
 * @return    TRUE or FALSE
 **/

uint8_t ucswqueueread(swqueue_t *ptrtoswqueueobj, uint8_t* ptrtoreaddata)
{

	if(isswqueueempty(ptrtoswqueueobj)==TRUE)
		return FALSE;

	/* Reading the value into the application buffer from the buffer inside the respective queue object */
	*ptrtoreaddata=ptrtoswqueueobj->swqueuebuffer[ptrtoswqueueobj->readindex];
	/* Incrementing the write index for next cycle operation */
	ptrtoswqueueobj->readindex++;
	/* Overflow management for the read index */
	ptrtoswqueueobj->readindex = ptrtoswqueueobj->readindex	% ptrtoswqueueobj->count;

	/* Returning TRUE if successfully read from the queue location */
  return TRUE;

}







