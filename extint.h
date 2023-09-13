


#ifndef EXTINT_H_
#define EXTINT_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

enum {LOW_LEVEL=0,HIGH_LEVEL=1};
enum {EXTINTDISABLE=0,EXTINTENABLE=1};
enum {FALLING_EDGE=0, RISING_EDGE=1};

enum {LEVEL_SENSITIVE=0,EDGE_SENSITIVE=1};
enum {EINT0=0,EINT1,EINT2,EINT3};

/* Private define ------------------------------------------------------------*/


#define TESTCASE0    0
#define TESTCASE1    0
#define TESTCASE2    0
#define TESTCASE3    1


/*Bits in External Interrupt Flag register (EXTINT - address 0x400F C140)*/
#define BIT_EXTINT_EINT0		0
#define BIT_EXTINT_EINT1		1
#define BIT_EXTINT_EINT2		2
#define BIT_EXTINT_EINT3		3


/*Bits in External Interrupt Mode register (EXTMODE - address 0x400F C148)*/
#define BIT_EXTMODE_EXTMODE0	0
#define BIT_EXTMODE_EXTMODE1	1
#define BIT_EXTMODE_EXTMODE2	2
#define BIT_EXTMODE_EXTMODE3	3


/*Bits in External Interrupt Polarity register (EXTPOLAR - address 0x400F C14C)*/
#define BIT_EXTPOLAR_EXTPOLAR0	0
#define BIT_EXTPOLAR_EXTPOLAR1	1
#define BIT_EXTPOLAR_EXTPOLAR2	2
#define BIT_EXTPOLAR_EXTPOLAR3	3

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Function pointer declaration */
typedef void (*extintfuncptr)(void);

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief     External interrupt Initialization
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @return    No return value
 **/

void vExtIntInitialize(uint8_t ucExtintnum);

/**
 * @brief     Enabling the interrupt and for clearing/re enabling the interrupt
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @param[in] mode  value can be EXTINTENABLE or EXTINTDISABLE
 * @return    No return value
 **/
void vExtIntSel(uint8_t ucExtintnum, uint8_t mode);

/**
 * @brief     External interrupt call back registration
 * @param[in] ucExtintnum value can be EINT0,EINT1,EINT2,EINT3
 * @param[in] vExtintisrobject address of the function which should be called when the respective interrupt occurs
 * @return    No return value
 **/

void vExtIntAttachCallback(uint8_t ucExtintnum, extintfuncptr vExtintisrobject);




#endif /* EXTINT_H_ */
