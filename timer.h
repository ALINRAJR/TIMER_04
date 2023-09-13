


#ifndef TIMER_H_
#define TIMER_H_



/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include <LPC17xx.h>

/* Private typedef -----------------------------------------------------------*/
enum {TIMER_0=0,TIMER_1,TIMER_2,TIMER_3};
enum {TIMER_COUNTER_DISABLE=0,TIMER_COUNTER_ENABLE};
enum {PRESCALE_COUNTER_DISABLE=0,PRESCALE_COUNTER_ENABLE};
enum {TIMER_MODE=0,TC_INCREMENT_RISING_EDGE,TC_INCREMENT_FALLING_EDGE,TC_INCREMENT_BOTH_EDGE};
enum {CAPn_0=0,CAPn_1};
enum {MATCH0=0,MATCH1,MATCH2,MATCH3,CAPTURE0,CAPTURE1};
enum {MR0INT=0x01,MR1INT=0x02,MR2INT=0x04,MR3INT=0x08,CAP0INT=0x10,CAP1INT=0x20};

/* Function pointer declaration */
typedef void (*timerisrfuncptr)(uint32_t interruptsource);

/* Private define ------------------------------------------------------------*/

/* Prescalar values for 1us, 1ms and 1 sec for peripheral clock 25MHz */
#define PRESCALAR_US        25
#define PRESCALAR_MS        25000
#define PRESCALAR_SEC       25000000


#define TIMER0   LPC_TIM0
#define TIMER1   LPC_TIM1
#define TIMER2   LPC_TIM2
#define TIMER3   LPC_TIM3






/*Bits in Interrupt Register (T[0/1/2/3]IR - 0x4000 4000, 0x4000 8000, 0x4009 0000, 0x4009 4000)*/
#define BIT_IR_MR0INT                    0
#define BIT_IR_MR1INT                    1
#define BIT_IR_MR2INT                    2
#define BIT_IR_MR3INT                    3
#define BIT_IR_CR0INT                    4
#define BIT_IR_CR1INT                    5

/*Bits in Timer Control Register
(TCR, TIMERn: TnTCR - addresses 0x4000 4004, 0x4000 8004, 0x4009 0004, 0x4009 4004)*/
#define BIT_TCR_COUNTER_ENABLE			 0
#define BIT_TCR_COUNTER_RESET  			 1

/*Bits in Count Control Register
(T[0/1/2/3]CTCR - addresses 0x4000 4070, 0x4000 8070, 0x4009 0070, 0x4009 4070)*/
#define BIT_CTCR_COUNTER_TIMER_MODE          0
#define BIT_CTCR_COUNTER_INPUT_SELECT        2

/*Bits in Match Control Register
(T[0/1/2/3]MCR - addresses 0x4000 4014, 0x4000 8014, 0x4009 0014, 0x4009 4014)*/
#define BIT_MCR_MR0I		       0
#define BIT_MCR_MR0R		       1
#define BIT_MCR_MR0S		       2
#define BIT_MCR_MR1I		       3
#define BIT_MCR_MR1R		       4
#define BIT_MCR_MR1S		       5
#define BIT_MCR_MR2I		       6
#define BIT_MCR_MR2R		       7
#define BIT_MCR_MR2S		       8
#define BIT_MCR_MR3I		       9
#define BIT_MCR_MR3R		       10
#define BIT_MCR_MR3S		       11

/*Bits in Capture Control Register
(T[0/1/2/3]CCR - addresses 0x4000 4028, 0x4000 8020, 0x4009 0028, 0x4009 4028)*/
#define BIT_CCR_CAP0RE		       0
#define BIT_CCR_CAP0FE		       1
#define BIT_CCR_CAP0I		       2
#define BIT_CCR_CAP1RE		       3
#define BIT_CCR_CAP1FE		       4
#define BIT_CCR_CAP1I		       5

/*Bits in External Match Register
(T[0/1/2/3]EMR - addresses 0x4000 403C, 0x4000 803C, 0x4009 003C, 0x4009 403C)*/
#define BIT_EMR_EM0		       	   0
#define BIT_EMR_EM1		       	   1
#define BIT_EMR_EM2		           2
#define BIT_EMR_EM3		           3
#define BIT_EMR_EMC0		       4
#define BIT_EMR_EMC1		       6
#define BIT_EMR_EMC2		       8
#define BIT_EMR_EMC3		       10

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief     Initializing the selected TIMER Peripheral
 * @param[in] ucTIMERNum value can be TIMER_0, TIMER_1, TIMER_2, TIMER_3
 * @return    No return value
 **/

void vTimerInitialize(uint8_t ucTIMERNum);

/**
 * @brief     Selecting timer mode or counter mode for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] mode value can be TIMER_MODE,TC_INCREMENT_RISING_EDGE,TC_INCREMENT_FALLING_EDGE,TC_INCREMENT_BOTH_EDGE
 * @return    No return value
 **/
void vTimerCountControl(LPC_TIM_TypeDef* TIMERx, uint8_t mode);

/**
 * @brief     Selecting cap pin from which the counting to be done for counter mode for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] countinputsel value can be CAPn_0=0,CAPn_1
 * @return    No return value
 **/
void vTimerCountControlInpSel(LPC_TIM_TypeDef* TIMERx, uint8_t countinputsel);

/**
 * @brief     Enabling  the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] mode value can be TIMER_COUNTER_DISABLE,TIMER_COUNTER_ENABLE
 * @return    No return value
 **/
void vTimerControl(LPC_TIM_TypeDef* TIMERx,uint8_t mode);


/**
 * @brief     Resetting the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @return    No return value
 **/
void vTimerControlReset(LPC_TIM_TypeDef* TIMERx);

/**
 * @brief     Setting the prescalar value for  the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] prescale value can be PRESCALAR_US,PRESCALAR_MS,PRESCALAR_SEC
 * @return    No return value
 **/
void vTimerPrescalarSel(LPC_TIM_TypeDef* TIMERx,uint32_t prescale);





/**
 * @brief     Generating delay in milliseconds using polling method for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] delayinms value can be 32 bit integer value
 * @return    No return value
 **/
void vTimerDelayinMs(LPC_TIM_TypeDef* TIMERx,uint32_t delayinms);
/**
 * @brief     Generating delay in microseconds using polling method for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] delayinus value can be 32 bit integer value
 * @return    No return value
 **/
void vTimerDelayinUs(LPC_TIM_TypeDef* TIMERx,uint32_t delayinus);

/**
 * @brief     Generating delay in seconds using polling method for the selected timer peripheral
 * @param[in] TIMERx value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] delayinsecs value can be 32 bit integer value
 * @return    No return value
 **/
void vTimerDelayinSecs(LPC_TIM_TypeDef* TIMERx,uint32_t delayinsecs);


/**
  * @brief     Reseting the respective Match Register for the selected timer
  * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
  * @param[in] matchsource value can be MATCH0,MATCH1,MATCH2,MATCH3
  **/
void vTimerMatchReset(LPC_TIM_TypeDef* TIMERx,uint8_t matchsource);

/**
 * @brief     Stopping the respective Match Register for the selected timer
 * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] matchsource value can be MATCH0,MATCH1,MATCH2,MATCH3
 **/
void vTimerMatchStop(LPC_TIM_TypeDef* TIMERx, uint8_t matchsource);

/**
 * @brief     Loading the respective Match Register for the selected timer
 * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
 * @param[in] matchsource value can be MATCH0,MATCH1,MATCH2,MATCH3
 * @param[in] loadvalue value can be 32bit integer value
 **/
void vTimerLoadMatchRegister(LPC_TIM_TypeDef* TIMERx, uint8_t matchsource, uint32_t loadvalue);

/**
  * @brief     Enabling Interrupt for the Timer and the source for interrupt
  * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
  * @param[in] interruptsource value can be MATCH0,MATCH1,MATCH2,MATCH3,CAPTURE0,CAPTURE1
  * @return    No return value
  **/
void vTimerInterruptEnable(LPC_TIM_TypeDef* TIMERx,uint8_t matchsource);

/**
  * @brief     Disabling Interrupt for the Timer and the source for interrupt
  * @param[in] TIMERx base address of the selected uart value can be TIMER0,TIMER1,TIMER2,TIMER3
  * @param[in] interruptsource value can be MATCH0INT,MATCH1INT,MATCH2INT,MATCH3INT,CAPTURE0INT,CAPTURE1INT
  * @return    No return value
  **/
void vTimerInterruptDisable(LPC_TIM_TypeDef* TIMERx,uint8_t interruptsource);


/**
 * @brief     Timer interrupt call back registration
 * @param[in] ucTIMERNum value can be TIMER_0, TIMER_1, TIMER_2, TIMER_3
 * @param[in] vTimerintisrobject address of the function which should be called when the respective interrupt occurs
 * @return    No return value
 **/

void vTimerIntAttachCallback(uint8_t ucTIMERNum, timerisrfuncptr vTimerintisrobject);


#endif /* TIMER_H_ */
