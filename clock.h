
#ifndef CLOCK_H_
#define CLOCK_H_

/* Private includes ----------------------------------------------------------*/
#include "configboard.h"
#include "LPC17xx.h"
/* Private typedef -----------------------------------------------------------*/
enum {ONEFOURTH=0,SAME=1,HALF=2,ONEEIGTH=3};
/* Private define ------------------------------------------------------------*/

/*Bits in System Controls and Status register - 0x400F C1A0*/
#define BIT_SCS_OSCRANGE 4
#define BIT_SCS_OSCEN 5
#define BIT_SCS_OSCSTAT 6

/*Bits in Clock Source Select register - 0x400FC10C */
#define BIT_CLKSRCSEL_CLKSRC  0

/*Bits in control Register - 0x400F C080*/
#define BIT_PLL0CON_PLLE0 0
#define BIT_PLL0CON_PLLC0 1

/*Bits in configuration Register - 0x400F C084*/
#define BIT_PLL0CFG_MSEL0 0
#define BIT_PLL0CFG_NSEL0 16

/*Bits in PLLO Status register (PLL0STAT - 0x400F C088)*/
#define BIT_PLL0STAT_MSEL0 0
#define BIT_PLL0STAT_NSEL0 16
#define BIT_PLL0STAT_PLLE0_STAT 24
#define BIT_PLL0STAT_PLLC0_STAT 25
#define BIT_PLL0STAT_PLOCK0 26

/*Bits in PLL0 Feed register (PLL0FEED - 0x400F C08C)*/
#define BIT_PLL0FEED 0

/*Bits in CPU Clock Configuration register (CCLKCFG - address 0x400F C104)*/
#define BIT_CCLKCFG_CCLKSEL 0

/*Bits in Peripheral Clock Selection register 0 - 0x400FC1A8 */
#define BIT_PCLKSEL0_PCLK_WDT 0
#define BIT_PCLKSEL0_PCLK_TIMER0 2
#define BIT_PCLKSEL0_PCLK_TIMER1 4
#define BIT_PCLKSEL0_PCLK_UART0 6
#define BIT_PCLKSEL0_PCLK_UART1 8
#define BIT_PCLKSEL0_PCLK_PWM1 12
#define BIT_PCLKSEL0_PCLK_I2C0 14
#define BIT_PCLKSEL0_PCLK_SPI 16
#define BIT_PCLKSEL0_PCLK_SSP1 20
#define BIT_PCLKSEL0_PCLK_DAC 22
#define BIT_PCLKSEL0_PCLK_ADC 24
#define BIT_PCLKSEL0_PCLK_CAN1 26
#define BIT_PCLKSEL0_PCLK_CAN2 28
#define BIT_PCLKSEL0_PCLK_ACF 30

/*Bits in Peripheral Clock Selection register 1 - 0x400FC1AC */
#define BIT_PCLKSEL1_PCLK_QEI 0
#define BIT_PCLKSEL1_PCLK_GPIOINT 2
#define BIT_PCLKSEL1_PCLK_PCB 4
#define BIT_PCLKSEL1_PCLK_I2C1 6
#define BIT_PCLKSEL1_PCLK_SSP0 10
#define BIT_PCLKSEL1_PCLK_TIMER2 12
#define BIT_PCLKSEL1_PCLK_TIMER3 14
#define BIT_PCLKSEL1_PCLK_UART2 16
#define BIT_PCLKSEL1_PCLK_UART3 18
#define BIT_PCLKSEL1_PCLK_I2C2 20
#define BIT_PCLKSEL1_PCLK_I2S 22
#define BIT_PCLKSEL1_PCLK_RIT 26
#define BIT_PCLKSEL1_PCLK_SYSCON 28
#define BIT_PCLKSEL1_PCLK_MC 30


/* Bits in Clock out Configuration register - 0x400F C1C8 */
#define BIT_CLKOUTCFG_CLKOUTSEL  0
#define BIT_CLKOUTCFG_CLKOUTDIV  4
#define BIT_CLKOUTCFG_CLKOUT_EN  8
#define BIT_CLKOUTCFG_CLKOUT_ACT 9

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief     Clock Initialization to the user desired frequency
  * @return    No return value
  **/
void vClockInitialization(void);

#endif /* CLOCK_H_ */
