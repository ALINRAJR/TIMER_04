


#ifndef PINMUX_H_
#define PINMUX_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
#include <LPC17xx.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct Pinconfig_t
{
	LPC_GPIO_TypeDef *GPIOx;
   	uint8_t pin;
    uint8_t func;
}Pinconfig_t;

/* Private define ------------------------------------------------------------*/
enum {FUNC0=0,FUNC1,FUNC2,FUNC3};

enum   {
/*		P0_0=0,RD1=0,TXD3=0,SDA1=0,  //CAN_RD_1
    	P0_1=1,TD1=1,RXD3=1,SCL1=1,//CAN_TD_1
		P0_2=2,TXD0=2,AD0_7=2,//UART_TX_0
		P0_3=3,RXD0=3,AD0_6=3,//UART_RX_0
		P0_4=4,//I2SRX_CLK=4,RD2=4,CAP2_0=4,//P0_4
		P0_5=5,//I2SRX_WS=5,TD2=5,CAP2_1=5,//P0_5
		P0_6=6,//I2SRX_SDA=6,SSEL=6,MAT2_0=6,//P0_6
		P0_7=7,I2STX_CLK=7,SCK1=7,MAT2_1=7,//P0_7
		P0_8=8,I2STX_WS=8,MISO1=8,MAT2_2=8,//P0_8
		P0_9=9,I2STX_SDA=9,MOSI1=9,MAT2_3=9,//P0_9
		P0_10=10,TXD2=10,SDA2=10,MAT3_0=10,//P0_10
		P0_11=11,RXD2=11,SCL2=11,MAT3_1=11,//P0_11
		P0_15=15,TXD1=15,SCK0=15,SCK=15,//SPI_SCK_0
		P0_16=16,RXD1=16,SSEL0=16,SSEL=16,//SPI_SSEL_0
		P0_17=17,CTS1=17,MISO0=17,MISO=17,//SPI_MISO_0
		P0_18=18,DCD1=18,MOSI0=18,MOSI=18,//SPI_MOSI_0
		P0_19=19,DSR1=19,SDA1=19,//P0_19
		P0_20=20,DTR1=20,SCL1=20,//P0_20
		P0_21=21,RI1=21,RD1=21,//P0_21
		P0_22=22,RTS1=22,TD1=22,//P0_22
		P0_23=23,AD0_0=23,I2SRX_CLK=23,CAP3_0=23,//AD0_0
		P0_24=24,AD0_1=24,I2SRX_WS=24,CAP3_1=24,//AD0_1
		P0_25=25,AD0_2=25,I2SRX_SDA=25,TXD3=25,//AD0_2
		P0_26=26,AD0_3=26,A_OUT=26,RXD3=26,//AD0_3/AOUT
		P0_27=27,SDA0=27,USB_SDA=27,//I2C_SDA_0
		P0_28=28,SCL0=28,USB_SCL=28,//I2C_SCL_0
		P0_29=29,USB_DP=29,//USB_DP
		P0_30=30,USB_DN=30,//USB_DN
		P1_0=0,ENET_TXD0=0,//ENET_TXD0
		P1_1=1,ENET_TXD1=1,//ENET_TXD1
		P1_4=4,ENET_TX_EN=4,//ENET_TX_EN
		P1_8=8,ENET_CRS=8,//ENET_CRS
		P1_9=9,ENET_RXD0=9,//ENET_RXD0
		P1_10=10,ENET_RXD1=10,//ENET_RXD1
		P1_14=14,ENET_RX_ER=14,//ENET_RX_ER
		P1_15=15,ENET_REF_CLK=15,//ENET_REF_CLK
		P1_16=16,ENET_MDC=16,//ENET_MDC
		P1_17=17,ENET_MDIO=17,//ENET_MDIO
		P1_18=18,USB_UP_LED=18,PWM1_1=18,CAP1_0=18,//USB_UP-LED
*/		P1_19=19,MC0A=19,nUSB_PPw=19,CAP1_1=19,//P1_19
		P1_20=20,MCFB0=20,//PWM1_2=20,SCK0=20,//P1_20
/*		P1_21=21,MCABORT=21,PWM1_3=21,SSEL0=21,//P1_21
		P1_22=22,MC0B=22,USB_PWRD=22,MAT1_0=22,//P1_22
		P1_23=23,MCFB1=23,PWM1_4=23,MISO0=23,//P1_23
		P1_24=24,MCFB2=24,PWM1_5=24,MOSI0=24,//P1_24
		P1_25=25,MC1A=25,MAT1_1=25,//P1_25
		P1_26=26,MC1B=26,PWM1_6=26,CAP0_0=26,//P1_26
		P1_27=27,CLKOUT=27,nUSB_OVRCR=27,CAP0_1=27,//P1_27
		P1_28=28,MC2A1_0=28,MAT0_0=28,//P1_28
*/		P1_29=29,MC2B=29,PCAP1_1=29,MAT0_1=29,//TEST_LED
/*		P1_30=30,VBUS=30,AD0_4=30,//VBUS
		P1_31=31,SCK1=31,AD0_5=31,//AD0_5
		P2_0=0,PWM1_1=0,TXD1=0,//UART_TX_1
		P2_1=1,PWM1_2=1,RXD1=1,//UART_RX_1
		P2_2=2,PWM1_3=2,CTS1=2,TRACEDATA3=2,//PWM 1_3
		P2_3=3,PWM1_4=3,DCD1=3,TRACEDATA2=3,//PWM 1_4
		P2_4=4,PWM1_5=4,DSR1=4,TRACEDATA1=4,//PWM 1_5
		P2_5=5,PWM1_6=5,DTR1=5,TRACEDATA0=5,//PWM 1_6
		P2_6=6,PCAP1_0=6,RI1=6,TRACECLK=6,//P2_6
		P2_7=7,RD2=7,RTS1=7,//P2_7
		P2_8=8,TD2=8,TXD2=8,//P2_8
		P2_9=9,USB_CONNECT=9,RXD2=9,//CONNECT
*/		P2_10=10,nENT0=10,NMI=10,//P2_10/BLD
		P2_11=11,nENT1=11,I2STX_CLK=11,//P2_11
		P2_12=12,nENT2=12,I2STX_CWS=12,//P2_12
		P2_13=13,nENT3=13,I2STX_SDA=13,//P2_13
/*		P3_25=25,MAT0_0=25,PWM1_2=25,//PWM1_2
		P3_26=26,STCLK=26,MAT0_1=26,PWM1_3=26,//P3_26
		P4_28=28,RX_MCLK=28,MAT2_0=28,TXD3=28,//P4_28
		P4_29=29,TX_MCLK=29,MAT2_1=29,RXD3=29//P4_29
*/
};

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Initializing the selected functionality for the selected pins
 * @return None
 **/
void vPinmuxInitialize(void);
/**
  * @brief Setting required functionality for the selected pin
  * @param[in] uiGPIOx specify the Port base address
  * @param[in] ucPinnum specify the Pin number
  * @param[in] ucfunctionality specify the functionality for the selected pin
  * @return None
  **/

void vPinmuxSel(LPC_GPIO_TypeDef* uiGPIOx, uint8_t ucPinnum, uint8_t ucfunctionality);


#endif /* PINMUX_H_ */
