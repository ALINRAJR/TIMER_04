
#ifndef POWERCONTROL_H_
#define POWERCONTROL_H_

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

enum {POWEROFF=0,POWERON=1};


/* Private define ------------------------------------------------------------*/

/*Bits in Power Mode Control Register (PCON - 0x400F C0C0)*/

#define BIT_PCON_PM0		0
#define BIT_PCON_PM1		1
#define BIT_PCON_BODRPM		2
#define BIT_PCON_BOGD		3
#define BIT_PCON_BORD		4
#define BIT_PCON_SMFLAG		8
#define BIT_PCON_DSFLAG		9
#define BIT_PCON_PDFLAG		10
#define BIT_PCON_DPDFLAG	11

/*Bits in Power Control for Peripherals register r (PCONP - 0x400F C0C4)*/
#define BIT_PCONP_PCTIM0	1
#define BIT_PCONP_PCTIM1	2
#define BIT_PCONP_PCUART0	3
#define BIT_PCONP_PCUART1	4
#define BIT_PCONP_PCPWM1	6
#define BIT_PCONP_PCI2C0	7
#define BIT_PCONP_PCSPI		8
#define BIT_PCONP_PCRTC		9
#define BIT_PCONP_PCSSP1	10
#define BIT_PCONP_PCADC		12
#define BIT_PCONP_PCCAN1	13
#define BIT_PCONP_PCCAN2	14
#define BIT_PCONP_PCGPIO	15
#define BIT_PCONP_PCRIT		16
#define BIT_PCONP_PCMCPWM	17
#define BIT_PCONP_PCQEI 	18
#define BIT_PCONP_PCI2C1	19
#define BIT_PCONP_PCSSP0 	21
#define BIT_PCONP_PCTIM2	22
#define BIT_PCONP_PCTIM3	23
#define BIT_PCONP_PCUART2	24
#define BIT_PCONP_PCUART3	25
#define BIT_PCONP_PCI2C2	26
#define BIT_PCONP_PCI2S		27
#define BIT_PCONP_PCGPDMA	29
#define BIT_PCONP_PCENET	30
#define BIT_PCONP_PCUSB		31

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


#endif /* POWERCONTROL_H_ */
