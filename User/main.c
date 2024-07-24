/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 14/11/20 2:18p $
 * @brief    NUC029 Series I2C Driver Sample Code for EEPROM 24LC64
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC029xAN.h"

#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000

#define BMP180_ADDRESS (0x77)
#define REGISTER_ADDRESS (0xAA)
#include "NUC029xAN.h"
#include <stdio.h>

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
uint8_t g_au8TxData[3];
uint8_t *g_pu8RxData;
uint8_t g_u8DataLen;
uint8_t u8Size;
volatile uint8_t g_u8EndFlag = 0;
typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2C0HandlerFn = NULL;


void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C0->I2CSTATUS;

    if(I2C0->I2CTOC & I2C_I2CTOC_TIF_Msk)
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

void I2C0_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++] << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        printf("d");
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        printf("e");
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);

    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 3)
        {
            I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
            printf("v");

        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            g_u8EndFlag = 1;
            printf("c");

        }
    }
    else if (u32Status == 0xF8)    /*I2C wave keeps going*/
    {
        printf("i");
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}


void I2C0_MasterRx(uint32_t u32Status)
{
	if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
	    {
	        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++] << 1;     /* Write SLA+W to Register I2CDAT */
	        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	        printf("f");
	    }
	    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
	    {
	        I2C0->I2CDAT = g_au8TxData[g_u8DataLen];
	        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	        printf("e");

	    }
	    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
	    {
	        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
	    }
	    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
	    {
	        if(g_u8DataLen != 2)
	        {
	            I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
	            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
		        printf("r");

	        }
	        else
	        {
	            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
		        printf("m");

	        }
	    }
	    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
	    {
	        I2C0->I2CDAT = ((0x77 << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
	        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	        printf("c");

	    }
	    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
	    {
	        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	        printf("i");

	    }
	    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
	    {
	    	g_pu8RxData[g_u8DataLen++] = I2C0->I2CDAT;
	        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
	        g_u8EndFlag = 1;
	        printf("l");

	    }
	    else
	    {
	        /* TO DO */
	        printf("Status 0x%x is NOT processed\n", u32Status);
	    }
//    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
//    {
//        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++] << 1;     /* Write SLA+W to Register I2CDAT */
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
//    {
//        I2C0->I2CDAT = g_au8TxData[g_u8DataLen];
//        g_u8DataLen = 0;
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
//    {
//    	I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
//    }
//    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
//    {
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
//    }
//
//    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
//    {
//        I2C0->I2CDAT = ((g_au8TxData[0] << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
//    {
//        if (u8Size > 1)
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
//        }
//        else
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//        }
//    }
//    else if (u32Status == 0x50)                 /* DATA has been received has been returned */
//    {
//        g_pu8RxData[g_u8DataLen++] = I2C0->I2CDAT;
//        if ((g_u8DataLen + 1) < u8Size)
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
//        }
//        else
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//        }
//    }
//    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
//    {
//        g_pu8RxData[g_u8DataLen++] = I2C0->I2CDAT;
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
//        g_u8EndFlag = 1;
//    }
//    else if (u32Status == 0xF8)    /*I2C wave keeps going*/
//    {
//
//    }
//    else
//    {
//        /* TO DO */
//        printf("Status 0x%x is NOT processed\n", u32Status);
//        while(1);
//    }
}

void I2C0_Write(uint8_t u8Address, uint8_t u8Reg, uint8_t u8Data)
{
    g_au8TxData[0] = u8Address;
    g_au8TxData[1] = u8Reg;
    g_au8TxData[2] = u8Data;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C0_MasterTx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);

    /* Wait I2C Tx Finish */
    while (g_u8EndFlag == 0);
}


void I2C0_Read(uint8_t u8Address, uint8_t u8Reg, uint8_t u8Byte, uint8_t *pu8Data)
{
    g_au8TxData[0] = u8Address;
    g_au8TxData[1] = u8Reg;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;
    u8Size = u8Byte;

    g_pu8RxData = pu8Data;

    /* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C0_MasterRx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);

    /* Wait I2C Rx Finish */
    while (g_u8EndFlag == 0);
}

void BMP_180read(uint8_t au8Data[22]){
    I2C0_Read(BMP180_ADDRESS, 0xAA , 22, au8Data);
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_I2C0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD, and set P3.4 and P3.5 for I2C0 SDA and SCL */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk | SYS_MFP_P34_Msk | SYS_MFP_P35_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0 | SYS_MFP_P34_SDA0 | SYS_MFP_P35_SCL0);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C0_Init(void)
{
    /* Reset I2C0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->I2CON |= I2C_I2CON_ENS1_Msk;

    /* I2C0 clock divider, I2C Bus Clock = PCLK / (4*125) = 100kHz */
    I2C0->I2CLK = 125 - 1;

    /* Get I2C0 Bus Clock */
    //printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->I2CLK) + 1) << 2)));

    /* Slave Address : 0x77 */
   // I2C0->I2CADDR0 = (I2C0->I2CADDR0 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x77 << I2C_I2CADDR_I2CADDR_Pos);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->I2CON &= ~I2C_I2CON_EI_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->I2CON &= ~I2C_I2CON_ENS1_Msk;
    CLK->APBCLK &= ~CLK_APBCLK_I2C0_EN_Msk;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    //uint32_t i;
    uint8_t au8Data[22];
    /* Unlock protected registers */
    SYS_UnlockReg();
 //   printf("\n Hello!!!\n");

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */

    /* Init UART0 for printf */
    UART0_Init();

//    printf("\n Hello!!!\n");

   I2C0_Init();

    SYS_LockReg();
    printf("\n Slave Address 0x77!!!\n");

    while(1){
    BMP_180read(au8Data);
     for(uint32_t i= 0; i<23 ; i++)
     printf("calibrate data: %d\n", au8Data[i]);
        printf("Hello!!!\n");
        for(volatile uint32_t t = 0 ;t < 1000000;t++);
    }
    return 0;
}
