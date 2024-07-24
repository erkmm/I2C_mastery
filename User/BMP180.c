//#include "NUC029xAN.h"
//#include <stdio.h>
//
///*---------------------------------------------------------------------------*/
///* Global variables                                                          */
///*---------------------------------------------------------------------------*/
//uint8_t g_au8TxData[3];
//uint8_t *g_pu8RxData;
//uint8_t g_u8DataLen;
//uint8_t u8Size;
//volatile uint8_t g_u8EndFlag = 0;
//typedef void (*I2C_FUNC)(uint32_t u32Status);
//static I2C_FUNC s_I2C0HandlerFn = NULL;
//
//
//void I2C0_IRQHandler(void)
//{
//    uint32_t u32Status;
//
//    u32Status = I2C0->I2CSTATUS;
//
//    if(I2C0->I2CTOC & I2C_I2CTOC_TIF_Msk)
//    {
//        /* Clear I2C0 Timeout Flag */
//        I2C0->I2CTOC |= I2C_I2CTOC_TIF_Msk;
//    }
//    else
//    {
//        if(s_I2C0HandlerFn != NULL)
//            s_I2C0HandlerFn(u32Status);
//    }
//}
//
//void I2C0_MasterTx(uint32_t u32Status)
//{
//    if (u32Status == 0x08)                      /* START has been transmitted */
//    {
//        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++] << 1;     /* Write SLA+W to Register I2CDAT */
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
//    {
//        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
//    {
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA | I2C_I2CON_STO | I2C_I2CON_SI);
//    }
//    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
//    {
//        if (g_u8DataLen != 3)
//        {
//            I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//        }
//        else
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO | I2C_I2CON_SI);
//            g_u8EndFlag = 1;
//        }
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
//}
//
//
//void I2C0_MasterRx(uint32_t u32Status)
//{
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
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA | I2C_I2CON_STO | I2C_I2CON_SI);
//    }
//    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
//    {
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA | I2C_I2CON_SI);
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
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI | I2C_I2CON_AA);
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
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI | I2C_I2CON_AA);
//        }
//        else
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//        }
//    }
//    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
//    {
//        g_pu8RxData[g_u8DataLen++] = I2C0->I2CDAT;
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO | I2C_I2CON_SI);
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
//}
//
//void I2C0_Write(uint8_t u8Address, uint8_t u8Reg, uint8_t u8Data)
//{
//    g_au8TxData[0] = u8Address;
//    g_au8TxData[1] = u8Reg;
//    g_au8TxData[2] = u8Data;
//
//    g_u8DataLen = 0;
//    g_u8EndFlag = 0;
//
//    /* I2C function to write data to slave */
//    s_I2C0HandlerFn = (I2C_FUNC)I2C0_MasterTx;
//
//    /* I2C as master sends START signal */
//    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);
//
//    /* Wait I2C Tx Finish */
//    while (g_u8EndFlag == 0);
//}
//
//
//void I2C0_Read(uint8_t u8Address, uint8_t u8Reg, uint8_t u8Byte, uint8_t *pu8Data)
//{
//    g_au8TxData[0] = u8Address;
//    g_au8TxData[1] = u8Reg;
//
//    g_u8DataLen = 0;
//    g_u8EndFlag = 0;
//    u8Size = u8Byte;
//
//    g_pu8RxData = pu8Data;
//
//    /* I2C function to write data to slave */
//    s_I2C0HandlerFn = (I2C_FUNC)I2C0_MasterRx;
//
//    /* I2C as master sends START signal */
//    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);
//
//    /* Wait I2C Rx Finish */
//    while (g_u8EndFlag == 0);
//}
//
