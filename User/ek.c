//
//#define I2C_TIMEOUT_ERR    (-1L)
//
//int32_t g_I2C_i32ErrCode = 0;
////#define BMP180_ADDR 0xEE  // 8-bit address (0x77 << 1)
////#define CALIB_START_ADDR 0xAA
////#define CALIB_DATA_LENGTH 22
////
////void I2C_RegRead(uint8_t address, uint8_t *data, uint8_t size)
////{
////	uint8_t count = 0;
////
////	(I2C0->I2CON = (I2C0->I2CON | I2C_I2CON_SI_Msk) | I2C_I2CON_STA_Msk);// I2C0 start
////	while(!(I2C0->I2CON & I2C_I2CON_SI_Msk));
////	I2C0->I2CADDR0  = (BMP180_ADDR << 1) | I2C_GCMODE_DISABLE; //set slave address
////	I2C0->I2CDAT = address; //SendData
////
////	(I2C0->I2CON = (I2C0->I2CON | I2C_I2CON_SI_Msk) | I2C_I2CON_STA_Msk);// I2C0 start
////	I2C0->I2CADDR0  = (BMP180_ADDR << 1) | I2C_GCMODE_ENABLE; //set slave address
////
////
////}
////int32_t BMP180_ReadCalibrationData(uint8_t *calibration_data){
////	(I2C0->I2CON = (I2C0->I2CON | I2C_I2CON_SI_Msk) | I2C_I2CON_STA_Msk);
////
////	while(!(I2C0->I2CON & I2C_I2CON_SI_Msk));
////
////}
///**
//  * @brief      Specify two bytes register address and write multi bytes to Slave
//  *
//  * @param[in]  i2c            Point to I2C peripheral
//  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
//  * @param[in]  u16DataAddr     Specify a address (2 bytes) of data write to
//  * @param[in]  data[]          A data array for write data to Slave
//  * @param[in]  u32wLen         How many bytes need to write to Slave
//  *
//  * @return     A length of how many bytes have been transmitted.
//  *
//  * @details    The function is used for I2C Master specify two bytes address that multi data write to in Slave.
//  *
//  */
//
//uint32_t I2C_WriteMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen)
//{
//    uint8_t u8Xfering = 1U, u8Err = 0U, u8Addr = 1U, u8Ctrl = 0U;
//    uint32_t u32txLen = 0U;
//    uint32_t u32TimeOutCount;
//
//    g_I2C_i32ErrCode = 0;
//
//    I2C_START(i2c);                                                     /* Send START */
//
//    while (u8Xfering && (u8Err == 0U))
//    {
//        u32TimeOutCount = SystemCoreClock;
//        I2C_WAIT_READY(i2c)
//        {
//            u32TimeOutCount--;
//            if(u32TimeOutCount == 0)
//            {
//                g_I2C_i32ErrCode = I2C_TIMEOUT_ERR;
//                break;
//            }
//        }
//
//        switch (I2C_GET_STATUS(i2c))
//        {
//        case 0x08:
//            I2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1U | 0x00U));               /* Write SLA+W to Register I2CDAT */
//            u8Ctrl = I2C_I2CON_SI;                                        /* Clear SI */
//            break;
//
//        case 0x18:                                                      /* Slave Address ACK */
//            I2C_SET_DATA(i2c, (uint8_t)((u16DataAddr & 0xFF00U) >> 8U));  /* Write Hi byte address of register */
//            break;
//
//        case 0x20:                                                      /* Slave Address NACK */
//        case 0x30:                                                      /* Master transmit data NACK */
//            u8Ctrl = I2C_I2CON_STO_SI;                                    /* Clear SI and send STOP */
//            u8Err = 1U;
//            break;
//
//        case 0x28:
//            if (u8Addr)
//            {
//                I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFFU));       /* Write Lo byte address of register */
//                u8Addr = 0U;
//            }
//            else if ((u32txLen < u32wLen) && (u8Addr == 0U))
//                I2C_SET_DATA(i2c, data[u32txLen++]);                    /* Write data to Register I2CDAT*/
//            else
//            {
//                u8Ctrl = I2C_I2CON_STO_SI;                                /* Clear SI and send STOP */
//                u8Xfering = 0U;
//            }
//
//            break;
//
//        case 0x38:                                                      /* Arbitration Lost */
//        default:                                                        /* Unknow status */
//            I2C_SET_CONTROL_REG(i2c, I2C_I2CON_STO_SI);                   /* Clear SI and send STOP */
//            u8Ctrl = I2C_I2CON_SI;
//            u8Err = 1U;
//            break;
//        }
//
//        I2C_SET_CONTROL_REG(i2c, u8Ctrl);                               /* Write controlbit to I2C_CTL register */
//    }
//
//    u32TimeOutCount = SystemCoreClock;
//    while ((i2c)->I2CON & I2C_I2CON_STO_Msk)
//    {
//        u32TimeOutCount--;
//        if(u32TimeOutCount == 0)
//        {
//            g_I2C_i32ErrCode = I2C_TIMEOUT_ERR;
//            break;
//        }
//    }
//
//    return u32txLen;                                                    /* Return bytes length that have been transmitted */
//}
//
//
//
//
//
///**
//  * @brief      Read multi bytes from Slave
//  *
//  * @param[in]  i2c            Point to I2C peripheral
//  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
//  * @param[out] rdata[]         A data array to store data from Slave
//  * @param[in]  u32rLen         How many bytes need to read from Slave
//  *
//  * @return     A length of how many bytes have been received
//  *
//  * @details    The function is used for I2C Master to read multi data bytes from Slave.
//  *
//  *
//  */
//uint32_t I2C_ReadMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen)
//{
//    uint8_t u8Xfering = 1U, u8Err = 0U, u8Ctrl = 0U;
//    uint32_t u32rxLen = 0U;
//    uint32_t u32TimeOutCount;
//
//    g_I2C_i32ErrCode = 0;
//
//    I2C_START(i2c);                                          /* Send START */
//
//    while (u8Xfering && (u8Err == 0U))
//    {
//        u32TimeOutCount = SystemCoreClock;
//        I2C_WAIT_READY(i2c)
//        {
//            u32TimeOutCount--;
//            if(u32TimeOutCount == 0)
//            {
//                g_I2C_i32ErrCode = I2C_TIMEOUT_ERR;
//                break;
//            }
//        }
//
//        switch (I2C_GET_STATUS(i2c))
//        {
//        case 0x08:
//            I2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1U) | 0x01U));  /* Write SLA+R to Register I2CDAT */
//            u8Ctrl = I2C_I2CON_SI;                             /* Clear SI */
//            break;
//
//        case 0x40:                                           /* Slave Address ACK */
//            u8Ctrl = I2C_I2CON_SI_AA;                          /* Clear SI and set ACK */
//            break;
//
//        case 0x48:                                           /* Slave Address NACK */
//            u8Ctrl = I2C_I2CON_STO_SI;                         /* Clear SI and send STOP */
//            u8Err = 1;
//            break;
//
//        case 0x50:
//            rdata[u32rxLen++] = (uint8_t) I2C_GET_DATA(i2c);  /* Receive Data */
//
//            if (u32rxLen < (u32rLen - 1))
//            {
//                u8Ctrl = I2C_I2CON_SI_AA;                             /* Clear SI and set ACK */
//            }
//            else
//            {
//                u8Ctrl = I2C_I2CON_SI;                                /* Clear SI */
//            }
//
//            break;
//
//        case 0x58:
//            rdata[u32rxLen++] = (uint8_t) I2C_GET_DATA(i2c);  /* Receive Data */
//            u8Ctrl = I2C_I2CON_STO_SI;                                /* Clear SI and send STOP */
//            u8Xfering = 0U;
//            break;
//
//        case 0x38:                                                  /* Arbitration Lost */
//        default:                                                    /* Unknow status */
//            I2C_SET_CONTROL_REG(i2c, I2C_I2CON_STO_SI);               /* Clear SI and send STOP */
//            u8Ctrl = I2C_I2CON_SI;
//            u8Err = 1U;
//            break;
//        }
//
//        I2C_SET_CONTROL_REG(i2c, u8Ctrl);                           /* Write controlbit to I2C_CTL register */
//    }
//
//    u32TimeOutCount = SystemCoreClock;
//    while ((i2c)->I2CON & I2C_I2CON_STO_Msk)
//    {
//        u32TimeOutCount--;
//        if(u32TimeOutCount == 0)
//        {
//            g_I2C_i32ErrCode = I2C_TIMEOUT_ERR;
//            break;
//        }
//    }
//
//    return u32rxLen;                                                /* Return bytes length that have been received */
//}
//void I2C0_IRQHandler(void)
//{
//    /* Read the status first */
//    uint32_t status = I2C0->I2CSTATUS;
//    uint8_t u8Ctrl = 0;
//
//    /* Judge the status */
//    switch (status)
//    {
//        /* START has been transmitted and prepare SLA+W */
//        case 0x08:
//            I2C0->I2CDAT = Device_Addr | I2C_WR;                            // SLA+W
//            u8Ctrl = I2C_I2CON_SI;                                                  // Clear SI
//            DataLen = 0;
//            break;
//
//        /* SLA+W has been transmitted and ACK has been received */
//        case 0x18:
//            I2C0->I2CDAT = Register_Addr;                                           // Register addr
//            u8Ctrl = I2C_I2CON_SI;                                                  // Clear SI
//            break;
//
//        /* SLA+W has been transmitted and NACK has been received */
//        case 0x20:
//            u8Ctrl = I2C_I2CON_STO_SI;                                              // Stop
//            printf("Slave Device returns NACK (SLA+W)!!!\n");
//            EndFlag = 0;
//            break;
//
//        /* DATA has been transmitted and ACK has been received */
//        case 0x28:
//
//            /* for write */
//            if (EndFlag & 0x80)
//            {
//                if (DataLen < MaxDataLen)
//                {
//                    I2C0->I2CDAT = pu8Data[DataLen++];                      // Next data
//                    u8Ctrl = I2C_I2CON_SI;                                              // Clear SI
//                }
//                else
//                {
//                    u8Ctrl = I2C_I2CON_STO_SI;                                      // Last data had been transmitted, then Stop
//                    EndFlag = 0;
//                }
//            }
//            /* for read */
//            else
//                u8Ctrl = I2C_I2CON_STA_SI;                                          // Register addr had been transmitted, then Repeat Start
//
//            break;
//
//        /* DATA has been transmitted and NACK has been received */
//        case 0x30:
//            u8Ctrl = I2C_I2CON_STO_SI;                                              // Stop
//            printf("Slave Device returns NACK (DATA)!!!\n");
//            EndFlag = 0;
//            break;
//
//        /* Arbitration Lost */
//        case 0x38:
//            u8Ctrl = I2C_I2CON_STO_SI;                                              // Stop
//            printf("Arbitration Lost!!!\n");
//            EndFlag = 0;
//            break;
//
//        //-----------------------------------------------------------------------------------------------------------------------
//        /* Repeat START has been transmitted and prepare SLA+R */
//        case 0x10:
//            I2C0->I2CDAT = Device_Addr | I2C_RD;                            // SLA+R
//            u8Ctrl = I2C_I2CON_SI;                                                      // Clear SI
//            DataLen = 0;
//            break;
//
//        /* SLA+R has been transmitted and ACK has been received */
//        case 0x40:
//            if (MaxDataLen > 1)
//                u8Ctrl = I2C_I2CON_SI_AA;                                               // Prepare to return ACK for next data
//            else
//                u8Ctrl = I2C_I2CON_SI;                                                  // Last data has been received, NACK will be returned
//
//            break;
//
//        /* SLA+R has been transmitted and NACK has been received */
//        case 0x48:
//            u8Ctrl = I2C_I2CON_STO_SI;                                              // Stop
//            printf("Slave Device returns NACK (SLA+R)!!!\n");
//            EndFlag = 0;
//            break;
//
//        /* DATA has been received and ACK has been returned */
//        case 0x50:
//            pu8Data[DataLen++] = I2C0->I2CDAT;
//
//            if (DataLen < MaxDataLen - 1)
//                u8Ctrl = I2C_I2CON_SI_AA;                                               // Prepare to return ACK for next data
//            else
//                u8Ctrl = I2C_I2CON_SI;                                                  // Last data has been received, NACK will be returned
//
//            break;
//
//        /* DATA has been received and NACK has been returned */
//        case 0x58:
//            pu8Data[DataLen] = I2C0->I2CDAT;
//            u8Ctrl = I2C_I2CON_STO_SI;                                              // All of datas have been received, then Stop
//            EndFlag = 0;
//            break;
//
//        /* Default */
//        default:
//            printf("Status 0x%x is NOT processed!!!\n", status);
//            break;
//    }
//
//    I2C_SET_CONTROL_REG(I2C0, u8Ctrl);
//}
//
//void I2C_Write_Byte(uint8_t device, uint8_t addr, uint8_t data)
//{
//    Device_Addr = device;
//    Register_Addr = addr;
//    pu8Data = &data;
//    MaxDataLen = 1;
//    EndFlag = 0x81;                 // for write
//    /* Send Start signal */
//    I2C_START(I2C0);
//
//    /* Wait to I2C complete */
//    while (EndFlag);
//}
//
//uint8_t I2C_Read_Byte(uint8_t device, uint8_t addr)
//{
//    uint8_t data;
//    Device_Addr = device;
//    Register_Addr = addr;
//    pu8Data = &data;
//    MaxDataLen = 1;
//    EndFlag = 0x01;                 // for read
//    /* Send Start signal */
//    I2C_START(I2C0);
//
//    /* Wait to I2C complete */
//    while (EndFlag);
//
//    return data;
//}

//uint32_t I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen)
//{
//    uint8_t u8Xfering = 1U, u8Err = 0U, u8Addr = 1U, u8Ctrl = 0U;
//    uint32_t u32rxLen = 0U;
//    uint32_t u32TimeOutCount;
//
//    g_I2C_i32ErrCode = 0;
//
//    I2C_START((i2c));                                                   /* Send START */
//
//    while (u8Xfering && (u8Err == 0U))
//    {
//        u32TimeOutCount = SystemCoreClock;
//        I2C_WAIT_READY((i2c))
//        {
//            u32TimeOutCount--;
//            if(u32TimeOutCount == 0)
//            {
//                g_I2C_i32ErrCode = I2C_TIMEOUT_ERR;
//                break;
//            }
//        }
//
//        switch (I2C_GET_STATUS((i2c)))
//        {
//        case 0x08:
//            I2C_SET_DATA((i2c), (uint8_t)(u8SlaveAddr << 1U | 0x00U));             /* Write SLA+W to Register I2CDAT */
//            u8Ctrl = I2C_I2CON_SI;                                      /* Clear SI */
//            break;
//
//        case 0x18:                                                    /* Slave Address ACK */
//            I2C_SET_DATA((i2c), (uint8_t)((u16DataAddr & 0xFF00U) >> 8U));/* Write Hi byte address of register */
//            break;
//
//        case 0x20:                                                    /* Slave Address NACK */
//        case 0x30:                                                    /* Master transmit data NACK */
//            u8Ctrl = I2C_I2CON_STO_SI;                                  /* Clear SI and send STOP */
//            u8Err = 1U;
//            break;
//
//        case 0x28:
//            if (u8Addr)
//            {
//                I2C_SET_DATA((i2c), (uint8_t)(u16DataAddr & 0xFFU));     /* Write Lo byte address of register */
//                u8Addr = 0U;
//            }
//            else
//                u8Ctrl = I2C_I2CON_STA_SI;                              /* Clear SI and send repeat START */
//
//            break;
//
//        case 0x10:
//            I2C_SET_DATA((i2c), (uint8_t)((u8SlaveAddr << 1U) | 0x01U));           /* Write SLA+R to Register I2CDAT */
//            u8Ctrl = I2C_I2CON_SI;                                      /* Clear SI */
//            break;
//
//        case 0x40:                                                    /* Slave Address ACK */
//            u8Ctrl = I2C_I2CON_SI_AA;                                   /* Clear SI and set ACK */
//            break;
//
//        case 0x48:                                                    /* Slave Address NACK */
//            u8Ctrl = I2C_I2CON_STO_SI;                                  /* Clear SI and send STOP */
//            u8Err = 1U;
//            break;
//
//        case 0x50:
//            rdata[u32rxLen++] = (uint8_t) I2C_GET_DATA((i2c));    /* Receive Data */
//
//            if (u32rxLen < (u32rLen - 1U))
//            {
//                u8Ctrl = I2C_I2CON_SI_AA;                               /* Clear SI and set ACK */
//            }
//            else
//            {
//                u8Ctrl = I2C_I2CON_SI;                                  /* Clear SI */
//            }
//
//            break;
//
//        case 0x58:
//            rdata[u32rxLen++] = (uint8_t) I2C_GET_DATA((i2c));    /* Receive Data */
//            u8Ctrl = I2C_I2CON_STO_SI;                                  /* Clear SI and send STOP */
//            u8Xfering = 0U;
//            break;
//
//        case 0x38:                                                    /* Arbitration Lost */
//        default:                                                      /* Unknow status */
//            I2C_SET_CONTROL_REG((i2c), I2C_I2CON_STO_SI);                 /* Clear SI and send STOP */
//            u8Ctrl = I2C_I2CON_SI;
//            u8Err = 1U;
//            break;
//        }
//
//        I2C_SET_CONTROL_REG((i2c), u8Ctrl);                             /* Write controlbit to I2C_CTL register */
//    }
//
//    u32TimeOutCount = SystemCoreClock;
//    while ((i2c)->I2CON & I2C_I2CON_STO_Msk)
//    {
//        u32TimeOutCount--;
//        if(u32TimeOutCount == 0)
//        {
//            g_I2C_i32ErrCode = I2C_TIMEOUT_ERR;
//            break;
//        }
//    }
//
//    return u32rxLen;                                                  /* Return bytes length that have been received */
//}
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
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
///*---------------------------------------------------------------------------------------------------------*/
///*  I2C Rx Callback Function                                                                               */
///*---------------------------------------------------------------------------------------------------------*/
//void I2C_MasterRx(uint32_t u32Status)
//{
//    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
//    {
//        I2C0->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
//    {
//        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
//    {
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
//    }
//    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
//    {
//        if(g_u8DataLen != 2)
//        {
//            I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//        }
//        else
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
//        }
//    }
//    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
//    {
//        I2C0->I2CDAT = ((g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
//    {
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
//    {
//        g_u8RxData = I2C0->I2CDAT;
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
//        g_u8EndFlag = 1;
//    }
//    else
//    {
//        /* TO DO */
//        printf("Status 0x%x is NOT processed\n", u32Status);
//    }
//}
//
///*---------------------------------------------------------------------------------------------------------*/
///*  I2C Tx Callback Function                                                                               */
///*---------------------------------------------------------------------------------------------------------*/
//void I2C_MasterTx(uint32_t u32Status)
//{
//    if(u32Status == 0x08)                       /* START has been transmitted */
//    {
//        I2C0->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
//    {
//        I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//    }
//    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
//    {
//        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
//    }
//    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
//    {
//        if(g_u8DataLen != 3)
//        {
//            I2C0->I2CDAT = g_au8TxData[g_u8DataLen++];
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
//        }
//        else
//        {
//            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
//            g_u8EndFlag = 1;
//        }
//    }
//    else
//    {
//        /* TO DO */
//        printf("Status 0x%x is NOT processed\n", u32Status);
//    }
//}
//void I2C_Write(uint8_t addr, uint8_t data) {
//    I2C_START(I2C0);                          // Send START
//    I2C_WAIT_READY(I2C0);                     // Wait for START to be transmitted
//
//    I2C_SET_DATA(I2C0, (addr << 1));          // Send slave address with write bit
//    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_Msk);    // Clear SI flag
//    I2C_WAIT_READY(I2C0);                     // Wait for address to be transmitted
//
//    I2C_SET_DATA(I2C0, data);                 // Send register address
//    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_Msk);    // Clear SI flag
//    I2C_WAIT_READY(I2C0);                     // Wait for data to be transmitted
//
//    I2C_STOP(I2C0);                           // Send STOP
//}
//
//uint8_t I2C_Read(uint8_t addr) {
//    uint8_t data;
//
//    I2C_START(I2C0);                          // Send START
//    I2C_WAIT_READY(I2C0);                     // Wait for START to be transmitted
//
//    I2C_SET_DATA(I2C0, (addr << 1) | 1);      // Send slave address with read bit
//    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_Msk);    // Clear SI flag
//    I2C_WAIT_READY(I2C0);                     // Wait for address to be transmitted
//
//    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_Msk | I2C_I2CON_AA_Msk); // Clear SI flag and set AA for ACK
//    I2C_WAIT_READY(I2C0);                     // Wait for data to be received
//
//    data = I2C_GET_DATA(I2C0);                // Read data
//
//    I2C_STOP(I2C0);                           // Send STOP
//
//    return data;
//}

/* Use Multi Bytes Read from EEPROM (Two Registers) */
//
//while(I2C_ReadMultiBytesTwoRegs(I2C0, BMP180_ADDRESS, REGISTER_ADDRESS, calibdata, 1) < 1);
//printf("|InSaN|\n");
// while (1)
// {
//    printf("Read Data:  ");
//    for(i = 0; i < 1; i++)
//        printf("  %d",calibdata[i]);
//    printf("\n");
//    printf("Multi bytes Read access Pass.....\n\n");
//  }

//    for( uint32_t i=0; i < 23 ;i++)
//    {
//    I2C_Write(BMP180_ADDRESS, REGISTER_ADDRESS);   // Send the register address
//    calibdata[0] = I2C_Read(BMP180_ADDRESS);       // Read the data

//    printf("Calibration data: 0x%x\n", calibdata[0]);
//   }

////    for(i = 0; i < 22; i++)
////    {
//        g_au8TxData[0] = (uint8_t)((0xAA));
//
//
//        g_u8DataLen = 0;
//        g_u8EndFlag = 0;
//
//        /* I2C function to write data to slave */
//        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
//
//        /* I2C as master sends START signal */
//        I2C_SET_CONTROL_REG(I2C0,   I2C_I2CON_STA);
//
//        /* Wait I2C Tx Finish */
//        while(g_u8EndFlag == 0);
//        g_u8EndFlag = 0;
//
//        /* I2C function to read data from slave */
//        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
//
//        /* Set EEPROM Addres */
//        g_u8DataLen = 0;
//        g_u8DeviceAddr = 0xFF;
//
//        I2C_SET_CONTROL_REG(I2C0,   I2C_I2CON_STA);
//
//        /* Wait I2C Rx Finish */
//        while(g_u8EndFlag == 0);
//
//        /* Compare data */
////        if(g_u8RxData != g_au8TxData[2])
////        {
////            printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
////            return -1;        }
//        printf("Calibration data: 0x%x\n", calibdata[0]);
//
////    printf("I2C Access EEPROM Test OK\n");


//s_I2C0HandlerFn = NULL;
