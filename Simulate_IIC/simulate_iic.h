/*
 * @Descripttion: 
 * @version: 
 * @Author: Newt
 * @Date: 2022-08-31 17:00:14
 * @LastEditors: Newt
 * @LastEditTime: 2022-09-01 10:35:19
 */
#ifndef __SIMULATE_IIC__H
#define __SIMULATE_IIC__H

/* plz add ur configuration below here */
#define DELAY_FOR_A_WHILE
#define GPIO_SDA
#define GPIO_SCL

#define ENABLE_SDA_OUT
#define ENABLE_SCL_OUT
#define ENABLE_SDA_IN

#define SET_SDA_LOW
#define SET_SDA_HIGH
#define SET_SCL_LOW
#define SET_SCL_HIGH
#define GET_SDA_LEVEL

typedef enum {
    IIC_OK,
    IIC_ERROR,
    IIC_NOACK
} simulateIICStatus;

simulateIICStatus simulate_iic_send_data(uint8_t deviceAddress, uint8_t *wDate, uint16_t wSize);
simulateIICStatus simulate_iic_read_data(uint8_t deviceAddress,uint8_t *rData, uint16_t rSize);

#endif /* __SIMULATE_IIC__H */