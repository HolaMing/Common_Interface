/*
 * @Descripttion:
 * @version:
 * @Author: Newt
 * @Date: 2022-08-31 16:58:48
 * @LastEditors: Newt
 * @LastEditTime: 2022-09-06 11:09:02
 */
#include <stdio.h>
#include <stdint.h>

#include "simulate_iic.h"

static void simulate_iic_start(void)
{
    ENABLE_SCL_OUT;
    ENABLE_SDA_OUT;

    SET_SCL_HIGH;
    SET_SDA_HIGH;
    DELAY_FOR_A_WHILE;

    SET_SDA_LOW;
    DELAY_FOR_A_WHILE;

    SET_SCL_LOW;
    DELAY_FOR_A_WHILE;
}

static void simualte_iic_stop(void)
{
    ENABLE_SDA_OUT;

    SET_SCL_LOW;
    SET_SDA_LOW;

    DELAY_FOR_A_WHILE;
    SET_SCL_HIGH;
    DELAY_FOR_A_WHILE;
    SET_SDA_HIGH;
}

static simulateIICStatus simulate_iic_wait_ack(void)
{
    uint8_t timeout = 255;

    SET_SDA_HIGH;
    DELAY_FOR_A_WHILE;

    SET_SCL_HIGH;
    ENABLE_SDA_IN;
    DELAY_FOR_A_WHILE;

    while (GET_SDA_LEVEL)
    {
        if (--timeout)
        {
            simualte_iic_stop();
            return IIC_ERROR;
        }
    }

    SET_SCL_LOW;
    DELAY_FOR_A_WHILE;

    return IIC_OK;
}

static void simulate_iic_ack(void)
{
    ENABLE_SDA_OUT;

    SET_SDA_LOW;
    DELAY_FOR_A_WHILE;

    SET_SCL_HIGH;
    DELAY_FOR_A_WHILE;

    SET_SCL_LOW;
    DELAY_FOR_A_WHILE;

    SET_SDA_HIGH;
    DELAY_FOR_A_WHILE;
}

static void simulate_iic_no_ack(void)
{
    ENABLE_SDA_OUT;

    SET_SDA_HIGH;
    DELAY_FOR_A_WHILE;

    SET_SCL_HIGH;
    DELAY_FOR_A_WHILE;

    SET_SCL_LOW;
    DELAY_FOR_A_WHILE;
}

static void simulate_iic_sendbyte(uint8_t data)
{
    SET_SCL_LOW;
    ENABLE_SDA_OUT;

    for (uint8_t count = 0; count < 8; count++)
    {
        if (data & 0x80)
        {
            SET_SDA_HIGH;
        }
        else
        {
            SET_SDA_LOW;
        }
        data <<= 1;

        // DELAY_FOR_A_WHILE;
        SET_SCL_HIGH;
        DELAY_FOR_A_WHILE;
        SET_SCL_LOW;
        // DELAY_FOR_A_WHILE;
    }
}

static uint8_t simulate_iic_readbyte(void)
{
    uint8_t data = 0;

    SET_SDA_HIGH;
    ENABLE_SDA_IN;

    for (uint8_t count = 0; count < 8; count++) {
        SET_SCL_HIGH;
        DELAY_FOR_A_WHILE;
        
        data <<= 1;
        if (GET_SDA_LEVEL) {
            data++;
        }
        
        SET_SCL_LOW;
        DELAY_FOR_A_WHILE;
    }

    return data;
}

/**
 * @brief: send data
 * @param {uint8_t} deviceAddress
 * @param {uint8_t} *wDate
 * @param {uint16_t} wSize
 */
simulateIICStatus simulate_iic_send_data(uint8_t deviceAddress, uint8_t *wDate, uint16_t wSize)
{
    simulate_iic_start();

    simulate_iic_sendbyte(deviceAddress);
    if (simulate_iic_wait_ack()) {
        return IIC_NOACK;
    }

    while (wSize--) {
        simulate_iic_sendbyte(*wDate);
        if (simulate_iic_wait_ack()) {
            return IIC_NOACK;
        }
        wDate++;
        DELAY_FOR_A_WHILE;
    }

    simualte_iic_stop();
    
    return IIC_OK;
}

/**
 * @brief: read data
 * @param {uint8_t} deviceAddress
 * @param {uint8_t} *rData
 * @param {uint16_t} rSize
 */
simulateIICStatus simulate_iic_read_data(uint8_t deviceAddress,uint8_t *rData, uint16_t rSize)
{
    simulate_iic_start();

    simulate_iic_sendbyte(deviceAddress);
    if (simulate_iic_wait_ack()) {
        return IIC_NOACK;
    }

    while (rSize--) {
        *rData = simulate_iic_readbyte();
        rData++;
        if (rData[rSize] == 0) {
            simulate_iic_no_ack();
        }
        else {
            simulate_iic_ack();
            DELAY_FOR_A_WHILE;
        }
    }
    
    simualte_iic_stop();
    
    return IIC_OK;
}