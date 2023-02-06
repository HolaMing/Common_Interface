#include "lf_xqxy_uart.h"
#include <string.h>
#include <stdio.h>
#include <blog.h>
#include <vfs.h>
#include <aos/kernel.h>
#include <aos/yloop.h>
#include <event_device.h>
#include <device/vfs_uart.h>
#include <lf_uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include <lf_gpio.h>
#include <easyflash.h>
#include <hal_sys.h>

static int uart_fd;                                              //串口通信描述符
static uint8_t id_buf[ID_DATA_LEN] = {0x01,0x01,0x01,0x02,0x02,0x02,0x03,0x03,0x03,0x04,0x04,0x04};            //计量模块ID信息数据帧


/**
 * @brief: compare time 
 * @param {uint32_t} t_end
 * @param {uint32_t} t_now
 * @return {*}  left time
 */
static uint32_t _time_left(uint32_t t_end, uint32_t t_now)
{
    uint32_t t_left;

    if (t_end > t_now) {
        t_left = t_end - t_now;
    } else {
        t_left = 0;
    }

    return t_left;
}

/**
 * @brief  数据校验
 * @param  ptr: 需要校验的数据
 * @param  len: 数据长度
 * @return 校验值
 */
static uint8_t data_check(uint8_t *ptr, uint8_t len)
{
    uint16_t data_result = 0; 

    for (uint8_t i = 0; i < len; i++) {
        data_result += ptr[i];
    }
    data_result = ~data_result + 0x33;

    return (data_result % 256) & 0xff; 
}

/**
 * @brief  串口初始化
 * @param  无
 * @return 0：成功 -1：失败
 */
static void uart1_cfg(int fd, void *param)
{
    int ret = -1;

    ret = aos_ioctl(uart_fd, IOCTL_UART_IOC_BAUD_MODE, UART_BAUD);
    if (ret < 0) {
        log_error("uart set baud failed %d\r\n", ret);
    }

    lf_uart_setconfig(1, UART_BAUD, UART_PARITY_EVEN);
}

/**
 * @brief: pack electric data
 * @param {uint8_t} *buf 
 * @param {float} value
 * @param {uint8_t} len
 * @return {*}
 */
static void pack_valid_data(uint8_t *buf, float value, uint8_t len)
{
    uint8_t tmp0, tmp1, tmp2, tmp3;
    uint8_t units, tens;
    
    memset(buf, 0, len);
    if (len >= 4) {
        tmp0 = (int)(value * 100) / 1000000;
        tens = tmp0 / 10;
        units = tmp0 % 10;
        buf[0] = (tens << 4) | units;
        tmp1 = (int)(value * 100) / 10000 % 100;
        tens = tmp1 / 10;
        units = tmp1 % 10;
        buf[1] = (tens << 4) | units;
        tmp2 = (int)(value * 100) % 10000 / 100;
        tens = tmp2 / 10;
        units = tmp2 % 10;
        buf[2] = (tens << 4) | units;
        tmp3 = (int)(value * 100) % 100;
        tens = tmp3 / 10;
        units = tmp3 % 10;
        buf[3] = (tens << 4) | units;
    }

    // printf("temp = %02u %02u %02u %02u\r\n", tmp0, tmp1, tmp2, tmp3);
}

static void pack_time_data(uint8_t *time_buf, uint8_t *valid_buf, uint8_t len)
{
    uint8_t units, tens;
    for (uint8_t i = 0; i < len; i++) {
        tens = time_buf[i] / 10;
        units = time_buf[i] % 10;
        valid_buf[i] = (tens << 4) | units;
    }
}

/**
 * @brief: 组包
 * @return {*}
 */
static void pack_send_buf(uint8_t *buf, uint8_t buf_len, uint8_t head, uint8_t cmd_type, uint8_t func_id, \
                             uint8_t *data_buf, uint8_t data_len)
{
    buf[0] = head;
    buf[1] = cmd_type;
    buf[2] = func_id;
    buf[3] = data_len;
    memcpy(buf + 4, data_buf, data_len);
    buf[buf_len-1] = data_check(buf,buf_len-1);
}

/**
 * @brief: ensure uart send data integrity and set timeout
 * @param {int} fd
 * @param {const uint8_t} *data
 * @param {uint8_t} len
 * @param {uint32_t} timeout_ms
 * @return {*} 0 success
 */
static int uart_write_data(int fd, const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    int      ret;
    uint32_t len_sent;
    uint32_t t_end;

    t_end    = xTaskGetTickCount() + timeout_ms;
    len_sent = 0;
    ret      = 1; /* send one time if timeout_ms is value 0 */

    do {
        if (ret > 0) {
            ret = aos_write(fd, data + len_sent, len - len_sent);
            if (ret > 0) {
                len_sent += ret;
            }
            else if (ret == 0) {
                log_error("No data be sent.\r\n");
            }
            else {
                log_error("uart send fault.\r\n");
                break;
            }
        }
    } while ((len_sent < len) && (_time_left(t_end, xTaskGetTickCount()) > 0));

    return len_sent > 0 ? 0 : ret;
}

/**
 * @brief  发送数据包
 * @param  recv_valid_data: 接收到的有效数据
 * @param  func_id: 功能标识符
 * @return 0：成功 
 */
static uint8_t uart1_send_package(uint8_t func_id)
{
    int8_t ret;
    uint8_t send_buf[SEND_DATA_LEN] = {0}; 
    uint8_t data_buf[SEND_DATA_LEN - UART_OTHER_DATA_LEN] = {0}; 
    uint8_t buf_len = 0;
    uint8_t data_len = 0;
    uint8_t data_buf_offset = ID_DATA_LEN;
    float value = 0;
    uint8_t valid_buf[MAX_VAL_LEN] = {1 ,2 ,3, 4, 5, 6, 7, 8, 9, 10, 11}; /* default value*/

    memcpy(data_buf, id_buf, data_buf_offset); /* set ID information */

    switch (func_id)
    {
        case ID__NUM:
            //发送ID
            data_len = ID_DATA_LEN;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case SPE_ID:
            //发送ID
            data_len = ID_DATA_LEN;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_ELEC:
            //获取总有功电量
            data_len = ID_DATA_LEN + 4;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 123456.78;

            pack_valid_data(valid_buf, value, 4);
            memcpy(data_buf + data_buf_offset, valid_buf, 4);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_AELEC:
            //发送A相有功电量
            data_len = ID_DATA_LEN + 4;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 123456.78;

            pack_valid_data(valid_buf, value, 4);
            memcpy(data_buf + data_buf_offset, valid_buf, 4);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_BELEC:
            //发送B相有功电量
            data_len = ID_DATA_LEN + 4;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 123456.78;

            pack_valid_data(valid_buf, value, 4);
            memcpy(data_buf + data_buf_offset, valid_buf, 4);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_CELEC:
            //发送C相有功电量
            data_len = ID_DATA_LEN + 4;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 123456.78;

            pack_valid_data(valid_buf, value, 4);
            memcpy(data_buf + data_buf_offset, valid_buf, 4);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case A_VOL:
            //发送A相电压
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 123.4;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case B_VOL:
            //发送B相电压
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 123.4;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case C_VOL:
            //发送C相电压
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 123.4;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case A_CUR:
            //发送A相电流
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case B_CUR:
            //发送B相电流
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case C_CUR:
            //发送C相电流
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case FREQ:
            //发送频率
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.34;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_ACTPOW:
            //发送总有功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case A_ACTPOW:
            //发送A相有功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case B_ACTPOW:
            //发送B相有功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case C_ACTPOW:
            //发送C相有功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_REAPOW:
            //发送总无功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case A_REAPOW:
            //发送A相无功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case B_REAPOW:
            //发送B相无功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case C_REAPOW:
            //发送C相无功功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_APPAPOW:
            //发送总视在功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case A_APPAPOW:
            //发送A相视在功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case B_APPAPOW:
            //发送B相视在功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case C_APPAPOW:
            //发送C相视在功率
            data_len = ID_DATA_LEN + 3;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 12.3456;

            pack_valid_data(valid_buf, value, 3);
            memcpy(data_buf + data_buf_offset, valid_buf, 3);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TOTAL_PFC:
            //发送总功率因数
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 1.234;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case A_PFC:
            //发送A相功率因数
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 1.234;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case B_PFC:
            //发送B相功率因数
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 1.234;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case C_PFC:
            //发送C相功率因数
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value */
            value = 1.234;

            pack_valid_data(valid_buf, value, 2);
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case TEMP:
            //发送温度
            data_len = ID_DATA_LEN + 2;
            log_error("dont define data format"); 
            break;
        case TIME:
            //发送时间
            data_len = ID_DATA_LEN + 6;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign timer_buf about time information*/
            uint8_t timer_buf[6] = {23, 1, 13, 14, 15, 16};

            pack_time_data(timer_buf, valid_buf, 6);
            memcpy(data_buf + data_buf_offset, valid_buf, 6);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            break;
        case ELEC_FIND:
            {
                //用电数据查询
                data_len = ID_DATA_LEN + 11;
                buf_len = data_len + UART_OTHER_DATA_LEN;
                /* assign value about active power and electric quantity */
                uint8_t time_stamp[4] = {0};
                float active_power = 12.3456;
                float power = 123456.78;

                memcpy(valid_buf, time_stamp, 4);
                pack_valid_data(valid_buf + 4, active_power, 3);
                pack_valid_data(valid_buf + 4 + 3, power, 4);
                memcpy(data_buf + data_buf_offset, valid_buf, 11);
                pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
                break;
            }
        case OPPO_FIND:
            {
                //数据反向查询
                data_len = ID_DATA_LEN + 11;
                buf_len = data_len + UART_OTHER_DATA_LEN;
                /* assign value about active power and electric quantity */
                uint8_t time_stamp[4] = {0};
                float active_power = 12.3456;
                float power = 123456.78;

                memcpy(valid_buf, time_stamp, 4);
                pack_valid_data(valid_buf + 4, active_power, 3);
                pack_valid_data(valid_buf + 4 + 3, power, 4);
                memcpy(data_buf + data_buf_offset, valid_buf, 11);
                pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
                break;
            }
        case ALARM_STATE:
            //告警状态字
            data_len = ID_DATA_LEN + 2;
            buf_len = data_len + UART_OTHER_DATA_LEN;
            /* assign value about electric status*/
            float A_volt = 200.1;
            float B_volt = 200.1;
            float C_volt = 200.1;
            float A_current = 0.1;
            float B_current = 0.1;
            float C_current = 0.1;
            uint8_t lines_error_flag = 0;
            uint8_t store_error_flag = 0;

            memset(valid_buf, 0, sizeof(valid_buf));
            if (A_current - CURRENT_MAX > 0) {
                valid_buf[0] |= 0x01;
            }
            if (B_current - CURRENT_MAX > 0) {
                valid_buf[0] |= 0x02;
            }
            if (C_current - CURRENT_MAX > 0) {
                valid_buf[0] |= 0x04;
            }
            if (VOLT_MIN - A_volt >= 0) {
                valid_buf[0] |= 0x08;
            }
            if (VOLT_MIN - B_volt > 0) {
                valid_buf[0] |= 0x10;
            }
            if (VOLT_MIN - C_volt > 0) {
                valid_buf[0] |= 0x20;
            }
            if (A_volt - VOLT_MAX > 0) {
                valid_buf[0] |= 0x40;
            }
            if (B_volt - VOLT_MAX > 0) {
                valid_buf[0] |= 0x80;
            }
            if (C_volt - VOLT_MAX > 0) {
                valid_buf[1] |= 0x01;
            }
            if (lines_error_flag > 0) {
                valid_buf[1] |= 0x02;
            }
            if (store_error_flag > 0) {
                valid_buf[1] |= 0x80;
            }
            
            memcpy(data_buf + data_buf_offset, valid_buf, 2);
            pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_READ, func_id, data_buf, data_len);
            
            break;
        default:
            break;
    }

    ret = uart_write_data(uart_fd, send_buf, buf_len, 0);

    printf("send data = ");
    for (uint8_t i = 0; i < buf_len; i++) {
        printf("%02x ", send_buf[i]);
    }
    printf("\r\n");

    return ret;
}

/**
 * @brief  根据串口1接收数据类型进行相应功能
 * @param  data: 获取到的数据帧
 * @param  total_len: 数据帧长度
 * @param  cmd: 数据帧消息类型
 * @return 0：成功 -1：失败
 */
static int8_t uart1_recv_data_parse(uint8_t *data, uint8_t total_len, uint8_t cmd)
{
    printf("parse data = ");
    for (uint8_t i = 0; i < total_len; i++) {
        printf("%02x ", data[i]);
    }
    printf("\r\n");
    /* check ID */
    uint8_t func_id = data[2];
    if (func_id != ID__NUM && func_id != SPE_ID) {
        uint8_t recv_id_buf[total_len - UART_OTHER_DATA_LEN];
        memcpy(recv_id_buf, data + UART_FRAME_HEAD_LEN, sizeof(recv_id_buf));
        if (memcmp(recv_id_buf, id_buf, ID_DATA_LEN) != 0) {
            log_error("recv_id not match id_buf\r\n");
            return -1;
        }
    }
    /* check */
    uint8_t crc = data_check(data, total_len-1);
    printf("crc ================== %02x\r\n",crc);
    if (crc != data[total_len-1]) {
        log_error("crc failed\r\n");
        return -1;
    }
    /* detect cmd type */
    switch (cmd)
    {
        case AWAKEN_READ:
            uart1_send_package(func_id);
            break;
        case AWAKEN_WRITE:
            if(func_id == TIME){
                /* assign flag about writing time*/
                uint8_t flag = 0; /* 0 means successful, 1 means failed */

                uint8_t data_len, buf_len;
                uint8_t send_buf[SEND_DATA_LEN];
                uint8_t data_buf[SEND_DATA_LEN - UART_OTHER_DATA_LEN];

                data_len = ID_DATA_LEN + 1;
                buf_len = data_len + UART_OTHER_DATA_LEN;
                memcpy(data_buf, id_buf, ID_DATA_LEN); /* set ID information */
                if (flag) {
                    data_buf[ID_DATA_LEN] = 1;  
                }
                else {
                    data_buf[ID_DATA_LEN] = 0; 
                }
                pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_WRITE, func_id, data_buf, data_len);
                uart_write_data(uart_fd, send_buf, buf_len, 0);

                printf("send data = ");
                for (uint8_t i = 0; i < buf_len; i++) {
                    printf("%02x ", send_buf[i]);
                }
                printf("\r\n");
            } 
            else {
                return -1;
            }
            break;
        case AWAKEN_UPLOAD:
            
            break;
        case AWAKEN_CTRL:
            if(func_id == CLEAR_POWER){
                /* transfer message about clearing eletricity*/
                
                uint8_t data_len, buf_len;
                uint8_t send_buf[SEND_DATA_LEN];
                uint8_t data_buf[SEND_DATA_LEN - UART_OTHER_DATA_LEN];

                memcpy(data_buf, id_buf, ID_DATA_LEN); /* set ID information */
                data_len = ID_DATA_LEN;
                buf_len = data_len + UART_OTHER_DATA_LEN;
                pack_send_buf(send_buf, buf_len, AWAKEN_HEAD, AWAKEN_CTRL, func_id, data_buf, data_len);
                uart_write_data(uart_fd, send_buf, buf_len, 0);

                printf("send data = ");
                for (uint8_t i = 0; i < buf_len; i++) {
                    printf("%02x ", send_buf[i]);
                }
                printf("\r\n");
            } else {
                return -1;
            }
            break;
        default:
            break;
    }

    return 0;
}

/**
 * @brief: uart1 receive packets task
 * @param {int} fd
 * @param {void} *pvParameters
 * @return {*}
 */
static void uart1_read_task(int fd, void *pvParameters)
{
    static uint8_t recv_buf[RCV_DATA_LEN] = {0};
    static uint8_t recv_len = 0;
    int16_t ret = 0;
    uint8_t data_len = 0;
    uint16_t index = 0;

    ret = aos_read(fd, recv_buf + recv_len, sizeof(recv_buf) - recv_len);
    if (ret <= 0) {
        return; // no data received
    }
    else {
        recv_len += ret;
    }
    // check package complete
    while (index < recv_len)
    {
        if (recv_buf[index] == AWAKEN_HEAD)
            break;
        index++;
    }
    // skip no header data
    if (index > 0) 
    {
        memcpy(recv_buf, recv_buf + index, recv_len - index);
        recv_len -= index;
    }
    // 接收长度大于除数据内容以外的最小长度
    while (recv_len > UART_OTHER_DATA_LEN) 
    {
        data_len = recv_buf[UART_FRAME_LEN_INDEX] + UART_OTHER_DATA_LEN;
        // 至少完成一个包的接收
        if (recv_len >= data_len) {
            uart1_recv_data_parse(recv_buf, data_len, recv_buf[UART_FRAME_TYPE_INDEX]);
            if (recv_len > data_len) {
                memcpy(recv_buf, recv_buf + data_len, recv_len - data_len);
                recv_len -= data_len;
            }
            else {
                // complete parse all data
                memset(recv_buf, 0, recv_len);
                recv_len = 0;
                break;
            }
        }
        else {
            break;
        }
    }
}

/**
 * @brief  设置ID信息数据域
 * @param  
 * @return 0：成功 -1：失败
 */
uint8_t lf_uart1_set_id(lf_mod_info id_info)
{
    memset(id_buf, 0, sizeof(id_buf));
    id_buf[0]=id_info.ptl_ver;
    id_buf[1]=id_info.lock_ver;
    id_buf[2]=id_info.sw_ver;
    id_buf[3]=id_info.ted;
    id_buf[4]=id_info.fac_num;
    id_buf[5]=id_info.xh_num;
    id_buf[6]=id_info.batch_num;
    memcpy(id_buf + 7, id_info.uni_ide, sizeof(id_info.uni_ide));

    return 0; 
}

/**
 * @brief  串口线程入口
 * @param  无
 * @return 0：成功 -1：失败
 */
int lf_uart1_task_init(void)
{ 
    int ret = -1;

    uart_fd = aos_open("/dev/ttyS1", 1);

    if (uart_fd < 0) {
        log_error("open uart1 failed\r\n");
        return ret;
    }  
    else {
        uart1_cfg(0, NULL);
        printf("Init uart1 successfully\r\n");
        aos_poll_read_fd(uart_fd, uart1_read_task, NULL);
    }

    return 0;
}