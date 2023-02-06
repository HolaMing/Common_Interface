/*
 * @Descripttion: 
 * @version: 
 * @Author: Newt
 * @Date: 2022-12-01 17:39:19
 * @LastEditors: Newt
 * @LastEditTime: 2022-12-02 11:27:57
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

static uint32_t MON1[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //平年
static uint32_t MON2[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //闰年
static uint32_t FOUR_YEARS = (366 + 365 + 365 + 365);                         //每个四年的总天数
static uint32_t DAY_S = 24 * 60 * 60;                                        //每天的秒数

static void transform_month_day(uint32_t *remain_days, uint32_t *month, uint32_t *day, bool IsLeapYear)
{
    uint32_t *pMonths = (IsLeapYear == 1) ? MON2 : MON1;
    //循环减去12个月中每个月的天数，直到剩余天数小于等于0，就找到了对应的月份
    for (uint8_t i = 0; i < 12; ++i) {
        int32_t nTemp = *remain_days - pMonths[i];
        if (nTemp <= 0) {
            *month = i + 1;
            if (nTemp == 0) {   //表示刚好是这个月的最后一天，那么天数就是这个月的总天数了
                *day = pMonths[i];
            } 
            else {
                *day = *remain_days;
            }    
            return;
        }
        *remain_days = nTemp;
    }
}

static void timestamp_transform_time(uint32_t timestamp)
{
    uint32_t n_days, n_years, remain_days, remain_seconds;
    uint32_t year;
    uint32_t month, day, hour, minute;
    bool leap_year_flag = 0;

    printf("time = %u\r\n", timestamp);
    n_days = timestamp / DAY_S + 1;  
    n_years = (n_days / FOUR_YEARS) * 4;   
    remain_days = n_days % FOUR_YEARS;   
    printf("remain days = %d\r\n", remain_days);
    
    if (remain_days < 365) {

    } else if (remain_days < (365 * 2)) {
        n_years++;
        remain_days -= 365;
    } else if (remain_days < (365 * 3)) {
        n_years += 2;
        remain_days -= (365 * 2);
    } else {
        n_years += 3;
        remain_days -= (365 * 3);
        leap_year_flag = 1;
    }
    
    transform_month_day(&remain_days, &month, &day, leap_year_flag);
    printf("month = %u\r\n", month);
    remain_seconds = timestamp - (n_days - 1) * DAY_S;
    hour = remain_seconds / (60 * 60) + 8;
    if (hour >= 24) {
        hour -= 24;
        day++;
    }
    minute = remain_seconds % (60 * 60) / 60;
    year = 1970 + n_years;
    printf("%4u-%02u-%02u-%02u:%02u\r\n", year, month, day, hour, minute);     
}   

void main(void)
{
    timestamp_transform_time(1669948095);
}
