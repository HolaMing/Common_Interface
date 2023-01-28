/*
 * @Descripttion: 
 * @version: 
 * @Author: Newt
 * @Date: 2022-10-25 17:20:34
 * @LastEditors: Newt
 * @LastEditTime: 2022-10-26 16:37:01
 */
/*
 * Copyright (c) 2020 LeapFive.
 *
 * This file is part of
 *     *** LeapFive Software Dev Kit ***
 *      (see www.leapfive.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of LeapFive Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __OTA_H__
#define __OTA_H__

#include <stdio.h>
#include <string.h>
#include <aos/kernel.h>
#include <hal_boot2.h>
#include <hal_sys.h>
#include <utils_sha256.h>
#include <lf_mtd.h>

#define CHECK_IMG_BUF_SIZE   3 * 1024
#define OTA_HEADER_SIZE      (sizeof(ota_header_t))

typedef struct {
    union {
        struct {
            uint8_t header[16];

            uint8_t type[4];//RAW XZ
            uint32_t len;//body len
            uint8_t pad0[8];

            uint8_t ver_hardware[16];
            uint8_t ver_software[16];

            uint8_t sha256[32];
        } s;
        uint8_t _pad[512];
    } u;
} ota_header_t;

typedef struct ota_parm_s 
{
    lf_mtd_handle_t mtd_handle;
    uint32_t file_size;
}ota_parm_t;

int hal_ota_check_header(uint8_t *ota_data, char *ota_ver);
int hal_ota_start(uint32_t file_size);
int hal_ota_update(uint32_t filesize, uint32_t offset, uint8_t *buf, uint32_t buf_len);
int hal_ota_finish(uint8_t check_hash, uint8_t auto_reset);

#endif /* __OTA_H__ */