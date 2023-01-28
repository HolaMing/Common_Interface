/*
 * @Descripttion: 
 * @version: 
 * @Author: Newt
 * @Date: 2022-10-24 17:42:10
 * @LastEditors: Newt
 * @LastEditTime: 2022-10-26 16:16:19
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
#include <stdio.h>
#include <string.h>
#include <aos/kernel.h>
#include <hal_boot2.h>
#include <hal_sys.h>
#include <utils_sha256.h>
#include <lf_mtd.h>
#include "ota.h"

static ota_parm_t *ota_parm = NULL; 
static ota_header_t *ota_header = NULL;
static uint8_t sha256_img[32];
static HALPartition_Entry_Config ptEntry;

int hal_ota_check_header(uint8_t *ota_data, char *ota_ver)
{
    char str[33];//assume max segment size
    int i;
    uint32_t ota_len;
    ota_header = (ota_header_t *)ota_data;

    memcpy(str, ota_header->u.s.header, sizeof(ota_header->u.s.header));
    if (memcmp(ota_header->u.s.header, ota_ver, 16) != 0) {
        return -1;
    }
    str[sizeof(ota_header->u.s.header)] = '\0';
    printf("[OTA] [HEADER] ota header is ");
    printf(str);
    printf("\r\n");

    memcpy(str, ota_header->u.s.type, sizeof(ota_header->u.s.type));
    str[sizeof(ota_header->u.s.type)] = '\0';
    printf("[OTA] [HEADER] file type is ");
    printf(str);
    printf("\r\n");
    // if (strstr(str, "XZ")) {
    //     *use_xz = 1;
    // } else {
    //     *use_xz = 0;
    // }

    memcpy(&ota_len, &(ota_header->u.s.len), 4);
    printf("[OTA] [HEADER] file length (exclude ota header) is %lu\r\n", ota_len);

    memcpy(str, ota_header->u.s.ver_hardware, sizeof(ota_header->u.s.ver_hardware));
    str[sizeof(ota_header->u.s.ver_hardware)] = '\0';
    printf("[OTA] [HEADER] ver_hardware is ");
    printf(str);
    printf("\r\n");

    memcpy(str, ota_header->u.s.ver_software, sizeof(ota_header->u.s.ver_software));
    str[sizeof(ota_header->u.s.ver_software)] = '\0';
    printf("[OTA] [HEADER] ver_software is ");
    printf(str);
    printf("\r\n");

    memcpy(str, ota_header->u.s.sha256, sizeof(ota_header->u.s.sha256));
    memcpy(sha256_img, ota_header->u.s.sha256, sizeof(ota_header->u.s.sha256));
    str[sizeof(ota_header->u.s.sha256)] = '\0';
    printf("[OTA] [HEADER] sha256 is ");
    for (i = 0; i < sizeof(ota_header->u.s.sha256); i++) {
        printf("%02X", str[i]);
    }
    printf("\r\n");

    return 0;
}

int hal_ota_start(uint32_t file_size)
{
    int ret;
    uint32_t ota_size, ota_addr;
   
    if (ota_parm != NULL) {
        printf("ota had start\r\n");
        return 0;
    }
    
    ota_parm = (ota_parm_t *)aos_malloc(sizeof(ota_parm_t));
    if (NULL == ota_parm) {
        printf("have not enough memory\r\n");
        return -1;
    }
    memset(ota_parm, 0, sizeof(ota_parm_t));

    ota_parm->file_size = file_size;

    ret = lf_mtd_open(LF_MTD_PARTITION_NAME_FW_DEFAULT, &ota_parm->mtd_handle, LF_MTD_OPEN_FLAG_BACKUP);
    if (ret) {
        aos_free(ota_parm);
        ota_parm = NULL;
        printf("Open Default FW partition failed\r\n");
        return -1;
    }

    if (hal_boot2_get_active_entries(BOOT2_PARTITION_TYPE_FW, &ptEntry)) {
        lf_mtd_close(ota_parm->mtd_handle);
        aos_free(ota_parm);
        ota_parm = NULL;
        
        printf("PtTable_Get_Active_Entries fail\r\n");
        return -1;
    }
    ota_addr = ptEntry.Address[!ptEntry.activeIndex];
    ota_size = ptEntry.maxLen[!ptEntry.activeIndex];
    
    if (file_size > ota_size) {
        printf("file size is more than partition size\r\n");
        lf_mtd_close(ota_parm->mtd_handle);
        aos_free(ota_parm);
        ota_parm = NULL;
        return -1;
    }

    printf("Starting OTA test. OTA size is %lu\r\n", ota_size);
    printf("[OTA] [TEST] activeIndex is %u, use OTA address=%08x\r\n", ptEntry.activeIndex, (unsigned int)ota_addr);
    printf("[OTA] [TEST] Erase flash with size %lu...", ota_size);
 
    ret = lf_mtd_erase_all(ota_parm->mtd_handle);
    if (ret) {
        lf_mtd_close(ota_parm->mtd_handle);
        aos_free(ota_parm);
        ota_parm = NULL;
        printf("erase flash error\r\n");
        return -1;
    }
    printf("Done\r\n");
    return ret;
}

int hal_ota_update(uint32_t filesize, uint32_t offset, uint8_t *buf, uint32_t buf_len)
{
    uint32_t file_size, ret;
    if (ota_parm == NULL) {
        printf("please start ota first\r\n");
        return -1;
    }
    
    file_size = ota_parm->file_size;
    if ((filesize > file_size) || (NULL == buf) || ((offset + buf_len) > file_size) || (buf_len == 0)) {
        printf("parm is error!\r\n");
        return -1;
    }

    ret = lf_mtd_write(ota_parm->mtd_handle, offset, buf_len, buf);
    if (ret) {
        printf("mtd write failed\r\n");
        return -1;
    }
    
    return ret;
}

int hal_ota_finish(uint8_t check_hash, uint8_t auto_reset)
{
    if (ota_parm == NULL) {
        printf("please start ota first\r\n");
        return -1;
    }
    uint32_t ota_size; 
   
    if (ota_parm->file_size <= 32) {
        lf_mtd_close(ota_parm->mtd_handle);
        aos_free(ota_parm);
        ota_parm = NULL;
        return -1;
    }

    if (check_hash) {
        uint8_t sha256_result[32] = {0};
        uint32_t read_size;
        iot_sha256_context sha256_ctx;
        int offset = 0;
        uint8_t *r_buf;

        r_buf = aos_malloc(CHECK_IMG_BUF_SIZE);
        if (r_buf == NULL) {
            lf_mtd_close(ota_parm->mtd_handle);
            aos_free(ota_parm);
            ota_parm = NULL;
            printf("malloc error\r\n");
            return -1;
        }

        utils_sha256_init(&sha256_ctx);
        utils_sha256_starts(&sha256_ctx);
    
        memset(sha256_result, 0, 32);
        offset = 0;
        ota_size = ota_parm->file_size - OTA_HEADER_SIZE;
        while (offset < ota_size) {
            (ota_size - offset >= CHECK_IMG_BUF_SIZE) ? (read_size = CHECK_IMG_BUF_SIZE):(read_size = ota_size - offset);
            if (lf_mtd_read(ota_parm->mtd_handle, offset, read_size, r_buf)) {
                printf("mtd read failed\r\n");
                lf_mtd_close(ota_parm->mtd_handle);
                aos_free(ota_parm);
                ota_parm = NULL;
                aos_free(r_buf);
                return -1;
            }
            
            utils_sha256_update(&sha256_ctx, r_buf, read_size);
            offset += read_size;
        }

        utils_sha256_finish(&sha256_ctx, sha256_result);
        aos_free(r_buf);

        printf("\r\n[OTA] Calculated SHA256 Checksum:");
        for (int i = 0; i < sizeof(sha256_result); i++) {
            printf("%02X", sha256_result[i]);
        }
        printf("\r\n[OTA] Header SET SHA256 Checksum:");
        for (int i = 0; i < sizeof(sha256_img); i++) {
            printf("%02X", sha256_img[i]);
        }
    
        if (memcmp(sha256_result, (const void *)sha256_img, sizeof(sha256_img)) != 0) {
            printf("\r\nsha256 check error\r\n");
            lf_mtd_close(ota_parm->mtd_handle);
            aos_free(ota_parm);
            ota_parm = NULL;
            utils_sha256_free(&sha256_ctx);
            return -1;
        }

        utils_sha256_free(&sha256_ctx);
    } 
    else {
        ota_size = ota_parm->file_size;
    }
    ptEntry.len = ota_size;
    printf("[OTA] [TCP] Update PARTITION, partition len is %lu\r\n", ptEntry.len);
    hal_boot2_update_ptable(&ptEntry);
    lf_mtd_close(ota_parm->mtd_handle);
    aos_free(ota_parm);
    ota_parm = NULL;
    
    if (auto_reset) {
        hal_reboot();
    }
    
    return 0;
}






