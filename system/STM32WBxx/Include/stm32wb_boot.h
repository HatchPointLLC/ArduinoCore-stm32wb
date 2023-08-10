/*
 * Copyright (c) 2022-2023 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#if !defined(_STM32WB_BOOT_H)
#define _STM32WB_BOOT_H

#include "armv7m.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const uint32_t stm32wb_boot_vectors[];

typedef struct _stm32wb_boot_vectors_t {
    uint32_t                      reserved_0[8];
    uint32_t                      magic;             // IDCODE[0], IDCODE[1], 'B', 'L'
    uint32_t                      base;              // 0x08000000
    uint32_t                      size;              // 16384
    uint32_t                      reserved_1[2];
    uint32_t                      length;            // 0
    uint32_t                      reserved_2[2];
} stm32wb_boot_vectors_t;
  
typedef struct _stm32wb_boot_rsa2048_key_t {
    uint32_t                      n0inv;    /* -1 / n[0] mod 2^32 */
    uint32_t                      n[64];    /* modulus as little endian array */
    uint32_t                      rr[64];   /* R^2 as little endian array */
    uint32_t                      exponent; /* 65537 */
} stm32wb_boot_rsa2048_key_t;
  
/*

 * Some random default UUID: 8f7e3f71-3def-4302-a8c7-b76bacb21f12
 */

#define STM32WB_BOOT_UUID_SIZE                    16
#define STM32WB_BOOT_DEFAULT_UUID                 { 0x8f, 0x7e, 0x3f, 0x71, 0x3d, 0xef, 0x43, 0x02, 0xa8, 0xc7, 0xb7, 0x6b, 0xac, 0xb2, 0x1f, 0x12 }

#define STM32WB_BOOT_MAGIC                        0x49420495
#define STM32WB_BOOT_BASE                         0x08000000
#define STM32WB_BOOT_LIMIT                        0x08004000
#define STM32WB_BOOT_SIZE                         0x00004000
#define STM32WB_BOOT_KEY_BASE                     0x08003fc0
#define STM32WB_BOOT_KEY_SIZE                     64
#define STM32WB_BOOT_INFO_BASE                    0x08000140
#define STM32WB_BOOT_INFO_LIMIT                   0x08000360
#define STM32WB_BOOT_INFO_SIZE                    544

#define STM32WB_BOOT_OPTION_RDP_LEVEL_MASK        0xff
#define STM32WB_BOOT_OPTION_RDP_LEVEL_0           0xaa
#define STM32WB_BOOT_OPTION_RDP_LEVEL_1           0x00
#define STM32WB_BOOT_OPTION_RDP_LEVEL_2           0xcc
  
typedef struct _stm32wb_boot_version_t {
    uint16_t                      revision;
    uint8_t                       minor;
    uint8_t                       major;
} stm32wb_boot_version_t;

typedef struct _stm32wb_boot_uuid_t {
    uint8_t                       uuid[16];
} stm32wb_boot_uuid_t;
  
typedef struct _stm32wb_boot_info_t {
    stm32wb_boot_uuid_t           uuid;
    stm32wb_boot_version_t        version;
    uint32_t                      options;
    stm32wb_boot_rsa2048_key_t    rsa2048_key;
} stm32wb_boot_info_t;
  
extern stm32wb_boot_info_t stm32wb_boot_info;

typedef struct _stm32wb_application_vectors_t {
    uint32_t                      reserved_0[8];
    uint32_t                      magic;             // IDCODE[0], IDCODE[1], 'A', 'P'
    uint32_t                      base;              // 0x08004000
    uint32_t                      size;              // application size 
    uint32_t                      reserved_1[2];
    uint32_t                      length;            // image length
    uint32_t                      reserved_2[2];
} stm32wb_application_vectors_t;

#define STM32WB_APPLICATION_MAGIC                 0x50410495
#define STM32WB_APPLICATION_BASE                  0x08004000
#define STM32WB_APPLICATION_LIMIT                 0x08064000
#define STM32WB_APPLICATION_SIZE                  0x0005fff0
#define STM32WB_APPLICATION_INFO_BASE             0x08004140
#define STM32WB_APPLICATION_INFO_LIMIT            0x08004280
#define STM32WB_APPLICATION_INFO_SIZE             320

typedef struct _stm32wb_application_version_t {
    uint16_t                      revision;
    uint8_t                       minor;
    uint8_t                       major;
} stm32wb_application_version_t;

typedef struct _stm32wb_application_uuid_t {
    uint8_t                       uuid[16];
} stm32wb_application_uuid_t;
  
typedef struct _stm32wb_application_info_t {
    stm32wb_application_uuid_t    uuid;
    stm32wb_application_version_t version;
    uint32_t                      sequence;
    uint8_t                       rfu[28];
    uint32_t                      epoch;
    uint32_t                      nonce[2];
    uint32_t                      signature[64];
} stm32wb_application_info_t;

#define STM32WB_IMAGE_LENGTH_MASK                0x00ffffff
#define STM32WB_IMAGE_OPTION_MASK                0xff000000
#define STM32WB_IMAGE_OPTION_ENCRYPTED           0x80000000
#define STM32WB_IMAGE_MAGIC                      0x41544F24 // $OTA
  
typedef struct _stm32wb_image_info_t {
    uint32_t                      crc32;
    uint32_t                      magic;
} stm32wb_image_info_t;

#define STM32WB_BOOT_FWU_BASE                         0x08064000
#define STM32WB_BOOT_FWU_LIMIT                        0x080c4000
#define STM32WB_BOOT_FWU_SIZE                         (STM32WB_BOOT_FWU_LIMIT - STM32WB_BOOT_FWU_BASE)

#define STM32WB_BOOT_FWU_CANDIDATE_BASE               0x08064000
#define STM32WB_BOOT_FWU_STATUS_BASE                  0x08064008
#define STM32WB_BOOT_FWU_REQUEST_BASE                 0x08064010
#define STM32WB_BOOT_FWU_STATE_BASE                   0x08064018

#define STM32WB_BOOT_FWU_APPLICATION_BASE             0x08064020
#define STM32WB_BOOT_FWU_APPLICATION_LIMIT            STM32WB_BOOT_FWU_LIMIT
#define STM32WB_BOOT_FWU_APPLICATION_SIZE             (STM32WB_BOOT_FWU_APPLICATION_LIMIT - STM32WB_BOOT_FWU_APPLICATION_BASE)

#define STM32WB_BOOT_FWU_WIRELESS_BASE                0x08064020
#define STM32WB_BOOT_FWU_WIRELESS_LIMIT               STM32WB_BOOT_FWU_LIMIT
#define STM32WB_BOOT_FWU_WIRELESS_SIZE                (STM32WB_BOOT_FWU_WIRELESS_LIMIT - STM32WB_BOOT_FWU_WIRELESS_BASE)
  
#define STM32WB_BOOT_FWU_CANDIDATE_NONE               0xffffffff
#define STM32WB_BOOT_FWU_CANDIDATE_IMAGE              0x00000000

#define STM32WB_BOOT_FWU_STATUS_NO_ERROR              0x00000000
#define STM32WB_BOOT_FWU_STATUS_ERR_TARGET            0x00000001
#define STM32WB_BOOT_FWU_STATUS_ERR_FILE              0x00000002
#define STM32WB_BOOT_FWU_STATUS_ERR_WRITE             0x00000003
#define STM32WB_BOOT_FWU_STATUS_ERR_ERASE             0x00000004
#define STM32WB_BOOT_FWU_STATUS_ERR_CHECK_ERASED      0x00000005
#define STM32WB_BOOT_FWU_STATUS_ERR_PROG              0x00000006
#define STM32WB_BOOT_FWU_STATUS_ERR_VERIFY            0x00000007
#define STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL          0x0000000e
#define STM32WB_BOOT_FWU_STATUS_NONE                  0xffffffff
  
#define STM32WB_BOOT_FWU_REQUEST_NONE                 0xffffffff
#define STM32WB_BOOT_FWU_REQUEST_INSTALL_APPLICATION  0xaaaaaaaa
#define STM32WB_BOOT_FWU_REQUEST_INSTALL_WIRELESS     0x55555555
#define STM32WB_BOOT_FWU_REQUEST_CANCEL               0x00000000

#define STM32WB_BOOT_FWU_STATE_NONE                   0xffffffff
#define STM32WB_BOOT_FWU_STATE_UPGRADE                0x00000000
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32WB_BOOT_H */
