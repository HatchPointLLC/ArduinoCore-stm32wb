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

#include "stm32wb_boot.h"
#include "stm32wb_fwu.h"
#include "stm32wb_flash.h"
#include "stm32wb_system.h"
#include "stm32wb_ipcc.h"

typedef struct _stm32wb_fwu_control_t {
    volatile uint8_t            state;
    volatile uint8_t            control;
    uint16_t                    mode;
    uint32_t                    address;
    uint32_t                    data[2];
    stm32wb_fwu_request_t       *request;
    stm32wb_flash_request_t     flash;
} stm32wb_fwu_control_t;

static stm32wb_fwu_control_t stm32wb_fwu_control;

static bool stm32wb_fwu_check_application(void);
static bool stm32wb_fwu_check_wireless(void);

/* BOOT erases first block on failure, and last block on success.
 */


static uint32_t stm32wb_fwu_state(void)
{
    const uint32_t *data, *data_e;
    uint32_t address;

    if (stm32wb_fwu_control.state == STM32WB_FWU_STATE_NONE)
    {
        if (((volatile uint32_t*)STM32WB_BOOT_FWU_REQUEST_BASE)[0] != STM32WB_BOOT_FWU_REQUEST_NONE)
        {
            if (((volatile uint32_t*)STM32WB_BOOT_FWU_STATUS_BASE)[0] != STM32WB_BOOT_FWU_STATUS_NO_ERROR)
            {
                stm32wb_fwu_control.state = STM32WB_FWU_STATE_FAILED;
            }
            else
            {
                stm32wb_fwu_control.state = STM32WB_FWU_STATE_UPDATED;
            }
        }
        else
        {
            if (((volatile uint32_t*)STM32WB_BOOT_FWU_CANDIDATE_BASE)[0] == STM32WB_BOOT_FWU_CANDIDATE_IMAGE)
            {
                stm32wb_fwu_control.state = STM32WB_FWU_STATE_CANDIDATE;
            }
            else
            {
                stm32wb_fwu_control.state = STM32WB_FWU_STATE_READY;

                for (address = STM32WB_BOOT_FWU_BASE; address < STM32WB_BOOT_FWU_LIMIT; address += FLASH_PAGE_SIZE)
                {
                    for (data = (const uint32_t*)(address), data_e = (const uint32_t*)(address + FLASH_PAGE_SIZE); data < data_e; data += 2)
                    {
                        if ((uint32_t)data > STM32WB_BOOT_FWU_APPLICATION_BASE)
                        {
                            if ((data[0] & data[1]) != 0xffffffff)
                            {
                                stm32wb_fwu_control.state = STM32WB_FWU_STATE_WRITING;
                            }
                        }
                    }
                }
            }
        }
    }

    return stm32wb_fwu_control.state;
}

static uint32_t stm32wb_fwu_crc32(const uint8_t *data, uint32_t size, uint32_t crc32)
{
    uint8_t c;

    static const uint32_t lut[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };
    
    while (size--)
    {
        c = *data++;
        
        crc32 = (crc32 >> 4) ^ lut[(crc32 ^ c       ) & 15];
        crc32 = (crc32 >> 4) ^ lut[(crc32 ^ (c >> 4)) & 15];
    }
    
    return crc32;
}

static bool stm32wb_fwu_check_application(void)
{
    const stm32wb_image_info_t *image_info;
    uint32_t candidate_magic, candidate_base, candidate_size, candidate_length, image_offset;

    candidate_magic = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->magic;
    candidate_base = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->base;
    candidate_size = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->size;
    candidate_length = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->length & ~STM32WB_IMAGE_OPTION_ENCRYPTED;
    
    if ((candidate_magic != STM32WB_APPLICATION_MAGIC) || (candidate_base != STM32WB_APPLICATION_BASE) || (candidate_size > STM32WB_APPLICATION_SIZE))
    {
        return false;
    }

    if ((candidate_length > STM32WB_BOOT_FWU_APPLICATION_SIZE) || (candidate_length & 7))
    {
        return false;
    }

    image_offset = candidate_length - sizeof(stm32wb_image_info_t);
    image_info = (const stm32wb_image_info_t*)(STM32WB_BOOT_FWU_APPLICATION_BASE + image_offset);

    if (image_info->magic != STM32WB_IMAGE_MAGIC)
    {
        return false;
    }
    
    if (image_info->crc32 != stm32wb_fwu_crc32((const uint8_t*)STM32WB_BOOT_FWU_APPLICATION_BASE, ((uint32_t)&image_info->crc32 - STM32WB_BOOT_FWU_APPLICATION_BASE), 0xffffffff))
    {
        return false;
    }

    return true;
}

static bool stm32wb_fwu_check_wireless(void)
{
    uint32_t magic;
    const stm32wb_ipcc_fus_signature_t *signature;

    signature = (const stm32wb_ipcc_fus_signature_t*)(STM32WB_BOOT_FWU_WIRELESS_BASE + STM32WB_BOOT_FWU_WIRELESS_SIZE - sizeof(stm32wb_ipcc_fus_signature_t));
    
    while (STM32WB_BOOT_FWU_WIRELESS_BASE < (uint32_t)signature)
    {
        if (signature->Magic == STM32WB_IPCC_FUS_MAGIC_CUST_SIGNATURE)
        {
            magic = ((const stm32wb_ipcc_fus_signature_t*)((const uint8_t*)signature - (signature->MemorySize & 0x000000ff) - sizeof(stm32wb_ipcc_fus_signature_t)))->Magic;
            
            if (magic == STM32WB_IPCC_FUS_MAGIC_STM_SIGNATURE)
            {
                signature = (const stm32wb_ipcc_fus_signature_t*)((const uint8_t*)signature - (signature->MemorySize & 0x000000ff) - sizeof(stm32wb_ipcc_fus_signature_t));
            }
        }

        if (signature->Magic == STM32WB_IPCC_FUS_MAGIC_STM_SIGNATURE)
        {
            magic = ((const stm32wb_ipcc_fus_image_t*)((const uint8_t*)signature - (signature->MemorySize & 0x000000ff) - sizeof(stm32wb_ipcc_fus_image_t)))->Magic;

            if ((magic == STM32WB_IPCC_FUS_MAGIC_WIRELESS_IMAGE) || (magic == STM32WB_IPCC_FUS_MAGIC_FUS_IMAGE))
            {
                return true;
            }
        }

        signature = (const stm32wb_ipcc_fus_signature_t*)((const uint8_t*)signature - 4);
    }

    return false;
}

static void stm32wb_fwu_process()
{
    stm32wb_fwu_request_t *request = stm32wb_fwu_control.request;
    stm32wb_fwu_done_callback_t callback;
    void *context;
    const uint32_t *data, *data_e;
    uint32_t address, count;
    bool success;

    success = true;
    
    if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_CLEAN)
    {
        for (address = stm32wb_fwu_control.address; address < STM32WB_BOOT_FWU_LIMIT; address += FLASH_PAGE_SIZE)
        {
            for (data = (const uint32_t*)(address), data_e = (const uint32_t*)(address + FLASH_PAGE_SIZE); data < data_e; data += 2)
            {
                if ((data[0] & data[1]) != 0xffffffff)
                {
                    stm32wb_fwu_control.address = address + FLASH_PAGE_SIZE;
                    
                    stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_ERASE;
                    stm32wb_fwu_control.flash.address = address;
                    stm32wb_fwu_control.flash.count = FLASH_PAGE_SIZE;
                    stm32wb_fwu_control.flash.data = NULL;
                    stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
                    stm32wb_fwu_control.flash.context = NULL;
                    
                    if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
                    {
                        return;
                    }
                    
                    stm32wb_fwu_control.control = 0;
                    
                    success = false;
                }
            }
        }

        if (success)
        {
            stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_CLEAN;
            stm32wb_fwu_control.state = STM32WB_FWU_STATE_READY;
        }
    }
    
    if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_START)
    {
        stm32wb_fwu_control.mode = request->mode;

        stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_START;
        stm32wb_fwu_control.state = STM32WB_FWU_STATE_WRITING;
    }

    if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_WRITE)
    {
        stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
        stm32wb_fwu_control.flash.address = ((stm32wb_fwu_control.mode == STM32WB_FWU_MODE_APPLICATION) ? STM32WB_BOOT_FWU_APPLICATION_BASE : STM32WB_BOOT_FWU_WIRELESS_BASE) + request->offset;
        stm32wb_fwu_control.flash.count = request->size;
        stm32wb_fwu_control.flash.data = request->data;
        stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
        stm32wb_fwu_control.flash.context = NULL;
                        
        if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
        {
            stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_WRITE;

            return;
        }

        stm32wb_fwu_control.control = 0;
                        
        success = false;
    }

    if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_FINISH)
    {
        stm32wb_fwu_control.data[0] = STM32WB_BOOT_FWU_CANDIDATE_IMAGE;
        stm32wb_fwu_control.data[1] = STM32WB_BOOT_FWU_LIMIT;

        count = 8;
        
        stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
        stm32wb_fwu_control.flash.address = STM32WB_BOOT_FWU_CANDIDATE_BASE;
        stm32wb_fwu_control.flash.count = count;
        stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
        stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
        stm32wb_fwu_control.flash.context = NULL;
                        
        if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
        {
            stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_FINISH;
            stm32wb_fwu_control.state = STM32WB_FWU_STATE_CANDIDATE;

            return;
        }

        stm32wb_fwu_control.control = 0;
                        
        success = false;
    }

    if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_CANCEL)
    {
        stm32wb_fwu_control.data[0] = STM32WB_BOOT_FWU_REQUEST_CANCEL;
        stm32wb_fwu_control.data[1] = 0x00000000;

        count = 8;
        
        stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
        stm32wb_fwu_control.flash.address = STM32WB_BOOT_FWU_REQUEST_BASE;
        stm32wb_fwu_control.flash.count = count;
        stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
        stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
        stm32wb_fwu_control.flash.context = NULL;
                        
        if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
        {
            stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_CANCEL;
            stm32wb_fwu_control.state = STM32WB_FWU_STATE_FAILED;

            return;
        }

        stm32wb_fwu_control.control = 0;
                        
        success = false;
    }
    
    if (stm32wb_fwu_control.control & STM32WB_FWU_CONTROL_INSTALL)
    {
        stm32wb_fwu_control.data[0] = STM32WB_BOOT_FWU_REQUEST_CANCEL;
        stm32wb_fwu_control.data[1] = 0x00000000;

        count = 8;

        if (stm32wb_fwu_control.mode == STM32WB_FWU_MODE_APPLICATION)
        {
            if (stm32wb_fwu_check_application())
            {
                stm32wb_fwu_control.data[0] = STM32WB_BOOT_FWU_REQUEST_INSTALL_APPLICATION;
                stm32wb_fwu_control.data[1] = STM32WB_BOOT_FWU_APPLICATION_BASE;
            }
        }
        else
        {
            if (stm32wb_fwu_check_wireless())
            {
                stm32wb_fwu_control.data[0] = STM32WB_BOOT_FWU_REQUEST_INSTALL_WIRELESS;
                stm32wb_fwu_control.data[1] = STM32WB_BOOT_FWU_WIRELESS_BASE;
            }
        }
        
        stm32wb_fwu_control.flash.control = STM32WB_FLASH_CONTROL_PROGRAM;
        stm32wb_fwu_control.flash.address = STM32WB_BOOT_FWU_REQUEST_BASE;
        stm32wb_fwu_control.flash.count = count;
        stm32wb_fwu_control.flash.data = (const uint8_t*)&stm32wb_fwu_control.data[0];
        stm32wb_fwu_control.flash.callback = (stm32wb_flash_done_callback_t)stm32wb_fwu_process;
        stm32wb_fwu_control.flash.context = NULL;
                        
        if (stm32wb_flash_request(&stm32wb_fwu_control.flash))
        {
            stm32wb_fwu_control.control &= ~STM32WB_FWU_CONTROL_INSTALL;
            stm32wb_fwu_control.state = (stm32wb_fwu_control.data[0] == STM32WB_BOOT_FWU_REQUEST_CANCEL) ? STM32WB_FWU_STATE_FAILED : STM32WB_FWU_STATE_STAGED;

            return;
        }
        
        stm32wb_fwu_control.control = 0;
                        
        success = false;
    }
    
    if (!stm32wb_fwu_control.control)
    {
        callback = request->callback;
        context = request->context;

        stm32wb_fwu_control.request = NULL;
        
        request->status = success ? STM32WB_FWU_STATUS_SUCCESS : STM32WB_FWU_STATUS_FAILURE;

        if (callback)
        {
            (*callback)(context);
        }
    }
}


static bool __svc_stm32wb_fwu_query(stm32wb_fwu_info_t *info)
{
    const stm32wb_application_info_t *application_info;
    
    if (!info)
    {
        return false;
    }

    application_info = (const stm32wb_application_info_t*)STM32WB_APPLICATION_INFO_BASE;
    
    info->state = stm32wb_fwu_state();
    info->version.major = application_info->version.major;
    info->version.minor = application_info->version.minor;
    info->version.revision = application_info->version.revision;

    return true;
}

static bool __svc_stm32wb_fwu_request(stm32wb_fwu_request_t *request)
{
    uint8_t state, mode;
        
    if (stm32wb_fwu_control.request)
    {
        return false;
    }

    state = stm32wb_fwu_state();
    mode = stm32wb_fwu_control.mode;
    
    if (request->control & STM32WB_FWU_CONTROL_CLEAN)
    {
        state = STM32WB_FWU_STATE_READY;

        stm32wb_fwu_control.address = STM32WB_BOOT_FWU_BASE;
    }

    if (request->control & STM32WB_FWU_CONTROL_START)
    {
        if (state != STM32WB_FWU_STATE_READY)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_WRITING;
        mode = request->mode;
    }

    if (request->control & STM32WB_FWU_CONTROL_WRITE)
    {
        if (state != STM32WB_FWU_STATE_WRITING)
        {
            return false;
        }

        if ((request->offset & 7) || (request->size & 7))
        {
            return false;
        }

        if (mode == STM32WB_FWU_MODE_APPLICATION)
        {
            if ((request->offset + request->size) > STM32WB_BOOT_FWU_APPLICATION_SIZE)
            {
                return false;
            }
        }
        else
        {
            if ((request->offset + request->size) > STM32WB_BOOT_FWU_WIRELESS_SIZE)
            {
                return false;
            }
        }
    }

    if (request->control & STM32WB_FWU_CONTROL_FINISH)
    {
        if (state != STM32WB_FWU_STATE_WRITING)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_CANDIDATE;
    }

    if (request->control & STM32WB_FWU_CONTROL_INSTALL)
    {
        if (state != STM32WB_FWU_STATE_CANDIDATE)
        {
            return false;
        }

        state = STM32WB_FWU_STATE_STAGED;
    }
    
    if (request->control & STM32WB_FWU_CONTROL_CANCEL)
    {
        if ((state != STM32WB_FWU_STATE_WRITING) && (state != STM32WB_FWU_STATE_CANDIDATE))
        {
            return false;
        }
    }
    
    stm32wb_fwu_control.request = request;
    stm32wb_fwu_control.control = request->control;

    request->status = STM32WB_FWU_STATUS_BUSY;

    stm32wb_fwu_process();

    return true;
}

bool stm32wb_fwu_query(stm32wb_fwu_info_t *info)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_fwu_query, (uint32_t)info);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_fwu_query(info);
    }

    return false;
}

bool stm32wb_fwu_request(stm32wb_fwu_request_t *request)
{
    if (armv7m_core_is_in_thread())
    {
        return armv7m_svcall_1((uint32_t)&__svc_stm32wb_fwu_request, (uint32_t)request);
    }

    if (armv7m_core_is_in_svcall_or_pendsv())
    {
        return __svc_stm32wb_fwu_request(request);
    }

    return false;
}
