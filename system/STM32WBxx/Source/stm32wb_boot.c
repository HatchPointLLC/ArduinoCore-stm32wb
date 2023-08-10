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

#define __STM32WB_BOOT_CODE__

#include "stm32wb_boot.h"
#include "stm32wb_rtc.h"
#include "stm32wb_system.h"

/************************************************************************************************************************************/

void stm32wb_boot_entry(void);

static void stm32wb_boot_nmi(void);
static void stm32wb_boot_hardfault(void);
static void stm32wb_boot_svcall(void);
static void stm32wb_boot_pendsv(void);
static void __attribute__((noreturn)) stm32wb_boot_reset(void);
static void __attribute__((noreturn)) stm32wb_boot_continue(uint32_t vtor_address);

static void stm32wb_boot_usbd_dcd_event(void);
static void stm32wb_boot_usbd_dcd_usb_interrupt(void);
static void stm32wb_boot_usbd_dcd_crs_interrupt(void);

/************************************************************************************************************************************/

static void stm32wb_boot_memset(void *d, uint8_t c, size_t n);
static void stm32wb_boot_memcpy(void *d, const void *s, size_t n);
static int stm32wb_boot_memcmp(const void *a, const void *b, size_t n);

static uint32_t stm32wb_boot_crc32(const uint8_t *s, size_t n, uint32_t crc32);

/************************************************************************************************************************************/

#define STM32WB_FLASH_PAGE_SIZE 4096

static bool stm32wb_boot_flash_erase(uint32_t address);
static bool stm32wb_boot_flash_program(uint32_t address, const uint8_t *data, uint32_t count);

static uint32_t stm32wb_boot_flash_check(uint32_t base, uint32_t limit);
static bool stm32wb_boot_flash_clean(uint32_t base, uint32_t limit);

/************************************************************************************************************************************/

#define STM32WB_BOOT_AES128_BLOCK_SIZE 16

typedef struct _stm32wb_boot_aes128_context_t {
    uint32_t                   rk[176 / 4];
} stm32wb_boot_aes128_context_t;

/*static */ void stm32wb_boot_aes128_get_key(uint32_t *key);
static void stm32wb_boot_aes128_set_key(stm32wb_boot_aes128_context_t *ctx, const uint32_t *key);
static void stm32wb_boot_aes128_encrypt(stm32wb_boot_aes128_context_t *ctx, const uint32_t *in, uint32_t *out);

/************************************************************************************************************************************/

#define STM32WB_BOOT_SHA256_BLOCK_SIZE  64
#define STM32WB_BOOT_SHA256_HASH_SIZE   32    
  
typedef struct _stm32wb_boot_sha256_context_t {
    uint32_t                   hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t                   length;
    uint32_t                   index;
    uint8_t                    data[STM32WB_BOOT_SHA256_BLOCK_SIZE];
} stm32wb_boot_sha256_context_t;

static void stm32wb_boot_sha256_init(stm32wb_boot_sha256_context_t *sha256_ctx);
static void stm32wb_boot_sha256_update(stm32wb_boot_sha256_context_t *sha256_ctx, const uint8_t *data, size_t size);
static void stm32wb_boot_sha256_final(stm32wb_boot_sha256_context_t *sha256_ctx, uint32_t *hash);

/************************************************************************************************************************************/

static bool stm32wb_boot_rsa2048_verify(const stm32wb_boot_rsa2048_key_t *key, const uint32_t *signature, const uint32_t *digest);

/************************************************************************************************************************************/

static uint32_t stm32wb_boot_fwu(void);
static void __attribute__((noreturn)) stm32wb_boot_dfu(bool firmware);

/************************************************************************************************************************************/

extern uint32_t __boot_data_start__[];
extern uint32_t __boot_data_end__[];
extern uint32_t __boot_data_flash__[];
extern uint32_t __boot_bss_start__[];
extern uint32_t __boot_bss_end__[];
extern uint32_t __boot_stack_top__[];

const uint32_t __attribute__((section(".boot.vectors"), used)) stm32wb_boot_vectors[16+64] =
{
    (uint32_t)&__boot_stack_top__,                               /* Top of Stack */
    (uint32_t)stm32wb_boot_entry,                                /* Reset Handler */
    (uint32_t)stm32wb_boot_nmi,                                  /* NMI Handler */
    (uint32_t)stm32wb_boot_hardfault,                            /* Hard Fault Handler */
    0,                                                           /* MPU Fault Handler */
    0,                                                           /* Bus Fault Handler */
    0,                                                           /* Usage Fault Handler */
    0,                                                           /* Reserved */
    STM32WB_BOOT_MAGIC,                                          /* Reserved (magic) */
    STM32WB_BOOT_BASE,                                           /* Reserved (base) */
    STM32WB_BOOT_SIZE,                                           /* Reserved (size) */
    (uint32_t)stm32wb_boot_svcall,                               /* SVCall Handler */
    0,                                                           /* Debug Monitor Handler */
    0,                                                           /* Reserved (length) */
    (uint32_t)stm32wb_boot_pendsv,                               /* PendSV Handler */
    0,                                                           /* SysTick Handler */

    /* External interrupts */
    0,                                                           /* WWDG_IRQHandler */
    0,                                                           /* PVD_PVM_IRQHandler */
    0,                                                           /* TAMP_STAMP_LSECSS_IRQHandler */
    0,                                                           /* RTC_WKUP_IRQHandler */
    0,                                                           /* FLASH_IRQHandler */
    0,                                                           /* RCC_IRQHandler */
    0,                                                           /* EXTI0_IRQHandler */
    0,                                                           /* EXTI1_IRQHandler */
    0,                                                           /* EXTI2_IRQHandler */
    0,                                                           /* EXTI3_IRQHandler */
    0,                                                           /* EXTI4_IRQHandler */
    0,                                                           /* DMA1_Channel1_IRQHandler */
    0,                                                           /* DMA1_Channel2_IRQHandler */
    0,                                                           /* DMA1_Channel3_IRQHandler */
    0,                                                           /* DMA1_Channel4_IRQHandler */
    0,                                                           /* DMA1_Channel5_IRQHandler */
    0,                                                           /* DMA1_Channel6_IRQHandler */
    0,                                                           /* DMA1_Channel7_IRQHandler */
    0,                                                           /* ADC1_IRQHandler */
    0,                                                           /* USB_HP_IRQHandler */
    (uint32_t)stm32wb_boot_usbd_dcd_usb_interrupt,               /* USB_LP_IRQHandler */
    0,                                                           /* C2SEV_PWR_C2H_IRQHandler */
    0,                                                           /* COMP_IRQHandler */
    0,                                                           /* EXTI9_5_IRQHandler */
    0,                                                           /* TIM1_BRK_IRQHandler */
    0,                                                           /* TIM1_UP_TIM16_IRQHandler */
    0,                                                           /* TIM1_TRG_COM_TIM17_IRQHandler */
    0,                                                           /* TIM1_CC_IRQHandler */
    0,                                                           /* TIM2_IRQHandler */
    0,                                                           /* PKA_IRQHandler */
    0,                                                           /* I2C1_EV_IRQHandler */
    0,                                                           /* I2C1_ER_IRQHandler */
    0,                                                           /* I2C3_EV_IRQHandler */
    0,                                                           /* I2C3_ER_IRQHandler */
    0,                                                           /* SPI1_IRQHandler */
    0,                                                           /* SPI2_IRQHandler */
    0,                                                           /* USART1_IRQHandler */
    0,                                                           /* LPUART1_IRQHandler */
    0,                                                           /* SAI1_IRQHandler */
    0,                                                           /* TSC_IRQHandler */
    0,                                                           /* EXTI15_10_IRQHandler */
    0,                                                           /* RTC_Alarm_IRQHandler */
    (uint32_t)stm32wb_boot_usbd_dcd_crs_interrupt,               /* CRS_IRQHandler */
    0,                                                           /* PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler */
    0,                                                           /* IPCC_C1_RX_IRQHandler */
    0,                                                           /* IPCC_C1_TX_IRQHandler */
    0,                                                           /* HSEM_IRQHandler */
    0,                                                           /* LPTIM1_IRQHandler */
    0,                                                           /* LPTIM2_IRQHandler */
    0,                                                           /* LCD_IRQHandler */
    0,                                                           /* QUADSPI_IRQHandler */
    0,                                                           /* AES1_IRQHandler */
    0,                                                           /* AES2_IRQHandler */
    0,                                                           /* RNG_IRQHandler */
    0,                                                           /* FPU_IRQHandler */
    0,                                                           /* DMA2_Channel1_IRQHandler */
    0,                                                           /* DMA2_Channel2_IRQHandler */
    0,                                                           /* DMA2_Channel3_IRQHandler */
    0,                                                           /* DMA2_Channel4_IRQHandler */
    0,                                                           /* DMA2_Channel5_IRQHandler */
    0,                                                           /* DMA2_Channel6_IRQHandler */
    0,                                                           /* DMA2_Channel7_IRQHandler */
    0,                                                           /* DMAMUX1_OVR_IRQHandler */
};

stm32wb_boot_info_t __attribute__((section(".boot.info"), used)) stm32wb_boot_info = {
    .uuid              = { .uuid = STM32WB_BOOT_DEFAULT_UUID },
    .version           = { .major = 0, .minor = 0, .revision = 0 },
    .options           = STM32WB_BOOT_OPTION_RDP_LEVEL_0,
};

uint32_t __attribute__((section(".stack"), used)) stm32wb_boot_stack[4096 / 4];

/************************************************************************************************************************************/

static void __attribute__((noinline)) stm32wb_boot_nmi(void)
{
    while (1)
    {
        __WFE();
    }
}

static void __attribute__((noinline)) stm32wb_boot_hardfault(void)
{
    while (1)
    {
        __WFE();
    }
}

static void __attribute__((naked, noinline)) stm32wb_boot_svcall(void)
{
    __asm__(
        "   mov     r2, sp                                  \n"
        "   push    { r2, lr }                              \n"
        "   ldmia   r2, { r0, r1, r2, r3 }                  \n"
        "   blx     r7                                      \n"
        "   pop     { r2, r3 }                              \n"
        "   str     r0, [r2, #0]                            \n"
        "   bx      r3                                      \n"
        :
        :
        );
}

static void __attribute__((noinline)) stm32wb_boot_pendsv(void)
{
    stm32wb_boot_usbd_dcd_event();
}

static void __attribute__((noreturn, noinline)) stm32wb_boot_reset(void)
{
    register uint32_t aircr_address __asm__("r0");
    register uint32_t aircr_data __asm__("r1");
    register uint32_t zero_base __asm__("r2");
    register uint32_t zero_limit __asm__("r3");

    // __disable_irq();
    
    aircr_address = (uint32_t)&SCB->AIRCR;
    aircr_data = 0x05fa0004;
    
    zero_base = (uint32_t)__boot_data_start__;
    zero_limit = (uint32_t)__boot_stack_top__;

    __DSB();
    
    __asm__ volatile (
        "   movs    r4, #0                                  \n"
        "   movs    r5, #0                                  \n"
        "1: strd    r4, r5, [%2], #4                        \n"
        "   cmp     %2, %3                                  \n"
        "   bne     1b                                      \n"
        "   mov     r2, r4                                  \n"
        "   mov     r3, r5                                  \n"
        "   mov     r6, r2                                  \n"
        "   mov     r7, r2                                  \n"
        "   mov     r8, r2                                  \n"
        "   mov     r9, r2                                  \n"
        "   mov     r10, r2                                 \n"
        "   mov     r11, r2                                 \n"
        "   mov     r12, r2                                 \n"
        "   str     %1, [%0]                                \n"
        "   b       .                                       \n"
        :
        : "l" (aircr_address), "l" (aircr_data), "l" (zero_base), "l" (zero_limit)
        );
}

static void __attribute__((noreturn, noinline, section(".rodata"))) stm32wb_boot_continue(uint32_t vtor_address)
{
    register uint32_t stack_address __asm__("r0");
    register uint32_t entry_address __asm__("r1");
    register uint32_t zero_base __asm__("r2");
    register uint32_t zero_limit __asm__("r3");

    // __disable_irq();

    if (vtor_address)
    {
        MPU->RBAR = 0x08000000 | MPU_RBAR_VALID_Msk | (0 << MPU_RBAR_REGION_Pos);
        MPU->RASR = MPU_RASR_XN_Msk | (1 << MPU_RASR_SRD_Pos) | (13 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;
        MPU->CTRL = (MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk | MPU_CTRL_ENABLE_Msk);
    
        __DSB();
        __ISB();
    }
    
    stack_address = ((volatile uint32_t * volatile )vtor_address)[0];
    entry_address = ((volatile uint32_t * volatile )vtor_address)[1];

    zero_base = (uint32_t)__boot_data_start__;
    zero_limit = (uint32_t)__boot_stack_top__;
    
    __asm__ volatile (
        "   mov     sp, %0                                  \n"
        "   mov     lr, %1                                  \n"
        "   movs    %0, #0                                  \n"
        "   movs    %1, #0                                  \n"
        "1: strd    %0, %1, [%2], #4                        \n"
        "   cmp     %2, %3                                  \n"
        "   bne     1b                                      \n"
        "   mov     r2, r0                                  \n"
        "   mov     r3, r1                                  \n"
        "   mov     r4, r2                                  \n"
        "   mov     r5, r2                                  \n"
        "   mov     r6, r2                                  \n"
        "   mov     r7, r2                                  \n"
        "   mov     r8, r2                                  \n"
        "   mov     r9, r2                                  \n"
        "   mov     r10, r2                                 \n"
        "   mov     r11, r2                                 \n"
        "   mov     r12, r2                                 \n"
        "   bx      lr                                      \n"
        :
        : "l" (stack_address), "l" (entry_address), "l" (zero_base), "l" (zero_limit)
        );
}

/************************************************************************************************************************************/

static void __attribute__((naked, noinline)) stm32wb_boot_memset(void *d, uint8_t c, size_t n)
{
    __asm__(
        "   cmp     r2, #16                                 \n"
        "   bls     2f                                      \n"
        "   lsls    r3, r0, #30                             \n"
        "   bne     2f                                      \n"
        "   uxtb    r1, r1                                  \n"
	"   orr     r1, r1, r1, lsl #8                      \n"
	"   orr     r1, r1, r1, lsl #16                     \n"
        "   bic     r3, r2, #3                              \n"
        "   add     r3, r0                                  \n"
        "1: str     r1, [r0], #4                            \n"
        "   cmp     r3, r0                                  \n"
        "   bne     1b                                      \n"
        "   lsrs    r3, r2, #2                              \n"
        "   it      cs                                      \n"
        "   strhcs  r1, [r0], #2                            \n"
        "   lsrs    r3, r2, #1                              \n"
        "   it      cs                                      \n"
        "   strbcs  r1, [r0], #1                            \n"
        "   bx      lr                                      \n"
        "2: cbz     r2, 4f                                  \n"
        "   add     r2, r0                                  \n"
        "3: strb    r1, [r0], #1                            \n"
        "   cmp     r2, r0                                  \n"
        "   bne     3b                                      \n"
        "4: bx      lr                                      \n"
        );
}

static void __attribute__((naked, noinline)) stm32wb_boot_memcpy(void *d, const void *s, size_t n)
{
    __asm__(
        "   cmp     r2, #16                                 \n"
        "   bls     2f                                      \n"
        "   lsls    r3, r0, #30                             \n"
        "   bne     2f                                      \n"
        "   bic     r3, r2, #3                              \n"
        "   add     r3, r0                                  \n"
        "1: ldr     r12, [r1], #4                           \n"
        "   str     r12, [r0], #4                           \n"
        "   cmp     r3, r0                                  \n"
        "   bne     1b                                      \n"
        "   lsrs    r3, r2, #2                              \n"
        "   itt     cs                                      \n"
        "   ldrhcs  r12, [r1], #2                           \n"
        "   strhcs  r12, [r0], #2                           \n"
        "   lsrs    r3, r2, #1                              \n"
        "   itt     cs                                      \n"
        "   ldrbcs  r12, [r1], #1                           \n"
        "   strbcs  r12, [r0], #1                           \n"
        "   bx      lr                                      \n"
        "2: cbz     r2, 4f                                  \n"
        "   add     r2, r0                                  \n"
        "3: ldrb    r12, [r1], #1                           \n"
        "   strb    r12, [r0], #1                           \n"
        "   cmp     r2, r0                                  \n"
        "   bne     3b                                      \n"
        "4: bx      lr                                      \n"
        );
}

static int __attribute__((noinline)) stm32wb_boot_memcmp(const void *a, const void *b, size_t n)
{
    size_t i;
    
    if (n)
    {
        for (i = 0; i < n; i++)
        {
            if (((const uint8_t*)a)[i] != ((const uint8_t*)b)[i])
            {
                return (((const uint8_t*)a)[i] - ((const uint8_t*)b)[i]);
            }
        }
    }

    return 0;
}
/************************************************************************************************************************************/

#define armv7m_atomic_and              stm32wb_boot_atomic_and
#define armv7m_atomic_andh             stm32wb_boot_atomic_andh
#define armv7m_atomic_or               stm32wb_boot_atomic_or
#define armv7m_atomic_orh              stm32wb_boot_atomic_orh 
#define armv7m_atomic_swap             stm32wb_boot_atomic_swap
#define armv7m_atomic_swapb            stm32wb_boot_atomic_swapb
#define armv7m_atomic_cas              stm32wb_boot_atomic_cas

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_and(volatile uint32_t *p_data, uint32_t data)
{
    return __armv7m_atomic_and(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_andh(volatile uint16_t *p_data, uint32_t data)
{
    return __armv7m_atomic_andh(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_or(volatile uint32_t *p_data, uint32_t data)
{
    return __armv7m_atomic_or(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_orh(volatile uint16_t *p_data, uint32_t data)
{
    return __armv7m_atomic_orh(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_swap(volatile uint32_t *p_data, uint32_t data)
{
    return __armv7m_atomic_swap(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_swapb(volatile uint8_t *p_data, uint32_t data)
{
    return __armv7m_atomic_swapb(p_data, data);
}

static uint32_t __attribute__((noinline)) stm32wb_boot_atomic_cas(volatile uint32_t *p_data, uint32_t data_expected, uint32_t data)
{
    return __armv7m_atomic_cas(p_data, data_expected, data);
}

/************************************************************************************************************************************/

#define armv7m_core_udelay             stm32wb_boot_udelay

static void __attribute__((noinline)) stm32wb_boot_udelay(uint32_t delay)
{
    uint32_t n;

    n = (delay * (64000000 / 15625) + 255) / 256;

    __asm__ __volatile__(
                         "1: subs %0, #1 \n"
                         "   nop         \n"
                         "   bne  1b     \n"
                         : "+r" (n));
}

/************************************************************************************************************************************/

static uint32_t __attribute__((noinline)) stm32wb_boot_crc32(const uint8_t *d, size_t n, uint32_t crc32)
{
    uint8_t c;

    static const uint32_t lut[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };
    
    while (n--)
    {
        c = *d++;
        
        crc32 = (crc32 >> 4) ^ lut[(crc32 ^ c       ) & 15];
        crc32 = (crc32 >> 4) ^ lut[(crc32 ^ (c >> 4)) & 15];
    }
    
    return crc32;
}

/************************************************************************************************************************************/

static bool __attribute__((noinline)) stm32wb_boot_flash_erase(uint32_t address)
{
    uint32_t flash_sr, flash_acr;

    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xcdef89ab;
    
    FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
    FLASH->CR = FLASH_CR_STRT | FLASH_CR_PER | (((address - FLASH_BASE) / STM32WB_FLASH_PAGE_SIZE) << 3);

    do
    {
        flash_sr = FLASH->SR;
    }
    while (flash_sr & FLASH_SR_BSY);

    FLASH->CR = FLASH_CR_LOCK;
    
    if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
    {
        return false;
    }

    flash_acr = FLASH->ACR;

    FLASH->ACR = flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    FLASH->ACR = flash_acr;
    
    return true;
}

static bool stm32wb_boot_flash_program(uint32_t address, const uint8_t *data, uint32_t count)
{
    uint32_t flash_sr, flash_acr, data_0, data_1;
    bool success = true;

    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xcdef89ab;
    
    FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
    FLASH->CR = FLASH_CR_PG;

    for (; count; address += 8, data += 8, count -= 8)
    {
        data_0 = ((const uint32_t*)data)[0];
        data_1 = ((const uint32_t*)data)[1];

        __asm__ volatile("": : : "memory");
        
        ((volatile uint32_t*)address)[0] = data_0;
        ((volatile uint32_t*)address)[1] = data_1;
        
        do
        {
            flash_sr = FLASH->SR;
        }
        while (flash_sr & FLASH_SR_BSY);

        if (flash_sr & (FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR))
        {
            success = false;

            break;
        }
    }
    
    FLASH->CR = FLASH_CR_LOCK;
    
    flash_acr = FLASH->ACR;

    FLASH->ACR = flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    FLASH->ACR = flash_acr;
    
    return success;
}

static uint32_t stm32wb_boot_flash_check(uint32_t base, uint32_t limit)
{
    const uint32_t *data, *data_e;
    uint32_t address, count;

    for (count = 0, address = base; address < limit; address += STM32WB_FLASH_PAGE_SIZE)
    {
        for (data = (const uint32_t*)(address), data_e = (const uint32_t*)(address + STM32WB_FLASH_PAGE_SIZE); data < data_e; data += 2)
        {
            if ((data[0] & data[1]) != 0xffffffff)
            {
                count++;

                break;
            }
        }
    }

    return count;
}

static bool stm32wb_boot_flash_clean(uint32_t base, uint32_t limit)
{
    const uint32_t *data, *data_e;
    uint32_t address;
    
    for (address = base; address < limit; address += STM32WB_FLASH_PAGE_SIZE)
    {
        for (data = (const uint32_t*)(address), data_e = (const uint32_t*)(address + STM32WB_FLASH_PAGE_SIZE); data < data_e; data += 2)
        {
            if ((data[0] & data[1]) != 0xffffffff)
            {
                if (!stm32wb_boot_flash_erase(address))
                {
                    return false;
                }
                
                break;
            }
        }
    }

    return true;
}

/************************************************************************************************************************************/

static const uint8_t __attribute__((used, aligned(4))) stm32wb_boot_aes128_sbox[256] = {
    0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
    0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0, 0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0,
    0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
    0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
    0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0, 0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84,
    0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
    0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
    0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5, 0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
    0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
    0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB,
    0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C, 0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
    0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
    0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A,
    0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E, 0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E,
    0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
    0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16
};

/* static */ void __attribute__((naked, noinline, section(".boot.aes128_get_key"), used)) stm32wb_boot_aes128_get_key(uint32_t *key)
{
    __asm__(
        "   nop                                             \n"
        "   push    {r1-r4}                                 \n"
        "   movw    r1, #:lower16:0x33221100                \n"
        "   movt    r1, #:upper16:0x33221100                \n"
        "   movw    r2, #:lower16:0x77665544                \n"
        "   movt    r2, #:upper16:0x77665544                \n"
        "   movw    r3, #:lower16:0xbbaa9988                \n"
        "   movt    r3, #:upper16:0xbbaa9988                \n"
        "   movw    r4, #:lower16:0xffeeddcc                \n"
        "   movt    r4, #:upper16:0xffeeddcc                \n"
        "   stm     r0, {r1-r4}                             \n"
        "   pop     {r1-r4}                                 \n"
        "   bx      lr                                      \n"
        "   nop                                             \n"
        "   nop                                             \n"
        "   bkpt    #0                                      \n"
            );
}

static void __attribute__((naked, noinline)) stm32wb_boot_aes128_set_key(stm32wb_boot_aes128_context_t *ctx, const uint32_t *key)
{
    register uint32_t _ctx __asm__("r0") = (uint32_t)ctx;
    register uint32_t _key __asm__("r1") = (uint32_t)key;
    
    __asm__(
        "   push    {r4-r8, lr}                             \n"

        "   movw    r14, #:lower16:stm32wb_boot_aes128_sbox \n"
        "   movt    r14, #:upper16:stm32wb_boot_aes128_sbox \n"

        "   ldmia.w r1, {r2-r5}                             \n" // load key once // align loop entry to 8 bytes
        "   mov.w   r1, #0x01000000                         \n" // calculate rcon in highest byte to use a carry flag

        //just copy a key
        "   stmia.w r0!, {r2-r5}                            \n" // align loop entry to 8 bytes

        "1: uxtb    r6, r5, ror #8                          \n"
        "   uxtb    r7, r5, ror #16                         \n"
        "   uxtb    r8, r5, ror #24                         \n"
        "   uxtb    r12, r5                                 \n"

        "   ldrb    r6, [r14, r6]                           \n"
        "   ldrb    r7, [r14, r7]                           \n"
        "   ldrb    r8, [r14, r8]                           \n"
        "   ldrb    r12, [r14, r12]                         \n"

        "   eor     r2, r2, r1, lsr #24                     \n" // rcon is in highest byte
        "   eors    r2, r2, r6                              \n"
        "   eor     r2, r2, r7, lsl #8                      \n"
        "   eor     r2, r2, r8, lsl #16                     \n"
        "   eor     r2, r2, r12, lsl #24                    \n"
        "   eors    r3, r2                                  \n"
        "   eors    r4, r3                                  \n"
        "   eors    r5, r4                                  \n"

        "   lsls    r1, #1                                  \n" // next rcon

        "   it      cs \n" // 0x1b reduction when carry set
        "   movcs   r1, #0x1b000000                         \n"

        "   cmp     r1, #0x6c000000                         \n"

        //write roundkey
        "   stmia   r0!, {r2-r5}                            \n"

        "   bne     1b                                      \n"

        "   pop     {r4-r8, pc}                             \n"
        :
        : "l" (_ctx), "l" (_key)
        );
}

static void  __attribute__((naked, noinline)) stm32wb_boot_aes128_encrypt(stm32wb_boot_aes128_context_t *ctx, const uint32_t *in, uint32_t *out)
{
    register uint32_t _ctx __asm__("r0") = (uint32_t)ctx;
    register uint32_t _in  __asm__("r1") = (uint32_t)in;
    register uint32_t _out __asm__("r2") = (uint32_t)out;

    __asm__(
        "   add     r3, r0, #160                           \n" //rk_end-16 = rk + rounds * 16
        "   push    {r2,r3,r4-r11,lr}                      \n" //stack out, rk_end-16

        "   mov     r14, r0                                \n"

        //load input
        "   ldmia   r1!, {r4-r7}                           \n"
        //load key
        "   ldmia   r14!, {r0-r3}                          \n"

        //initial addroundkey
        "   eors    r0, r4                                 \n"
        "   eors    r1, r5                                 \n"
        "   eors    r2, r6                                 \n"
        "   eors    r3, r7                                 \n"

        "   movw    r7, #:lower16:stm32wb_boot_aes128_sbox \n"
        "   movt    r7, #:upper16:stm32wb_boot_aes128_sbox \n"

        //shiftrows and subbytes
        //row 2 - ST2x
        "1: uxtb    r8, r2, ror #16                        \n"
        "   uxtb    r9, r3, ror #16                        \n"
        "   uxtb    r10, r0, ror #16                       \n"
        "   uxtb    r11, r1, ror #16                       \n"

        //row 3 - ST3x
        "   lsrs    r4, r3, #24                            \n"
        "   lsrs    r5, r0, #24                            \n"
        "   lsrs    r6, r1, #24                            \n"
        "   uxtb    r12, r2, ror #24                       \n"

        //halfway sboxing
        "   ldrb    r4, [r7, r4]                           \n"
        "   ldrb    r5, [r7, r5]                           \n"
        "   ldrb    r6, [r7, r6]                           \n"
        "   ldrb    r12, [r7, r12]                         \n"
        "   ldrb    r8, [r7, r8]                           \n"
        "   ldrb    r9, [r7, r9]                           \n"
        "   ldrb    r10, [r7, r10]                         \n"
        "   ldrb    r11, [r7, r11]                         \n"

        //repack upper part (keep in bottom half)
        "   orr     r8, r8, r4, lsl #8                     \n"
        "   orr     r9, r9, r5, lsl #8                     \n"
        "   orr     r10, r10, r6, lsl #8                   \n"
        "   orr     r11, r11, r12, lsl #8                  \n"

        //row 1 - ST1x
        "   uxtb    r4, r1, ror #8                         \n"
        "   uxtb    r5, r2, ror #8                         \n"
        "   uxtb    r6, r3, ror #8                         \n"
        "   uxtb    r12, r0, ror #8                        \n"

        //row 0 - ST0x
        "   uxtb    r0, r0                                 \n"
        "   uxtb    r1, r1                                 \n"
        "   uxtb    r2, r2                                 \n"
        "   uxtb    r3, r3                                 \n"

        //rest of the sboxing
        "   ldrb    r0, [r7, r0]                           \n"
        "   ldrb    r1, [r7, r1]                           \n"
        "   ldrb    r2, [r7, r2]                           \n"
        "   ldrb    r3, [r7, r3]                           \n"
        "   ldrb    r4, [r7, r4]                           \n"
        "   ldrb    r5, [r7, r5]                           \n"
        "   ldrb.w  r6, [r7, r6]                           \n" // loses cycles if .n below r12 load // align next load
        "   ldrb    r12, [r7, r12]                         \n"

        //repack bottom part
        "   orr     r0, r0, r4, lsl #8                     \n"
        "   orr     r1, r1, r5, lsl #8                     \n"
        "   orr     r2, r2, r6, lsl #8                     \n"
        "   orr     r3, r3, r12, lsl #8                    \n"

        //repack wholly
        "   orr     r0, r0, r8, lsl #16                    \n"
        "   orr     r1, r1, r9, lsl #16                    \n"
        "   orr     r2, r2, r10, lsl #16                   \n"
        "   orr     r3, r3, r11, lsl #16                   \n"

        // do mix columns as
        // tmp = s0 ^ s1 ^ s2 ^ s3
        // s0` ^= tmp ^ gmul2(s0^s1) // s1^s2^s3^gmul2(s0^s1)
        // s1` ^= tmp ^ gmul2(s1^s2) // s0^s2^s3^gmul2(s1^s2)
        // s2` ^= tmp ^ gmul2(s2^s3) // s0^s1^s3^gmul2(s2^s3)
        // S3` ^= tmp ^ gmul2(s3^s0) // s0^s1^s2^gmul2(s3^s0)
        
        //col 0 - STx0
        "   eor     r4, r0, r0, ror #8                     \n" // r4 = s0^s1 | s1^s2 | s2^s3 | s3^s0
        "   eor     r5, r4, r0, ror #16                    \n" // r5 = s0^s1^s2 | s1^s2^s3 | s0^s2^s3 | s0^s1^s3

        //perform quad gfmul in constant time
        "   uadd8   r6, r4, r4                             \n" // quad lsl #1
        "   eor     r8, r6, #0x1b1b1b1b                    \n"
        "   sel     r4, r8, r6                             \n" // if uadd carried then take reduced byte

        "   eor     r0, r4, r5, ror #8                     \n" // effective r5 = s1^s2^s3 | s0^s2^s3 | s0^s1^s3 | s0^s1^s2

        //col 1 - STx1
        "   eor     r4, r1, r1, ror #8                     \n" // r4 = s0^s1 | s1^s2 | s2^s3 | s3^s0
        "   eor     r5, r4, r1, ror #16                    \n" // r5 = s0^s1^s2 | s1^s2^s3 | s0^s2^s3 | s0^s1^s3

        //perform quad gfmul in constant time
        "   uadd8   r6, r4, r4                             \n" // quad lsl #1
        "   eor     r8, r6, #0x1b1b1b1b                                 \n"
        "   sel     r4, r8, r6                             \n" // if uadd carried then take reduced byte

        "   eor     r1, r4, r5, ror #8                     \n" // effective r5 = s1^s2^s3 | s0^s2^s3 | s0^s1^s3 | s0^s1^s2

        //col 2 - STx2
        "   eor     r4, r2, r2, ror #8                     \n" // r4 = s0^s1 | s1^s2 | s2^s3 | s3^s0
        "   eor     r5, r4, r2, ror #16                    \n" // r5 = s0^s1^s2 | s1^s2^s3 | s0^s2^s3 | s0^s1^s3

        //perform quad gfmul in constant time
        "   uadd8   r6, r4, r4                             \n" // quad lsl #1
        "   eor     r8, r6, #0x1b1b1b1b                    \n"
        "   sel     r4, r8, r6                             \n" // if uadd carried then take reduced byte

        "   eor     r2, r4, r5, ror #8                     \n" // effective r5 = s1^s2^s3 | s0^s2^s3 | s0^s1^s3 | s0^s1^s2

        //col 3 - STx3
        "   eor     r4, r3, r3, ror #8                     \n" // r4 = s0^s1 | s1^s2 | s2^s3 | s3^s0
        "   eor     r5, r4, r3, ror #16                    \n" // r5 = s0^s1^s2 | s1^s2^s3 | s0^s2^s3 | s0^s1^s3

        //perform quad gfmul in constant time
        "   uadd8   r6, r4, r4                             \n" // quad lsl #1
        "   eor     r8, r6, #0x1b1b1b1b                    \n"
        "   sel     r4, r8, r6                             \n" // if uadd carried then take reduced byte

        "   eor     r3, r4, r5, ror #8                     \n" // effective r5 = s1^s2^s3 | s0^s2^s3 | s0^s1^s3 | s0^s1^s2

        //addroundkey
        // aggregate loads by source in case it lies in different memory blocks
        "   ldr     r5, [r14, #4]                          \n"
        "   ldr     r8, [r14, #8]                          \n"
        "   ldr     r9, [r14, #12]                         \n"
        "   ldr     r4, [r14], #16                         \n"
        "   ldr     r6, [sp, #4]                           \n" // get final condition

        "   eors    r0, r4                                 \n"
        "   eors    r1, r5                                 \n"

        "   cmp     r6, r14                                \n"

        "   eor.w   r2, r8                                 \n"
        "   eor.w   r3, r9                                 \n"

        "   bne.w   1b                                     \n" // out of range

        //final round
        //row 2 - ST2x
        "   uxtb    r8, r2, ror #16                        \n"
        "   uxtb    r9, r3, ror #16                        \n"
        "   uxtb    r10, r0, ror #16                       \n"
        "   uxtb    r11, r1, ror #16                       \n"

        //row 3 - ST3x
        "   lsrs    r4, r3, #24                            \n"
        "   lsrs    r5, r0, #24                            \n"
        "   lsrs    r6, r1, #24                            \n"
        "   uxtb    r12, r2, ror #24                       \n"

        //halfway sboxing
        "   ldrb    r4, [r7, r4]                           \n"
        "   ldrb    r5, [r7, r5]                           \n"
        "   ldrb    r6, [r7, r6]                           \n"
        "   ldrb    r12, [r7, r12]                         \n"
        "   ldrb    r8, [r7, r8]                           \n"
        "   ldrb    r9, [r7, r9]                           \n"
        "   ldrb    r10, [r7, r10]                         \n"
        "   ldrb    r11, [r7, r11]                         \n"

        //repack upper part (keep in bottom half)
        "   orr     r8, r8, r4, lsl #8                     \n"
        "   orr     r9, r9, r5, lsl #8                     \n"
        "   orr     r10, r10, r6, lsl #8                   \n"
        "   orr     r11, r11, r12, lsl #8                  \n"

        //row 1 - ST1x
        "   uxtb    r4, r1, ror #8                         \n"
        "   uxtb    r5, r2, ror #8                         \n"
        "   uxtb    r6, r3, ror #8                         \n"
        "   uxtb    r12, r0, ror #8                        \n"

        //row 0 - ST0x
        "   uxtb    r0, r0                                 \n"
        "   uxtb    r1, r1                                 \n"
        "   uxtb    r2, r2                                 \n"
        "   uxtb    r3, r3                                 \n"

        //rest of the sboxing
        "   ldrb    r0, [r7, r0]                           \n"
        "   ldrb    r1, [r7, r1]                           \n"
        "   ldrb    r2, [r7, r2]                           \n"
        "   ldrb    r3, [r7, r3]                           \n"
        "   ldrb    r4, [r7, r4]                           \n"
        "   ldrb    r5, [r7, r5]                           \n"
        "   ldrb.w  r6, [r7, r6]                           \n" // align next load
        "   ldrb    r12, [r7, r12]                         \n"

        //repack bottom part
        "   orr     r0, r0, r4, lsl #8                     \n"
        "   orr     r1, r1, r5, lsl #8                     \n"
        "   orr     r2, r2, r6, lsl #8                     \n"
        "   orr     r3, r3, r12, lsl #8                    \n"

        //repack wholly
        "   orr     r0, r0, r8, lsl #16                    \n"
        "   orr     r1, r1, r9, lsl #16                    \n"
        "   orr     r2, r2, r10, lsl #16                   \n"
        "   orr     r3, r3, r11, lsl #16                   \n"

        "   ldr     r7, [sp], #8                           \n" // load output pointer and clear stack

        //final addroudkey
        "   ldr     r4, [r14]                              \n"
        "   ldr     r5, [r14, #4]                          \n"
        "   ldr     r6, [r14, #8]                          \n"
        "   ldr     r8, [r14, #12]                         \n"

        "   eors    r0, r4                                 \n"
        "   eors    r1, r5                                 \n"
        "   eors    r2, r6                                 \n"
        "   eor.w   r3, r8                                 \n"

        "   str     r0, [r7]                               \n"
        "   str     r1, [r7, #4]                           \n"
        "   str     r2, [r7, #8]                           \n"
        "   str     r3, [r7, #12]                          \n"

        "   pop     {r4-r11,pc}                            \n"
        :
        : "r" (_ctx), "r" (_in), "r" (_out)
        );
}


/************************************************************************************************************************************/

#define SWAP(_x)      (uint32_t)((((_x) >> 24) & 0x000000ff) | (((_x) >> 8) & 0x0000ff00) | (((_x) << 8) & 0x00ff0000) |  (((_x) << 24) & 0xff000000))
#define SHR(_x,_n)    (uint32_t)(((_x) >> (_n)))
#define ROTR(_x,_n)   (uint32_t)(((_x) >> (_n)) | ((_x) << (32 - (_n))))
#define ROTL(_x,_n)   (uint32_t)(((_x) << (_n)) | ((_x) >> (32 - (_n))))
#define CH(_x,_y,_z)  (uint32_t)(((_x) & (_y)) ^ ( (~(_x)) & (_z)))
#define MAJ(_x,_y,_z) (uint32_t)(((_x) & (_y)) ^ ((_x) & (_z)) ^ ((_y) & (_z)))

#define BSIG0(_x)     (uint32_t)(ROTR((_x), 2) ^ ROTR((_x),13) ^ ROTR((_x),22))
#define BSIG1(_x)     (uint32_t)(ROTR((_x), 6) ^ ROTR((_x),11) ^ ROTR((_x),25))
#define SSIG0(_x)     (uint32_t)(ROTR((_x), 7) ^ ROTR((_x),18) ^  SHR((_x), 3))
#define SSIG1(_x)     (uint32_t)(ROTR((_x),17) ^ ROTR((_x),19) ^  SHR((_x),10))

static const uint32_t stm32wb_boot_sha256_const_K[64] =
{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

#if 0
static const uint32_t stm32wb_boot_sha256_const_H[8] =
{
    // running out of .rodata space
    0x5be0cd19, 0x1f83d9ab, 0x9b05688c, 0x510e527f, 0xa54ff53a, 0x3c6ef372, 0xbb67ae85, 0x6a09e667,
};
#endif

static void __attribute__((noinline, optimize("O3"))) stm32wb_boot_sha256_process(uint32_t *H, const uint32_t *Mp)
{
    uint32_t T1, T2, W[72];
    uint32_t *Wp, *Wp_e;
    volatile uint32_t *Hp;
    const uint32_t *Kp;

    for (Wp = &W[0], Wp_e = Wp + 8, Hp = &H[0]; Wp < Wp_e; Wp++, Hp++)
    {
        Wp[0] = Hp[0];
    }

    for (Wp_e = Wp + 16; Wp < Wp_e; Wp++, Mp++)
    {
        Wp[0] = SWAP(Mp[0]);
    }
    
    for (Wp = &W[8], Wp_e = Wp + 48; Wp < Wp_e; Wp++)
    {
        Wp[16] = SSIG1(Wp[14]) + Wp[9] + SSIG0(Wp[1]) + Wp[0];
    }
    
    for (Wp = &W[0], Wp_e = Wp + 64, Kp = &stm32wb_boot_sha256_const_K[0]; Wp < Wp_e; Wp++, Kp++)
    {
        T1 = Wp[0] + BSIG1(Wp[3]) + CH(Wp[3],Wp[2],Wp[1]) + Kp[0] + Wp[8];
        T2 = BSIG0(Wp[7]) + MAJ(Wp[7],Wp[6],Wp[5]);

        Wp[4] = Wp[4] + T1;
        Wp[8] = T1 + T2;
    }

    for (Wp_e = Wp + 8, Hp = &H[0]; Wp < Wp_e; Wp++, Hp++)
    {
        Hp[0] += Wp[0];
    }
}

static void stm32wb_boot_sha256_init(stm32wb_boot_sha256_context_t *sha256_ctx)
{
    sha256_ctx->length = 0;
    sha256_ctx->index = 0;

#if 0
    // running out of .rodata space
    stm32wb_boot_memcpy(&sha256_ctx->hash[0], &stm32wb_boot_sha256_const_H[0], 32);
#endif
    
    sha256_ctx->hash[0] = 0x5be0cd19;
    sha256_ctx->hash[1] = 0x1f83d9ab;
    sha256_ctx->hash[2] = 0x9b05688c;
    sha256_ctx->hash[3] = 0x510e527f;
    sha256_ctx->hash[4] = 0xa54ff53a;
    sha256_ctx->hash[5] = 0x3c6ef372;
    sha256_ctx->hash[6] = 0xbb67ae85;
    sha256_ctx->hash[7] = 0x6a09e667;
}

static void stm32wb_boot_sha256_update(stm32wb_boot_sha256_context_t *sha256_ctx, const uint8_t *data, size_t size)
{
    uint32_t index;
  
    sha256_ctx->length += size;

    index = sha256_ctx->index;
    
    while (size)
    {
        if ((size >= 64) && (index == 0) && !((uint32_t)data & 3))
        {
            do
            {
                stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)data);
                
                data += 64;
                size -= 64;
            }
            while (size >= 64);
        }
        else
        {
            sha256_ctx->data[index++] = *data++;
            
            size--;
            
            if (index == 64)
            {
                stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0]);
                
                index = 0;
            }
        }
    }
    
    sha256_ctx->index = index;
}

static void stm32wb_boot_sha256_final(stm32wb_boot_sha256_context_t *sha256_ctx, uint32_t *hash)
{
    uint32_t *Hp, *Hp_e;
    
    sha256_ctx->data[sha256_ctx->index++] = 0x80;

    if (sha256_ctx->index > (64-8))
    {
        stm32wb_boot_memset(&sha256_ctx->data[sha256_ctx->index], 0x00, 64 - sha256_ctx->index);
        
        stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0]);

        sha256_ctx->index = 0;
    }

    if (sha256_ctx->index != (64-8+3))
    {
        stm32wb_boot_memset(&sha256_ctx->data[sha256_ctx->index], 0x00, (64-8+3) - sha256_ctx->index);
    }

    sha256_ctx->data[59] = sha256_ctx->length >> (32-3);
    sha256_ctx->data[60] = sha256_ctx->length >> (24-3);
    sha256_ctx->data[61] = sha256_ctx->length >> (16-3);
    sha256_ctx->data[62] = sha256_ctx->length >>  (8-3);
    sha256_ctx->data[63] = sha256_ctx->length <<    (3);

    stm32wb_boot_sha256_process(&sha256_ctx->hash[0], (const uint32_t*)&sha256_ctx->data[0]);

    for (Hp = &sha256_ctx->hash[0], Hp_e = Hp + 8, hash = hash + 7; Hp < Hp_e; Hp++, hash--)
    {
        hash[0] = SWAP(Hp[0]);
    }
}

/************************************************************************************************************************************/

#define STM32WB_BOOT_RSA2048_NUM_BYTES  256
#define STM32WB_BOOT_RSA2048_NUM_WORDS  64

static inline uint64_t __attribute__((optimize("O3"))) stm32wb_boot_rsa2048_mula32(uint32_t a, uint32_t b, uint32_t c)
{
    uint64_t ret = a;
    ret *= b;
    ret += c;
    return ret;
}

static inline uint64_t __attribute__((optimize("O3"))) stm32wb_boot_rsa2048_mulaa32(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    uint64_t ret = a;
    ret *= b;
    ret += c;
    ret += d;
    return ret;
}

/**
 * a[] -= mod
 */
static void __attribute((noinline, optimize("O3"))) stm32wb_boot_rsa2048_mod_sub(const stm32wb_boot_rsa2048_key_t *key, uint32_t *a)
{
    const uint32_t *nn, *nn_e;
    uint32_t *aa;
    int64_t A;

    aa = a;
    nn = key->n;
    nn_e = key->n + STM32WB_BOOT_RSA2048_NUM_WORDS;

    A = 0;
    
    do
    {
        A += (uint64_t)aa[0] - (uint64_t)nn[0];
        aa[0] = (uint32_t)A;
        A >>= 32;

        nn++;
        aa++;
    }
    while (nn != nn_e);
}

/**
 * Return sign of a[] - mod
 */
static int __attribute((noinline, optimize("O3"))) stm32wb_boot_rsa2048_mod_cmp(const stm32wb_boot_rsa2048_key_t *key, const uint32_t *a)
{
    const uint32_t *aa, *nn, *nn_e;

    aa = a + STM32WB_BOOT_RSA2048_NUM_WORDS - 1;
    nn = key->n + STM32WB_BOOT_RSA2048_NUM_WORDS - 1;
    nn_e = key->n -1;
    
    do
    {
        if (aa[0] < nn[0])
        {
            return -1;
        }

        if (aa[0] > nn[0])
        {
            return 1;
        }

        aa--;
        nn--;
    }
    while (nn != nn_e);

    return 0;
}

/**
 * Montgomery c[] = a[] * b[] / R % mod
 */
static void __attribute__((noinline, optimize("O3"))) stm32wb_boot_rsa2048_mod_mul(const stm32wb_boot_rsa2048_key_t *key, uint32_t *c, const uint32_t *a, const uint32_t *b)
{
    const uint32_t *aa, *aa_e, *bb, *nn, *nn_e;
    uint32_t *cc;
    uint64_t A, B;
    uint32_t d0;
    
    stm32wb_boot_memset((uint8_t*)c, 0, STM32WB_BOOT_RSA2048_NUM_WORDS * 4);

    cc = c;
    
    aa = a;
    aa_e = aa + STM32WB_BOOT_RSA2048_NUM_WORDS;
    nn_e = key->n + STM32WB_BOOT_RSA2048_NUM_WORDS;
    
    do
    {
        nn = key->n;
        bb = b;
        cc = c;
        
        A = stm32wb_boot_rsa2048_mula32(aa[0], bb[0], cc[0]);
        d0 = (uint32_t)A * key->n0inv;
        B = stm32wb_boot_rsa2048_mula32(d0, nn[0], A);
        
        nn++;
        bb++;
        
        do
        {
            A = stm32wb_boot_rsa2048_mulaa32(aa[0], bb[0], cc[1], A >> 32);
            B = stm32wb_boot_rsa2048_mulaa32(d0, nn[0], A, B >> 32);
            cc[0] = (uint32_t)B;
            
            nn++;
            bb++;
            cc++;
        }
        while (nn != nn_e);
        
        A = (A >> 32) + (B >> 32);
        
        cc[0] = (uint32_t)A;
        
        if (A >> 32)
        {
            stm32wb_boot_rsa2048_mod_sub(key, c);
        }

        aa++;
    }
    while (aa != aa_e);
}

static void stm32wb_boot_rsa2048_mod_exp(const stm32wb_boot_rsa2048_key_t *key, uint32_t *b, const uint32_t *in)
{
    const uint32_t *in_e;
    uint32_t *a, *aa, *aaa, *b_e;
    uint32_t a_r[STM32WB_BOOT_RSA2048_NUM_WORDS];
    uint32_t aa_r[STM32WB_BOOT_RSA2048_NUM_WORDS];
    int i;

    a = b;
    aa = a + STM32WB_BOOT_RSA2048_NUM_WORDS -1;
    aaa = aa_r;
        
    in_e = in + STM32WB_BOOT_RSA2048_NUM_WORDS;

    do
    {
        aa[0] = SWAP(in[0]);

        aa--;
        in++;
    }
    while (in != in_e);
        
    /* Exponent 65537 */

    stm32wb_boot_rsa2048_mod_mul(key, a_r, a, key->rr); /* a_r = a * RR / R mod M */

    for (i = 0; i < 16; i += 2)
    {
        stm32wb_boot_rsa2048_mod_mul(key, aa_r, a_r, a_r); /* aa_r = a_r * a_r / R mod M */
        stm32wb_boot_rsa2048_mod_mul(key, a_r, aa_r, aa_r); /* a_r = aa_r * aa_r / R mod M */
    }

    stm32wb_boot_rsa2048_mod_mul(key, aaa, a_r, a); /* aaa = a_r * a / R mod M */

    if (stm32wb_boot_rsa2048_mod_cmp(key, aaa) >= 0)
    {
        stm32wb_boot_rsa2048_mod_sub(key, aaa);
    }

    aa = aaa + STM32WB_BOOT_RSA2048_NUM_WORDS -1;
    b_e = b + STM32WB_BOOT_RSA2048_NUM_WORDS;

    do
    {
        b[0] = SWAP(aa[0]);

        aa--;
        b++;
    }
    while (b != b_e);
}

#define STM32WB_BOOT_RSA2048_SIGNATURE_SIZE 256
#define STM32WB_BOOT_RSA2048_MASK_LEN       (STM32WB_BOOT_RSA2048_SIGNATURE_SIZE - STM32WB_BOOT_SHA256_HASH_SIZE - 1)
#define STM32WB_BOOT_RSA2048_ZERO_LEN       (STM32WB_BOOT_RSA2048_MASK_LEN - STM32WB_BOOT_SHA256_HASH_SIZE - 1)

#define STM32WB_BOOT_RSA2048_ZERO_OFFSET    0
#define STM32WB_BOOT_RSA2048_ONE_OFFSET     STM32WB_BOOT_RSA2048_ZERO_LEN
#define STM32WB_BOOT_RSA2048_SALT_OFFSET    (STM32WB_BOOT_RSA2048_ONE_OFFSET + 1)
#define STM32WB_BOOT_RSA2048_HASH_OFFSET    STM32WB_BOOT_RSA2048_MASK_LEN

static const uint8_t stm32wb_boot_rsa2048_zeros[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

static bool stm32wb_boot_rsa2048_verify(const stm32wb_boot_rsa2048_key_t *key, const uint32_t *signature, const uint32_t *digest)
{
    stm32wb_boot_sha256_context_t sha256_ctx;
    uint32_t hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t scratch[STM32WB_BOOT_RSA2048_NUM_WORDS];
    uint8_t *out, *mask, C[4] = { 0, 0, 0, 0 };;
    uint32_t index, count, size;

    stm32wb_boot_rsa2048_mod_exp(key, scratch, signature);
    
    out = (uint8_t*)&scratch[0];

    if (out[STM32WB_BOOT_RSA2048_SIGNATURE_SIZE - 1] != 0xbc)
    {
        return false;
    }

    index = 0;
    size = STM32WB_BOOT_RSA2048_MASK_LEN;

    while (size)
    {
        stm32wb_boot_sha256_init(&sha256_ctx);
        stm32wb_boot_sha256_update(&sha256_ctx, &out[STM32WB_BOOT_RSA2048_HASH_OFFSET], STM32WB_BOOT_SHA256_HASH_SIZE);
        stm32wb_boot_sha256_update(&sha256_ctx, C, 4);
        stm32wb_boot_sha256_final(&sha256_ctx, hash);

        C[3]++;

        count = STM32WB_BOOT_SHA256_HASH_SIZE;

        if (count > size)
        {
            count = size;
        }

        size -= count;
        mask = (uint8_t*)&hash[0];

        while (count--)
        {
            out[index++] ^= *mask++;
        }
    }
    
    out[0] &= 0x7f;
    
    for (index = 0; index < STM32WB_BOOT_RSA2048_ZERO_LEN; index++)
    {
        if (out[index] != 0x00)
        {
            return false;
        }
    }

    if (out[STM32WB_BOOT_RSA2048_ONE_OFFSET] != 0x01)
    {
        return false;
    }

    stm32wb_boot_sha256_init(&sha256_ctx);
    stm32wb_boot_sha256_update(&sha256_ctx, stm32wb_boot_rsa2048_zeros, 8);
    stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)digest, STM32WB_BOOT_SHA256_HASH_SIZE);
    stm32wb_boot_sha256_update(&sha256_ctx, &out[STM32WB_BOOT_RSA2048_SALT_OFFSET], STM32WB_BOOT_SHA256_HASH_SIZE);
    stm32wb_boot_sha256_final(&sha256_ctx, hash);

    if (stm32wb_boot_memcmp(&out[STM32WB_BOOT_RSA2048_HASH_OFFSET], (const uint8_t*)hash, STM32WB_BOOT_SHA256_HASH_SIZE))
    {
        return false;
    }
    
    return true;
}

/************************************************************************************************************************************/

void stm32wb_boot_entry(void)
{
    stm32wb_boot_sha256_context_t sha256_ctx;
    const stm32wb_application_info_t *application_info;
    uint32_t sha256_hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t application_magic, application_base, application_size, vtor_address, stack_address, entry_address;
    bool boot_dfu;
    
    RCC->CIER = 0x00000000;

    if (!(RCC->CR & RCC_CR_HSION))
    {
        RCC->CR |= RCC_CR_HSION;
        
        while (!(RCC->CR & RCC_CR_HSIRDY))
        {
        }
    }

    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
            
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
        {
        }
    }

    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSI;

    RCC->CR &= ~RCC_CR_PLLON;
    RCC->CR &= ~RCC_CR_MSION;

    if (PWR->EXTSCR & PWR_EXTSCR_C1SBF)
    {
        application_base = STM32WB_APPLICATION_BASE;

        vtor_address  = application_base;
        stack_address = ((volatile uint32_t * volatile)application_base)[0];
        entry_address = ((volatile uint32_t * volatile)application_base)[1];
        
        SCB->VTOR = vtor_address;
        __DSB();

        __asm__ volatile (
            "   mov     sp, %0       \n"
            "   bx      %1           \n"
            :
            : "l" (stack_address),  "l" (entry_address)
            );
    }
    
    /* The STM32 BOOTLOADER does not do a reset when jumping to the newly flashed
     * code. Hence CPU2 might be booted. If so, do a software reset.
     */
    if (PWR->CR4 & PWR_CR4_C2BOOT)
    {
        stm32wb_boot_reset();
    }

    /* Switch to Main Flash @ 0x00000000. Make sure the I/D CACHE is
     * disabled to avoid stale data in the cache.
     */

    FLASH->ACR = FLASH->ACR & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    
    SYSCFG->MEMRMP = 0;

    FLASH->ACR = FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;
    FLASH->ACR;

    if ((PWR->CR1 & PWR_CR1_VOS) != PWR_CR1_VOS_RANGE_1)
    {
        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_RANGE_1;
            
        while (PWR->SR2 & PWR_SR2_VOSF)
        {
        }
    }

    /* Add WPROT/PCROP and BOOT/PH3 override if not RDP Level 0.
     */
    if (((stm32wb_boot_info.options & STM32WB_BOOT_OPTION_RDP_LEVEL_MASK) != STM32WB_BOOT_OPTION_RDP_LEVEL_0) && ((stm32wb_boot_info.options & STM32WB_BOOT_OPTION_RDP_LEVEL_MASK) != (FLASH->OPTR & FLASH_OPTR_RDP)))
    {
        FLASH->KEYR      = 0x45670123;
        FLASH->KEYR      = 0xcdef89ab;
        
        FLASH->OPTKEYR   = 0x08192a3b;
        FLASH->OPTKEYR   = 0x4c5d6e7f;
        
        FLASH->OPTR      = (FLASH->OPTR & ~(FLASH_OPTR_nSWBOOT0 | FLASH_OPTR_RDP)) | (stm32wb_boot_info.options & STM32WB_BOOT_OPTION_RDP_LEVEL_MASK);
        FLASH->PCROP1ASR = 0x00000001;
        FLASH->PCROP1AER = 0x80000007;
        FLASH->WRP1AR    = 0x00030000;

        __DSB();
        __ISB();
        
        FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
        FLASH->CR = FLASH_CR_OPTSTRT;
        
        while (FLASH->SR & FLASH_SR_BSY)
        {
        }
        
        FLASH->CR |= FLASH_CR_OBL_LAUNCH;
        
        __DSB();
        __ISB();
        
        FLASH->CR |= (FLASH_CR_LOCK | FLASH_CR_OPTLOCK);
        
        stm32wb_boot_reset();
    }

    if (!(FLASH->OPTR & FLASH_OPTR_nSWBOOT0))
    {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
        RCC->AHB2ENR;

        GPIOH->MODER = (GPIOH->MODER & ~0x000000c0) | 0x00000000;

        boot_dfu = !!(GPIOH->IDR & 0x0008);

        GPIOH->MODER = (GPIOH->MODER & ~0x000000c0) | 0x000000c0;

        RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOHEN;
    }
    else
    {
        boot_dfu = false;
    }
    
    RCC->CR = (RCC->CR & ~(RCC_CR_MSIRANGE | RCC_CR_MSIPLLEN)) | RCC_CR_MSIRANGE_6 | RCC_CR_MSION;
    
    __DSB();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    
    while (!(RCC->CR & RCC_CR_MSIRDY))
    {
    }

    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_MSI;

    RCC->PLLCFGR = ((1 << RCC_PLLCFGR_PLLR_Pos) | (64000000 / (4000000 / 2)) << RCC_PLLCFGR_PLLN_Pos) | (((1-1) << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLSRC_MSI);

    RCC->CR |= RCC_CR_PLLON;

    while (!(RCC->CR & RCC_CR_PLLRDY))
    {
    }

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) | (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2);

    while ((RCC->CFGR & (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F)) != (RCC_CFGR_HPREF | RCC_CFGR_PPRE1F | RCC_CFGR_PPRE2F))
    {
    }

    RCC->EXTCFGR = (RCC->EXTCFGR & ~(RCC_EXTCFGR_SHDHPRE | RCC_EXTCFGR_C2HPRE | RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) | RCC_EXTCFGR_SHDHPRE_DIV1 | RCC_EXTCFGR_C2HPRE_DIV512;
    
    while ((RCC->EXTCFGR & (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF)) != (RCC_EXTCFGR_SHDHPREF | RCC_EXTCFGR_C2HPREF))
    {
    }
    
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
    }

    SCB->CCR = 0;
    SCB->SHCSR = 0;
#if (__FPU_PRESENT == 1)
    SCB->CPACR = 0x00f00000;
#endif /* __FPU_PRESENT == 1 */
    
    NVIC_SetPriority(SVCall_IRQn, ARMV7M_IRQ_PRIORITY_SVCALL);
    NVIC_SetPriority(PendSV_IRQn, ARMV7M_IRQ_PRIORITY_PENDSV);
    
    stm32wb_boot_memcpy((uint8_t*)__boot_data_start__, (const uint8_t*)__boot_data_flash__, (uint32_t)__boot_data_end__ - (uint32_t)__boot_data_start__);
    stm32wb_boot_memset((uint8_t*)__boot_bss_start__, 0, (uint32_t)__boot_bss_end__ - (uint32_t)__boot_bss_start__);
    
    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
    
    PWR->CR1 |= PWR_CR1_DBP;
    
    while (!(PWR->CR1 & PWR_CR1_DBP))
    {
    }
    
#if 0    
    if (RCC->BDCR & RCC_BDCR_RTCEN)
    {
        if ((((RTC->BKP16R & STM32WB_RTC_BKP16R_DATA_MASK) >> STM32WB_RTC_BKP16R_DATA_SHIFT) != ((~RTC->BKP16R & STM32WB_RTC_BKP16R_NOT_DATA_MASK) >> STM32WB_RTC_BKP16R_NOT_DATA_SHIFT)) ||
            ((RTC->BKP16R & STM32WB_RTC_BKP16R_REVISION_MASK) != STM32WB_RTC_BKP16R_REVISION_CURRENT))
        {
            /* Reset RTC after a BKP mismatch, but also go throu a CPU reset to start off clean.
             */
            RCC->BDCR &= ~RCC_BDCR_RTCEN;
            RCC->BDCR |= RCC_BDCR_BDRST;
            RCC->BDCR &= ~RCC_BDCR_BDRST;

            stm32wb_boot_reset();
        }
    }
#endif


    if ((((const uint32_t*)STM32WB_BOOT_FWU_REQUEST_BASE)[0] != STM32WB_BOOT_FWU_REQUEST_NONE) &&
        (((const uint32_t*)STM32WB_BOOT_FWU_STATUS_BASE)[0] == STM32WB_BOOT_FWU_STATUS_NONE))
    {
        stm32wb_boot_fwu();

        stm32wb_boot_reset();
    }
    
    application_magic = ((const stm32wb_application_vectors_t*)STM32WB_APPLICATION_BASE)->magic;
    application_base = ((const stm32wb_application_vectors_t*)STM32WB_APPLICATION_BASE)->base;
    application_size = ((const stm32wb_application_vectors_t*)STM32WB_APPLICATION_BASE)->size;
    
    if ((application_magic != STM32WB_APPLICATION_MAGIC) || (application_base != STM32WB_APPLICATION_BASE) || (application_size > STM32WB_APPLICATION_SIZE) || (application_size & 7))
    {
        stm32wb_boot_dfu(false);
    }
    
    application_info = (const stm32wb_application_info_t*)STM32WB_APPLICATION_INFO_BASE;
    
    if (stm32wb_boot_memcmp(&stm32wb_boot_info.uuid, &application_info->uuid, sizeof(stm32wb_boot_info.uuid)))
    {
        stm32wb_boot_dfu(false);
    }
    
    if (*((volatile uint32_t*)&stm32wb_boot_info.rsa2048_key.exponent))
    {
        stm32wb_boot_sha256_init(&sha256_ctx);
        stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)STM32WB_APPLICATION_BASE, (uint8_t*)&application_info->signature[0] - (uint8_t*)STM32WB_APPLICATION_BASE);
        stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&application_info->signature[64], application_size - ((uint8_t*)&application_info->signature[64] - (uint8_t*)STM32WB_APPLICATION_BASE));
        stm32wb_boot_sha256_final(&sha256_ctx, sha256_hash);
        
        if (!stm32wb_boot_rsa2048_verify(&stm32wb_boot_info.rsa2048_key, (const uint32_t*)&application_info->signature[0], (const uint32_t*)&sha256_hash[0]))
        {
            stm32wb_boot_dfu(false);
        }
    }

    if (RCC->BDCR & RCC_BDCR_RTCEN)
    {
        if (RTC->BKP16R & STM32WB_RTC_BKP16R_DFU)
        {
            stm32wb_boot_dfu(true);
        }
    }

    if (boot_dfu)
    {
        stm32wb_boot_dfu(true);
    }
    
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
            
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }

    RCC->SMPSCR = (RCC->SMPSCR & ~RCC_SMPSCR_SMPSSEL) | RCC_SMPSCR_SMPSSEL_HSI;

    RCC->CR &= ~RCC_CR_PLLON;
    RCC->CR &= ~RCC_CR_MSION;
    
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_RTCAPBEN;

    PWR->CR1 &= ~PWR_CR1_DBP;

    SCB->VTOR = STM32WB_APPLICATION_BASE;

    stm32wb_boot_continue(STM32WB_APPLICATION_BASE);
}

/************************************************************************************************************************************/

static uint32_t stm32wb_boot_fwu_application(void)
{
    stm32wb_boot_aes128_context_t aes128_ctx;
    stm32wb_boot_sha256_context_t sha256_ctx;
    const stm32wb_application_info_t *application_info, *candidate_info;
    const stm32wb_image_info_t *image_info;
    uint32_t aes128_data[STM32WB_BOOT_SHA256_BLOCK_SIZE /4], aes128_key[4], aes128_iv[4], aes128_xor[4];
    uint32_t sha256_hash[STM32WB_BOOT_SHA256_HASH_SIZE / 4];
    uint32_t candidate_magic, candidate_base, candidate_limit, candidate_size, candidate_length, image_offset, offset, count;
    bool encrypted;

    candidate_magic = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->magic;
    candidate_base = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->base;
    candidate_size = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->size;
    candidate_length = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->length & STM32WB_IMAGE_LENGTH_MASK;
    
    if ((candidate_magic != STM32WB_APPLICATION_MAGIC) || (candidate_base != STM32WB_APPLICATION_BASE) || (candidate_size > STM32WB_APPLICATION_SIZE) || (candidate_size & 7))
    {
        return STM32WB_BOOT_FWU_STATUS_ERR_TARGET;
    }

    if ((candidate_length > STM32WB_BOOT_FWU_APPLICATION_SIZE) || (candidate_length & 7))
    {
        return STM32WB_BOOT_FWU_STATUS_ERR_FILE;

    }

    if (candidate_length)
    {
        image_offset = candidate_length - sizeof(stm32wb_image_info_t);
        image_info = (const stm32wb_image_info_t*)(STM32WB_BOOT_FWU_APPLICATION_BASE + image_offset);

        if (image_info->magic != STM32WB_IMAGE_MAGIC)
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_FILE;
        }
        
        if (image_info->crc32 != stm32wb_boot_crc32((const uint8_t*)STM32WB_BOOT_FWU_APPLICATION_BASE, ((uint32_t)&image_info->crc32 - STM32WB_BOOT_FWU_APPLICATION_BASE), 0xffffffff))
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_FILE;
        }
    }
    else
    {
        image_info = NULL;
    }
    
    candidate_info = (const stm32wb_application_info_t*)(STM32WB_BOOT_FWU_APPLICATION_BASE + (STM32WB_APPLICATION_INFO_BASE - STM32WB_APPLICATION_BASE));

    if (stm32wb_boot_memcmp(&stm32wb_boot_info.uuid, &candidate_info->uuid, sizeof(stm32wb_boot_info.uuid)))
    {
        return STM32WB_BOOT_FWU_STATUS_ERR_TARGET;
    }

    if ((((const uint32_t*)STM32WB_APPLICATION_BASE)[0] != 0xffffffff) &&
        (((const uint32_t*)STM32WB_APPLICATION_BASE)[1] != 0xffffffff))
    {
        application_info = (const stm32wb_application_info_t*)STM32WB_APPLICATION_INFO_BASE;

        if (application_info->sequence > candidate_info->sequence)
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_FILE;
        }
    }

    if (((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->length & STM32WB_IMAGE_OPTION_ENCRYPTED)
    {
        stm32wb_boot_aes128_get_key(&aes128_key[0]);
        stm32wb_boot_aes128_set_key(&aes128_ctx, &aes128_key[0]);
        
        encrypted = true;
    }
    else
    {
        encrypted = false;
    }
    

    if (*((volatile uint32_t*)&stm32wb_boot_info.rsa2048_key.exponent))
    {
        stm32wb_boot_sha256_init(&sha256_ctx);
        
        if (encrypted)
        {
            aes128_iv[0] = 0;
            aes128_iv[1] = candidate_info->epoch;
            aes128_iv[2] = candidate_info->nonce[0];
            aes128_iv[3] = candidate_info->nonce[1];

            for (candidate_base = STM32WB_BOOT_FWU_APPLICATION_BASE, candidate_limit = candidate_base + candidate_size; candidate_base < candidate_limit; candidate_base += count)
            {
                count = 64;

                if (count > (candidate_limit - candidate_base))
                {
                    count = candidate_limit - candidate_base;
                }

                if ((candidate_base <  (STM32WB_BOOT_FWU_APPLICATION_BASE + (STM32WB_APPLICATION_INFO_BASE - STM32WB_APPLICATION_BASE))) ||
                    (candidate_base >= (STM32WB_BOOT_FWU_APPLICATION_BASE + (STM32WB_APPLICATION_INFO_LIMIT - STM32WB_APPLICATION_BASE))))
                {
                    for (offset = 0; offset < 16; offset += 4, aes128_iv[0]++)
                    {
                        stm32wb_boot_aes128_encrypt(&aes128_ctx, &aes128_iv[0], &aes128_xor[0]);
                    
                        if (aes128_iv[0] == 2)
                        {
                            aes128_data[offset+0] = ((const uint32_t*)candidate_base)[offset+0];
                            aes128_data[offset+1] = ((const uint32_t*)candidate_base)[offset+1];
                            aes128_data[offset+2] = ((const uint32_t*)candidate_base)[offset+2];
                            aes128_data[offset+3] = ((const uint32_t*)candidate_base)[offset+3] ^ aes128_xor[3];
                        }
                        else if (aes128_iv[0] == 3)
                        {
                            aes128_data[offset+0] = ((const uint32_t*)candidate_base)[offset+0] ^ aes128_xor[0];
                            aes128_data[offset+1] = 0;
                            aes128_data[offset+2] = ((const uint32_t*)candidate_base)[offset+2] ^ aes128_xor[2];
                            aes128_data[offset+3] = ((const uint32_t*)candidate_base)[offset+3] ^ aes128_xor[3];
                        }
                        else
                        {
                            aes128_data[offset+0] = ((const uint32_t*)candidate_base)[offset+0] ^ aes128_xor[0];
                            aes128_data[offset+1] = ((const uint32_t*)candidate_base)[offset+1] ^ aes128_xor[1];
                            aes128_data[offset+2] = ((const uint32_t*)candidate_base)[offset+2] ^ aes128_xor[2];
                            aes128_data[offset+3] = ((const uint32_t*)candidate_base)[offset+3] ^ aes128_xor[3];
                        }
                    }

                    stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&aes128_data[0], count);
                }
                else
                {
                    aes128_iv[0] += 4;

                    if (candidate_base < (STM32WB_BOOT_FWU_APPLICATION_BASE + (STM32WB_APPLICATION_INFO_LIMIT - STM32WB_APPLICATION_BASE) - sizeof(candidate_info->signature)))
                    {
                        stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)candidate_base, count);
                    }
                }
            }
        }
        else
        {
            stm32wb_boot_memcpy(&aes128_data[0], (const uint8_t*)STM32WB_BOOT_FWU_APPLICATION_BASE, 64);
            
            ((stm32wb_application_vectors_t*)&aes128_data[0])->length = 0;
            
            stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&aes128_data[0], 64);
            stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)(STM32WB_BOOT_FWU_APPLICATION_BASE + 64), (uint32_t)&candidate_info->signature[0] - STM32WB_BOOT_FWU_APPLICATION_BASE - 64);
            stm32wb_boot_sha256_update(&sha256_ctx, (const uint8_t*)&candidate_info->signature[64], candidate_size - ((uint32_t)&candidate_info->signature[64] - STM32WB_BOOT_FWU_APPLICATION_BASE));
        }
        
        stm32wb_boot_sha256_final(&sha256_ctx, sha256_hash);
        
        if (!stm32wb_boot_rsa2048_verify(&stm32wb_boot_info.rsa2048_key, (const uint32_t*)&candidate_info->signature[0], (const uint32_t*)&sha256_hash[0]))
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_FILE;
        }
    }

    if (!stm32wb_boot_flash_clean(STM32WB_APPLICATION_BASE, STM32WB_APPLICATION_LIMIT))
    {
        return STM32WB_BOOT_FWU_STATUS_ERR_ERASE;
    }

    if (encrypted)
    {
        aes128_iv[0] = 0;
        aes128_iv[1] = candidate_info->epoch;
        aes128_iv[2] = candidate_info->nonce[0];
        aes128_iv[3] = candidate_info->nonce[1];
        
        for (candidate_base = STM32WB_BOOT_FWU_APPLICATION_BASE, candidate_limit = candidate_base + candidate_size; candidate_base < candidate_limit; candidate_base += count)
        {
            count = 64;
            
            if (count > (candidate_limit - candidate_base))
            {
                count = candidate_limit - candidate_base;
            }
            
            if ((candidate_base <  (STM32WB_BOOT_FWU_APPLICATION_BASE + (STM32WB_APPLICATION_INFO_BASE - STM32WB_APPLICATION_BASE))) ||
                (candidate_base >= (STM32WB_BOOT_FWU_APPLICATION_BASE + (STM32WB_APPLICATION_INFO_LIMIT - STM32WB_APPLICATION_BASE))))
            {
                for (offset = 0; offset < 16; offset += 4, aes128_iv[0]++)
                {
                    stm32wb_boot_aes128_encrypt(&aes128_ctx, &aes128_iv[0], &aes128_xor[0]);
                    
                    if (aes128_iv[0] == 2)
                    {
                        aes128_data[offset+0] = ((const uint32_t*)candidate_base)[offset+0];
                        aes128_data[offset+1] = ((const uint32_t*)candidate_base)[offset+1];
                        aes128_data[offset+2] = ((const uint32_t*)candidate_base)[offset+2];
                        aes128_data[offset+3] = ((const uint32_t*)candidate_base)[offset+3] ^ aes128_xor[3];
                    }
                    else if (aes128_iv[0] == 3)
                    {
                        aes128_data[offset+0] = ((const uint32_t*)candidate_base)[offset+0] ^ aes128_xor[0];
                        aes128_data[offset+1] = 0;
                        aes128_data[offset+2] = ((const uint32_t*)candidate_base)[offset+2] ^ aes128_xor[2];
                        aes128_data[offset+3] = ((const uint32_t*)candidate_base)[offset+3] ^ aes128_xor[3];
                    }
                    else
                    {
                        aes128_data[offset+0] = ((const uint32_t*)candidate_base)[offset+0] ^ aes128_xor[0];
                        aes128_data[offset+1] = ((const uint32_t*)candidate_base)[offset+1] ^ aes128_xor[1];
                        aes128_data[offset+2] = ((const uint32_t*)candidate_base)[offset+2] ^ aes128_xor[2];
                        aes128_data[offset+3] = ((const uint32_t*)candidate_base)[offset+3] ^ aes128_xor[3];
                    }
                }

                if (!stm32wb_boot_flash_program(STM32WB_APPLICATION_BASE + (candidate_base - STM32WB_BOOT_FWU_APPLICATION_BASE), (const uint8_t*)&aes128_data[0], count))
                {
                    return STM32WB_BOOT_FWU_STATUS_ERR_PROG;
                }
            }
            else
            {
                aes128_iv[0] += 4;

                if (!stm32wb_boot_flash_program(STM32WB_APPLICATION_BASE + (candidate_base - STM32WB_BOOT_FWU_APPLICATION_BASE), (const uint8_t*)candidate_base, count))
                {
                    return STM32WB_BOOT_FWU_STATUS_ERR_PROG;
                }
            }
        }
    }
    else
    {
        stm32wb_boot_memcpy(&aes128_data[0], (const uint8_t*)STM32WB_BOOT_FWU_APPLICATION_BASE, 64);

        ((stm32wb_application_vectors_t*)&aes128_data[0])->length = 0;

        if (!stm32wb_boot_flash_program(STM32WB_APPLICATION_BASE, (const uint8_t*)&aes128_data[0], 64))
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_PROG;
        }
        
        if (!stm32wb_boot_flash_program(STM32WB_APPLICATION_BASE + 64, (const uint8_t*)(STM32WB_BOOT_FWU_APPLICATION_BASE + 64), (candidate_size - 64)))
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_PROG;
        }
    }

    return STM32WB_BOOT_FWU_STATUS_NO_ERROR;
}

/************************************************************************************************************************************/

#define stm32wb_ipcc_device            stm32wb_boot_ipcc_device

#define stm32wb_ipcc_sys_enable        stm32wb_boot_ipcc_sys_enable
#define stm32wb_ipcc_sys_disable       stm32wb_boot_ipcc_sys_disable
#define stm32wb_ipcc_sys_state         stm32wb_boot_ipcc_sys_state
#define stm32wb_ipcc_sys_info          stm32wb_boot_ipcc_sys_info
#define stm32wb_ipcc_sys_command       stm32wb_boot_ipcc_sys_command

#define stm32wb_ipcc_fus_state         stm32wb_boot_ipcc_fus_state
#define stm32wb_ipcc_fus_command       stm32wb_boot_ipcc_fus_command

#define MB_QueueInit                   stm32wb_boot_ipcc_queue_init
#define MB_QueueIsEmpty                stm32wb_boot_ipcc_queue_is_empty
#define MB_QueueInsert                 stm32wb_boot_ipcc_queue_insert
#define MB_QueueRemove                 stm32wb_boot_ipcc_queue_remove
#define MB_QueueCopy                   stm32wb_boot_ipcc_queue_copy

#include "stm32wb_ipcc.c"

/************************************************************************************************************************************/

static const stm32wb_ipcc_fus_image_t * stm32wb_boot_fus_image(uint32_t base, uint32_t limit)
{
    const stm32wb_ipcc_fus_signature_t *signature;
    uint32_t magic;

    signature = (const stm32wb_ipcc_fus_signature_t*)(limit - sizeof(stm32wb_ipcc_fus_signature_t));
        
    while (base < (uint32_t)signature)
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
                return ((const stm32wb_ipcc_fus_image_t*)((const uint8_t*)signature - (signature->MemorySize & 0x000000ff) - sizeof(stm32wb_ipcc_fus_image_t)));
            }
        }

        signature = (const stm32wb_ipcc_fus_signature_t*)((const uint8_t*)signature - 4);
    }

    return NULL;
}

static bool stm32wb_boot_fus_state(stm32wb_ipcc_fus_state_t *p_state_return)
{
    stm32wb_ipcc_sys_command_t command;
    
    command.opcode = STM32WB_IPCC_SYS_OPCODE_FUS_GET_STATE;
    command.cparam = NULL;
    command.clen = 0;
    command.rparam = (void*)p_state_return;
    command.rsize = sizeof(stm32wb_ipcc_fus_state_t);

    stm32wb_ipcc_sys_command(&command);

    return (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS);
}

static bool stm32wb_boot_fus_command(uint32_t opcode)
{
    stm32wb_ipcc_sys_command_t command;
    
    command.opcode = opcode;
    command.cparam = NULL;
    command.clen = 0;
    command.rparam = NULL;
    command.rsize = 0;

    stm32wb_ipcc_sys_command(&command);

    return (command.status == STM32WB_IPCC_SYS_COMMAND_STATUS_SUCCESS);
}

static uint32_t stm32wb_boot_fwu_wireless(void)
{
    const stm32wb_ipcc_fus_image_t *fus_image;
    stm32wb_ipcc_fus_state_t fus_state;
    stm32wb_ipcc_sys_info_t sys_info;
    uint32_t sys_state, fwu_base, fwu_limit, fus_address, fus_base, fus_limit, data[2];

    fwu_base  = ((const uint32_t*)STM32WB_BOOT_FWU_REQUEST_BASE)[1];
    fwu_limit = ((const uint32_t*)STM32WB_BOOT_FWU_CANDIDATE_BASE)[1];

    if ((fwu_base > fwu_limit) || (fwu_base < STM32WB_BOOT_LIMIT) || (fwu_limit > STM32WB_BOOT_FWU_LIMIT))
    {
        return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
    }
    
    stm32wb_boot_ipcc_sys_enable();

    if (!stm32wb_boot_fus_state(&fus_state))
    {
        return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
    }
    
    if (!stm32wb_boot_ipcc_sys_info(&sys_info))
    {
        return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
    }

    sys_state = STM32WB_IPCC_SYS_STATE_FUS;
    
    do
    {
        if (fus_state.State == STM32WB_IPCC_FUS_STATE_ERROR)
        {
            if (fus_state.ErrorCode == STM32WB_IPCC_FUS_ERROR_CODE_NOT_RUNNING)
            {
                sys_state = STM32WB_IPCC_SYS_STATE_WIRELESS;
                
                break;
            }

            return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
        }

        if (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE)
        {
            stm32wb_boot_udelay(100000);
            
            if (!stm32wb_boot_fus_state(&fus_state))
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
            }
        }
    }
    while (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE);
    
    if (((volatile uint32_t*)STM32WB_BOOT_FWU_STATE_BASE)[0] != STM32WB_BOOT_FWU_STATE_UPGRADE)
    {
        fus_image = stm32wb_boot_fus_image(fwu_base, fwu_limit);

        if (!fus_image)
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_FILE;
        }

        fus_limit = 0x08000000 + (FLASH->SFR & FLASH_SFR_SFSA) * STM32WB_FLASH_PAGE_SIZE;
        
        if (fus_image->Magic == STM32WB_IPCC_FUS_MAGIC_FUS_IMAGE)
        {
            if ((fus_image->Version & 0xffffff00) < (sys_info.FusVersion & 0xffffff00))
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_FILE;
            }

            fus_base = 0x080ec000;
        }
        else
        {
            if (fus_image->Info1 == 0x5afeb007)
            {
                if ((fus_image->Version & 0xffffff00) < (sys_info.SafeBootVersion & 0xffffff00))
                {
                    return STM32WB_BOOT_FWU_STATUS_ERR_FILE;
                }

                fus_base = 0x080f0000;
            }
            else
            {
                fus_base = fus_limit - (fus_image->MemorySize & 0x000000ff) * STM32WB_FLASH_PAGE_SIZE;
            }
        }

        if (sys_state == STM32WB_IPCC_SYS_STATE_WIRELESS)
        {
            /* Reboot into FUS.
             */
            
            if (!stm32wb_boot_fus_state(&fus_state))
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
            }
            
            while (1)
            {
            }
        }
        
        if (sys_info.WirelessStackMemorySize)
        {
            if (!stm32wb_boot_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_DELETE))
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
            }

            do
            {
                stm32wb_boot_udelay(100000);
                
                if (!stm32wb_ipcc_fus_state(&fus_state))
                {
                    return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
                }
                
                if (fus_state.State == STM32WB_IPCC_FUS_STATE_ERROR)
                {
                    return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
                }
            }
            while (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE);

            /* If we get here throu an idle-state, CM0+ for some reason did not reset.
             * Hence use CM4 to do so.
             */
            stm32wb_boot_reset();
        }

        /* FUS does something odd with erased pages, so that a simple blank detection will
         * not work. Just brute force erase the whole area instead of trying to be smart.
         */
        for (fus_address = fus_base; fus_address < fus_limit; fus_address += STM32WB_FLASH_PAGE_SIZE)
        {
            if (!stm32wb_boot_flash_erase(fus_address))
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_ERASE;
            }
        }
        
        if (!stm32wb_boot_flash_program(fus_base, (const uint8_t *)STM32WB_BOOT_FWU_WIRELESS_BASE, (fus_limit - fus_base)))
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_PROG;
        }
        
        /* Ok, all the prepwork had been done. The image has been verified (sort of), FUS is booted, and the
         * previous wireless stack has been deleted.
         */
        data[0] = STM32WB_BOOT_FWU_STATE_UPGRADE;
        data[1] = fus_base;

        if (!stm32wb_boot_flash_program(STM32WB_BOOT_FWU_STATE_BASE, (const uint8_t*)&data[0], 8))
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_PROG;
        }

        if (!stm32wb_boot_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_FW_UPGRADE))
        {
            return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
        }

        do
        {
            stm32wb_boot_udelay(100000);

            if (!stm32wb_ipcc_fus_state(&fus_state))
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
            }
            
            if (fus_state.State == STM32WB_IPCC_FUS_STATE_ERROR)
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
            }
        }
        while (fus_state.State != STM32WB_IPCC_FUS_STATE_IDLE);

        /* If we get here throu an idle-state, CM0+ for some reason did not reset.
         * Hence use CM4 to do so.
         */
        stm32wb_boot_reset();
    }

    if (sys_info.WirelessStackMemorySize)
    {
        if (sys_state != STM32WB_IPCC_SYS_STATE_WIRELESS)
        {
            if(!stm32wb_boot_fus_command(STM32WB_IPCC_SYS_OPCODE_FUS_START_WS))
            {
                return STM32WB_BOOT_FWU_STATUS_ERR_INTERNAL;
            }
        }
    }

    return STM32WB_BOOT_FWU_STATUS_NO_ERROR;
}

/************************************************************************************************************************************/

static uint32_t stm32wb_boot_fwu(void)
{
    uint32_t status, data[2];
        
    status = STM32WB_BOOT_FWU_STATUS_NO_ERROR;
    
    if (((const uint32_t*)STM32WB_BOOT_FWU_REQUEST_BASE)[0] == STM32WB_BOOT_FWU_REQUEST_INSTALL_APPLICATION)
    {
        status = stm32wb_boot_fwu_application();
    }
    
    if (((const uint32_t*)STM32WB_BOOT_FWU_REQUEST_BASE)[0] == STM32WB_BOOT_FWU_REQUEST_INSTALL_WIRELESS)
    {
        status = stm32wb_boot_fwu_wireless();
    }
    
    data[0] = status;
    data[1] = 0xdeadbeef;
    
    stm32wb_boot_flash_program(STM32WB_BOOT_FWU_STATUS_BASE, (const uint8_t*)&data[0], 8);
    
    stm32wb_boot_flash_clean(STM32WB_BOOT_FWU_BASE + STM32WB_FLASH_PAGE_SIZE, STM32WB_BOOT_FWU_LIMIT);

    return status;
}

/************************************************************************************************************************************/

#define stm32wb_usbd_dcd_device        stm32wb_boot_usbd_dcd_device

#define stm32wb_usbd_dcd_address       stm32wb_boot_usbd_dcd_address
#define stm32wb_usbd_dcd_configure     stm32wb_boot_usbd_dcd_configure
#define stm32wb_usbd_dcd_connect       stm32wb_boot_usbd_dcd_connect
#define stm32wb_usbd_dcd_disable       stm32wb_boot_usbd_dcd_disable
#define stm32wb_usbd_dcd_disconnect    stm32wb_boot_usbd_dcd_disconnect
#define stm32wb_usbd_dcd_enable        stm32wb_boot_usbd_dcd_enable
#define stm32wb_usbd_dcd_event         stm32wb_boot_usbd_dcd_event
#define stm32wb_usbd_dcd_ep0_count     stm32wb_boot_usbd_dcd_ep0_count
#define stm32wb_usbd_dcd_ep0_receive   stm32wb_boot_usbd_dcd_ep0_receive
#define stm32wb_usbd_dcd_ep0_stall     stm32wb_boot_usbd_dcd_ep0_stall
#define stm32wb_usbd_dcd_ep0_transmit  stm32wb_boot_usbd_dcd_ep0_transmit
#define stm32wb_usbd_dcd_pma_read      stm32wb_boot_usbd_dcd_pma_read
#define stm32wb_usbd_dcd_pma_write     stm32wb_boot_usbd_dcd_pma_write
#define stm32wb_usbd_dcd_reset         stm32wb_boot_usbd_dcd_reset

#define stm32wb_usbd_dcd_usb_interrupt stm32wb_boot_usbd_dcd_usb_interrupt
#define stm32wb_usbd_dcd_crs_interrupt stm32wb_boot_usbd_dcd_crs_interrupt

#include "stm32wb_usbd_dcd.c"

#define stm32wb_usbd_control           stm32wb_boot_usbd_control

#define stm32wb_usbd_bos               stm32wb_boot_usbd_bos
#define stm32wb_usbd_configuration     stm32wb_boot_usbd_configuration
#define stm32wb_usbd_configure         stm32wb_boot_usbd_configure
#define stm32wb_usbd_connect           stm32wb_boot_usbd_connect
#define stm32wb_usbd_device            stm32wb_boot_usbd_device
#define stm32wb_usbd_disable           stm32wb_boot_usbd_disable
#define stm32wb_usbd_disconnect        stm32wb_boot_usbd_disconnect
#define stm32wb_usbd_enable            stm32wb_boot_usbd_enable
#define stm32wb_usbd_event_callback    stm32wb_boot_usbd_event_callback
#define stm32wb_usbd_is_suspended      stm32wb_boot_usbd_is_suspended
#define stm32wb_usbd_language          stm32wb_boot_usbd_language
#define stm32wb_usbd_msos20            stm32wb_boot_usbd_msos20
#define stm32wb_usbd_request           stm32wb_boot_usbd_request
#define stm32wb_usbd_serial            stm32wb_boot_usbd_serial
#define stm32wb_usbd_set_address       stm32wb_boot_usbd_set_address
#define stm32wb_usbd_start             stm32wb_boot_usbd_start
#define stm32wb_usbd_state             stm32wb_boot_usbd_state
#define stm32wb_usbd_stop              stm32wb_boot_usbd_stop
#define stm32wb_usbd_string            stm32wb_boot_usbd_string
#define __svc_stm32wb_usbd_disable     __svc_stm32wb_boot_usbd_disable
#define __svc_stm32wb_usbd_enable      __svc_stm32wb_boot_usbd_enable
#define __svc_stm32wb_usbd_start       __svc_stm32wb_boot_usbd_start
#define __svc_stm32wb_usbd_stop        __svc_stm32wb_boot_usbd_stop

#include "stm32wb_usbd.c"

#define STM32WB_BOOT_DFU_CONFIGURATION_SIZE         (9 + (9+9+9))
#define STM32WB_BOOT_DFU_STRING_COUNT               3
#define STM32WB_BOOT_DFU_CLASS_COUNT                1
#define STM32WB_BOOT_DFU_INTERFACE_COUNT            1

static void stm32wb_boot_dfu_configure(void *context, uint8_t interface);
static void stm32wb_boot_dfu_start(void *context);
static void stm32wb_boot_dfu_stop(void *context);
static int stm32wb_boot_dfu_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return);
static void stm32wb_boot_dfu_suspend(void *context);
static void stm32wb_boot_dfu_resume(void *context);

static const stm32wb_usbd_class_t stm32wb_boot_dfu_class = {
    stm32wb_boot_dfu_configure,
    stm32wb_boot_dfu_start,
    stm32wb_boot_dfu_stop,
    stm32wb_boot_dfu_request,
    stm32wb_boot_dfu_suspend,
    stm32wb_boot_dfu_resume,
    NULL,
    STM32WB_BOOT_DFU_INTERFACE_COUNT,
    0,
};

static const uint8_t stm32wb_boot_dfu_configuration[STM32WB_BOOT_DFU_CONFIGURATION_SIZE] =
{
    /**** Configuration Descriptor ****/
    9,                                                           /* bLength */
    USB_DESCRIPTOR_TYPE_CONFIGURATION,                           /* bDescriptorType */
    STM32WB_BOOT_DFU_CONFIGURATION_SIZE & 255,                   /* wTotalLength */
    STM32WB_BOOT_DFU_CONFIGURATION_SIZE >> 8,
    1,                                                           /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x02,                                                        /* iConfiguration */
    0x80,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** DFU Interface (Application) ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x00,                                                        /* bNumEndpoints */
    0xfe,                                                        /* bInterfaceClass */
    0x01,                                                        /* bInterfaceSubClass */
    0x02,                                                        /* nInterfaceProtocol */
    0x04,                                                        /* iInterface */

    /**** DFU Interface (Wireless Stack) ****/
    9,                                                           /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x01,                                                        /* bAlternateSetting */
    0x00,                                                        /* bNumEndpoints */
    0xfe,                                                        /* bInterfaceClass */
    0x01,                                                        /* bInterfaceSubClass */
    0x02,                                                        /* nInterfaceProtocol */
    0x05,                                                        /* iInterface */
    
    /**** DFU Descriptor ****/
    0x09,                                                        /* blength = 9 Bytes */
    0x21,                                                        /* bDescriptorType */
    0x0d,                                                        /* bmAttribute */
    0xff, 0xff,                                                  /* wDetachTimeOut= 255 ms */
    0x00, 0x04,                                                  /* wTransferSize = 1024 Bytes */
    0x10, 0x01,                                                  /* bcdDFUVersion */
};

static const char * const stm32wb_boot_dfu_strings[STM32WB_BOOT_DFU_STRING_COUNT] =
{
    "STM32WB Application",                    // 4
    "STM32WB Wireless Stack",                 // 5
};

static const stm32wb_usbd_class_t * const stm32wb_boot_dfu_classes[STM32WB_BOOT_DFU_CLASS_COUNT] =
{
    &stm32wb_boot_dfu_class,
};

static const uint8_t stm32wb_boot_dfu_bos[] = {
    // BOS
    0x05,                                            // bLength
    USB_DESCRIPTOR_TYPE_BOS,                         // bDescriptorType
    0x21, 0x00,                                      // wTotalLength
    0x01,                                            // bNumDeviceCaps

    // Microsoft OS 2.0 platform capability
    0x1c,                                            // bLength
    USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,           // bDescriptorType
    USB_DEVICE_CAPABILITY_TYPE_PLATFORM,             // bDevCapabilityType
    0x00,                                            // bReserved
    0xdf, 0x60, 0xdd, 0xd8, 0x89, 0x45, 0xc7, 0x4c,  // platformCapabilityUUID[16]     
    0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f,  // platformCapabilityUUID[16]     
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wLength (MSOS20)
    USB_REQ_MS_VENDOR_CODE,                          // bMS_VendorCode
    0x00,                                            // bAltEnumCode
};

static const uint8_t stm32wb_boot_dfu_msos20[] = {
    // Microsoft OS 2.0 descriptor set header
    0x0a, 0x00,                                      // wLength
    0x00, 0x00,                                      // wDescriptorType
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion (Windows 8.1)
    0xb2, 0x00,                                      // wTotalLength

    // Microsoft OS 2.0 configuration subset header
    0x08, 0x00,                                      // wLength
    0x01, 0x00,                                      // wDescriptorType
    0x00,                                            // bConfigurationValue
    0x00,                                            // bReserved
    0xa8, 0x00,                                      // wTotalLength (size of entire configuration subset)

    // Microsoft OS 2.0 function subset header
    0x08, 0x00,                                      // wLength
    0x02, 0x00,                                      // wDescriptorType
    0x00,                                            // bFirstInterface
    0x00,                                            // bReserved
    0xa0, 0x00,                                      // wTotalLength  (size of entine function subset)

    // Microsoft OS 2.0 compatible ID
    0x14, 0x00,                                      // wLength
    0x03, 0x00,                                      // wDescriptorType
    'W',  'I',  'N',  'U',  'S',  'B',  0x00, 0x00,  // compatible ID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID
                                        
    // Microsoft OS 2.0 registry property
    0x84, 0x00,                                      // wLength
    0x04, 0x00,                                      // wDescriptorType
    0x07, 0x00,                                      // wPropertyDataType
    0x2a, 0x00,                                      // wPropertyNameLength
    'D',  0x00, 'e',  0x00, 'v',  0x00, 'i',  0x00,  // bPropertyName "DeviceInterfaceGUIDs"
    'c',  0x00, 'e',  0x00, 'I',  0x00, 'n',  0x00,    
    't',  0x00, 'e',  0x00, 'r',  0x00, 'f',  0x00,    
    'a',  0x00, 'c',  0x00, 'e',  0x00, 'G',  0x00,    
    'U',  0x00, 'I',  0x00, 'D',  0x00, 's',  0x00,        
    0x00, 0x00,
    0x50, 0x00,                                      // wPropertyDataLength
    '{',  0x00, 'D',  0x00, 'E',  0x00, '5',  0x00,  // bProperyData "{DE50DD7B-4DEF-4F9F-98AA-88144B16A383}"
    '0',  0x00, 'D',  0x00, 'D',  0x00, '7',  0x00,
    'B',  0x00, '-',  0x00, '4',  0x00, 'D',  0x00,
    'E',  0x00, 'F',  0x00, '-',  0x00, '4',  0x00,
    'F',  0x00, '9',  0x00, 'F',  0x00, '-',  0x00,
    '9',  0x00, '8',  0x00, 'A',  0x00, 'A',  0x00,
    '-',  0x00, '8',  0x00, '8',  0x00, '1',  0x00,
    '4',  0x00, '4',  0x00, 'B',  0x00, '1',  0x00,
    '6',  0x00, 'A',  0x00, '3',  0x00, '8',  0x00,
    '3',  0x00, '}',  0x00,
    0x00, 0x00, 0x00, 0x00,
};

static uint8_t stm32wb_boot_dfu_ep0_data[1024];

static const stm32wb_usbd_device_t stm32wb_boot_dfu_device =
{
    0x0483,
    0xdf11,
    0x0200,
    "Tlera Corporation",
    "STM32WB in DFU Mode",
};

static const stm32wb_usbd_params_t stm32wb_boot_dfu_params =
{
    STM32WB_GPIO_PIN_NONE,
};

const __attribute__((used)) stm32wb_usbd_info_t stm32wb_boot_dfu_info =
{
    stm32wb_boot_dfu_ep0_data,
    sizeof(stm32wb_boot_dfu_ep0_data),
    STM32WB_BOOT_DFU_STRING_COUNT,
    STM32WB_BOOT_DFU_CLASS_COUNT,
    stm32wb_boot_dfu_configuration,
    stm32wb_boot_dfu_bos,
    stm32wb_boot_dfu_msos20,
    stm32wb_boot_dfu_strings,
    stm32wb_boot_dfu_classes,
};

#define DFU_ALTSETTING_APPLICATION       0
#define DFU_ALTSETTING_WIRELESS          1

#define DFU_REQUEST_DETACH               0x00
#define DFU_REQUEST_DNLOAD               0x01
#define DFU_REQUEST_UPLOAD               0x02
#define DFU_REQUEST_GETSTATUS            0x03
#define DFU_REQUEST_CLRSTATUS            0x04
#define DFU_REQUEST_GETSTATE             0x05
#define DFU_REQUEST_ABORT                0x06

#define DFU_STATUS_OK                    0x00
#define DFU_STATUS_ERR_TARGET            0x01
#define DFU_STATUS_ERR_FILE              0x02
#define DFU_STATUS_ERR_WRITE             0x03
#define DFU_STATUS_ERR_ERASE             0x04
#define DFU_STATUS_ERR_CHECK_ERASED      0x05
#define DFU_STATUS_ERR_PROG              0x06
#define DFU_STATUS_ERR_VERIFY            0x07
#define DFU_STATUS_ERR_ADDRESS           0x08
#define DFU_STATUS_ERR_NOTDONE           0x09
#define DFU_STATUS_ERR_FIRMWARE          0x0a
#define DFU_STATUS_ERR_VENDOR            0x0b
#define DFU_STATUS_ERR_USBR              0x0c
#define DFU_STATUS_ERR_POR               0x0d
#define DFU_STATUS_ERR_UNKNOWN           0x0e
#define DFU_STATUS_ERR_STALLEDPKT        0x0f

#define DFU_STATE_APP_IDLE               0
#define DFU_STATE_APP_DETACH             1
#define DFU_STATE_IDLE                   2
#define DFU_STATE_DNLOAD_SYNC            3
#define DFU_STATE_DNBUSY                 4
#define DFU_STATE_DNLOAD_IDLE            5
#define DFU_STATE_MANIFEST_SYNC          6
#define DFU_STATE_MANIFEST               7
#define DFU_STATE_MANIFEST_WAIT_RESET    8
#define DFU_STATE_UPLOAD_IDLE            9
#define DFU_STATE_ERROR                  10

typedef struct _stm32wb_boot_dfu_control_t {
    uint8_t                 interface;
    uint8_t                 altsetting;
    volatile uint8_t        status;
    volatile uint8_t        state;

    volatile uint32_t       events;
  
    uint32_t                program_address;
    uint32_t                boot_size;
    uint32_t                application_size;
    uint32_t                image_size;
    uint32_t                image_count;
    uint8_t                 image_data[2048];
} stm32wb_boot_dfu_control_t;

static stm32wb_boot_dfu_control_t stm32wb_boot_dfu_control;

#define STM32WB_DFU_FLASH_PROGRAM_SIZE 1024
#define STM32WB_DFU_FLASH_ERASE_TIME   24
#define STM32WB_DFU_FLASH_PROGRAM_TIME 12

#define STM32WB_DFU_EVENT_START        0x00000001
#define STM32WB_DFU_EVENT_WRITE        0x00000002
#define STM32WB_DFU_EVENT_FINISH       0x00000004
#define STM32WB_DFU_EVENT_CANCEL       0x00000008
#define STM32WB_DFU_EVENT_INSTALL      0x00000010
#define STM32WB_DFU_EVENT_DETACH       0x00000020

static bool stm32wb_boot_dfu_check_application(void)
{
    const stm32wb_image_info_t *image_info;
    uint32_t candidate_magic, candidate_base, candidate_size, candidate_length, image_offset;

    candidate_magic = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->magic;
    candidate_base = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->base;
    candidate_size = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->size;
    candidate_length = ((const stm32wb_application_vectors_t*)STM32WB_BOOT_FWU_APPLICATION_BASE)->length & STM32WB_IMAGE_LENGTH_MASK;
    
    if ((candidate_magic != STM32WB_APPLICATION_MAGIC) || (candidate_base != STM32WB_APPLICATION_BASE) || (candidate_size > STM32WB_APPLICATION_SIZE))
    {
        return false;
    }

    if ((candidate_length > STM32WB_BOOT_FWU_APPLICATION_SIZE) || (candidate_length & 7))
    {
        return false;
    }

    if (candidate_length)
    {
        image_offset = candidate_length - sizeof(stm32wb_image_info_t);
        image_info = (const stm32wb_image_info_t*)(STM32WB_BOOT_FWU_APPLICATION_BASE + image_offset);
        
        if (image_info->crc32 != stm32wb_boot_crc32((const uint8_t*)STM32WB_BOOT_FWU_APPLICATION_BASE, ((uint32_t)&image_info->crc32 - STM32WB_BOOT_FWU_APPLICATION_BASE), 0xffffffff))
        {
            return false;
        }
    }
    
    return true;
}

static bool stm32wb_boot_dfu_check_wireless(void)
{
    return (stm32wb_boot_fus_image(STM32WB_BOOT_FWU_WIRELESS_BASE, STM32WB_BOOT_FWU_WIRELESS_LIMIT) != NULL);
}

static void stm32wb_boot_dfu_detach()
{
    __armv7m_atomic_or(&stm32wb_boot_dfu_control.events, STM32WB_DFU_EVENT_DETACH);
}

static void stm32wb_boot_dfu_dnload()
{
    uint32_t events;

    events = STM32WB_DFU_EVENT_WRITE;
    
    if (!stm32wb_boot_dfu_control.program_address)
    {
        stm32wb_boot_dfu_control.program_address = (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION) ? STM32WB_BOOT_FWU_APPLICATION_BASE : STM32WB_BOOT_FWU_WIRELESS_BASE;

        events |= STM32WB_DFU_EVENT_START;
    }

    __armv7m_atomic_or(&stm32wb_boot_dfu_control.events, events);
}

static void stm32wb_boot_dfu_manifest()
{
    uint32_t events;

    events = STM32WB_DFU_EVENT_FINISH | STM32WB_DFU_EVENT_INSTALL;

    if (stm32wb_boot_dfu_control.image_count)
    {
        events |= STM32WB_DFU_EVENT_WRITE;
    }

    __armv7m_atomic_or(&stm32wb_boot_dfu_control.events, events);
}

static void stm32wb_boot_dfu_configure(void *context, uint8_t interface)
{
    stm32wb_boot_dfu_control.interface = interface;
}

static void stm32wb_boot_dfu_start(void *context)
{
    stm32wb_boot_dfu_control.altsetting = DFU_ALTSETTING_APPLICATION;
}

static void stm32wb_boot_dfu_stop(void *context)
{
}

static int stm32wb_boot_dfu_request(void *context, int state, const stm32wb_usbd_request_t *request, uint8_t *data, const uint8_t **p_data_return, uint16_t *p_length_return, stm32wb_usbd_status_routine_t *p_status_routine_return)
{
    uint16_t interface;
    int status, dfu_state, dfu_status, dfu_timeout;
    uint32_t application_size, application_length, image_offset;

    status = STM32WB_USBD_REQUEST_STATUS_UNHANDLED;

    dfu_state = DFU_STATE_ERROR;
    dfu_status = DFU_STATUS_ERR_STALLEDPKT;
    
    switch (request->bmRequestType & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE: {
	interface = request->wIndex;
	
	if (interface == stm32wb_boot_dfu_control.interface)
	{
	    switch (request->bmRequestType & USB_REQ_TYPE_MASK) {
            case USB_REQ_TYPE_STANDARD: {
                switch (request->bRequest) {
                case USB_REQ_CODE_GET_INTERFACE: {
                    *p_data_return = data;
                    *p_length_return = 1;
                        
                    data[0] = stm32wb_boot_dfu_control.altsetting;
                    
                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                            
                case USB_REQ_CODE_SET_INTERFACE: {
                    if ((request->wValue == 0) || (request->wValue == 1))
                    {
                        stm32wb_boot_dfu_control.altsetting = request->wValue;

                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    }
                    break;
                }

                default:
                    break;
                }
                break;
            }
                
	    case USB_REQ_TYPE_CLASS: {
		switch (request->bRequest) {
                    
                case DFU_REQUEST_DETACH: {
                    *p_status_routine_return = stm32wb_boot_dfu_detach;
                        
                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                    
                case DFU_REQUEST_DNLOAD: {     
                    if ((stm32wb_boot_dfu_control.state == DFU_STATE_IDLE) || (stm32wb_boot_dfu_control.state == DFU_STATE_DNLOAD_IDLE))
                    {
                        if (stm32wb_boot_dfu_control.state == DFU_STATE_IDLE)
                        {
                            stm32wb_boot_dfu_control.program_address = 0;
                            stm32wb_boot_dfu_control.boot_size = 0;
                            stm32wb_boot_dfu_control.application_size = 0;
                            stm32wb_boot_dfu_control.image_size = 0;
                            stm32wb_boot_dfu_control.image_count = 0;
                        }

                        if (request->wLength)
                        {
                            stm32wb_boot_memcpy(&stm32wb_boot_dfu_control.image_data[stm32wb_boot_dfu_control.image_count], data, request->wLength);
                                
                            stm32wb_boot_dfu_control.image_size += request->wLength;
                            stm32wb_boot_dfu_control.image_count += request->wLength;

                            if (stm32wb_boot_dfu_control.image_count >= STM32WB_DFU_FLASH_PROGRAM_SIZE)
                            {
                                if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                                {
                                    image_offset = stm32wb_boot_dfu_control.image_size - stm32wb_boot_dfu_control.image_count;
                                        
                                    if (image_offset == 0)
                                    {
                                        if ((((const stm32wb_boot_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->magic == STM32WB_BOOT_MAGIC) &&
                                            (((const stm32wb_boot_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->base == STM32WB_BOOT_BASE) &&
                                            (((const stm32wb_boot_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->size == STM32WB_BOOT_SIZE) &&
                                            (((const stm32wb_boot_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->length == 0))
                                        {
                                            stm32wb_boot_dfu_control.boot_size = STM32WB_BOOT_SIZE;
                                        }
                                            
                                        if ((((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->magic == STM32WB_APPLICATION_MAGIC) &&
                                            (((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->base == STM32WB_APPLICATION_BASE))
                                        {
                                            application_size = ((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->size;
                                            application_length = ((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->length & STM32WB_IMAGE_LENGTH_MASK; 
                                                
                                            if ((application_size <= STM32WB_APPLICATION_SIZE) && !(application_size & 7) && (application_length <= STM32WB_BOOT_FWU_APPLICATION_SIZE) && !(application_length & 7))
                                            {
                                                stm32wb_boot_dfu_control.application_size = application_length ? application_length : application_size;
                                            }
                                        }
                                            
                                        if ((stm32wb_boot_dfu_control.boot_size == 0) && (stm32wb_boot_dfu_control.application_size == 0))
                                        {
                                            dfu_state = DFU_STATE_ERROR;
                                            dfu_status = DFU_STATUS_ERR_FILE;
                                                
                                            status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                        }
                                    }
                                        
                                    if ((image_offset == stm32wb_boot_dfu_control.boot_size) && (stm32wb_boot_dfu_control.application_size == 0))
                                    {
                                        if ((((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->magic == STM32WB_APPLICATION_MAGIC) &&
                                            (((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->base == STM32WB_APPLICATION_BASE))
                                        {
                                            application_size = ((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->size;
                                            application_length = ((const stm32wb_application_vectors_t*)&stm32wb_boot_dfu_control.image_data[0])->length & STM32WB_IMAGE_LENGTH_MASK;
                                                
                                            if ((application_size <= STM32WB_APPLICATION_SIZE) && !(application_size & 7) && (application_length <= STM32WB_BOOT_FWU_APPLICATION_SIZE) && !(application_length & 7))
                                            {
                                                stm32wb_boot_dfu_control.application_size = application_length ? application_length : application_size;
                                            }
                                        }
                                            
                                        if (stm32wb_boot_dfu_control.application_size == 0)
                                        {
                                            dfu_state = DFU_STATE_ERROR;
                                            dfu_status = DFU_STATUS_ERR_FILE;
                                                
                                            status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                        }
                                    }
                                        
                                    if (status != STM32WB_USBD_REQUEST_STATUS_FAILURE)
                                    {
                                        if (stm32wb_boot_dfu_control.image_size > (stm32wb_boot_dfu_control.boot_size + stm32wb_boot_dfu_control.application_size))
                                        {
                                            dfu_state = DFU_STATE_ERROR;
                                            dfu_status = DFU_STATUS_ERR_ADDRESS;
                                                
                                            status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                        }
                                    }
                                        
                                    if (status != STM32WB_USBD_REQUEST_STATUS_FAILURE)
                                    {
                                        if (stm32wb_boot_dfu_control.boot_size > image_offset)
                                        {
                                            stm32wb_boot_dfu_control.image_count -= STM32WB_DFU_FLASH_PROGRAM_SIZE;
                                                
                                            if (stm32wb_boot_dfu_control.image_count)
                                            {
                                                stm32wb_boot_memcpy(&stm32wb_boot_dfu_control.image_data[0],
                                                                    &stm32wb_boot_dfu_control.image_data[STM32WB_DFU_FLASH_PROGRAM_SIZE],
                                                                    stm32wb_boot_dfu_control.image_count);
                                            }
                                                
                                            dfu_state = DFU_STATE_DNLOAD_IDLE;
                                            dfu_status = DFU_STATUS_OK;
                                        }
                                        else
                                        {
                                            dfu_state = DFU_STATE_DNLOAD_SYNC;
                                            dfu_status = DFU_STATUS_OK;
                                        }
                                            
                                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                                    }
                                }
                                else
                                {
                                    if (stm32wb_boot_dfu_control.image_size > STM32WB_BOOT_FWU_WIRELESS_SIZE)
                                    {
                                        dfu_state = DFU_STATE_ERROR;
                                        dfu_status = DFU_STATUS_ERR_ADDRESS;
                                        
                                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                    }
                                    else
                                    {
                                        dfu_state = DFU_STATE_DNLOAD_SYNC;
                                        dfu_status = DFU_STATUS_OK;
                                        
                                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                                    }
                                }
                            }
                            else
                            {
                                dfu_state = DFU_STATE_DNLOAD_IDLE;
                                dfu_status = DFU_STATUS_OK;
                                        
                                status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                            }
                        }
                        else
                        {
                            if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                            {
                                if (stm32wb_boot_dfu_control.application_size == 0)
                                {
                                    dfu_state = DFU_STATE_ERROR;
                                    dfu_status = DFU_STATUS_ERR_FILE;
                                    
                                    status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                }
                                else
                                {
                                    if (stm32wb_boot_dfu_control.image_size != (stm32wb_boot_dfu_control.boot_size + stm32wb_boot_dfu_control.application_size))
                                    {
                                        dfu_state = DFU_STATE_ERROR;
                                        dfu_status = DFU_STATUS_ERR_NOTDONE;
                                        
                                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                                    }
                                }
                            }
                                
                            if (status != STM32WB_USBD_REQUEST_STATUS_FAILURE)
                            {
                                dfu_state = DFU_STATE_MANIFEST_SYNC;
                                dfu_status = DFU_STATUS_OK;
                                    
                                status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                            }
                        }
                    }
                    else
                    {
                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    }
                    break;
                }

                case DFU_REQUEST_UPLOAD: {     
                    status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    break;
                }
                    
                case DFU_REQUEST_GETSTATUS: {
                    dfu_state = stm32wb_boot_dfu_control.state;
                    dfu_status = stm32wb_boot_dfu_control.status;
                    dfu_timeout = 0;
                    
                    *p_data_return = data;
                    *p_length_return = 6;

                    if (dfu_state == DFU_STATE_DNBUSY)
                    {
                        dfu_timeout = 6;
                    }

                    if (dfu_state == DFU_STATE_DNLOAD_SYNC)
                    {
                        *p_status_routine_return = stm32wb_boot_dfu_dnload;

                        dfu_state = DFU_STATE_DNBUSY;

                        dfu_timeout += STM32WB_DFU_FLASH_PROGRAM_TIME;
                        
                        if (!stm32wb_boot_dfu_control.program_address)
                        {
                            dfu_timeout += (STM32WB_DFU_FLASH_ERASE_TIME * stm32wb_boot_flash_check(STM32WB_BOOT_FWU_BASE, STM32WB_BOOT_FWU_LIMIT));
                        }
                    }

                    if (dfu_state == DFU_STATE_MANIFEST)
                    {
                        dfu_timeout = 200;
                    }

                    if (dfu_state == DFU_STATE_MANIFEST_SYNC)
                    {
                        *p_status_routine_return = stm32wb_boot_dfu_manifest;

                        dfu_state = DFU_STATE_MANIFEST;

                        dfu_timeout = 200;
                    }
                    
                    data[0] = dfu_status;
                    data[1] = dfu_timeout >> 0;
                    data[2] = dfu_timeout >> 8;
                    data[3] = dfu_timeout >> 16;
                    data[4] = dfu_state;
                    data[5] = 0;

                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                    
                case DFU_REQUEST_CLRSTATUS: {
                    if (stm32wb_boot_dfu_control.state == DFU_STATE_ERROR)
                    {
                        dfu_state = DFU_STATE_IDLE;
                        dfu_status = DFU_STATUS_OK;

                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    }
                    else
                    {
                        dfu_state = DFU_STATE_ERROR;
                        dfu_status = stm32wb_boot_dfu_control.status;
                        
                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    }
                    break;
                }
                    
                case DFU_REQUEST_GETSTATE: {
                    dfu_state = stm32wb_boot_dfu_control.state;
                    dfu_status = stm32wb_boot_dfu_control.status;

                    *p_data_return = data;
                    *p_length_return = 1;

                    data[0] = dfu_state;
                    
                    status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    break;
                }
                    
                case DFU_REQUEST_ABORT: {
                    if ((stm32wb_boot_dfu_control.state == DFU_STATE_IDLE) || (stm32wb_boot_dfu_control.state == DFU_STATE_DNLOAD_IDLE))
                    {
                        dfu_state = DFU_STATE_IDLE;
                        dfu_status = DFU_STATUS_OK;

                        status = STM32WB_USBD_REQUEST_STATUS_SUCCESS;
                    }
                    else
                    {
                        status = STM32WB_USBD_REQUEST_STATUS_FAILURE;
                    }
                    break;
                }
                    
		default:
		    break;
		}
		break;
	    }

	    default:
		break;
	    }
	}
	break;
    }
	
    default:
	break;
    }

    if (status != STM32WB_USBD_REQUEST_STATUS_UNHANDLED)
    {
        stm32wb_boot_dfu_control.state = dfu_state;
        stm32wb_boot_dfu_control.status = dfu_status;
    }
    
    return status;
}

static void stm32wb_boot_dfu_suspend(void *context)
{
}

static void stm32wb_boot_dfu_resume(void *context)
{
}

static void __attribute__((noreturn, noinline)) stm32wb_boot_dfu(bool firmware)
{
    uint32_t flash_acr;
    uint32_t events, count, data[8];
    uint8_t dfu_state, dfu_status;

    RTC->WPR = 0xca;
    RTC->WPR = 0x53;
                    
    RTC->BKP16R = ((RTC->BKP16R & ~(STM32WB_RTC_BKP16R_DFU)) | ((STM32WB_RTC_BKP16R_DFU) << STM32WB_RTC_BKP16R_NOT_DATA_SHIFT));
    
    RTC->WPR = 0x00;

    if (!(FLASH->OPTR & FLASH_OPTR_nSWBOOT0) || *((volatile uint32_t*)&stm32wb_boot_info.rsa2048_key.exponent))
    {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

        if (firmware)
        {
            stm32wb_boot_dfu_control.status = DFU_STATUS_OK;
            stm32wb_boot_dfu_control.state = DFU_STATE_IDLE;
        }
        else
        {
            stm32wb_boot_dfu_control.status = DFU_STATUS_ERR_FIRMWARE;
            stm32wb_boot_dfu_control.state = DFU_STATE_ERROR;
        }
        
        stm32wb_usbd_configure(&stm32wb_boot_dfu_device, &stm32wb_boot_dfu_info, &stm32wb_boot_dfu_params);

        stm32wb_usbd_enable(NULL, NULL);
        
        while (1)
        {
            __WFE();

            events = __armv7m_atomic_swap(&stm32wb_boot_dfu_control.events, 0);

            if (events)
            {
                dfu_state = stm32wb_boot_dfu_control.state;
                dfu_status = stm32wb_boot_dfu_control.status;
            
                if (events & STM32WB_DFU_EVENT_START)
                {
                    if (dfu_status == DFU_STATUS_OK)
                    {
                        if (!stm32wb_boot_flash_clean(STM32WB_BOOT_FWU_BASE, STM32WB_BOOT_FWU_LIMIT))
                        {
                            dfu_status = DFU_STATUS_ERR_ERASE;
                        }
                    }
                }
            
                if (events & STM32WB_DFU_EVENT_WRITE)
                {
                    if (dfu_status == DFU_STATUS_OK)
                    {
                        count = STM32WB_DFU_FLASH_PROGRAM_SIZE;

                        if (count > stm32wb_boot_dfu_control.image_count)
                        {
                            count = stm32wb_boot_dfu_control.image_count;
                        }

                        if (!stm32wb_boot_flash_program(stm32wb_boot_dfu_control.program_address, &stm32wb_boot_dfu_control.image_data[0], ((count + 7) & ~7)))
                        {
                            dfu_status = DFU_STATUS_ERR_PROG;
                        }
                        else
                        {
                            stm32wb_boot_dfu_control.program_address += count;

                            stm32wb_boot_dfu_control.image_count -= count;
                        
                            if (stm32wb_boot_dfu_control.image_count)
                            {
                                stm32wb_boot_memcpy(&stm32wb_boot_dfu_control.image_data[0],
                                                    &stm32wb_boot_dfu_control.image_data[count],
                                                    stm32wb_boot_dfu_control.image_count);
                            }

                            dfu_state = DFU_STATE_DNLOAD_IDLE;
                        }
                    }
                }
                
                if (events & STM32WB_DFU_EVENT_FINISH)
                {
                    if (dfu_status == DFU_STATUS_OK)
                    {
                        data[0] = STM32WB_BOOT_FWU_CANDIDATE_IMAGE;
                        data[1] = STM32WB_BOOT_FWU_LIMIT;

                        count = 8;
                        
                        if (!stm32wb_boot_flash_program(STM32WB_BOOT_FWU_CANDIDATE_BASE, (const uint8_t*)&data[0], count))
                        {
                            dfu_status = DFU_STATUS_ERR_PROG;
                        }
                    }
                }
            
                if (events & STM32WB_DFU_EVENT_CANCEL)
                {
                    data[0] = STM32WB_BOOT_FWU_REQUEST_CANCEL;
                    data[1] = 0x00000000;

                    count = 8;
                    
                    if (!stm32wb_boot_flash_program(STM32WB_BOOT_FWU_REQUEST_BASE, (const uint8_t*)&data[0], count))
                    {
                        dfu_status = DFU_STATUS_ERR_PROG;
                    }
                }

                if (events & STM32WB_DFU_EVENT_INSTALL)
                {
                    data[0] = STM32WB_BOOT_FWU_REQUEST_CANCEL;
                    data[1] = 0x00000000;

                    count = 8;

                    if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                    {
                        if (stm32wb_boot_dfu_check_application())
                        {
                            data[0] = STM32WB_BOOT_FWU_REQUEST_INSTALL_APPLICATION;
                            data[1] = STM32WB_BOOT_FWU_APPLICATION_BASE;
                        }
                    }
                    else
                    {
                        if (stm32wb_boot_dfu_check_wireless())
                        {
                            data[0] = STM32WB_BOOT_FWU_REQUEST_INSTALL_WIRELESS;
                            data[1] = STM32WB_BOOT_FWU_WIRELESS_BASE;
                        }
                    }
                    
                    if (!stm32wb_boot_flash_program(STM32WB_BOOT_FWU_REQUEST_BASE, (const uint8_t*)&data[0], count))
                    {
                        dfu_status = DFU_STATUS_ERR_PROG;
                    }
                    else
                    {
                        if (stm32wb_boot_dfu_control.altsetting == DFU_ALTSETTING_APPLICATION)
                        {
                            dfu_status = stm32wb_boot_fwu();
                        }
                    }
                    
                    dfu_state = DFU_STATE_IDLE;
                }
            
                if (events & STM32WB_DFU_EVENT_DETACH)
                {
                    stm32wb_usbd_disable();

                    stm32wb_boot_reset();
                }

                if (dfu_status != DFU_STATUS_OK)
                {
                    dfu_state = DFU_STATE_ERROR;
                }

                stm32wb_boot_dfu_control.status = dfu_status;
                stm32wb_boot_dfu_control.state = dfu_state;
            }
        }
    }

    flash_acr = FLASH->ACR;
    
    FLASH->ACR = flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    
    SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;
    
    FLASH->ACR = (flash_acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | (FLASH_ACR_ICRST | FLASH_ACR_DCRST);
    FLASH->ACR = flash_acr;
    
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN); 
    
    /* Allow only USB/DFU and USART1 (PA9/PA10/PA11/PA12) */
    GPIOA->LCKR = 0x0001e1ff;
    GPIOA->LCKR = 0x0000e1ff;
    GPIOA->LCKR = 0x0001e1ff;
    GPIOA->LCKR;
    GPIOB->LCKR = 0x0001ffff;
    GPIOB->LCKR = 0x0000ffff;
    GPIOB->LCKR = 0x0001ffff;
    GPIOB->LCKR;
    GPIOC->LCKR = 0x00011fff;
    GPIOC->LCKR = 0x00001fff;
    GPIOC->LCKR = 0x00011fff;
    GPIOC->LCKR;
    
    RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN); 
    
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_RTCAPBEN;
    
    PWR->CR1 &= ~PWR_CR1_DBP;
    
    stm32wb_boot_continue(0x00000000);
}

/************************************************************************************************************************************/
