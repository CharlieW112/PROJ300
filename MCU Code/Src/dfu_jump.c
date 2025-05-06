#include "dfu_jump.h"
#include "stm32h7xx_hal.h"

#define DFU_MAGIC 0xDEADBEEF
#define DFU_FLAG_REG TAMP->BKP0R

void trigger_dfu(void)
{
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKP_CLK_ENABLE();
    DFU_FLAG_REG = DFU_MAGIC;
    NVIC_SystemReset();  // Restart and handle jump on boot
}

void bootloader_check(void)
{
    HAL_PWR_EnableBkUpAccess();
    if (DFU_FLAG_REG == DFU_MAGIC)
    {
        DFU_FLAG_REG = 0;
        jump_to_dfu();
    }
}

void jump_to_dfu(void)
{
    uint32_t boot_addr = 0x1FF09800;  // STM32H7 system memory
    void (*sys_mem_boot)(void) = (void (*)(void)) (*(__IO uint32_t*)(boot_addr + 4));

    HAL_RCC_DeInit();
    HAL_DeInit();

    __disable_irq();

    SCB_DisableICache();
    SCB_DisableDCache();

    __set_MSP(*(__IO uint32_t*)boot_addr);  // Set main stack pointer
    sys_mem_boot();                         // Jump to bootloader

    while (1);  // Should never return
}

