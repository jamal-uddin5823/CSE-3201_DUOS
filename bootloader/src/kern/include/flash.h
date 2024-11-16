#ifndef __FLASH_H__
#define __FLASH_H__
#ifdef __cplusplus
extern "C" {
#endif
    #include <stdint.h>
    #include <kstdio.h>
    // #include<stm32f446xx.h>
    #include<sys_bus_matrix.h>

    // Flash memory constants for STM32F446RE
    #define MMIO8(address)              (*(volatile uint8_t *)(address))
    #define MMIO32(address)             (*(volatile uint32_t *)(address)) 

    #define FLASH_KEYR_KEY1             0x45670123
    #define FLASH_KEYR_KEY2             0xCDEF89AB

    #define FLASH_CR_SNB_MASK		    0x1f
    #define FLASH_CR_PROGRAM_SHIFT		8
    #define FLASH_CR_SNB_SHIFT		    3


    #define SECTOR_NUMBER               4

    uint8_t flash_read_byte(uint32_t* address);
    uint32_t flash_read_register(uint32_t* address);
    void flash_unlock(void);
    void flash_lock(void);
    void flash_wait_for_last_operation(void);
    void flash_program_byte(uint32_t address, uint8_t data);
    void flash_program_4_bytes(uint32_t address, uint32_t data,uint32_t iteration);
    void flash_program(uint32_t address, const uint8_t *data, uint32_t len, int iteration);
    void flash_erase_sector(uint8_t sector, uint32_t program_size);

#ifdef __cplusplus
}
#endif
#endif