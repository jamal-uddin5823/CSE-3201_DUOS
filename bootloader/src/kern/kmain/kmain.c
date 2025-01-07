/*
 * Copyright (c) 2022
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys_init.h>
#include <cm4.h>
#include <kmain.h>
#include <stdint.h>
#include <sys_usart.h>
#include <kstdio.h>
#include <sys_rtc.h>
#include <kstring.h>
#include <system_config.h>
#include <UsartRingBuffer.h>
#include <flash.h>
#include <stdbool.h>
#include <sys_bus_matrix.h>
#ifndef DEBUG
#define DEBUG 1
// #define VERSION_ADDR ((volatile uint8_t *)0x2000FFFCU)
#endif



#define BOOTLOADER_SIZE         (0x10000u)
#define MAIN_APP_START_ADDRESS  (0x08010000U)
#define DATA_SIZE               128
#define VERSION_ADDR        ((volatile uint8_t *) 0x2000FFFC)

typedef struct {
    uint8_t* crc;
    uint8_t* data;
} PACKET;

#define PACKET_SIZE             sizeof(PACKET)

void CRC_Init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  CRC_1->CR |= 1;
}

uint32_t CRC_Calculate(const uint8_t* data, uint32_t length) {
  // Ensure CRC is initialized
  if(!(RCC->AHB1ENR & RCC_AHB1ENR_CRCEN)) {
    CRC_Init();
  }
  
  // Reset CRC before calculation
  CRC_1->CR |= CRC_CR_RESET;
  
  // Process data word by word
  for(uint32_t i = 0; i < length; i += 4) {
    // Ensure we don't read past the end of the buffer
    uint32_t word;
    if (i + 3 < length) {
      word = (data[i] << 24) | (data[i+1] << 16) | (data[i+2] << 8) | data[i+3];
    } else {
      // Handle remaining bytes
      word = 0;
      for (uint32_t j = 0; j < (length - i); j++) {
        word |= data[i+j] << (24 - (j * 8));
      }
    }
    
    // Write word to CRC data register
    CRC_1->DR = word;
  }
  
  // XOR with 0xFFFFFFFF as per standard CRC-32
  return CRC_1->DR ^ 0xFFFFFFFF;
}



static void test_flash(void) {
    ms_delay(100);
  kprintf("Initiating flash test...\n");
    ms_delay(100);

  flash_lock();                                                            
  flash_unlock();
  flash_erase_sector(SECTOR_NUMBER, 0x02);

  // uint8_t data = 101;
  // flash_program_byte(TARGET_ADDRESS, data);

  uint8_t arr[] = {10, 12};
  flash_program_byte(MAIN_APP_START_ADDRESS, arr[0]);
  flash_program_byte(MAIN_APP_START_ADDRESS + 4, arr[1]);
  // flash_program(TARGET_ADDRESS, arr, 2);

  flash_lock();

  uint8_t read_value = *(volatile uint32_t*) MAIN_APP_START_ADDRESS;
  ms_delay(100);
  kprintf("Value: %d\n", read_value);
  ms_delay(100);

  read_value = *(volatile uint32_t*) (MAIN_APP_START_ADDRESS + 4);
  ms_delay(100);
  kprintf("Second Value: %d\n", read_value);
  kprintf("Data written and verified successfully!\n");
  ms_delay(100);
}

static void vector_setup(void)
{
    
    SCB->VTOR = 0x08000000; // Bootloader start address
}

static void jump_to_main(void){

        kprintf("Hello from bootloader\n");

        typedef void(*void_fn)(void);

       
        uint32_t *reset_vector_entry = (uint32_t *)(MAIN_APP_START_ADDRESS + 4U);
        uint32_t *reset_vector= (uint32_t *)(*reset_vector_entry);

        void_fn jump_fn= (void_fn)reset_vector;
        kprintf("Reset vector: %x\n", *reset_vector_entry & 0xFFFFFFFE);
        kprintf("Reset vector entry: %x\n", reset_vector_entry);

        Uart_flush(__CONSOLE);
        

        ms_delay(100);

        SCB->VTOR = MAIN_APP_START_ADDRESS; // Main app start address
        jump_fn();

}





int new_version_available(char* server_version, char* os_version) {
    char server_major[2], server_minor[2];
    char os_major[2], os_minor[2];

    server_major[0] = server_version[0];
    server_major[1] = server_version[1];

    server_minor[0] = server_version[3];
    server_minor[1] = server_version[4];

    os_major[0] = os_version[0];
    os_major[1] = os_version[1];

    os_minor[0] = os_version[3];
    os_minor[1] = os_version[4];

    int server_major_int = __str_to_num(server_major, 10);
    int server_minor_int = __str_to_num(server_minor, 10);
    int os_major_int = __str_to_num(os_major, 10);
    int os_minor_int = __str_to_num(os_minor, 10);

    if(server_major_int > os_major_int) {
        return 1;
    } else if(server_major_int < os_major_int) {
        return 0;
    } else if(server_minor_int > os_minor_int) {
        return 1;
    } else if(server_minor_int < os_minor_int) {
        return 0;
    }
    return 0;
}

void sleep(int t) {
    for (volatile int i = 0; i < t * 1000000; i++);
}

int ceiling_divide(int num, int divisor) {
    if (divisor == 0) {
        return -1; // Error: division by zero
    }
    return (num + divisor - 1) / divisor;
}


static void write_init(void) {
  flash_lock();                                                            
  flash_unlock();
  flash_erase_sector(SECTOR_NUMBER, 0x02);
}

static uint32_t bits32_from_4_bytes(uint8_t *bytes){
  uint32_t sum = 0; 
  uint32_t power = 1 << 8; 
  for(uint8_t i = 0; i < 4; i++){
    sum *= power; 
    sum += bytes[i];
  }
  return sum; 
}

static int packet_from_bytes(uint8_t* data_bytes, uint8_t* crc_bytes, PACKET* packet) {

  
  for(uint8_t i = 0; i < 4; i++) {
    // packet->crc[i] = packet->crc[i]<<8 | data_bytes[i];
    packet->crc[i] = crc_bytes[i];
    // kprintf("%x ", packet->crc[i]);
  }
  // kprintf("\n");

  
  for(uint8_t i = 0 ; i < DATA_SIZE ; i ++) {
    // i + 2 because cmd takes 1 byte, and length takes 1 byte
    packet->data[i] = data_bytes[i];
    kprintf("%x ", packet->data[i]);
  }
  kprintf("\n");




  // no idea why
  uint32_t crc = bits32_from_4_bytes(packet->crc);

  uint32_t calculated_crc = CRC_Calculate((uint32_t*)packet->data,DATA_SIZE);
  kprintf("CRC: %x, Calculated CRC: %x\n", crc, calculated_crc);

  if(crc != calculated_crc) {
    kprintf("CRC mismatch\n");
    ms_delay(100);
    return 1;
  }
  return 0;
}


static int receive_packet(struct Packet* packet) {
  uint8_t crc[4];
  uint8_t data[DATA_SIZE];
  for (int i = 0; i < 4; i++)
  {
    crc[i] = UART_READ(__CONSOLE);
  }
  
  for(uint8_t i = 0; i < DATA_SIZE; i++) {
    data[i] = UART_READ(__CONSOLE);
  }
  Uart_flush(__CONSOLE);
  return packet_from_bytes(data,crc, packet);
}


static void write(PACKET* packet, uint32_t chunk_index) {
  uint32_t current_target_address = MAIN_APP_START_ADDRESS + chunk_index * DATA_SIZE;
  for(uint32_t i = 0; i < DATA_SIZE; i += 4) {
    uint32_t data_to_write = 0, power = (1 << 8);
    for(int j = 3; j >= 0; j--) {
      data_to_write = (data_to_write * power) + (uint32_t) packet->data[i + j];
    }
    
    flash_program_4_bytes(current_target_address + i, data_to_write,i);
  }
  // ms_delay(100);
  ms_delay(100);
  kprintf("Packet written to flash\n");
}




void kmain(void)
{
    vector_setup();
    __sys_init();
    write_init();
    // ms_delay(100);
    // test_flash();
    strcopy(VERSION_ADDR, "00.00");
    // *VERSION_ADDR = "0.0";
    char buff[100];
    int file_size;
    int iteration;
    uint8_t data[1024];
    uint8_t crc[4];
    int last_packet_size;
    uint32_t current_address = 0x08010000;
    int retry = 0;
    bool failed = false;

    PACKET packet = {
        .crc = crc,
        .data = data
    };




    kprintf("VERSION_REQ:\n");
    char server_version[100];
    read_str(server_version,5);
    kprintf("Server version: %s, OS Version: %s\n", server_version, VERSION_ADDR);
    ms_delay(100);
    if(new_version_available(server_version, VERSION_ADDR)) {
        kprintf("FIRMWARE_REQ:\n");
        ms_delay(100);
        
        read_str(buff,5);
        // ms_delay(100);
        file_size = __str_to_num(buff, 10);
        iteration = ceiling_divide(file_size, DATA_SIZE);
        if(file_size % DATA_SIZE != 0) {
            last_packet_size = file_size % DATA_SIZE;
        } else {
            last_packet_size = DATA_SIZE;
        }
        kprintf("File size: %d, Iteration %d, Last pack %d byes\n", file_size,iteration,last_packet_size);
        ms_delay(100);
        // kprintf("Iteration: %d\n", iteration);
        // ms_delay(100);
        // erase_os_memory_in_flash();
        int i=0;
        for (i = 0; i < iteration; i++) {
            
            // kscanf("%d",&packet->crc);
            // ms_delay(100);
            // kprintf("CRC %d\n", packet->crc);
            // ms_delay(100);
            ms_delay(100);
            while(1)
            {
                int res = receive_packet(&packet);
                if(res){
                    retry++;
                    if(retry>3) {
                      failed = true;
                      break;
                    }
                    kprintf("NACK %d", i);
                    ms_delay(100);
                    Uart_flush(__CONSOLE);
                }
                else{
                    write(&packet, i);
                    kprintf("ACK %d", i);
                    ms_delay(100);
                    Uart_flush(__CONSOLE);
                    break;
                }
            }
            if(failed) {
              break;
            }
            // int res = receive_packet(&packet);
            // if(res) {
            //     kprintf("CRC mismatch\n");
            //     ms_delay(100);
            //     kprintf("NACK %d", i);
            //     // break;
            // }
            
            // sleep(5);
            // kprintf("Packet received\n");
            // ms_delay(100);
            // write(&packet, i);
            // ms_delay(100);
    
            
            // ms_delay(100);
            // ms_delay(500);
            // kprintf("ACK %d", i);
            // sleep(5); 
        }
        // if(i != iteration) {
        //    kprintf("Error in writing\n");
        // }
        
        
    } else {
        kprintf("NO_UPDATE_NEEDED\n");
        ms_delay(100);
    }
    flash_lock();
    if(failed) {
      kprintf("Failed to update\n");
    }
    ms_delay(1000);
    kprintf("\nJumping to main app\n");
    ms_delay(5000);
    jump_to_main();
    while (1)
    {
      
    }
}
