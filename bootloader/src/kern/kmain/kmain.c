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
#ifndef DEBUG
#define DEBUG 1
// #define VERSION_ADDR ((volatile uint8_t *)0x2000FFFCU)
#endif



#define BOOTLOADER_SIZE         (0x10000u)
#define MAIN_APP_START_ADDRESS  (0x08000000U + BOOTLOADER_SIZE)
#define DATA_SIZE               1024

typedef struct {
    uint8_t* crc;
    uint8_t* data;
} PACKET;

#define PACKET_SIZE             sizeof(PACKET)



static void test_flash(void) {
    sleep(1);
  kprintf("Initiating flash test...\n");
    sleep(1);

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
  sleep(1);
  kprintf("Value: %d\n", read_value);
  sleep(1);

  read_value = *(volatile uint32_t*) (MAIN_APP_START_ADDRESS + 4);
  sleep(1);
  kprintf("Second Value: %d\n", read_value);
  kprintf("Data written and verified successfully!\n");
  sleep(1);
}


void jump_to_main(void){

        typedef void(*void_fn)(void);
        //second entry -> reset vector 
        uint32_t *reset_vector_entry = (uint32_t *)(MAIN_APP_START_ADDRESS + 4U);
        uint32_t *reset_vector= (uint32_t *)(*reset_vector_entry);

        void_fn jump_fn= (void_fn)reset_vector;
        sleep(1);
        kprintf("Jumping to main application\n");
        sleep(1);
        SCB->VTOR = BOOTLOADER_SIZE;
        jump_fn();

}



int new_version_available(char* server_version, char* os_version) {
    if(strcomp(server_version, os_version) == 0) {
        return 0;
    }
    return 1;
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

static void packet_from_bytes(uint8_t* data_bytes, PACKET* packet, uint32_t data_length) {
  
  for(uint8_t i = 0; i < 4; i++) {
    packet->crc[i] = packet->crc[i]<<8 | data_bytes[i];
  }

  
  for(uint8_t i = 0 ; i < data_length ; i ++) {
    // i + 2 because cmd takes 1 byte, and length takes 1 byte
    packet->data[i] = data_bytes[i + 4];
  }


  // no idea why
  uint32_t crc = bits32_from_4_bytes(packet->crc);
}


static void receive_packet(struct Packet* packet,uint32_t length) {
  uint8_t data[PACKET_SIZE];
  for(uint8_t i = 0; i < length; i++) {
    data[i] = UART_READ(__CONSOLE);
  }
  Uart_flush(__CONSOLE);
  packet_from_bytes(data, packet,length-4);
}


static void write(PACKET* packet, uint32_t chunk_index) {
  uint32_t current_target_address = MAIN_APP_START_ADDRESS + chunk_index * DATA_SIZE;

  // uint8_t data = 101;
  // flash_program_byte(TARGET_ADDRESS, data);

  // uint8_t arr[] = {10, 12};
  // flash_program_byte(TARGET_ADDRESS, arr[0]);
  // flash_program_byte(TARGET_ADDRESS + 4, arr[1]);
  // flash_program(TARGET_ADDRESS, arr, 2);
  for(uint32_t i = 0; i < DATA_SIZE; i += 4) {
    uint32_t data_to_write = 0, power = (1 << 8);
    for(int j = 3; j >= 0; j--) {
      data_to_write = (data_to_write * power) + (uint32_t) packet->data[i + j];
    }
    
    // debug("Chunk Index");
    // debug(convertu32(i, 10));
    // debug("data to write:");
    // debug(convertu32(data_to_write, 10));
    flash_program_4_bytes(current_target_address + i, data_to_write,i);
  }
  sleep(1);
  kprintf("Packet written to flash\n");
  sleep(1);
}




void kmain(void)
{
    __sys_init();
    write_init();
    // sleep(1);
    // test_flash();

    char *VERSION_ADDR = "0.0";
    char buff[100];
    int file_size;
    int iteration;
    uint8_t data[1024];
    uint8_t crc[4];
    int last_packet_size;
    uint32_t current_address = 0x08010000;

    PACKET packet = {
        .crc = crc,
        .data = data
    };




    kprintf("VERSION_REQ:\n");
    char server_version[100];
    read_str(server_version,5);
    kprintf("Server version: %s\n", server_version);
    sleep(1);
    if(new_version_available(server_version, VERSION_ADDR)) {
        kprintf("FIRMWARE_REQ:\n");
        sleep(1);
        
        read_str(buff,5);
        // sleep(1);
        file_size = __str_to_num(buff, 10);
        iteration = ceiling_divide(file_size, DATA_SIZE);
        if(file_size % DATA_SIZE != 0) {
            last_packet_size = file_size % DATA_SIZE;
        } else {
            last_packet_size = DATA_SIZE;
        }
        kprintf("File size: %d, Iteration %d\n", file_size,iteration);
        sleep(1);
        // kprintf("Iteration: %d\n", iteration);
        // sleep(1);
        // erase_os_memory_in_flash();
       
        for (int i = 0; i < iteration; i++) {
            
            // kscanf("%d",&packet->crc);
            // sleep(1);
            // kprintf("CRC %d\n", packet->crc);
            // sleep(1);
            sleep(1);
            if(i < iteration-1)
                receive_packet(&packet, PACKET_SIZE);
                
            else 
                receive_packet(&packet, last_packet_size + 4);
            sleep(5);
            kprintf("Packet received\n");
            sleep(1);
            write(&packet, i);
            sleep(1);
            // flash_program(current_address, data, (i < iteration-1) ? 1024 : last_packet_size,i);
            // current_address += 1024;


            // uint8_t status = flash_write(data, (i < iteration-1) ? 1024 : last_packet_size, current_address);
            // if(status == 1) {
            //     sleep(1);
            //     kprintf("WRITE ERROR %d", i);
            // } else {
            //     status = flash_verify(data, (i < iteration-1) ? 1024 : last_packet_size, current_address);
            //     if (status == 1) {
            //         sleep(1);
            //         kprintf("VERIFY ERROR %d", i);
            //     } else {
            //         current_address += 1024; // Move to next chunk address
            //         // sleep(1);
            //         kprintf("ACK %d", i);
            //         sleep(1);
            //     }
            // }
            
            // sleep(1);
            
            sleep(1);
            kprintf("ACK %d", i);
            sleep(5);
        }
        
        
    } else {
        kprintf("NO_UPDATE_NEEDED\n");
        sleep(1);
    }
    
    jump_to_main();
    while (1)
    {
      
    }
}
