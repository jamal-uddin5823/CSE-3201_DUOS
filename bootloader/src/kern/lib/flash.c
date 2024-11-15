#include <flash.h>

static void sleep(int t) {
    for (volatile int i = 0; i < t * 1000000; i++);
}

uint8_t flash_read_byte(uint32_t* address)
{
  return MMIO8(address);  // Read a byte from the specified address
}

uint32_t flash_read_register(uint32_t* address) {
  return MMIO32(address);
}


void flash_unlock(void)
{
    //sleep(1);
  kprintf("flash unlock process started\n");
    //sleep(1);
  if ((FLASH->CR & FLASH_CR_LOCK) != 0) {  // Check if flash is locked
    /* Authorize the FPEC access. */
    // int32_t data = flash_read_register(FLASH_KEYR);
    // kprintf("value of FLASH_KEYR %d\n", data);

    FLASH->KEYR = FLASH_KEYR_KEY1;
    // data = flash_read_register(FLASH_KEYR);
    // kprintf("value of FLASH_KEYR %d\n", data);
    
    FLASH->KEYR = FLASH_KEYR_KEY2;
    // data = flash_read_register(FLASH_KEYR);
    // kprintf("value of FLASH_KEYR %d\n", data);
  }

  if ((FLASH->CR & FLASH_CR_LOCK) != 0) {  // Check if flash is locked
    //sleep(1);
    kprintf("Sad. Did not get unlocked.\n");
    //sleep(1);
    return;
  }
//sleep(1);
//   kprintf("flash should be unlocked now.\n");
  //sleep(1);
}


void flash_lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}


static inline void flash_set_program_size(uint32_t psize)
{
	FLASH->CR &= ~(FLASH_CR_PG_Msk << FLASH_CR_PROGRAM_SHIFT);
	FLASH->CR |= psize << FLASH_CR_PROGRAM_SHIFT;
    //sleep(1);
    // kprintf("flash program size set.\n");
    //sleep(1);
}


void flash_wait_for_last_operation(void)
{
  // while ((MMIO8(FLASH_SR) & FLASH_SR_BSY) != 0);
	// while ((MMIO32(FLASH_SR) & FLASH_SR_BSY) == FLASH_SR_BSY);
    //sleep(1);
//   kprintf("checking if flash is busy.\n");
  //sleep(1);
  while(FLASH->SR & FLASH_SR_BSY == FLASH_SR_BSY) {
  }
  //sleep(1);
//   kprintf("flash is now free to work\n");
  //sleep(1);
}

void flash_program_byte(uint32_t address, uint8_t data)
{
    //sleep(1);
//   kprintf("flashing a byte to address %d.\n", address);
//sleep(1);

	flash_wait_for_last_operation();
	flash_set_program_size(0);

	FLASH->CR |= FLASH_CR_PG;

	// MMIO8(address) = data;
  (* (volatile uint32_t*) address) = data;
  // *address = data

	flash_wait_for_last_operation();

	FLASH->CR &= ~FLASH_CR_PG;		/* Disable the PG bit. */
//sleep(1);
//   kprintf("hopefully a byte is flashed.\n");
  //sleep(1);
}

void flash_program_4_bytes(uint32_t address, uint32_t data)
{
  kprintf("flashing a byte to address %d.\n", address);

	flash_wait_for_last_operation();
	flash_set_program_size(0);

	FLASH->CR |= FLASH_CR_PG;

	// MMIO8(address) = data;
  (* (volatile uint32_t*) address) = data;
  // *address = data

	flash_wait_for_last_operation();

	FLASH->CR &= ~FLASH_CR_PG;		/* Disable the PG bit. */

  kprintf("hopefully a byte is flashed.\n");
}

void flash_program(uint32_t address, const uint8_t *data, uint32_t len, int iteration)
{
	/* TODO: Use dword and word size program operations where possible for
	 * turbo speed.
	 */
	uint32_t i;
	for (i = 0; i < len; i++) {
		flash_program_byte(address+i, data[i]);
        // //sleep(1);
        // kprintf("Written %d out of %d bytes\n",i,len);
        // //sleep(1);
	}
    // sleep(1);
    // kprintf("ACK %d",iteration);
    kprintf("Written packet %d to flash\n",iteration);
    sleep(1);
}

void flash_erase_sector(uint8_t sector, uint32_t program_size)
{
	flash_wait_for_last_operation();
	flash_set_program_size(program_size);

	/* Sector numbering is not contiguous internally! */
	if (sector >= 12) {
		sector += 4;
	}

	FLASH->CR &= ~(FLASH_CR_SNB_MASK << FLASH_CR_SNB_SHIFT);
	FLASH->CR |= (sector & FLASH_CR_SNB_MASK) << FLASH_CR_SNB_SHIFT;
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_STRT;

	flash_wait_for_last_operation();
	FLASH->CR &= ~FLASH_CR_SER;
	FLASH->CR &= ~(FLASH_CR_SNB_MASK << FLASH_CR_SNB_SHIFT);

}