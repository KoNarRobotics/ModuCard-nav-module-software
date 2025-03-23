#include "stm32h7xx_hal.h"


#pragma once

#ifdef __cplusplus
extern "C" {
#endif


static uint8_t QSPI_WriteEnable(void);
static uint8_t QSPI_AutoPollingMemReady(void);
static uint8_t QSPI_Configuration(void);
static uint8_t QSPI_ResetChip(void);

/*MX25L512 memory parameters*/
#define MEMORY_FLASH_SIZE 0x7A12000 /* 128 MBits => 16MBytes */
#define MEMORY_BLOCK_SIZE 0x10000   /* 1024 sectors of 64KBytes */
#define MEMORY_SECTOR_SIZE 0x1000   /* 16384 subsectors of 4kBytes */
#define MEMORY_PAGE_SIZE 0x100      /* 262144 pages of 256 bytes */


/*MX25L512 commands */
#define WRITE_ENABLE_CMD 0x06
#define READ_STATUS_REG_CMD 0x05
#define WRITE_STATUS_REG_CMD 0x01
#define SECTOR_ERASE_CMD 0x20
#define CHIP_ERASE_CMD 0xC7
#define QUAD_IN_FAST_PROG_CMD 0x38
#define READ_CONFIGURATION_REG_CMD 0x15
#define QUAD_READ_IO_CMD 0xEC
#define QUAD_OUT_FAST_READ_CMD 0x6B
#define QPI_ENABLE_CMD 0x35
#define DUMMY_CLOCK_CYCLES_READ_QUAD 10
#define RESET_ENABLE_CMD 0x66
#define RESET_EXECUTE_CMD 0x99
#define DISABLE_QIP_MODE 0xf5


#ifdef __cplusplus
}
#endif