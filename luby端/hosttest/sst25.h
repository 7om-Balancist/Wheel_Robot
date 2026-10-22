#ifndef __SST25_H
#define __SST25_H
#include <stdint.h>
#include <stdbool.h>
#define SST25_SECTOR_SIZE 0x1000UL
#define SST25_DEV_SIZE    0x200000UL
typedef enum { SST25_OK=0, SST25_ERR_PARAM, SST25_ERR_ID, SST25_ERR_WINDOW, SST25_ERR_PROTECTED,
               SST25_ERR_WEL, SST25_ERR_NOT_ERASED, SST25_ERR_TIMEOUT, SST25_ERR_VERIFY } Sst25Status_t;
Sst25Status_t SST25_Read(uint32_t addr, uint8_t *dst, uint32_t len);
Sst25Status_t SST25_Erase4K(uint32_t addr);
Sst25Status_t SST25_ProgramBytes(uint32_t addr, const uint8_t *src, uint32_t len);
bool          SST25_IsErased(uint32_t addr, uint32_t len);
#endif
