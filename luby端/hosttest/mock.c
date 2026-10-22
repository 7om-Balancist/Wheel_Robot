#include "sst25.h"
#include <string.h>
#include <stdio.h>
#define DEV 0x200000UL
static unsigned char flash[DEV];
int g_write_budget = -1;        /* -1=不限; >=0 时每写一字节减一, 减到 0 就"断电" */
int g_erase_budget = -1;
int g_power_lost   = 0;

void mock_reset(void){ memset(flash,0xFF,sizeof(flash)); g_write_budget=-1; g_erase_budget=-1; g_power_lost=0; }
unsigned char *mock_raw(void){ return flash; }

Sst25Status_t SST25_Read(uint32_t a, uint8_t *d, uint32_t n){
    if(!d||a+n>DEV) return SST25_ERR_PARAM;
    memcpy(d, flash+a, n); return SST25_OK; }
bool SST25_IsErased(uint32_t a, uint32_t n){
    for(uint32_t i=0;i<n;i++) if(flash[a+i]!=0xFF) return false;
    return true; }
Sst25Status_t SST25_Erase4K(uint32_t a){
    a &= ~0xFFFUL;
    if(g_erase_budget>=0){ if(g_erase_budget==0){ g_power_lost=1; return SST25_ERR_TIMEOUT; }
        g_erase_budget--; 
        /* 部分擦除: 只擦一半就断电 */
        memset(flash+a,0xFF,2048); g_power_lost=1; return SST25_ERR_TIMEOUT; }
    memset(flash+a,0xFF,4096); return SST25_OK; }
Sst25Status_t SST25_ProgramBytes(uint32_t a, const uint8_t *s, uint32_t n){
    if(!SST25_IsErased(a,n)) return SST25_ERR_NOT_ERASED;   /* 与真实驱动同规则 */
    for(uint32_t i=0;i<n;i++){
        if(g_write_budget==0){ g_power_lost=1; return SST25_ERR_TIMEOUT; }
        if(g_write_budget>0) g_write_budget--;
        flash[a+i] &= s[i];        /* NOR: 只能 1->0 */
    }
    return SST25_OK; }
