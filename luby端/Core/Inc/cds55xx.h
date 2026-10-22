#ifndef __CDS55XX_H
#define __CDS55XX_H

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart3;

#define CDS_UART                  (&huart3)
#define CDS55XX_MODE_SERVO        0U
#define CDS55XX_MODE_MOTOR        1U

/* 电机模式加速度(寄存器0x22, 范围0~255, 出厂默认32)。
 * 值越大加速越猛、启动电流冲击越大, 建议在32~255之间实测取平衡。 */
#define CDS55XX_DEFAULT_ACCEL     180U

HAL_StatusTypeDef CDS55XX_WriteData(uint8_t id, uint8_t start_addr,
                                    const uint8_t *data, uint8_t data_len);
HAL_StatusTypeDef CDS55XX_SetOperatingMode(uint8_t id, uint8_t motor_mode);
HAL_StatusTypeDef CDS55XX_EnableTorque(uint8_t id, uint8_t enable);
HAL_StatusTypeDef CDS55XX_SetAcceleration(uint8_t id, uint8_t accel);
HAL_StatusTypeDef CDS55XX_SetMotorMode(uint8_t id, uint16_t speed,
                                      uint8_t direction);
HAL_StatusTypeDef CDS55XX_InitDriveMotors(void);
HAL_StatusTypeDef CDS55XX_SyncWrite(uint8_t start_addr, uint8_t data_len,
                                    const uint8_t *entries, uint8_t count);

#endif
