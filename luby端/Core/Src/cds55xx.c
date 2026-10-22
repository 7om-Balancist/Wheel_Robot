#include "cds55xx.h"
#include "main.h"

#define CDS55XX_MAX_DATA_LEN 25U
#define CDS55XX_TX_TIMEOUT_MS 20U

HAL_StatusTypeDef CDS55XX_WriteData(uint8_t id, uint8_t start_addr,
                                    const uint8_t *data, uint8_t data_len)
{
    uint8_t tx_buffer[32];
    uint8_t checksum = 0U;
    uint8_t frame_len;
    uint32_t wait_start;
    HAL_StatusTypeDef status;

    if ((data == NULL) || (data_len > CDS55XX_MAX_DATA_LEN) ||
        (id == 0xFFU)) {
        return HAL_ERROR;
    }

    tx_buffer[0] = 0xFFU;
    tx_buffer[1] = 0xFFU;
    tx_buffer[2] = id;
    tx_buffer[3] = (uint8_t)(data_len + 3U);
    tx_buffer[4] = 0x03U;
    tx_buffer[5] = start_addr;

    for (uint8_t i = 0U; i < data_len; i++) {
        tx_buffer[6U + i] = data[i];
    }
    for (uint8_t i = 2U; i < (uint8_t)(6U + data_len); i++) {
        checksum = (uint8_t)(checksum + tx_buffer[i]);
    }
    tx_buffer[6U + data_len] = (uint8_t)(~checksum);
    frame_len = (uint8_t)(7U + data_len);

    HAL_GPIO_WritePin(RX3EN_GPIO_Port, RX3EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TX3EN_GPIO_Port, TX3EN_Pin, GPIO_PIN_SET);
    status = HAL_UART_Transmit(CDS_UART, tx_buffer, frame_len,
                               CDS55XX_TX_TIMEOUT_MS);

    if (status == HAL_OK) {
        wait_start = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(CDS_UART, UART_FLAG_TC) == RESET) {
            if ((HAL_GetTick() - wait_start) >= CDS55XX_TX_TIMEOUT_MS) {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    HAL_GPIO_WritePin(TX3EN_GPIO_Port, TX3EN_Pin, GPIO_PIN_RESET);
    return status;
}

HAL_StatusTypeDef CDS55XX_SetOperatingMode(uint8_t id, uint8_t motor_mode)
{
    static const uint8_t wheel_mode[4] = {0U, 0U, 0U, 0U};
    static const uint8_t servo_mode[4] = {0U, 0U, 0xFFU, 0x03U};
    const uint8_t *data;
    HAL_StatusTypeDef status;

    if (motor_mode > CDS55XX_MODE_MOTOR) {
        return HAL_ERROR;
    }

    data = (motor_mode == CDS55XX_MODE_MOTOR) ? wheel_mode : servo_mode;
    status = CDS55XX_WriteData(id, 0x06U, data, 4U);
    HAL_Delay(30U);
    return status;
}

HAL_StatusTypeDef CDS55XX_EnableTorque(uint8_t id, uint8_t enable)
{
    uint8_t data = (enable != 0U) ? 1U : 0U;
    return CDS55XX_WriteData(id, 0x18U, &data, 1U);
}

HAL_StatusTypeDef CDS55XX_SetAcceleration(uint8_t id, uint8_t accel)
{
    uint8_t data = accel;
    return CDS55XX_WriteData(id, 0x22U, &data, 1U);
}

HAL_StatusTypeDef CDS55XX_SetMotorMode(uint8_t id, uint16_t speed,
                                      uint8_t direction)
{
    uint8_t data[2];
    uint16_t value;

    if ((speed > 1023U) || (direction > 1U)) {
        return HAL_ERROR;
    }

    value = speed;
    if (direction != 0U) {
        value |= 0x0400U;
    }
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    return CDS55XX_WriteData(id, 0x20U, data, 2U);
}

HAL_StatusTypeDef CDS55XX_InitDriveMotors(void)
{
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(GPIOE, TX3EN_Pin | RX3EN_Pin, GPIO_PIN_RESET);

    status = CDS55XX_SetOperatingMode(1U, CDS55XX_MODE_MOTOR);
    if (status != HAL_OK) return status;
    status = CDS55XX_SetOperatingMode(2U, CDS55XX_MODE_MOTOR);
    if (status != HAL_OK) return status;
    status = CDS55XX_EnableTorque(1U, 1U);  
    if (status != HAL_OK) return status;
    status = CDS55XX_EnableTorque(2U, 1U);
    if (status != HAL_OK) return status;
    status = CDS55XX_SetAcceleration(1U, CDS55XX_DEFAULT_ACCEL);
    if (status != HAL_OK) return status;
    status = CDS55XX_SetAcceleration(2U, CDS55XX_DEFAULT_ACCEL);
    if (status != HAL_OK) return status;
    status = CDS55XX_SetMotorMode(1U, 0U, 0U);
    if (status != HAL_OK) return status;
    return CDS55XX_SetMotorMode(2U, 0U, 0U);
}

HAL_StatusTypeDef CDS55XX_SyncWrite(uint8_t start_addr, uint8_t data_len,
                                    const uint8_t *entries, uint8_t count)
{
    uint8_t tx_buffer[48];
    uint8_t checksum = 0U;
    uint8_t entry_size;
    uint8_t body_len;
    uint8_t frame_len;
    uint32_t wait_start;
    HAL_StatusTypeDef status;

    if ((entries == NULL) || (count == 0U) || (data_len == 0U)) {
        return HAL_ERROR;
    }

    entry_size = (uint8_t)(1U + data_len);
    body_len = (uint8_t)(entry_size * count);
    frame_len = (uint8_t)(8U + body_len);  /* 2(header)+ID+LEN+INST+ADDR+DLEN+body+CS */

    if (frame_len > sizeof(tx_buffer)) {
        return HAL_ERROR;
    }

    tx_buffer[0] = 0xFFU;
    tx_buffer[1] = 0xFFU;
    tx_buffer[2] = 0xFEU;
    tx_buffer[3] = (uint8_t)(body_len + 4U);
    tx_buffer[4] = 0x83U;
    tx_buffer[5] = start_addr;
    tx_buffer[6] = data_len;

    for (uint8_t i = 0U; i < body_len; i++) {
        tx_buffer[7U + i] = entries[i];
    }
    for (uint8_t i = 2U; i < (uint8_t)(frame_len - 1U); i++) {
        checksum = (uint8_t)(checksum + tx_buffer[i]);
    }
    tx_buffer[frame_len - 1U] = (uint8_t)(~checksum);

    HAL_GPIO_WritePin(RX3EN_GPIO_Port, RX3EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TX3EN_GPIO_Port, TX3EN_Pin, GPIO_PIN_SET);
    status = HAL_UART_Transmit(CDS_UART, tx_buffer, frame_len,
                               CDS55XX_TX_TIMEOUT_MS);

    if (status == HAL_OK) {
        wait_start = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(CDS_UART, UART_FLAG_TC) == RESET) {
            if ((HAL_GetTick() - wait_start) >= CDS55XX_TX_TIMEOUT_MS) {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    HAL_GPIO_WritePin(TX3EN_GPIO_Port, TX3EN_Pin, GPIO_PIN_RESET);
    return status;
}
