#include "hardware_test.h"

#include "dw1000_hw.h"
#include "uart_tx.h"

#include <stdio.h>
#include <zephyr/sys/util.h>

int HardwareTest_Run(void)
{
    char report[160];
    int init_error;
    uint32_t device_id;
    uint32_t config_mismatch = 0xFFFFFFFFU;
    uint8_t spi_mhz = 0U;
    uint8_t temperature_raw = 0U;

    if (UART_TX_Init() != 0) {
        return -1;
    }

    init_error = DW1000_Init();
    device_id = DW1000_ReadDeviceID();
    if (init_error == 0) {
        DW1000_Configure();
        spi_mhz = DW1000_EnableFastSPI();
        config_mismatch = DW1000_VerifyConfig();
        temperature_raw = DW1000_ReadTemperatureRaw();
    }

    int length = snprintf(report, sizeof(report),
                          "DWM1001_TEST,init=%d,devid=0x%08lx,spi_mhz=%u,"
                          "config=0x%08lx,temp_raw=%u\r\n",
                          init_error, (unsigned long)device_id,
                          (unsigned int)spi_mhz,
                          (unsigned long)config_mismatch,
                          (unsigned int)temperature_raw);
    if (length > 0) {
        uint16_t output_length = (uint16_t)MIN(length, (int)sizeof(report) - 1);
        (void)UART_TX_Write((const uint8_t *)report, output_length);
    }

    return init_error == 0 && device_id == DW1000_DEVICE_ID &&
           spi_mhz != 0U && config_mismatch == 0U ? 0 : -2;
}
