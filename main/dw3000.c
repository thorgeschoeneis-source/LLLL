#include "dw3000.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include "pin_config.h"

static const char *TAG = "DW3000";
spi_device_handle_t uwb_spi = NULL;
extern volatile bool uwb_irq_flag;

static esp_err_t uwb_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    if (!tx_data || len == 0 || uwb_spi == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t trans = {
        .flags = 0,
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    return spi_device_polling_transmit(uwb_spi, &trans);
}

esp_err_t dw3000_read_reg(uint16_t reg_addr, uint8_t *data, size_t len)
{
    if (!data || len == 0 || len > 127) {
        return ESP_ERR_INVALID_ARG;
    }

    // DW3000 SPI header format: [Register Address (8-bit)]
    // Bit 7: Read/Write (0=write, 1=read)
    // Bits 6-0: Register address
    uint8_t header = (1 << 7) | (reg_addr & 0x7F);  // Read bit + register address

    uint8_t *tx_buffer = malloc(len + 1);
    uint8_t *rx_buffer = malloc(len + 1);

    if (!tx_buffer || !rx_buffer) {
        free(tx_buffer);
        free(rx_buffer);
        return ESP_ERR_NO_MEM;
    }

    tx_buffer[0] = header;
    memset(tx_buffer + 1, 0x00, len);  // Dummy data for read

    esp_err_t err = uwb_spi_transfer(tx_buffer, rx_buffer, len + 1);
    if (err == ESP_OK) {
        memcpy(data, rx_buffer + 1, len);
    }

    free(tx_buffer);
    free(rx_buffer);
    return err;
}

esp_err_t dw3000_write_reg(uint16_t reg_addr, const uint8_t *data, size_t len)
{
    if (!data || len == 0 || len > 127) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t header = (0 << 7) | (reg_addr & 0x7F);
    uint8_t *tx_buffer = malloc(len + 1);
    if (!tx_buffer) {
        return ESP_ERR_NO_MEM;
    }

    tx_buffer[0] = header;
    memcpy(tx_buffer + 1, data, len);

    esp_err_t err = uwb_spi_transfer(tx_buffer, NULL, len + 1);

    free(tx_buffer);
    return err;
}

static esp_err_t dw3000_read_sys_status(uint32_t *status)
{
    uint8_t sys_status[5] = {0};
    esp_err_t err = dw3000_read_reg(DW3000_SYS_STATUS, sys_status, 5);
    if (err != ESP_OK) {
        return err;
    }

    *status = sys_status[0] | ((uint32_t)sys_status[1] << 8) | ((uint32_t)sys_status[2] << 16) | ((uint32_t)sys_status[3] << 24);
    return ESP_OK;
}

static esp_err_t dw3000_clear_sys_status(uint32_t mask)
{
    uint8_t sys_status_clear[5] = {0};
    sys_status_clear[0] = mask & 0xFF;
    sys_status_clear[1] = (mask >> 8) & 0xFF;
    sys_status_clear[2] = (mask >> 16) & 0xFF;
    sys_status_clear[3] = (mask >> 24) & 0xFF;
    return dw3000_write_reg(DW3000_SYS_STATUS, sys_status_clear, 5);
}

esp_err_t dw3000_get_rx_timestamp(uint64_t *timestamp)
{
    if (!timestamp) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[5];
    esp_err_t err = dw3000_read_reg(DW3000_RX_TIME, buf, 5);
    if (err != ESP_OK) {
        return err;
    }

    *timestamp = 0;
    for (int i = 0; i < 5; i++) {
        *timestamp |= ((uint64_t)buf[i] << (8 * i));
    }
    return ESP_OK;
}

esp_err_t dw3000_get_tx_timestamp(uint64_t *timestamp)
{
    if (!timestamp) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[5];
    esp_err_t err = dw3000_read_reg(DW3000_TX_TIME, buf, 5);
    if (err != ESP_OK) {
        return err;
    }

    *timestamp = 0;
    for (int i = 0; i < 5; i++) {
        *timestamp |= ((uint64_t)buf[i] << (8 * i));
    }
    return ESP_OK;
}

esp_err_t dw3000_set_tx_timestamp(uint64_t timestamp)
{
    uint8_t buf[5];
    for (int i = 0; i < 5; i++) {
        buf[i] = (timestamp >> (8 * i)) & 0xFF;
    }
    return dw3000_write_reg(DW3000_TX_TIME, buf, 5);
}

esp_err_t dw3000_wait_tx_done(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS < timeout_ms) {
        uint32_t status = 0;
        esp_err_t err = dw3000_read_sys_status(&status);
        if (err != ESP_OK) {
            return err;
        }
        if (status & DW3000_SYS_STATUS_TXFRS) {
            dw3000_clear_sys_status(DW3000_SYS_STATUS_TXFRS);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t dw3000_reset(void)
{
    gpio_set_level(UWB_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(UWB_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

esp_err_t dw3000_wakeup(void)
{
    // Wakeup via GPIO
    gpio_set_level(UWB_WAKEUP_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    return ESP_OK;
}

esp_err_t dw3000_init(void)
{
    ESP_LOGI(TAG, "Initializing DW3000 / DWM300...");

    // Reset the device
    ESP_ERROR_CHECK(dw3000_reset());
    ESP_ERROR_CHECK(dw3000_wakeup());

    // Wait for device to be ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Try to read device ID to verify communication
    uint8_t dev_id[4];
    esp_err_t err = dw3000_read_reg(DW3000_DEV_ID, dev_id, 4);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "DW3000/DWM300 Device ID: %02x%02x%02x%02x", dev_id[3], dev_id[2], dev_id[1], dev_id[0]);

    // Check if it's a valid DW3000 device ID (should be 0xDECA0130 for DW3000/DWM300)
    if (dev_id[3] != 0xDE || dev_id[2] != 0xCA || dev_id[1] != 0x01 || dev_id[0] != 0x30) {
        ESP_LOGE(TAG, "Invalid device ID - DW3000/DWM300 not detected!");
        if (dev_id[3] == 0x00 && dev_id[2] == 0x00 && dev_id[1] == 0x00 && dev_id[0] == 0x00) {
            ESP_LOGE(TAG, "Readback is all zero - check SPI wiring, CS pin, RESET and WAKEUP.");
        }
        return ESP_ERR_NOT_FOUND;
    }

    // Basic system configuration
    uint8_t sys_cfg[4] = {0x00, 0x00, 0x00, 0x00};
    ESP_ERROR_CHECK(dw3000_write_reg(DW3000_SYS_CFG, sys_cfg, 4));

    // Configure channel (channel 5)
    uint8_t chan_ctrl[4] = {0x05, 0x00, 0x00, 0x00};
    ESP_ERROR_CHECK(dw3000_write_reg(DW3000_CHAN_CTRL, chan_ctrl, 4));

    // Configure TX power (simple configuration)
    uint8_t tx_power[4] = {0x1F, 0x1F, 0x1F, 0x1F};
    ESP_ERROR_CHECK(dw3000_write_reg(DW3000_TX_POWER, tx_power, 4));

    ESP_LOGI(TAG, "DW3000 initialized successfully");
    return ESP_OK;
}

esp_err_t dw3000_set_address(uint16_t short_addr)
{
    uint8_t panadr[4];
    panadr[0] = short_addr & 0xFF;
    panadr[1] = (short_addr >> 8) & 0xFF;
    panadr[2] = 0x00;  // PAN ID low
    panadr[3] = 0x00;  // PAN ID high

    return dw3000_write_reg(DW3000_PANADR, panadr, 4);
}

esp_err_t dw3000_configure_channel(uint8_t channel)
{
    uint8_t chan_ctrl[4];
    chan_ctrl[0] = channel & 0x0F;
    chan_ctrl[1] = 0x00;
    chan_ctrl[2] = 0x00;
    chan_ctrl[3] = 0x00;

    return dw3000_write_reg(DW3000_CHAN_CTRL, chan_ctrl, 4);
}

esp_err_t dw3000_start_rx(void)
{
    uint8_t sys_ctrl[4] = {0x00, 0x00, 0x00, 0x00};
    sys_ctrl[1] |= 0x01;  // RXENAB bit

    // Clear old status bits before RX
    dw3000_clear_sys_status(DW3000_SYS_STATUS_RXFCG);
    return dw3000_write_reg(DW3000_SYS_CTRL, sys_ctrl, 4);
}

esp_err_t dw3000_stop_rx(void)
{
    uint8_t sys_ctrl[4] = {0x00, 0x00, 0x00, 0x00};
    sys_ctrl[0] |= 0x40;  // TRXOFF bit

    return dw3000_write_reg(DW3000_SYS_CTRL, sys_ctrl, 4);
}

bool dw3000_is_frame_ready(void)
{
    uint8_t sys_status[5];
    if (dw3000_read_reg(DW3000_SYS_STATUS, sys_status, 5) != ESP_OK) {
        return false;
    }

    return (sys_status[0] & 0x40) != 0;  // RXFCG bit
}

esp_err_t dw3000_send_frame(const uwb_frame_t *frame)
{
    if (!frame || frame->payload_len > 64) {
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare frame data
    uint8_t tx_buffer[127];
    size_t frame_len = 9 + frame->payload_len;  // Header + payload

    tx_buffer[0] = frame_len + 2;  // Frame length + FCS
    tx_buffer[1] = 0x00;  // Frame control low
    tx_buffer[2] = 0x00;  // Frame control high
    tx_buffer[3] = frame->seq_num;
    tx_buffer[4] = frame->dst_addr[0];
    tx_buffer[5] = frame->dst_addr[1];
    tx_buffer[6] = frame->src_addr[0];
    tx_buffer[7] = frame->src_addr[1];
    tx_buffer[8] = frame->frame_type;
    memcpy(&tx_buffer[9], frame->payload, frame->payload_len);

    // Write to TX buffer
    ESP_ERROR_CHECK(dw3000_write_reg(DW3000_TX_BUFFER, tx_buffer, frame_len));

    // Configure TX frame control
    uint8_t tx_fctrl[5];
    tx_fctrl[0] = frame_len & 0xFF;
    tx_fctrl[1] = (frame_len >> 8) & 0xFF;
    tx_fctrl[2] = 0x00;
    tx_fctrl[3] = 0x00;
    tx_fctrl[4] = 0x00;
    ESP_ERROR_CHECK(dw3000_write_reg(DW3000_TX_FCTRL, tx_fctrl, 5));

    // Start transmission
    uint8_t sys_ctrl[4] = {0x00, 0x00, 0x00, 0x00};
    sys_ctrl[1] |= 0x02;  // TXSTRT bit
    ESP_ERROR_CHECK(dw3000_write_reg(DW3000_SYS_CTRL, sys_ctrl, 4));

    return ESP_OK;
}

esp_err_t dw3000_receive_frame(uwb_frame_t *frame, uint32_t timeout_ms)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < timeout_ms) {
        if (dw3000_is_frame_ready()) {
            // Read frame info
            uint8_t rx_finfo[4];
            esp_err_t err = dw3000_read_reg(DW3000_RX_FINFO, rx_finfo, 4);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read RX_FINFO: %s", esp_err_to_name(err));
                continue;
            }

            uint16_t frame_len = rx_finfo[0] | ((rx_finfo[1] & 0x03) << 8);
            ESP_LOGI(TAG, "Frame length: %d", frame_len);

            if (frame_len < 9 || frame_len > 127) {
                ESP_LOGW(TAG, "Invalid frame length: %d", frame_len);
                // Clear RX status and continue
                uint8_t sys_status_clear[4] = {0x40, 0x00, 0x00, 0x00};
                dw3000_write_reg(DW3000_SYS_STATUS, sys_status_clear, 4);
                continue;
            }

            // Read frame data
            uint8_t rx_buffer[127];
            err = dw3000_read_reg(DW3000_RX_BUFFER, rx_buffer, frame_len);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read RX_BUFFER: %s", esp_err_to_name(err));
                continue;
            }

            // Parse frame - simplified format
            frame->seq_num = rx_buffer[3];
            frame->dst_addr[0] = rx_buffer[4];
            frame->dst_addr[1] = rx_buffer[5];
            frame->src_addr[0] = rx_buffer[6];
            frame->src_addr[1] = rx_buffer[7];
            frame->frame_type = rx_buffer[8];
            frame->payload_len = frame_len - 9;  // Total length minus header
            if (frame->payload_len > 64) {
                frame->payload_len = 64;
            }
            if (frame->payload_len > 0) {
                memcpy(frame->payload, &rx_buffer[9], frame->payload_len);
            }

            // Clear RX status
            uint8_t sys_status_clear[4] = {0x40, 0x00, 0x00, 0x00};
            dw3000_write_reg(DW3000_SYS_STATUS, sys_status_clear, 4);

            ESP_LOGI(TAG, "Frame parsed successfully");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Check every 10ms
    }

    return ESP_ERR_TIMEOUT;
}