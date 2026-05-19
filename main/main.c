#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dw3000.h"

#define LED_GPIO           GPIO_NUM_8
#define UWB_SPI_SCLK_GPIO  GPIO_NUM_3
#define UWB_SPI_MISO_GPIO  GPIO_NUM_4
#define UWB_SPI_MOSI_GPIO  GPIO_NUM_7
#define UWB_SPI_CS_GPIO    GPIO_NUM_6
#define UWB_SPI_IRQ_GPIO   GPIO_NUM_5
#define UWB_RST_GPIO       GPIO_NUM_14
#define UWB_WAKEUP_GPIO    GPIO_NUM_15

// Board configuration - change this for each board
#define BOARD_IS_MASTER    false  // Set to false for the second board
#define BOARD_ADDRESS      (BOARD_IS_MASTER ? 0x0100 : 0x0200)
#define PEER_ADDRESS       (BOARD_IS_MASTER ? 0x0200 : 0x0100)

static const char *TAG = "UWB_COMM";
static spi_device_handle_t uwb_spi = NULL;
static volatile bool uwb_irq_flag = false;
static uint8_t seq_num = 0;

static void IRAM_ATTR uwb_irq_isr(void *arg)
{
    uwb_irq_flag = true;
}

esp_err_t uwb_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    return spi_device_transmit(uwb_spi, &t);
}

static esp_err_t uwb_gpio_init(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO) | (1ULL << UWB_RST_GPIO) | (1ULL << UWB_WAKEUP_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << UWB_SPI_IRQ_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&out_conf));
    ESP_ERROR_CHECK(gpio_config(&in_conf));

    gpio_set_level(LED_GPIO, 0);
    gpio_set_level(UWB_RST_GPIO, 1);   // Reset high = normal operation
    gpio_set_level(UWB_WAKEUP_GPIO, 1); // Wakeup high = active

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(UWB_SPI_IRQ_GPIO, uwb_irq_isr, NULL));

    return ESP_OK;
}

static esp_err_t uwb_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = UWB_SPI_MOSI_GPIO,
        .miso_io_num = UWB_SPI_MISO_GPIO,
        .sclk_io_num = UWB_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2000000,
        .mode = 0,
        .spics_io_num = UWB_SPI_CS_GPIO,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &uwb_spi));

    return ESP_OK;
}

static void uwb_communication_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting UWB communication task");

    // Simple test: just blink LED to show task is running
    ESP_LOGI(TAG, "Running simple LED blink test");

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        ESP_LOGI(TAG, "LED ON");
        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(LED_GPIO, 0);
        ESP_LOGI(TAG, "LED OFF");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /*
    // Original UWB code - commented out for testing
    // Try to initialize DW3000
    ESP_LOGI(TAG, "Initializing DW3000...");
    esp_err_t init_err = dw3000_init();
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "DW3000 initialization failed: %s", esp_err_to_name(init_err));
        // Continue with basic LED blinking to show the task is running
        while (1) {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(900));
        }
    }

    ESP_ERROR_CHECK(dw3000_set_address(BOARD_ADDRESS));

    if (BOARD_IS_MASTER) {
        ESP_LOGI(TAG, "Board configured as MASTER (address 0x%04x)", BOARD_ADDRESS);

        while (1) {
            // Send ping message
            uwb_frame_t ping_frame = {
                .frame_type = FRAME_TYPE_DATA,
                .seq_num = seq_num++,
                .src_addr = {BOARD_ADDRESS & 0xFF, (BOARD_ADDRESS >> 8) & 0xFF},
                .dst_addr = {PEER_ADDRESS & 0xFF, (PEER_ADDRESS >> 8) & 0xFF},
                .payload = "PING",
                .payload_len = 4
            };

            ESP_LOGI(TAG, "Sending PING (seq: %d)", ping_frame.seq_num);
            esp_err_t err = dw3000_send_frame(&ping_frame);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send frame: %s", esp_err_to_name(err));
            }

            // Wait for response with shorter timeout
            uwb_frame_t response_frame;
            err = dw3000_receive_frame(&response_frame, 500);  // 500ms timeout
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Received response: %.*s (seq: %d)",
                        response_frame.payload_len, response_frame.payload, response_frame.seq_num);
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_GPIO, 0);
            } else if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGI(TAG, "No response received (timeout)");
            } else {
                ESP_LOGE(TAG, "Receive error: %s", esp_err_to_name(err));
            }

            vTaskDelay(pdMS_TO_TICKS(2000));  // Send ping every 2 seconds
        }
    } else {
        ESP_LOGI(TAG, "Board configured as SLAVE (address 0x%04x)", BOARD_ADDRESS);

        // Start continuous RX
        esp_err_t rx_err = dw3000_start_rx();
        if (rx_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start RX: %s", esp_err_to_name(rx_err));
            while (1) {
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(800));
            }
        }

        ESP_LOGI(TAG, "RX started, waiting for messages...");

        while (1) {
            uwb_frame_t received_frame;
            esp_err_t err = dw3000_receive_frame(&received_frame, 10000);  // 10 second timeout

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Received: %.*s (seq: %d)",
                        received_frame.payload_len, received_frame.payload, received_frame.seq_num);

                // Send response
                uwb_frame_t response_frame = {
                    .frame_type = FRAME_TYPE_DATA,
                    .seq_num = received_frame.seq_num,
                    .src_addr = {BOARD_ADDRESS & 0xFF, (BOARD_ADDRESS >> 8) & 0xFF},
                    .dst_addr = {PEER_ADDRESS & 0xFF, (PEER_ADDRESS >> 8) & 0xFF},
                    .payload = "PONG",
                    .payload_len = 4
                };

                ESP_LOGI(TAG, "Sending PONG (seq: %d)", response_frame.seq_num);
                dw3000_send_frame(&response_frame);

                // Blink LED to indicate communication
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(LED_GPIO, 0);
            } else if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGI(TAG, "Waiting for message...");
                // Slow blink to show we're waiting
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(1950));
            } else {
                ESP_LOGE(TAG, "Receive error: %s", esp_err_to_name(err));
                // Fast blink to indicate error
                for (int i = 0; i < 5; i++) {
                    gpio_set_level(LED_GPIO, 1);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    gpio_set_level(LED_GPIO, 0);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
    */
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting UWB Communication Example");

    ESP_ERROR_CHECK(uwb_gpio_init());
    ESP_ERROR_CHECK(uwb_spi_init());

    ESP_LOGI(TAG, "UWB GPIO and SPI initialized");

    // Create communication task
    xTaskCreate(uwb_communication_task, "uwb_comm", 4096, NULL, 5, NULL);

    // Main task just blinks LED slowly
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "System running...");
    }
}