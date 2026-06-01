#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dw3000.h"
#include "pin_config.h"

#define IS_TAG 1
#define TAG_ID 10
#define FIRST_ANCHOR_ID 1
#define NUM_ANCHORS 3
#define ANCHOR_ID 1

#define RESPONSE_TIMEOUT_MS 500
#define MAX_RETRIES 3

#define MIN_DISTANCE_CM 0.0f
#define MAX_DISTANCE_CM 1000.0f

#define FRAME_TYPE_POLL   0x10
#define FRAME_TYPE_RESP   0x11
#define FRAME_TYPE_FINAL  0x12
#define FRAME_TYPE_INFO   0x14
#define FRAME_TYPE_REPORT 0x13

typedef struct {
    int anchor_id;
    float distance;
    float distance_history[30];
    int history_index;
    float filtered_distance;
    int32_t t_roundA;
    int32_t t_replyA;
    int32_t t_roundB;
    int32_t t_replyB;
} anchor_data_t;

static const char *TAG = "UWB_RTLS";
static anchor_data_t anchors[NUM_ANCHORS];
static int current_anchor_index = 0;

static void write_u32_le(uint8_t *buf, uint32_t value)
{
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = (value >> 16) & 0xFF;
    buf[3] = (value >> 24) & 0xFF;
}

static uint32_t read_u32_le(const uint8_t *buf)
{
    return ((uint32_t)buf[0]) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static uint16_t read_u16_le(const uint8_t *buf)
{
    return ((uint16_t)buf[0]) | ((uint16_t)buf[1] << 8);
}

static bool is_valid_distance(float distance)
{
    return distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM;
}

static float calculate_median(float arr[], int size)
{
    float temp[30];
    memcpy(temp, arr, sizeof(temp));
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (temp[j] < temp[i]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    if (size % 2 == 0) {
        return (temp[size / 2 - 1] + temp[size / 2]) / 2.0f;
    }
    return temp[size / 2];
}

static void update_filtered_distance(anchor_data_t *anchor)
{
    anchor->distance_history[anchor->history_index] = anchor->distance;
    anchor->history_index = (anchor->history_index + 1) % 30;

    float valid_distances[30];
    int valid_count = 0;

    for (int i = 0; i < 30; i++) {
        if (is_valid_distance(anchor->distance_history[i])) {
            valid_distances[valid_count++] = anchor->distance_history[i];
        }
    }

    if (valid_count > 0) {
        anchor->filtered_distance = calculate_median(valid_distances, valid_count);
    } else {
        anchor->filtered_distance = 0;
    }
}

static float convert_time_to_cm(int32_t raw_time)
{
    if (raw_time <= 0) {
        return 0.0f;
    }
    return raw_time * DW3000_TIME_UNIT_METERS * 100.0f;
}

static int32_t calculate_ranging_time(int32_t t_roundA, int32_t t_replyA, int32_t t_roundB, int32_t t_replyB)
{
    int64_t reply_diff = (int64_t)t_replyA - (int64_t)t_replyB;
    int64_t first_rt = (int64_t)t_roundA - (int64_t)t_replyB;
    int64_t second_rt = (int64_t)t_roundB - (int64_t)t_replyA;
    int64_t combined_rt = (first_rt + second_rt - reply_diff) / 2;
    return (int32_t)(combined_rt / 2);
}

static void initialize_anchors(void)
{
    for (int i = 0; i < NUM_ANCHORS; i++) {
        anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
        anchors[i].distance = 0.0f;
        anchors[i].history_index = 0;
        anchors[i].filtered_distance = 0.0f;
        anchors[i].t_roundA = 0;
        anchors[i].t_replyA = 0;
        anchors[i].t_roundB = 0;
        anchors[i].t_replyB = 0;
        memset(anchors[i].distance_history, 0, sizeof(anchors[i].distance_history));
    }
}

static void print_all_distances(void)
{
    ESP_LOGI(TAG, "-- Distances --");
    for (int i = 0; i < NUM_ANCHORS; i++) {
        if (anchors[i].filtered_distance > 0) {
            ESP_LOGI(TAG, "Anchor %d: %.2f cm (raw %.2f)", anchors[i].anchor_id, anchors[i].filtered_distance, anchors[i].distance);
        } else {
            ESP_LOGI(TAG, "Anchor %d: INVALID", anchors[i].anchor_id);
        }
    }
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
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&out_conf));
    ESP_ERROR_CHECK(gpio_config(&in_conf));

    gpio_set_level(LED_GPIO, 0);
    gpio_set_level(UWB_RST_GPIO, 1);
    gpio_set_level(UWB_WAKEUP_GPIO, 1);

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
        .clock_speed_hz = 4000000,
        .mode = 0,
        .spics_io_num = UWB_SPI_CS_GPIO,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &uwb_spi));

    return ESP_OK;
}

static void copy_address_to_frame(uint16_t addr, uint8_t *target)
{
    target[0] = addr & 0xFF;
    target[1] = (addr >> 8) & 0xFF;
}

static bool address_matches(const uint8_t *addr_bytes, uint16_t addr)
{
    return addr_bytes[0] == (addr & 0xFF) && addr_bytes[1] == ((addr >> 8) & 0xFF);
}

static esp_err_t send_poll(uint16_t dst_addr)
{
    uwb_frame_t frame = {0};
    frame.frame_type = FRAME_TYPE_POLL;
    frame.seq_num = 0;
    copy_address_to_frame(TAG_ID, frame.src_addr);
    copy_address_to_frame(dst_addr, frame.dst_addr);
    frame.payload_len = 0;
    return dw3000_send_frame(&frame);
}

static esp_err_t send_response(uint16_t dst_addr)
{
    uwb_frame_t frame = {0};
    frame.frame_type = FRAME_TYPE_RESP;
    frame.seq_num = 0;
    copy_address_to_frame(ANCHOR_ID, frame.src_addr);
    copy_address_to_frame(dst_addr, frame.dst_addr);
    frame.payload_len = 0;
    return dw3000_send_frame(&frame);
}

static esp_err_t send_final(uint16_t dst_addr)
{
    uwb_frame_t frame = {0};
    frame.frame_type = FRAME_TYPE_FINAL;
    frame.seq_num = 0;
    copy_address_to_frame(TAG_ID, frame.src_addr);
    copy_address_to_frame(dst_addr, frame.dst_addr);
    frame.payload_len = 0;
    return dw3000_send_frame(&frame);
}

static esp_err_t send_info(uint16_t dst_addr, int32_t t_roundA, int32_t t_replyA)
{
    uwb_frame_t frame = {0};
    frame.frame_type = FRAME_TYPE_INFO;
    frame.seq_num = 0;
    copy_address_to_frame(TAG_ID, frame.src_addr);
    copy_address_to_frame(dst_addr, frame.dst_addr);
    write_u32_le(frame.payload, (uint32_t)t_roundA);
    write_u32_le(frame.payload + 4, (uint32_t)t_replyA);
    frame.payload_len = 8;
    return dw3000_send_frame(&frame);
}

static esp_err_t send_report(uint16_t dst_addr, int32_t t_roundB, int32_t t_replyB)
{
    uwb_frame_t frame = {0};
    frame.frame_type = FRAME_TYPE_REPORT;
    frame.seq_num = 0;
    copy_address_to_frame(ANCHOR_ID, frame.src_addr);
    copy_address_to_frame(dst_addr, frame.dst_addr);
    write_u32_le(frame.payload, (uint32_t)t_roundB);
    write_u32_le(frame.payload + 4, (uint32_t)t_replyB);
    frame.payload_len = 8;
    return dw3000_send_frame(&frame);
}

static void tag_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting TAG mode");
    initialize_anchors();

    while (true) {
        anchor_data_t *current_anchor = &anchors[current_anchor_index];
        uint16_t anchor_id = current_anchor->anchor_id;

        ESP_LOGI(TAG, "Sending POLL to Anchor %d", anchor_id);
        if (send_poll(anchor_id) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send POLL");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        uint64_t tx_poll_ts = 0;
        dw3000_get_tx_timestamp(&tx_poll_ts);

        dw3000_start_rx();
        uwb_frame_t response = {0};
        if (dw3000_receive_frame(&response, RESPONSE_TIMEOUT_MS) != ESP_OK || response.frame_type != FRAME_TYPE_RESP || !address_matches(response.src_addr, anchor_id)) {
            ESP_LOGW(TAG, "No valid response from Anchor %d", anchor_id);
            current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint64_t rx_response_ts = 0;
        dw3000_get_rx_timestamp(&rx_response_ts);
        int32_t t_roundA = (int32_t)(rx_response_ts - tx_poll_ts);
        ESP_LOGI(TAG, "Received response, t_roundA=%d", t_roundA);

        if (send_final(anchor_id) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send FINAL");
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        uint64_t tx_final_ts = 0;
        dw3000_get_tx_timestamp(&tx_final_ts);
        int32_t t_replyA = (int32_t)(tx_final_ts - rx_response_ts);
        current_anchor->t_roundA = t_roundA;
        current_anchor->t_replyA = t_replyA;

        if (send_info(anchor_id, t_roundA, t_replyA) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send INFO to Anchor %d", anchor_id);
            current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        dw3000_start_rx();
        uwb_frame_t report = {0};
        if (dw3000_receive_frame(&report, RESPONSE_TIMEOUT_MS) != ESP_OK || report.frame_type != FRAME_TYPE_REPORT || !address_matches(report.src_addr, anchor_id)) {
            ESP_LOGW(TAG, "No valid report from Anchor %d", anchor_id);
            current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
            continue;
        }

        uint32_t t_roundB = read_u32_le(report.payload);
        uint32_t t_replyB = read_u32_le(report.payload + 4);
        ESP_LOGI(TAG, "Anchor %d report t_roundB=%u t_replyB=%u", anchor_id, t_roundB, t_replyB);

        int32_t raw = calculate_ranging_time(t_roundA, t_replyA, (int32_t)t_roundB, (int32_t)t_replyB);
        current_anchor->distance = convert_time_to_cm(raw);
        update_filtered_distance(current_anchor);
        print_all_distances();

        current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void anchor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting ANCHOR mode (ID=%d)", ANCHOR_ID);

    while (true) {
        dw3000_start_rx();
        uwb_frame_t incoming = {0};

        if (dw3000_receive_frame(&incoming, RESPONSE_TIMEOUT_MS) != ESP_OK || incoming.frame_type != FRAME_TYPE_POLL || !address_matches(incoming.dst_addr, ANCHOR_ID)) {
            continue;
        }

        uint64_t rx_poll_ts = 0;
        dw3000_get_rx_timestamp(&rx_poll_ts);
        ESP_LOGI(TAG, "Received POLL from Tag");

        if (send_response(read_u16_le(incoming.src_addr)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send RESPONSE");
            continue;
        }

        uint64_t tx_response_ts = 0;
        dw3000_get_tx_timestamp(&tx_response_ts);
        int32_t t_replyB = (int32_t)(tx_response_ts - rx_poll_ts);

        vTaskDelay(pdMS_TO_TICKS(20));
        dw3000_start_rx();
        uwb_frame_t final_frame = {0};
        if (dw3000_receive_frame(&final_frame, RESPONSE_TIMEOUT_MS) != ESP_OK || final_frame.frame_type != FRAME_TYPE_FINAL || !address_matches(final_frame.dst_addr, ANCHOR_ID)) {
            continue;
        }

        uint64_t rx_final_ts = 0;
        dw3000_get_rx_timestamp(&rx_final_ts);
        int32_t t_roundB = (int32_t)(rx_final_ts - tx_response_ts);

        ESP_LOGI(TAG, "Received FINAL from Tag");

        dw3000_start_rx();
        uwb_frame_t info_frame = {0};
        if (dw3000_receive_frame(&info_frame, RESPONSE_TIMEOUT_MS) != ESP_OK || info_frame.frame_type != FRAME_TYPE_INFO || !address_matches(info_frame.src_addr, read_u16_le(final_frame.src_addr))) {
            ESP_LOGW(TAG, "Expected INFO frame after FINAL");
            continue;
        }

        int32_t t_roundA = (int32_t)read_u32_le(info_frame.payload);
        int32_t t_replyA = (int32_t)read_u32_le(info_frame.payload + 4);
        ESP_LOGI(TAG, "INFO received t_roundA=%d t_replyA=%d", t_roundA, t_replyA);

        if (send_report(read_u16_le(info_frame.src_addr), t_roundB, t_replyB) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send REPORT");
            continue;
        }

        ESP_LOGI(TAG, "Sent REPORT to Tag");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting UWB RTLS application");

    ESP_ERROR_CHECK(uwb_gpio_init());
    ESP_ERROR_CHECK(uwb_spi_init());
    ESP_ERROR_CHECK(dw3000_init());

    if (IS_TAG) {
        ESP_ERROR_CHECK(dw3000_set_address(TAG_ID));
    } else {
        ESP_ERROR_CHECK(dw3000_set_address(ANCHOR_ID));
    }

    xTaskCreate(IS_TAG ? tag_task : anchor_task, "uwb_ranging", 8192, NULL, 5, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "UWB system running");
    }
}
