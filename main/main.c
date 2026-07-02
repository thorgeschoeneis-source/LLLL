#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dw3000_hw.h"
#include "dwhw.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"

#include "esp_log.h"

#define PANID 0xdeca
#define MAC16 0x0002

static const char *LOG_TAG = "UWB_TAG";

#define LOG_INF(...) ESP_LOGI(LOG_TAG, __VA_ARGS__)
#define LOG_ERR(...) ESP_LOGE(LOG_TAG, __VA_ARGS__)

void app_main(void)
{
	LOG_INF("UWB tag bringup");

	// decadriver init
	if (dw3000_hw_init() != ESP_OK) {
		LOG_ERR("DW3000 HW init failed");
		return;
	}
	dw3000_hw_reset();
	if (dw3000_hw_init_interrupt() != ESP_OK) {
		LOG_ERR("DW3000 interrupt init failed");
		return;
	}

	// libdeca init
	if (!dwhw_init()) {
		LOG_ERR("DW3000 hardware probe/init failed");
		return;
	}
	if (!dwphy_config()) {
		LOG_ERR("DW3000 PHY config failed");
		return;
	}
	dwphy_set_antenna_delay(DWPHY_ANTENNA_DELAY);
	if (!dwmac_init(PANID, MAC16, dwprot_rx_handler, NULL, NULL)) {
		LOG_ERR("DWMAC init failed");
		return;
	}
	dwmac_set_frame_filter();

	LOG_INF("Tag ready, waiting for the next localization step");

	while (true) {
		vTaskDelay(pdMS_TO_TICKS(5000));
		LOG_INF("UWB tag bringup alive");
	}
}
