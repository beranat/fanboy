#include <esp_system.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <soc/rtc.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "fan.h"
#include "main.h"

constexpr const char *APPLICATION = "app";
constexpr TickType_t UPDATE_PERIOD = pdMS_TO_TICKS(2250);

void fatalError(esp_err_t error, const char *info, ...) {
	if (ESP_OK != error) {
		static char message[128];
		va_list list;
		va_start(list, info);
		vsnprintf(message, sizeof(message)/sizeof(*message), info, list);
		va_end(list);

		ESP_LOGE(APPLICATION, "Fatal Error: %s(%d) %s", esp_err_to_name(error), error, message);
		esp_deep_sleep_start();
	}
}

void initHardware() {
#ifdef CONFIG_MAIN_LED_ENABLED
	{
		ESP_LOGI(APPLICATION, "Led config (GPIO%d)", CONFIG_MAIN_LED_GPIO);
		const gpio_config_t conf = {
			BIT64(CONFIG_MAIN_LED_GPIO),
			GPIO_MODE_OUTPUT,
			GPIO_PULLUP_DISABLE,
			GPIO_PULLDOWN_DISABLE,
			GPIO_INTR_DISABLE
		};
		fatalError(gpio_config(&conf), "LED gpio_config");
	}

	// Hardware TESTS
	static const uint64_t delay = 250000;
	for (int i = 0; i < 3; ++i) {
		ledEnable(0 == (i&1));
		const uint64_t end = esp_timer_get_time() + delay;
		while (end > esp_timer_get_time());
	}
	ledEnable(false);
#endif
}

void ledEnable(bool on) {
#ifdef CONFIG_MAIN_LED_ENABLED
	gpio_set_level(static_cast<gpio_num_t>(CONFIG_MAIN_LED_GPIO), on?1:0);
#endif
}

extern "C" void app_main() {
	ESP_LOGW(APPLICATION, "Init HW");
	initHardware();

	ESP_LOGW(APPLICATION, "Init Fan");
	fatalError(fanInit()?ESP_OK:ESP_FAIL, "Fan initialization");

	TickType_t update = 0;
	for (bool on = true; true; on = !on) {
		ledEnable(on);
		if (update <= xTaskGetTickCount()) {
			fanStatus();
			update = xTaskGetTickCount() + UPDATE_PERIOD;
		}
		vTaskDelay(pdMS_TO_TICKS(250));
	} while (true);

	ESP_LOGE(APPLICATION, "System halted");
	esp_deep_sleep_start();
}
