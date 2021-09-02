#include <esp_types.h>

#include <atomic>
#include <sstream>
#include <iomanip>
#include <limits>

#include <esp_system.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <soc/rtc.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "main.h"

#include "config.hpp"
#include "fan.h"

constexpr mcpwm_capture_on_edge_t MCPWM_BOTH_EDGE = static_cast<mcpwm_capture_on_edge_t>(MCPWM_POS_EDGE | MCPWM_NEG_EDGE);
constexpr const char *TAG = "fan";

constexpr size_t FAN_COUNT = 3;
constexpr TickType_t UPDATE_DELAY = pdMS_TO_TICKS(250);
constexpr size_t STACK_SIZE = 2*configMINIMAL_STACK_SIZE + 1024;
constexpr TickType_t VALID_PERIOD = pdMS_TO_TICKS(2000);

#define PWMIN_UNIT MCPWM_UNIT_0
#define TACHO_UNIT MCPWM_UNIT_1

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

constexpr mcpwm_io_signals_t tachoSignals[FAN_COUNT] = {
	MCPWM0A,
	MCPWM1A,
	MCPWM2A
};

constexpr mcpwm_io_signals_t  pwmSignal[FAN_COUNT] = {
	MCPWM_CAP_0,
	MCPWM_CAP_1,
	MCPWM_CAP_2
};

constexpr mcpwm_capture_signal_t pwmSelect[FAN_COUNT] = {
	MCPWM_SELECT_CAP0,
	MCPWM_SELECT_CAP1,
	MCPWM_SELECT_CAP2
};

constexpr uint32_t pwmInterrupt[FAN_COUNT] = {
	CAP0_INT_EN,
	CAP1_INT_EN,
	CAP2_INT_EN
};

constexpr mcpwm_timer_t tachoTimers[FAN_COUNT] = {
	MCPWM_TIMER_0,
	MCPWM_TIMER_1,
	MCPWM_TIMER_2
};

struct PwmData {
	std::atomic<uint32_t> duration;
	std::atomic<uint32_t> negative;
	std::atomic<TickType_t> timestamp;
	gpio_num_t pin;
	std::atomic<uint32_t> prevPos;
	std::atomic<uint32_t> prevNeg;
};

static volatile PwmData IRAM_ATTR pwmData[FAN_COUNT] = { 0 };

static void IRAM_ATTR isrOnCaptured(unsigned int num) noexcept {
	volatile PwmData &data = pwmData[num];

	const uint32_t value = mcpwm_capture_signal_get_value(PWMIN_UNIT, pwmSelect[num]);
//	const bool isPositive = (1 == gpio_get_level(data.pin));
	const bool isPositive = true;

	if (isPositive) {
		const uint32_t duration = static_cast<uint32_t>(value - data.prevPos);
		const uint32_t negative = static_cast<uint32_t>(value - data.prevNeg);

		data.prevPos = value;

		data.duration.store(duration, std::memory_order_release);
		data.negative.store(negative, std::memory_order_release);
		data.timestamp.store(xTaskGetTickCountFromISR(), std::memory_order_release);
		std::atomic_thread_fence(std::memory_order_release);
	}
	else {
		data.prevNeg = value;
	}
}

static void IRAM_ATTR isrCaptureHandler(void*) noexcept {
	static_assert(PWMIN_UNIT == MCPWM_UNIT_0);

    const uint32_t status = MCPWM0.int_st.val;
    if (status & MCPWM_CAP0_INT_ENA)
		isrOnCaptured(0);

    if (status & MCPWM_CAP1_INT_ENA)
		isrOnCaptured(1);

    if (status & MCPWM_CAP2_INT_ENA)
		isrOnCaptured(2);

    MCPWM0.int_clr.val = status;
}

static void init(unsigned int num) {
	const Config &fi = Config::get(num);

	if (!fi.isEnabled) {
		ESP_LOGI(TAG, "[%u] disabled", num+1);
		return;
	}

	// Enable signal
	if (GPIO_NUM_NC != fi.enSignal) {
		ESP_LOGI(TAG, "[%u] En %s active (%u)", num+1, fi.enActive?"HIGH":"low", fi.enSignal);
		const gpio_config_t conf = {
			BIT64(fi.enSignal),
			GPIO_MODE_OUTPUT,
			GPIO_PULLUP_DISABLE,
			GPIO_PULLDOWN_DISABLE,
			GPIO_INTR_DISABLE
		};
		fatalError(gpio_config(&conf), "FAN[%u]-en gpio_config", num+1);
	}
	// PWM-IN
	if (GPIO_NUM_NC != fi.pwm) {
		ESP_LOGI(TAG, "[%u] PWM-in enabled (%u)", num+1, fi.pwm);
		pwmData[num].pin = fi.pwm;
		fatalError(mcpwm_gpio_init(PWMIN_UNIT, pwmSignal[num], fi.pwm), "FAN[%u] mcpwm_gpio", num+1);
    	fatalError(mcpwm_capture_enable(PWMIN_UNIT, pwmSelect[num], MCPWM_POS_EDGE, 0), "FAN[%u] mcpwm_capture", num+1);
    	//fatalError(mcpwm_capture_enable(PWMIN_UNIT, pwmSelect[num], MCPWM_BOTH_EDGE, 0), "FAN[%u] mcpwm_capture", num+1);
		MCPWM0.int_ena.val |= pwmInterrupt[num];
    	fatalError(mcpwm_isr_register(PWMIN_UNIT, &isrCaptureHandler, NULL, ESP_INTR_FLAG_IRAM, NULL), "FAN[%u]  mcpwm_isr", num+1);
	}

	// TACHO-OUT
	ESP_LOGI(TAG, "[%u] Tacho-out (%u)", num+1, fi.tacho);
	fatalError(mcpwm_gpio_init(TACHO_UNIT, tachoSignals[num], fi.tacho), "FAN[%u] mcpwm_gpio", num+1);
    mcpwm_config_t config = { 0 };
    config.frequency = 1000;
    config.cmpr_a = 50;
    config.cmpr_b = 0;
    config.counter_mode = MCPWM_UP_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;

	fatalError(mcpwm_init(TACHO_UNIT, tachoTimers[num], &config), "FAN[%u] mcpwm_init", num+1);
	fatalError(mcpwm_set_signal_low(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_A), "FAN[%u] disable A", num+1);
	fatalError(mcpwm_set_signal_low(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_B), "FAN[%u] disable B", num+1);
}

static void setTacho(unsigned int num, unsigned int freq) {
    if (0 == freq) {
		fatalError(mcpwm_set_signal_low(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_A), "FAN[%u] disable A", num+1);
		return;
	}

	if (freq != mcpwm_get_frequency(TACHO_UNIT, tachoTimers[num])) {
		fatalError(mcpwm_set_duty(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_A, 50.0f), "FAN[%u] mcpwm_set_duty", num+1);
		fatalError(mcpwm_set_duty_type(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_A, MCPWM_DUTY_MODE_0), "FAN[%u] mcpwm_set_duty_type", num+1);
		fatalError(mcpwm_set_frequency(TACHO_UNIT, tachoTimers[num], freq), "FAN[%u] mcpwm_set_freq", num+1);
	}
}

static void update(unsigned int num) {
	const Config &fi = Config::get(num);
	const volatile PwmData &data = pwmData[num];

	 if (!fi.isEnabled)
		return;

	if (GPIO_NUM_NC != fi.enSignal) {
		bool isDisabled = (0 == gpio_get_level(static_cast<gpio_num_t>(fi.enSignal))) == fi.enActive;
		if (isDisabled) {
			setTacho(num, 0);
			return;
		}
	}

	float ratio = 1.0f;

	if (GPIO_NUM_NC != fi.pwm) {
		atomic_thread_fence(std::memory_order_acquire);
		const TickType_t timeout = data.timestamp.load(std::memory_order_acquire) + VALID_PERIOD;
/*
		const uint32_t duty = data.duty.load(std::memory_order_acquire);

		if (xTaskGetTickCount() <= timeout)
			ratio = static_cast<float>(duty) / std::numeric_limits<uint16_t>::max();
*/
	}

	const unsigned int pulses = static_cast<unsigned int>(fi.maxSpeed * fi.ppr * ratio / 60);
	setTacho(num, pulses);
}

static void task(void*) noexcept {
	do {
		for (size_t ind = 0; ind < FAN_COUNT; ++ind) {
			update(ind);
		}
		vTaskDelay(UPDATE_DELAY);
	} while (true);
}

bool fanInit() {
//	apb_freq = rtc_clk_apb_freq_get();
	for (size_t ind = 0; ind < FAN_COUNT; ++ind) {
		init(ind);
	}

	const BaseType_t result = xTaskCreate(&task, "fan-task", STACK_SIZE, nullptr, tskIDLE_PRIORITY, nullptr);
    if (pdPASS!=result)
		fatalError(ESP_FAIL, "Fan task creation %d", static_cast<int>(result));

	return true;
}

void fanStatus() {
	bool isActive = false;
	for (size_t ind = 0; ind < FAN_COUNT; ++ind) {
		const Config &config = Config::get(ind);
		if (!config)
			continue;

		isActive = true;
		const volatile PwmData &data = pwmData[ind];

		std::ostringstream info;

		std::atomic_thread_fence(std::memory_order_acquire);
		const TickType_t timeout = data.timestamp.load(std::memory_order_relaxed) + VALID_PERIOD;
		const unsigned int duration = data.duration.load(std::memory_order_relaxed);
		const unsigned int negative = data.negative.load(std::memory_order_relaxed);
		const unsigned int freq = mcpwm_get_frequency(TACHO_UNIT, tachoTimers[ind]);
		if (GPIO_NUM_NC != config.enSignal) {
			const bool isDisabled = (0 == gpio_get_level(static_cast<gpio_num_t>(config.enSignal))) == config.enActive;
			info<<" EN="<<(isDisabled?"off":"ON");
		}

		if (GPIO_NUM_NC != config.pwm) {
			info<<" PWM=";
			if (xTaskGetTickCount() <= timeout) {
				info<<std::fixed<<std::setprecision(3)<<rtc_clk_apb_freq_get() / static_cast<float>(duration);
			}
			else {
				info<<"timeout";
			}
		}

		info<<" TACHO="<<freq<<" Hz ["<<std::fixed<<std::setprecision(3)<<freq*60.0f/config.ppr<<" rpm]";

		ESP_LOGI(TAG, "#[%zu]%s", ind+1, info.str().c_str());
	}

	if (!isActive) {
		ESP_LOGW(TAG, "All disabled");
	}
}
