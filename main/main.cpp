#include <atomic>
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

constexpr mcpwm_capture_on_edge_t MCPWM_BOTH_EDGE = static_cast<mcpwm_capture_on_edge_t>(MCPWM_POS_EDGE | MCPWM_NEG_EDGE);

constexpr const char *APPLICATION = "app";
constexpr const char *FANINIT = "fan-init";

constexpr size_t FAN_COUNT = 3;

struct FanConfig {
	bool isEnabled;
	unsigned int maxSpeed = 0;
	gpio_num_t  enSignal = GPIO_NUM_NC;
	bool		enActive = true;
	gpio_num_t  pwm = GPIO_NUM_NC;
	unsigned int ppr = 0;
	gpio_num_t tacho = GPIO_NUM_NC;
};

constexpr const FanConfig fanConfigs[FAN_COUNT] = {
	{
	#ifdef CONFIG_FAN1_ENABLED
		true,
		CONFIG_FAN1_MAX_ROTATION_SPEED,
		#ifdef CONFIG_FAN1_EN_ENABLED
			static_cast<gpio_num_t>(CONFIG_FAN1_EN_GPIO),
		#else
			GPIO_NUM_NC,
		#endif
		#ifdef CONFIG_FAN1_EN_ACTIVE
			true,
		#else
			false,
		#endif
		#ifdef CONFIG_FAN1_PWM_ENABLED
			static_cast<gpio_num_t>(CONFIG_FAN1_PWM_GPIO),
		#else
			GPIO_NUM_NC,
		#endif
		CONFIG_FAN1_TACHO_PULSE_PER_ROUND,
		static_cast<gpio_num_t>(CONFIG_FAN1_TACHO_GPIO)
	#else
		false
	#endif
	},
	{
	#ifdef CONFIG_FAN2_ENABLED
		true,
		CONFIG_FAN2_MAX_ROTATION_SPEED,
		#ifdef CONFIG_FAN2_EN_ENABLED
			static_cast<gpio_num_t>(CONFIG_FAN2_EN_GPIO),
		#else
			GPIO_NUM_NC,
		#endif
		#ifdef CONFIG_FAN2_EN_ACTIVE
			true,
		#else
			false,
		#endif
		#ifdef CONFIG_FAN2_PWM_ENABLED
			static_cast<gpio_num_t>(CONFIG_FAN2_PWM_GPIO),
		#else
			GPIO_NUM_NC,
		#endif
		CONFIG_FAN2_TACHO_PULSE_PER_ROUND,
		static_cast<gpio_num_t>(CONFIG_FAN2_TACHO_GPIO)
	#else
		false
	#endif
	},
	{
	#ifdef CONFIG_FAN3_ENABLED
		true,
		CONFIG_FAN3_MAX_ROTATION_SPEED,
		#ifdef CONFIG_FAN3_EN_ENABLED
			static_cast<gpio_num_t>(CONFIG_FAN3_EN_GPIO),
		#else
			GPIO_NUM_NC,
		#endif
		#ifdef CONFIG_FAN3_EN_ACTIVE
			true,
		#else
			false,
		#endif
		#ifdef CONFIG_FAN3_PWM_ENABLED
			static_cast<gpio_num_t>(CONFIG_FAN3_PWM_GPIO),
		#else
			GPIO_NUM_NC,
		#endif
		CONFIG_FAN3_TACHO_PULSE_PER_ROUND,
		static_cast<gpio_num_t>(CONFIG_FAN3_TACHO_GPIO)
	#else
		false
	#endif
	}
};

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

#define PWMIN_UNIT MCPWM_UNIT_0
#define TACHO_UNIT MCPWM_UNIT_1

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

constexpr mcpwm_timer_t tachoTimers[FAN_COUNT] = {
	MCPWM_TIMER_0,
	MCPWM_TIMER_1,
	MCPWM_TIMER_2
};

struct {
	std::atomic<uint32_t> freq;
	std::atomic<uint32_t> duty; // 1/65536
	std::atomic<TickType_t> timestamp;
} PWM[FAN_COUNT];

static void IRAM_ATTR isrOnCaptured(unsigned int num) noexcept {
	bool isPositive = (1 == mcpwm_capture_signal_get_edge(PWMIN_UNIT, pwmSelect[num]));
	uint32_t value = mcpwm_capture_signal_get_value(PWMIN_UNIT, pwmSelect[num]);

	static uint32_t prevPos[FAN_COUNT] = { 0 };
	static uint32_t prevNeg[FAN_COUNT] = { 0 };

	if (isPositive) {
		const uint32_t duration = static_cast<uint32_t>(value - prevPos[num]);
		const float freq = duration / static_cast<float>(rtc_clk_apb_freq_get());
		const float duty = static_cast<uint32_t>(prevNeg[num] - prevPos[num]) / static_cast<float>(duration?duration:1);
		prevPos[num] = num;

		PWM[num].freq.store(static_cast<uint32_t>(freq), std::memory_order_relaxed);
		PWM[num].duty.store(static_cast<uint32_t>(duty*std::numeric_limits<uint16_t>::max()), std::memory_order_relaxed);
		PWM[num].timestamp.store(xTaskGetTickCountFromISR(), std::memory_order_relaxed);
		atomic_thread_fence(std::memory_order_release);
	}
	else {
		prevNeg[num] = value;
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

void fanInit(unsigned int num) {
	const FanConfig &fi = fanConfigs[num];

	if (!fi.isEnabled) {
		ESP_LOGI(FANINIT, "[%u] disabled", num+1);
		return;
	}

	// Enable signal
	if (GPIO_NUM_NC != fi.enSignal) {
		ESP_LOGI(FANINIT, "[%u] En %s active", num+1, fi.enActive?"HIGH":"low");
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
		mcpwm_gpio_init(PWMIN_UNIT, pwmSignal[num], fi.pwm);
    	mcpwm_capture_enable(PWMIN_UNIT, pwmSelect[num], MCPWM_BOTH_EDGE, 0);
    	mcpwm_isr_register(PWMIN_UNIT, &isrCaptureHandler, NULL, ESP_INTR_FLAG_IRAM, NULL);
	}

	// TACHO-OUT
	fatalError(mcpwm_gpio_init(TACHO_UNIT, tachoSignals[num], fi.tacho), "FAN[%u] mcpwm_gpio", num+1);
    const mcpwm_config_t config = {
		2, // frequency
		50.0f, 0.0f,	// cmpr a/b
		MCPWM_DUTY_MODE_0,
		MCPWM_UP_COUNTER
	};
	fatalError(mcpwm_init(TACHO_UNIT, tachoTimers[num], &config), "FAN[%u] mcpwm_init", num+1);
	fatalError(mcpwm_set_signal_low(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_A), "FAN[%u] disable A", num+1);
	fatalError(mcpwm_set_signal_low(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_B), "FAN[%u] disable B", num+1);
}

void fanTacho(unsigned int num, unsigned int freq) {
	fatalError(mcpwm_set_signal_low(TACHO_UNIT, tachoTimers[num], MCPWM_OPR_A), "FAN[%u] disable A", num+1);
    if (0 == freq)
		return;
	fatalError(mcpwm_set_frequency(TACHO_UNIT, tachoTimers[num], freq), "FAN[%u] mcpwm_set_freq", num+1);
}

void fanUpdate(unsigned int num) {
	const FanConfig &fi = fanConfigs[num];

	 if (!fi.isEnabled)
		return;

	if (GPIO_NUM_NC != fi.enSignal) {
		bool isDisabled = (0 == gpio_get_level(static_cast<gpio_num_t>(fi.enSignal))) == fi.enActive;
		if (isDisabled) {
			fanTacho(num, 0);
			return;
		}
	}

	float ratio = 1.0f;

	if (GPIO_NUM_NC != fi.pwm) {
		atomic_thread_fence(std::memory_order_acquire);
		const TickType_t ts = PWM[num].timestamp.load(std::memory_order_relaxed);
		const uint32_t duty = PWM[num].duty.load(std::memory_order_relaxed);

		if (xTaskGetTickCount() + pdMS_TO_TICKS(1000) <= ts)
			ratio = static_cast<float>(duty) / std::numeric_limits<uint16_t>::max();
	}

	const unsigned int pulses = static_cast<unsigned int>(fi.maxSpeed * fi.ppr * ratio / 60);
	fanTacho(num, pulses);
}

extern "C" void app_main() {
	ESP_LOGW(APPLICATION, "Init HW");
	initHardware();

	ESP_LOGW(APPLICATION, "Init Fan");
	for (size_t ind = 0; ind < FAN_COUNT; ++ind) {
		fanInit(ind);
	}

	for (bool on = true; true; on = !on) {
		ledEnable(on);
		for (size_t ind = 0; ind < FAN_COUNT; ++ind) {
			fanUpdate(ind);
		}
		vTaskDelay(pdMS_TO_TICKS(250));
	} while (true);

	ESP_LOGE(APPLICATION, "System halted");
	esp_deep_sleep_start();
}
