#include <cassert>
#include <algorithm>

#include "config.hpp"

static constexpr const Config disabled = {
	false
};

static constexpr const Config configs[3] = {
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
		#if defined(FAN1_PWM_CURVE_LINEAR)
			Config::Curve::linear,
		#elif defined(FAN1_PWM_CURVE_5)
			Config::Curve::percent5,
		#else
			Config::Curve::none,
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
		#if defined(FAN2_PWM_CURVE_LINEAR)
			Config::Curve::linear,
		#elif defined(FAN2_PWM_CURVE_5)
			Config::Curve::percent5,
		#else
			Config::Curve::none,
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
		#if defined(FAN3_PWM_CURVE_LINEAR)
			Config::Curve::linear,
		#elif defined(FAN3_PWM_CURVE_5)
			Config::Curve::percent5,
		#else
			Config::Curve::none,
		#endif
		CONFIG_FAN3_TACHO_PULSE_PER_ROUND,
		static_cast<gpio_num_t>(CONFIG_FAN3_TACHO_GPIO)
	#else
		false
	#endif
	}
};

float Config::getPower(const float ratio) noexcept {
	switch (curve) {
		case Curve::none:
			return 1.0f;
		case Curve::linear:
			return std::max(0.0f, std::min(1.0f, ratio));
		case Curve::percent5:
			if (ratio <= 0.05f)
				return 1.0f;
			return std::max(0.0f, std::min(1.0f, ratio));
	}

	return 1.0f;
}

const Config& Config::get(size_t index) noexcept {
	if (index <= sizeof(configs)/sizeof(*configs))
		return configs[index];

	assert(false);
	return disabled;
}
