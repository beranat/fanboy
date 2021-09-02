#include <driver/gpio.h>

struct Config {
	enum class Curve {
		none = 0,
		linear,
		percent5
	};

	bool isEnabled;
	unsigned int maxSpeed = 0;
	gpio_num_t  enSignal = GPIO_NUM_NC;
	bool		enActive = true;
	gpio_num_t  pwm = GPIO_NUM_NC;
	Curve curve = Curve::linear;
	unsigned int ppr = 0;
	gpio_num_t tacho = GPIO_NUM_NC;

	constexpr bool operator!() const noexcept {
		return !static_cast<bool>(*this);
	}

	constexpr operator bool() const noexcept {
		return isEnabled;
	}

	float getPower(const float ratio) noexcept;

	static const Config& get(size_t index) noexcept;
};
