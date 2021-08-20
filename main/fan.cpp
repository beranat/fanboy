#include "fan.hpp"

Fan::Fan(const Config &config, const std::function<void(unsigned int)> &setTacho) : config_(config), setTacho_(setTacho) {
}

Fan::~Fan() noexcept {
	setTacho_(0);
	isEnabled_ = false;
	currentPwm = 0;
}

void Fan::setEn(bool b) noexcept {
	request.enable.store(b, std::memory_order_release);
}

void Fan::setPwm(float f) noexcept {
	request.pwm.store(f, std::memory_order_release);
}
