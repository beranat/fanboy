#pragma once

class Fan {
private:
	Fan(const Fan&) = delete;
	Fan& operator=(const Fan&) = delete;
	Fan(Fan &&) = delete;
	Fan& operator=(Fan &&) = delete;

public:
	struct Config {
		bool hasEn;
		struct {
			bool active;
			float delay;
		} en;
		std::tuple<float, float> *pwmIn;
		float maxRpm;
		unsigned int ppr;	// pulses per round, 0 no tacho
		struct {
			float delay;	// seconds
			float speed;	// up-% per second
		} spinUp;
		struct {
			float delay;	// seconds
			float speed;	// down-% per second
		} spinDown;
	};

private:
	const Config config_;
	const std::function<void(unsigned int)> setTacho_;

public:
	constexpr Fan() noexcept {
	}

	Fan(const Config &config, const std::function<void(unsigned int)> &setTacho);

	~Fan() noexcept;

	void setEn(bool b) noexcept;
	void setPwm(float f) noexcept;
};
