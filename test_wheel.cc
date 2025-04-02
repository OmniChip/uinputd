#include "band.h"
#include "vecs.h"

#include <glibmm.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <string>

namespace /* anonymous */ {

struct TimesPair
{
	uint64_t ts;
	uint64_t us;

	TimesPair() : ts(0), us(0) {}

	TimesPair(uint64_t ts_)
		: ts(ts_), us(sys_usecs())
	{
	}

	static uint64_t sys_usecs() noexcept
	{
		struct timespec tv;

		if (!clock_gettime(CLOCK_MONOTONIC, &tv))
			return tv.tv_sec * 1000000 + tv.tv_nsec / 1000;
		else
			return 0;
	}
};

struct WheelDetector
{
	std::unique_ptr<BandDetector> detector;
	
	DataRing<TimesPair, 128> left, right;

	int64_t ts_diff() const
	{
		return right.newest().ts - left.newest().ts;
	}

	template <size_t N>
	static float s_freq(const DataRing<TimesPair, N>& w)
	{
		return N * 1000000.0 / (w.newest().ts - w.oldest().ts);
	}

	float left_s_freq() const { return s_freq(left); }
	float right_s_freq() const { return s_freq(right); }

	template <size_t N>
	static float p_freq(const DataRing<TimesPair, N>& w)
	{
		return N * 1000000.0 / (w.newest().us - w.oldest().us);
	}

	float left_p_freq() const { return p_freq(left); }
	float right_p_freq() const { return p_freq(right); }

	template <size_t N>
	static float freq_dev(const DataRing<TimesPair, N>& w)
	{
		return (double)(w.newest().ts - w.oldest().ts) / (w.newest().us - w.oldest().us);
	}

	float left_freq_dev() const { return freq_dev(left); }
	float right_freq_dev() const { return freq_dev(right); }

	WheelDetector()
		: detector(BandDetector::create())
       	{
		BandDetector::gesture_bitmap_t bm;
		bm.set(BandDetector::WHEEL);
		detector->enable_gestures(bm);
	}

	std::list<BandDetector::GestureData> process(BandDetector::BandLocation loc, const BandDevice::SensorData& data)
	{
		TimesPair tp{data.timestamp};

		switch (loc) {
		case BandDetector::FOREARM_L:
			left.push(tp);
			break;

		case BandDetector::FOREARM_R:
			right.push(tp);
			break;
		}

		return detector->process(loc, data);
	}
};

struct WheelTester : public BandDevice
{
	explicit WheelTester(BandDeviceLL *dev, WheelDetector& det) noexcept
		: detector(det), which(BandDetector::TORSO), valid(false)
	{
		BandDevice::operator=(dev);
	}

	~WheelTester() noexcept = default;
private:
	WheelDetector& detector;
	BandDetector::BandLocation which;
	bool valid;

	virtual void device_initialized(const std::string& name, uint64_t ts, const DevIdData& devid) noexcept override;
	virtual void device_removed() noexcept override;
	virtual void data_received(SensorData data) noexcept;
};

void WheelTester::device_initialized(const std::string& name, uint64_t ts, const DevIdData& devid) noexcept
{
	if (name.size() < 3)
		return;
	switch (name.back() - '0') {
	case 1:
		which = BandDetector::FOREARM_L;
		valid = true;
		break;
	case 2:
		which = BandDetector::FOREARM_R;
		valid = true;
		break;
	default:
		which = BandDetector::TORSO;
		valid = false;
		break;
	}
}

void WheelTester::data_received(SensorData data) noexcept
{
	if (!valid)
		return;

	auto glist = detector.process(which, data);

	int64_t ts_diff = detector.ts_diff();
	bool neg = ts_diff < 0;
	ts_diff = neg ? -ts_diff : ts_diff;

	for (auto& g : glist) {
		if (g.type != BandDetector::WHEEL)
			continue;
		uint64_t ts = g.timestamp;
		float value = g.value.x;

		char buf[256], *p = buf, *e = buf + sizeof(buf) - 1;
		*e = 0;
		p += snprintf(p, e - p, "\rW=%+1.4f   @%4u.%06u   LtR-ts-diff %c%d.%06u   Lf=%1.5f Rf=%1.5f  fs=%2.2f,%2.2f  pps=%2.2f,%2.2f       ",
			      value, ts / 1000000, ts % 1000000,
			      neg ? '-' : '+', ts_diff / 1000000, ts_diff % 1000000,
			      detector.left_freq_dev(), detector.right_freq_dev(),
			      detector.left_s_freq(), detector.right_s_freq(),
			      detector.left_p_freq(), detector.right_p_freq());
		std::cout << buf << std::flush;
	}
}

void WheelTester::device_removed() noexcept
{
	delete this;
}

static void handle_new_band(BandDeviceLL *dev)
{
	static WheelDetector detector;

	try {
		new WheelTester(dev, detector);
	} catch (...) {
		/* oh, well */
	}
}

} /* anonymous namespace */

int main(int argc, char **argv)
{
	Glib::init();

	auto app = BandManager::create();
	app->on_new_band = &handle_new_band;
	app->run();
	return 0;
}
