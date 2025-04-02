#include "band.h"

#include <glibmm.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <string>

namespace /* anonymous */ {

struct BandTester : public BandDevice
{
	explicit BandTester(BandDeviceLL *dev) noexcept
		: count(0), detector(BandDetector::create())
	{
		BandDevice::operator=(dev);
	}

	~BandTester() noexcept = default;
private:
	std::string my_name;
	size_t count;
	std::unique_ptr<BandDetector> detector;

	virtual void device_initialized(const std::string& name, uint64_t ts, const DevIdData& devid) noexcept override
	{
		my_name = name;
		BandDetector::gesture_bitmap_t bm;
		bm.set(BandDetector::PUNCH_LEFT);
		detector->enable_gestures(bm);
	}

	virtual void device_removed() noexcept override
	{
		delete this;
	}

	virtual void data_received(SensorData data) noexcept override;
	void report_punch(const BandDetector::GestureData& data);
};

void BandTester::data_received(SensorData data) noexcept
{
	for (auto& g : detector->process(BandDetector::FOREARM_L, data))
		if (g.type == BandDetector::PUNCH_LEFT)
			report_punch(g);
}

void BandTester::report_punch(const BandDetector::GestureData& data)
{
	uint32_t ts = data.timestamp;
	++count;

	char buf[256], *p = buf, *e = buf + sizeof(buf) - 1;
	*e = 0;
	p += snprintf(p, e - p, "%-15.15s:  ts=%4llu.%06llu  PUNCH %5u = %3.2f @ %+1.3f\n",
		my_name.c_str(),
		ts / 1000000ll, ts % 1000000ll,
		count, data.value.x /* str */, data.value.y /* pitch */);

	std::cout << buf << std::flush;
}

static void handle_new_band(BandDeviceLL *dev)
{
	try {
		new BandTester(dev);
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
