#include "band.h"

#include <glibmm.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#define DEBUG_GVEC 0

namespace /* anonymous */ {

static std::unique_ptr<BandManager> app;

struct Calibrator : public BandDevice
{
	explicit Calibrator(BandDeviceLL *dev) : cal(BandCalibrator::create()), already_done(false) { BandDevice::operator=(dev); }
	~Calibrator() = default;
private:
	std::unique_ptr<BandCalibrator> cal;
	bool already_done;

	virtual void device_initialized(const std::string& name, uint64_t ts, const DevIdData& devid) noexcept override {}

	virtual void device_removed() noexcept override
	{
		delete this;
	}

	virtual void data_received(SensorData data) noexcept;
};

void Calibrator::data_received(SensorData data) noexcept
{
	bool done = cal->process(data);
	if (done && !already_done) {
		already_done = true;
		adjust_zero(cal->zero_offset());
	}
}

static void handle_new_band(BandDeviceLL *dev)
{
	try {
		new Calibrator(dev);
	} catch (...) {
		/* oh, well */
	}
}

static BandDevice::AxisData& operator-=(BandDevice::AxisData& s, const BandDevice::AxisData& v)
{
	for (size_t i = 0; i < 6; ++i)
		*(&s.gx + i) -= *(&v.gx + i);

	return s;
}

static void dry_calib(const std::string& prefix, const BandDevice::AxisData& ozero)
{
	std::string line;

	auto cal = BandCalibrator::create();
	size_t i = 0;

	while (getline(std::cin, line)) {
		++i;
		if (line.substr(0, prefix.size()) != prefix)
			continue;
		line.erase(0, prefix.size());

		for (char& c : line)
			if (c == ',')
				c = ' ';

		BandDevice::SensorData data;
		data.timestamp = i;
		std::istringstream(line) >> data.v.ax >> data.v.ay >> data.v.az >> data.v.gx >> data.v.gy >> data.v.gz;

		data.v -= ozero;
		cal->process(data);
	}

	if (cal->is_done()) {
		BandDevice::AxisData zero = cal->zero_offset();
		std::cout << "DONE: G= " << zero.gx << ' ' << zero.gy << ' ' << zero.gz
			  << "   A= " << zero.ax << ' ' << zero.ay << ' ' << zero.az << std::endl;
	} else {
		std::cout << "UNFINISHED: " << cal->status() << std::endl;
	}
}

} /* anonymous namespace */

int main(int argc, char **argv)
{
	Glib::init();

	if (argc == 1) {
		app = BandManager::create();

		app->on_new_band = &handle_new_band;
		app->run();
	} else {
		--argc, ++argv;

		std::string prefix = *argv++;
		prefix = '"' + prefix + "\",";
		--argc;

		BandDevice::AxisData zero = { 0, };

		for (size_t i = 0; i < 6; ++i) {
			if (!argc--)
				break;
			std::istringstream(*argv++) >> *(&zero.gx + i);
		}

		dry_calib(prefix, zero);
	}
}
