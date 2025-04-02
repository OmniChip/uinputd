#include "band.h"
#include "vecs.h"
#include "uin-dev.h"

#include <glibmm.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <string>

namespace /* anonymous */ {

struct BandDataLogger : public BandDevice
{
	explicit BandDataLogger(BandDeviceLL *dev) noexcept
		: count(0), prev_ts(0)
	{
		BandDevice::operator=(dev);
	}

	~BandDataLogger() noexcept = default;

	enum LogFormat {
		FMT_TXT,
		FMT_CSV,
	};

	static LogFormat format;
private:
	std::string my_name;
	Glib::Timer timer;
	size_t count;
	uint32_t prev_ts;

	virtual void device_initialized(const std::string& name, uint64_t ts, const DevIdData& devid) noexcept override
	{
		my_name = name;
		timer.reset();
	}

	virtual void device_removed() noexcept override
	{
		delete this;
	}

	virtual void data_received(SensorData data) noexcept;
};

BandDataLogger::LogFormat BandDataLogger::format;

void BandDataLogger::data_received(SensorData data) noexcept
{
	static const char *const fmt[] = {
		"%-15.15s:  a= %+5d %+5d %+5d  g= %+5d %+5d %+5d  ts=%4u.%06u  dt=%1u.%06u  f=%3.2f",
		"\"%s\",%d,%d,%d,%d,%d,%d,%1u.%06u,%1u.%06u,%3.2f",
	};

	uint32_t ts = data.timestamp;
	uint32_t dt = ts - prev_ts;

	++count;
	prev_ts = ts;

	char buf[256], *p = buf, *e = buf + sizeof(buf) - 1;
	*e = 0;
	p += snprintf(p, e - p, fmt[format],
		my_name.c_str(),
		data.v.ax, data.v.ay, data.v.az,
		data.v.gx, data.v.gy, data.v.gz,
		ts / 1000000, ts % 1000000,
		dt / 1000000, dt % 1000000,
		1000000.0 / dt);

	if (format == FMT_TXT) {
		double te = timer.elapsed();
		if (te >= 3) {
			p += snprintf(p, e - p, "  (%4.2f eps)", count / te);

			timer.reset();
			count = 0;
		}
	}

	std::cout << buf << std::endl;
}

static void handle_new_band(BandDeviceLL *dev)
{
	try {
		new BandDataLogger(dev);
	} catch (...) {
		/* oh, well */
	}
}

} /* anonymous namespace */

static const std::string fmt_name[] = { "txt", "csv" };
static constexpr size_t n_fmts = sizeof(fmt_name) / sizeof(*fmt_name);

int main(int argc, char **argv)
{
	Glib::init();

	if (argc > 1) {
		std::string arg = argv[1];
		size_t i;

		for (i = 0; i < n_fmts; ++i)
			if (fmt_name[i] == arg)
				break;

		if (i >= n_fmts) {
			std::cerr << "unknown format: " << arg << '\n';
			return 1;
		}
		BandDataLogger::format = (BandDataLogger::LogFormat)i;
	}

	auto app = BandManager::create();
	app->on_new_band = &handle_new_band;
	app->run();
	return 0;
}
