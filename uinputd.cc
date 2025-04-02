#include "band.h"
#include "vecs.h"
#include "uin-dev.h"

#include <glibmm.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#define DEBUG_GVEC 0

namespace /* anonymous */ {

static std::unique_ptr<BandManager> app;

struct BandInputForwarder : public BandDevice
{
	explicit BandInputForwarder(BandDeviceLL *dev);
	~BandInputForwarder();

	bool is_created() const { return indev.is_created(); }
private:
	UInputDevice indev;
	std::unique_ptr<BandCalibrator> cal;
	GVectorTracker gvec;

	static BandInputForwarder *first;
	std::string my_name;
	Glib::Timer timer;
	std::future<void> calib_done;
	unsigned count;

	virtual void device_initialized(const std::string& name, uint64_t ts, const DevIdData& devid) noexcept override;

	virtual void device_removed() noexcept override
	{
		delete this;
	}

	virtual void data_received(SensorData data) noexcept;
};


BandInputForwarder *BandInputForwarder::first = nullptr;


BandInputForwarder::BandInputForwarder(BandDeviceLL *dev)
{
	indev.set_prop(INPUT_PROP_ACCELEROMETER);
	indev.setup_msc_entry(MSC_TIMESTAMP);
	indev.setup_abs_axis(ABS_X, -32767, +32767, 32768/16 /* units/g */);
	indev.setup_abs_axis(ABS_Y, -32767, +32767, 32768/16 /* units/g */);
	indev.setup_abs_axis(ABS_Z, -32767, +32767, 32768/16 /* units/g */);
	indev.setup_abs_axis(ABS_RX, -32767, +32767, 32768/2048 /* units/deg/s */);
	indev.setup_abs_axis(ABS_RY, -32767, +32767, 32768/2048 /* units/deg/s */);
	indev.setup_abs_axis(ABS_RZ, -32767, +32767, 32768/2048 /* units/deg/s */);
	indev.setup_abs_axis(ABS_MISC, 0, 0x7F, 0);

	BandDevice::operator=(dev);
}

BandInputForwarder::~BandInputForwarder()
{
	if (first == this)
		first = nullptr;
}

void BandInputForwarder::device_initialized(const std::string& name, uint64_t ts, const DevIdData& id) noexcept
{
	try {
		uint16_t bus;

		switch (id.registry) {
			case 1: bus = BUS_USB; break;
			case 2: bus = BUS_BLUETOOTH; break;
			default: bus = BUS_VIRTUAL; break;
		}

		indev.setup_dev(name.c_str(), 0, bus, id.vendor, id.product, id.version);
		std::cout << "inputdev: " << indev.create() << " for " << name << '\n';

		my_name = name;
		timer.reset();
		count = 0;
		cal = BandCalibrator::create();
	} catch (...) {
		std::cerr << "create() for " << name << " failed\n";
		delete this;
	}
}

void BandInputForwarder::data_received(SensorData data) noexcept
{
	if (calib_done.valid() && calib_done.wait_for(std::chrono::seconds::zero()) == std::future_status::ready) {
		try {
			calib_done.get();
			cal.reset();
		} catch (...) {
			std::cerr << "adjust_zero() for " << my_name << " failed\n";
			// reset just in case
			cal = BandCalibrator::create();
		}
	}

	if (cal) {
		bool done = cal->process(data);
		indev.emit(EV_ABS, ABS_MISC, cal->calibration_needed_mask());
		if (done && !calib_done.valid())
			calib_done = adjust_zero(cal->zero_offset());
	}

	indev.emit(EV_MSC, MSC_TIMESTAMP, data.timestamp);
	indev.emit(EV_ABS, ABS_X, data.v.ax);
	indev.emit(EV_ABS, ABS_Y, data.v.ay);
	indev.emit(EV_ABS, ABS_Z, data.v.az);
	indev.emit(EV_ABS, ABS_RX, data.v.gx);
	indev.emit(EV_ABS, ABS_RY, data.v.gy);
	indev.emit(EV_ABS, ABS_RZ, data.v.gz);
	indev.emit(EV_SYN, SYN_REPORT, 0);
	++count;

	if (DEBUG_GVEC) {
		gvec.update(data.timestamp, Vec<int, 3>{data.v.gx, data.v.gy, data.v.gz}, Vec<int, 3>{data.v.ax, data.v.ay, data.v.az});

		Vec<double, 3> g = gvec;
		char buf[256];
		snprintf(buf, sizeof(buf)-1, "%-15.15s: g= %+5.3f %+5.3f %+5.3f\n",
			my_name.c_str(), g[0], g[1], g[2]);
		std::cerr << buf;
	}

	double te = timer.elapsed();
	if (te >= 3) {
		if (!first)
			first = this;

		bool me_first = first == this;
		char statbuf[256];

		snprintf(statbuf, sizeof(statbuf)-1, "%s%-15.15s: %4.2f eps%s%s\t",
			me_first ? "\r" : "", my_name.c_str(), count / te,
			cal ? "\t" : "", cal ? cal->status().c_str() : "");
		std::cerr << statbuf;

		timer.reset();
		count = 0;
	}
}

static void handle_new_band(BandDeviceLL *dev)
{
	try {
		new BandInputForwarder(dev);
	} catch (...) {
		/* oh, well */
	}
}

} /* anonymous namespace */

int main(int argc, char **argv)
{
	Glib::init();

	app = BandManager::create();

	app->on_new_band = &handle_new_band;
	app->run();
}
