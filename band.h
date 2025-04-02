#ifndef GAMEINN__BAND_H_
#define GAMEINN__BAND_H_

#include <array>
#include <bitset>
#include <functional>
#include <future>
#include <memory>
#include <list>
#include <vector>

struct BandDeviceLL;

struct __attribute__((visibility ("default"))) BandDevice
{
	friend class BandDeviceLL;
	friend class BandManager;

	struct DevIdData
	{
		uint16_t registry;
		uint16_t vendor;
		uint16_t product;
		uint16_t version;
	};

	struct AxisData
	{
		int gx, gy, gz;
		int ax, ay, az;
	};

	struct SensorData
	{
		uint64_t timestamp;
		AxisData v;
	};

#ifndef __ANDROID__
	BandDevice() noexcept : dev(nullptr) {}
	BandDevice(const BandDevice&) = delete;
	BandDevice(BandDevice&&) = delete;
	virtual ~BandDevice() noexcept;

	std::future<void> adjust_zero(const AxisData& zero);
	std::future<void> send_vibe(uint64_t effect, uint32_t when = 0);
protected:
	BandDevice& operator=(BandDeviceLL *ll) noexcept;
private:
	BandDeviceLL *dev;

	virtual void device_initialized(const std::string& name, uint64_t ts, const DevIdData& devid) noexcept;
	virtual void device_removed() noexcept = 0;

	virtual void data_received(SensorData data) noexcept = 0;
#endif /* !__ANDROID__ */
};

#ifndef __ANDROID__

struct __attribute__((visibility ("default"))) BandManager
{
	std::function<void (BandDeviceLL *)> on_new_band;

	virtual ~BandManager() = default;

	virtual std::vector<BandDeviceLL *> bands() = 0;

	virtual void run() = 0;
	virtual void stop() = 0;

	virtual std::future<void> adjust_zero(BandDeviceLL *dev, const BandDevice::AxisData& zero) = 0;
	virtual std::future<void> send_vibe(BandDeviceLL *dev, uint64_t effect, uint32_t when = 0) = 0;

	static std::unique_ptr<BandManager> create();
};

#endif /* !__ANDROID__ */

struct __attribute__((visibility ("default"))) BandCalibrator
{
	virtual ~BandCalibrator() = default;

	virtual bool process(BandDevice::SensorData& input) = 0;

	virtual BandDevice::AxisData zero_offset() const = 0;
	virtual void zero_applied(const BandDevice::AxisData& offset) = 0;

	virtual bool is_done() const = 0;
	virtual std::string status() const = 0;
	virtual uint8_t calibration_needed_mask() const = 0;

	static std::unique_ptr<BandCalibrator> create();
};

struct __attribute__((visibility ("default"))) BandDetector
{
	enum BandLocation {
		FOREARM_L,
		FOREARM_R,
		CALF_L,
		CALF_R,
		TORSO,
		__NUM_LOCATIONS
	};

	enum Gesture {
		NONE,
		POINT_FL_DOWN,
		POINT_FL_LOW,
		POINT_FL_LEVEL,
		POINT_FL_HIGH,
		POINT_FL_UP,
		POINT_FR_DOWN,
		POINT_FR_LOW,
		POINT_FR_LEVEL,
		POINT_FR_HIGH,
		POINT_FR_UP,
		POINT_ARM_DOWN,
		POINT_ARM_LOW,
		POINT_ARM_LEVEL,
		POINT_ARM_HIGH,
		POINT_ARM_UP,
		POINT_CL_DOWN,
		POINT_CL_LOW,
		POINT_CL_LEVEL,
		POINT_CR_DOWN,
		POINT_CR_LOW,
		POINT_CR_LEVEL,
		POINT_LEG_DOWN,
		POINT_LEG_LOW,
		POINT_LEG_LEVEL,
		SQUAT,
		WHEEL,		// detector enable
		GUARD_UP,
		GUARD_DOWN,
		PUNCH,
		PUNCH_LOW,
		PUNCH_LEVEL,
		PUNCH_HIGH,
		PUNCH_LEFT,
		PUNCH_LEFT_LOW,
		PUNCH_LEFT_LEVEL,
		PUNCH_LEFT_HIGH,
		PUNCH_RIGHT,
		PUNCH_RIGHT_LOW,
		PUNCH_RIGHT_LEVEL,
		PUNCH_RIGHT_HIGH,
		__NUM_GESTURES
	};

	struct GestureData
	{
		uint64_t timestamp;
		Gesture type;
		struct {
			float x, y, z;
		} value;

		GestureData(Gesture type_ = NONE, uint64_t ts = 0)
			: timestamp(ts), type(type_), value{0, 0, 0} {}
	};

	using gesture_bitmap_t = std::bitset<__NUM_GESTURES>;

	virtual ~BandDetector() = default;

	virtual gesture_bitmap_t get_gestures(unsigned band_mask) const noexcept = 0;
	virtual void enable_gestures(const gesture_bitmap_t& mask) = 0;
	virtual std::list<GestureData> process(BandLocation which, const BandDevice::SensorData& input) = 0;
	virtual void reset() = 0;

	static std::unique_ptr<BandDetector> create();
};

#endif /* GAMEINN__BAND_H_ */
