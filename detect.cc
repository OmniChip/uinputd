#include "band.h"
#include "vecs.h"

#include <array>
#include <cmath>
#include <complex>
#include <deque>
#include <iostream>
#include <iomanip>

namespace {

static constexpr bool DEBUG_SQUAT = false;
static constexpr bool DEBUG_PUNCH = false;

static constexpr size_t MAX_PKT_JITTER = 20;		// samples

static constexpr double VMAG_1G = 1000000.0 / 488;
static constexpr double ROT_360 = 360000.0 / 70;
static constexpr int STABLE_ROT = 200;
static constexpr double STABLE_FORCE_DIFF = 0.15;
static constexpr float POINT_DIR_ACCURACY = 0.15f;

static constexpr int UPDOWN_THRESHOLD = 200;
static constexpr int UPDOWN_HIST = 100;
static constexpr int UPDOWN_SIDE_ROT_MAX = 400;
static constexpr int UPDOWN_TIME_RATIO_MIN = 700;
static constexpr int UPDOWN_TIME_RATIO_DENOM = 1024;
static constexpr int HEIGHT_YROT_FACTOR = 50000;
static constexpr uint64_t UPDOWN_RESET_TIME = 500000;	// us
static constexpr uint64_t SQUAT_TIME_MIN = 500000;	// us
static constexpr double SQUAT_POS_MAX = 30;
static constexpr double SQUAT_SPD_MAX = 25;
static constexpr double SQUAT_LOW_POS_MAX = -100;

using SensorData = BandDevice::SensorData;
using Gesture = BandDetector::Gesture;
using BandLocation = BandDetector::BandLocation;
using GestureData = BandDetector::GestureData;
using gesture_bitmap_t = BandDetector::gesture_bitmap_t;

static constexpr unsigned BAND_LOC_MASK = (1 << BandLocation::__NUM_LOCATIONS) - 1;

template <BandLocation L>
static constexpr bool band_enabled(unsigned mask) { return mask & (1 << (int)L); }

/* wskazanie jednego z pięciu kierunków względem ziemi */

struct PointDirectionDetector
{
	std::array<int, 4> last_dir;
	gesture_bitmap_t enabled;
	unsigned b_mask;

	PointDirectionDetector()
		: b_mask(0)
	{
		reset();
	}

	template <BandLocation LOC, bool Either>
	static constexpr Gesture offset(int i) noexcept;

	template <BandLocation LOC>
	void gestures(std::list<GestureData>& det, uint64_t ts, int i)
	{
		GestureData g;
		g.timestamp = ts;
		g.type = offset<LOC, false>(i);
		if (enabled.test(g.type))
			det.push_back(g);
		g.type = offset<LOC, true>(i);
		if (enabled.test(g.type))
			det.push_back(g);
	}

	void update_enabled(const gesture_bitmap_t& mask) noexcept;

	void reset()
	{
		last_dir.fill(-1);
	}

	static bool detect_dir_range(const SensorData& data, float& value) noexcept
	{
		Vec<int, 3> rot{data.v.gx, data.v.gy, data.v.gz};

		if (ANY(abs(rot) > STABLE_ROT))
			return false;

		double fa = std::abs(std::hypot(data.v.ax, data.v.ay, data.v.az) / VMAG_1G) - 1;
//		std::cerr << "--- fa = " << fa << '\n';
		if (abs(fa) > STABLE_FORCE_DIFF)
			return false;

		value = atan2(-data.v.ax, std::hypot(data.v.ay, data.v.az)) * 2 / M_PI;	// 1 is up, -1 is down
		return true;
	}

	static bool detect_dir(const SensorData& data, int& dir) noexcept
	{
		float value;

		if (!detect_dir_range(data, value))
			return false;

		return detect_dir(value, dir);
	}

	static bool detect_dir(float value, int& dir) noexcept
	{
		// round to nearest i/2: [-1,+1] -> [0,4]
		float vdir = std::floor((value + 1.25f) * 2.0f);	// round to nearest n/2

		float diff = (vdir - 2.0f) / 2.0f - value;
		if (std::abs(diff) > POINT_DIR_ACCURACY)
			return false;

		dir = vdir;
		return true;
	}

	static bool dir_detected(const SensorData& data, int wanted_dir) noexcept
	{
		int dir;
		return detect_dir(data, dir) && dir == wanted_dir;
	}

	std::list<GestureData> process(BandLocation which, const SensorData& data)
	{
		std::list<GestureData> detected;

		if (which >= BandLocation::TORSO)
			return detected;

		int& cur_dir = last_dir[which];
		int dir;

		if (!detect_dir(data, dir))
			return detected;

		if (which >= BandLocation::CALF_L && dir > 2)	// calf: max = LEVEL
			dir = 2;

		if (dir == cur_dir)
			return detected;

		cur_dir = dir;
//		std::cerr << "--> " << id << " dir " << dir << " (diff " << diff << ") ts " << data.timestamp << "\n";

		switch (which) {
		case BandLocation::FOREARM_L: gestures<BandLocation::FOREARM_L>(detected, data.timestamp, dir); break;
		case BandLocation::FOREARM_R: gestures<BandLocation::FOREARM_R>(detected, data.timestamp, dir); break;
		case BandLocation::CALF_L: gestures<BandLocation::CALF_L>(detected, data.timestamp, dir); break;
		case BandLocation::CALF_R: gestures<BandLocation::CALF_R>(detected, data.timestamp, dir); break;
		default: __builtin_unreachable();
		}

		return detected;
	}
};


/* detekcja rzutów / uderzeń */

template <BandLocation LOC>
struct PunchDetector
{
	enum State {
		DISABLED,
		FIND_START,
		FIND_DMIN,
		FIND_END,
		ACQUIRE_DIRECTION,
	};

	static constexpr float PUNCH_MIN_X_RATIO = 0.2f;
	static constexpr float PUNCH_MIN_XRATIO_DIFF = 0.3f;
	static constexpr float PUNCH_XRATIO_DIFF_HIST = 0.2f;
	static constexpr uint64_t PUNCH_DIR_TIME_US = 300000;
	static constexpr uint64_t PUNCH_TIME_US = 200000;



	WindowedStats<float, float, 4> dirs;
	DataWindow<SensorData, 64> decel;
	gesture_bitmap_t enabled;
	float prev_xval, max_dxval;
	uint64_t timeout;
	float min_dir, min_var;
	State state;

	PunchDetector()
		: prev_xval(0), max_dxval(0), timeout(0), min_dir(0), min_var(0), state(DISABLED)
	{
	}

	template <bool Either>
	static constexpr Gesture offset(int i) noexcept;

	void gestures(std::list<GestureData>& det, uint64_t ts, float pitch, float value)
	{
		GestureData g(Gesture::NONE, ts);
		g.value.x = value;
		g.value.y = pitch;

		g.type = offset<false>(0);
		if (enabled.test(g.type))
			det.push_back(g);

		g.type = offset<true>(0);
		if (enabled.test(g.type))
			det.push_back(g);

		int dir;
		if (!PointDirectionDetector::detect_dir(pitch, dir))
			return;

		if (dir < 1 || dir > 3)
			return;

		g.type = offset<false>(dir);
		if (enabled.test(g.type))
			det.push_back(g);

		g.type = offset<true>(dir);
		if (enabled.test(g.type))
			det.push_back(g);
	}

	void update_enabled(const gesture_bitmap_t& mask) noexcept;

	void reset()
	{
		state = enabled.any() ? FIND_START : DISABLED;
		prev_xval = 0;
		max_dxval = PUNCH_MIN_XRATIO_DIFF;
		decel.clear();
	}

	std::list<GestureData> process(BandLocation which, const SensorData& data) noexcept
	{
		std::list<GestureData> detected;

		if (which != LOC)
			return detected;

		float next_xval = (float)data.v.ax / std::hypot(data.v.ax, data.v.ay, data.v.az);
		float dxval = next_xval - prev_xval;
		prev_xval = next_xval;

		float dir = 0, strength = 0;
		uint64_t ts = 0;

		switch (state) {
		case DISABLED:
			break;

		case FIND_START:
			if (next_xval < PUNCH_MIN_X_RATIO) {
				reset();
				break;
			}

			if (dxval > max_dxval) {
				decel.clear();
				max_dxval = dxval;
			} else if (!decel.size())
				break;

			decel.push(data);
			if (decel.size() > 1 && dxval < 0) {
				state = FIND_DMIN;
				timeout = data.timestamp + PUNCH_TIME_US;
			}

			break;

		case FIND_DMIN:
			decel.push(data);

			if (dxval < -PUNCH_XRATIO_DIFF_HIST)
				state = FIND_END;
			else if (data.timestamp > timeout) {
				reset();
				break;
			}
			break;

		case FIND_END:
			if (dxval < PUNCH_XRATIO_DIFF_HIST) {
				if (data.timestamp < timeout)
					decel.push(data);
				else
					reset();
				break;
			}

			state = ACQUIRE_DIRECTION;
			timeout = data.timestamp + PUNCH_DIR_TIME_US;
			dirs.clear();
			min_var = 10;
			break;

		case ACQUIRE_DIRECTION:
			if (PointDirectionDetector::detect_dir_range(data, dir)) {
				dirs.add(dir);

				float v = dirs.var();
				if (v < min_var) {
					min_var = v;
					min_dir = dirs.mean();
				}
			}

			if (data.timestamp < timeout)
				break;

			std::tie(ts, strength) = calc_when_and_strength();
			if (strength > 0)
				gestures(detected, ts, min_dir, strength);
			reset();
			break;
		}

		if (DEBUG_PUNCH && state != DISABLED)
			std::cerr << 'B' << LOC << ": state " << state << " det " << detected.size() << ": @"
				  << data.timestamp / 1000000ull << '.' << std::setfill('0') << std::setw(6) << data.timestamp % 1000000
				  << std::setfill(' ') << ' ' << data.v.ax / VMAG_1G
				  << " @ " << dir << " , " << strength
				  << "  dir " << dirs.mean() << " -- " << dirs.var() << std::endl;
		return detected;
	}

	std::pair<uint64_t, float> calc_when_and_strength() const
	{
		// assert(decel.size() >= 1);

		uint64_t min_dt = 0;
		uint64_t prev_ts = decel.front().timestamp;

		for (const auto& data : decel) {
			if (!min_dt /* first */) {
				min_dt = ~0ull;
				continue;
			}

			if (data.timestamp <= prev_ts)	/* bug? time step? */
				return { 0, 0 };

			if (min_dt > data.timestamp - prev_ts)
				min_dt = data.timestamp - prev_ts;

			prev_ts = data.timestamp;
		}

		int ax_max = -100000;
		uint64_t ts = 0;
		float strength = 0;
		size_t count = 0;
		prev_ts = decel.front().timestamp;

		for (const auto& data : decel) {
			ssize_t n = (data.timestamp - prev_ts + min_dt / 2) / min_dt;
			n += !n;	// first
			prev_ts = data.timestamp;

			strength += n * data.v.ax / VMAG_1G;
			count += n;

			if (ax_max < data.v.ax) {
				ts = data.timestamp;
				ax_max = data.v.ax;
			}
		}

		if (DEBUG_PUNCH)
		std::cerr << 'B' << LOC << ": strength " << strength << " / " << count << " @ "
			  << ts / 1000000ull << '.' << std::setfill('0') << std::setw(6) << ts % 1000000 << std::setfill(' ')
			  << "  min(dt)= " << min_dt << "  n= " << decel.size()
			  << std::endl;

		return { ts, strength };
	}
};


/* detekcja przysiadów */

struct GroundAccelDetector
{
	uint64_t prev_ts;
	uint64_t last_ud_ts;
	uint64_t ud_time;
	int updown;

	GroundAccelDetector() : prev_ts(0), last_ud_ts(0), ud_time(0), updown(0) {}

	void reset() noexcept
	{
		if (DEBUG_SQUAT && ud_time)
			std::cerr << "UD: reset after time " << ud_time << std::endl;
		updown = 0;
		ud_time = 0;
	}

	int update(uint64_t ts, double maga, int gx, int gz) noexcept
	{
		int thup = UPDOWN_THRESHOLD - (updown > 0 ? UPDOWN_HIST : 0);
		int thdn = -UPDOWN_THRESHOLD + (updown < 0 ? UPDOWN_HIST : 0);

		if (gx * gx + gz * gz > UPDOWN_SIDE_ROT_MAX * UPDOWN_SIDE_ROT_MAX)
			updown = 0;
		else if (maga > thup)
			updown = 1;
		else if (maga < thdn)
			updown = -1;
		else
			updown = 0;

		if (updown) {
			if (prev_ts)
				ud_time += ts - prev_ts;
			last_ud_ts = ts;
			if (DEBUG_SQUAT)
				std::cerr << "UD: " << updown << "  time " << ud_time << std::endl;
		}

		prev_ts = ts;
		return updown;
	}
};

struct HeightDetector
{
	uint64_t prev_ts;
	double prev_ma;
	bool got_prev;
	double pos, spd, min_pos;

	HeightDetector() : prev_ts(0), prev_ma(0), got_prev(false), pos(0), spd(0), min_pos(0) {}

	void reset() noexcept
	{
		min_pos = pos = spd = 0;
	}

	void update(uint64_t ts, double ma) noexcept
	{
		if (got_prev) {
			double dt = (ts - prev_ts) / 1000000.0;
			pos += dt * dt / 6 * (prev_ma + 2 * ma) + dt * spd;
			spd += dt * (prev_ma + ma) / 2;
			if (pos < min_pos)
				min_pos = pos;
		}

		prev_ts = ts;
		prev_ma = ma;
		got_prev = true;
	}
};

struct SquatDetector
{
	GroundAccelDetector updown;
	HeightDetector height;

	uint64_t ref_ts;
	bool enabled;

	SquatDetector() : ref_ts(0), enabled(false) {}

	void update_enabled(const gesture_bitmap_t& mask) noexcept;

	void reset() noexcept
	{
		updown.reset();
		height.reset();
	}

	std::list<GestureData> process(BandLocation which, const SensorData& data)
	{
		std::list<GestureData> detected;

		if (!enabled || which != BandLocation::TORSO)
			return detected;

		int xcorr = data.v.gy * data.v.gy / HEIGHT_YROT_FACTOR;
		double ma = std::hypot(data.v.ax + xcorr, data.v.ay, data.v.az) - VMAG_1G;
		updown.update(data.timestamp, ma, data.v.gx, data.v.gz);
		height.update(data.timestamp, ma);

		if (updown.last_ud_ts < data.timestamp - UPDOWN_RESET_TIME) {
			ref_ts = data.timestamp;
			reset();
			return detected;
		} else if (!ref_ts)
			ref_ts = data.timestamp;

		if (!check_squat_end(data.timestamp - ref_ts))
			return detected;

		ref_ts = data.timestamp;
		height.min_pos = height.pos;
		detected.emplace(detected.end(), Gesture::SQUAT, data.timestamp);
		return detected;
	}

	bool check_squat_end(uint64_t dt) noexcept
	{
		if (DEBUG_SQUAT)
			std::cerr << "SQ?  " << dt << " /  " << updown.ud_time << "   hmin " << height.min_pos
				  << "  pos " << height.pos << "  spd " << height.spd << std::endl;
		if (dt < SQUAT_TIME_MIN)
			return false;
		if (updown.ud_time < dt * UPDOWN_TIME_RATIO_MIN / UPDOWN_TIME_RATIO_DENOM)
			return false;
		if (height.min_pos > SQUAT_LOW_POS_MAX)
			return false;
		if (std::abs(height.pos) > SQUAT_POS_MAX)
			return false;
		if (std::abs(height.spd) > SQUAT_SPD_MAX)
			return false;
		return true;
	}
};

/* detekcja kątów Eulera */

struct EulerDirectionDetector
{
	using angles_t = Vec<float, 3>;	// roll, pich, yaw
	using axes_t = Vec<float, 3>;

	EulerDirectionDetector() : n_pending(0) {}

	void process(const SensorData& input) noexcept;
	// TODO: magnetometer input -> yaw

	bool get_next(uint64_t& ts, angles_t& angles) noexcept;
private:
	size_t n_pending;
	DataRing<SensorData, MAX_PKT_JITTER> queue;
};

void EulerDirectionDetector::process(const SensorData& input) noexcept
{
	queue.push(input);
	if (++n_pending > queue.size())
		n_pending = queue.size();
}

bool EulerDirectionDetector::get_next(uint64_t& ts, angles_t& angles) noexcept
{
	if (n_pending == 0)
		return false;

	const SensorData& data = queue[--n_pending];

	ts = data.timestamp;
	angles[0] = atan2(data.v.az, data.v.ay) / M_PI;	// 0 is level, +/-1 is upside-down
	angles[1] = atan2(data.v.ax, std::hypot(data.v.ay, data.v.az)) * 2 / M_PI;	// 1 is up, -1 is down
	angles[2] = 0;	// TODO

	return true;
}

template <size_t N>
struct BandDataSyncerBase
{
	BandDataSyncerBase() : datan{0, } {}

	void reset() noexcept { datan.fill(0); }

	bool get_next(uint64_t& ts, std::array<SensorData *, N>& data) noexcept;
protected:
	void push(size_t i, const SensorData& input) noexcept
	{
		queue[i].push(input);
		if (datan[i] < queue[i].size())
			++datan[i];
	}
private:
	std::array<DataRing<SensorData, MAX_PKT_JITTER>, N> queue;
	std::array<size_t, N> datan;
};

template <size_t N>
bool BandDataSyncerBase<N>::get_next(uint64_t& ts, std::array<SensorData *, N>& data) noexcept
{
	for (size_t n : datan)
		if (!n)
			return false;

	size_t i_min = 0;
	ts = queue[0][datan[0] - 1].timestamp;

	for (size_t i = 0; i < N; ++i) {
		auto& rec = queue[i][datan[i] - 1];

		data[i] = &rec;
		if (ts > rec.timestamp) {
			ts = rec.timestamp;
			i_min = i;
		}
	}

	--datan[i_min];
	return true;
}

template <BandLocation... Bands>
struct BandDataSyncer : BandDataSyncerBase<sizeof...(Bands)>
{
	static constexpr size_t N = sizeof...(Bands);

	void process(BandLocation which, const SensorData& input) noexcept
	{
		const std::array<BandLocation, N> bands{Bands...};

		size_t i = 0;
		for (auto bw : bands) {
			if (bw == which)
				return this->push(i, input);
			++i;
		}
	}
};

template <typename Derived>
struct BothHandsDetector
{
	BothHandsDetector() : enabled(false) {}

	void reset() noexcept { sync.reset(); }

	std::list<GestureData> process(BandLocation which, const SensorData& input)
	{
		if (!enabled)
			return {};

		sync.process(which, input);
		return static_cast<Derived&>(*this).detected();
	}
protected:
	BandDataSyncer<BandLocation::FOREARM_L, BandLocation::FOREARM_R> sync;
	bool enabled;
};

/* Blok (~ ręce w górę) */

struct ArmGuardDetector : BothHandsDetector<ArmGuardDetector>
{
	ArmGuardDetector() : last_detected(false) {}

	void update_enabled(const gesture_bitmap_t& mask) noexcept
	{
		enabled = mask.test(Gesture::GUARD_UP) || mask.test(Gesture::GUARD_DOWN);
	}

	std::list<GestureData> detected();
private:
	bool last_detected;

	static bool hand_is_up(const SensorData& data) noexcept;
};

bool ArmGuardDetector::hand_is_up(const SensorData& data) noexcept
{
	Vec<int, 3> rot{data.v.gx, data.v.gy, data.v.gz};

	if (ANY(abs(rot) > 4 * STABLE_ROT))
		return false;

	double fa = std::abs(std::hypot(data.v.ax, data.v.ay, data.v.az) / VMAG_1G) - 1;
//	std::cerr << "--- fa = " << fa << '\n';
	if (abs(fa) > 4 * STABLE_FORCE_DIFF)
		return false;

	float value = atan2(-data.v.ax, std::hypot(data.v.ay, data.v.az)) * 2 / M_PI;	// 1 is up, -1 is down
	return value > 1 - 2 * POINT_DIR_ACCURACY;
}

std::list<GestureData> ArmGuardDetector::detected()
{
	std::array<SensorData *, 2> datap;
	std::list<GestureData> det;
	uint64_t ts;
	size_t i = 0;

	while (sync.get_next(ts, datap)) {
		bool this_detected = true;
		for (SensorData *pd : datap) {
			if (!hand_is_up(*pd))
				this_detected = false;
		}

		if (last_detected == this_detected)
			continue;

		last_detected = this_detected;

		Gesture g = this_detected ? Gesture::GUARD_UP : Gesture::GUARD_DOWN;
		det.emplace(det.end(), g, ts);
	}

	return det;
}

/* Kręcenie kierownicą (zakłada, że ręce są symetrycznie) */

struct SteeringWheelDetector : BothHandsDetector<SteeringWheelDetector>
{
	void update_enabled(const gesture_bitmap_t& mask) noexcept { enabled = mask.test(Gesture::WHEEL); }
	std::list<GestureData> detected();
private:
	bool get_next(uint64_t& ts, float& angle) noexcept;
};

bool SteeringWheelDetector::get_next(uint64_t& ts, float& angle) noexcept
{
	std::array<SensorData *, 2> datap;
	if (!sync.get_next(ts, datap))
		return false;

	const SensorData& ldata = *datap[0];
	const SensorData& rdata = *datap[1];

	// przy: L =  Ly - jLz, R..., W = arg(R) + (arg(L) - arg(R))/2 - pi/2	[arg = (-pi,+pi)]
	//  ---> L =  Lz + jLy, R..., W = arg(R) + (arg(L) - arg(R))/2
	std::complex<float> lv(ldata.v.az, ldata.v.ay);
	std::complex<float> rv(rdata.v.az, rdata.v.ay);
	auto la = std::arg(lv) / M_PI, ra = std::arg(rv) / M_PI;	// [-1,1]

	auto da = la - ra;
	if (da < 0)
		da += 2;
	else if (da >= 2)
		da -= 2;
	da /= 2;	// [0, 1]

	da += ra;
	if (da > 1)
		da -= 2;

	// 0 is level, -0.5 is left, +0.5 is right
	angle = da;

	return true;
}

std::list<GestureData> SteeringWheelDetector::detected()
{
	std::list<GestureData> det;
	GestureData g(Gesture::WHEEL);

	while (get_next(g.timestamp, g.value.x))
		det.push_back(g);

	return det;
}

/* analizator danych */

struct BandDetectorImpl : public BandDetector
{
	gesture_bitmap_t enabled_mask;

	PointDirectionDetector point;
	PunchDetector<BandLocation::FOREARM_L> lpunch;
	PunchDetector<BandLocation::FOREARM_R> rpunch;
	SquatDetector squat;
	ArmGuardDetector guard;
	SteeringWheelDetector wheel;

	BandDetectorImpl();

	gesture_bitmap_t get_gestures(unsigned mask) const noexcept override;
	void enable_gestures(const gesture_bitmap_t& mask) override;
	std::list<GestureData> process(BandLocation which, const SensorData& input) override;
	void reset() override;
};

BandDetectorImpl::BandDetectorImpl()
	: enabled_mask(0)
{
}

static constexpr bool has_band(unsigned mask, BandLocation loc)
{
	return mask & (1u << (int)loc);
}

template <BandLocation LOC> struct PointGesture;
template <BandLocation LOC> struct PunchGesture;

template <BandLocation LOC>
static gesture_bitmap_t leg_gestures(unsigned mask)
{
	gesture_bitmap_t bm;
	if (!has_band(mask, LOC))
		return bm;

	Gesture first = PointGesture<LOC>::down;
	Gesture sum_first = PointGesture<LOC>::either_down;

	bm.set((int)first + 0);	// DOWN
	bm.set((int)first + 1);	// LOW
	bm.set((int)first + 2);	// LEVEL
	bm.set((int)sum_first + 0);
	bm.set((int)sum_first + 1);
	bm.set((int)sum_first + 2);

	return bm;
}

template <BandLocation LOC>
static gesture_bitmap_t arm_gestures(unsigned mask)
{
	gesture_bitmap_t bm = leg_gestures<LOC>(mask);
	if (!has_band(mask, LOC))
		return bm;

	Gesture first = PointGesture<LOC>::down;
	Gesture sum_first = PointGesture<LOC>::either_down;

	bm.set((int)first + 3);	// HIGH
	bm.set((int)first + 4);	// UP
	bm.set((int)sum_first + 3);
	bm.set((int)sum_first + 4);
	return bm;
}

template <BandLocation LOC>
static gesture_bitmap_t punch_gestures(unsigned mask)
{
	gesture_bitmap_t bm;
	if (!has_band(mask, LOC))
		return bm;

	Gesture first = PunchGesture<LOC>::base;
	Gesture sum_first = PunchGesture<LOC>::either_base;

	bm.set((int)first);
	bm.set((int)first + 1);	// LOW
	bm.set((int)first + 2);	// LEVEL
	bm.set((int)first + 3);	// HIGH
	bm.set((int)sum_first);
	bm.set((int)sum_first + 1);
	bm.set((int)sum_first + 2);
	bm.set((int)sum_first + 3);
	return bm;
}

template <BandLocation LOC, Gesture DOWN> struct PointArmGesture {
	static constexpr Gesture down = DOWN;
	static constexpr Gesture either_down = Gesture::POINT_ARM_DOWN;
	static gesture_bitmap_t gestures(unsigned mask) { return arm_gestures<LOC>(mask); }
};
template <BandLocation LOC, Gesture DOWN> struct PointLegGesture {
	static constexpr Gesture down = DOWN;
	static constexpr Gesture either_down = Gesture::POINT_LEG_DOWN;
	static gesture_bitmap_t gestures(unsigned mask) { return leg_gestures<LOC>(mask); }
};

template <BandLocation LOC, Gesture BASE> struct PunchGestureBase {
	static constexpr Gesture base = BASE;
	static constexpr Gesture either_base = Gesture::PUNCH;
	static gesture_bitmap_t gestures(unsigned mask) { return punch_gestures<LOC>(mask); }
};

template <> struct PointGesture<BandLocation::FOREARM_L> : PointArmGesture<BandLocation::FOREARM_L, Gesture::POINT_FL_DOWN> {};
template <> struct PointGesture<BandLocation::FOREARM_R> : PointArmGesture<BandLocation::FOREARM_R, Gesture::POINT_FR_DOWN> {};
template <> struct PointGesture<BandLocation::CALF_L> : PointLegGesture<BandLocation::CALF_L, Gesture::POINT_CL_DOWN> {};
template <> struct PointGesture<BandLocation::CALF_R> : PointLegGesture<BandLocation::CALF_R, Gesture::POINT_CR_DOWN> {};
template <> struct PunchGesture<BandLocation::FOREARM_L> : PunchGestureBase<BandLocation::FOREARM_L, Gesture::PUNCH_LEFT> {};
template <> struct PunchGesture<BandLocation::FOREARM_R> : PunchGestureBase<BandLocation::FOREARM_R, Gesture::PUNCH_RIGHT> {};

static gesture_bitmap_t all_gestures(unsigned mask)
{
	gesture_bitmap_t bm;

	bm |= PointGesture<BandLocation::FOREARM_L>::gestures(mask);
	bm |= PointGesture<BandLocation::FOREARM_R>::gestures(mask);
	bm |= PointGesture<BandLocation::CALF_L>::gestures(mask);
	bm |= PointGesture<BandLocation::CALF_R>::gestures(mask);
	bm |= PunchGesture<BandLocation::FOREARM_L>::gestures(mask);
	bm |= PunchGesture<BandLocation::FOREARM_R>::gestures(mask);

	if (has_band(mask, BandLocation::TORSO))
		bm.set(Gesture::SQUAT);

	if (has_band(mask, BandLocation::FOREARM_L) && has_band(mask, BandLocation::FOREARM_R)) {
		bm.set(Gesture::WHEEL);
		bm.set(Gesture::GUARD_UP);
		bm.set(Gesture::GUARD_DOWN);
	}

	return bm;
}

gesture_bitmap_t BandDetectorImpl::get_gestures(unsigned mask) const noexcept
{
	return all_gestures(mask);
}

template <BandLocation LOC>
static constexpr unsigned limp_band(const gesture_bitmap_t& mask)
{
	unsigned bm = 1u << (int)LOC;
	return (mask & PointGesture<LOC>::gestures(BAND_LOC_MASK)).any() ? bm : 0;
}

void PointDirectionDetector::update_enabled(const gesture_bitmap_t& mask) noexcept
{
	b_mask = 0;

	b_mask |= limp_band<BandLocation::FOREARM_L>(mask);
	b_mask |= limp_band<BandLocation::FOREARM_R>(mask);
	b_mask |= limp_band<BandLocation::CALF_L>(mask);
	b_mask |= limp_band<BandLocation::CALF_R>(mask);

	// reset unused bands' state
	for (int i = (int)BandLocation::FOREARM_L; i <= (int)BandLocation::CALF_R; ++i)
		if (~b_mask & (1u << i))
			last_dir[i - (int)BandLocation::FOREARM_L] = -1;

	enabled = mask;
}

template <BandLocation LOC, bool Either>
constexpr Gesture PointDirectionDetector::offset(int i) noexcept
{
	auto base = Either ? PointGesture<LOC>::either_down : PointGesture<LOC>::down;
	return (Gesture)((int)base + i);
}

template <BandLocation LOC>
void PunchDetector<LOC>::update_enabled(const gesture_bitmap_t& mask) noexcept
{
	bool was_enabled = enabled.any();
	enabled = mask & PunchGesture<LOC>::gestures(BAND_LOC_MASK);
	if (was_enabled ^ enabled.any())
		reset();
	if (DEBUG_PUNCH)
		std::cerr << 'B' << LOC << ": enable " << was_enabled << " -> " << enabled.any()
			  << " (new mask " << mask << '/' << PunchGesture<LOC>::gestures(BAND_LOC_MASK) << ')'
			  << std::endl;
}

template <BandLocation LOC> template <bool Either>
constexpr Gesture PunchDetector<LOC>::offset(int i) noexcept
{
	auto base = Either ? PunchGesture<LOC>::either_base : PunchGesture<LOC>::base;
	return (Gesture)((int)base + i);
}

void SquatDetector::update_enabled(const gesture_bitmap_t& mask) noexcept
{
	enabled = mask.test(Gesture::SQUAT);

	// reset if disabled
	if (!enabled)
		reset();
}

void BandDetectorImpl::enable_gestures(const gesture_bitmap_t& mask)
{
	enabled_mask = mask;

	point.update_enabled(mask);
	lpunch.update_enabled(mask);
	rpunch.update_enabled(mask);
	squat.update_enabled(mask);
	guard.update_enabled(mask);
	wheel.update_enabled(mask);
}

std::list<GestureData> BandDetectorImpl::process(BandLocation which, const SensorData& input)
{
	std::list<GestureData> detected;

	auto compare_ts = [] (const GestureData& a, const GestureData& b) -> bool {
		return a.timestamp < b.timestamp;
	};

	detected.merge(point.process(which, input), compare_ts);
	detected.merge(lpunch.process(which, input), compare_ts);
	detected.merge(rpunch.process(which, input), compare_ts);
	detected.merge(squat.process(which, input), compare_ts);
	detected.merge(guard.process(which, input), compare_ts);
	detected.merge(wheel.process(which, input), compare_ts);

	return detected;
}

void BandDetectorImpl::reset()
{
	point.reset();
	lpunch.reset();
	rpunch.reset();
	squat.reset();
	guard.reset();
	wheel.reset();
}

} /* anonymous namespace */

std::unique_ptr<BandDetector> BandDetector::create()
{
	return std::make_unique<BandDetectorImpl>();
}
