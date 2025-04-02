#include "vecs.h"

#include <cmath>

template <typename TS = double>
static void rotate3d(Vec<TS, 3>& vec, size_t axis, TS angle)
{
	TS& x = vec[(axis + 1) % 3];
	TS& y = vec[(axis + 2) % 3];
	TS sina = std::sin(angle), cosa = std::cos(angle);

	TS rx = x * cosa - y * sina;
	TS ry = x * sina + y * cosa;

	x = rx;
	y = ry;
}

void GVectorTracker::update(uint64_t ts, const Vec<int, 3>& inrot, const Vec<int, 3>& acc)
{
	// TODO: kalibracja prędkości obrotowej
	constexpr double ROTRES = 0.070 * 256 / 1000000;
	constexpr double ROTS = ROTRES * M_PI / 180;
	constexpr double ONEG_MAG = 1000000 / 488.0;
	constexpr int MAX_STABLE_ROT2 = 8;
	constexpr int MAX_G_DIFF = 128;

	Vec<int, 3> rot = (inrot + gprev) / 2;
	gprev = inrot;

	double rot_scale = (ts - last_ts) * ROTS;
	last_ts = ts;

	auto accm = sqrt(acc.mag2<double>());
	if (rot.mag2() < MAX_STABLE_ROT2 && abs(accm - ONEG_MAG) < MAX_G_DIFF) {
		vec = Vec<double, 3>(acc) / accm;
	} else {
		rotate3d(vec, 2, rot[2] * rot_scale);
		rotate3d(vec, 1, rot[1] * rot_scale);
		rotate3d(vec, 0, rot[0] * rot_scale);
	}
}
