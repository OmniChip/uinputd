#include "band.h"
#include "vecs.h"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <memory>
#include <numeric>

#include <time.h>

namespace /* anonymous */ {

#ifdef __ANDROID___
  #undef DEBUG
#endif

#ifndef DEBUG
  #define DEBUG (false)
#endif

struct BandCalibratorImpl final : BandCalibrator
{
	static constexpr size_t VarWindowN = 128;
	static constexpr double RotVarStable = 10;
	static constexpr double AccVarStable = 50;
	static constexpr double OneG = 2048;
	static constexpr double MaxGDev = 0.02;
	static constexpr double MinGAxis = OneG * 3 / 4;

	BandCalibratorImpl()
		: data_us(0), data_pps(0), data_cnt(0), axis_mask(0), last_stable(false)
	{
	}

	virtual bool process(BandDevice::SensorData& input) override;
	virtual bool is_done() const override { return !(~axis_mask & 0x3F); }
	virtual std::string status() const override;
	virtual uint8_t calibration_needed_mask() const override;
	virtual BandDevice::AxisData zero_offset() const override;
	virtual void zero_applied(const BandDevice::AxisData& offset) override;
private:
	Vec<int, 3> azero, gzero;
	std::array<Vec<double, 3>, 6> accv;
	std::array<size_t, 6> accn;
	mutable uint64_t data_us;
	mutable uint32_t data_pps;
	mutable uint8_t data_cnt;
	uint8_t axis_mask;
	bool last_stable;
	WindowedStats<Vec<int16_t, 3>, Vec<int64_t, 3>, VarWindowN> astats;
	WindowedStats<Vec<int16_t, 3>, Vec<int64_t, 3>, VarWindowN> gstats;
	CumulativeStats<Vec<int16_t, 3>, Vec<int64_t, 3>> acur;

	bool calibrate();
	void calc_acc_zero();

	template <typename TV>
	void debug_input(unsigned ts, const TV& accel, const TV& gyro, bool good_for_cal) const;
};

template <typename T, size_t N>
static size_t append_axes(char *p, const char *fmt, const Vec<T, N>& vec)
{
	char *q = p;
	for (const T& e : vec.v)
		p += sprintf(p, fmt, e);
	return p - q;
}

template <typename T, size_t N>
static std::string& append_axes(std::string& s, const char *fmt, const Vec<T, N>& vec)
{
	size_t n = s.size();
	s.append(24*N, '\0');
	s.resize(n += append_axes(&s[n], fmt, vec));
	return s;
}

template <typename TV>
void BandCalibratorImpl::debug_input(unsigned ts, const TV& accel, const TV& gyro, bool good_for_cal) const
{
	++data_cnt;
	if (!data_cnt) {
		struct timespec ts;

		if (!clock_gettime(CLOCK_MONOTONIC, &ts)) {
			uint64_t us = ts.tv_nsec / 1000 + ts.tv_sec * 1000000;
			uint64_t dt = us - data_us;
			data_us = us;

			if (dt)
				data_pps = 1000000l / dt;
		}
	}

	char buf[512], *p = buf;

	p += sprintf(p, "--- A(");
	p += append_axes(p, " %6d", accel);
	p += sprintf(p, ")  G(");
	p += append_axes(p, " %6d", gyro);
	p += sprintf(p, ") @%7u", ts);

	int am = sqrt(accel.template mag2<long>());
	p += sprintf(p, "  |A| = %6d  %s%s  @%u", am, status().c_str(), good_for_cal ? "  CAL" : "", ts);
	if (data_pps)
		p += sprintf(p, "  %1.2f pps", data_pps / 256.0);

	if (!is_done()) {
		p += sprintf(p, "\n... A: MEAN ");
		p += append_axes(p, " %6ld", astats.mean());
		p += sprintf(p, "  STDDEV ");
		p += append_axes(p, " %6ld", astats.stddev());

		p += sprintf(p, " / G: MEAN ");
		p += append_axes(p, " %6ld", gstats.mean());
		p += sprintf(p, "  STDDEV ");
		p += append_axes(p, " %6ld", gstats.stddev());
		p += sprintf(p, "  vsum=%6lld", (long long)gstats.var().sum());
	}

	p += sprintf(p, "\n");
	std::cerr << buf;
}

bool BandCalibratorImpl::process(BandDevice::SensorData& cur)
{
	Vec<int, 3> acc = { cur.v.ax, cur.v.ay, cur.v.az };
	Vec<int, 3> gyro = { cur.v.gx, cur.v.gy, cur.v.gz };

	astats.add(acc);
	gstats.add(gyro);

	bool good_for_cal = calibrate();

	gyro.sub_sat(gzero);
	acc.sub_sat(azero);

	if (DEBUG)
		debug_input(cur.timestamp, acc, gyro, good_for_cal);

	cur.v.ax = acc[0];
	cur.v.ay = acc[1];
	cur.v.az = acc[2];
	cur.v.gx = gyro[0];
	cur.v.gy = gyro[1];
	cur.v.gz = gyro[2];

	return is_done();
}

BandDevice::AxisData BandCalibratorImpl::zero_offset() const
{
	return { gzero[0], gzero[1], gzero[2], azero[0], azero[1], azero[2] };
}

void BandCalibratorImpl::zero_applied(const BandDevice::AxisData& offset)
{
	Vec<int, 3> go = { offset.gx, offset.gy, offset.gz };
	Vec<int, 3> ao = { offset.ax, offset.ay, offset.az };

	gzero -= go;
	azero -= ao;

	for (auto& v : accv)
		v -= ao;
}

static bool is_power_of_2(unsigned v)
{
	return (v & (v - 1)) == 0;
}

static int fls(unsigned v)
{
	return sizeof(v) * 8 - 1 - __builtin_clz(v);
}

bool BandCalibratorImpl::calibrate()
{
	bool do_clear = !last_stable;
	last_stable = false;

	if (gstats.var().sum() >= RotVarStable)
		return false;
	if (astats.var().sum() >= AccVarStable)
		return false;

	last_stable = true;

	gzero = gstats.mean();
	axis_mask |= 0x40;

	if (do_clear) {
		acur.clear();
		for (auto& v : astats.data)
			acur.add(v);
	} else {
		acur.add(astats.data.newest());
	}

	auto mean = astats.mean();
	uint8_t m_axis_mask = 0;
	for (size_t i = 0; i < 3; ++i) {
		m_axis_mask |= (mean[i] < -MinGAxis) << (2 * i);
		m_axis_mask |= (mean[i] > +MinGAxis) << (2 * i + 1);
	}

	if (!is_power_of_2(m_axis_mask))
		return true;

	axis_mask |= m_axis_mask;
	size_t i = fls(m_axis_mask);

	if (acur.n > accn[i]) {
		accv[i] = mean;
		accn[i] = acur.n;
	}

	if (is_done())
		calc_acc_zero();

	return true;
}

std::string BandCalibratorImpl::status() const
{
	std::string s;

	if (is_done()) {
		s = "[calibrated: A=";
		append_axes(s, " %6d", azero);
		s.append("  G=");
		append_axes(s, " %6d", gzero);
		s.append("]");
	} else {
		s = "[missing: -x+ -y+ -z+]";
		for (size_t i = 0; i < 6; ++i)
			if ((axis_mask >> i) & 1)
				s[10 + 2 * i] = ' ';
		for (size_t i = 0; i < 3; ++i)
			if (!((~axis_mask >> (2 * i)) & 3))
				s[11 + 4 * i] = ' ';
	}

	return s;
}

uint8_t BandCalibratorImpl::calibration_needed_mask() const
{
	return ~axis_mask & 0x7F;
}

template <typename TS, size_t N>
static constexpr TS diag_mul(const Vec<Vec<TS, N>, N>& m, size_t offs, int step)
{
	TS v = 1;

	for (size_t i = 0; i < N; ++i)
		v *= m[i][(N + offs + i * step) % N];

	return v;
}

template <typename TS>
static constexpr TS matrix_det(const Vec<Vec<TS, 3>, 3>& m)
{
	TS v = 0;

	for (int i = 0; i < 3; ++i) {
		v += diag_mul(m, i, 1);
		v -= diag_mul(m, i, -1);
	}

	return v;
}

template <typename TS, size_t N>
static void swap_col(Vec<Vec<TS, N>, N>& m, Vec<TS, N>& v, size_t i)
{
	for (size_t j = 0; j < N; ++j)
		std::swap(m[j][i], v[j]);
}

template <typename TS, size_t N, typename TV = Vec<TS, N>>
static TS inplace_det(Vec<Vec<TS, N>, N>& m, TV& r, size_t i)
{
	swap_col(m, r, i);
	TS det = matrix_det(m);
	swap_col(m, r, i);
	return det;
}

template <typename TS, size_t N, typename TV = Vec<TS, N>>
static TV inplace_solve_linear(Vec<Vec<TS, N>, N>& m, TV& r)
{
	TS da = matrix_det(m);
	TV cp;
	for (size_t i = 0; i < N; ++i)
		cp[i] = inplace_det(m, r, i) / da;

	return cp;
}

#if 0
template <typename TS = double, typename TV = Vec<TS, 3>>
static TV calc_sphere_center(const Vec<TS, 3>& a, const TV& b, const TV& c, const TV& d)
{
	Vec<TV, 3> m{ a - b, a - c, a - d };

	TS ra = a.template mag2<TS>();
	TV r{ b.template mag2<TS>() - ra, c.template mag2<TS>() - ra, d.template mag2<TS>() - ra };

	// układ równań na środek sfery
	// v=[x,y,z], i=[b,c,d]: SUM_v(2v_i*v) = SUM_v(v_i^2 - v_a^2)
	//
	// M * V = R
	// punkty na osiach X,Y,Z -> w przybliżeniu:
	//
	// [2 0 0]     = [0]
	// [1 1 0] * V = [0]
	// [1 0 1]     = [0]
	//
	// (tj. wiemy, że det(A) =~ +/-2 != 0

	TV cp = inplace_solve_linear(m, r);
#if 0
	for (size_t i = 0; i < N; ++i) {
		char buf[256];
		sprintf(buf, "%c: %6.0f %6.0f %6.0f %6.0f => %6.0f\n",
			char('X' + i), a[i], b[i], c[i], d[i], cp[i]);
		std::cerr << buf;
	}
#endif
	return cp;
}

void BandCalibratorImpl::calc_acc_zero()
{
	Vec<double, 3> mid;

	for (int k = 0; k < 3; ++k)
		for (int i = 0; i < 4; ++i) {
			int o2 = (2*k + 2 + (i & 1)) % 6;
			int o3 = (2*k + 4 + (i >> 1)) % 6;
			mid += calc_sphere_center(accv[2*k], accv[2*k+1], accv[o2], accv[o3]);
		}
	mid /= 3 * 4;

	azero = mid;
}
#else
// https://www.researchgate.net/publication/278049014_Fast_Geometric_Fit_Algorithm_for_Sphere_Using_Exact_Solution

template <typename TS, size_t N, typename TV = Vec<TS, 3>>
static TV calc_sphere_center(const std::array<Vec<TS, 3>, N>& points)
{
	TV sum;			// [i] = sum(v[i])
	Vec<TV, 3> sum2;	// [i][j] = sum(v[i]*v[j])
	Vec<TV, 3> sum3;	// [i][j] = sum(v[i]*v[j]*v[j])

	for (const TV& v : points) {
		TV v2 = v * v;	// element-by-element

		sum += v;

		for (int i = 0; i < 3; ++i)
			sum2[i] += v * v[i];

		for (int i = 0; i < 3; ++i)
			sum3[i] += v2 * v[i];
	}

	Vec<TV, 3> a;	// [col][row]
	TV b;		// [row]

	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j) {
			a[j][i] = 2 * sum[j] * sum[i] - 2 * N * sum2[j][i];
			b[j] += sum[j] * sum2[i][i] - N * sum3[j][i];
		}

	return inplace_solve_linear(a, b);
}

void BandCalibratorImpl::calc_acc_zero()
{
	azero = calc_sphere_center(accv);
}

#endif

} /* anonymous namespace */

std::unique_ptr<BandCalibrator> BandCalibrator::create()
{
	return std::make_unique<BandCalibratorImpl>();
}
