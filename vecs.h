#ifndef GAMEINN_VECS_H_
#define GAMEINN_VECS_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

template <typename T, size_t N = 3>
struct Vec
{
	std::array<T, N> v;

	constexpr Vec() : v() {}
	template <typename TO, typename = std::void_t<decltype(std::declval<T&>() = std::declval<TO>())>> \
	constexpr Vec(const Vec<TO, N>& o) { std::copy_n(o.v.begin(), v.size(), v.begin()); }
	constexpr Vec(const std::array<T, N>& iv) : v(iv) {}
	constexpr Vec(std::initializer_list<T> iv)
	{
		//static_assert(iv.size() <= N);
		std::copy_n(iv.begin(), iv.size(), v.begin());
	}

	#define _GAMEINN_OP2(r,op) \
	template <typename TO, typename = std::void_t<decltype(std::declval<T>() op std::declval<TO>())>> \
	constexpr r operator op(const Vec<TO, N>& b) const	\
	{						\
		r out{false, };				\
		for (size_t i = 0; i < N; ++i)		\
			out[i] = v[i] op b[i];		\
		return out;				\
	}						\
	constexpr r operator op(const T& s) const	\
	{						\
		r out{false, };				\
		for (size_t i = 0; i < N; ++i)		\
			out[i] = v[i] op s;		\
		return out;				\
	}

	#define _GAMEINN_MATHOP(op) _GAMEINN_OP2(Vec,op) \
	template <typename TO, typename = std::void_t<decltype(std::declval<Vec<T,N>>() op std::declval<TO>())>> \
	Vec& operator op##=(const TO& b) 		\
	{						\
		return *this = *this op b;		\
	}						\

	using bools_t = std::array<bool, N>;
	_GAMEINN_MATHOP(+)
	_GAMEINN_MATHOP(-)
	_GAMEINN_MATHOP(*)
	_GAMEINN_MATHOP(/)
	_GAMEINN_OP2(bools_t, <)
	_GAMEINN_OP2(bools_t, <=)
	_GAMEINN_OP2(bools_t, >=)
	_GAMEINN_OP2(bools_t, >)
	_GAMEINN_OP2(bools_t, ==)
	_GAMEINN_OP2(bools_t, !=)

	#undef _GAMEINN_OP2
	#undef _GAMEINN_MATHOP

	template <typename TO, typename = std::void_t<decltype(std::declval<T&>() = std::declval<TO>())>> \
	Vec& operator=(const Vec<TO, N>& b)
	{
		for (size_t i = 0; i < N; ++i)
			v[i] = b.v[i];
		return *this;
	}

	Vec& operator=(const T& b)
	{
		v.fill(b);
		return *this;
	}

	#define _GAMEINN_F1(f) \
	friend constexpr Vec f(const Vec& a)	\
	{					\
		Vec r;				\
						\
		for (size_t i = 0; i < N; ++i)	\
			r[i] = f(a[i]);		\
						\
		return r;			\
	}

	_GAMEINN_F1(sqrt)
	_GAMEINN_F1(abs)

	#undef _GAMEINN_F1

	constexpr T& operator[](size_t i) { return v[i]; }
	constexpr const T& operator[](size_t i) const { return v[i]; }

	template <typename TV = T>
	constexpr TV sum() const
	{
		TV s{0};

		for (auto& a : v)
			s += a;

		return s;
	}

	template <typename TV = T, typename TO = T>
	constexpr TV dot(const Vec<TO, N>& b) const
	{
		TV s{0};

		for (size_t i = 0; i < N; ++i)
			s += (TV)v[i] * b[i];

		return s;
	}

	template <typename TV = T>
	constexpr TV mag2() const
	{
		return dot<TV>(*this);
	}

	Vec& sub_sat(const Vec& b)
	{
		for (size_t i = 0; i < N; ++i)
			if (__builtin_sub_overflow(v[i], b[i], &v[i]))
				v[i] = b[i] < 0 ? std::numeric_limits<T>::max() : std::numeric_limits<T>::min();

		return *this;
	}
};

template <bool UseOr, size_t N>
static inline constexpr bool ACCUMULATE_BOOLS(const std::array<bool, N>& ba)
{
	for (size_t i = 0; i < N; ++i)
		if (ba[i] == UseOr)
			return UseOr;
	return !UseOr;
}

template <size_t N>
static inline constexpr bool ALL(const std::array<bool, N>& ba)
{
	return ACCUMULATE_BOOLS<false>(ba);
}

template <size_t N>
static inline constexpr bool ANY(const std::array<bool, N>& ba)
{
	return ACCUMULATE_BOOLS<true>(ba);
}

template <typename T, size_t N>
struct DataRing
{
	size_t icur;
	std::array<T, N> data;

	DataRing() : icur(0) {}

	void clear() noexcept
	{
		data.fill(T());
		icur = 0;
	}

	void push(const T& input)
	{
		data[icur++] = input;
		icur %= data.size();
	}

	size_t size() const noexcept { return N; }
	T& operator[](size_t i) { return data[(N + icur - i) % N]; }
	const T& operator[](size_t i) const { return data[(N + icur - i) % N]; }
	const T& oldest() const { return data[icur]; }
	const T& newest() const { return data[(icur + N - 1) % N]; }

	struct const_iterator {
		const_iterator(const DataRing& r_, size_t i_, bool end) : r(r_), i(i_), inc(end) {}

		const T& operator*() const { return r.data[i]; }
		const T *operator->() const { return &r.data[i]; }

		const_iterator& operator++() { i = (i + 1) % N; inc = true; return *this; }
		const_iterator operator++(int) { const_iterator c(*this); operator++(); return c; }

		bool operator==(const const_iterator& other) const { return i == other.i && inc == other.inc; }
		bool operator!=(const const_iterator& other) const { return !operator==(other); }
	private:
		const DataRing& r;
		size_t i;
		bool inc;
	};

	const_iterator begin() const { return const_iterator(*this, icur, false); }
	const_iterator end() const { return const_iterator(*this, icur, true); }
};

template <typename T, size_t N>
struct DataWindow
{
	DataRing<T, N> data;
	size_t n;

	DataWindow() : n(0) {}

	void clear() noexcept
	{
		n = 0;
	}

	void push(const T& input) noexcept
	{
		data.push(input);
		if (n < data.size())
			++n;
	}

	size_t size() const noexcept { return n; }

	using const_iterator = typename DataRing<T, N>::const_iterator;

	const_iterator begin() const { return {data, (N + data.icur - n) % N, false}; }
	const_iterator end() const { return {data, data.icur, !!n}; }

	const T& front() const { return *begin(); }
};

template <size_t N>
struct CondSize {
	operator size_t() const { return N; }

	CondSize& operator=(size_t) { return *this; }
};

template <>
struct CondSize<0> {
	size_t n;

	CondSize() : n(0) {}

	operator size_t() const { return n; }

	CondSize<0>& operator++() noexcept { ++n; return *this; }
	CondSize<0>& operator=(size_t v) noexcept { n = v; return *this; }
};

template <typename T, typename TSums = T, size_t N = 0>
struct CumulativeStats
{
	TSums sum, sqsum;
	CondSize<N> n;

	void clear() noexcept
	{
		sum = sqsum = TSums();
		n = N;
	}

	void update(const T& in, const T& out = T())
	{
		TSums adiff = in, asum = adiff;
		adiff -= out;
		asum += out;

		sum += adiff;
		sqsum += asum * adiff;
	}

	template <size_t N_ = N, typename = typename std::enable_if<N_ == 0>::type>
	void add(const T& in)
	{
		update(in);
		++n;
	}

	template <typename TR = TSums>
	constexpr TR mean() const
	{
		TR out = sum;
		out /= n;
		return out;
	}

	template <typename TR = TSums>
	constexpr TR var() const
	{
		TR out = sum;
		out *= sum;
		out /= n;
		out = sqsum - out;
		out /= n;
		return out;
	}

	template <typename TR = TSums>
	constexpr TR stddev() const
	{
		return sqrt(var<TR>());
	}
};

template <typename T, typename TSums = T, size_t N = 128>
struct WindowedStats : private CumulativeStats<T, TSums, N>
{
	using Base = CumulativeStats<T, TSums, N>;
	using Ring = DataRing<T, N>;

	Ring data;

	void clear() noexcept
	{
		Base::clear();
		data.clear();
	}

	void add(const T& input)
	{
		Base::update(input, data.oldest());
		data.push(input);
	}

	using Base::mean;
	using Base::var;
	using Base::stddev;
};

struct __attribute__((visibility ("default"))) GVectorTracker
{
	operator const Vec<double, 3>& () const noexcept { return vec; }
	operator const std::array<double, 3>& () const noexcept { return vec.v; }

	const Vec<double, 3>& get() const noexcept { return vec; }

	void update(uint64_t ts, const Vec<int, 3>& rot, const Vec<int, 3>& acc);

	GVectorTracker() : last_ts(0) {}
private:
	Vec<double, 3> vec;
	Vec<int, 3> gprev;
	uint64_t last_ts;
};

#endif /* _GAMEINN_MATHOP */
