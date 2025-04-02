#include "band.h"
#include "dbus-base.h"
#include "ble.h"
#include "vecs.h"

#include <giomm.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <random>
#include <set>

#include <endian.h>
#include <time.h>
#include <unistd.h>


struct __attribute__((visibility ("default"))) BandDeviceLL
{
	virtual ~BandDeviceLL() = default;

	static void on_device_removed(BandDevice& bd) noexcept
	{
		bd.dev = nullptr;
		bd.device_removed();
	}

	static void on_device_initialized(BandDevice& bd, const std::string& name, uint64_t ts, const BandDevice::DevIdData& devid) noexcept
	{
		bd.device_initialized(name, ts, devid);
	}

	static void on_data_received(BandDevice& bd, const BandDevice::SensorData& data) noexcept
	{
		bd.data_received(data);
	}
};


namespace /* anonymous */ {

#define VENDOR_OMNICHIP	0xEEFA

static const std::string BAND_INERTIA_SERVICE_UUID{"00001833-0000-1000-8000-00805f9b34fb"};
static const std::string BAND_MOTOR_SERVICE_UUID{"00001844-0000-1000-8000-00805f9b34fb"};
static const std::string BAND_TIME_SERVICE_UUID{"00001805-0000-1000-8000-00805f9b34fb"};
static const std::string DIS_SERVICE_UUID{"0000180a-0000-1000-8000-00805f9b34fb"};
static const std::string BAND_INERTIA_DATA_UUID{"000029fe-0000-1000-8000-00805f9b34fb"};
static const std::string BAND_INERTIA_ZERO_UUID{"000029fd-0000-1000-8000-00805f9b34fb"};
static const std::string BAND_VIBE_CTL_UUID{"000029f0-0000-1000-8000-00805f9b34fb"};
static const std::string DIS_MFG_NAME_UUID{"00002a29-0000-1000-8000-00805f9b34fb"};
static const std::string DIS_PNP_ID_UUID{"00002a50-0000-1000-8000-00805f9b34fb"};
static const std::string TIME_SYNC_CONTROL_UUID{"000029fc-0000-1000-8000-00805f9b34fb"};
static const std::string TIME_SYNC_COUNTER_UUID{"000029fb-0000-1000-8000-00805f9b34fb"};
static const std::string TIME_SYNC_TAG_UUID{"000029fa-0000-1000-8000-00805f9b34fb"};

struct pnpid_data {
	uint8_t id_source;
	uint16_t vendor, product, version;
} __attribute__((packed));

struct band_data {
	uint32_t ts;
	int16_t gx, gy, gz;
	int16_t ax, ay, az;
};

struct vibe_ctl_data {
	uint64_t effect;
	uint32_t when;
} __attribute__((packed));

static uint64_t get_monotonic_clock()
{
	struct timespec tv;

	if (!clock_gettime(CLOCK_MONOTONIC, &tv))
		return tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

	return 0;
}

static uint64_t random64()
{
	static std::random_device rd;
	std::uniform_int_distribution<uint64_t> dist;
	return dist(rd);
}

static void ignore_call_result(const Glib::RefPtr<Gio::DBus::Proxy>& obj, const Glib::RefPtr<Gio::AsyncResult>& res)
{
	try {
		obj->call_finish(res);
	} catch (...) {
		/* ignore */
	}
}

static bool match_bluez_EINPROGRESS(const Glib::Error& e)
{
	if (!e.matches(g_io_error_quark(), G_IO_ERROR_DBUS_ERROR))
		return false;

	if (e.what() == "GDBus.Error:org.bluez.Error.Failed: Operation already in progress")
		return true;
	if (e.what() == "GDBus.Error:org.bluez.Error.InProgress: Operation already in progress")
		return true;

	return false;
}

static bool match_bluez_NOTREADY(const Glib::Error& e)
{
	if (!e.matches(g_io_error_quark(), G_IO_ERROR_DBUS_ERROR))
		return false;

	if (e.what() == "GDBus.Error:org.bluez.Error.NotReady: Resource Not Ready")
		return true;

	return false;
}

static bool match_bluez_AUTHFAILED(const Glib::Error& e)
{
	if (!e.matches(g_io_error_quark(), G_IO_ERROR_DBUS_ERROR))
		return false;

	if (e.what() == "GDBus.Error:org.bluez.Error.AuthenticationFailed: Authentication Failed")
		return true;

	return false;
}

using props_t = BluezObjectManager::interface_properties_map_t;

struct BandManagerImpl;
struct BandDeviceImpl;


template <typename Property>
static typename Property::value_type get_property_from_map(const Gio::DBus::Proxy::MapChangedProperties& props)
{
	auto pp = props.find(Property::name);
	if (pp == props.end())
		throw std::runtime_error("get_prop missing property: " + Property::name);

	return Glib::VariantBase::cast_dynamic<Glib::Variant<typename Property::value_type>>(pp->second).get();
}

template <typename Property>
static typename Property::value_type get_property_from_map(const props_t& props)
{
	auto pi = props.find(Property::interface_type::name);
	if (pi == props.end())
		throw std::runtime_error("get_prop missing interface: " + Property::interface_type::name);

	return get_property_from_map<Property>(pi->second);
}

struct BLEAdapter : public BTAdapter
{
	using BTAdapter::BTAdapter;

	BLEAdapter(const Glib::DBusObjectPathString& path)
		: BTAdapter(path), cancel(Gio::Cancellable::create())
	{
	}

	~BLEAdapter() {
		cancel->cancel();
	}

	void init_adapter(GattProfileStub& bleapp)
	{
		auto gattmgr = std::make_shared<GattManager>(path());

		auto cb = [gattmgr, this, &bleapp] (const Glib::RefPtr<Gio::AsyncResult>& res) {
			ignore_call_result(gattmgr->get_obj(), res);
			regapp_done(bleapp);
		};

		gattmgr->RegisterApplication_casync(cancel, std::move(cb), "/", std::map<Glib::ustring, Glib::VariantBase>());
	}

	void regapp_done(GattProfileStub& bleapp)
	{
		std::map<Glib::ustring, Glib::VariantBase> dfilter = {
		       	{ "UUIDs", Glib::Variant<std::vector<Glib::ustring>>::create(bleapp.uuids) },
	       	};
		SetDiscoveryFilter(std::move(dfilter));

		StartDiscovery_casync(cancel, sigc::mem_fun(this, &BLEAdapter::start_discovery_done));
		std::cerr << "initialized adapter " << path() << '\n';
	}

	void start_discovery_done(const Glib::RefPtr<Gio::AsyncResult>& res)
	{
		try {
			finish_call(res);
		} catch (Glib::Error& e) {
			std::cerr << path() << " scan enable failed: " << e.what() << '\n';
			if (!match_bluez_EINPROGRESS(e) && !match_bluez_NOTREADY(e))
				StartDiscovery_casync(cancel, sigc::mem_fun(this, &BLEAdapter::start_discovery_done));
		}
	}

	virtual void on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties& props, const std::vector<Glib::ustring>& invd) override
	{
		BTAdapter::on_properties_changed(props, invd);

		try {
			bool powered = get_property_from_map<org::bluez::Adapter1::Properties::Powered>(props);
			if (!powered)
				return;	/* wait */

			bool scan = get_property_from_map<org::bluez::Adapter1::Properties::Discovering>(props);
			if (!scan) {
				std::cerr << path() << " scan disabled?\n";
				StartDiscovery_casync(cancel, sigc::mem_fun(this, &BLEAdapter::start_discovery_done));
			}
		} catch (...) {
			/* ignore */
		}
	}
private:
	Glib::RefPtr<Gio::Cancellable> cancel;
};


struct EventBurstCounter
{
	EventBurstCounter(unsigned secs, unsigned count)
		: seconds(secs), trigger_count(count), count(0) {}

	bool notice_event()
	{
		if (timer.elapsed() > seconds) {
			count = 0;
			timer.reset();
		}

		return ++count > trigger_count;
	}
private:
	const double seconds;
	const unsigned trigger_count;
	Glib::Timer timer;
	unsigned count;
};


struct InertiaDataChar : public GattCharacteristic
{
	BandDeviceImpl& dev;
	uint64_t ts_offset;
	bool past_half;

	explicit InertiaDataChar(BandDeviceImpl& dev_);

	virtual void on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties&, const std::vector<Glib::ustring>&) override;
	uint64_t fixup_timestamp(uint32_t ts);
};


struct TimeSyncChar : public GattCharacteristic
{
	BandDeviceImpl& dev;

	explicit TimeSyncChar(BandDeviceImpl& dev_) : dev(dev_) {}

	virtual void on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties&, const std::vector<Glib::ustring>&) override;
};


struct BandDeviceImpl final : BandDeviceLL, BLEDevice
{
	friend class InertiaDataChar;
	friend class TimeSyncChar;

	BandDeviceImpl(BandManagerImpl&, const Glib::DBusObjectPathString& path);
	~BandDeviceImpl();

	void pair();
	void connect();
	void start();
	bool found_service(const Glib::DBusObjectPathString& path, const props_t& props);
	void found_char(const Glib::DBusObjectPathString& path, const props_t& props);
	std::future<void> adjust_zero(const BandDevice::AxisData& zero);
	std::future<void> send_vibe(uint64_t effect, uint32_t when);

	void add_observer(BandDevice& bd) noexcept;
	void del_observer(BandDevice& bd) noexcept;

	void restart_time_sync();
private:
	struct char_info_t : std::pair<GattCharacteristic BandDeviceImpl::*, bool BandDeviceImpl::*>
	{
		char_info_t(GattCharacteristic BandDeviceImpl::*a, bool BandDeviceImpl::*b = nullptr) : pair(a, b) {}
	};

	using char_map_t = std::map<std::string, char_info_t>;

	static const char_map_t band_characteristics;

	BandManagerImpl& app;
	Glib::RefPtr<Gio::Cancellable> cancel;
	InertiaDataChar inertia_data;
	TimeSyncChar time_sync_count;
	GattCharacteristic inertia_zero, vibe_ctl, dis_pnpid, dis_mfg_name;
	GattCharacteristic time_sync_ctl, time_sync_tag;
	EventBurstCounter disconnects;
	std::set<BandDevice *> observers;
	BandDevice::DevIdData devid;
	std::string name;

	std::atomic_uint working;
	bool is_initialized;
	bool has_vibe;
	bool timing_master;
	bool time_synced;
	bool time_resync_pending;

	void handle_call_response(const Glib::RefPtr<Gio::AsyncResult>& res);
	void handle_pnpid_value(const Glib::RefPtr<Gio::AsyncResult>& res);
	void start_timesync(const Glib::RefPtr<Gio::AsyncResult>& res, int stage);
	void time_sync_done();
	virtual void on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties& props, const std::vector<Glib::ustring>& invd) override;
	void bond_lost();
};

struct BandManagerImpl final : public BandManager, sigc::trackable
{
	using oinfo_t = BluezObjectManager::object_interfaces_map_t::value_type;

	Glib::RefPtr<Glib::MainLoop> ioloop;
	dbus::ObjectManager om;
	GattProfileStub bleapp;
	BluezObjectManager bluez;
	std::map<Glib::DBusObjectPathString, BLEAdapter> adapters;
	std::map<Glib::DBusObjectPathString, BandDeviceImpl> devices;
	std::map<Glib::DBusObjectPathString, BandDeviceImpl *> service_dev;
	uint64_t timesync_id;
	uint64_t ts0;
	bool auto_pair;
	bool has_timing_master;

	BandManagerImpl();
	~BandManagerImpl();

	void bluez_init(Glib::RefPtr<Gio::AsyncResult>& result);
	void on_bluez_interfaces_added(const Glib::DBusObjectPathString& path, const props_t& props);
	void on_bluez_interfaces_removed(const Glib::DBusObjectPathString& path, const std::vector<Glib::ustring>& ifaces);
	void remove_device(const Glib::DBusObjectPathString& path);
	void handle_object(const oinfo_t& oinfo);

	template <typename Iface>
	bool handle_iface(void (BandManagerImpl::*handler)(const Glib::DBusObjectPathString&, const props_t&), const oinfo_t& oinfo);

	BLEAdapter& remember_adapter(const Glib::DBusObjectPathString& path);
	void init_adapter(const Glib::DBusObjectPathString& path, const props_t&);

	BandDeviceImpl& remember_device(const Glib::DBusObjectPathString& path);
	void forget_device(BandDeviceImpl *devp);
	void check_device(const Glib::DBusObjectPathString& path, const props_t& props);
	void check_service(const Glib::DBusObjectPathString& path, const props_t& props);
	void check_char(const Glib::DBusObjectPathString& path, const props_t& props);
	bool select_timing_master();
	void lost_timing_master();

	virtual std::vector<BandDeviceLL *> bands() override;
	virtual void run() override;
	virtual void stop() override;

	virtual std::future<void> adjust_zero(BandDeviceLL *dev, const BandDevice::AxisData& zero) override;
	virtual std::future<void> send_vibe(BandDeviceLL *dev, uint64_t effect, uint32_t when) override;
};

void BandManagerImpl::run()
{
	ioloop->run();
}

void BandManagerImpl::stop()
{
	ioloop->quit();
}

BandManagerImpl::BandManagerImpl()
	: ioloop(Glib::MainLoop::create()),
	  bleapp({ BAND_INERTIA_SERVICE_UUID, BAND_MOTOR_SERVICE_UUID }),
	  timesync_id(random64()), ts0(get_monotonic_clock()),
	  auto_pair(true), has_timing_master(false)
{
	om.register_on_bus(Gio::DBus::Connection::get_sync(Gio::DBus::BUS_TYPE_SYSTEM));
	om.add_object("/bands", bleapp);

	bluez.on_interfaces_added = sigc::mem_fun(this, &BandManagerImpl::on_bluez_interfaces_added);
	bluez.on_interfaces_removed = sigc::mem_fun(this, &BandManagerImpl::on_bluez_interfaces_removed);
	bluez.GetManagedObjects_async(sigc::mem_fun(this, &BandManagerImpl::bluez_init));
}

BandManagerImpl::~BandManagerImpl()
{
	adapters.clear();
	service_dev.clear();
	devices.clear();
}

void BandManagerImpl::bluez_init(Glib::RefPtr<Gio::AsyncResult>& result)
{
	Glib::Variant<BluezObjectManager::object_interfaces_map_t> objs;
	bluez.finish_call(result).get_child(objs);

	for (auto& oinfo : objs.get())
		handle_object(oinfo);
}

void BandManagerImpl::on_bluez_interfaces_added(const Glib::DBusObjectPathString& path, const props_t& props)
{
#if 0
	std::cerr << "bluez/ifAdd " << path << '\n';
#endif
	handle_object(std::make_pair(std::cref(path), std::cref(props)));
}

void BandManagerImpl::on_bluez_interfaces_removed(const Glib::DBusObjectPathString& path, const std::vector<Glib::ustring>& ifaces)
{
#if 0
	std::cerr << "bluez/ifDel " << path << '\n';
#endif
	remove_device(path);
}

void BandManagerImpl::remove_device(const Glib::DBusObjectPathString& path)
{
	adapters.erase(path);
	devices.erase(path);
}

void BandManagerImpl::handle_object(const oinfo_t& oinfo)
{
	handle_iface<org::bluez::GattManager1>(&BandManagerImpl::init_adapter, oinfo) ||
	handle_iface<org::bluez::Device1>(&BandManagerImpl::check_device, oinfo) ||
	handle_iface<org::bluez::GattService1>(&BandManagerImpl::check_service, oinfo) ||
	handle_iface<org::bluez::GattCharacteristic1>(&BandManagerImpl::check_char, oinfo);
}

template <typename Iface>
bool BandManagerImpl::handle_iface(void (BandManagerImpl::*handler)(const Glib::DBusObjectPathString&, const props_t&), const oinfo_t& oinfo)
{
	auto pg = oinfo.second.find(Iface::name);
	if (pg == oinfo.second.end())
		return false;

	(this->*handler)(oinfo.first, oinfo.second);
	return true;
}

BLEAdapter& BandManagerImpl::remember_adapter(const Glib::DBusObjectPathString& path)
{
	return adapters.emplace(std::piecewise_construct, std::forward_as_tuple(path),
		std::forward_as_tuple(path)).first->second;
}

void BandManagerImpl::init_adapter(const Glib::DBusObjectPathString& path, const props_t&)
{
	remember_adapter(path).init_adapter(bleapp);
}

BandDeviceImpl& BandManagerImpl::remember_device(const Glib::DBusObjectPathString& path)
{
	return devices.emplace(std::piecewise_construct, std::forward_as_tuple(path),
		std::forward_as_tuple(*this, path)).first->second;
}

void BandManagerImpl::forget_device(BandDeviceImpl *devp)
{
	auto p = service_dev.begin();
	while (p != service_dev.end())
		if (p->second == devp)
			p = service_dev.erase(p);
		else
			++p;
}

void BandManagerImpl::check_device(const Glib::DBusObjectPathString& path, const props_t& props)
{
	const std::vector<Glib::ustring>& uuids = get_property_from_map<org::bluez::Device1::Properties::UUIDs>(props);
	if (!uuids.empty() && std::find(uuids.begin(), uuids.end(), BAND_INERTIA_SERVICE_UUID) == uuids.end()) {
		std::cerr << "ignored dev: " << path << '\n';
		return;
	}

	bool paired = get_property_from_map<org::bluez::Device1::Properties::Paired>(props);
	if (!paired) {
		if (!auto_pair) {
			std::cerr << "ignored unpaired dev: " << path << '\n';
			return;
		}

		remember_device(path).pair();
		return;
	}

	bool connected = get_property_from_map<org::bluez::Device1::Properties::Connected>(props);
	if (!connected) {
		remember_device(path).connect();
		return;
	}

	remember_device(path).start();
}

void BandManagerImpl::check_service(const Glib::DBusObjectPathString& path, const props_t& props)
{
	const Glib::DBusObjectPathString& devpath = get_property_from_map<org::bluez::GattService1::Properties::Device>(props);

	auto dp = devices.find(devpath);
	if (dp != devices.end()) {
		if (dp->second.found_service(path, props))
			service_dev[path] = &dp->second;
		else
			std::cerr << devpath << " not interested in " << path << '\n';
	} else
		std::cerr << devpath << " not seen for " << path << '\n';
}

void BandManagerImpl::check_char(const Glib::DBusObjectPathString& path, const props_t& props)
{
	const Glib::DBusObjectPathString& svcpath = get_property_from_map<org::bluez::GattCharacteristic1::Properties::Service>(props);

	auto dp = service_dev.find(svcpath);
	if (dp != service_dev.end())
		dp->second->found_char(path, props);
	else
		std::cerr << svcpath << " not seen for " << path << '\n';
}

std::future<void> BandManagerImpl::adjust_zero(BandDeviceLL *dev, const BandDevice::AxisData& zero)
{
	return static_cast<BandDeviceImpl&>(*dev).adjust_zero(zero);
}

std::future<void> BandManagerImpl::send_vibe(BandDeviceLL *dev, uint64_t effect, uint32_t when)
{
	return static_cast<BandDeviceImpl&>(*dev).send_vibe(effect, when);
}

std::vector<BandDeviceLL *> BandManagerImpl::bands()
{
	std::vector<BandDeviceLL *> bv;

	bv.reserve(devices.size());
	for (auto& mp : devices)
		bv.push_back(std::addressof(mp.second));

	return bv;
}

bool BandManagerImpl::select_timing_master()
{
	if (has_timing_master)
		return false;

	uint64_t grp = be64toh(timesync_id) >> 24;
	std::cerr << "new timing master, group: " << std::hex << grp << std::dec << '\n';

	has_timing_master = true;
	return true;
}

void BandManagerImpl::lost_timing_master()
{
	has_timing_master = false;
	timesync_id = random64();
	uint64_t grp = be64toh(timesync_id) >> 24;

	std::cerr << "lost timing master, new group: " << std::hex << grp << std::dec << '\n';

	for (auto& p : devices)
		p.second.restart_time_sync();
}

BandDeviceImpl::BandDeviceImpl(BandManagerImpl& app_, const Glib::DBusObjectPathString& path)
	: BLEDevice(path), app(app_), inertia_data(*this), time_sync_count(*this), cancel(Gio::Cancellable::create()),
	  disconnects(3, 5), working(0), has_vibe(false), timing_master(false), time_synced(false), time_resync_pending(false)
{
}

BandDeviceImpl::~BandDeviceImpl()
{
	cancel->cancel();
	app.forget_device(this);
	std::cerr << path() << " removed.\n";

	while (!observers.empty()) {
		BandDevice *bd = *observers.begin();
		observers.erase(observers.begin());
		on_device_removed(*bd);
	}
}

void BandDeviceImpl::add_observer(BandDevice& bd) noexcept
{
	observers.insert(std::addressof(bd));
}

void BandDeviceImpl::del_observer(BandDevice& bd) noexcept
{
	observers.erase(std::addressof(bd));
}

void BandDeviceImpl::pair()
{
	if (working)
		return;

	++working;
	Pair_casync(cancel, sigc::mem_fun(this, &BandDeviceImpl::handle_call_response));
}

void BandDeviceImpl::connect()
{
	if (working)
		return;

	++working;
	Connect_casync(cancel, sigc::mem_fun(this, &BandDeviceImpl::handle_call_response));
}

bool BandDeviceImpl::found_service(const Glib::DBusObjectPathString& path, const props_t& props)
{
	const Glib::ustring& uuid = get_property_from_map<org::bluez::GattService1::Properties::UUID>(props);
	std::cerr << this->path() << " considering " << path << " uuid " << uuid << '\n';
	if (uuid == BAND_INERTIA_SERVICE_UUID || uuid == DIS_SERVICE_UUID || uuid == BAND_TIME_SERVICE_UUID)
		return true;
	if (uuid == BAND_MOTOR_SERVICE_UUID)
		return ((has_vibe = true));
	return false;
}

const BandDeviceImpl::char_map_t BandDeviceImpl::band_characteristics = {
	{ DIS_PNP_ID_UUID, &BandDeviceImpl::dis_pnpid },
	{ DIS_MFG_NAME_UUID, &BandDeviceImpl::dis_mfg_name },
	{ BAND_INERTIA_DATA_UUID, reinterpret_cast<GattCharacteristic BandDeviceImpl::*>(&BandDeviceImpl::inertia_data) },
	{ BAND_INERTIA_ZERO_UUID, &BandDeviceImpl::inertia_zero },
	{ TIME_SYNC_TAG_UUID, &BandDeviceImpl::time_sync_tag },
	{ TIME_SYNC_CONTROL_UUID, &BandDeviceImpl::time_sync_ctl },
	{ TIME_SYNC_COUNTER_UUID, reinterpret_cast<GattCharacteristic BandDeviceImpl::*>(&BandDeviceImpl::time_sync_count) },
	{ BAND_VIBE_CTL_UUID, char_info_t{&BandDeviceImpl::vibe_ctl, &BandDeviceImpl::has_vibe} },
};

void BandDeviceImpl::found_char(const Glib::DBusObjectPathString& path, const props_t& props)
{
	Glib::ustring uuid = get_property_from_map<org::bluez::GattCharacteristic1::Properties::UUID>(props);

	auto p = band_characteristics.find(uuid);
	if (p == band_characteristics.end())
		return;

	GattCharacteristic& char_obj = this->*(p->second.first);

	auto cb = [this, &char_obj, uuid = std::move(uuid)] (const Glib::RefPtr<Gio::AsyncResult>& res) {
		--working;

		try {
			char_obj.set_obj(Gio::DBus::Proxy::create_finish(res));
			std::cerr << char_obj.path() << " noticed, uuid " << uuid << '\n';
		} catch (...) {
			std::cerr << char_obj.path() << " setup call failed for char uuid " << uuid << '\n';
		}

		this->start();
	};

	++working;
	Gio::DBus::Proxy::create(obj->get_connection(), obj->get_name(), path,
		org::bluez::GattCharacteristic1::name, std::move(cb), cancel);
}

static void finish_with_char_write(
	Glib::RefPtr<Gio::Cancellable> cancel,
	std::shared_ptr<std::promise<void>> p,
	const GattCharacteristic& obj,
	std::vector<uint8_t>&& data,
	const Glib::ustring& type = Glib::ustring())
{
	auto cb = [&obj, p{std::move(p)}] (const Glib::RefPtr<Gio::AsyncResult>& res) {
		try {
			obj.finish_call(res);
			p->set_value();
		} catch (...) {
			p->set_exception(std::current_exception());
		}
	};

	std::map<Glib::ustring, Glib::VariantBase> opts;
	if (!type.empty())
		opts.emplace("type", Glib::Variant<Glib::ustring>::create(type));
	obj.WriteValue_casync(std::move(cancel), std::move(cb), std::move(data), std::move(opts));
}

std::future<void> BandDeviceImpl::adjust_zero(const BandDevice::AxisData& zero)
{
	auto p = std::make_shared<std::promise<void>>();
	std::future<void> f{p->get_future()};

	if (!inertia_zero.is_valid()) {
		p->set_exception(std::make_exception_ptr(std::runtime_error("no zero-offset characteristic found")));
		return f;
	}

	auto cb = [this, zero, p = std::move(p)] (const Glib::RefPtr<Gio::AsyncResult>& res) mutable {
		std::vector<uint8_t> data;
		try {
			Glib::Variant<std::vector<uint8_t>> val;
			inertia_zero.finish_call(res).get_child(val);
			data = val.get();
		} catch (...) {
			p->set_exception(std::current_exception());
			return;
		}

		if (data.size() != 6 * 2) {
			std::cerr << path() << " (ERR) zero char returned " << data.size() << " bytes\n";
			data.clear();
			data.resize(6 * 2, 0);
		}

		Vec<int, 6> old_zero, new_zero;
		int16_t *bz = reinterpret_cast<int16_t *>(data.data());
		const int *nz = &zero.gx;
		for (size_t i = 0; i < 6; ++i) {
			old_zero[i] = le16toh(bz[i]);
			new_zero[i] = old_zero[i] + nz[i];
			bz[i] = htole16(std::clamp<int>(new_zero[i],
				std::numeric_limits<int16_t>::min(),
				std::numeric_limits<int16_t>::max()));
		}

		std::cerr << path() << " zero { " << old_zero[0] << ' ' << old_zero[1] << ' ' << old_zero[2]
			  << "; " << old_zero[3] << ' ' << old_zero[4] << ' ' << old_zero[5]
			  << " } -> {" << new_zero[0] << ' ' << new_zero[1] << ' ' << new_zero[2]
			  << "; " << new_zero[3] << ' ' << new_zero[4] << ' ' << new_zero[5] << " }\n";

		finish_with_char_write(cancel, std::move(p), inertia_zero, std::move(data));
	};

	std::map<Glib::ustring, Glib::VariantBase> opts;
	inertia_zero.ReadValue_casync(cancel, std::move(cb), std::move(opts));

	return f;
}

std::future<void> BandDeviceImpl::send_vibe(uint64_t effect, uint32_t when)
{
	auto p = std::make_shared<std::promise<void>>();
	std::future<void> f{p->get_future()};

	if (!vibe_ctl.is_valid()) {
		p->set_exception(std::make_exception_ptr(std::runtime_error("no vibe-control characteristic found")));
		return f;
	}

	std::vector<uint8_t> data(sizeof(vibe_ctl_data));
	*reinterpret_cast<vibe_ctl_data *>(&data[0]) = { htole64(effect), htole32(when) };
	if (!when)
		data.resize(sizeof(effect));

	finish_with_char_write(cancel, std::move(p), vibe_ctl, std::move(data), "command");

	return f;
}

void BandDeviceImpl::start()
{
	if (working)
		return;

	bool svok = ServicesResolved();
	if (!svok)
		return connect();

	for (auto& p : band_characteristics)
		if (!(this->*p.second.first).is_valid() && (!p.second.second || this->*p.second.second))
			return; /* wait */

	std::cerr << path() << " got chars, device " << (is_initialized ? "" : "NOT ") << "initialized, "
		  << (time_synced ? "" : "NOT ") << "synced\n";

	if (!is_initialized || time_resync_pending)
		restart_time_sync();

	if (is_initialized) {
		auto cb = [this] (const Glib::RefPtr<Gio::AsyncResult>& res) {
			--working;
			try {
				inertia_data.finish_call(res);
			} catch (Glib::Error& e) {
				std::cerr << inertia_data.path() << " failed to enable notify: " << e.what() << '\n';
			}
			this->start();
		};

		try {
			if (!inertia_data.Notifying())
				inertia_data.StartNotify_casync(cancel, std::move(cb));
		} catch (...) {
			inertia_data.StartNotify_casync(cancel, std::move(cb));
		}

		return;
	}

	++working;
	dis_pnpid.ReadValue_casync(cancel, sigc::mem_fun(this, &BandDeviceImpl::handle_pnpid_value), std::map<Glib::ustring, Glib::VariantBase>());
}

void BandDeviceImpl::handle_pnpid_value(const Glib::RefPtr<Gio::AsyncResult>& res)
{
	try {
		Glib::Variant<std::vector<uint8_t>> vval;
		dis_pnpid.finish_call(res).get_child(vval);
		auto val = vval.get();

		if (val.size() != sizeof(struct pnpid_data))
			throw std::runtime_error("bad PNP_ID data size");

		name = Name();
		const auto& id = *(const struct pnpid_data *)val.data();
		devid.registry = id.id_source;
		devid.vendor = le16toh(id.vendor);
		devid.product = le16toh(id.product);
		devid.version = le16toh(id.version);

		inertia_data.ts_offset = app.ts0;

		try {
			if (app.on_new_band)
				app.on_new_band(this);
		} catch (...) {
			/* ignore */
		}

		restart_time_sync();
		--working;
	} catch (...) {
		--working;
		this->start();
	}
}

void BandDeviceImpl::start_timesync(const Glib::RefPtr<Gio::AsyncResult>& res, int stage)
{
	try {
		switch (stage) {
			case 0:
			case 3:
			case 6:
				time_sync_ctl.finish_call(res);
				break;
			case 1:
				time_sync_tag.finish_call(res);
				break;
			case 5:
				time_sync_count.finish_call(res);
				break;
			default:
				break;
		}

		++stage;
	} catch (...) {
		std::cerr << path() << " failed time sync stage " << stage << " --- retrying\n";
	}

	if (time_resync_pending) {
		time_resync_pending = false;
		stage = 0;
	}

	if (stage == 2) {
		++stage;
		timing_master = app.select_timing_master();
		if (!timing_master)
			stage += 2;
	}

	std::cerr << path() << " starting time sync stage " << stage << "\n";

	auto cb = sigc::bind(sigc::mem_fun(this, &BandDeviceImpl::start_timesync), stage);
	std::map<Glib::ustring, Glib::VariantBase> opts;

	std::vector<uint8_t> data;
	bool done = false;

	switch (stage) {
	case 0:
		// disable
		data.resize(1, 0);
		time_sync_ctl.WriteValue_casync(cancel, std::move(cb), std::move(data), std::move(opts));
		time_synced = false;
		break;

	case 1:
		// set group key
		data.resize(5, 0);
		memcpy(&data[0], &app.timesync_id, data.size());
		time_sync_tag.WriteValue_casync(cancel, std::move(cb), std::move(data), std::move(opts));
		break;

	case 2:
		__builtin_unreachable();
		break;

	case 3:	// = MAKE_TIMING_MASTER
		// enable tx
		data.resize(1, 2);
		time_sync_ctl.WriteValue_casync(cancel, std::move(cb), std::move(data), std::move(opts));
		break;

	case 4:
		time_sync_done();
		break;

	case 5:
		// enable sync notify
		time_sync_count.StartNotify_casync(cancel, std::move(cb));
		break;

	case 6:
		// enable rx
		data.resize(1, 1);
		time_sync_ctl.WriteValue_casync(cancel, std::move(cb), std::move(data), std::move(opts));
		break;

	case 7:
		/* wait for sync */
		break;
	}
}

void BandDeviceImpl::time_sync_done()
{
	--working;

	if (!time_synced)
		time_synced = true;

	if (!is_initialized) {
		is_initialized = true;
		for (BandDevice *bd : observers)
			on_device_initialized(*bd, name, inertia_data.ts_offset, devid);
	}

	this->start();
}

void BandDeviceImpl::restart_time_sync()
{
	if (working) {
		time_resync_pending = true;
		return;
	}

	++working;
	Glib::RefPtr<Gio::AsyncResult> res;
	time_synced = false;
	start_timesync(res, -1);
}

void BandDeviceImpl::handle_call_response(const Glib::RefPtr<Gio::AsyncResult>& res)
{
	--working;

	try {
		finish_call(res);
	} catch (std::exception& e) {
		std::cerr << path() << " connect failed: [std] " << e.what() << '\n';
	} catch (Glib::Error& e) {
		if (e.matches(g_io_error_quark(), G_IO_ERROR_CANCELLED))
			return;

		if (e.matches(g_io_error_quark(), G_IO_ERROR_DBUS_ERROR)) {
			if (match_bluez_EINPROGRESS(e)) {
				/* just ignore and wait for props changed event */;
			} else {
				std::cerr << path() << " connect failed: [dbus] " << e.what() << '\n';
				if (match_bluez_AUTHFAILED(e))
					bond_lost();
			}
		} else {
			std::cerr << path() << " connect failed: [glib] " << g_quark_to_string(e.domain()) << '/' << e.code() << ": " << e.what() << '\n';
		}
	}
}

void BandDeviceImpl::bond_lost()
{
	std::cerr << path() << " assuming bond information lost - removing.\n";

	is_initialized = false;
	time_synced = false;

	if (timing_master) {
		timing_master = false;
		app.lost_timing_master();
	}

	auto a = adapter();
	a.RemoveDevice_async(std::bind(ignore_call_result, a.get_obj(), std::placeholders::_1), path());
}

void BandDeviceImpl::on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties& props, const std::vector<Glib::ustring>& invd)
{
	WrappedObject::on_properties_changed(props, invd);

	try {
		bool connected = get_property_from_map<org::bluez::Device1::Properties::Connected>(props);
		if (!connected && disconnects.notice_event())
			return bond_lost();
	} catch (...) {
		/* continue */
	}

	if (!get_property<org::bluez::Device1::Properties::Connected>())
		return connect();

	try {
		bool svok = get_property_from_map<org::bluez::Device1::Properties::ServicesResolved>(props);
		if (!svok)
			return bond_lost();
	} catch (...) {
		/* continue */
	}

	if (!get_property<org::bluez::Device1::Properties::ServicesResolved>())
		return;

	static const std::string *const prop_names[] = {
		&org::bluez::Device1::Properties::Name::name,
		&org::bluez::Device1::Properties::UUIDs::name,
		&org::bluez::Device1::Properties::Connected::name,
		&org::bluez::Device1::Properties::Paired::name,
		&org::bluez::Device1::Properties::ServicesResolved::name,
		&org::bluez::Device1::Properties::Adapter::name,
	};

	props_t pmap;
	auto& prop = pmap[org::bluez::Device1::name];
	for (const auto& n : prop_names)
		obj->get_cached_property(prop[*n], *n);

	app.check_device(path(), std::move(pmap));
}

InertiaDataChar::InertiaDataChar(BandDeviceImpl& dev_)
	: dev(dev_), ts_offset(dev.app.ts0), past_half(false)
{
}

uint64_t InertiaDataChar::fixup_timestamp(uint32_t ts)
{
	uint32_t margin = 1u << 28, half = 1u << 31;

	if (past_half && margin < ts && ts < half) {
		past_half = false;
		ts_offset += 1ull << 32;
		dev.app.ts0 += 1ull << 32;
	} else if (!past_half && ts > half + margin) {
		past_half = true;
	}

	uint64_t fts = ts_offset + ts;

	if (past_half && (int32_t)ts >= 0)
		fts += 1ull << 32;

	return fts;
}

void InertiaDataChar::on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties& props, const std::vector<Glib::ustring>& invd)
{
	if (!dev.is_initialized)
		return GattCharacteristic::on_properties_changed(props, invd);

	try {
		auto indata = get_property_from_map<org::bluez::GattCharacteristic1::Properties::Value>(props);
		auto p = reinterpret_cast<const struct band_data *>(indata.data());
		if (indata.size() != sizeof(*p))
			throw std::runtime_error("band inertial sensor data size");

		BandDevice::SensorData data;
		data.timestamp = fixup_timestamp(le32toh(p->ts));
		data.v.ax = (int16_t)le16toh(p->ax);
		data.v.ay = (int16_t)le16toh(p->ay);
		data.v.az = (int16_t)le16toh(p->az);
		data.v.gx = (int16_t)le16toh(p->gx);
		data.v.gy = (int16_t)le16toh(p->gy);
		data.v.gz = (int16_t)le16toh(p->gz);

		for (BandDevice *bd : dev.observers)
			dev.on_data_received(*bd, data);
	} catch (Glib::Error& e) {
		GattCharacteristic::on_properties_changed(props, invd);
		std::cerr << path() << " propchange: [glib] " << e.what() << '\n';
	} catch (std::exception& e) {
		GattCharacteristic::on_properties_changed(props, invd);
		std::cerr << path() << " propchange: [std] " << e.what() << '\n';
	}
}

void TimeSyncChar::on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties& props, const std::vector<Glib::ustring>& invd)
{
	GattCharacteristic::on_properties_changed(props, invd);
	dev.time_sync_done();
}

} // anonymous namespace

std::unique_ptr<BandManager> BandManager::create()
{
	return std::make_unique<BandManagerImpl>();
}

static BandDeviceImpl& LL(BandDeviceLL *ptr) noexcept
{
	return static_cast<BandDeviceImpl&>(*ptr);
}

BandDevice::~BandDevice() noexcept
{
	if (dev)
		LL(dev).del_observer(*this);
}

BandDevice& BandDevice::operator=(BandDeviceLL *band) noexcept
{
	if (dev)
		LL(dev).del_observer(*this);

	dev = band;

	if (dev)
		LL(dev).add_observer(*this);

	return *this;
}

void BandDevice::device_initialized(const std::string& name, uint64_t ts, const DevIdData& id) noexcept
{
	/* just a notification */
}

std::future<void> BandDevice::adjust_zero(const AxisData& zero)
{
	return LL(dev).adjust_zero(zero);
}

std::future<void> BandDevice::send_vibe(uint64_t effect, uint32_t when)
{
	return LL(dev).send_vibe(effect, when);
}

static void init_giomm() __attribute__((constructor));
static void init_giomm()
{
	Gio::init();
}
