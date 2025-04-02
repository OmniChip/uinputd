#ifndef GAMEINN_BLE_H_
#define GAMEINN_BLE_H_

#include <string>
#include <atomic>

#include <giomm.h>

#include "dbus-base.h"
#include "dbus-bluez.h"

struct WrappedBTObjectBase : public dbus::WrappedObject
{
	using WrappedObject::WrappedObject;
	WrappedBTObjectBase(const Glib::DBusObjectPathString& path, const Glib::ustring& iface);
};

template <typename Iface>
struct WrappedBTObject : public WrappedBTObjectBase
{
	using WrappedBTObjectBase::WrappedBTObjectBase;
	explicit WrappedBTObject(const Glib::DBusObjectPathString& path)
		: WrappedBTObjectBase(path, Iface::name) {}
};

struct BLEDevice;
struct BTAdapter;

struct GattDescriptor : public WrappedBTObject<org::bluez::GattDescriptor1>
{
	friend class BLEDevice;

	FORWARD_PROP_READ(org::bluez::GattDescriptor1, Characteristic);
	FORWARD_PROP_READ(org::bluez::GattDescriptor1, UUID);
	FORWARD_PROP_READ(org::bluez::GattDescriptor1, Value);

	using WrappedBTObject::WrappedBTObject;
};

struct GattCharacteristic : public WrappedBTObject<org::bluez::GattCharacteristic1>
{
	friend class BLEDevice;

	using WrappedBTObject::WrappedBTObject;

	FORWARD_PROP_READ(org::bluez::GattCharacteristic1, Service);
	FORWARD_PROP_READ(org::bluez::GattCharacteristic1, UUID);
	FORWARD_PROP_READ(org::bluez::GattCharacteristic1, Value);
	FORWARD_PROP_READ(org::bluez::GattCharacteristic1, Notifying);

	FORWARD_CALL(org::bluez::GattCharacteristic1, ReadValue)
	FORWARD_CALL(org::bluez::GattCharacteristic1, WriteValue)
	FORWARD_CALL(org::bluez::GattCharacteristic1, StartNotify)
	FORWARD_CALL(org::bluez::GattCharacteristic1, StopNotify)

	const auto& descriptors() const { return desc_map; }
	GattDescriptor& find_descriptor(const std::string& uuid);
private:
	std::map<Glib::DBusObjectPathString, GattDescriptor> desc_map;
};

struct GattService : public WrappedBTObject<org::bluez::GattService1>
{
	friend class BLEDevice;

	using WrappedBTObject::WrappedBTObject;

	FORWARD_PROP_READ(org::bluez::GattService1, Device);
	FORWARD_PROP_READ(org::bluez::GattService1, UUID);

	const auto& characteristics() const { return char_map; }
	GattCharacteristic& find_characteristic(const std::string& uuid);
private:
	std::map<Glib::DBusObjectPathString, GattCharacteristic> char_map;
};

struct BLEDevice : public WrappedBTObject<org::bluez::Device1>
{
	using addr_t = uint8_t [6];

	using WrappedBTObject::WrappedBTObject;
	BLEDevice(const addr_t& addr, const std::string& hci = std::string{"hci0"});

	static std::string format_addr(const addr_t& addr, char sep = ':');

	FORWARD_PROP_READ(org::bluez::Device1, Name)
	FORWARD_PROP_READ(org::bluez::Device1, Connected)
	FORWARD_PROP_READ(org::bluez::Device1, Paired)
	FORWARD_PROP_READ(org::bluez::Device1, ServicesResolved)
	FORWARD_PROP_READ(org::bluez::Device1, Adapter)

	FORWARD_CALL(org::bluez::Device1, Connect)
	FORWARD_CALL(org::bluez::Device1, Disconnect)
	FORWARD_CALL(org::bluez::Device1, Pair)
	FORWARD_CALL(org::bluez::Device1, CancelPairing)

	void full_connect();

	const auto& services() const { return service_map; }
	GattService& find_service(const std::string& uuid);

	BTAdapter adapter() const;

	std::vector<uint8_t> get_pnpid();
private:
	std::map<Glib::DBusObjectPathString, GattService> service_map;

	void refresh_gatt_info();
};

struct GattManager : public WrappedBTObject<org::bluez::GattManager1>
{
	using WrappedBTObject::WrappedBTObject;
	GattManager(const std::string& hci, int);

	FORWARD_CALL(org::bluez::GattManager1, RegisterApplication)
	FORWARD_CALL(org::bluez::GattManager1, UnregisterApplication)

	BTAdapter adapter() const;
};

struct BTAdapter : public WrappedBTObject<org::bluez::Adapter1>
{
	using WrappedBTObject::WrappedBTObject;

	FORWARD_CALL(org::bluez::Adapter1, GetDiscoveryFilters)
	FORWARD_CALL(org::bluez::Adapter1, SetDiscoveryFilter)
	FORWARD_CALL(org::bluez::Adapter1, StartDiscovery)
	FORWARD_CALL(org::bluez::Adapter1, StopDiscovery)
	FORWARD_CALL(org::bluez::Adapter1, RemoveDevice)
};

inline BTAdapter BLEDevice::adapter() const { return BTAdapter(Adapter()); }
inline BTAdapter GattManager::adapter() const { return BTAdapter(path()); }

struct BluezObjectManager : public dbus::ObjectManagerProxy
{
	using interface_properties_map_t = org::freedesktop::DBus::ObjectManager::interface_properties_map_t;
	using object_interfaces_map_t = org::freedesktop::DBus::ObjectManager::object_interfaces_map_t;

	BluezObjectManager();
};

struct GattProfileStub : public dbus::StubObject, private org::bluez::GattProfile1
{
	const std::vector<Glib::ustring> uuids;

	explicit GattProfileStub(std::vector<Glib::ustring> uuids);
	GattProfileStub(const GattProfileStub&) = delete;

	virtual void call_method(Glib::VariantBase& ret, const Glib::ustring& interface_name, const Glib::ustring& method_name, const Glib::VariantBase& args) override;
	virtual void get_property(Glib::VariantBase& var, const Glib::ustring& iface, const Glib::ustring& propname) const override;
private:
	void Release();
};

#endif /* GAMEINN_BLE_H_ */
