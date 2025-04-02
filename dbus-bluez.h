#ifndef DBUS_ORG_BLUEZ_H_
#define DBUS_ORG_BLUEZ_H_

#include "dbus.h"

namespace org::bluez {

struct Adapter1
{
	DEF_DBUS_IFACE_NAME("org.bluez.Adapter1");

	using discovery_filter_t = std::map<Glib::ustring, Glib::VariantBase>;
 
	struct Properties
	{
		DEF_DBUS_RO_PROPERTY(Adapter1, Discovering, bool);
		DEF_DBUS_RW_PROPERTY(Adapter1, Discoverable, bool);
		DEF_DBUS_RW_PROPERTY(Adapter1, Pairable, bool);
		DEF_DBUS_RW_PROPERTY(Adapter1, Powered, bool);
		DEF_DBUS_RO_PROPERTY(Adapter1, Address, Glib::ustring);
		DEF_DBUS_RO_PROPERTY(Adapter1, AddressType, Glib::ustring);
		DEF_DBUS_RW_PROPERTY(Adapter1, Alias, Glib::ustring);
		DEF_DBUS_RO_PROPERTY(Adapter1, Modalias, Glib::ustring);
		DEF_DBUS_RO_PROPERTY(Adapter1, Name, Glib::ustring);
		DEF_DBUS_RW_PROPERTY(Adapter1, DiscoverableTimeout, uint32_t);
		DEF_DBUS_RW_PROPERTY(Adapter1, PairableTimeout, uint32_t);
		DEF_DBUS_RW_PROPERTY(Adapter1, UUIDs, std::vector<Glib::ustring>);
  	};

	struct Methods
	{
		DEF_DBUS_CALL(Adapter1, StartDiscovery, void);
		DEF_DBUS_CALL(Adapter1, StopDiscovery, void);
		DEF_DBUS_CALL(Adapter1, RemoveDevice, void, Glib::DBusObjectPathString);
		DEF_DBUS_CALL(Adapter1, GetDiscoveryFilters, std::vector<Glib::ustring>);
		DEF_DBUS_CALL(Adapter1, SetDiscoveryFilter, void, discovery_filter_t);
	};
};

struct Device1
{
	DEF_DBUS_IFACE_NAME("org.bluez.Device1");

	struct Properties
	{
		DEF_DBUS_RO_PROPERTY(Device1, Name, Glib::ustring);
		DEF_DBUS_RO_PROPERTY(Device1, Connected, bool);
		DEF_DBUS_RO_PROPERTY(Device1, Paired, bool);
		DEF_DBUS_RO_PROPERTY(Device1, ServicesResolved, bool);
		DEF_DBUS_RW_PROPERTY(Device1, Trusted, bool);
		DEF_DBUS_RO_PROPERTY(Device1, Adapter, Glib::DBusObjectPathString);
		DEF_DBUS_RO_PROPERTY(Device1, UUIDs, std::vector<Glib::ustring>);
  	};

	struct Methods
	{
		DEF_DBUS_CALL(Device1, Connect, void);
		DEF_DBUS_CALL(Device1, Disconnect, void);
		DEF_DBUS_CALL(Device1, Pair, void);
		DEF_DBUS_CALL(Device1, CancelPairing, void);
	};
};

struct GattManager1
{
	DEF_DBUS_IFACE_NAME("org.bluez.GattManager1");

	using options_t = std::map<Glib::ustring, Glib::VariantBase>;

	struct Methods
	{
		DEF_DBUS_CALL(GattManager1, RegisterApplication, void, Glib::DBusObjectPathString, options_t);
		DEF_DBUS_CALL(GattManager1, UnregisterApplication, void, Glib::DBusObjectPathString);
	};
};

struct GattProfile1
{
	DEF_DBUS_IFACE_NAME("org.bluez.GattProfile1");

	struct Methods
	{
		DEF_DBUS_CALL(GattProfile1, Release, void);
	};

	struct Properties
	{
		DEF_DBUS_RO_PROPERTY(GattProfile1, UUIDs, std::vector<Glib::ustring>);
  	};
};

struct GattService1
{
	DEF_DBUS_IFACE_NAME("org.bluez.GattService1");

	struct Properties
	{
		DEF_DBUS_RO_PROPERTY(GattService1, Includes, std::vector<Glib::DBusObjectPathString>);
		DEF_DBUS_RO_PROPERTY(GattService1, Primary, bool);
		DEF_DBUS_RO_PROPERTY(GattService1, Device, Glib::DBusObjectPathString);
		DEF_DBUS_RO_PROPERTY(GattService1, UUID, Glib::ustring);
  	};
};

struct GattCharacteristic1
{
	DEF_DBUS_IFACE_NAME("org.bluez.GattCharacteristic1");

	struct Properties
	{
		DEF_DBUS_RO_PROPERTY(GattCharacteristic1, Value, std::vector<uint8_t>);
		DEF_DBUS_RO_PROPERTY(GattCharacteristic1, Flags, std::vector<Glib::ustring>);
		DEF_DBUS_RO_PROPERTY(GattCharacteristic1, NotifyAcquired, bool);
		DEF_DBUS_RO_PROPERTY(GattCharacteristic1, Notifying, bool);
		DEF_DBUS_RO_PROPERTY(GattCharacteristic1, WriteAcquired, bool);
		DEF_DBUS_RO_PROPERTY(GattCharacteristic1, Service, Glib::DBusObjectPathString);
		DEF_DBUS_RO_PROPERTY(GattCharacteristic1, UUID, Glib::ustring);
  	};

	struct Methods
	{
		DEF_DBUS_CALL(GattCharacteristic1, ReadValue, std::vector<uint8_t>, std::map<Glib::ustring, Glib::VariantBase>);
		DEF_DBUS_CALL(GattCharacteristic1, WriteValue, void, std::vector<uint8_t>, std::map<Glib::ustring, Glib::VariantBase>);
		DEF_DBUS_CALL(GattCharacteristic1, StartNotify, void);
		DEF_DBUS_CALL(GattCharacteristic1, StopNotify, void);
	};
};

struct GattDescriptor1
{
	DEF_DBUS_IFACE_NAME("org.bluez.GattDescriptor1");

	struct Properties
	{
		DEF_DBUS_RO_PROPERTY(GattDescriptor1, Value, std::vector<uint8_t>);
		DEF_DBUS_RO_PROPERTY(GattDescriptor1, Characteristic, Glib::DBusObjectPathString);
		DEF_DBUS_RO_PROPERTY(GattDescriptor1, UUID, Glib::ustring);
  	};
};

} /* namespace org::bluez */


#endif /* DBUS_ORG_BLUEZ_H_ */
