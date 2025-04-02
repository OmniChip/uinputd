#include "ble.h"
#include "dbus-base.h"

#include <cstdio>
#include <iostream>

#include <unistd.h>

static const Glib::ustring BLUEZ_BUSNAME{"org.bluez"};

static const std::string BT_BASE_UUID{"00000000-0000-1000-8000-00805f9b34fb"};
static const std::string DEVINFO_SERVICE_UUID{"0000180a-0000-1000-8000-00805f9b34fb"};
static const std::string PNPID_CHAR_UUID{"00002a50-0000-1000-8000-00805f9b34fb"};

static const char xml_bluez_info[] = R"XML_DELIMITER(
<!DOCTYPE node PUBLIC "-//freedesktop//DTD D-BUS Object Introspection 1.0//EN" "http://www.freedesktop.org/standards/dbus/1.0/introspect.dtd">
<node>
<interface name="org.bluez.GattProfile1">
	<method name="Release"></method>
	<property name="UUIDs" type="as" access="read"></property>
</interface>
</node>
)XML_DELIMITER";

static const Glib::RefPtr<Gio::DBus::NodeInfo>& nodeinfo()
{
	static uintptr_t initialized = 0;
	static Glib::RefPtr<Gio::DBus::NodeInfo> nodeinfo;

	if (g_once_init_enter(&initialized)) {
		nodeinfo = Gio::DBus::NodeInfo::create_for_xml(xml_bluez_info);
		g_once_init_leave(&initialized, 1);
	}

	return nodeinfo;
}

template <typename Iface>
static Glib::RefPtr<Gio::DBus::InterfaceInfo> interface_info()
{
	return nodeinfo()->lookup_interface(Iface::name);
}

std::string BLEDevice::format_addr(const addr_t& addr, char sep)
{
	std::string buf(3 * 6 - 1, '\0');

	sprintf(const_cast<char *>(buf.data()), "%02hhX%c%02hhX%c%02hhX%c%02hhX%c%02hhX%c%02hhX",
		addr[0], sep, addr[1], sep, addr[2], sep, addr[3], sep, addr[4], sep, addr[5]);

	return buf;
}

WrappedBTObjectBase::WrappedBTObjectBase(const Glib::DBusObjectPathString& path, const Glib::ustring& iface)
	: WrappedObject(Gio::DBus::BUS_TYPE_SYSTEM, BLUEZ_BUSNAME, path, iface)
{
}

BluezObjectManager::BluezObjectManager()
	: ObjectManagerProxy(Gio::DBus::BUS_TYPE_SYSTEM, BLUEZ_BUSNAME)
{
}

GattManager::GattManager(const std::string& hci, int)
	: WrappedBTObject("/org/bluez/" + hci)
{
}

BLEDevice::BLEDevice(const addr_t& addr, const std::string& hci)
	: WrappedBTObject("/org/bluez/" + hci + "/dev_" + format_addr(addr, '_'))
{
}

static bool starts_with(const std::string& a, const std::string& b)
{
	return a.substr(0, b.size()) == b;
}

using obj_match_fn = void (const Glib::DBusObjectPathString& /*this*/, const Glib::DBusObjectPathString& /*parent*/);

static void grep_objects(const org::freedesktop::DBus::ObjectManager::object_interfaces_map_t& pathinfo,
	const std::string& interface, const std::string& parent_prop, std::function<obj_match_fn> process)
{
	for (const auto& pathifaces : pathinfo) {
		auto p = pathifaces.second.find(interface);
		if (p == pathifaces.second.end())
			continue;

		const auto& props = p->second;
		auto q = props.find(parent_prop);
		if (q == props.end())
			continue;

		auto parent_path = Glib::VariantBase::cast_dynamic<Glib::Variant<Glib::DBusObjectPathString>>(q->second);
		process(pathifaces.first, parent_path.get());
	}
}

void BLEDevice::refresh_gatt_info()
{
	service_map.clear();

	auto&& pathinfo = dbus::ObjectManagerProxy(Gio::DBus::BUS_TYPE_SYSTEM, BLUEZ_BUSNAME).GetManagedObjects();

	/* find services */

	grep_objects(pathinfo, org::bluez::GattService1::name, "Device", [this](auto& objpath, auto& parent_path) {
		if (parent_path == path())
			service_map.emplace(objpath, objpath);
	});

	/* find service characteristics */

	std::map<Glib::DBusObjectPathString, GattCharacteristic&> all_chars;
	grep_objects(pathinfo, org::bluez::GattCharacteristic1::name, "Service", [this, &all_chars](auto& objpath, auto& parent_path) {
		auto p = service_map.find(parent_path);
		if (p == service_map.end())
			return;

		auto& chr = p->second.char_map.emplace(objpath, objpath).first->second;
		all_chars.emplace(objpath, chr);
	});

	/* find characteristic descriptors */

	grep_objects(pathinfo, org::bluez::GattDescriptor1::name, "Characteristic", [this, &all_chars](auto& objpath, auto& parent_path) {
		auto p = all_chars.find(parent_path);
		if (p == all_chars.end())
			return;

		p->second.desc_map.emplace(objpath, objpath);
	});
}

template <typename MappedT>
static MappedT& grep_children_by_uuid(
	std::map<Glib::DBusObjectPathString, MappedT>& child_map, const std::string& uuid)
{
	for (auto& pathchild : child_map) {
		if (pathchild.second.UUID() == uuid)
			return pathchild.second;
	}

	throw std::system_error(std::make_error_code(std::errc::no_such_file_or_directory));
}

GattService& BLEDevice::find_service(const std::string& uuid)
{
	return grep_children_by_uuid(service_map, uuid);
}

GattCharacteristic& GattService::find_characteristic(const std::string& uuid)
{
	return grep_children_by_uuid(char_map, uuid);
}

GattDescriptor& GattCharacteristic::find_descriptor(const std::string& uuid)
{
	return grep_children_by_uuid(desc_map, uuid);
}

std::vector<uint8_t> BLEDevice::get_pnpid()
{
	try {
		GattService& dis = find_service(DEVINFO_SERVICE_UUID);
		GattCharacteristic& chr = dis.find_characteristic(PNPID_CHAR_UUID);
		return chr.Value();
	} catch (std::system_error) {
		return std::vector<uint8_t>{};
	}
}

GattProfileStub::GattProfileStub(std::vector<Glib::ustring> uuids_)
	: StubObject{::interface_info<org::bluez::GattProfile1>()}, uuids{std::move(uuids_)}
{
}

void GattProfileStub::call_method(Glib::VariantBase& ret, const Glib::ustring& interface_name, const Glib::ustring& method_name, const Glib::VariantBase& args)
{
	if (method_matches<Methods::Release>(interface_name, method_name))
		Release();
	else
		StubObject::call_method(ret, interface_name, method_name, args);
}

void GattProfileStub::get_property(Glib::VariantBase& var,
	const Glib::ustring& interface_name,
	const Glib::ustring& property_name) const
{
	if (method_matches<Properties::UUIDs>(interface_name, property_name))
		var = Glib::Variant<std::vector<Glib::ustring>>::create(uuids);
	else
		StubObject::get_property(var, interface_name, property_name);
}

void GattProfileStub::Release()
{
	std::cerr << "GattProfile::Release()\n";
}
