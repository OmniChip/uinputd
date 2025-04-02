#include "dbus-base.h"

#include <iostream>

using namespace std::placeholders;

namespace dbus {

static const char xml_dbus_info[] = R"XML_DELIMITER(
<!DOCTYPE node PUBLIC "-//freedesktop//DTD D-BUS Object Introspection 1.0//EN" "http://www.freedesktop.org/standards/dbus/1.0/introspect.dtd">
<node>
<interface name="org.freedesktop.DBus.Introspectable">
	<method name="Introspect"><arg name="xml" type="s" direction="out"/></method>
</interface>
<interface name="org.freedesktop.DBus.ObjectManager">
	<method name="GetManagedObjects"><arg name="objects" type="a{oa{sa{sv}}}" direction="out"/></method>
	<signal name="InterfacesAdded"><arg name="object" type="o"/><arg name="interfaces" type="a{sa{sv}}"/></signal>
	<signal name="InterfacesRemoved"><arg name="object" type="o"/><arg name="interfaces" type="as"/></signal>
</interface>
<interface name="org.freedesktop.DBus.Properties">
	<method name="Get"><arg name="interface" type="s" direction="in"/><arg name="name" type="s" direction="in"/><arg name="value" type="v" direction="out"/></method>
	<method name="Set"><arg name="interface" type="s" direction="in"/><arg name="name" type="s" direction="in"/><arg name="value" type="v" direction="in"/></method>
	<method name="GetAll"><arg name="interface" type="s" direction="in"/><arg name="properties" type="a{sv}" direction="out"/></method>
	<signal name="PropertiesChanged"><arg name="interface" type="s"/><arg name="changed_properties" type="a{sv}"/><arg name="invalidated_properties" type="as"/></signal>
</interface>
</node>
)XML_DELIMITER";

static const Glib::RefPtr<Gio::DBus::NodeInfo>& nodeinfo()
{
	static uintptr_t initialized = 0;
	static Glib::RefPtr<Gio::DBus::NodeInfo> nodeinfo;

	if (g_once_init_enter(&initialized)) {
		nodeinfo = Gio::DBus::NodeInfo::create_for_xml(xml_dbus_info);
		g_once_init_leave(&initialized, 1);
	}

	return nodeinfo;
}

template <typename Iface>
static Glib::RefPtr<Gio::DBus::InterfaceInfo> interface_info()
{
	return nodeinfo()->lookup_interface(Iface::name);
}

WrappedObject::WrappedObject(Gio::DBus::BusType bus, const Glib::ustring& busname,
				   const Glib::DBusObjectPathString& path, const Glib::ustring& iface)
	: prop_changes(0), obj(Gio::DBus::Proxy::create_for_bus_sync(bus, busname, path, iface))
{
	set_signal();
}

WrappedObject::WrappedObject(const Glib::RefPtr<Gio::DBus::Proxy>& obj_)
	: prop_changes(0), obj(obj_)
{
	set_signal();
}

WrappedObject::~WrappedObject()
{
	propchg_conn.disconnect();
}

void WrappedObject::set_obj(const Glib::RefPtr<Gio::DBus::Proxy>& obj_)
{
	propchg_conn.disconnect();
	obj = obj_;
	set_signal();
}

void WrappedObject::set_signal()
{
	if (obj)
		propchg_conn = obj->signal_properties_changed().connect(sigc::mem_fun(this, &WrappedObject::on_properties_changed));
}

void WrappedObject::on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties& pp, const std::vector<Glib::ustring>& ip)
{
	std::cerr << "PropsChanged: " << ++prop_changes << "; on " << path() << '\n';

	for (auto& prop : pp)
		std::cerr << "\t" << prop.first << " =\t" << prop.second.get_type_string() << ": " << prop.second.print() << '\n';
	for (auto& prop : ip)
		std::cerr << "\t" << prop << " ---\n";
}

ObjectManagerProxy::ObjectManagerProxy(Gio::DBus::BusType bus, const Glib::ustring& busname,
				   const Glib::DBusObjectPathString& path)
	: WrappedObject(bus, busname, path, org::freedesktop::DBus::ObjectManager::name)
{
	obj->signal_signal().connect(sigc::mem_fun(this, &ObjectManagerProxy::on_mgr_signal));
}

void ObjectManagerProxy::on_mgr_signal(const Glib::ustring& /*sender_name*/, const Glib::ustring& signal_name, const Glib::VariantContainerBase& parameters)
{
	using interface_properties_map_t = org::freedesktop::DBus::ObjectManager::interface_properties_map_t;

	if (signal_name == org::freedesktop::DBus::ObjectManager::Signals::InterfacesAdded::name) {
		auto args = Glib::VariantBase::cast_dynamic<Glib::Variant<std::tuple<Glib::DBusObjectPathString, interface_properties_map_t>>>(parameters).get();
		on_interfaces_added(std::get<0>(args), std::get<1>(args));
	} else if (signal_name == org::freedesktop::DBus::ObjectManager::Signals::InterfacesRemoved::name) {
		auto args = Glib::VariantBase::cast_dynamic<Glib::Variant<std::tuple<Glib::DBusObjectPathString, std::vector<Glib::ustring>>>>(parameters).get();
		on_interfaces_removed(std::get<0>(args), std::get<1>(args));
	}
}

StubObject::StubObject(Glib::RefPtr<Gio::DBus::InterfaceInfo> interface_info_)
	: dbus_vtable{
		sigc::mem_fun(this, &StubObject::on_interface_method_call),
		sigc::mem_fun(this, &StubObject::on_interface_get_property),
		sigc::mem_fun(this, &StubObject::on_interface_set_property)},
	  interface_info{std::move(interface_info_)}
{
}

StubObject::~StubObject() = default;

void StubObject::call_method(Glib::VariantBase& ret, const Glib::ustring& iface, const Glib::ustring& methodname, const Glib::VariantBase& args)
{
}

void StubObject::get_property(Glib::VariantBase& var, const Glib::ustring& iface, const Glib::ustring& propname) const
{
}

bool StubObject::set_property(const Glib::ustring& /*iface*/, const Glib::ustring& /*propname*/, const Glib::VariantBase& /*var*/)
{
	return false;
}

void StubObject::on_interface_method_call(
	const Glib::RefPtr<Gio::DBus::Connection>& /*connection*/,
	const Glib::ustring& /*sender*/,
	const Glib::ustring& /*object_path*/,
	const Glib::ustring& interface_name,
	const Glib::ustring& method_name,
	const Glib::VariantContainerBase& parameters,
	const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation)
{
	try {
		Glib::VariantContainerBase ret;
		call_method(ret, interface_name, method_name, parameters);
		if (ret.is_of_type(Glib::VARIANT_TYPE_ANY))
			invocation->return_value(ret);
	} catch (Glib::Error& e) {
		invocation->return_error(e);
	} catch (std::bad_alloc& e) {
		invocation->return_error(Gio::DBus::Error(Gio::DBus::Error::NO_MEMORY, e.what()));
	} catch (std::exception& e) {
		invocation->return_error(Gio::DBus::Error(Gio::DBus::Error::FAILED, e.what()));
	} catch (...) {
		invocation->return_error(Gio::DBus::Error(Gio::DBus::Error::FAILED, ""));
	}
}

void StubObject::on_interface_get_property(
	Glib::VariantBase& property,
	const Glib::RefPtr<Gio::DBus::Connection>& /*connection*/,
	const Glib::ustring& /*sender*/,
	const Glib::ustring& /*object_path*/,
	const Glib::ustring& interface_name,
	const Glib::ustring& property_name)
{
	get_property(property, interface_name, property_name);
}

bool StubObject::on_interface_set_property(
	const Glib::RefPtr<Gio::DBus::Connection>& /*connection*/,
	const Glib::ustring& /*sender*/,
	const Glib::ustring& /*object_path*/,
	const Glib::ustring& interface_name,
	const Glib::ustring& property_name,
	const Glib::VariantBase& value)
{
	return set_property(interface_name, property_name, value);
}

void StubObject::register_for_path(Gio::DBus::BusType bustype, const Glib::DBusObjectPathString& path)
{
	bus = Gio::DBus::Connection::get_sync(bustype);
	bus->register_object(path, interface_info, dbus_vtable);
}

void StubObject::register_for_path(Glib::RefPtr<Gio::DBus::Connection> busconn, const Glib::DBusObjectPathString& path)
{
	bus = std::move(busconn);
	bus->register_object(path, interface_info, dbus_vtable);
}

ObjectManager::ObjectManager()
	: StubObject(::dbus::interface_info<org::freedesktop::DBus::ObjectManager>())
{
}

void ObjectManager::register_on_bus(Glib::RefPtr<Gio::DBus::Connection> bus)
{
	register_for_path(std::move(bus), "/");
}

void ObjectManager::add_object(const Glib::DBusObjectPathString& path, StubObject& obj)
{
	auto& objinfo = managed_objects[path];
	if (!objinfo.empty())
		throw std::runtime_error("object already exists at provided path");

	auto iface = obj.get_interface_info();
	Glib::ustring ifacename = iface->gobj()->name;
	auto& props = objinfo[ifacename];

	for (GDBusPropertyInfo **ppropinfo = iface->gobj()->properties; *ppropinfo; ++ppropinfo) {
		GDBusPropertyInfo *propinfo = *ppropinfo;

		Glib::ustring propname = propinfo->name;
		obj.get_property(props[propname], ifacename, propname);
	}

	obj.register_for_path(bus, path);

	// TODO: signal InterfacesAdded
}

void ObjectManager::on_interface_method_call(
	const Glib::RefPtr<Gio::DBus::Connection>& connection,
	const Glib::ustring& sender,
	const Glib::ustring& object_path,
	const Glib::ustring& interface_name,
	const Glib::ustring& method_name,
	const Glib::VariantContainerBase& parameters,
	const Glib::RefPtr<Gio::DBus::MethodInvocation>& invocation)
{
	if (!method_matches<Methods::GetManagedObjects>(interface_name, method_name))
		return StubObject::on_interface_method_call(connection, sender, object_path,
			interface_name, method_name, parameters, invocation);

	update_object_map();
	Glib::VariantContainerBase objs_var = Glib::Variant<object_map_t>::create(managed_objects);
	Glib::VariantContainerBase response = Glib::VariantContainerBase::create_tuple(std::move(objs_var));
	invocation->return_value(std::move(response));
}

} /* namespace dbus */
