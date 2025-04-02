#ifndef DBUS_BASE_H_
#define DBUS_BASE_H_

#include "dbus.h"

/* generic interfaces */

namespace org::freedesktop::DBus {

struct ObjectManager
{
	DEF_DBUS_IFACE_NAME("org.freedesktop.DBus.ObjectManager");

	using property_values_map_t = std::map<Glib::ustring, Glib::VariantBase>;
	using interface_properties_map_t = std::map<Glib::ustring, property_values_map_t>;
	using object_interfaces_map_t = std::map<Glib::DBusObjectPathString, interface_properties_map_t>;

	struct Methods {
		DEF_DBUS_CALL(ObjectManager, GetManagedObjects, object_interfaces_map_t);
	};

	struct Signals {
		DEF_DBUS_SIGNAL(ObjectManager, InterfacesAdded, Glib::DBusObjectPathString, interface_properties_map_t);
		DEF_DBUS_SIGNAL(ObjectManager, InterfacesRemoved, Glib::DBusObjectPathString, std::vector<Glib::ustring>);
	};
};

struct Properties
{
	DEF_DBUS_IFACE_NAME("org.freedesktop.DBus.Properties");

	using property_values_map_t = ObjectManager::property_values_map_t;

	struct Methods {
	};

	struct Signals {
		DEF_DBUS_SIGNAL(Properties, PropertiesChanged, Glib::ustring, property_values_map_t, std::vector<Glib::ustring>);
	};
};

} /* namespace org::freedesktop::DBus */

namespace dbus {

struct ObjectManagerProxy : public WrappedObject
{
	using interface_properties_map_t = org::freedesktop::DBus::ObjectManager::interface_properties_map_t;

	sigc::slot<void, const Glib::DBusObjectPathString&, const interface_properties_map_t&> on_interfaces_added;
	sigc::slot<void, const Glib::DBusObjectPathString&, const std::vector<Glib::ustring>&> on_interfaces_removed;

	ObjectManagerProxy(Gio::DBus::BusType bus, const Glib::ustring& busname, const Glib::DBusObjectPathString& path = "/");

	FORWARD_CALL(org::freedesktop::DBus::ObjectManager, GetManagedObjects)
private:
	void on_mgr_signal(const Glib::ustring& /*sender_name*/, const Glib::ustring& signal_name, const Glib::VariantContainerBase& parameters);
};

struct ObjectManager : public StubObject, private org::freedesktop::DBus::ObjectManager
{
	using object_map_t = object_interfaces_map_t;

	ObjectManager();

	void register_on_bus(Glib::RefPtr<Gio::DBus::Connection> bus);
	void add_object(const Glib::DBusObjectPathString& path, StubObject& obj);
protected:
	object_map_t managed_objects;

	virtual void on_interface_method_call(
		const Glib::RefPtr<Gio::DBus::Connection>& /*connection*/,
		const Glib::ustring& /*sender*/,
		const Glib::ustring& /*object_path*/,
		const Glib::ustring& /*interface_name*/,
		const Glib::ustring& /*method_name*/,
		const Glib::VariantContainerBase& /*parameters*/,
		const Glib::RefPtr<Gio::DBus::MethodInvocation>& /*invocation*/) override;

	virtual void update_object_map() {}
};

} /* namespace dbus */

#endif /* DBUS_BASE_H_ */
