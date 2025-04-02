#ifndef GAMEINN_DBUS_H_
#define GAMEINN_DBUS_H_

#include <giomm.h>

#include <atomic>
#include <map>
#include <vector>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>

namespace dbus {

struct WrappedObject : sigc::trackable
{
	std::atomic<unsigned> prop_changes;

	WrappedObject() : prop_changes(0) {}
	WrappedObject(const WrappedObject& other) = delete;
	WrappedObject(WrappedObject&& other) : prop_changes(other.prop_changes.load(std::memory_order_relaxed)), obj(std::move(other.obj)) {}
	WrappedObject(Gio::DBus::BusType bus, const Glib::ustring& busname, const Glib::DBusObjectPathString& path, const Glib::ustring& iface);
	explicit WrappedObject(const Glib::RefPtr<Gio::DBus::Proxy>& obj_);
	~WrappedObject();

	Glib::DBusObjectPathString path() const { return Glib::DBusObjectPathString{obj->get_object_path()}; }
	bool is_valid() const { return !!obj; }
	const Glib::RefPtr<Gio::DBus::Proxy>& get_obj() const { return obj; }
	void set_obj(const Glib::RefPtr<Gio::DBus::Proxy>& obj_);
private:
	void set_signal();
protected:
	Glib::RefPtr<Gio::DBus::Proxy> obj;
	sigc::connection propchg_conn;

	template <typename Prop>
	auto get_property() const -> typename Prop::value_type
	{
		Glib::VariantBase val;
		obj->get_cached_property(val, Prop::name);
		if (!val)
			throw std::invalid_argument(Prop::name);
		return Glib::VariantBase::cast_dynamic<Glib::Variant<typename Prop::value_type>>(val).get();
	}

	virtual void on_properties_changed(const Gio::DBus::Proxy::MapChangedProperties&, const std::vector<Glib::ustring>&);

	static Glib::VariantContainerBase wrap_with_variant(const std::tuple<>&) { return {}; }
	template <typename... Args>
	static auto wrap_with_variant(std::tuple<Args...>&& args) { return Glib::Variant<std::tuple<Args...>>::create(std::move(args)); }

	template <typename Method, typename = std::enable_if_t<!std::is_void_v<typename Method::return_type>>>
	auto call_method(typename Method::argument_type args) const -> typename Method::return_type
	{
		auto res = obj->call_sync(Method::name, wrap_with_variant(args));
		return Glib::VariantBase::cast_dynamic<Glib::Variant<typename Method::return_type>>(res).get();
	}

	template <typename Method, typename = std::enable_if_t<std::is_void_v<typename Method::return_type>>>
	void call_method(typename Method::argument_type&& args) const
	{
		obj->call_sync(Method::name, wrap_with_variant(std::move(args)));
	}

	template <typename Method>
	void call_method_async(const Gio::SlotAsyncReady& slot, typename Method::argument_type&& args) const
	{
		obj->call(Method::name, slot, wrap_with_variant(std::move(args)));
	}

	template <typename Method>
	void call_method_async(const Glib::RefPtr<Gio::Cancellable>& cancel, const Gio::SlotAsyncReady& slot, typename Method::argument_type&& args) const
	{
		obj->call(Method::name, slot, cancel, wrap_with_variant(std::move(args)));
	}

	template <typename Method, typename = std::enable_if_t<std::is_void_v<typename Method::return_type>>>
	void call_method_noreply(typename Method::argument_type&& args) const
	{
		obj->call(Method::name, wrap_with_variant(std::move(args)));
	}
public:
	Glib::VariantContainerBase finish_call(const Glib::RefPtr<Gio::AsyncResult>& res) const
	{
		return obj->call_finish(res);
	}
};

struct StubObject
{
	friend class ObjectManager;

	explicit StubObject(Glib::RefPtr<Gio::DBus::InterfaceInfo> interface_info_);
	virtual ~StubObject();

	Glib::RefPtr<Gio::DBus::InterfaceInfo> get_interface_info() const { return interface_info; }

	virtual void call_method(Glib::VariantBase& ret, const Glib::ustring& iface, const Glib::ustring& methodname, const Glib::VariantBase& args);
	virtual void get_property(Glib::VariantBase& var, const Glib::ustring& iface, const Glib::ustring& propname) const;
	virtual bool set_property(const Glib::ustring& iface, const Glib::ustring& propname, const Glib::VariantBase& var);

	template <typename Method>
	static bool method_matches(const Glib::ustring& interface_name, const Glib::ustring& method_name)
	{
		return interface_name == Method::interface_type::name && method_name == Method::name;
	}
protected:
	void register_for_path(Gio::DBus::BusType bus, const Glib::DBusObjectPathString& path);
	void register_for_path(Glib::RefPtr<Gio::DBus::Connection> busconn, const Glib::DBusObjectPathString& path);

	virtual void on_interface_method_call(
		const Glib::RefPtr<Gio::DBus::Connection>& /*connection*/,
		const Glib::ustring& /*sender*/,
		const Glib::ustring& /*object_path*/,
		const Glib::ustring& /*interface_name*/,
		const Glib::ustring& /*method_name*/,
		const Glib::VariantContainerBase& /*parameters*/,
		const Glib::RefPtr<Gio::DBus::MethodInvocation>& /*invocation*/);

	virtual void on_interface_get_property(
		Glib::VariantBase& /*property*/,
		const Glib::RefPtr<Gio::DBus::Connection>& /*connection*/,
		const Glib::ustring& /*sender*/,
		const Glib::ustring& /*object_path*/,
		const Glib::ustring& /*interface_name*/,
		const Glib::ustring& /*property_name*/);

	virtual bool on_interface_set_property(
		const Glib::RefPtr<Gio::DBus::Connection>& /*connection*/,
		const Glib::ustring& /*sender*/,
		const Glib::ustring& /*object_path*/,
		const Glib::ustring& /*interface_name*/,
		const Glib::ustring& /*property_name*/,
		const Glib::VariantBase& /*value*/);
private:
	const Gio::DBus::InterfaceVTable dbus_vtable;
	const Glib::RefPtr<Gio::DBus::InterfaceInfo> interface_info;
protected:
	Glib::RefPtr<Gio::DBus::Connection> bus;
};

/* access wrapper generators */

#define FORWARD_PROP_READ(t, n) \
	auto n() const { return get_property<t::Properties::n>(); }

#define FORWARD_CALL(t, n) \
	template <typename... Args> \
	auto n(Args&&... args) const \
	{ return call_method<t::Methods::n>(std::forward_as_tuple<Args...>(std::forward<Args>(args)...)); } \
	\
	template <typename... Args> \
	void n##_async(const Gio::SlotAsyncReady& slot, Args&&... args) const \
	{ call_method_async<t::Methods::n>(slot, std::forward_as_tuple<Args...>(std::forward<Args>(args)...)); } \
	\
	template <typename... Args> \
	void n##_casync(const Glib::RefPtr<Gio::Cancellable>& cancel, const Gio::SlotAsyncReady& slot, Args&&... args) const \
	{ call_method_async<t::Methods::n>(cancel, slot, std::forward_as_tuple<Args...>(std::forward<Args>(args)...)); } \
	\
	template <typename... Args> \
	void n##_noreply(Args&&... args) const { call_method_noreply<t::Methods::n>(std::forward_as_tuple<Args...>(std::forward<Args>(args)...)); }

/* interface description helpers */

#define DEF_DBUS_PROPERTY(i,n,t,r,w) \
	struct n \
	{ \
		using interface_type = i; \
		using value_type = t; \
		\
		inline static const std::string name = #n; \
		static const bool readable = r; \
		static const bool writable = w; \
	}

#define DEF_DBUS_RO_PROPERTY(i,n,t) DEF_DBUS_PROPERTY(i,n,t,true,false)
#define DEF_DBUS_RW_PROPERTY(i,n,t) DEF_DBUS_PROPERTY(i,n,t,true,true)

#define DEF_DBUS_CALL(i,n,r,t...) \
	struct n \
	{ \
		using interface_type = i; \
		using argument_type = std::tuple<t>; \
		using return_type = r; \
		\
		inline static const std::string name = #n; \
	}

#define DEF_DBUS_SIGNAL(i,n,t...) DEF_DBUS_CALL(i,n,void,t)

#define DEF_DBUS_IFACE_NAME(n) \
inline static const std::string name = n


#if 0
template <typename SD>
struct SignalKeeper
{
	using ArgumentType = typename SD::ArgumentType;
	using Handler = typename core::dbus::Signal<SD, ArgumentType>::Handler;

	SignalKeeper() = default;
	SignalKeeper(const SignalKeeper&) = delete;
	SignalKeeper(SignalKeeper&& o) noexcept : sig(std::move(o.sig)), tok(std::move(o.tok)) {}
	~SignalKeeper() { if (sig) sig->disconnect(tok); }

	SignalKeeper& operator=(SignalKeeper&& o) noexcept
	{
		std::swap(sig, o.sig);
		std::swap(tok, o.tok);
		return *this;
	}

	void activate(const core::dbus::Object::Ptr& obj, const Handler& f)
	{
		auto s = obj->get_signal<SD>();
		tok = s->connect(f);
		sig = std::move(s);
	}

	void deactivate()
	{
		auto s = std::move(sig);
		sig->disconnect(tok);
	}
private:
	typename core::dbus::Signal<SD, ArgumentType>::Ptr sig;
	typename core::dbus::Signal<SD, ArgumentType>::SubscriptionToken tok;
};

struct UniqueService : public core::dbus::Service
{
protected:
	struct this_is_private {};
public:
	UniqueService(const core::dbus::Bus::Ptr& bus, this_is_private) : UniqueService(bus) { stub = false; }

	static Ptr make(const core::dbus::Bus::Ptr& bus) { return std::make_shared<UniqueService>(bus, this_is_private()); }
protected:
	UniqueService(const core::dbus::Bus::Ptr&);
};

template <typename Derived, typename Iface>
struct ObjectSkeleton
{
	ObjectSkeleton(const core::dbus::Service::Ptr& service_, const core::dbus::types::ObjectPath& path)
		: service(service_), obj(service->root_object()->add_object_for_path(path))
	{
	}
private:
	core::dbus::Service::Ptr service;
	core::dbus::Object::Ptr obj;

	template <typename, typename = void>
	struct has_args : std::false_type {};
	template <typename T>
	struct has_args<T, std::void_t<typename T::ArgumentType>> : std::true_type {};

	template <typename Method>
	using returns_void = std::is_same<void, typename Method::ResultType>;

protected:
	template <typename Method, typename = std::enable_if_t<!returns_void<Method>::value>>
	void install_method_handler(typename Method::ResultType (Derived::*handler)(typename Method::ArgumentType))
	{
		obj->install_method_handler<Method>([this, handler] (const core::dbus::Message::Ptr& msg) {
			std::cerr << Method::name() << "  -- na\n";
			typename Method::ArgumentType in;
			msg->reader() >> in;
			auto out = (static_cast<Derived *>(this)->*handler)(std::forward<Method::ArgumentType>(in));
			auto reply = core::dbus::Message::make_method_return(msg);
			reply->writer() << out;
			this->service->get_connection()->send(reply);
		});
	}

	template <typename Method, typename = std::enable_if_t<returns_void<Method>::value>>
	void install_method_handler(void (Derived::*handler)(typename Method::ArgumentType))
	{
		obj->install_method_handler<Method>([this, handler] (const core::dbus::Message::Ptr& msg) {
			std::cerr << Method::name() << "  -- va\n";
			typename Method::ArgumentType in;
			msg->reader() >> in;
			(static_cast<Derived *>(this)->*handler)(std::forward<Method::ArgumentType>(in));
			auto reply = core::dbus::Message::make_method_return(msg);
			this->service->get_connection()->send(reply);
		});
	}

	template <typename Method, typename = std::enable_if_t<!returns_void<Method>::value && !has_args<Method>::value>>
	void install_method_handler(typename Method::ResultType (Derived::*handler)())
	{
		obj->install_method_handler<Method>([this, handler] (const core::dbus::Message::Ptr& msg) {
			std::cerr << Method::name() << "  -- nv\n";
			auto out = (static_cast<Derived *>(this)->*handler)();
			auto reply = core::dbus::Message::make_method_return(msg);
			reply->writer() << out;
			this->service->get_connection()->send(reply);
		});
	}
	template <typename Method, typename = std::enable_if_t<returns_void<Method>::value && !has_args<Method>::value>>
	void install_method_handler(void (Derived::*handler)())
	{
		obj->install_method_handler<Method>([this, handler] (const core::dbus::Message::Ptr& msg) {
			std::cerr << Method::name() << "  -- vv\n";
			(static_cast<Derived *>(this)->*handler)();
			auto reply = core::dbus::Message::make_method_return(msg);
			this->service->get_connection()->send(reply);
		});
	}
};
#endif

} /* namespace dbus */

#endif /* GAMEINN_DBUS_H_ */
