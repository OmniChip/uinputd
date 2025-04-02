#include "uin-dev.h"

#include <cstring>
#include <iostream>
#include <system_error>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>


#define UIN_SYSNAME_LEN 16

__attribute__((noinline,noreturn))
static void throw_errno(const char *what)
{
	throw std::system_error(errno, std::system_category(), what);
}

UInputDevice::UInputDevice()
	: fd(::open("/dev/uinput", O_RDWR|O_NONBLOCK|O_NOCTTY)), created(false), has_abs(false), has_msc(false)
{
	if (fd < 0)
		throw_errno("open(/dev/uinput)");

	unsigned int ui_proto_ver;
	if (::ioctl(fd, UI_GET_VERSION, &ui_proto_ver) < 0)
		throw_errno("ioctl(GET_VERSION)");
#if 0
	std::cerr << "uinput version " << ui_proto_ver << '\n';
#endif
}

UInputDevice::~UInputDevice()
{
	if (created)
		::ioctl(fd, UI_DEV_DESTROY);
	::close(fd);
}

std::string UInputDevice::create()
{
	if (::ioctl(fd, UI_DEV_CREATE) < 0)
		throw_errno("ioctl(DEV_CREATE)");
	created = true;

	std::string buf(UIN_SYSNAME_LEN, 0);
	if (::ioctl(fd, UI_GET_SYSNAME(UIN_SYSNAME_LEN), buf.data()) < 0)
		throw_errno("ioctl(GET_SYSNAME)");
	buf.resize(::strlen(buf.data()));

	return buf;
}

void UInputDevice::ioctl_set(unsigned long req, int arg)
{
	if (::ioctl(fd, req, arg) < 0)
		throw_errno("ioctl_set()");
}

void UInputDevice::setup_dev(const std::string& name, uint32_t max_ff_effects, uint16_t bus, uint16_t vendor, uint16_t product, uint16_t version)
{
	struct uinput_setup info = { { bus, vendor, product, version }, { 0, }, max_ff_effects };

	strncpy(info.name, name.c_str(), UINPUT_MAX_NAME_SIZE - 1);
	if (::ioctl(fd, UI_DEV_SETUP, &info) < 0)
		throw_errno("ioctl(DEV_SETUP)");
}

void UInputDevice::set_prop(int bit)
{
	ioctl_set(UI_SET_PROPBIT, bit);
}

void UInputDevice::setup_abs_axis(uint16_t axis, int min, int max, int res)
{
	if (!has_abs) {
		ioctl_set(UI_SET_EVBIT, EV_ABS);
		has_abs = true;
	}

	struct uinput_abs_setup info = { axis, { 0, min, max, 0, 0, res } };

	if (::ioctl(fd, UI_ABS_SETUP, &info) < 0)
		throw_errno("ioctl(ABS_SETUP)");
}

void UInputDevice::setup_msc_entry(uint16_t msc_id)
{
	if (!has_msc) {
		ioctl_set(UI_SET_EVBIT, EV_MSC);
		has_msc = true;
	}

	ioctl_set(UI_SET_MSCBIT, msc_id);
}

void UInputDevice::emit(int type, int code, int val)
{
	struct input_event ie;

	ie.input_event_sec = 0;
	ie.input_event_usec = 0;
	ie.type = type;
	ie.code = code;
	ie.value = val;

	if (::write(fd, &ie, sizeof(ie)) < 0)
		throw_errno("write(event)");
}

