#ifndef GAMEINN__WRAP_UIN_H_
#define GAMEINN__WRAP_UIN_H_

#include <linux/uinput.h>

#include <string>

struct UInputDevice
{
	UInputDevice();
	~UInputDevice();

	bool is_created() const { return created; }

	void setup_dev(const std::string& name, uint32_t max_ff_effects, uint16_t bus, uint16_t vendor, uint16_t product, uint16_t version);
	void set_prop(int bit);
	void setup_abs_axis(uint16_t axis, int min, int max, int res);
	void setup_msc_entry(uint16_t msc_id);
	std::string create();

	void emit(int type, int code, int val);
private:
	int fd;
	unsigned created:1;
	unsigned has_abs:1;
	unsigned has_msc:1;

	void ioctl_set(unsigned long req, int arg);
};

#endif /* GAMEINN__WRAP_UIN_H_ */
