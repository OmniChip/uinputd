#!make -f

LIB_SOURCES := band.cc ble.cc dbus.cc calibration.cc gvector.cc detect.cc
UI_SOURCES := uinputd.cc uin-dev.cc
DL_SOURCES := datalog.cc
TESTS := squats wheel punch calib

PKGS := glibmm-2.4 giomm-2.4

CFLAGS := -Og -ggdb -std=gnu++17

CFLAGS += $(shell pkg-config --cflags $(PKGS))
LDFLAGS += -Wl,--as-needed $(shell pkg-config --libs $(PKGS)) -lpthread


LIB_OBJS := $(patsubst %.cc,%.s_o,$(LIB_SOURCES))
UI_OBJS := $(patsubst %.cc,%.o,$(UI_SOURCES) $(LIB_SOURCES))
DL_OBJS := $(patsubst %.cc,%.o,$(DL_SOURCES) $(LIB_SOURCES))
TEST_BINS := $(patsubst %,test_%,$(TESTS))

%.o: %.cc
	g++ $(CFLAGS) $(CXXFLAGS) -o $@ -MMD -MF $(patsubst %.o,.%.d,$@) -c $<

%.s_o: %.cc
	g++ $(CFLAGS) $(CXXFLAGS) -o $@ -MMD -MF $(patsubst %.s_o,.%.s_d,$@) -fPIC -fvisibility=hidden -c $<


.PHONY: all
all: band-uinputd datalog test_squats $(TEST_BINS) libgameinn_bands.so

band-uinputd: $(UI_OBJS) libgameinn_bands.so
	g++ $(CFLAGS) $(CXXFLAGS) -o $@ $(UI_OBJS) $(LDFLAGS)

datalog: $(DL_OBJS) libgameinn_bands.so
	g++ $(CFLAGS) $(CXXFLAGS) -o $@ $(DL_OBJS) $(LDFLAGS)

test_%: test_%.o libgameinn_bands.so
	g++ $(CFLAGS) $(CXXFLAGS) -o $@ $< $(LDFLAGS) -L. -lgameinn_bands -Wl,"-rpath=$$(dirname "$$(realpath -e libgameinn_bands.so)")"

libgameinn_bands.so: $(LIB_OBJS)
	g++ -shared $(CFLAGS) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: clean
clean:
	rm -f *.o *.s_o $(TEST_BINS) band-uinputd datalog libgameinn_bands.so

-include $(wildcard .*.d)
-include $(wildcard .*.s_d)
