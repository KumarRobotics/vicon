LIB := libvicon_driver.so
SRC := src/vicon_driver.cpp src/vicon_calib.cpp

OBJ := $(SRC:.cpp=.o)

CFLAGS := -Wall -Wextra -O2 -fPIC
CXXFLAGS := $(CFLAGS)
LDFLAGS := -Wl,-O1 -Wl,--as-needed

# The eigen3 and yaml-cpp dependencies are only needed for vicon_calib.cpp
INCS := -Iinclude `pkg-config --cflags eigen3 yaml-cpp`
LIBS := -lpthread -lrt `pkg-config --libs eigen3 yaml-cpp`

uname_S := $(shell sh -c 'uname -s 2>/dev/null')
uname_M := $(shell sh -c 'uname -m 2>/dev/null')

ifneq ($(uname_S), Linux)
$(error This driver only works on Linux since the included Vicon SDK is only for Linux)
endif

ifeq ($(uname_M), i686)
VICON_SDK_DIR := $(shell pwd)/vicon_sdk/vicon_datastream_SDK_1.2.59611_x86
endif
ifeq ($(uname_M), x86_64)
VICON_SDK_DIR := $(shell pwd)/vicon_sdk/vicon_datastream_SDK_1.2.59611_x64
endif

INCS += -I$(VICON_SDK_DIR)
LIBS += -Wl,-rpath,$(VICON_SDK_DIR) -L$(VICON_SDK_DIR) -lViconDataStreamSDK_CPP

all: $(LIB)

$(LIB): $(OBJ)
	@echo "  [LIB] $@"
	@$(CXX) $(CXXFLAGS) -shared -o $(LIB) $(OBJ) $(LDFLAGS) $(LIBS)

%.o: %.cpp
	@echo "  [CPP] $@"
	@$(CXX) $(CXXFLAGS) $(INCS) -c $< -o $@

%.o: %.c
	@echo "  [CC]  $@"
	@$(CC) $(CFLAGS) $(INCS) -c $< -o $@

clean:
	rm -rf $(LIB) $(OBJ)
