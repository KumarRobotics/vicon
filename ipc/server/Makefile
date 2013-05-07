XDRGEN ?= `which xdrgen`

BIN := vicon_ipc
SRC := src/vicon_ipc.cpp

XDR_DIR := ../xdr
MSG_DIR := include/msgs

OBJ := $(SRC:.cpp=.o)
XDR := $(notdir $(wildcard $(XDR_DIR)/*.xdr))
MSG := $(XDR:.xdr=.h)

CFLAGS := -Wall -O2 -fPIC
CXXFLAGS := $(CFLAGS)
LDFLAGS := -Wl,-O1 -Wl,--as-needed -Wl,-unresolved-symbols=ignore-in-shared-libs

INCS := -I../../libvicon_driver/include -Iinclude `pkg-config --cflags eigen3 yaml-cpp`
LIBS := -Wl,-rpath,../../libvicon_driver -L../../libvicon_driver -lvicon_driver \
	-lipc `pkg-config --libs eigen3 yaml-cpp`

all: $(MSG) $(BIN)

$(BIN): $(OBJ)
	@echo "  [LD]  $@"
	@$(CXX) $(CXXFLAGS) -o $(BIN) $(OBJ) $(LDFLAGS) $(LIBS)

%.o: %.cpp
	@echo "  [CPP] $@"
	@$(CXX) $(CXXFLAGS) $(INCS) -c $< -o $@

%.o: %.c
	@echo "  [CC]  $@"
	@$(CC) $(CFLAGS) $(INCS) -c $< -o $@

%.h: $(XDR_DIR)/%.xdr
	@echo "  [XDR] $@"
	@mkdir -p $(MSG_DIR)
	@$(XDRGEN) --lang=c++ $< $(MSG_DIR)/$@

clean:
	rm -rf $(BIN) $(OBJ) $(MSG_DIR)
