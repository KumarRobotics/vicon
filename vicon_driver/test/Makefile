BIN := vicon_test
SRC := vicon_test.cpp

OBJ := $(SRC:.cpp=.o)

CFLAGS := -Wall -O2 -fPIC
CXXFLAGS := $(CFLAGS)
LDFLAGS := -Wl,-O1 -Wl,--as-needed

INCS := -I../include
LIBS := -Wl,-rpath,.. -L.. -lvicon_driver -lpthread -lrt

all: $(BIN)

$(BIN): $(OBJ)
	@echo "  [LD]  $@"
	@$(CXX) $(CXXFLAGS) -o $(BIN) $(OBJ) $(LDFLAGS) $(LIBS)

%.o: %.cpp
	@echo "  [CPP] $@"
	@$(CXX) $(CXXFLAGS) $(INCS) -c $< -o $@

%.o: %.c
	@echo "  [CC]  $@"
	@$(CC) $(CFLAGS) $(INCS) -c $< -o $@

clean:
	rm -rf $(BIN) $(OBJ)
