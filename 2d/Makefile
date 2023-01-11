SKYNET_ROOT ?= ../../skynet
include $(SKYNET_ROOT)/platform.mk

PLAT ?= none

TARGET = ./bump2d.so

CXX=g++

ifeq ($(PLAT), macosx)
	CXXFLAGS = -g -O2 -dynamiclib -Wl,-undefined,dynamic_lookup
else
ifeq ($(PLAT), linux)
	CXXFLAGS = -g -O2 -shared -fPIC
endif
endif

LUA_INC ?= $(SKYNET_ROOT)/3rd/lua/

SRC = .

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(foreach dir, $(SRC), $(wildcard $(dir)/*.cpp))
	$(CXX) $(CXXFLAGS) -o $@ $^ -I$(LUA_INC)


clean:
	rm -f *.o $(TARGET) && \
	rm -rf *.dSYM