UNAME_S = $(shell uname -s)
WORK_DIR  = $(shell pwd)
BUILD_DIR = $(WORK_DIR)/build
MAKE = make

GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
END='\033[0m'

SERIAL_DIR= $(shell find ./3rdparty -name "src-serial")
THIRD_PARTY_LIB_DIR= $(shell find ./3rdparty -name "lib")

CC = g++
CPPFLAGS = -std=c++20 -O0 -g
#CPPFLAGS += -Wall
CPPFLAGS += -I$(WORK_DIR)/include
CPPFLAGS += -I$(WORK_DIR)/include/gimbal
CPPFLAGS += -I$(WORK_DIR)/include/configs
CPPFLAGS += -I$(WORK_DIR)/include/chassis
CPPFLAGS += -I$(WORK_DIR)/include/device
CPPFLAGS += -I$(WORK_DIR)/include/utils
CPPFLAGS += -I$(WORK_DIR)/include/device/referee

# NOTE: turn on debug here
CPPFLAGS += -D__DEBUG__

# FIXME: imtui dependency linking
CPPFLAGS += -I "./3rdparty/include"
CPPFLAGS += -L "./3rdparty/lib"

LDFLAGS += -lm -lpthread -ldl -lrt -lserial

#LDFLAGS = `pkg-config sdl --libs`

SRC = $(wildcard src/*.cc) $(wildcard src/**/*.cc) $(wildcard src/**/**/*.cc)
INCLUDES = $(wildcard include/*.hpp) $(wildcard include/**/*.hpp) $(wildcard include/**/**/*.hpp)
OBJ = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(SRC))))
BIN = rx78-2

.PHONY: all clean sentry hero infantry

all: dirs $(BIN)

sentry: CPPFLAGS += -DCONFIG_SENTRY=1
sentry: dirs $(BIN)

hero: CPPFLAGS += -DCONFIG_HERO=1
hero: dirs $(BIN)

infantry: CPPFLAGS += -DCONFIG_INFANTRY=1
infantry: dirs $(BIN)

dirs:
	@echo -e + $(BLUE)MKDIR$(END) $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)

run: all
	$(BUILD_DIR)/$(BIN)

clean-build: clean
	@make all -j8

serial: $(SERIAL_DIR)
	@$(MAKE) -C $< -j8
	@echo -e + $(BLUE)MV$(END) $(SERIAL_DIR)/build/libserial.a $(THIRD_PARTY_LIB_DIR)
	@mv $(SERIAL_DIR)/build/libserial.a $(THIRD_PARTY_LIB_DIR)

$(BIN): $(OBJ) serial
	@echo -e + $(GREEN)LN$(END) $(BUILD_DIR)/$(BIN)
	@$(CC) -o $(BUILD_DIR)/$(BIN) $(OBJ) $(CPPFLAGS) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cc $(INCLUDES)
	@mkdir -p $(dir $@) 
	@echo -e + $(GREEN)CC$(END) $<
	@$(CC) -o $@ -c $< $(CPPFLAGS)

clean-serial: $(SERIAL_DIR)
	$(MAKE) -C $< clean

clean: clean-serial
	@echo -e + $(BLUE)RM$(END) 3rdparty/lib/libserial.a
	@rm 3rdparty/lib/libserial.a
	@echo -e + $(BLUE)RM$(END) $(BUILD_DIR)/$(BIN) OBJs
	@rm -rf $(BUILD_DIR)/$(BIN) $(OBJ)
