# 로봇 클라이언트 빌드. 리포 루트에서 `make -C robot`. 산출물은 ../build/{bin,obj}. cmake 없이 순수 Makefile.
CXX := clang++
BUILD := ../build
BIN := $(BUILD)/bin
OBJ := $(BUILD)/obj
BREW := $(shell brew --prefix 2>/dev/null || echo /opt/homebrew)
UNAME_S := $(shell uname -s)

# Qt6 Widgets -- GUI(gui.cpp, robot_client 전용)에만 필요. macOS 는 프레임워크,
# Linux(Ubuntu, CLAUDE.md 최종 타깃)는 pkg-config(qt6-base-dev) 로 잡는다.
ifeq ($(UNAME_S),Darwin)
QT_PREFIX := $(BREW)/opt/qt
QT_CXXFLAGS := -F$(QT_PREFIX)/lib
QT_LDFLAGS := -F$(QT_PREFIX)/lib -framework QtWidgets -framework QtGui -framework QtCore
else
QT_CXXFLAGS := $(shell pkg-config --cflags Qt6Widgets 2>/dev/null)
QT_LDFLAGS := $(shell pkg-config --libs Qt6Widgets 2>/dev/null)
endif

# MuJoCo (물리 엔진). brew 패키지가 없어서 robot/tools/setup_mujoco.sh 가 third_party/mujoco/ 에 설치한다.
# 다른 곳에 있으면 make MUJOCO_DIR=/경로 (include/mujoco/mujoco.h 와 lib/libmujoco* 가 있어야 한다)
MUJOCO_DIR ?= third_party/mujoco
MUJOCO_HDR := $(MUJOCO_DIR)/include/mujoco/mujoco.h
MUJOCO_LDFLAGS := -L $(MUJOCO_DIR)/lib -lmujoco -Wl,-rpath,$(abspath $(MUJOCO_DIR))/lib

# -MMD -MP: 헤더 의존성 자동 생성. 이게 없으면 schemas.hpp 를 고쳐도 그걸
# include 하는 .cpp 가 재컴파일되지 않아 struct 레이아웃이 어긋난 .o 들이
# 링크되고, 런타임에 조용히 세그폴트난다 (스키마 동기화 작업에서 실제로 밟음).
CXXFLAGS := -std=c++20 -O2 -Wall -Wextra -Wno-unused-parameter -MMD -MP \
            -I include -I $(MUJOCO_DIR)/include -I $(BREW)/opt/mosquitto/include -I $(BREW)/opt/cjson/include $(QT_CXXFLAGS)
LDFLAGS := -L $(BREW)/opt/mosquitto/lib -L $(BREW)/opt/cjson/lib -lmosquitto -lcjson -lpthread

CORE_SRC := src/schemas.cpp src/map_model.cpp src/kinematics.cpp src/localizer.cpp \
            src/dynamic_obstacles.cpp src/planner.cpp src/executor.cpp src/comm.cpp \
            src/map_sync.cpp src/path_refine.cpp src/physics_world.cpp src/recorder.cpp \
            src/scan_matcher.cpp src/chargers.cpp src/local_planner.cpp
CORE_OBJ := $(addprefix $(OBJ)/,$(CORE_SRC:.cpp=.o))
GUI_OBJ := $(OBJ)/src/gui.o
MAIN_OBJ := $(OBJ)/src/main.o
SCHEMAS_OBJ := $(OBJ)/src/schemas.o
TOOL_BINS := $(BIN)/send_order $(BIN)/place_obstacle $(BIN)/obstacle_zone

.SECONDARY:
.PHONY: all clean tools

all: $(BIN)/robot_client tools

$(BIN)/robot_client: $(MAIN_OBJ) $(CORE_OBJ) $(GUI_OBJ)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(MUJOCO_LDFLAGS) $(QT_LDFLAGS)

# MuJoCo 가 없으면 physics_world.cpp 컴파일 전에 안내하고 멈춘다
$(MUJOCO_HDR):
	@echo "MuJoCo 가 없다 ($(MUJOCO_HDR)). 먼저 robot/tools/setup_mujoco.sh 를 실행하라." >&2; exit 1

$(OBJ)/src/physics_world.o: | $(MUJOCO_HDR)

tools: $(TOOL_BINS)

$(BIN)/%: $(OBJ)/tools/%.o $(SCHEMAS_OBJ)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJ)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

DEPS := $(CORE_OBJ:.o=.d) $(MAIN_OBJ:.o=.d) $(GUI_OBJ:.o=.d) \
        $(addprefix $(OBJ)/tools/,send_order.d place_obstacle.d obstacle_zone.d)
-include $(DEPS)

clean:
	rm -rf $(BUILD)
