CC = cc
LD = cc

DIR = build

SRCS = bullet.c budy.c blw.c

BULLET_OPTS = -DBUILD_CPU_DEMOS=OFF
BULLET_OPTS := $(BULLET_OPTS) -DBUILD_BULLET2_DEMOS=OFF
BULLET_OPTS := $(BULLET_OPTS) -DBUILD_CPU_DEMOS=OFF
BULLET_OPTS := $(BULLET_OPTS) -DBUILD_EGL=OFF
# BULLET_OPTS := $(BULLET_OPTS) -DBUILD_EXTRAS=OFF
BULLET_OPTS := $(BULLET_OPTS) -DBUILD_OPENGL3_DEMOS=OFF
BULLET_OPTS := $(BULLET_OPTS) -DBUILD_PYBULLET=OFF
BULLET_OPTS := $(BULLET_OPTS) -DBUILD_UNIT_TESTS=OFF
BULLET_OPTS := $(BULLET_OPTS) -DUSE_GLUT=OFF
BULLET_OPTS := $(BULLET_OPTS) -DUSE_GRAPHICAL_BENCHMARK=OFF
BULLET_OPTS := $(BULLET_OPTS) -DBUILD_SHARED_LIBS=on
BULLET_OPTS := $(BULLET_OPTS) -DCMAKE_BUILD_TYPE=Release
BULLET_LIBS = $(DIR)/bullet3_release/Extras/BulletRobotics/libBulletRobotics.so

OBJS_REL = $(patsubst %.c, $(DIR)/%.o, $(SRCS))
OBJS_DEB = $(patsubst %.c, $(DIR)/%.debug.o, $(SRCS))

CFLAGS = $(PARENTCFLAGS) -Ibullet3/examples/SharedMemory/ -Wall -Wno-unused-function

CFLAGS_REL = $(CFLAGS) -O3

CFLAGS_DEB = $(CFLAGS) -g3

##############################################################################

all: $(DIR)/libs $(BULLET_LIBS) 
	echo $(BULLET_LIBS) > $(DIR)/res

$(DIR)/libs: $(DIR)/export.a
	echo bullet.candle/$(DIR)/export.a > $@

$(DIR)/export.a: init $(OBJS_REL)
	$(AR) rs $(DIR)/export.a $(OBJS_REL)

$(DIR)/%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS_REL)

$(BULLET_LIBS):
	cmake -B $(DIR)/bullet3_release bullet3 $(BULLET_OPTS)  -G "Unix Makefiles"
	cmake --build $(DIR)/bullet3_release

##############################################################################

debug: $(DIR)/libs_debug $(BULLET_LIBS) 
	echo $(BULLET_LIBS) > $(DIR)/res

$(DIR)/libs_debug: $(DIR)/export_debug.a
	echo bullet.candle/$(DIR)/export_debug.a > $@

$(DIR)/export_debug.a: init $(BULLET_LIBS) $(OBJS_DEB)
	$(AR) rs $(DIR)/export_debug.a $(OBJS_DEB)

$(DIR)/%.debug.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS_DEB)

##############################################################################

init:
	mkdir -p $(DIR)

##############################################################################

clean:
	rm -r $(DIR)

.PHONY: clean all init

# vim:ft=make
