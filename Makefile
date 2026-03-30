# Makefile for C51 (SDCC-based) project - Windows environment
# All build artifacts go into ./build/
#
# Build chain:
#   c51 -c   : .c -> .obj  (compile + assemble, no link)
#   l51      : .obj + .lib -> .hex (link)

# Project settings
# Try common Windows locations
CROSSIDE_DIR := $(or \
    $(wildcard C:/CrossIDE), \
    $(wildcard D:/CrossIDE), \
    $(wildcard E:/CrossIDE) \
)

ifeq ($(CROSSIDE_DIR),)
    $(error CrossIDE not found. Set it manually: make CROSSIDE_DIR=C:/CrossIDE)
endif

SRC_DIR   = src
INC_DIR   = inc
BUILD_DIR = build
TARGET    = main

# Tool paths (CrossIDE Call51 toolchain)
CALL51_DIR = $(CROSSIDE_DIR)/Call51
C51        = $(CALL51_DIR)/Bin/c51.exe
L51        = $(CALL51_DIR)/Bin/l51.exe

# Standard include and lib paths from the toolchain
STD_INC = $(CALL51_DIR)/Include
STD_LIB = $(CALL51_DIR)/LiB/Small

# Libraries needed for linking
LIBS = $(STD_LIB)/libint.lib \
       $(STD_LIB)/liblong.lib \
       $(STD_LIB)/libfloat.lib \
       $(STD_LIB)/libc51f.lib

# Compiler flags
CFLAGS = -I$(INC_DIR) -I$(STD_INC)

# Source and object files
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.obj,$(SRCS))

# Ensure build directory exists
$(shell mkdir -p $(BUILD_DIR))

.PHONY: all clean

all: $(BUILD_DIR)/$(TARGET).hex

# Link all .obj files + libraries into final .hex
$(BUILD_DIR)/$(TARGET).hex: $(OBJS)
	@echo --- Linking ---
	$(L51) $^ $(LIBS) -hex $(BUILD_DIR)/$(TARGET).hex -map $(BUILD_DIR)/$(TARGET).map

# Compile each .c to .obj individually
$(BUILD_DIR)/%.obj: $(SRC_DIR)/%.c
	@echo --- Compiling $< ---
	$(C51) -c $(CFLAGS) -o $@ $<

clean:
	rm -rf $(BUILD_DIR)
