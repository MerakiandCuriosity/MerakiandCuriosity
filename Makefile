# Makefile for STM32F103C8T6 Laser Pointer Project

# Project name
PROJECT = laser_pointer_triangle

# MCU settings
MCU = cortex-m3
MCUFLAGS = -mcpu=$(MCU) -mthumb

# Directories
SRCDIR = .
HARDWARE_DIR = HARDWARE
MINIBALANCE_DIR = MiniBalance
SYSTEM_DIR = SYSTEM

# Include paths
INCLUDES = -I$(HARDWARE_DIR) -I$(MINIBALANCE_DIR) -I$(SYSTEM_DIR) -ISTM32F10x_StdPeriph_Driver/inc -ICMSIS

# Compiler settings
CC = arm-none-eabi-gcc
CFLAGS = $(MCUFLAGS) -Wall -Wextra -O2 $(INCLUDES) -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

# Source files
SOURCES = $(MINIBALANCE_DIR)/control.c

# Object files
OBJECTS = $(SOURCES:.c=.o)

# Build target
all: $(PROJECT).elf

$(PROJECT).elf: $(OBJECTS)
	@echo "Linking $(PROJECT).elf"
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	@echo "Compiling $<"
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(PROJECT).elf

# Syntax check target
syntax-check:
	@echo "Performing syntax check on control.c"
	$(CC) $(CFLAGS) -fsyntax-only $(MINIBALANCE_DIR)/control.c

.PHONY: all clean syntax-check