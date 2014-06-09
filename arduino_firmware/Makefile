#_______________________________________________________________________________
#
#                         edam's Arduino makefile
#_______________________________________________________________________________
#                                                                    version 0.3
#
# Copyright (C) 2011, 1012 Tim Marston <tim@ed.am>.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#_______________________________________________________________________________
#
# Modified 17 Sept, 2012 by Philip Roan, Robert Bosch LLC.
#
# Modified to use this makefile to compile Arduino firmware that is compatible
# with drivers from the Bosch ROS repository: http://ros.org/wiki/bosch-ros-pkg
#
# Changes from edam's Arduino makefile version 0.3 are subject to the following
# software license:
#
#*********************************************************************
#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Robert Bosch LLC.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Robert Bosch nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# 
#*********************************************************************
#
#_______________________________________________________________________________
#
# This is a general purpose makefile for use with Arduino hardware and
# software.  It works with the arduino-1.0 software release.  To download the
# latest version of this makefile, visit the following website, where you can
# also find more information and documentation on it's use.  The following text
# can only really be considered a reference to it's use.
#
#   http://ed.am/dev/make/arduino-mk
#
# The Arduino software (version 1.0 or later) is required.  If you are using
# Debian (or a derivative), type `apt-get install arduino`.  Otherwise, you
# will have to download the Arduino software manually from http://arduino.cc/.
# It is suggested that you install it at ~/opt/arduino if you are unsure.
#
# If you downloaded the Arduino software manually and unpacked it somewhere
# other than ~/opt/arduino, you will need to set up ARDUINO_DIR to be the path
# where you unpacked it.  (If unset, ARDUINO_DIR defaults to ~/opt/arduino and
# then /usr/share/arduino, in that order.)  You might be best to set this in
# your ~/.profile by adding something like this:
#
#   export ARDUINO_DIR=~/somewhere/arduino-1.0
#
# You will also need to set BOARD to the type of Arduino you're building for.
# Type `make boards` for a list of acceptable values.  You could set a default
# in your ~/.profile if you want, but it is suggested that you specify this at
# build time, especially if you work with different types of Arduino.  For
# example:
#
#   $ export BOARD=uno
#   $ make
#
# You may also need to set SERIALDEV if it is not detected correctly.
#
# The presence of a .ino (or .pde) file causes the Makefile to automatically
# determine values for SOURCES, TARGET and LIBRARIES.  Any .c, .cc and .cpp
# files in the project directory (or any "util" or "utility" subdirectories)
# are automatically included in the build and are scanned for Arduino libraries
# that have been #included. Note, there can only be one .ino (or .pde) file.
#
# Alternatively, if you want to manually specify build variables, create a
# Makefile that defines SOURCES and LIBRARIES and then includes this Makefile.
# (There is no need to define TARGET).  Here is an example Makefile:
#
#   SOURCES := main.cc other.cc
#   LIBRARIES := EEPROM
#   include ~/src/arduino.mk
#
# Here is a complete list of configuration parameters:
#
# ARDUINO_DIR  The path where the Arduino software is installed on your system.
#
# ARDUINO_VER  The Arduino software version, as an integer, used to define the
#              ARDUINO version constant. This defaults to 100 if undefined.
#
# AVRDUDECONF  The avrdude.conf to use. If undefined, this defaults to a guess
#              based on where the avrdude in use is. If empty, no avrdude.conf
#              is passed to avrdude (to the system default is used).
#
# AVRTOOLSPATH A space-separated list of directories to search in order when
#              looking for the avr build tools. This defaults to the system PATH
#              followed by subdirectories in ARDUINO_DIR if undefined.
#
# BOARD        Specify a target board type.  Run `make boards` to see available
#              board types.
#
# LIBRARIES    A list of Arduino libraries to build and include.  This is set
#              automatically if a .ino (or .pde) is found.
#
# SERIALDEV    The unix device name of the serial device that is the Arduino.
#              If unspecified, an attempt is made to determine the name of a
#              connected Arduino's serial device.
#
# SOURCES      A list of all source files of whatever language.  The language
#              type is determined by the file extension.  This is set
#              automatically if a .ino (or .pde) is found.
#
# TARGET       The name of the target file.  This is set automatically if a
#              .ino (or .pde) is found, but it is not necessary to set it
#              otherwise.
#
# This makefile also defines the following goals for use on the command line
# when you run make:
#
# all          This is the default if no goal is specified.  It builds the
#              target.
#
# target       Builds the target.
#
# upload       Uploads the last built target to an attached Arduino.
#
# clean        Deletes files created during the build.
#
# boards       Display a list of available board names, so that you can set the
#              BOARD environment variable appropriately.
#
# monitor      Start `screen` on the serial device.  This is meant to be an
#              equivalent to the Arduino serial monitor.
#
# size         Displays size information about the bulit target.
#
# <file>       Builds the specified file, either an object file or the target,
#              from those that that would be built for the project.
#_______________________________________________________________________________
#


#_______________________________________________________________________________
#                                                            ARDUINO INFORMATION

# Check for Arduino software, look in default install directories
ifndef ARDUINO_DIR
ARDUINO_DIR := $(firstword $(wildcard ~/opt/arduino /usr/share/arduino))
endif
ifeq "$(wildcard $(ARDUINO_DIR)/hardware/arduino/boards.txt)" ""
$(error ARDUINO_DIR is not set correctly; arduino software not found)
endif

# Arduino IDE version
ARDUINO_VER ?= 101


#_______________________________________________________________________________
#                                                                 SERIAL DEVICES

# No serial device? make a poor attempt to detect an arduino
SERIALDEVGUESS := 0
ifeq "$(SERIALDEV)" ""
SERIALDEV := $(firstword $(wildcard \
	/dev/ttyACM? /dev/ttyUSB? /dev/tty.usbserial* /dev/tty.usbmodem*))
SERIALDEVGUESS := 1
endif

# figure out which arg to use with stty
STTYFARG := $(shell stty --help > /dev/null 2>&1 && echo -F || echo -f)


#_______________________________________________________________________________
#                                                       SOURCE and LIBRARY FILES

# Use .ino or .pde file as the source
INOFILE := $(wildcard *.ino *.pde)
ifdef INOFILE
ifneq "$(words $(INOFILE))" "1"
$(error There is more than one .pde or .ino file in this directory!)
endif

# Automatically determine sources and targeet
TARGET := $(basename $(INOFILE))
SOURCES := $(INOFILE) \
	$(wildcard *.c *.cc *.cpp) \
	$(wildcard $(addprefix util/, *.c *.cc *.cpp)) \
	$(wildcard $(addprefix utility/, *.c *.cc *.cpp))

# Automatically determine Arduino libraries included by source files
LIBRARIES := $(filter $(notdir $(wildcard $(ARDUINO_DIR)/libraries/*)), \
	$(shell sed -ne "s/^ *\# *include *[<\"]\(.*\)\.h[>\"]/\1/p" $(SOURCES)))

endif # End .ino or .pde specific instructions

# User source
TARGET := $(if $(TARGET),$(TARGET),a.out)
OBJECTS := $(addsuffix .o, $(basename $(SOURCES)))
BUILD_DIR := build
DEP_FILES := $(patsubst %, $(BUILD_DIR)/%.dep, $(basename $(SOURCES)))
LIB_DIR := $(BUILD_DIR)/lib

# Arduino core library
ARDUINO_CORE_LIB := $(LIB_DIR)/libarduino.a
ARDUINO_CORE_DIR := $(ARDUINO_DIR)/hardware/arduino/cores/arduino
ARDUINO_CORE_OBJS := $(foreach dir, $(ARDUINO_CORE_DIR), \
	$(patsubst %, $(LIB_DIR)/core/%.o, \
	$(notdir $(wildcard $(addprefix $(dir)/, *.c *.cpp *.cc *.C)))))

# Arduino add-in libraries
ARDUINO_LIBS := $(addprefix $(LIB_DIR)/,$(addsuffix .a,$(LIBRARIES)))
ARDUINO_LIBS_DIR := $(ARDUINO_DIR)/libraries
ARDUINO_LIBS_PATH := $(foreach lib, $(LIBRARIES), \
	$(ARDUINO_DIR)/libraries/$(lib)/ $(ARDUINO_LIBS_DIR)/$(lib)/utility/ )
ARDUINO_LIB_OBJS := $(foreach dir, $(ARDUINO_LIBS_PATH), \
	$(patsubst %, $(LIB_DIR)/%.o, \
	$(patsubst $(ARDUINO_LIBS_DIR)/%, %, \
	$(wildcard $(addprefix $(dir), *.c *.cpp *.cc *.C)))))

# Bosch header files
#BOSCH_INCLUDE := ../../arduino_constants.hpp
BOSCH_INCLUDE := $(shell rospack find arduino_interface)/include/arduino_interface
BOSCH_INCLUDE += $(shell rospack find bosch_drivers_common)/include/bosch_drivers_common


# File name for reading the firmware currently programmed into the Arduino
READ_TARGET = current_firmware


#_______________________________________________________________________________
#                                                                 BOARD SETTINGS

# Set default Arduino board/model
ifndef BOARD
# Some examples:
#BOARD := mega2560
#BOARD := leonardo
BOARD := uno

endif

# no board?
ifndef BOARD
ifneq "$(MAKECMDGOALS)" "boards"
ifneq "$(MAKECMDGOALS)" "clean"
$(error BOARD is unset.  Type 'make boards' to see possible values)
endif
endif
endif

# Obtain board-specific parameters from the boards.txt file
BOARDS_FILE := $(ARDUINO_DIR)/hardware/arduino/boards.txt
BOARD_BUILD_MCU := \
	$(shell sed -ne "s/$(BOARD).build.mcu=\(.*\)/\1/p" $(BOARDS_FILE))
BOARD_BUILD_FCPU := \
	$(shell sed -ne "s/$(BOARD).build.f_cpu=\(.*\)/\1/p" $(BOARDS_FILE))
BOARD_BUILD_VARIANT := \
	$(shell sed -ne "s/$(BOARD).build.variant=\(.*\)/\1/p" $(BOARDS_FILE))
BOARD_UPLOAD_SPEED := \
	$(shell sed -ne "s/$(BOARD).upload.speed=\(.*\)/\1/p" $(BOARDS_FILE))
BOARD_UPLOAD_PROTOCOL := \
	$(shell sed -ne "s/$(BOARD).upload.protocol=\(.*\)/\1/p" $(BOARDS_FILE))

# Leonardo (and maybe others) require some additional parameters
ifeq "$(BOARD)" "leonardo"
BOARD_VID := \
	$(shell sed -ne "s/$(BOARD).build.vid=\(.*\)/\1/p" $(BOARDS_FILE))
BOARD_PID := \
	$(shell sed -ne "s/$(BOARD).build.pid=\(.*\)/\1/p" $(BOARDS_FILE))
endif

# invalid board?
ifeq "$(BOARD_BUILD_MCU)" ""
ifneq "$(MAKECMDGOALS)" "boards"
ifneq "$(MAKECMDGOALS)" "clean"
$(error BOARD is invalid.  Type 'make boards' to see possible values)
endif
endif
endif

#_______________________________________________________________________________
#                                                                 BUILD SETTINGS

# Find AVR Tools for building
ifndef AVRTOOLSPATH
AVRTOOLSPATH := $(subst :, , $(PATH))
AVRTOOLSPATH += $(ARDUINO_DIR)/hardware/tools
AVRTOOLSPATH += $(ARDUINO_DIR)/hardware/tools/avr/bin
endif

findsoftware = $(firstword $(wildcard $(addsuffix /$(1), $(AVRTOOLSPATH))))
CC := $(call findsoftware,avr-gcc)
CXX := $(call findsoftware,avr-g++)
LD := $(call findsoftware,avr-ld)
AR := $(call findsoftware,avr-ar)
OBJCOPY := $(call findsoftware,avr-objcopy)
AVRDUDE := $(call findsoftware,avrdude)
AVRSIZE := $(call findsoftware,avr-size)


# Compiler flags, starting with shared flags (CPPFLAGS)
CPPFLAGS := -Os -g
CPPFLAGS += -Wall -Wextra -Wshadow
#CPPFLAGS += -Werror # Too many errors in Arduino core libraries for this flag
CPPFLAGS += -ffunction-sections -fdata-sections
CPPFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CPPFLAGS += -mmcu=$(BOARD_BUILD_MCU)
CPPFLAGS += -DF_CPU=$(BOARD_BUILD_FCPU) -DARDUINO=$(ARDUINO_VER)

# Directories to search for included files
CPPFLAGS += -I. -Iutil -Iutility -I$(ARDUINO_CORE_DIR)
CPPFLAGS += -I$(ARDUINO_DIR)/hardware/arduino/variants/$(BOARD_BUILD_VARIANT)/
CPPFLAGS += $(addprefix -I$(ARDUINO_DIR)/libraries/, $(LIBRARIES))
CPPFLAGS += $(patsubst %, -I$(ARDUINO_DIR)/libraries/%/utility, $(LIBRARIES))
CPPFLAGS += $(addprefix -I,$(BOSCH_INCLUDE))

# Leonardo requires some additional flags
ifeq "$(BOARD)" "leonardo"
CPPFLAGS += -DUSB_VID=$(BOARD_VID) -DUSB_PID=$(BOARD_PID)
endif

# C only flags
CFLAGS += -Wstrict-prototypes

# C++ only flags
CXXFLAGS :=  -fno-exceptions

# Flags for automatically generating dependencies
CXXDEPFLAGS = -MMD -MP -MF $(BUILD_DIR)/$(basename $<).dep

# Flags for compiling .ino files as .cpp
CXXINOFLAGS := -x c++ -include $(ARDUINO_CORE_DIR)/Arduino.h

# Linker flags
LDFLAGS := -Os -Wl,--gc-sections -mmcu=$(BOARD_BUILD_MCU)

# Programmer flags
AVRDUDEFLAGS = $(addprefix -C , $(AVRDUDECONF)) -DV
AVRDUDEFLAGS += -p $(BOARD_BUILD_MCU) -P $(SERIALDEV)
AVRDUDEFLAGS += -c $(BOARD_UPLOAD_PROTOCOL) -b $(BOARD_UPLOAD_SPEED)
ifeq "$(AVRDUDECONF)" ""
ifeq "$(AVRDUDE)" "$(ARDUINO_DIR)/hardware/tools/avr/bin/avrdude"
AVRDUDECONF := $(ARDUINO_DIR)/hardware/tools/avr/etc/avrdude.conf
else
AVRDUDECONF := $(wildcard $(AVRDUDE).conf)
endif
endif

# include dependencies
ifneq "$(MAKECMDGOALS)" "clean"
-include $(DEP_FILES)
endif


#_______________________________________________________________________________
#                                                                          RULES

# Default rule
.DEFAULT_GOAL := all

.PHONY:	all target libs size upload download erase boards clean monitor

all: target size

target: $(TARGET).hex


# Build the Arduino libraries
libs: $(ARDUINO_CORE_LIB) $(ARDUINO_LIBS) 


# Determine the size of the copiled code.
size: $(TARGET).elf
	@echo && $(AVRSIZE) --format=avr --mcu=$(BOARD_BUILD_MCU) $(TARGET).elf


# Upload $(TARGET).hex to the Arduino
upload:
	@echo "\nUploading firmware to board."
	@test -n "$(SERIALDEV)" || { \
		echo "error: SERIALDEV could not be determined automatically." >&2; \
		exit 1; }
	@test 0 -eq $(SERIALDEVGUESS) || { \
		echo "*GUESSING* at serial device:" $(SERIALDEV); \
		echo; }
# Place Leonardo boards into bootloader mode
ifeq "$(BOARD)" "leonardo"
	stty -F $(SERIALDEV) 1200
	sleep 2
endif
	stty $(STTYFARG) $(SERIALDEV) hupcl
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$(TARGET).hex:i


# Download the code on the arduino to $(READ_TARGET).hex
download:
	@echo "\nDownloading firmware from board."
	@test -n "$(SERIALDEV)" || { \
		echo "error: SERIALDEV could not be determined automatically." >&2; \
		exit 1; }
	@test 0 -eq $(SERIALDEVGUESS) || { \
		echo "*GUESSING* at serial device:" $(SERIALDEV); \
		echo; }
	stty $(STTYFARG) $(SERIALDEV) hupcl
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:r:$(READ_TARGET).hex:i


# Erase the code on the Arduino
erase:
	@echo "\nErasing board..."
	$(AVRDUDE) $(AVRDUDEFLAGS) -e


# List the different Arduino boards this makefile can be used to program
boards:
	@echo Available values for BOARD:
	@sed -ne '/^#/d; /^[^.]\+\.name=/p' $(BOARDS_FILE) | \
		sed -e 's/\([^.]\+\)\.name=\(.*\)/\1            \2/' \
			-e 's/\(.\{12\}\) *\(.*\)/\1 \2/'


# Remove all build files and libraries.
clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).eep
	rm -rf $(BUILD_DIR)


# Run the serial monitor for talking to the Arduino
monitor:
	@test -n "$(SERIALDEV)" || { \
		echo "error: SERIALDEV could not be determined automatically." >&2; \
		exit 1; }
	@test -n `which screen` || { \
		echo "error: can't find GNU screen, you might need to install it." >&2 \
		exit 1; }
	@test 0 -eq $(SERIALDEVGUESS) || { \
		echo "*GUESSING* at serial device:" $(SERIALDEV); \
		echo; }
	screen $(SERIALDEV)


# Building the target

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(TARGET).eep: $(TARGET).elf
	$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 \
	--no-change-warnings -O ihex $< $@

.PRECIOUS: $(TARGET).elf

$(TARGET).elf: $(ARDUINO_CORE_LIB) $(ARDUINO_LIBS) $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(ARDUINO_CORE_LIB) $(ARDUINO_LIBS) -o $@

%.o: %.c
	mkdir -p $(BUILD_DIR)/$(dir $<)
	$(COMPILE.c) $(CXXDEPFLAGS) -o $@ $<

%.o: %.cpp
	mkdir -p $(BUILD_DIR)/$(dir $<)
	$(COMPILE.cpp) $(CXXDEPFLAGS) -o $@ $<

%.o: %.cc
	mkdir -p $(BUILD_DIR)/$(dir $<)
	$(COMPILE.cpp) $(CXXDEPFLAGS) -o $@ $<

%.o: %.C
	mkdir -p $(BUILD_DIR)/$(dir $<)
	$(COMPILE.cpp) $(CXXDEPFLAGS) -o $@ $<

%.o: %.ino
	mkdir -p $(BUILD_DIR)/$(dir $<)
	$(COMPILE.cpp) $(CXXDEPFLAGS) -o $@ $(CXXINOFLAGS) $<

%.o: %.pde
	mkdir -p $(BUILD_DIR)/$(dir $<)
	$(COMPILE.cpp) $(CXXDEPFLAGS) -o $@ -x c++ -include $(ARDUINO_CORE_DIR)/Arduino.h $<


# Building the Arduino core library

$(ARDUINO_CORE_LIB): $(ARDUINO_CORE_OBJS)
	@echo
	@echo Building Arduino library
	$(AR) rcs $@ $?

$(LIB_DIR)/core/%.c.o: $(ARDUINO_CORE_DIR)/%.c
#$(LIB_DIR)/%.c.o: %.c
	mkdir -p $(dir $@)
	$(COMPILE.c) -o $@ $<

$(LIB_DIR)/core/%.cpp.o: $(ARDUINO_CORE_DIR)/%.cpp
#$(LIB_DIR)/%.cpp.o: %.cpp
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<

$(LIB_DIR)/%.cc.o: $(ARDUINO_CORE_DIR)/%.cc
#$(LIB_DIR)/%.cc.o: %.cc
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<

$(LIB_DIR)/%.C.o: $(ARDUINO_CORE_DIR)/%.C
#$(LIB_DIR)/%.C.o: %.C
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<


# Build add-in Arduino libraries

$(ARDUINO_LIBS): $(ARDUINO_LIB_OBJS)
	@echo
	@echo Building Arduino add-in library $(notdir $@)
	$(AR) rcs $@ $(filter $(basename $@)%,$?)

$(LIB_DIR)/%.c.o: $(ARDUINO_LIBS_DIR)/%.c
	mkdir -p $(dir $@)
	$(COMPILE.c) -o $@ $<

$(LIB_DIR)/%.cpp.o: $(ARDUINO_LIBS_DIR)/%.cpp
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<

$(LIB_DIR)/%.c.o: $(ARDUINO_LIBS_DIR)/%.cc
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<

$(LIB_DIR)/%.cpp.o: $(ARDUINO_LIBS_DIR)/%.C
	mkdir -p $(dir $@)
	$(COMPILE.cpp) -o $@ $<
