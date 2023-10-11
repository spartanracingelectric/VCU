###############################################################################
#                                                                             #
#  XC2000  IO-Library                                                         #
#                                                                             #
#  Settings for the make file                                                 #
#                                                                             #
###############################################################################
SHELL=cmd.exe

###############################################################################
#
# PATH TO IO-DRIVER FILES
#
###############################################################################
ifeq ($(IODRIVER_DIR), )
	IODRIVER_DIR = $(CURDIR)
endif

# Path to Board Support Package
BSP_PATH = $(IODRIVER_DIR)\bsp

###############################################################################
#
# FLAGS FOR BUILD PROCESS
#
###############################################################################

# check if target has been set
ifeq ($(origin TARGET), undefined)
    $(info *)
    $(info * Please specify the TARGET which shall be compiled)
    $(info * use:)
    $(info *      TARGET=TTC94E       for HY-TTC 94E )
    $(info *      TARGET=TTC94        for HY-TTC 94  )
    $(info *      TARGET=TTC90        for HY-TTC 90  )
    $(info *      TARGET=TTC60        for HY-TTC 60  )
    $(info *      TARGET=TTC50        for HY-TTC 50  )
    $(info *) 
    $(info * example: make TARGET=TTC60)
    $(info *)
    $(warning WARNING: using default TARGET=TTC60)
    TARGET=TTC60
endif

# check if area has been set
ifeq ($(origin AREA), undefined)
	AREA=appl
endif

# classic compiler not supported any more.
# print warning if a makefile selects the old compiler
ifeq ($(COMPILER), classic)
    $(warning WARNING: classic compiler not supported any more)
    $(info using viper compiler)
endif
COMPILER = viper

# Generation of Debug information
# set the variable DEBUGINFO=false to omit debug info from library/binary
# by default debug information is generated
ifeq "$(origin DEBUGINFO)" "undefined"
	DEBUGINFO=true
endif

###############################################################################
#
# PATH TO COMPILER INSTALATIONS
#
###############################################################################

# Tasking VX toolset
TSK_VIPER_PATH = c:\Program Files (x86)\TASKING\C166-VX v4.0r1

###############################################################################
#
# COMPILER OPTIONS 
#
###############################################################################

# Tasking Viper compiler flags
TSK_VIPER_COMP_FLAGS = -t -Wa-gAHLs --emit-locals=-equ,-symbols -Wa-OgsaJ --section-info=-console -Wa--error-limit=42
TSK_VIPER_COMP_FLAGS += -Mh --iso=99 --language=+cmp14,+div32,-gcc,-strings -O3 --mac --tradeoff=0 --source
TSK_VIPER_COMP_FLAGS += -Wc--no-tasking-sfr
TSK_VIPER_COMP_FLAGS += -DTASKING_VIPER

ifeq ($(DEBUGINFO),true)
	TSK_VIPER_COMP_FLAGS += -Wc-ga
endif

###############################################################################
#
# LINKER OPTIONS 
#
###############################################################################

# Tasking Viper linker flags
TSK_VIPER_LINK_FLAGS = -t -Wl-OtZXycL -mcrfiklSmNuQ -Wl--error-limit=42 -Wc-ga -Mh

###############################################################################
#
# PATHES AND FILES
#
###############################################################################

#
# PATH SETTINGS     
#

# Directory where sources are located
IODRIVER_SDIR = $(subst \,/,$(IODRIVER_DIR)\src)

# Directory where lib will be stored
IODRIVER_LDIR = $(subst \,/,$(IODRIVER_DIR)\lib)

# Include directories for compiler
INCDIRS = -I"$(IODRIVER_DIR)\inc"
INCDIRS += -I"$(IODRIVER_DIR)\inc_int"
INCDIRS += -I"$(IODRIVER_DIR)\bsp" 

#
# SOURCE FILE LISTS
#

# list of source, and object files
TEMP_C_FILES = $(notdir $(basename $(wildcard $(IODRIVER_SDIR)/*.c)))
TEMP_ASM_FILES = $(notdir $(basename $(wildcard $(IODRIVER_SDIR)/*.asm)))


###############################################################################
#
# TARGET DEPENDENCIES
#
###############################################################################

include $(IODRIVER_DIR)/build/targets/$(TARGET).mk

CPUNAME_SHORT = $(firstword  $(subst -, , $(CPUNAME)))

# set target specific compiler options
TSK_VIPER_COMP_FLAGS += -D$(TARGET)
TSK_VIPER_COMP_FLAGS += -C$(CPUNAME) -Wa-C$(CPUNAME_SHORT)


TSK_VIPER_LINK_FLAGS += -C$(CPUNAME)

ifeq ($(SAFETY),true)
	TSK_VIPER_COMP_FLAGS += -DSAFETY
	TSK_VIPER_LINK_FLAGS += -DSAFETY
endif

# build list of object files
OBJ_FILES := $(addprefix ./build/$(TARGET)/, $(addsuffix .obj, $(notdir $(C_FILES)))) $(addprefix ./build/$(TARGET)/asm_, $(addsuffix .obj, $(notdir $(ASM_FILES))))

###############################################################################
#
# INTERNAL SETTINGS - GENERATED FROM OPTIONS SET ABOVE
#
#   DO NOT CHANGE ANYTHING HERE
#
###############################################################################

#
# Executeables for Toolset
#
TSK_VIPER_CC = $(TSK_VIPER_PATH)\bin\cc166
TSK_VIPER_LNK = $(TSK_VIPER_PATH)\bin\cc166
TSK_VIPER_AR = $(TSK_VIPER_PATH)\bin\ar166
TSK_VIPER_ASM = $(TSK_VIPER_PATH)\bin\as166

TASKING_CC  = $(TSK_VIPER_CC)
TASKING_LNK  = $(TSK_VIPER_LNK)
TASKING_ASM  = $(TSK_VIPER_ASM)
TASKING_AR  = $(TSK_VIPER_AR)

# insert the build date
BUILDDATE_F = $(IODRIVER_DIR)\build\builddate\getbuilddate.exe
BUILDDATE_FUNC = $(subst \,\\,$(BUILDDATE_F))
DATE_S := $(shell $(BUILDDATE_FUNC))

BUILD_DAY = $(word 1, $(DATE_S))
BUILD_MONTH = $(word 2, $(DATE_S))
BUILD_YEAR = $(word 3, $(DATE_S))
BUILD_HOUR = $(word 4, $(DATE_S))
BUILD_MIN = $(word 5, $(DATE_S))

TSK_VIPER_COMP_FLAGS += -DRTS_TTC_FLASH_DATE_YEAR=$(BUILD_YEAR)
TSK_VIPER_COMP_FLAGS += -DRTS_TTC_FLASH_DATE_MONTH=$(BUILD_MONTH)
TSK_VIPER_COMP_FLAGS += -DRTS_TTC_FLASH_DATE_DAY=$(BUILD_DAY)
TSK_VIPER_COMP_FLAGS += -DRTS_TTC_FLASH_DATE_HOUR=$(BUILD_HOUR)
TSK_VIPER_COMP_FLAGS += -DRTS_TTC_FLASH_DATE_MINUTE=$(BUILD_MIN)

COMP_FLAGS = $(TSK_VIPER_COMP_FLAGS)
