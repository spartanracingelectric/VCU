###############################################################################
#                                                                             #
#  XC2000  IO-Library                                                         #
#                                                                             #
#  Makefile for building test module                                          #
#                                                                             #
###############################################################################

COMPILER=viper
AREA=appl
DEBUGINFO=true
#TARGET=TTC50
TARGET=TTC60


ifeq ($(origin TARGET), undefined)
    $(info *)
    $(info * Please specify the TARGET which shall be compiled)
    $(info * use:)
    $(info *      TARGET=TTC90       for TTC-90 target)
    $(info *      TARGET=TTC94       for TTC-94 target)
    $(info *      TARGET=TTC60       for TTC-60 target)
    $(info *      TARGET=TTC50       for TTC-50 target)
    $(info *)
    $(info * example: make COMPILER=viper TARGET=TTC60)
    $(info *)
    $(warning WARNING: using default TARGET=TTC60)
    TARGET=TTC60
endif

IODRIVER_DIR = ..

#
# Includes
#
include ../build/settings.mk

INCDIRS += -I"."


# library directory
IODRIVER_LDIR = $(subst \,/,$(IODRIVER_DIR)\lib)

#
# list of source, and object files
#
# test module files
FILES = $(notdir $(basename $(wildcard ./*.c)))
OBJ_FILES := $(addprefix build/, $(addsuffix .obj, $(notdir $(FILES))))

# bsp files
BSP_SDIR = $(subst \,/,$(BSP_PATH))

ifeq ($(TARGET), TTC90)
    # TTC90 
  	BSP_OBJ_FILES = $(BSP_SDIR)/bin/bsp_cstart_ttc90.obj
	DOWNLOADER_HW_TYPE = 0x00000801
else 
	ifeq ($(TARGET), TTC94)
	    # TTC90 
		BSP_OBJ_FILES = $(BSP_SDIR)/bin/bsp_cstart_ttc94.obj
		DOWNLOADER_HW_TYPE = 0x00200801
	else 
		ifeq ($(TARGET), TTC50)
			# TTC50 
			BSP_OBJ_FILES = $(BSP_SDIR)/bin/bsp_cstart_ttc50.obj    
			DOWNLOADER_HW_TYPE = 0x00000401
		else
			# DEFAULT target: TTC60
			BSP_OBJ_FILES = $(BSP_SDIR)/bin/bsp_cstart_ttc60.obj  
			DOWNLOADER_HW_TYPE = 0x00000001  
    		endif
    	endif
endif


LSL_FILE=application.lsl

#
# rules for building
.PHONY: all clean build/%.obj
all : build/main.elf postbuild

# call linker
build/main.elf : $(IODRIVER_LDIR)/$(LIB_NAME) $(BSP_OBJ_FILES) $(OBJ_FILES)
	@echo linking $@
	@"$(TSK_VIPER_LNK)" -o"$@" $(BSP_OBJ_FILES) $(OBJ_FILES) $(IODRIVER_LDIR)/$(LIB_NAME) -Wl-o"build/main.hex":IHEX -Wl-o"build/main.s19":SREC $(TSK_VIPER_LINK_FLAGS) -Wl--map-file="build/main.mapxml":XML $(BSP_SDIR)/lsl/$(TARGET)/$(LSL_FILE)
	@echo  -------------
	@echo  done linking.
	@echo  -------------

	
# build test files
build/%.obj : %.c
	@echo compiling: $<
	@"$(TSK_VIPER_CC)" -c -o $@ $(TSK_VIPER_COMP_FLAGS) $(INCDIRS) $<

clean:
	@echo cleaning up test module files
	-@del /F /Q build\*.*
	@echo done.
	
#---------------------------------------------
# postbuild rule
#---------------------------------------------
postbuild:
	echo $(BSP_SDIR)
	@echo  ---------------------------------
	@echo   Creating APDB...
	@echo  ---------------------------------
	@if exist build\main.hex.addAPDB del build\main.hex.addAPDB
	@if exist build\main.s19.addAPDB del build\main.s19.addAPDB	
	@"$(IODRIVER_DIR)"\\build\\AddAPDB.exe -t $(DOWNLOADER_HW_TYPE) -k "$(IODRIVER_DIR)"\\build\\public.key -v 2.6 -w 32 -f I "$(IODRIVER_DIR)"\\build\\targets.xml build\main.hex build\main.hex.addAPDB
	@"$(IODRIVER_DIR)"\\build\\AddAPDB.exe -t $(DOWNLOADER_HW_TYPE) -k "$(IODRIVER_DIR)"\\build\\public.key -v 2.6 -w 32 -f M "$(IODRIVER_DIR)"\\build\\targets.xml build\main.s19 build\main.s19.addAPDB	
	@copy /Y build\main.hex.addAPDB build\main.hex 
	@copy /Y build\main.s19.addAPDB build\main.s19 	
	@if exist build\main.hex.addAPDB del build\main.hex.addAPDB
	@if exist build\main.s19.addAPDB del build\main.s19.addAPDB

