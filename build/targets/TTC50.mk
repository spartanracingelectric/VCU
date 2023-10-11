#
# TTC50 uses normal CPU type,
# has NO external RAM, 
# and provides NO safety features
#

# target name
override TARGET = TTC50
APDB_HWTYPE = 0x00000401

# lib file name
LIB_NAME = xc2000_ttc50.lib 

# safety features
SAFETY = false

# CPU settings
CPUNAME = XC2287-96F
C_SFR_FILE = sfr_regxc2287.h

# included files
# TEMP_C_FILES and TEMP_ASM_FILES hold all *.c/*.asm files in src directory
C_FILES = $(filter-out DIAG_% IO_INT_WD% MM_%,$(TEMP_C_FILES))
ASM_FILES =

# other settings
TSK_VIPER_COMP_FLAGS += -D__USIC_MEM_TYPE=__PAGE_080_MEM_TYPE
TSK_VIPER_COMP_FLAGS += -D__MULTICAN_MEM_TYPE=__PAGE_080_MEM_TYPE
