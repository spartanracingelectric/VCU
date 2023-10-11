#
# TTC90 uses M-Type CPU,
# has an external RAM, 
# and provides all safety features
#

# target name
override TARGET = TTC90
APDB_HWTYPE = 0x00000801

# lib file name
LIB_NAME = xc2000_ttc90.lib 

# safety features
SAFETY = true

# CPU settings
CPUNAME = XC2287M-104F
C_SFR_FILE = sfr_regxc2287m.h    

# included files
# TEMP_C_FILES and TEMP_ASM_FILES hold all *.c/*.asm files in src directory
C_FILES = $(filter-out MM_%, $(TEMP_C_FILES))
ASM_FILES = $(TEMP_ASM_FILES)