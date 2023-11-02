#
# Minimodule for Current Measeurement
# build settings like for TTC60
#

# target name
override TARGET = MM_MCM
# lib file name
LIB_NAME = mm_mcm.lib

# safety features
SAFETY = false

# CPU settings
CPUNAME = XC2287-96F
C_SFR_FILE = sfr_regxc2287.h

# included files
# TEMP_C_FILES and TEMP_ASM_FILES hold all *.c/*.asm files in src directory
C_FILES = $(filter MM_INT_% MM_MCM%, $(TEMP_C_FILES))
ASM_FILES = 
