// @(#)default.lsl  1.16
//
//  Linker script file for the VX-toolset for C166
//

// Define the near page addresses. Each DPP will point to a near page.
// It is recommended to keep __DPP3_ADDR at 0x00C000
#define __DPP0_ADDR 0xC10000
#define __DPP1_ADDR 0xC20000
#define __DPP2_ADDR 0xE00000
#define __DPP3_ADDR 0x00C000

// Define the __MEMORY macro when you want to 
// define system memory yourself in this LSL file.
//#define __MEMORY

// Include the CPU specific .lsl file
// The CPU is specified by the __CPU__ macro
#ifndef __CPU__
#error No CPU defined, please link with -D__CPU__=<cpu>
#endif
#define __QUOTE_ME2(x) #x
#define __QUOTE_ME(x) __QUOTE_ME2(x)
#define CPULSL_FILE __QUOTE_ME(__CPU__.lsl)
#include CPULSL_FILE

// Flash Area for application software
#define APPL_FLASH_AREA_START 0xC10080
#define APPL_FLASH_AREA_END   0xCD0000

section_layout ::huge
{
    group my_huge_dsram ( run_addr = mem:spe:DSRAM )
    {
        select "huge_io_driver_data_section";
    }
    group ( ordered, run_addr = mem:xram )
    {
        select "volatile_xram";
    }
    group my_huge_ram ( run_addr = mem:spe:PSRAM )
    {
        select "*" (attributes = +r+w);
    }
}

memory xram
{
    mau = 8;
    type = reserved ram;
    size = 512k;
    map( dest=bus:spe:address, src_offset=0x0, dest_offset=0x400000, size=512k );
}


// Define interrupt vector table
// Resevere the whole first flash sector for the vector table.
section_setup ::code
{
    vector_table "vector_table" ( vector_size = 4, size = 112, run_addr = 0xC20000, 
                template="__vector_template", template_symbol="__lc_vector_target", 
                vector_prefix=".vector.", fill = loop)
    {
        vector (id=0, fill="__cstart");
    }
}

// Define the system stack
section_layout ::shuge (direction = high_to_low)
{
    group ( run_addr = [0xF800..0xFE00], ordered ) stack "system_stack" ( size = 1536 );
}

// reserve memory locations used by others
section_setup ::huge
{
    // registers area in DPRAM
    reserved 0xF600 .. 0xF620;
}

// reserve everything except the fist flash sector for the application
section_setup ::code
{
    reserved 0xC00000 .. 0xC0F000;
}

section_layout ::code
{
    group ( run_addr = [APPL_FLASH_AREA_START .. APPL_FLASH_AREA_END])
    {
        select "code_*";
    }
}

section_layout ::near
{
    group my_near_dsram ( run_addr = mem:spe:DSRAM )
    {
        select "user_stack";
    }
    group bl_near ( run_addr = [0xE00000 .. 0xE04000], fill)
    {
        select (attributes = +w);
    }
}

#define __DTC_START
// using on-chip memory only

#define __DTC_END




#define __FLASH_START
/*******************************************************************************
 * TASKING Linker Script part for flash devices
 * Automatically-generated. Do not edit!
 ******************************************************************************/
#define __FLASH_END


