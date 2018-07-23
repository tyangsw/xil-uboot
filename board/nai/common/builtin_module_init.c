/*
 * (C) Copyright 2017 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <i2c.h>
#include "fpga_address.h"
#include "builtin_module_init.h"
#include "cpld.h"

/* Exported global variables */
BUILTIN_MOD_COMMON_AREA *pBuiltinModCommon = (BUILTIN_MOD_COMMON_AREA *) BUILTIN_MOD_COMMON_AREA_BASE;


/* Global variables */
//static u8 pc_buf[PC_FILE_SIZE];
static CPLD_REGS *pCpld = (CPLD_REGS *)BUILTIN_MOD_CPLD_BASE;

/*DT buil-tin module Global variables */
static SYS_REGS  *pSysRegs = (SYS_REGS *)SYS_REGS_BASE;
static CH_REGS   *pChRegs[NUM_CHANNELS];
static BANK_REGS *pBankRegs[NUM_BANKS];
static CAL_DATA  *pCalData = (CAL_DATA *)BUILTIN_MOD_CAL_BASE;
static ID_REGS   *pIdRegs = (ID_REGS *)ID_REGS_BASE;


/* Local static functions */
/*static function prototype*/
static void _init_common(void);
static void _load_param_file(void);
static void _load_cal_file(void);
static void _program_cpld(u8 cpld);

/*DT builtin module static function prototype*/
static void _dt_init(void);
static void _dt_load_params(void);
static void _dt_load_caldata(void);

static void _dt_init(void)
{
    CHANNEL ch;
    BANK    bank;

    /* Initialize global pointers to registers */
    for (bank = BANK_A; bank < NUM_BANKS; ++bank)
    {
        pBankRegs[bank] = (BANK_REGS *)BANK_REGS_BASE + bank;
    }
    for (ch = CHANNEL_1; ch < NUM_CHANNELS; ++ch)
    {
        pChRegs[ch] = (CH_REGS *)CH_REGS_BASE + ch;
    }

    /* Load configuration parameters */
    _dt_load_params();

    /* Load calibration data */
    _dt_load_caldata();

    /* Tell FPGA which 'type' we are */
    //pIdRegs->id = atol(MODULE_NAME + 2);
    /*TODO removed hardcode FPGA type value*/
    pIdRegs->id = 4;

}


static void _dt_load_params(void)
{
    CHANNEL ch;
    BANK    bank;

    if (!(pBuiltinModCommon->mod_ready & MOD_READY_PARAM_LOADED))
    {
        /* Use default values */
        pSysRegs->write_out     = 0x0;
        pSysRegs->io_format_lo  = 0x0;
        pSysRegs->io_format_hi  = 0x0;
        pSysRegs->pullup_down   = 0x0;

        for (ch = CHANNEL_1; ch < NUM_CHANNELS; ++ch)
        {
            pChRegs[ch]->max_hi_thr     = 0x32;
            pChRegs[ch]->upper_thr      = 0x28;
            pChRegs[ch]->lower_thr      = 0x10;
            pChRegs[ch]->min_lo_thr     = 0xA;
            pChRegs[ch]->debounce_time  = 0x0;
        }

        for (bank = BANK_A; bank < NUM_BANKS; ++bank)
        {
            pBankRegs[bank]->source_sink_current = 0x0;
        }
    }
}


static void _dt_load_caldata(void)
{
    CHANNEL ch;
    BANK    bank;

    if (!(pBuiltinModCommon->mod_ready & MOD_READY_CAL_LOADED))
    {
        /* Use default values */
        for (ch = CHANNEL_1; ch < NUM_CHANNELS; ++ch)
        {
            pCalData->volt_gain_ch[ch]          = 0xC80;
            pCalData->current_gain[ch]          = 0xC000;
            pCalData->bit_volt_gain[ch]         = 0x1030;
            pCalData->volt_offset_ch[ch]        = 0x0;
            pCalData->current_offset[ch]        = 0x0;
            pCalData->bit_volt_offset[ch]       = 0x0;
            pCalData->current_offset_60v[ch]    = 0x0;
        }

        for (bank = BANK_A; bank < NUM_BANKS; ++bank)
        {
            pCalData->volt_gain_bank[bank]      = 0x800;
            pCalData->volt_offset_bank[bank]    = 0x40;
        }
    }
}

void _load_param_file(void)
{
    /*TODO load param file*/
    /* Load parameter file from flash */
    /* Verify signature */
    /* Verify and write the data */
        /* End of file? */
        /* Valid address? */
        /* Write it */
    /* Parameter file has been successfully loaded */
    return;
}

void _load_cal_file(void)
{
     /*TODO load cal file*/
    /* Load calibration file from flash */
    /* Verify signature */
    /* Verify module ID */
    /* Verify module S/N; allow James Bond (Agent 007) to bypass it */
    /* Verify and write the data */
        /* End of file? */
        /* Valid address? */
        /* Write it */
    /* Calibration file has been successfully loaded */
    return;
}

void _program_cpld(u8 cpld)
{
    //u32     *pBuf = (u32 *)SCRATCH_MEM_BASE;
    u32     *pFifo = (cpld == CPLD_1) ? (u32 *)&pCpld->fifo1 : (u32 *)&pCpld->fifo2;
    u32     size;
    u32     pos;
    ulong   time;
    
    /*TODO Load CPLD image header from flash */
    //if (flash_read(addr, CPLD_IMAGE_HDR_SIZE, (u8 *)pBuf))
    //    return;

    /*TODO Verify signature */
    //if (Xil_In16((u32)pBuf) == CPLD_IMAGE_SIG)
    //    return;

    /* Find CPLD part string */
    //part = memmem(pBuf, CPLD_IMAGE_HDR_SIZE, CPLD_PART_STRING, CPLD_PART_STRING_LEN);
    //if (!part)
    //    return;

    /*TODO Get CPLD image size */
    //part += CPLD_PART_STRING_LEN;
    //if (memcmp(part, "LM2", 3) == 0)
    //    size = CPLD_IMAGE_SIZE_LM2;
    //else if ((memcmp(part, "HX8", 3) == 0) ||
    //      (memcmp(part, "HX4", 3) == 0))
    //    size = CPLD_IMAGE_SIZE_HX8;
    //else
    //    return;

    /*TODO Load entire CPLD image from flash */
    //if (flash_read(addr, size, (u8 *)pBuf))
    //    return;

    /* Enable CPLD programming */
    pCpld->prog_en = 1;

    /* Verify CPLD is ready for programming */
    time = get_timer(0);
    while (pCpld->ready != CPLD_READY_STATUS)
    {
        /* Evaluate timeout */
        if (get_timer(time) > CPLD_READY_TIMEOUT)
            return;
    }
    
    size = CPLD_IMAGE_SIZE_HX8;
    pos = 0;
    
    /* Program it */
    while (size)
    {
        //*pFifo = __builtin_bswap32(*pBuf++);
        *pFifo = (CPLD_bin[pos]     << 24) | 
                 (CPLD_bin[pos + 1] << 16) | 
                 (CPLD_bin[pos + 2] << 8)  | 
                 (CPLD_bin[pos + 3] );
                  
        size -= 4;
        pos += 4;
    }
    
    /* CPLD has been successfully programmed */
    pBuiltinModCommon->mod_ready |= (cpld == CPLD_1) ? MOD_READY_CPLD1_PROGRAMMED : MOD_READY_CPLD2_PROGRAMMED;
}


static void _init_common(void)
{
    /*TODO: Get CPLD timestamp*/
    
    /* Common area is now valid */
    pBuiltinModCommon->mod_ready |= MOD_READY_COMMON_VALID;
}

void builtin_mod_init(void)
{
    /* Tell the world we are running */
    pBuiltinModCommon->mod_ready |= MOD_READY_FW_ENTRY;
    /* Initialize module common area */
    _init_common();

    /* Load parameter file */
    _load_param_file();

    /* Load calibration file */
    _load_cal_file();

    /* Program CPLDs */
    _program_cpld(CPLD_1);
    
    /*DT4 init*/
    /*TODO: add run-time decision on module init based pCommonModule->mod_id*/
    _dt_init();
    
    pBuiltinModCommon->mod_ready |= MOD_READY_INIT_DONE;
}

