/*
 * (C) Copyright 2017 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _BUILTIN_MODULE_INIT_H
#define _BUILTIN_MODULE_INIT_H


#define SYS_REGS_BASE           BUILTIN_MOD_REG_BASE
#define CH_REGS_BASE            (BUILTIN_MOD_REG_BASE + 0x1080)
#define BANK_REGS_BASE          CH_REGS_BASE
#define PATTERN_GEN_REGS_BASE   (BUILTIN_MOD_REG_BASE + 0x2000)
#define PATTERN_GEN_RAM_BASE    (PATTERN_GEN_REGS_BASE + 0x1000)
#define ID_REGS_BASE            (BUILTIN_MOD_PRIV_REG_BASE + 0x9000000)

/*
 * Module common area
 */
#define MOD_READY_FW_ENTRY          (1UL << 16)
#define MOD_READY_COMMON_VALID      (1UL << 17)
#define MOD_READY_PARAM_LOADED      (1UL << 18)
#define MOD_READY_CAL_LOADED        (1UL << 19)
#define MOD_READY_CPLD1_PROGRAMMED  (1UL << 20)
#define MOD_READY_CPLD2_PROGRAMMED  (1UL << 21)
#define MOD_READY_INIT_DONE         (1UL << 31)

/*Built-in module common memory area map ultrascale 0x8000 0000*/
typedef volatile struct {
    u8      if_mod_sn[16];              /* 0x000: Interface module S/N */
    u8      func_mod_sn[16];            /* 0x010: Functional module S/N */
    u8      func2_mod_sn[16];           /* 0x020: Functional module #2 (top) S/N */
    u32     fpga_build_cnt;             /* 0x030: FPGA compile count */
    u32     builtin_mod_size;           /* 0x034: Built-in module size in HEX */
    u32     _reserved1;                 /* 0x038: reserved */
    u32     fpga_rev;                   /* 0x03C: FPGA revision */
    u32     builtin_mod_id;             /* 0x040: Built-in module ID in ascii */
    u8      if_mod_cpld_rev[16];        /* 0x044: Interface module CPLD revisions */
    u8      func_mod_cpld_rev[16];      /* 0x054: Functional module CPLD revisions */
    u8      func2_mod_cpld_rev[16];     /* 0x064: Functional module #2 (top) CPLD revisions */
    u32     fw_ver;                     /* 0x074: Firmware version */
    u32     uboot_ver;                  /* 0x078: U-Boot version */
    u32     fsbl_ver;                   /* 0x07C: FSBL version */
    char    fw_build_time[24];          /* 0x080: Firmware build time */
    char    uboot_build_time[24];       /* 0x098: U-Boot build time */
    char    fsbl_build_time[24];        /* 0x0B0: FSBL build time */
    u32     _reserved2[2];              /* 0x0C8 - 0x0CC: Reserved */
    u8      if_mod_cpld_img_num[16];    /* 0x0D0: Interface module CPLD image file numbers */
    u8      func_mod_cpld_img_num[16];  /* 0x0E0: Functional module CPLD image file numbers */
    u8      func2_mod_cpld_img_num[16]; /* 0x0F0: Functional module #2 (top) CPLD image file numbers */
    char    if_mod_cpld0_ts[24];        /* 0x100: CPLD Image File #0 Compile Time*/
    char    if_mod_cpld1_ts[24];        /* 0x118: CPLD Image File #1 Compile Time*/
    char    if_mod_cpld2_ts[24];        /* 0x130: CPLD Image File #2 Compile Time*/
    char    if_mod_cpld3_ts[24];        /* 0x148: CPLD Image File #3 Compile Time*/
    u32     _reserved3[39];             /* 0x160 - 0x1F8: Reserved */
    u32     mem_map_rev;                /* 0x1FC: Module Common / Specific Memory Map revision */
    s8      if_mod_temp[8];             /* 0x200: Interface module ambient temperature */
    s8      func_mod_temp[8];           /* 0x208: Functional module ambient temperature */
    s8      func2_mod_temp[8];          /* 0x210: Functional module #2 (top) ambient temperature */
    s8      if_mod_temp_max[8];         /* 0x218: Interface module maximum temperature */
    s8      if_mod_temp_min[8];         /* 0x220: Interface module minimum temperature */
    s8      func_mod_temp_max[8];       /* 0x228: Functional module maximum temperature */
    s8      func_mod_temp_min[8];       /* 0x230: Functional module minimum temperature */
    s8      func2_mod_temp_max[8];      /* 0x238: Functional module #2 (top) maximum temperature */
    s8      func2_mod_temp_min[8];      /* 0x240: Functional module #2 (top) minimum temperature */
    u32     test_enable;                /* 0x248: Test enable */
    u32     test_verify;                /* 0x24C: D2 test verify */
    u32     ps_enable;                  /* 0x250: Power supply enable */
    u32     output_enable;              /* 0x254: Output enable */
    u32     in_calibration;             /* 0x258: Channel currently calibrating */
    u32     mod_ready;                  /* 0x25C: Module ready */
    u32     master_ready;               /* 0x260: Master ready */
    u32     _reserved4[6];              /* 0x264 - 0x278: Reserved */
    u32     cal_disable;                /* 0x27C: Calibration disable */
    u32     _reserved5;                 /* 0x280: Reserved */
    u32     cal_unlock;                 /* 0x284: Calibration Enable - Write 0x41303037 ("A007") to enable calibration mode */
    u32     cal_addr;                   /* 0x288: Calibration Address - Address that will map to RAMs and/or registers */
    u32     cal_data;                   /* 0x28C: Data that will be written to or read from the address set in "Calibration Address" register */
    u32     cal_op;                     /* 0x290: Calibration Operation - Write '1'  to write "Calibration Data" to "Calibration Address"; write '2' to read it */
    u32     test_value;                 /* 0x294: D0 Test Value (Angle, voltage, position, etc) */
    u32     test_range;                 /* 0x298: D0 Test Range (Example AD module has a volt range) */
    u32     zynq_core_volt;             /* 0x29C: Zynq Core voltage */
    u32     zynq_aux_volt;              /* 0x2A0: Zynq Aux voltage */
    u32     zynq_ddr_volt;              /* 0x2A4: Zynq DDR voltage */
    u32     _reserved6[2];              /* 0x2A8 - 0x2AC: Reserved */
    u32     ch_status_en;               /* 0x2B0: Channel Status Enable - 0=disable, 1=enable (bitmapped) */
    u32     _reserved7[3];              /* 0x2B4 - 0x2BC: Reserved */
    u32     if_mod_temp_ex[8];          /* 0x2C0: Interface module ambient temperature (extended) */
    u32     func_mod_temp_ex[8];        /* 0x2E0: Functional module ambient temperature (extended) */
    u32     func2_mod_temp_ex[8];       /* 0x300: Functional module #2 (top) ambient temperature (extended) */
    u32     _reserved8[312];            /* 0x320 - 0x7FC: Reserved */
    struct {                            /* 0x800 - 0xC0C: Interrupt registers */
        u32     raw_status;             /*         0x000: Dynamic status */
        u32     status;                 /*         0x004: Latched status */
        u32     mask;                   /*         0x008: Mask */
        u32     edge_level;             /*         0x00C: Edge/Level */
    } irq[65];
    u32     _reserved9[60];             /* 0xC10 - 0xCFC: Reserved */
    u8      func2_mod_eeprom[256];      /* 0xD00: Functional module #2 (top) EEPROM */
    u8      func_mod_eeprom[256];       /* 0xE00: Functional module EEPROM */
    u8      if_mod_eeprom[256];         /* 0xF00: Interface module EEPROM */
} BUILTIN_MOD_COMMON_AREA;

/*
 * Parameter/calibration file layout
 */
#define PC_FILE_SIZE            0x2000
#define PC_FILE_HDR_SIZE        32
#define PC_FILE_ENTRY_SIZE      8
#define PC_FILE_NUM_ENTRIES     ((PC_FILE_SIZE - PC_FILE_HDR_SIZE) / PC_FILE_ENTRY_SIZE)
#define PARAM_FILE_SIG          "PAR1"
#define CAL_FILE_SIG            "CAL1"
#define CAL_FILE_SN_BYPASS      "\xA0\x07"
#define PARAM_FILE_FLASH_ADDR   0x600000
#define CAL_FILE_FLASH_ADDR     0x610000

typedef volatile struct {
    u32     sig;                /* 0x000: Signature */
    u32     mod_id;             /* 0x004: Module ID */
    u8      mod_sn[16];         /* 0x008: Module S/N */
    u32     _reserved[2];       /* 0x018 - 0x01C: Reserved */
    struct {                    /* 0x020 - XXX: Address/Value pairs */
        u32     addr;
        u32     value;
    } data[];
} PC_FILE;

/*
 * CPLD image
 */
#define CPLD_IMAGE_HDR_SIZE     128
#define CPLD_IMAGE_SIG          0xFF00
#define CPLD_IMAGE_SIZE_LM2     68224
#define CPLD_IMAGE_SIZE_HX8     (CPLD_IMAGE_SIZE_LM2 * 2)
#define CPLD_PART_STRING        "Part: iCE40"
#define CPLD_PART_STRING_LEN    11
#define CPLD_DATE_STRING        "Date: "
#define CPLD_DATE_STRING_LEN    6
#define CPLD_READY_STATUS       0xAA55
#define CPLD_READY_TIMEOUT      150 // 150 ms
#define CPLD1_FLASH_ADDR        0x620000
#define CPLD2_FLASH_ADDR        0x6A0000

typedef enum
{
    CPLD_1 = 0,
    CPLD_2,
    NUM_CPLDS
} CPLD;

typedef volatile struct {
    u32     _reserved[2];       /* 0x00 - 0x04: Reserved */
    u32     prog_en;            /* 0x08: Enable Programming */
    u32     fifo4;              /* 0x0C: CPLD4 FIFO */
    u32     fifo3;              /* 0x10: CPLD3 FIFO */
    u32     fifo2;              /* 0x14: CPLD2 FIFO */
    u32     fifo1;              /* 0x18: CPLD1 FIFO */
    u32     ready;              /* 0x1C: Ready status */
} CPLD_REGS;


/*DT Module Specific Defines*/
/*
 * Are parameter and/or calibration files required for this module?
 */
#define PARAM_FILE_REQUIRED     0
#define CAL_FILE_REQUIRED       1

/*
 * Is programmable CPLD 1 implemented on this module?
 * Should it be programmed before or after module specific initialization?
 */
#define CPLD1_IMPLEMENTED       1
#define CPLD1_PROGRAM_PRE_INIT  0
#define CPLD1_PROGRAM_POST_INIT 1

/*
 * Is programmable CPLD 2 implemented on this module?
 * Should it be programmed before or after module specific initialization?
 */
#define CPLD2_IMPLEMENTED       0
#define CPLD2_PROGRAM_PRE_INIT  0
#define CPLD2_PROGRAM_POST_INIT 0

/*
 * Does the baremetal application need to write to QSPI and thus need to
 * use the QSPI driver? Using the QSPI driver has the side effect of putting
 * QSPI into I/O mode which disables the QSPI controller thus causing the JTAG
 * debugger to not display variable values. If read-only QSPI is needed, simply
 * accessing the QSPI memory directly is what Xilinx recommended.
 * ex: pc_buf[i] = *(volatile u8 *)(FLASH_BASE + (PARAM_FILE_FLASH_ADDR+i));
 * where FLASH_BASE = 0xFC000000 and PARAM_FILE_FLASH_ADDR = 0x00600000
 */
#define QSPI_DRIVER_REQUIRED	0

typedef enum
{
    CHANNEL_1 = 0,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    CHANNEL_8,
    CHANNEL_9,
    CHANNEL_10,
    CHANNEL_11,
    CHANNEL_12,
    CHANNEL_13,
    CHANNEL_14,
    CHANNEL_15,
    CHANNEL_16,
    CHANNEL_17,
    CHANNEL_18,
    CHANNEL_19,
    CHANNEL_20,
    CHANNEL_21,
    CHANNEL_22,
    CHANNEL_23,
    CHANNEL_24,
    NUM_CHANNELS
} CHANNEL;

typedef enum
{
    BANK_A = 0,
    BANK_B,
    BANK_C,
    BANK_D,
    NUM_BANKS
} BANK;

/*
 * System registers
 */
typedef volatile struct {
    u32     read_io;                /* 0x000: Read I/O */
    u32     _reserved1[8];          /* 0x004 - 0x020: Reserved */
    u32     write_out;              /* 0x024: Write Outputs (DATAOUT register) */
    u32     _reserved2[4];          /* 0x028 - 0x34: Reserved */
    u32     io_format_lo;           /* 0x038: IO Format LO */
    u32     io_format_hi;           /* 0x03C: IO Format HI */
    u32     _reserved3[48];         /* 0x040 - 0x0FC: Reserved */
    u32     oc_reset;               /* 0x100: Overcurrent Reset */
    u32     pullup_down;            /* 0x104: Select pullup or pulldown */
} SYS_REGS;

/*
 * Channel registers
 */
typedef volatile struct {
    u32     _reserved1[16];         /* 0x0 - 0x03C: Reserved */
    u32     max_hi_thr;             /* 0x040: Max high threshold (above is sometimes an error) */
    u32     upper_thr;              /* 0x044: Upper threshold (below this, hysteresis applies) */
    u32     lower_thr;              /* 0x048: Lower threshold (above this hysteresis applies) */
    u32     min_lo_thr;             /* 0x04C: Min low threshold (below is sometimes an error) */
    u32     _reserved2;             /* 0x050: Reserved */
    u32     debounce_time;          /* 0x054: Debounce time in ticks (temporary) */
    u32     _reserved3[2];          /* 0x058 - 0x05C: Reserved */
    u32     volt;                   /* 0x060: Measured channel voltage */
    u32     current;                /* 0x064: Measured channel current */
    u32     bit_volt;               /* 0x068: Measured BIT voltage */
    u32     _reserved4[5];          /* 0x06C - 0x07C: Reserved */
} CH_REGS;

/*
 * Bank registers
 */
typedef volatile struct {
    u32     _reserved1[20];         /* 0x000 - 0x04C: Reserved */
    u32     source_sink_current;    /* 0x050: Source/Sink Current */
    u32     _reserved2[6];          /* 0x054 - 0x068: Reserved */
    u32     vcc;                    /* 0x06C: Measured VCC */
    u32     _reserved3[4];          /* 0x070 - 0x07C: Reserved */
} BANK_REGS;

/*
 * Calibration data
 */
typedef volatile struct {
    u32     _reserved1;                         /* 0x000: Reserved */
    u32     volt_gain_ch[NUM_CHANNELS];         /* 0x004 - 0x060: voltage gain calibration datum */
    u32     volt_gain_bank[NUM_BANKS];          /* 0x064 - 0x070: voltage gain calibration datum */
    u32     _reserved2[4];                      /* 0x074 - 0x080: Reserved */
    u32     current_gain[NUM_CHANNELS];         /* 0x084 - 0x0E0: current gain calibration datum */
    u32     _reserved3[8];                      /* 0x0E4 - 0x100: Reserved */
    u32     bit_volt_gain[NUM_CHANNELS];        /* 0x104 - 0x160: BIT voltage gain calibration datum */
    u32     _reserved4[40];                     /* 0x164 - 0x200: Reserved */
    u32     volt_offset_ch[NUM_CHANNELS];       /* 0x204 - 0x260: voltage offset calibration datum */
    u32     volt_offset_bank[NUM_BANKS];        /* 0x264 - 0x270: voltage offset calibration datum */
    u32     _reserved5[4];                      /* 0x274 - 0x280: Reserved */
    u32     current_offset[NUM_CHANNELS];       /* 0x284 - 0x2E0: current offset calibration datum */
    u32     _reserved6[8];                      /* 0x2E4 - 0x300: Reserved */
    u32     bit_volt_offset[NUM_CHANNELS];      /* 0x304 - 0x360: BIT voltage offset calibration datum */
    u32     _reserved7[200];                    /* 0x364 - 0x680: Reserved */
    u32     current_offset_60v[NUM_CHANNELS];   /* 0x684 - 0x6E0: current offset 60V calibration datum */
} CAL_DATA;

/*
 * ID registers
 */
typedef volatile struct {
    u32     id;      /* 0x00: ID (1=DT1, 2=DT2, etc.) */
} ID_REGS;

/*
 * Global variables
 */
extern BUILTIN_MOD_COMMON_AREA *pBuiltinModCommon;

/*Function prototype*/
void builtin_mod_init(void);

#endif /*_BUILTIN_MODULE_INIT_H*/
