/*
 * (C) Copyright 2018 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */
#ifndef _ASM_ARCH_NAI_MB_FPGA_ADDRESS_H
#define _ASM_ARCH_NAI_MB_FPGA_ADDRESS_H

#define BUILTIN_MOD_MEM_BASE            0x80000000UL
#define BUILTIN_MOD_COMMON_AREA_BASE    BUILTIN_MOD_MEM_BASE
#define BUILTIN_MOD_COMMON_AREA_SIZE    0x1000UL
#define BUILTIN_MOD_REG_BASE            (BUILTIN_MOD_COMMON_AREA_BASE + BUILTIN_MOD_COMMON_AREA_SIZE)
#define BUILTIN_MOD_PRIV_REG_BASE       0xA0000000UL
#define BUILTIN_MOD_CAL_BASE            (BUILTIN_MOD_PRIV_REG_BASE + 0x8000000UL)
#define BUILTIN_MOD_CAL_MAX_SIZE        0x8000000UL
#define BUILTIN_MOD_CPLD_BASE           0xAFFFFFE0UL

#define PS2FPGA_HSS_COM_BASE            0x90000000UL

#define PS2FPGA_BASE_ADDRESS                    0x83C00000UL
#define PS2FPGA_TOP_MB_REV_OFFSET               (PS2FPGA_BASE_ADDRESS + 0x0)
#define PS2FPGA_TOP_PS_REV_OFFSET               (PS2FPGA_BASE_ADDRESS + 0x4)
#define PS2FPGA_TOP_MODULE_REV_OFFSET           (PS2FPGA_BASE_ADDRESS + 0x8)
#define PS2FPGA_TOP_SATA_REV_OFFSET             (PS2FPGA_BASE_ADDRESS + 0xC)
#define PS2FPGA_TOP_VME_REV_OFFSET              (PS2FPGA_BASE_ADDRESS + 0x10)
#define PS2FPGA_TOP_VME_IP_REV_OFFSET           (PS2FPGA_BASE_ADDRESS + 0x14)
#define PS2FPGA_TOP_PCIE_REV_OFFSET             (PS2FPGA_BASE_ADDRESS + 0x18)
#define PS2FPGA_TOP_CPIC_REV_OFFSET             (PS2FPGA_BASE_ADDRESS + 0x1C)
#define PS2FPGA_MB_COMPILE_COUNT_OFFSET         (PS2FPGA_BASE_ADDRESS + 0x20)
/* Module ID and Size 0x0028 - 0x005C */
#define PS2FPGA_MOD_INFO_OFFSET 		(PS2FPGA_BASE_ADDRESS + 0x28) 

/* PS2FPGA_CONFIG_STATUS_BASE_ADDRESS 0x83C1 0000 */
#define PS2FPGA_CONFIG_STATUS_BASE_ADDRESS      (PS2FPGA_BASE_ADDRESS + 0x10000UL)
#define PS2FPGA_MB_BUILTIN_MOD_BASE             (PS2FPGA_BASE_ADDRESS + 0x10300UL)
#define PS2FPGA_VME_GEO_ADDR_BASE_ADDR          (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x0)
#define PS2FPGA_VME_GEO_ADDR_OFFSET             (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x0)
#define PS2FPGA_PS_READY_OFFSET                 (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x4)
#define PS2FPGA_RESET_DURATION_OFFSET           (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x8)
#define PS2FPGA_RESET_OFFSET                    (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0xC)

#define PS2FPGA_PCIE_DDR_MAP                    (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x10)
#define PS2FPGA_PCIE_DDR_MASK                   (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x14)

#define PS2FPGA_VME_DDR_MAP                     (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x18)
#define PS2FPGA_VME_DDR_MASK                    (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x1C)

#define PS2FPGA_CPCI_DDR_MAP                    (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x18)
#define PS2FPGA_CPCI_DDR_MASK                    (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x1C)

#define PS2FPGA_GREEN_LED_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x20)
#define PS2FPGA_YELLOW_LED_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x24)
#define PS2FPGA_RED_LED_OFFSET                    (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x28)

#define PS2FPGA_MODULE1_ADDR_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x30)
#define PS2FPGA_MODULE1_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x34)
#define PS2FPGA_MODULE2_ADDR_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x38)
#define PS2FPGA_MODULE2_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x3C)
#define PS2FPGA_MODULE3_ADDR_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x40)
#define PS2FPGA_MODULE3_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x44)
#define PS2FPGA_MODULE4_ADDR_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x48)
#define PS2FPGA_MODULE4_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x4C)
#define PS2FPGA_MODULE5_ADDR_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x50)
#define PS2FPGA_MODULE5_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x54)
#define PS2FPGA_MODULE6_ADDR_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x58)
#define PS2FPGA_MODULE6_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x5C)
#define PS2FPGA_MODULE7_ADDR_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x60)
#define PS2FPGA_MODULE7_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x64)
#define PS2FPGA_ALL_MODULE_ADDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x68)

#define PS2FPGA_PS_IRQ0_VECTOR_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x70)
#define PS2FPGA_PS_IRQ1_VECTOR_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x74)
#define PS2FPGA_PS_IRQ2_VECTOR_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x78)
#define PS2FPGA_PS_IRQS_CLEAR_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x7C)
#define PS2FPGA_PS_IRQ3_VECTOR_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x80)

#define PS2FPGA_EEPROM_WP_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x90)
#define PS2FPGA_SATA_WP_OFFSET                    (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x94)

#define PS2FPGA_VME_DMA_DDR_MAP_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x118)
#define PS2FPGA_VME_DMA_DDR_MASK_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x11C)

#define PS2FPGA_CPCI_DMA_DDR_MAP_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x118)
#define PS2FPGA_CPCI_DMA_DDR_MASK_OFFSET        (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x11C)

#define PS2FPGA_PICE_IRQ_VECTOR_OFFSET            (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x120)
#define PS2FPGA_PICE_IRQ_SET_OFFSET                (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x124)
#define PS2FPGA_MB_SLV_ICB_CONFIG_OFFSET         (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x200)
#define PS2FPGA_MB_SLV_ICB_IDELAYCNTVAL_OFFSET         (PS2FPGA_CONFIG_STATUS_BASE_ADDRESS + 0x204)

/* PS2FPGA_SATA_CONFIG_OFFSET 0x83C2 0000 */
#define PS2FPGA_SATA_CONFIG_OFFSET                (PS2FPGA_BASE_ADDRESS + 0x20000)
#define PS2FPGA_SATA_CONFIG_SIZE                0x4000

/* PS2FPGA_SATA2_CONFIG_OFFSET 0x83C3 0000 */
#define PS2FPGA_SATA2_CONFIG_OFFSET                (PS2FPGA_BASE_ADDRESS + 0x30000)
#define PS2FPGA_SATA2_CONFIG_SIZE                0x4000

/* PS2FPGA_MB_COMMON_AREA_OFFSET 0x83C4 0000 */
#define PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS            (PS2FPGA_BASE_ADDRESS + 0x40000)
#define PS2FPGA_MB_COMMON_VME_CONFIG_BASE_ADDR        (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x164)
#define PS2FPGA_MB_COMMON_VME_ADDR_CONFIG_OFFSET    (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x164)
#define PS2FPGA_MB_COMMON_FSBL_REV_OFFSET             (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x260)
#define PS2FPGA_MB_COMMON_SLAVE_FSBL_REV_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x264)
#define PS2FPGA_MB_COMMON_UBOOT_REV_OFFSET             (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x268)
#define PS2FPGA_MB_COMMON_SLAVE_UBOOT_REV_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x26C)
#define PS2FPGA_MB_COMMON_FPGA_REV_OFFSET             (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x270)
#define PS2FPGA_MB_COMMON_FPGA_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x274)
#define PS2FPGA_MB_COMMON_SLAVE_FPGA_REV_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x278)
#define PS2FPGA_MB_COMMON_SLAVE_FPGA_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x27C)
#define PS2FPGA_MB_COMMON_SLAVE_FW_REV_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x280)
#define PS2FPGA_MB_COMMON_KERNEL_REV_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x284)
#define PS2FPGA_MB_COMMON_ROOTFS_REV_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x288)
#define PS2FPGA_MB_COMMON_FSBL_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x28C)
#define PS2FPGA_MB_COMMON_SLAVE_FSBL_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x2A4)
#define PS2FPGA_MB_COMMON_UBOOT_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x2BC)
#define PS2FPGA_MB_COMMON_SLAVE_UBOOT_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x2D4)
#define PS2FPGA_MB_COMMON_SLAVE_FW_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x2EC)
#define PS2FPGA_MB_COMMON_KERNEL_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x304)
#define PS2FPGA_MB_COMMON_ROOTFS_BUILD_TIME_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x31C)
//Module Status Register is configure by SW
//upper 16 bits - Bitmap for each module phyical connection status (we will use I2C to probe each module slot)
//lower 16 bits - Bitmap for each module serdes ready status
#define PS2FPGA_MB_COMMON_MODULE_STATUS_BASE_ADDR     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x3D8)
#define PS2FPGA_MB_COMMON_MODULE_STATUS_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x3F8)
#define PS2FPGA_MB_COMMON_MOD_SLOT_ADDR_RDY_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x3FC)
#define PS2FPGA_MB_COMMON_MOD_SLOT1_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x400)
#define PS2FPGA_MB_COMMON_MOD_SLOT2_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x404)
#define PS2FPGA_MB_COMMON_MOD_SLOT3_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x408)
#define PS2FPGA_MB_COMMON_MOD_SLOT4_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x40C)
#define PS2FPGA_MB_COMMON_MOD_SLOT5_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x410)
#define PS2FPGA_MB_COMMON_MOD_SLOT6_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x414)
#define PS2FPGA_MB_COMMON_MOD_SLOT7_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x418)
#define PS2FPGA_MB_COMMON_MOD_SLOT8_ADDR_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x41C)
#define PS2FPGA_MB_COMMON_MOD_SLOT1_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x430)
#define PS2FPGA_MB_COMMON_MOD_SLOT2_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x434)
#define PS2FPGA_MB_COMMON_MOD_SLOT3_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x438)
#define PS2FPGA_MB_COMMON_MOD_SLOT4_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x43C)
#define PS2FPGA_MB_COMMON_MOD_SLOT5_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x440)
#define PS2FPGA_MB_COMMON_MOD_SLOT6_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x444)
#define PS2FPGA_MB_COMMON_MOD_SLOT7_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x448)
#define PS2FPGA_MB_COMMON_MOD_SLOT8_SIZE_OFFSET     (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x44C)
#define PS2FPGA_MB_COMMON_MOD_SLOT1_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x460)
#define PS2FPGA_MB_COMMON_MOD_SLOT2_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x464)
#define PS2FPGA_MB_COMMON_MOD_SLOT3_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x468)
#define PS2FPGA_MB_COMMON_MOD_SLOT4_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x46C)
#define PS2FPGA_MB_COMMON_MOD_SLOT5_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x470)
#define PS2FPGA_MB_COMMON_MOD_SLOT6_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x474)
#define PS2FPGA_MB_COMMON_MOD_SLOT7_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x478)
#define PS2FPGA_MB_COMMON_MOD_SLOT8_ID_OFFSET         (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x47C)
#define PS2FPGA_MB_VERSION_BASE_ADDRESS            (PS2FPGA_MB_COMMON_AREA_BASE_ADDRESS + 0x1600)
#define PS2FPGA_MB_COMMON_SIZE                    0x4000

/* PS2FPGA_MODULE_COMMON_AREA_OFFSET 0x83C4 4000 */
#define PS2FPGA_MODULE_BASE_ADDRESS                    (PS2FPGA_BASE_ADDRESS + 0x44000)
#define PS2FPGA_MODULE_CONFIG_BASE_ADDR             (PS2FPGA_MODULE_BASE_ADDRESS + 0x100)
#define PS2FPGA_MODULE_DETECT_STATUS_OFFSET         (PS2FPGA_MODULE_BASE_ADDRESS + 0x100)
#define PS2FPGA_MODULE_LINK_STATUS_OFFSET             (PS2FPGA_MODULE_BASE_ADDRESS + 0x104)
#define PS2FPGA_MODULE_RESET_OFFSET                 (PS2FPGA_MODULE_BASE_ADDRESS + 0x108)
#define PS2FPGA_MODULE_POWER_OFFSET                    (PS2FPGA_MODULE_BASE_ADDRESS + 0x10C)
#define PS2FPGA_MODULE_CLK_OFFSET                      (PS2FPGA_MODULE_BASE_ADDRESS + 0x114)
#define PS2FPGA_MODULE_HSS_OFFSET                      (PS2FPGA_MODULE_BASE_ADDRESS + 0x11C)
#define PS2FPGA_MODULE_DLL_OFFSET                    (PS2FPGA_MODULE_BASE_ADDRESS + 0x120)
#define PS2FPGA_MODULE_CONFIG_MODE_OFFSET           (PS2FPGA_MODULE_BASE_ADDRESS + 0x124)
#define PS2FPGA_MODULE_REF_CLK_OFFSET               (PS2FPGA_MODULE_BASE_ADDRESS + 0x128)
#define PS2FPGA_MODULE_COMMON_SIZE                    0x1000

/*Master and Slave ICB 0x83C4 8000*/
#define ICB_CAL_BASE_ADDRESS                (PS2FPGA_BASE_ADDRESS + 0x48000)
#define ICB_IDELAYCNTVAL_OUT_CH1             (ICB_CAL_BASE_ADDRESS + 0x000)
#define ICB_IDELAYCNTVAL_IN_CH1             (ICB_CAL_BASE_ADDRESS + 0x008)
#define ICB_NUM_SAMP_CH1                    (ICB_CAL_BASE_ADDRESS + 0x010)
#define ICB_SAMP_CNTR_CH1                    (ICB_CAL_BASE_ADDRESS + 0x018)
#define ICB_ERR_CNTR_CH1                    (ICB_CAL_BASE_ADDRESS + 0x020)
#define ICB_ISERDES_DATA_HI_CH1                (ICB_CAL_BASE_ADDRESS + 0x028)
#define ICB_ISERDES_DATA_LO_CH1                (ICB_CAL_BASE_ADDRESS + 0x030)
#define ICB_IDELAYCNTVAL_LD_CH1                (ICB_CAL_BASE_ADDRESS + 0x038)
#define ICB_BITSLIP_CH1                        (ICB_CAL_BASE_ADDRESS + 0x040)
#define ICB_ERR_CNT_RST_CH1                    (ICB_CAL_BASE_ADDRESS + 0x048)
#define ICB_CALDONE_CH1                        (ICB_CAL_BASE_ADDRESS + 0x050)
#define ICB_ISERDES_RESET_IN                (ICB_CAL_BASE_ADDRESS + 0x058)
#define ICB_DATAMATCHSEL_CH1                (ICB_CAL_BASE_ADDRESS + 0x060)
#define ICB_ISERDES_DATA_CH1_LANE1             (ICB_CAL_BASE_ADDRESS + 0x080)
#define ICB_ISERDES_DATA_CH2_LANE1             (ICB_CAL_BASE_ADDRESS + 0x0C0)
#define ICB_IDELAYCNTVAL_OUT_CH1_LANE1         (ICB_CAL_BASE_ADDRESS + 0x100)
#define ICB_IDELAYCNTVAL_OUT_CH2_LANE1         (ICB_CAL_BASE_ADDRESS + 0x140)
#define ICB_IDELAYCNTVAL_IN_CH1_LANE1         (ICB_CAL_BASE_ADDRESS + 0x180)
#define ICB_IDELAYCNTVAL_IN_CH2_LANE1         (ICB_CAL_BASE_ADDRESS + 0x1C0)
#define ICB_IDELAYCNTVAL_LD_CH1_LANE1         (ICB_CAL_BASE_ADDRESS + 0x200)
#define ICB_IDELAYCNTVAL_LD_CH2_LANE1         (ICB_CAL_BASE_ADDRESS + 0x240)
#define ICB_BITSLIP_CH1_LANE1                (ICB_CAL_BASE_ADDRESS + 0x280)
#define ICB_BITSLIP_CH2_LANE1                (ICB_CAL_BASE_ADDRESS + 0x2C0)


/* PS2FPGA_VME_CONFIG_BASE_ADDRESS 0x83C8 0000 */
#define PS2FPGA_VME_CONFIG_BASE_ADDRESS             (PS2FPGA_BASE_ADDRESS + 0x80000)
#define PS2FPGA_VME_CORE_IRQ_REG_OFFSET             (PS2FPGA_VME_CONFIG_BASE_ADDRESS + 0x0)
#define PS2FPGA_VME_CORE_SLAVE_REG_OFFSET        (PS2FPGA_VME_CONFIG_BASE_ADDRESS + 0x4)
#define PS2FPGA_VME_CORE_MASTER_REG_OFFSET        (PS2FPGA_VME_CONFIG_BASE_ADDRESS + 0x8)
#define PS2FPGA_VME_EXT_ADDR_SWITCH_OFFSET        (PS2FPGA_VME_CONFIG_BASE_ADDRESS + 0xC)
#define    PS2FPGA_VME_MASTER_WINDOW_START_ADDR_OFFSET (PS2FPGA_VME_CONFIG_BASE_ADDRESS + 0x100)
#define    PS2FPGA_VME_MASTER_WINDOW_END_ADDR_OFFSET     (PS2FPGA_VME_CONFIG_BASE_ADDRESS + 0x154)
#define PS2FPGA_VME_PORT_ADDRESS                0x80000000
#define VME_LOCAL_USR_CSR_BASEADDR                (PS2FPGA_VME_PORT_ADDRESS)
#define VME_LOCAL_CSR_BASEADDR                    (VME_LOCAL_USR_CSR_BASEADDR + 0x700)

/* PS2FPGA_CPCI_CONFIG_BASE_ADDRESS 0x83C9 0000 */
#define PS2FPGA_CPCI_CONFIG_BASE_ADDRESS        (PS2FPGA_BASE_ADDRESS + 0x90000)
#define PS2FPGA_CPCI_CORE_STATUS_REG_OFFSET        (PS2FPGA_CPCI_CONFIG_BASE_ADDRESS + 0x0)
#define PS2FPGA_CPCI_CORE_SLAVE_REG_OFFSET        (PS2FPGA_CPCI_CONFIG_BASE_ADDRESS + 0x4)
#define PS2FPGA_CPCI_CORE_MASTER_REG_OFFSET        (PS2FPGA_CPCI_CONFIG_BASE_ADDRESS + 0x8)
#define PS2FPGA_CPCI_BAR0_SETUP_REG_OFFSET        (PS2FPGA_CPCI_CONFIG_BASE_ADDRESS + 0xC)
#define PS2FPGA_CPCI_BAR1_SETUP_REG_OFFSET        (PS2FPGA_CPCI_CONFIG_BASE_ADDRESS + 0x10)
#define PS2FPGA_CPCI_PORT_ADDRESS                0x60000000

#define PS2FPGA_XILINX_PCIE_RC_ADDR                    0x83CA0000
#define PS2FPGA_XILINX_PCIE_BUS_MEM_ADDR             0x80000000
#define PS2FPGA_XILINX_PCI_BUS_MEM_SIZE                0x20000000
#define PS2FPGA_XILINX_PCIE_PHY_MEM_ADDR              PS2FPGA_XILINX_PCIE_BUS_MEM_ADDR
#define PS2FPGA_XILINX_PCIE_BUS_CONFIG_ADDR         0xB0000000

/*Slave Zynq BASE_ADDRESS */
#define SLAVE_MEM_BASE            0x40000000
#define SLAVE_DETECTED_MODULE     (SLAVE_MEM_BASE + 0x0060)
#define SLAVE_FW_REVISION        (SLAVE_MEM_BASE + 0x0074)
#define SLAVE_UBOOT_REVISION    (SLAVE_MEM_BASE + 0x0078)
#define SLAVE_FSBL_REVISION        (SLAVE_MEM_BASE + 0x007C)
#define SLAVE_FW_BUILD_TIME        (SLAVE_MEM_BASE + 0x0080)
#define SLAVE_UBOOT_BUILD_TIME    (SLAVE_MEM_BASE + 0x0098)
#define SLAVE_FSBL_BUILD_TIME    (SLAVE_MEM_BASE + 0x00B0)
#define SLAVE_FPGA_BUILD_TIME    (SLAVE_MEM_BASE + 0x1000)
#define SLAVE_FPGA_BOARD_TYPE   (SLAVE_MEM_BASE + 0x1020)
#define SLAVE_FPGA_REVISION        (SLAVE_MEM_BASE + 0x1004)
#define SLAVE_VME_BACKPLANE_GEOADDR             (SLAVE_MEM_BASE + 0x101C)
//Module EEPROM image data are stored in MB Slave common memory area
#define SLAVE_MODULE6_INF_EEPROM_DATA    (SLAVE_MEM_BASE + 0x0A00)
#define SLAVE_MODULE5_INF_EEPROM_DATA    (SLAVE_MEM_BASE + 0x0B00)
#define SLAVE_MODULE4_INF_EEPROM_DATA    (SLAVE_MEM_BASE + 0x0C00)
#define SLAVE_MODULE3_INF_EEPROM_DATA    (SLAVE_MEM_BASE + 0x0D00)
#define SLAVE_MODULE2_INF_EEPROM_DATA    (SLAVE_MEM_BASE + 0x0E00)
#define SLAVE_MODULE1_INF_EEPROM_DATA    (SLAVE_MEM_BASE + 0x0F00)

#endif /* _ASM_ARCH_NAI_MB_FPGA_ADDRESS_H */
