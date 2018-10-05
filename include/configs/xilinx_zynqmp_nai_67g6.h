/*
 * Configuration for Xilinx ZynqMP zcu102
 *
 * (C) Copyright 2017 North Atlantics, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_ZYNQMP_67G6_H
#define __CONFIG_ZYNQMP_67G6_H

#define CONFIG_SYS_I2C_ZYNQ
/*#define CONFIG_PCA953X*/

#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1
/*#define CONFIG_ZYNQ_EEPROM_BUS		5*/
/*#define CONFIG_ZYNQ_GEM_EEPROM_ADDR	0x54*/
/*#define CONFIG_ZYNQ_GEM_I2C_MAC_OFFSET	0x20*/

#define CONFIG_NAI_MODULE_SUPPORT
#define CONFIG_NAI_BUILTIN_MODULE_SUPPORT

#ifdef CONFIG_NAI_MODULE_SUPPORT
/* NAI Module Feature */
#define NAI_MAX_MODULE_SLOT 6 //max slot: 1 ~ 6
#define NAI_MODULE_SUPPORT
#define NAI_MODULE_ID_IIC

     #ifdef CONFIG_NAI_BUILTIN_MODULE_SUPPORT
        #define NAI_BUILDIN_MODULE_SUPPORT	
        #define NAI_BUILDIN_MOD_SLOT_NUM 1
	    #define NAI_BUILDIN_MOD_SLOT NAI_MAX_MODULE_SLOT
	 #endif
#endif

#define NAI_MDIO_MUX_WR
#ifdef NAI_MDIO_MUX_WR
	#define ZYNQ_GEM1_MDIO_BASEADDR	0xFF0C0000UL
#endif

/*Module AXI I2C Feature */
#define CONFIG_SYS_I2C_ZYNQ_AXI
#ifdef CONFIG_SYS_I2C_ZYNQ_AXI
	/* AXI i2c base address */
	#define ZYNQ_AXI_I2C_BASEADDR0	0x81000000UL
	#define ZYNQ_AXI_I2C_BASEADDR1 	0x81001000UL
	#define ZYNQ_AXI_I2C_BASEADDR2 	0x81002000UL
	#define ZYNQ_AXI_I2C_BASEADDR3 	0x81003000UL
	#define ZYNQ_AXI_I2C_BASEADDR4 	0x81004000UL
	#define ZYNQ_AXI_I2C_BASEADDR5 	0x81005000UL
#endif		
#ifdef NAI_MAX_MODULE_SLOT
	#define NAI_MAX_AXI_I2C_PORT NAI_MAX_MODULE_SLOT
	#define NAI_MODULE_EEPROM_ADDR 0x50
	#define NAI_MODULE1_EEPROM_BUS_NUM 2
	#define NAI_MODULE2_EEPROM_BUS_NUM 3
	#define NAI_MODULE3_EEPROM_BUS_NUM 4
	#define NAI_MODULE4_EEPROM_BUS_NUM 5
	#define NAI_MODULE5_EEPROM_BUS_NUM 6
	#define NAI_MODULE6_EEPROM_BUS_NUM 7
#endif

#include <configs/xilinx_zynqmp_nai.h>

#endif /* __CONFIG_ZYNQMP_67G6_H */
