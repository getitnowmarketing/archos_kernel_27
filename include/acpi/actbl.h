/******************************************************************************
 *
 * Name: actbl.h - Basic ACPI Table Definitions
 *
 *****************************************************************************/

/*
 * Copyright (C) 2000 - 2008, Intel Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef __ACTBL_H__
#define __ACTBL_H__

/*
 * Values for description table header signatures. Useful because they make
 * it more difficult to inadvertently type in the wrong signature.
 */
#define ACPI_SIG_DSDT           "DSDT"	/* Differentiated System Description Table */
#define ACPI_SIG_FADT           "FACP"	/* Fixed ACPI Description Table */
#define ACPI_SIG_FACS           "FACS"	/* Firmware ACPI Control Structure */
#define ACPI_SIG_PSDT           "PSDT"	/* Persistent System Description Table */
#define ACPI_SIG_RSDP           "RSD PTR "	/* Root System Description Pointer */
#define ACPI_SIG_RSDT           "RSDT"	/* Root System Description Table */
#define ACPI_SIG_XSDT           "XSDT"	/* Extended  System Description Table */
#define ACPI_SIG_SSDT           "SSDT"	/* Secondary System Description Table */
#define ACPI_RSDP_NAME          "RSDP"	/* Short name for RSDP, not signature */

/*
 * All tables and structures must be byte-packed to match the ACPI
 * specification, since the tables are provided by the system BIOS
 */
#pragma pack(1)

/*
 * These are the ACPI tables that are directly consumed by the subsystem.
 *
 * The RSDP and FACS do not use the common ACPI table header. All other ACPI
 * tables use the header.
 *
 * Note about bitfields: The u8 type is used for bitfields in ACPI tables.
 * This is the only type that is even remotely portable. Anything else is not
 * portable, so do not use any other bitfield types.
 */

/*******************************************************************************
 *
 * ACPI Table Header. This common header is used by all tables except the
 * RSDP and FACS. The define is used for direct inclusion of header into
 * other ACPI tables
 *
 ******************************************************************************/

struct acpi_table_header {
	char signature[ACPI_NAME_SIZE];	/* ASCII table signature */
	u32 length;		/* Length of table in bytes, including this header */
	u8 revision;		/* ACPI Specification minor version # */
	u8 checksum;		/* To make sum of entire table == 0 */
	char oem_id[ACPI_OEM_ID_SIZE];	/* ASCII OEM identification */
	char oem_table_id[ACPI_OEM_TABLE_ID_SIZE];	/* ASCII OEM table identification */
	u32 oem_revision;	/* OEM revision number */
	char asl_compiler_id[ACPI_NAME_SIZE];	/* ASCII ASL compiler vendor ID */
	u32 asl_compiler_revision;	/* ASL compiler version */
};

/*
 * GAS - Generic Address Structure (ACPI 2.0+)
 *
 * Note: Since this structure is used in the ACPI tables, it is byte aligned.
 * If misalignment is not supported, access to the Address field must be
 * performed with care.
 */
struct acpi_generic_address {
	u8 space_id;		/* Address space where struct or register exists */
	u8 bit_width;		/* Size in bits of given register */
	u8 bit_offset;		/* Bit offset within the register */
	u8 access_width;	/* Minimum Access size (ACPI 3.0) */
	u64 address;		/* 64-bit address of struct or register */
};

/*******************************************************************************
 *
 * RSDP - Root System Description Pointer (Signature is "RSD PTR ")
 *
 ******************************************************************************/

struct acpi_table_rsdp {
	char signature[8];	/* ACPI signature, contains "RSD PTR " */
	u8 checksum;		/* ACPI 1.0 checksum */
	char oem_id[ACPI_OEM_ID_SIZE];	/* OEM identification */
	u8 revision;		/* Must be (0) for ACPI 1.0 or (2) for ACPI 2.0+ */
	u32 rsdt_physical_address;	/* 32-bit physical address of the RSDT */
	u32 length;		/* Table length in bytes, including header (ACPI 2.0+) */
	u64 xsdt_physical_address;	/* 64-bit physical address of the XSDT (ACPI 2.0+) */
	u8 extended_checksum;	/* Checksum of entire table (ACPI 2.0+) */
	u8 reserved[3];		/* Reserved, must be zero */
};

#define ACPI_RSDP_REV0_SIZE     20	/* Size of original ACPI 1.0 RSDP */

/*******************************************************************************
 *
 * RSDT/XSDT - Root System Description Tables
 *
 ******************************************************************************/

struct acpi_table_rsdt {
	struct acpi_table_header header;	/* Common ACPI table header */
	u32 table_offset_entry[1];	/* Array of pointers to ACPI tables */
};

struct acpi_table_xsdt {
	struct acpi_table_header header;	/* Common ACPI table header */
	u64 table_offset_entry[1];	/* Array of pointers to ACPI tables */
};

/*******************************************************************************
 *
 * FACS - Firmware ACPI Control Structure (FACS)
 *
 ******************************************************************************/

struct acpi_table_facs {
	char signature[4];	/* ASCII table signature */
	u32 length;		/* Length of structure, in bytes */
	u32 hardware_signature;	/* Hardware configuration signature */
	u32 firmware_waking_vector;	/* 32-bit physical address of the Firmware Waking Vector */
	u32 global_lock;	/* Global Lock for shared hardware resources */
	u32 flags;
	u64 xfirmware_waking_vector;	/* 64-bit version of the Firmware Waking Vector (ACPI 2.0+) */
	u8 version;		/* Version of this table (ACPI 2.0+) */
	u8 reserved[31];	/* Reserved, must be zero */
};

/* Flag macros */

#define ACPI_FACS_S4_BIOS_PRESENT (1)	/* 00: S4BIOS support is present */

/* Global lock flags */

#define ACPI_GLOCK_PENDING      0x01	/* 00: Pending global lock ownership */
#define ACPI_GLOCK_OWNED        0x02	/* 01: Global lock is owned */

/*******************************************************************************
 *
 * FADT - Fixed ACPI Description Table (Signature "FACP")
 *
 ******************************************************************************/

/* Fields common to all versions of the FADT */

struct acpi_table_fadt {
	struct acpi_table_header header;	/* Common ACPI table header */
	u32 facs;		/* 32-bit physical address of FACS */
	u32 dsdt;		/* 32-bit physical address of DSDT */
	u8 model;		/* System Interrupt Model (ACPI 1.0) - not used in ACPI 2.0+ */
	u8 preferred_profile;	/* Conveys preferred power management profile to OSPM. */
	u16 sci_interrupt;	/* System vector of SCI interrupt */
	u32 smi_command;	/* 32-bit Port address of SMI command port */
	u8 acpi_enable;		/* Value to write to smi_cmd to enable ACPI */
	u8 acpi_disable;	/* Value to write to smi_cmd to disable ACPI */
	u8 S4bios_request;	/* Value to write to SMI CMD to enter S4BIOS state */
	u8 pstate_control;	/* Processor performance state control */
	u32 pm1a_event_block;	/* 32-bit Port address of Power Mgt 1a Event Reg Blk */
	u32 pm1b_event_block;	/* 32-bit Port address of Power Mgt 1b Event Reg Blk */
	u32 pm1a_control_block;	/* 32-bit Port address of Power Mgt 1a Control Reg Blk */
	u32 pm1b_control_block;	/* 32-bit Port address of Power Mgt 1b Control Reg Blk */
	u32 pm2_control_block;	/* 32-bit Port address of Power Mgt 2 Control Reg Blk */
	u32 pm_timer_block;	/* 32-bit Port address of Power Mgt Timer Ctrl Reg Blk */
	u32 gpe0_block;		/* 32-bit Port address of General Purpose Event 0 Reg Blk */
	u32 gpe1_block;		/* 32-bit Port address of General Purpose Event 1 Reg Blk */
	u8 pm1_event_length;	/* Byte Length of ports at pm1x_event_block */
	u8 pm1_control_length;	/* Byte Length of ports at pm1x_control_block */
	u8 pm2_control_length;	/* Byte Length of ports at pm2_control_block */
	u8 pm_timer_length;	/* Byte Length of ports at pm_timer_block */
	u8 gpe0_block_length;	/* Byte Length of ports at gpe0_block */
	u8 gpe1_block_length;	/* Byte Length of ports at gpe1_block */
	u8 gpe1_base;		/* Offset in GPE number space where GPE1 events start */
	u8 cst_control;		/* Support for the _CST object and C States change notification */
	u16 C2latency;		/* Worst case HW latency to enter/exit C2 state */
	u16 C3latency;		/* Worst case HW latency to enter/exit C3 state */
	u16 flush_size;		/* Processor's memory cache line width, in bytes */
	u16 flush_stride;	/* Number of flush strides that need to be read */
	u8 duty_offset;		/* Processor duty cycle index in processor's P_CNT reg */
	u8 duty_width;		/* Processor duty cycle value bit width in P_CNT register. */
	u8 day_alarm;		/* Index to day-of-month alarm in RTC CMOS RAM */
	u8 month_alarm;		/* Index to month-of-year alarm in RTC CMOS RAM */
	u8 century;		/* Index to century in RTC CMOS RAM */
	u16 boot_flags;		/* IA-PC Boot Architecture Flags. See Table 5-10 for description */
	u8 reserved;		/* Reserved, must be zero */
	u32 flags;		/* Miscellaneous flag bits (see below for individual flags) */
	struct acpi_generic_address reset_register;	/* 64-bit address of the Reset register */
	u8 reset_value;		/* Value to write to the reset_register port to reset the system */
	u8 reserved4[3];	/* Reserved, must be zero */
	u64 Xfacs;		/* 64-bit physical address of FACS */
	u64 Xdsdt;		/* 64-bit physical address of DSDT */
	struct acpi_generic_address xpm1a_event_block;	/* 64-bit Extended Power Mgt 1a Event Reg Blk address */
	struct acpi_generic_address xpm1b_event_block;	/* 64-bit Extended Power Mgt 1b Event Reg Blk address */
	struct acpi_generic_address xpm1a_control_block;	/* 64-bit Extended Power Mgt 1a Control Reg Blk address */
	struct acpi_generic_address xpm1b_control_block;	/* 64-bit Extended Power Mgt 1b Control Reg Blk address */
	struct acpi_generic_address xpm2_control_block;	/* 64-bit Extended Power Mgt 2 Control Reg Blk address */
	struct acpi_generic_address xpm_timer_block;	/* 64-bit Extended Power Mgt Timer Ctrl Reg Blk address */
	struct acpi_generic_address xgpe0_block;	/* 64-bit Extended General Purpose Event 0 Reg Blk address */
	struct acpi_generic_address xgpe1_block;	/* 64-bit Extended General Purpose Event 1 Reg Blk address */
};

/* FADT flags */

#define ACPI_FADT_WBINVD            (1)	/* 00: The wbinvd instruction works properly */
#define ACPI_FADT_WBINVD_FLUSH      (1<<1)	/* 01: The wbinvd flushes but does not invalidate */
#define ACPI_FADT_C1_SUPPORTED      (1<<2)	/* 02: All processors support C1 state */
#define ACPI_FADT_C2_MP_SUPPORTED   (1<<3)	/* 03: C2 state works on MP system */
#define ACPI_FADT_POWER_BUTTON      (1<<4)	/* 04: Power button is handled as a generic feature */
#define ACPI_FADT_SLEEP_BUTTON      (1<<5)	/* 05: Sleep button is handled as a generic feature, or  not present */
#define ACPI_FADT_FIXED_RTC         (1<<6)	/* 06: RTC wakeup stat not in fixed register space */
#define ACPI_FADT_S4_RTC_WAKE       (1<<7)	/* 07: RTC wakeup stat not possible from S4 */
#define ACPI_FADT_32BIT_TIMER       (1<<8)	/* 08: tmr_val is 32 bits 0=24-bits */
#define ACPI_FADT_DOCKING_SUPPORTED (1<<9)	/* 09: Docking supported */
#define ACPI_FADT_RESET_REGISTER    (1<<10)	/* 10: System reset via the FADT RESET_REG supported */
#define ACPI_FADT_SEALED_CASE       (1<<11)	/* 11: No internal expansion capabilities and case is sealed */
#define ACPI_FADT_HEADLESS          (1<<12)	/* 12: No local video capabilities or local input devices */
#define ACPI_FADT_SLEEP_TYPE        (1<<13)	/* 13: Must execute native instruction after writing  SLP_TYPx register */
#define ACPI_FADT_PCI_EXPRESS_WAKE  (1<<14)	/* 14: System supports PCIEXP_WAKE (STS/EN) bits (ACPI 3.0) */
#define ACPI_FADT_PLATFORM_CLOCK    (1<<15)	/* 15: OSPM should use platform-provided timer (ACPI 3.0) */
#define ACPI_FADT_S4_RTC_VALID      (1<<16)	/* 16: Contents of RTC_STS valid after S4 wake (ACPI 3.0) */
#define ACPI_FADT_REMOTE_POWER_ON   (1<<17)	/* 17: System is compatible with remote power on (ACPI 3.0) */
#define ACPI_FADT_APIC_CLUSTER      (1<<18)	/* 18: All local APICs must use cluster model (ACPI 3.0) */
#define ACPI_FADT_APIC_PHYSICAL     (1<<19)	/* 19: All local x_aPICs must use physical dest mode (ACPI 3.0) */

/*
 * FADT Prefered Power Management Profiles
 */
enum acpi_prefered_pm_profiles {
	PM_UNSPECIFIED = 0,
	PM_DESKTOP = 1,
	PM_MOBILE = 2,
	PM_WORKSTATION = 3,
	PM_ENTERPRISE_SERVER = 4,
	PM_SOHO_SERVER = 5,
	PM_APPLIANCE_PC = 6
};

/* FADT Boot Arch Flags */

#define BAF_LEGACY_DEVICES              0x0001
#define BAF_8042_KEYBOARD_CONTROLLER    0x0002
#define BAF_MSI_NOT_SUPPORTED           0x0008
#define BAF_PCIE_ASPM_CONTROL           0x0010

#define FADT2_REVISION_ID               3
#define FADT2_MINUS_REVISION_ID         2

/* Reset to default packing */

#pragma pack()

#define ACPI_FADT_OFFSET(f)             (u8) ACPI_OFFSET (struct acpi_table_fadt, f)

/*
 * Get the remaining ACPI tables
 */

#include <acpi/actbl1.h>

#endif				/* __ACTBL_H__ */
