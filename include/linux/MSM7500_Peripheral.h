
#ifndef __MSM7500_PERIPHERAL_HEADER
#define __MSM7500_PERIPHERAL_HEADER

#include "MSM7500Base.h"


/////////////////////////////////////////////////////////////////////
// NAND
/////////////////////////////////////////////////////////////////////


// Read/Write register
typedef struct _tagNAND_FLASH_CMD
{
	unsigned long op_cmd					: 4;
	unsigned long page_acc					: 1;
	unsigned long last_page					: 1;
	unsigned long auto_detect				: 1;
	unsigned long auto_detect_data_xfr_size	: 10;
}
NAND_FLASH_CMD;


// Read/Write register
typedef struct _tagNAND_FLASH_CHIP_SELECT
{
	unsigned long nand_dev_sel				: 1;
	unsigned long break_xfrs				: 1;
}
NAND_FLASH_CHIP_SELECT;


// Read/Write register
typedef struct _tagNANDC_EXEC_CMD
{
	unsigned long exec_cmd					: 1;
}
NANDC_EXEC_CMD;


// Read/Write register, DEV_STATUS is read only field
typedef struct _tagNAND_FLASH_STATUS
{
	unsigned long oper_status				: 4;
	unsigned long op_err					: 1;
	unsigned long ready_bsy_n				: 1;
	unsigned long reserved_bit6				: 1;					// not used
	unsigned long prog_erase_op_result		: 1;
	unsigned long mpu_error					: 1;
	unsigned long auto_detect_done			: 1;
	unsigned long n512byte_device			: 1;
	unsigned long n2kbyte_device			: 1;
	unsigned long codeword_cntr				: 3;
	unsigned long reserved_bit15			: 1;					// not used
	unsigned long dev_status				: 16;				// read only
}
NAND_FLASH_STATUS;


// Read/Write register
typedef struct _tagNANDC_BUFFER_STATUS
{
	unsigned long num_errors				: 3;
	unsigned long uncorrectable				: 1;
	unsigned long reserved					: 12;				// not used
	unsigned long bad_block_status			: 16;
}
NANDC_BUFFER_STATUS;


// Read/Write register (DEVn, n could be 0 or 1)
typedef struct _tagNAND_DEVn_CFG0
{
	unsigned long col_addr_cycles				: 3;
	unsigned long row_addr_cycles				: 3;
	unsigned long cw_per_page					: 3;
	unsigned long ud_size_bytes					: 10;
	unsigned long ecc_parity_size_bytes			: 4;
	unsigned long spare_size_bytes				: 4;
	unsigned long num_addr_cycles				: 3;
	unsigned long status_bfr_read				: 1;
	unsigned long set_set_rd_mode_after_status	: 1;
}
NAND_DEVn_CFG0;


// Read/Write register (DEVn, n could be 0 or 1)
typedef struct _tagNAND_DEVn_CFG1
{
	unsigned long ecc_disable						: 1;
	unsigned long wide_flash						: 1;
	unsigned long nand_recovery_cycles				: 3;
	unsigned long cs_active_bsy						: 1;
	unsigned long bad_block_byte_num				: 10;
	unsigned long bad_block_in_spare_area			: 1;
	unsigned long wr_rd_bsy_gap						: 6;
	unsigned long page_acc_mpu_partition_addr_range	: 4;
	unsigned long reserved							: 1;
	unsigned long block_acc_mpu_partition_addr_range	: 4;
}
NAND_DEVn_CFG1;


// Read/Write register
typedef struct _tagNAND_FLASH_CONFIG
{
	unsigned long re_en								: 1;
	unsigned long we_en								: 1;
	unsigned long cle								: 1;
	unsigned long ale								: 1;
	unsigned long cs0_n								: 1;
	unsigned long cs1_n								: 1;
	unsigned long cs2_n								: 1;
	unsigned long cs3_n								: 1;
	unsigned long cs4_n								: 1;
	unsigned long cs5_n								: 1;
	unsigned long data_in_en						: 3;
	unsigned long data_out_en						: 1;
	unsigned long addr_out							: 2;
	unsigned long data_out							: 16;
}
NAND_FLASH_CONFIG;


// Read/Write register
typedef struct _tagNAND_FLASH_CONFIG_MODE
{
	unsigned long config_acc						: 1;
}
NAND_FLASH_CONFIG_MODE;


// Read/Write register
typedef struct _tagNAND_FLASH_CONFIG_STATUS
{
	unsigned long config_mode						: 1;
}
NAND_FLASH_CONFIG_STATUS;


// Read/Write register
typedef struct _tagFLASH_MACRO1_REG
{
	unsigned long flash_data_macro0					: 14;
	unsigned long flash_data_macro1					: 6;
}
FLASH_MACRO1_REG;


// Read/Write register
typedef struct _tagFLASH_XFR_STEP
{
	unsigned long extra_read_wait					: 2;
	unsigned long data_wide							: 1;
	unsigned long data_re_en						: 1;
	unsigned long data_we_en						: 1;
	unsigned long data_ale_pin						: 1;
	unsigned long data_cle_en						: 1;
	unsigned long data_ce_en						: 1;
	unsigned long data_data_en						: 1;
	unsigned long data_aout_en						: 1;
	unsigned long data_step1_wait					: 4;
	unsigned long data_seq_step_number				: 2;
	unsigned long reserved							: 2;
	unsigned long cmd_wide							: 1;
	unsigned long cmd_re_en							: 1;
	unsigned long cmd_we_en							: 1;
	unsigned long cmd_ale_pin						: 1;
	unsigned long cmd_cle_en						: 1;
	unsigned long cmd_ce_en							: 1;
	unsigned long cmd_data_en						: 1;
	unsigned long cmd_aout_en						: 1;
	unsigned long cmd_step1_wait					: 4;
	unsigned long cmd_seq_step_number				: 2;
}
FLASH_XFR_STEP;


// Read/Write register
typedef struct _tagFLASH_DEV_CMD0
{
	unsigned long erase_addr						: 8;
	unsigned long erase_start						: 8;
	unsigned long write_addr						: 8;
	unsigned long write_start						: 8;
}
FLASH_DEV_CMD0;


// Read/Write register
typedef struct _tagFLASH_DEV_CMD1
{
	unsigned long read_addr							: 8;
	unsigned long read_start						: 8;
	unsigned long seq_read_mode_addr				: 8;
	unsigned long seq_read_mode_start				: 8;
}
FLASH_DEV_CMD1;


// Read/Write register
typedef struct _tagFLASH_DEV_CMD2
{
	unsigned long read_id							: 8;
	unsigned long read_status						: 8;
	unsigned long reset_cmd							: 8;
	unsigned long read_stop_cmd						: 8;
}
FLASH_DEV_CMD2;


// Read/Write register
typedef struct _tagFLASH_DEV_CMD_VLD
{
	unsigned long read_start_vld					: 1;
	unsigned long read_stop_vld						: 1;
	unsigned long write_start_vld					: 1;
	unsigned long erase_start_vld					: 1;
	unsigned long seq_read_start_vld				: 1;
}
FLASH_DEV_CMD_VLD;


/////////////////////////////////////////////////////////////////////

typedef struct _tagNAND_REGS
{
	volatile NAND_FLASH_CMD			nand_flash_cmd;					// offset 0x00
	volatile unsigned long			nand_addr0;						// offset 0x04		Read/Write [31:0]
	volatile unsigned long			nand_addr1;						// offset 0x08		Read/Write [7:0]
	volatile NAND_FLASH_CHIP_SELECT	nand_flash_chip_select;			// offset 0x0C
	volatile NANDC_EXEC_CMD			nand_exec_cmd;					// offset 0x10
	volatile NAND_FLASH_STATUS		nand_flash_status;				// offset 0x14
	volatile NANDC_BUFFER_STATUS	nandc_buffer_status;			// offset 0x18
	volatile unsigned long			filler1;						// offset 0x1C
	volatile NAND_DEVn_CFG0			nand_dev0_cfg0;					// offset 0x20
	volatile NAND_DEVn_CFG1			nand_dev0_cfg1;					// offset 0x24
	volatile unsigned long			filler2[2];						// offset 0x28-0x2C
	volatile NAND_DEVn_CFG0			nand_dev1_cfg0;					// offset 0x30
	volatile NAND_DEVn_CFG1			nand_dev1_cfg1;					// offset 0x34
	volatile unsigned long			filler3[2];						// offset 0x38-0x3C
	volatile unsigned long			nand_flash_read_id;				// offset 0x40		Read/Write [31:0]
	volatile unsigned long			nand_flash_read_status;			// offset 0x44		Read/Write [15:0]
	volatile unsigned long			filler4[2];						// offset 0x48-0x4C
	volatile unsigned long			nand_flash_config_data;			// offset 0x50		Read/Write [31:0]
	volatile NAND_FLASH_CONFIG		nand_flash_config;				// offset 0x54
	volatile NAND_FLASH_CONFIG_MODE	nand_flash_config_mode;			// offset 0x58
	volatile unsigned long			filler5;						// offset 0x5C
	volatile NAND_FLASH_CONFIG_STATUS	nand_flash_config_status;	// offset 0x60
	volatile FLASH_MACRO1_REG		flash_macro1_reg;				// offset 0x64
	volatile unsigned long			filler6[2];						// offset 0x68-0x6C
	volatile FLASH_XFR_STEP			flash_xfr_step1;				// offset 0x70
	volatile FLASH_XFR_STEP			flash_xfr_step2;				// offset 0x74
	volatile FLASH_XFR_STEP			flash_xfr_step3;				// offset 0x78
	volatile FLASH_XFR_STEP			flash_xfr_step4;				// offset 0x7C
	volatile FLASH_XFR_STEP			flash_xfr_step5;				// offset 0x80
	volatile FLASH_XFR_STEP			flash_xfr_step6;				// offset 0x84
	volatile FLASH_XFR_STEP			flash_xfr_step7;				// offset 0x88
	volatile unsigned long			filler7[5];						// offset 0x8C-0x9C
	volatile FLASH_DEV_CMD0			flash_dev_cmd0;					// offset 0xA0
	volatile FLASH_DEV_CMD1			flash_dev_cmd1;					// offset 0xA4
	volatile FLASH_DEV_CMD2			flash_dev_cmd2;					// offset 0xA8
	volatile FLASH_DEV_CMD_VLD		flash_dev_cmd_vld;				// offset 0xAC
	volatile unsigned long			ebi2_misr_sig_reg;				// offse 0xB0		Read only [31:0]
}
NAND_REGS, *PNAND_REGS;


/////////////////////////////////////////////////////////////////////
// UART
/////////////////////////////////////////////////////////////////////


// Read/Write register
typedef struct _tagUART_MR1
{
	unsigned long auto_rfr_level0	: 6;
	unsigned long cts_ctl			: 1;
	unsigned long rx_rdy_ctl		: 1;
	unsigned long auto_rfr_level1	: 3;
}
UART_MR1;


// Read/Write register
typedef struct _tagUART_MR2
{
	unsigned long parity_mode		: 2;
	unsigned long stop_bit_len		: 2;
	unsigned long bits_per_char		: 2;
	unsigned long error_mode		: 1;
}
UART_MR2;


// Write only register
//typedef struct _tagUART_CSR
//{
//	unsigned long uart_tx_clk_sel	: 4;
//	unsigned long uart_rx_clk_sel	: 4;
//}
//UART_CSR;


// Write only register
//typedef struct _tagUART_TF
//{
//	unsigned long uart_tf			: 8;
//}
//UART_TF;

// Write only register
//typedef struct _tagUART_CR
//{
//	unsigned long uart_rx_en		: 1;
//	unsigned long uart_rx_disable	: 1;
//	unsigned long uart_tx_en		: 1;
//	unsigned long uart_tx_disable	: 1;
//	unsigned long channel_command	: 4;
//}
//UART_CR;
#define BIT_UART_CR_RX_EN					(0x0001 << 0)
#define BIT_UART_CR_RX_DISABLE				(0x0001 << 1)
#define BIT_UART_CR_TX_EN					(0x0001 << 2)
#define BIT_UART_CR_TX_DISABLE				(0x0001 << 3)

#define BIT_UART_CR_CH_CMD_NULL				(0x0000 << 4)
#define BIT_UART_CR_CH_CMD_RESET_RX		(0x0001 << 4)
#define BIT_UART_CR_CH_CMD_RESET_TX		(0x0002 << 4)
#define BIT_UART_CR_CH_CMD_RESET_ERROR		(0x0003 << 4)	// reset error status
#define BIT_UART_CR_CH_CMD_RESET_BREAK		(0x0004 << 4)	// reset break change interrupt
#define BIT_UART_CR_CH_CMD_START_BREAK		(0x0005 << 4)
#define BIT_UART_CR_CH_CMD_STOP_BREAK		(0x0006 << 4)
#define BIT_UART_CR_CH_CMD_RESET_CTS_N		(0x0007 << 4)
#define BIT_UART_CR_CH_CMD_RESET_TX_ERROR	(0x0008 << 4)
#define BIT_UART_CR_CH_CMD_PACKET_MODE	(0x0009 << 4)
#define BIT_UART_CR_CH_CMD_MODE_RESET		(0x000C << 4)
#define BIT_UART_CR_CH_CMD_SET_RFR_N		(0x000D << 4)
#define BIT_UART_CR_CH_CMD_SET_RFR_ND		(0x000E << 4)
#define BIT_UART_CR_CH_CMD_CLEAR_TX_DONE	(0x0010 << 4)


// Write only register
//typedef struct _tagUART_IMR
//{
//	unsigned long txlev				: 1;
//	unsigned long rxhunt			: 1;
//	unsigned long rxbreak			: 1;
//	unsigned long rxstale			: 1;
//	unsigned long rxlev				: 1;
//	unsigned long delta_cts			: 1;
//	unsigned long current_cts		: 1;
//}
//UART_IMR;


// Read/Write register
typedef struct _tagUART_IPR
{
	unsigned long stale_timeout_lsb	: 5;
	unsigned long rxstale_last		: 1;
	unsigned long sample_data		: 1;
	unsigned long stale_timeout_msb	: 4;
}
UART_IPR;


// Read/Write register
typedef struct _tagUART_TFWR
{
	unsigned long tfw				: 9;
}
UART_TFWR;


// Read/Write register
typedef struct _tagUART_RFWR
{
	unsigned long rfw				: 9;
}
UART_RFWR;


// Read/Write register
typedef struct _tagUART_HCR
{
	unsigned long data				: 8;
}
UART_HCR;


// Read/Write register
typedef struct _tagUART_MREG
{
	unsigned long data				: 8;
}
UART_MREG;


// Read/Write register
typedef struct _tagUART_NREG
{
	unsigned long uart_nreg			: 8;
}
UART_NREG;


// Read/Write register
typedef struct _tagUART_DREG
{
	unsigned long data				: 8;
}
UART_DREG;


// Read/Write register
typedef struct _tagUART_MNDREG
{
	unsigned long dreg_lsb			: 2;
	unsigned long nreg_lsb			: 3;
	unsigned long mreg_lsb			: 1;
}
UART_MNDREG;


// Read/Write register
typedef struct _tagUART_IRDA
{
	unsigned long irda_en			: 1;
	unsigned long invert_irda_rx	: 1;
	unsigned long invert_irda_tx	: 1;
	unsigned long irda_loopback		: 1;
}
UART_IRDA;


// Read/Write register
typedef struct _tagUART_MISR_MODE
{
	unsigned long mode				: 2;
}
UART_MISR_MODE;


// Read/Write register
typedef struct _tagUART_MISR_RESET
{
	unsigned long reset				: 1;
}
UART_MISR_RESET;


// Read/Write register
typedef struct _tagUART_MISR_EXPORT
{
	unsigned long export				: 1;
}
UART_MISR_EXPORT;


// Read/Write register
typedef struct _tagUART_MISR_VAL
{
	unsigned long val;
}
UART_MISR_VAL;


// Read/Write register
typedef struct _tagUART_TEST_CTRL
{
	unsigned long test_sel				: 4;
	unsigned long test_en				: 1;
}
UART_TEST_CTRL;


// Read only register
typedef struct _tagUART_SR
{
	unsigned long rxrdy					: 1;
	unsigned long rxfull				: 1;
	unsigned long txrdy					: 1;
	unsigned long txemt					: 1;
	unsigned long uart_overrub			: 1;
	unsigned long par_frame_err			: 1;
	unsigned long rx_break				: 1;
	unsigned long hunt_char				: 1;
}
UART_SR;


// Read only register
//typedef struct _tagUART_RF
//{
//	unsigned long uart_rf			: 8;
//}
//UART_RF;


// Read only register
//typedef struct _tagUART_MISR
//{
//	unsigned long uart_misr			: 7;
//}
//UART_MISR;


// Read only register
typedef struct _tagUART_ISR
{
	unsigned long txlev				: 1;
	unsigned long rxhunt			: 1;
	unsigned long rxbreak			: 1;
	unsigned long rxstale			: 1;
	unsigned long rxlev				: 1;
	unsigned long delta_cts			: 1;
	unsigned long current_cts		: 1;
	unsigned long tx_ready			: 1;
}
UART_ISR, UART2_ISR, UART3_ISR;


/////////////////////////////////////////////////////////////////////

typedef struct _tagUART_REGS
{
	volatile UART_MR1			uart_mr1;			// offset 0x00
	volatile UART_MR2			uart_mr2;			// offset 0x04
	union
	{
		volatile unsigned long	uart_csr;			// offset 0x08
		volatile UART_SR		uart_sr;			// offset 0x08		// for read
	};
	union
	{
		volatile unsigned long	uart_tf;			// offset 0x0C
		volatile unsigned long	uart_rf;			// offset 0x0C		// for read
	};
	union
	{
		volatile unsigned long	uart_cr;			// offset 0x10
		volatile unsigned long	uart_misr;			// offset 0x10		// for read
	};
	union
	{
		volatile unsigned long	uart_imr;			// offset 0x14
		volatile UART_ISR		uart_isr;			// offset 0x14
	};
	volatile UART_IPR			uart_ipr;			// offset 0x18
	volatile UART_TFWR			uart_tfwr;			// offset 0x1C
	volatile UART_RFWR			uart_rfwr;			// offset 0x20
	volatile UART_HCR			uart_hcr;			// offset 0x24
	volatile UART_MREG			uart_mreg;			// offset 0x28
	volatile UART_NREG			uart_nreg;			// offset 0x2C
	volatile UART_DREG			uart_dreg;			// offset 0x30
	volatile UART_MNDREG		uart_mndreg;		// offset 0x34
	volatile UART_IRDA			uart_irda;			// offset 0x38
	volatile unsigned long		filler_1;
	volatile UART_MISR_MODE		uart_misr_mode;		// offset 0x40
	volatile UART_MISR_RESET	uart_misr_reset;	// offset 0x44
	volatile UART_MISR_EXPORT	uart_misr_export;	// offset 0x48
	volatile UART_MISR_VAL		uart_misr_val;		// offset 0x4C
	volatile UART_TEST_CTRL		uart_test_ctrl;		// offset 0x50
}
UART_REGS, *PUART_REGS;


/////////////////////////////////////////////////////////////////////
//
//	UART1DM
//
/////////////////////////////////////////////////////////////////////

typedef struct _tagUART_DM_REGS
{
	volatile UART_MR1           uart_dm_mr1;			// offset 0x0000
	volatile UART_MR2           uart_dm_mr2;			// offset 0x0004
	union
	{
		volatile unsigned long  uart_dm_csr;			// offset 0x0008
		volatile UART_SR        uart_dm_sr;			    // offset 0x0008    // for read
	};
	volatile unsigned long		filler_0;			    // offset 0x000C
	union
	{
		volatile unsigned long	uart_dm_cr;			    // offset 0x0010
		volatile unsigned long	uart_dm_misr;			// offset 0x0010    // for read
	};
	union
	{
		volatile unsigned long	uart_dm_imr;			// offset 0x0014
		volatile UART_ISR       uart_dm_isr;			// offset 0x0014
	};
	volatile UART_IPR           uart_dm_ipr;			// offset 0x0018
	volatile UART_TFWR          uart_dm_tfwr;			// offset 0x001C
	volatile UART_RFWR          uart_dm_rfwr;			// offset 0x0020
	volatile UART_HCR           uart_dm_hcr;			// offset 0x0024
	volatile UART_MREG          uart_dm_mreg;			// offset 0x0028
	volatile UART_NREG          uart_dm_nreg;			// offset 0x002C
	volatile UART_DREG          uart_dm_dreg;			// offset 0x0030
	volatile unsigned long		uart_dm_dmrx;			// offset 0x0034
	union
	{
		volatile UART_IRDA      uart_dm_irda;			// offset 0x0038
		volatile unsigned long	uart_dm_rx_total_snap;	// offset 0x0038
	};
	volatile unsigned long		uart_dm_dmen;			// offset 0x003C
	volatile unsigned long		uart_dm_no_chars_for_tx;// offset 0x0040
	volatile unsigned long		uart_dm_badr;			// offset 0x0044
	volatile unsigned long		uart_dm_testsl;			// offset 0x0048
	volatile unsigned long		uart_dm_txfs;			// offset 0x004C
	volatile unsigned long		uart_dm_rxfs;			// offset 0x0050
	volatile unsigned long		filler_1;			    // offset 0x0054
	volatile unsigned long		filler_2;			    // offset 0x0058
	volatile unsigned long		filler_3;			    // offset 0x005C
	volatile unsigned long		uart_dm_misr_mode;      // offset 0x0060
	volatile unsigned long		uart_dm_misr_reset;     // offset 0x0064
	volatile unsigned long		uart_dm_misr_export;	// offset 0x0068
	volatile unsigned long		uart_dm_misr_val;		// offset 0x006C
	union
	{
		volatile unsigned long	uart_dm_tf;			    // offset 0x0070
		volatile unsigned long	uart_dm_rf;			    // offset 0x0070    // for read
	};
	union
	{
		volatile unsigned long	uart_dm_tf_2;			// offset 0x0074
		volatile unsigned long	uart_dm_rf_2;			// offset 0x0074    // for read
	};
	union
	{
		volatile unsigned long	uart_dm_tf_3;			// offset 0x0078
		volatile unsigned long	uart_dm_rf_3;			// offset 0x0078    // for read
	};
	union
	{
		volatile unsigned long	uart_dm_tf_4;			// offset 0x007C
		volatile unsigned long	uart_dm_rf_4;			// offset 0x007C    // for read
	};
}
UART_DM_REGS, *PUART_DM_REGS;

/////////////////////////////////////////////////////////////////////
// USB
/////////////////////////////////////////////////////////////////////


// Read/Write register
typedef struct _tagUSB_HNP_ASSIST_CTRL
{
	unsigned long hnp_assist_en		: 1;
}
USB_HNP_ASSIST_CTRL;


// Read/Write register
typedef struct _tagUSB_INT_MASK
{
	unsigned long reversed			: 2;
	unsigned long host_wakeup_mask	: 1;
	unsigned long func_wakeup_mask	: 1;
	unsigned long otg_wakeup_mask	: 1;
	unsigned long oe_int_n_mask		: 1;
}
USB_INT_MASK;


// Read only register
typedef struct _tagUSB_INT_STATUS
{
	unsigned long host_int_status		: 1;
	unsigned long func_int_status		: 1;
	unsigned long dma_int_status		: 1;
	unsigned long host_wakeup_status	: 1;
	unsigned long func_wakeup_status	: 1;
	unsigned long otg_wakeup_sttaus		: 1;
	unsigned long oe_int_n_status		: 1;
}
USB_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_TEST_CFG
{
	unsigned long sim_cfg			: 7;
	unsigned long force_test_usb_en	: 1;
}
USB_TEST_CFG;


// Read/Write register
typedef struct _tagUSB_TEST_MODE
{
	unsigned long test_mode			: 3;
}
USB_TEST_MODE;


// Write only register
typedef struct _tagUSB_TEST_MISR_RESET
{
	unsigned long misr_reset		: 1;
}
USB_TEST_MISR_RESET;


// Read/Write register
typedef struct _tagUSB_TEST_EXPORT_MISR
{
	unsigned long export_misr		: 1;
}
USB_TEST_EXPORT_MISR;


// Read/Write register
typedef struct _tagUSB_HARDWARE_MODE
{
	unsigned long crecfg			: 2;
	unsigned long bemde				: 1;
	unsigned long reversed03		: 1;
	unsigned long hostxcvr			: 2;
	unsigned long otgxcvr			: 2;
	unsigned long rh2_2wire_sel		: 1;
	unsigned long rh1_2wire_sel		: 1;
	unsigned long reversed10_13		: 4;
	unsigned long anasdben			: 1;
	unsigned long tstmde			: 1;
	unsigned long hstrev			: 8;
	unsigned long funcrev			: 8;
}
USB_HARDWARE_MODE;


// Read/Write register
typedef struct _tagUSB_CORE_INT_STATUS
{
	unsigned long hcint				: 1;
	unsigned long fcint				: 1;
	unsigned long hnpint			: 1;
	unsigned long ashcint			: 1;
	unsigned long asfcint			: 1;
	unsigned long ashnpint			: 1;
}
USB_CORE_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_CORE_INT_ENA
{
	unsigned long hcinten			: 1;
	unsigned long fcinten			: 1;
	unsigned long hnpinten			: 1;
	unsigned long ashcinten			: 1;
	unsigned long asfcinten			: 1;
	unsigned long ashnpinten		: 1;
}
USB_CORE_INT_ENA;


// Read/Write register
typedef struct _tagUSB_CLOCK_CTL
{
	unsigned long mainclk			: 1;
	unsigned long hstclk			: 1;
	unsigned long funcclk			: 1;
}
USB_CLOCK_CTL;


// Read/Write register
typedef struct _tagUSB_RESET_CTL
{
	unsigned long rsthc				: 1;
	unsigned long rsthsie			: 1;
	unsigned long rstrh				: 1;
	unsigned long rstfsie			: 1;
	unsigned long rstfc				: 1;
	unsigned long rstctrl			: 1;
	unsigned long reversed_06_14	: 9;
	unsigned long rsti2c			: 1;
}
USB_RESET_CTL;


// Read/Write register
typedef struct _tagUSB_FRAME_INTERVAL
{
	unsigned long frmint			: 14;
	unsigned long reversed_14		: 1;
	unsigned long rstfrm			: 1;
	unsigned long frminper			: 14;
}
USB_FRAME_INTERVAL;


// Read/Write register
typedef struct _tagUSB_FRAME_REMAINING_BIT_WIDTH
{
	unsigned long reversed_00_15	: 16;
	unsigned long frmremn			: 14;
}
USB_FRAME_REMAINING_BIT_WIDTH;


// Read/Write register
typedef struct _tagUSB_HNP_CTL_STATUS
{
	unsigned long reversed_00		: 1;
	unsigned long abbusreq			: 1;
	unsigned long adropbus			: 1;
	unsigned long clrerror			: 1;
	unsigned long hnpstate			: 5;
	unsigned long swpddm			: 1;
	unsigned long reversed_10		: 1;
	unsigned long swpudp			: 1;
	unsigned long swautorst			: 1;
	unsigned long reversed_13_14	: 2;
	unsigned long swvbspul			: 1;
	unsigned long reversed_16		: 1;
	unsigned long isadev			: 1;
	unsigned long isbdev			: 1;
	unsigned long cmpen				: 1;
	unsigned long bgen				: 1;
	unsigned long master			: 1;
	unsigned long slave				: 1;
	unsigned long reversed_23		: 1;
	unsigned long bhnpen			: 1;
	unsigned long armthnpe			: 1;
	unsigned long reversed_26		: 1;
	unsigned long vbusgtavv			: 1;
	unsigned long vbusabsv			: 1;
	unsigned long vbusbse			: 1;
	unsigned long hnpdat			: 1;
}
USB_HNP_CTL_STATUS;


// Read/Write register
typedef struct _tagUSB_HNP_TIMERS1
{
	unsigned long awaitvrise		: 8;
	unsigned long awaitconn			: 8;
	unsigned long awaitdisc			: 8;
	unsigned long bwaitconn			: 8;
}
USB_HNP_TIMERS1;


// Read/Write register
typedef struct _tagUSB_HNP_TIMERS2
{
	unsigned long srpdpulw			: 5;
	unsigned long srpvpulw			: 5;
	unsigned long reversed_10_15	: 6;
	unsigned long bsrpfail			: 16;
}
USB_HNP_TIMERS2;


// Read/Write register
typedef struct _tagUSB_HNP_TIMERS3_PULSE_CTL
{
	unsigned long reversed_00_03	: 4;
	unsigned long vbuspulse			: 1;
	unsigned long datapulse			: 1;
	unsigned long insertion			: 1;
	unsigned long reversed_07_15	: 9;
	unsigned long a_sdb_win			: 8;
	unsigned long lngdbnc			: 8;
}
USB_HNP_TIMERS3_PULSE_CTL;


// Read/Write register
typedef struct _tagUSB_HNP_INT_STATUS
{
	unsigned long idchange			: 1;
	unsigned long masslvchg			: 1;
	unsigned long asessvalid		: 1;
	unsigned long bsessvalid		: 1;
	unsigned long vbuserror			: 1;
	unsigned long srpint			: 1;
	unsigned long srpsucfail		: 1;
	unsigned long aidlebdto			: 1;
	unsigned long awaitbto			: 1;
	unsigned long reversed_09_14	: 6;
	unsigned long i2cotgint			: 1;
}
USB_HNP_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_HNP_INT_ENA
{
	unsigned long idchangeen		: 1;
	unsigned long masslvchgen		: 1;
	unsigned long avbusvaliden		: 1;
	unsigned long absesvaliden		: 1;
	unsigned long vbuserroren		: 1;
	unsigned long srpinten			: 1;
	unsigned long srpsucfailen		: 1;
	unsigned long aidlebdtoen		: 1;
	unsigned long awaitbtoen		: 1;
	unsigned long reversed_09_14	: 6;
	unsigned long i2cotginten		: 1;
}
USB_HNP_INT_ENA;


// Read/Write register
typedef struct _tagUSB_FUNCTION_CMD_STATUS
{
	unsigned long resetdet			: 1;
	unsigned long rsminprog			: 1;
	unsigned long suspdet			: 1;
	unsigned long badisoap			: 1;
	unsigned long reversed_04_06	: 3;
	unsigned long softreset			: 1;
}
USB_FUNCTION_CMD_STATUS;


// Read/Write register
typedef struct _tagUSB_DEVICE_ADDRESS
{
	unsigned long devaddr			: 7;
}
USB_DEVICE_ADDRESS;


// Read/Write register
typedef struct _tagUSB_F_SYSTEM_INT_STATUS
{
	unsigned long resetint			: 1;
	unsigned long rsmfinint			: 1;
	unsigned long suspdetint		: 1;
	unsigned long doneregint		: 1;
	unsigned long sofdetint			: 1;
}
USB_F_SYSTEM_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_F_SYSTEM_INT_ENA
{
	unsigned long resetien			: 1;
	unsigned long rsmfinien			: 1;
	unsigned long suspdetien		: 1;
	unsigned long doneregien		: 1;
	unsigned long sofdetien			: 1;
}
USB_F_SYSTEM_INT_ENA;


// Read/Write register
typedef struct _tagUSB_F_X_BUFFER_INT_STATUS
{
	unsigned long xbuf0outint		: 1;
	unsigned long xbuf1inint		: 1;
	unsigned long xbuf2outint		: 1;
	unsigned long xbuf3inint		: 1;
	unsigned long xbuf4outint		: 1;
	unsigned long xbuf5inint		: 1;
	unsigned long xbuf6outint		: 1;
	unsigned long xbuf7inint		: 1;
	unsigned long xbuf8outint		: 1;
	unsigned long xbuf9inint		: 1;
	unsigned long xbuf10outint		: 1;
	unsigned long xbuf11inint		: 1;
	unsigned long xbuf12outint		: 1;
	unsigned long xbuf13inint		: 1;
	unsigned long xbuf14outint		: 1;
	unsigned long xbuf15inint		: 1;
	unsigned long xbuf16outint		: 1;
	unsigned long xbuf17inint		: 1;
	unsigned long xbuf18outint		: 1;
	unsigned long xbuf19inint		: 1;
	unsigned long xbuf20outint		: 1;
	unsigned long xbuf21inint		: 1;
	unsigned long xbuf22outint		: 1;
	unsigned long xbuf23inint		: 1;
	unsigned long xbuf24outint		: 1;
	unsigned long xbuf25inint		: 1;
	unsigned long xbuf26outint		: 1;
	unsigned long xbuf27inint		: 1;
	unsigned long xbuf28outint		: 1;
	unsigned long xbuf29inint		: 1;
	unsigned long xbuf30outint		: 1;
	unsigned long xbuf31inint		: 1;
}
USB_F_X_BUFFER_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_F_Y_BUFFER_INT_STATUS
{
	unsigned long ybuf0outint		: 1;
	unsigned long ybuf1inint		: 1;
	unsigned long ybuf2outint		: 1;
	unsigned long ybuf3inint		: 1;
	unsigned long ybuf4outint		: 1;
	unsigned long ybuf5inint		: 1;
	unsigned long ybuf6outint		: 1;
	unsigned long ybuf7inint		: 1;
	unsigned long ybuf8outint		: 1;
	unsigned long ybuf9inint		: 1;
	unsigned long ybuf10outint		: 1;
	unsigned long ybuf11inint		: 1;
	unsigned long ybuf12outint		: 1;
	unsigned long ybuf13inint		: 1;
	unsigned long ybuf14outint		: 1;
	unsigned long ybuf15inint		: 1;
	unsigned long ybuf16outint		: 1;
	unsigned long ybuf17inint		: 1;
	unsigned long ybuf18outint		: 1;
	unsigned long ybuf19inint		: 1;
	unsigned long ybuf20outint		: 1;
	unsigned long ybuf21inint		: 1;
	unsigned long ybuf22outint		: 1;
	unsigned long ybuf23inint		: 1;
	unsigned long ybuf24outint		: 1;
	unsigned long ybuf25inint		: 1;
	unsigned long ybuf26outint		: 1;
	unsigned long ybuf27inint		: 1;
	unsigned long ybuf28outint		: 1;
	unsigned long ybuf29inint		: 1;
	unsigned long ybuf30outint		: 1;
	unsigned long ybuf31inint		: 1;
}
USB_F_Y_BUFFER_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_F_XY_INT_ENA
{
	unsigned long xy0outen			: 1;
	unsigned long xy1inen			: 1;
	unsigned long xy2outen			: 1;
	unsigned long xy3inen			: 1;
	unsigned long xy4outen			: 1;
	unsigned long xy5inen			: 1;
	unsigned long xy6outen			: 1;
	unsigned long xy7inen			: 1;
	unsigned long xy8outen			: 1;
	unsigned long xy9inen			: 1;
	unsigned long xy10outen			: 1;
	unsigned long xy11inen			: 1;
	unsigned long xy12outen			: 1;
	unsigned long xy13inen			: 1;
	unsigned long xy14outen			: 1;
	unsigned long xy15inen			: 1;
	unsigned long xy16outen			: 1;
	unsigned long xy17inen			: 1;
	unsigned long xy18outen			: 1;
	unsigned long xy19inen			: 1;
	unsigned long xy20outen			: 1;
	unsigned long xy21inen			: 1;
	unsigned long xy22outen			: 1;
	unsigned long xy23inen			: 1;
	unsigned long xy24outen			: 1;
	unsigned long xy25inen			: 1;
	unsigned long xy26outen			: 1;
	unsigned long xy27inen			: 1;
	unsigned long xy28outen			: 1;
	unsigned long xy29inen			: 1;
	unsigned long xy30outen			: 1;
	unsigned long xy31inen			: 1;
}
USB_F_XY_INT_ENA;


// Read/Write register
typedef struct _tagUSB_F_X_FILLED_STATUS
{
	unsigned long xfill0out			: 1;
	unsigned long xfill1in			: 1;
	unsigned long xfill2out			: 1;
	unsigned long xfill3in			: 1;
	unsigned long xfill4out			: 1;
	unsigned long xfill5in			: 1;
	unsigned long xfill6out			: 1;
	unsigned long xfill7in			: 1;
	unsigned long xfill8out			: 1;
	unsigned long xfill9in			: 1;
	unsigned long xfill10out		: 1;
	unsigned long xfill11in			: 1;
	unsigned long xfill12out		: 1;
	unsigned long xfill13in			: 1;
	unsigned long xfill14out		: 1;
	unsigned long xfill15in			: 1;
	unsigned long xfill16out		: 1;
	unsigned long xfill17in			: 1;
	unsigned long xfill18out		: 1;
	unsigned long xfill19in			: 1;
	unsigned long xfill20out		: 1;
	unsigned long xfill21in			: 1;
	unsigned long xfill22out		: 1;
	unsigned long xfill23in			: 1;
	unsigned long xfill24out		: 1;
	unsigned long xfill25in			: 1;
	unsigned long xfill26out		: 1;
	unsigned long xfill27in			: 1;
	unsigned long xfill28out		: 1;
	unsigned long xfill29in			: 1;
	unsigned long xfill30out		: 1;
	unsigned long xfill31in			: 1;
}
USB_F_X_FILLED_STATUS;


// Read/Write register
typedef struct _tagUSB_F_Y_FILLED_STATUS
{
	unsigned long yfill0out			: 1;
	unsigned long yfill1in			: 1;
	unsigned long yfill2out			: 1;
	unsigned long yfill3in			: 1;
	unsigned long yfill4out			: 1;
	unsigned long yfill5in			: 1;
	unsigned long yfill6out			: 1;
	unsigned long yfill7in			: 1;
	unsigned long yfill8out			: 1;
	unsigned long yfill9in			: 1;
	unsigned long yfill10out		: 1;
	unsigned long yfill11in			: 1;
	unsigned long yfill12out		: 1;
	unsigned long yfill13in			: 1;
	unsigned long yfill14out		: 1;
	unsigned long yfill15in			: 1;
	unsigned long yfill16out		: 1;
	unsigned long yfill17in			: 1;
	unsigned long yfill18out		: 1;
	unsigned long yfill19in			: 1;
	unsigned long yfill20out		: 1;
	unsigned long yfill21in			: 1;
	unsigned long yfill22out		: 1;
	unsigned long yfill23in			: 1;
	unsigned long yfill24out		: 1;
	unsigned long yfill25in			: 1;
	unsigned long yfill26out		: 1;
	unsigned long yfill27in			: 1;
	unsigned long yfill28out		: 1;
	unsigned long yfill29in			: 1;
	unsigned long yfill30out		: 1;
	unsigned long yfill31in			: 1;
}
USB_F_Y_FILLED_STATUS;


// Read/Write register
typedef struct _tagUSB_ENDPOINT_ENA
{
	unsigned long ep0outen			: 1;
	unsigned long ep1inen			: 1;
	unsigned long ep2outen			: 1;
	unsigned long ep3inen			: 1;
	unsigned long ep4outen			: 1;
	unsigned long ep5inen			: 1;
	unsigned long ep6outen			: 1;
	unsigned long ep7inen			: 1;
	unsigned long ep8outen			: 1;
	unsigned long ep9inen			: 1;
	unsigned long ep10outen			: 1;
	unsigned long ep11inen			: 1;
	unsigned long ep12outen			: 1;
	unsigned long ep13inen			: 1;
	unsigned long ep14outen			: 1;
	unsigned long ep15inen			: 1;
	unsigned long ep16outen			: 1;
	unsigned long ep17inen			: 1;
	unsigned long ep18outen			: 1;
	unsigned long ep19inen			: 1;
	unsigned long ep20outen			: 1;
	unsigned long ep21inen			: 1;
	unsigned long ep22outen			: 1;
	unsigned long ep23inen			: 1;
	unsigned long ep24outen			: 1;
	unsigned long ep25inen			: 1;
	unsigned long ep26outen			: 1;
	unsigned long ep27inen			: 1;
	unsigned long ep28outen			: 1;
	unsigned long ep29inen			: 1;
	unsigned long ep30outen			: 1;
	unsigned long ep31inen			: 1;
}
USB_ENDPOINT_ENA;


// Read/Write register
typedef struct _tagUSB_ENDPOINT_READY
{
	unsigned long ep0outready		: 1;
	unsigned long ep1inready		: 1;
	unsigned long ep2outready		: 1;
	unsigned long ep3inready		: 1;
	unsigned long ep4outready		: 1;
	unsigned long ep5inready		: 1;
	unsigned long ep6outready		: 1;
	unsigned long ep7inready		: 1;
	unsigned long ep8outready		: 1;
	unsigned long ep9inready		: 1;
	unsigned long ep10outready		: 1;
	unsigned long ep11inready		: 1;
	unsigned long ep12outready		: 1;
	unsigned long ep13inready		: 1;
	unsigned long ep14outready		: 1;
	unsigned long ep15inready		: 1;
	unsigned long ep16outready		: 1;
	unsigned long ep17inready		: 1;
	unsigned long ep18outready		: 1;
	unsigned long ep19inready		: 1;
	unsigned long ep20outready		: 1;
	unsigned long ep21inready		: 1;
	unsigned long ep22outready		: 1;
	unsigned long ep23inready		: 1;
	unsigned long ep24outready		: 1;
	unsigned long ep25inready		: 1;
	unsigned long ep26outready		: 1;
	unsigned long ep27inready		: 1;
	unsigned long ep28outready		: 1;
	unsigned long ep29inready		: 1;
	unsigned long ep30outready		: 1;
	unsigned long ep31inready		: 1;
}
USB_ENDPOINT_READY;


// Read/Write register
typedef struct _tagUSB_F_IMMEDIATE_INT
{
	unsigned long im0outint			: 1;
	unsigned long im1inint			: 1;
	unsigned long im2outint			: 1;
	unsigned long im3inint			: 1;
	unsigned long im4outint			: 1;
	unsigned long im5inint			: 1;
	unsigned long im6outint			: 1;
	unsigned long im7inint			: 1;
	unsigned long im8outint			: 1;
	unsigned long im9inint			: 1;
	unsigned long im10outint		: 1;
	unsigned long im11inint			: 1;
	unsigned long im12outint		: 1;
	unsigned long im13inint			: 1;
	unsigned long im14outint		: 1;
	unsigned long im15inint			: 1;
	unsigned long im16outint		: 1;
	unsigned long im17inint			: 1;
	unsigned long im18outint		: 1;
	unsigned long im19inint			: 1;
	unsigned long im20outint		: 1;
	unsigned long im21inint			: 1;
	unsigned long im22outint		: 1;
	unsigned long im23inint			: 1;
	unsigned long im24outint		: 1;
	unsigned long im25inint			: 1;
	unsigned long im26outint		: 1;
	unsigned long im27inint			: 1;
	unsigned long im28outint		: 1;
	unsigned long im29inint			: 1;
	unsigned long im30outint		: 1;
	unsigned long im31inint			: 1;
}
USB_F_IMMEDIATE_INT;


// Read/Write register
typedef struct _tagUSB_F_ENDPOINT_DONE_STATUS
{
	unsigned long ep0outdone		: 1;
	unsigned long ep1indone			: 1;
	unsigned long ep2outdone		: 1;
	unsigned long ep3indone			: 1;
	unsigned long ep4outdone		: 1;
	unsigned long ep5indone			: 1;
	unsigned long ep6outdone		: 1;
	unsigned long ep7indone			: 1;
	unsigned long ep8outdone		: 1;
	unsigned long ep9indone			: 1;
	unsigned long ep10outdone		: 1;
	unsigned long ep11indone		: 1;
	unsigned long ep12outdone		: 1;
	unsigned long ep13indone		: 1;
	unsigned long ep14outdone		: 1;
	unsigned long ep15indone		: 1;
	unsigned long ep16outdone		: 1;
	unsigned long ep17indone		: 1;
	unsigned long ep18outdone		: 1;
	unsigned long ep19indone		: 1;
	unsigned long ep20outdone		: 1;
	unsigned long ep21indone		: 1;
	unsigned long ep22outdone		: 1;
	unsigned long ep23indone		: 1;
	unsigned long ep24outdone		: 1;
	unsigned long ep25indone		: 1;
	unsigned long ep26outdone		: 1;
	unsigned long ep27indone		: 1;
	unsigned long ep28outdone		: 1;
	unsigned long ep29indone		: 1;
	unsigned long ep30outdone		: 1;
	unsigned long ep31indone		: 1;
}
USB_F_ENDPOINT_DONE_STATUS;


// Read/Write register
typedef struct _tagUSB_ENDPOINT_DONE_ENA
{
	unsigned long ep0outdnen		: 1;
	unsigned long ep1indnen			: 1;
	unsigned long ep2outdnen		: 1;
	unsigned long ep3indnen			: 1;
	unsigned long ep4outdnen		: 1;
	unsigned long ep5indnen			: 1;
	unsigned long ep6outdnen		: 1;
	unsigned long ep7indnen			: 1;
	unsigned long ep8outdnen		: 1;
	unsigned long ep9indnen			: 1;
	unsigned long ep10outdnen		: 1;
	unsigned long ep11indnen		: 1;
	unsigned long ep12outdnen		: 1;
	unsigned long ep13indnen		: 1;
	unsigned long ep14outdnen		: 1;
	unsigned long ep15indnen		: 1;
	unsigned long ep16outdnen		: 1;
	unsigned long ep17indnen		: 1;
	unsigned long ep18outdnen		: 1;
	unsigned long ep19indnen		: 1;
	unsigned long ep20outdnen		: 1;
	unsigned long ep21indnen		: 1;
	unsigned long ep22outdnen		: 1;
	unsigned long ep23indnen		: 1;
	unsigned long ep24outdnen		: 1;
	unsigned long ep25indnen		: 1;
	unsigned long ep26outdnen		: 1;
	unsigned long ep27indnen		: 1;
	unsigned long ep28outdnen		: 1;
	unsigned long ep29indnen		: 1;
	unsigned long ep30outdnen		: 1;
	unsigned long ep31indnen		: 1;
}
USB_ENDPOINT_DONE_ENA;


// Read/Write register
typedef struct _tagUSB_ENDPOINT_TOGGLE_BITS
{
	unsigned long ep0outtog			: 1;
	unsigned long ep1intog			: 1;
	unsigned long ep2outtog			: 1;
	unsigned long ep3intog			: 1;
	unsigned long ep4outtog			: 1;
	unsigned long ep5intog			: 1;
	unsigned long ep6outtog			: 1;
	unsigned long ep7intog			: 1;
	unsigned long ep8outtog			: 1;
	unsigned long ep9intog			: 1;
	unsigned long ep10outtog		: 1;
	unsigned long ep11intog			: 1;
	unsigned long ep12outtog		: 1;
	unsigned long ep13intog			: 1;
	unsigned long ep14outtog		: 1;
	unsigned long ep15intog			: 1;
	unsigned long ep16outtog		: 1;
	unsigned long ep17intog			: 1;
	unsigned long ep18outtog		: 1;
	unsigned long ep19intog			: 1;
	unsigned long ep20outtog		: 1;
	unsigned long ep21intog			: 1;
	unsigned long ep22outtog		: 1;
	unsigned long ep23intog			: 1;
	unsigned long ep24outtog		: 1;
	unsigned long ep25intog			: 1;
	unsigned long ep26outtog		: 1;
	unsigned long ep27intog			: 1;
	unsigned long ep28outtog		: 1;
	unsigned long ep29intog			: 1;
	unsigned long ep30outtog		: 1;
	unsigned long ep31intog			: 1;
}
USB_ENDPOINT_TOGGLE_BITS;


// Read/Write register
typedef struct _tagUSB_F_FRAME_NUMBER
{
	unsigned long frmnumb			: 11;
}
USB_F_FRAME_NUMBER;


// Read/Write register
typedef struct _tagUSB_HOST_CTL
{
	unsigned long ctlblksr			: 2;
	unsigned long hcusbste			: 2;
	unsigned long rmtwuen			: 1;
	unsigned long revresed_05_15	: 11;
	unsigned long schedovr			: 2;
	unsigned long revresed_18_30	: 13;
	unsigned long hcreset			: 1;
}
USB_HOST_CTL;


// Read/Write register
typedef struct _tagUSB_H_SYSTEM_INT_STATUS
{
	unsigned long sorint			: 1;
	unsigned long doneint			: 1;
	unsigned long sofint			: 1;
	unsigned long resdetint			: 1;
	unsigned long herrint			: 1;
	unsigned long fmofint			: 1;
	unsigned long pscint			: 1;
}
USB_H_SYSTEM_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_H_SYSTEM_INT_ENA
{
	unsigned long sorien			: 1;
	unsigned long doneien			: 1;
	unsigned long sofien			: 1;
	unsigned long resdetien			: 1;
	unsigned long herrien			: 1;
	unsigned long fmofien			: 1;
	unsigned long pscien			: 1;
}
USB_H_SYSTEM_INT_ENA;


// Read/Write register
typedef struct _tagUSB_ROOT_HUB_DESCRIPTOR_A
{
	unsigned long ndnstmprt			: 8;
	unsigned long nopwrswt			: 1;
	unsigned long pwrswtmd			: 1;
	unsigned long devtype			: 1;
	unsigned long ovrcurpm			: 1;
	unsigned long noovrcurp			: 1;
	unsigned long reversed_13_23	: 11;
	unsigned long pwrtogood			: 8;
}
USB_ROOT_HUB_DESCRIPTOR_A;


// Read/Write register
typedef struct _tagUSB_ROOT_HUB_DESCRIPTOR_B
{
	unsigned long devremove			: 8;
	unsigned long reversed_08_15	: 8;
	unsigned long prtpwrcm			: 8;
}
USB_ROOT_HUB_DESCRIPTOR_B;


// Read/Write register
typedef struct _tagUSB_ROOT_HUB_STATUS
{
	unsigned long locpwrs			: 1;
	unsigned long ovrcuri			: 1;
	unsigned long reversed_02_14	: 13;
	unsigned long devconwue			: 1;
	unsigned long locpwrsc			: 1;
	unsigned long ovrcurchg			: 1;
	unsigned long reversed_18_30	: 13;
	unsigned long clrmtwue			: 1;
}
USB_ROOT_HUB_STATUS;


// Read/Write register
typedef struct _tagUSB_PORT_STATUS
{
	unsigned long curconst			: 1;
	unsigned long prtenabst			: 1;
	unsigned long prtsuspst			: 1;
	unsigned long prtovrcuri		: 1;
	unsigned long prtrstst			: 1;
	unsigned long reversed_05_07	: 3;
	unsigned long prtpwrst			: 1;
	unsigned long lsdevcon			: 1;
	unsigned long reversed_10_15	: 6;
	unsigned long connectsc			: 1;
	unsigned long prtenblsc			: 1;
	unsigned long prtstatsc			: 1;
	unsigned long ovrcuric			: 1;
	unsigned long prtrstsc			: 1;
}
USB_PORT_STATUS;


// Read/Write register
typedef struct _tagUSB_DMA_REVISION
{
	unsigned long revcode			: 8;
}
USB_DMA_REVISION;


// Read/Write register
typedef struct _tagUSB_DMA_INT_STATUS
{
	unsigned long etderr			: 1;
	unsigned long eperr				: 1;
}
USB_DMA_INT_STATUS;


// Read/Write register
typedef struct _tagUSB_DMA_INT_ENA
{
	unsigned long etderrinten		: 1;
	unsigned long eperrinten		: 1;
}
USB_DMA_INT_ENA;


// Read/Write register
typedef struct _tagUSB_MISC_CTL
{
	unsigned long filtcc			: 1;
	unsigned long arbmode			: 1;
	unsigned long skprtry			: 1;
	unsigned long isoprevfrm		: 1;
}
USB_MISC_CTL;


/////////////////////////////////////////////////////////////////////

typedef struct _tagUSB_RGES
{
	volatile USB_HNP_ASSIST_CTRL			usb_hnp_assist_ctrl;			// offset 0x00
	volatile USB_INT_MASK					usb_int_mask;					// offset 0x04
	volatile USB_INT_STATUS					usb_int_status;					// offset 0x08	Read only
	volatile USB_TEST_CFG					usb_test_cfg;					// offset 0x0C
	volatile USB_TEST_MODE					usb_test_mode;					// offset 0x10
	volatile USB_TEST_MISR_RESET			usb_test_misr_reset;			// offset 0x14	Write only
	volatile USB_TEST_EXPORT_MISR			usb_test_export_misr;			// offset 0x18
	volatile unsigned long					usb_misr_curr_val;				// offset 0x1C	Read only

	volatile unsigned long					reversed_0x0020[1016];			// offset 0x20-0xFFC

	volatile USB_HARDWARE_MODE				usb_hardware_mode;				// offset 0x1000
	volatile USB_CORE_INT_STATUS			usb_core_int_status;			// offset 0x1004
	volatile USB_CORE_INT_ENA				usb_core_int_ena;				// offset 0x1008
	volatile USB_CLOCK_CTL					usb_clock_ctl;					// offset 0x100C
	volatile USB_RESET_CTL					usb_reset_ctl;					// offset 0x1010
	volatile USB_FRAME_INTERVAL				usb_frame_interval;				// offset 0x1014
	volatile USB_FRAME_REMAINING_BIT_WIDTH	usb_frame_remaining_bit_width;	// offset 0x1018
	volatile USB_HNP_CTL_STATUS				usb_hnp_ctl_status;				// offset 0x101C
	volatile USB_HNP_TIMERS1				usb_hnp_timers1;				// offset 0x1020
	volatile USB_HNP_TIMERS2				usb_hnp_timers2;				// offset 0x1024
	volatile USB_HNP_TIMERS3_PULSE_CTL		usb_hnp_timers3_pulse_ctl;		// offset 0x1028
	volatile USB_HNP_INT_STATUS				usb_hnp_int_status;				// offset 0x102C
	volatile USB_HNP_INT_ENA				usb_hnp_int_ena;				// offset 0x1030

	volatile unsigned long					reversed_0x1034[3];				// offset 0x1034-0x103C

	volatile USB_FUNCTION_CMD_STATUS		usb_function_cmd_status;		// offset 0x1040
	volatile USB_DEVICE_ADDRESS				usb_device_address;				// offset 0x1044
	volatile USB_F_SYSTEM_INT_STATUS		usb_f_system_int_status;		// offset 0x1048
	volatile USB_F_SYSTEM_INT_ENA			usb_f_system_int_ena;			// offset 0x104C
	volatile USB_F_X_BUFFER_INT_STATUS		usb_f_x_buffer_int_status;		// offset 0x1050
	volatile USB_F_Y_BUFFER_INT_STATUS		usb_f_y_buffer_int_status;		// offset 0x1054
	volatile USB_F_XY_INT_ENA				usb_f_xy_int_ena;				// offset 0x1058
	volatile USB_F_X_FILLED_STATUS			usb_f_x_filled_status;			// offset 0x105C
	volatile USB_F_Y_FILLED_STATUS			usb_f_y_filled_status;			// offset 0x1060
	volatile USB_ENDPOINT_ENA				usb_endpoint_ena;				// offset 0x1064
	volatile USB_ENDPOINT_READY				usb_endpoint_ready;				// offset 0x1068
	volatile USB_F_IMMEDIATE_INT			usb_f_immediate_int;			// offset 0x106C
	volatile USB_F_ENDPOINT_DONE_STATUS		usb_f_endpoint_done_status;		// offset 0x1070
	volatile USB_ENDPOINT_DONE_ENA			usb_endpoint_done_ena;			// offset 0x1074
	volatile USB_ENDPOINT_TOGGLE_BITS		usb_endpoint_toggle_bits;		// offset 0x1078
	union
	{
		volatile USB_F_FRAME_NUMBER			usb_f_frame_number;				// offset 0x107C	Read only
		volatile unsigned long				usb_endpoint_ready_clr;			// offset 0x107C	Write only
	};

	volatile USB_HOST_CTL					usb_host_ctl;					// offset 0x1080
	volatile unsigned long					reversed_0x1084;				// offset 0x1084
	volatile USB_H_SYSTEM_INT_STATUS		usb_h_system_int_status;		// offset 0x1088
	volatile USB_H_SYSTEM_INT_ENA			usb_h_system_int_ena;			// offset 0x108C
	volatile unsigned long					reversed_0x1090[2];				// offset 0x1090-0x1094
	volatile unsigned long					usb_h_x_buffer_int_status;		// offset 0x1098
	volatile unsigned long					usb_h_y_buffer_int_status;		// offset 0x109C
	volatile unsigned long					usb_h_xy_int_ena;				// offset 0x10A0
	volatile unsigned long					reversed_0x10A4;				// offset 0x10A4
	volatile unsigned long					usb_h_x_filled_status;			// offset 0x10A8
	volatile unsigned long					usb_h_y_filled_status;			// offset 0x10AC
	volatile unsigned long					reversed_0x10B0[4];				// offset 0x10B0-0x10BC
	volatile unsigned long					usb_etd_ena;					// offset 0x10C0
	volatile unsigned long					usb_etd_ena_clear;				// offset 0x10C4
	volatile unsigned long					reversed_0x10C8;				// offset 0x10C8
	volatile unsigned long					usb_h_immediate_int;			// offset 0x10CC
	volatile unsigned long					usb_h_etd_done_status;			// offset 0x10D0
	volatile unsigned long					usb_etd_done_ena;				// offset 0x10D4
	volatile unsigned long					reversed_0x10D8[2];				// offset 0x10D8-0x10DC
	volatile unsigned long					usb_h_frame_number;				// offset 0x10E0
	volatile unsigned long					usb_low_speed_thresh;			// offset 0x10E4
	volatile USB_ROOT_HUB_DESCRIPTOR_A		usb_root_hub_descriptior_a;		// offset 0x10E8
	volatile USB_ROOT_HUB_DESCRIPTOR_B		usb_root_hub_descriptior_b;		// offset 0x10EC
	volatile USB_ROOT_HUB_STATUS			usb_root_hub_status;			// offset 0x10F0
	volatile USB_PORT_STATUS				usb_port_status[3];				// offset 0x10F4	USB_PORT_STATUS_n

	volatile unsigned long					reversed_0x1100[448];			// offset 0x1100-0x17FC

	volatile USB_DMA_REVISION				usb_dma_revision;				// offset 0x1800
	volatile USB_DMA_INT_STATUS				usb_dma_int_status;				// offset 0x1804
	volatile USB_DMA_INT_ENA				usb_dma_int_ena;				// offset 0x1808
	volatile unsigned long					usb_etd_dma_error_status;		// offset 0x180C
	volatile unsigned long					usb_ep_dma_error_status;		// offset 0x1810
	volatile unsigned long					reversed_0x1814[3];				// offset 0x1814-0x181C
	volatile unsigned long					usb_etd_dma_ena;				// offset 0x1820
	volatile unsigned long					usb_ep_dma_ena;					// offset 0x1824
	volatile unsigned long					usb_etd_dma_ena_x_triger_req;	// offset 0x1828
	volatile unsigned long					usb_ep_dma_ena_x_triger_req;	// offset 0x182C
	volatile unsigned long					usb_etd_dma_ena_xy_triger_req;	// offset 0x1830
	volatile unsigned long					usb_ep_dma_ena_xy_triger_req;	// offset 0x1834
	volatile unsigned long					usb_etd_dma_burst4_ena;			// offset 0x1838
	volatile unsigned long					usb_ep_dma_burst4_ena;			// offset 0x183C
	volatile USB_MISC_CTL					usb_misc_ctl;					// offset 0x1840
	volatile unsigned long					reversed_0x1844;				// offset 0x1844
	volatile unsigned long					usb_etd_dma_channel_clear;		// offset 0x1848
	volatile unsigned long					usb_ep_dma_channel_clear;		// offset 0x184C

	volatile unsigned long					reversed_0x1850[44];			// offset 0x1850-0x18FC

	volatile unsigned long					usb_etd_sys_mem_start_addr[32];	// offset 0x1900-0x197C		USB_ETDn_SYS_MEM_START_ADDR
	volatile unsigned long					usb_ep_sys_mem_start_addr[32];	// offset 0x1980-0x19FC		USB_EPn_SYS_MEM_START_ADDR
	volatile unsigned long					usb_etd_dma_buffer_xfer_ptr[32];// offset 0x1A00-0x1A7C		USB_ETDn_DMA_BUFFER_XFER_PTR
	volatile unsigned long					usb_ep_dma_buffer_xfer_ptr[32];	// offset 0x1A80-0x1AFC		USB_EPn_DMA_BUFFER_XFER_PTR
}
USB_REGS, *PUSB_REGS;


/////////////////////////////////////////////////////////////////////
// CODEC SSBI Registers
/////////////////////////////////////////////////////////////////////

typedef struct _tagCODEC_SSBI_CTL
{
	unsigned long	TBD			: 32;
}
CODEC_SSBI_CTL;


typedef struct _tagCODEC_SSBI_RESET
{
	unsigned long	TBD			: 32;
}
CODEC_SSBI_RESET;


typedef struct _tagCODEC_SSBI_CMD
{
	unsigned long	TBD			: 32;
}
CODEC_SSBI_CMD;


typedef struct _tagCODEC_SSBI_BYPASS
{
	unsigned long	TBD			: 32;
}
CODEC_SSBI_BYPASS;


typedef struct _tagCODEC_SSBI_RD
{
	unsigned long	TBD			: 32;
}
CODEC_SSBI_RD;


typedef struct _tagCODEC_SSBI_STATUS
{
	unsigned long	TBD			: 32;
}
CODEC_SSBI_STATUS;


typedef struct _tagCODEC_SSBI_PRIORITIES
{
	unsigned long	priority0	: 3;
	unsigned long	priority1	: 3;
	unsigned long	priority2	: 3;
	unsigned long	reserved	: 23;
}
CODEC_SSBI_PRIORITIES;


typedef struct _tagCODECSSBI_RGES
{
	volatile CODEC_SSBI_CTL			codec_ssbi_ctl;			// offset 0x00		Read/Write
	volatile CODEC_SSBI_RESET		codec_ssbi_reset;		// offset 0x04		Write
	volatile CODEC_SSBI_CMD			codec_ssbi_cmd;			// offset 0x08		Write
	volatile CODEC_SSBI_BYPASS		codec_ssbi_bypass;		// offset 0x0C		Read/Write
	volatile CODEC_SSBI_RD			codec_ssbi_rd;			// offset 0x10		Read
	volatile CODEC_SSBI_STATUS		codec_ssbi_status;		// offset 0x14		Read
	volatile CODEC_SSBI_PRIORITIES	codec_ssbi_priorities;	// offset 0x18		Read/Write
}
CODECSSBI_REGS, *PCODECSSBI_REGS;


/////////////////////////////////////////////////////////////////////
// TSSC and Misc Registers
/////////////////////////////////////////////////////////////////////

typedef struct _tagTSSC_CTL
{
	unsigned long	enable			: 1;
	unsigned long	command_wr		: 1;
	unsigned long	tssc_sw_reset	: 1;
	unsigned long	mode			: 2;
	unsigned long	en_average		: 1;
	unsigned long	debounce_en		: 1;
	unsigned long	debounce_time	: 3;
	unsigned long	intr_flag1		: 1;
	unsigned long	data_flag		: 1;
	unsigned long	intr_flag2		: 1;
	unsigned long	reserved31_13	: 19;
}
TSSC_CTL;


typedef struct _tagTSSC_OPN
{
	unsigned long	resolution1		: 2;
	unsigned long	resolution2		: 2;
	unsigned long	resolution3		: 2;
	unsigned long	resolution4		: 2;
	unsigned long	num_sample1		: 2;
	unsigned long	num_sample2		: 2;
	unsigned long	num_sample3		: 2;
	unsigned long	num_sample4		: 2;
	unsigned long	operation1		: 4;
	unsigned long	operation2		: 4;
	unsigned long	operation3		: 4;
	unsigned long	operation4		: 4;
}
TSSC_OPN;


typedef struct _tagTSSC_SAMPLING_INT
{
	unsigned long	sampling_int	: 5;
	unsigned long	reserved31_5	: 27;
}
TSSC_SAMPLING_INT;


typedef struct _tagTSSC_STATUS
{
	unsigned long	error				: 1;
	unsigned long	samples_collected	: 5;
	unsigned long	operation			: 3;
	unsigned long	penirq_status		: 1;
	unsigned long	adc_eoc_status		: 1;
	unsigned long	error_code			: 2;
	unsigned long	busy				: 1;
	unsigned long	tssc_fsm_state		: 4;
	unsigned long	tssc_ssbi_fsm_state	: 3;
	unsigned long	reserved31_21		: 11;
}
TSSC_STATUS;


typedef struct _tagTSSC_AVG_12
{
	unsigned long	samples_avg_1	: 16;
	unsigned long	samples_avg_2	: 16;
}
TSSC_AVG_12;


typedef struct _tagTSSC_AVG_34
{
	unsigned long	samples_avg_3	: 16;
	unsigned long	samples_avg_4	: 16;
}
TSSC_AVG_34;


typedef struct _tagTSSC_SAMPLE_1_1
{
	unsigned long	raw_sample_1	: 16;
	unsigned long	raw_sample_2	: 16;
}
TSSC_SAMPLE_1_1;


typedef struct _tagTSSC_SAMPLE_1_2
{
	unsigned long	raw_sample_3	: 16;
	unsigned long	raw_sample_4	: 16;
}
TSSC_SAMPLE_1_2;


typedef struct _tagTSSC_SAMPLE_1_3
{
	unsigned long	raw_sample_5	: 16;
	unsigned long	raw_sample_6	: 16;
}
TSSC_SAMPLE_1_3;


typedef struct _tagTSSC_SAMPLE_1_4
{
	unsigned long	raw_sample_7	: 16;
	unsigned long	raw_sample_8	: 16;
}
TSSC_SAMPLE_1_4;


typedef struct _tagTSSC_SAMPLE_1_5
{
	unsigned long	raw_sample_9	: 16;
	unsigned long	raw_sample_10	: 16;
}
TSSC_SAMPLE_1_5;


typedef struct _tagTSSC_SAMPLE_1_6
{
	unsigned long	raw_sample_11	: 16;
	unsigned long	raw_sample_12	: 16;
}
TSSC_SAMPLE_1_6;


typedef struct _tagTSSC_SAMPLE_1_7
{
	unsigned long	raw_sample_13	: 16;
	unsigned long	raw_sample_14	: 16;
}
TSSC_SAMPLE_1_7;


typedef struct _tagTSSC_SAMPLE_1_8
{
	unsigned long	raw_sample_15	: 16;
	unsigned long	raw_sample_16	: 16;
}
TSSC_SAMPLE_1_8;


typedef struct _tagTSSC_SAMPLE_2_1
{
	unsigned long	raw_sample_1	: 16;
	unsigned long	raw_sample_2	: 16;
}
TSSC_SAMPLE_2_1;


typedef struct _tagTSSC_SAMPLE_2_2
{
	unsigned long	raw_sample_3	: 16;
	unsigned long	raw_sample_4	: 16;
}
TSSC_SAMPLE_2_2;


typedef struct _tagTSSC_SAMPLE_2_3
{
	unsigned long	raw_sample_5	: 16;
	unsigned long	raw_sample_6	: 16;
}
TSSC_SAMPLE_2_3;


typedef struct _tagTSSC_SAMPLE_2_4
{
	unsigned long	raw_sample_7	: 16;
	unsigned long	raw_sample_8	: 16;
}
TSSC_SAMPLE_2_4;


typedef struct _tagTSSC_SAMPLE_2_5
{
	unsigned long	raw_sample_9	: 16;
	unsigned long	raw_sample_10	: 16;
}
TSSC_SAMPLE_2_5;


typedef struct _tagTSSC_SAMPLE_2_6
{
	unsigned long	raw_sample_11	: 16;
	unsigned long	raw_sample_12	: 16;
}
TSSC_SAMPLE_2_6;


typedef struct _tagTSSC_SAMPLE_2_7
{
	unsigned long	raw_sample_13	: 16;
	unsigned long	raw_sample_14	: 16;
}
TSSC_SAMPLE_2_7;


typedef struct _tagTSSC_SAMPLE_2_8
{
	unsigned long	raw_sample_15	: 16;
	unsigned long	raw_sample_16	: 16;
}
TSSC_SAMPLE_2_8;


typedef struct _tagTSSC_SAMPLE_3_1
{
	unsigned long	raw_sample_1	: 16;
	unsigned long	raw_sample_2	: 16;
}
TSSC_SAMPLE_3_1;


typedef struct _tagTSSC_SAMPLE_3_2
{
	unsigned long	raw_sample_3	: 16;
	unsigned long	raw_sample_4	: 16;
}
TSSC_SAMPLE_3_2;


typedef struct _tagTSSC_SAMPLE_3_3
{
	unsigned long	raw_sample_5	: 16;
	unsigned long	raw_sample_6	: 16;
}
TSSC_SAMPLE_3_3;


typedef struct _tagTSSC_SAMPLE_3_4
{
	unsigned long	raw_sample_7	: 16;
	unsigned long	raw_sample_8	: 16;
}
TSSC_SAMPLE_3_4;


typedef struct _tagTSSC_SAMPLE_3_5
{
	unsigned long	raw_sample_9	: 16;
	unsigned long	raw_sample_10	: 16;
}
TSSC_SAMPLE_3_5;


typedef struct _tagTSSC_SAMPLE_3_6
{
	unsigned long	raw_sample_11	: 16;
	unsigned long	raw_sample_12	: 16;
}
TSSC_SAMPLE_3_6;


typedef struct _tagTSSC_SAMPLE_3_7
{
	unsigned long	raw_sample_13	: 16;
	unsigned long	raw_sample_14	: 16;
}
TSSC_SAMPLE_3_7;


typedef struct _tagTSSC_SAMPLE_3_8
{
	unsigned long	raw_sample_15	: 16;
	unsigned long	raw_sample_16	: 16;
}
TSSC_SAMPLE_3_8;


typedef struct _tagTSSC_SAMPLE_4_1
{
	unsigned long	raw_sample_1	: 16;
	unsigned long	raw_sample_2	: 16;
}
TSSC_SAMPLE_4_1;


typedef struct _tagTSSC_SAMPLE_4_2
{
	unsigned long	raw_sample_3	: 16;
	unsigned long	raw_sample_4	: 16;
}
TSSC_SAMPLE_4_2;


typedef struct _tagTSSC_SAMPLE_4_3
{
	unsigned long	raw_sample_5	: 16;
	unsigned long	raw_sample_6	: 16;
}
TSSC_SAMPLE_4_3;


typedef struct _tagTSSC_SAMPLE_4_4
{
	unsigned long	raw_sample_7	: 16;
	unsigned long	raw_sample_8	: 16;
}
TSSC_SAMPLE_4_4;


typedef struct _tagTSSC_SAMPLE_4_5
{
	unsigned long	raw_sample_9	: 16;
	unsigned long	raw_sample_10	: 16;
}
TSSC_SAMPLE_4_5;


typedef struct _tagTSSC_SAMPLE_4_6
{
	unsigned long	raw_sample_11	: 16;
	unsigned long	raw_sample_12	: 16;
}
TSSC_SAMPLE_4_6;


typedef struct _tagTSSC_SAMPLE_4_7
{
	unsigned long	raw_sample_13	: 16;
	unsigned long	raw_sample_14	: 16;
}
TSSC_SAMPLE_4_7;


typedef struct _tagTSSC_SAMPLE_4_8
{
	unsigned long	raw_sample_15	: 16;
	unsigned long	raw_sample_16	: 16;
}
TSSC_SAMPLE_4_8;


typedef struct _tagTSSC_TEST_1
{
	unsigned long	test_penirq_n		: 1;
	unsigned long	test_mode			: 1;
	unsigned long	gate_debounce_en	: 1;
	unsigned long	reserved_bits23_3	: 21;
	unsigned long	test_ssbi_rd_data	: 8;
}
TSSC_TEST_1;


typedef struct _tagTSSC_TEST_2
{
	unsigned long	reserved_bits31_0	: 32;
}
TSSC_TEST_2;


typedef struct _tagWEB_MISC_WR_APPS
{
	unsigned long	web_misc_wr_a0	: 1;
	unsigned long	web_misc_wr_a1	: 1;
	unsigned long	web_misc_wr_a2	: 1;
	unsigned long	web_misc_wr_a3	: 1;
	unsigned long	web_misc_wr_a4	: 1;
	unsigned long	web_misc_wr_a5	: 1;
	unsigned long	web_misc_wr_a6	: 1;
	unsigned long	web_misc_wr_a7	: 1;
	unsigned long	reserved		: 24;
}
WEB_MISC_WR_APPS;


typedef struct _tagWEB_MISC_RD_APPS
{
	unsigned long	web_misc_rd_a0	: 1;
	unsigned long	web_misc_rd_a1	: 1;
	unsigned long	web_misc_rd_a2	: 1;
	unsigned long	web_misc_rd_a3	: 1;
	unsigned long	web_misc_rd_a4	: 1;
	unsigned long	web_misc_rd_a5	: 1;
	unsigned long	web_misc_rd_a6	: 1;
	unsigned long	web_misc_rd_a7	: 1;
	unsigned long	reserved		: 24;
}
WEB_MISC_RD_APPS;


/////////////////////////////////////////////////////////////////////

typedef struct _tagTSSC_RGES
{
	volatile TSSC_CTL			tssc_ctl;					// offset 0x00		Read/Write
	volatile TSSC_OPN			tssc_opn;					// offset 0x04		Read/Write
	volatile TSSC_SAMPLING_INT	tssc_sampling_int;			// offset 0x08		Read/Write
	volatile TSSC_STATUS		tssc_status;				// offset 0x0C		Read only
	volatile TSSC_AVG_12		tssc_avg_12;				// offset 0x10		Read only
	volatile TSSC_AVG_34		tssc_avg_34;				// offset 0x14		Read only
	volatile TSSC_SAMPLE_1_1	tssc_sample_1_1;			// offset 0x18		Read only
	volatile TSSC_SAMPLE_1_2	tssc_sample_1_2;			// offset 0x1C		Read only
	volatile TSSC_SAMPLE_1_3	tssc_sample_1_3;			// offset 0x20		Read only
	volatile TSSC_SAMPLE_1_4	tssc_sample_1_4;			// offset 0x24		Read only
	volatile TSSC_SAMPLE_1_5	tssc_sample_1_5;			// offset 0x28		Read only
	volatile TSSC_SAMPLE_1_6	tssc_sample_1_6;			// offset 0x2C		Read only
	volatile TSSC_SAMPLE_1_7	tssc_sample_1_7;			// offset 0x30		Read only
	volatile TSSC_SAMPLE_1_8	tssc_sample_1_8;			// offset 0x34		Read only
	volatile TSSC_SAMPLE_2_1	tssc_sample_2_1;			// offset 0x38		Read only
	volatile TSSC_SAMPLE_2_2	tssc_sample_2_2;			// offset 0x3C		Read only
	volatile TSSC_SAMPLE_2_3	tssc_sample_2_3;			// offset 0x40		Read only
	volatile TSSC_SAMPLE_2_4	tssc_sample_2_4;			// offset 0x44		Read only
	volatile TSSC_SAMPLE_2_5	tssc_sample_2_5;			// offset 0x48		Read only
	volatile TSSC_SAMPLE_2_6	tssc_sample_2_6;			// offset 0x4C		Read only
	volatile TSSC_SAMPLE_2_7	tssc_sample_2_7;			// offset 0x50		Read only
	volatile TSSC_SAMPLE_2_8	tssc_sample_2_8;			// offset 0x54		Read only
	volatile TSSC_SAMPLE_3_1	tssc_sample_3_1;			// offset 0x58		Read only
	volatile TSSC_SAMPLE_3_2	tssc_sample_3_2;			// offset 0x5C		Read only
	volatile TSSC_SAMPLE_3_3	tssc_sample_3_3;			// offset 0x60		Read only
	volatile TSSC_SAMPLE_3_4	tssc_sample_3_4;			// offset 0x64		Read only
	volatile TSSC_SAMPLE_3_5	tssc_sample_3_5;			// offset 0x68		Read only
	volatile TSSC_SAMPLE_3_6	tssc_sample_3_6;			// offset 0x6C		Read only
	volatile TSSC_SAMPLE_3_7	tssc_sample_3_7;			// offset 0x70		Read only
	volatile TSSC_SAMPLE_3_8	tssc_sample_3_8;			// offset 0x74		Read only
	volatile TSSC_SAMPLE_4_1	tssc_sample_4_1;			// offset 0x78		Read only
	volatile TSSC_SAMPLE_4_2	tssc_sample_4_2;			// offset 0x7C		Read only
	volatile TSSC_SAMPLE_4_3	tssc_sample_4_3;			// offset 0x80		Read only
	volatile TSSC_SAMPLE_4_4	tssc_sample_4_4;			// offset 0x84		Read only
	volatile TSSC_SAMPLE_4_5	tssc_sample_4_5;			// offset 0x88		Read only
	volatile TSSC_SAMPLE_4_6	tssc_sample_4_6;			// offset 0x8C		Read only
	volatile TSSC_SAMPLE_4_7	tssc_sample_4_7;			// offset 0x90		Read only
	volatile TSSC_SAMPLE_4_8	tssc_sample_4_8;			// offset 0x94		Read only
	volatile TSSC_TEST_1		tssc_test_1;				// offset 0x98		Read/Write
	volatile TSSC_TEST_2		tssc_test_2;				// offset 0x9C		Write/Command
	volatile WEB_MISC_WR_APPS	web_misc_wr_apps;			// offset 0xA0		Read/Write
	volatile WEB_MISC_RD_APPS	web_misc_rd_apps;			// offset 0xA4		Read only
}
TSSC_REGS, *PTSSC_REGS;


/////////////////////////////////////////////////////////////////////
// MMDI_CAMIF Registers
/////////////////////////////////////////////////////////////////////

typedef struct _tagMDDI_CAMIF_CFG
{
	unsigned long mtv_8b_10b_data			: 1;
	unsigned long cam_sel					: 1;
	unsigned long rx_intr_ena				: 1;
	unsigned long tx_intr_ena				: 1;
	unsigned long wake_up_intr_ena			: 1;
	unsigned long vbuf_err_intr_ena			: 1;
	unsigned long overflow_intr_ena			: 1;
	unsigned long mddi_clk_chicken_bit		: 1;
	unsigned long ext_cam_data_edge_sel		: 1;
	unsigned long ext_cam_vsync_edge_sel	: 1;
	unsigned long ext_cam_hsync_edge_sel	: 1;
	unsigned long mtv_8b_12b_data			: 1;
	unsigned long sw_byte_clk_en			: 1;
	unsigned long byte_bound_intr_ena		: 1;
	unsigned long crc_err_intr_ena			: 1;
	unsigned long ext_cam_vsync_pol_sel		: 1;
	unsigned long ext_cam_hsync_pol_sel		: 1;
}
MDDI_CAMIF_CFG;


typedef struct _tagMDDI_CAMIF_INTR_STATUS
{
	unsigned long rx_intr_status			: 1;
	unsigned long tx_intr_status			: 1;
	unsigned long wake_up_intr_status		: 1;
	unsigned long vbuf_err_intr_status		: 1;
	unsigned long overflow_status			: 1;
	unsigned long byte_bound_intr_status	: 1;
	unsigned long crc_err_intr_status		: 1;
}
MDDI_CAMIF_INTR_STATUS;


typedef struct _tagMDDI_CAMIF_INTR_CLEAR
{
	unsigned long rx_intr_clear				: 1;
	unsigned long tx_intr_clear				: 1;
	unsigned long wake_up_intr_clear		: 1;
	unsigned long vbuf_err_intr_clear		: 1;
	unsigned long host_reg_ovf_clr			: 1;
	unsigned long byte_bound_intr_clr		: 1;
	unsigned long crc_err_intr_clr			: 1;
}
MDDI_CAMIF_INTR_CLEAR;


typedef struct _tagMDDI_CAMIF_CORE_RESET
{
	unsigned long core_reset				: 1;
}
MDDI_CAMIF_CORE_RESET;


typedef struct _tagMDDI_CLIENT_PAD_CNTL
{
	unsigned long data0_drv_en				: 1;
	unsigned long data0_std_rcv_en			: 1;
	unsigned long data0_hib_rcv_en			: 1;
	unsigned long stb_rcv_en				: 1;
	unsigned long stb_rcv_en_offset			: 1;
	unsigned long data0_gate				: 1;
	unsigned long data0_drv_ctl				: 2;
	unsigned long data0_mux_ctl				: 2;
	unsigned long spare_ctl					: 3;
	unsigned long hib_rcv_mux_ctl			: 2;
	unsigned long nibl_mux_data0_ctl		: 2;
	unsigned long data0_drv_in				: 1;
	unsigned long bandgap_en				: 1;
	unsigned long data1_std_rcv_en			: 1;
	unsigned long nib_mux_data1_ctl			: 2;
	unsigned long pdn_tempsense				: 1;
	unsigned long amux_en					: 1;
	unsigned long client_core_test_pad		: 1;
}
MDDI_CLIENT_PAD_CNTL;


typedef struct _tagMDDI_CLIENT_PAD_DELAY
{
	unsigned long data0_delay				: 8;
	unsigned long data1_delay				: 8;
	unsigned long strobe_delay				: 8;
	unsigned long data1_delay_offset		: 8;
}
MDDI_CLIENT_PAD_DELAY;


/////////////////////////////////////////////////////////////////////

typedef struct _tagMDDI_CAMIF_REGS
{
	volatile MDDI_CAMIF_CFG			mddi_camif_cfg;			// offset 0x00		Read/Write
	volatile MDDI_CAMIF_INTR_STATUS	mddi_camif_intr_status;	// offset 0x04		Read only
	volatile MDDI_CAMIF_INTR_CLEAR	mddi_camif_intr_clear;	// offset 0x08		Read/Write
	volatile MDDI_CAMIF_CORE_RESET	mddi_camif_core_reset;	// offset 0x0C		Write only
	volatile unsigned long			not_defined_yet_10;		// offset 0x10
	volatile unsigned long			not_defined_yet_14;		// offset 0x14
	volatile unsigned long			not_defined_yet_18;		// offset 0x18
	volatile unsigned long			not_defined_yet_1C;		// offset 0x1C
	volatile unsigned long			not_defined_yet_20;		// offset 0x20
	volatile unsigned long			not_defined_yet_24;		// offset 0x24
	volatile unsigned long			not_defined_yet_28;		// offset 0x28
	volatile unsigned long			not_defined_yet_2C;		// offset 0x2C
	volatile unsigned long			not_defined_yet_30;		// offset 0x30
	volatile unsigned long			not_defined_yet_34;		// offset 0x34
	volatile unsigned long			not_defined_yet_38;		// offset 0x38
	volatile unsigned long			not_defined_yet_3C;		// offset 0x3C
	volatile unsigned long			not_defined_yet_40;		// offset 0x40
	volatile unsigned long			not_defined_yet_44;		// offset 0x44
	volatile MDDI_CLIENT_PAD_CNTL	mddi_client_pad_cntl;	// offset 0x48		Read/Write
	volatile MDDI_CLIENT_PAD_DELAY	mddi_client_pad_delay;	// offset 0x4C		Read/Write
	volatile unsigned long			reversed_0x50_5C[4];	// offset 0x50-0x5C
	volatile unsigned long			not_defined_yet_60;		// offset 0x60
	volatile unsigned long			not_defined_yet_64;		// offset 0x64
	volatile unsigned long			not_defined_yet_68;		// offset 0x68
	volatile unsigned long			not_defined_yet_6C;		// offset 0x6C
	volatile unsigned long			not_defined_yet_70;		// offset 0x70
	volatile unsigned long			not_defined_yet_74;		// offset 0x74
	volatile unsigned long			MDDI_REV_REG_START_ADDR;// offset 0x78		Read/Write
	volatile unsigned long			MDDI_REV_REG_DATA1;		// offset 0x7C		Read/Write
	volatile unsigned long			MDDI_REV_REG_DATA2;		// offset 0x80		Read/Write
	volatile unsigned long			MDDI_REV_REG_DATA3;		// offset 0x84		Read/Write
	volatile unsigned long			not_defined_yet_88;		// offset 0x88
	volatile unsigned long			not_defined_yet_8C;		// offset 0x8C
	volatile unsigned long			not_defined_yet_90;		// offset 0x90
	volatile unsigned long			reversed_0x94_FC[27];	// offset 0x94-0xFC

	volatile unsigned long			MDDI_START_OF_FWD_REG_BUF;	// offset 0x100		Read/Write
	volatile unsigned long			reversed_0x104_178[30];		// offset 0x104-0x178
	volatile unsigned long			MDDI_END_OF_FWD_REG_BUF;	// offset 0x17C		Read/Write
	volatile unsigned long			not_defined_yet_180;		// offset 0x180
	volatile unsigned long			not_defined_yet_184;		// offset 0x184
	volatile unsigned long			not_defined_yet_188;		// offset 0x188
	volatile unsigned long			not_defined_yet_18C;		// offset 0x18C
	volatile unsigned long			not_defined_yet_190;		// offset 0x190
}
MDDI_CAMIF_REGS, *PMDDI_CAMIF_REGS;


/////////////////////////////////////////////////////////////////////
// MMDI_CAMIF Registers
/////////////////////////////////////////////////////////////////////

typedef struct _tagMDDI_CMD_REG
{
	unsigned long command_params			: 8;
	unsigned long command					: 8;
}
MDDI_CMD_REG;


typedef struct _tagMDDI_VERSION_REG
{
	unsigned long version					: 16;
}
MDDI_VERSION_REG;


typedef struct _tagMDDI_PAD_CTL_REG
{
	unsigned long mddi_dout_ctl				: 2;
	unsigned long mddi_stb_ctl				: 2;
	unsigned long mddi_spare_ctl			: 3;
	unsigned long rcv_ena					: 1;
	unsigned long hw_ctl_rcv_ena			: 1;
	unsigned long offset_rcv_ena			: 1;
	unsigned long hw_ctl_offset_rcv_e		: 1;
	unsigned long stb_drv_input_mux			: 2;
	unsigned long data_drv_input_mux		: 2;
	unsigned long bandgap_ena				: 1;
	unsigned long stb_driver_ena			: 1;
	unsigned long stb_driver_hw_ena			: 1;
	unsigned long data_driver_ena			: 1;
	unsigned long data_driver_hw_ena		: 1;
	unsigned long spare_status				: 3;
}
MDDI_PAD_CTL_REG;

 
/////////////////////////////////////////////////////////////////////

typedef struct _tagMDDIH_REGS
{
	volatile MDDI_CMD_REG			mddi_cmd_reg;			// offset 0x00		Write only
	volatile MDDI_VERSION_REG		mddi_version_reg;		// offset 0x04		Read/Write
	volatile unsigned long			mddi_pri_ptr_reg;		// offset 0x08		Read/Write
	volatile unsigned long			mddi_sec_ptr_reg;		// offset 0x0C		Read/Write
	volatile unsigned long			mddi_bps_reg;			// offset 0x10		Read/Write
	volatile unsigned long			not_defined_yet_14;		// offset 0x14
	volatile unsigned long			not_defined_yet_18;		// offset 0x18
	volatile unsigned long			not_defined_yet_1C;		// offset 0x1C
	volatile unsigned long			mddi_rev_ptr_reg;		// offset 0x20		Read/Write
	volatile unsigned long			not_defined_yet_24;		// offset 0x24
	volatile unsigned long			not_defined_yet_28;		// offset 0x28
	volatile unsigned long			not_defined_yet_2C;		// offset 0x2C
	volatile unsigned long			not_defined_yet_30;		// offset 0x30
	volatile unsigned long			not_defined_yet_34;		// offset 0x34
	volatile unsigned long			not_defined_yet_38;		// offset 0x38
	volatile unsigned long			not_defined_yet_3C;		// offset 0x3C
	volatile unsigned long			not_defined_yet_40;		// offset 0x40
	volatile unsigned long			not_defined_yet_44;		// offset 0x44
	volatile unsigned long			not_defined_yet_48;		// offset 0x48
	volatile unsigned long			not_defined_yet_4C;		// offset 0x4C
	volatile unsigned long			not_defined_yet_50;		// offset 0x50
	volatile unsigned long			not_defined_yet_54;		// offset 0x54
	volatile unsigned long			not_defined_yet_58;		// offset 0x58
	volatile unsigned long			reversed_0x5C_64[3];	// offset 0x5C-0x64
	volatile MDDI_PAD_CTL_REG		mddi_pad_ctl_reg;		// offset 0x68		Read/Write
	volatile unsigned long			not_defined_yet_6C;		// offset 0x6C
	volatile unsigned long			mddi_next_pri_ptr_reg;	// offset 0x70		Read/Write
	volatile unsigned long			mddi_next_sec_ptr_reg;	// offset 0x74		Read/Write
	volatile unsigned long			not_defined_yet_78;		// offset 0x78
	volatile unsigned long			not_defined_yet_7C;		// offset 0x7C
	volatile unsigned long			mddi_mf_cnt_reg;		// offset 0x80		Read/Write
	volatile unsigned long			mddi_curr_rev_ptr_reg;	// offset 0x84		Read/Write
	volatile unsigned long			not_defined_yet_88;		// offset 0x88
	volatile unsigned long			not_defined_yet_8C;		// offset 0x8C
}
MDDIH_REGS, *PMDDIH_REGS;

typedef struct _tagAPWBMIS_REGS
{
	unsigned long web_misc_wr_modem;		//offset: 0x0000
	unsigned long web_misc_rd_modem;		//offset: 0x0004
	unsigned long undefined;			//offset: 0x0008
	unsigned long tcxo_pdm_ctl;			//offset: 0x000C
	unsigned long pdm0_ctl;				//offset: 0x0010
	unsigned long pdm1_ctl;				//offset: 0x0014
	unsigned long gp_mn_clk_mdiv;			//offset: 0x0018
	unsigned long gp_mn_clk_ndiv;			//offset: 0x001C
	unsigned long gp_mn_clk_duty;			//offset: 0x0020
}
APWBMIS_REGS, *PAPWBMIS_REGS;

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
#define	LINK_TYPE	extern "C"
#else
#ifdef FIRST_ENTRY
#define LINK_TYPE
#else
#define LINK_TYPE	extern
#endif
#endif


/////////////////////////////////////////////////////////////////////

#ifdef FIRST_ENTRY

#ifdef KERNEL_MODE
LINK_TYPE PNAND_REGS pNandRegs = (PNAND_REGS) NAND_REGS_VIRTUAL_BASE;

LINK_TYPE PUART_REGS pUart1Regs = (PUART_REGS) UART1_REGS_VIRTUAL_BASE;
LINK_TYPE PUART_REGS pUart2Regs = (PUART_REGS) UART2_REGS_VIRTUAL_BASE;
LINK_TYPE PUART_REGS pUart3Regs = (PUART_REGS) UART3_REGS_VIRTUAL_BASE;
LINK_TYPE PUART_DM_REGS pUart1DMRegs = (PUART_DM_REGS) UART1DM_REGS_VIRTUAL_BASE;
LINK_TYPE PUART_DM_REGS pUart2DMRegs = (PUART_DM_REGS) UART2DM_REGS_VIRTUAL_BASE;

LINK_TYPE PUSB_REGS pUsbRegs = (PUSB_REGS) USB_REGS_VIRTUAL_BASE;
LINK_TYPE PCODECSSBI_REGS pCodecSsbiRegs = (PCODECSSBI_REGS) CODECSSBI_REGS_VIRTUAL_BASE;
LINK_TYPE PTSSC_REGS pTsscRegs = (PTSSC_REGS) TSSC_REGS_VIRTUAL_BASE;

LINK_TYPE PMDDI_CAMIF_REGS pMddiCamifRegs = (PMDDI_CAMIF_REGS) MDC_BASE_VIRT;
LINK_TYPE PMDDIH_REGS pPmdhRegs = (PMDDIH_REGS) PMDH_REGS_VIRTUAL_BASE;
LINK_TYPE PMDDIH_REGS pEmdhRegs = (PMDDIH_REGS) EMDH_REGS_VIRTUAL_BASE;
LINK_TYPE PAPWBMIS_REGS pApwbMisRegs = (PAPWBMIS_REGS) APWBMIS_REGS_VIRTUAL_BASE;
#else
LINK_TYPE PNAND_REGS pNandRegs = NULL;

LINK_TYPE PUART_REGS pUart1Regs = NULL;
LINK_TYPE PUART_REGS pUart2Regs = NULL;
LINK_TYPE PUART_REGS pUart3Regs = NULL;
LINK_TYPE PUART_DM_REGS pUart1DMRegs = NULL;
LINK_TYPE PUART_DM_REGS pUart2DMRegs = NULL;

LINK_TYPE PUSB_REGS pUsbRegs = NULL;
LINK_TYPE PCODECSSBI_REGS pCodecSsbiRegs = NULL;
LINK_TYPE PTSSC_REGS pTsscRegs = NULL;

LINK_TYPE PMDDI_CAMIF_REGS pMddiCamifRegs = NULL;
LINK_TYPE PMDDIH_REGS pPmdhRegs = NULL;
LINK_TYPE PMDDIH_REGS pEmdhRegs = NULL;
LINK_TYPE PAPWBMIS_REGS pApwbMisRegs = NULL;
#endif

#else // not first entry

LINK_TYPE PNAND_REGS pNandRegs;

LINK_TYPE PUART_REGS pUart1Regs;
LINK_TYPE PUART_REGS pUart2Regs;
LINK_TYPE PUART_REGS pUart3Regs;
LINK_TYPE PUART_DM_REGS pUart1DMRegs;
LINK_TYPE PUART_DM_REGS pUart2DMRegs;

LINK_TYPE PUSB_REGS pUsbRegs;
LINK_TYPE PCODECSSBI_REGS pCodecSsbiRegs;
LINK_TYPE PTSSC_REGS pTsscRegs;

LINK_TYPE PMDDI_CAMIF_REGS pMddiCamifRegs;
LINK_TYPE PMDDIH_REGS pPmdhRegs;
LINK_TYPE PMDDIH_REGS pEmdhRegs;
LINK_TYPE PAPWBMIS_REGS pApwbMisRegs;

#endif


#endif // end ifndef MSM7500_PERIPHERAL_HEADER
