
#ifndef __MSM7500_CORE_HEADER
#define __MSM7500_CORE_HEADER

#include "MSM7500Base.h"


typedef unsigned long DWORD;


/////////////////////////////////////////////////////////////////////
// EBI1
/////////////////////////////////////////////////////////////////////

typedef struct _tagEBI1_CS_ADEC
{
	unsigned long ebi1_cs0_range	: 1;
}
EBI1_CS_ADEC;


typedef struct _tagEBI1_TEST_BUS_CFG
{
	unsigned long aximemc_dbg_bus_sel		: 5;
	unsigned long sdramcc_diag_bus_sel		: 4;
	unsigned long ebi1_test_bus_sel			: 2;
	unsigned long ebi1_test_bus_en			: 1;
	unsigned long ebi1_bc_test_bus_sel		: 1;
	unsigned long ebi1_test_bus_misr_sel	: 2;
}
EBI1_TEST_BUS_CFG;


typedef struct _tagEBI1_MISR_CTRL
{
	unsigned long ebi1_misr_reset	: 1;
	unsigned long ebi1_misr_en		: 1;
	unsigned long ebi1_misr_ctrl	: 2;
}
EBI1_MISR_CTRL;


typedef struct _tagEBI1_MISR_OUT
{
	unsigned long ebi1_misr_out		: 16;
}
EBI1_MISR_OUT;


typedef struct _tagEBI_SDRAM_INTERFACE_CFG1
{
	unsigned long dev0_sr		: 1;
	unsigned long dev1_sr		: 1;
	unsigned long revesred		: 1;
	unsigned long rfrsh_en		: 1;
	unsigned long rfrsh_tmr		: 12;
}
EBI_SDRAM_INTERFACE_CFG1;


typedef struct _tagEBI_SDRAM_INTERFACE_CFG2
{
	unsigned long burst_en				: 3;
	unsigned long revesred				: 1;
	unsigned long high_temp0_rfsh_tmr	: 12;
}
EBI_SDRAM_INTERFACE_CFG2;


typedef struct _tagEBI_SDRAM_DEVICE_MODE1
{
	unsigned long we_n						: 1;
	unsigned long cas_n						: 1;
	unsigned long ras_n						: 1;
	unsigned long cs_n						: 2;
	unsigned long ba						: 2;
	unsigned long cke						: 2;
	unsigned long dqm						: 4;
	unsigned long cke0_toggle_in_cfg_acc	: 1;
	unsigned long cke1_toggle_in_cfg_acc	: 1;
	unsigned long cke_power_down_en			: 1;
}
EBI_SDRAM_DEVICE_MODE1;


typedef struct _tagEBI_SDRAM_DEVICE_MODE2
{
	unsigned long adr_pins					: 15;
}
EBI_SDRAM_DEVICE_MODE2;


typedef struct _tagEBI_SDRAM_MEM_DEVICE_PARAMETER_CFG1
{
	unsigned long reversed					: 2;
	unsigned long trc						: 4;
	unsigned long tras						: 3;
	unsigned long trp						: 3;
	unsigned long trfc						: 4;
}
EBI_SDRAM_MEM_DEVICE_PARAMETER_CFG1;


typedef struct _tagEBI_SDRAM_MEM_DEVICE_PARAMETER_CFG2
{
	unsigned long trrd						: 3;
	unsigned long tdal						: 3;
	unsigned long trcd						: 3;
	unsigned long twr						: 3;
}
EBI_SDRAM_MEM_DEVICE_PARAMETER_CFG2;


typedef struct _tagEBI_SDRAM_MEM_DEVICE_PARAMETER_CFG3
{
	unsigned long col_size					: 3;
	unsigned long row_size					: 3;
	unsigned long bank_size					: 1;
	unsigned long cas_lat					: 2;
	unsigned long reserved_09_11			: 3;
	unsigned long bank_il					: 1;
}
EBI_SDRAM_MEM_DEVICE_PARAMETER_CFG3;


typedef struct _tagEBI_SDRAM_CONTROLLER_CFG
{
	unsigned long data_bus_width			: 1;
	unsigned long next_cmd_opt_en			: 1;
	unsigned long cntn_clr_en				: 1;
	unsigned long ahb_mode					: 1;
	unsigned long bus_mem_clk_sync			: 1;
	unsigned long cmd_timing_en				: 1;
	unsigned long cmd_opt_en				: 1;
	unsigned long ready_fifo_start_early	: 1;
	unsigned long ready_fifo_out_reg		: 1;
	unsigned long max_pend_rfrsh_cnt		: 2;
	unsigned long clk_pwr_down_en			: 1;
	unsigned long force_ddr_clk_off			: 1;
	unsigned long dq_dqs_pad_oe				: 2;
	unsigned long fifo_clk_sync				: 1;
}
EBI_SDRAM_CONTROLLER_CFG;


typedef struct _tagEBI_SDRAM_DEVICE_CFG
{
	unsigned long config_start			: 1;
}
EBI_SDRAM_DEVICE_CFG;


typedef struct _tagEBI_SDRAM_DEVICE_PRECHARGE
{
	unsigned long precharge_start		: 1;
}
EBI_SDRAM_DEVICE_PRECHARGE;


typedef struct _tagEBI_SDRAM_DEVICE_REFRESH
{
	unsigned long refresh_start			: 1;
}
EBI_SDRAM_DEVICE_REFRESH;


typedef struct _tagEBI_SDRAM_INTERFACE_CFG3
{
	unsigned long reserved_0_3			: 4;
	unsigned long high_temp1_rfsh_tmr	: 12;
}
EBI_SDRAM_INTERFACE_CFG3;


typedef struct _tagEBI_SDRAM_DEVICE_STATUS
{
	unsigned long config_done			: 1;
	unsigned long precharge_done		: 1;
	unsigned long refresh_done			: 1;
}
EBI_SDRAM_DEVICE_STATUS;


typedef struct _tagEBI_SDRAM_CDC_CFG
{
	unsigned long cdcn_bypass_n			: 1;
	unsigned long cdc_internal1_bit6	: 1;
	unsigned long cdc_internal1_bit7	: 1;
	unsigned long cdc_internal1_bit9_8	: 2;
	unsigned long cdc_internal1_bit5	: 1;
	unsigned long cdc_internal1_bit3	: 1;
	unsigned long cdc_internal1_bit4	: 1;
	unsigned long cdcn_atest_1			: 1;
	unsigned long cdcn_atest_1_sel		: 2;
	unsigned long cdc_internal1_bit2	: 1;
	unsigned long cdc_internal1_bit1	: 1;
	unsigned long cdc_internal1_bit0	: 1;
	unsigned long cdcn_freeze_io_n		: 1;
	unsigned long cdc_clmp				: 1;
}
EBI_SDRAM_CDC_CFG;


typedef struct _tagEBI_SDRAM_CDC_MODE0
{
	unsigned long cdc_internal1_bit20_15	: 6;
	unsigned long cdc_internal1_bit14_13	: 2;
	unsigned long cdc_internal1_bit12		: 1;
	unsigned long cdc_internal1_bit11_10	: 2;
	unsigned long cdcn_load_master			: 1;
	unsigned long cdcn_start_cal			: 1;
	unsigned long cdc_internal1_bit23		: 1;
	unsigned long cdc_internal1_bit22_21	: 2;
}
EBI_SDRAM_CDC_MODE0;


typedef struct _tagEBI_SDRAM_CDC_MODE1
{
	unsigned long cdcn_offset_val		: 6;
	unsigned long cdcn_load_offset		: 1;
	unsigned long cdcn_load_slave		: 1;
}
EBI_SDRAM_CDC_MODE1;


typedef struct _tagEBI_SDRAM_CDC0_MODE2
{
	unsigned long autoload_slave_cdcn	: 1;
	unsigned long autocalib_cdc			: 3;
	unsigned long debug_sel				: 4;
	unsigned long cdc_cre_pwr_sel		: 1;
}
EBI_SDRAM_CDC0_MODE2;


typedef struct _tagEBI_SDRAM_CDC_MODE2
{
	unsigned long autoload_slave_cdcn	: 1;
}
EBI_SDRAM_CDC_MODE2;


/////////////////////////////////////////////////////////////////////

typedef struct _tagEBI1_REGS
{
	volatile EBI1_CS_ADEC 							ebi1_cs_adec;							// offset 0x00, Read/Write
	volatile EBI1_TEST_BUS_CFG						ebi1_test_bus_cfg;						// offset 0x04, Read/Write
	volatile EBI1_MISR_CTRL							ebi1_misr_ctrl;							// offset 0x08, Read/Write
	volatile EBI1_MISR_OUT							ebi1_misr_out0;							// offset 0x0C, Read only
	volatile EBI1_MISR_OUT							ebi1_misr_out1;							// offset 0x10, Read only
	volatile EBI1_MISR_OUT							ebi1_misr_out2;							// offset 0x14, Read only
	volatile EBI1_MISR_OUT							ebi1_misr_out3;							// offset 0x18, Read only
	volatile unsigned long							filler01[9];							// offset 0x1C-0x3C
	volatile EBI_SDRAM_INTERFACE_CFG1				ebi_sdram_interface_cfg1;				// offset 0x40, Read/Write
	volatile EBI_SDRAM_INTERFACE_CFG2				ebi_sdram_interface_cfg2;				// offset 0x44, Read/Write
	volatile EBI_SDRAM_DEVICE_MODE1					ebi_sdram_device_mode1;					// offset 0x48, Read/Write
	volatile EBI_SDRAM_DEVICE_MODE2					ebi_sdram_device_mode2;					// offset 0x4C, Read/Write
	volatile EBI_SDRAM_MEM_DEVICE_PARAMETER_CFG1	ebi_sdram_mem_device_parameter_cfg1;	// offset 0x50, Read/Write
	volatile EBI_SDRAM_MEM_DEVICE_PARAMETER_CFG2	ebi_sdram_mem_device_parameter_cfg2;	// offset 0x54, Read/Write
	volatile EBI_SDRAM_MEM_DEVICE_PARAMETER_CFG3	ebi_sdram_mem_device_parameter_cfg3;	// offset 0x58, Read/Write
	volatile EBI_SDRAM_CONTROLLER_CFG				ebi_sdram_controller_cfg;				// offset 0x5C, Read/Write
	volatile EBI_SDRAM_DEVICE_CFG					ebi_sdram_device_cfg;					// offset 0x60, Read/Write
	volatile EBI_SDRAM_DEVICE_PRECHARGE				ebi_sdram_device_precharge;				// offset 0x64, Read/Write
	volatile EBI_SDRAM_DEVICE_REFRESH				ebi_sdram_device_refresh;				// offset 0x68, Read/Write
	volatile EBI_SDRAM_INTERFACE_CFG3				ebi_sdram_interface_cfg3;				// offset 0x6C, Read/Write
	volatile EBI_SDRAM_DEVICE_STATUS				ebi_sdram_device_status;				// offset 0x70, Read/Write
	volatile unsigned long							filler02[3];							// offset 0x74-0x7C
	volatile EBI_SDRAM_CDC_CFG						ebi_sdram_cdc0_cfg;						// offset 0x80, Read/Write
	volatile EBI_SDRAM_CDC_MODE0					ebi_sdram_cdc0_mode0;					// offset 0x84, Read/Write
	volatile EBI_SDRAM_CDC_MODE1					ebi_sdram_cdc0_mode1;					// offset 0x88, Read/Write
	volatile EBI_SDRAM_CDC0_MODE2					ebi_sdram_cdc0_mode2;					// offset 0x8C, Read/Write
	volatile EBI_SDRAM_CDC_CFG						ebi_sdram_cdc1_cfg;						// offset 0x90, Read/Write
	volatile EBI_SDRAM_CDC_MODE0					ebi_sdram_cdc1_mode0;					// offset 0x94, Read/Write
	volatile EBI_SDRAM_CDC_MODE1					ebi_sdram_cdc1_mode1;					// offset 0x98, Read/Write
	volatile EBI_SDRAM_CDC_MODE2					ebi_sdram_cdc1_mode2;					// offset 0x9C, Read/Write
	volatile EBI_SDRAM_CDC_CFG						ebi_sdram_cdc2_cfg;						// offset 0xA0, Read/Write
	volatile EBI_SDRAM_CDC_MODE0					ebi_sdram_cdc2_mode0;					// offset 0xA4, Read/Write
	volatile EBI_SDRAM_CDC_MODE1					ebi_sdram_cdc2_mode1;					// offset 0xA8, Read/Write
	volatile EBI_SDRAM_CDC_MODE2					ebi_sdram_cdc2_mode2;					// offset 0xAC, Read/Write
	volatile EBI_SDRAM_CDC_CFG						ebi_sdram_cdc3_cfg;						// offset 0xB0, Read/Write
	volatile EBI_SDRAM_CDC_MODE0					ebi_sdram_cdc3_mode0;					// offset 0xB4, Read/Write
	volatile EBI_SDRAM_CDC_MODE1					ebi_sdram_cdc3_mode1;					// offset 0xB8, Read/Write
	volatile EBI_SDRAM_CDC_MODE2					ebi_sdram_cdc3_mode2;					// offset 0xBC, Read/Write
}
EBI1_REGS, *PEB1_REGS;


/////////////////////////////////////////////////////////////////////
// EBI2CR
/////////////////////////////////////////////////////////////////////

typedef struct _tagEBI2_CHIP_SELECT_CFG0
{
	unsigned long cs0_cfg		: 2;
	unsigned long cs1_cfg		: 2;
	unsigned long cs2_cfg		: 1;
	unsigned long cs3_cfg		: 1;
	unsigned long cs4_cfg		: 2;
	unsigned long cs5_cfg		: 2;
	unsigned long gpio2_cfg		: 1;
}
EBI2_CHIP_SELECT_CFG0;


typedef struct _tagEBI2_XMEM_CFG0
{
	unsigned long rd_cs_hold		: 5;
	unsigned long rd_active			: 5;
	unsigned long cs_wr_rd_setup	: 5;
	unsigned long wr_cs_hold		: 5;
	unsigned long wr_active			: 5;
	unsigned long addr_cs_setup		: 5;
}
EBI2_XMEM_CFG0;


typedef struct _tagEBI2_XMEM_CFG1
{
	unsigned long recov_cycles			: 5;
	unsigned long cs0_if_width			: 1;
	unsigned long cs1_if_width			: 1;
	unsigned long cs2_if_width			: 1;
	unsigned long cs3_if_width			: 1;
	unsigned long cs4_if_width			: 1;
	unsigned long cs5_if_width			: 1;
	unsigned long etm_gpio2_cs_if_width	: 1;
}
EBI2_XMEM_CFG1;


typedef struct _tagEBI2_LCD_CFG0
{
	unsigned long rd_cs_hold		: 5;
	unsigned long rd_active			: 5;
	unsigned long cs_wr_rd_setup	: 5;
	unsigned long wr_cs_hold		: 5;
	unsigned long wr_active			: 5;
	unsigned long addr_cs_setup		: 5;
}
EBI2_LCD_CFG0;


typedef struct _tagEBI2_LCD_CFG1
{
	unsigned long cs_wrenbl_setup		: 5;
	unsigned long wrenbl_active			: 5;
	unsigned long 						: 1;
	unsigned long cs_rdenbl_setup		: 5;
	unsigned long rdenbl_active			: 6;
	unsigned long lcd_if_enable			: 1;
	unsigned long lcd_enbl_polarity		: 1;
	unsigned long lcd_if_width			: 1;
	unsigned long lcd_16bit_add_mode	: 2;
	unsigned long lcd_recov_cycles		: 5;
}
EBI2_LCD_CFG1;


typedef struct _tagEBI2_ARBITER_CFG
{
	unsigned long lcdc_priority			: 2;
	unsigned long nandc_priority		: 2;
	unsigned long xmemc_priority		: 2;
	unsigned long nandc_cfg_priority	: 2;
}
EBI2_ARBITER_CFG;


typedef struct _tagEBI2_IO_DRVR1_CFG
{
	unsigned long addr_drvr				: 3;
	unsigned long addr18_drvr			: 3;
	unsigned long addr19_drvr			: 3;
	unsigned long addr20_drvr			: 3;
	unsigned long data_hi_byte_drvr		: 3;
	unsigned long data_low_byte_drvr	: 3;
	unsigned long ub_drvr				: 3;
	unsigned long lb_drvr				: 3;
	unsigned long we_drvr				: 3;
	unsigned long oe_drvr				: 3;
}
EBI2_IO_DRVR1_CFG;


typedef struct _tagEBI2_IO_DRVR2_CFG
{
	unsigned long cs0			: 3;
	unsigned long cs1			: 3;
	unsigned long cs2			: 3;
	unsigned long cs3			: 3;
	unsigned long cs4			: 3;
	unsigned long cs5			: 3;
	unsigned long busy1			: 3;
}
EBI2_IO_DRVR2_CFG;


typedef struct _tagEBI2_IO_DRVR3_CFG
{
	unsigned long addr18_pull	: 2;
	unsigned long addr19_pull	: 2;
	unsigned long addr20_pull	: 2;
	unsigned long busy0_pull	: 2;
	unsigned long busy1_pull	: 2;
}
EBI2_IO_DRVR3_CFG;


typedef struct _tagEBI2_DEBUG_SEL
{
	unsigned long debug_sel	: 16;
}
EBI2_DEBUG_SEL;


/////////////////////////////////////////////////////////////////////

typedef struct _tagEBI2CR_REGS
{
	volatile EBI2_CHIP_SELECT_CFG0	ebi2_chip_select_cfg0;		// offset 0x00		Read/Write
	volatile unsigned long			filler1[3];					// offset 0x04-0x0C
	volatile EBI2_XMEM_CFG0			ebi2_xmem_cfg0;				// offset 0x10		Read/Write
	volatile EBI2_XMEM_CFG1			ebi2_xmem_cfg1;				// offset 0x14		Read/Write
	volatile unsigned long			filler2[2];					// offset 0x18-0x1C
	volatile EBI2_LCD_CFG0			ebi2_lcd_cfg0;				// offset 0x20		Read/Write
	volatile EBI2_LCD_CFG1			ebi2_lcd_cfg1;				// offset 0x24		Read/Write
	volatile unsigned long			filler3[2];					// offset 0x28-0x2C
	volatile EBI2_ARBITER_CFG		ebi2_arbiter_cfg;			// offset 0x30		Read/Write
	volatile unsigned long			filler4[3];					// offset 0x34-0x3C
	volatile EBI2_IO_DRVR1_CFG		ebi2_io_drvr1_cfg;			// offset 0x40		Read/Write
	volatile EBI2_IO_DRVR2_CFG		ebi2_io_drvr2_cfg;			// offset 0x44		Read/Write
	volatile EBI2_IO_DRVR3_CFG		ebi2_io_drvr3_cfg;			// offset 0x48		Read/Write
	volatile unsigned long				filler5;				// offset 0x4C
	volatile EBI2_DEBUG_SEL			ebi2_debug_sel;				// offset 0x50		Read/Write
}
EBI2CR_REGS, *PEBI2CR_REGS;


/////////////////////////////////////////////////////////////////////
// CLK_CTL
/////////////////////////////////////////////////////////////////////

typedef struct _tagGLBL_CLK_ENA
{
	unsigned long axi_smi_clk_ena		: 1;
	unsigned long axi_li_a11s_clk_ena	: 1;
	unsigned long axi_li_apps_clk_ena	: 1;
	unsigned long axi_li_vg_clk_ena		: 1;
	unsigned long marm_etm_clk_ena		: 1;
	unsigned long adm_clk_ena			: 1;
	unsigned long ce_clk_ena			: 1;
	unsigned long sdc1_h_clk_ena		: 1;
	unsigned long sdc2_h_clk_ena		: 1;
	unsigned long mdp_clk_ena			: 1;
	unsigned long axi_arb_clk_ena		: 1;
	unsigned long axi_li_mss_clk_en		: 1;
	unsigned long axi_ebi1_clk_ena		: 1;
	unsigned long ebi2_clk_ena			: 1;
	unsigned long pbus_clk_ena			: 1;
	unsigned long ahb0_clk_ena			: 1;
	unsigned long ahb1_clk_ena			: 1;
	unsigned long uart1dm_p_clk_ena		: 1;
	unsigned long tsif_p_clk_ena		: 1;
	unsigned long reversed				: 1;
	unsigned long marm_clk_ena			: 1;
	unsigned long unused_bits_24_21		: 4;
	unsigned long	usbh_p_clk_ena			: 1;
	unsigned long	uart2dm_p_clk_ena		: 1;
	unsigned long	sdc3_h_clk_ena			: 1;
	unsigned long sdc4_h_clk_ena			: 1;
	unsigned long glbl_root_ena			: 1;
}
GLBL_CLK_ENA;


typedef struct _tagGLBL_CLK_STATE
{
	unsigned long axi_smi_clk		: 1;
	unsigned long axi_li_a11s_clk	: 1;
	unsigned long axi_li_apps_clk	: 1;
	unsigned long axi_li_vg_clk		: 1;
	unsigned long marm_etm_clk		: 1;
	unsigned long adm_clk			: 1;
	unsigned long ce_clk			: 1;
	unsigned long sdc1_h_clk		: 1;
	unsigned long sdc2_h_clk		: 1;
	unsigned long mdp_clk			: 1;
	unsigned long axi_arb_clk		: 1;
	unsigned long axi_li_mss_clk	: 1;
	unsigned long axi_ebi1_clk		: 1;
	unsigned long ebi2_clk			: 1;
	unsigned long pbus_clk			: 1;
	unsigned long ahb0_clk			: 1;
	unsigned long ahb1_clk			: 1;
	unsigned long uart1dm_p_clk		: 1;
	unsigned long tsif_p_clk		: 1;
	unsigned long reversed			: 1;
	unsigned long marm_clk			: 1;
	unsigned long unused_bits_24_21	: 4;
	unsigned long usbh_p_clk		: 1;
	unsigned long uart2dm_p_clk : 1;
	unsigned long sdc3_h_clk		: 1;
	unsigned long	sdc4_h_clk    : 1;
}
GLBL_CLK_STATE,
GLBL_CLK_INV;						// struct GLBL_CLK_STATE and GLBL_CLK_INV are the same


typedef struct _tagGLBL_SRC_NS_REG
{
	unsigned long src_sel			: 3;
	unsigned long src_div			: 4;
}
GLBL_SRC0_NS_REG,
GLBL_SRC1_NS_REG;					// struct GLBL_SRC0_NS_REG and GLBL_SRC10_NS_REG are the same


typedef struct _tagGLBL_SRC_OUT_SEL
{
	unsigned long nbid_sel			: 1;
}
GLBL_SRC_OUT_SEL;


typedef struct _tagGLBL_CLK_DIV
{
	unsigned long axi_marm_src		: 2;
	unsigned long pbus_src			: 2;
	unsigned long mahb1_src			: 2;
}
GLBL_CLK_DIV;


typedef struct _tagGLBL_SLEEP_EN
{
	unsigned long axi_smi_clk		: 1;
	unsigned long axi_li_a11s_clk	: 1;
	unsigned long axi_li_apps_clk	: 1;
	unsigned long axi_li_vg_clk		: 1;
	unsigned long marm_etm_clk		: 1;
	unsigned long adm_clk			: 1;
	unsigned long ce_clk			: 1;
	unsigned long sdc1_h_clk		: 1;
	unsigned long sdc2_h_clk		: 1;
	unsigned long mdp_clk			: 1;
	unsigned long axi_arb_clk		: 1;
	unsigned long axi_li_mss_clk	: 1;
	unsigned long axi_ebi1_clk		: 1;
	unsigned long ebi2_clk			: 1;
	unsigned long pbus_clk			: 1;
	unsigned long ahb0_clk			: 1;
	unsigned long ahb1_clk			: 1;
	unsigned long uart1dm_p_clk		: 1;
	unsigned long tsif_p_clk		: 1;
	unsigned long reverse19			: 1;
	unsigned long marm_clk			: 1;
	unsigned long reverse21			: 1;
	unsigned long imem_clk			: 1;
	unsigned long ebib_2x_clk		: 1;
	unsigned long ebib_clk			: 1;
	unsigned long ebi1_2x_clk		: 1;
	unsigned long ebi1_clk			: 1;
	unsigned long smi_2x_clk		: 1;
	unsigned long smi_clk			: 1;
	unsigned long glbl_src_clk		: 1;
	unsigned long marm_clk_pst		: 1;
	unsigned long marm_clk_pre		: 1;
}
GLBL_SLEEP_EN;


typedef struct _tagEBIB_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long src_div					: 4;
	unsigned long ebib_2x_clk_branch_ena	: 1;
	unsigned long ebib_2x_clk_inv			: 1;
	unsigned long ebib_clk_branch_ena		: 1;
	unsigned long ebib_clk_inv				: 1;
	unsigned long ebib_root_ena				: 1;
	unsigned long ebib_clk_div				: 2;
	unsigned long clk_sel					: 2;
}
EBIB_NS_REG;


typedef struct _tagEBI1_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long src_div					: 4;
	unsigned long ebi1_2x_clk_branch_ena	: 1;
	unsigned long ebi1_2x_clk_inv			: 1;
	unsigned long ebi1_clk_branch_ena		: 1;
	unsigned long ebi1_clk_inv				: 1;
	unsigned long ebi1_root_ena				: 1;
	unsigned long ebi1_clk_div				: 2;
	unsigned long clk_sel					: 1;
}
EBI1_NS_REG;


typedef struct _tagSMI_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long src_div					: 4;
	unsigned long smi_2x_clk_branch_ena		: 1;
	unsigned long smi_2x_clk_inv			: 1;
	unsigned long smi_clk_branch_ena		: 1;
	unsigned long smi_clk_inv				: 1;
	unsigned long smi_root_ena				: 1;
	unsigned long smi_clk_div				: 2;
	unsigned long clk_sel					: 1;
}
SMI_NS_REG;


typedef struct _tagADSP_NS_REG
{
	unsigned long src_sel						: 3;
	unsigned long src_div						: 4;
	unsigned long adsp_etm_clk_branch_ena		: 1;
	unsigned long adsp_etm_clk_inv				: 1;
	unsigned long adsp_clk_branch_ena			: 1;
	unsigned long adsp_clk_inv					: 1;
	unsigned long adsp_src_root_ena				: 1;
	unsigned long adspdiv2_vdc_clk_div			: 2;
	unsigned long adsp_tcxo_clk_branch_ena		: 1;
	unsigned long adsp_tcxo_clk_inv				: 1;
	unsigned long adsp_tcxo_clk_root_ena		: 1;
	unsigned long adsp_jtag_clk_branch_ena		: 1;
	unsigned long adsp_jtag_clk_inv				: 1;
	unsigned long adsp_jtag_clk_root_ena		: 1;
	unsigned long adspdiv2_vdc_clk_branch_ena	: 1;
	unsigned long adspdiv2_vdc_clk_inv			: 1;
}
ADSP_NS_REG;


typedef struct _tagXX_MD_REG
{
	unsigned long d_val						: 16;
	unsigned long m_val						: 16;
}
BTPCM_MD_REG,
CAM_VFE_MD_REG,
ECODEC_MD_REG,
GP_MD_REG,
GSM_DAI_MD_REG,
ICODEC_RX_MD_REG,
ICODEC_TX_MD_REG;							// all MD_REG structures are the same


typedef struct _tagBTPCM_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long pre_div_sel				: 2;
	unsigned long mncntr_mode				: 2;
	unsigned long mncntr_rst				: 1;
	unsigned long mncntr_en					: 1;
	unsigned long btpcm_clk_branch_ena		: 1;
	unsigned long btpcm_clk_inv				: 1;
	unsigned long btpcm_root_ena			: 1;
	unsigned long reversed15_12				: 4;
	unsigned long btpcm_n_val				: 16;
}
BTPCM_NS_REG;


typedef struct _tagCAM_VFE_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long pre_div_sel				: 2;
	unsigned long mncntr_mode				: 2;
	unsigned long mncntr_rst				: 1;
	unsigned long mncntr_en					: 1;
	unsigned long vfe_clk_branch_ena		: 1;
	unsigned long vfe_clk_inv				: 1;
	unsigned long vfe_mdc_clk_branch_ena	: 1;
	unsigned long vfe_mdc_clk_inv			: 1;
	unsigned long vfe_root_ena				: 1;
	unsigned long vfe_src_sel				: 1;
	unsigned long reversed15				: 1;
	unsigned long vfe_n_val					: 16;
}
CAM_VFE_NS_REG;


typedef struct _tagECODEC_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long pre_div_sel				: 2;
	unsigned long mncntr_mode				: 2;
	unsigned long mncntr_rst				: 1;
	unsigned long mncntr_en					: 1;
	unsigned long ecidecif_clk_branch_ena	: 1;
	unsigned long ecidecif_clk_inv			: 1;
	unsigned long ecidec_root_ena			: 1;
	unsigned long ecidec_clk_branch_ena		: 1;
	unsigned long ecidec_clk_inv			: 1;
	unsigned long reversed15_14				: 2;
	unsigned long ecodec_n_val				: 16;
}
ECODEC_NS_REG;


typedef struct _tagEMDH_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long src_div					: 4;
	unsigned long reversed					: 2;
	unsigned long emdh_clk_branch_ena		: 1;
	unsigned long emdh_clk_inv				: 1;
	unsigned long emdh_root_ena				: 1;
}
EMDH_NS_REG;


typedef struct _tagFUSE_NS_REG
{
	unsigned long reversed					: 9;
	unsigned long fuse_clk_branch_ena		: 1;
	unsigned long fuse_clk_inv				: 1;
	unsigned long fuse_root_ena				: 1;
}
FUSE_NS_REG;


typedef struct _tagGP_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long pre_div_sel				: 2;
	unsigned long mncntr_mode				: 2;
	unsigned long mncntr_rst				: 1;
	unsigned long mncntr_en					: 1;
	unsigned long gp_clk_branch_ena			: 1;
	unsigned long gp_clk_inv				: 1;
	unsigned long gp_root_ena				: 1;
	unsigned long reversed15_12				: 4;
	unsigned long gp_n_val					: 16;
}
GP_NS_REG;


typedef struct _tagGSM_DAI_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long pre_div_sel				: 2;
	unsigned long mncntr_mode				: 2;
	unsigned long mncntr_rst				: 1;
	unsigned long mncntr_en					: 1;
	unsigned long gsm_dai_clk_branch_ena	: 1;
	unsigned long gsm_dai_clk_inv			: 1;
	unsigned long gsm_dai_root_ena			: 1;
	unsigned long reversed15_12				: 4;
	unsigned long gsm_dai_n_val				: 16;
}
GSM_DAI_NS_REG;


typedef struct _tagI2C_NS_REG
{
	unsigned long reversed					: 9;
	unsigned long i2c_clk_branch_ena		: 1;
	unsigned long i2c_clk_inv				: 1;
	unsigned long i2c_root_ena				: 1;
}
I2C_NS_REG;


typedef struct _tagICODEC_RX_NS_REG
{
	unsigned long src_sel						: 3;
	unsigned long pre_div_sel					: 2;
	unsigned long mncntr_mode					: 2;
	unsigned long mncntr_rst					: 1;
	unsigned long mncntr_en						: 1;
	unsigned long icodec_rx_clk_branch_ena		: 1;
	unsigned long icodec_rx_clk_inv				: 1;
	unsigned long icodec_rx_root_ena			: 1;
	unsigned long icodec_2x_rx_clk_branch_ena	: 1;
	unsigned long icodec_2x_rx_clk_inv			: 1;
	unsigned long icodec_rx_clk_div				: 2;
	unsigned long icodec_rx_n_val				: 16;
}
ICODEC_RX_NS_REG;


typedef struct _tagICODEC_TX_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long pre_div_sel				: 2;
	unsigned long mncntr_mode				: 2;
	unsigned long mncntr_rst				: 1;
	unsigned long mncntr_en					: 1;
	unsigned long icodec_tx_clk_branch_ena	: 1;
	unsigned long icodec_tx_clk_inv			: 1;
	unsigned long icodec_tx_root_ena		: 1;
	unsigned long reversed13_12				: 2;
	unsigned long clk_sel					: 1;
	unsigned long reversed15				: 1;
	unsigned long icodec_tx_n_val			: 16;
}
ICODEC_TX_NS_REG;


typedef struct _tagMDC_NS_REG
{
	unsigned long src_sel					: 2;
	unsigned long reversed8_2				: 7;
	unsigned long mdc_clk_branch_ena		: 1;
	unsigned long mdc_clk_inv				: 1;
	unsigned long mdc_root_ena				: 1;
}
MDC_NS_REG;


typedef struct _tagPRPH_WEB_NS_REG
{
	unsigned long reversed					: 9;
	unsigned long prph_web_clk_branch_ena	: 1;
	unsigned long prph_web_clk_inv			: 1;
	unsigned long prph_web_root_ena			: 1;
}
PRPH_WEB_NS_REG;

typedef struct _tagGRP_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long src_div					: 4;
	unsigned long grp_clk_branch_ena		: 1;
	unsigned long grp_clk_inv				: 1;
	unsigned long imem_clk_branch_ena		: 1;
	unsigned long imem_clk_inv				: 1;
	unsigned long grp_root_ena				: 1;
	unsigned long axi_grp_clk_div			: 2;
	unsigned long clk_sel					: 1;
	unsigned long reversed					: 17;
}
GRP_NS_REG;

typedef struct _tagPCM_NS_REG
{
	unsigned long reversed0					: 9;
	unsigned long pcm_clk_stop_ena			: 1;
	unsigned long pcm_clk_inv				: 1;
	unsigned long pcm_root_ena				: 1;
	unsigned long reversed1					: 20;
}
PCM_NS_REG;

typedef struct _tagPMDH_NS_REG
{
	unsigned long src_sel					: 3;
	unsigned long src_div					: 4;
	unsigned long reversed0					: 2;
	unsigned long pmdh_clk_branch_ena		: 1;
	unsigned long pmdh_clk_inv				: 1;
	unsigned long pmdh_root_ena				: 1;
	unsigned long reversed1					: 20;
}
PMDH_NS_REG;

typedef struct _tagPUART_NS_REG
{
	unsigned long uart1_src_sel				: 3;
	unsigned long uart1_clk_inv				: 1;
	unsigned long uart1_root_ena			: 1;
	unsigned long uart1_clk_branch_ena		: 1;
	unsigned long uart2_src_sel				: 3;
	unsigned long uart2_clk_inv				: 1;
	unsigned long uart2_root_ena			: 1;
	unsigned long uart2_clk_branch_ena		: 1;
	unsigned long uart3_src_sel				: 3;
	unsigned long uart3_clk_inv				: 1;
	unsigned long uart3_root_ena			: 1;
	unsigned long uart3_clk_branch_ena		: 1;
				
	unsigned long reserved_bits_24_18		: 7;
	unsigned long tcxo6_src_en	            : 1;
	unsigned long tcxo5_src_en				: 1;
	unsigned long reversed0					: 5;
}
PUART_NS_REG;

typedef volatile struct _tagUARTDM_MD_REG
{
	unsigned long   d_val : 16; // bit 0~15
	unsigned long   m_val : 16; // bit 16~31
} UARTDM_MD_REG, *PUARTDM_MD_REG;

typedef volatile struct _tagUARTDM_NS_REG
{
	unsigned long src_sel : 3; // bit0~2
	unsigned long pre_div_sel : 2; // bit3~4
	unsigned long mncntr_mode : 2; // bit5~6
	unsigned long mncntr_rst : 1; // bit7
	unsigned long mncntr_en : 1; // bit8
	unsigned long uartdm_clk_branch_ena : 1; // bit9
	unsigned long uartdm_clk_inv : 1; // bit10
	unsigned long uartdm_root_ena : 1; // bit11
	unsigned long reserved : 4; // bit12~15
	unsigned long n_val : 16; // bit16~31
} UARTDM_NS_REG, *PUARTDM_NS_REG;

typedef struct _tagPLL0_MODE
{
	unsigned long pll_outctrl				: 1;
	unsigned long pll_bypassnl				: 1;
	unsigned long pll_reset_n				: 1;
	unsigned long pll_plltest				: 1;
	unsigned long pll_pre_div				: 4;
	unsigned long reversed0					: 24;
	
}
PLL0_MODE;

typedef struct _tagPLL0_L_VAL_REG
{
	unsigned long pll_l						: 8;
	unsigned long reversed0					: 24;
	
}
PLL0_L_VAL_REG;

typedef struct _tagPLL0_M_VAL
{
	unsigned long pll_m						: 11;
	unsigned long reversed0					: 21;
	
}
PLL0_M_VAL;

typedef struct _tagPLL0_N_VAL
{
	unsigned long pll_n						: 12;
	unsigned long reversed0					: 20;
	
}
PLL0_N_VAL;

typedef struct _tagPLL0_INTERNAL1
{
	unsigned long pll0_internal1			 : 32;
	
}
PLL0_INTERNAL1;

typedef struct _tagPLL0_INTERNAL2
{
	unsigned long pll0_internal2			 : 32;
	
}
PLL0_INTERNAL2;

typedef struct _tagPLL0_INTERNAL3
{
	unsigned long pll0_internal3			 : 32;
	
}
PLL0_INTERNAL3;

/////////////////////////////////////////////////////////////////////

typedef struct _tagCLK_CTL_REGS
{
	volatile GLBL_CLK_ENA			glbl_clk_ena;				// offset 0x00		Read/Write
	volatile GLBL_CLK_STATE			glbl_clk_state;				// offset 0x04		Read only
	volatile GLBL_SRC0_NS_REG		glbl_src0_ns_reg;			// offset 0x08		Read/Write
	volatile GLBL_SRC1_NS_REG		glbl_src1_ns_reg;			// offset 0x0C		Read/Write
	volatile GLBL_SRC_OUT_SEL		glbl_src_out_sel;			// offset 0x10		Read/Write
	volatile GLBL_CLK_DIV			glbl_clk_div;				// offset 0x14		Read/Write
	volatile GLBL_CLK_INV			glbl_clk_inv;				// offset 0x18		Read/Write
	volatile GLBL_SLEEP_EN			glbl_sleep_en;				// offset 0x1C		Read/Write
	
	// HTC_Brian_7200A, newly added in 7200A.
	volatile unsigned long			glbl_sleep_en2;				// offset 0x20		Read/Write
	
volatile unsigned long			dual_modem_ns_reg;			// offset 0x24		Read/Write
	volatile EBIB_NS_REG			ebib_ns_reg;				// offset 0x28		Read/Write
	volatile EBI1_NS_REG			ebi1_ns_reg;				// offset 0x2C		Read/Write
	volatile SMI_NS_REG				smi_ns_reg;					// offset 0x30		Read/Write
	volatile ADSP_NS_REG			adsp_ns_reg;				// offset 0x34		Read/Write
	volatile BTPCM_MD_REG			btpcm_md_reg;				// offset 0x38		Read/Write  7200A spec doesn't have this
	volatile BTPCM_NS_REG			btpcm_ns_reg;				// offset 0x3C		Read/Write
	volatile CAM_VFE_MD_REG			cam_vfe_md_reg;				// offset 0x40		Read/Write
	volatile CAM_VFE_NS_REG			cam_vfe_ns_reg;				// offset 0x44		Read/Write
	volatile ECODEC_MD_REG			ecodec_md_reg;				// offset 0x48		Read/Write
	volatile ECODEC_NS_REG			ecodec_ns_reg;				// offset 0x4C		Read/Write
	volatile EMDH_NS_REG			emdh_ns_reg;				// offset 0x50		Read/Write
	volatile FUSE_NS_REG			fuse_ns_reg;				// offset 0x54		Read/Write
	volatile GP_MD_REG				gp_md_reg;					// offset 0x58		Read/Write
	volatile GP_NS_REG				gp_ns_reg;					// offset 0x5C		Read/Write	
	volatile GSM_DAI_MD_REG			gsm_dai_md_reg;				// offset 0x60		Read/Write  7200A spec doesn't have this
	volatile GSM_DAI_NS_REG			gsm_dai_ns_reg;				// offset 0x64		Read/Write  7200A spec doesn't have this
	volatile I2C_NS_REG				i2c_ns_reg;					// offset 0x68		Read/Write
	volatile ICODEC_RX_MD_REG		icodec_rx_md_reg;			// offset 0x6C		Read/Write
	volatile ICODEC_RX_NS_REG		icodec_rx_ns_reg;			// offset 0x70		Read/Write
	volatile ICODEC_TX_MD_REG		icodec_tx_md_reg;			// offset 0x74		Read/Write
	volatile ICODEC_TX_NS_REG		icodec_tx_ns_reg;			// offset 0x78		Read/Write
	volatile MDC_NS_REG				mdc_ns_reg;					// offset 0x7C		Read/Write
	volatile PRPH_WEB_NS_REG		prph_web_ns_reg;			// offset 0x80		Read/Write
	volatile GRP_NS_REG				grp_ns_reg;					// offset 0x84		Read/Write
	volatile PCM_NS_REG				pcm_ns_reg;					// offset 0x88		Read/Write
	volatile PMDH_NS_REG			pmdh_ns_reg;				// offset 0x8C		Read/Write
	volatile DWORD					qmembist_ns_reg;			// offset 0x90
	volatile DWORD					sbi_ns_reg;					// offset 0x94
	volatile DWORD					sdac_md_reg;				// offset 0x98
	volatile DWORD					sdac_ns_reg;				// offset 0x9C
	volatile DWORD					sdc1_md_reg;				// offset 0xa0
	volatile DWORD					sdc1_ns_reg;				// offset 0xa4
	volatile DWORD					sdc2_md_reg;				// offset 0xa8
	volatile DWORD					sdc2_ns_reg;				// offset 0xac
	volatile DWORD					sdc3_md_reg; 				// offset 0xb0 
	volatile DWORD					sdc3_ns_reg; 				// offset 0xb4
	volatile DWORD					sdc4_md_reg; 				// offset 0xb8
	volatile DWORD					sdc4_ns_reg; 				// offset 0xbc	
	volatile DWORD					tlmm_ns_reg;				// offset 0xC0		Read/Write
	volatile DWORD					tsif_ns_reg;				// offset 0xC4              
	volatile DWORD					tv_md_reg;					// offset 0xC8              
	volatile DWORD					tv_ns_reg;					// offset 0xCC              
	volatile DWORD					uart1dm_md_reg;			// offset 0xD0	
	volatile DWORD					uart1dm_ns_reg;			// offset 0xD4
	volatile DWORD					uart2dm_md_reg;			// offset 0xD8
	volatile DWORD					uart2dm_ns_reg;			// offset 0xDC
//	volatile DWORD					uart_ns_reg;	   		// offset 0xE0
	volatile PUART_NS_REG				puart_ns_reg;	   		// offset 0xE0
	volatile DWORD					usb_md_reg;   			// offset 0xE4			                
	volatile DWORD					usb_ns_reg;					// offset 0xE8              
	volatile DWORD					vdc_md_reg;					// offset 0xEC              
	volatile DWORD					vdc_ns_reg;					// offset 0xF0               
	volatile DWORD					msm_clk_ringosc;		// offset 0xF4 	            
	volatile DWORD					tcxo_cnt;					  // offset 0xF8              
	volatile DWORD					ringosc_cnt;				// offset 0xFC 
	volatile DWORD					ADSP_FS;        		// offset 0x100     
	volatile DWORD					MARM_FS;        		// offset 0x104
	volatile DWORD					CLK_HALT_STATEA;		// offset 0x108
	volatile DWORD					CLK_HALT_STATEB;		// offset 0x10C
	volatile DWORD					MISC_CLK_CTL;   		// offset 0x110
	volatile DWORD					PLLTEST_PAD_CFG;		// offset 0x114
	volatile DWORD					CLKTEST_PAD_CFG;		// offset 0x118
	
	unsigned long           PAD1[0x39];					// offset 0x11C		(0x200-0X11C)/4	
	
	volatile DWORD					reset_all;					// offset 0x200
	volatile DWORD					mss_reset;					// offset 0x204
	volatile DWORD					axi_reset;					// offset 0x208
	volatile DWORD					a11_reset;					// offset 0x20C
	volatile DWORD					apps_reset;					// offset 0x210
	volatile DWORD					row_reset;					// offset 0x214
	volatile DWORD					clk_reset;					// offset 0x218
	volatile DWORD					reserved_21c;				// offset 0x21c
	
	unsigned long                   PAD2[0x18]; 					// offset 0x220		(0x280-0x220)/4
	
	volatile DWORD					vdd_vfe_gfs_ctl;			// offset 0x280			
	volatile DWORD					vdd_grp_gfs_ctl;			// offset 0x284
	volatile DWORD					vdd_vdc_gfs_ctl;			// offset 0x288
	volatile DWORD					vdd_tstm_gfs_ctl;			// offset 0x28C
	volatile DWORD					msm_rail_clamp_io;			// offset 0x290
	volatile DWORD					gfs_ctl_status;				// offset 0x294
	volatile DWORD					vdd_apc_plevel0;			// offset 0x298
	volatile DWORD					vdd_apc_plevel1;			// offset 0x29C
	volatile DWORD					vdd_apc_plevel2;			// offset 0x2A0
	volatile DWORD					vdd_apc_plevel3;			// offset 0x2A4
	volatile DWORD					vdd_apc_plevel4;			// offset 0x2A8
	volatile DWORD					vdd_apc_plevel5;			// offset 0x2AC
	volatile DWORD					vdd_apc_plevel6;			// offset 0x2B0
	volatile DWORD					vdd_apc_plevel7;			// offset 0x2B4
	volatile DWORD					vdd_apc_ssbi_addr;			// offset 0x2B8
	// New for 7200A
	volatile DWORD					usbh_md_reg;				// offset 0x2BC
	volatile DWORD					usbh_ns_reg;				// offset 0x2C0
	
	unsigned long                   PAD3[0xF]; 					// offset 0x2C4		(0x300-0x2C4)/4
	
	volatile DWORD					pll0_mode;					// offset 0x300     Read/Write
	volatile DWORD					pll0_l_val_reg;				// offset 0x304     Read/Write
	volatile DWORD					pll0_m_val;					// offset 0x308     Read/Write
	volatile DWORD					pll0_n_val;					// offset 0x30C     Read/Write
	volatile PLL0_INTERNAL1			pll0_internal1;				// offset 0x310     Read/Write
	volatile PLL0_INTERNAL2			pll0_internal2;				// offset 0x314     Read/Write
	volatile PLL0_INTERNAL3			pll0_internal3;				// offset 0x318     Read/Write
	
	volatile DWORD					pll1_mode;					// offset 0x31C     Read/Write
	volatile DWORD					pll1_l_val_reg;				// offset 0x320     Read/Write
	volatile DWORD					pll1_m_val;					// offset 0x324     Read/Write
	volatile DWORD					pll1_n_val;					// offset 0x328     Read/Write
	volatile PLL0_INTERNAL1			pll1_internal1;				// offset 0x32C     Read/Write
	volatile PLL0_INTERNAL2			pll1_internal2;				// offset 0x330     Read/Write
	volatile PLL0_INTERNAL3			pll1_internal3;				// offset 0x334     Read/Write
	
	volatile DWORD					pll2_mode;					// offset 0x338     Read/Write
	volatile DWORD					pll2_l_val_reg;				// offset 0x33C     Read/Write
	volatile DWORD					pll2_m_val;					// offset 0x340     Read/Write
	volatile DWORD					pll2_n_val;					// offset 0x344     Read/Write
	volatile PLL0_INTERNAL1			pll2_internal1;				// offset 0x348     Read/Write
	volatile PLL0_INTERNAL2			pll2_internal2;				// offset 0x34C     Read/Write
	volatile PLL0_INTERNAL3			pll2_internal3;				// offset 0x350     Read/Write
	
	volatile PLL0_MODE				pll3_mode;					// offset 0x354     Read/Write
	volatile PLL0_L_VAL_REG			pll3_l_val_reg;				// offset 0x358     Read/Write
	volatile PLL0_M_VAL				pll3_m_val;					// offset 0x35C     Read/Write
	volatile PLL0_N_VAL				pll3_n_val;					// offset 0x360     Read/Write
	volatile PLL0_INTERNAL1			pll3_internal1;				// offset 0x364     Read/Write
	volatile PLL0_INTERNAL2			pll3_internal2;				// offset 0x368     Read/Write
	volatile PLL0_INTERNAL3			pll3_internal3;				// offset 0x36C     Read/Write
	
	//TBD: the following registers are not defined yet.
}
CLK_CTL_REGS, *PCLK_CTL_REGS;


/////////////////////////////////////////////////////////////////////
// ARM11S_CSR
/////////////////////////////////////////////////////////////////////

typedef struct _tagAGPT_ENABLE
{
	unsigned long		en				: 1;
	unsigned long		clr_on_match_en	: 1;		// enable if count should be clear when agpt_count_val == agpt_match_val
}
AGPT_ENABLE;


typedef struct _tagADPT_ENABLE
{
	unsigned long		en				: 1;
	unsigned long		clr_on_match_en	: 1;		// enable if count should be clear when agpt_count_val == agpt_match_val
}
ADPT_ENABLE;


typedef struct _tagACSRPROTRCTION
{
	unsigned long		en				: 1;
}
ACSRPROTRCTION;


typedef struct _tagA11S_CLK_CNTL
{
	unsigned long		clk_src1_div		: 4;
	unsigned long		clk_src1_sel		: 4;
	unsigned long		clk_src0_div		: 4;
	unsigned long		clk_src0_sel		: 4;
	unsigned long		wt_st_cnt			: 16;
}
A11S_CLK_CNTL;


typedef struct _tagA11S_CLK_SEL
{
	unsigned long		clk_sel_src1n0	: 1;
	unsigned long		ahb_clk_div		: 2;
}
A11S_CLK_SEL;


typedef struct _tagA11S_STANDBY_CTL
{
	unsigned long		wakeup_dly_cnt	: 3;
	unsigned long		a11ram_bb_en	: 1;
}
A11S_STANDBY_CTL;


typedef struct _tagA11S_TESTCLK
{
	unsigned long		sel			: 3;
}
A11S_TESTCLK;


typedef struct _tagA11S_CLK_ROOT_EN
{
	unsigned long		a11_clk_en		: 1;
	unsigned long		dgt_clk_en		: 1;
	unsigned long		gpt_clk_en		: 1;
}
A11S_CLK_ROOT_EN;


typedef struct _tagA11S_CLK_ENABLE
{
	unsigned long		clk_en_arm11_core		: 1;
	unsigned long		clk_en_arm11_ahb		: 1;
	unsigned long		clk_en_data_bridge		: 1;
	unsigned long		clk_en_dma_bridge		: 1;
	unsigned long		clk_en_peripheral		: 1;
	unsigned long		clk_en_debug_timer		: 1;
	unsigned long		clk_en_general_timer	: 1;
}
A11S_CLK_ENABLE;


typedef struct _tagA11S_CLK_INV
{
	unsigned long		clk_inv_arm11_core		: 1;
	unsigned long		clk_inv_arm11_ahb		: 1;
	unsigned long		clk_inv_data_bridge		: 1;
	unsigned long		clk_inv_dma_bridge		: 1;
	unsigned long		clk_inv_peripheral		: 1;
	unsigned long		clk_inv_debug_timer		: 1;
	unsigned long		clk_inv_general_timer	: 1;
}
A11S_CLK_INV;


typedef struct _tagA11S_SLEEP_EN
{
	unsigned long		slp_en_arm11_core		: 1;
	unsigned long		slp_en_arm11_ahb		: 1;
	unsigned long		slp_en_data_bridge		: 1;
	unsigned long		slp_en_dma_bridge		: 1;
	unsigned long		slp_en_peripheral		: 1;
	unsigned long		slp_en_debug_timer		: 1;
	unsigned long		slp_en_general_timer	: 1;
}
A11S_SLEEP_EN;


typedef struct _tagA11S_CLK_STATUS
{
	unsigned long		clk_sts_arm11_core		: 1;
	unsigned long		clk_sts_arm11_ahb		: 1;
	unsigned long		clk_sts_data_bridge		: 1;
	unsigned long		clk_sts_dma_bridge		: 1;
	unsigned long		clk_sts_peripheral		: 1;
	unsigned long		clk_sts_debug_timer		: 1;
	unsigned long		clk_sts_general_timer	: 1;
}
A11S_CLK_STATUS;


typedef struct _tagVDD_SVS_PLEVEL
{
	unsigned long		svs_ctl_status			: 3;
	unsigned long		next_plevel				: 3;
	unsigned long		reversed6				: 1;
	unsigned long		svs_req					: 1;
}
VDD_SVS_PLEVEL;


typedef struct _tagA11S_PWRDOWN
{
	unsigned long		req_on_swfi				: 1;
}
A11S_PWRDOWN;


typedef struct _tagETM11_RAIL_PWR_DWN
{
	unsigned long		vdd_etm11_pwr_dwn		: 1;
}
ETM11_RAIL_PWR_DWN;


typedef struct _tagETM11_RAIL_CLAMP_IO
{
	unsigned long		vdd_etm11_clamp		: 1;
}
ETM11_RAIL_CLAMP_IO;


typedef struct _tagA11RAMBACKBIAS
{
	unsigned long		a11ram_bb			: 1;
}
A11RAMBACKBIAS;


typedef struct _tagA11RINGOSC
{
	unsigned long		ro_nt_en			: 1;
	unsigned long		ro_lt_en			: 1;
	unsigned long		reversed7_2			: 6;
	unsigned long		ring_osc_sel		: 6;
}
A11RINGOSC;


typedef struct _tagA11S_TEST_BUS_SEL
{
	unsigned long		sel					: 5;
	unsigned long		reversed23_5		: 19;
	unsigned long		spare_cfg			: 8;
}
A11S_TEST_BUS_SEL;


typedef struct _tagA11S_TIC_RESET_DISABLE
{
	unsigned long		dis					: 1;
}
A11S_TIC_RESET_DISABLE;


/////////////////////////////////////////////////////////////////////

typedef struct _tagARM11_CSR_RGES
{
	volatile unsigned long		agpt_match_val;			// offset 0x00		Read/Write
	volatile unsigned long		agpt_count_val;			// offset 0x04		Read/Write
	volatile AGPT_ENABLE		agpt_enable;			// offset 0x08		Read/Write
	volatile unsigned long		agpt_clear;				// offset 0x0C		Write only
	volatile unsigned long		adpt_match_val;			// offset 0x10		Read/Write
	volatile unsigned long		adpt_count_val;			// offset 0x14		Read/Write
	volatile ADPT_ENABLE		adpt_enable;			// offset 0x18		Read/Write
	volatile unsigned long		adpt_clear;				// offset 0x1C		Write only
	volatile ACSRPROTRCTION		acsrprotection;			// offset 0x20		Read/Write
	volatile unsigned long		filler01[0x37];			// offset 0x24 to 0xFC

	volatile A11S_CLK_CNTL		a11s_clk_cntl;			// offset 0x100		Read/Write
	volatile A11S_CLK_SEL		a11s_clk_sel;			// offset 0x104		Read/Write
	volatile A11S_STANDBY_CTL	a11s_standby_ctl;		// offset 0x108		Read/Write
	volatile A11S_TESTCLK		a11s_testclk;			// offset 0x10C		Read/Write
	volatile A11S_CLK_ROOT_EN	a11s_clk_root_en;		// offset 0x110		Read/Write
	volatile A11S_CLK_ENABLE	a11s_clk_enable;		// offset 0x114		Read/Write
	volatile A11S_CLK_INV		a11s_clk_inv;			// offset 0x118		Read/Write
	volatile A11S_SLEEP_EN		a11s_sleep_en;			// offset 0x11C		Read/Write
	volatile A11S_CLK_STATUS	a11s_clk_status;		// offset 0x120		Read only
	volatile VDD_SVS_PLEVEL		vdd_svs_plevel;			// offset 0x124		Read/Write
	volatile unsigned long		filler02[0xB6];			// offset 0x128 to 0x3FC

	volatile unsigned long		a2m_int[7];				// offset 0x400 to 0x418 Write only
	volatile unsigned long		filler03[0x09];			// offset 0x41C to 0x43C
	
	volatile A11S_PWRDOWN		a11s_pwrdown;			// offset 0x440		Read/Write
	volatile unsigned long		filler04[0x2F];			// offset 0x444 to 0x4FC
	
	volatile ETM11_RAIL_PWR_DWN	etm11_rail_pwr_dwn;		// offset 0x500		Read/Write
	volatile ETM11_RAIL_CLAMP_IO etm11_rail_clamp_io;	// offset 0x504		Read/Write
	volatile A11RAMBACKBIAS		a11rambackbias;			// offset 0x508		Read/Write
	volatile unsigned long		reversed_50C;			// offset 0x50C		Read/Write
	volatile A11RINGOSC			a11ringosc;				// offset 0x510		Read/Write
	volatile unsigned long		reversed_514;			// offset 0x514		Read/Write
	volatile A11S_TEST_BUS_SEL	a11s_test_bus_sel;		// offset 0x518		Read/Write
	volatile A11S_TIC_RESET_DISABLE a11s_tic_reset_disable; // offset 0x51C		Read/Write
}
ARM11S_CSR_REGS, *PARM11S_CSR_REGS;


/////////////////////////////////////////////////////////////////////
// ARM11_VIC
/////////////////////////////////////////////////////////////////////

#define TOTAL_INTERRUPT_SOURCE		54

#define VICINT0_A9_M2A_0_BIT		(1 << 0)
#define VICINT0_A9_M2A_1_BIT		(1 << 1)
#define VICINT0_A9_M2A_2_BIT		(1 << 2)
#define VICINT0_A9_M2A_3_BIT		(1 << 3)
#define VICINT0_A9_M2A_4_BIT		(1 << 4)
#define VICINT0_A9_M2A_5_BIT		(1 << 5)
#define VICINT0_A9_M2A_6_BIT		(1 << 6)
#define VICINT0_GP_TIMER_EXP_BIT	(1 << 7)
#define VICINT0_DEBUG_TIMER_EXP_BIT	(1 << 8)
#define VICINT0_UART1_BIT			(1 << 9)
#define VICINT0_UART2_BIT			(1 << 10)
#define VICINT0_UART3_BIT			(1 << 11)
#define VICINT0_UART1_RX_BIT		(1 << 12)
#define VICINT0_UART2_RX_BIT		(1 << 13)
#define VICINT0_UART3_RX_BIT		(1 << 14)
#define VICINT0_USB_OTG_BIT			(1 << 15)
#define VICINT0_MDDI_PRI_BIT		(1 << 16)
#define VICINT0_MDDI_EXT_BIT		(1 << 17)
#define VICINT0_MDDI_CLIENT_BIT		(1 << 18)
#define VICINT0_MDP_BIT				(1 << 19)
#define VICINT0_GRAPHICS_BIT		(1 << 20)
#define VICINT0_ADM_AARM_BIT		(1 << 21)
#define VICINT0_ADSP_A11_BIT		(1 << 22)
#define VICINT0_ADSP_A9A11_BIT		(1 << 23)
#define VICINT0_SDC1_0_BIT			(1 << 24)
#define VICINT0_SDC1_1_BIT			(1 << 25)
#define VICINT0_SDC2_0_BIT			(1 << 26)
#define VICINT0_SDC2_1_BIT			(1 << 27)
#define VICINT0_KEYSENSE_BIT		(1 << 28)
#define VICINT0_TCHSCRN_SSBI_BIT	(1 << 29)
#define VICINT0_TCHSCRN1_BIT		(1 << 30)
#define VICINT0_TCHSCRN2_BIT		(1 << 31)

#define VICINT1_GPIO_GROUP_BIT		(1 << 0)
#define VICINT1_GPIO_GROUP2_BIT		(1 << 1)
#define VICINT1_PWB_I2C_BIT			(1 << 2)
#define VICINT1_NAND_WR_ER_DONE_BIT	(1 << 3)
#define VICINT1_NAND_OP_DONE_BIT	(1 << 4)
#define VICINT1_SOFTRESET_BIT		(1 << 5)
#define VICINT1_PUBS_ARM11_BIT		(1 << 6)
#define VICINT1_AXI_MPU_SMI_BIT		(1 << 7)
#define VICINT1_AXI_MPU_EBI1_BIT	(1 << 8)
#define VICINT1_AD_HSSD_BIT			(1 << 9)
#define VICINT1_ARM11_PM_BIT		(1 << 10)
#define VICINT1_ARM11_DMA_BIT		(1 << 11)
#define VICINT1_TSIF_IRQ_BIT		(1 << 12)
#define VICINT1_UART1DM_IRQ_BIT		(1 << 13)
#define VICINT1_UART1DM_RX_BIT		(1 << 14)
#define VICINT1_SPARE0_BIT			(1 << 15)


typedef struct _tagVIC_STATUS_0
{
	unsigned long	a9_m2a_0		: 1;
	unsigned long	a9_m2a_1		: 1;
	unsigned long	a9_m2a_2		: 1;
	unsigned long	a9_m2a_3		: 1;
	unsigned long	a9_m2a_4		: 1;
	unsigned long	a9_m2a_5		: 1;
	unsigned long	a9_m2a_6		: 1;
	unsigned long	gp_timer_exp	: 1;
	unsigned long	debug_timer_exp	: 1;
	unsigned long	uart1			: 1;
	unsigned long	uart2			: 1;
	unsigned long	uart3			: 1;
	unsigned long	uart1_rx		: 1;
	unsigned long	uart2_rx		: 1;
	unsigned long	uart3_rx		: 1;
	unsigned long	usb_otg			: 1;
	unsigned long	mddi_pri		: 1;
	unsigned long	mddi_ext		: 1;
	unsigned long	mddi_client		: 1;
	unsigned long	mdp				: 1;
	unsigned long	graphics		: 1;
	unsigned long	adm_aarm		: 1;
	unsigned long	adsp_a11		: 1;
	unsigned long	adsp_a9a11		: 1;
	unsigned long	sdc1_0			: 1;
	unsigned long	sdc1_1			: 1;
	unsigned long	sdc2_0			: 1;
	unsigned long	sdc2_1			: 1;
	unsigned long	keysense		: 1;
	unsigned long	tchscrn_ssbi	: 1;
	unsigned long	tchscrn1		: 1;
	unsigned long	tchscrn2		: 1;
}
VIC_STATUS_0;


typedef struct _tagVIC_STATUS_1
{
	unsigned long	gpio_group		: 1;
	unsigned long	gpio_group2		: 1;
	unsigned long	pwb_i2c			: 1;
	unsigned long	softreset		: 1;
	unsigned long	nand_wr_er_done	: 1;
	unsigned long	nand_op_done	: 1;	
	unsigned long	pubs_arm11		: 1;
	unsigned long	axi_mpu_smi		: 1;
	unsigned long	axi_mpu_ebi1	: 1;
	unsigned long	ad_hssd			: 1;
	unsigned long	arm11_pm		: 1;
	unsigned long	arm11_dma		: 1;
	unsigned long	tsif_irq		: 1;
	unsigned long	uart1dm_irq		: 1;
	unsigned long	uart1dm_rx		: 1;
	unsigned long	usb_hs			: 1;	
	unsigned long	sdc3_0			: 1;
	unsigned long	sdc3_1			: 1;
	unsigned long	sdc4_0			: 1;
	unsigned long	sdc4_1			: 1;
	unsigned long	uart2dm_irq		: 1;
	unsigned long	uart2dm_rx		: 1;
}
VIC_STATUS_1;


typedef struct _tagVICINTMASTEREN
{
	unsigned long	irq_enable		: 1;
	unsigned long	fiq_enable		: 1;
}
VICINTMASTEREN;


typedef struct _tagVICPROTECTION
{
	unsigned long	en				: 1;
}
VICPROTECTION;


typedef struct _tagVICCONFIG
{
	unsigned long	vicport_en		: 1;
}
VICCONFIG;


/////////////////////////////////////////////////////////////////////

typedef struct _tagARM11_VIC_RGES
{
	volatile VIC_STATUS_0	vicintselect_0;					// offset 0x00		Read/Write (0:IRQ, 1:FIQ)
	volatile VIC_STATUS_1	vicintselect_1;					// offset 0x04		Read/Write (0:IRQ, 1:FIQ)
	volatile unsigned long  reserved008;					// offset 0x08
	volatile unsigned long  reserved00C;					// offset 0x0C
	
	volatile VIC_STATUS_0	vicinten_0;						// offset 0x10		Read/Write (0:Disable, 1:Enable)
	volatile VIC_STATUS_1	vicinten_1;						// offset 0x14		Read/Write (0:Disable, 1:Enable)
	volatile unsigned long  reserved018;					// offset 0x18
	volatile unsigned long  reserved01C;					// offset 0x1C
	
	volatile unsigned long	vicintenclear_0;				// offset 0x20		Write only
	volatile unsigned long	vicintenclear_1;				// offset 0x24		Write only
	volatile unsigned long  reserved028;					// offset 0x28
	volatile unsigned long  reserved02C;					// offset 0x2C
	
	volatile unsigned long	vicintenset_0;					// offset 0x30		Write only
	volatile unsigned long	vicintenset_1;					// offset 0x34		Write only
	volatile unsigned long  reserved038;					// offset 0x38
	volatile unsigned long  reserved03C;					// offset 0x3C
	
	volatile VIC_STATUS_0	vicinttype_0;					// offset 0x40		Read/Write (0:Level, 1:Edge)
	volatile VIC_STATUS_1	vicinttype_1;					// offset 0x44
	volatile unsigned long  reserved048;					// offset 0x48
	volatile unsigned long  reserved04C;					// offset 0x4C
	
	
	volatile unsigned long	vicintpolarity_0;				// offset 0x50
	volatile unsigned long	vicintpolarity_1;				// offset 0x54
	volatile unsigned long  reserved058;					// offset 0x58
	volatile unsigned long  reserved05C;					// offset 0x5C
	
	volatile unsigned long	vicnopendval;					// offset 0x60		Read/Write	
	volatile VICINTMASTEREN	vicintmasteren;					// offset 0x64
	volatile VICCONFIG		vicconfig;						// offset 0x68
	volatile VICPROTECTION	vicprotection;					// offset 0x6c
	
	volatile unsigned long  reserved07x[4];					// offset 0x70,74,78,7C
	
	volatile VIC_STATUS_0	vicirq_status_0;				// offset 0x80		Read only
	volatile VIC_STATUS_1	vicirq_status_1;				// offset 0x84		Read only
	volatile unsigned long  reserved088;					// offset 0x88
	volatile unsigned long  reserved08C;					// offset 0x8C
	
	volatile VIC_STATUS_0	vicfiq_status_0;				// offset 0x90		Read only
	volatile VIC_STATUS_1	vicfiq_status_1;				// offset 0x94		Read only
	volatile unsigned long  reserved098;					// offset 0x98
	volatile unsigned long  reserved09C;					// offset 0x9C
	
	volatile VIC_STATUS_0	vicraw_status_0;				// offset 0xA0		Read only
	volatile VIC_STATUS_1	vicraw_status_1;				// offset 0xA4		Read only
	volatile unsigned long  reserved0A8;					// offset 0xA8
	volatile unsigned long  reserved0AC;					// offset 0xAC
	
	volatile unsigned long	vicintclear_0;					// offset 0xB0		Write only
	volatile unsigned long	vicintclear_1;					// offset 0xB4		Write only
	volatile unsigned long  reserved0B8;					// offset 0xB8
	volatile unsigned long  reserved0BC;					// offset 0xBC
		
	volatile unsigned long	vicsoftint_0;					// offset 0xC0		Write only
	volatile unsigned long	vicsoftint_1;					// offset 0xC4		Write only
	volatile unsigned long  reserved0C8;					// offset 0xC8
	volatile unsigned long  reserved0CC;					// offset 0xCC
	
	volatile unsigned long	vic_vec_rd;						// offset 0x0D0
	volatile unsigned long	vic_vec_pend_rd;				// offset 0x0D4
	volatile unsigned long	vic_irq_vec_wr;					// offset 0x0D8
	volatile unsigned long  reserved0DC;					// offset 0x0DC
	
	volatile unsigned long	vic_irq_in_service;				// offset 0x0E0
	volatile unsigned long	vic_irq_in_stack;				// offset 0x0E4
	volatile unsigned long	vic_test_bus_sel;				// offset 0x0E8
	volatile unsigned long  reserved0EC;					// offset 0x0EC
	
	volatile unsigned long  reserved0Fx[4];					// offset 0xF0,F4,F8,FC
	
	volatile unsigned long  reserved1xx[64];				// offset 0x100~0x1FF
	
	volatile unsigned long	vicpriority[64];				// offset 0x200		Read/Write (Vector[0..53]
	
	volatile unsigned long  reserved3xx[64];				// offset 0x300~0x3FF
	
	volatile unsigned long	vicvectaddr[64];				// offset 0x400		Read/Write (Vector[0..53]
		
}
ARM11S_VIC_REGS, *PARM11S_VIC_REGS;

typedef struct {
	
	volatile DWORD	reserved[0x20];			// offset 0x0
	volatile DWORD  halt_req;				// offset 0x80
	volatile DWORD  halt_ack;				// offset 0x84
	
}AXILVG_REGS;

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
LINK_TYPE PEB1_REGS pEbi1Regs = (PEB1_REGS) EBI1_REGS_VIRTUAL_BASE;
LINK_TYPE PEBI2CR_REGS pEbi2crRegs = (PEBI2CR_REGS) EBI2CR_REGS_VIRTUAL_BASE;
LINK_TYPE PCLK_CTL_REGS pClkCtlRegs = (PCLK_CTL_REGS) CLK_CTL_REGS_VIRTUAL_BASE;
LINK_TYPE PARM11S_CSR_REGS pArm11sCsrRegs = (PARM11S_CSR_REGS) ARM11S_CSR_REGS_VIRTUAL_BASE;
LINK_TYPE PARM11S_VIC_REGS pArm11sVicRegs = (PARM11S_VIC_REGS) ARM11S_VIC_REGS_VIRTUAL_BASE;
#else
LINK_TYPE PEB1_REGS pEbi1Regs = NULL;
LINK_TYPE PEBI2CR_REGS pEbi2crRegs = NULL;
LINK_TYPE PCLK_CTL_REGS pClkCtlRegs = NULL;
LINK_TYPE PARM11S_CSR_REGS pArm11sCsrRegs = NULL;
LINK_TYPE PARM11S_VIC_REGS pArm11sVicRegs = NULL;
#endif

#else // not first entry

LINK_TYPE PEB1_REGS pEbi1Regs;
LINK_TYPE PEBI2CR_REGS pEbi2crRegs;
LINK_TYPE PCLK_CTL_REGS pClkCtlRegs;
LINK_TYPE PARM11S_CSR_REGS pArm11sCsrRegs;
LINK_TYPE PARM11S_VIC_REGS pArm11sVicRegs;

#endif


#endif // __MSM7500_CORE_HEADER

