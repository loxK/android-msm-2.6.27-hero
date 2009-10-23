/*
 * arch/arm/mach-msm/board-heroc-camsensor.c - s5k3e2f sensor driver
 *
 *  Copyright (C) 2008 becker hsieh <becker0213@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>  
#include <linux/freezer.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/clk.h>
#include <net/sock.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_iomap.h>
#include <mach/s5k3e2fx.h>
#include "board-hero.h"
#include "board-hero-camsensor.h"

typedef struct 
{
  uint8_t pre_pll_clk_div;               // 0x0305
  uint8_t pll_multiplier_msb;            // 0x0306
  uint8_t pll_multiplier_lsb;            // 0x0307
  uint8_t vt_pix_clk_div;                // 0x0301
  uint8_t vt_sys_clk_div;                // 0x0303
  uint8_t op_pix_clk_div;                // 0x0309
  uint8_t op_sys_clk_div;                // 0x030B
  uint8_t ccp_data_format_msb;           // 0x0112
  uint8_t ccp_data_format_lsb;           // 0x0113
  uint8_t x_output_size_msb;             // 0x034C
  uint8_t x_output_size_lsb;             // 0x034D
  uint8_t y_output_size_msb;             // 0x034E
  uint8_t y_output_size_lsb;             // 0x034F
  uint8_t x_even_inc;                    // 0x0381
  uint8_t x_odd_inc;                     // 0x0383 
  uint8_t y_even_inc;                    // 0x0385 
  uint8_t y_odd_inc;                     // 0x0387 
  uint8_t binning_enable;                // 0x3014
  uint8_t frame_length_lines_msb;        // 0x0340
  uint8_t frame_length_lines_lsb;        // 0x0341
  uint8_t line_length_pck_msb;           // 0x0342
  uint8_t line_length_pck_lsb;           // 0x0343
  uint8_t shade_clk_enable ;             // 0x30AC
  uint8_t sel_ccp;                       // 0x30C4
  uint8_t vpix;                          // 0x3024
  uint8_t clamp_on;                      // 0x3015
  uint8_t offset;                        // 0x307E
  uint8_t ld_start;                      // 0x3000
  uint8_t ld_end;                        // 0x3001
  uint8_t sl_start;                      // 0x3002
  uint8_t sl_end;                        // 0x3003
  uint8_t rx_start;                      // 0x3004
  uint8_t s1_start;                      // 0x3005
  uint8_t s1_end;                        // 0x3006
  uint8_t s1s_start;                     // 0x3007
  uint8_t s1s_end;                       // 0x3008
  uint8_t s3_start;                      // 0x3009
  uint8_t s3_end;                        // 0x300A
  uint8_t cmp_en_start;                  // 0x300B
  uint8_t clp_sl_start;                  // 0x300C
  uint8_t clp_sl_end;                    // 0x300D
  uint8_t off_start;                     // 0x300E
  uint8_t rmp_en_start;                  // 0x300F
  uint8_t tx_start;                      // 0x3010
  uint8_t tx_end;                        // 0x3011
  uint8_t stx_width;                     // 0x3012                                       
  uint8_t reg_3152_reserved;             // 0x3152
  uint8_t reg_315A_reserved;             // 0x315A
  uint8_t analogue_gain_code_global_msb; // 0x0204
  uint8_t analogue_gain_code_global_lsb; // 0x0205
  uint8_t fine_integration_time;         // 0x0200
  uint8_t coarse_integration_time;       // 0x0202
} reg_struct;


/***********************************************************
* global variable
************************************************************/
static uint16_t chipid;
static struct i2c_client *pclient_i2c;
static struct clk *vfe_clk;
/* camif clocks */
static struct clk *vfe_mdc_clk;
static struct clk *mdc_clk;
static int mdc_clk_enabled;
static int vfe_mdc_clk_enabled;
static int vfe_clk_enabled;
static int powered;
static uint16_t line_keep = 994;
static const uint32_t fps_divider = 1;

static reg_struct s5k3e2fx_reg_pattern[2] = 
{ 
  {
    0x06,  // pre_pll_clk_div               REG=0x0305
    0x00,  // pll_multiplier_msb            REG=0x0306
    0x88,  // pll_multiplier_lsb            REG=0x0307 0x88
    0x0a,  // vt_pix_clk_div                REG=0x0301
    0x01,  // vt_sys_clk_div                REG=0x0303
    0x0a,  // op_pix_clk_div                REG=0x0309
    0x01,  // op_sys_clk_div                REG=0x030B
    0x0a,  // ccp_data_format_msb           REG=0x0112*
    0x0a,  // ccp_data_format_lsb           REG=0x0113*
    0x05,  // x_output_size_msb             REG=0x034C
    0x10,  // x_output_size_lsb             REG=0x034D
    0x03,  // y_output_size_msb             REG=0x034E
    0xcc,  // y_output_size_lsb             REG=0x034F

    /* enable binning for preview */
    0x01,  // x_even_inc                    REG=0x0381
    0x01,  // x_odd_inc                     REG=0x0383
    0x01,  // y_even_inc                    REG=0x0385
    0x03,  // y_odd_inc                     REG=0x0387
    0x06,  // binning_enable                REG=0x3014
           // 
    0x03,  // frame_length_lines_msb        REG=0x0340
    0xe2,  // frame_length_lines_lsb        REG=0x0341 /0xde
    0x0a,  // line_length_pck_msb           REG=0x0342
    0xac,  // line_length_pck_lsb           REG=0x0343
    0x81,  // shade_clk_enable              REG=0x30AC
    0x01,  // sel_ccp                       REG=0x30C4
    0x04,  // vpix                          REG=0x3024
    0x00,  // clamp_on                      REG=0x3015
    0x02,  // offset                        REG=0x307E
    0x03,  // ld_start                      REG=0x3000
    0x94,  // ld_end                        REG=0x3001 //0x9c
    0x02,  // sl_start                      REG=0x3002
    0x95,  // sl_end                        REG=0x3003 //0x9e
    0x04,  // rx_start                      REG=0x3004 //0x05
    0x05,  // s1_start                      REG=0x3005 //0x0f
    0x3c,  // s1_end                        REG=0x3006 //0x24
    0x8c,  // s1s_start                     REG=0x3007 //0x7c
    0x93,  // s1s_end                       REG=0x3008 //0x9a
    0x05,  // s3_start                      REG=0x3009 //0x10
    0x3a,  // s3_end                        REG=0x300A //0x24
    0x10,  // cmp_en_start                  REG=0x300B
    0x02,  // clp_sl_start                  REG=0x300C //0x04
    0x3e,  // clp_sl_end                    REG=0x300D //0x26
    0x02,  // off_start                     REG=0x300E
    0x0e,  // rmp_en_start                  REG=0x300F
    0x46,  // tx_start                      REG=0x3010 //0x30
    0x64,  // tx_end                        REG=0x3011 //0x4e
    0x1E,  // stx_width                     REG=0x3012
    0x08,  // reg_3152_reserved             REG=0x3152
    0xFF,  //20,  // reg_315A_reserved             REG=0x315A //0x10
    0x00,  // analogue_gain_code_global_msb REG=0x0204
    0x80,  // analogue_gain_code_global_lsb REG=0x0205
    0x02,  // fine_integration_time         REG=0x0200
    0x03   // coarse_integration_time       REG=0x0202
  },
  { /*Snapshot*/
    0x06,  // pre_pll_clk_div               REG=0x0305
    0x00,  // pll_multiplier_msb            REG=0x0306
    0x88,  // pll_multiplier_lsb            REG=0x0307 //0x88
    0x0a,  // vt_pix_clk_div                REG=0x0301
    0x01,  // vt_sys_clk_div                REG=0x0303
    0x0a,  // op_pix_clk_div                REG=0x0309
    0x01,  // op_sys_clk_div                REG=0x030B
    0x0a,  // ccp_data_format_msb           REG=0x0112*
    0x0a,  // ccp_data_format_lsb           REG=0x0113*
    0x0a,  // x_output_size_msb             REG=0x034C
    0x30,  // x_output_size_lsb             REG=0x034D
    0x07,  // y_output_size_msb             REG=0x034E
    0xa8,  // y_output_size_lsb             REG=0x034F
            
    /* disable binning for snapshot */       
    0x01,  // x_even_inc                    REG=0x0381
    0x01,  // x_odd_inc                     REG=0x0383
    0x01,  // y_even_inc                    REG=0x0385
    0x01,  // y_odd_inc                     REG=0x0387
    0x00,  // binning_enable                REG=0x3014
                       
    0x07,  // frame_length_lines_msb        REG=0x0340
    0xb6,  // frame_length_lines_lsb        REG=0x0341
    0x0a,  // line_length_pck_msb           REG=0x0342
    0xac,  // line_length_pck_lsb           REG=0x0343
    0x81,  // shade_clk_enable              REG=0x30AC
    0x01,  // sel_ccp                       REG=0x30C4
    0x04,  // vpix                          REG=0x3024
    0x00,  // clamp_on                      REG=0x3015
    0x02,  // offset                        REG=0x307E
    0x03,  // ld_start                      REG=0x3000
    0x94,  // ld_end                        REG=0x3001 //0x9c
    0x02,  // sl_start                      REG=0x3002
    0x95,  // sl_end                        REG=0x3003 //0x9e
    0x04,  // rx_start                      REG=0x3004 //0x05
    0x05,  // s1_start                      REG=0x3005 //0x0f
    0x3c,  // s1_end                        REG=0x3006 //0x24
    0x8c,  // s1s_start                     REG=0x3007 //0x7c
    0x93,  // s1s_end                       REG=0x3008 //0x9a
    0x05,  // s3_start                      REG=0x3009 //0x10
    0x3a,  // s3_end                        REG=0x300A //0x24
    0x10,  // cmp_en_start                  REG=0x300B
    0x02,  // clp_sl_start                  REG=0x300C //0x04
    0x3e,  // clp_sl_end                    REG=0x300D //0x26
    0x02,  // off_start                     REG=0x300E
    0x0e,  // rmp_en_start                  REG=0x300F
    0x46,  // tx_start                      REG=0x3010 //0x30
    0x64,  // tx_end                        REG=0x3011 //0x4e
    0x1E,  // stx_width                     REG=0x3012
    0x08,  // reg_3152_reserved             REG=0x3152
    0xFF,  //20,  // reg_315A_reserved             REG=0x315A //0x10
    0x00,  // analogue_gain_code_global_msb REG=0x0204
    0x80,  // analogue_gain_code_global_lsb REG=0x0205
    0x02,  // fine_integration_time         REG=0x0200
    0x03   // coarse_integration_time       REG=0x0202
  }
};
/***********************************************************
* definition
************************************************************/
#define S5K3E2FX_SS5M0_RESET_DELAY_MSECS    66 
#define S5K3E2FX_SS5M0_DEFAULT_CLOCK_RATE   24000000 
#define S5K3E2FX_SS5M0_HRZ_FULL_BLK_PIXELS   124
#define S5K3E2FX_SS5M0_VER_FULL_BLK_LINES     14
#define S5K3E2FX_SS5M0_HRZ_QTR_BLK_PIXELS   1436
#define S5K3E2FX_SS5M0_VER_QTR_BLK_LINES      18
#define S5K3E2FX_SS5M0_OFFSET     4
#define S5K3E2FX_SS5M0_MAX_SNAPSHOT_EXPOSURE_LINE_COUNT 3961

/***********************************************************
* sensor register define
************************************************************/
#define S5K3E2FX_SS5M0_REG_MODEL_ID   0x0000
#define S5K3E2FX_SS5M0_MODEL_ID       0x3E2F

/* PLL Registers */
#define REG_PRE_PLL_CLK_DIV           0x0305
#define REG_PLL_MULTIPLIER_MSB        0x0306
#define REG_PLL_MULTIPLIER_LSB        0x0307
#define REG_VT_PIX_CLK_DIV            0x0301
#define REG_VT_SYS_CLK_DIV            0x0303
#define REG_OP_PIX_CLK_DIV            0x0309
#define REG_OP_SYS_CLK_DIV            0x030B

/* Data Format Registers */
#define REG_CCP_DATA_FORMAT_MSB       0x0112
#define REG_CCP_DATA_FORMAT_LSB       0x0113

/* Output Size */
#define REG_X_OUTPUT_SIZE_MSB         0x034C
#define REG_X_OUTPUT_SIZE_LSB         0x034D
#define REG_Y_OUTPUT_SIZE_MSB         0x034E
#define REG_Y_OUTPUT_SIZE_LSB         0x034F

/* Binning */
#define REG_X_EVEN_INC                0x0381
#define REG_X_ODD_INC                 0x0383 
#define REG_Y_EVEN_INC                0x0385 
#define REG_Y_ODD_INC                 0x0387 
/*Reserved register */
#define REG_BINNING_ENABLE            0x3014

/* Frame Fotmat */
#define REG_FRAME_LENGTH_LINES_MSB    0x0340
#define REG_FRAME_LENGTH_LINES_LSB    0x0341
#define REG_LINE_LENGTH_PCK_MSB       0x0342
#define REG_LINE_LENGTH_PCK_LSB       0x0343

/* MSR setting */
/* Reserved registers */
#define REG_SHADE_CLK_ENABLE          0x30AC
#define REG_SEL_CCP                   0x30C4
#define REG_VPIX                      0x3024
#define REG_CLAMP_ON                  0x3015
#define REG_OFFSET                    0x307E

/* CDS timing settings */
/* Reserved registers */
#define REG_LD_START                  0x3000
#define REG_LD_END                    0x3001
#define REG_SL_START                  0x3002
#define REG_SL_END                    0x3003
#define REG_RX_START                  0x3004
#define REG_S1_START                  0x3005
#define REG_S1_END                    0x3006
#define REG_S1S_START                 0x3007
#define REG_S1S_END                   0x3008
#define REG_S3_START                  0x3009
#define REG_S3_END                    0x300A
#define REG_CMP_EN_START              0x300B
#define REG_CLP_SL_START              0x300C
#define REG_CLP_SL_END                0x300D
#define REG_OFF_START                 0x300E
#define REG_RMP_EN_START              0x300F
#define REG_TX_START                  0x3010
#define REG_TX_END                    0x3011
#define REG_STX_WIDTH                 0x3012     
#define REG_TYPE1_AF_ENABLE           0x3130  
#define DRIVER_ENABLED              0x0001
#define AUTO_START_ENABLED          0x0010
#define REG_NEW_POSITION              0x3131
#define REG_3152_RESERVED             0x3152
#define REG_315A_RESERVED             0x315A
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB 0x0204
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB 0x0205
#define REG_FINE_INTEGRATION_TIME         0x0200
#define REG_COARSE_INTEGRATION_TIME       0x0202
#define REG_COARSE_INTEGRATION_TIME_LSB   0x0203

/* Mode select register */
#define S5K3E2FX_REG_MODE_SELECT      0x0100
#define S5K3E2FX_MODE_SELECT_STREAM     0x01   /* start streaming */ 
#define S5K3E2FX_MODE_SELECT_SW_STANDBY 0x00   /* software standby */ 
#define S5K3E2FX_REG_SOFTWARE_RESET   0x0103
#define S5K3E2FX_SOFTWARE_RESET         0x01
#define REG_TEST_PATTERN_MODE         0x0601

static int rc;
#define CLK_GET(clk) do {						\
	if (!clk) {							\
		clk = clk_get(NULL, #clk);				\
		printk(KERN_INFO 					\
			"s5k3e2fx: clk_get(%s): %p\n", #clk, clk);	\
	}								\
} while(0)


#define CLK_DISABLE_AND_PUT(clk) do {					\
	if (clk) {							\
		if (clk##_enabled) {					\
			printk(KERN_INFO "s5k3e2fx: disabling "#clk"\n");\
			clk_disable(clk);				\
			clk##_enabled = 0;				\
		}							\
		printk(KERN_INFO 					\
			"s5k3e2fx: clk_put(%s): %p\n", #clk, clk);	\
		clk_put(clk);						\
		clk = NULL; 						\
	}								\
} while(0)

#define CHECK() ({ 							\
	if (!mdc_clk_enabled || !vfe_mdc_clk_enabled) { 		\
		printk(KERN_ERR "s5k3e2fx error: one or more clocks"	\
			" are NULL.\n"); 				\
		rc = -EIO; 						\
	} 								\
	!rc; })

/***********************************************************
* VFE CAMIF setting
************************************************************/

#define out_dword(addr, val) \
	(*((volatile unsigned long  *)(addr)) = ((unsigned long)(val)))

#define out_dword_masked_ns(io, mask, val, current_reg_content)	    \
  (void) out_dword(io, ((current_reg_content & (uint32_t)(~(mask))) | \
			 ((uint32_t)((val) & (mask)))))

#define __inpdw(port) (*((volatile uint32_t *) (port)))
#define in_dword_masked(addr, mask) (__inpdw(addr) & (uint32_t)mask )

#define HWIO_MDDI_CAMIF_CFG_ADDR MSM_MDC_BASE
#define HWIO_MDDI_CAMIF_CFG_RMSK 0x1fffff
#define HWIO_MDDI_CAMIF_CFG_IN \
  in_dword_masked(HWIO_MDDI_CAMIF_CFG_ADDR, HWIO_MDDI_CAMIF_CFG_RMSK)

#define HWIO_MDDI_CAMIF_CFG_OUTM(m,v) \
  out_dword_masked_ns(HWIO_MDDI_CAMIF_CFG_ADDR,m,v,HWIO_MDDI_CAMIF_CFG_IN);
#define __msmhwio_outm(hwiosym, mask, val) HWIO_##hwiosym##_OUTM(mask, val)
#define HWIO_OUTM(hwiosym, mask, val) __msmhwio_outm(hwiosym, mask, val)

#define HWIO_MDDI_CAMIF_CFG_CAM_SEL_BMSK 0x2
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_SRC_SEL_BMSK 0x60000
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_INVERT_BMSK 0x80000
#define HWIO_MDDI_CAMIF_CFG_CAM_PAD_REG_SW_RESET_BMSK 0x100000

#define HWIO_MDDI_CAMIF_CFG_CAM_SEL_SHFT 0x1
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_SRC_SEL_SHFT 0x11
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_INVERT_SHFT 0x13
#define HWIO_MDDI_CAMIF_CFG_CAM_PAD_REG_SW_RESET_SHFT 0x14

#define __msmhwio_shft(hwio_regsym, hwio_fldsym) HWIO_##hwio_regsym##_##hwio_fldsym##_SHFT
#define HWIO_SHFT(hwio_regsym, hwio_fldsym) __msmhwio_shft(hwio_regsym, hwio_fldsym)

#define __msmhwio_fmsk(hwio_regsym, hwio_fldsym) HWIO_##hwio_regsym##_##hwio_fldsym##_BMSK
#define HWIO_FMSK(hwio_regsym, hwio_fldsym) __msmhwio_fmsk(hwio_regsym, hwio_fldsym)

#define HWIO_APPS_RESET_ADDR (MSM_CLK_CTL_BASE + 0x00000210)
#define HWIO_APPS_RESET_RMSK 0x1fff
#define HWIO_APPS_RESET_VFE_BMSK 1
#define HWIO_APPS_RESET_VFE_SHFT 0 
#define HWIO_APPS_RESET_IN in_dword_masked(HWIO_APPS_RESET_ADDR, HWIO_APPS_RESET_RMSK)
#define HWIO_APPS_RESET_OUTM(m,v) out_dword_masked_ns(HWIO_APPS_RESET_ADDR,m,v,HWIO_APPS_RESET_IN)



/***********************************************************
* privite function variable
************************************************************/
/***********************************************************
* I2C read write
************************************************************/
static int s5k3e2fx_i2c_rx_data(char* rxData, int length)
{
	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = pclient_i2c->addr,
			.flags = 0,      
			.len = 2,
			.buf = rxData,
		},
		{
			.addr = pclient_i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(pclient_i2c->adapter, msgs, 2);
	if (rc < 0) {
		printk(KERN_ERR "s5k3e2fx: s5k3e2fx_i2c_rx_data error %d\n", rc);
		return rc;
	}
	return 0;
}

int s5k3e2fx_i2c_tx_data(char* txData, int length)
{
	int rc; 
	struct i2c_msg msg[] = {
		{
			.addr = pclient_i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,		
		},
	};
    
	rc = i2c_transfer(pclient_i2c->adapter, msg, 1);
	if (rc < 0) {
		printk(KERN_ERR "s5k3e2fx: s5k3e2fx_i2c_tx_data error %d\n", rc);
		return rc;
	}
	return 0;
}

/***********************************************************
* sensor i2c write
************************************************************/

int hero_s5k3e2fx_i2c_write(unsigned short waddr, unsigned short wdata)
{
	int rc;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0x00FF);
	rc = s5k3e2fx_i2c_tx_data(buf, 3);

	if(rc < 0)
	    printk(KERN_ERR "s5k3e2fx: txdata error %d add:0x%02x data:0x%02x\n",
			rc, waddr, wdata);
	return rc;
}

/***********************************************************
* sensor i2c read
************************************************************/
int hero_s5k3e2fx_i2c_read(unsigned short u_addr, unsigned short *pu_data)
{
	int rc;
	unsigned char buf[2];

	buf[0] = (u_addr & 0xFF00)>>8;
	buf[1] = (u_addr & 0x00FF);
	rc = s5k3e2fx_i2c_rx_data(buf, 2);
	if (!rc)
		*pu_data = buf[0]<<8 | buf[1];
	else printk(KERN_ERR "s5k3e2fx: i2c read failed\n");
	return rc;	
}

/***********************************************************
* enable VFE clk
************************************************************/
static int hero_msm_camio_vfe_clk_enable(void)
{
	CLK_GET(vfe_clk);
	if (vfe_clk && !vfe_clk_enabled) {
		vfe_clk_enabled = !clk_enable(vfe_clk);
		printk(KERN_INFO "s5k3e2fx: enable vfe_clk\n");
	}
	return vfe_clk_enabled ? 0 : -EIO;
}



/***********************************************************
* Samsung sensor lens sharding setting
************************************************************/
static void hero_s5k3e2fx_i2c_sensor_lens_sharding(void){

	//Kevin add lens correction settings  20090505 start
	hero_s5k3e2fx_i2c_write(0x3200,0x00);
	hero_s5k3e2fx_i2c_write(0x3201,0x7d);
	hero_s5k3e2fx_i2c_write(0x3202,0x51);
	hero_s5k3e2fx_i2c_write(0x3203,0x0f);
	hero_s5k3e2fx_i2c_write(0x3204,0xe4);
	hero_s5k3e2fx_i2c_write(0x3205,0x9d);
	hero_s5k3e2fx_i2c_write(0x3206,0x00);
	hero_s5k3e2fx_i2c_write(0x3207,0x11);
	hero_s5k3e2fx_i2c_write(0x3208,0xd2);
	hero_s5k3e2fx_i2c_write(0x3209,0x0f);
	hero_s5k3e2fx_i2c_write(0x320a,0xfa);
	hero_s5k3e2fx_i2c_write(0x320b,0x94);
	hero_s5k3e2fx_i2c_write(0x320c,0x0f);
	hero_s5k3e2fx_i2c_write(0x320d,0xff);
	hero_s5k3e2fx_i2c_write(0x320e,0xd9);
	hero_s5k3e2fx_i2c_write(0x320f,0x00);
	hero_s5k3e2fx_i2c_write(0x3210,0x05);
	hero_s5k3e2fx_i2c_write(0x3211,0xc3);
	hero_s5k3e2fx_i2c_write(0x3212,0x0f);
	hero_s5k3e2fx_i2c_write(0x3213,0xd8);
	hero_s5k3e2fx_i2c_write(0x3214,0xb0);
	hero_s5k3e2fx_i2c_write(0x3215,0x00);
	hero_s5k3e2fx_i2c_write(0x3216,0x09);
	hero_s5k3e2fx_i2c_write(0x3217,0xde);
	hero_s5k3e2fx_i2c_write(0x3218,0x0f);
	hero_s5k3e2fx_i2c_write(0x3219,0xf8);
	hero_s5k3e2fx_i2c_write(0x321a,0x6f);
	hero_s5k3e2fx_i2c_write(0x321b,0x00);
	hero_s5k3e2fx_i2c_write(0x321c,0x03);
	hero_s5k3e2fx_i2c_write(0x321d,0xf2);
	hero_s5k3e2fx_i2c_write(0x321e,0x0f);
	hero_s5k3e2fx_i2c_write(0x321f,0xff);
	hero_s5k3e2fx_i2c_write(0x3220,0x23);
	hero_s5k3e2fx_i2c_write(0x3221,0x0f);
	hero_s5k3e2fx_i2c_write(0x3222,0xfa);
	hero_s5k3e2fx_i2c_write(0x3223,0x20);
	hero_s5k3e2fx_i2c_write(0x3224,0x00);
	hero_s5k3e2fx_i2c_write(0x3225,0x1d);
	hero_s5k3e2fx_i2c_write(0x3226,0xf0);
	hero_s5k3e2fx_i2c_write(0x3227,0x0f);
	hero_s5k3e2fx_i2c_write(0x3228,0xf4);
	hero_s5k3e2fx_i2c_write(0x3229,0x52);
	hero_s5k3e2fx_i2c_write(0x322a,0x00);
	hero_s5k3e2fx_i2c_write(0x322b,0x04);
	hero_s5k3e2fx_i2c_write(0x322c,0x16);
	hero_s5k3e2fx_i2c_write(0x322d,0x00);
	hero_s5k3e2fx_i2c_write(0x322e,0x01);
	hero_s5k3e2fx_i2c_write(0x322f,0x82);
	hero_s5k3e2fx_i2c_write(0x3230,0x0f);
	hero_s5k3e2fx_i2c_write(0x3231,0xff);
	hero_s5k3e2fx_i2c_write(0x3232,0xec);
	hero_s5k3e2fx_i2c_write(0x3233,0x00);
	hero_s5k3e2fx_i2c_write(0x3234,0x07);
	hero_s5k3e2fx_i2c_write(0x3235,0x44);
	hero_s5k3e2fx_i2c_write(0x3236,0x0f);
	hero_s5k3e2fx_i2c_write(0x3237,0xf0);
	hero_s5k3e2fx_i2c_write(0x3238,0x20);
	hero_s5k3e2fx_i2c_write(0x3239,0x00);
	hero_s5k3e2fx_i2c_write(0x323a,0x09);
	hero_s5k3e2fx_i2c_write(0x323b,0x09);
	hero_s5k3e2fx_i2c_write(0x323c,0x00);
	hero_s5k3e2fx_i2c_write(0x323d,0x03);
	hero_s5k3e2fx_i2c_write(0x323e,0xb7);
	hero_s5k3e2fx_i2c_write(0x323f,0x0f);
	hero_s5k3e2fx_i2c_write(0x3240,0xf7);
	hero_s5k3e2fx_i2c_write(0x3241,0x1e);
	hero_s5k3e2fx_i2c_write(0x3242,0x0f);
	hero_s5k3e2fx_i2c_write(0x3243,0xfd);
	hero_s5k3e2fx_i2c_write(0x3244,0x69);
	hero_s5k3e2fx_i2c_write(0x3245,0x0f);
	hero_s5k3e2fx_i2c_write(0x3246,0xfc);
	hero_s5k3e2fx_i2c_write(0x3247,0xa7);
	hero_s5k3e2fx_i2c_write(0x3248,0x00);
	hero_s5k3e2fx_i2c_write(0x3249,0x07);
	hero_s5k3e2fx_i2c_write(0x324a,0x85);
	hero_s5k3e2fx_i2c_write(0x324b,0x0f);
	hero_s5k3e2fx_i2c_write(0x324c,0xfd);
	hero_s5k3e2fx_i2c_write(0x324d,0xc4);
	hero_s5k3e2fx_i2c_write(0x324e,0x0f);
	hero_s5k3e2fx_i2c_write(0x324f,0xf6);
	hero_s5k3e2fx_i2c_write(0x3250,0xd3);
	hero_s5k3e2fx_i2c_write(0x3251,0x00);
	hero_s5k3e2fx_i2c_write(0x3252,0x0b);
	hero_s5k3e2fx_i2c_write(0x3253,0xca);
	hero_s5k3e2fx_i2c_write(0x3254,0x00);
	hero_s5k3e2fx_i2c_write(0x3255,0x04);
	hero_s5k3e2fx_i2c_write(0x3256,0xe0);
	hero_s5k3e2fx_i2c_write(0x3257,0x0f);
	hero_s5k3e2fx_i2c_write(0x3258,0xff);
	hero_s5k3e2fx_i2c_write(0x3259,0xf1);
	hero_s5k3e2fx_i2c_write(0x325a,0x00);
	hero_s5k3e2fx_i2c_write(0x325b,0x00);
	hero_s5k3e2fx_i2c_write(0x325c,0xa8);
	hero_s5k3e2fx_i2c_write(0x325d,0x0f);
	hero_s5k3e2fx_i2c_write(0x325e,0xff);
	hero_s5k3e2fx_i2c_write(0x325f,0x7e);
	hero_s5k3e2fx_i2c_write(0x3260,0x00);
	hero_s5k3e2fx_i2c_write(0x3261,0x02);
	hero_s5k3e2fx_i2c_write(0x3262,0xfa);
	hero_s5k3e2fx_i2c_write(0x3263,0x0f);
	hero_s5k3e2fx_i2c_write(0x3264,0xfd);
	hero_s5k3e2fx_i2c_write(0x3265,0x89);
	hero_s5k3e2fx_i2c_write(0x3266,0x0f);
	hero_s5k3e2fx_i2c_write(0x3267,0xfb);
	hero_s5k3e2fx_i2c_write(0x3268,0x22);
	hero_s5k3e2fx_i2c_write(0x3269,0x0f);
	hero_s5k3e2fx_i2c_write(0x326a,0xff);
	hero_s5k3e2fx_i2c_write(0x326b,0x51);
	hero_s5k3e2fx_i2c_write(0x326c,0x00);
	hero_s5k3e2fx_i2c_write(0x326d,0x7d);
	hero_s5k3e2fx_i2c_write(0x326e,0xdf);
	hero_s5k3e2fx_i2c_write(0x326f,0x0f);
	hero_s5k3e2fx_i2c_write(0x3270,0xe3);
	hero_s5k3e2fx_i2c_write(0x3271,0x36);
	hero_s5k3e2fx_i2c_write(0x3272,0x00);
	hero_s5k3e2fx_i2c_write(0x3273,0x12);
	hero_s5k3e2fx_i2c_write(0x3274,0x7d);
	hero_s5k3e2fx_i2c_write(0x3275,0x0f);
	hero_s5k3e2fx_i2c_write(0x3276,0xfa);
	hero_s5k3e2fx_i2c_write(0x3277,0xd9);
	hero_s5k3e2fx_i2c_write(0x3278,0x0f);
	hero_s5k3e2fx_i2c_write(0x3279,0xff);
	hero_s5k3e2fx_i2c_write(0x327a,0x88);
	hero_s5k3e2fx_i2c_write(0x327b,0x00);
	hero_s5k3e2fx_i2c_write(0x327c,0x05);
	hero_s5k3e2fx_i2c_write(0x327d,0xe7);
	hero_s5k3e2fx_i2c_write(0x327e,0x0f);
	hero_s5k3e2fx_i2c_write(0x327f,0xdc);
	hero_s5k3e2fx_i2c_write(0x3280,0x20);
	hero_s5k3e2fx_i2c_write(0x3281,0x00);
	hero_s5k3e2fx_i2c_write(0x3282,0x07);
	hero_s5k3e2fx_i2c_write(0x3283,0x28);
	hero_s5k3e2fx_i2c_write(0x3284,0x0f);
	hero_s5k3e2fx_i2c_write(0x3285,0xfd);
	hero_s5k3e2fx_i2c_write(0x3286,0x93);
	hero_s5k3e2fx_i2c_write(0x3287,0x0f);
	hero_s5k3e2fx_i2c_write(0x3288,0xfe);
	hero_s5k3e2fx_i2c_write(0x3289,0xe0);
	hero_s5k3e2fx_i2c_write(0x328a,0x00);
	hero_s5k3e2fx_i2c_write(0x328b,0x00);
	hero_s5k3e2fx_i2c_write(0x328c,0x25);
	hero_s5k3e2fx_i2c_write(0x328d,0x0f);
	hero_s5k3e2fx_i2c_write(0x328e,0xfc);
	hero_s5k3e2fx_i2c_write(0x328f,0x11);
	hero_s5k3e2fx_i2c_write(0x3290,0x00);
	hero_s5k3e2fx_i2c_write(0x3291,0x15);
	hero_s5k3e2fx_i2c_write(0x3292,0x09);
	hero_s5k3e2fx_i2c_write(0x3293,0x0f);
	hero_s5k3e2fx_i2c_write(0x3294,0xfb);
	hero_s5k3e2fx_i2c_write(0x3295,0x0f);
	hero_s5k3e2fx_i2c_write(0x3296,0x0f);
	hero_s5k3e2fx_i2c_write(0x3297,0xfb);
	hero_s5k3e2fx_i2c_write(0x3298,0xb9);
	hero_s5k3e2fx_i2c_write(0x3299,0x00);
	hero_s5k3e2fx_i2c_write(0x329a,0x05);
	hero_s5k3e2fx_i2c_write(0x329b,0x21);
	hero_s5k3e2fx_i2c_write(0x329c,0x00);
	hero_s5k3e2fx_i2c_write(0x329d,0x05);
	hero_s5k3e2fx_i2c_write(0x329e,0x53);
	hero_s5k3e2fx_i2c_write(0x329f,0x0f);
	hero_s5k3e2fx_i2c_write(0x32a0,0xff);
	hero_s5k3e2fx_i2c_write(0x32a1,0x95);
	hero_s5k3e2fx_i2c_write(0x32a2,0x0f);
	hero_s5k3e2fx_i2c_write(0x32a3,0xfb);
	hero_s5k3e2fx_i2c_write(0x32a4,0x5f);
	hero_s5k3e2fx_i2c_write(0x32a5,0x00);
	hero_s5k3e2fx_i2c_write(0x32a6,0x00);
	hero_s5k3e2fx_i2c_write(0x32a7,0xa1);
	hero_s5k3e2fx_i2c_write(0x32a8,0x00);
	hero_s5k3e2fx_i2c_write(0x32a9,0x0d);
	hero_s5k3e2fx_i2c_write(0x32aa,0x0e);
	hero_s5k3e2fx_i2c_write(0x32ab,0x0f);
	hero_s5k3e2fx_i2c_write(0x32ac,0xf5);
	hero_s5k3e2fx_i2c_write(0x32ad,0x14);
	hero_s5k3e2fx_i2c_write(0x32ae,0x0f);
	hero_s5k3e2fx_i2c_write(0x32af,0xf5);
	hero_s5k3e2fx_i2c_write(0x32b0,0x0a);
	hero_s5k3e2fx_i2c_write(0x32b1,0x00);
	hero_s5k3e2fx_i2c_write(0x32b2,0x05);
	hero_s5k3e2fx_i2c_write(0x32b3,0xf7);
	hero_s5k3e2fx_i2c_write(0x32b4,0x00);
	hero_s5k3e2fx_i2c_write(0x32b5,0x04);
	hero_s5k3e2fx_i2c_write(0x32b6,0x65);
	hero_s5k3e2fx_i2c_write(0x32b7,0x0f);
	hero_s5k3e2fx_i2c_write(0x32b8,0xfe);
	hero_s5k3e2fx_i2c_write(0x32b9,0x4f);
	hero_s5k3e2fx_i2c_write(0x32ba,0x0f);
	hero_s5k3e2fx_i2c_write(0x32bb,0xf6);
	hero_s5k3e2fx_i2c_write(0x32bc,0x6a);
	hero_s5k3e2fx_i2c_write(0x32bd,0x00);
	hero_s5k3e2fx_i2c_write(0x32be,0x08);
	hero_s5k3e2fx_i2c_write(0x32bf,0x22);
	hero_s5k3e2fx_i2c_write(0x32c0,0x00);
	hero_s5k3e2fx_i2c_write(0x32c1,0x0b);
	hero_s5k3e2fx_i2c_write(0x32c2,0x87);
	hero_s5k3e2fx_i2c_write(0x32c3,0x0f);
	hero_s5k3e2fx_i2c_write(0x32c4,0xfc);
	hero_s5k3e2fx_i2c_write(0x32c5,0x2f);
	hero_s5k3e2fx_i2c_write(0x32c6,0x0f);
	hero_s5k3e2fx_i2c_write(0x32c7,0xfc);
	hero_s5k3e2fx_i2c_write(0x32c8,0x3f);
	hero_s5k3e2fx_i2c_write(0x32c9,0x00);
	hero_s5k3e2fx_i2c_write(0x32ca,0x06);
	hero_s5k3e2fx_i2c_write(0x32cb,0x5d);
	hero_s5k3e2fx_i2c_write(0x32cc,0x0f);
	hero_s5k3e2fx_i2c_write(0x32cd,0xfa);
	hero_s5k3e2fx_i2c_write(0x32ce,0xb0);
	hero_s5k3e2fx_i2c_write(0x32cf,0x00);
	hero_s5k3e2fx_i2c_write(0x32d0,0x05);
	hero_s5k3e2fx_i2c_write(0x32d1,0xa5);
	hero_s5k3e2fx_i2c_write(0x32d2,0x0f);
	hero_s5k3e2fx_i2c_write(0x32d3,0xf7);
	hero_s5k3e2fx_i2c_write(0x32d4,0x70);
	hero_s5k3e2fx_i2c_write(0x32d5,0x0f);
	hero_s5k3e2fx_i2c_write(0x32d6,0xfd);
	hero_s5k3e2fx_i2c_write(0x32d7,0xda);
	hero_s5k3e2fx_i2c_write(0x32d8,0x00);
	hero_s5k3e2fx_i2c_write(0x32d9,0x6c);
	hero_s5k3e2fx_i2c_write(0x32da,0x4b);
	hero_s5k3e2fx_i2c_write(0x32db,0x0f);
	hero_s5k3e2fx_i2c_write(0x32dc,0xec);
	hero_s5k3e2fx_i2c_write(0x32dd,0x0e);
	hero_s5k3e2fx_i2c_write(0x32de,0x00);
	hero_s5k3e2fx_i2c_write(0x32df,0x09);
	hero_s5k3e2fx_i2c_write(0x32e0,0x48);
	hero_s5k3e2fx_i2c_write(0x32e1,0x00);
	hero_s5k3e2fx_i2c_write(0x32e2,0x02);
	hero_s5k3e2fx_i2c_write(0x32e3,0x22);
	hero_s5k3e2fx_i2c_write(0x32e4,0x0f);
	hero_s5k3e2fx_i2c_write(0x32e5,0xfe);
	hero_s5k3e2fx_i2c_write(0x32e6,0x01);
	hero_s5k3e2fx_i2c_write(0x32e7,0x00);
	hero_s5k3e2fx_i2c_write(0x32e8,0x01);
	hero_s5k3e2fx_i2c_write(0x32e9,0x4d);
	hero_s5k3e2fx_i2c_write(0x32ea,0x0f);
	hero_s5k3e2fx_i2c_write(0x32eb,0xe2);
	hero_s5k3e2fx_i2c_write(0x32ec,0x69);
	hero_s5k3e2fx_i2c_write(0x32ed,0x00);
	hero_s5k3e2fx_i2c_write(0x32ee,0x05);
	hero_s5k3e2fx_i2c_write(0x32ef,0x90);
	hero_s5k3e2fx_i2c_write(0x32f0,0x00);
	hero_s5k3e2fx_i2c_write(0x32f1,0x02);
	hero_s5k3e2fx_i2c_write(0x32f2,0x67);
	hero_s5k3e2fx_i2c_write(0x32f3,0x0f);
	hero_s5k3e2fx_i2c_write(0x32f4,0xf7);
	hero_s5k3e2fx_i2c_write(0x32f5,0x54);
	hero_s5k3e2fx_i2c_write(0x32f6,0x00);
	hero_s5k3e2fx_i2c_write(0x32f7,0x02);
	hero_s5k3e2fx_i2c_write(0x32f8,0xd1);
	hero_s5k3e2fx_i2c_write(0x32f9,0x00);
	hero_s5k3e2fx_i2c_write(0x32fa,0x02);
	hero_s5k3e2fx_i2c_write(0x32fb,0x07);
	hero_s5k3e2fx_i2c_write(0x32fc,0x00);
	hero_s5k3e2fx_i2c_write(0x32fd,0x15);
	hero_s5k3e2fx_i2c_write(0x32fe,0xba);
	hero_s5k3e2fx_i2c_write(0x32ff,0x0f);
	hero_s5k3e2fx_i2c_write(0x3300,0xfa);
	hero_s5k3e2fx_i2c_write(0x3301,0x07);
	hero_s5k3e2fx_i2c_write(0x3302,0x0f);
	hero_s5k3e2fx_i2c_write(0x3303,0xfe);
	hero_s5k3e2fx_i2c_write(0x3304,0xb2);
	hero_s5k3e2fx_i2c_write(0x3305,0x00);
	hero_s5k3e2fx_i2c_write(0x3306,0x06);
	hero_s5k3e2fx_i2c_write(0x3307,0x03);
	hero_s5k3e2fx_i2c_write(0x3308,0x00);
	hero_s5k3e2fx_i2c_write(0x3309,0x03);
	hero_s5k3e2fx_i2c_write(0x330a,0x48);
	hero_s5k3e2fx_i2c_write(0x330b,0x0f);
	hero_s5k3e2fx_i2c_write(0x330c,0xf9);
	hero_s5k3e2fx_i2c_write(0x330d,0xec);
	hero_s5k3e2fx_i2c_write(0x330e,0x0f);
	hero_s5k3e2fx_i2c_write(0x330f,0xf6);
	hero_s5k3e2fx_i2c_write(0x3310,0x61);
	hero_s5k3e2fx_i2c_write(0x3311,0x00);
	hero_s5k3e2fx_i2c_write(0x3312,0x04);
	hero_s5k3e2fx_i2c_write(0x3313,0x13);
	hero_s5k3e2fx_i2c_write(0x3314,0x00);
	hero_s5k3e2fx_i2c_write(0x3315,0x01);
	hero_s5k3e2fx_i2c_write(0x3316,0x72);
	hero_s5k3e2fx_i2c_write(0x3317,0x0f);
	hero_s5k3e2fx_i2c_write(0x3318,0xfb);
	hero_s5k3e2fx_i2c_write(0x3319,0x9b);
	hero_s5k3e2fx_i2c_write(0x331a,0x0f);
	hero_s5k3e2fx_i2c_write(0x331b,0xf9);
	hero_s5k3e2fx_i2c_write(0x331c,0x2d);
	hero_s5k3e2fx_i2c_write(0x331d,0x00);
	hero_s5k3e2fx_i2c_write(0x331e,0x06);
	hero_s5k3e2fx_i2c_write(0x331f,0xdb);
	hero_s5k3e2fx_i2c_write(0x3320,0x00);
	hero_s5k3e2fx_i2c_write(0x3321,0x04);
	hero_s5k3e2fx_i2c_write(0x3322,0xfb);
	hero_s5k3e2fx_i2c_write(0x3323,0x0f);
	hero_s5k3e2fx_i2c_write(0x3324,0xfe);
	hero_s5k3e2fx_i2c_write(0x3325,0x19);
	hero_s5k3e2fx_i2c_write(0x3326,0x0f);
	hero_s5k3e2fx_i2c_write(0x3327,0xfd);
	hero_s5k3e2fx_i2c_write(0x3328,0xe2);
	hero_s5k3e2fx_i2c_write(0x3329,0x00);
	hero_s5k3e2fx_i2c_write(0x332a,0x04);
	hero_s5k3e2fx_i2c_write(0x332b,0xc7);
	hero_s5k3e2fx_i2c_write(0x332c,0x00);
	hero_s5k3e2fx_i2c_write(0x332d,0x00);
	hero_s5k3e2fx_i2c_write(0x332e,0x8f);
	hero_s5k3e2fx_i2c_write(0x332f,0x00);
	hero_s5k3e2fx_i2c_write(0x3330,0x03);
	hero_s5k3e2fx_i2c_write(0x3331,0x84);
	hero_s5k3e2fx_i2c_write(0x3332,0x0f);
	hero_s5k3e2fx_i2c_write(0x3333,0xff);
	hero_s5k3e2fx_i2c_write(0x3334,0x23);
	hero_s5k3e2fx_i2c_write(0x3335,0x00);
	hero_s5k3e2fx_i2c_write(0x3336,0x03);
	hero_s5k3e2fx_i2c_write(0x3337,0x52);
	hero_s5k3e2fx_i2c_write(0x3338,0x0f);
	hero_s5k3e2fx_i2c_write(0x3339,0xfa);
	hero_s5k3e2fx_i2c_write(0x333a,0x88);
	hero_s5k3e2fx_i2c_write(0x333b,0x00);
	hero_s5k3e2fx_i2c_write(0x333c,0x05);
	hero_s5k3e2fx_i2c_write(0x333d,0xe0);
	hero_s5k3e2fx_i2c_write(0x333e,0x0f);
	hero_s5k3e2fx_i2c_write(0x333f,0xfe);
	hero_s5k3e2fx_i2c_write(0x3340,0xb9);
	hero_s5k3e2fx_i2c_write(0x3341,0x0f);
	hero_s5k3e2fx_i2c_write(0x3342,0xf7);
	hero_s5k3e2fx_i2c_write(0x3343,0x5e);
	hero_s5k3e2fx_i2c_write(0x3344,0x00);
	hero_s5k3e2fx_i2c_write(0x3345,0x7e);
	hero_s5k3e2fx_i2c_write(0x3346,0xc2);
	hero_s5k3e2fx_i2c_write(0x3347,0x0f);
	hero_s5k3e2fx_i2c_write(0x3348,0xe8);
	hero_s5k3e2fx_i2c_write(0x3349,0x8c);
	hero_s5k3e2fx_i2c_write(0x334a,0x00);
	hero_s5k3e2fx_i2c_write(0x334b,0x0b);
	hero_s5k3e2fx_i2c_write(0x334c,0xb7);
	hero_s5k3e2fx_i2c_write(0x334d,0x00);
	hero_s5k3e2fx_i2c_write(0x334e,0x01);
	hero_s5k3e2fx_i2c_write(0x334f,0x40);
	hero_s5k3e2fx_i2c_write(0x3350,0x0f);
	hero_s5k3e2fx_i2c_write(0x3351,0xfc);
	hero_s5k3e2fx_i2c_write(0x3352,0xc9);
	hero_s5k3e2fx_i2c_write(0x3353,0x00);
	hero_s5k3e2fx_i2c_write(0x3354,0x04);
	hero_s5k3e2fx_i2c_write(0x3355,0x3d);
	hero_s5k3e2fx_i2c_write(0x3356,0x0f);
	hero_s5k3e2fx_i2c_write(0x3357,0xd8);
	hero_s5k3e2fx_i2c_write(0x3358,0xf4);
	hero_s5k3e2fx_i2c_write(0x3359,0x00);
	hero_s5k3e2fx_i2c_write(0x335a,0x07);
	hero_s5k3e2fx_i2c_write(0x335b,0x31);
	hero_s5k3e2fx_i2c_write(0x335c,0x0f);
	hero_s5k3e2fx_i2c_write(0x335d,0xfe);
	hero_s5k3e2fx_i2c_write(0x335e,0xe5);
	hero_s5k3e2fx_i2c_write(0x335f,0x0f);
	hero_s5k3e2fx_i2c_write(0x3360,0xfc);
	hero_s5k3e2fx_i2c_write(0x3361,0x37);
	hero_s5k3e2fx_i2c_write(0x3362,0x00);
	hero_s5k3e2fx_i2c_write(0x3363,0x01);
	hero_s5k3e2fx_i2c_write(0x3364,0x55);
	hero_s5k3e2fx_i2c_write(0x3365,0x0f);
	hero_s5k3e2fx_i2c_write(0x3366,0xfe);
	hero_s5k3e2fx_i2c_write(0x3367,0x2a);
	hero_s5k3e2fx_i2c_write(0x3368,0x00);
	hero_s5k3e2fx_i2c_write(0x3369,0x18);
	hero_s5k3e2fx_i2c_write(0x336a,0x6d);
	hero_s5k3e2fx_i2c_write(0x336b,0x0f);
	hero_s5k3e2fx_i2c_write(0x336c,0xf6);
	hero_s5k3e2fx_i2c_write(0x336d,0x5d);
	hero_s5k3e2fx_i2c_write(0x336e,0x0f);
	hero_s5k3e2fx_i2c_write(0x336f,0xfe);
	hero_s5k3e2fx_i2c_write(0x3370,0x9c);
	hero_s5k3e2fx_i2c_write(0x3371,0x00);
	hero_s5k3e2fx_i2c_write(0x3372,0x08);
	hero_s5k3e2fx_i2c_write(0x3373,0x24);
	hero_s5k3e2fx_i2c_write(0x3374,0x0f);
	hero_s5k3e2fx_i2c_write(0x3375,0xfd);
	hero_s5k3e2fx_i2c_write(0x3376,0xe4);
	hero_s5k3e2fx_i2c_write(0x3377,0x00);
	hero_s5k3e2fx_i2c_write(0x3378,0x03);
	hero_s5k3e2fx_i2c_write(0x3379,0x5d);
	hero_s5k3e2fx_i2c_write(0x337a,0x0f);
	hero_s5k3e2fx_i2c_write(0x337b,0xf8);
	hero_s5k3e2fx_i2c_write(0x337c,0x7d);
	hero_s5k3e2fx_i2c_write(0x337d,0x00);
	hero_s5k3e2fx_i2c_write(0x337e,0x09);
	hero_s5k3e2fx_i2c_write(0x337f,0x40);
	hero_s5k3e2fx_i2c_write(0x3380,0x00);
	hero_s5k3e2fx_i2c_write(0x3381,0x03);
	hero_s5k3e2fx_i2c_write(0x3382,0x0d);
	hero_s5k3e2fx_i2c_write(0x3383,0x0f);
	hero_s5k3e2fx_i2c_write(0x3384,0xf8);
	hero_s5k3e2fx_i2c_write(0x3385,0x37);
	hero_s5k3e2fx_i2c_write(0x3386,0x0f);
	hero_s5k3e2fx_i2c_write(0x3387,0xfd);
	hero_s5k3e2fx_i2c_write(0x3388,0x85);
	hero_s5k3e2fx_i2c_write(0x3389,0x0f);
	hero_s5k3e2fx_i2c_write(0x338a,0xfd);
	hero_s5k3e2fx_i2c_write(0x338b,0x9b);
	hero_s5k3e2fx_i2c_write(0x338c,0x00);
	hero_s5k3e2fx_i2c_write(0x338d,0x04);
	hero_s5k3e2fx_i2c_write(0x338e,0xee);
	hero_s5k3e2fx_i2c_write(0x338f,0x0f);
	hero_s5k3e2fx_i2c_write(0x3390,0xfb);
	hero_s5k3e2fx_i2c_write(0x3391,0x2a);
	hero_s5k3e2fx_i2c_write(0x3392,0x0f);
	hero_s5k3e2fx_i2c_write(0x3393,0xfc);
	hero_s5k3e2fx_i2c_write(0x3394,0xc2);
	hero_s5k3e2fx_i2c_write(0x3395,0x00);
	hero_s5k3e2fx_i2c_write(0x3396,0x03);
	hero_s5k3e2fx_i2c_write(0x3397,0xc4);
	hero_s5k3e2fx_i2c_write(0x3398,0x00);
	hero_s5k3e2fx_i2c_write(0x3399,0x04);
	hero_s5k3e2fx_i2c_write(0x339a,0x2e);
	hero_s5k3e2fx_i2c_write(0x339b,0x00);
	hero_s5k3e2fx_i2c_write(0x339c,0x05);
	hero_s5k3e2fx_i2c_write(0x339d,0x5e);
	hero_s5k3e2fx_i2c_write(0x339e,0x0f);
	hero_s5k3e2fx_i2c_write(0x339f,0xfd);
	hero_s5k3e2fx_i2c_write(0x33a0,0xde);
	hero_s5k3e2fx_i2c_write(0x33a1,0x00);
	hero_s5k3e2fx_i2c_write(0x33a2,0x02);
	hero_s5k3e2fx_i2c_write(0x33a3,0x12);
	hero_s5k3e2fx_i2c_write(0x33a4,0x0f);
	hero_s5k3e2fx_i2c_write(0x33a5,0xfd);
	hero_s5k3e2fx_i2c_write(0x33a6,0x56);
	hero_s5k3e2fx_i2c_write(0x33a7,0x00);
	hero_s5k3e2fx_i2c_write(0x33a8,0x05);
	hero_s5k3e2fx_i2c_write(0x33a9,0xab);
	hero_s5k3e2fx_i2c_write(0x33aa,0x0f);
	hero_s5k3e2fx_i2c_write(0x33ab,0xfa);
	hero_s5k3e2fx_i2c_write(0x33ac,0xbb);
	hero_s5k3e2fx_i2c_write(0x33ad,0x0f);
	hero_s5k3e2fx_i2c_write(0x33ae,0xfa);
	hero_s5k3e2fx_i2c_write(0x33af,0x34);
	hero_s5k3e2fx_i2c_write(0x309D,0x62);
	hero_s5k3e2fx_i2c_write(0x309d,0x22);
	//Kevin add lens correction settings  20090505 end

	return;
}

/***********************************************************
* disable and pull all clock
************************************************************/
static void hero_msm_camio_disable_put(void){
	CLK_DISABLE_AND_PUT(mdc_clk);
	CLK_DISABLE_AND_PUT(vfe_mdc_clk);
	CLK_DISABLE_AND_PUT(vfe_clk);
}
		

/***********************************************************
* Samsung init sensor sequence
************************************************************/
static void hero_s5k3e2fx_sensor_init(void)
{
	int ret;
	printk(KERN_INFO "s5k3e2fx: init\n");
	if (!pclient_i2c) 
		return;
	/*pull hi reset*/
	printk(KERN_INFO "s5k3e2fx: s5k3e2fx_register_init\n");
	ret = gpio_request(HERO_GPIO_CAM_RST_N, "s5k3e2fx");
	if (!ret) {
		gpio_direction_output(HERO_GPIO_CAM_RST_N, 1);
		printk(KERN_INFO "s5k3e2fx: camera sensor_reset set as 1\n");
	} else
		printk(KERN_ERR "s5k3e2fx error: request gpio %d failed: "
				"%d\n", HERO_GPIO_CAM_RST_N, ret);
	gpio_free(HERO_GPIO_CAM_RST_N);
	msleep(2);
	/* set mclk */
	ret = hero_msm_camio_clk_rate_set(24000000);
	if(ret < 0)
		printk(KERN_ERR "camio clk rate select error\n");
	msleep(2);

	/* enable clk */
	hero_msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	hero_msm_camio_clk_enable(CAMIO_MDC_CLK);

	/* reset CAMIF */
	hero_s5k3e2fx_camif_pad_reg_reset();

	/* enable gpio */
	config_hero_camera_on_gpios();

	/* delay 2 ms */
	msleep(2);
	printk(KERN_INFO "s5k3e2fx: camera sensor init sequence done\n");
}

/***********************************************************
* Samsung sensor suspend
************************************************************/
void hero_s5k3e2fx_sensor_deinit(void)
{
	printk(KERN_INFO "s5k3e2fx: camera sensor suspend sequence\n");
	if (!pclient_i2c) {
		return;
	}
	/*disable clk*/
	hero_msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
	hero_msm_camio_clk_disable(CAMIO_MDC_CLK);
	CLK_DISABLE_AND_PUT(vfe_clk); /* this matches clk_select(1) */
	/* disable gpios */
	config_hero_camera_off_gpios();
	printk(KERN_INFO "s5k3e2fx: camera sensor suspend sequence done\n");
}



/***********************************************************
* Samsung sensor camif app reset2
************************************************************/
void hero_s5k3e2fx_camif_reset2(void){
	if (CHECK()) {
		HWIO_OUTM (MDDI_CAMIF_CFG,
			HWIO_FMSK (MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
				1 << HWIO_SHFT (MDDI_CAMIF_CFG,
				CAM_PAD_REG_SW_RESET));
		msleep(10);
		HWIO_OUTM (MDDI_CAMIF_CFG,
			HWIO_FMSK (MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
			0 << HWIO_SHFT (MDDI_CAMIF_CFG,
			CAM_PAD_REG_SW_RESET));
		msleep(10);
	}
}

/***********************************************************
* Samsung sensor camif app reset
************************************************************/

int hero_s5k3e2fx_camif_app_reset(void){
	int rc;
	if (CHECK()) {
		rc = hero_camif_clk_select(1);
		if(rc < 0) {
			printk(KERN_ERR "s5k3e2fx error switching to internal clock\n");
			return rc;
		}
		HWIO_OUTM (APPS_RESET,
			HWIO_FMSK(APPS_RESET,VFE),
			1 << HWIO_SHFT(APPS_RESET,VFE));
			udelay(10);
		HWIO_OUTM (APPS_RESET,
			HWIO_FMSK(APPS_RESET,VFE),
			0 << HWIO_SHFT(APPS_RESET,VFE));
		udelay(10);
		rc = hero_camif_clk_select(0); /* external */
		if(rc < 0) {
			printk(KERN_ERR "s5k3e2fx error switching to external clock\n");
			return rc;
		}
	}
	return rc;
}

/***********************************************************
* Samsung sensor camif reset
************************************************************/

int hero_s5k3e2fx_camif_pad_reg_reset(void)
{
	int rc = hero_camif_clk_select(1);
	if(rc < 0) {
		printk(KERN_ERR "s5k3e2fx error switching to internal clock\n");
		return rc;
	}
	HWIO_OUTM (MDDI_CAMIF_CFG,
		HWIO_FMSK (MDDI_CAMIF_CFG, CAM_SEL) |
		HWIO_FMSK (MDDI_CAMIF_CFG, CAM_PCLK_SRC_SEL) |
		HWIO_FMSK (MDDI_CAMIF_CFG, CAM_PCLK_INVERT),
		1 << HWIO_SHFT (MDDI_CAMIF_CFG, CAM_SEL) |
		3 << HWIO_SHFT (MDDI_CAMIF_CFG, CAM_PCLK_SRC_SEL) |
		0 << HWIO_SHFT (MDDI_CAMIF_CFG, CAM_PCLK_INVERT));
	msleep(10);
	HWIO_OUTM (MDDI_CAMIF_CFG,
		HWIO_FMSK (MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
		1 << HWIO_SHFT (MDDI_CAMIF_CFG,
		CAM_PAD_REG_SW_RESET));
	msleep(10);
	HWIO_OUTM (MDDI_CAMIF_CFG,
		HWIO_FMSK (MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
		0 << HWIO_SHFT (MDDI_CAMIF_CFG,
		CAM_PAD_REG_SW_RESET));
	msleep(10);
	rc = hero_camif_clk_select(0); /* external */
	if(rc < 0) {
		printk(KERN_ERR "s5k3e2fx error switching to external clock\n");
		return rc;
	}

	return rc;
}



/***********************************************************
* Select clk source of CAMIF
************************************************************/
int hero_camif_clk_select(int internal)
{
	int rc = -EIO; 
	printk(KERN_INFO "s5k3e2fx: clk select %d\n", internal);
	CLK_GET(vfe_clk);
	if (vfe_clk != NULL) {
		extern int clk_set_flags(struct clk *clk, unsigned long flags);
		rc = clk_set_flags(vfe_clk, 0x00000100 << internal);
		if (!rc && internal) 
			rc = hero_msm_camio_vfe_clk_enable();
	}
	return rc;
}

/***********************************************************
* Enable CLK
************************************************************/
int hero_msm_camio_clk_enable (int clk_type)
{
	struct clk *clk = NULL;
	int *enabled = NULL;

	switch (clk_type) {
	case CAMIO_VFE_MDC_CLK:
		CLK_GET(vfe_mdc_clk);
		clk = vfe_mdc_clk;
		enabled = &vfe_mdc_clk_enabled;
		break;
	case CAMIO_MDC_CLK:
		CLK_GET(mdc_clk);
		clk = mdc_clk;
		enabled = &mdc_clk_enabled;
		break;
	default:
		break;
	}

	if (clk != NULL && !*enabled) {
		int rc = clk_enable(clk);
		*enabled = !rc;	
		return rc;
	}

	return -EINVAL; 
}

/***********************************************************
* Disable CLK
************************************************************/
int hero_msm_camio_clk_disable(int clk_type)
{
	int rc = 0;
	struct clk *clk = NULL;
	int *enabled = NULL;

	switch (clk_type) {
	case CAMIO_VFE_MDC_CLK:
		clk = vfe_mdc_clk;
		enabled = &vfe_mdc_clk_enabled;
		break;
	case CAMIO_MDC_CLK:
		clk = mdc_clk;
		enabled = &mdc_clk_enabled;
		break;
	case CAMIO_VFE_CLK:
		clk = vfe_clk;
		enabled = &vfe_clk_enabled;
	default:
		rc = -1;
		break;
	}

	if (clk != NULL && *enabled) {
		clk_disable(clk);
		*enabled = 0;
		return 0;
	}

	return -EINVAL;
}


/***********************************************************
* set VFE clk rate
************************************************************/
int hero_msm_camio_clk_rate_set(int rate)
{
	int rc = hero_msm_camio_vfe_clk_enable();
	if (!rc && vfe_clk_enabled)
		rc = clk_set_rate(vfe_clk, rate);
	return rc;
}


/***********************************************************
* Samsung sensor power up
************************************************************/
int hero_s5k3e2fx_power_up(void)
{
	printk(KERN_INFO "s5k3e2fx: power up\n");

	if (powered) {
		printk(KERN_INFO "s5k3e2fx: already powered up\n");
		return 0;
	}

/* 980515 Horng take off as optical team Kevin's request */
#if 0
	//becker change for SS sensor setting
	hero_s5k3e2fx_i2c_write(0x3150, 0x50);//Kevin add base on Samsung suggested settings 20090413
	//becker take this off for SS SW sequence
	msleep(5);
	hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_MODE_SELECT, S5K3E2FX_MODE_SELECT_STREAM);
#endif

	powered = 1;
	return 0;
}
/***********************************************************
* Samsung sensor power down
************************************************************/

int hero_s5k3e2fx_power_down(void)
{

	printk(KERN_INFO "s5k3e2fx: power down\n");

	if (!powered) {
		printk(KERN_INFO "s5k3e2fx: already powered down\n");
		return 0;
	}
	/*becker:for power safing*/
	hero_s5k3e2fx_i2c_write(0x3130,0x00);
	msleep(2);
	hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_MODE_SELECT, S5K3E2FX_MODE_SELECT_SW_STANDBY);
	/*becker change for SS sensor setting*/
	msleep(2);
	/*Kevin add base on Samsung suggested settings 20090413*/
	hero_s5k3e2fx_i2c_write(0x3150, 0x51);
	msleep(260);
	#if 0/*becker take this off for SS SW sequence*/
	msleep(S5K3E2FX_SS5M0_RESET_DELAY_MSECS);
	msleep((line_keep*2732/54400000)*1000 + 10); /* 2732=linelengthpck; 65600000=pclk; +10ms buffer;//Kevin modify 20090520 */
	#endif
	powered  = 0;
	
	return 0;
}


/***********************************************************
* Samsung sensor system suspend
************************************************************/

int hero_s5k3e2fx_suspend(void *client,pm_message_t mesg)
{
	return 0;
}


/***********************************************************
* Samsung sensor system resume
************************************************************/
int hero_s5k3e2fx_resume(void *client)
{
	return 0;
}

/***********************************************************
* Samsung init sensor setting 
************************************************************/
int hero_s5k3e2fx_sensor_setting(unsigned long arg)
{
	uint16_t total_lines_per_frame;
	uint32_t update = arg & 1;
	uint32_t rt = (arg & 2) >> 1;

	if (rt > 1 || update > 1) {
		printk(KERN_ERR "s5k3e2fx: invalid values %d of rt or %d of update\n",
			rt, update);
		return -EINVAL;
	}

	switch (update) {
	case CAMSENSOR_REG_UPDATE_PERIODIC: {
		uint16_t pclk_div_adj = arg >> 16;

		printk(KERN_INFO "CAMSENSOR_REG_UPDATE_PERIODIC (rt %d)\n", rt);
		
		if (!pclk_div_adj || pclk_div_adj > 2) {
			printk(KERN_ERR "s5k3e2fx: invalid value %d of pclk_div_adj\n",
					pclk_div_adj); 
			return -EINVAL;
		}

        	hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_MODE_SELECT, S5K3E2FX_MODE_SELECT_SW_STANDBY);
		/*Horng add 980420, the delay after software stand by depends on frame rate*/
		msleep((line_keep*2732/54400000)*1000 + 10); /*2732=linelengthpck; 65600000=pclk; +10ms buffer*/

		/* Data Format */
        	hero_s5k3e2fx_i2c_write(REG_CCP_DATA_FORMAT_MSB, s5k3e2fx_reg_pattern[rt].ccp_data_format_msb);
        	hero_s5k3e2fx_i2c_write(REG_CCP_DATA_FORMAT_LSB, s5k3e2fx_reg_pattern[rt].ccp_data_format_lsb);
       		 /*Output Size */
        	hero_s5k3e2fx_i2c_write(REG_X_OUTPUT_SIZE_MSB, s5k3e2fx_reg_pattern[rt].x_output_size_msb);
        	hero_s5k3e2fx_i2c_write(REG_X_OUTPUT_SIZE_LSB, s5k3e2fx_reg_pattern[rt].x_output_size_lsb);
        	hero_s5k3e2fx_i2c_write(REG_Y_OUTPUT_SIZE_MSB, s5k3e2fx_reg_pattern[rt].y_output_size_msb);
       	 	hero_s5k3e2fx_i2c_write(REG_Y_OUTPUT_SIZE_LSB, s5k3e2fx_reg_pattern[rt].y_output_size_lsb);
        	hero_s5k3e2fx_i2c_write(REG_X_EVEN_INC, s5k3e2fx_reg_pattern[rt].x_even_inc);
        	hero_s5k3e2fx_i2c_write(REG_X_ODD_INC, s5k3e2fx_reg_pattern[rt].x_odd_inc);
        	hero_s5k3e2fx_i2c_write(REG_Y_EVEN_INC, s5k3e2fx_reg_pattern[rt].y_even_inc);
        	hero_s5k3e2fx_i2c_write(REG_Y_ODD_INC, s5k3e2fx_reg_pattern[rt].y_odd_inc);
        	hero_s5k3e2fx_i2c_write(REG_BINNING_ENABLE, s5k3e2fx_reg_pattern[rt].binning_enable);
		/* Frame format */
		/* Update registers with correct frame_length_line value for AFR */
		total_lines_per_frame = (uint16_t)((s5k3e2fx_reg_pattern[rt].frame_length_lines_msb << 8) & 0xFF00)
		                            + s5k3e2fx_reg_pattern[rt].frame_length_lines_lsb;
		total_lines_per_frame = total_lines_per_frame * fps_divider;
		hero_s5k3e2fx_i2c_write(REG_FRAME_LENGTH_LINES_MSB, (total_lines_per_frame & 0xFF00) >> 8);
		hero_s5k3e2fx_i2c_write(REG_FRAME_LENGTH_LINES_LSB, (total_lines_per_frame & 0x00FF));
		hero_s5k3e2fx_i2c_write(REG_LINE_LENGTH_PCK_MSB, s5k3e2fx_reg_pattern[rt].line_length_pck_msb);
		hero_s5k3e2fx_i2c_write(REG_LINE_LENGTH_PCK_LSB, s5k3e2fx_reg_pattern[rt].line_length_pck_lsb);
    		/* MSR setting */
        	hero_s5k3e2fx_i2c_write(REG_SHADE_CLK_ENABLE, s5k3e2fx_reg_pattern[rt].shade_clk_enable);
        	hero_s5k3e2fx_i2c_write(REG_SEL_CCP, s5k3e2fx_reg_pattern[rt].sel_ccp);
        	hero_s5k3e2fx_i2c_write(REG_VPIX, s5k3e2fx_reg_pattern[rt].vpix);
        	hero_s5k3e2fx_i2c_write(REG_CLAMP_ON, s5k3e2fx_reg_pattern[rt].clamp_on);
        	hero_s5k3e2fx_i2c_write(REG_OFFSET, s5k3e2fx_reg_pattern[rt].offset);
        	/* CDS timing setting */
        	hero_s5k3e2fx_i2c_write(REG_LD_START, s5k3e2fx_reg_pattern[rt].ld_start);
        	hero_s5k3e2fx_i2c_write(REG_LD_END, s5k3e2fx_reg_pattern[rt].ld_end);
        	hero_s5k3e2fx_i2c_write(REG_SL_START, s5k3e2fx_reg_pattern[rt].sl_start);
        	hero_s5k3e2fx_i2c_write(REG_SL_END, s5k3e2fx_reg_pattern[rt].sl_end);
        	hero_s5k3e2fx_i2c_write(REG_RX_START, s5k3e2fx_reg_pattern[rt].rx_start);
        	hero_s5k3e2fx_i2c_write(REG_S1_START, s5k3e2fx_reg_pattern[rt].s1_start);
        	hero_s5k3e2fx_i2c_write(REG_S1_END, s5k3e2fx_reg_pattern[rt].s1_end);
        	hero_s5k3e2fx_i2c_write(REG_S1S_START, s5k3e2fx_reg_pattern[rt].s1s_start);
        	hero_s5k3e2fx_i2c_write(REG_S1S_END, s5k3e2fx_reg_pattern[rt].s1s_end);
        	hero_s5k3e2fx_i2c_write(REG_S3_START, s5k3e2fx_reg_pattern[rt].s3_start);
        	hero_s5k3e2fx_i2c_write(REG_S3_END, s5k3e2fx_reg_pattern[rt].s3_end);
        	hero_s5k3e2fx_i2c_write(REG_CMP_EN_START, s5k3e2fx_reg_pattern[rt].cmp_en_start);
        	hero_s5k3e2fx_i2c_write(REG_CLP_SL_START, s5k3e2fx_reg_pattern[rt].clp_sl_start);
        	hero_s5k3e2fx_i2c_write(REG_CLP_SL_END, s5k3e2fx_reg_pattern[rt].clp_sl_end);
        	hero_s5k3e2fx_i2c_write(REG_OFF_START, s5k3e2fx_reg_pattern[rt].off_start);
        	hero_s5k3e2fx_i2c_write(REG_RMP_EN_START, s5k3e2fx_reg_pattern[rt].rmp_en_start);
        	hero_s5k3e2fx_i2c_write(REG_TX_START, s5k3e2fx_reg_pattern[rt].tx_start);
        	hero_s5k3e2fx_i2c_write(REG_TX_END, s5k3e2fx_reg_pattern[rt].tx_end);
        	hero_s5k3e2fx_i2c_write(REG_STX_WIDTH, s5k3e2fx_reg_pattern[rt].stx_width);
        	hero_s5k3e2fx_i2c_write(REG_3152_RESERVED, s5k3e2fx_reg_pattern[rt].reg_3152_reserved);
        	hero_s5k3e2fx_i2c_write(REG_315A_RESERVED, s5k3e2fx_reg_pattern[rt].reg_315A_reserved);
        	hero_s5k3e2fx_i2c_write(REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, s5k3e2fx_reg_pattern[rt].analogue_gain_code_global_msb);
        	hero_s5k3e2fx_i2c_write(REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, s5k3e2fx_reg_pattern[rt].analogue_gain_code_global_lsb);
        	hero_s5k3e2fx_i2c_write(REG_FINE_INTEGRATION_TIME, s5k3e2fx_reg_pattern[rt].fine_integration_time);
       	 	hero_s5k3e2fx_i2c_write(REG_COARSE_INTEGRATION_TIME, s5k3e2fx_reg_pattern[rt].coarse_integration_time);
		hero_s5k3e2fx_i2c_write(0x301d, 0x3f); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3028, 0x40); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3070, 0xdf); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3072, 0x20); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x301b, 0x73); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x30bd, 0x06); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x30c2, 0x0b); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3051, 0xe6); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3029, 0x02); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x30bf, 0x00); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x3022, 0x87); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x30c2, 0x0b); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3046, 0x3c); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x3159, 0x0F); //Kevin add base on Samsung suggested settings 20090413
        	hero_s5k3e2fx_i2c_write(0x315A, 0xFF); //Kevin add base on Samsung suggested settings 20090413

		//lens correction
		//Kevin add lens correction settings  20090505 start
		if (rt == 0 )
		{
		hero_s5k3e2fx_i2c_write(0x309e, 0x52);
		hero_s5k3e2fx_i2c_write(0x309f, 0x3e);
		hero_s5k3e2fx_i2c_write(0x30a0, 0x03);
		hero_s5k3e2fx_i2c_write(0x30a1, 0x1f);
		hero_s5k3e2fx_i2c_write(0x30a2, 0x04);
		hero_s5k3e2fx_i2c_write(0x30a3, 0x21);
		hero_s5k3e2fx_i2c_write(0x30a4, 0x04);
		hero_s5k3e2fx_i2c_write(0x30a5, 0x00);
		hero_s5k3e2fx_i2c_write(0x30a6, 0x0c);
		hero_s5k3e2fx_i2c_write(0x30a7, 0x7c);
		hero_s5k3e2fx_i2c_write(0x30a8, 0x04);
		hero_s5k3e2fx_i2c_write(0x30a9, 0x00);
		hero_s5k3e2fx_i2c_write(0x30aa, 0x10);
		hero_s5k3e2fx_i2c_write(0x30ab, 0x84);
		hero_s5k3e2fx_i2c_write(0x0344, 0x00); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0345, 0x08); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0346, 0x00); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0347, 0x08); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0348, 0x0a); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0349, 0x27); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x034a, 0x07); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x034b, 0x9f); //Kevin add base on Samsung suggested settings 20090420

		}
		else
		{
		hero_s5k3e2fx_i2c_write(0x309e, 0x52);
		hero_s5k3e2fx_i2c_write(0x309f, 0x7b);
		hero_s5k3e2fx_i2c_write(0x30a0, 0x03);
		hero_s5k3e2fx_i2c_write(0x30a1, 0x1f);
		hero_s5k3e2fx_i2c_write(0x30a2, 0x02);
		hero_s5k3e2fx_i2c_write(0x30a3, 0x15);
		hero_s5k3e2fx_i2c_write(0x30a4, 0x00);
		hero_s5k3e2fx_i2c_write(0x30a5, 0x00);
		hero_s5k3e2fx_i2c_write(0x30a6, 0x00);
		hero_s5k3e2fx_i2c_write(0x30a7, 0x00);
		hero_s5k3e2fx_i2c_write(0x30a8, 0x00);
		hero_s5k3e2fx_i2c_write(0x30a9, 0x00);
		hero_s5k3e2fx_i2c_write(0x30aa, 0x00);
		hero_s5k3e2fx_i2c_write(0x30ab, 0x00);
		hero_s5k3e2fx_i2c_write(0x0344, 0x00); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0345, 0x00); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0346, 0x00); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0347, 0x00); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0348, 0x0a); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x0349, 0x2F); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x034a, 0x07); //Kevin add base on Samsung suggested settings 20090420
		hero_s5k3e2fx_i2c_write(0x034b, 0xA7); //Kevin add base on Samsung suggested settings 20090420

		}

        	/*Streaming ON*/
		//becker change for SS sensor setting
		hero_s5k3e2fx_i2c_write(0x3150, 0x50);//Kevin add base on Samsung suggested settings 20090413
		msleep(5);
		hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_MODE_SELECT, S5K3E2FX_MODE_SELECT_STREAM);
		msleep(5);
		}
		break;
		//
	case CAMSENSOR_REG_INIT:
		printk(KERN_INFO "CAMSENSOR_REG_INIT (rt %d)\n", rt);
        	hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_MODE_SELECT, S5K3E2FX_MODE_SELECT_SW_STANDBY);
		//becker add for SS sequence adjust
		if (hero_s5k3e2fx_set_pclk(rt, 1) < 0)
			return -EIO;
		/*Data Format */
        	hero_s5k3e2fx_i2c_write(REG_CCP_DATA_FORMAT_MSB, s5k3e2fx_reg_pattern[rt].ccp_data_format_msb);
        	hero_s5k3e2fx_i2c_write(REG_CCP_DATA_FORMAT_LSB, s5k3e2fx_reg_pattern[rt].ccp_data_format_lsb);
        	/*Output Size */
        	hero_s5k3e2fx_i2c_write(REG_X_OUTPUT_SIZE_MSB, s5k3e2fx_reg_pattern[rt].x_output_size_msb);
        	hero_s5k3e2fx_i2c_write(REG_X_OUTPUT_SIZE_LSB, s5k3e2fx_reg_pattern[rt].x_output_size_lsb);
        	hero_s5k3e2fx_i2c_write(REG_Y_OUTPUT_SIZE_MSB, s5k3e2fx_reg_pattern[rt].y_output_size_msb);
        	hero_s5k3e2fx_i2c_write(REG_Y_OUTPUT_SIZE_LSB, s5k3e2fx_reg_pattern[rt].y_output_size_lsb);
        	/* Binning */
        	hero_s5k3e2fx_i2c_write(REG_X_EVEN_INC, s5k3e2fx_reg_pattern[rt].x_even_inc);
        	hero_s5k3e2fx_i2c_write(REG_X_ODD_INC, s5k3e2fx_reg_pattern[rt].x_odd_inc );
        	hero_s5k3e2fx_i2c_write(REG_Y_EVEN_INC, s5k3e2fx_reg_pattern[rt].y_even_inc);
        	hero_s5k3e2fx_i2c_write(REG_Y_ODD_INC, s5k3e2fx_reg_pattern[rt].y_odd_inc);
        	hero_s5k3e2fx_i2c_write(REG_BINNING_ENABLE, s5k3e2fx_reg_pattern[rt].binning_enable);
        	/* Frame format */
        	hero_s5k3e2fx_i2c_write(REG_FRAME_LENGTH_LINES_MSB, s5k3e2fx_reg_pattern[rt].frame_length_lines_msb);
        	hero_s5k3e2fx_i2c_write(REG_FRAME_LENGTH_LINES_LSB, s5k3e2fx_reg_pattern[rt].frame_length_lines_lsb);
        	hero_s5k3e2fx_i2c_write(REG_LINE_LENGTH_PCK_MSB, s5k3e2fx_reg_pattern[rt].line_length_pck_msb);
        	hero_s5k3e2fx_i2c_write(REG_LINE_LENGTH_PCK_LSB, s5k3e2fx_reg_pattern[rt].line_length_pck_lsb);
        	/* MSR setting */
        	hero_s5k3e2fx_i2c_write(REG_SHADE_CLK_ENABLE, s5k3e2fx_reg_pattern[rt].shade_clk_enable);
        	hero_s5k3e2fx_i2c_write(REG_SEL_CCP, s5k3e2fx_reg_pattern[rt].sel_ccp);
        	hero_s5k3e2fx_i2c_write(REG_VPIX, s5k3e2fx_reg_pattern[rt].vpix);
        	hero_s5k3e2fx_i2c_write(REG_CLAMP_ON, s5k3e2fx_reg_pattern[rt].clamp_on);
        	hero_s5k3e2fx_i2c_write(REG_OFFSET, s5k3e2fx_reg_pattern[rt].offset);
        	/* CDS timing setting */
        	hero_s5k3e2fx_i2c_write(REG_LD_START, s5k3e2fx_reg_pattern[rt].ld_start);
        	hero_s5k3e2fx_i2c_write(REG_LD_END, s5k3e2fx_reg_pattern[rt].ld_end);
        	hero_s5k3e2fx_i2c_write(REG_SL_START, s5k3e2fx_reg_pattern[rt].sl_start);
        	hero_s5k3e2fx_i2c_write(REG_SL_END, s5k3e2fx_reg_pattern[rt].sl_end);
        	hero_s5k3e2fx_i2c_write(REG_RX_START, s5k3e2fx_reg_pattern[rt].rx_start);
        	hero_s5k3e2fx_i2c_write(REG_S1_START, s5k3e2fx_reg_pattern[rt].s1_start);
        	hero_s5k3e2fx_i2c_write(REG_S1_END, s5k3e2fx_reg_pattern[rt].s1_end);
        	hero_s5k3e2fx_i2c_write(REG_S1S_START, s5k3e2fx_reg_pattern[rt].s1s_start);
        	hero_s5k3e2fx_i2c_write(REG_S1S_END, s5k3e2fx_reg_pattern[rt].s1s_end);
        	hero_s5k3e2fx_i2c_write(REG_S3_START, s5k3e2fx_reg_pattern[rt].s3_start);
        	hero_s5k3e2fx_i2c_write(REG_S3_END, s5k3e2fx_reg_pattern[rt].s3_end);
        	hero_s5k3e2fx_i2c_write(REG_CMP_EN_START, s5k3e2fx_reg_pattern[rt].cmp_en_start);
       		hero_s5k3e2fx_i2c_write(REG_CLP_SL_START, s5k3e2fx_reg_pattern[rt].clp_sl_start);
        	hero_s5k3e2fx_i2c_write(REG_CLP_SL_END, s5k3e2fx_reg_pattern[rt].clp_sl_end);
        	hero_s5k3e2fx_i2c_write(REG_OFF_START, s5k3e2fx_reg_pattern[rt].off_start);
        	hero_s5k3e2fx_i2c_write(REG_RMP_EN_START, s5k3e2fx_reg_pattern[rt].rmp_en_start);
        	hero_s5k3e2fx_i2c_write(REG_TX_START, s5k3e2fx_reg_pattern[rt].tx_start);
        	hero_s5k3e2fx_i2c_write(REG_TX_END, s5k3e2fx_reg_pattern[rt].tx_end);
        	hero_s5k3e2fx_i2c_write(REG_STX_WIDTH, s5k3e2fx_reg_pattern[rt].stx_width);
        	hero_s5k3e2fx_i2c_write(REG_3152_RESERVED, s5k3e2fx_reg_pattern[rt].reg_3152_reserved);
        	hero_s5k3e2fx_i2c_write(REG_315A_RESERVED, s5k3e2fx_reg_pattern[rt].reg_315A_reserved);
        	hero_s5k3e2fx_i2c_write(REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, s5k3e2fx_reg_pattern[rt].analogue_gain_code_global_msb);
        	hero_s5k3e2fx_i2c_write(REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, s5k3e2fx_reg_pattern[rt].analogue_gain_code_global_lsb);
        	hero_s5k3e2fx_i2c_write(REG_FINE_INTEGRATION_TIME, s5k3e2fx_reg_pattern[rt].fine_integration_time);
        	hero_s5k3e2fx_i2c_write(REG_COARSE_INTEGRATION_TIME, s5k3e2fx_reg_pattern[rt].coarse_integration_time);
        	hero_s5k3e2fx_i2c_write(0x301d, 0x3f); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3028, 0x40); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3070, 0xdf); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3072, 0x20); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x301b, 0x73); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x30bd, 0x06); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x30c2, 0x0b); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3051, 0xe6); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3029, 0x02); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x30bf, 0x00); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x3022, 0x87); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x30c2, 0x0b); //Kevin add base on Samsung suggested settings 20090224
        	hero_s5k3e2fx_i2c_write(0x3046, 0x3c); //Kevin add base on Samsung suggested settings 20090420
        	hero_s5k3e2fx_i2c_write(0x3159, 0x0F); //Kevin add base on Samsung suggested settings 20090413
        	hero_s5k3e2fx_i2c_write(0x315A, 0xFF); //Kevin add base on Samsung suggested settings 20090413

        	/*Streaming ON*/
		//becker change for SS sensor setting
		hero_s5k3e2fx_i2c_write(0x3150, 0x50);//Kevin add base on Samsung suggested settings 20090413
		msleep(5);
		hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_MODE_SELECT, S5K3E2FX_MODE_SELECT_STREAM);
		msleep(5);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/***********************************************************
* Samsung set pclk
************************************************************/
int hero_s5k3e2fx_set_pclk(int rt, int div_adj){
	//becker take this off for power down sequence
	#if 0
	int rc;
	if ((rc = s5k3e2fx_i2c_power_down()) < 0) return rc; 
	#endif
	/* PLL setting */
	hero_s5k3e2fx_i2c_write(REG_PRE_PLL_CLK_DIV, s5k3e2fx_reg_pattern[rt].pre_pll_clk_div);
	hero_s5k3e2fx_i2c_write(REG_PLL_MULTIPLIER_MSB, s5k3e2fx_reg_pattern[rt].pll_multiplier_msb);
	hero_s5k3e2fx_i2c_write(REG_PLL_MULTIPLIER_LSB, s5k3e2fx_reg_pattern[rt].pll_multiplier_lsb);
	hero_s5k3e2fx_i2c_write(REG_VT_PIX_CLK_DIV, s5k3e2fx_reg_pattern[rt].vt_pix_clk_div);
	hero_s5k3e2fx_i2c_write(REG_VT_SYS_CLK_DIV, s5k3e2fx_reg_pattern[rt].vt_sys_clk_div);
	hero_s5k3e2fx_i2c_write(REG_OP_PIX_CLK_DIV, s5k3e2fx_reg_pattern[rt].op_pix_clk_div);
	hero_s5k3e2fx_i2c_write(REG_OP_SYS_CLK_DIV, s5k3e2fx_reg_pattern[rt].op_sys_clk_div);
	//becker take this off for SS initial sequence turning
	//if ((rc = s5k3e2fx_i2c_power_up()) < 0) return rc;
	return 0;
}

/***********************************************************
* Samsung write exposure gain
************************************************************/

int hero_s5k3e2fx_write_exposuregain(
	uint32_t mode, uint16_t line, uint16_t gain, 
	uint16_t linelengthpck, uint16_t framelengthlines)
{
	int32_t rc = 0;
	uint8_t gain_msb, gain_lsb;
	uint8_t line_msb, line_lsb;
#if 1
	uint8_t linelengthpck_msb, linelengthpck_lsb;
	uint16_t IT1 = 0;
	uint16_t gain2 = 0;
	uint16_t linelengthpck2 = 0;
	if (hero_s5k3e2fx_i2c_read(REG_COARSE_INTEGRATION_TIME, &IT1) < 0)
		return 0;
	if (hero_s5k3e2fx_i2c_read(REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, &gain2) < 0)
		return 0;
	if (hero_s5k3e2fx_i2c_read(REG_LINE_LENGTH_PCK_MSB, &linelengthpck2) < 0)
		return 0;

	if (IT1 != line || gain2 != gain || linelengthpck != linelengthpck2) {
		gain_msb = (uint8_t) ((gain & 0xFF00) >> 8);
		gain_lsb = (uint8_t) (gain & 0x00FF);
		hero_s5k3e2fx_i2c_write(REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, gain_msb);
		hero_s5k3e2fx_i2c_write(REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, gain_lsb);

		linelengthpck_msb = (uint8_t) ((linelengthpck & 0xFF00) >> 8);
		linelengthpck_lsb = (uint8_t) (linelengthpck & 0x00FF);
		hero_s5k3e2fx_i2c_write(REG_LINE_LENGTH_PCK_MSB, linelengthpck_msb);
		hero_s5k3e2fx_i2c_write(REG_LINE_LENGTH_PCK_LSB, linelengthpck_lsb);

		line_keep = line;
		line_msb = (uint8_t) ((line & 0xFF00) >> 8);
		line_lsb = (uint8_t) (line & 0x00FF);
		hero_s5k3e2fx_i2c_write(REG_COARSE_INTEGRATION_TIME, line_msb);
		hero_s5k3e2fx_i2c_write(REG_COARSE_INTEGRATION_TIME_LSB, line_lsb);
	}
#endif
#if 0
	uint16_t pre_line = 0;
	uint16_t pre_frame = 0;
	uint16_t pre_gain = 0;
	uint8_t framelengthlines_msb, framelengthlines_lsb;
	//Kevin modify start 20090420
	if(line != pre_frame || line != pre_line ||gain != pre_gain)
	{
		hero_s5k3e2fx_i2c_write(0x0104, 0x01);

		if (mode == 0)
		{
			if(line != pre_frame)
			{
				framelengthlines_msb = (uint8_t) (((line+4) & 0xFF00) >> 8);
				framelengthlines_lsb = (uint8_t) ((line+4) & 0x00FF);

				if(line>0x3e2)
				{
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_MSB, framelengthlines_msb);
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_LSB, framelengthlines_lsb);
					pre_frame = line;
				}
				else if(line<=0x03e2 && line != pre_frame)
				{
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_MSB, 0x03);
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_LSB, 0xe2);
					pre_frame = 0x03e2;
				}
			}
		}
	    else if(mode == 1)
	    {
			if(line != pre_frame)
			{
				framelengthlines_msb = (uint8_t) (((line+4) & 0xFF00) >> 8);
				framelengthlines_lsb = (uint8_t) ((line+4) & 0x00FF);

				if(line>0x7b6)
				{
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_MSB, framelengthlines_msb);
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_LSB, framelengthlines_lsb);
					pre_frame = line;
				}
				else if(line<=0x07b6 && line != pre_frame)
				{
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_MSB, 0x07);
					hero_s5k3e2fx_i2c_write(
						REG_FRAME_LENGTH_LINES_LSB, 0xb6);
					pre_frame = 0x07b6;
				}
			}
	    }

	    if(line != pre_line)
	    {
			line_msb = (uint8_t) ((line & 0xFF00) >> 8);
			line_lsb = (uint8_t) (line & 0x00FF);
			hero_s5k3e2fx_i2c_write(
				REG_COARSE_INTEGRATION_TIME, line_msb);
			hero_s5k3e2fx_i2c_write(
				REG_COARSE_INTEGRATION_TIME_LSB, line_lsb);
			pre_line = line;
			line_keep = line;
	    }

	    if(gain != pre_gain)
	    {
			gain_msb = (uint8_t) ((gain & 0xFF00) >> 8);
			gain_lsb = (uint8_t) (gain & 0x00FF);
			hero_s5k3e2fx_i2c_write(
				REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, gain_msb);
			hero_s5k3e2fx_i2c_write(
				REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, gain_lsb);
			pre_gain = gain;
	    }

		hero_s5k3e2fx_i2c_write(0x0104, 0x00);
		// Kevin modify end 20090420
	}
#endif
	return 0;
}
/***********************************************************
* Samsung probe init sequence
************************************************************/

int hero_s5k3e2fx_probe_init(void *client){
	int rc;
	pclient_i2c=(struct i2c_client*)client;
	hero_s5k3e2fx_sensor_init();
	printk(KERN_ERR "s5k3e2fx_probe : early detect chip id after sensor init\n");
	printk(KERN_ERR "s5k3e2fx_probe : this may cause I2C read error in the next step\n");
	printk(KERN_ERR "s5k3e2fx_probe : but it's okay, because we force reading chipid\n");
	printk(KERN_ERR "                 in order to judge which sensor exist\n");
	chipid = 0;
	/* RESET the sensor via I2C register */
	if (hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_SOFTWARE_RESET, S5K3E2FX_SOFTWARE_RESET) < 0) {
		printk(KERN_ERR "s5k3e2fx_probe: software reset fail\n");
		hero_s5k3e2fx_sensor_deinit();
		return -EIO;
	}
	msleep(S5K3E2FX_SS5M0_RESET_DELAY_MSECS);

	if ((rc = hero_s5k3e2fx_i2c_read(S5K3E2FX_SS5M0_REG_MODEL_ID, &chipid)) < 0) {
		printk(KERN_ERR "s5k3e2fx_probe: could not read chip id, rc:%d\n", rc);
		hero_s5k3e2fx_sensor_deinit();
		return rc;
	}
	printk(KERN_INFO "s5k3e2fx_probe: chip id: %d(0x%x)\n", chipid, chipid);

	if (chipid != S5K3E2FX_SS5M0_MODEL_ID) {
		printk(KERN_INFO "s5k3e2fx_probe: chip id %d(0x%x) is invalid\n",
							chipid, chipid);
		hero_s5k3e2fx_sensor_deinit();
		return -EINVAL;
	}
	/*becker add for ss initial sequence*/
	hero_s5k3e2fx_sensor_setting(CAMSENSOR_REG_INIT);
	/*stream off*/
	hero_s5k3e2fx_i2c_write(
			S5K3E2FX_REG_MODE_SELECT, 0x00);
	msleep(260);
	/*lens sharding*/
	hero_s5k3e2fx_i2c_sensor_lens_sharding();
	/*stream on*/
	hero_s5k3e2fx_i2c_write(
			S5K3E2FX_REG_MODE_SELECT, S5K3E2FX_MODE_SELECT_STREAM);
	/*software standby*/
	msleep(60);
	hero_s5k3e2fx_i2c_write(S5K3E2FX_REG_MODE_SELECT, 
		S5K3E2FX_MODE_SELECT_SW_STANDBY);
	msleep(1);
	hero_s5k3e2fx_i2c_write(0x3150, 0x51);
	msleep(260);
	//pull down RST to low
	hero_s5k3e2fx_sensor_deinit();
	return rc;
}



