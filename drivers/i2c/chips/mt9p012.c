/*
 * Copyright (C) 2007-2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
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
#include <linux/wakelock.h>
#include <net/sock.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/vreg.h>
#include <mach/board.h>
#include <mach/mt9p012.h> /* define ioctls */
#include <mach/perflock.h>


#define ALLOW_USPACE_RW		0

static const uint32_t fps_divider = 1;

#define AF_I2C_ID	0x18  /* actuator's slave address */


static struct i2c_client *pclient;

/* we need this to set the clock rate */
static struct clk *vfe_clk;

/* camif clocks */
static struct clk *vfe_mdc_clk;
static struct clk *mdc_clk;

static int mdc_clk_enabled;
static int vfe_mdc_clk_enabled;
static int vfe_clk_enabled;
static int opened;
static int pclk_set;

static struct perf_lock camera_perf_lock;

static const struct mt9p012_reg_pat mt9p012_reg_pattern = { .reg = {
	{ /* preview 2x2 binning 20fps, pclk MHz, MCLK 24MHz */
		5,      //vt_pix_clk_div    REG=0x0300
	    2,      //vt_sys_clk_div    REG=0x0302
	    2,      //pre_pll_clk_div   REG=0x0304 
	    60,      //pll_multiplier    REG=0x0306   60 for 20fps preview
	    10,     //op_pix_clk_div    REG=0x0308
	    1,      //op_sys_clk_div    REG=0x030A
	    16,     //scale_m           REG=0x0404
	    0x0111, //row_speed         REG=0x3016
	    8,      //x_addr_start      REG=0x3004
	    2597,   //x_addr_end        REG=0x3008
	    8,      //y_addr_start      REG=0x3002
	    1949,   //y_addr_end        REG=0x3006
	 // 0x046C, //read_mode         REG=0x3040 /*Preview 2x2 binning*/
	//#ifdef MT9P012_REV_7
	    0x04C3, //read_mode         REG=0x3040 /*Preview 2x2 skipping*///Kevin enable binning 20090120
	//#else
	//    0x006C, //read_mode         REG=0x3040 /*Preview 2x2 skipping*/
	//#endif
	    1320,//1296,   //x_output_size     REG=0x034C
	    972,    //y_output_size     REG=0x034E
	    3400,   //line_length_pck   REG=0x300C  //20 fps preview
	    1057,   //frame_length_lines       REG=0x300A
	    16,     //coarse_integration_time  REG=0x3012
	    1764    //fine_integration_time    REG=0x3014
	},
	{ /* snapshot */
		5,     //vt_pix_clk_div    REG=0x0300
	    2,     //vt_sys_clk_div    REG=0x0302
	    2,     //pre_pll_clk_div   REG=0x0304 
	    60,    //pll_multiplier    REG=0x0306   60 for 10fps snapshot
	    10,    //op_pix_clk_div    REG=0x0308
	    1,     //op_sys_clk_div    REG=0x030A
	    16,    //scale_m           REG=0x0404
	    0x0111,//row_speed         REG=0x3016
	    8,     //x_addr_start      REG=0x3004
	    2615,  //x_addr_end        REG=0x3008
	    8,     //y_addr_start      REG=0x3002
	    1967,  //y_addr_end        REG=0x3006
	//#ifdef MT9P012_REV_7
	    0x0041,//read_mode         REG=0x3040
	//#else
	//    0x0024,//read_mode         REG=0x3040
	//#endif
	    2608,  //x_output_size     REG=0x034C
	    1960,  //y_output_size     REG=0x034E
	    3788,  //line_length_pck   REG=0x300C
	    2058,  //frame_length_lines   REG=0x300A //10 fps snapshot
	    16,    //coarse_integration_time   REG=0x3012
	    882    //fine_integration_time     REG=0x3014
	}
}};

#define MT9P012_REV_7
#define MT9P012_MU5M0_REG_MODEL_ID		0x0000
#ifdef MT9P012_REV_7
  #define MT9P012_MU5M0_MODEL_ID          0x2801
#else 
  #define MT9P012_MU5M0_MODEL_ID          0x2800
#endif
#define REG_GROUPED_PARAMETER_HOLD		0x0104
#define GROUPED_PARAMETER_HOLD			0x0100
#define GROUPED_PARAMETER_UPDATE		0x0000
#define REG_COARSE_INTEGRATION_TIME		0x3012
#define REG_VT_PIX_CLK_DIV			0x0300
#define REG_VT_SYS_CLK_DIV			0x0302
#define REG_PRE_PLL_CLK_DIV			0x0304
#define REG_PLL_MULTIPLIER			0x0306
#define REG_OP_PIX_CLK_DIV			0x0308
#define REG_OP_SYS_CLK_DIV			0x030A
#define REG_SCALE_M				0x0404
#define REG_FRAME_LENGTH_LINES			0x300A
#define REG_LINE_LENGTH_PCK			0x300C
#define REG_X_ADDR_START			0x3004
#define REG_Y_ADDR_START			0x3002
#define REG_X_ADDR_END				0x3008
#define REG_Y_ADDR_END				0x3006
#define REG_X_OUTPUT_SIZE			0x034C
#define REG_Y_OUTPUT_SIZE			0x034E
#define REG_FINE_INTEGRATION_TIME		0x3014
#define REG_ROW_SPEED				0x3016
#define MT9P012_REG_RESET_REGISTER		0x301A
#define MT9P012_RESET_REGISTER_PWON		0x10CC   /*enable paralled and start streaming*/
#define MT9P012_RESET_REGISTER_PWOFF		0x1008 /*stop streaming*/
#define REG_READ_MODE				0x3040
#define REG_GLOBAL_GAIN				0x305E
#define REG_TEST_PATTERN_MODE			0x3070

static struct wake_lock mt9p012_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&mt9p012_wake_lock, WAKE_LOCK_IDLE, "mt9p012");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&mt9p012_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&mt9p012_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&mt9p012_wake_lock);
}

#define CLK_GET(clk) do {						\
	if (!clk) {							\
		clk = clk_get(NULL, #clk);				\
		printk(KERN_INFO 					\
			"mt9p012: clk_get(%s): %p\n", #clk, clk);	\
	}								\
} while(0)

DECLARE_MUTEX(sem_mt9p012);

static struct msm_camera_device_platform_data  *cam5M;

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

struct mt9p012_data {
	struct work_struct work;
};

static DECLARE_WAIT_QUEUE_HEAD(g_data_ready_wait_queue);

static int mt9p012_i2c_sensor_init(struct mt9p012_init *init);
static int mt9p012_i2c_sensor_setting(unsigned long arg);
static int mt9p012_i2c_exposure_gain(uint32_t mode, uint16_t line, uint16_t gain, uint16_t linelengthpck, uint16_t framelengthlines);
static int mt9p012_i2c_move_focus(uint16_t position);
static int mt9p012_i2c_set_default_focus(uint8_t step);
static int mt9p012_i2c_power_up(void);
static int mt9p012_i2c_power_down(void);
static int mt9p012_camif_pad_reg_reset(void);
static int mt9p012_lens_power(int on);

int mt9p012_i2c_lens_tx_data(unsigned char slave_addr, char* txData, int length)
{
	int rc;
	struct i2c_msg msg[] = {
		{
			.addr = slave_addr,
			.flags = 0,
			.len = length,
			.buf = txData,		
		},
	};

#if 0
	{
		int i;
		/* printk(KERN_INFO "mt9p012_i2c_lens_tx_data: af i2c client addr = %x,"
		   " register addr = 0x%02x%02x:\n", slave_addr, txData[0], txData[1]); 
		*/
		for (i = 0; i < length - 2; i++)
			printk(KERN_INFO "\tdata[%d]: 0x%02x\n", i, txData[i+2]);
	}
#endif
    
	rc = i2c_transfer(pclient->adapter, msg, 1);
	if (rc < 0) {
		printk(KERN_ERR "mt9p012_i2c_lens_tx_data: i2c_transfer error %d\n", rc);
		return rc;
	}
	return 0;
}

static int mt9p012_i2c_lens_write(unsigned char slave_addr, unsigned char u_addr, unsigned char u_data)
{
	unsigned char buf[2] = { u_addr, u_data };
	return mt9p012_i2c_lens_tx_data(slave_addr, buf, sizeof(buf));
}

static int mt9p012_i2c_rx_data(char* rxData, int length)
{
	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = pclient->addr,
			.flags = 0,      
			.len = 2,
			.buf = rxData,
		},
		{
			.addr = pclient->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(pclient->adapter, msgs, 2);
	if (rc < 0) {
		printk(KERN_ERR "mt9p012: mt9p012_i2c_rx_data error %d\n", rc);
		return rc;
	}
#if 0
	else {
		int i;
		for (i = 0; i < length; i++)
			printk(KERN_INFO "\tdata[%d]: 0x%02x\n", i, rxData[i]);
	}
#endif

	return 0;
}

int mt9p012_i2c_tx_data(char* txData, int length)
{
	int rc; 

	struct i2c_msg msg[] = {
		{
			.addr = pclient->addr,
			.flags = 0,
			.len = length,
			.buf = txData,		
		},
	};
    
	rc = i2c_transfer(pclient->adapter, msg, 1);
	if (rc < 0) {
		printk(KERN_ERR "mt9p012: mt9p012_i2c_tx_data error %d\n", rc);
		return rc;
	}
	return 0;
}

static int mt9p012_i2c_write(unsigned short u_addr, unsigned short u_data)
{
	int rc, count = 0;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (u_addr & 0xFF00) >> 8;
	buf[1] = u_addr & 0x00FF;
	buf[2] = (u_data & 0xFF00) >> 8;
	buf[3] = u_data & 0x00FF;

retry:
	rc = mt9p012_i2c_tx_data(buf, sizeof(buf));
	if (rc < 0) {
		printk(KERN_ERR
			"mt9p012: txdata error %d add:0x%02x data:0x%02x\n",
			rc, u_addr, u_data);
		printk(KERN_ERR "starting retry policy count:%d\n", count);
		udelay(10);
		count++;
		if (count < 20) {
			if (count > 10)
				udelay(100);
		} else
			return rc;
		goto retry;
	}
	return rc;
}

static int mt9p012_i2c_read(unsigned short u_addr, unsigned short *pu_data)
{
	int rc;
	unsigned char buf[2];

	buf[0] = (u_addr & 0xFF00)>>8;
	buf[1] = (u_addr & 0x00FF);
	rc = mt9p012_i2c_rx_data(buf, 2);
	if (!rc)
		*pu_data = buf[0]<<8 | buf[1];
	else printk(KERN_ERR "mt9p012: i2c read failed\n");
	return rc;	
}

static int msm_camio_clk_enable (int clk_type)
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

static int msm_camio_clk_disable(int clk_type)
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

static int msm_camio_vfe_clk_enable(void)
{
	CLK_GET(vfe_clk);
	if (vfe_clk && !vfe_clk_enabled) {
		vfe_clk_enabled = !clk_enable(vfe_clk);
		printk(KERN_INFO "mt9p012: enable vfe_clk\n");
	}
	return vfe_clk_enabled ? 0 : -EIO;
}

static int msm_camio_clk_rate_set(int rate)
{
	int rc = msm_camio_vfe_clk_enable();
	if (!rc && vfe_clk_enabled)
		rc = clk_set_rate(vfe_clk, rate);
	return rc;
}

static int clk_select(int internal)
{
	int rc = -EIO; 
	printk(KERN_INFO "mt9p012: clk select %d\n", internal);
	CLK_GET(vfe_clk);
	if (vfe_clk != NULL) {
		extern int clk_set_flags(struct clk *clk, unsigned long flags);
		rc = clk_set_flags(vfe_clk, 0x00000100 << internal);
		if (!rc && internal) rc = msm_camio_vfe_clk_enable();
	}
	return rc;
}

static void mt9p012_sensor_init(void)
{
	int ret;
	printk(KERN_INFO "mt9p012: init\n");
	if (!pclient) 
		return;
	
	/*pull hi reset*/
	printk(KERN_INFO "mt9p012: mt9p012_register_init\n");
	ret = gpio_request(cam5M->sensor_reset, "mt9p012");
	if (!ret) {
		gpio_direction_output(cam5M->sensor_reset, 1);
		printk(KERN_INFO "mt9p012: camera sensor_reset set as 1\n");
	} else
		printk(KERN_ERR "mt9p012 error: request gpio %d failed: "
				"%d\n", cam5M->sensor_reset, ret);
	mdelay(2);

	/* pull down power down */
	ret = gpio_request(cam5M->sensor_pwd, "mt9p012");
	if (!ret || ret == -EBUSY)
		gpio_direction_output(cam5M->sensor_pwd, 0);
	else printk(KERN_ERR "mt913t013 error: request gpio %d failed: "
			"%d\n", cam5M->sensor_pwd, ret);
	gpio_free(cam5M->sensor_pwd);

	/* enable clk */
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);

	/* reset CAMIF */
	mt9p012_camif_pad_reg_reset();

	/* set mclk */
	ret = msm_camio_clk_rate_set(24000000);
	if(ret < 0)
		printk(KERN_ERR "camio clk rate select error\n");
	mdelay(2);
	
	/* enable gpio */
	cam5M->config_gpio_on();

	/* delay 2 ms */
	mdelay(2);

	/* reset sensor sequency */
	gpio_direction_output(cam5M->sensor_reset, 0);
	mdelay(2);
	gpio_direction_output(cam5M->sensor_reset, 1);
	gpio_free(cam5M->sensor_reset);
	mdelay(2);

	printk(KERN_INFO "mt9p012: camera sensor init sequence done\n");
}

#define CLK_DISABLE_AND_PUT(clk) do {					\
	if (clk) {							\
		if (clk##_enabled) {					\
			printk(KERN_INFO "mt9p012: disabling "#clk"\n");\
			clk_disable(clk);				\
			clk##_enabled = 0;				\
		}							\
		printk(KERN_INFO 					\
			"mt9p012: clk_put(%s): %p\n", #clk, clk);	\
		clk_put(clk);						\
		clk = NULL; 						\
	}								\
} while(0)

static void mt9p012_sensor_suspend(void)
{
	printk(KERN_INFO "mt9p012: camera sensor suspend sequence\n");
	if (!pclient) {
		return;
	}
	/*disable clk*/
	msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	CLK_DISABLE_AND_PUT(vfe_clk); /* this matches clk_select(1) */
	/* disable gpios */
	cam5M->config_gpio_off();

	printk(KERN_INFO "mt9p012: camera sensor suspend sequence done\n");
}

static int mt9p012_open(struct inode *ip, struct file *fp)
{
	int rc = -EBUSY;
	down(&sem_mt9p012);
	printk(KERN_INFO "mt9p012: open\n");
	if (!opened) {
		printk(KERN_INFO "mt9p012: prevent collapse on idle\n");
		prevent_suspend();
		cam5M->config_gpio_on();

		perf_lock(&camera_perf_lock);

		opened = 1;
		rc = 0;
	}
	up(&sem_mt9p012);
	return rc;
}

static int mt9p012_release(struct inode *ip, struct file *fp)
{
	int rc = -EBADF;
	printk(KERN_INFO "mt9p012: release\n");
	down(&sem_mt9p012);
	if (opened) {
		printk(KERN_INFO "mt9p012: release clocks\n");

		/* mt9p012_i2c_power_down() should be called before closing MCLK */
		/* otherwise I2C_WRITE will always fail                          */
		mt9p012_i2c_power_down();

		CLK_DISABLE_AND_PUT(mdc_clk);
		CLK_DISABLE_AND_PUT(vfe_mdc_clk);
		CLK_DISABLE_AND_PUT(vfe_clk);
		mt9p012_lens_power(1);/*For 5M*/

		cam5M->config_gpio_off();

		perf_unlock(&camera_perf_lock);

		printk(KERN_INFO "mt9p012: allow collapse on idle\n");
		allow_suspend();
		rc = pclk_set = opened = 0;
	}
	up(&sem_mt9p012);
	return rc;
}

#undef CLK_DISABLE_AND_PUT

#define CHECK() ({ 							\
	if (!mdc_clk_enabled || !vfe_mdc_clk_enabled) { 		\
		printk(KERN_ERR "mt9p012 error: one or more clocks"	\
			" are NULL.\n"); 				\
		rc = -EIO; 						\
	} 								\
	!rc; })

static int mt9p012_camif_pad_reg_reset(void)
{
	int rc = clk_select(1);
	if(rc < 0) {
		printk(KERN_ERR "mt9p012 error switching to internal clock\n");
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
	rc = clk_select(0); /* external */
	if(rc < 0) {
		printk(KERN_ERR "mt9p012 error switching to external clock\n");
		return rc;
	}

	return rc;
}

#if ALLOW_USPACE_RW
#define COPY_FROM_USER(size) ({                                         \
        if (copy_from_user(rwbuf, argp, size)) rc = -EFAULT;            \
        !rc; })
#endif

static long mt9p012_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rc = 0;
	
#if ALLOW_USPACE_RW
	unsigned short addr = 0;
	unsigned short data = 0;
	char rwbuf[4];
#endif

	down(&sem_mt9p012);

	switch(cmd) {
#if ALLOW_USPACE_RW
	case MT9P012_I2C_IOCTL_W:
		if (/* CHECK() && */ COPY_FROM_USER(4)) {
			addr = *((unsigned short *)rwbuf);
			data = *((unsigned short *)(rwbuf+2));
			rc = mt9p012_i2c_write(addr, data);
		} else
			printk(KERN_ERR "mt9p012: write: err %d\n", rc);
		break;

	case MT9P012_I2C_IOCTL_R:
		if (/* CHECK() && */ COPY_FROM_USER(4)) {
			addr = *((unsigned short*) rwbuf);
			rc = mt9p012_i2c_read(addr, (unsigned short *)(rwbuf+2));
			if (!rc) {
				if (copy_to_user(argp, rwbuf, 4)) {
					printk(KERN_ERR "mt9p012: read: err " \
							"writeback -EFAULT\n");
					rc = -EFAULT;
				}
			}
		} else
			printk(KERN_ERR "mt9p012: read: err %d\n", rc);
		break;

	case MT9P012_I2C_IOCTL_AF_W:
		if (/* CHECK() && */ COPY_FROM_USER(3))
			rc = mt9p012_i2c_lens_write(*rwbuf, *(rwbuf + 1), *(rwbuf + 2));
		else
			printk(KERN_ERR "mt9p012: af write: err %d\n", rc);
		break;
#endif /* ALLOW_USPACE_RW */

	case MT9P012_I2C_IOCTL_CAMIF_PAD_REG_RESET:
		printk(KERN_INFO "mt9p012: CAMIF_PAD_REG_RESET\n"); 
		if (CHECK())
			rc = mt9p012_camif_pad_reg_reset();
		break;

	case MT9P012_I2C_IOCTL_CAMIF_PAD_REG_RESET_2:
		printk(KERN_INFO "mt9p012: CAMIF_PAD_REG_RESET_2 (pclk_set %d)\n",
				pclk_set);
		if (!pclk_set)
			rc = -EIO;
		else if (CHECK()) {
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
		break;

	case MT9P012_I2C_IOCTL_CAMIF_APPS_RESET:
		printk(KERN_INFO "mt9p012: CAMIF_APPS_RESET\n"); 
		if (CHECK()) {
			rc = clk_select(1);
			if(rc < 0) {
				printk(KERN_ERR "mt9p012 error switching to internal clock\n");
				break;	
			}
			HWIO_OUTM (APPS_RESET,
				HWIO_FMSK(APPS_RESET,VFE),
				1 << HWIO_SHFT(APPS_RESET,VFE));
			udelay(10);
			HWIO_OUTM (APPS_RESET,
				HWIO_FMSK(APPS_RESET,VFE),
				0 << HWIO_SHFT(APPS_RESET,VFE));
			udelay(10);
			rc = clk_select(0); /* external */
			if(rc < 0) {
				printk(KERN_ERR "mt9p012 error switching to external clock\n");
				break;
			}
		}
		break;

	case CAMERA_LENS_POWER_ON:
		rc = mt9p012_lens_power(0);/*For 5M*/
		break;

	case CAMERA_LENS_POWER_OFF:
		rc = mt9p012_lens_power(1);/*For 5M*/
		break;

	case MT9P012_I2C_IOCTL_CLK_ENABLE:
		printk(KERN_INFO "mt9p012: clk enable %ld\n", arg);
		rc = msm_camio_clk_enable(arg);
		break;

	case MT9P012_I2C_IOCTL_CLK_DISABLE:
		printk(KERN_INFO "mt9p012: clk disable %ld\n", arg);
		rc = msm_camio_clk_disable(arg);
		break;

	case MT9P012_I2C_IOCTL_CLK_SELECT:
		printk(KERN_INFO "mt9p012: clk select %ld\n", arg);
		rc = clk_select(!!arg);
		break;

	case MT9P012_I2C_IOCTL_CLK_FREQ_PROG:
		printk(KERN_INFO "mt9p012: clk rate select %ld\n", arg);
		rc = msm_camio_clk_rate_set(arg);
		break;

	case MT9P012_I2C_IOCTL_GET_REGISTERS:
		printk(KERN_INFO "mt9p012: get registers\n");
		if (copy_to_user(argp, &mt9p012_reg_pattern.reg, sizeof(mt9p012_reg_pattern.reg)))
			rc = -EFAULT;
		break;

	case MT9P012_I2C_IOCTL_SENSOR_SETTING:
		printk(KERN_INFO "mt9p012: sensor setting 0x%lx\n", arg);
		rc = mt9p012_i2c_sensor_setting(arg);
		break;

	case MT9P012_I2C_IOCTL_EXPOSURE_GAIN: {
		struct mt9p012_exposure_gain exp;
		if (copy_from_user(&exp, argp, sizeof(exp))) {
			printk(KERN_ERR "mt9p012: (exposure gain) invalid user pointer\n");
			rc = -EFAULT;
			break;
		}
		rc = mt9p012_i2c_exposure_gain(exp.mode, exp.line, exp.gain, exp.linelengthpck, exp.framelengthlines);
	}
		break;

	case MT9P012_I2C_IOCTL_MOVE_FOCUS:
		printk(KERN_INFO "mt9p012: move focus %ld\n", arg);
		rc = mt9p012_i2c_move_focus((uint16_t)arg);
		break;

	case MT9P012_I2C_IOCTL_SET_DEFAULT_FOCUS:
		printk(KERN_INFO "mt9p012: set default focus %ld\n", arg);
		rc = mt9p012_i2c_set_default_focus((uint8_t)arg);
		break;

	case MT9P012_I2C_IOCTL_POWER_DOWN:
		rc = mt9p012_i2c_power_down();
		break;

	case MT9P012_I2C_IOCTL_INIT: {
		struct mt9p012_init init;
		printk(KERN_INFO "mt9p012: init\n");
		if (copy_from_user(&init, argp, sizeof(init))) {
			printk(KERN_ERR "mt9p012: (init) invalid user pointer\n");
			rc = -EFAULT;
			break;
		}
		rc = mt9p012_i2c_sensor_init(&init);
		if (copy_to_user(argp, &init, sizeof(init)))
			rc = -EFAULT;
	}
		break;

	case CAMERA_CONFIGURE_GPIOS:
	case CAMERA_UNCONFIGURE_GPIOS: 
		break;

	default:
		printk(KERN_INFO "mt9p012: unknown ioctl %d\n", cmd);
		break;
	}

	up(&sem_mt9p012);

	return rc;
}

#undef CHECK

static int mt9p012_lens_power(int on)
{
	int rc;
	printk(KERN_INFO "mt9p012: lens power %d\n", on);
	rc = gpio_request(cam5M->vcm_pwd, "mt9p012");
	if (!rc)
		gpio_direction_output(cam5M->vcm_pwd, !on);
	else printk(KERN_ERR "mt9p012 error: request gpio %d failed:"
		" %d\n", cam5M->vcm_pwd, rc);
	gpio_free(cam5M->vcm_pwd);
	return rc;
}

#define I2C_WRITE(reg,data) if (!mt9p012_i2c_write(reg, data) < 0) return -EIO
#define MT9P012_MU5M0_RESET_DELAY_MSECS    66

static int mt9p012_i2c_sensor_init(struct mt9p012_init *init)
{
	int rc;
	
	/* RESET the sensor via I2C register */
	I2C_WRITE(MT9P012_REG_RESET_REGISTER, 0x10CC|0x0001);
	msleep(MT9P012_MU5M0_RESET_DELAY_MSECS);

	if ((rc = mt9p012_i2c_read(MT9P012_MU5M0_REG_MODEL_ID, &init->chipid)) < 0) {
		printk(KERN_ERR "mt9p012: could not read chip id: %d\n", rc);
		return rc;
	}
	printk(KERN_INFO "mt9p012: chip id: %d\n", init->chipid);

	if (init->chipid != MT9P012_MU5M0_MODEL_ID) {
		printk(KERN_INFO "mt9p012: chip id %d is invalid\n", init->chipid);
		return -EINVAL;
	}
#ifndef MT9P012_REV_7
	I2C_WRITE(0x306E, 0x1080);
#else
	I2C_WRITE(0x306E, 0x9000);
#endif
	I2C_WRITE(0x301A, 0x10CC);
	I2C_WRITE(0x3064, 0x0805);
	msleep(MT9P012_MU5M0_RESET_DELAY_MSECS);

	if ((rc = mt9p012_i2c_sensor_setting(CAMSENSOR_REG_INIT |
					((init->preview ? 0 : 1) << 1))) < 0) {
		printk(KERN_INFO "mt9p012: failed to configure the sensor\n");
		return rc;
	}

	mt9p012_i2c_power_up();

	return 0;
}

static int mt9p012_mu5m0_set_lc(void)
{
	if (machine_is_hero())	/* Hero */
	{
		printk(KERN_INFO "mt9p012_mu5m0_set_lc() for machine Hero\n");
		/* lens shading 85% TL84 *///Kevin update from Topaz final settings
		I2C_WRITE(0x360A, 0x0210); // P_RD_P0Q0
		I2C_WRITE(0x360C, 0x10CD); // P_RD_P0Q1
		I2C_WRITE(0x360E, 0x13B1); // P_RD_P0Q2
		I2C_WRITE(0x3610, 0x28CD); // P_RD_P0Q3
		I2C_WRITE(0x3612, 0x87F1); // P_RD_P0Q4
		I2C_WRITE(0x364A, 0xCB2C); // P_RD_P1Q0
		I2C_WRITE(0x364C, 0x694C); // P_RD_P1Q1
		I2C_WRITE(0x364E, 0x2A8E); // P_RD_P1Q2
		I2C_WRITE(0x3650, 0xC3AE); // P_RD_P1Q3
		I2C_WRITE(0x3652, 0xD20D); // P_RD_P1Q4
		I2C_WRITE(0x368A, 0x2B51); // P_RD_P2Q0
		I2C_WRITE(0x368C, 0x3BF0); // P_RD_P2Q1
		I2C_WRITE(0x368E, 0xBCB2); // P_RD_P2Q2
		I2C_WRITE(0x3690, 0xC992); // P_RD_P2Q3
		I2C_WRITE(0x3692, 0x3411); // P_RD_P2Q4
		I2C_WRITE(0x36CA, 0x7F2D); // P_RD_P3Q0
		I2C_WRITE(0x36CC, 0xA42F); // P_RD_P3Q1
		I2C_WRITE(0x36CE, 0x89D1); // P_RD_P3Q2
		I2C_WRITE(0x36D0, 0x03D2); // P_RD_P3Q3
		I2C_WRITE(0x36D2, 0x0192); // P_RD_P3Q4
		I2C_WRITE(0x370A, 0x90F1); // P_RD_P4Q0
		I2C_WRITE(0x370C, 0xCBB2); // P_RD_P4Q1
		I2C_WRITE(0x370E, 0xB7F0); // P_RD_P4Q2
		I2C_WRITE(0x3710, 0x6A54); // P_RD_P4Q3
		I2C_WRITE(0x3712, 0x4A34); // P_RD_P4Q4
		I2C_WRITE(0x3600, 0x0550); // P_GR_P0Q0
		I2C_WRITE(0x3602, 0xDD2D); // P_GR_P0Q1
		I2C_WRITE(0x3604, 0x0AF1); // P_GR_P0Q2
		I2C_WRITE(0x3606, 0x7E2E); // P_GR_P0Q3
		I2C_WRITE(0x3608, 0x9CB1); // P_GR_P0Q4
		I2C_WRITE(0x3640, 0xDE8C); // P_GR_P1Q0
		I2C_WRITE(0x3642, 0x0C0B); // P_GR_P1Q1
		I2C_WRITE(0x3644, 0x1F6E); // P_GR_P1Q2
		I2C_WRITE(0x3646, 0xEBEE); // P_GR_P1Q3
		I2C_WRITE(0x3648, 0xE32D); // P_GR_P1Q4
		I2C_WRITE(0x3680, 0x02F1); // P_GR_P2Q0
		I2C_WRITE(0x3682, 0x5950); // P_GR_P2Q1
		I2C_WRITE(0x3684, 0xB912); // P_GR_P2Q2
		I2C_WRITE(0x3686, 0xABD1); // P_GR_P2Q3
		I2C_WRITE(0x3688, 0x2511); // P_GR_P2Q4
		I2C_WRITE(0x36C0, 0x590D); // P_GR_P3Q0
		I2C_WRITE(0x36C2, 0xBC8F); // P_GR_P3Q1
		I2C_WRITE(0x36C4, 0xC650); // P_GR_P3Q2
		I2C_WRITE(0x36C6, 0x5491); // P_GR_P3Q3
		I2C_WRITE(0x36C8, 0x01D2); // P_GR_P3Q4
		I2C_WRITE(0x3700, 0xFFF0); // P_GR_P4Q0
		I2C_WRITE(0x3702, 0xDF91); // P_GR_P4Q1
		I2C_WRITE(0x3704, 0xDE72); // P_GR_P4Q2
		I2C_WRITE(0x3706, 0x1D33); // P_GR_P4Q3
		I2C_WRITE(0x3708, 0x2B55); // P_GR_P4Q4
		I2C_WRITE(0x3614, 0x0190); // P_BL_P0Q0
		I2C_WRITE(0x3616, 0xD7AD); // P_BL_P0Q1
		I2C_WRITE(0x3618, 0x53B0); // P_BL_P0Q2
		I2C_WRITE(0x361A, 0x5EAE); // P_BL_P0Q3
		I2C_WRITE(0x361C, 0xDB70); // P_BL_P0Q4
		I2C_WRITE(0x3654, 0xC7EB); // P_BL_P1Q0
		I2C_WRITE(0x3656, 0x70AC); // P_BL_P1Q1
		I2C_WRITE(0x3658, 0x0F6F); // P_BL_P1Q2
		I2C_WRITE(0x365A, 0xB26F); // P_BL_P1Q3
		I2C_WRITE(0x365C, 0x8BEE); // P_BL_P1Q4
		I2C_WRITE(0x3694, 0x5690); // P_BL_P2Q0
		I2C_WRITE(0x3696, 0x4810); // P_BL_P2Q1
		I2C_WRITE(0x3698, 0xDAD2); // P_BL_P2Q2
		I2C_WRITE(0x369A, 0xB5F1); // P_BL_P2Q3
		I2C_WRITE(0x369C, 0x0973); // P_BL_P2Q4
		I2C_WRITE(0x36D4, 0x60AE); // P_BL_P3Q0
		I2C_WRITE(0x36D6, 0xE86F); // P_BL_P3Q1
		I2C_WRITE(0x36D8, 0x8611); // P_BL_P3Q2
		I2C_WRITE(0x36DA, 0x2172); // P_BL_P3Q3
		I2C_WRITE(0x36DC, 0x73AF); // P_BL_P3Q4
		I2C_WRITE(0x3714, 0xCB50); // P_BL_P4Q0
		I2C_WRITE(0x3716, 0xCBF1); // P_BL_P4Q1
		I2C_WRITE(0x3718, 0x6372); // P_BL_P4Q2
		I2C_WRITE(0x371A, 0x28D3); // P_BL_P4Q3
		I2C_WRITE(0x371C, 0xD18F); // P_BL_P4Q4
		I2C_WRITE(0x361E, 0x00F0); // P_GB_P0Q0
		I2C_WRITE(0x3620, 0x7F0C); // P_GB_P0Q1
		I2C_WRITE(0x3622, 0x0571); // P_GB_P0Q2
		I2C_WRITE(0x3624, 0x48E7); // P_GB_P0Q3
		I2C_WRITE(0x3626, 0x82F1); // P_GB_P0Q4
		I2C_WRITE(0x365E, 0x6D6A); // P_GB_P1Q0
		I2C_WRITE(0x3660, 0x484B); // P_GB_P1Q1
		I2C_WRITE(0x3662, 0x43CE); // P_GB_P1Q2
		I2C_WRITE(0x3664, 0x9D6E); // P_GB_P1Q3
		I2C_WRITE(0x3666, 0x026A); // P_GB_P1Q4
		I2C_WRITE(0x369E, 0x0211); // P_GB_P2Q0
		I2C_WRITE(0x36A0, 0x616F); // P_GB_P2Q1
		I2C_WRITE(0x36A2, 0xC4F2); // P_GB_P2Q2
		I2C_WRITE(0x36A4, 0xFD71); // P_GB_P2Q3
		I2C_WRITE(0x36A6, 0x6E11); // P_GB_P2Q4
		I2C_WRITE(0x36DE, 0x51AE); // P_GB_P3Q0
		I2C_WRITE(0x36E0, 0xEA4E); // P_GB_P3Q1
		I2C_WRITE(0x36E2, 0xF7D0); // P_GB_P3Q2
		I2C_WRITE(0x36E4, 0x35D1); // P_GB_P3Q3
		I2C_WRITE(0x36E6, 0x00D1); // P_GB_P3Q4
		I2C_WRITE(0x371E, 0xDD50); // P_GB_P4Q0
		I2C_WRITE(0x3720, 0x8CF2); // P_GB_P4Q1
		I2C_WRITE(0x3722, 0xC951); // P_GB_P4Q2
		I2C_WRITE(0x3724, 0x23B4); // P_GB_P4Q3
		I2C_WRITE(0x3726, 0x6C74); // P_GB_P4Q4
		I2C_WRITE(0x3782, 0x04F4); // Original LC 2 // POLY_ORIGIN_C
		I2C_WRITE(0x3784, 0x03C8); // POLY_ORIGIN_R
		I2C_WRITE(0x3780, 0x8000); // POLY_SC_ENABLE
	}
	else	/* Sapphire */
	{
		printk(KERN_INFO "mt9p012_mu5m0_set_lc() for machine Sapphire\n");
		/* lens shading 85% TL84 */
		I2C_WRITE(0x360A, 0x0230); // P_RD_P0Q0
		I2C_WRITE(0x360C, 0x0B8D); // P_RD_P0Q1
		I2C_WRITE(0x360E, 0x0771); // P_RD_P0Q2
		I2C_WRITE(0x3610, 0xD1AC); // P_RD_P0Q3
		I2C_WRITE(0x3612, 0xB090); // P_RD_P0Q4
		I2C_WRITE(0x364A, 0xF3AC); // P_RD_P1Q0
		I2C_WRITE(0x364C, 0xF3AA); // P_RD_P1Q1
		I2C_WRITE(0x364E, 0x522D); // P_RD_P1Q2
		I2C_WRITE(0x3650, 0x222C); // P_RD_P1Q3
		I2C_WRITE(0x3652, 0x8AEE); // P_RD_P1Q4
		I2C_WRITE(0x368A, 0x30F1); // P_RD_P2Q0
		I2C_WRITE(0x368C, 0x0E0E); // P_RD_P2Q1
		I2C_WRITE(0x368E, 0xE471); // P_RD_P2Q2
		I2C_WRITE(0x3690, 0xFDF0); // P_RD_P2Q3
		I2C_WRITE(0x3692, 0x62AE); // P_RD_P2Q4
		I2C_WRITE(0x36CA, 0x79CE); // P_RD_P3Q0
		I2C_WRITE(0x36CC, 0x172D); // P_RD_P3Q1
		I2C_WRITE(0x36CE, 0xB34E); // P_RD_P3Q2
		I2C_WRITE(0x36D0, 0x9FAE); // P_RD_P3Q3
		I2C_WRITE(0x36D2, 0x37ED); // P_RD_P3Q4
		I2C_WRITE(0x370A, 0xC0B1); // P_RD_P4Q0
		I2C_WRITE(0x370C, 0x8B51); // P_RD_P4Q1
		I2C_WRITE(0x370E, 0xB0B0); // P_RD_P4Q2
		I2C_WRITE(0x3710, 0x1173); // P_RD_P4Q3
		I2C_WRITE(0x3712, 0x66D3); // P_RD_P4Q4
		I2C_WRITE(0x3600, 0x0470); // P_GR_P0Q0
		I2C_WRITE(0x3602, 0xA98D); // P_GR_P0Q1
		I2C_WRITE(0x3604, 0x14F1); // P_GR_P0Q2
		I2C_WRITE(0x3606, 0x4ACD); // P_GR_P0Q3
		I2C_WRITE(0x3608, 0xEA50); // P_GR_P0Q4
		I2C_WRITE(0x3640, 0xA0AC); // P_GR_P1Q0
		I2C_WRITE(0x3642, 0x8C0D); // P_GR_P1Q1
		I2C_WRITE(0x3644, 0x698C); // P_GR_P1Q2
		I2C_WRITE(0x3646, 0xDF2C); // P_GR_P1Q3
		I2C_WRITE(0x3648, 0xECCC); // P_GR_P1Q4
		I2C_WRITE(0x3680, 0x7E70); // P_GR_P2Q0
		I2C_WRITE(0x3682, 0x1CCF); // P_GR_P2Q1
		I2C_WRITE(0x3684, 0xD2B1); // P_GR_P2Q2
		I2C_WRITE(0x3686, 0x19AF); // P_GR_P2Q3
		I2C_WRITE(0x3688, 0x39AF); // P_GR_P2Q4
		I2C_WRITE(0x36C0, 0x3D8D); // P_GR_P3Q0
		I2C_WRITE(0x36C2, 0x2D0D); // P_GR_P3Q1
		I2C_WRITE(0x36C4, 0x29EF); // P_GR_P3Q2
		I2C_WRITE(0x36C6, 0x7BCF); // P_GR_P3Q3
		I2C_WRITE(0x36C8, 0xBB30); // P_GR_P3Q4
		I2C_WRITE(0x3700, 0xBB50); // P_GR_P4Q0
		I2C_WRITE(0x3702, 0xC7ED); // P_GR_P4Q1
		I2C_WRITE(0x3704, 0x9FD2); // P_GR_P4Q2
		I2C_WRITE(0x3706, 0xAB12); // P_GR_P4Q3
		I2C_WRITE(0x3708, 0x1154); // P_GR_P4Q4
		I2C_WRITE(0x3614, 0x0110); // P_BL_P0Q0
		I2C_WRITE(0x3616, 0xA26D); // P_BL_P0Q1
		I2C_WRITE(0x3618, 0x1151); // P_BL_P0Q2
		I2C_WRITE(0x361A, 0x5C6D); // P_BL_P0Q3
		I2C_WRITE(0x361C, 0xD8B0); // P_BL_P0Q4
		I2C_WRITE(0x3654, 0xC609); // P_BL_P1Q0
		I2C_WRITE(0x3656, 0xB18C); // P_BL_P1Q1
		I2C_WRITE(0x3658, 0x3F0E); // P_BL_P1Q2
		I2C_WRITE(0x365A, 0x9D2B); // P_BL_P1Q3
		I2C_WRITE(0x365C, 0xC0CD); // P_BL_P1Q4
		I2C_WRITE(0x3694, 0x0F51); // P_BL_P2Q0
		I2C_WRITE(0x3696, 0x4C0F); // P_BL_P2Q1
		I2C_WRITE(0x3698, 0xB372); // P_BL_P2Q2
		I2C_WRITE(0x369A, 0x732F); // P_BL_P2Q3
		I2C_WRITE(0x369C, 0x3A92); // P_BL_P2Q4
		I2C_WRITE(0x36D4, 0x13AF); // P_BL_P3Q0
		I2C_WRITE(0x36D6, 0x448D); // P_BL_P3Q1
		I2C_WRITE(0x36D8, 0x09CE); // P_BL_P3Q2
		I2C_WRITE(0x36DA, 0x5F4F); // P_BL_P3Q3
		I2C_WRITE(0x36DC, 0xC4B1); // P_BL_P3Q4
		I2C_WRITE(0x3714, 0xBA51); // P_BL_P4Q0
		I2C_WRITE(0x3716, 0x468E); // P_BL_P4Q1
		I2C_WRITE(0x3718, 0x58D1); // P_BL_P4Q2
		I2C_WRITE(0x371A, 0x9693); // P_BL_P4Q3
		I2C_WRITE(0x371C, 0x40D2); // P_BL_P4Q4
		I2C_WRITE(0x361E, 0x01B0); // P_GB_P0Q0
		I2C_WRITE(0x3620, 0x4FEC); // P_GB_P0Q1
		I2C_WRITE(0x3622, 0x03D1); // P_GB_P0Q2
		I2C_WRITE(0x3624, 0xAA4D); // P_GB_P0Q3
		I2C_WRITE(0x3626, 0xCA70); // P_GB_P0Q4
		I2C_WRITE(0x365E, 0x58A7); // P_GB_P1Q0
		I2C_WRITE(0x3660, 0xCF2B); // P_GB_P1Q1
		I2C_WRITE(0x3662, 0x292E); // P_GB_P1Q2
		I2C_WRITE(0x3664, 0x232B); // P_GB_P1Q3
		I2C_WRITE(0x3666, 0xFD8B); // P_GB_P1Q4
		I2C_WRITE(0x369E, 0x6A90); // P_GB_P2Q0
		I2C_WRITE(0x36A0, 0x84AD); // P_GB_P2Q1
		I2C_WRITE(0x36A2, 0xE531); // P_GB_P2Q2
		I2C_WRITE(0x36A4, 0xDBAF); // P_GB_P2Q3
		I2C_WRITE(0x36A6, 0x2351); // P_GB_P2Q4
		I2C_WRITE(0x36DE, 0x360E); // P_GB_P3Q0
		I2C_WRITE(0x36E0, 0x500C); // P_GB_P3Q1
		I2C_WRITE(0x36E2, 0x1C6F); // P_GB_P3Q2
		I2C_WRITE(0x36E4, 0xEC6D); // P_GB_P3Q3
		I2C_WRITE(0x36E6, 0xE0D1); // P_GB_P3Q4
		I2C_WRITE(0x371E, 0xF0AE); // P_GB_P4Q0
		I2C_WRITE(0x3720, 0xE42F); // P_GB_P4Q1
		I2C_WRITE(0x3722, 0xECB1); // P_GB_P4Q2
		I2C_WRITE(0x3724, 0x6A72); // P_GB_P4Q3
		I2C_WRITE(0x3726, 0x20D3); // P_GB_P4Q4
		I2C_WRITE(0x3782, 0x051C); // Original LC 2 // POLY_ORIGIN_C
		I2C_WRITE(0x3784, 0x03A0); // POLY_ORIGIN_R
		I2C_WRITE(0x3780, 0x8000); // POLY_SC_ENABLE
	}

	return 0;
}

static int mt9p012_set_pclk(int rt, int div_adj)
{
	int rc;
	if ((rc = mt9p012_i2c_power_down()) < 0) return rc; 
	I2C_WRITE(REG_VT_PIX_CLK_DIV, mt9p012_reg_pattern.reg[rt].vt_pix_clk_div);
	I2C_WRITE(REG_VT_SYS_CLK_DIV, mt9p012_reg_pattern.reg[rt].vt_sys_clk_div);
	I2C_WRITE(REG_PRE_PLL_CLK_DIV, mt9p012_reg_pattern.reg[rt].pre_pll_clk_div * div_adj);
	I2C_WRITE(REG_PLL_MULTIPLIER, mt9p012_reg_pattern.reg[rt].pll_multiplier);
	I2C_WRITE(REG_OP_PIX_CLK_DIV, mt9p012_reg_pattern.reg[rt].op_pix_clk_div);
	I2C_WRITE(REG_OP_SYS_CLK_DIV, mt9p012_reg_pattern.reg[rt].op_sys_clk_div);
	if ((rc = mt9p012_i2c_power_up()) < 0) return rc; 
	pclk_set = 1;
	return 0;
}

static int mt9p012_i2c_sensor_setting(unsigned long arg) 
{
	uint32_t update = arg & 1;
	uint32_t rt = (arg & 2) >> 1;

	if (rt > 1 || update > 1) {
		printk(KERN_ERR "mt9p012: invalid values %d of rt or %d of update\n",
			rt, update);
		return -EINVAL;
	}

	switch (update) {
	case CAMSENSOR_REG_UPDATE_PERIODIC: {
		uint16_t pclk_div_adj = arg >> 16;

		printk(KERN_INFO "CAMSENSOR_REG_UPDATE_PERIODIC (rt %d)\n", rt);
		
		if (!pclk_div_adj || pclk_div_adj > 2) {
			printk(KERN_ERR "mt9p012: invalid value %d of pclk_div_adj\n",
					pclk_div_adj); 
			return -EINVAL;
		}
		
		if (mt9p012_set_pclk(rt, pclk_div_adj) < 0)
			return -EIO;

		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
		I2C_WRITE(REG_ROW_SPEED, mt9p012_reg_pattern.reg[rt].row_speed);
		I2C_WRITE(REG_X_ADDR_START, mt9p012_reg_pattern.reg[rt].x_addr_start);
		I2C_WRITE(REG_X_ADDR_END, mt9p012_reg_pattern.reg[rt].x_addr_end);
		I2C_WRITE(REG_Y_ADDR_START, mt9p012_reg_pattern.reg[rt].y_addr_start);
		I2C_WRITE(REG_Y_ADDR_END, mt9p012_reg_pattern.reg[rt].y_addr_end);
		I2C_WRITE(REG_READ_MODE, mt9p012_reg_pattern.reg[rt].read_mode);
		I2C_WRITE(REG_SCALE_M, mt9p012_reg_pattern.reg[rt].scale_m);
		I2C_WRITE(REG_X_OUTPUT_SIZE, mt9p012_reg_pattern.reg[rt].x_output_size);
		I2C_WRITE(REG_Y_OUTPUT_SIZE, mt9p012_reg_pattern.reg[rt].y_output_size);
		I2C_WRITE(REG_LINE_LENGTH_PCK, mt9p012_reg_pattern.reg[rt].line_length_pck);
		I2C_WRITE(REG_FRAME_LENGTH_LINES, (uint16_t) (mt9p012_reg_pattern.reg[rt].frame_length_lines * fps_divider));
		I2C_WRITE(REG_COARSE_INTEGRATION_TIME, mt9p012_reg_pattern.reg[rt].coarse_integration_time);
		I2C_WRITE(REG_FINE_INTEGRATION_TIME, mt9p012_reg_pattern.reg[rt].fine_integration_time);
		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);
	}
		break;
		
	case CAMSENSOR_REG_INIT:
		printk(KERN_INFO "CAMSENSOR_REG_INIT (rt %d)\n", rt);

		if (mt9p012_set_pclk(rt, 1) < 0) return -EIO;

		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
		
		/* additional power saving mode ok around 38.2MHz */
#ifdef MT9P012_REV_7
		I2C_WRITE(0x30B0, 0x0001);
	    I2C_WRITE(0x308E, 0xE060);
	    I2C_WRITE(0x3092, 0x0A52);
	    I2C_WRITE(0x3094, 0x4656);
		I2C_WRITE(0x3096, 0x5652);
		I2C_WRITE(0x30CA, 0x8006);
		I2C_WRITE(0x312A, 0xDD02);
		I2C_WRITE(0x312C, 0x00E4);
		I2C_WRITE(0x3170, 0x299A);
#endif
	    /* optimized settings for noise */
	    I2C_WRITE(0x3088, 0x6FF6);
	    I2C_WRITE(0x3154, 0x0282);
	    I2C_WRITE(0x3156, 0x0381);
	    I2C_WRITE(0x3162, 0x04CE);
	    I2C_WRITE(0x0204, 0x0010);
	    I2C_WRITE(0x0206, 0x0010);
		I2C_WRITE(0x0208, 0x0010);
	    I2C_WRITE(0x020A, 0x0010);
	    I2C_WRITE(0x020C, 0x0010);
		mdelay(5);
		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);

		/* set preview or snapshot mode */
		I2C_WRITE(REG_ROW_SPEED,
			mt9p012_reg_pattern.reg[rt].row_speed);
		I2C_WRITE(REG_X_ADDR_START,
			mt9p012_reg_pattern.reg[rt].x_addr_start);
		I2C_WRITE(REG_X_ADDR_END,
			mt9p012_reg_pattern.reg[rt].x_addr_end);
		I2C_WRITE(REG_Y_ADDR_START,
			mt9p012_reg_pattern.reg[rt].y_addr_start);
		I2C_WRITE(REG_Y_ADDR_END,
			mt9p012_reg_pattern.reg[rt].y_addr_end);
		I2C_WRITE(REG_READ_MODE, mt9p012_reg_pattern.reg[rt].read_mode);
		I2C_WRITE(REG_SCALE_M, mt9p012_reg_pattern.reg[rt].scale_m);
		I2C_WRITE(REG_X_OUTPUT_SIZE, mt9p012_reg_pattern.reg[rt].x_output_size);
		I2C_WRITE(REG_Y_OUTPUT_SIZE, mt9p012_reg_pattern.reg[rt].y_output_size);
		I2C_WRITE(REG_LINE_LENGTH_PCK, mt9p012_reg_pattern.reg[rt].line_length_pck);
		I2C_WRITE(REG_FRAME_LENGTH_LINES, mt9p012_reg_pattern.reg[rt].frame_length_lines);
		I2C_WRITE(REG_COARSE_INTEGRATION_TIME, mt9p012_reg_pattern.reg[rt].coarse_integration_time);
		I2C_WRITE(REG_FINE_INTEGRATION_TIME, mt9p012_reg_pattern.reg[rt].fine_integration_time);

		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);

		/* load lens shading */
		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
		if(mt9p012_mu5m0_set_lc() < 0) return -EIO;
		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);

		break;
		
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt9p012_i2c_exposure_gain(uint32_t mode, uint16_t line, uint16_t gain, uint16_t linelengthpck, uint16_t framelengthlines)
{
	/* Remove unused variable */
	/*
	static const uint16_t max_legal_gain  = 0x01FF;
	*/
	uint16_t IT1 = 0;//Kevin add for solving AEC lock issue 20080821
	uint16_t gain2 = 0;//Kevin add for solving AEC lock issue 20080821
	uint16_t linelengthpck2 = 0;

	/*
	printk(KERN_INFO "wilson mt9p012_i2c_exposure_gain (gain %d)\n", gain);
	printk(KERN_INFO "mt9p012_i2c_exposure_gain (line %d)\n", line);
	printk(KERN_INFO "mt9p012_i2c_exposure_gain (linelengthpck %d)\n", linelengthpck);
	printk(KERN_INFO "wilson mt9p012_i2c_exposure_gain (framelengthlines %d)\n", framelengthlines);
	*/
	/*Kevin add for solving AEC lock issue 20080821*/
	mt9p012_i2c_read(0x3012, &IT1);
	mt9p012_i2c_read(0x305E, &gain2);
	mt9p012_i2c_read(0x300C, &linelengthpck2);

/*
	printk(KERN_INFO "wilson mt9p012_i2c_exposure_gain (IT1 %d, gain2 : %d)\n", IT1, gain2);
	printk(KERN_INFO "wilson mt9p012_i2c_exposure_gain (line %d, gain : %d)\n", line, gain);
	printk(KERN_INFO "wilson mt9p012_i2c_exposure_gain (linelengthpck %d, linelengthpck2 : %d)\n", linelengthpck, linelengthpck2);
*/
    if (IT1 != line || gain2 != gain || linelengthpck != linelengthpck2) {
		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
		I2C_WRITE(REG_GLOBAL_GAIN, gain);
		I2C_WRITE(REG_LINE_LENGTH_PCK, linelengthpck);
		/*I2C_WRITE(REG_FRAME_LENGTH_LINES, framelengthlines); //Add by Wilson for 5M sensor*/
		I2C_WRITE(REG_COARSE_INTEGRATION_TIME, line);
		I2C_WRITE(REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);
	}

  if (mode == 1) {
    I2C_WRITE(MT9P012_REG_RESET_REGISTER, 0x10cc|0x0002); /*RESET REGISTER RESTART*/
  }

	return 0;
}

#define I2C_AF_WRITE(command, data) if (mt9p012_i2c_lens_write(AF_I2C_ID >> 1, command, data) < 0) return -EIO;

static int mt9p012_i2c_move_focus(uint16_t position)
{
	uint8_t code_val_msb = (position >> 8);
	uint8_t code_val_lsb = (position & 0xFF);

	printk(KERN_INFO "mt9p012_i2c_move_focus (code_val_msb %d)\n", code_val_msb);
	printk(KERN_INFO "mt9p012_i2c_move_focus (code_val_lsb %d)\n", code_val_lsb);

	I2C_AF_WRITE(code_val_msb, code_val_lsb);
	return 0;
}

static int mt9p012_i2c_set_default_focus(uint8_t step)
{
	I2C_AF_WRITE(0x01, step);
    	return 0; 
}

static int powered;

static int mt9p012_i2c_power_up(void)
{
	printk(KERN_INFO "mt9p012: power up\n");
	if (powered) {
		printk(KERN_INFO "mt9p012: already powered up\n");
		return 0;
	}
	I2C_WRITE(MT9P012_REG_RESET_REGISTER, MT9P012_RESET_REGISTER_PWON);
	mdelay(5);
	powered = 1;
	return 0;
}

static int mt9p012_i2c_power_down(void)
{
	printk(KERN_INFO "mt9p012: power down\n");
	if (!powered) {
		printk(KERN_INFO "mt9p012: already powered down\n");
		return 0;
	}
	I2C_WRITE(MT9P012_REG_RESET_REGISTER, MT9P012_RESET_REGISTER_PWOFF);
	mdelay(5);
	powered = pclk_set = 0;
	return 0;
}

#undef I2C_WRITE
#undef I2C_AF_WRITE

static int mt9p012_init_client(struct i2c_client *client)
{
	/* Initialize the MT9P012 Chip */
	init_waitqueue_head(&g_data_ready_wait_queue);
	return 0;
}

static struct file_operations mt9p012_fops = {
        .owner 	= THIS_MODULE,
        .open 	= mt9p012_open,
        .release = mt9p012_release,
        .unlocked_ioctl = mt9p012_ioctl,
};

static struct miscdevice mt9p012_device = {
        .minor 	= MISC_DYNAMIC_MINOR,
        .name 	= "mt9p012",
        .fops 	= &mt9p012_fops,
};

static const char *MT9P012Vendor = "micron";
static const char *MT9P012NAME = "mt9p012";
static const char *MT9P012Size = "5M";



static ssize_t sensor5M_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", MT9P012Vendor, MT9P012NAME, MT9P012Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor5M_vendor_show, NULL);


static struct kobject *android_mt9p012;

static int mt9p012_sysfs_init(void)
{
	int ret ;
	printk(KERN_INFO "mt9p012:kobject creat and add\n");
	android_mt9p012 = kobject_create_and_add("android_camera", NULL);
	if (android_mt9p012 == NULL) {
		printk(KERN_INFO "mt9p012_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	printk(KERN_INFO "mt9p012:sysfs_create_file\n");
	ret = sysfs_create_file(android_mt9p012, &dev_attr_sensor.attr);
	if (ret) {
		printk(KERN_INFO "mt9p012_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_mt9p012);
	}
	return 0 ;
}



static int mt9p012_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mt9p012_data *mt;
	int err = ENODEV;
	int rc;
	unsigned short chipid;

	printk(KERN_INFO "mt9p012: probe start\n");

	cam5M = client->dev.platform_data;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		goto exit_check_functionality_failed;		
	
	if(!(mt = kzalloc( sizeof(struct mt9p012_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, mt);
	mt9p012_init_client(client);
	pclient = client;

	mt9p012_sensor_init();


	/* Horng add for early detect chip id after sensor init                 */
	/* -------------------------------------------------------------------- */
	printk(KERN_ERR "mt9p012_probe : early detect chip id after sensor init\n");
	printk(KERN_ERR "mt9p012_probe : this may cause I2C read error in the next step\n");
	printk(KERN_ERR "mt9p012_probe : but it's okay, because we force reading chipid\n");
	printk(KERN_ERR "                in order to judge which sensor exist\n");

	chipid = 0;

	/* RESET the sensor via I2C register */
	if (mt9p012_i2c_write(MT9P012_REG_RESET_REGISTER, 0x10CC|0x0001) < 0) {
		printk(KERN_ERR "mt9p012_probe: software reset fail\n");
		mt9p012_sensor_suspend();
		return -EIO;
	}
	msleep(MT9P012_MU5M0_RESET_DELAY_MSECS);

	if ((rc = mt9p012_i2c_read(MT9P012_MU5M0_REG_MODEL_ID, &chipid)) < 0) {
		printk(KERN_ERR "mt9p012_probe: could not read chip id, rc:%d\n", rc);
		mt9p012_sensor_suspend();
		return rc;
	}
	printk(KERN_INFO "mt9p012_probe: chip id: %d(0x%x)\n", chipid, chipid);

	if (chipid != MT9P012_MU5M0_MODEL_ID) {
		printk(KERN_INFO "mt9p012_probe: chip id %d(0x%x) is invalid\n",
							chipid, chipid);
		mt9p012_sensor_suspend();
		return -EINVAL;
	}
	/* -------------------------------------------------------------------- */


	mt9p012_sensor_suspend();

	/* Register a misc device */
	err = misc_register(&mt9p012_device);
	if(err) {
		printk(KERN_ERR "mt9p012_probe: misc_register failed \n");
		goto exit_misc_device_register_failed;
	}

	perf_lock_init(&camera_perf_lock, PERF_LOCK_HIGHEST, "camera_mt9p012");

	init_suspend();
	mt9p012_sysfs_init();
	return 0;
	
exit_misc_device_register_failed:
exit_alloc_data_failed:
exit_check_functionality_failed:
	
	return err;
}

	
static int mt9p012_remove(struct i2c_client *client)
{
	struct mt9p012_data *mt;

	printk(KERN_INFO "mt9p012: remove\n");

	if (!pclient) {
		printk(KERN_INFO "pclient is NULL, stop remove !\n");
		return 0;
	}

	mt = i2c_get_clientdata(client);
	free_irq(client->irq, mt);
	deinit_suspend();
	i2c_detach_client(client);
	pclient = NULL;
	misc_deregister(&mt9p012_device);
	kfree(mt);
	return 0;
}

static const struct i2c_device_id mt9p012_id[] = {
	{ "mt9p012", 0 },
	{ }
};

static struct i2c_driver mt9p012_driver = {
	.probe = mt9p012_probe,
	.remove = mt9p012_remove,
	.id_table = mt9p012_id,
	.driver = {		
		.name   = "mt9p012",
	},
};

static int __init mt9p012_init(void)
{
	return i2c_add_driver(&mt9p012_driver);
}

static void __exit mt9p012_exit(void)
{
	i2c_del_driver(&mt9p012_driver);
}

module_init(mt9p012_init);
module_exit(mt9p012_exit);

MODULE_AUTHOR("Wilson Chien");
MODULE_DESCRIPTION("MT9P012 Driver");
MODULE_LICENSE("GPL");

