/*
 * Copyright (C) 2008 QUALCOMM Incorporated.
 * Author: Haibo Jeff Zhong <hzhong@qualcomm.com>
 *
 */

#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <media/msm_camera_sensor.h>
#include <mach/camera.h>
#include <mach/msm_camio.h>
#include "s5k4b2fx.h"
#include <linux/wakelock.h>

/*============================================================================
				  SENSOR REGISTER DEFINES
=============================================================================*/

#define REG_PAGE			  0xFC

#define REG_VENDOR_ID   	  0xB0
#define VENDOR_ID   		  0xAB

#define REG_SW_VERSION  	  0xB1
#define SW_VERSION  		  0x01

#define REG_CHIP_ID 		  0xB2
#define CHIP_ID 			  0xD0

#define REG_CHIP_VERSION	  0xB3
#define CHIP_VERSION		  0x81


static struct wake_lock s5k4b2fx_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&s5k4b2fx_wake_lock, WAKE_LOCK_IDLE, "s5k4b2fx");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&s5k4b2fx_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&s5k4b2fx_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&s5k4b2fx_wake_lock);
}


#if 1
typedef struct {
	uint8_t hs_start_msb;
	uint8_t hs_start_lsb;
	uint8_t hs_width_msb;
	uint8_t hs_width_lsb;

	uint8_t fine_integration_time_msb;
	uint8_t fine_integration_time_lsb;
	uint8_t coarse_integration_time_msb;
	uint8_t coarse_integration_time_lsb;

	uint8_t frame_length_line_msb;
	uint8_t frame_length_line_lsb;
	uint8_t line_length_dck_msb;
	uint8_t line_length_dck_lsb;

	uint8_t x_addr_start_msb;
	uint8_t x_addr_start_lsb;
	uint8_t y_addr_start_msb;
	uint8_t y_addr_start_lsb;
	uint8_t x_addr_end_msb;
	uint8_t x_addr_end_lsb;
	uint8_t y_addr_end_msb;
	uint8_t y_addr_end_lsb;
	uint8_t x_output_size_msb;
	uint8_t x_output_size_lsb;
	uint8_t y_output_size_msb;
	uint8_t y_output_size_lsb;

	uint8_t pll_p;
	uint8_t pll_m_msb;
	uint8_t pll_m_lsb;
	uint8_t dck_div;
	uint8_t pck_div;
} reg_struct;

reg_struct s5k4b2fx_reg_pat[2] =
{
  { /*Preview*/
	0x00,
	0x2A,
	0x03,
	0x30,

	0x05,
	0xC8,
	0x00,
	0xC6,

	0x02,
	0x70,
	0x06,
	0xEA,

	0x00,
	0x00,
	0x00,
	0x00,
	0x06,
	0x40 + 16,
	0x04,
	0xB0 + 16,
	0x03,
	0x30,
	0x02,
	0x68,

	0x02,
	0x00,
	0x6C,
	0x28,
	0x28,
  },
  { /*Snapshot*/
	0x00,
	0x2A,
	0x06,
	0x40+16,

	0x05,
	0xC8,
	0x00,
	0xC6,

	0x04,
	0xC8,
	0x06,
	0xEA,

	0x00,
	0x00,
	0x00,
	0x00,
	0x06,
	0x40+15,
	0x04,
	0xB0+15,
	0x06,
	0x40+16,
	0x04,
	0xB0+16,

	0x02,
	0x00,
	0x6C,
	0x28,
	0x28,
  }
};

typedef enum {
	S5K4B2FX_SS2M0_RES_PREVIEW,
	S5K4B2FX_SS2M0_RES_CAPTURE
} resolution_type;
#endif

enum s5k4b2fx_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum s5k4b2fx_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum s5k4b2fx_reg_update_t {
  /* Sensor egisters that need to be updated during initialization */
  REG_INIT,
  /* Sensor egisters that needs periodic I2C writes */
  UPDATE_PERIODIC,
  /* All the sensor Registers will be updated */
  UPDATE_ALL,
  /* Not valid update */
  UPDATE_INVALID
};

enum s5k4b2fx_setting_t {
	RES_PREVIEW,
	RES_CAPTURE
};

/*============================================================================
						CONSTANT DEFINITIONS
============================================================================*/

#define S5K4B2FX_IMG_I2C_ADDR	0x22>>1		   /* Sensor slave address  */


#define S5K4B2FX_SS2M0_FULL_SIZE_DUMMY_PIXELS      162

#define  S5K4B2FX_SS2M0_TOTAL_STEPS_NEAR_TO_FAR   26
#define  S5K4B2FX_SS2M0_STEPS_NEAR_TO_CLOSEST_INF 26


/*============================================================================
						DATA DECLARATIONS
============================================================================*/



/*****************************************************************************
 *  						RUN TIME VARIABLES
 ****************************************************************************/


#define  DELTA   16
static uint16_t   my_reg_gain;
static uint32_t   my_reg_line_count;

static uint32_t qtr_size_width = 816;
static uint32_t qtr_size_height = 616;

static uint32_t full_size_width = 1612;
static uint32_t full_size_height = 1206;

/* Save current frame rate */
static uint16_t current_fps;

/* increace low light brightness 20070116*/
static uint16_t temp_pixel_clock_per_line;
static uint32_t temp_pixel_clock;

static uint16_t s5k4b2fx_current_lens_position;
static uint32_t camclk_po_hz;
//static uint32_t s5k4b2fx_mclk = 10031020;
static uint32_t s5k4b2fx_mclk = 9600000;


struct s5k4b2fx_work_t {
	struct work_struct work;
};

struct s5k4b2fx_ctrl_t {
	int8_t  opened;
	struct  msm_camera_sensor_info *sensordata;
	struct  s5k4b2fx_work_t *sensorw;
	struct  i2c_client *client;

	enum sensor_mode_t sensormode;
	uint32_t fps_divider; /* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum s5k4b2fx_resolution_t prev_res;
	enum s5k4b2fx_resolution_t pict_res;
	enum s5k4b2fx_resolution_t curr_res;
  	enum s5k4b2fx_test_mode_t  set_test;
};

struct s5k4b2fx_i2c_reg_conf {
	unsigned short saddr;
	unsigned short waddr;
	unsigned short wdata;
};

static struct s5k4b2fx_ctrl_t *s5k4b2fx_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(s5k4b2fx_wait_queue);
DECLARE_MUTEX(s5k4b2fx_sem);

#define POLY    (0x1070U << 3)
static struct i2c_client *pclient;
static enum s5k4b2fx_setting_t lastrt = RES_PREVIEW;
static s32 ss_i2c_smbus_xfer_emulated(struct i2c_adapter * adapter, u16 addr,
                                   unsigned short flags,
                                   char read_write, u16 command, int size,
                                   union i2c_smbus_data * data);
s32 ss_i2c_smbus_xfer(struct i2c_adapter * adapter, u16 addr, unsigned short flags,
                   char read_write, u16 command, int size,
                   union i2c_smbus_data * data);      
s32 ss_i2c_smbus_read_byte_data(struct i2c_client *client, u16 command); 
s32 ss_i2c_smbus_write_byte_data(struct i2c_client *client, u16 command, u8 value); 
static u8 ss_i2c_smbus_pec(u8 crc, u8 *p, size_t count);        
static u8 ss_i2c_smbus_msg_pec(u8 pec, struct i2c_msg *msg);   
static inline void ss_i2c_smbus_add_pec(struct i2c_msg *msg);
static int ss_i2c_smbus_check_pec(u8 cpec, struct i2c_msg *msg);   
static u8 crc8(u16 data);                      

static int s5k4b2fx_i2c_rxdata(unsigned short saddr,
                              unsigned char *rxdata, int length)
{
	int rc;
#if 0	
    struct i2c_msg msgs[] = {
        {.addr   = saddr,
            .flags = 0,
            .len   = 2,
            .buf   = rxdata,
        },
        {.addr   = saddr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxdata,
        },
    };

    if (i2c_transfer(s5k4b2fx_ctrl->client->adapter, msgs, 2) < 0) {
        CDBG("s5k4b2fx_i2c_rxdata failed!\n");
        return -EIO;
    }
#else
	pclient->addr = S5K4B2FX_IMG_I2C_ADDR;
	rc = ss_i2c_smbus_read_byte_data(pclient, saddr);
	if  (rc >= 0)
		*rxdata = rc;    
#endif
    return 0;
}

static int32_t s5k4b2fx_i2c_read_w(unsigned short saddr,
                                  unsigned short raddr, unsigned short *rdata)
{
    int32_t rc = 0;
    unsigned char buf[4];

    if (!rdata)
        return -EIO;

    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00)>>8;
    buf[1] = (raddr & 0x00FF);

    rc = s5k4b2fx_i2c_rxdata(saddr, buf, 2);
    if (rc < 0)
        return rc;

    *rdata = buf[0] << 8 | buf[1];

    if (rc < 0)
        CDBG("s5k4b2fx_i2c_read failed!\n");

    return rc;
}

static int32_t s5k4b2fx_i2c_txdata(unsigned short saddr,
                                  unsigned char *txdata, int length)
{
    struct i2c_msg msg[] = {
        {.addr = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    if (i2c_transfer(s5k4b2fx_ctrl->client->adapter, msg, 1) < 0)
    {
        CDBG("s5k4b2fx_i2c_txdata failed\n");
        return -EIO;
    }

    return 0;
}

static int32_t s5k4b2fx_i2c_write_b(unsigned short saddr,
                                   unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EFAULT;
#if 0
    unsigned char buf[2];

    memset(buf, 0, sizeof(buf));
    buf[0] = waddr;
    buf[1] = wdata;
    rc = s5k4b2fx_i2c_txdata(saddr, buf, 2);

    if (rc < 0)
        CDBG("i2c_write failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
#else
	pclient->addr = S5K4B2FX_IMG_I2C_ADDR;
	CDBG("i2c_write , addr = 0x%x, val = 0x%x!\n",
             waddr, wdata);
	rc = ss_i2c_smbus_write_byte_data(pclient, waddr, wdata);
#endif             

    return rc;
}

static int32_t s5k4b2fx_i2c_write_w(unsigned short saddr,
                                   unsigned short waddr, unsigned short wdata)
{
    int32_t rc = -EFAULT;
    unsigned char buf[4];

    memset(buf, 0, sizeof(buf));
    buf[0] = (waddr & 0xFF00)>>8;
    buf[1] = (waddr & 0x00FF);
    buf[2] = (wdata & 0xFF00)>>8;
    buf[3] = (wdata & 0x00FF);

    rc = s5k4b2fx_i2c_txdata(saddr, buf, 4);

    if (rc < 0)
        CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
             waddr, wdata);

    return rc;
}

static uint32_t s5k4b2fx_reset(struct msm_camera_sensor_info *dev)
{
    int rc;

    rc = gpio_request(dev->sensor_reset, "s5k4b2fx");
    CDBG("--- s5k4b2fx_reset: rc=%d reset=%d\n", rc, dev->sensor_reset);
    {
        gpio_direction_output(dev->sensor_reset, 0);
        mdelay(2);
        gpio_direction_output(dev->sensor_reset, 1);
    }
    gpio_free(dev->sensor_reset);

    return 0;

}

/*==========================================================================*/
static int32_t s5k4b2fx_test(enum s5k4b2fx_test_mode_t mo)
{
    int32_t rc = 0;

    

    return rc;
}

/*==========================================================================*/
static int32_t s5k4b2fx_set_lc(void)
{
    int32_t rc;

    return rc;


}

/*==========================================================================*/
static int32_t s5k4b2fx_set_default_focus(void)
{
    int32_t rc = 0;

    return rc;
}

/*==========================================================================*/
static void s5k4b2fx_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
    /* input fps is preview fps in Q8 format */
    uint16_t snapshot_height, preview_height;
	uint16_t snapshot_fps = fps;
    uint16_t divider;   /*Q10 */

    if (s5k4b2fx_ctrl->prev_res == QTR_SIZE) {
        preview_height = qtr_size_height;
    } else {
        preview_height = full_size_height;
    }
    
    if (s5k4b2fx_ctrl->pict_res == QTR_SIZE) {
        snapshot_height = qtr_size_height;  
    } else {
        snapshot_height = full_size_height;
    }

	divider = (uint16_t)(((snapshot_height / preview_height) * 10 + 5) / 10);
	snapshot_fps = snapshot_fps / divider;
	
	*pfps = snapshot_fps;
}

/*==========================================================================*/
static uint16_t s5k4b2fx_get_prev_lines_pf(void)
{
	uint16_t frame_length_line = 0;
	
    if (s5k4b2fx_ctrl->prev_res == QTR_SIZE) {
		frame_length_line = s5k4b2fx_reg_pat[RES_PREVIEW].frame_length_line_msb<<8;
		frame_length_line = frame_length_line | s5k4b2fx_reg_pat[RES_PREVIEW].frame_length_line_lsb;
	} else {
		frame_length_line = s5k4b2fx_reg_pat[RES_CAPTURE].frame_length_line_msb<<8;
		frame_length_line = frame_length_line | s5k4b2fx_reg_pat[RES_CAPTURE].frame_length_line_lsb;
	}

	return frame_length_line;
}

/*==========================================================================*/
static uint16_t s5k4b2fx_get_prev_pixels_pl(void)
{
	uint16_t line_length_dck = 0;
	
    if (s5k4b2fx_ctrl->prev_res == QTR_SIZE) {
		line_length_dck = s5k4b2fx_reg_pat[RES_PREVIEW].line_length_dck_msb<<8;
		line_length_dck = line_length_dck | s5k4b2fx_reg_pat[RES_PREVIEW].line_length_dck_lsb;
	} else {
		line_length_dck = s5k4b2fx_reg_pat[RES_CAPTURE].line_length_dck_msb<<8;
		line_length_dck = line_length_dck | s5k4b2fx_reg_pat[RES_CAPTURE].line_length_dck_lsb;
	}
	
	return line_length_dck;
}

/*==========================================================================*/
static uint16_t s5k4b2fx_get_pict_lines_pf(void)
{
    uint16_t frame_length_line = 0;

	frame_length_line = s5k4b2fx_reg_pat[RES_CAPTURE].frame_length_line_msb<<8;
	frame_length_line = frame_length_line | s5k4b2fx_reg_pat[RES_CAPTURE].frame_length_line_lsb;

	return frame_length_line;
}

/*==========================================================================*/
static uint16_t s5k4b2fx_get_pict_pixels_pl(void)
{
    uint16_t line_length_dck = 0;

	line_length_dck = s5k4b2fx_reg_pat[RES_CAPTURE].line_length_dck_msb<<8;
	line_length_dck = line_length_dck | s5k4b2fx_reg_pat[RES_CAPTURE].line_length_dck_lsb;

	return line_length_dck;
}

/*==========================================================================*/
static uint32_t s5k4b2fx_get_pict_max_exp_lc(void)
{
    uint16_t snapshot_lines_per_frame;

	if (s5k4b2fx_ctrl->pict_res == QTR_SIZE) {
		snapshot_lines_per_frame = qtr_size_height;
	} else {
		snapshot_lines_per_frame = full_size_height;
	}

	return snapshot_lines_per_frame;
}

/*==========================================================================*/
static int32_t s5k4b2fx_set_fps(struct fps_cfg *fps)
{
    /* input is new fps in Q8 format */
    int32_t rc = 0;
	uint32_t camclk_freq = s5k4b2fx_mclk;
	uint16_t max_fps = 30;
	uint16_t half_fps = 15;
	uint16_t one_quarter_fps = 8;

	if (fps == max_fps) {
		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x58, 0x28);
		if (rc < 0)
			return rc;

		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x59, 0x28);
		if (rc < 0)
			return rc;
		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x93, 0x00);
		if (rc < 0)
			return rc;
	} else if (fps >= half_fps) {
		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x58, 0x48);
		if (rc < 0)
			return rc;

		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x59, 0x48);
		if (rc < 0)
			return rc;
		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x93, 0x00);
		if (rc < 0)
			return rc;
	} else if (fps >= one_quarter_fps) {
		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x58, 0x88);
		if (rc < 0)
			return rc;

		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x59, 0x88);
		if (rc < 0)
			return rc;

		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x93, 0x80);
		if (rc < 0)
			return rc;
	} else {
		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x58, 0x28);
		if (rc < 0)
			return rc;

		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x59, 0x28);
		if (rc < 0)
			return rc;

		rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x93, 0x00);
		if (rc < 0)
			return rc;
	}

	
	return rc;
}

/*==========================================================================*/
static int32_t s5k4b2fx_write_exp_gain(uint16_t gain, uint32_t line)
{
    uint8_t glob_gain, coarse_int_time_lsb, coarse_int_time_msb; 
    int32_t rc = 0;

    my_reg_gain = gain;
    my_reg_line_count = (uint16_t)line;

	glob_gain = (uint8_t) gain;
	coarse_int_time_lsb = (uint8_t) (line & 0x00FF);
	coarse_int_time_msb = (uint8_t) ((line & 0xFF00) >> 8);
	/*
	rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x12, glob_gain);
	if (rc < 0)
		return rc;
	rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x1E, coarse_int_time_msb);
	if (rc < 0)
		return rc;

	rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x1F, coarse_int_time_lsb);
	if (rc < 0)
		return rc;
	*/
	return rc;
}

/*????? ==========================================================================*/
static int32_t s5k4b2fx_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
    int32_t rc = 0;

    rc =
    s5k4b2fx_write_exp_gain(gain, line);
    if (rc < 0)
		return rc;

	return rc;
}

/*==========================================================================*/
static int32_t s5k4b2fx_setting(enum s5k4b2fx_reg_update_t rupdate, 
                               enum s5k4b2fx_setting_t rt)
{
    int32_t rc = 0;

	switch(rupdate)
    {
    case UPDATE_PERIODIC: {
			if (((rt == RES_PREVIEW) && (lastrt == RES_CAPTURE)) ||
				rt == RES_CAPTURE) {
				lastrt = rt;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, REG_PAGE, 0x00);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x03, 0x00);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x24, s5k4b2fx_reg_pat[rt].x_addr_start_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x25, s5k4b2fx_reg_pat[rt].x_addr_start_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x26, s5k4b2fx_reg_pat[rt].y_addr_start_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x27, s5k4b2fx_reg_pat[rt].y_addr_start_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x28, s5k4b2fx_reg_pat[rt].x_addr_end_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x29, s5k4b2fx_reg_pat[rt].x_addr_end_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2A, s5k4b2fx_reg_pat[rt].y_addr_end_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2B, s5k4b2fx_reg_pat[rt].y_addr_end_lsb);
				if (rc < 0)
					return rc;

				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2C, s5k4b2fx_reg_pat[rt].x_output_size_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2D, s5k4b2fx_reg_pat[rt].x_output_size_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2E, s5k4b2fx_reg_pat[rt].y_output_size_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2F, s5k4b2fx_reg_pat[rt].y_output_size_lsb);
				if (rc < 0)
					return rc;

				if (rt == RES_PREVIEW) {
					rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x31, 0x03);
					if (rc < 0)
						return rc;
					rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x33, 0x03);
					if (rc < 0)
						return rc;
				} else {
					rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x31, 0x01);
					if (rc < 0)
						return rc;
					rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x33, 0x01);
					if (rc < 0)
						return rc;
				}

				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0A, s5k4b2fx_reg_pat[rt].hs_start_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0B, s5k4b2fx_reg_pat[rt].hs_start_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0C, s5k4b2fx_reg_pat[rt].hs_width_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0D, s5k4b2fx_reg_pat[rt].hs_width_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x20, s5k4b2fx_reg_pat[rt].frame_length_line_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x21, s5k4b2fx_reg_pat[rt].frame_length_line_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x03, 0x10);
				if (rc < 0)
					return rc;
            }
        }
        break; /* UPDATE_PERIODIC */

    case REG_INIT: {
			lastrt = rt;
			if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, REG_PAGE, 0x00);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x03, 0x00);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x14, 0x00);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x5A, 0x38);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x88, 0x31);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x41, 0x1F);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x70, 0x02);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x12, 0x00);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x1E, 0x02);
				if (rc < 0)
					return rc;

				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x1F, 0x6E);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x24, s5k4b2fx_reg_pat[rt].x_addr_start_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x25, s5k4b2fx_reg_pat[rt].x_addr_start_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x26, s5k4b2fx_reg_pat[rt].y_addr_start_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x27, s5k4b2fx_reg_pat[rt].y_addr_start_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x28, s5k4b2fx_reg_pat[rt].x_addr_end_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x29, s5k4b2fx_reg_pat[rt].x_addr_end_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2A, s5k4b2fx_reg_pat[rt].y_addr_end_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2B, s5k4b2fx_reg_pat[rt].y_addr_end_lsb);
				if (rc < 0)
					return rc;

				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2C, s5k4b2fx_reg_pat[rt].x_output_size_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2D, s5k4b2fx_reg_pat[rt].x_output_size_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2E, s5k4b2fx_reg_pat[rt].y_output_size_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x2F, s5k4b2fx_reg_pat[rt].y_output_size_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x31, 0x03);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x33, 0x03);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x02, 0x50);
				if (rc < 0)
					return rc;

				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x54, s5k4b2fx_reg_pat[rt].pll_p);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x56, s5k4b2fx_reg_pat[rt].pll_m_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x57, s5k4b2fx_reg_pat[rt].pll_m_lsb);
				if  (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x58, s5k4b2fx_reg_pat[rt].dck_div);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x59, s5k4b2fx_reg_pat[rt].pck_div);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0A, s5k4b2fx_reg_pat[rt].hs_start_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0B, s5k4b2fx_reg_pat[rt].hs_start_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0C, s5k4b2fx_reg_pat[rt].hs_width_msb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0D, s5k4b2fx_reg_pat[rt].hs_width_lsb);
				if (rc < 0)
					return rc;

				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x20, s5k4b2fx_reg_pat[rt].frame_length_line_msb);
				if (rc < 0)
					return rc;

				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x21, s5k4b2fx_reg_pat[rt].frame_length_line_lsb);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x0E, 0x4C);
				if (rc < 0)
					return rc;
				rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x03, 0x10);
				if (rc < 0)
					return rc;

		    }
		}
        break; /* case REG_INIT: */

    default:
        rc = -EFAULT;
        break;
    } /* switch (rupdate) */

	return rc;
}

/*?????==========================================================================*/
static int32_t s5k4b2fx_video_config(enum sensor_mode_t mode,
                                    enum sensor_resolution_t res)
{
    int32_t rc;

    switch(res)
    {
    case QTR_SIZE:
        rc = s5k4b2fx_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0)
			return rc;

        CDBG("s5k4b2fx sensor configuration done!\n");
        break;

    case FULL_SIZE:
        rc =
        s5k4b2fx_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;
        break;

    default:
        return 0;
    } /* switch */

    s5k4b2fx_ctrl->prev_res = res;
    s5k4b2fx_ctrl->curr_res = res;
    s5k4b2fx_ctrl->sensormode = mode;

	return rc;
}

/*?????==========================================================================*/
static int32_t s5k4b2fx_snapshot_config(enum sensor_mode_t mode)
{
    int32_t rc = 0;

    rc = s5k4b2fx_setting(UPDATE_PERIODIC, RES_CAPTURE);
    if (rc < 0)
		return rc;

    s5k4b2fx_ctrl->curr_res = s5k4b2fx_ctrl->pict_res;
    s5k4b2fx_ctrl->sensormode = mode;
	return rc;
}

/*?????==========================================================================*/
static int32_t s5k4b2fx_raw_snapshot_config(enum sensor_mode_t mode)
{
    int32_t rc = 0;

    rc = s5k4b2fx_setting(UPDATE_PERIODIC, RES_CAPTURE);
    if (rc < 0)
		return rc;

    s5k4b2fx_ctrl->curr_res = s5k4b2fx_ctrl->pict_res;

    s5k4b2fx_ctrl->sensormode = mode;

	return rc;
}

/*==========================================================================*/
static int32_t s5k4b2fx_power_down(void)
{
    int32_t rc = 0;
	s5k4b2fx_i2c_rxdata(0x03, &rc, 0x01);
	rc &= 0xEF;
	rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x03, rc);
	if (rc < 0)
		return rc;
	mdelay(300);
	rc = 2;
	s5k4b2fx_i2c_rxdata(0x03, &rc, 0x01);
	rc |= 0x80;
	rc = s5k4b2fx_i2c_write_b(S5K4B2FX_IMG_I2C_ADDR, 0x03, rc);
	if (rc < 0)
		return rc;
	mdelay(600);

	return rc;
}

/*==========================================================================*/
static int32_t s5k4b2fx_move_focus(enum sensor_move_focus_t direction,
                                  int32_t num_steps)
{
	return 0;
}

/*==========================================================================*/
static int32_t s5k4b2fx_sensor_init(
    struct msm_camera_sensor_info *camdev)
{
    int32_t  rc;
    uint16_t chipid;

    /* enable mclk first */
    CDBG("s5k4b2fx: s5k4b2fx_sensor_init s5k4b2fx_mclk = %d\n", s5k4b2fx_mclk);
    msm_camio_clk_rate_set(s5k4b2fx_mclk);
	mdelay(50);
	if (s5k4b2fx_ctrl->prev_res == QTR_SIZE)
		rc = s5k4b2fx_setting(REG_INIT, RES_PREVIEW);
	else
		rc = s5k4b2fx_setting(REG_INIT, RES_CAPTURE);

	return rc;
}

/*????? ==========================================================================*/
static int s5k4b2fx_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&s5k4b2fx_wait_queue);
	return 0;
}

/*????? ==========================================================================*/
static int32_t s5k4b2fx_set_sensor_mode(enum sensor_mode_t mode,
                                       enum sensor_resolution_t res)
{
    int32_t rc = 0;

    switch(mode)
    {
    case SENSOR_PREVIEW_MODE:
        rc = s5k4b2fx_video_config(mode, res);
        break;

    case SENSOR_SNAPSHOT_MODE:
        rc = s5k4b2fx_snapshot_config(mode);
        break;

    case SENSOR_RAW_SNAPSHOT_MODE:
        rc = s5k4b2fx_raw_snapshot_config(mode);
        break;

    default:
        rc = -EINVAL;
        break;
    }

	return rc;
}

/*==========================================================================*/
static int s5k4b2fx_open(struct inode *inode, struct file *fp)
{
    int32_t rc = 0;

    down(&s5k4b2fx_sem);
    CDBG("s5k4b2fx: open = %d\n", s5k4b2fx_ctrl->opened);

    if (s5k4b2fx_ctrl->opened) {
        rc = 0;
        goto open_done;
    }

    rc = s5k4b2fx_sensor_init(s5k4b2fx_ctrl->sensordata);

    CDBG("s5k4b2fx_open: sensor init rc = %d\n", rc);

    if (rc >= 0)
        s5k4b2fx_ctrl->opened = 1;
    else
        CDBG("s5k4b2fx_open: sensor init failed!\n");

open_done:
    prevent_suspend();
    up(&s5k4b2fx_sem);
	return rc;
}

/*==========================================================================*/
static long s5k4b2fx_ioctl(struct file *filp, unsigned int cmd, 
                          unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    struct sensor_cfg_data_t cdata;
    long   rc = 0;

    if (copy_from_user(&cdata, (void *)argp, sizeof(struct sensor_cfg_data_t)))
        return -EFAULT;

    down(&s5k4b2fx_sem);

    switch(cmd)
    {
    case MSM_CAMSENSOR_IO_CFG: {
            switch(cdata.cfgtype)
            {
            case CFG_GET_PICT_FPS:
				s5k4b2fx_get_pict_fps(cdata.cfg.gfps.prevfps,
                                     &(cdata.cfg.gfps.pictfps));

				if (copy_to_user((void *)argp, &cdata,
                                sizeof(struct sensor_cfg_data_t)))
                    rc = -EFAULT;
                break;

            case CFG_GET_PREV_L_PF:
				cdata.cfg.prevl_pf = s5k4b2fx_get_prev_lines_pf();

				if (copy_to_user((void *)argp, &cdata,
                                sizeof(struct sensor_cfg_data_t)))
                    rc = -EFAULT;
                break;

            case CFG_GET_PREV_P_PL:
				cdata.cfg.prevp_pl = s5k4b2fx_get_prev_pixels_pl();

				if (copy_to_user((void *)argp, &cdata,
                                sizeof(struct sensor_cfg_data_t)))
                    rc = -EFAULT;
                break;

            case CFG_GET_PICT_L_PF:
				cdata.cfg.pictl_pf = s5k4b2fx_get_pict_lines_pf();

				if (copy_to_user((void *)argp, &cdata,
                                sizeof(struct sensor_cfg_data_t)))
                    rc = -EFAULT;
                break;

            case CFG_GET_PICT_P_PL:
				cdata.cfg.pictp_pl = s5k4b2fx_get_pict_pixels_pl();

				if (copy_to_user((void *)argp, &cdata,
                                sizeof(struct sensor_cfg_data_t)))
                    rc = -EFAULT;
                break;

            case CFG_GET_PICT_MAX_EXP_LC:
				cdata.cfg.pict_max_exp_lc = s5k4b2fx_get_pict_max_exp_lc();

				if (copy_to_user((void *)argp, &cdata,
                                sizeof(struct sensor_cfg_data_t)))
                    rc = -EFAULT;
                break;

            case CFG_SET_FPS:
            case CFG_SET_PICT_FPS:
                rc = s5k4b2fx_set_fps(&(cdata.cfg.fps));
                break;

            case CFG_SET_EXP_GAIN:
                rc = s5k4b2fx_write_exp_gain(cdata.cfg.exp_gain.gain,
                                            cdata.cfg.exp_gain.line);
                break;

            case CFG_SET_PICT_EXP_GAIN:
                rc = s5k4b2fx_set_pict_exp_gain(cdata.cfg.exp_gain.gain,
                                               cdata.cfg.exp_gain.line);
                break;

            case CFG_SET_MODE:
                rc = s5k4b2fx_set_sensor_mode(cdata.mode, cdata.rs);
                break;

            case CFG_PWR_DOWN:
                rc = s5k4b2fx_power_down();
                break;

            case CFG_MOVE_FOCUS:
                rc = s5k4b2fx_move_focus(cdata.cfg.focus.dir,
                                        cdata.cfg.focus.steps);
                break;

            case CFG_SET_DEFAULT_FOCUS:
                rc = s5k4b2fx_set_default_focus();
                break;

            case CFG_SET_EFFECT:
                rc = s5k4b2fx_set_default_focus();
                break;

            default:
                rc = -EFAULT;
                break;
            }
        }
        break;

    default:
        rc = -EFAULT;
        break;
    }

    up(&s5k4b2fx_sem);

    return rc;
}

/*==========================================================================*/
static int s5k4b2fx_release(struct inode *ip, struct file *fp)
{
    int rc = -EBADF;

    down(&s5k4b2fx_sem);
    if (s5k4b2fx_ctrl->opened)
        rc = s5k4b2fx_ctrl->opened = 0;
    allow_suspend();
    up(&s5k4b2fx_sem);

    return rc;
}

/*==========================================================================*/
static struct file_operations s5k4b2fx_fops = {
    .owner  = THIS_MODULE,
    .open   = s5k4b2fx_open,
    .release = s5k4b2fx_release,
    .unlocked_ioctl = s5k4b2fx_ioctl,
};

static struct miscdevice s5k4b2fx_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "s5k4b2fx",
    .fops   = &s5k4b2fx_fops,
};


static const char *S5K4B2FXVendor = "samsung";
static const char *S5K4B2FXNAME = "s5k4b2fx";
static const char *S5K4B2FXSize = "2M";



static ssize_t sensor2M_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", S5K4B2FXVendor, S5K4B2FXNAME, S5K4B2FXSize);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor2M_vendor_show, NULL);


static struct kobject *android_s5k4b2fx;

static int s5k4b2fx_sysfs_init(void)
{
	int ret ;
	printk(KERN_INFO "s5k4b2fx:kobject creat and add\n");
	android_s5k4b2fx = kobject_create_and_add("android_camera", NULL);
	if (android_s5k4b2fx == NULL) {
		printk(KERN_INFO "s5k4b2fx_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	printk(KERN_INFO "s5k4b2fx:sysfs_create_file\n");
	ret = sysfs_create_file(android_s5k4b2fx, &dev_attr_sensor.attr);
	if (ret) {
		printk(KERN_INFO "s5k4b2fx_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_s5k4b2fx);
	}
	return 0 ;
}


/*==========================================================================*/
static int s5k4b2fx_probe(struct i2c_client *client)
{
    int rc = 0;
    CDBG("s5k4b2fx_probe called!\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        goto probe_failure;

    s5k4b2fx_ctrl->sensorw = kzalloc(sizeof(struct s5k4b2fx_work_t), GFP_KERNEL);

    if (!s5k4b2fx_ctrl->sensorw) {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, s5k4b2fx_ctrl->sensorw);
    s5k4b2fx_init_client(client);
    pclient = client;
    s5k4b2fx_ctrl->client = client;
	/* Wilson Added to enable MCLK for RESET function */
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_rate_set(s5k4b2fx_mclk);
	mdelay(2);
	/****************************************************/
    rc = gpio_request(s5k4b2fx_ctrl->sensordata->sensor_reset, "s5k4b2fx");
    if (!rc)
	gpio_direction_output(s5k4b2fx_ctrl->sensordata->sensor_reset, 1);

    gpio_free(s5k4b2fx_ctrl->sensordata->sensor_reset);
    if (rc)
        goto probe_failure;

	/* Wilson Added to enable MCLK for RESET function */
	mdelay(300);
	s5k4b2fx_power_down();
	msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	/****************************************************/
    /* Register a misc device */
    rc = misc_register(&s5k4b2fx_device);
    if (rc) {
        CDBG("s5k4b2fx_probe misc_register failed!\n");
        goto probe_failure;
    }
    init_suspend();
	s5k4b2fx_sysfs_init();
    CDBG("s5k4b2fx_probe successed! rc = %d\n", rc);
    return 0;
    
probe_failure:
    CDBG("s5k4b2fx_probe failed! rc = %d\n", rc);
    return rc;
}

/*==========================================================================*/
static int s5k4b2fx_remove(struct i2c_client *client)
{
    struct s5k4b2fx_work_t *sensorw = i2c_get_clientdata(client);
    free_irq(client->irq, sensorw);
    deinit_suspend();
    i2c_detach_client(client);
    s5k4b2fx_ctrl->client = NULL;
    misc_deregister(&s5k4b2fx_device);
    kfree(sensorw);
    return 0;
}

/*==========================================================================*/
static const struct i2c_device_id s5k4b2fx_i2c_id[] = {
	{ "s5k4b2fx", 0 },
	{ }
};

static struct i2c_driver s5k4b2fx_driver = {
	.probe  = s5k4b2fx_probe,
	.remove = s5k4b2fx_remove,
	.driver = {
		.name = "s5k4b2fx",
		.owner = THIS_MODULE,
	},
	.id_table = s5k4b2fx_i2c_id,
};

/*==========================================================================*/
int32_t s5k4b2fx_init(void *pdata)
{
    int32_t rc = 0;
    struct msm_camera_sensor_info *data = 
        (struct msm_camera_sensor_info *)pdata;

    CDBG("s5k4b2fx_init called!\n");

    s5k4b2fx_ctrl = kzalloc(sizeof(struct s5k4b2fx_ctrl_t), GFP_KERNEL);
    if (!s5k4b2fx_ctrl) {
        CDBG("s5k4b2fx_init failed!\n");
        rc = -ENOMEM;
        goto init_failure;
    }

    s5k4b2fx_ctrl->fps_divider = 1 * 0x00000400;
    s5k4b2fx_ctrl->pict_fps_divider = 1 * 0x00000400;
    s5k4b2fx_ctrl->set_test = TEST_OFF;
    s5k4b2fx_ctrl->prev_res = QTR_SIZE;
    s5k4b2fx_ctrl->pict_res = FULL_SIZE;

    if (data) {
        s5k4b2fx_ctrl->sensordata = data;
        CDBG("s5k4b2fx_init calling i2c_add_driver\n");
        rc = i2c_add_driver(&s5k4b2fx_driver);
    }

init_failure:
    return rc;
}

void s5k4b2fx_exit(void)
{
	i2c_del_driver(&s5k4b2fx_driver);
}



/* Simulate a SMBus command using the i2c protocol
   No checking of parameters is done!  */
static s32 ss_i2c_smbus_xfer_emulated(struct i2c_adapter * adapter, u16 addr,
                                   unsigned short flags,
                                   char read_write, u16 command, int size,
                                   union i2c_smbus_data * data)
{
	/* So we need to generate a series of msgs. In the case of writing, we
	  need to use only one message; when reading, we need two. We initialize
	  most things with sane defaults, to keep the code below somewhat
	  simpler. */
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
	int num = read_write == I2C_SMBUS_READ?2:1;
	struct i2c_msg msg[2] = { { addr, flags, 1, msgbuf0 },
	                          { addr, flags | I2C_M_RD, 0, msgbuf1 }
	                        };
	int i;
	u8 partial_pec = 0;
	msgbuf0[0] = (command & 0x00FF);
	if (read_write == I2C_SMBUS_READ) {
		msg[0].len = 1;
		msg[1].len = 1;
	} else {
		msg[0].len = 2;
		msgbuf0[1] = data->byte;
	}
	i = ((flags & I2C_CLIENT_PEC) && size != I2C_SMBUS_QUICK
				      && size != I2C_SMBUS_I2C_BLOCK_DATA);
	if (i) {
		/* Compute PEC if first message is a write */
		if (!(msg[0].flags & I2C_M_RD)) {
			if (num == 1) /* Write only */
				ss_i2c_smbus_add_pec(&msg[0]);
			else /* Write followed by read */
				partial_pec = ss_i2c_smbus_msg_pec(0, &msg[0]);
		}
		/* Ask for PEC if last message is a read */
/*		if (msg[num-1].flags & I2C_M_RD)
			msg[num-1].len++;*/
	}
	if (i2c_transfer(adapter, msg, num) < 0) {
		return -1;
	}

	/* Check PEC if last message is a read */

	if (i && (msg[num-1].flags & I2C_M_RD)) {
		if (ss_i2c_smbus_check_pec(partial_pec, &msg[num-1]) < 0) {
			return -1;
		}
	}
	if (read_write == I2C_SMBUS_READ) {
		data->byte = msgbuf1[0];
	}
	return 0;
}

s32 ss_i2c_smbus_xfer(struct i2c_adapter * adapter, u16 addr, unsigned short flags,
                   char read_write, u16 command, int size,
                   union i2c_smbus_data * data)
{
	s32 res;

	flags &= I2C_M_TEN | I2C_CLIENT_PEC;
	res = ss_i2c_smbus_xfer_emulated(adapter,addr,flags,read_write,
	                                      command,size,data);
	return res;
}

s32 ss_i2c_smbus_write_byte_data(struct i2c_client *client, u16 command, u8 value)
{
	union i2c_smbus_data data;
	
	data.byte = value;
	return ss_i2c_smbus_xfer(client->adapter,client->addr,client->flags,
	                      I2C_SMBUS_WRITE,command,
	                      I2C_SMBUS_BYTE_DATA,&data);
}

s32 ss_i2c_smbus_read_byte_data(struct i2c_client *client, u16 command)
{
	union i2c_smbus_data data;
	
	if (ss_i2c_smbus_xfer(client->adapter,client->addr,client->flags,
		I2C_SMBUS_READ , command , I2C_SMBUS_BYTE_DATA , &data)) {
		return -1;
	}
	else
		return data.byte;
}

static u8 ss_i2c_smbus_pec(u8 crc, u8 *p, size_t count)
{
	int i;

	for(i = 0; i < count; i++)
		crc = crc8((crc ^ p[i]) << 8);
	return crc;
}

/* Assume a 7-bit address, which is reasonable for SMBus */
static u8 ss_i2c_smbus_msg_pec(u8 pec, struct i2c_msg *msg)
{
	/* The address will be sent first */
	u8 addr = (msg->addr << 1) | !!(msg->flags & I2C_M_RD);
	pec = ss_i2c_smbus_pec(pec, &addr, 1);

	/* The data buffer follows */
	return ss_i2c_smbus_pec(pec, msg->buf, msg->len);
}

/* Used for write only transactions */
static inline void ss_i2c_smbus_add_pec(struct i2c_msg *msg)
{
	msg->buf[msg->len] = ss_i2c_smbus_msg_pec(0, msg);
	msg->len++;
}

/* Return <0 on CRC error
   If there was a write before this read (most cases) we need to take the
   partial CRC from the write part into account.
   Note that this function does modify the message (we need to decrease the
   message length to hide the CRC byte from the caller). */
static int ss_i2c_smbus_check_pec(u8 cpec, struct i2c_msg *msg)
{
	u8 rpec = msg->buf[--msg->len];
	cpec = ss_i2c_smbus_msg_pec(cpec, msg);

	if (rpec != cpec) {
		pr_debug("i2c-core: Bad PEC 0x%02x vs. 0x%02x\n",
			rpec, cpec);
		return -1;
	}
	return 0;
}

static u8 crc8(u16 data)
{
	int i;

	for(i = 0; i < 8; i++) {
		if (data & 0x8000)
			data = data ^ POLY;
		data = data << 1;
	}
	return (u8)(data >> 8);
}
