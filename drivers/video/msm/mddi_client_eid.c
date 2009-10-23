/* drivers/video/msm_fb/mddi_client_eid.c
 *
 * Support for eid mddi client devices with Samsung S6D05A0
 *
 * Copyright (C) 2007 HTC Incorporated
 * Author: Jay Tu (jay_tu@htc.com)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
#include <mach/proc_comm.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>

#define SLPIN		0x10
#define SLPOUT 		0x11
#define CASET		0x2a
#define PASET		0x2b
#define RAMWR		0x2c
#define TEON		0x35
#define MADCTL		0x36
#define COLMOD		0x3a
#define WRDISBV 	0x51
#define RDDISBV		0x52
#define WRCTRLD		0x53
#define WRCABC		0x55
#define WRCABCMB	0x5e
#define MIECTL1		0xca
#define BCMODE 		0xcb
#define MIECTL2		0xcc
#define MIECTL3		0xcd
#define DCON		0xef
#define DISCTL 		0xf2
#define PWRCTL 		0xf3
#define VCMCTL 		0xf4
#define SRCCTL 		0xf5
#define GAMCTL1		0xf7
#define GAMCTL2		0xf8
#define GATECTL		0xfd
#define MDDICTL		0xe0
#define DCON		0xef

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif

#define DEBUG_EID_CMD 	0

enum {
	PANEL_SHARP,
	PANEL_SAMSUNG,
	PANEL_EID_40pin,
	PANEL_EID_24pin,
	PANEL_EID_BOTTOM,
	PANEL_TPO,
	PANEL_UNKNOWN,
};

#define DEBUG_VSYNC_INT 0

#define VSYNC_EN (MSM_GPIO1_BASE + 0x800 + 0x8c)
#define VSYNC_CLEAR (MSM_GPIO1_BASE + 0x800 + 0x9c)
#define VSYNC_STATUS (MSM_GPIO1_BASE + 0x800 + 0xac)

static DECLARE_WAIT_QUEUE_HEAD(samsung_vsync_wait);

struct panel_info {
	struct msm_mddi_client_data *client_data;
	struct platform_device pdev;
	struct msm_panel_data panel_data;
	struct msmfb_callback *fb_callback;
	struct wake_lock idle_lock;
	int samsung_got_int;
	atomic_t depth;
	irqreturn_t (*vsync_isr)(int irq, void *data);
};

/* for CABC */
static struct cabc_platform_data {
	int panel;
	int shrink;
	struct msm_mddi_client_data *client;
	int (*bl_handle)(struct platform_device *, int);
} cabc_config;

static struct platform_device mddi_samsung_cabc = {
	.name = "samsung_cabc",
	.id = 0,
};

static struct clk *ebi1_clk;
static void samsung_dump_vsync(void)
{
	unsigned long rate = clk_get_rate(ebi1_clk);

	printk(KERN_INFO "STATUS %d %s EBI1 %lu\n",
		readl(VSYNC_STATUS) & 0x04,
		readl(VSYNC_EN) & 0x04 ? "ENABLED" : "DISABLED",
		rate);
}

static inline void samsung_clear_vsync(void)
{
	unsigned val;
	int retry = 1000;

	while (retry--) {
		writel((1U << (97 - 95)), VSYNC_CLEAR);
		wmb();
		val = readl(VSYNC_STATUS);
		if (!!(val & 0x04) == 0)
			break;
	}

	if (retry == 0)
		printk(KERN_ERR "%s: clear vsync failed!\n", __func__);
}

static void
samsung_request_vsync(struct msm_panel_data *panel_data,
		      struct msmfb_callback *callback)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	panel->fb_callback = callback;
	if (panel->samsung_got_int) {
		panel->samsung_got_int = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}
	
	if (atomic_read(&panel->depth) <= 0) {
		atomic_inc(&panel->depth);
		samsung_clear_vsync();
		enable_irq(gpio_to_irq(97));
	}
}

static void samsung_wait_vsync(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	if (panel->samsung_got_int) {
		panel->samsung_got_int = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}

	if (wait_event_timeout(samsung_vsync_wait, panel->samsung_got_int,
			       HZ / 2) == 0)
		printk(KERN_ERR "timeout waiting for VSYNC\n");

	panel->samsung_got_int = 0;
	/* interrupt clears when screen dma starts */
}

/* --------------------------------------------------------------------------- */

static void
eid_pwrctl(struct msm_mddi_client_data *client_data, int panel_type,
	   uint8_t prm2, uint8_t prm5)
{
#if DEBUG_EID_CMD
	int i;
#endif
	uint8_t prm[12];

	memset(prm, 0, 12);
	prm[1] = prm2;
	prm[4] = prm5;
	prm[2] = 0x2a;
	if (panel_type == PANEL_EID_40pin) {
		prm[5] = 0x33;
		prm[6] = 0x38;
		prm[7] = 0x38;
	} else if (panel_type == PANEL_TPO) {
		prm[5] = 0x33;
		prm[6] = 0x75;
		prm[7] = 0x75;
	} else {
		prm[5] = 0x33;
		prm[6] = 0x29;
		prm[7] = 0x29;
	}

	client_data->remote_write_vals(client_data, prm, PWRCTL, 12);
#if DEBUG_EID_CMD
	printk(KERN_DEBUG "0x%x ", PWRCTL);
	for (i = 0; i < 12; i++)
		printk("0x%x ", prm[i]);
	printk("\n");
#endif
}

static void
eid_cmd(struct msm_mddi_client_data *client_data, unsigned cmd,
	uint8_t *prm, int size, uint8_t attrs, ...)
{
	int i;
	uint8_t tmp;
	va_list attr_list;

	if (size <= 0)
		return;

	prm[0] = attrs;

	va_start(attr_list, attrs);

	for (i = 1; i < size; i++) {
		tmp = (uint8_t) va_arg(attr_list, int);
		prm[i] = tmp;
	}

	va_end(attr_list);
#if DEBUG_EID_CMD
	printk(KERN_DEBUG "0x%x ", cmd);
	for (i = 0; i < size; i++)
		printk("0x%x ", prm[i]);
	printk("\n");
#endif
	client_data->remote_write_vals(client_data, prm, cmd, size);
}

static int
eid_client_init(struct msm_mddi_bridge_platform_data *bridge,
		struct msm_mddi_client_data *client_data)
{
	uint8_t prm[20];
#if DEBUG_EID_CMD
	printk(KERN_DEBUG "%s: enter.\n", __func__);
#endif
	client_data->auto_hibernate(client_data, 0);

	eid_pwrctl(client_data, bridge->panel_type, 0x00, 0x00);
	eid_cmd(client_data, SLPOUT, prm, 4, 0x00, 0x00, 0x00, 0x00);
	msleep(20);

	if (bridge->panel_type == PANEL_TPO)
		eid_cmd(client_data, DISCTL, prm, 12, 0x16, 0x16, 0x0f, 0x02,
			0x03, 0x02, 0x03, 0x10, 0x00, 0x17, 0x17, 0x00);
	else
		eid_cmd(client_data, DISCTL, prm, 12, 0x16, 0x16, 0x0f, 0x11,
			0x11, 0x11, 0x11, 0x10, 0x00, 0x16, 0x16, 0x00);
	eid_pwrctl(client_data, bridge->panel_type, 0x01, 0x00);

	if (bridge->panel_type == PANEL_EID_40pin)
		eid_cmd(client_data, VCMCTL, prm, 8, 0x2a, 0x2a, 0x19, 0x19,
			0x00, 0x00, 0x00, 0x00);
	else if (bridge->panel_type == PANEL_TPO)
		eid_cmd(client_data, VCMCTL, prm, 8, 0x75, 0x75, 0x7f, 0x7f,
			0x77, 0x00, 0x00, 0x00);
	else
		eid_cmd(client_data, VCMCTL, prm, 8, 0x1b, 0x1b, 0x18, 0x18,
			0x00, 0x00, 0x00, 0x00);

	if (bridge->panel_type == PANEL_TPO)
		eid_cmd(client_data, SRCCTL, prm, 8, 0x12, 0x00, 0x06, 0x01,
			0x01, 0x1d, 0x00, 0x00);
	else
		eid_cmd(client_data, SRCCTL, prm, 8, 0x12, 0x00, 0x0a, 0x01,
			0x01, 0x1d, 0x00, 0x00);
	if (bridge->panel_type == PANEL_TPO)
		eid_cmd(client_data, GATECTL, prm, 4, 0x11, 0x3b, 0x00, 0x00);
	else
		eid_cmd(client_data, GATECTL, prm, 4, 0x44, 0x3b, 0x00, 0x00);
	msleep(20);

	eid_pwrctl(client_data, bridge->panel_type, 0x03, 0x00);
	msleep(20);

	eid_pwrctl(client_data, bridge->panel_type, 0x07, 0x00);
	msleep(20);

	eid_pwrctl(client_data, bridge->panel_type, 0x0f, 0x02);
	msleep(20);

	eid_pwrctl(client_data, bridge->panel_type, 0x1f, 0x02);
	msleep(20);

	eid_pwrctl(client_data, bridge->panel_type, 0x3f, 0x08);
	msleep(30);

	eid_pwrctl(client_data, bridge->panel_type, 0x7f, 0x08);
	msleep(40);

	eid_cmd(client_data, TEON, prm, 4, 0x00, 0x00, 0x00, 0x00);

	if (bridge->panel_type == PANEL_EID_BOTTOM ||
	    bridge->panel_type == PANEL_TPO)
		eid_cmd(client_data, MADCTL, prm, 4, 0x48, 0x00, 0x00, 0x00);
	else
		eid_cmd(client_data, MADCTL, prm, 4, 0x98, 0x00, 0x00, 0x00);
	eid_cmd(client_data, COLMOD, prm, 4, 0x66, 0x00, 0x00, 0x00); /* 666 */

	if (bridge->panel_type == PANEL_EID_40pin) {
		eid_cmd(client_data, GAMCTL1, prm, 16, 0x00, 0x0d, 0x00, 0x05,
			0x1f, 0x2b, 0x2a, 0x2b, 0x12, 0x12, 0x10, 0x16, 0x06,
			0x22, 0x22, 0x00);
		eid_cmd(client_data, GAMCTL2, prm, 16, 0x00, 0x0d, 0x00, 0x05,
			0x1f, 0x2b, 0x2a, 0x2b, 0x12, 0x12, 0x10, 0x16, 0x06,
			0x22, 0x22, 0x00);
	} else if (bridge->panel_type == PANEL_TPO) {
		eid_cmd(client_data, GAMCTL1, prm, 16, 0x83, 0x1d, 0x00, 0x1f,
			0x2f, 0x2a, 0x27, 0x2a, 0x11, 0x0d, 0x07, 0x0e,
			0x08, 0x22, 0x22, 0x00);
		eid_cmd(client_data, GAMCTL2, prm, 16, 0x80, 0x1a, 0x00, 0x19,
			0x29, 0x22, 0x22, 0x23, 0x17, 0x11, 0x07, 0x0e,
			0x08, 0x22, 0x22, 0x00);
		eid_cmd(client_data, 0xf9, prm, 16, 0x83, 0x1d, 0x00, 0x1f,
			0x2f, 0x2a, 0x27, 0x2a, 0x11, 0x0d, 0x07, 0x0e,
			0x08, 0x22, 0x22, 0x00);
		eid_cmd(client_data, 0xfa, prm, 16, 0x80, 0x1a, 0x00, 0x19,
			0x29, 0x22, 0x22, 0x23, 0x17, 0x11, 0x07, 0x0e,
			0x08, 0x22, 0x22, 0x00);
		eid_cmd(client_data, 0xfb, prm, 16, 0x83, 0x1d, 0x00, 0x1f,
			0x2f, 0x2a, 0x27, 0x2a, 0x11, 0x0d, 0x07, 0x0e,
			0x08, 0x22, 0x22, 0x00);
		eid_cmd(client_data, 0xfc, prm, 16, 0x80, 0x1a, 0x00, 0x19,
			0x29, 0x22, 0x22, 0x23, 0x17, 0x11, 0x07, 0x0e,
			0x08, 0x22, 0x22, 0x00);
		eid_cmd(client_data, BCMODE, prm, 4, 0x02, 0x00, 0x00, 0x00);
		eid_cmd(client_data, WRCABC, prm, 4, 0x00, 0x00, 0x00, 0x00);
	} else {
		eid_cmd(client_data, GAMCTL1, prm, 16, 0x14, 0x00, 0x00, 0x1d,
			0x25, 0x32, 0x34, 0x36, 0x06, 0x03, 0x08, 0x16,
			0x00, 0x41, 0x81, 0x00);
		eid_cmd(client_data, GAMCTL2, prm, 16, 0x00, 0x14, 0x00, 0x1d,
			0x25, 0x32, 0x34, 0x36, 0x06, 0x03, 0x08, 0x16,
			0x00, 0x41, 0x81, 0x00);
		eid_cmd(client_data, 0xf9, prm, 16, 0x00, 0x00, 0x00, 0x1d,
			0x25, 0x2b, 0x2a, 0x2b, 0x13, 0x11, 0x11, 0x11,
			0x00, 0x12, 0x44, 0x00);
		eid_cmd(client_data, 0xfa, prm, 16, 0x00, 0x00, 0x00, 0x1d,
			0x25, 0x2b, 0x2a, 0x2b, 0x13, 0x11, 0x11, 0x11,
			0x00, 0x12, 0x44, 0x00);
		eid_cmd(client_data, 0xfb, prm, 16, 0x14, 0x00, 0x00, 0x28,
			0x34, 0x3a, 0x3a, 0x3c, 0x00, 0x00, 0x00, 0x04,
			0x00, 0x11, 0x11, 0x00);
		eid_cmd(client_data, 0xfc, prm, 16, 0x00, 0x14, 0x00, 0x28,
			0x34, 0x3a, 0x3a, 0x3c, 0x00, 0x00, 0x00, 0x04,
			0x00, 0x11, 0x11, 0x00);

		/* auto */
		eid_cmd(client_data, BCMODE, prm, 4, 0x02, 0x00, 0x00, 0x00);
		/* CABC off */
		eid_cmd(client_data, WRCABC, prm, 4, 0x00, 0x00, 0x00, 0x00);
	}
	eid_cmd(client_data, CASET, prm, 4, 0x00, 0x00, 0x01, 0x3f);
	eid_cmd(client_data, PASET, prm, 4, 0x00, 0x00, 0x01, 0xdf);
	eid_cmd(client_data, DCON, prm, 4, 0x06, 0x00, 0x00, 0x00);
	eid_cmd(client_data, RAMWR, prm, 4, 0x00, 0x00, 0x00, 0x00);
	msleep(100);
	eid_cmd(client_data, DCON, prm, 4, 0x07, 0x00, 0x00, 0x00);

	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
eid_client_uninit(struct msm_mddi_bridge_platform_data *bridge,
		  struct msm_mddi_client_data *client_data)
{
	uint8_t prm[20];

	B(KERN_DEBUG "%s: enter.\n", __func__);
	client_data->auto_hibernate(client_data, 0);

	eid_cmd(client_data, DCON, prm, 4, 0x06, 0x00, 0x00, 0x00);
	msleep(45);
	if (bridge->panel_type == PANEL_TPO)
		eid_cmd(client_data, DCON, prm, 4, 0x07, 0x00, 0x00, 0x00);
	else
		eid_cmd(client_data, DCON, prm, 4, 0x00, 0x00, 0x00, 0x00);
	msleep(30);
	eid_pwrctl(client_data, bridge->panel_type, 0x00, 0x00);
	eid_cmd(client_data, SLPIN, prm, 4, 0x00, 0x00, 0x00, 0x00);
	if (bridge->panel_type == PANEL_EID_24pin)
		eid_cmd(client_data, MDDICTL, prm, 4, 0x02, 0x00, 0x00, 0x00);
	msleep(210);

	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
samsung_client_init(struct msm_mddi_bridge_platform_data *bridge,
		    struct msm_mddi_client_data *client_data)
{
	uint8_t prm[20];

	B(KERN_DEBUG "%s: enter.\n", __func__);
	client_data->auto_hibernate(client_data, 0);

	eid_cmd(client_data, PWRCTL, prm, 12, 0x00, 0x00, 0x26, 0x26, 0x0a,
		0x23, 0x6f, 0x6f, 0x00, 0x00, 0x00, 0x00);
	eid_cmd(client_data, SLPOUT, prm, 4, 0x00, 0x00, 0x00, 0x00);
	msleep(20);

	eid_cmd(client_data, DISCTL, prm, 12, 0x15, 0x15, 0x0f, 0x11, 0x11,
		0x11, 0x11, 0x10, 0x04, 0x15, 0x15, 0x00);
	eid_cmd(client_data, PWRCTL, prm, 12, 0x00, 0x01, 0x26, 0x26, 0x0a,
		0x23, 0x6f, 0x6f, 0x00, 0x00, 0x00, 0x00);
	eid_cmd(client_data, VCMCTL, prm, 8, 0x6d, 0x6d, 0x7a, 0x7a, 0x44, 0x00,
		0x00, 0x00);
	eid_cmd(client_data, SRCCTL, prm, 8, 0x12, 0x00, 0x06, 0xf1, 0x41, 0x1f,
		0x00, 0x00);
	eid_cmd(client_data, GATECTL, prm, 4, 0x33, 0x3b, 0x00, 0x00);
	msleep(20);

	eid_cmd(client_data, PWRCTL, prm, 12, 0x00, 0x03, 0x26, 0x26, 0x0a,
		0x23, 0x6f, 0x6f, 0x00, 0x00, 0x00, 0x00);
	msleep(20);

	eid_cmd(client_data, PWRCTL, prm, 12, 0x00, 0x07, 0x26, 0x26, 0x0a,
		0x23, 0x6f, 0x6f, 0x00, 0x00, 0x00, 0x00);
	msleep(20);

	eid_cmd(client_data, PWRCTL, prm, 12, 0x00, 0x0f, 0x26, 0x26, 0x0a,
		0x23, 0x6f, 0x6f, 0x00, 0x00, 0x00, 0x00);
	msleep(20);

	eid_cmd(client_data, PWRCTL, prm, 12, 0x00, 0x3f, 0x26, 0x26, 0x0a,
		0x23, 0x6f, 0x6f, 0x00, 0x00, 0x00, 0x00);
	msleep(25);

	eid_cmd(client_data, PWRCTL, prm, 12, 0x00, 0x7f, 0x26, 0x26, 0x0a,
		0x23, 0x6f, 0x6f, 0x00, 0x00, 0x00, 0x00);
	msleep(35);

	eid_cmd(client_data, TEON, prm, 4, 0x00, 0x00, 0x00, 0x00);
	eid_cmd(client_data, MADCTL, prm, 4, 0x98, 0x00, 0x00, 0x00);
	eid_cmd(client_data, COLMOD, prm, 4, 0x66, 0x00, 0x00, 0x00);
	eid_cmd(client_data, GAMCTL1, prm, 16, 0x00, 0x1c, 0x00, 0x04,
		0x3b, 0x38, 0x33, 0x33, 0x0d, 0x0e, 0x0d, 0x16,
		0x07, 0x02, 0x22, 0x00);
	eid_cmd(client_data, GAMCTL2, prm, 16, 0x00, 0x20, 0x00, 0x02,
		0x3b, 0x38, 0x33, 0x32, 0x0d, 0x0e, 0x0d, 0x16,
		0x07, 0x02, 0x22, 0x00);
	eid_cmd(client_data, 0xf9, prm, 16, 0x00, 0x1c, 0x00, 0x02,
		0x39, 0x36, 0x31, 0x31, 0x12, 0x13, 0x12, 0x1b,
		0x0c, 0x02, 0x22, 0x00);
	eid_cmd(client_data, 0xfa, prm, 16, 0x00, 0x20, 0x00, 0x00,
		0x39, 0x36, 0x31, 0x30, 0x12, 0x13, 0x12, 0x1b,
		0x0c, 0x02, 0x22, 0x00);
	eid_cmd(client_data, 0xfb, prm, 16, 0x00, 0x1c, 0x00, 0x05,
		0x37, 0x34, 0x30, 0x2b, 0x19, 0x1f, 0x1e, 0x27,
		0x18, 0x02, 0x22, 0x00);
	eid_cmd(client_data, 0xfc, prm, 16, 0x00, 0x20, 0x00, 0x03,
		0x37, 0x34, 0x2f, 0x2b, 0x19, 0x1f, 0x1d, 0x27,
		0x18, 0x02, 0x22, 0x00);
	eid_cmd(client_data, BCMODE, prm, 4, 0x02, 0x00, 0x00, 0x00);
	eid_cmd(client_data, WRCABC, prm, 4, 0x00, 0x00, 0x00, 0x00);
	eid_cmd(client_data, CASET, prm, 4, 0x00, 0x00, 0x01, 0x3f);
	eid_cmd(client_data, PASET, prm, 4, 0x00, 0x00, 0x01, 0xdf);
	eid_cmd(client_data, DCON, prm, 4, 0x06, 0x00, 0x01, 0xdf);
	eid_cmd(client_data, RAMWR, prm, 4, 0x00, 0x00, 0x00, 0x00);
	msleep(40);
	eid_cmd(client_data, DCON, prm, 4, 0x07, 0x00, 0x01, 0xdf);

	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
samsung_client_uninit(struct msm_mddi_bridge_platform_data *bridge,
		      struct msm_mddi_client_data *client_data)
{
	uint8_t prm[20];

	B(KERN_DEBUG "%s: enter.\n", __func__);

	client_data->auto_hibernate(client_data, 0);

	eid_cmd(client_data, 0xf0, prm, 4, 0x5a, 0x30, 0x00, 0x00);
	eid_cmd(client_data, 0xb0, prm, 4, 0x01, 0x00, 0x00, 0x00);

	client_data->auto_hibernate(client_data, 1);
	return 0;
}

/* got called by msmfb_suspend */
static int samsung_suspend(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
		client_data->private_client_data;
	int ret;

	wake_lock(&panel->idle_lock);
	ret = bridge_data->uninit(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);
	if (ret) {
		printk(KERN_ERR "mddi samsung client: non zero return from "
				"uninit\n");
		return ret;
	}

	client_data->suspend(client_data);
	return 0;
}

/* got called by msmfb_resume */
static int samsung_resume(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
		client_data->private_client_data;
	int ret;

	client_data->resume(client_data);

	wake_lock(&panel->idle_lock);
	ret = bridge_data->init(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);
	return ret;
}

static int samsung_blank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	if (cabc_config.bl_handle)
		cabc_config.bl_handle(&mddi_samsung_cabc, LED_OFF);
	return bridge_data->blank(bridge_data, client_data);
}

static int samsung_unblank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	if (cabc_config.bl_handle) {
		mdelay(40);
		cabc_config.bl_handle(&mddi_samsung_cabc, LED_FULL);
	}
	return bridge_data->unblank(bridge_data, client_data);
}

static irqreturn_t eid_vsync_interrupt(int irq, void *data)
{
	struct panel_info *panel = data;

#if DEBUG_VSYNC_INT
	uint32_t val;
	val = readl(VSYNC_STATUS);
	if (!!(val & 0x04) == 0)
		printk(KERN_ERR "BUG!! val: %x\n", val);
#endif
	if (atomic_read(&panel->depth) > 0) {
		atomic_dec(&panel->depth);
		disable_irq(gpio_to_irq(97));
	}

	if (panel->fb_callback) {
		panel->fb_callback->func(panel->fb_callback);
		panel->fb_callback = NULL;
	}

	panel->samsung_got_int = 1;
	wake_up(&samsung_vsync_wait);
	return IRQ_HANDLED;
}

static int setup_vsync(struct panel_info *panel, int init)
{
	int ret;
	int gpio = 97;
	uint32_t config;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	config = PCOM_GPIO_CFG(97, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	ret = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	if (ret)
		goto err_gpio_direction_input_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;
	
	samsung_clear_vsync();
	ret = request_irq(irq, panel->vsync_isr, IRQF_TRIGGER_HIGH,
			"vsync", panel);
	if (ret)
		goto err_request_irq_failed;
	disable_irq(irq);

	printk(KERN_INFO "vsync on gpio %d now %d\n", gpio,
			gpio_get_value(gpio));
	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), panel->client_data);
err_request_irq_failed:
err_get_irq_num_failed:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_samsung_probe(struct platform_device *pdev)
{
	int ret, err = -EINVAL;
	struct panel_info *panel;
	struct msm_mddi_client_data *client_data = pdev->dev.platform_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	B(KERN_DEBUG "%s: enter\n", __func__);

	panel = kzalloc(sizeof(struct panel_info), GFP_KERNEL);
	if (!panel) {
		err = -ENOMEM;
		goto err_out;
	}
	platform_set_drvdata(pdev, panel);

	if ((bridge_data->panel_type == PANEL_EID_40pin) ||
			(bridge_data->panel_type == PANEL_EID_24pin) ||
			(bridge_data->panel_type == PANEL_TPO) ||
			(bridge_data->panel_type == PANEL_EID_BOTTOM)) {
		bridge_data->init = eid_client_init;
		bridge_data->uninit = eid_client_uninit;
		panel->vsync_isr = eid_vsync_interrupt;
		cabc_config.shrink = 1;
	} else if (bridge_data->panel_type == PANEL_SAMSUNG) {
		bridge_data->init = samsung_client_init;
		bridge_data->uninit = samsung_client_uninit;
		panel->vsync_isr = eid_vsync_interrupt;
	} else {
		err = -EINVAL;
		goto err_panel;
	}

	ret = setup_vsync(panel, 1);
	if (ret) {
		dev_err(&pdev->dev, "mddi_samsung_setup_vsync failed\n");
		err = -EIO;
		goto err_panel;
	}

	if (bridge_data->caps & MSMFB_CAP_CABC) {
		printk(KERN_INFO "CABC enabled\n");
		cabc_config.panel = bridge_data->panel_type;
		cabc_config.client = client_data;
		mddi_samsung_cabc.dev.platform_data = &cabc_config;
		platform_device_register(&mddi_samsung_cabc);
	}

	panel->client_data = client_data;
	panel->panel_data.suspend = samsung_suspend;
	panel->panel_data.resume = samsung_resume;
	panel->panel_data.wait_vsync = samsung_wait_vsync;
	panel->panel_data.request_vsync = samsung_request_vsync;
	panel->panel_data.blank = samsung_blank;
	panel->panel_data.unblank = samsung_unblank;
	/* panel->panel_data.clear_vsync = samsung_clear_vsync; */
	panel->panel_data.dump_vsync = samsung_dump_vsync;
	panel->panel_data.fb_data = &bridge_data->fb_data;
	panel->panel_data.caps = ~MSMFB_CAP_PARTIAL_UPDATES;
	atomic_set(&panel->depth, 0);
	panel->pdev.name = "msm_panel";
	panel->pdev.id = pdev->id;
	panel->pdev.resource = client_data->fb_resource;
	panel->pdev.num_resources = 1;
	panel->pdev.dev.platform_data = &panel->panel_data;
	platform_device_register(&panel->pdev);
	wake_lock_init(&panel->idle_lock, WAKE_LOCK_IDLE, "eid_idle_lock");
	/* for debuging vsync issue */
	ebi1_clk = clk_get(NULL, "ebi1_clk");

	return 0;

err_panel:
	kfree(panel);
err_out:
	return err;
}

static int mddi_samsung_remove(struct platform_device *pdev)
{
	struct panel_info *panel = platform_get_drvdata(pdev);

	setup_vsync(panel, 0);
	kfree(panel);
	return 0;
}

static struct platform_driver mddi_client_0101_0000 = {
	.probe = mddi_samsung_probe,
	.remove = mddi_samsung_remove,
	.driver = {.name = "mddi_c_0101_0000"},
};

static int __init mddi_client_samsung_init(void)
{
	return platform_driver_register(&mddi_client_0101_0000);
}

module_init(mddi_client_samsung_init);
