/*
 * Copyright (c) 2008 QUALCOMM Incorporated
 * Author: Haibo Jeff Zhong <hzhong@qualcomm.com>
 */

#include <linux/delay.h>
#include <mach/msm_iomap.h>
#include <linux/clk.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/io.h>
#include <mach/msm_camio.h>

#define out_dword(addr, val) \
	(*((unsigned long  *)(addr)) = ((unsigned long)(val)))

#define out_dword_masked_ns(io, mask, val, current_reg_content)     \
	((void) out_dword(io, ((current_reg_content & (uint32_t)(~(mask))) | \
	((uint32_t)((val) & (mask))))))

#define __inpdw(port) (*((uint32_t *) (port)))
#define in_dword_masked(addr, mask) (__inpdw(addr) & (uint32_t)mask)

#define HWIO_MDDI_CAMIF_CFG_ADDR MSM_MDC_BASE
#define HWIO_MDDI_CAMIF_CFG_RMSK 0x1fffff
#define HWIO_MDDI_CAMIF_CFG_IN \
	in_dword_masked(HWIO_MDDI_CAMIF_CFG_ADDR, HWIO_MDDI_CAMIF_CFG_RMSK)

#define HWIO_MDDI_CAMIF_CFG_OUTM(m,v) \
	out_dword_masked_ns(HWIO_MDDI_CAMIF_CFG_ADDR,\
	m, v, HWIO_MDDI_CAMIF_CFG_IN);
#define __msmhwio_outm(hwiosym, mask, val) HWIO_##hwiosym##_OUTM(mask, val)
#define HWIO_OUTM(hwiosym, mask, val) __msmhwio_outm(hwiosym, mask, val)

#define HWIO_MDDI_CAMIF_CFG_CAM_SEL_BMSK 0x2
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_SRC_SEL_BMSK 0x60000
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_INVERT_BMSK 0x80000
#define HWIO_MDDI_CAMIF_CFG_CAM_PAD_REG_SW_RESET_BMSK 0x100000

#define HWIO_MDDI_CAMIF_CFG_EXT_CAM_HSYNC_POL_SEL_BMSK 0x10000
#define HWIO_MDDI_CAMIF_CFG_EXT_CAM_VSYNC_POL_SEL_BMSK 0x8000
#define HWIO_MDDI_CAMIF_CFG_MDDI_CLK_CHICKEN_BIT_BMSK  0x80

#define HWIO_MDDI_CAMIF_CFG_CAM_SEL_SHFT 0x1
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_SRC_SEL_SHFT 0x11
#define HWIO_MDDI_CAMIF_CFG_CAM_PCLK_INVERT_SHFT 0x13
#define HWIO_MDDI_CAMIF_CFG_CAM_PAD_REG_SW_RESET_SHFT 0x14

#define HWIO_MDDI_CAMIF_CFG_EXT_CAM_HSYNC_POL_SEL_SHFT 0x10
#define HWIO_MDDI_CAMIF_CFG_EXT_CAM_VSYNC_POL_SEL_SHFT 0xF
#define HWIO_MDDI_CAMIF_CFG_MDDI_CLK_CHICKEN_BIT_SHFT  0x7

#define __msmhwio_shft(hwio_regsym, hwio_fldsym) \
	HWIO_##hwio_regsym##_##hwio_fldsym##_SHFT
#define HWIO_SHFT(hwio_regsym, hwio_fldsym) \
	__msmhwio_shft(hwio_regsym, hwio_fldsym)

#define __msmhwio_fmsk(hwio_regsym, hwio_fldsym) \
	HWIO_##hwio_regsym##_##hwio_fldsym##_BMSK

#define HWIO_FMSK(hwio_regsym, hwio_fldsym) \
	__msmhwio_fmsk(hwio_regsym, hwio_fldsym)

#define HWIO_APPS_RESET_ADDR (MSM_CLK_CTL_BASE + 0x00000210)
#define HWIO_APPS_RESET_RMSK 0x1fff
#define HWIO_APPS_RESET_VFE_BMSK 1
#define HWIO_APPS_RESET_VFE_SHFT 0
#define HWIO_APPS_RESET_IN \
	in_dword_masked(HWIO_APPS_RESET_ADDR, HWIO_APPS_RESET_RMSK)
#define HWIO_APPS_RESET_OUTM(m,v) \
	out_dword_masked_ns(HWIO_APPS_RESET_ADDR, m, v, HWIO_APPS_RESET_IN)

static struct clk *camio_vfe_mdc_clk;
static struct clk *camio_mdc_clk;
static struct clk *camio_vfe_clk;
static struct clk *camio_vfe_axi_clk;

void msm_camio_gpio_enable(void)
{
	config_camera_on_gpios();
}

void msm_camio_gpio_disable()
{
	config_camera_off_gpios();
}

int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		camio_vfe_mdc_clk =
		clk = clk_get(NULL, "vfe_mdc_clk");
		break;

	case CAMIO_MDC_CLK:
		camio_mdc_clk =
		clk = clk_get(NULL, "mdc_clk");
		break;

	case CAMIO_VFE_CLK:
		camio_vfe_clk =
		clk = clk_get(NULL, "vfe_clk");
		break;

	case CAMIO_VFE_AXI_CLK:
		camio_vfe_axi_clk =
		clk = clk_get(NULL, "vfe_axi_clk");
		break;

	default:
		break;
	}

	if (clk != NULL)
		clk_enable(clk);
	else
		rc = -1;

	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk;
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		break;

	case CAMIO_VFE_AXI_CLK:
		clk = camio_vfe_axi_clk;
		break;

	default:
		break;
	}

	if (clk != NULL) {
		clk_disable(clk);
		clk_put(clk);
	} else
		rc = -1;

	return rc;
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_vfe_clk;
	/* todo: check return */
	clk_set_rate(clk, rate);
}

int msm_camio_enable(struct platform_device *dev)
{
	msm_camio_gpio_enable();
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_VFE_AXI_CLK);
}

void msm_camio_disable(void)
{
	msm_camio_gpio_disable();
	msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_VFE_AXI_CLK);
}

void msm_camio_camif_pad_reg_reset(void)
{
	struct clk *clk = NULL;

	/* select CLKRGM_VFE_SRC_CAM_VFE_SRC:  internal source */
	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_INTERNAL);

#if 0
	HWIO_OUTM(MDDI_CAMIF_CFG,
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_SEL) |
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PCLK_SRC_SEL) |
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PCLK_INVERT),
		1 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_SEL) |
		3 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PCLK_SRC_SEL) |
		0 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PCLK_INVERT));
#else
  HWIO_OUTM(MDDI_CAMIF_CFG,
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_SEL) |
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PCLK_SRC_SEL) |
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PCLK_INVERT)  |
		HWIO_FMSK(MDDI_CAMIF_CFG, EXT_CAM_HSYNC_POL_SEL) |
		HWIO_FMSK(MDDI_CAMIF_CFG, EXT_CAM_VSYNC_POL_SEL) |
		HWIO_FMSK(MDDI_CAMIF_CFG, MDDI_CLK_CHICKEN_BIT),
		1 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_SEL) |
		3 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PCLK_SRC_SEL) |
		0 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PCLK_INVERT) |
		0 << HWIO_SHFT(MDDI_CAMIF_CFG, EXT_CAM_HSYNC_POL_SEL) |
		0 << HWIO_SHFT(MDDI_CAMIF_CFG, EXT_CAM_VSYNC_POL_SEL) |
		0 << HWIO_SHFT(MDDI_CAMIF_CFG, MDDI_CLK_CHICKEN_BIT))
#endif

	mdelay(10);

	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_EXTERNAL);
	mdelay(10);

	HWIO_OUTM(MDDI_CAMIF_CFG,
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
		1 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET));

	mdelay(10);
	HWIO_OUTM(MDDI_CAMIF_CFG,
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
		0 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET));

	mdelay(10);

#if 0
	/* select CLKRGM_VFE_SRC_TLMM_CAM_PCLK: external source */
	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_EXTERNAL);
	mdelay(10);
#endif

	clk = camio_vfe_mdc_clk;

	/* todo: check return */
	clk_set_rate(clk, 96000000);
}

void msm_camio_vfe_blk_reset(void)
{
	HWIO_OUTM(APPS_RESET, HWIO_FMSK(APPS_RESET, VFE),
		1 << HWIO_SHFT(APPS_RESET, VFE));

	mdelay(10);

	HWIO_OUTM(APPS_RESET, HWIO_FMSK(APPS_RESET, VFE),
		0 << HWIO_SHFT(APPS_RESET, VFE));

	mdelay(10);
}

void msm_camio_camif_pad_reg_reset_2(void)
{
	HWIO_OUTM(MDDI_CAMIF_CFG,
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
		1 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET));

	mdelay(10);

	HWIO_OUTM(MDDI_CAMIF_CFG,
		HWIO_FMSK(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET),
		0 << HWIO_SHFT(MDDI_CAMIF_CFG, CAM_PAD_REG_SW_RESET));

	mdelay(10);
}

void msm_camio_clk_sel(enum msm_camio_clk_src_type srctype)
{
	struct clk *clk = NULL;

	clk = camio_vfe_mdc_clk;

	if (clk != NULL) {
		switch (srctype) {
		case MSM_CAMIO_CLK_SRC_INTERNAL:
			clk_set_flags(clk, 0x00000100 << 1);
			break;

		case MSM_CAMIO_CLK_SRC_EXTERNAL:
			clk_set_flags(clk, 0x00000100);
			break;

		default:
			break;
		}
	}
}

void msm_camio_clk_axi_rate_set(int rate)
{
	struct clk *clk = camio_vfe_axi_clk;
	/* todo: check return */
	clk_set_rate(clk, rate);
}
