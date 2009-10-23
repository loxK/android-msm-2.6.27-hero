/* linux/arch/arm/mach-msm/devices.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/timed_gpio.h>

#include <mach/gpio_chip.h>
#include <linux/dma-mapping.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/board.h>
#include <mach/msm_hsusb.h>
#include <linux/usb/mass_storage_function.h>

#include <asm/mach/flash.h>
#include <asm/setup.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/android_pmem.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/proc_comm.h>

#include "clock.h"

static char *df_serialno = "000000000000";

struct flash_platform_data msm_nand_data = {
	.parts		= 0,
	.nr_parts	= 0,
};

static struct resource msm_nand_resources[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(msm_nand_resources),
	.resource	= msm_nand_resources,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.id	= -1,
};

static struct resource resources_i2c[] = {
	{
		.start	= MSM_I2C_PHYS,
		.end	= MSM_I2C_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_i2c),
	.resource	= resources_i2c,
};

#define GPIO_I2C_CLK 60
#define GPIO_I2C_DAT 61
void msm_set_i2c_mux(bool gpio, int *gpio_clk, int *gpio_dat)
{
	unsigned id;
	if (gpio) {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		*gpio_clk = GPIO_I2C_CLK;
		*gpio_dat = GPIO_I2C_DAT;
	} else {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT , 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_4MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

struct platform_device *devices[] __initdata = {
	&msm_device_nand,
	&msm_device_smd,
	&msm_device_i2c,
};

void __init msm_add_devices(void)
{
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

#define HSUSB_API_INIT_PHY_PROC	2
#define HSUSB_API_PROG		0x30000064
#define HSUSB_API_VERS		0x10001
static void internal_phy_reset(void)
{
	struct msm_rpc_endpoint *usb_ep;
	int rc;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	printk(KERN_INFO "msm_hsusb_phy_reset\n");

	usb_ep = msm_rpc_connect(HSUSB_API_PROG, HSUSB_API_VERS, 0);
	if (IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(usb_ep));
		goto close;
	}
	rc = msm_rpc_call(usb_ep, HSUSB_API_INIT_PHY_PROC,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(usb_ep);
}

/* adjust eye diagram, disable vbusvalid interrupts */
static int hsusb_phy_init_seq[] = { 0x40, 0x31, 0x1, 0x0D, 0x1, 0x10, -1 };

static char *usb_functions[] = {

#if defined(CONFIG_USB_FUNCTION_MASS_STORAGE) || defined(CONFIG_USB_FUNCTION_UMS)
	"usb_mass_storage",
#endif
#if defined(CONFIG_USB_FUNCTION_ADB)
	"adb",
#endif
#if defined(CONFIG_USB_FUNCTION_FSYNC)
	"fsync",
#endif
#if defined(CONFIG_USB_FUNCTION_DIAG)
	"diag",
#endif
#if defined(CONFIG_USB_FUNCTION_SERIAL)
	"fserial",
#endif
#if defined(CONFIG_USB_FUNCTION_PROJECTOR)
	"projector",
#endif
#if defined(CONFIG_USB_FUNCTION_MTP_TUNNEL)
	"mtp_tunnel",
#endif
#if defined(CONFIG_USB_FUNCTION_ETHER)
	"ether",
#endif

};

/* about .functions variable, please refer: drivers/usb/function/usb_function.h  */
static struct msm_hsusb_product usb_products[] = {
	{
		.product_id	= 0x0c01,
		.functions	= 0x00000001, /* usb_mass_storage */
	},
	{
		.product_id	= 0x0c02,
		.functions	= 0x00000003, /* usb_mass_storage + adb */
	},
	{
		.product_id	= 0x0c03,
		.functions	= 0x00000011, /* fserial + mass_storage */
	},
	{
		.product_id	= 0x0c04,
		.functions	= 0x00000013, /* fserial + adb + mass_storage */
	},
	{
		.product_id = 0x0c05,
		.functions	= 0x00000021, /* Projector + mass_storage */
	},
	{
		.product_id = 0x0c06,
		.functions	= 0x00000023, /* Projector + adb + mass_storage */
	},
	{
		.product_id	= 0x0c07,
		.functions	= 0x0000000B, /* diag + adb + mass_storage */
	},
	{
		.product_id = 0x0c08,
		.functions	= 0x00000009, /* diag + mass_storage */
	},
	{
		.product_id = 0x0FFE,
		.functions	= 0x00000004, /* internet sharing */
	},

};

struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_reset = internal_phy_reset,
	.phy_init_seq = hsusb_phy_init_seq,
	.vendor_id = 0x0bb4,
	.product_id = 0x0c02,
	.version = 0x0100,
	.product_name = "Android Phone",
	.manufacturer_name = "HTC",

	.functions = usb_functions,
	.num_functions = ARRAY_SIZE(usb_functions),
	.products = usb_products,
	.num_products = ARRAY_SIZE(usb_products),
};

static struct resource resources_hsusb[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
		.platform_data = &msm_hsusb_pdata,
	},
};

#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns = 1,
	.buf_size = 16384,
	.vendor = "HTC     ",
	.product = "Android Phone   ",
	.release = 0x0100,
};

static struct platform_device msm_device_usb_mass_storage = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};
#endif
void __init msm_add_usb_devices(void (*phy_reset) (void), void (*phy_shutdown) (void))
{
	if (phy_reset)
		msm_hsusb_pdata.phy_reset = phy_reset;

	if (phy_shutdown)
		msm_hsusb_pdata.phy_shutdown = phy_shutdown;

	platform_device_register(&msm_device_hsusb);
#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
	platform_device_register(&msm_device_usb_mass_storage);
#endif
}
/*
void __init msm_change_usb_id(__u16 vendor_id, __u16 product_id)
{
	msm_hsusb_pdata.vendor_id = vendor_id;
	msm_hsusb_pdata.product_id = product_id;
}
*/
static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_camera_pdata = {
	.name = "pmem_camera",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_gpu0_pdata = {
	.name = "pmem_gpu0",
	.no_allocator = 1,
	.cached = 0,
	.buffered = 1,
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.no_allocator = 1,
	.cached = 0,
	.buffered = 1,
};

#ifdef CONFIG_BUILD_CIQ
static struct android_pmem_platform_data pmem_ciq_pdata = {
	.name = "pmem_ciq",
	.no_allocator = 0,
	.cached = 0,
};
#endif

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

static struct platform_device pmem_gpu0_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &pmem_gpu0_pdata },
};

static struct platform_device pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &pmem_gpu1_pdata },
};

static struct platform_device pmem_camera_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &pmem_camera_pdata },
};

#ifdef CONFIG_BUILD_CIQ
static struct platform_device pmem_ciq_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &pmem_ciq_pdata },
};
#endif

static struct resource ram_console_resource[] = {
	{
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resource),
	.resource       = ram_console_resource,
};

void __init msm_add_mem_devices(struct msm_pmem_setting *setting)
{
	if(setting->pmem_size) {
		pmem_pdata.start = setting->pmem_start;
		pmem_pdata.size = setting->pmem_size;
		platform_device_register(&pmem_device);
	}

	if(setting->pmem_adsp_size) {
		pmem_adsp_pdata.start = setting->pmem_adsp_start;
		pmem_adsp_pdata.size = setting->pmem_adsp_size;
		platform_device_register(&pmem_adsp_device);
	}

	if(setting->pmem_gpu0_size) {
		pmem_gpu0_pdata.start = setting->pmem_gpu0_start;
		pmem_gpu0_pdata.size = setting->pmem_gpu0_size;
		platform_device_register(&pmem_gpu0_device);
	}

	if(setting->pmem_gpu1_size) {
		pmem_gpu1_pdata.start = setting->pmem_gpu1_start;
		pmem_gpu1_pdata.size = setting->pmem_gpu1_size;
		platform_device_register(&pmem_gpu1_device);
	}

	if(setting->pmem_camera_size) {
		pmem_camera_pdata.start = setting->pmem_camera_start;
		pmem_camera_pdata.size = setting->pmem_camera_size;
		platform_device_register(&pmem_camera_device);
	}

	if(setting->ram_console_size) {
		ram_console_resource[0].start = setting->ram_console_start;
		ram_console_resource[0].end = setting->ram_console_start
			+ setting->ram_console_size -1;
		platform_device_register(&ram_console_device);
	}

#ifdef CONFIG_BUILD_CIQ
	if(setting->pmem_ciq_size) {
		pmem_ciq_pdata.start = setting->pmem_ciq_start;
		pmem_ciq_pdata.size = setting->pmem_ciq_size;
		platform_device_register(&pmem_ciq_device);
	}
#endif
}

#define PM_LIBPROG      0x30000061
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif

#define HTC_PROCEDURE_SET_VIB_ON_OFF	21
#define GPIO_PMIC_VIBRATOR	(512)
#define PMIC_VIBRATOR_LEVEL	(3000)

#ifdef CONFIG_MSM_AMSS_VERSION_6225
static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
#else
static int proc_comm_vibrator_id=90;
#endif

static int pmic_vibrator_enable;
#ifdef CONFIG_MSM_AMSS_VERSION_6225
static void set_pmic_vibrator(int on)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}

	if (on)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);

	msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
}

static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
}

#endif
static int pmic_vib_configure(struct gpio_chip *chip, unsigned int gpio, unsigned long flags)
{
	return 0;
}

static int pmic_vib_get_irq_num(struct gpio_chip *chip, unsigned int gpio, unsigned int *irqp, unsigned long *irqnumflagsp)
{
	return -1;
}

static int pmic_vib_read(struct gpio_chip *chip, unsigned n)
{
	return pmic_vibrator_enable;
}

static int pmic_vib_write(struct gpio_chip *chip, unsigned n, unsigned on)
{
	pmic_vibrator_enable = on;
#ifdef CONFIG_MSM_AMSS_VERSION_6225
	if (on)
		schedule_work(&work_vibrator_on);
	else
		schedule_work(&work_vibrator_off);
#else
	unsigned nc = 0;
	msm_proc_comm(proc_comm_vibrator_id, &on, &nc);

#endif
	return 0;
}

static struct gpio_chip pmic_gpio = {
	.start = GPIO_PMIC_VIBRATOR,
	.end = GPIO_PMIC_VIBRATOR,
	.configure = pmic_vib_configure,
	.get_irq_num = pmic_vib_get_irq_num,
	.read = pmic_vib_read,
	.write = pmic_vib_write,

};

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = GPIO_PMIC_VIBRATOR,
		.max_timeout = 15000,
	}
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device pmic_vibrator = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &timed_gpio_data,
	},
};

void __init msm_init_pmic_vibrator(void)
{
	register_gpio_chip(&pmic_gpio);
#ifdef CONFIG_MSM_AMSS_VERSION_6225
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);
#endif
	platform_device_register(&pmic_vibrator);
}


static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart3[] = {
	{
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART3_PHYS,
		.end	= MSM_UART3_PHYS + MSM_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};

struct platform_device msm_device_uart3 = {
	.name	= "msm_serial",
	.id	= 2,
	.num_resources	= ARRAY_SIZE(resources_uart3),
	.resource	= resources_uart3,
};

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART1DM_RX,
		.end   = INT_UART1DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart2_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART2DM_RX,
		.end   = INT_UART2DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART2_TX_CHAN,
		.end   = DMOV_HSUART2_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART2_TX_CRCI,
		.end   = DMOV_HSUART2_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm2_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm2 = {
	.name = "msm_serial_hs",
	.id = 1,
	.num_resources = ARRAY_SIZE(msm_uart2_dm_resources),
	.resource = msm_uart2_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/*sync to arch/arm/plat-msm7k/include/mach/devices.h if order changed.*/
static struct platform_device *msm_serial_devices[] __initdata = {
	&msm_device_uart1,
	&msm_device_uart2,
	&msm_device_uart3,
	#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	#endif
};


int __init msm_add_serial_devices(unsigned num)
{
	if (num > MSM_SERIAL_NUM)
		return -EINVAL;

	return platform_device_register(msm_serial_devices[num]);
}

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_PHYS,
		.end	= MSM_SDC1_PHYS + MSM_SDC1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_PHYS,
		.end	= MSM_SDC2_PHYS + MSM_SDC2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_PHYS,
		.end	= MSM_SDC3_PHYS + MSM_SDC3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_PHYS,
		.end	= MSM_SDC4_PHYS + MSM_SDC4_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
};

int __init msm_add_sdcc_devices(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

static struct resource resources_mddi0[] = {
	{
		.start	= MSM_PMDH_PHYS,
		.end	= MSM_PMDH_PHYS + MSM_PMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_PRI,
		.end	= INT_MDDI_PRI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource resources_mddi1[] = {
	{
		.start	= MSM_EMDH_PHYS,
		.end	= MSM_EMDH_PHYS + MSM_EMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_EXT,
		.end	= INT_MDDI_EXT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mddi0 = {
	.name = "msm_mddi",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mddi0),
	.resource = resources_mddi0,
};

struct platform_device msm_device_mddi1 = {
	.name = "msm_mddi",
	.id = 1,
	.num_resources = ARRAY_SIZE(resources_mddi1),
	.resource = resources_mddi1,
};

static struct resource resources_mdp[] = {
	{
		.start	= MSM_MDP_PHYS,
		.end	= MSM_MDP_PHYS + MSM_MDP_SIZE - 1,
		.name	= "mdp",
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= INT_MDP,
		.end	= INT_MDP,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mdp = {
	.name = "msm_mdp",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mdp),
	.resource = resources_mdp,
};

#define CLOCK(clk_name, clk_id, clk_dev, clk_flags, clk_arch) {	\
	.name = clk_name, \
	.id = clk_id, \
	.flags = (clk_flags) | ((clk_arch) & CLKFLAG_ARCH_ALL), \
	.dev = clk_dev, \
	}

#define CLK_ALL(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_ALL)
#define CLK_7X00A(name, id, dev, flags) \
		CLOCK(name, id, dev, flags, CLKFLAG_ARCH_MSM7X00A)

#define OFF CLKFLAG_AUTO_OFF
#define MINMAX (CLKFLAG_USE_MIN_TO_SET | CLKFLAG_USE_MAX_TO_SET)
#define USE_MIN CLKFLAG_USE_MIN_TO_SET

struct clk msm_clocks[] = {
	CLK_ALL("adm_clk",	ADM_CLK,	NULL, 0),
	CLK_ALL("adsp_clk",	ADSP_CLK,	NULL, 0),
	CLK_ALL("ebi1_clk",	EBI1_CLK,	NULL, USE_MIN),
	CLK_ALL("ebi2_clk",	EBI2_CLK,	NULL, 0),
	CLK_ALL("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLK_ALL("mddi_clk",	EMDH_CLK,	&msm_device_mddi1.dev, OFF),
	CLK_ALL("gp_clk",		GP_CLK,		NULL, 0),
	CLK_ALL("grp_clk",	GRP_CLK,	NULL, OFF),
	CLK_ALL("i2c_clk",	I2C_CLK,	&msm_device_i2c.dev, 0),
	CLK_ALL("icodec_rx_clk",	ICODEC_RX_CLK,	NULL, 0),
	CLK_ALL("icodec_tx_clk",	ICODEC_TX_CLK,	NULL, 0),
	CLK_ALL("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_ALL("mdc_clk",	MDC_CLK,	NULL, 0),
	CLK_ALL("mdp_clk",	MDP_CLK,	&msm_device_mdp.dev, OFF),
	CLK_ALL("pbus_clk",	PBUS_CLK,	NULL, 0),
	CLK_ALL("pcm_clk",	PCM_CLK,	NULL, 0),
	CLK_ALL("mddi_clk",	PMDH_CLK,	&msm_device_mddi0.dev, OFF | MINMAX),
	CLK_ALL("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLK_ALL("sdc_clk",	SDC1_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_ALL("sdc_pclk",	SDC1_PCLK,	&msm_device_sdc1.dev, OFF),
	CLK_ALL("sdc_clk",	SDC2_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_ALL("sdc_pclk",	SDC2_PCLK,	&msm_device_sdc2.dev, OFF),
	CLK_ALL("sdc_clk",	SDC3_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_ALL("sdc_pclk",	SDC3_PCLK,	&msm_device_sdc3.dev, OFF),
	CLK_ALL("sdc_clk",	SDC4_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_ALL("sdc_pclk",	SDC4_PCLK,	&msm_device_sdc4.dev, OFF),
	CLK_ALL("tsif_clk",	TSIF_CLK,	NULL, 0),
	CLK_ALL("tsif_ref_clk",	TSIF_REF_CLK,	NULL, 0),
	CLK_ALL("tv_dac_clk",	TV_DAC_CLK,	NULL, 0),
	CLK_ALL("tv_enc_clk",	TV_ENC_CLK,	NULL, 0),
	CLK_ALL("uart_clk",	UART1_CLK,	&msm_device_uart1.dev, OFF),
	CLK_ALL("uart_clk",	UART2_CLK,	&msm_device_uart2.dev, 0),
	CLK_ALL("uart_clk",	UART3_CLK,	&msm_device_uart3.dev, OFF),
	CLK_ALL("uartdm_clk",	UART1DM_CLK,	&msm_device_uart_dm1.dev, OFF),
	CLK_ALL("uartdm_clk",	UART2DM_CLK,	&msm_device_uart_dm2.dev, 0),
	CLK_ALL("usb_hs_clk",	USB_HS_CLK,	&msm_device_hsusb.dev, OFF),
	CLK_ALL("usb_hs_pclk",	USB_HS_PCLK,	&msm_device_hsusb.dev, OFF),
	CLK_ALL("usb_otg_clk",	USB_OTG_CLK,	NULL, 0),
	CLK_ALL("vdc_clk",	VDC_CLK,	NULL, OFF | MINMAX),
	CLK_ALL("vfe_clk",	VFE_CLK,	NULL, OFF),
	CLK_ALL("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, OFF),

	CLOCK(NULL, 0, NULL, 0, 0),
};


#define ATAG_SMI 0x4d534D71
/* setup calls mach->fixup, then parse_tags, parse_cmdline
 * We need to setup meminfo in mach->fixup, so this function
 * will need to traverse each tag to find smi tag.
 */
int __init parse_tag_smi(const struct tag * tags)
{
	int smi_sz = 0, find = 0;
	struct tag * t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SMI) {
			printk(KERN_DEBUG "find the smi tag\n");
			find = 1;
			break;
		}
	}
	if (!find)
		return -1;

	printk(KERN_DEBUG "parse_tag_smi: smi size = %d\n", t->u.mem.size);
	smi_sz = t->u.mem.size;
	return smi_sz;
}
__tagtable(ATAG_SMI, parse_tag_smi);

#define ATAG_HWID 0x4d534D72
int __init parse_tag_hwid(const struct tag * tags)
{
	int hwid = 0, find = 0;
	struct tag * t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_HWID) {
			printk(KERN_DEBUG "find the hwid tag\n");
			find = 1;
			break;
		}
	}

	if(find)
		hwid = t->u.revision.rev;
	printk(KERN_DEBUG "parse_tag_hwid: hwid = 0x%x\n", hwid);
	return hwid;
}
__tagtable(ATAG_HWID, parse_tag_hwid);

#define ATAG_SKUID 0x4d534D73
int __init parse_tag_skuid(const struct tag * tags)
{
	int skuid = 0, find = 0;
	struct tag * t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SKUID) {
			printk(KERN_DEBUG "find the skuid tag\n");
			find = 1;
			break;
		}
	}

	if(find)
		skuid = t->u.revision.rev;
	printk(KERN_DEBUG "parse_tag_skuid: skuid = 0x%x\n", skuid);
	return skuid;
}
__tagtable(ATAG_SKUID, parse_tag_skuid);

#define ATAG_ENGINEERID 0x4d534D75
unsigned engineer_id;
int __init parse_tag_engineerid(const struct tag * tags)
{
	int engineerid = 0, find = 0;
	struct tag * t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_ENGINEERID) {
			printk(KERN_DEBUG "find the engineer tag\n");
			find = 1;
			break;
		}
	}

	if (find) {
		engineerid = t->u.revision.rev;
		engineer_id = engineerid;
	}
	printk(KERN_DEBUG "parse_tag_engineerid: 0x%x\n", engineerid);
	return engineerid;
}
__tagtable(ATAG_ENGINEERID, parse_tag_engineerid);

#define ATAG_HERO_PANEL_TYPE 0x4d534D74
int panel_type;
int __init tag_panel_parsing(const struct tag * tags)
{
	panel_type = tags->u.revision.rev;

	printk(KERN_DEBUG "%s: panel type = %d\n", __func__,
		panel_type);

	return panel_type;
}
__tagtable(ATAG_HERO_PANEL_TYPE, tag_panel_parsing);

/* MFG_BUILD START */
static int mfg_mode;
int __init board_mfg_mode_init(char *s)
{
	if(!strcmp(s, "normal"))
		mfg_mode = 0;
	else if(!strcmp(s, "factory2"))
		mfg_mode = 1;
	else if(!strcmp(s, "recovery"))
		mfg_mode = 2;
	else if(!strcmp(s, "charge"))
		mfg_mode = 3;

	return 1;
}
__setup("androidboot.mode=", board_mfg_mode_init);

/* MFG_BUILD END */

int board_mfg_mode(void)
{
	return mfg_mode;
}

static int __init board_serialno_setup(char *serialno)
{
	if (board_mfg_mode() || !strlen(serialno))
		msm_hsusb_pdata.serial_number = df_serialno;
	else
		msm_hsusb_pdata.serial_number = serialno;
	return 1;
}

__setup("androidboot.serialno=", board_serialno_setup);

EXPORT_SYMBOL(board_mfg_mode);
static char *keycap_tag = NULL;
static int __init board_keycaps_tag(char *get_keypads)
{
	if(strlen(get_keypads))
		keycap_tag = get_keypads;
	else
		keycap_tag = NULL;
	return 1;
}
__setup("androidboot.keycaps=", board_keycaps_tag);
void board_get_keycaps_tag(char **ret_data)
{
	*ret_data = keycap_tag;
}
EXPORT_SYMBOL(board_get_keycaps_tag);

static char *cid_tag = NULL;
static int __init board_set_cid_tag(char *get_hboot_cid)
{
	if(strlen(get_hboot_cid))
		cid_tag = get_hboot_cid;
	else
		cid_tag = NULL;
	return 1;
}
__setup("androidboot.cid=", board_set_cid_tag);
void board_get_cid_tag(char **ret_data)
{
	*ret_data = cid_tag;
}
EXPORT_SYMBOL(board_get_cid_tag);

static char *carrier_tag = NULL;
static int __init board_set_carrier_tag(char *get_hboot_carrier)
{
	if(strlen(get_hboot_carrier))
		carrier_tag = get_hboot_carrier;
	else
		carrier_tag = NULL;
	return 1;
}
__setup("androidboot.carrier=", board_set_carrier_tag);
void board_get_carrier_tag(char **ret_data)
{
	*ret_data = carrier_tag;
}
EXPORT_SYMBOL(board_get_carrier_tag);

static char *mid_tag = NULL;
static int __init board_set_mid_tag(char *get_hboot_mid)
{
	if(strlen(get_hboot_mid))
		mid_tag = get_hboot_mid;
	else
		mid_tag = NULL;
	return 1;
}
__setup("androidboot.mid=", board_set_mid_tag);
void board_get_mid_tag(char **ret_data)
{
	*ret_data = mid_tag;
}
EXPORT_SYMBOL(board_get_mid_tag);
