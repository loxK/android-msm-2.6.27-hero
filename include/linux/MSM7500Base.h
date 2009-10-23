
#ifndef __MSM7500_BASE_H
#define __MSM7500_BASE_H


/////////////////////////////////////////////////////////////////////
// 5020    
#define GPIO1_REGS_PHYSICAL_BASE		0xA9000000
#define GPIO1_REGS_VIRTUAL_BASE		0xB2F00000
#define GPIO2_REGS_PHYSICAL_BASE		0xA9100000
#define GPIO2_REGS_VIRTUAL_BASE		0xB2E00000
#define GPIOSH1_REGS_PHYSICAL_BASE	0xA9200000
#define GPIOSH1_REGS_VIRTUAL_BASE		0xB2D00000
#define GPIOSH2_REGS_PHYSICAL_BASE	0xA9300000
#define GPIOSH2_REGS_VIRTUAL_BASE		0xB2C00000

/////////////////////////////////////////////////////////////////////
#define NAND_REGS_PHYSICAL_BASE		0xA0A00000
#define NAND_REGS_VIRTUAL_BASE		0xB1200000

/////////////////////////////////////////////////////////////////////
// UART

// Johnnie Chen, 2006/10/17
// 5020
#define UART1DM_REGS_PHYSICAL_BASE	0xA0200000
#define UART1DM_REGS_VIRTUAL_BASE	0xB1900000

#define UART2DM_REGS_PHYSICAL_BASE	0xA0300000
#define UART2DM_REGS_VIRTUAL_BASE	0xB1A00000

// 5020
#define UART1_REGS_PHYSICAL_BASE		0xA9A00000
#define UART1_REGS_VIRTUAL_BASE		0xB2600000

#define UART2_REGS_PHYSICAL_BASE		0xA9B00000
#define UART2_REGS_VIRTUAL_BASE		0xB2500000

#define UART3_REGS_PHYSICAL_BASE		0xA9C00000
#define UART3_REGS_VIRTUAL_BASE		0xB2400000

#define DEBUG_SERIAL_UART_REGS_VIRTUAL_BASE			UART1_REGS_VIRTUAL_BASE


/////////////////////////////////////////////////////////////////////
// 5020
#define USB_REGS_PHYSICAL_BASE		0xA9800000
#define USB_REGS_VIRTUAL_BASE			0xB2700000

/////////////////////////////////////////////////////////////////////
// 5020
#define CLK_CTL_REGS_PHYSICAL_BASE	0xA8600000
#define CLK_CTL_REGS_VIRTUAL_BASE		0xB3000000

/////////////////////////////////////////////////////////////////////
// 5020
#define EBI1_REGS_PHYSICAL_BASE		0xA8300000
#define EBI1_REGS_VIRTUAL_BASE			0xB3200000

/////////////////////////////////////////////////////////////////////

#define EBI2CR_REGS_PHYSICAL_BASE		0xA0D00000
#define EBI2CR_REGS_VIRTUAL_BASE		0xB0F00000

/////////////////////////////////////////////////////////////////////
// 5020
#define ARM11S_VIC_REGS_PHYSICAL_BASE	0xC0000000
#if (CSP_VERSION==1220) || (CSP_VERSION==1230)
#define ARM11S_VIC_REGS_VIRTUAL_BASE		0xB3900000
#else
#define ARM11S_VIC_REGS_VIRTUAL_BASE		0xB4F00000
#endif
/////////////////////////////////////////////////////////////////////
// 5020
#define ARM11S_CSR_REGS_PHYSICAL_BASE	0xC0100000
#if (CSP_VERSION==1220) || (CSP_VERSION==1230)
#define ARM11S_CSR_REGS_VIRTUAL_BASE		0xB3A00000
#else
#define ARM11S_CSR_REGS_VIRTUAL_BASE		0xB5000000
#endif

/////////////////////////////////////////////////////////////////////
// 5020
#define CODECSSBI_REGS_PHYSICAL_BASE	0xA8100000
//#define CODECSSBI_REGS_VIRTUAL_BASE		0xB1F00000
#define CODECSSBI_REGS_VIRTUAL_BASE		0xE1004000

/////////////////////////////////////////////////////////////////////
// 5020
// !!! Have to use Physical + offset 100, because TSSC_CTL start from 0x100
#define TSSC_REGS_PHYSICAL_BASE			0xAA300100
//#define TSSC_REGS_VIRTUAL_BASE			0xB1E00100
#define TSSC_REGS_VIRTUAL_BASE			0xE1005100

/////////////////////////////////////////////////////////////////////
// 5020
#define SDC1_REGS_PHYSICAL_BASE			0xA0400000
#define SDC1_REGS_VIRTUAL_BASE			0xB1800000

#define SDC2_REGS_PHYSICAL_BASE			0xA0500000
#define SDC2_REGS_VIRTUAL_BASE			0xB1700000

#define SDC3_REGS_PHYSICAL_BASE			0xA0600000
#define SDC3_REGS_VIRTUAL_BASE			0xB1600000

#define SDC4_REGS_PHYSICAL_BASE			0xA0700000
#define SDC4_REGS_VIRTUAL_BASE			0xB1500000

/////////////////////////////////////////////////////////////////////
// 5020
#define APWBMIS_REGS_PHYSICAL_BASE			0xA9D00000
#define APWBMIS_REGS_VIRTUAL_BASE			0xB2000000

/////////////////////////////////////////////////////////////////////
// define all constants used in msmhwioreg.h

#define MSM_EBI2ND_BASE					0xB1200000
#define MSM_EBI2ND_SIZE					0x00001000


// 5020
#define MSM_ADMH00_BASE					0xB2B00000
#define MSM_ADMH00_SIZE					0x00001000

#define MSM_ADMH01_BASE					0xB2A00400
#define MSM_ADMH01_SIZE					0x00001000

#define MSM_ADMH02_BASE					0xB2900800
#define MSM_ADMH02_SIZE					0x00001000

#define MSM_ADMH03_BASE					0xB2800C00
#define MSM_ADMH03_SIZE					0x00001000

#define MSM_CLK_CTL_BASE				CLK_CTL_REGS_VIRTUAL_BASE
#define MSM_CLK_CTL_SIZE				0x00001000

#define MSM_MSS_INTCTL_BASE				0xB8000500
#define MSM_MSS_INTCTL_SIZE				0x00001000

/////////////////////////////////////////////////////////////////////
// <<<rhine_fixme>>>
// 5020
#define MDC_BASE_PHYSICAL				0xAA500000
#define MDC_BASE_VIRT					0xB1D00000

// 5020
#define MDP_BASE_PHY					0xAA200000
#define MDP_BASE_VIRT					0xB2300000

// NOT USED
//#define MDP_LIST_VTR					0xAC230000	
//#define MDP_LIST_PHY					0x11600100

// 5020
#define MDDI_HOST_VIRTUAL_ADDRESS		0xB1C00000
#define MDDI_HOST_VIRTUAL_ADDRESS_E		0xB1C00000

// HTC_Brian_7200A
#define APWBI2C_ADDR					0xB2100000


// 5020
#define PMDH_REGS_PHYSICAL_BASE			0xAA600000
#define PMDH_REGS_VIRTUAL_BASE			0xB1C00000
#define EMDH_REGS_PHYSICAL_BASE			0xAA600000	// not define in 7225
#define EMDH_REGS_VIRTUAL_BASE			0xB1C00000	// not define in 7225


#endif // __MSM7500_BASE_H
