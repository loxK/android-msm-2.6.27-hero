#ifndef __MSM_VFE8X_REG_H__
#define __MSM_VFE8X_REG_H__

#include <asm/arch/msm_iomap.h>
#include <asm/arch/camera.h>
#include "msm_vfe8x.h"

/* at start of camif,  bit 1:0 = 0x01: enable
 * image data capture at frame boundary. */
#define CAMIF_COMMAND_START  0x00000005

/* bit 2= 0x1:  clear the CAMIF_STATUS register
 * value. */
#define CAMIF_COMMAND_CLEAR  0x00000004

/* at stop of vfe pipeline, for now it is assumed
 * that camif will stop at any time. Bit 1:0 = 0x10:
 * disable image data capture immediately. */
#define CAMIF_COMMAND_STOP_IMMEDIATELY  0x00000002

/* at stop of vfe pipeline, for now it is assumed
 * that camif will stop at any time. Bit 1:0 = 0x00:
 * disable image data capture at frame boundary */
#define CAMIF_COMMAND_STOP_AT_FRAME_BOUNDARY  0x00000000

/* to halt axi bridge */
#define AXI_HALT  0x00000001

/* clear the halt bit. */
#define AXI_HALT_CLEAR  0x00000000

/* reset the pipeline when stop command is issued.
 * (without reset the register.) bit 26-31 = 0,
 * domain reset, bit 0-9 = 1 for module reset, except
 * register module. */
#define VFE_RESET_UPON_STOP_CMD  0x000003ef

/* reset the pipeline when reset command.
 * bit 26-31 = 0, domain reset, bit 0-9 = 1 for module reset. */
#define VFE_RESET_UPON_RESET_CMD  0x000003ff

/* bit 5 is for axi status idle or busy.
 * 1 =  halted,  0 = busy */
#define AXI_STATUS_BUSY_MASK 0x00000020

/* bit 0 & bit 1 = 1, both y and cbcr irqs need to be present
 * for frame done interrupt */
#define VFE_COMP_IRQ_BOTH_Y_CBCR 3

/* bit 1 = 1, only cbcr irq triggers frame done interrupt */
#define VFE_COMP_IRQ_CBCR_ONLY 2

/* bit 0 = 1, only y irq triggers frame done interrupt */
#define VFE_COMP_IRQ_Y_ONLY 1

/* bit 0 = 1, PM go;   bit1 = 1, PM stop */
#define VFE_PERFORMANCE_MONITOR_GO   0x00000001
#define VFE_PERFORMANCE_MONITOR_STOP 0x00000002

/* bit 0 = 1, test gen go;   bit1 = 1, test gen stop */
#define VFE_TEST_GEN_GO   0x00000001
#define VFE_TEST_GEN_STOP 0x00000002

/* the chroma is assumed to be interpolated between
 * the luma samples.  JPEG 4:2:2 */
#define VFE_CHROMA_UPSAMPLE_INTERPOLATED 0

/* constants for irq registers */
#define VFE_DISABLE_ALL_IRQS 0
/* bit =1 is to clear the corresponding bit in VFE_IRQ_STATUS.  */
#define VFE_CLEAR_ALL_IRQS   0xffffffff
/* imask for while waiting for stop ack,  driver has already
 * requested stop, waiting for reset irq,
 * bit 29,28,27,26 for async timer, bit 9 for reset */
#define VFE_IMASK_WHILE_STOPPING  0x3c000200

/* when normal case, don't want to block error status.
 * bit 0,6,20,21,22,30,31 */
#define VFE_IMASK_ERROR_ONLY             0xC0700041
#define VFE_REG_UPDATE_TRIGGER           1
#define VFE_PM_BUF_MAX_CNT_MASK          0xFF
#define VFE_DMI_CFG_DEFAULT              0x00000100
#define LENS_ROLL_OFF_DELTA_TABLE_OFFSET 32
#define VFE_AF_PINGPONG_STATUS_BIT       0x100
#define VFE_AWB_PINGPONG_STATUS_BIT      0x200

/* VFE I/O registers */
#define VFE_HW_VERSION                     (MSM_VFE_BASE + 0x00000000)
#define VFE_GLOBAL_RESET_CMD               (MSM_VFE_BASE + 0x00000004)
#define VFE_MODULE_RESET                   (MSM_VFE_BASE + 0x00000008)
#define VFE_CGC_OVERRIDE                   (MSM_VFE_BASE + 0x0000000C)
#define VFE_MODULE_CFG                     (MSM_VFE_BASE + 0x00000010)
#define VFE_CFG                            (MSM_VFE_BASE + 0x00000014)
#define VFE_IRQ_MASK                       (MSM_VFE_BASE + 0x00000018)
#define VFE_IRQ_CLEAR                      (MSM_VFE_BASE + 0x0000001C)
#define VFE_IRQ_STATUS                     (MSM_VFE_BASE + 0x00000020)
#define VFE_IRQ_COMPOSITE_MASK             (MSM_VFE_BASE + 0x00000024)
#define VFE_BUS_CMD                        (MSM_VFE_BASE + 0x00000028)
#define VFE_BUS_CFG                        (MSM_VFE_BASE + 0x0000002C)
#define VFE_BUS_ENC_Y_WR_PING_ADDR         (MSM_VFE_BASE + 0x00000030)
#define VFE_BUS_ENC_Y_WR_PONG_ADDR         (MSM_VFE_BASE + 0x00000034)
#define VFE_BUS_ENC_Y_WR_IMAGE_SIZE        (MSM_VFE_BASE + 0x00000038)
#define VFE_BUS_ENC_Y_WR_BUFFER_CFG        (MSM_VFE_BASE + 0x0000003C)
#define VFE_BUS_ENC_CBCR_WR_PING_ADDR      (MSM_VFE_BASE + 0x00000040)
#define VFE_BUS_ENC_CBCR_WR_PONG_ADDR      (MSM_VFE_BASE + 0x00000044)
#define VFE_BUS_ENC_CBCR_WR_IMAGE_SIZE     (MSM_VFE_BASE + 0x00000048)
#define VFE_BUS_ENC_CBCR_WR_BUFFER_CFG     (MSM_VFE_BASE + 0x0000004C)
#define VFE_BUS_VIEW_Y_WR_PING_ADDR        (MSM_VFE_BASE + 0x00000050)
#define VFE_BUS_VIEW_Y_WR_PONG_ADDR        (MSM_VFE_BASE + 0x00000054)
#define VFE_BUS_VIEW_Y_WR_IMAGE_SIZE       (MSM_VFE_BASE + 0x00000058)
#define VFE_BUS_VIEW_Y_WR_BUFFER_CFG       (MSM_VFE_BASE + 0x0000005C)
#define VFE_BUS_VIEW_CBCR_WR_PING_ADDR     (MSM_VFE_BASE + 0x00000060)
#define VFE_BUS_VIEW_CBCR_WR_PONG_ADDR     (MSM_VFE_BASE + 0x00000064)
#define VFE_BUS_VIEW_CBCR_WR_IMAGE_SIZE    (MSM_VFE_BASE + 0x00000068)
#define VFE_BUS_VIEW_CBCR_WR_BUFFER_CFG    (MSM_VFE_BASE + 0x0000006C)
#define VFE_BUS_STATS_AF_WR_PING_ADDR      (MSM_VFE_BASE + 0x00000070)
#define VFE_BUS_STATS_AF_WR_PONG_ADDR      (MSM_VFE_BASE + 0x00000074)
#define VFE_BUS_STATS_AWB_WR_PING_ADDR     (MSM_VFE_BASE + 0x00000078)
#define VFE_BUS_STATS_AWB_WR_PONG_ADDR     (MSM_VFE_BASE + 0x0000007C)
#define VFE_BUS_STATS_HIST_WR_PING_ADDR    (MSM_VFE_BASE + 0x00000080)
#define VFE_BUS_STATS_HIST_WR_PONG_ADDR    (MSM_VFE_BASE + 0x00000084)
#define VFE_BUS_STATS_WR_PRIORITY          (MSM_VFE_BASE + 0x00000088)
#define VFE_BUS_STRIPE_RD_ADDR_0           (MSM_VFE_BASE + 0x0000008C)
#define VFE_BUS_STRIPE_RD_ADDR_1           (MSM_VFE_BASE + 0x00000090)
#define VFE_BUS_STRIPE_RD_ADDR_2           (MSM_VFE_BASE + 0x00000094)
#define VFE_BUS_STRIPE_RD_ADDR_3           (MSM_VFE_BASE + 0x00000098)
#define VFE_BUS_STRIPE_RD_VSIZE            (MSM_VFE_BASE + 0x0000009C)
#define VFE_BUS_STRIPE_RD_HSIZE            (MSM_VFE_BASE + 0x000000A0)
#define VFE_BUS_STRIPE_RD_BUFFER_CFG       (MSM_VFE_BASE + 0x000000A4)
#define VFE_BUS_STRIPE_RD_UNPACK_CFG       (MSM_VFE_BASE + 0x000000A8)
#define VFE_BUS_STRIPE_RD_UNPACK           (MSM_VFE_BASE + 0x000000AC)
#define VFE_BUS_STRIPE_RD_PAD_SIZE         (MSM_VFE_BASE + 0x000000B0)
#define VFE_BUS_STRIPE_RD_PAD_L_UNPACK     (MSM_VFE_BASE + 0x000000B4)
#define VFE_BUS_STRIPE_RD_PAD_R_UNPACK     (MSM_VFE_BASE + 0x000000B8)
#define VFE_BUS_STRIPE_RD_PAD_TB_UNPACK    (MSM_VFE_BASE + 0x000000BC)
#define VFE_BUS_PINGPONG_IRQ_EN            (MSM_VFE_BASE + 0x000000C0)
#define VFE_BUS_PINGPONG_STATUS            (MSM_VFE_BASE + 0x000000C4)
#define VFE_BUS_PM_CMD                     (MSM_VFE_BASE + 0x000000C8)
#define VFE_BUS_PM_CFG                     (MSM_VFE_BASE + 0x000000CC)
#define VFE_BUS_ENC_Y_WR_PM_STATS_0        (MSM_VFE_BASE + 0x000000D0)
#define VFE_BUS_ENC_Y_WR_PM_STATS_1        (MSM_VFE_BASE + 0x000000D4)
#define VFE_BUS_ENC_CBCR_WR_PM_STATS_0     (MSM_VFE_BASE + 0x000000D8)
#define VFE_BUS_ENC_CBCR_WR_PM_STATS_1     (MSM_VFE_BASE + 0x000000DC)
#define VFE_BUS_VIEW_Y_WR_PM_STATS_0       (MSM_VFE_BASE + 0x000000E0)
#define VFE_BUS_VIEW_Y_WR_PM_STATS_1       (MSM_VFE_BASE + 0x000000E4)
#define VFE_BUS_VIEW_CBCR_WR_PM_STATS_0    (MSM_VFE_BASE + 0x000000E8)
#define VFE_BUS_VIEW_CBCR_WR_PM_STATS_1    (MSM_VFE_BASE + 0x000000EC)
#define VFE_BUS_MISR_CFG                   (MSM_VFE_BASE + 0x000000F4)
#define VFE_BUS_MISR_MAST_CFG_0            (MSM_VFE_BASE + 0x000000F8)
#define VFE_BUS_MISR_MAST_CFG_1            (MSM_VFE_BASE + 0x000000FC)
#define VFE_BUS_MISR_RD_VAL                (MSM_VFE_BASE + 0x00000100)
#define VFE_AXI_CMD                        (MSM_VFE_BASE + 0x00000104)
#define VFE_AXI_CFG                        (MSM_VFE_BASE + 0x00000108)
#define VFE_AXI_STATUS                     (MSM_VFE_BASE + 0x0000010C)
#define CAMIF_COMMAND                      (MSM_VFE_BASE + 0x00000110)
#define CAMIF_CONFIG                       (MSM_VFE_BASE + 0x00000114)
#define CAMIF_EFS_CONFIG                   (MSM_VFE_BASE + 0x00000118)
#define CAMIF_FRAME_CONFIG                 (MSM_VFE_BASE + 0x0000011C)
#define CAMIF_WINDOW_WIDTH_CONFIG          (MSM_VFE_BASE + 0x00000120)
#define CAMIF_WINDOW_HEIGHT_CONFIG         (MSM_VFE_BASE + 0x00000124)
#define CAMIF_SUBSAMPLE1_CONFIG            (MSM_VFE_BASE + 0x00000128)
#define CAMIF_SUBSAMPLE2_CONFIG            (MSM_VFE_BASE + 0x0000012C)
#define CAMIF_EPOCH_IRQ                    (MSM_VFE_BASE + 0x00000130)
#define CAMIF_STATUS                       (MSM_VFE_BASE + 0x00000134)
#define CAMIF_MISR                         (MSM_VFE_BASE + 0x00000138)
#define VFE_SYNC_TIMER_CMD                 (MSM_VFE_BASE + 0x0000013C)
#define VFE_SYNC_TIMER0_LINE_START         (MSM_VFE_BASE + 0x00000140)
#define VFE_SYNC_TIMER0_PIXEL_START        (MSM_VFE_BASE + 0x00000144)
#define VFE_SYNC_TIMER0_PIXEL_DURATION     (MSM_VFE_BASE + 0x00000148)
#define VFE_SYNC_TIMER1_LINE_START         (MSM_VFE_BASE + 0x0000014C)
#define VFE_SYNC_TIMER1_PIXEL_START        (MSM_VFE_BASE + 0x00000150)
#define VFE_SYNC_TIMER1_PIXEL_DURATION     (MSM_VFE_BASE + 0x00000154)
#define VFE_SYNC_TIMER2_LINE_START         (MSM_VFE_BASE + 0x00000158)
#define VFE_SYNC_TIMER2_PIXEL_START        (MSM_VFE_BASE + 0x0000015C)
#define VFE_SYNC_TIMER2_PIXEL_DURATION     (MSM_VFE_BASE + 0x00000160)
#define VFE_SYNC_TIMER_POLARITY            (MSM_VFE_BASE + 0x00000164)
#define VFE_ASYNC_TIMER_CMD                (MSM_VFE_BASE + 0x00000168)
#define VFE_ASYNC_TIMER0_CFG_0             (MSM_VFE_BASE + 0x0000016C)
#define VFE_ASYNC_TIMER0_CFG_1             (MSM_VFE_BASE + 0x00000170)
#define VFE_ASYNC_TIMER1_CFG_0             (MSM_VFE_BASE + 0x00000174)
#define VFE_ASYNC_TIMER1_CFG_1             (MSM_VFE_BASE + 0x00000178)
#define VFE_ASYNC_TIMER2_CFG_0             (MSM_VFE_BASE + 0x0000017C)
#define VFE_ASYNC_TIMER2_CFG_1             (MSM_VFE_BASE + 0x00000180)
#define VFE_ASYNC_TIMER3_CFG_0             (MSM_VFE_BASE + 0x00000184)
#define VFE_ASYNC_TIMER3_CFG_1             (MSM_VFE_BASE + 0x00000188)
#define VFE_TIMER_SEL                      (MSM_VFE_BASE + 0x0000018C)
#define VFE_REG_UPDATE_CMD                 (MSM_VFE_BASE + 0x00000190)
#define VFE_BLACK_EVEN_EVEN_VALUE          (MSM_VFE_BASE + 0x00000194)
#define VFE_BLACK_EVEN_ODD_VALUE           (MSM_VFE_BASE + 0x00000198)
#define VFE_BLACK_ODD_EVEN_VALUE           (MSM_VFE_BASE + 0x0000019C)
#define VFE_BLACK_ODD_ODD_VALUE            (MSM_VFE_BASE + 0x000001A0)
#define VFE_ROLLOFF_CFG_0                  (MSM_VFE_BASE + 0x000001A4)
#define VFE_ROLLOFF_CFG_1                  (MSM_VFE_BASE + 0x000001A8)
#define VFE_ROLLOFF_CFG_2                  (MSM_VFE_BASE + 0x000001AC)
#define VFE_DEMUX_CFG                      (MSM_VFE_BASE + 0x000001B0)
#define VFE_DEMUX_GAIN_0                   (MSM_VFE_BASE + 0x000001B4)
#define VFE_DEMUX_GAIN_1                   (MSM_VFE_BASE + 0x000001B8)
#define VFE_DEMUX_EVEN_CFG                 (MSM_VFE_BASE + 0x000001BC)
#define VFE_DEMUX_ODD_CFG                  (MSM_VFE_BASE + 0x000001C0)
#define VFE_DEMOSAIC_CFG                   (MSM_VFE_BASE + 0x000001C4)
#define VFE_DEMOSAIC_ABF_CFG_0             (MSM_VFE_BASE + 0x000001C8)
#define VFE_DEMOSAIC_ABF_CFG_1             (MSM_VFE_BASE + 0x000001CC)
#define VFE_DEMOSAIC_BPC_CFG_0             (MSM_VFE_BASE + 0x000001D0)
#define VFE_DEMOSAIC_BPC_CFG_1             (MSM_VFE_BASE + 0x000001D4)
#define VFE_DEMOSAIC_STATUS                (MSM_VFE_BASE + 0x000001D8)
#define VFE_CHROMA_UPSAMPLE_CFG            (MSM_VFE_BASE + 0x000001DC)
#define VFE_CROP_WIDTH_CFG                 (MSM_VFE_BASE + 0x000001E0)
#define VFE_CROP_HEIGHT_CFG                (MSM_VFE_BASE + 0x000001E4)
#define VFE_COLOR_CORRECT_COEFF_0          (MSM_VFE_BASE + 0x000001E8)
#define VFE_COLOR_CORRECT_COEFF_1          (MSM_VFE_BASE + 0x000001EC)
#define VFE_COLOR_CORRECT_COEFF_2          (MSM_VFE_BASE + 0x000001F0)
#define VFE_COLOR_CORRECT_COEFF_3          (MSM_VFE_BASE + 0x000001F4)
#define VFE_COLOR_CORRECT_COEFF_4          (MSM_VFE_BASE + 0x000001F8)
#define VFE_COLOR_CORRECT_COEFF_5          (MSM_VFE_BASE + 0x000001FC)
#define VFE_COLOR_CORRECT_COEFF_6          (MSM_VFE_BASE + 0x00000200)
#define VFE_COLOR_CORRECT_COEFF_7          (MSM_VFE_BASE + 0x00000204)
#define VFE_COLOR_CORRECT_COEFF_8          (MSM_VFE_BASE + 0x00000208)
#define VFE_COLOR_CORRECT_OFFSET_0         (MSM_VFE_BASE + 0x0000020C)
#define VFE_COLOR_CORRECT_OFFSET_1         (MSM_VFE_BASE + 0x00000210)
#define VFE_COLOR_CORRECT_OFFSET_2         (MSM_VFE_BASE + 0x00000214)
#define VFE_COLOR_CORRECT_COEFF_Q          (MSM_VFE_BASE + 0x00000218)
#define VFE_LA_CFG                         (MSM_VFE_BASE + 0x0000021C)
#define VFE_LUT_BANK_SEL                   (MSM_VFE_BASE + 0x00000220)
#define VFE_CHROMA_ENHAN_A                 (MSM_VFE_BASE + 0x00000224)
#define VFE_CHROMA_ENHAN_B                 (MSM_VFE_BASE + 0x00000228)
#define VFE_CHROMA_ENHAN_C                 (MSM_VFE_BASE + 0x0000022C)
#define VFE_CHROMA_ENHAN_D                 (MSM_VFE_BASE + 0x00000230)
#define VFE_CHROMA_ENHAN_K                 (MSM_VFE_BASE + 0x00000234)
#define VFE_COLOR_CONVERT_COEFF_0          (MSM_VFE_BASE + 0x00000238)
#define VFE_COLOR_CONVERT_COEFF_1          (MSM_VFE_BASE + 0x0000023C)
#define VFE_COLOR_CONVERT_COEFF_2          (MSM_VFE_BASE + 0x00000240)
#define VFE_COLOR_CONVERT_OFFSET           (MSM_VFE_BASE + 0x00000244)
#define VFE_ASF_CFG                        (MSM_VFE_BASE + 0x00000248)
#define VFE_ASF_SHARP_CFG_0                (MSM_VFE_BASE + 0x0000024C)
#define VFE_ASF_SHARP_CFG_1                (MSM_VFE_BASE + 0x00000250)
#define VFE_ASF_SHARP_COEFF_0              (MSM_VFE_BASE + 0x00000254)
#define VFE_ASF_SHARP_COEFF_1              (MSM_VFE_BASE + 0x00000258)
#define VFE_ASF_SHARP_COEFF_2              (MSM_VFE_BASE + 0x0000025C)
#define VFE_ASF_SHARP_COEFF_3              (MSM_VFE_BASE + 0x00000260)
#define VFE_ASF_MAX_EDGE                   (MSM_VFE_BASE + 0x00000264)
#define VFE_ASF_CROP_WIDTH_CFG             (MSM_VFE_BASE + 0x00000268)
#define VFE_ASF_CROP_HEIGHT_CFG            (MSM_VFE_BASE + 0x0000026C)
#define VFE_SCALE_CFG                      (MSM_VFE_BASE + 0x00000270)
#define VFE_SCALE_H_IMAGE_SIZE_CFG         (MSM_VFE_BASE + 0x00000274)
#define VFE_SCALE_H_PHASE_CFG              (MSM_VFE_BASE + 0x00000278)
#define VFE_SCALE_H_STRIPE_CFG             (MSM_VFE_BASE + 0x0000027C)
#define VFE_SCALE_V_IMAGE_SIZE_CFG         (MSM_VFE_BASE + 0x00000280)
#define VFE_SCALE_V_PHASE_CFG              (MSM_VFE_BASE + 0x00000284)
#define VFE_SCALE_V_STRIPE_CFG             (MSM_VFE_BASE + 0x00000288)
#define VFE_SCALE_Y_CFG                    (MSM_VFE_BASE + 0x0000028C)
#define VFE_SCALE_Y_H_IMAGE_SIZE_CFG       (MSM_VFE_BASE + 0x00000290)
#define VFE_SCALE_Y_H_PHASE_CFG            (MSM_VFE_BASE + 0x00000294)
#define VFE_SCALE_Y_V_IMAGE_SIZE_CFG       (MSM_VFE_BASE + 0x00000298)
#define VFE_SCALE_Y_V_PHASE_CFG            (MSM_VFE_BASE + 0x0000029C)
#define VFE_SCALE_CBCR_CFG                 (MSM_VFE_BASE + 0x000002A0)
#define VFE_SCALE_CBCR_H_IMAGE_SIZE_CFG    (MSM_VFE_BASE + 0x000002A4)
#define VFE_SCALE_CBCR_H_PHASE_CFG         (MSM_VFE_BASE + 0x000002A8)
#define VFE_SCALE_CBCR_V_IMAGE_SIZE_CFG    (MSM_VFE_BASE + 0x000002AC)
#define VFE_SCALE_CBCR_V_PHASE_CFG         (MSM_VFE_BASE + 0x000002B0)
#define VFE_WB_CFG                         (MSM_VFE_BASE + 0x000002B4)
#define VFE_CHROMA_SUPPRESS_CFG_0          (MSM_VFE_BASE + 0x000002B8)
#define VFE_CHROMA_SUPPRESS_CFG_1          (MSM_VFE_BASE + 0x000002BC)
#define VFE_CHROMA_SUBSAMPLE_CFG           (MSM_VFE_BASE + 0x000002C0)
#define VFE_CHROMA_SUB_CROP_WIDTH_CFG      (MSM_VFE_BASE + 0x000002C4)
#define VFE_CHROMA_SUB_CROP_HEIGHT_CFG     (MSM_VFE_BASE + 0x000002C8)
#define VFE_FRAMEDROP_ENC_Y_CFG            (MSM_VFE_BASE + 0x000002CC)
#define VFE_FRAMEDROP_ENC_CBCR_CFG         (MSM_VFE_BASE + 0x000002D0)
#define VFE_FRAMEDROP_ENC_Y_PATTERN        (MSM_VFE_BASE + 0x000002D4)
#define VFE_FRAMEDROP_ENC_CBCR_PATTERN     (MSM_VFE_BASE + 0x000002D8)
#define VFE_FRAMEDROP_VIEW_Y_CFG           (MSM_VFE_BASE + 0x000002DC)
#define VFE_FRAMEDROP_VIEW_CBCR_CFG        (MSM_VFE_BASE + 0x000002E0)
#define VFE_FRAMEDROP_VIEW_Y_PATTERN       (MSM_VFE_BASE + 0x000002E4)
#define VFE_FRAMEDROP_VIEW_CBCR_PATTERN    (MSM_VFE_BASE + 0x000002E8)
#define VFE_CLAMP_MAX_CFG                  (MSM_VFE_BASE + 0x000002EC)
#define VFE_CLAMP_MIN_CFG                  (MSM_VFE_BASE + 0x000002F0)
#define VFE_STATS_CMD                      (MSM_VFE_BASE + 0x000002F4)
#define VFE_STATS_AF_CFG                   (MSM_VFE_BASE + 0x000002F8)
#define VFE_STATS_AF_DIM                   (MSM_VFE_BASE + 0x000002FC)
#define VFE_STATS_AF_GRID_0                (MSM_VFE_BASE + 0x00000300)
#define VFE_STATS_AF_GRID_1                (MSM_VFE_BASE + 0x00000304)
#define VFE_STATS_AF_GRID_2                (MSM_VFE_BASE + 0x00000308)
#define VFE_STATS_AF_GRID_3                (MSM_VFE_BASE + 0x0000030C)
#define VFE_STATS_AF_HEADER                (MSM_VFE_BASE + 0x00000310)
#define VFE_STATS_AF_COEF0                 (MSM_VFE_BASE + 0x00000314)
#define VFE_STATS_AF_COEF1                 (MSM_VFE_BASE + 0x00000318)
#define VFE_STATS_AWBAE_CFG                (MSM_VFE_BASE + 0x0000031C)
#define VFE_STATS_AXW_HEADER               (MSM_VFE_BASE + 0x00000320)
#define VFE_STATS_AWB_MCFG                 (MSM_VFE_BASE + 0x00000324)
#define VFE_STATS_AWB_CCFG1                (MSM_VFE_BASE + 0x00000328)
#define VFE_STATS_AWB_CCFG2                (MSM_VFE_BASE + 0x0000032C)
#define VFE_STATS_HIST_HEADER              (MSM_VFE_BASE + 0x00000330)
#define VFE_STATS_HIST_INNER_OFFSET        (MSM_VFE_BASE + 0x00000334)
#define VFE_STATS_HIST_INNER_DIM           (MSM_VFE_BASE + 0x00000338)
#define VFE_STATS_FRAME_SIZE               (MSM_VFE_BASE + 0x0000033C)
#define VFE_DMI_CFG                        (MSM_VFE_BASE + 0x00000340)
#define VFE_DMI_ADDR                       (MSM_VFE_BASE + 0x00000344)
#define VFE_DMI_DATA_HI                    (MSM_VFE_BASE + 0x00000348)
#define VFE_DMI_DATA_LO                    (MSM_VFE_BASE + 0x0000034C)
#define VFE_DMI_RAM_AUTO_LOAD_CMD          (MSM_VFE_BASE + 0x00000350)
#define VFE_DMI_RAM_AUTO_LOAD_STATUS       (MSM_VFE_BASE + 0x00000354)
#define VFE_DMI_RAM_AUTO_LOAD_CFG          (MSM_VFE_BASE + 0x00000358)
#define VFE_DMI_RAM_AUTO_LOAD_SEED         (MSM_VFE_BASE + 0x0000035C)
#define VFE_TESTBUS_SEL                    (MSM_VFE_BASE + 0x00000360)
#define VFE_TESTGEN_CFG                    (MSM_VFE_BASE + 0x00000364)
#define VFE_SW_TESTGEN_CMD                 (MSM_VFE_BASE + 0x00000368)
#define VFE_HW_TESTGEN_CMD                 (MSM_VFE_BASE + 0x0000036C)
#define VFE_HW_TESTGEN_CFG                 (MSM_VFE_BASE + 0x00000370)
#define VFE_HW_TESTGEN_IMAGE_CFG           (MSM_VFE_BASE + 0x00000374)
#define VFE_HW_TESTGEN_SOF_OFFSET_CFG      (MSM_VFE_BASE + 0x00000378)
#define VFE_HW_TESTGEN_EOF_NOFFSET_CFG     (MSM_VFE_BASE + 0x0000037C)
#define VFE_HW_TESTGEN_SOL_OFFSET_CFG      (MSM_VFE_BASE + 0x00000380)
#define VFE_HW_TESTGEN_EOL_NOFFSET_CFG     (MSM_VFE_BASE + 0x00000384)
#define VFE_HW_TESTGEN_HBI_CFG             (MSM_VFE_BASE + 0x00000388)
#define VFE_HW_TESTGEN_VBL_CFG             (MSM_VFE_BASE + 0x0000038C)
#define VFE_HW_TESTGEN_SOF_DUMMY_LINE_CFG2 (MSM_VFE_BASE + 0x00000390)
#define VFE_HW_TESTGEN_EOF_DUMMY_LINE_CFG2 (MSM_VFE_BASE + 0x00000394)
#define VFE_HW_TESTGEN_COLOR_BARS_CFG      (MSM_VFE_BASE + 0x00000398)
#define VFE_HW_TESTGEN_RANDOM_CFG          (MSM_VFE_BASE + 0x0000039C)
#define VFE_SPARE                          (MSM_VFE_BASE + 0x000003A0)

#define ping 0x0
#define pong 0x1

struct vfe_bus_cfg_data {
	boolean                  stripeRdPathEn;
	boolean                  encYWrPathEn;
	boolean                  encCbcrWrPathEn;
	boolean                  viewYWrPathEn;
	boolean                  viewCbcrWrPathEn;
	enum VFE_RAW_PIXEL_DATA_SIZE rawPixelDataSize;
	enum VFE_RAW_WR_PATH_SEL     rawWritePathSelect;
};

struct vfe_camif_cfg_data {
	boolean camif2OutputEnable;
	boolean camif2BusEnable;
	struct vfe_cmds_camif_cfg camifCfgFromCmd;
};

struct vfe_irq_composite_mask_config {
	uint8_t encIrqComMask;
	uint8_t viewIrqComMask;
	uint8_t ceDoneSel;
};

/* define a structure for each output path.*/
struct vfe_output_path {
	uint32_t addressBuffer[8];
	uint16_t fragIndex;
	boolean  hwCurrentFlag;
	uint8_t  *hwRegPingAddress;
	uint8_t  *hwRegPongAddress;
};

struct vfe_output_path_combo {
	boolean           whichOutputPath;
	boolean           pathEnabled;
	boolean           multiFrag;
	uint8_t           fragCount;
	boolean           ackPending;
	uint8_t           currentFrame;
	uint32_t          nextFrameAddrBuf[8];
	struct vfe_output_path   yPath;
	struct vfe_output_path   cbcrPath;
	uint8_t           snapshotPendingCount;
	boolean           pmEnabled;
	uint8_t           cbcrStatusBit;
};

struct vfe_stats_control {
	boolean  ackPending;
	uint32_t addressBuffer[2];
	uint32_t nextFrameAddrBuf;
	boolean  pingPongStatus;
	uint8_t  *hwRegPingAddress;
	uint8_t  *hwRegPongAddress;
	uint32_t droppedStatsFrameCount;
	uint32_t bufToRender;
};

struct vfe_gamma_lut_sel {
	boolean  ch0BankSelect;
	boolean  ch1BankSelect;
	boolean  ch2BankSelect;
};

struct vfe_interrupt_mask {
	boolean  camifErrorIrq;
	boolean  camifSofIrq;
	boolean  camifEolIrq;
	boolean  camifEofIrq;
	boolean  camifEpoch1Irq;
	boolean  camifEpoch2Irq;
	boolean  camifOverflowIrq;
	boolean  ceIrq;
	boolean  regUpdateIrq;
	boolean  resetAckIrq;
	boolean  encYPingpongIrq;
	boolean  encCbcrPingpongIrq;
	boolean  viewYPingpongIrq;
	boolean  viewCbcrPingpongIrq;
	boolean  rdPingpongIrq;
	boolean  afPingpongIrq;
	boolean  awbPingpongIrq;
	boolean  histPingpongIrq;
	boolean  encIrq;
	boolean  viewIrq;
	boolean  busOverflowIrq;
	boolean  afOverflowIrq;
	boolean  awbOverflowIrq;
	boolean  syncTimer0Irq;
	boolean  syncTimer1Irq;
	boolean  syncTimer2Irq;
	boolean  asyncTimer0Irq;
	boolean  asyncTimer1Irq;
	boolean  asyncTimer2Irq;
	boolean  asyncTimer3Irq;
	boolean  axiErrorIrq;
	boolean  violationIrq;
};

enum vfe_interrupt_name {
	CAMIF_ERROR_IRQ,
	CAMIF_SOF_IRQ,
	CAMIF_EOL_IRQ,
	CAMIF_EOF_IRQ,
	CAMIF_EPOCH1_IRQ,
	CAMIF_EPOCH2_IRQ,
	CAMIF_OVERFLOW_IRQ,
	CE_IRQ,
	REG_UPDATE_IRQ,
	RESET_ACK_IRQ,
	ENC_Y_PINGPONG_IRQ,
	ENC_CBCR_PINGPONG_IRQ,
	VIEW_Y_PINGPONG_IRQ,
	VIEW_CBCR_PINGPONG_IRQ,
	RD_PINGPONG_IRQ,
	AF_PINGPONG_IRQ,
	AWB_PINGPONG_IRQ,
	HIST_PINGPONG_IRQ,
	ENC_IRQ,
	VIEW_IRQ,
	BUS_OVERFLOW_IRQ,
	AF_OVERFLOW_IRQ,
	AWB_OVERFLOW_IRQ,
	SYNC_TIMER0_IRQ,
	SYNC_TIMER1_IRQ,
	SYNC_TIMER2_IRQ,
	ASYNC_TIMER0_IRQ,
	ASYNC_TIMER1_IRQ,
	ASYNC_TIMER2_IRQ,
	ASYNC_TIMER3_IRQ,
	AXI_ERROR_IRQ,
	VIOLATION_IRQ
};

enum VFE_DMI_RAM_SEL {
	NO_MEM_SELECTED          = 0,
	ROLLOFF_RAM              = 0x1,
	RGBLUT_RAM_CH0_BANK0     = 0x2,
	RGBLUT_RAM_CH0_BANK1     = 0x3,
	RGBLUT_RAM_CH1_BANK0     = 0x4,
	RGBLUT_RAM_CH1_BANK1     = 0x5,
	RGBLUT_RAM_CH2_BANK0     = 0x6,
	RGBLUT_RAM_CH2_BANK1     = 0x7,
	STATS_HIST_CB_EVEN_RAM   = 0x8,
	STATS_HIST_CB_ODD_RAM    = 0x9,
	STATS_HIST_CR_EVEN_RAM   = 0xa,
	STATS_HIST_CR_ODD_RAM    = 0xb,
	RGBLUT_CHX_BANK0         = 0xc,
	RGBLUT_CHX_BANK1         = 0xd,
	LUMA_ADAPT_LUT_RAM_BANK0 = 0xe,
	LUMA_ADAPT_LUT_RAM_BANK1 = 0xf
};

struct vfe_module_enable {
	boolean  blackLevelCorrectionEnable;
	boolean  lensRollOffEnable;
	boolean  demuxEnable;
	boolean  chromaUpsampleEnable;
	boolean  demosaicEnable;
	boolean  statsEnable;
	boolean  cropEnable;
	boolean  mainScalerEnable;
	boolean  whiteBalanceEnable;
	boolean  colorCorrectionEnable;
	boolean  yHistEnable;
	boolean  skinToneEnable;
	boolean  lumaAdaptationEnable;
	boolean  rgbLUTEnable;
	boolean  chromaEnhanEnable;
	boolean  asfEnable;
	boolean  chromaSuppressionEnable;
	boolean  chromaSubsampleEnable;
	boolean  scaler2YEnable;
	boolean  scaler2CbcrEnable;
};

struct vfe_bus_cmd_data {
	boolean  stripeReload;
	boolean  busPingpongReload;
	boolean  statsPingpongReload;
};

struct vfe_stats_cmd_data {
	boolean  autoFocusEnable;
	boolean  axwEnable;
	boolean  histEnable;
	boolean  clearHistEnable;
	boolean  histAutoClearEnable;
	boolean  colorConversionEnable;
};

struct vfe_hw_ver_t {
	uint32_t minorVersion    : 8;
	uint32_t majorVersion    : 8;
	uint32_t coreVersion     : 4;
	uint32_t /* reserved */  : 12;
} __attribute__((packed, aligned(4)));

struct vfe_cfg_t {
	uint32_t pixelPattern     : 3;
	uint32_t /* reserved */   :13;
	uint32_t inputSource      : 2;
	uint32_t /* reserved */   :14;
} __attribute__((packed, aligned(4)));

struct vfe_buscmd_t {
	uint32_t  stripeReload         : 1;
	uint32_t  /* reserved */       : 3;
	uint32_t  busPingpongReload    : 1;
	uint32_t  statsPingpongReload  : 1;
	uint32_t  /* reserved */       :26;
} __attribute__((packed, aligned(4)));

struct VFE_Irq_Composite_MaskType {
	uint32_t  encIrqComMaskBits    : 2;
	uint32_t  viewIrqComMaskBits   : 2;
	uint32_t  ceDoneSelBits        : 5;
	uint32_t  /* reserved */       :23;
} __attribute__((packed, aligned(4)));

struct vfe_mod_enable_t {
	uint32_t blackLevelCorrectionEnable   : 1;
	uint32_t lensRollOffEnable            : 1;
	uint32_t demuxEnable                  : 1;
	uint32_t chromaUpsampleEnable         : 1;
	uint32_t demosaicEnable               : 1;
	uint32_t statsEnable                  : 1;
	uint32_t cropEnable                   : 1;
	uint32_t mainScalerEnable             : 1;
	uint32_t whiteBalanceEnable           : 1;
	uint32_t colorCorrectionEnable        : 1;
	uint32_t yHistEnable                  : 1;
	uint32_t skinToneEnable               : 1;
	uint32_t lumaAdaptationEnable         : 1;
	uint32_t rgbLUTEnable                 : 1;
	uint32_t chromaEnhanEnable            : 1;
	uint32_t asfEnable                    : 1;
	uint32_t chromaSuppressionEnable      : 1;
	uint32_t chromaSubsampleEnable        : 1;
	uint32_t scaler2YEnable               : 1;
	uint32_t scaler2CbcrEnable            : 1;
	uint32_t /* reserved */               :14;
} __attribute__((packed, aligned(4)));

struct vfe_irqenable_t {
	uint32_t camifErrorIrq       :1;
	uint32_t camifSofIrq         :1;
	uint32_t camifEolIrq         :1;
	uint32_t camifEofIrq         :1;
	uint32_t camifEpoch1Irq      :1;
	uint32_t camifEpoch2Irq      :1;
	uint32_t camifOverflowIrq    :1;
	uint32_t ceIrq               :1;
	uint32_t regUpdateIrq        :1;
	uint32_t resetAckIrq         :1;
	uint32_t encYPingpongIrq     :1;
	uint32_t encCbcrPingpongIrq  :1;
	uint32_t viewYPingpongIrq    :1;
	uint32_t viewCbcrPingpongIrq :1;
	uint32_t rdPingpongIrq       :1;
	uint32_t afPingpongIrq       :1;
	uint32_t awbPingpongIrq      :1;
	uint32_t histPingpongIrq     :1;
	uint32_t encIrq              :1;
	uint32_t viewIrq             :1;
	uint32_t busOverflowIrq      :1;
	uint32_t afOverflowIrq       :1;
	uint32_t awbOverflowIrq      :1;
	uint32_t syncTimer0Irq       :1;
	uint32_t syncTimer1Irq       :1;
	uint32_t syncTimer2Irq       :1;
	uint32_t asyncTimer0Irq      :1;
	uint32_t asyncTimer1Irq      :1;
	uint32_t asyncTimer2Irq      :1;
	uint32_t asyncTimer3Irq      :1;
	uint32_t axiErrorIrq         :1;
	uint32_t violationIrq        :1;
} __attribute__((packed, aligned(4)));

struct vfe_upsample_cfg_t {
	uint32_t chromaCositingForYCbCrInputs :  1;
	uint32_t /* reserved */               : 31;
} __attribute__((packed, aligned(4)));

struct VFE_CAMIFConfigType {
	/* CAMIF Config */
	uint32_t  /* reserved */       : 1;
	uint32_t  VSyncEdge            : 1;
	uint32_t  HSyncEdge            : 1;
	uint32_t  syncMode             : 2;
	uint32_t  vfeSubsampleEnable   : 1;
	uint32_t  /* reserved */       : 1;
	uint32_t  busSubsampleEnable   : 1;
	uint32_t  camif2vfeEnable      : 1;
	uint32_t  /* reserved */       : 1;
	uint32_t  camif2busEnable      : 1;
	uint32_t  irqSubsampleEnable   : 1;
	uint32_t  binningEnable        : 1;
	uint32_t  /* reserved */       :18;
	uint32_t  misrEnable           : 1;
} __attribute__((packed, aligned(4)));

struct vfe_camifcfg_t {
	/* EFS_Config */
	uint32_t efsEndOfLine              :  8;
	uint32_t efsStartOfLine            :  8;
	uint32_t efsEndOfFrame             :  8;
	uint32_t efsStartOfFrame           :  8;
	/* Frame Config */
	uint32_t frameConfigPixelsPerLine  : 14;
	uint32_t /* reserved */            :  2;
	uint32_t frameConfigLinesPerFrame  : 14;
	uint32_t /* reserved */            :  2;
	/* Window Width Config */
	uint32_t windowWidthCfgLastPixel   : 14;
	uint32_t /* reserved */            :  2;
	uint32_t windowWidthCfgFirstPixel  : 14;
	uint32_t /* reserved */            :  2;
	/* Window Height Config */
	uint32_t windowHeightCfglastLine   : 14;
	uint32_t /* reserved */            :  2;
	uint32_t windowHeightCfgfirstLine  : 14;
	uint32_t /* reserved */            :  2;
	/* Subsample 1 Config */
	uint32_t subsample1CfgPixelSkip    : 16;
	uint32_t subsample1CfgLineSkip     : 16;
	/* Subsample 2 Config */
	uint32_t subsample2CfgFrameSkip    :  4;
	uint32_t subsample2CfgFrameSkipMode:  1;
	uint32_t subsample2CfgPixelSkipWrap:  1;
	uint32_t /* reserved */            : 26;
	/* Epoch Interrupt */
	uint32_t epoch1Line                : 14;
	uint32_t /* reserved */            :  2;
	uint32_t epoch2Line                : 14;
	uint32_t /* reserved */            :  2;
} __attribute__((packed, aligned(4)));

struct vfe_camifframe_update_t {
	uint32_t pixelsPerLine   :14;
	uint32_t /* reserved */  : 2;
	uint32_t linesPerFrame   :14;
	uint32_t /* reserved */  : 2;
} __attribute__((packed, aligned(4)));

struct vfe_axi_bus_cfg_t {
	uint32_t  stripeRdPathEn      :  1;
	uint32_t  /* reserved */      :  3;
	uint32_t  encYWrPathEn        :  1;
	uint32_t  encCbcrWrPathEn     :  1;
	uint32_t  viewYWrPathEn       :  1;
	uint32_t  viewCbcrWrPathEn    :  1;
	uint32_t  rawPixelDataSize    :  2;
	uint32_t  rawWritePathSelect  :  2;
	uint32_t  /* reserved */      : 20;
} __attribute__((packed, aligned(4)));

struct vfe_axi_out_cfg_t {
	uint32_t  out2YPingAddr                 : 32;
	uint32_t  out2YPongAddr                 : 32;
	uint32_t  out2YImageHeight              : 12;
	uint32_t  /* reserved */                : 4;
	uint32_t  out2YImageWidthin64bit        : 10;
	uint32_t  /* reserved */                :  6;
	uint32_t  out2YBurstLength              :  2;
	uint32_t  /* reserved */                :  2;
	uint32_t  out2YNumRows                  : 12;
	uint32_t  out2YRowIncrementIn64bit      : 12;
	uint32_t  /* reserved */                :  4;
	uint32_t  out2CbcrPingAddr              : 32;
	uint32_t  out2CbcrPongAddr              : 32;
	uint32_t  out2CbcrImageHeight           : 12;
	uint32_t  /* reserved */                :  4;
	uint32_t  out2CbcrImageWidthIn64bit     : 10;
	uint32_t  /* reserved */                :  6;
	uint32_t  out2CbcrBurstLength           :  2;
	uint32_t  /* reserved */                :  2;
	uint32_t  out2CbcrNumRows               : 12;
	uint32_t  out2CbcrRowIncrementIn64bit   : 12;
	uint32_t  /* reserved */                :  4;
	uint32_t  out1YPingAddr                 : 32;
	uint32_t  out1YPongAddr                 : 32;
	uint32_t  out1YImageHeight              : 12;
	uint32_t  /* reserved */                :  4;
	uint32_t  out1YImageWidthin64bit        : 10;
	uint32_t  /* reserved */                :  6;
	uint32_t  out1YBurstLength              :  2;
	uint32_t  /* reserved */                :  2;
	uint32_t  out1YNumRows                  : 12;
	uint32_t  out1YRowIncrementIn64bit      : 12;
	uint32_t  /* reserved */                :  4;
	uint32_t  out1CbcrPingAddr              : 32;
	uint32_t  out1CbcrPongAddr              : 32;
	uint32_t  out1CbcrImageHeight           : 12;
	uint32_t  /* reserved */                :  4;
	uint32_t  out1CbcrImageWidthIn64bit     : 10;
	uint32_t  /* reserved */                :  6;
	uint32_t  out1CbcrBurstLength           :  2;
	uint32_t  /* reserved */                :  2;
	uint32_t  out1CbcrNumRows               : 12;
	uint32_t  out1CbcrRowIncrementIn64bit   : 12;
	uint32_t  /* reserved */                :  4;
} __attribute__((packed, aligned(4)));

struct vfe_output_clamp_cfg_t {
	/* Output Clamp Maximums */
	uint32_t yChanMax       :  8;
	uint32_t cbChanMax      :  8;
	uint32_t crChanMax      :  8;
	uint32_t /* reserved */ :  8;
	/* Output Clamp Minimums */
	uint32_t yChanMin       :  8;
	uint32_t cbChanMin      :  8;
	uint32_t crChanMin      :  8;
	uint32_t /* reserved */ :  8;
} __attribute__((packed, aligned(4)));

struct vfe_fov_crop_cfg_t {
	uint32_t lastPixel       : 12;
	uint32_t /* reserved */  :  4;
	uint32_t firstPixel      : 12;
	uint32_t /* reserved */  :  4;

	/* FOV Corp, Part 2 */
	uint32_t lastLine        : 12;
	uint32_t /* reserved */  :  4;
	uint32_t firstLine       : 12;
	uint32_t /* reserved */  :  4;
} __attribute__((packed, aligned(4)));

struct VFE_FRAME_SKIP_UpdateCmdType {
	uint32_t  yPattern       : 32;
	uint32_t  cbcrPattern    : 32;
} __attribute__((packed, aligned(4)));

struct vfe_frame_skip_cfg_t {
	/* Frame Drop Enc (output2) */
	uint32_t output2YPeriod			 :  5;
	uint32_t /* reserved */			 : 27;
	uint32_t output2CbCrPeriod	 :  5;
	uint32_t /* reserved */			 : 27;
	uint32_t output2YPattern     : 32;
	uint32_t output2CbCrPattern  : 32;
	/* Frame Drop View (output1) */
	uint32_t output1YPeriod      :  5;
	uint32_t /* reserved */      : 27;
	uint32_t output1CbCrPeriod   :  5;
	uint32_t /* reserved */      : 27;
	uint32_t output1YPattern     : 32;
	uint32_t output1CbCrPattern  : 32;
} __attribute__((packed, aligned(4)));

struct vfe_main_scaler_cfg_t {
	/* Scaler Enable Config */
	uint32_t hEnable             :  1;
	uint32_t vEnable             :  1;
	uint32_t /* reserved */      : 30;
	/* Scale H Image Size Config */
	uint32_t inWidth             : 12;
	uint32_t /* reserved */      :  4;
	uint32_t outWidth            : 12;
	uint32_t /* reserved */      :  4;
	/* Scale H Phase Config */
	uint32_t horizPhaseMult      : 18;
	uint32_t /* reserved */      :  2;
	uint32_t horizInterResolution:  2;
	uint32_t /* reserved */      : 10;
	/* Scale H Stripe Config */
	uint32_t horizMNInit         : 12;
	uint32_t /* reserved */      :  4;
	uint32_t horizPhaseInit      : 15;
	uint32_t /* reserved */      :  1;
	/* Scale V Image Size Config */
	uint32_t inHeight            : 12;
	uint32_t /* reserved */      :  4;
	uint32_t outHeight           : 12;
	uint32_t /* reserved */      :  4;
	/* Scale V Phase Config */
	uint32_t vertPhaseMult       : 18;
	uint32_t /* reserved */      :  2;
	uint32_t vertInterResolution :  2;
	uint32_t /* reserved */      : 10;
	/* Scale V Stripe Config */
	uint32_t vertMNInit          : 12;
	uint32_t /* reserved */      :  4;
	uint32_t vertPhaseInit       : 15;
	uint32_t /* reserved */      :  1;
} __attribute__((packed, aligned(4)));

struct vfe_scaler2_cfg_t {
	/* Scaler   Enable Config */
	uint32_t  hEnable                 :  1;
	uint32_t  vEnable                 :  1;
	uint32_t  /* reserved */          : 30;
	/* Scaler   H Image Size Config */
	uint32_t  inWidth                 : 12;
	uint32_t  /* reserved */          :  4;
	uint32_t  outWidth                : 12;
	uint32_t  /* reserved */          :  4;
	/* Scaler   H Phase Config */
	uint32_t  horizPhaseMult          : 18;
	uint32_t  /* reserved */          :  2;
	uint32_t  horizInterResolution    :  2;
	uint32_t  /* reserved */          : 10;
	/* Scaler   V Image Size Config */
	uint32_t  inHeight                : 12;
	uint32_t  /* reserved */          :  4;
	uint32_t  outHeight               : 12;
	uint32_t  /* reserved */          :  4;
	/* Scaler   V Phase Config */
	uint32_t  vertPhaseMult           : 18;
	uint32_t  /* reserved */          :  2;
	uint32_t  vertInterResolution     :  2;
	uint32_t  /* reserved */          : 10;
} __attribute__((packed, aligned(4)));

struct vfe_rolloff_cfg_t {
	/* Rolloff 0 Config */
	uint32_t  gridWidth     :  9;
	uint32_t  gridHeight    :  9;
	uint32_t  yDelta        :  9;
	uint32_t  /* reserved */:  5;
	/* Rolloff 1 Config*/
	uint32_t  gridX         :  4;
	uint32_t  gridY         :  4;
	uint32_t  pixelX        :  9;
	uint32_t  /* reserved */:  3;
	uint32_t  pixelY        :  9;
	uint32_t  /* reserved */:  3;
	/* Rolloff 2 Config */
	uint32_t  yDeltaAccum   : 12;
	uint32_t  /* reserved */: 20;
} __attribute__((packed, aligned(4)));

struct vfe_asf_update_t {
	/* ASF Config Command */
	uint32_t smoothEnable     :  1;
	uint32_t sharpMode        :  2;
	uint32_t /* reserved */   :  1;
	uint32_t smoothCoeff1     :  4;
	uint32_t smoothCoeff0     :  8;
	uint32_t pipeFlushCount   : 12;
	uint32_t pipeFlushOvd     :  1;
	uint32_t flushHaltOvd     :  1;
	uint32_t cropEnable       :  1;
	uint32_t /* reserved */   :  1;
	/* Sharpening Config 0 */
	uint32_t sharpThresholdE1 :  7;
	uint32_t /* reserved */   :  1;
	uint32_t sharpDegreeK1    :  5;
	uint32_t /* reserved */   :  3;
	uint32_t sharpDegreeK2    :  5;
	uint32_t /* reserved */   :  3;
	uint32_t normalizeFactor  :  7;
	uint32_t /* reserved */   :  1;
	/* Sharpening Config 1 */
	uint32_t sharpThresholdE2 :  8;
	uint32_t sharpThresholdE3 :  8;
	uint32_t sharpThresholdE4 :  8;
	uint32_t sharpThresholdE5 :  8;
	/* Sharpening Coefficients 0 */
	uint32_t F1Coeff0         :  6;
	uint32_t F1Coeff1         :  6;
	uint32_t F1Coeff2         :  6;
	uint32_t F1Coeff3         :  6;
	uint32_t F1Coeff4         :  6;
	uint32_t /* reserved */   :  2;
	/* Sharpening Coefficients 1 */
	uint32_t F1Coeff5         :  6;
	uint32_t F1Coeff6         :  6;
	uint32_t F1Coeff7         :  6;
	uint32_t F1Coeff8         :  7;
	uint32_t /* reserved */   :  7;
	/* Sharpening Coefficients 2 */
	uint32_t F2Coeff0         :  6;
	uint32_t F2Coeff1         :  6;
	uint32_t F2Coeff2         :  6;
	uint32_t F2Coeff3         :  6;
	uint32_t F2Coeff4         :  6;
	uint32_t /* reserved */   :  2;
	/* Sharpening Coefficients 3 */
	uint32_t F2Coeff5         :  6;
	uint32_t F2Coeff6         :  6;
	uint32_t F2Coeff7         :  6;
	uint32_t F2Coeff8         :  7;
	uint32_t /* reserved */   :  7;
} __attribute__((packed, aligned(4)));

struct vfe_asfcrop_cfg_t {
	/* ASF Crop Width Config */
	uint32_t lastPixel        : 12;
	uint32_t /* reserved */   :  4;
	uint32_t firstPixel       : 12;
	uint32_t /* reserved */   :  4;
	/* ASP Crop Height Config */
	uint32_t lastLine         : 12;
	uint32_t /* reserved */   :  4;
	uint32_t firstLine        : 12;
	uint32_t /* reserved */   :  4;
} __attribute__((packed, aligned(4)));

struct vfe_chroma_suppress_cfg_t {
	/* Chroma Suppress 0 Config */
	uint32_t m1             :  8;
	uint32_t m3             :  8;
	uint32_t n1             :  3;
	uint32_t /* reserved */ :  1;
	uint32_t n3             :  3;
	uint32_t /* reserved */ :  9;
	/* Chroma Suppress 1 Config */
	uint32_t mm1            :  8;
	uint32_t nn1            :  3;
	uint32_t /* reserved */ : 21;
} __attribute__((packed, aligned(4)));

struct vfe_chromasubsample_cfg_t {
	/* Chroma Subsample Selection */
	uint32_t  hCositedPhase       : 1;
	uint32_t  vCositedPhase       : 1;
	uint32_t  hCosited            : 1;
	uint32_t  vCosited            : 1;
	uint32_t  hsubSampleEnable    : 1;
	uint32_t  vsubSampleEnable    : 1;
	uint32_t  cropEnable          : 1;
	uint32_t  /* reserved */      :25;
	uint32_t  cropWidthLastPixel  :12;
	uint32_t  /* reserved */      : 4;
	uint32_t  cropWidthFirstPixel :12;
	uint32_t  /* reserved */      : 4;
	uint32_t  cropHeightLastLine  :12;
	uint32_t  /* reserved */      : 4;
	uint32_t  cropHeightFirstLine :12;
	uint32_t  /* reserved */      : 4;
} __attribute__((packed, aligned(4)));

struct vfe_blacklevel_cfg_t {
	/* Black Even-Even Value Config */
	uint32_t    evenEvenAdjustment    : 9;
	uint32_t   /* reserved */         :23;
	/* Black Even-Odd Value Config */
	uint32_t    evenOddAdjustment     : 9;
	uint32_t   /* reserved */         :23;
	/* Black Odd-Even Value Config */
	uint32_t    oddEvenAdjustment     : 9;
	uint32_t   /* reserved */         :23;
	/* Black Odd-Odd Value Config */
	uint32_t    oddOddAdjustment      : 9;
	uint32_t   /* reserved */         :23;
} __attribute__((packed, aligned(4)));

struct vfe_demux_cfg_t {
	/* Demux Gain 0 Config */
	uint32_t  ch0EvenGain        :10;
	uint32_t  /* reserved */     : 6;
	uint32_t  ch0OddGain         :10;
	uint32_t  /* reserved */     : 6;
	/* Demux Gain 1 Config */
	uint32_t  ch1Gain            :10;
	uint32_t  /* reserved */     : 6;
	uint32_t  ch2Gain            :10;
	uint32_t  /* reserved */     : 6;
} __attribute__((packed, aligned(4)));

struct vfe_bps_info_t {
  uint32_t greenBadPixelCount   : 8;
  uint32_t /* reserved */       : 8;
  uint32_t RedBlueBadPixelCount : 8;
  uint32_t /* reserved */       : 8;
} __attribute__((packed, aligned(4)));

struct vfe_demosaic_cfg_t {
	/* Demosaic Config */
	uint32_t abfEnable          : 1;
	uint32_t badPixelCorrEnable : 1;
	uint32_t forceAbfOn         : 1;
	uint32_t /* reserved */     : 1;
	uint32_t abfShift           : 4;
	uint32_t fminThreshold      : 7;
	uint32_t /* reserved */     : 1;
	uint32_t fmaxThreshold      : 7;
	uint32_t /* reserved */     : 5;
	uint32_t slopeShift         : 3;
	uint32_t /* reserved */     : 1;
} __attribute__((packed, aligned(4)));

struct vfe_demosaic_bpc_cfg_t {
	/* Demosaic BPC Config 0 */
	uint32_t blueDiffThreshold   : 12;
	uint32_t redDiffThreshold    : 12;
	uint32_t /* reserved */      :  8;
	/* Demosaic BPC Config 1 */
	uint32_t greenDiffThreshold  : 12;
	uint32_t /* reserved */      : 20;
} __attribute__((packed, aligned(4)));

struct vfe_demosaic_abf_cfg_t {
	/* Demosaic ABF Config 0 */
	uint32_t lpThreshold    : 10;
	uint32_t /* reserved */ : 22;
	/* Demosaic ABF Config 1 */
	uint32_t ratio          :  4;
	uint32_t minValue       : 10;
	uint32_t /* reserved */ :  2;
	uint32_t maxValue       : 10;
	uint32_t /* reserved */ :  6;
} __attribute__((packed, aligned(4)));

struct vfe_color_correction_cfg_t {
	/* Color Corr. Coefficient 0 Config */
	uint32_t   c0                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 1 Config */
	uint32_t   c1                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 2 Config */
	uint32_t   c2                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 3 Config */
	uint32_t   c3                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 4 Config */
	uint32_t   c4                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 5 Config */
	uint32_t   c5                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 6 Config */
	uint32_t   c6                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 7 Config */
	uint32_t   c7                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Coefficient 8 Config */
	uint32_t   c8                  : 12;
	uint32_t   /* reserved */      : 20;
	/* Color Corr. Offset 0 Config */
	uint32_t   k0                  : 11;
	uint32_t   /* reserved */      : 21;
	/* Color Corr. Offset 1 Config */
	uint32_t   k1                  : 11;
	uint32_t   /* reserved */      : 21;
	/* Color Corr. Offset 2 Config */
	uint32_t   k2                  : 11;
	uint32_t   /* reserved */      : 21;
	/* Color Corr. Coefficient Q Config */
	uint32_t   coefQFactor         : 2;
	uint32_t   /* reserved */      : 30;
} __attribute__((packed, aligned(4)));

struct VFE_LumaAdaptation_ConfigCmdType {
	/* LA Config */
	uint32_t   lutBankSelect    :  1;
	uint32_t   /* reserved */   : 31;
} __attribute__((packed, aligned(4)));

struct vfe_wb_cfg_t {
	/* WB Config */
	uint32_t ch0Gain        : 9;
	uint32_t ch1Gain        : 9;
	uint32_t ch2Gain        : 9;
	uint32_t /* reserved */ : 5;
} __attribute__((packed, aligned(4)));

struct VFE_GammaLutSelect_ConfigCmdType {
	/* LUT Bank Select Config */
	uint32_t   ch0BankSelect     :  1;
	uint32_t   ch1BankSelect     :  1;
	uint32_t   ch2BankSelect     :  1;
	uint32_t   /* reserved */    : 29;
} __attribute__((packed, aligned(4)));

struct vfe_chroma_enhance_cfg_t {
	/* Chroma Enhance A Config */
	uint32_t ap               :11;
	uint32_t /* reserved */   : 5;
	uint32_t am               :11;
	uint32_t /* reserved */   : 5;
	/* Chroma Enhance B Config */
	uint32_t bp               :11;
	uint32_t /* reserved */   : 5;
	uint32_t bm               :11;
	uint32_t /* reserved */   : 5;
	/* Chroma Enhance C Config */
	uint32_t cp               :11;
	uint32_t /* reserved */   : 5;
	uint32_t cm               :11;
	uint32_t /* reserved */   : 5;
	/* Chroma Enhance D Config */
	uint32_t dp               :11;
	uint32_t /* reserved */   : 5;
	uint32_t dm               :11;
	uint32_t /* reserved */   : 5;
	/* Chroma Enhance K Config */
	uint32_t kcb              :11;
	uint32_t /* reserved */   : 5;
	uint32_t kcr              :11;
	uint32_t /* reserved */   : 5;
} __attribute__((packed, aligned(4)));

struct vfe_color_convert_cfg_t {
	/* Conversion Coefficient 0 */
	uint32_t v0             :12;
	uint32_t /* reserved */ :20;
	/* Conversion Coefficient 1 */
	uint32_t v1             :12;
	uint32_t /* reserved */ :20;
	/* Conversion Coefficient 2 */
	uint32_t v2             :12;
	uint32_t /* reserved */ :20;
	/* Conversion Offset */
	uint32_t ConvertOffset  : 8;
	uint32_t /* reserved */ :24;
} __attribute__((packed, aligned(4)));

struct VFE_SyncTimer_ConfigCmdType {
	/* Timer Line Start Config */
	uint32_t       timerLineStart      : 12;
	uint32_t       /* reserved */      : 20;
	/* Timer Pixel Start Config */
	uint32_t       timerPixelStart     : 18;
	uint32_t       /* reserved */      : 14;
	/* Timer Pixel Duration Config */
	uint32_t       timerPixelDuration  : 28;
	uint32_t       /* reserved */      : 4;
	/* Sync Timer Polarity Config */
	uint32_t       timer0Polarity      : 1;
	uint32_t       timer1Polarity      : 1;
	uint32_t       timer2Polarity      : 1;
	uint32_t       /* reserved */      : 29;
} __attribute__((packed, aligned(4)));

struct VFE_AsyncTimer_ConfigCmdType {
	/* Async Timer Config 0 */
	uint32_t     inactiveLength     : 20;
	uint32_t     numRepetition      : 10;
	uint32_t     /* reserved */     : 1;
	uint32_t     polarity           : 1;
	/* Async Timer Config 1 */
	uint32_t     activeLength       : 20;
	uint32_t     /* reserved */     : 12;
} __attribute__((packed, aligned(4)));

struct VFE_AWBAEStatistics_ConfigCmdType {
	/* AWB autoexposure Config */
	uint32_t    aeRegionConfig      : 1;
	uint32_t    aeSubregionConfig   : 1;
	uint32_t    /* reserved */      : 14;
	uint32_t    awbYMin             : 8;
	uint32_t    awbYMax             : 8;
	/* AXW Header */
	uint32_t    axwHeader           : 8;
	uint32_t    /* reserved */      : 24;
	/* AWB Mconfig */
	uint32_t    m4                  : 8;
	uint32_t    m3                  : 8;
	uint32_t    m2                  : 8;
	uint32_t    m1                  : 8;
	/* AWB Cconfig */
	uint32_t    c2                  : 12;
	uint32_t    /* reserved */      : 4;
	uint32_t    c1                  : 12;
	uint32_t    /* reserved */      : 4;
	/* AWB Cconfig 2 */
	uint32_t    c4                  : 12;
	uint32_t    /* reserved */      : 4;
	uint32_t    c3                  : 12;
	uint32_t    /* reserved */      : 4;
} __attribute__((packed, aligned(4)));

struct VFE_TestGen_ConfigCmdType {
	/* HW Test Gen Config */
	uint32_t   numFrame              : 10;
	uint32_t   /* reserved */        : 2;
	uint32_t   pixelDataSelect       : 1;
	uint32_t   systematicDataSelect  : 1;
	uint32_t   /* reserved */        : 2;
	uint32_t   pixelDataSize         : 2;
	uint32_t   hsyncEdge             : 1;
	uint32_t   vsyncEdge             : 1;
	uint32_t   /* reserved */        : 12;
	/* HW Test Gen Image Config */
	uint32_t   imageWidth            : 14;
	uint32_t   /* reserved */        : 2;
	uint32_t   imageHeight           : 14;
	uint32_t   /* reserved */        : 2;
	/* SOF Offset Config */
	uint32_t   sofOffset             : 24;
	uint32_t   /* reserved */        : 8;
	/* EOF NOffset Config */
	uint32_t   eofNOffset            : 24;
	uint32_t   /* reserved */        : 8;
	/* SOL Offset Config */
	uint32_t   solOffset             : 9;
	uint32_t   /* reserved */        : 23;
	/* EOL NOffset Config */
	uint32_t   eolNOffset            : 9;
	uint32_t   /* reserved */        : 23;
	/* HBI Config */
	uint32_t   hBlankInterval        : 14;
	uint32_t   /* reserved */        : 18;
	/* VBL Config */
	uint32_t   vBlankInterval        : 14;
	uint32_t   /* reserved */        : 2;
	uint32_t   vBlankIntervalEnable  : 1;
	uint32_t   /* reserved */        : 15;
	/* SOF Dummy Line Config */
	uint32_t   sofDummy              : 8;
	uint32_t   /* reserved */        : 24;
	/* EOF Dummy Line Config */
	uint32_t   eofDummy              : 8;
	uint32_t   /* reserved */        : 24;
	/* Color Bars Config */
	uint32_t   unicolorBarSelect     : 3;
	uint32_t   /* reserved */        : 1;
	uint32_t   unicolorBarEnable     : 1;
	uint32_t   splitEnable           : 1;
	uint32_t   pixelPattern          : 2;
	uint32_t   rotatePeriod          : 6;
	uint32_t   /* reserved */        : 18;
	/* Random Config */
	uint32_t   randomSeed            : 16;
	uint32_t   /* reserved */        : 16;
} __attribute__((packed, aligned(4)));

struct VFE_Bus_Pm_ConfigCmdType {
	/* VFE Bus Performance Monitor Config */
	uint32_t  output2YWrPmEnable           : 1;
	uint32_t  output2CbcrWrPmEnable        : 1;
	uint32_t  output1YWrPmEnable           : 1;
	uint32_t  output1CbcrWrPmEnable        : 1;
	uint32_t  /* reserved */               : 28;
} __attribute__((packed, aligned(4)));

struct vfe_asf_info_t {
	/* asf max edge  */
	uint32_t maxEdge         :  13;
	uint32_t /* reserved */  :   3;
	/* HBi count  */
	uint32_t HBICount        :  12;
	uint32_t /* reserved */  :   4;
} __attribute__((packed, aligned(4)));

struct vfe_camif_stats_t {
  uint32_t  pixelCount     :  14;
  uint32_t  /* reserved */ :   2;
  uint32_t  lineCount      :  14;
  uint32_t  /* reserved */ :   1;
  uint32_t  camifHalt      :   1;
} __attribute__((packed, aligned(4)));

struct VFE_StatsCmdType {
	uint32_t  autoFocusEnable       :  1;
	uint32_t  axwEnable             :  1;
	uint32_t  histEnable            :  1;
	uint32_t  clearHistEnable       :  1;
	uint32_t  histAutoClearEnable   :  1;
	uint32_t  colorConversionEnable :  1;
	uint32_t  /* reserved */        : 26;
} __attribute__((packed, aligned(4)));


struct vfe_statsframe_t {
	uint32_t lastPixel      :  12;
	uint32_t /* reserved */ :   4;
	uint32_t lastLine       :  12;
	uint32_t /* reserved */ :   4;
} __attribute__((packed, aligned(4)));

struct vfe_busstats_wrprio_t {
	uint32_t afBusPriority     : 4;
	uint32_t awbBusPriority    : 4;
	uint32_t histBusPriority   : 4;
	uint32_t afBusPriorityEn   : 1;
	uint32_t awbBusPriorityEn  : 1;
	uint32_t histBusPriorityEn : 1;
	uint32_t /* reserved */    : 17;
} __attribute__((packed, aligned(4)));

struct vfe_statsaf_update_t {
	/* VFE_STATS_AF_CFG */
	uint32_t windowVOffset   :   12;
	uint32_t /* reserved */  :    4;
	uint32_t windowHOffset   :   12;
	uint32_t /* reserved */  :    3;
	uint32_t windowMode      :    1;

	/* VFE_STATS_AF_DIM */
	uint32_t windowHeight    :   12;
	uint32_t /* reserved */  :    4;
	uint32_t windowWidth     :   12;
	uint32_t /* reserved */  :    4;
} __attribute__((packed, aligned(4)));

struct vfe_statsaf_cfg_t {
	/* VFE_STATS_AF_GRID_0 */
	uint32_t  entry00        :   8;
	uint32_t  entry01        :   8;
	uint32_t  entry02        :   8;
	uint32_t  entry03        :   8;

	/* VFE_STATS_AF_GRID_1 */
	uint32_t  entry10        :   8;
	uint32_t  entry11        :   8;
	uint32_t  entry12        :   8;
	uint32_t  entry13        :   8;

	/* VFE_STATS_AF_GRID_2 */
	uint32_t  entry20        :   8;
	uint32_t  entry21        :   8;
	uint32_t  entry22        :   8;
	uint32_t  entry23        :   8;

	/* VFE_STATS_AF_GRID_3 */
	uint32_t  entry30        :   8;
	uint32_t  entry31        :   8;
	uint32_t  entry32        :   8;
	uint32_t  entry33        :   8;

	/* VFE_STATS_AF_HEADER */
	uint32_t  afHeader       :   8;
	uint32_t  /* reserved */ :  24;
	/*  VFE_STATS_AF_COEF0 */
	uint32_t  a00            :    5;
	uint32_t  a04            :    5;
	uint32_t  fvMax          :   11;
	uint32_t  fvMetric       :    1;
	uint32_t  /* reserved */ :   10;

	/* VFE_STATS_AF_COEF1 */
	uint32_t  a20            :   5;
	uint32_t  a21            :   5;
	uint32_t  a22            :   5;
	uint32_t  a23            :   5;
	uint32_t  a24            :   5;
	uint32_t  /* reserved */ :   7;
} __attribute__((packed, aligned(4)));

struct vfe_statsawbae_update_t {
	uint32_t  aeRegionCfg    :   1;
	uint32_t  aeSubregionCfg :   1;
	uint32_t  /* reserved */ :  14;
	uint32_t  awbYMin        :   8;
	uint32_t  awbYMax        :   8;
} __attribute__((packed, aligned(4)));

struct vfe_statsaxw_hdr_cfg_t {
	/* Stats AXW Header Config */
	uint32_t axwHeader      :   8;
	uint32_t /* reserved */ :  24;
} __attribute__((packed, aligned(4)));

struct vfe_statsawb_update_t {
	/* AWB MConfig */
	uint32_t  m4             :  8;
	uint32_t  m3             :  8;
	uint32_t  m2             :  8;
	uint32_t  m1             :  8;

	/* AWB CConfig1 */
	uint32_t  c2             :  12;
	uint32_t  /* reserved */ :   4;
	uint32_t  c1             :  12;
	uint32_t  /* reserved */ :   4;

	/* AWB CConfig2 */
	uint32_t  c4             :  12;
	uint32_t  /* reserved */ :   4;
	uint32_t  c3             :  12;
	uint32_t  /* reserved */ :   4;
} __attribute__((packed, aligned(4)));

struct VFE_SyncTimerCmdType {
	uint32_t  hsyncCount     : 12;
	uint32_t  /* reserved */ : 20;
	uint32_t  pclkCount      : 18;
	uint32_t  /* reserved */ : 14;
	uint32_t  outputDuration : 28;
	uint32_t  /* reserved */ :  4;
} __attribute__((packed, aligned(4)));

struct VFE_AsyncTimerCmdType {
	/*  config 0 */
	uint32_t    inactiveCount : 20;
	uint32_t    repeatCount   : 10;
	uint32_t    /* reserved */:  1;
	uint32_t    polarity      :  1;
	/*  config 1 */
	uint32_t    activeCount   : 20;
	uint32_t    /* reserved */: 12;
} __attribute__((packed, aligned(4)));

struct VFE_AxiInputCmdType {
	uint32_t   stripeStartAddr0   : 32;
	uint32_t   stripeStartAddr1   : 32;
	uint32_t   stripeStartAddr2   : 32;
	uint32_t   stripeStartAddr3   : 32;

	uint32_t   ySize              : 12;
	uint32_t   yOffsetDelta       : 12;
	uint32_t   /* reserved */     : 8;

	/* bus_stripe_rd_hSize */
	uint32_t   /* reserved */     : 16;
	uint32_t   xSizeWord          : 10;
	uint32_t   /* reserved */     : 6;

	/* bus_stripe_rd_buffer_cfg */
	uint32_t   burstLength        : 2;
	uint32_t   /* reserved */     : 2;
	uint32_t   NumOfRows          : 12;
	uint32_t   RowIncrement       : 12;
	uint32_t   /* reserved */     : 4;

	/* bus_stripe_rd_unpack_cfg */
	uint32_t   mainUnpackHeight   : 12;
	uint32_t   mainUnpackWidth    : 13;
	uint32_t   mainUnpackHbiSel   : 3;
	uint32_t   mainUnpackPhase    : 3;
	uint32_t   /* reserved */     : 1;

	/* bus_stripe_rd_unpack */
	uint32_t   unpackPattern      : 32;

	/* bus_stripe_rd_pad_size */
	uint32_t   padLeft            : 7;
	uint32_t   /* reserved */     : 1;
	uint32_t   padRight           : 7;
	uint32_t   /* reserved */     : 1;
	uint32_t   padTop             : 7;
	uint32_t   /* reserved */     : 1;
	uint32_t   padBottom          : 7;
	uint32_t   /* reserved */     : 1;

	/* bus_stripe_rd_pad_L_unpack */
	uint32_t   leftUnpackPattern0 : 4;
	uint32_t   leftUnpackPattern1 : 4;
	uint32_t   leftUnpackPattern2 : 4;
	uint32_t   leftUnpackPattern3 : 4;
	uint32_t   leftUnpackStop0    : 1;
	uint32_t   leftUnpackStop1    : 1;
	uint32_t   leftUnpackStop2    : 1;
	uint32_t   leftUnpackStop3    : 1;
	uint32_t   /* reserved */     : 12;

	/* bus_stripe_rd_pad_R_unpack */
	uint32_t   rightUnpackPattern0 : 4;
	uint32_t   rightUnpackPattern1 : 4;
	uint32_t   rightUnpackPattern2 : 4;
	uint32_t   rightUnpackPattern3 : 4;
	uint32_t   rightUnpackStop0    : 1;
	uint32_t   rightUnpackStop1    : 1;
	uint32_t   rightUnpackStop2    : 1;
	uint32_t   rightUnpackStop3    : 1;
	uint32_t   /* reserved */      : 12;

	/* bus_stripe_rd_pad_tb_unpack */
	uint32_t   topUnapckPattern    : 4;
	uint32_t   /* reserved */      : 12;
	uint32_t   bottomUnapckPattern : 4;
	uint32_t   /* reserved */      : 12;
} __attribute__((packed, aligned(4)));

struct VFE_AxiRdFragIrqEnable {
	uint32_t stripeRdFragirq0Enable   : 1;
	uint32_t stripeRdFragirq1Enable   : 1;
	uint32_t stripeRdFragirq2Enable   : 1;
	uint32_t stripeRdFragirq3Enable   : 1;
	uint32_t   /* reserved */           : 28;
} __attribute__((packed, aligned(4)));

int vfe_cmd_init(struct msm_vfe_resp *, struct platform_device *);
void vfe_parse_irq(void);
void vfe_stats_af_stop(void);
void vfe_stop(void);
void vfe_update(void);
int vfe_rgb_gamma_update(struct vfe_cmd_rgb_gamma_config *);
int vfe_rgb_gamma_config(struct vfe_cmd_rgb_gamma_config *);
void vfe_stats_wb_exp_ack(struct vfe_cmd_stats_wb_exp_ack *);
void vfe_stats_af_ack(struct vfe_cmd_stats_af_ack *);
void vfe_start(struct vfe_cmd_start *);
void vfe_la_update(struct vfe_cmd_la_config *);
void vfe_la_config(struct vfe_cmd_la_config *);
void vfe_test_gen_start(struct vfe_cmd_test_gen_start *);
void vfe_frame_skip_update(struct vfe_cmd_frame_skip_update *);
void vfe_frame_skip_config(struct vfe_cmd_frame_skip_config *);
void vfe_output_clamp_config(struct vfe_cmd_output_clamp_config *);
void vfe_camif_frame_update(struct vfe_cmds_camif_frame *);
void vfe_color_correction_config(struct vfe_cmd_color_correction_config *);
void vfe_demosaic_abf_update(struct vfe_cmd_demosaic_abf_update *);
void vfe_demosaic_bpc_update(struct vfe_cmd_demosaic_bpc_update *);
void vfe_demosaic_config(struct vfe_cmd_demosaic_config *);
void vfe_demux_channel_gain_update(struct vfe_cmd_demux_channel_gain_config *);
void vfe_demux_channel_gain_config(struct vfe_cmd_demux_channel_gain_config *);
void vfe_black_level_update(struct vfe_cmd_black_level_config *);
void vfe_black_level_config(struct vfe_cmd_black_level_config *);
void vfe_asf_update(struct vfe_cmd_asf_update *);
void vfe_asf_config(struct vfe_cmd_asf_config *);
void vfe_white_balance_config(struct vfe_cmd_white_balance_config *);
void vfe_chroma_sup_config(struct vfe_cmd_chroma_suppression_config *);
void vfe_roll_off_config(struct vfe_cmd_roll_off_config *);
void vfe_chroma_subsample_config(struct vfe_cmd_chroma_subsample_config *);
void vfe_chroma_enhan_config(struct vfe_cmd_chroma_enhan_config *);
void vfe_scaler2cbcr_config(struct vfe_cmd_scaler2_config *);
void vfe_scaler2y_config(struct vfe_cmd_scaler2_config *);
void vfe_main_scaler_config(struct vfe_cmd_main_scaler_config *);
void vfe_stats_wb_exp_stop(void);
void vfe_stats_update_wb_exp(struct vfe_cmd_stats_wb_exp_update *);
void vfe_stats_update_af(struct vfe_cmd_stats_af_update *);
void vfe_stats_start_wb_exp(struct vfe_cmd_stats_wb_exp_start *);
void vfe_stats_start_af(struct vfe_cmd_stats_af_start *);
void vfe_stats_setting(struct vfe_cmd_stats_setting *);
void vfe_axi_input_config(struct vfe_cmd_axi_input_config *);
void vfe_stats_config(struct vfe_cmd_stats_setting *);
void vfe_axi_output_config(struct vfe_cmd_axi_output_config *);
void vfe_camif_config(struct vfe_cmd_camif_config *);
void vfe_fov_crop_config(struct vfe_cmd_fov_crop_config *);
void vfe_get_hw_version(struct vfe_cmd_hw_version *);
void vfe_reset(void);
void vfe_cmd_release(void);
void vfe_output1_ack(struct vfe_cmd_output_ack *);
void vfe_output2_ack(struct vfe_cmd_output_ack *);
#endif /* __MSM_VFE8X_REG_H__ */
