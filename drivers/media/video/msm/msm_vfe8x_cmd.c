/*lint -e571 (Suspicious cast) OK from uint16 to uint32 */
/*lint -e732 (Loss of sign (assignment)) The sign is kept, at bit packing.*/
/*lint -e632 (Assignment to strong type 'uint32)   packing to 32bit registers.*/
/*==============================================================================

FILE:      vfe_hal.c          
           
SERVICES:  
            VFE HAL in kernal space.
            
GENERAL DESCRIPTION:

PUBLIC CLASSES AND STATIC FUNCTIONS:

INITIALIZATION & SEQUENCING REQUIREMENTS:

    

        Copyright © 2008 QUALCOMM Incorporated.
 	 Copyright (c) 2008 QUALCOMM USA, INC.
 
  All source code in this file is licensed under the following license
  except where indicated.
 
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, you can find it at http://www.fsf.org
==============================================================================*/
#include "AEEstd.h"
#include "vfe_hal.h"
#include "vfe_hal_if.h"

#include "vfe_hw_reg.h"
#include "vfe_utils.h"
#include "CameraSensorCommon.h"

uint8* VFE_BASE=NULL;


/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/
/*========================    Common Utility  ===============================*/
/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/




/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_program_hw
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_program_hw(uint8_t *hwreg
	uint32_t *inptr, uint32_t regcnt)
{
	uint32_t i;
	volatile uint32_t *p;
      
	p = (volatile uint32_t *)(hwreg);

	for (i = 0; i < (regcnt >> 2); i++)
		*p++ = *inptr++;
}

void vfe_8x_program_register_value(uint8_t *hwRegAddress,
	uint32_t inputValue)
{
	volatile uint32_t *tempPtr;

	tempPtr = (volatile uint32_t*)(hwRegAddress);

	*tempPtr = inputValue;
}

void vfe_8x_program_register_value2(uint8_t *hwRegAddress,
	uint32_t* pIn)
{
	volatile uint32_t* tempPtr;

	tempPtr = (volatile uint32_t*)(hwRegAddress);
	*tempPtr = *pIn;
}

uint32_t vfe_8x_read_reg_value(uint8_t *hwRegAddress)
{
	volatile uint32_t* tempPtr;

	tempPtr = (volatile uint32_t*)(hwRegAddress);

	return *tempPtr;
}

void vfe_8x_read_reg_values(uint8_t *hwRegAddress, uint32_t *destiPtr,
	uint32_t count)
{
	volatile uint32_t* tempPtr;
  uint32_t i;

	tempPtr = (volatile uint32_t*)(hwRegAddress);

	for (i = 0; i < count; i++)
		*destiPtr++ = *tempPtr++;
}

void vfe_8x_reset(uint32_t imask)
{
	/* disable all interrupts.  */
	vfe_8x_program_register_value(VFE_IRQ_COMPOSITE_MASK, VFE_DISABLE_ALL_IRQS);

	/* clear all pending interrupts*/   
	vfe_8x_program_register_value(VFE_IRQ_CLEAR, VFE_CLEAR_ALL_IRQS);

	/* enable reset_ack interrupt.  */
	vfe_8x_program_register_value(VFE_IRQ_MASK, imask);

	/* Write to VFE_GLOBAL_RESET_CMD to reset the vfe hardware.  Once reset is done,
	   hardware interrupt will be generated.  VFE ist processes the interrupt to complete
	   the function call.  Note that the reset function is synchronous. */
	vfe_8x_program_register_value(VFE_GLOBAL_RESET_CMD, VFE_RESET_UPON_RESET_CMD);
}

vfe_interrupt_status vfe_8x_parse_interrupt_status (uint32_t irqStatusIn)
{

	struct VFE_IrqStatusType hwIrqStatus;
	struct vfe_interrupt_status returnData;
    boolean    tempStatus;

	(void)memset((void*)&hwIrqStatus, 0, sizeof(hwIrqStatus));
	(void)memset((void*)&returnData, 0, sizeof(returnData));

	hwIrqStatus = *((VFE_IrqStatusType*)(&irqStatusIn));

    /* convert the status bit to data structure to DAL */
	returnData.camifErrorIrq       = (boolean)hwIrqStatus.camifErrorIrq;          
	returnData.camifSofIrq         = (boolean)hwIrqStatus.camifSofIrq;
	returnData.camifEolIrq         = (boolean)hwIrqStatus.camifEolIrq;        
	returnData.camifEofIrq         = (boolean)hwIrqStatus.camifEofIrq;        
	returnData.camifEpoch1Irq      = (boolean)hwIrqStatus.camifEpoch1Irq;     
	returnData.camifEpoch2Irq      = (boolean)hwIrqStatus.camifEpoch2Irq;     
	returnData.camifOverflowIrq    = (boolean)hwIrqStatus.camifOverflowIrq;   
	returnData.ceIrq               = (boolean)hwIrqStatus.ceIrq;              
	returnData.regUpdateIrq        = (boolean)hwIrqStatus.regUpdateIrq;       
	returnData.resetAckIrq         = (boolean)hwIrqStatus.resetAckIrq;        
	returnData.encYPingpongIrq     = (boolean)hwIrqStatus.encYPingpongIrq;    
	returnData.encCbcrPingpongIrq  = (boolean)hwIrqStatus.encCbcrPingpongIrq; 
	returnData.viewYPingpongIrq    = (boolean)hwIrqStatus.viewYPingpongIrq;   
	returnData.viewCbcrPingpongIrq = (boolean)hwIrqStatus.viewCbcrPingpongIrq;
	returnData.rdPingpongIrq       = (boolean)hwIrqStatus.rdPingpongIrq;      
	returnData.afPingpongIrq       = (boolean)hwIrqStatus.afPingpongIrq;      
	returnData.awbPingpongIrq      = (boolean)hwIrqStatus.awbPingpongIrq;     
	returnData.histPingpongIrq     = (boolean)hwIrqStatus.histPingpongIrq;    
	returnData.encIrq              = (boolean)hwIrqStatus.encIrq;             
	returnData.viewIrq             = (boolean)hwIrqStatus.viewIrq;            
	returnData.busOverflowIrq      = (boolean)hwIrqStatus.busOverflowIrq;     
	returnData.afOverflowIrq       = (boolean)hwIrqStatus.afOverflowIrq;      
	returnData.awbOverflowIrq      = (boolean)hwIrqStatus.awbOverflowIrq;     
	returnData.syncTimer0Irq       = (boolean)hwIrqStatus.syncTimer0Irq;      
	returnData.syncTimer1Irq       = (boolean)hwIrqStatus.syncTimer1Irq;      
	returnData.syncTimer2Irq       = (boolean)hwIrqStatus.syncTimer2Irq;      
	returnData.asyncTimer0Irq      = (boolean)hwIrqStatus.asyncTimer0Irq;     
	returnData.asyncTimer1Irq      = (boolean)hwIrqStatus.asyncTimer1Irq;     
	returnData.asyncTimer2Irq      = (boolean)hwIrqStatus.asyncTimer2Irq;     
	returnData.asyncTimer3Irq      = (boolean)hwIrqStatus.asyncTimer3Irq;     
	returnData.axiErrorIrq         = (boolean)hwIrqStatus.axiErrorIrq;                
	returnData.violationIrq        = (boolean)hwIrqStatus.violationIrq;               


	/* logic OR of any error bits*/
	/* although each irq corresponds to a bit, the data type here is a boolean already.
	   hence use logic operation.*/
    tempStatus = returnData.camifErrorIrq || returnData.camifOverflowIrq ||
		         returnData.afOverflowIrq || returnData.awbPingpongIrq ||
                 returnData.busOverflowIrq || returnData.axiErrorIrq ||
		         returnData.violationIrq;
	returnData.anyErrorIrqs = tempStatus;  


	/* logic OR of any output path bits*/
    tempStatus = returnData.encYPingpongIrq || returnData.encCbcrPingpongIrq ||
                 returnData.encIrq;
	returnData.anyOutput2PathIrqs = tempStatus;  

    tempStatus = returnData.viewYPingpongIrq || returnData.viewCbcrPingpongIrq ||
                 returnData.viewIrq;
    returnData.anyOutput1PathIrqs = tempStatus;

	returnData.anyOutputPathIrqs = returnData.anyOutput1PathIrqs || returnData.anyOutput2PathIrqs;



	/* logic OR of any sync timer bits*/
	tempStatus = returnData.syncTimer0Irq || returnData.syncTimer1Irq ||
				 returnData.syncTimer2Irq ;
	returnData.anySyncTimerIrqs = tempStatus;  


	/* logic OR of any async timer bits*/
	tempStatus = returnData.asyncTimer0Irq || returnData.asyncTimer1Irq ||
				 returnData.asyncTimer2Irq || returnData.asyncTimer3Irq ;
	returnData.anyAsyncTimerIrqs = tempStatus;  




	// bool for all interrupts that are not allowed in idle state
    tempStatus = returnData.anyErrorIrqs || returnData.anyOutputPathIrqs 
		      || returnData.anySyncTimerIrqs|| returnData.regUpdateIrq
              || returnData.awbPingpongIrq || returnData.afPingpongIrq
              || returnData.camifSofIrq || returnData.camifEpoch2Irq
              || returnData.camifEpoch1Irq;
	returnData.anyIrqForActiveStatesOnly = tempStatus;


	return returnData;
																		 
}
																		 
 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_read_interrupt_status
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

 void vfe_8x_read_interrupt_status (vfe_irq_thread_msg* pOut)
{

	volatile uint32_t* tempPtr;

	(void)memset((void*)pOut, 0, sizeof(vfe_irq_thread_msg));

    tempPtr = (volatile uint32_t*)VFE_IRQ_STATUS;
	pOut->vfeIrqStatus = *tempPtr;


	tempPtr = (volatile uint32_t*)CAMIF_STATUS;
	pOut->camifStatus = *tempPtr;
	CDBG("camif status (packed) is 0x%x\n", pOut->camifStatus);


	tempPtr = (volatile uint32_t*)VFE_DEMOSAIC_STATUS;
	pOut->demosaicStatus = *tempPtr;


	tempPtr = (volatile uint32_t*)VFE_ASF_MAX_EDGE;
	pOut->asfMaxEdge = *tempPtr;



	tempPtr = (volatile uint32_t*)VFE_BUS_ENC_Y_WR_PM_STATS_0;

	pOut->pmInfo.encPathPmInfo.yWrPmStats0      = *tempPtr++;
	pOut->pmInfo.encPathPmInfo.yWrPmStats1      = *tempPtr++;
	pOut->pmInfo.encPathPmInfo.cbcrWrPmStats0   = *tempPtr++;
	pOut->pmInfo.encPathPmInfo.cbcrWrPmStats1   = *tempPtr++;
	pOut->pmInfo.viewPathPmInfo.yWrPmStats0     = *tempPtr++;
	pOut->pmInfo.viewPathPmInfo.yWrPmStats1     = *tempPtr++;
	pOut->pmInfo.viewPathPmInfo.cbcrWrPmStats0  = *tempPtr++;
	pOut->pmInfo.viewPathPmInfo.cbcrWrPmStats1  = *tempPtr;


}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_get_asf_frame_info
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

vfe_frame_asf_info  vfe_8x_get_asf_frame_info(vfe_irq_thread_msg* pIn){

    struct VFE_AsfInfoType     asfInfoTemp;
    struct vfe_frame_asf_info  returnData;

	(void)memset((void*)&returnData, 0, sizeof(returnData));
	(void)memset((void*)&asfInfoTemp, 0, sizeof(asfInfoTemp));


    asfInfoTemp = *((VFE_AsfInfoType*)(&(pIn->asfMaxEdge)));

    returnData.asfHbiCount = asfInfoTemp.HBICount;
	returnData.asfMaxEdge  = asfInfoTemp.maxEdge;

	return returnData;

}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_get_demosaic_frame_info
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

vfe_frame_bpc_info  vfe_8x_get_demosaic_frame_info(vfe_irq_thread_msg* pIn){

    struct VFE_BpcInfoType     bpcInfoTemp;
    struct vfe_frame_bpc_info  returnData;

	(void)memset((void*)&returnData, 0, sizeof(returnData));
	(void)memset((void*)&bpcInfoTemp, 0, sizeof(bpcInfoTemp));


    bpcInfoTemp = *((VFE_BpcInfoType*)(&(pIn->demosaicStatus)));

	returnData.greenDefectPixelCount = bpcInfoTemp.greenBadPixelCount;
	returnData.redBlueDefectPixelCount  = bpcInfoTemp.RedBlueBadPixelCount;
     
	return returnData;


}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_get_camif_status
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

vfe_msg_camif_status  vfe_8x_get_camif_status (vfe_irq_thread_msg* pIn){

	struct VFE_CamifStatusType camifStatusTemp;
    struct vfe_msg_camif_status returnData;

	(void)memset((void*)&returnData, 0, sizeof(returnData));
	(void)memset((void*)&camifStatusTemp, 0, sizeof(camifStatusTemp));


    camifStatusTemp = *((VFE_CamifStatusType*)(&(pIn->camifStatus)));
    returnData.camifState = (boolean)camifStatusTemp.camifHalt;
    returnData.lineCount =  camifStatusTemp.lineCount;
    returnData.pixelCount = camifStatusTemp.pixelCount;

	return returnData;
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_irq_pack
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

uint32_t vfe_8x_irq_pack(vfe_interrupt_mask data){

    struct VFE_IrqEnableType packedData;

	(void)memset((void*)&packedData, 0, sizeof(packedData));


    packedData.camifErrorIrq          = data.camifErrorIrq;       
	packedData.camifSofIrq            = data.camifSofIrq;  
	packedData.camifEolIrq            = data.camifEolIrq;  
	packedData.camifEofIrq            = data.camifEofIrq;           
	packedData.camifEpoch1Irq         = data.camifEpoch1Irq;        
	packedData.camifEpoch2Irq         = data.camifEpoch2Irq;        
	packedData.camifOverflowIrq       = data.camifOverflowIrq;      
	packedData.ceIrq                  = data.ceIrq;                 
	packedData.regUpdateIrq           = data.regUpdateIrq;          
	packedData.resetAckIrq            = data.resetAckIrq;           
	packedData.encYPingpongIrq        = data.encYPingpongIrq;       
	packedData.encCbcrPingpongIrq     = data.encCbcrPingpongIrq;     
	packedData.viewYPingpongIrq       = data.viewYPingpongIrq;      
	packedData.viewCbcrPingpongIrq    = data.viewCbcrPingpongIrq;   
	packedData.rdPingpongIrq          = data.rdPingpongIrq;         
	packedData.afPingpongIrq          = data.afPingpongIrq;         
	packedData.awbPingpongIrq         = data.awbPingpongIrq;        
	packedData.histPingpongIrq        = data.histPingpongIrq;       
	packedData.encIrq                 = data.encIrq;                
	packedData.viewIrq                = data.viewIrq;               
	packedData.busOverflowIrq         = data.busOverflowIrq;        
	packedData.afOverflowIrq          = data.afOverflowIrq;         
	packedData.awbOverflowIrq         = data.awbOverflowIrq;        
	packedData.syncTimer0Irq          = data.syncTimer0Irq;         
    packedData.syncTimer1Irq          = data.syncTimer1Irq;         
	packedData.syncTimer2Irq          = data.syncTimer2Irq;         
	packedData.asyncTimer0Irq         = data.asyncTimer0Irq;        
	packedData.asyncTimer1Irq         = data.asyncTimer1Irq;        
	packedData.asyncTimer2Irq         = data.asyncTimer2Irq;        
	packedData.asyncTimer3Irq         = data.asyncTimer3Irq;        
	packedData.axiErrorIrq            = data.axiErrorIrq;           
	packedData.violationIrq           = data.violationIrq;

    return *((uint32_t*)&packedData);
}





 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_irq_composite_pack
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
uint32_t vfe_8x_irq_composite_pack(vfe_irq_composite_mask_config data){

	struct VFE_Irq_Composite_MaskType packedData;

	(void)memset((void*)&packedData, 0, sizeof(packedData));

	packedData.encIrqComMaskBits   = data.encIrqComMask;
	packedData.viewIrqComMaskBits  = data.viewIrqComMask;
	packedData.ceDoneSelBits       = data.ceDoneSel;

    return *((uint32_t*)&packedData);
    
}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_read_irq_mask
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
VFE_IrqEnableType vfe_8x_read_irq_mask(void){

	volatile uint32_t* tempPtr;
    struct VFE_IrqEnableType returnData;

	(void)memset((void*)&returnData, 0, sizeof(returnData));

	tempPtr = (volatile uint32_t*)(VFE_IRQ_MASK);

	returnData = *((VFE_IrqEnableType*)tempPtr);
    
	return returnData;
}




/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_bus_performance_monitor
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
vfe_bus_performance_monitor  vfe_8x_get_performance_monitor_data (vfe_irq_thread_msg* pIn)

{
	 struct vfe_bus_performance_monitor returnData;

	 (void)memset((void*)&returnData, 0, sizeof(returnData));

     returnData.encPathPmInfo.yWrPmStats0    = pIn->pmInfo.encPathPmInfo.yWrPmStats0;   
	 returnData.encPathPmInfo.yWrPmStats1    = pIn->pmInfo.encPathPmInfo.yWrPmStats1;   
	 returnData.encPathPmInfo.cbcrWrPmStats0 = pIn->pmInfo.encPathPmInfo.cbcrWrPmStats0;
	 returnData.encPathPmInfo.cbcrWrPmStats1 = pIn->pmInfo.encPathPmInfo.cbcrWrPmStats1;

     returnData.viewPathPmInfo.yWrPmStats0    = pIn->pmInfo.viewPathPmInfo.yWrPmStats0;    
	 returnData.viewPathPmInfo.yWrPmStats1    = pIn->pmInfo.viewPathPmInfo.yWrPmStats1;    
	 returnData.viewPathPmInfo.cbcrWrPmStats0 = pIn->pmInfo.viewPathPmInfo.cbcrWrPmStats0; 
	 returnData.viewPathPmInfo.cbcrWrPmStats1 = pIn->pmInfo.viewPathPmInfo.cbcrWrPmStats1; 

	 return returnData;
}






/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_get_hw_version
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_8x_get_hw_version(vfe_cmd_hw_version *pOut){

	 uint32_t vfeHwVersionPacked;
	 struct VFE_HW_VersionType vfeHwVersion;

     vfeHwVersionPacked = vfe_8x_read_reg_value(VFE_HW_VERSION);
     vfeHwVersion = *((VFE_HW_VersionType*)&vfeHwVersionPacked);
 
	 pOut->coreVersion  = vfeHwVersion.coreVersion;
	 pOut->minorVersion = vfeHwVersion.minorVersion;
	 pOut->majorVersion = vfeHwVersion.majorVersion;

	}






/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/
/*========================    Module specific ===============================*/
/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/
/*===========================================================================*/




/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_FOV_crop_config
 * ----------------------------------------------------------------------------*/
void vfe_8x_crop_config(struct vfe_cmd_fov_crop_config *pIn)
{
	struct VFE_FOV_CropConfigCmdType hwCommand;

	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

	/* FOV Corp, Part 1 */
	hwCommand.lastPixel  = pIn->lastPixel;
	hwCommand.firstPixel = pIn->firstPixel;

	/* FOV Corp, Part 2 */
	hwCommand.lastLine   = pIn->lastLine;
	hwCommand.firstLine  = pIn->firstLine;

	vfe_program_hw(VFE_CROP_WIDTH_CFG,
		(uint32_t *)&hwCommand, sizeof(hwCommand));

}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_camif_config
 * ----------------------------------------------------------------------------*/

void vfe_8x_camif_config(vfe_cmd_camif_config *pIn){

     struct VFE_CAMIFConfigCmdType  hwCommand;

	 (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

	 /* EFS_Config */
	 hwCommand.efsEndOfLine     = pIn->EFS.efsEndOfLine; 
	 hwCommand.efsStartOfLine   = pIn->EFS.efsStartOfLine;
	 hwCommand.efsEndOfFrame    = pIn->EFS.efsEndOfFrame;
	 hwCommand.efsStartOfFrame  = pIn->EFS.efsStartOfFrame; 

	 /* Frame Config */
     hwCommand.frameConfigPixelsPerLine  = pIn->frame.pixelsPerLine;
     hwCommand.frameConfigLinesPerFrame  = pIn->frame.linesPerFrame;

    /* Window Width Config */
     hwCommand.windowWidthCfgLastPixel  = pIn->window.lastPixel;
     hwCommand.windowWidthCfgFirstPixel = pIn->window.firstPixel;

    /* Window Height Config */
	 hwCommand.windowHeightCfglastLine   = pIn->window.lastLine;
	 hwCommand.windowHeightCfgfirstLine  = pIn->window.firstLine;

    /* Subsample 1 Config */
	 hwCommand.subsample1CfgPixelSkip = pIn->subsample.pixelSkipMask;
	 hwCommand.subsample1CfgLineSkip  = pIn->subsample.lineSkipMask;

    /* Subsample 2 Config */
	 hwCommand.subsample2CfgFrameSkip      = (uint32_t)pIn->subsample.frameSkip;
	 hwCommand.subsample2CfgFrameSkipMode  = pIn->subsample.frameSkipMode;
	 hwCommand.subsample2CfgPixelSkipWrap  = pIn->subsample.pixelSkipWrap;

    /* Epoch Interrupt */
     hwCommand.epoch1Line = pIn->epoch1.lineIndex;
     hwCommand.epoch2Line = pIn->epoch2.lineIndex;

     vfe_program_hw (CAMIF_EFS_CONFIG, (uint32_t*)&hwCommand, sizeof(hwCommand));

}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_axi_output_config
 * ----------------------------------------------------------------------------*/

void vfe_8x_axi_output_config (vfe_cmd_axi_output_config  *pIn, 
								vfe_output_path_combo* output1Path,  
								vfe_output_path_combo* output2Path,
								uint16 outPpw){

	// note ppw really is color component per hardware word.  For bayer, it is the same.
	// for YUV, upper layer is expected to pass down width * 2 (???)
	struct VFE_AXIOutputConfigCmdType  hwCommand;

	uint16 temp;

//    uint32_t burstLength = pIn->burstLength;
    uint32_t burstLength;
  
	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

/*
	switch (burstLength) {
	    case 2:
			burstLength = 1;
			break;
		case 4:
			burstLength = 1;
			break;
		case 8:
			burstLength = 1; 
			break;
		case 16:
			burstLength = 1;  
			break;

	}
*/

	/* force it to burst length 4, hardware does not support it. */
	burstLength = 1;


	/* AXI Output 2 Y Configuration*/
	/* VFE_BUS_ENC_Y_WR_PING_ADDR  */
    hwCommand.out2YPingAddr = output2Path->yPath.addressBuffer[0];                       

	/* VFE_BUS_ENC_Y_WR_PONG_ADDR  */
	hwCommand.out2YPongAddr = output2Path->yPath.addressBuffer[1];;                 

	/* VFE_BUS_ENC_Y_WR_IMAGE_SIZE     */
	hwCommand.out2YImageHeight= pIn->output2.outputY.imageHeight;           
	/* convert the image width and row increment to be in unit of 64bit (8 bytes) */
	temp = (pIn->output2.outputY.imageWidth + (outPpw-1))/outPpw;     /* round up */
	hwCommand.out2YImageWidthin64bit=temp;              

	/* VFE_BUS_ENC_Y_WR_BUFFER_CFG     */
	hwCommand.out2YBurstLength = burstLength;                 
	hwCommand.out2YNumRows =pIn->output2.outputY.outRowCount;                        
	temp = (pIn->output2.outputY.outRowIncrement + (outPpw-1))/outPpw;     /* round up */
	hwCommand.out2YRowIncrementIn64bit =temp;            

    /* AXI Output 2 Cbcr Configuration*/
	/* VFE_BUS_ENC_Cbcr_WR_PING_ADDR  */
    hwCommand.out2CbcrPingAddr =output2Path->cbcrPath.addressBuffer[0];                 

	/* VFE_BUS_ENC_Cbcr_WR_PONG_ADDR  */
	hwCommand.out2CbcrPongAddr =output2Path->cbcrPath.addressBuffer[1];                   

	/* VFE_BUS_ENC_Cbcr_WR_IMAGE_SIZE     */
	hwCommand.out2CbcrImageHeight= pIn->output2.outputCbcr.imageHeight;                 
	temp = (pIn->output2.outputCbcr.imageWidth + (outPpw-1))/outPpw;     /* round up */
	hwCommand.out2CbcrImageWidthIn64bit =temp;          

	/* VFE_BUS_ENC_Cbcr_WR_BUFFER_CFG     */
	hwCommand.out2CbcrBurstLength = burstLength;                
	hwCommand.out2CbcrNumRows =pIn->output2.outputCbcr.outRowCount;                    
	temp = (pIn->output2.outputCbcr.outRowIncrement + (outPpw-1))/outPpw;     /* round up */
	hwCommand.out2CbcrRowIncrementIn64bit =temp;        

    /* AXI Output 1 Y Configuration*/
	/* VFE_BUS_VIEW_Y_WR_PING_ADDR  */
    hwCommand.out1YPingAddr =output1Path->yPath.addressBuffer[0];                        

	/* VFE_BUS_VIEW_Y_WR_PONG_ADDR  */
	hwCommand.out1YPongAddr =output1Path->yPath.addressBuffer[1];                      

	/* VFE_BUS_VIEW_Y_WR_IMAGE_SIZE     */
	hwCommand.out1YImageHeight = pIn->output1.outputY.imageHeight;  
	temp = (pIn->output1.outputY.imageWidth + (outPpw-1))/outPpw;     /* round up */
	hwCommand.out1YImageWidthin64bit=temp;               

	/* VFE_BUS_VIEW_Y_WR_BUFFER_CFG     */
	hwCommand.out1YBurstLength = burstLength;                   
	hwCommand.out1YNumRows=pIn->output1.outputY.outRowCount;                        

	temp = (pIn->output1.outputY.outRowIncrement + (outPpw-1))/outPpw;     /* round up */
	hwCommand.out1YRowIncrementIn64bit =temp;           

    /* AXI Output 1 Cbcr Configuration*/
	/* VFE_BUS_VIEW_Cbcr_WR_PING_ADDR  */
    hwCommand.out1CbcrPingAddr =output1Path->cbcrPath.addressBuffer[0];                   

	/* VFE_BUS_VIEW_Cbcr_WR_PONG_ADDR  */
	hwCommand.out1CbcrPongAddr =output1Path->cbcrPath.addressBuffer[1];                   

	/* VFE_BUS_VIEW_Cbcr_WR_IMAGE_SIZE     */
	hwCommand.out1CbcrImageHeight = pIn->output1.outputCbcr.imageHeight;                 
	temp = (pIn->output1.outputCbcr.imageWidth + (outPpw-1))/outPpw;  /* round up */
	hwCommand.out1CbcrImageWidthIn64bit =temp;          


	/* VFE_BUS_VIEW_Cbcr_WR_BUFFER_CFG     */
	hwCommand.out1CbcrBurstLength = burstLength;                
	hwCommand.out1CbcrNumRows =pIn->output1.outputCbcr.outRowCount;                     
	temp = (pIn->output1.outputCbcr.outRowIncrement + (outPpw-1))/outPpw;     /* round up */
	hwCommand.out1CbcrRowIncrementIn64bit =temp;         

	vfe_program_hw (VFE_BUS_ENC_Y_WR_PING_ADDR, (uint32_t*)&hwCommand, sizeof(hwCommand)); 

}


/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_axi_input_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_axi_input_config (vfe_cmd_axi_input_config  *pIn){

	struct VFE_AxiInputCmdType hwCommand;
    uint32_t xSizeWord,axiRdUnpackPattern;
	uint8  axiInputPpw;

	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

/**  Note to how to decide read unpack pattern
 *   Each 64 bit word can hold up to 8 color component
 *   bit allocation is:  ( 32-bit word)
 *   cycle    fetch           byte-sel[2:0]
 *     0      bit 3            bit 2:0
 *     1      bit 7            bit 6:4
 *     2      bit 11           bit 10:8
 *     3      bit 15           bit 14:12
 *     4      bit 19           bit 18:16
 *     5      bit 23           bit 22:20
 *     6      bit 27           bit 26:24
 *     7      bit 31           bit 30:28
 * 
 *   Fetch bit = 0 means no need to fetch new word
 *   Fetch bit = 1 means fetch new word
 * 
 *   byte selection is to determine which color component to
 *   select.  ( byte really means color component in this case.)
 * 
 *   Example for 8-bit color component ( 8bit bayer and YUV)
 *   cycle    fetch     byte-sel[2:0]
 *     0      0          0
 *     1      0          1
 *     2      0          2
 *     3      0          3
 *     4      0          4
 *     5      0          5
 *     6      0          6
 *     7      1          7
 * The value is 0xF6543210
 * 
 *   Example for 10-bit bayer
 *   cycle    fetch     byte-sel[2:0]
 *     0      0          0
 *     1      0          1
 *     2      0          2
 *     3      0          3
 *     4      0          4
 *     5      1          5
 *     6      0          0
 *     7      0          0
 * The value is 0xD43210
 * 
 * 
 * *   Example for 12-bit bayer
 *   cycle    fetch     byte-sel[2:0]
 *     0      0          0
 *     1      0          1
 *     2      0          2
 *     3      0          3
 *     4      1          4
 *     5      0          0
 *     6      0          0
 *     7      0          0
 * The value is 0xC3210

 */


    switch (pIn->pixelSize)
	{
	case VFE_RAW_PIXEL_DATA_SIZE_10BIT:
		axiInputPpw = 6;
        axiRdUnpackPattern = 0xD43210;

		break;
	case VFE_RAW_PIXEL_DATA_SIZE_12BIT:
		axiInputPpw = 5;
		axiRdUnpackPattern = 0xC3210;
		break;
	case VFE_RAW_PIXEL_DATA_SIZE_8BIT:
	default:
		axiInputPpw = 8;
		axiRdUnpackPattern = 0xF6543210;
		break;
	}

	// xSizeWord = ceiling of {((x_offset % axiInputPpw) + x_size) / axiInputPpw }
    xSizeWord = ((((pIn->xOffset % axiInputPpw)+pIn->xSize)+(axiInputPpw-1))/axiInputPpw)-1;   // program n means (n+1)

	hwCommand.stripeStartAddr0  = pIn->fragAddr[0];
	hwCommand.stripeStartAddr1  = pIn->fragAddr[1];
	hwCommand.stripeStartAddr2  = pIn->fragAddr[2];
	hwCommand.stripeStartAddr3  = pIn->fragAddr[3];

	hwCommand.ySize             = pIn->ySize;
	hwCommand.yOffsetDelta      = 0;   // limited by the driver.   (y Size alway == height of the stripe)

	hwCommand.xSizeWord         = xSizeWord;   // derived from xSize and xOffset

	hwCommand.burstLength       = 1;   // forced to burst length = 4
	hwCommand.NumOfRows         = pIn->numOfRows;    // each buffer needs to be equal height.
	hwCommand.RowIncrement      = (pIn->rowIncrement + (axiInputPpw-1))/axiInputPpw;   // round up. 

	hwCommand.mainUnpackHeight  = pIn->ySize;
	hwCommand.mainUnpackWidth   = pIn->xSize -1;     // xSize program n means n,  unpackWidth program n means (n+1)
	hwCommand.mainUnpackHbiSel  = (uint32_t)pIn->unpackHbi;
	hwCommand.mainUnpackPhase   = pIn->unpackPhase; 

	hwCommand.unpackPattern     = axiRdUnpackPattern;

	hwCommand.padLeft           = pIn->padRepeatCountLeft;                        
	hwCommand.padRight          = pIn->padRepeatCountRight;                      
	hwCommand.padTop            = pIn->padRepeatCountTop;                        
	hwCommand.padBottom         = pIn->padRepeatCountBottom;                     

	hwCommand.leftUnpackPattern0   = pIn->padLeftComponentSelectCycle0;                 
	hwCommand.leftUnpackPattern1   = pIn->padLeftComponentSelectCycle1;                 
	hwCommand.leftUnpackPattern2   = pIn->padLeftComponentSelectCycle2;                 
	hwCommand.leftUnpackPattern3   = pIn->padLeftComponentSelectCycle3;                 
	hwCommand.leftUnpackStop0      = pIn->padLeftStopCycle0;                 
	hwCommand.leftUnpackStop1      = pIn->padLeftStopCycle1;                 
	hwCommand.leftUnpackStop2      = pIn->padLeftStopCycle2;                 
	hwCommand.leftUnpackStop3      = pIn->padLeftStopCycle3;                 

	hwCommand.rightUnpackPattern0  = pIn->padRightComponentSelectCycle0;
	hwCommand.rightUnpackPattern1  = pIn->padRightComponentSelectCycle1;
	hwCommand.rightUnpackPattern2  = pIn->padRightComponentSelectCycle2;
	hwCommand.rightUnpackPattern3  = pIn->padRightComponentSelectCycle3;
	hwCommand.rightUnpackStop0     = pIn->padRightStopCycle0;
	hwCommand.rightUnpackStop1     = pIn->padRightStopCycle1;
	hwCommand.rightUnpackStop2     = pIn->padRightStopCycle2;
	hwCommand.rightUnpackStop3     = pIn->padRightStopCycle3;

	hwCommand.topUnapckPattern     = pIn->padTopLineCount;
	hwCommand.bottomUnapckPattern  = pIn->padBottomLineCount;
									 
	/*  program vfe_bus_cfg */                       
	vfe_program_hw (VFE_BUS_STRIPE_RD_ADDR_0, (uint32_t*)&hwCommand, sizeof(hwCommand));


}



/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_reg_bus_cfg
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_reg_bus_cfg(vfe_bus_cfg_data* pIn)
{
	struct VFE_AXIBusCfgType       hwBusCfgCommand;

	(void)memset((void*)&hwBusCfgCommand, 0, sizeof(hwBusCfgCommand));

	/*  vfe_bus_cfg takes derived data structure                */
	hwBusCfgCommand.stripeRdPathEn      = pIn->stripeRdPathEn;  
	hwBusCfgCommand.encYWrPathEn        = pIn->encYWrPathEn;       
	hwBusCfgCommand.encCbcrWrPathEn     = pIn->encCbcrWrPathEn;    
	hwBusCfgCommand.viewYWrPathEn       = pIn->viewYWrPathEn;      
	hwBusCfgCommand.viewCbcrWrPathEn    = pIn->viewCbcrWrPathEn;   
	hwBusCfgCommand.rawPixelDataSize    = (uint32_t)pIn->rawPixelDataSize;   
	hwBusCfgCommand.rawWritePathSelect  = (uint32_t)pIn->rawWritePathSelect; 
	/*  program vfe_bus_cfg */                        
	vfe_8x_program_register_value (VFE_BUS_CFG, *((uint32_t*)&hwBusCfgCommand));



}





/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_reg_camif_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_reg_camif_config(vfe_camif_cfg_data *pIn)
{

	struct VFE_CAMIFConfigType     hwCamifCfg;

	(void)memset((void*)&hwCamifCfg, 0, sizeof(hwCamifCfg));


	hwCamifCfg.VSyncEdge         =(uint32_t)pIn->camifCfgFromCmd.vSyncEdge;
	hwCamifCfg.HSyncEdge         =(uint32_t)pIn->camifCfgFromCmd.hSyncEdge;
	hwCamifCfg.syncMode          =(uint32_t)pIn->camifCfgFromCmd.syncMode;
	hwCamifCfg.vfeSubsampleEnable=pIn->camifCfgFromCmd.vfeSubSampleEnable;
	hwCamifCfg.busSubsampleEnable=pIn->camifCfgFromCmd.busSubSampleEnable;
	hwCamifCfg.camif2vfeEnable   =pIn->camif2OutputEnable;            
	hwCamifCfg.camif2busEnable   =pIn->camif2BusEnable;         
	hwCamifCfg.irqSubsampleEnable=pIn->camifCfgFromCmd.irqSubSampleEnable;
	hwCamifCfg.binningEnable     =pIn->camifCfgFromCmd.binningEnable;
	hwCamifCfg.misrEnable        =pIn->camifCfgFromCmd.misrEnable;

	/*  program camif_config */                        
	vfe_8x_program_register_value (CAMIF_CONFIG, *((uint32_t*)&hwCamifCfg));


}

/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_reg_bus_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_reg_bus_cmd(vfe_bus_cmd_data *pIn)
{

     struct VFE_BusCmdType  hwVfeBusCmd;
	 (void)memset((void*)&hwVfeBusCmd, 0, sizeof(hwVfeBusCmd));


	 hwVfeBusCmd.stripeReload        =pIn->stripeReload;   
	 hwVfeBusCmd.busPingpongReload      =pIn->busPingpongReload; 
	 hwVfeBusCmd.statsPingpongReload =pIn->statsPingpongReload;

	 vfe_8x_program_register_value (VFE_BUS_CMD, *((uint32_t*)&hwVfeBusCmd));
	 CDBG("bus command = 0x%x\n", (*((uint32_t*)&hwVfeBusCmd)));

	 // this is needed, as the control bits are pulse based.  Don't want to reload bus pingpong again.
	 pIn->busPingpongReload = 0;
	 pIn->statsPingpongReload = 0;
	 pIn->stripeReload = 0;
 }



/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_reg_module_cfg
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_reg_module_cfg(vfe_module_enable *pIn)
{

	struct VFE_ModuleEnableType    hwModuleEnable;

	(void)memset((void*)&hwModuleEnable, 0, sizeof(hwModuleEnable));


	hwModuleEnable.blackLevelCorrectionEnable =pIn->blackLevelCorrectionEnable;
	hwModuleEnable.lensRollOffEnable          =pIn->lensRollOffEnable;          
	hwModuleEnable.demuxEnable                =pIn->demuxEnable;                
	hwModuleEnable.chromaUpsampleEnable       =pIn->chromaUpsampleEnable;       
	hwModuleEnable.demosaicEnable             =pIn->demosaicEnable;             
	hwModuleEnable.statsEnable                =pIn->statsEnable;                
	hwModuleEnable.cropEnable                 =pIn->cropEnable;                 
	hwModuleEnable.mainScalerEnable           =pIn->mainScalerEnable;           
	hwModuleEnable.whiteBalanceEnable         =pIn->whiteBalanceEnable;         
	hwModuleEnable.colorCorrectionEnable      =pIn->colorCorrectionEnable;     
	hwModuleEnable.yHistEnable                =pIn->yHistEnable;                
	hwModuleEnable.skinToneEnable             =pIn->skinToneEnable;             
	hwModuleEnable.lumaAdaptationEnable       =pIn->lumaAdaptationEnable;       
	hwModuleEnable.rgbLUTEnable               =pIn->rgbLUTEnable;               
	hwModuleEnable.chromaEnhanEnable          =pIn->chromaEnhanEnable;          
	hwModuleEnable.asfEnable                  =pIn->asfEnable;                  
	hwModuleEnable.chromaSuppressionEnable    =pIn->chromaSuppressionEnable;    
	hwModuleEnable.chromaSubsampleEnable      =pIn->chromaSubsampleEnable;      
	hwModuleEnable.scaler2YEnable             =pIn->scaler2YEnable;             
	hwModuleEnable.scaler2CbcrEnable          =pIn->scaler2CbcrEnable;    


	//temp = *((uint32_t*)&hwModuleEnable);
	//CDBG("vfe module config register = %x\n", (*((uint32_t*)&hwModuleEnable)));




	//  enable vfe modules 
	vfe_8x_program_register_value (VFE_MODULE_CFG, *((uint32_t*)&hwModuleEnable));

 }


/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_stats_setting
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_stats_setting (vfe_cmd_stats_setting  *pIn){


    struct VFE_StatsFrameType  hwCommand1;
    struct VFE_BusStatsWrPriorityType hwCommand2;

	(void)memset((void*)&hwCommand1, 0, sizeof(hwCommand1));
	(void)memset((void*)&hwCommand2, 0, sizeof(hwCommand2));


    hwCommand1.lastPixel = pIn->frameHDimension;
	hwCommand1.lastLine = pIn->frameVDimension;

	vfe_program_hw (VFE_STATS_FRAME_SIZE, (uint32_t*)&hwCommand1, sizeof(hwCommand1));


	hwCommand2.afBusPriority    =pIn->afBusPriority;    
	hwCommand2.awbBusPriority   =pIn->awbBusPriority;   
	hwCommand2.histBusPriority  =pIn->histBusPriority;  
								 
	hwCommand2.afBusPriorityEn  =pIn->afBusPrioritySelection;  
	hwCommand2.awbBusPriorityEn =pIn->awbBusPrioritySelection; 
	hwCommand2.histBusPriorityEn=pIn->histBusPrioritySelection;

	vfe_program_hw (VFE_BUS_STATS_WR_PRIORITY, (uint32_t*)&hwCommand2, sizeof(hwCommand2));

    // Program the bus ping pong address for statistics modules.
    vfe_8x_program_register_value(VFE_BUS_STATS_AF_WR_PING_ADDR, pIn->afBuffer[0]);
	vfe_8x_program_register_value(VFE_BUS_STATS_AF_WR_PONG_ADDR, pIn->afBuffer[1]);

	vfe_8x_program_register_value(VFE_BUS_STATS_AWB_WR_PING_ADDR, pIn->awbBuffer[0]);
	vfe_8x_program_register_value(VFE_BUS_STATS_AWB_WR_PONG_ADDR, pIn->awbBuffer[1]);

	vfe_8x_program_register_value(VFE_BUS_STATS_HIST_WR_PING_ADDR, pIn->histBuffer[0]);
	vfe_8x_program_register_value(VFE_BUS_STATS_HIST_WR_PONG_ADDR, pIn->histBuffer[1]);

}





/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_reg_stats_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_stats_cmd(vfe_stats_cmd_data *pIn){

	struct VFE_StatsCmdType        hwStatsCmd;

	(void)memset((void*)&hwStatsCmd, 0, sizeof(hwStatsCmd));


	hwStatsCmd.autoFocusEnable        = pIn->autoFocusEnable;      
	hwStatsCmd.axwEnable              = pIn->axwEnable;
	hwStatsCmd.histEnable             = pIn->histEnable;
	hwStatsCmd.clearHistEnable        = pIn->clearHistEnable;       
	hwStatsCmd.histAutoClearEnable    = pIn->histAutoClearEnable;   
	hwStatsCmd.colorConversionEnable  = pIn->colorConversionEnable; 
	//  program stats command register. 
	vfe_8x_program_register_value (VFE_STATS_CMD, *((uint32_t*)&hwStatsCmd));


}




 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_stats_start_af
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_stats_start_af (vfe_cmd_stats_af_start * pIn){


    struct VFE_StatsAf_UpdateType hwCommand;
    struct VFE_StatsAf_CfgType    hwCommand2;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
    (void)memset((void*)&hwCommand2, 0, sizeof(hwCommand2));

    hwCommand.windowVOffset        =pIn->windowVOffset;
    hwCommand.windowHOffset        =pIn->windowHOffset;
    hwCommand.windowMode           =pIn->windowMode;
    hwCommand.windowHeight         =pIn->windowHeight;
    hwCommand.windowWidth          =pIn->windowWidth;

	vfe_program_hw(VFE_STATS_AF_CFG, (uint32_t*) &hwCommand, sizeof(hwCommand));

	hwCommand2.a00         =pIn->highPassCoef[0];
	hwCommand2.a04         =pIn->highPassCoef[1];

	hwCommand2.a20         =pIn->highPassCoef[2];
	hwCommand2.a21         =pIn->highPassCoef[3];
	hwCommand2.a22         =pIn->highPassCoef[4];
	hwCommand2.a23         =pIn->highPassCoef[5];
	hwCommand2.a24         =pIn->highPassCoef[6];

    hwCommand2.fvMax       =pIn->metricMax;
    hwCommand2.fvMetric    =pIn->metricSelection;

	hwCommand2.afHeader    =pIn->bufferHeader;

	hwCommand2.entry00     = pIn->gridForMultiWindows[0];   
	hwCommand2.entry01     = pIn->gridForMultiWindows[1];
	hwCommand2.entry02     = pIn->gridForMultiWindows[2];
	hwCommand2.entry03     = pIn->gridForMultiWindows[3];
	hwCommand2.entry10     = pIn->gridForMultiWindows[4];
	hwCommand2.entry11     = pIn->gridForMultiWindows[5];
	hwCommand2.entry12     = pIn->gridForMultiWindows[6];
	hwCommand2.entry13     = pIn->gridForMultiWindows[7];
	hwCommand2.entry20     = pIn->gridForMultiWindows[8];
	hwCommand2.entry21     = pIn->gridForMultiWindows[9];
	hwCommand2.entry22     = pIn->gridForMultiWindows[10];
	hwCommand2.entry23     = pIn->gridForMultiWindows[11];
	hwCommand2.entry30     = pIn->gridForMultiWindows[12];
	hwCommand2.entry31     = pIn->gridForMultiWindows[13];
	hwCommand2.entry32     = pIn->gridForMultiWindows[14];
	hwCommand2.entry33     = pIn->gridForMultiWindows[15];

    vfe_program_hw(VFE_STATS_AF_GRID_0, (uint32_t*) &hwCommand2, sizeof(hwCommand2));
}
 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_stats_start_wb_exp
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_stats_start_wb_exp (vfe_cmd_stats_wb_exp_start * pIn){

    struct VFE_StatsAwb_UpdateType       hwCommand;
    struct VFE_StatsAwbae_UpdateType     hwCommand2;

    struct VFE_StatsAxwHdr_CfgType       hwCommand3;


    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
	(void)memset((void*)&hwCommand2, 0, sizeof(hwCommand2));
	(void)memset((void*)&hwCommand3, 0, sizeof(hwCommand3));


    hwCommand.m1  = pIn->awbMCFG[0];
	hwCommand.m2  = pIn->awbMCFG[1];
	hwCommand.m3  = pIn->awbMCFG[2];
	hwCommand.m4  = pIn->awbMCFG[3];

	hwCommand.c1  = pIn->awbCCFG[0];
	hwCommand.c2  = pIn->awbCCFG[1];
	hwCommand.c3  = pIn->awbCCFG[2];
	hwCommand.c4  = pIn->awbCCFG[3];

    vfe_program_hw(VFE_STATS_AWB_MCFG, (uint32_t*) &hwCommand, sizeof(hwCommand));



    hwCommand2.aeRegionCfg          =pIn->wbExpRegions;
    hwCommand2.aeSubregionCfg       =pIn->wbExpSubRegion;
    hwCommand2.awbYMin              =pIn->awbYMin;
    hwCommand2.awbYMax              =pIn->awbYMax;
    vfe_program_hw(VFE_STATS_AWBAE_CFG, (uint32_t*) &hwCommand2, sizeof(hwCommand2));


	hwCommand3.axwHeader            =pIn->axwHeader;
	vfe_program_hw(VFE_STATS_AXW_HEADER, (uint32_t*) &hwCommand3, sizeof(hwCommand3));


}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_stats_update_af
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_8x_stats_update_af (vfe_cmd_stats_af_update * pIn){

    struct VFE_StatsAf_UpdateType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

    hwCommand.windowVOffset        =pIn->windowVOffset;
    hwCommand.windowHOffset        =pIn->windowHOffset;
    hwCommand.windowMode           =pIn->windowMode;
    hwCommand.windowHeight         =pIn->windowHeight;
    hwCommand.windowWidth          =pIn->windowWidth;

	vfe_program_hw(VFE_STATS_AF_CFG, (uint32_t*) &hwCommand, sizeof(hwCommand));

}




 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_stats_update_wb_exp
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_stats_update_wb_exp (vfe_cmd_stats_wb_exp_update * pIn){
	struct VFE_StatsAwb_UpdateType       hwCommand;
	struct VFE_StatsAwbae_UpdateType     hwCommand2;

	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
	(void)memset((void*)&hwCommand2, 0, sizeof(hwCommand2));

	hwCommand.m1  = pIn->awbMCFG[0];
	hwCommand.m2  = pIn->awbMCFG[1];
	hwCommand.m3  = pIn->awbMCFG[2];
	hwCommand.m4  = pIn->awbMCFG[3];

	hwCommand.c1  = pIn->awbCCFG[0];
	hwCommand.c2  = pIn->awbCCFG[1];
	hwCommand.c3  = pIn->awbCCFG[2];
	hwCommand.c4  = pIn->awbCCFG[3];
	vfe_program_hw(VFE_STATS_AWB_MCFG, (uint32_t*) &hwCommand, sizeof(hwCommand));

	hwCommand2.aeRegionCfg          =pIn->wbExpRegions;
	hwCommand2.aeSubregionCfg       =pIn->wbExpSubRegion;
	hwCommand2.awbYMin              =pIn->awbYMin;
	hwCommand2.awbYMax              =pIn->awbYMax;
	vfe_program_hw(VFE_STATS_AWBAE_CFG, (uint32_t*) &hwCommand2, sizeof(hwCommand2));

}
 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_stats_stop_af
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_stats_stop_af (void){
}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_stats_stop_wb_exp
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_stats_stop_wb_exp (void){
}

void vfe_8x_start(struct vfe_cmd_start *pIn)
{
	/* derived from other commands.  ( camif config, axi output config, etc )*/
	struct VFE_CFGType             hwVfeCfg;
	struct VFE_UpsampleConfigType  hwVfeChromaUpsampleCfg;

	(void)memset((void*)&hwVfeCfg, 0, sizeof(hwVfeCfg));
	(void)memset((void*)&hwVfeChromaUpsampleCfg, 0, sizeof(hwVfeChromaUpsampleCfg));

	/* directly from start command */
	hwVfeCfg.pixelPattern = (uint32_t)pIn->inputPixelPattern;
	hwVfeCfg.inputSource = (uint32_t)pIn->inputSource;
	vfe_8x_program_register_value(VFE_CFG, *((uint32_t*)&hwVfeCfg));

	hwVfeChromaUpsampleCfg.chromaCositingForYCbCrInputs =
		(uint32_t)pIn->yuvInputCositingMode;

	vfe_8x_program_register_value(VFE_CHROMA_UPSAMPLE_CFG,
		*((uint32_t*)&hwVfeChromaUpsampleCfg));

	vfe_8x_program_register_value(VFE_BUS_MISR_MAST_CFG_0, 0xd8);
}


void vfe_8x_main_scaler_config (struct vfe_cmd_main_scaler_config * pIn){

    struct VFE_Main_Scaler_ConfigCmdType  hwCommand;

	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
    hwCommand.hEnable              =pIn->hconfig.enable;        
	hwCommand.vEnable              =pIn->vconfig.enable;

    hwCommand.inWidth              =pIn->hconfig.inputSize;
	hwCommand.outWidth             =pIn->hconfig.outputSize;
	hwCommand.horizPhaseMult       =pIn->hconfig.phaseMultiplicationFactor;
	hwCommand.horizInterResolution =pIn->hconfig.interpolationResolution;

	hwCommand.horizMNInit          =pIn->MNInitH.MNCounterInit;
	hwCommand.horizPhaseInit       =pIn->MNInitH.phaseInit;


	hwCommand.inHeight             =pIn->vconfig.inputSize;                  
	hwCommand.outHeight            =pIn->vconfig.outputSize;                 
	hwCommand.vertPhaseMult        =pIn->vconfig.phaseMultiplicationFactor;  
	hwCommand.vertInterResolution  =pIn->vconfig.interpolationResolution; 

	hwCommand.vertMNInit           =pIn->MNInitV.MNCounterInit;                                           
	hwCommand.vertPhaseInit        =pIn->MNInitV.phaseInit;                  
								                
	vfe_program_hw (VFE_SCALE_CFG, (uint32_t *)&hwCommand, sizeof(hwCommand));

 }



void vfe_8x_scaler2y_config(struct vfe_cmd_scaler2y_config* pIn){

    struct VFE_Scaler2y_ConfigCmdType hwCommand;

	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

	hwCommand.hEnable               =pIn->hconfig.enable;        
	hwCommand.vEnable               =pIn->vconfig.enable;  

	hwCommand.inWidth               =pIn->hconfig.inputSize;  
	hwCommand.outWidth              =pIn->hconfig.outputSize;  
	hwCommand.horizPhaseMult        =pIn->hconfig.phaseMultiplicationFactor;  
	hwCommand.horizInterResolution  =pIn->hconfig.interpolationResolution;  

	hwCommand.inHeight              =pIn->vconfig.inputSize;                  
	hwCommand.outHeight             =pIn->vconfig.outputSize;                 
	hwCommand.vertPhaseMult         =pIn->vconfig.phaseMultiplicationFactor; 
	hwCommand.vertInterResolution   =pIn->vconfig.interpolationResolution;  

	vfe_program_hw (VFE_SCALE_Y_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));

}

void vfe_8x_scaler2cbcr_config (struct vfe_cmd_scaler2cbcr_config * pIn){

	 struct VFE_Scaler2CbCr_ConfigCmdType hwCommand;

	 (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
	 hwCommand.hEnable               =pIn->hconfig.enable;        
	 hwCommand.vEnable               =pIn->vconfig.enable;  

	 hwCommand.inWidth               =pIn->hconfig.inputSize;  
	 hwCommand.outWidth              =pIn->hconfig.outputSize;  
	 hwCommand.horizPhaseMult        =pIn->hconfig.phaseMultiplicationFactor;  
	 hwCommand.horizInterResolution  =pIn->hconfig.interpolationResolution;  

	 hwCommand.inHeight              =pIn->vconfig.inputSize;                  
	 hwCommand.outHeight             =pIn->vconfig.outputSize;                 
	 hwCommand.vertPhaseMult         =pIn->vconfig.phaseMultiplicationFactor; 
	 hwCommand.vertInterResolution   =pIn->vconfig.interpolationResolution;  

	 vfe_program_hw (VFE_SCALE_CBCR_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));
 }



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_chroma_enhan_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_chroma_enhan_config(vfe_cmd_chroma_enhan_config *pIn){

    struct VFE_ChromaEnhance_ConfigCmdType hwCommand;
    struct VFE_ColorConvert_ConfigCmdType hwCommand2;
    
    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
    (void)memset((void*)&hwCommand2, 0, sizeof(hwCommand2));

    hwCommand.ap                =(pIn->ap);
    hwCommand.am                =(pIn->am);
    hwCommand.bp                =(pIn->bp);
    hwCommand.bm                =(pIn->bm);
    hwCommand.cp                =(pIn->cp);
    hwCommand.cm                =(pIn->cm);
    hwCommand.dp                =(pIn->dp);
    hwCommand.dm                =(pIn->dm);
    hwCommand.kcb               =(pIn->kcb);
    hwCommand.kcr               =(pIn->kcr);

    hwCommand2.v0               =pIn->RGBtoYConversionV0;
    hwCommand2.v1               =pIn->RGBtoYConversionV1;
    hwCommand2.v2               =pIn->RGBtoYConversionV2;
    hwCommand2.ConvertOffset    =pIn->RGBtoYConversionOffset;

    vfe_program_hw (VFE_CHROMA_ENHAN_A, (uint32_t*)&hwCommand, sizeof(hwCommand));
    vfe_program_hw (VFE_COLOR_CONVERT_COEFF_0, (uint32_t*)&hwCommand2, sizeof(hwCommand2));
}
 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_chroma_enhan_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_chroma_enhan_update(vfe_cmd_chroma_enhan_update *pIn){

    struct VFE_ChromaEnhance_ConfigCmdType hwCommand;
    struct VFE_ColorConvert_ConfigCmdType hwCommand2;
    
    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
    (void)memset((void*)&hwCommand2, 0, sizeof(hwCommand2));

    hwCommand.ap                =pIn->ap;
    hwCommand.am                =pIn->am;
    hwCommand.bp                =pIn->bp;
    hwCommand.bm                =pIn->bm;
    hwCommand.cp                =pIn->cp;
    hwCommand.cm                =pIn->cm;
    hwCommand.dp                =pIn->dp;
    hwCommand.dm                =pIn->dm;
    hwCommand.kcb               =pIn->kcb;
    hwCommand.kcr               =pIn->kcr;

    hwCommand2.v0               =pIn->RGBtoYConversionV0;
    hwCommand2.v1               =pIn->RGBtoYConversionV1;
    hwCommand2.v2               =pIn->RGBtoYConversionV2;
    hwCommand2.ConvertOffset    =pIn->RGBtoYConversionOffset;

    vfe_program_hw (VFE_CHROMA_ENHAN_A, (uint32_t*)&hwCommand, sizeof(hwCommand));
    vfe_program_hw (VFE_COLOR_CONVERT_COEFF_0, (uint32_t*)&hwCommand2, sizeof(hwCommand2));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_chroma_subsample_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_chroma_subsample_config(vfe_cmd_chroma_subsample_config *pIn){


    struct VFE_ChromaSubsampleConfigCmdType hwCommand;

	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

	hwCommand.hCositedPhase         =pIn->hCositedPhase;       
	hwCommand.vCositedPhase         =pIn->vCositedPhase;      
	hwCommand.hCosited              =pIn->hCosited;           
	hwCommand.vCosited              =pIn->vCosited;           
	hwCommand.hsubSampleEnable      =pIn->hsubSampleEnable;   
	hwCommand.vsubSampleEnable      =pIn->vsubSampleEnable;   
	hwCommand.cropEnable            =pIn->cropEnable;         
													   
	hwCommand.cropWidthLastPixel    =pIn->cropWidthLastPixel; 
	hwCommand.cropWidthFirstPixel   =pIn->cropWidthFirstPixel;
													   
	hwCommand.cropHeightLastLine    =pIn->cropHeightLastLine; 
	hwCommand.cropHeightFirstLine   =pIn->cropHeightFirstLine;

	vfe_program_hw (VFE_CHROMA_SUBSAMPLE_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));


}



/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_roll_off_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_roll_off_config(vfe_cmd_roll_off_config  *pIn){

    struct VFE_Rolloff_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

    hwCommand.gridWidth         =pIn->gridWidth;
    hwCommand.gridHeight        =pIn->gridHeight;
    hwCommand.yDelta            =pIn->yDelta;
    hwCommand.gridX             =pIn->gridXIndex;
    hwCommand.gridY             =pIn->gridYIndex;
    hwCommand.pixelX            =pIn->gridPixelXIndex;
    hwCommand.pixelY            =pIn->gridPixelYIndex;
    hwCommand.yDeltaAccum       =pIn->yDeltaAccum;

    vfe_program_hw (VFE_ROLLOFF_CFG_0, (uint32_t*)&hwCommand, sizeof(hwCommand));

	vfe_8x_write_lens_roll_off_table(pIn);
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_chroma_suppression_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_chroma_suppression_config(vfe_cmd_chroma_suppression_config  *pIn){

    struct VFE_ChromaSuppress_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_ChromaSuppress_ConfigCmdType));

    hwCommand.m1        =pIn->m1;
    hwCommand.m3        =pIn->m3;
    hwCommand.n1        =pIn->n1;
    hwCommand.n3        =pIn->n3;
    hwCommand.mm1       =pIn->mm1;
    hwCommand.nn1       =pIn->nn1;

    vfe_program_hw (VFE_CHROMA_SUPPRESS_CFG_0, (uint32_t*)&hwCommand, sizeof(hwCommand));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_chroma_suppression_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_chroma_suppression_update(vfe_cmd_chroma_suppression_update  *pIn){

    struct VFE_ChromaSuppress_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_ChromaSuppress_ConfigCmdType));

    hwCommand.m1        =pIn->m1;
    hwCommand.m3        =pIn->m3;
    hwCommand.n1        =pIn->n1;
    hwCommand.n3        =pIn->n3;
    hwCommand.mm1       =pIn->mm1;
    hwCommand.nn1       =pIn->nn1;

    vfe_program_hw (VFE_CHROMA_SUPPRESS_CFG_0, (uint32_t*)&hwCommand, sizeof(hwCommand));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_white_balance_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_white_balance_config(vfe_cmd_white_balance_config *pIn){

    struct VFE_WhiteBalance_ConfigCmdType  hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_WhiteBalance_ConfigCmdType));

    hwCommand.ch0Gain       =pIn->ch0Gain;
    hwCommand.ch1Gain       =pIn->ch1Gain;
    hwCommand.ch2Gain       =pIn->ch2Gain;

    vfe_program_hw (VFE_WB_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_white_balance_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_white_balance_update(vfe_cmd_white_balance_update *pIn){

    struct VFE_WhiteBalance_ConfigCmdType  hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_WhiteBalance_ConfigCmdType));

    hwCommand.ch0Gain       =pIn->ch0Gain;
    hwCommand.ch1Gain       =pIn->ch1Gain;
    hwCommand.ch2Gain       =pIn->ch2Gain;

    vfe_program_hw (VFE_WB_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_asf_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_asf_config(vfe_cmd_asf_config *pIn){

    struct VFE_ASF_UpdateCmdType       hwCommand;    // not a typo, update structure is a subset of config
    struct VFE_ASFCrop_ConfigCmdType   hwCommand2;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
	(void)memset((void*)&hwCommand2, 0, sizeof(hwCommand2));

    hwCommand.smoothEnable              =pIn->smoothFilterEnabled;
    hwCommand.sharpMode                 =pIn->sharpMode;
    hwCommand.smoothCoeff0              =pIn->smoothCoefCenter;
    hwCommand.smoothCoeff1              =pIn->smoothCoefSurr;

    hwCommand.cropEnable                =pIn->cropEnable;

    hwCommand.sharpThresholdE1          =pIn->sharpThreshE1;
    hwCommand.sharpDegreeK1             =pIn->sharpK1;
    hwCommand.sharpDegreeK2             =pIn->sharpK2;
    hwCommand.normalizeFactor           =pIn->normalizeFactor;

    hwCommand.sharpThresholdE2          =pIn->sharpThreshE2;
    hwCommand.sharpThresholdE3          =pIn->sharpThreshE3;
    hwCommand.sharpThresholdE4          =pIn->sharpThreshE4;
	hwCommand.sharpThresholdE5          =pIn->sharpThreshE5;

    hwCommand.F1Coeff0                  =pIn->filter1Coefficients[0];
    hwCommand.F1Coeff1                  =pIn->filter1Coefficients[1];
    hwCommand.F1Coeff2                  =pIn->filter1Coefficients[2];
    hwCommand.F1Coeff3                  =pIn->filter1Coefficients[3];
    hwCommand.F1Coeff4                  =pIn->filter1Coefficients[4];
    hwCommand.F1Coeff5                  =pIn->filter1Coefficients[5];
    hwCommand.F1Coeff6                  =pIn->filter1Coefficients[6];
    hwCommand.F1Coeff7                  =pIn->filter1Coefficients[7];   
    hwCommand.F1Coeff8                  =pIn->filter1Coefficients[8];
                                                     
    hwCommand.F2Coeff0                  =pIn->filter2Coefficients[0];
    hwCommand.F2Coeff1                  =pIn->filter2Coefficients[1];
    hwCommand.F2Coeff2                  =pIn->filter2Coefficients[2];
    hwCommand.F2Coeff3                  =pIn->filter2Coefficients[3];
    hwCommand.F2Coeff4                  =pIn->filter2Coefficients[4];
    hwCommand.F2Coeff5                  =pIn->filter2Coefficients[5];
    hwCommand.F2Coeff6                  =pIn->filter2Coefficients[6];
    hwCommand.F2Coeff7                  =pIn->filter2Coefficients[7];
    hwCommand.F2Coeff8                  =pIn->filter2Coefficients[8];

    vfe_program_hw  (VFE_ASF_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));

	hwCommand2.firstLine = pIn->cropFirstLine;
    hwCommand2.lastLine  = pIn->cropLastLine;
	hwCommand2.firstPixel = pIn->cropFirstPixel;
	hwCommand2.lastPixel  = pIn->cropLastPixel;

	vfe_program_hw  (VFE_ASF_CROP_WIDTH_CFG, (uint32_t*)&hwCommand2, sizeof(hwCommand2));


}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_asf_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_asf_update(vfe_cmd_asf_update *pIn){

    struct VFE_ASF_UpdateCmdType       hwCommand;    // not a typo, update structure is a subset of config

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

    hwCommand.smoothEnable              =pIn->smoothFilterEnabled;
    hwCommand.sharpMode                 =pIn->sharpMode;
    hwCommand.smoothCoeff1              =pIn->smoothCoefCenter;
    hwCommand.smoothCoeff0              =pIn->smoothCoefSurr;

    hwCommand.cropEnable                =pIn->cropEnable;

    hwCommand.sharpThresholdE1          =pIn->sharpThreshE1;
    hwCommand.sharpDegreeK1             =pIn->sharpK1;
    hwCommand.sharpDegreeK2             =pIn->sharpK2;
    hwCommand.normalizeFactor           =pIn->normalizeFactor;

    hwCommand.sharpThresholdE2          =pIn->sharpThreshE2;
    hwCommand.sharpThresholdE3          =pIn->sharpThreshE3;
    hwCommand.sharpThresholdE4          =pIn->sharpThreshE4;
	hwCommand.sharpThresholdE5          =pIn->sharpThreshE5;

    hwCommand.F1Coeff0                  =pIn->filter1Coefficients[0];
    hwCommand.F1Coeff1                  =pIn->filter1Coefficients[1];
    hwCommand.F1Coeff2                  =pIn->filter1Coefficients[2];
    hwCommand.F1Coeff3                  =pIn->filter1Coefficients[3];
    hwCommand.F1Coeff4                  =pIn->filter1Coefficients[4];
    hwCommand.F1Coeff5                  =pIn->filter1Coefficients[5];
    hwCommand.F1Coeff6                  =pIn->filter1Coefficients[6];
    hwCommand.F1Coeff7                  =pIn->filter1Coefficients[7];   
    hwCommand.F1Coeff8                  =pIn->filter1Coefficients[8];
                                                     
    hwCommand.F2Coeff0                  =pIn->filter2Coefficients[0];
    hwCommand.F2Coeff1                  =pIn->filter2Coefficients[1];
    hwCommand.F2Coeff2                  =pIn->filter2Coefficients[2];
    hwCommand.F2Coeff3                  =pIn->filter2Coefficients[3];
    hwCommand.F2Coeff4                  =pIn->filter2Coefficients[4];
    hwCommand.F2Coeff5                  =pIn->filter2Coefficients[5];
    hwCommand.F2Coeff6                  =pIn->filter2Coefficients[6];
    hwCommand.F2Coeff7                  =pIn->filter2Coefficients[7];
    hwCommand.F2Coeff8                  =pIn->filter2Coefficients[8];

    vfe_program_hw  (VFE_ASF_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_black_level_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_black_level_config(vfe_cmd_black_level_config *pIn){

    VFE_BlackLevel_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_BlackLevel_ConfigCmdType));

    hwCommand.evenEvenAdjustment    =pIn->evenEvenAdjustment;
    hwCommand.evenOddAdjustment     =pIn->evenOddAdjustment;
    hwCommand.oddEvenAdjustment     =pIn->oddEvenAdjustment;
    hwCommand.oddOddAdjustment      =pIn->oddOddAdjustment;

    vfe_program_hw (VFE_BLACK_EVEN_EVEN_VALUE, (uint32_t*)&hwCommand, sizeof(hwCommand));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_black_level_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_black_level_update (vfe_cmd_black_level_update  *pIn){

    VFE_BlackLevel_ConfigCmdType    hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_BlackLevel_ConfigCmdType));

    hwCommand.evenEvenAdjustment    =pIn->evenEvenAdjustment;
    hwCommand.evenOddAdjustment     =pIn->evenOddAdjustment;
    hwCommand.oddEvenAdjustment     =pIn->oddEvenAdjustment;
    hwCommand.oddOddAdjustment      =pIn->oddOddAdjustment;

    vfe_program_hw (VFE_BLACK_EVEN_EVEN_VALUE, (uint32_t*)&hwCommand, sizeof(hwCommand));
}

void vfe_8x_demux_channel_gain_config(vfe_cmd_demux_channel_gain_config *pIn){

    struct VFE_Demux_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

    hwCommand.ch0EvenGain       =pIn->ch0EvenGain;
    hwCommand.ch0OddGain        =pIn->ch0OddGain;
    hwCommand.ch1Gain           =pIn->ch1Gain;
    hwCommand.ch2Gain           =pIn->ch2Gain;

    vfe_program_hw (VFE_DEMUX_GAIN_0, (uint32_t*)&hwCommand, sizeof(hwCommand));
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_demux_channel_gain_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_demux_channel_gain_update(vfe_cmd_demux_channel_gain_update  *pIn){

    VFE_Demux_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_Demux_ConfigCmdType));

    hwCommand.ch0EvenGain       =pIn->ch0EvenGain;
    hwCommand.ch0OddGain        =pIn->ch0OddGain;
    hwCommand.ch1Gain           =pIn->ch1Gain;
    hwCommand.ch2Gain           =pIn->ch2Gain;

    vfe_program_hw (VFE_DEMUX_GAIN_0, (uint32_t*)&hwCommand, sizeof(hwCommand));
}

/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_demosaic_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_demosaic_config(vfe_cmd_demosaic_config *pIn){

    VFE_Demosaic_CfgCmdType hwCommand;
	VFE_DemosaicBPC_CmdType hwCommandBpc;
	VFE_DemosaicABF_CmdType hwCommandAbf;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
    (void)memset((void*)&hwCommandBpc, 0, sizeof(hwCommandBpc));
    (void)memset((void*)&hwCommandAbf, 0, sizeof(hwCommandAbf));

    hwCommand.abfEnable             =pIn->abfConfig.enable;
    hwCommand.badPixelCorrEnable    =pIn->bpcConfig.enable;

    hwCommand.forceAbfOn            =pIn->abfConfig.forceOn;
    hwCommand.abfShift              =pIn->abfConfig.shift;

    hwCommand.fminThreshold         =pIn->bpcConfig.fminThreshold;
    hwCommand.fmaxThreshold         =pIn->bpcConfig.fmaxThreshold;

    hwCommand.slopeShift            =pIn->slopeShift;

	vfe_program_hw (VFE_DEMOSAIC_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));


	hwCommandAbf.lpThreshold           =pIn->abfConfig.lpThreshold;
    hwCommandAbf.ratio                 =pIn->abfConfig.ratio;
    hwCommandAbf.minValue              =pIn->abfConfig.min;
    hwCommandAbf.maxValue              =pIn->abfConfig.max;
	vfe_program_hw (VFE_DEMOSAIC_ABF_CFG_0, (uint32_t*)&hwCommandAbf, sizeof(hwCommandAbf));


    hwCommandBpc.blueDiffThreshold     =pIn->bpcConfig.blueDiffThreshold;
    hwCommandBpc.redDiffThreshold      =pIn->bpcConfig.redDiffThreshold;
    hwCommandBpc.greenDiffThreshold    =pIn->bpcConfig.greenDiffThreshold;


	vfe_program_hw (VFE_DEMOSAIC_BPC_CFG_0, (uint32_t*)&hwCommandBpc, sizeof(hwCommandBpc));
}


/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_demosaic_bpc_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_demosaic_bpc_update (vfe_cmd_demosaic_bpc_update  *pIn){

    VFE_Demosaic_CfgCmdType hwCommand;
	VFE_DemosaicBPC_CmdType hwCommandBpc;
    uint32_t temp;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

    // must read the current demosaic_cfg content first.
    temp = vfe_8x_read_reg_value(VFE_DEMOSAIC_CFG);

    hwCommand = *((VFE_Demosaic_CfgCmdType*)(&temp));

    // then update.
    hwCommand.badPixelCorrEnable    =pIn->bpcUpdate.enable;
    hwCommand.fminThreshold         =pIn->bpcUpdate.fminThreshold;
    hwCommand.fmaxThreshold         =pIn->bpcUpdate.fmaxThreshold;

	vfe_program_hw (VFE_DEMOSAIC_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));


    hwCommandBpc.blueDiffThreshold     =pIn->bpcUpdate.blueDiffThreshold;
    hwCommandBpc.redDiffThreshold      =pIn->bpcUpdate.redDiffThreshold;
    hwCommandBpc.greenDiffThreshold    =pIn->bpcUpdate.greenDiffThreshold;


	vfe_program_hw (VFE_DEMOSAIC_BPC_CFG_0, (uint32_t*)&hwCommandBpc, sizeof(hwCommandBpc));
}

/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_demosaic_abf_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_demosaic_abf_update (vfe_cmd_demosaic_abf_update  *pIn){

    VFE_Demosaic_CfgCmdType hwCommand;
	VFE_DemosaicABF_CmdType hwCommandAbf;
    uint32_t temp;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));
    //memset((void*)&hwCommandAbf, 0, sizeof(hwCommandAbf));

	// must read the current demosaic_cfg content first.
	temp = vfe_8x_read_reg_value(VFE_DEMOSAIC_CFG);
    hwCommand = *((VFE_Demosaic_CfgCmdType*)(&temp));

	// then update.
    hwCommand.abfEnable             =pIn->abfUpdate.enable;
    hwCommand.forceAbfOn            =pIn->abfUpdate.forceOn;
    hwCommand.abfShift              =pIn->abfUpdate.shift;


	vfe_program_hw (VFE_DEMOSAIC_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));


	hwCommandAbf.lpThreshold           =pIn->abfUpdate.lpThreshold;
    hwCommandAbf.ratio                 =pIn->abfUpdate.ratio;
    hwCommandAbf.minValue              =pIn->abfUpdate.min;
    hwCommandAbf.maxValue              =pIn->abfUpdate.max;
	vfe_program_hw (VFE_DEMOSAIC_ABF_CFG_0, (uint32_t*)&hwCommandAbf, sizeof(hwCommandAbf));

}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_color_correction_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_color_correction_config (vfe_cmd_color_correction_config  *pIn){
    VFE_ColorCorrection_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_ColorCorrection_ConfigCmdType));

    hwCommand.c0            =pIn->C0;
    hwCommand.c1            =pIn->C1;
    hwCommand.c2            =pIn->C2;
    hwCommand.c3            =pIn->C3;
    hwCommand.c4            =pIn->C4;
    hwCommand.c5            =pIn->C5;
    hwCommand.c6            =pIn->C6;
    hwCommand.c7            =pIn->C7;
    hwCommand.c8            =pIn->C8;

    hwCommand.k0            =pIn->K0;
    hwCommand.k1            =pIn->K1;
    hwCommand.k2            =pIn->K2;

    hwCommand.coefQFactor   =(uint32_t)pIn->coefQFactor;

    vfe_program_hw (VFE_COLOR_CORRECT_COEFF_0, (uint32_t*)&hwCommand, VFE_ColorCorrection_ConfigCmdType_LEN);
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_color_correction_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_color_correction_update (vfe_cmd_color_correction_update  *pIn){

    VFE_ColorCorrection_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_ColorCorrection_ConfigCmdType));

    hwCommand.c0            =pIn->C0;
    hwCommand.c1            =pIn->C1;
    hwCommand.c2            =pIn->C2;
    hwCommand.c3            =pIn->C3;
    hwCommand.c4            =pIn->C4;
    hwCommand.c5            =pIn->C5;
    hwCommand.c6            =pIn->C6;
    hwCommand.c7            =pIn->C7;
    hwCommand.c8            =pIn->C8;

    hwCommand.k0            =pIn->K0;
    hwCommand.k1            =pIn->K1;
    hwCommand.k2            =pIn->K2;

    hwCommand.coefQFactor   =(uint32_t)pIn->coefQFactor;

    vfe_program_hw (VFE_COLOR_CORRECT_COEFF_0, (uint32_t*)&hwCommand, VFE_ColorCorrection_ConfigCmdType_LEN);
}
/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_get_hw_version
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_camif_frame_update(vfe_cmds_camif_frame* pIn){


     VFE_UpdateCamifFrameType hwCommand;

     (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

	 hwCommand.pixelsPerLine = pIn->pixelsPerLine;
	 hwCommand.linesPerFrame = pIn->linesPerFrame;

	 vfe_program_hw(CAMIF_FRAME_CONFIG, (uint32_t*)&hwCommand, sizeof(hwCommand));

}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_output_clamp_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_output_clamp_config(vfe_cmd_output_clamp_config* pIn){

	struct VFE_OutputClampConfigCmdType    hwCommand;
	(void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

	hwCommand.yChanMax  =pIn->maxCh0;
	hwCommand.cbChanMax =pIn->maxCh1;
	hwCommand.crChanMax =pIn->maxCh2;

	hwCommand.yChanMin  =pIn->minCh0;
	hwCommand.cbChanMin =pIn->minCh1;
	hwCommand.crChanMin =pIn->minCh2;

	vfe_program_hw(VFE_CLAMP_MAX_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));
}





/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_frame_skip_config
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_frame_skip_config(vfe_cmd_frame_skip_config* pIn){

     struct VFE_FRAME_SKIP_ConfigCmdType    hwCommand;
	 (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

	 hwCommand.output2YPeriod       =pIn->output2Period; 
	 hwCommand.output2CbCrPeriod    =pIn->output2Period;

	 hwCommand.output2YPattern      =pIn->output2Pattern;
	 hwCommand.output2CbCrPattern   =pIn->output2Pattern;

	 hwCommand.output1YPeriod       =pIn->output1Period;
	 hwCommand.output1CbCrPeriod    =pIn->output1Period;

	 hwCommand.output1YPattern      =pIn->output1Pattern;
	 hwCommand.output1CbCrPattern   =pIn->output1Pattern;

	 vfe_program_hw(VFE_FRAMEDROP_ENC_Y_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));
}



/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_frame_skip_update
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_frame_skip_update(vfe_cmd_frame_skip_update *pIn){

     struct VFE_FRAME_SKIP_UpdateCmdType hwCommand;

	 hwCommand.yPattern= pIn->output1Pattern;    
	 hwCommand.cbcrPattern = pIn->output1Pattern;

	 vfe_program_hw(VFE_FRAMEDROP_VIEW_Y_PATTERN, (uint32_t*)&hwCommand, sizeof(hwCommand));

	 hwCommand.yPattern= pIn->output2Pattern;    
	 hwCommand.cbcrPattern = pIn->output2Pattern;
	 vfe_program_hw(VFE_FRAMEDROP_ENC_Y_PATTERN, (uint32_t*)&hwCommand, sizeof(hwCommand));

}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_test_gen_start
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_test_gen_start(vfe_cmd_test_gen_start *pIn){

    struct VFE_TestGen_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_TestGen_ConfigCmdType));

    hwCommand.numFrame              =pIn->numFrame;
    hwCommand.pixelDataSelect       =pIn->pixelDataSelect;
    hwCommand.systematicDataSelect  =pIn->systematicDataSelect;
    hwCommand.pixelDataSize         =(uint32_t)pIn->pixelDataSize;
    hwCommand.hsyncEdge             =(uint32_t)pIn->hsyncEdge;
    hwCommand.vsyncEdge             =(uint32_t)pIn->vsyncEdge;

    hwCommand.imageWidth            =pIn->imageWidth;
    hwCommand.imageHeight           =pIn->imageHeight;

    hwCommand.sofOffset             =pIn->startOfFrameOffset;
    hwCommand.eofNOffset            =pIn->endOfFrameNOffset;
    hwCommand.solOffset             =pIn->startOfLineOffset;
    hwCommand.eolNOffset            =pIn->endOfLineNOffset;

    hwCommand.hBlankInterval        =pIn->hbi;
    hwCommand.vBlankInterval        =pIn->vbl;
    hwCommand.vBlankIntervalEnable  =pIn->vblEnable;

    hwCommand.sofDummy              =pIn->startOfFrameDummyLine;
    hwCommand.eofDummy              =pIn->endOfFrameDummyLine;

    hwCommand.unicolorBarSelect     =pIn->unicolorBarSelect;
    hwCommand.unicolorBarEnable     =pIn->unicolorBarEnable;
    hwCommand.splitEnable           =pIn->colorBarsSplitEnable;
    hwCommand.pixelPattern          =(uint32_t)pIn->colorBarsPixelPattern;
    hwCommand.rotatePeriod          =pIn->colorBarsRotatePeriod;

    hwCommand.randomSeed            =pIn->testGenRandomSeed;

    vfe_program_hw(VFE_HW_TESTGEN_CFG, (uint32_t*) &hwCommand, sizeof(hwCommand));
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_pm_start
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_pm_start(vfe_cmd_bus_pm_start *pIn){

    struct VFE_Bus_Pm_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(VFE_Bus_Pm_ConfigCmdType));

    hwCommand.output2YWrPmEnable        =pIn->output2YWrPmEnable;
    hwCommand.output2CbcrWrPmEnable     =pIn->output2CbcrWrPmEnable;
    hwCommand.output1YWrPmEnable        =pIn->output1YWrPmEnable;
    hwCommand.output1CbcrWrPmEnable     =pIn->output1CbcrWrPmEnable;

    vfe_program_hw(VFE_BUS_PM_CFG, (uint32_t*)&hwCommand, sizeof(hwCommand));
}

/* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_set_default_register_values
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_8x_set_default_register_values (void){

	vfe_8x_program_register_value(VFE_DEMUX_GAIN_0, 0x800080);
	vfe_8x_program_register_value(VFE_DEMUX_GAIN_1, 0x800080);

	// enable clocks for the corresponding modules  (desired)
	//vfe_8x_program_register_value (VFE_CGC_OVERRIDE, *((uint32_t*)&hwModuleEnable));

	// enable clocks for all modules for now.  Since VFE_CGC_OVERRIDE is not double buffered.  Hardware will 
    // automatically gate off the clock based on the value from module_cfg.  In update commands, modules can be turned 
    // on and off,  so enable all bits for CGC clock will allow us to do updates.
	// 
	vfe_8x_program_register_value (VFE_CGC_OVERRIDE, 0xFFFFF);


    // default frame drop period and pattern
	vfe_8x_program_register_value (VFE_FRAMEDROP_ENC_Y_CFG        , 0x1f);
	vfe_8x_program_register_value (VFE_FRAMEDROP_ENC_CBCR_CFG     , 0x1f);
	vfe_8x_program_register_value (VFE_FRAMEDROP_ENC_Y_PATTERN    , 0xFFFFFFFF);
	vfe_8x_program_register_value (VFE_FRAMEDROP_ENC_CBCR_PATTERN , 0xFFFFFFFF);
	vfe_8x_program_register_value (VFE_FRAMEDROP_VIEW_Y_CFG       , 0x1f);
	vfe_8x_program_register_value (VFE_FRAMEDROP_VIEW_CBCR_CFG    , 0x1f);
	vfe_8x_program_register_value (VFE_FRAMEDROP_VIEW_Y_PATTERN   , 0xFFFFFFFF);
	vfe_8x_program_register_value (VFE_FRAMEDROP_VIEW_CBCR_PATTERN, 0xFFFFFFFF);
	vfe_8x_program_register_value (VFE_CLAMP_MIN_CFG, 0);
	vfe_8x_program_register_value (VFE_CLAMP_MAX_CFG, 0xFFFFFF);
}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_config_demux
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_config_demux(uint32_t period, uint32_t even, uint32_t odd){


	vfe_8x_program_register_value(VFE_DEMUX_CFG, period);
	vfe_8x_program_register_value(VFE_DEMUX_EVEN_CFG, even);
	vfe_8x_program_register_value(VFE_DEMUX_ODD_CFG , odd);

}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_bus_pm_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_bus_pm_cmd(uint32_t value)
{
    vfe_8x_program_register_value(VFE_BUS_PM_CMD, value);
}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_bus_rd_irq_en
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_bus_rd_irq_en(uint32_t value)
{
    vfe_8x_program_register_value(VFE_BUS_PINGPONG_IRQ_EN, value);

}




 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_camif_go
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_camif_go(void)
{
    vfe_8x_program_register_value(CAMIF_COMMAND, CAMIF_COMMAND_START);
}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_camif_stop_immediately
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_camif_stop_immediately(void)
{
    vfe_8x_program_register_value(CAMIF_COMMAND, CAMIF_COMMAND_STOP_IMMEDIATELY);
	vfe_8x_program_register_value(VFE_CGC_OVERRIDE, 0); // turn off CGC.

}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_camif_stop_frame_boundary
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_camif_stop_frame_boundary(void)
{
    vfe_8x_program_register_value(CAMIF_COMMAND, CAMIF_COMMAND_STOP_AT_FRAME_BOUNDARY);
}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_program_reg_update_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_program_reg_update_cmd(uint32_t value)
{
    vfe_8x_program_register_value(VFE_REG_UPDATE_CMD, value);
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_bus_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_bus_cmd(uint32_t value)
{
    vfe_8x_program_register_value(VFE_BUS_CMD, value);
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_global_reset_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_global_reset_cmd(uint32_t value)
{
    vfe_8x_program_register_value(VFE_GLOBAL_RESET_CMD, value);
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_axi_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_axi_cmd(uint32_t value)
{
    vfe_8x_program_register_value(VFE_AXI_CMD, value);
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_program_hw_testgen_cmd
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_program_hw_testgen_cmd(uint32_t value)
{
    vfe_8x_program_register_value(VFE_HW_TESTGEN_CMD, value);
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_irq_clear
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_program_irq_clear(uint32_t value)
{
    vfe_8x_program_register_value(VFE_IRQ_CLEAR, value);
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_irq_composite_mask
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_program_irq_composite_mask(uint32_t value)
{
    vfe_8x_program_register_value(VFE_IRQ_COMPOSITE_MASK, value);
}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_program_irq_mask
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_program_irq_mask(uint32_t value)
{
    vfe_8x_program_register_value(VFE_IRQ_MASK, value);

}




void vfe_8x_program_chroma_upsample_cfg(uint32_t value){
    vfe_8x_program_register_value(VFE_CHROMA_UPSAMPLE_CFG, value);
}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_read_axi_status
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
uint32_t vfe_8x_read_axi_status(){

	return vfe_8x_read_reg_value(VFE_AXI_STATUS);
}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_read_pm_status_in_raw_capture
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
uint32_t vfe_read_pm_status_in_raw_capture()
{
	return vfe_8x_read_reg_value(VFE_BUS_ENC_CBCR_WR_PM_STATS_1);
}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_set_bus_pingpong_address
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_8x_set_bus_pingpong_address(vfe_output_path_combo* vPath, vfe_output_path_combo* ePath){

    vPath->yPath.hwRegPingAddress = VFE_BUS_VIEW_Y_WR_PING_ADDR;
	vPath->yPath.hwRegPongAddress = VFE_BUS_VIEW_Y_WR_PONG_ADDR;

    vPath->cbcrPath.hwRegPingAddress = VFE_BUS_VIEW_CBCR_WR_PING_ADDR;
	vPath->cbcrPath.hwRegPongAddress = VFE_BUS_VIEW_CBCR_WR_PONG_ADDR;


    ePath->yPath.hwRegPingAddress = VFE_BUS_ENC_Y_WR_PING_ADDR;
	ePath->yPath.hwRegPongAddress = VFE_BUS_ENC_Y_WR_PONG_ADDR;

	ePath->cbcrPath.hwRegPingAddress = VFE_BUS_ENC_CBCR_WR_PING_ADDR;
	ePath->cbcrPath.hwRegPongAddress = VFE_BUS_ENC_CBCR_WR_PONG_ADDR;

}
 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_set_stats_pingpong_address
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_8x_set_stats_pingpong_address(vfe_stats_control* afControl, vfe_stats_control* awbControl){

    afControl->hwRegPingAddress = VFE_BUS_STATS_AF_WR_PING_ADDR;
	afControl->hwRegPongAddress = VFE_BUS_STATS_AF_WR_PONG_ADDR;

    awbControl->hwRegPingAddress = VFE_BUS_STATS_AWB_WR_PING_ADDR;
	awbControl->hwRegPongAddress = VFE_BUS_STATS_AWB_WR_PONG_ADDR;

}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_read_camif_status
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
uint32_t vfe_8x_read_camif_status(void){


    return vfe_8x_read_reg_value(CAMIF_STATUS);

}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_lut_bank_sel
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_program_lut_bank_sel(vfe_gamma_lut_sel* pIn)
{

    VFE_GammaLutSelect_ConfigCmdType hwCommand;

    (void)memset((void*)&hwCommand, 0, sizeof(hwCommand));

    hwCommand.ch0BankSelect = pIn->ch0BankSelect;
    hwCommand.ch1BankSelect = pIn->ch1BankSelect;
    hwCommand.ch2BankSelect = pIn->ch2BankSelect;
	CDBG("VFE gamma lut bank selection is 0x%x\n", *((uint32_t*)&hwCommand));

	vfe_program_hw(VFE_LUT_BANK_SEL, (uint32_t*) &hwCommand, sizeof(hwCommand));
}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_la_cfg
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_la_cfg(uint32_t bank){

	vfe_8x_program_register_value(VFE_LA_CFG, bank);   // can only be bank 0 or bank 1 for now.
	CDBG("VFE Luma adaptation bank selection is 0x%x\n", *(uint32_t*)&bank);

}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_write_gamma_table
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_write_gamma_table(uint8 channel, boolean bank, int16* pTable){

	 uint16 i;
	 //uint32_t readOutData;


	 VFE_DMI_RAM_SEL dmiRamSel=NO_MEM_SELECTED;   // this is an enum.

	 //CDBG("channel is 0x%x\n", channel);
	 //CDBG("bank is 0x%x\n", bank);


	  // determine the SRAM selection
	 switch(channel)
	 {
	 case 0:
		 if (bank == 0) {

			 dmiRamSel = RGBLUT_RAM_CH0_BANK0;
		 }
		 else {
			 dmiRamSel = RGBLUT_RAM_CH0_BANK1;
		 }
		 break;

	 case 1:
		 if (bank == 0) {

			 dmiRamSel = RGBLUT_RAM_CH1_BANK0;
		 }
		 else {
			 dmiRamSel = RGBLUT_RAM_CH1_BANK1;
		 }

		 break;

	 case 2:
		 if (bank == 0) {

			 dmiRamSel = RGBLUT_RAM_CH2_BANK0;
		 }
		 else {
			 dmiRamSel = RGBLUT_RAM_CH2_BANK1;
		 }

		 break;

	 default:   // do nothing in default case? 
		 break;
	 }

	 //CDBG("dmiRamSel is 0x%x\n", dmiRamSel);


	 vfe_8x_program_dmi_cfg(dmiRamSel);




	 for(i=0; i<VFE_GAMMA_TABLE_LENGTH;i++){

		 vfe_8x_program_register_value(VFE_DMI_DATA_LO, (uint32_t)(*pTable));
		 //CDBG("gamma table value is 0x%x\n", (uint32_t)(*pTable));

		 pTable++;
	 }



	#if 0
	// added to test the read out part.
	vfe_8x_program_register_value(VFE_DMI_ADDR, 0);  

	for (i=0;i<6;i++)

	{
    	readOutData = vfe_8x_read_reg_value(VFE_DMI_DATA_LO);

		CDBG("gamma table value is 0x%x, channel is %d\n", readOutData, channel);

	}
    #endif




	 // After DMI transfer, need to set the DMI_CFG to unselect any SRAM
	 vfe_8x_program_register_value(VFE_DMI_CFG, VFE_DMI_CFG_DEFAULT);   // unselect the SRAM Bank.



}




 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_write_la_table
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_8x_write_la_table(uint32_t bank, int16* pTable){

	uint16 i;
	//uint32_t readOutData;

	VFE_DMI_RAM_SEL dmiRamSel;   // this is an enum.

	 // determine the SRAM selection
	if (bank == 0) {

		dmiRamSel = LUMA_ADAPT_LUT_RAM_BANK0;
	}
	else {
		dmiRamSel = LUMA_ADAPT_LUT_RAM_BANK1;
	}

	// configure the DMI_CFG to select right sram 
	vfe_8x_program_dmi_cfg(dmiRamSel);

	for(i=0; i<VFE_LA_TABLE_LENGTH;i++){

		vfe_8x_program_register_value(VFE_DMI_DATA_LO, (uint32_t)(*pTable));
		pTable++;
	}


	#if 0
	// added to test the read out part.
	vfe_8x_program_register_value(VFE_DMI_ADDR, 0);  
	for (i=0;i<6;i++)
	{
    	readOutData = vfe_8x_read_reg_value(VFE_DMI_DATA_LO);
		CDBG("la table value is 0x%x\n", readOutData);
	}
    #endif

	// After DMI transfer, to make it safe, need to set the DMI_CFG to unselect any SRAM
	vfe_8x_program_register_value(VFE_DMI_CFG, VFE_DMI_CFG_DEFAULT);   // unselect the SRAM Bank.

}



void vfe_8x_write_lens_roll_off_table(vfe_cmd_roll_off_config *pIn){

	uint16 i;
    uint32_t data;

	//uint32_t readOutData;

	uint16* pInitGr = pIn->initTableGr;
	uint16* pInitGb = pIn->initTableGb;
	uint16* pInitB =  pIn->initTableB;
	uint16* pInitR =  pIn->initTableR;

	int16* pDeltaGr = pIn->deltaTableGr;
	int16* pDeltaGb = pIn->deltaTableGb;
	int16* pDeltaB =  pIn->deltaTableB;
	int16* pDeltaR =  pIn->deltaTableR;


    vfe_8x_program_dmi_cfg(ROLLOFF_RAM);

	 // first pack and write init table
    for (i=0; i<VFE_ROLL_OFF_INIT_TABLE_SIZE; i++) {

		data = ((uint32_t)(*pInitR)) | (((uint32_t)(*pInitGr))<<16);
		pInitR++;
		pInitGr++;

		vfe_8x_program_register_value(VFE_DMI_DATA_LO, data);

		data = ((uint32_t)(*pInitB)) | (((uint32_t)(*pInitGb))<<16);
		pInitB++;
		pInitGb++;

		vfe_8x_program_register_value(VFE_DMI_DATA_LO, data);

	}

	#if 0
	// added to test the read out part.
	vfe_8x_program_register_value(VFE_DMI_ADDR, 0);  

	for (i=0;i<26;i++)

	{
    	readOutData = vfe_8x_read_reg_value(VFE_DMI_DATA_LO);

		CDBG("lens roll-off init table value is 0x%x\n", readOutData);

	}
    #endif


    // there are gaps between the init table and delta table, set the offset for delta table.
	vfe_8x_program_register_value(VFE_DMI_ADDR, LENS_ROLL_OFF_DELTA_TABLE_OFFSET);   

	// pack and write delta table
    for (i=0; i<VFE_ROLL_OFF_DELTA_TABLE_SIZE; i++) {

		data = ((uint32_t)(*pDeltaR)) | (((uint32_t)(*pDeltaGr))<<16);
		pDeltaR++;
		pDeltaGr++;

		vfe_8x_program_register_value(VFE_DMI_DATA_LO, data);

		data = ((uint32_t)(*pDeltaB)) | (((uint32_t)(*pDeltaGb))<<16);
		pDeltaB++;
		pDeltaGb++;

		vfe_8x_program_register_value(VFE_DMI_DATA_LO, data);

	}



	#if 0
	// added to test the read out part.
	vfe_8x_program_register_value(VFE_DMI_ADDR, LENS_ROLL_OFF_DELTA_TABLE_OFFSET);  

	for (i=0;i<26;i++)

	{
    	readOutData = vfe_8x_read_reg_value(VFE_DMI_DATA_LO);

		CDBG("lens roll-off delta table value is 0x%x\n", readOutData);

	}
    #endif

	// After DMI transfer, to make it safe, need to set the DMI_CFG to unselect any SRAM
	vfe_8x_program_register_value(VFE_DMI_CFG, VFE_DMI_CFG_DEFAULT);   // unselect the SRAM Bank.


}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_program_dmi_cfg
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
void vfe_8x_program_dmi_cfg(VFE_DMI_RAM_SEL bankSel){

     uint32_t value = VFE_DMI_CFG_DEFAULT;   // set bit 8 for auto increment.
     //uint32_t readOutData;

	 value += (uint32_t)bankSel;
	 //CDBG("dmi cfg input bank is  0x%x\n", bankSel);


	 vfe_8x_program_register_value(VFE_DMI_CFG, value);  // select the bank.
	 //CDBG("dmi cfg input  0x%x\n", value);

	 vfe_8x_program_register_value(VFE_DMI_ADDR, 0);   // by default, always starts with offset 0.

	 // check to make sure dmi bank is selected.
	 //readOutData = vfe_8x_read_reg_value(VFE_DMI_CFG);
	 //CDBG("dmi cfg is 0x%x\n", readOutData);



}
 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_get_af_pingpong_status
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
boolean vfe_8x_get_af_pingpong_status(void){


	uint32_t busPingPongStatus;


    busPingPongStatus = vfe_8x_read_reg_value(VFE_BUS_PINGPONG_STATUS);

	if ((busPingPongStatus & VFE_AF_PINGPONG_STATUS_BIT) == 0)

	{
		return FALSE;
	}
    else {
		return TRUE;
	}

}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_get_awb_pingpong_status
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
boolean vfe_8x_get_awb_pingpong_status(void){


	uint32_t busPingPongStatus;


    busPingPongStatus = vfe_8x_read_reg_value(VFE_BUS_PINGPONG_STATUS);

	if ((busPingPongStatus & VFE_AWB_PINGPONG_STATUS_BIT) == 0)

	{
		return FALSE;   // hw is working on ping
	}
    else {
		return TRUE;    // hw is working on pong
	}

}




 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_read_af_pingpong_buf_address
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
uint32_t vfe_8x_read_af_pingpong_buf_address(boolean pingpong){


     if (pingpong == FALSE)

	 {
		 return vfe_8x_read_reg_value(VFE_BUS_STATS_AF_WR_PING_ADDR);
	 }

	 else // TRUE is for pong buffer
	 {
		 return vfe_8x_read_reg_value(VFE_BUS_STATS_AF_WR_PONG_ADDR);

	 }

}


 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_read_awb_pingpong_buf_address
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */
uint32_t vfe_8x_read_awb_pingpong_buf_address(boolean pingpong){


     if (pingpong == FALSE)

	 {
		 return vfe_8x_read_reg_value(VFE_BUS_STATS_AWB_WR_PING_ADDR);
	 }

	 else // TRUE is for pong buffer
	 {
		 return vfe_8x_read_reg_value(VFE_BUS_STATS_AWB_WR_PONG_ADDR);

	 }

}

 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_update_af_pingpong_address
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void  vfe_8x_update_af_pingpong_address(boolean pingpong, uint32_t address){


	if (pingpong == FALSE)

	{
		vfe_8x_program_register_value(VFE_BUS_STATS_AF_WR_PING_ADDR, address);
		//CDBG("next af buffer address is  = 0x%x\n", address);

	}

	else // TRUE is for pong buffer
	{
		vfe_8x_program_register_value(VFE_BUS_STATS_AF_WR_PONG_ADDR, address);
		//CDBG("next af buffer address is  = 0x%x\n", address);
	}

}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_update_awb_pingpong_address
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void  vfe_8x_update_awb_pingpong_address(boolean pingpong, uint32_t address){

	if (pingpong == FALSE)

	{
		vfe_8x_program_register_value(VFE_BUS_STATS_AWB_WR_PING_ADDR, address);
		//CDBG("next awb buffer address is  = 0x%x\n", address);
	}

	else // TRUE is for pong buffer
	{
		vfe_8x_program_register_value(VFE_BUS_STATS_AWB_WR_PONG_ADDR, address);
		//CDBG("next awb buffer address is  = 0x%x\n", address);

	}

}



 /* ----------------------------------------------------------------------------
 *    FUNCTION        vfe_8x_process_frame_done_irq_no_frag
 *    DESCRIPTION     
 *    DEPENDENCIES    
 *    PARAMETERS      
 *                    
 *    RETURN VALUE    
 *    SIDE EFFECTS    
 * ----------------------------------------------------------------------------
 */

void vfe_8x_process_frame_done_irq_no_frag(vfe_output_path_combo* pIn, uint32_t* pNextAddr, uint32_t* pToRenderAddr){

	uint32_t busPingPongStatus;
    uint32_t tempAddress;

	// 1. read hw status register.
	busPingPongStatus = vfe_8x_read_reg_value(VFE_BUS_PINGPONG_STATUS);
    CDBG("hardware status is 0x%x\n", busPingPongStatus);

	// 2. determine ping or pong
	busPingPongStatus = busPingPongStatus & (1<<(pIn->cbcrStatusBit));  // use cbcr status

	// 3. read out address and update address
	if (busPingPongStatus == 0)   // hw is working on ping, render pong buffer
	{
        // a. read out pong address
         tempAddress = vfe_8x_read_reg_value(pIn->yPath.hwRegPongAddress);   // read out y address.
		 *pToRenderAddr++ = tempAddress;
		 tempAddress = vfe_8x_read_reg_value(pIn->cbcrPath.hwRegPongAddress);   // read out cbcr address.
		 *pToRenderAddr = tempAddress;

		// b. update pong address
        vfe_8x_program_register_value(pIn->yPath.hwRegPongAddress, *pNextAddr++);
		vfe_8x_program_register_value(pIn->cbcrPath.hwRegPongAddress, *pNextAddr);


	}
	else{  // hw is working on pong, render ping buffer

        // a. read out ping address
         tempAddress = vfe_8x_read_reg_value(pIn->yPath.hwRegPingAddress);   // read out y address.
		 *pToRenderAddr++ = tempAddress;
		 tempAddress = vfe_8x_read_reg_value(pIn->cbcrPath.hwRegPingAddress);   // read out cbcr address.
		 *pToRenderAddr = tempAddress;

		// b. update ping address
        vfe_8x_program_register_value(pIn->yPath.hwRegPingAddress, *pNextAddr++);
		vfe_8x_program_register_value(pIn->cbcrPath.hwRegPingAddress, *pNextAddr);

	}


    //CDBG("updated the hardware with Y addr 0x%x\n", *pNextAddr++);
    //CDBG("updated the hardware with cbcr addr 0x%x\n", *pNextAddr);

}

void vfe_8x_start(struct vfe_cmd_start *pin)
{
	uint32_t vfePmStatus = 0;
	uint8_t  rawModeIdle = 1;
	uint32_t demuxPeriod=0;
	uint32_t demuxEvenCfg=0;
	uint32_t demuxOddCfg=0;

	/* demux module is always enabled.  set the period and pattern correctly.
	 Programming gain is optional. Default gain is set after reset. */
	switch (pin->inpixel) {
	case VFE_BAYER_RGRGRG:
		demuxPeriod = 1;
		demuxEvenCfg = 0xC9;
		demuxOddCfg = 0xAC;
		break;

	case VFE_BAYER_GRGRGR:
		demuxPeriod = 1;
		demuxEvenCfg = 0x9C;
		demuxOddCfg = 0xCA;
		break;

	case VFE_BAYER_BGBGBG:
		demuxPeriod = 1;
		demuxEvenCfg = 0xCA;
		demuxOddCfg = 0x9C;
		break;

	case VFE_BAYER_GBGBGB:
		demuxPeriod = 1;
		demuxEvenCfg = 0xAC;
		demuxOddCfg = 0xC9;
		break;

	case VFE_YUV_YCbYCr:
		demuxPeriod = 3;
		demuxEvenCfg = 0x9CAC;
		demuxOddCfg = 0x9CAC;
		break;

	case VFE_YUV_YCrYCb:
		demuxPeriod = 3;
		demuxEvenCfg = 0xAC9C;
		demuxOddCfg = 0xAC9C;
		break;

	case VFE_YUV_CbYCrY:
		demuxPeriod = 3;
		demuxEvenCfg = 0xC9CA;
		demuxOddCfg = 0xC9CA;
		break;

	case VFE_YUV_CrYCbY:
		demuxPeriod = 3;
		demuxEvenCfg = 0xCAC9;
		demuxOddCfg = 0xCAC9;
		break;

	default:
		return -EINVAL;
	}

  /* default gain is programmed after reset. */
	vfe_config_demux(demuxPeriod,demuxEvenCfg, demuxOddCfg);

	/* program gamma lut bank selection
	default to use bank 0.  In case there are more than one table to be program, we still only
	need to program the bank selection once before starting. */
	vfe_program_lut_bank_sel(&vfeGammaLutSel);

	/* save variables to local.   */
	vfeOperationMode = pin->operationMode;

	if (vfeOperationMode == VFE_START_OPERATION_MODE_SNAPSHOT) {
		/* in snapshot mode, initialize snapshot count*/
    vfeSnapShotCount = pin->snapshotCount;

    /* assumption is to have the same pattern and period for both
		 * paths, if both paths are used. */
    if (viewPath.pathEnabled) {
        viewPath.snapshotPendingCount = pin->snapshotCount;
        vfeFrameSkipPattern = vfeFrameSkip.output1Pattern;
        vfeFrameSkipPeriod = vfeFrameSkip.output1Period;
    }

    if (encPath.pathEnabled) {
        encPath.snapshotPendingCount = pin->snapshotCount;
        vfeFrameSkipPattern = vfeFrameSkip.output2Pattern;
        vfeFrameSkipPeriod = vfeFrameSkip.output2Period;
    }
	}

	/* Stats related:
	enable color conversion for bayer sensor */
	if (pin->inputPixelPattern <=VFE_BAYER_GBGBGB ) {
		/* bayer sensor */
    vfeStatsCmdLocal.colorConversionEnable = TRUE;   /* if stats enabled, need to do color conversion. */
	}

  /* does not hurt if nothing is enabled. */
	vfe_8x_program_stats_cmd(&vfeStatsCmdLocal);

	/* Module enable */
	if (pin->inputPixelPattern >=VFE_YUV_YCbYCr ) {
		/* YUV sensor
		need to enable chroma upsample. */
		vfeModuleEnableLocal.chromaUpsampleEnable = TRUE;
	}

	/* demux module is always enabled.  Programming gain is optional. */
	vfeModuleEnableLocal.demuxEnable = TRUE;
	/* if any stats module is enabled, the main bit is enabled. */

	vfeModuleEnableLocal.statsEnable =
		vfeStatsCmdLocal.autoFocusEnable |
		vfeStatsCmdLocal.axwEnable;

	vfe_reg_module_cfg(&vfeModuleEnableLocal);

	/* in case of offline processing, do not need to config camif.
	 * Having bus output enabled in
	 * camif_config register might confuse the hardware? */
	if (pin->inputSource != VFE_START_INPUT_SOURCE_AXI) {
 
    vfe_reg_camif_config(&vfeCamifConfigLocal);
	} else {
		/* offline processing, enable axi read */
    vfeBusConfigLocal.stripeRdPathEn = TRUE;
    vfeBusCmdLocal.stripeReload = TRUE;
    vfeBusConfigLocal.rawPixelDataSize = axiInputDataSize;
		/* need to update the raw pixel size
		to overwrite output data size.  Since in this
		case, output will be YUV, so will be 8bit.
		but input can be any size. */
	}

	vfe_reg_bus_cfg(&vfeBusConfigLocal);

	/* call HAL function to program the registers. */
	vfe_start(pin);

	/*  clear all pending interrupts.         */
	vfe_program_irq_clear(VFE_CLEAR_ALL_IRQS);

	/*  define how composite interrupt work.  */
	vfeImaskCompositePacked = vfe_8x_irq_composite_pack(vfeIrqCompositeMaskLocal);
	vfe_program_irq_composite_mask(vfeImaskCompositePacked);

	/*  enable all necessary interrupts.      */
	vfeImaskLocal.camifSofIrq = TRUE;   // enable SOF irq.
	vfeImaskLocal.regUpdateIrq = TRUE;  // reg update irq
	vfeImaskLocal.resetAckIrq =TRUE;    // this is needed for stop command.

	/* epoch interrrupts are enabled at camif command.
	view/enc ping-pong irqs are enabled in axi-output command, as necessary. ( multi-fragment case)
	view/enc frame done irqs are enabled in axi-output command, as necessary. ( which path is used.)
	stats irqs-- to be done.
	sync timer irqs -- to be done.
	async timer irqs -- to be done. */

	vfeImaskPacked = vfe_8x_irq_pack(vfeImaskLocal);
	vfe_program_irq_mask(vfeImaskPacked);

	/* enable bus performance monitor:
	in this function, HAL also writes to pm_cmd to go. */
	vfe_pm_start(&vfeBusPmConfigLocal);

	/*  trigger vfe reg update */
	vfeStartAckPendingFlag= TRUE;

	/* write bus command to trigger reload of ping pong buffer. */
	vfeBusCmdLocal.busPingpongReload =TRUE;           /* strictly a strobe based value. */

	if (vfeModuleEnableLocal.statsEnable == TRUE) {

		vfeBusCmdLocal.statsPingpongReload=TRUE;   /* strictly a strobe based value. */
		vfeStatsPingPongReloadFlag = TRUE;         // Pingpong reload is one time !!!
	}

	vfe_program_reg_update_cmd(VFE_REG_UPDATE_TRIGGER);

	vfe_reg_bus_cmd(&vfeBusCmdLocal);   // program later than the reg update.

	/* per hardware team, fix the order of the following to:
	 * camif-go, reg-update, and then test-gen-go. */

	if ((pin->inputSource ==
			 VFE_START_INPUT_SOURCE_CAMIF) ||       // if input is from camif, do camif go
			(pin->inputSource ==
			 VFE_START_INPUT_SOURCE_TESTGEN)) {   // in the case of test gen, camif go also needed.

    /* write camif command to enable camif capture. */
    vfe_camif_go();
	}

	/* start test gen if it is enabled */
	if (vfeTestGenStartFlag == TRUE) {
	   vfeTestGenStartFlag = FALSE;
	   vfe_program_hw_testgen_cmd(VFE_TEST_GEN_GO);
	}

	/* in case of raw dump only, use bus performance monitor to indicate there is data live
	on the bus.  No reg_update_irq from hardware.
	since in this case only encoder cbcr path is used, so only check VFE_BUS_ENC_CBCR_WR_PM_STAT_1
	check the max count great than zero. */
	if (axiOutputMode == VFE_AXI_OUTPUT_MODE_CAMIFToAXIViaOutput2) {
		/* raw dump mode */
    while (rawModeIdle) {
        vfePmStatus = vfe_read_pm_status_in_raw_capture();

        if ((vfePmStatus & VFE_PM_BUF_MAX_CNT_MASK) !=0) {   /* max count > 0  */
             rawModeIdle = FALSE;
        }
    }

    /* FIXME: vfe_drv_complete_async_sequence (); -- unnecessary */
	}

}
