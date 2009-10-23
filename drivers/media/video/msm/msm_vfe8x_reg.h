
#ifndef __MSM_VFE8X_REG_H__
#define __MSM_VFE8X_REG_H__

/*---------------------------------------------------------------------------
      Include Files
----------------------------------------------------------------------------*/

// note all length are in the unit of 32-bit word.  to align with hardware register. 


// vfe_hw_version
#define vfe_hw_version_type_len 1    /*  32-bit word.  */
struct VFE_HW_VersionType {
	uint32_t minorVersion     :  8;
	uint32_t majorVersion     :  8;
	uint32_t coreVersion      :  4;
	uint32_t /* reserved */   :  12;
} __attribute__((packed, aligned(4)));

#define VFE_CFGType_LEN     1    /*  32-bit word.  */
struct VFE_CFGType {
  uint32 pixelPattern     :  3;  /* bayor or yuv pixel pattern  */
  uint32 /* reserved */   : 13;
  uint32 inputSource      :  2;  /* input source for vfe pipeline */
  uint32 /* reserved */   : 14;
} __attribute__((packed, aligned(4)));


#define VFE_BusCmdType_LEN     1    /*  32-bit word.  */



typedef __packed struct VFE_BusCmdType
{
  uint32                  stripeReload         :  1;
  uint32                  /* reserved */       :  3;
  uint32                  busPingpongReload    :  1;
  uint32                  statsPingpongReload  :  1;
  uint32                  /* reserved */       : 26;
} VFE_BusCmdType;


typedef __packed struct VFE_Irq_Composite_MaskType
{
	uint32                  encIrqComMaskBits         :  2;
	uint32                  viewIrqComMaskBits        :  2;
	uint32                  ceDoneSelBits             :  5;
	uint32                  /* reserved */            : 23;
}VFE_Irq_Composite_MaskType;

#define VFE_ModuleEnableType_LEN     1    /*  32-bit word.  */
struct VFE_ModuleEnableType {
	uint32_t blackLevelCorrectionEnable       :   1;
	uint32_t lensRollOffEnable                :   1;
	uint32_t demuxEnable                      :   1;
	uint32_t chromaUpsampleEnable             :   1;
	uint32_t demosaicEnable                   :   1;
	uint32_t statsEnable                      :   1;
	uint32_t cropEnable                       :   1;
	uint32_t mainScalerEnable                 :   1;
	uint32_t whiteBalanceEnable               :   1;
	uint32_t colorCorrectionEnable            :   1;
	uint32_t yHistEnable                      :   1;
	uint32_t skinToneEnable                   :   1;
	uint32_t lumaAdaptationEnable             :   1;
	uint32_t rgbLUTEnable                     :   1;
	uint32_t chromaEnhanEnable                :   1;
	uint32_t asfEnable                        :   1;
	uint32_t chromaSuppressionEnable          :   1;
	uint32_t chromaSubsampleEnable            :   1;
	uint32_t scaler2YEnable                   :   1;
	uint32_t scaler2CbcrEnable                :   1;
	uint32_t                  /* reserved */  :  14;
};


#define VFE_IrqEnableType_LEN     1    /*  32-bit word.  */
typedef __packed struct VFE_IrqEnableType
{
		uint32  camifErrorIrq           :   1;              
		uint32  camifSofIrq             :   1;                   
		uint32  camifEolIrq             :   1;                         
		uint32  camifEofIrq             :   1;                
		uint32  camifEpoch1Irq          :   1;                      
		uint32  camifEpoch2Irq          :   1;                         
		uint32  camifOverflowIrq        :   1;                          
		uint32  ceIrq                   :   1;                    
		uint32  regUpdateIrq            :   1;                  
		uint32  resetAckIrq             :   1;               
		uint32  encYPingpongIrq         :   1;                         
		uint32  encCbcrPingpongIrq      :   1;                      
		uint32  viewYPingpongIrq        :   1;                
		uint32  viewCbcrPingpongIrq     :   1;                        
		uint32  rdPingpongIrq           :   1;                   
		uint32  afPingpongIrq           :   1;                           
		uint32  awbPingpongIrq          :   1;             
		uint32  histPingpongIrq         :   1;               
		uint32  encIrq                  :   1;                      
		uint32  viewIrq                 :   1;                   
		uint32  busOverflowIrq          :   1;
		uint32  afOverflowIrq           :   1;
		uint32  awbOverflowIrq          :   1; 
		uint32  syncTimer0Irq           :   1; 
		uint32  syncTimer1Irq           :   1;        
		uint32  syncTimer2Irq           :   1;       
		uint32  asyncTimer0Irq          :   1;        
		uint32  asyncTimer1Irq          :   1;        
		uint32  asyncTimer2Irq          :   1;       
		uint32  asyncTimer3Irq          :   1;        
		uint32  axiErrorIrq             :   1;         
		uint32  violationIrq            :   1;        
}VFE_IrqEnableType;

typedef VFE_IrqEnableType  VFE_IrqStatusType;
#define VFE_IrqStatusType_LEN     1    /*  32-bit word.  */

typedef VFE_IrqEnableType  VFE_IrqClearType;
#define VFE_IrqClearType_LEN      1    /*  32-bit word.  */

// vfe_chroma_upsample_cfg
#define VFE_UpsampleConfigType_LEN      1    /*  32-bit word.  */
typedef __packed struct VFE_UpsampleConfigType
{
    uint32           chromaCositingForYCbCrInputs          :  1;
	uint32           /* reserved */                        : 31;

} VFE_UpsampleConfigType;



// CAMIF Config Command
#define VFE_CAMIFConfigType_LEN      1    /*  32-bit word.  */
typedef __packed struct VFE_CAMIFConfigType
{
    /* CAMIF Config */
  uint32               /* reserved */                     :  1;/*  Reserved    */
  uint32               VSyncEdge                          :  1;/*  bit 1       */      
  uint32               HSyncEdge                          :  1;/*  bit 2       */
  uint32               syncMode                           :  2;/*  bit 4:3     */
  uint32               vfeSubsampleEnable                 :  1;/*  bit 5       */
  uint32               /* reserved */                     :  1;/*  Reserved    */
  uint32               busSubsampleEnable                 :  1;/*  bit 7       */
  uint32               camif2vfeEnable                    :  1;/*  bit 8       */ 
  uint32               /* reserved */                     :  1;/*  Reserved    */ 
  uint32               camif2busEnable                    :  1;/*  bit 10      */ 
  uint32               irqSubsampleEnable                 :  1;/*  bit 11      */ 
  uint32               binningEnable                      :  1;/*  bit 12      */ 
  uint32               /* reserved */                     : 18;/*  Reserved    */ 
  uint32               misrEnable                         :  1;/*  bit 31      */

}VFE_CAMIFConfigType;

#define VFE_CAMIFConfigCmdType_LEN      7    /*  32-bit word.  */
typedef __packed struct VFE_CAMIFConfigCmdType
{
    /* EFS_Config */
  uint32                          efsEndOfLine                :  8;
  uint32                          efsStartOfLine              :  8;
  uint32                          efsEndOfFrame               :  8;
  uint32                          efsStartOfFrame             :  8;
    /* Frame Config */
  uint32                          frameConfigPixelsPerLine           : 14;
  uint32                       /* reserved */                        :  2;
  uint32                          frameConfigLinesPerFrame           : 14;
  uint32                       /* reserved */                        :  2;
    /* Window Width Config */
  uint32                          windowWidthCfgLastPixel            : 14;
  uint32                       /* reserved */                        :  2;
  uint32                          windowWidthCfgFirstPixel           : 14;
  uint32                       /* reserved */                        :  2;
    /* Window Height Config */
  uint32                          windowHeightCfglastLine            : 14;
  uint32                       /* reserved */                        :  2;
  uint32                          windowHeightCfgfirstLine           : 14;
  uint32                       /* reserved */                        :  2;
   /* Subsample 1 Config */
  uint32                          subsample1CfgPixelSkip             : 16;
  uint32                          subsample1CfgLineSkip              : 16;
   /* Subsample 2 Config */
  uint32                          subsample2CfgFrameSkip             :  4;
  uint32                          subsample2CfgFrameSkipMode         :  1;
  uint32                          subsample2CfgPixelSkipWrap         :  1;
  uint32                       /* reserved */                        : 26;
   /* Epoch Interrupt */
  uint32                          epoch1Line                         : 14;
  uint32                       /* reserved */                        :  2;
  uint32                          epoch2Line                         : 14;
  uint32                       /* reserved */                        :  2;
} VFE_CAMIFConfigCmdType;




#define VFE_UpdateCamifFrameType_LEN      1    /*  32-bit word.  */
typedef __packed struct VFE_UpdateCamifFrameType
{
  uint32                          pixelsPerLine           : 14;
  uint32                       /* reserved */             :  2;
  uint32                          linesPerFrame           : 14;
  uint32                       /* reserved */             :  2;
} VFE_UpdateCamifFrameType; 




// AXI Output Config Command  (VFE_BUS_CFG)
#define VFE_AXIBusCfgType_LEN      1    /*  32-bit word.  */
typedef __packed struct VFE_AXIBusCfgType
{
	/* VFE_BUS_CFG bit packing    */
    uint32                           stripeRdPathEn                   :  1;
	uint32                           /* reserved */                   :  3;
    uint32                           encYWrPathEn                     :  1;
	uint32                           encCbcrWrPathEn                  :  1;
	uint32                           viewYWrPathEn                    :  1;
	uint32                           viewCbcrWrPathEn                 :  1;
    uint32                           rawPixelDataSize                 :  2;
	uint32                           rawWritePathSelect               :  2;
    uint32                           /* reserved */                   : 20;
}VFE_AXIBusCfgType;






// AXI Output Config Command  (except VFE_BUS_CFG)
#define VFE_AXIOutputConfigCmdType_LEN      16    /*  32-bit word.  */
typedef __packed struct VFE_AXIOutputConfigCmdType
{
	/* AXI Output 2 Y Configuration*/
	/* VFE_BUS_ENC_Y_WR_PING_ADDR  */
    uint32                         out2YPingAddr                         : 32;
	/* VFE_BUS_ENC_Y_WR_PONG_ADDR  */
	uint32                         out2YPongAddr                         : 32;
	/* VFE_BUS_ENC_Y_WR_IMAGE_SIZE     */
	uint32                         out2YImageHeight                      : 12;
	uint32                         /* reserved */                        : 4;
	uint32                         out2YImageWidthin64bit                : 10;
	uint32                         /* reserved */                        : 6;
	/* VFE_BUS_ENC_Y_WR_BUFFER_CFG     */
	uint32                         out2YBurstLength                      :  2;
	uint32                         /* reserved */                        :  2;
	uint32                         out2YNumRows                          : 12;
	uint32                         out2YRowIncrementIn64bit              : 12;
	uint32                         /* reserved */                        :  4;
    /* AXI Output 2 Cbcr Configuration*/
	/* VFE_BUS_ENC_Cbcr_WR_PING_ADDR  */
    uint32                         out2CbcrPingAddr                      : 32;
	/* VFE_BUS_ENC_Cbcr_WR_PONG_ADDR  */
	uint32                         out2CbcrPongAddr                      : 32;
	/* VFE_BUS_ENC_Cbcr_WR_IMAGE_SIZE     */
	uint32                         out2CbcrImageHeight                   : 12;
	uint32                         /* reserved */                        : 4;
	uint32                         out2CbcrImageWidthIn64bit             : 10;
	uint32                         /* reserved */                        : 6;
	/* VFE_BUS_ENC_Cbcr_WR_BUFFER_CFG     */
	uint32                         out2CbcrBurstLength                   :  2;
	uint32                         /* reserved */                        :  2;
	uint32                         out2CbcrNumRows                       : 12;
	uint32                         out2CbcrRowIncrementIn64bit           : 12;
	uint32                         /* reserved */                        :  4;
    /* AXI Output 1 Y Configuration*/
	/* VFE_BUS_VIEW_Y_WR_PING_ADDR  */
    uint32                         out1YPingAddr                         : 32;
	/* VFE_BUS_VIEW_Y_WR_PONG_ADDR  */
	uint32                         out1YPongAddr                         : 32;
	/* VFE_BUS_VIEW_Y_WR_IMAGE_SIZE     */
	uint32                         out1YImageHeight                      : 12;
	uint32                         /* reserved */                        : 4;
	uint32                         out1YImageWidthin64bit                : 10;
	uint32                         /* reserved */                        : 6;
	/* VFE_BUS_VIEW_Y_WR_BUFFER_CFG     */
	uint32                         out1YBurstLength                      :  2;
	uint32                         /* reserved */                        :  2;
	uint32                         out1YNumRows                          : 12;
	uint32                         out1YRowIncrementIn64bit              : 12;
	uint32                         /* reserved */                        :  4;
    /* AXI Output 1 Cbcr Configuration*/
	/* VFE_BUS_VIEW_Cbcr_WR_PING_ADDR  */
    uint32                         out1CbcrPingAddr                      : 32;
	/* VFE_BUS_VIEW_Cbcr_WR_PONG_ADDR  */
	uint32                         out1CbcrPongAddr                      : 32;
	/* VFE_BUS_VIEW_Cbcr_WR_IMAGE_SIZE     */
	uint32                         out1CbcrImageHeight                   : 12;
	uint32                         /* reserved */                        : 4;
	uint32                         out1CbcrImageWidthIn64bit             : 10;
	uint32                         /* reserved */                        : 6;
	/* VFE_BUS_VIEW_Cbcr_WR_BUFFER_CFG     */
	uint32                         out1CbcrBurstLength                   :  2;
	uint32                         /* reserved */                        :  2;
	uint32                         out1CbcrNumRows                       : 12;
	uint32                         out1CbcrRowIncrementIn64bit           : 12;
	uint32                         /* reserved */                        :  4;
} VFE_AXIOutputConfigCmdType;




// Output Clamp Config Command
#define VFE_OutputClampConfigCmdType_LEN 2
typedef __packed struct VFE_OutputClampConfigCmdType
{

    /* Output Clamp Maximums */
  uint32                          yChanMax                :  8;
  uint32                          cbChanMax               :  8;
  uint32                          crChanMax               :  8;
  uint32                         /* reserved */           :  8;
    /* Output Clamp Minimums */
  uint32                          yChanMin                :  8;
  uint32                          cbChanMin               :  8;
  uint32                          crChanMin               :  8;
  uint32                         /* reserved */           :  8;
} VFE_OutputClampConfigCmdType;

struct VFE_FOV_CropConfigCmdType {
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

#define VFE_FRAME_SKIP_UpdateCmdType_LEN  2
typedef __packed struct VFE_FRAME_SKIP_UpdateCmdType
{
	uint32    yPattern                  : 32;
	uint32    cbcrPattern               : 32;
}VFE_FRAME_SKIP_UpdateCmdType;


#define VFE_FRAME_SKIP_ConfigCmdType_LEN  8
typedef __packed struct VFE_FRAME_SKIP_ConfigCmdType
{
        /* Frame Drop Enc (output2) */
    uint32       output2YPeriod			    : 5;
	uint32      /* reserved */			    : 27;
	uint32	     output2CbCrPeriod			: 5;
	uint32      /* reserved */			    : 27;
	uint32       output2YPattern            : 32;
	uint32       output2CbCrPattern         : 32;
        /* Frame Drop View (output1) */
    uint32       output1YPeriod             : 5;
    uint32      /* reserved */              : 27;
    uint32       output1CbCrPeriod          : 5;
    uint32      /* reserved */              : 27;
    uint32       output1YPattern            : 32;
    uint32       output1CbCrPattern         : 32;
} VFE_FRAME_SKIP_ConfigCmdType;



// Main Scaler Horizontal Config Command
#define VFE_Main_Scaler_ConfigCmdType_LEN 7
typedef __packed struct VFE_Main_Scaler_ConfigCmdType
{
        /* Scaler Enable Config */
    uint32                          hEnable                     : 1;
    uint32                          vEnable                     : 1;
    uint32                         /* reserved */               : 30;
        /* Scale H Image Size Config */
    uint32                          inWidth                     : 12;
    uint32                         /* reserved */               : 4;
    uint32                          outWidth                    : 12;
    uint32                         /* reserved */               : 4;
        /* Scale H Phase Config */
    uint32                          horizPhaseMult              : 18;
    uint32                         /* reserved */               : 2;
    uint32                          horizInterResolution        : 2;
    uint32                         /* reserved */               : 10;
        /* Scale H Stripe Config */
    uint32                          horizMNInit                 : 12;
    uint32                         /* reserved */               : 4;
    uint32                          horizPhaseInit              : 15;
    uint32                         /* reserved */               : 1;
        /* Scale V Image Size Config */
    uint32                          inHeight                    : 12;
    uint32                         /* reserved */               : 4;
    uint32                          outHeight                   : 12;
    uint32                         /* reserved */               : 4;
        /* Scale V Phase Config */
    uint32                          vertPhaseMult               : 18;
    uint32                         /* reserved */               : 2;
    uint32                          vertInterResolution         : 2;
    uint32                         /* reserved */               : 10;
        /* Scale V Stripe Config */
    uint32                          vertMNInit                  : 12;
    uint32                         /* reserved */               : 4;
    uint32                          vertPhaseInit               : 15;
    uint32                         /* reserved */               : 1;


} VFE_Main_Scaler_ConfigCmdType;



// Scaler 2y Config Command
#define VFE_Scaler2y_ConfigCmdType_LEN 5
typedef __packed struct VFE_Scaler2y_ConfigCmdType
{
        /* Scaler   Enable Config */
    uint32                    hEnable                       : 1;
    uint32                    vEnable                       : 1;
    uint32                   /* reserved */                 : 30;
        /* Scaler   H Image Size Config */
    uint32                    inWidth                       : 12;
    uint32                   /* reserved */                 : 4;
    uint32                    outWidth                      : 12;
    uint32                   /* reserved */                 : 4;
        /* Scaler   H Phase Config */
    uint32                    horizPhaseMult                : 18;
    uint32                   /* reserved */                 : 2;
    uint32                    horizInterResolution          : 2;
    uint32                   /* reserved */                 : 10;
        /* Scaler   V Image Size Config */
    uint32                    inHeight                      : 12;
    uint32                   /* reserved */                 : 4;
    uint32                    outHeight                     : 12;
    uint32                   /* reserved */                 : 4;
        /* Scaler   V Phase Config */
    uint32                    vertPhaseMult                : 18;
    uint32                    /* reserved */                : 2;
    uint32                    vertInterResolution          : 2;
    uint32                    /* reserved */                : 10;
} VFE_Scaler2y_ConfigCmdType;

// Scaler 2cbcr Config Command
typedef VFE_Scaler2y_ConfigCmdType   VFE_Scaler2CbCr_ConfigCmdType;
#define VFE_Scaler2CbCr_ConfigCmdType_LEN 5

// Lens Roll-off Correction Config   ( checked)
#define VFE_Rolloff_ConfigCmdType_LEN 3  
typedef __packed struct VFE_Rolloff_ConfigCmdType
{
        /* Rolloff 0 Config */
    uint32                      gridWidth               : 9;
    uint32                      gridHeight              : 9;
    uint32                      yDelta                  : 9;
    uint32                     /* reserved */           : 5;
        /* Rolloff 1 Config*/
    uint32                      gridX                   : 4;
    uint32                      gridY                   : 4;
    uint32                      pixelX                  : 9;
    uint32                     /* reserved */           : 3;
    uint32                      pixelY                  : 9;
    uint32                     /* reserved */           : 3;
        /* Rolloff 2 Config */
    uint32                      yDeltaAccum             : 12;
    uint32                     /* reserved */           : 20;
} VFE_Rolloff_ConfigCmdType;



// ASF Config 
#define VFE_ASF_UpdateCmdType_LEN 7
typedef __packed struct VFE_ASF_UpdateCmdType
{
        /* ASF Config Command */
    uint32                      smoothEnable            : 1;
    uint32                      sharpMode               : 2;
    uint32                     /* reserved */           : 1;
    uint32                      smoothCoeff1            : 4;
    uint32                      smoothCoeff0            : 8;
    uint32                      pipeFlushCount          : 12;
    uint32                      pipeFlushOvd            : 1;
    uint32                      flushHaltOvd            : 1;
    uint32                      cropEnable              : 1;
    uint32                     /* reserved */           : 1;
        /* Sharpening Config 0 */
    uint32                      sharpThresholdE1        : 7;
    uint32                     /* reserved */           : 1;
    uint32                      sharpDegreeK1           : 5;
    uint32                     /* reserved */           : 3;
    uint32                      sharpDegreeK2           : 5;
    uint32                     /* reserved */           : 3;
    uint32                      normalizeFactor         : 7;
    uint32                     /* reserved */           : 1;
        /* Sharpening Config 1 */
    uint32                      sharpThresholdE2        : 8;
    uint32                      sharpThresholdE3        : 8;
    uint32                      sharpThresholdE4        : 8;
    uint32                      sharpThresholdE5        : 8;
        /* Sharpening Coefficients 0 */
    uint32                      F1Coeff0                : 6;
    uint32                      F1Coeff1                : 6;
    uint32                      F1Coeff2                : 6;
    uint32                      F1Coeff3                : 6;
    uint32                      F1Coeff4                : 6;
    uint32                    /* reserved */            : 2;
        /* Sharpening Coefficients 1 */
    uint32                      F1Coeff5                : 6;
    uint32                      F1Coeff6                : 6;
    uint32                      F1Coeff7                : 6;
    uint32                      F1Coeff8                : 7;
    uint32                     /* reserved */           : 7;
        /* Sharpening Coefficients 2 */
    uint32                      F2Coeff0                : 6;
    uint32                      F2Coeff1                : 6;
    uint32                      F2Coeff2                : 6;
    uint32                      F2Coeff3                : 6;
    uint32                      F2Coeff4                : 6;
    uint32                     /* reserved */           : 2;
        /* Sharpening Coefficients 3 */
    uint32                      F2Coeff5                : 6;
    uint32                      F2Coeff6                : 6;
    uint32                      F2Coeff7                : 6;
    uint32                      F2Coeff8                : 7;
    uint32                     /* reserved */           : 7;
} VFE_ASF_UpdateCmdType;


// ASF Crop Config
#define VFE_ASFCrop_ConfigCmdType_LEN 2
typedef __packed struct VFE_ASFCrop_ConfigCmdType
{
        /* ASF Crop Width Config */
    uint32                      lastPixel               : 12;
    uint32                     /* reserved */           : 4;
    uint32                      firstPixel              : 12;
    uint32                     /* reserved */           : 4;
        /* ASP Crop Height Config */
    uint32                      lastLine                : 12;
    uint32                     /* reserved */           : 4;
    uint32                      firstLine               : 12;
    uint32                     /* reserved */           : 4;

}VFE_ASFCrop_ConfigCmdType;



// Chroma Suppression Config
#define VFE_ChromaSuppress_ConfigCmdType_LEN 2
typedef __packed struct VFE_ChromaSuppress_ConfigCmdType
{
        /* Chroma Suppress 0 Config */
    uint32                      m1                  : 8;
    uint32                      m3                  : 8;
    uint32                      n1                  : 3;
    uint32                     /* reserved */       : 1;
    uint32                      n3                  : 3;
    uint32                     /* reserved */       : 9;
        /* Chroma Suppress 1 Config */
    uint32                      mm1                 : 8;
    uint32                      nn1                 : 3;
    uint32                     /* reserved */       : 21; 
} VFE_ChromaSuppress_ConfigCmdType;



// Chroma Subsample Config Command
#define VFE_ChromaSubsampleConfigCmdType_LEN 3
typedef __packed struct VFE_ChromaSubsampleConfigCmdType
{

    /* Chroma Subsample Selection */
	uint32    hCositedPhase                             : 1;        
	uint32    vCositedPhase                             : 1;         
	uint32    hCosited                                  : 1;             
	uint32    vCosited                                  : 1;             
	uint32    hsubSampleEnable                          : 1;     
	uint32    vsubSampleEnable                          : 1;     
	uint32    cropEnable                                : 1;           
	uint32    /* reserved */                            :25;

	uint32    cropWidthLastPixel                        :12;   
	uint32    /* reserved */                            : 4;
	uint32    cropWidthFirstPixel                       :12;  
	uint32    /* reserved */                            : 4;

	uint32    cropHeightLastLine                        :12;   
	uint32    /* reserved */                            : 4;
	uint32    cropHeightFirstLine                       :12;  
	uint32    /* reserved */                            : 4;

}VFE_ChromaSubsampleConfigCmdType;




// Black Level Correction Config Command
#define VFE_BlackLevel_ConfigCmdType_LEN 4
typedef __packed struct VFE_BlackLevel_ConfigCmdType
{
        /* Black Even-Even Value Config */
    uint32      evenEvenAdjustment          : 9;
    uint32     /* reserved */               :23;
        /* Black Even-Odd Value Config */
    uint32      evenOddAdjustment           : 9;
    uint32     /* reserved */               :23;
        /* Black Odd-Even Value Config */
    uint32      oddEvenAdjustment           : 9;
    uint32     /* reserved */               :23;
        /* Black Odd-Odd Value Config */
    uint32      oddOddAdjustment            : 9;
    uint32     /* reserved */               :23;
}VFE_BlackLevel_ConfigCmdType;




// Demux Config Command
#define VFE_Demux_ConfigCmdType_LEN 2
typedef __packed struct VFE_Demux_ConfigCmdType
{
        /* Demux Gain 0 Config */
    uint32      ch0EvenGain                 :10;
    uint32     /* reserved */               : 6;
    uint32      ch0OddGain                  :10;
    uint32     /* reserved */               : 6;
        /* Demux Gain 1 Config */
    uint32      ch1Gain                     :10;
    uint32     /* reserved */               : 6;
    uint32      ch2Gain                     :10;
    uint32     /* reserved */               : 6;
}VFE_Demux_ConfigCmdType; 



// bad pixel correction info
#define VFE_BpcInfoType_LEN 1
typedef __packed struct VFE_BpcInfoType
{
  uint32                          greenBadPixelCount     :   8;
  uint32                         /* reserved */          :   8;
  uint32                          RedBlueBadPixelCount   :   8;
  uint32                         /* reserved */          :   8;
}VFE_BpcInfoType;




// Demosaic config
#define VFE_Demosaic_CfgCmdType_LEN 1
typedef __packed struct VFE_Demosaic_CfgCmdType
{
        /* Demosaic Config */
    uint32      abfEnable                       : 1;
    uint32      badPixelCorrEnable              : 1;
    uint32      forceAbfOn                      : 1;
    uint32     /* reserved */                   : 1;
    uint32      abfShift                        : 4;
    uint32      fminThreshold                   : 7;
    uint32     /* reserved */                   : 1;
    uint32      fmaxThreshold                   : 7;
    uint32     /* reserved */                   : 5;
    uint32      slopeShift                      : 3;
    uint32     /* reserved */                   : 1;
}VFE_Demosaic_CfgCmdType;


// Demosaic BPC Update Command
#define VFE_DemosaicBPC_CmdType_LEN 2
typedef __packed struct VFE_DemosaicBPC_UpdateCmdType
{
        /* Demosaic BPC Config 0 */
    uint32      blueDiffThreshold               :12;
    uint32      redDiffThreshold                :12;
    uint32     /* reserved */                   : 8;
        /* Demosaic BPC Config 1 */
    uint32      greenDiffThreshold              :12;
    uint32     /* reserved */                   :20;
}VFE_DemosaicBPC_CmdType;



//Demosaic ABF Update Command 
#define VFE_DemosaicABF_CmdType_LEN 2
typedef __packed struct VFE_DemosaicABF_CmdType
{
        /* Demosaic ABF Config 0 */
    uint32      lpThreshold                     :10;
    uint32     /* reserved */                   :22;
        /* Demosaic ABF Config 1 */
    uint32      ratio                           : 4;
    uint32      minValue                        :10;
    uint32     /* reserved */                   : 2;
    uint32      maxValue                        :10;
    uint32     /* reserved */                   : 6;
}VFE_DemosaicABF_CmdType;


// Color Correction Config
#define VFE_ColorCorrection_ConfigCmdType_LEN 13
typedef __packed struct VFE_ColorCorrection_ConfigCmdType
{
            /* Color Corr. Coefficient 0 Config */
    uint32      c0                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 1 Config */
    uint32      c1                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 2 Config */
    uint32      c2                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 3 Config */
    uint32      c3                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 4 Config */
    uint32      c4                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 5 Config */
    uint32      c5                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 6 Config */
    uint32      c6                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 7 Config */
    uint32      c7                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Coefficient 8 Config */
    uint32      c8                          :12;
    uint32     /* reserved */               :20;
            /* Color Corr. Offset 0 Config */
    uint32      k0                          :11;
    uint32     /* reserved */               :21;
            /* Color Corr. Offset 1 Config */
    uint32      k1                          :11;
    uint32     /* reserved */               :21;
            /* Color Corr. Offset 2 Config */
    uint32      k2                          :11;
    uint32     /* reserved */               :21;
        /* Color Corr. Coefficient Q Config */
    uint32      coefQFactor                 : 2;
    uint32     /* reserved */               :30;
}VFE_ColorCorrection_ConfigCmdType;




// Luma Adaptation Config
#define VFE_LumaAdaptation_ConfigCmdType_LEN 1
typedef __packed struct VFE_LumaAdaptation_ConfigCmdType
{
        /* LA Config */
    uint32      lutBankSelect               : 1;
    uint32     /* reserved */               :31;
}VFE_LumaAdaptation_ConfigCmdType;
// White Balance Config
#define VFE_WhiteBalance_ConfigCmdType_LEN 1
typedef __packed struct VFE_WhiteBalance_ConfigCmdType
{
        /* WB Config */
    uint32          ch0Gain             : 9;
    uint32          ch1Gain             : 9;
    uint32          ch2Gain             : 9;
    uint32         /* reserved */       : 5;
}VFE_WhiteBalance_ConfigCmdType;




// Gamma Correction Look-up Table Config
#define VFE_GammaLutSelect_ConfigCmdType_LEN 1
typedef __packed struct VFE_GammaLutSelect_ConfigCmdType
{
        /* LUT Bank Select Config */
    uint32      ch0BankSelect               : 1;
    uint32      ch1BankSelect               : 1;
    uint32      ch2BankSelect               : 1;
    uint32     /* reserved */               :29;
}VFE_GammaLutSelect_ConfigCmdType;




// Chroma Enhancement Config
#define VFE_ChromaEnhance_ConfigCmdType_LEN 5
typedef __packed struct VFE_ChromaEnhance_ConfigCmdType
{
        /* Chroma Enhance A Config */
    uint32      ap                      :11;
    uint32     /* reserved */           : 5;
    uint32      am                      :11;
    uint32     /* reserved */           : 5;
        /* Chroma Enhance B Config */
    uint32      bp                      :11;
    uint32     /* reserved */           : 5;
    uint32      bm                      :11;
    uint32     /* reserved */           : 5;
        /* Chroma Enhance C Config */
    uint32      cp                      :11;
    uint32     /* reserved */           : 5;
    uint32      cm                      :11;
    uint32     /* reserved */           : 5;
        /* Chroma Enhance D Config */
    uint32      dp                      :11;
    uint32     /* reserved */           : 5;
    uint32      dm                      :11;
    uint32     /* reserved */           : 5;
        /* Chroma Enhance K Config */
    uint32      kcb                     :11;
    uint32     /* reserved */           : 5;
    uint32      kcr                     :11;
    uint32     /* reserved */           : 5;
}VFE_ChromaEnhance_ConfigCmdType;




// Color Conversion (RGB to Y) Config
#define VFE_ColorConvert_ConfigCmdType_LEN 4
typedef __packed struct VFE_ColorConvert_ConfigCmdType
{
        /* Conversion Coefficient 0 */
    uint32      v0                      :12;
    uint32     /* reserved */           :20;
        /* Conversion Coefficient 1 */
    uint32      v1                      :12;
    uint32     /* reserved */           :20;
        /* Conversion Coefficient 2 */
    uint32      v2                      :12;
    uint32     /* reserved */           :20;
        /* Conversion Offset */
    uint32      ConvertOffset           : 8;
    uint32     /* reserved */           :24;
}VFE_ColorConvert_ConfigCmdType;




// Sync Timer Config
#define VFE_SyncTimer_ConfigCmdType_LEN 4
typedef __packed struct VFE_SyncTimer_ConfigCmdType
{
        /* Timer Line Start Config */
    uint32          timerLineStart             : 12;
    uint32         /* reserved */              : 20;
        /* Timer Pixel Start Config */
    uint32          timerPixelStart            : 18;
    uint32         /* reserved */              : 14;
        /* Timer Pixel Duration Config */
    uint32          timerPixelDuration         : 28;
    uint32         /* reserved */              : 4;
          /* Sync Timer Polarity Config */
    uint32          timer0Polarity              : 1;
    uint32          timer1Polarity              : 1;
    uint32          timer2Polarity              : 1;
    uint32         /* reserved */               : 29;
}VFE_SyncTimer_ConfigCmdType;




// ASYNC Timer Config
#define VFE_AsyncTimer_ConfigCmdType_LEN 2
typedef __packed struct VFE_AsyncTimer_ConfigCmdType
{   
        /* Async Timer Config 0 */
    uint32          inactiveLength             : 20;
    uint32          numRepetition              : 10;
    uint32         /* reserved */              : 1;
    uint32          polarity                   : 1;
        /* Async Timer Config 1 */
    uint32          activeLength               : 20;
    uint32         /* reserved */              : 12;
}VFE_AsyncTimer_ConfigCmdType;





// AWB Statistics Config 
#define VFE_AWBAEStatistics_ConfigCmdType_LEN 5
typedef __packed struct VFE_AWBAEStatistics_ConfigCmdType
{
        /* AWB autoexposure Config */
    uint32          aeRegionConfig              : 1;
    uint32          aeSubregionConfig           : 1;
    uint32         /* reserved */               : 14;
    uint32          awbYMin                     : 8;
    uint32          awbYMax                     : 8;
        /* AXW Header */
    uint32          axwHeader                   : 8;
    uint32         /* reserved */               : 24;
        /* AWB Mconfig */
    uint32          m4                          : 8;
    uint32          m3                          : 8;
    uint32          m2                          : 8;
    uint32          m1                          : 8;
        /* AWB Cconfig */
    uint32          c2                          : 12;
    uint32         /* reserved */               : 4;
    uint32          c1                          : 12;
    uint32         /* reserved */               : 4;
        /* AWB Cconfig 2 */
    uint32          c4                          : 12;
    uint32         /* reserved */               : 4;
    uint32          c3                          : 12;
    uint32         /* reserved */               : 4;
}VFE_AWBAEStatistics_ConfigCmdType;
//VFE Test Gen Config
#define VFE_TestGen_ConfigCmdType_LEN 12
typedef __packed struct VFE_TestGen_ConfigCmdType
{
       /* HW Test Gen Config */
    uint32          numFrame                        : 10;
    uint32         /* reserved */                   : 2;
    uint32          pixelDataSelect                 : 1;
    uint32          systematicDataSelect            : 1;
    uint32         /* reserved */                   : 2;
    uint32          pixelDataSize                   : 2;
    uint32          hsyncEdge                       : 1;
    uint32          vsyncEdge                       : 1;
    uint32         /* reserved */                   : 12;
        /* HW Test Gen Image Config */
    uint32          imageWidth                      : 14;
    uint32         /* reserved */                   : 2;
    uint32          imageHeight                     : 14;
    uint32         /* reserved */                   : 2;
        /* SOF Offset Config */
    uint32          sofOffset                       : 24;
    uint32         /* reserved */                   : 8;
        /* EOF NOffset Config */
    uint32          eofNOffset                      : 24;
    uint32         /* reserved */                   : 8;
        /* SOL Offset Config */
    uint32          solOffset                       : 9;
    uint32         /* reserved */                   : 23;
        /* EOL NOffset Config */
    uint32          eolNOffset                      : 9;
    uint32         /* reserved */                   : 23;
        /* HBI Config */
    uint32          hBlankInterval                  : 14;
    uint32         /* reserved */                   : 18;
        /* VBL Config */
    uint32          vBlankInterval                  : 14;
    uint32         /* reserved */                   : 2;
    uint32          vBlankIntervalEnable            : 1;
    uint32         /* reserved */                   : 15;
        /* SOF Dummy Line Config */
    uint32          sofDummy                        : 8;
    uint32         /* reserved */                   : 24;
        /* EOF Dummy Line Config */
    uint32          eofDummy                        : 8;
    uint32         /* reserved */                   : 24;
        /* Color Bars Config */
    uint32          unicolorBarSelect               : 3;
    uint32         /* reserved */                   : 1;
    uint32          unicolorBarEnable               : 1;
    uint32          splitEnable                     : 1;
    uint32          pixelPattern                    : 2;
    uint32          rotatePeriod                    : 6;
    uint32         /* reserved */                   : 18;
        /* Random Config */
    uint32          randomSeed                      : 16;
    uint32         /* reserved */                   : 16;
}VFE_TestGen_ConfigCmdType;





//VFE Bus Performance Monitor Config
#define VFE_Bus_Pm_ConfigCmdType_LEN 1
typedef __packed struct VFE_Bus_Pm_ConfigCmdType
{
        /* VFE Bus Performance Monitor Config */
    uint32            output2YWrPmEnable                  : 1;
    uint32            output2CbcrWrPmEnable               : 1;
    uint32            output1YWrPmEnable                  : 1;
    uint32            output1CbcrWrPmEnable               : 1;
    uint32         /* reserved */                       : 28;
}VFE_Bus_Pm_ConfigCmdType;





typedef __packed struct VFE_AsfInfoType
{
    /* asf max edge  */
  uint32                          maxEdge               :  13;
  uint32                         /* reserved */         :   3;
    /* HBi count  */
  uint32                          HBICount              :  12;
  uint32                         /* reserved */         :   4;
}VFE_AsfInfoType;





typedef __packed struct VFE_CamifStatusType
{
  uint32                          pixelCount             :   14;
  uint32                         /* reserved */          :    2;
  uint32                          lineCount              :   14;
  uint32                         /* reserved */          :    1;
  uint32                          camifHalt              :    1;    /* "1" = halt, "0" = capturing*/
}VFE_CamifStatusType;


 //Stats Command Word
typedef __packed struct VFE_StatsCmdType
{
  uint32        autoFocusEnable                       :  1;
  uint32        axwEnable                             :  1;
  uint32        histEnable                            :  1;
  uint32        clearHistEnable                       :  1;
  uint32        histAutoClearEnable                   :  1;
  uint32        colorConversionEnable                 :  1;
  uint32      /* reserved */                          : 26;
}VFE_StatsCmdType;
 //Stats frame size
#define VFE_StatsFrameType_LEN 1
typedef __packed struct VFE_StatsFrameType
{
  uint32        lastPixel                        :  12;
  uint32      /* reserved */                     :   4;
  uint32        lastLine                         :  12;
  uint32      /* reserved */                     :   4;

}VFE_StatsFrameType;


// Stats bus_wr_priority
#define VFE_BusStatsWrPriorityType_LEN 1
typedef __packed struct VFE_BusStatsWrPriorityType
{
  uint32        afBusPriority                   :   4;
  uint32        awbBusPriority                  :   4;
  uint32        histBusPriority                 :   4;

  uint32        afBusPriorityEn                 :   1;
  uint32        awbBusPriorityEn                :   1;
  uint32        histBusPriorityEn               :   1;

  uint32      /* reserved */                    :   17;

}VFE_BusStatsWrPriorityType;


#define VFE_StatsAf_UpdateType_LEN  2
typedef __packed struct VFE_StatsAf_UpdateType
{
  // VFE_STATS_AF_CFG
	uint32        windowVOffset                :   12;
	uint32      /* reserved */                 :    4;
	uint32        windowHOffset                :   12;
	uint32      /* reserved */                 :    3;
	uint32        windowMode                   :    1;

  // VFE_STATS_AF_DIM
    uint32        windowHeight                 :   12;
	uint32      /* reserved */                 :    4;
	uint32        windowWidth                  :   12;
	uint32      /* reserved */                 :    4;

}VFE_StatsAf_UpdateType;



#define VFE_StatsAf_CfgType_LEN  7
typedef __packed struct VFE_StatsAf_CfgType
{
	//  VFE_STATS_AF_GRID_0  

	uint32    entry00                   :   8;
	uint32    entry01                   :   8;
	uint32    entry02                   :   8;
	uint32    entry03                   :   8;

	//  VFE_STATS_AF_GRID_1                 
	uint32    entry10                   :   8;
	uint32    entry11                   :   8;
	uint32    entry12                   :   8;
	uint32    entry13                   :   8;

	//  VFE_STATS_AF_GRID_2                 
	uint32    entry20                   :   8;
	uint32    entry21                   :   8;
	uint32    entry22                   :   8;
	uint32    entry23                   :   8;

	//  VFE_STATS_AF_GRID_3                 
	uint32    entry30                   :   8;
	uint32    entry31                   :   8;
	uint32    entry32                   :   8;
	uint32    entry33                   :   8;

	//  VFE_STATS_AF_HEADER                 
	uint32    afHeader                  :   8;
	uint32      /* reserved */          :  24;
	//  VFE_STATS_AF_COEF0                  
    uint32    a00                       :    5;
	uint32    a04                       :    5;
	uint32    fvMax                     :   11;
	uint32    fvMetric                  :    1;
	uint32      /* reserved */          :   10;

	//  VFE_STATS_AF_COEF1                  
    uint32    a20                       :   5;
	uint32    a21                       :   5;
	uint32    a22                       :   5;
	uint32    a23                       :   5;
	uint32    a24                       :   5;
	uint32      /* reserved */          :   7;

}VFE_StatsAf_CfgType;



// Stats AWBAE Config
#define VFE_StatsAwbae_UpdateType_LEN 1
typedef __packed struct VFE_StatsAwbae_UpdateType
{
    // VFE_STATS_AWBAE_CFG
    uint32      aeRegionCfg             :   1;
    uint32      aeSubregionCfg          :   1;
    uint32       /* reserved */         :  14;
    uint32      awbYMin                 :   8;
    uint32      awbYMax                 :   8;
}VFE_StatsAwbae_UpdateType;



// Stats AXW Header 
#define VFE_StatsAxwHdr_CfgType_LEN 1
typedef __packed struct VFE_StatsAxwHdr_CfgType
{
    // Stats AXW Header Config
    uint32      axwHeader               :   8;
    uint32       /* reserved */         :  24;
}VFE_StatsAxwHdr_CfgType;



// Stats AWB Config
#define VFE_StatsAwb_UpdateType_LEN 3
typedef __packed struct VFE_StatsAwb_UpdateType
{
    // AWB MConfig
    uint32      m4                      :   8;
    uint32      m3                      :   8;
    uint32      m2                      :   8;
    uint32      m1                      :   8;

    // AWB CConfig1
    uint32      c2                      :  12;
    uint32       /* reserved */         :   4;
    uint32      c1                      :  12;
    uint32       /* reserved */         :   4;

    // AWB CConfig2
    uint32      c4                      :  12;
    uint32       /* reserved */         :   4;
    uint32      c3                      :  12;
    uint32       /* reserved */         :   4;
}VFE_StatsAwb_UpdateType;


//Sync timer 
#define VFE_SyncTimerCmdType_LEN 3
typedef __packed struct VFE_SyncTimerCmdType
{
  uint32       hsyncCount                             : 12;    /* line start */
  uint32      /* reserved */                          : 20;    
  uint32       pclkCount                              : 18;    /* pixel start */
  uint32      /* reserved */                          : 14;
  uint32       outputDuration                         : 28;    /* pixel duration */
  uint32      /* reserved */                          :  4;
}VFE_SyncTimerCmdType;


// Async timer 
#define VFE_AsyncTimerCmdType_LEN 2
typedef __packed struct VFE_AsyncTimerCmdType
{
  /*  config 0 */
  uint32      inactiveCount                          : 20;           
  uint32      repeatCount                            : 10;
  uint32      /* reserved */                         :  1;
  uint32      polarity                               :  1;
  /*  config 1 */
  uint32      activeCount                            : 20;
  uint32      /* reserved */                         : 12;

}VFE_AsyncTimerCmdType;



// axi input (offline processing)
#define VFE_AxiInputCmdType_LEN  13
typedef __packed struct VFE_AxiInputCmdType
{
    // 0x008c+4*n 
	uint32     stripeStartAddr0                      : 32;
	uint32     stripeStartAddr1                      : 32;
	uint32     stripeStartAddr2                      : 32;
	uint32     stripeStartAddr3                      : 32;
    //  bus_stripe_rd_Vsize
	uint32     ySize                                 : 12;
	uint32     yOffsetDelta                          : 12;
	uint32     /* reserved */                        : 8;

	// bus_stripe_rd_hSize
	uint32     /* reserved */                        : 16;
	uint32     xSizeWord                             : 10;
	uint32     /* reserved */                        : 6;

	// bus_stripe_rd_buffer_cfg
	uint32     burstLength                           : 2;
	uint32     /* reserved */                        : 2;
	uint32     NumOfRows                             : 12;
	uint32     RowIncrement                          : 12;
	uint32     /* reserved */                        : 4;

	// bus_stripe_rd_unpack_cfg
	uint32     mainUnpackHeight                      : 12;
	uint32     mainUnpackWidth                       : 13;
	uint32     mainUnpackHbiSel                      : 3;
	uint32     mainUnpackPhase                       : 3;
	uint32     /* reserved */                        : 1;

	// bus_stripe_rd_unpack
	uint32     unpackPattern                         : 32;

	// bus_stripe_rd_pad_size
	uint32     padLeft                               : 7;
	uint32     /* reserved */                        : 1;
	uint32     padRight                              : 7;
	uint32     /* reserved */                        : 1;
	uint32     padTop                                : 7;
	uint32     /* reserved */                        : 1;
	uint32     padBottom                             : 7;
	uint32     /* reserved */                        : 1;

	// bus_stripe_rd_pad_L_unpack
	uint32     leftUnpackPattern0                    : 4;
	uint32     leftUnpackPattern1                    : 4;
	uint32     leftUnpackPattern2                    : 4;
	uint32     leftUnpackPattern3                    : 4;
	uint32     leftUnpackStop0                       : 1;
	uint32     leftUnpackStop1                       : 1;
	uint32     leftUnpackStop2                       : 1;
	uint32     leftUnpackStop3                       : 1;
	uint32     /* reserved */                        : 12;

	// bus_stripe_rd_pad_R_unpack
	uint32     rightUnpackPattern0                    : 4;
	uint32     rightUnpackPattern1                    : 4;
	uint32     rightUnpackPattern2                    : 4;
	uint32     rightUnpackPattern3                    : 4;
	uint32     rightUnpackStop0                       : 1;
	uint32     rightUnpackStop1                       : 1;
	uint32     rightUnpackStop2                       : 1;
	uint32     rightUnpackStop3                       : 1;
	uint32     /* reserved */                         : 12;

	// bus_stripe_rd_pad_tb_unpack
	uint32     topUnapckPattern                       : 4;
	uint32     /* reserved */                         : 12;
	uint32     bottomUnapckPattern                    : 4;
	uint32     /* reserved */                         : 12;

}VFE_AxiInputCmdType;


#define VFE_AxiRdFragIrqEnable_LEN 1
typedef __packed struct VFE_AxiRdFragIrqEnable
{
	uint32     stripeRdFragirq0Enable                 : 1;
	uint32     stripeRdFragirq1Enable                 : 1;
	uint32     stripeRdFragirq2Enable                 : 1;
	uint32     stripeRdFragirq3Enable                 : 1;
	uint32     /* reserved */                         : 28;

}VFE_AxiRdFragIrqEnable;



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////








#ifdef FEATURE_WINCE
#pragma warning(pop)
#pragma pack(pop)
//#pragma align(reset)
#endif /* FEATURE_WINCE */
#endif /* __MSM_VFE8X_REG_H__ */
