/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/pwr.c
 * @brief Power control (PWR) support.
 */
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
#include <libmaple/pwr.h>
#include <libmaple/rcc.h>
/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup PWR 
  * @brief PWR driver modules
  * @{
  */ 

/** @defgroup PWR_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup PWR_Private_Defines
  * @{
  */
#define PWR_BASE_ADDR         ((uint32_t)0x40007000)
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */
/* --------- PWR registers bit address in the alias region ---------- */
#define PWR_OFFSET               (PWR_BASE_ADDR - PERIPH_BASE)

/* --- CR Register ---*/

/* Alias word address of DBP bit */
#define CR_OFFSET                (PWR_OFFSET + 0x00)
#define DBP_BitNumber            0x08
#define CR_DBP_BB                (PERIPH_BB_BASE + (CR_OFFSET * 32) + (DBP_BitNumber * 4))

/* Alias word address of PVDE bit */
#define PVDE_BitNumber           0x04
#define CR_PVDE_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PVDE_BitNumber * 4))

/* --- CSR Register ---*/

/* Alias word address of EWUP bit */
#define CSR_OFFSET               (PWR_OFFSET + 0x04)
#define EWUP_BitNumber           0x08
#define CSR_EWUP_BB              (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (EWUP_BitNumber * 4))

/* ------------------ PWR registers bit mask ------------------------ */

/* CR register bit mask */
#define CR_DS_MASK               ((uint32_t)0xFFFFFFFC)
#define CR_PLS_MASK              ((uint32_t)0xFFFFFF1F)


/**
  * @}
  */

/** @defgroup PWR_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup PWR_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup PWR_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup PWR_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the PWR peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void PWR_DeInit(void)
{
  //RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, ENABLE);
  //RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
  rcc_clk_enable(RCC_PWR);
  rcc_clk_disable(RCC_PWR);
}

/**
  * @brief  Enables or disables access to the RTC and backup registers.
  * @param  NewState: new state of the access to the RTC and backup registers.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void PWR_BackupAccessCmd(FunctionalState NewState)
{
  /* Check the parameters */
 // assert_param(IS_FUNCTIONAL_STATE(NewState));
  *(volatile uint32_t *) CR_DBP_BB = (uint32_t)NewState;
}

/**
  * @brief  Enables or disables the Power Voltage Detector(PVD).
  * @param  NewState: new state of the PVD.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void PWR_PVDCmd(FunctionalState NewState)
{
  /* Check the parameters */
  //assert_param(IS_FUNCTIONAL_STATE(NewState));
  *(volatile uint32_t *) CR_PVDE_BB = (uint32_t)NewState;
}

/**
  * @brief  Configures the voltage threshold detected by the Power Voltage Detector(PVD).
  * @param  PWR_PVDLevel: specifies the PVD detection level
  *   This parameter can be one of the following values:
  *     @arg PWR_PVDLevel_2V2: PVD detection level set to 2.2V
  *     @arg PWR_PVDLevel_2V3: PVD detection level set to 2.3V
  *     @arg PWR_PVDLevel_2V4: PVD detection level set to 2.4V
  *     @arg PWR_PVDLevel_2V5: PVD detection level set to 2.5V
  *     @arg PWR_PVDLevel_2V6: PVD detection level set to 2.6V
  *     @arg PWR_PVDLevel_2V7: PVD detection level set to 2.7V
  *     @arg PWR_PVDLevel_2V8: PVD detection level set to 2.8V
  *     @arg PWR_PVDLevel_2V9: PVD detection level set to 2.9V
  * @retval None
  */
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
 //assert_param(IS_PWR_PVD_LEVEL(PWR_PVDLevel));
  tmpreg = PWR_BASE->CR;
  /* Clear PLS[7:5] bits */
  tmpreg &= CR_PLS_MASK;
  /* Set PLS[7:5] bits according to PWR_PVDLevel value */
  tmpreg |= PWR_PVDLevel;
  /* Store the new value */
  PWR_BASE->CR = tmpreg;
}

/**
  * @brief  Enables or disables the WakeUp Pin functionality.
  * @param  NewState: new state of the WakeUp Pin functionality.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void PWR_WakeUpPinCmd(FunctionalState NewState)
{
  /* Check the parameters */
 // assert_param(IS_FUNCTIONAL_STATE(NewState));
  *(volatile uint32_t *) CSR_EWUP_BB = (uint32_t)NewState;
}

void SYS_Sleepy(void)
{  
  SCB->SCR |=0X00;  
 
  #if defined ( __CC_ARM   )  //这个我用jtag仿真发现停止函数也有执    行所以也加了上去  还请大神指教这句话的意思                        
  __force_stores();
  #endif
  /* Request Wait For Interrupt */
  __WFI();
}
/**
  * @brief  Enters STOP mode.
  * @param  PWR_Regulator: specifies the regulator state in STOP mode.
  *   This parameter can be one of the following values:
  *     @arg PWR_Regulator_ON: STOP mode with regulator ON
  *     @arg PWR_Regulator_LowPower: STOP mode with regulator in low power mode
  * @param  PWR_STOPEntry: specifies if STOP mode in entered with WFI or WFE instruction.
  *   This parameter can be one of the following values:
  *     @arg PWR_STOPEntry_WFI: enter STOP mode with WFI instruction
  *     @arg PWR_STOPEntry_WFE: enter STOP mode with WFE instruction
  * @retval None
  */
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
  //assert_param(IS_PWR_REGULATOR(PWR_Regulator));
  //assert_param(IS_PWR_STOP_ENTRY(PWR_STOPEntry));
  
  /* Select the regulator state in STOP mode ---------------------------------*/
  tmpreg = PWR_BASE->CR;
  /* Clear PDDS and LPDS bits */
  tmpreg &= CR_DS_MASK;
  /* Set LPDS bit according to PWR_Regulator value */
  tmpreg |= PWR_Regulator;
  /* Store the new value */
  PWR_BASE->CR = tmpreg;
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP;
  
  /* Select STOP mode entry --------------------------------------------------*/
  if(PWR_STOPEntry == PWR_STOPEntry_WFI)
  {   
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __WFE();
  }
  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);  
}

/**
  * @brief  Enters STANDBY mode.
  * @param  None
  * @retval None
  */
void PWR_EnterSTANDBYMode(void)
{
  /* Clear Wake-up flag */
  PWR_BASE->CR |= PWR_CSR_WUF;
  /* Select STANDBY mode */
  PWR_BASE->CR |= PWR_CSR_SBF;
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP;
/* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM   )
  __force_stores();
#endif
  /* Request Wait For Interrupt */
  __WFI();
}

/**
  * @brief  Checks whether the specified PWR flag is set or not.
  * @param  PWR_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg PWR_FLAG_WU: Wake Up flag
  *     @arg PWR_FLAG_SB: StandBy flag
  *     @arg PWR_FLAG_PVDO: PVD Output
  * @retval The new state of PWR_FLAG (SET or RESET).
  */
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  //assert_param(IS_PWR_GET_FLAG(PWR_FLAG));
  
  if ((PWR_BASE->CSR & PWR_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the flag status */
  return bitstatus;
}

/**
  * @brief  Clears the PWR's pending flags.
  * @param  PWR_FLAG: specifies the flag to clear.
  *   This parameter can be one of the following values:
  *     @arg PWR_FLAG_WU: Wake Up flag
  *     @arg PWR_FLAG_SB: StandBy flag
  * @retval None
  */
void PWR_ClearFlag(uint32_t PWR_FLAG)
{
  /* Check the parameters */
  //assert_param(IS_PWR_CLEAR_FLAG(PWR_FLAG));
         
  PWR_BASE->CR |=  PWR_FLAG << 2;
}

/**
 * Enables the power interface clock, and resets the power device.
 */
void pwr_init(void) {
    rcc_clk_enable(RCC_PWR);
    rcc_reset_dev(RCC_PWR);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
