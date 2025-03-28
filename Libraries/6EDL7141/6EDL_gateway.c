/**
 * @file 6EDL_gateway.c
 * @brief SPI communication API with 6EDL7141 registers
 * @Modified date: 2019-07-08
 *
 **********************************************************************************************************************
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *@endcond
 ***********************************************************************************************************************/

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "6EDL_gateway.h"
#include "xmc1_flash.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
extern uint8_t error_SPI;

uint8_t GUIwrReg_6EDL7141_addr = { 0x55 };
uint8_t WriteReg_6EDL7141_addr = { 0x55 };
uint16_t GUIwrReg_6EDL7141_data;
uint16_t WriteReg_6EDL7141_data;
uint8_t GuiMonitor_6EDL7141_addr;
uint16_t GuiMonitor_6EDL7141_value;

/* 6EDL EN_DRV pin */
const XMC_GPIO_CONFIG_t GPIO_EN_DRV_Config = { .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL, .output_level = 0, .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t) 0, };

/* 6EDL watchdog clock on EN_DRV pin */
const XMC_GPIO_CONFIG_t GPIO_EN_DRV_CLK_Config = { .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL, .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW, };

const XMC_GPIO_CONFIG_t GPIO_nBRAKE_Config = { .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL, .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH, };

const XMC_GPIO_CONFIG_t GPIO_AUTO_ZERO_Config = { .mode = XMC_GPIO_MODE_INPUT_PULL_UP, .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW, };

EDL_IO_CONTROL_t EdlIo;
edl7141_register_t Edl7141Reg;

/**
 * @brief Default 6EDL7141 register setting for testing purpose if macro PROGRAM_DEFAULT_PARAM set to 1U
 *
 * @param None
 *
 * @retval None
 */
void EDL7141_MOTOR_PARAM_set_default(void)
{
#if (EDL7141_CHIP_VERSION == 21)
  Edl7141Reg.FAULTS_CLR = 1 << FAULTS_CLR_CLR_FLTS_Pos | 1 << FAULTS_CLR_CLR_LATCH_Pos;
  Edl7141Reg.SUPPLY_CFG = PVCC_10V << SUPPLY_CFG_PVCC_SETPT_Pos |
                          VREF_DVDD_1_2 << SUPPLY_CFG_CS_REF_CFG_Pos |
                          OCP_THR_450 << SUPPLY_CFG_DVDD_OCP_CFG_Pos |
                          DVDD_SFTSTRT_100us << SUPPLY_CFG_DVDD_SFTSTRT_Pos |
                          // DVDD_5 << SUPPLY_CFG_DVDD_SETPT_Pos |
						  BK_FREQ_500K << SUPPLY_CFG_BK_FREQ_Pos |
						  DVDD_TON_DELAY_800us << SUPPLY_CFG_DVDD_TON_DELAY_Pos |
						  CP_PRECHAR_DIS << SUPPLY_CFG_CP_PRECHAR_EN_Pos;
  Edl7141Reg.ADC_CFG =    NO_ADC_REQ << ADC_CFG_ADC_OD_REQ_Pos |
                          ADC_IN_IDIGITAL << ADC_CFG_ADC_OD_INSEL_Pos |
						  ADC_DIS_FILTER << ADC_CFG_ADC_EN_FILT_Pos |
                          ADC_FILT_SAMP_8 << ADC_CFG_ADC_FILT_CFG_Pos |
                          ADC_FILT_PSAMP_32 << ADC_CFG_ADC_FILT_CFG_PVDD_Pos;
  Edl7141Reg.PWM_CFG =    PWM_COMM_MODE_6 << PWM_CFG_PWM_MODE_Pos |
		  				  DIODE_FREEW << PWM_CFG_PWM_FREEW_CFG_Pos |
						  BRAKE_HIGH_LOW << PWM_CFG_BRAKE_CFG_Pos |
                          BRAKE_RECIRC_DIS << PWM_CFG_PWM_RECIRC_Pos;
  Edl7141Reg.SENSOR_CFG = HALL_DEGLITCH_640ns << SENSOR_CFG_HALL_DEGLITCH_Pos |
		  				  OTEMP_PROT_DIS << SENSOR_CFG_OTS_DIS_Pos|
						  CS_ACTIVE_ALWAYS     << SENSOR_CFG_CS_TMODE_Pos;
  Edl7141Reg.WD_CFG =     0 << WD_CFG_WD_EN_Pos |
                          WDIN_DRV << WD_CFG_WD_INSEL_Pos |
                          WDOUT_STATUS << WD_CFG_WD_FLTCFG_Pos |
                          WD_PERIOD_100us << WD_CFG_WD_TIMER_T_Pos;
  Edl7141Reg.WD_CFG2 =    WD_REACT_NFAULT << WD_CFG2_WD_BRAKE_Pos |
                          WD_FLT_LATCH_DIS << WD_CFG2_WD_EN_LATCH_Pos |
                          RSTRT_ATT0 << WD_CFG2_WD_DVDD_RSTRT_ATT_Pos |
                          DVDD_RSTRT_DLY_0ms5 << WD_CFG2_WD_DVDD_RSTRT_DLY_Pos |
                          RLOCK_DIS << WD_CFG2_WD_RLOCK_EN_Pos |
                          RLOCK_WD_TMOUT_1s << WD_CFG2_WD_RLOCK_T_Pos |
                          BUCK_EN << WD_CFG2_WD_BK_DIS_Pos;
  Edl7141Reg.IDRIVE_CFG = IDRIVE_300mA << IDRIVE_CFG_IHS_SRC_Pos |
						  IDRIVE_300mA << IDRIVE_CFG_IHS_SINK_Pos |
						  IDRIVE_300mA << IDRIVE_CFG_ILS_SRC_Pos |
						  IDRIVE_300mA << IDRIVE_CFG_ILS_SINK_Pos;
  Edl7141Reg.IDRIVE_PRE_CFG =   IDRIVE_PRE_150mA << IDRIVE_PRE_CFG_I_PRE_SRC_Pos |
		  						IDRIVE_PRE_300mA << IDRIVE_PRE_CFG_I_PRE_SINK_Pos |
                                PRECHAR_MODE_EN << IDRIVE_PRE_CFG_I_PRE_EN_Pos;
  Edl7141Reg.TDRIVE_SRC_CFG =   TDRIVE1_70ns << TDRIVE_SRC_CFG_TDRIVE1_Pos |
		  						TDRIVE2_260ns << TDRIVE_SRC_CFG_TDRIVE2_Pos;
  Edl7141Reg.TDRIVE_SINK_CFG =  TDRIVE3_70ns << TDRIVE_SINK_CFG_TDRIVE3_Pos |
		  						TDRIVE4_510ns << TDRIVE_SINK_CFG_TDRIVE4_Pos;
  Edl7141Reg.DT_CFG =     DT_760ns << DT_CFG_DT_RISE_Pos |
		  				  DT_760ns << DT_CFG_DT_FALL_Pos;
  Edl7141Reg.CP_CFG =     CP_CLK_781_25kHz << CP_CFG_CP_CLK_CFG_Pos |
                          CP_CLK_SS_EN << CP_CFG_CP_CLK_SS_DIS_Pos;
  Edl7141Reg.CSAMP_CFG =  CS_GAIN_64V << CSAMP_CFG_CS_GAIN_Pos |
                          CS_GAIN_PROG_DIG << CSAMP_CFG_CS_GAIN_ANA_Pos |
                          CS_A_EN_B_EN_C_EN << CSAMP_CFG_CS_EN_Pos |
						  CS_BLANK_500ns << CSAMP_CFG_CS_BLANK_Pos |
      					  CS_CALIB_DIS        << CSAMP_CFG_CS_EN_DCCAL_Pos |
						  CS_DEGLITCH_8us << CSAMP_CFG_CS_OCP_DEGLITCH_Pos |
						  OCP_FLT_TRIG_8 << CSAMP_CFG_CS_OCPFLT_CFG_Pos;
  Edl7141Reg.CSAMP_CFG2 = OCP_POS_THR_300mV << CSAMP_CFG2_CS_OCP_PTHR_Pos |
		  				  OCP_NEG_THR_300mV << CSAMP_CFG2_CS_OCP_NTHR_Pos |
                          OCP_FLT_LATCH_DIS << CSAMP_CFG2_CS_OCP_LATCH_Pos |
                          CS_SENSE_SHUNT_RES << CSAMP_CFG2_CS_MODE_Pos |
						  OCP_FLT_BRAKE_DIS << CSAMP_CFG2_CS_OCP_BRAKE_Pos |
                          OCP_PWM_TRUNC_DIS << CSAMP_CFG2_CS_TRUNC_DIS_Pos | /* OCP_PWM_TRUNC_EN_POS: Enable cycle-by-cycle current limit */
                          CS_VREF_INT << CSAMP_CFG2_VREF_INSEL_Pos |
						  OCP_NEG_EN << CSAMP_CFG2_CS_NEG_OCP_DIS_Pos |
     /* external setting is used in flash memory setting*/                     CS_AUTOZERO_EN << CSAMP_CFG2_CS_AZ_CFG_Pos;
  Edl7141Reg.OTP_PROG =   0 << OTP_PROG_OTP_PROG_Pos | 0x0 << OTP_PROG_USER_ID_Pos; /* write a value different to the value in OTP, this helps to detect unexpected IC reset */

#endif //(EDL7141_CHIP_VERSION == 21)
  Edl7141Reg.chip_version = EDL7141_CHIP_VERSION;
}

/**
 * @brief If macro PROGRAM_DEFAULT_PARAM is 1U, it copy default 6EDL7141 register values to FLASH,
 * else it read the chip version from Flash address, if match, it will upload the 6EDL7141 register value from FLASH to RAM,
 * if chip version read back does not match, it will stay in IDLE state.
 *
 * @param None
 *
 * @retval None
 */
void EDL7141_FLASH_parameter_load(void)
{
  /* MC_INFO structure contains expected parameter version for the firmware. Head of parameter block should contain
   * same value, otherwise parameter block won't be loaded
   */
#if (PROGRAM_DEFAULT_PARAM == 1)
  /* For test only, program the parameter block with default value */
  /* Configure with default parameter */
  EDL7141_MOTOR_PARAM_set_default();

  /* Write default parameter into parameter block */
  XMC_FLASH_ProgramVerifyPage(Edl7141ParameterBlock_Addr, (uint32_t*) &Edl7141Reg);
  SystemVar.Edl7141Configured = true;
#else
  Edl7141Reg.chip_version = ((edl7141_register_t*)Edl7141ParameterBlock_Addr)->chip_version;
  if (Edl7141Reg.chip_version != EDL7141_CHIP_VERSION)
  {
    /* If there is no valid parameter block */
    /* Do nothing, state machine will stay at IDLE state, until all parameters (including chip_version) are configured by master control */
    SystemVar.Edl7141Configured = false;
  }
  else
  {
    /* Found parameter block with matching version, copy parameter block data into Edl7141Reg structure */
    edl7141_register_t *parameter_block = (edl7141_register_t*)Edl7141ParameterBlock_Addr;
    for (int32_t i = 0; i < CFG_ADDR_MAX; i++)
    {
      Edl7141Reg.table[i] = parameter_block->table[i];
    }
    SystemVar.Edl7141Configured = true;
  }
  /* Configure with default parameter */
   // EDL7141_MOTOR_PARAM_set_default();

#endif //PROGRAM_DEFAULT_PARAM
}

/**
 * @brief To read all the 6EDL7141 registers when startup
 *
 * @param None
 *
 * @retval None
 */
void SPI_read_6EDL7141_registers(void)
{
  /* Read status registers */
  for (uint32_t addr = ST_ADDR_MIN; addr < ST_ADDR_MAX; addr++)
  {
    Edl7141Reg.table[addr] = read_word_16b(addr);
  }
  /* Read parameter registers */
  for (uint32_t addr = CFG_ADDR_MIN; addr < CFG_ADDR_MAX; addr++)
  {
    Edl7141Reg.table[addr] = read_word_16b(addr);
  }
}

/**
 * @brief To write the 6EDL7141 configuration registers to default values
 *
 * @param None
 *
 * @retval None
 */
void SPI_write_6EDL7141_registers(void)
{
	/* Write default parameters, exclude: FAULTS_CLR & OTP_PROG */
	for (uint32_t addr = CFG_ADDR_MIN; addr < CFG_ADDR_MAX; addr++) {
		write_word_16b(addr, Edl7141Reg.table[addr]);
	}
}

/**
 * @brief Initialize SPI for 6EDL communication, write configure register value into device, read back all register values
 *
 * @param None
 *
 * @retval None
 */
void EDL7141_Config_init(void)
{
  /* Initializes SPI for 6EDL7141 */
  XMC_GPIO_Init(GPIO_EN_DRV, &GPIO_EN_DRV_Config);
  XMC_GPIO_SetOutputHigh(GPIO_EN_DRV);

  /*initialize 6EDL watchdog clock on EN_DRV pin*/
  //XMC_GPIO_Init(DRV_CLK_EN_PIN, &GPIO_EN_DRV_CLK_Config);
  XMC_GPIO_Init(GPIO_nBRAKE, &GPIO_nBRAKE_Config);
  XMC_GPIO_Init(GPIO_AUTO_ZERO, &GPIO_AUTO_ZERO_Config);

  EdlIo.en_drv_level = 0;
  EdlIo.nbrake_level = 1;
  /* Initialize SPI modules */
  SPI_master_init(SPI_MAS_CH);

  /* To write the 6EDL7141 configuration registers to default values */
  SPI_write_6EDL7141_registers();

  /* read all the 6EDL7141 registers to Edl7141Reg table */
  SPI_read_6EDL7141_registers();
}

/* --- End of File ------------------------------------------------ */

