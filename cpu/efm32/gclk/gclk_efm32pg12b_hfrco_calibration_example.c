/**************************************************************************//**
 * @file cmu_calibrate.c
 * @brief CMU RC calibration functions for EFM32 Gecko Series 1
 * @version  1.13

 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 * !!! THIS IS A MODIFIED VERSION OF THE ORIGINAL IMPLEMENTATION !!!
 *
 ******************************************************************************/

#include <stdlib.h> /* needed for abs() */
#include <stdint.h>
#include "kernel_defines.h" // needed for container_of macro
#include "cpu.h" // needed for RCC register definitions
#include "gclk.h"
#include "gclk/generic_gate.h"   /* @todo: remove/replace (only needed for accessing the gate instances */
#include "gclk/generic_mux.h"    /* @todo: remove/replace (only needed for accessing the mux instances */
#include "gclk/generic_scaler.h" /* @todo: remove/replace (only needed for accessing the mux instances */
#include "gclk_efm32_types.h"
#include "bitarithm.h"

#define DOWN_TOP_VALUE          0xFFFFF
// Global variables
CMU_Osc_TypeDef oscCal;                 // RC oscillator to calibrate
volatile bool downRef;                  // True if down clock is reference
volatile bool tuneEnd;                  // True if tuning is finished
volatile bool fineTune;                 // True if fine tuning is enabled
volatile bool upCountLess;              // upCounter < upCounterTuned
volatile bool upCountGreat;             // upCounter > upCounterTuned
uint16_t tuningVal;                     // Runtime tunning value
uint8_t fineTuningVal;                  // Runtime fine tunning value
uint32_t upCounter;                     // Runtime up counter value
uint32_t upCounterTuned;                // Target up counter value
volatile uint32_t upCounterPrevious;    // Previous up counter value

/***************************************************************************//**
 * @brief
 *   Get RC oscillator frequency fine tuning value.
 * @param[in] osc
 *   RC oscillator to get fine tuning value for, one of:
 *   @li #cmuOsc_HFRCO
 *   @li #cmuOsc_AUXHFRCO
 *   @li #cmuOsc_USHFRCO
 * @return
 *   The RC oscillator frequency fine tuning value in use.
 ******************************************************************************/
uint32_t CMU_OscillatorFineTuningGet(CMU_Osc_TypeDef osc)
{
  uint32_t ret;

  switch (osc)
  {
    case cmuOsc_HFRCO:
      ret = (CMU->HFRCOCTRL & _CMU_HFRCOCTRL_FINETUNING_MASK)
      >> _CMU_HFRCOCTRL_FINETUNING_SHIFT;
      break;

    case cmuOsc_AUXHFRCO:
      ret = (CMU->AUXHFRCOCTRL & _CMU_AUXHFRCOCTRL_FINETUNING_MASK)
      >> _CMU_AUXHFRCOCTRL_FINETUNING_SHIFT;
      break;

#if defined( _CMU_STATUS_USHFRCOENS_MASK )
    case cmuOsc_USHFRCO:
      ret = (CMU->USHFRCOCTRL & _CMU_USHFRCOCTRL_FINETUNING_MASK)
      >> _CMU_USHFRCOCTRL_FINETUNING_SHIFT;
      break;
#endif

  default:
    EFM_ASSERT(0);
    ret = 0;
    break;
  }

  return ret;
}

/***************************************************************************//**
 * @brief
 *   Set RC oscillator frequency fine tuning value.
 * @param[in] osc
 *   RC oscillator to set fine tuning value for, one of:
 *   @li #cmuOsc_HFRCO
 *   @li #cmuOsc_AUXHFRCO
 *   @li #cmuOsc_USHFRCO
 * @param[in] val
 *   The RC oscillator frequency fine tuning value to use.
 ******************************************************************************/
void CMU_OscillatorFineTuningSet(CMU_Osc_TypeDef osc, uint32_t val)
{
  switch (osc)
  {
    case cmuOsc_HFRCO:
      EFM_ASSERT(val <= (_CMU_HFRCOCTRL_FINETUNING_MASK
          >> _CMU_HFRCOCTRL_FINETUNING_SHIFT));
      while(BUS_RegBitRead(&CMU->SYNCBUSY, _CMU_SYNCBUSY_HFRCOBSY_SHIFT))
      {
      }
      CMU->HFRCOCTRL = (CMU->HFRCOCTRL & ~(_CMU_HFRCOCTRL_FINETUNING_MASK))
          | (val << _CMU_HFRCOCTRL_FINETUNING_SHIFT);
      break;

    case cmuOsc_AUXHFRCO:
      EFM_ASSERT(val <= (_CMU_AUXHFRCOCTRL_FINETUNING_MASK
          >> _CMU_AUXHFRCOCTRL_FINETUNING_SHIFT));
      while(BUS_RegBitRead(&CMU->SYNCBUSY, _CMU_SYNCBUSY_AUXHFRCOBSY_SHIFT))
      {
      }
      CMU->AUXHFRCOCTRL = (CMU->AUXHFRCOCTRL & ~(_CMU_AUXHFRCOCTRL_FINETUNING_MASK))
          | (val << _CMU_AUXHFRCOCTRL_FINETUNING_SHIFT);
      break;

#if defined( _CMU_STATUS_USHFRCOENS_MASK )
    case cmuOsc_USHFRCO:
      EFM_ASSERT(val <= (_CMU_USHFRCOCTRL_FINETUNING_MASK
          >> _CMU_USHFRCOCTRL_FINETUNING_SHIFT));
      CMU->USHFRCOCTRL = (CMU->USHFRCOCTRL & ~(_CMU_USHFRCOCTRL_FINETUNING_MASK))
          | (val << _CMU_USHFRCOCTRL_FINETUNING_SHIFT);
      break;
#endif

  default:
    EFM_ASSERT(0);
    break;
  }
}

/**************************************************************************//**
 * @brief End of tuning check.
 * @return
 *   True if tuning is finished.
 *****************************************************************************/
bool endOfTune(void)
{
  return(tuneEnd);
}

/**************************************************************************//**
 * @brief
 *   Get target and actual up counter value.
 * @param[in] targetCnt
 *   Select up counter value to return
 * @return
 *   Target up counter value if targetCnt is true.
 *   Actual up counter value if targetCnt is false.
 *****************************************************************************/
uint32_t getUpcount(bool targetCnt)
{
  if (targetCnt)
  {
    return upCounterTuned;
  }
  else
  {
    return(upCounter);
  }
}

/**************************************************************************//**
 * @brief CMU IRQ handler.
 *****************************************************************************/
void CMU_IRQHandler(void)
{
  // Clear interrupt flag, get the up counter value
  CMU_IntClear(CMU_IFC_CALRDY);
  upCounter = CMU_CalibrateCountGet();
  // Up counter result is smaller than the desired value
  if (upCounter < upCounterTuned)
  {
    if (upCountGreat)
    {
      if (!fineTune)          // End of calibration if no fine tuning
      {                       // and upCounter > upCounterTuned
        CMU_CalibrateStop();
        if ((upCounterTuned - upCounter) > (upCounterPrevious - upCounterTuned))
        {
          tuningVal++;
        }
        tuneEnd = true;
        return;
      }
      else
      {
        if ((upCounterTuned - upCounter) == 1)
        {
          CMU_CalibrateStop();
          tuneEnd = true;
          return;
        }
        /*
         * Next fine tuning if count difference not <= 1.  Increase
         * tuning value if down counter is reference; othewise decrease.
         */
        if (downRef)
        {
          fineTuningVal++;
        }
        else
        {
          fineTuningVal--;
        }
        CMU_OscillatorFineTuningSet(oscCal,fineTuningVal);
        upCountLess = true;  // Set upCounter < upCounterTuned

        if ((CMU->CALCTRL & CMU_CALCTRL_CONT) == 0)
        {
          // Reload TOP value if not continuous mode
          CMU->CALCNT = DOWN_TOP_VALUE;
          CMU->CMD = CMU_CMD_CALSTART;
        }

          return;
      }
    }
    else
    {
      // Store the previous value for comparison
      upCountLess = true;
      upCounterPrevious = upCounter;

      // Increase tuning value if down counter is reference
      if (downRef)
      {
        tuningVal++;
      }
      // Othewise decrease
      else
      {
        tuningVal--;
      }

      // Write the new tuning value to RC oscillator
#if defined( _CMU_STATUS_USHFRCOENS_MASK )
      if (oscCal == cmuOsc_USHFRCO)
      {
        USHFRCO_TUNE_SET;
      }
      else
      {
        CMU_OscillatorTuningSet(oscCal, tuningVal);
      }
#else
      CMU_OscillatorTuningSet(oscCal, tuningVal);
#endif
      if ((CMU->CALCTRL & CMU_CALCTRL_CONT) == 0)
      {
        // Reload TOP value if not continuous mode
        CMU->CALCNT = DOWN_TOP_VALUE;
        CMU->CMD = CMU_CMD_CALSTART;
      }

      return;
    }
  }

  // If up counter result is higher than the desired value
  if (upCounter > upCounterTuned)
  {
    if (upCountLess)
    {
      if (!fineTune)          // End of calibration if no fine tuning
      {                       // and upCounter < upCounterTuned
        CMU_CalibrateStop();
        if ((upCounter - upCounterTuned) > (upCounterTuned - upCounterPrevious))
        {
          tuningVal--;
        }
        tuneEnd = true;
        return;
      }
      else
      {
        if ((upCounter - upCounterTuned) == 1)
        {
          CMU_CalibrateStop();
          tuneEnd = true;
          return;
        }
        /*
         * Next fine tuning if count difference not <= 1.  Decrease
         * tuning value if down counter is reference; othewise increase.
         */
        if (downRef)
        {
          fineTuningVal--;
        }
        else
        {
          fineTuningVal++;
        }
        CMU_OscillatorFineTuningSet(oscCal,fineTuningVal);
        upCountGreat = true;  // Set upCounter > upCounterTuned before

        if ((CMU->CALCTRL & CMU_CALCTRL_CONT) == 0)
        {
        // Reload TOP value if not continuous mode
          CMU->CALCNT = DOWN_TOP_VALUE;
          CMU->CMD = CMU_CMD_CALSTART;
        }

        return;
      }
    }
    else
    {
      // Store the previous value for comparison
      upCountGreat = true;
      upCounterPrevious = upCounter;

      // Decrease tuning value if down counter is reference; othewise increase
      if (downRef)
      {
        tuningVal--;
      }
      else
      {
        tuningVal++;
      }
      // Write the new tuning value to RC oscillator
#if defined( _CMU_STATUS_USHFRCOENS_MASK )
      if (oscCal == cmuOsc_USHFRCO)
      {
        USHFRCO_TUNE_SET;
      }
      else
      {
        CMU_OscillatorTuningSet(oscCal, tuningVal);
      }
#else
      CMU_OscillatorTuningSet(oscCal, tuningVal);
#endif

      if ((CMU->CALCTRL & CMU_CALCTRL_CONT) == 0)
      {
        // Reload TOP value if not continuous mode
        CMU->CALCNT = DOWN_TOP_VALUE;
        CMU->CMD = CMU_CMD_CALSTART;
      }

      return;
    }
  }

  // Up counter result is equal to the desired value, end of calibration
  if (upCounter == upCounterTuned)
  {
    CMU_CalibrateStop();
    tuneEnd = true;
  }
}

/***************************************************************************//**
 * @brief
 *   Configure RC oscillator calibration
 *
 * @details
 *   Configure a calibration for a selectable clock source against another
 *   selectable reference clock.
 *   Refer to the reference manual, CMU chapter, for further details.
 *
 * @note
 *   After configuration, a call to CMU_CalibrateStart() is required, and
 *   the resulting calibration value can be read out with the
 *   CMU_CalibrateCountGet() function call in CMU ISR.
 *
 * @param[in] rcOsc
 *   The RC Oscillator for calibration, one of:
 *   @li #cmuOsc_LFRCO
 *   @li #cmuOsc_HFRCO
 *   @li #cmuOsc_AUXHFRCO
 *   @li #cmuOsc_USHFRCO
 *
 * @param[in] rcOscFreq
 *   The RC oscillator frequency in Hz to be tuned for.
 *
 * @param[in] calCont
 *   True to enable continuous mode if this feature is available.
 *
 * @param[in] fineEnable
 *   True to enable fine tuning if this feature is available.
******************************************************************************/
void calibrateRcOsc(CMU_Osc_TypeDef rcOsc, uint32_t rcOscFreq, bool calCont, bool fineEnable)
{
  oscCal = rcOsc;
  tuneEnd = false;
  fineTune = false;
  upCountLess = false;
  upCountGreat = false;

#ifdef CMU_OUT_EN
  // Reset CMU CLKOUTn output
  CMU->CTRL &= ~CMU_CLKOUTN_MASK;
#endif

  // Enable target RC oscillator
  CMU_OscillatorEnable(oscCal, true, true);

  // Get existing tuning value
#if defined( _CMU_STATUS_USHFRCOENS_MASK )
  if (oscCal == cmuOsc_USHFRCO)
  {
    tuningVal = USHFRCO_TUNE_GET;
    fineTuningVal = CMU_OscillatorFineTuningGet(oscCal);
  }
  else
  {
    tuningVal = CMU_OscillatorTuningGet(oscCal);
  }
#else
  tuningVal = CMU_OscillatorTuningGet(oscCal);
#endif

  switch (oscCal)
  {
  case cmuOsc_LFRCO:
    // Enable HFXO to tune LFRCO
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    upCounterTuned = (uint32_t)(((float)rcOscFreq/(float)SystemHFXOClockGet())*(float)(DOWN_TOP_VALUE+1));

    CMU_CalibrateConfig(DOWN_TOP_VALUE, cmuOsc_HFXO, oscCal);

#ifdef CMU_OUT_EN
    // Enable CMU CLKOUTn output for LFRCO
    CMU->CTRL |= (CMU_CTRL_CLKOUTSELN_LFRCOQ << CMU_CLKOUTN_SHIFT);
#endif

    downRef = false;

    break;

  case cmuOsc_HFRCO:
    // Enable LFXO to tune HFRCO
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    upCounterTuned = (uint32_t)(((float)SystemLFXOClockGet()/(float)rcOscFreq)*(float)(DOWN_TOP_VALUE+1));

    CMU_CalibrateConfig(DOWN_TOP_VALUE, oscCal, cmuOsc_LFXO);

#ifdef CMU_OUT_EN
    // Enable CMU CLKOUTn output for HFRCO
    CMU->CTRL |= (CMU_CTRL_CLKOUTSELN_HFRCOQ << CMU_CLKOUTN_SHIFT);
#endif

    // Check if using fine tuning
    if (fineEnable)
    {
      CMU->HFRCOCTRL |= CMU_HFRCOCTRL_FINETUNINGEN;
      fineTuningVal = CMU_OscillatorFineTuningGet(oscCal);
      fineTune = true;
    }

    downRef = true;

    break;

  case cmuOsc_AUXHFRCO:
    // Enable LFXO to tune AUXHFRCO
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    upCounterTuned = (uint32_t)(((float)SystemLFXOClockGet()/(float)rcOscFreq)*(float)(DOWN_TOP_VALUE+1));

    CMU_CalibrateConfig(DOWN_TOP_VALUE, oscCal, cmuOsc_LFXO);

#ifdef CMU_OUT_EN
    // Enable CMU CLKOUTn output for AUXHFRCO
    CMU->CTRL |= (CMU_CTRL_CLKOUTSELN_AUXHFRCOQ << CMU_CLKOUTN_SHIFT);
#endif

    // Check if using fine tuning
    if (fineEnable)
    {
      CMU->AUXHFRCOCTRL |= CMU_AUXHFRCOCTRL_FINETUNINGEN;
      fineTuningVal = CMU_OscillatorFineTuningGet(oscCal);
      fineTune = true;
    }

    downRef = true;

    break;

#if defined( _CMU_STATUS_USHFRCOENS_MASK )
  case cmuOsc_USHFRCO:
    // Enable LFXO to tune USHFRCO
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    upCounterTuned = (uint32_t)(((float)SystemLFXOClockGet()/(float)rcOscFreq)*(float)(DOWN_TOP_VALUE+1));

    /*
     * CMU_CalibrateConfig does not support the USHFRCO even though it
     * can be calibrated.  Work around this by first selecting the LFRCO
     * as the down counter reference, then manually switch to the USHFRCO
     * by writing directly to the CMU_CALCTRL register/
     */
    CMU_CalibrateConfig(DOWN_TOP_VALUE, cmuOsc_LFRCO, cmuOsc_LFXO);
    CMU->CALCTRL &= ~_CMU_CALCTRL_DOWNSEL_MASK;
    CMU->CALCTRL |= CMU_CALCTRL_DOWNSEL_USHFRCO;

    downRef = true;   // A higher tuning value means a lower frequency for the USHFRCO
    fineTune = true;  // Always perform fine tuning for the USHFRCO

#ifdef CMU_OUT_EN
    // Enable CMU CLKOUTn output for USHFRCO
    CMU->CTRL |= (CMU_CTRL_CLKOUTSELN_USHFRCOQ << CMU_CLKOUTN_SHIFT);
#endif

    break;
#endif

  default:
    EFM_ASSERT(0);
    break;
  }

  // Enable calibration ready interrupt
  //CMU_IntEnable(CMU_IEN_CALRDY);
  //NVIC_ClearPendingIRQ(CMU_IRQn);
  //NVIC_EnableIRQ(CMU_IRQn);

  // If selected, enable continuous calibration
  if (calCont)
    CMU_CalibrateCont(true);

  // Start calibration
  CMU_CalibrateStart();
}

void _calibrate_HFRCO(uint32_t freq)
{
    /* disable PLL before clibration */
    //gclk_reg_util_set_mask(edreg, _gate_dis_mask(gate));
    //while (CMU->STATUS & CMU_STATUS_DPLLENS) {}

    calibrateRcOsc(cmuOsc_HFRCO, freq, false, true);

    // Display result when finished
    while(!endOfTune()) {
        /* poll for IRQ and call IRQ handler */
        while (!(CMU->IF & CMU_IF_CALRDY)) {
            //printf("wait for CALRDY 0x%08lx\n", CMU->IF);
        }
        CMU_IRQHandler();
    }

    printf("Up count value: %lu\n", getUpcount(false));
    printf("New tuning value: %lu\n", CMU_OscillatorTuningGet(cmuOsc_HFRCO));
    printf("New fine tuning\nvalue: %lu\n", CMU_OscillatorFineTuningGet(cmuOsc_HFRCO));
}
