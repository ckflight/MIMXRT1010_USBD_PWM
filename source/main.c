
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1011.h"
#include "fsl_debug_console.h"


#include "fsl_iomuxc.h"
#include "fsl_pwm.h"
#include "fsl_xbara.h"

#include "CK_TIME.h"

#include "virtual_com.h"

int CK_MATH_ScaleRange(int x, int offset, int DesTo, int DesFrom, int SrcTo, int SrcFrom);

uint8_t rx_buffer[128];
uint16_t rx_buffer_index = 0;

int main(void){

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    CK_USBD_Init();

    CK_TIME_Init();

    // GPIO_03 is configured as FLEXPWM1_PWM1_B
	IOMUXC_SetPinMux(IOMUXC_GPIO_03_FLEXPWM1_PWM1_B, 0U);

	// GPIO_04 is configured as FLEXPWM1_PWM1_A
	IOMUXC_SetPinMux(IOMUXC_GPIO_04_FLEXPWM1_PWM1_A, 0U);

	// GPIO_03 PAD functional properties :
	// Slew Rate Field: Slow Slew Rate
	// Drive Strength Field: R0/4
	// Speed Field: fast(150MHz)
	// Open Drain Enable Field: Open Drain Disabled
	// Pull / Keep Enable Field: Pull/Keeper Enabled
	// Pull / Keep Select Field: Keeper
	// Pull Up / Down Config. Field: 100K Ohm Pull Down
	// Hyst. Enable Field: Hysteresis Disabled
	IOMUXC_SetPinConfig(IOMUXC_GPIO_03_FLEXPWM1_PWM1_B, 0x10A0U);

	// GPIO_04 PAD functional properties :
	// Slew Rate Field: Slow Slew Rate
	// Drive Strength Field: R0/4
	// Speed Field: fast(150MHz)
	// Open Drain Enable Field: Open Drain Disabled
	// Pull / Keep Enable Field: Pull/Keeper Enabled
	// Pull / Keep Select Field: Keeper
	// Pull Up / Down Config. Field: 100K Ohm Pull Down
	// Hyst. Enable Field: Hysteresis Disabled
	IOMUXC_SetPinConfig(IOMUXC_GPIO_04_FLEXPWM1_PWM1_A, 0x10A0U);


	CLOCK_SetDiv(kCLOCK_AhbDiv, 0x2); /* Set AHB PODF to 2, divide by 3 */
	CLOCK_SetDiv(kCLOCK_IpgDiv, 0x3); /* Set IPG PODF to 3, divide by 4 */

	// Set the PWM Fault inputs to a low value
	XBARA_Init(XBARA);
	XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault0);
	XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault1);
	XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault2);
	XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault3);

    /*
     * pwmConfig.enableDebugMode 		= false;
     * pwmConfig.enableWait 			= false;
     * pwmConfig.reloadSelect 			= kPWM_LocalReload;
     * pwmConfig.faultFilterCount 		= 0;
     * pwmConfig.faultFilterPeriod 		= 0;
     * pwmConfig.clockSource 			= kPWM_BusClock;
     * pwmConfig.prescale 				= kPWM_Prescale_Divide_1;
     * pwmConfig.initializationControl 	= kPWM_Initialize_LocalSync;
     * pwmConfig.forceTrigger 			= kPWM_Force_Local;
     * pwmConfig.reloadFrequency 		= kPWM_LoadEveryOportunity;
     * pwmConfig.reloadLogic 			= kPWM_ReloadImmediate;
     * pwmConfig.pairOperation 			= kPWM_Independent;
     */
	pwm_config_t pwmConfig;
    PWM_GetDefaultConfig(&pwmConfig);

    // Use full cycle reload
    PWM_Init(PWM1, kPWM_Module_1, &pwmConfig);


    uint32_t pwmSourceClockInHz = CLOCK_GetFreq(kCLOCK_IpgClk);
    uint32_t pwmFrequencyInHz = 10000;
    uint16_t deadTimeVal;
    uint16_t deadTime = 0; // was 650 ns

    deadTimeVal = ((uint64_t)pwmSourceClockInHz * deadTime) / 1000000000;

    pwm_signal_param_t pwmSignal[2];

    pwmSignal[0].pwmChannel       = kPWM_PwmA;
    pwmSignal[0].level            = kPWM_HighTrue;
    pwmSignal[0].dutyCyclePercent = 50;
    pwmSignal[0].deadtimeValue    = deadTimeVal;

    pwmSignal[1].pwmChannel = kPWM_PwmB;
    pwmSignal[1].level      = kPWM_HighTrue;
    pwmSignal[1].dutyCyclePercent = 50;
    pwmSignal[1].deadtimeValue    = deadTimeVal;

    PWM_SetupPwm(PWM1, kPWM_Module_1, pwmSignal, 2, kPWM_CenterAligned, pwmFrequencyInHz, pwmSourceClockInHz);

    // Set the load okay bit for all submodules to load registers from their buffer
    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_1, true);

    // Start the PWM generation from Submodules 1
    PWM_StartTimer(PWM1, kPWM_Control_Module_1);

    while(1) {

    	uint8_t data;
    	while(CK_USBD_ReadData(&data) == 1){

    		rx_buffer[rx_buffer_index++] = data;
		}

    	if(rx_buffer_index){

    		// Send 0 to 9 number usb and assign 10 times of it to pwm duty cycle.

    		// Received number is ascii
    		uint8_t num = rx_buffer[0] - 48;
    		uint8_t duty_number = num * 10;

        	PWM_UpdatePwmDutycycle(PWM1, kPWM_Module_1, kPWM_PwmA, kPWM_CenterAligned, duty_number);
        	PWM_UpdatePwmDutycycle(PWM1, kPWM_Module_1, kPWM_PwmB, kPWM_CenterAligned, duty_number);

        	PWM_SetPwmLdok(PWM1, kPWM_Control_Module_1, true);

    		rx_buffer_index = 0;

    	}

    	CK_TIME_DelayMilliSec(1);

    }


}

int CK_MATH_ScaleRange(int x, int offset, int DesTo, int DesFrom, int SrcTo, int SrcFrom){
	if(x>= SrcFrom && x<= SrcTo){
		int a = (x - offset) * (DesTo - DesFrom);
		int b = SrcTo - SrcFrom;
		int tmp = (a/b + DesFrom);
		return tmp;
	}
	return 0;
}






