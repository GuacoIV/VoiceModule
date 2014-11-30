//###########################################################################
//
// FILE:    VoiceModule.c
// EPWM2B is Launchpad J6 - 4 (Audio out)
// ADCINA4 is Launchpad J1 - 6 (Mic in)

#include <stdio.h>
#include <file.h>

#include "DSP28x_Project.h"

#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/sci.h"
#include "f2802x_common/include/sci_io.h"
#include "f2802x_common/include/wdog.h"
#include "Audio.h"

interrupt void adc_isr(void);
interrupt void epwm2_isr(void);
void Adc_Config(void);

ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
SCI_Handle mySci;
PWM_Handle myPwm, myPwm2;
CPU_Handle myCpu;

typedef struct
{
    PWM_Handle myPwmHandle;
    uint16_t EPwm_CMPA_Direction;
    uint16_t EPwm_CMPB_Direction;
    uint16_t EPwmTimerIntCount;
    uint16_t EPwmMaxCMPA;
    uint16_t EPwmMinCMPA;
    uint16_t EPwmMaxCMPB;
    uint16_t EPwmMinCMPB;
}EPWM_INFO;

EPWM_INFO epwm2_info;

#define FRAME_SIZE 256
#define EPWM_CMP_UP   1
#define MAX_SAMPLE_VALUE 4095
#define INCLUDE_PRINTF false

int ConversionCount = 0;
bool calibrateDC = true;
int DCBias = 0;
int16_t voltage[FRAME_SIZE];
//int16_t frame[FRAME_SIZE];
int indexToPlay = 0;
int loopCount = 0;
const float sampleRate = 1000000;
const float clockRate = 60000000;
const int highSpeedClockDiv = 5;

unsigned int periodForSampleRate;
unsigned int maxValue; //= sine256Q15[64];//2^bitResolution - 1;
struct Audio audioToPlay;
struct Audio secondPitch;
bool switchedPitches = false;
bool demo = true;
bool audioIsInFlash = false;
int demoState = 0;
int numInterrupt = 0;

#if INCLUDE_PRINTF
// SCIA  8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_init()
{
    CLK_enableSciaClock(myClk);

    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SCI_disableParity(mySci);
    SCI_setNumStopBits(mySci, SCI_NumStopBits_One);
    SCI_setCharLength(mySci, SCI_CharLength_8_Bits);

    SCI_enableTx(mySci);
    SCI_enableRx(mySci);
    SCI_enableTxInt(mySci);
    SCI_enableRxInt(mySci);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
    // Configured for 115.2kbps
#if (CPU_FRQ_60MHZ)
    SCI_setBaudRate(mySci, SCI_BaudRate_115_2_kBaud);
#elif (CPU_FRQ_50MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)13);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)10);
#endif

    SCI_enableFifoEnh(mySci);
    SCI_resetTxFifo(mySci);
    SCI_clearTxFifoInt(mySci);
    SCI_resetChannels(mySci);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_Empty);

    SCI_resetRxFifo(mySci);
    SCI_clearRxFifoInt(mySci);
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_4_Words);

    SCI_setPriority(mySci, SCI_Priority_FreeRun);

    SCI_enable(mySci);

    return;
}
#endif

/*
 * Simultaneously transmit and receive a byte on the SPI.
 *
 * Polarity and phase are assumed to be both 0, i.e.:
 *   - input data is captured on rising edge of SCLK.
 *   - output data is propagated on falling edge of SCLK.
 *
 * Returns the received byte.
 */
uint8_t SPI_transfer_byte(uint8_t byte_out)
{
    //uint8_t byte_in = 0;
    //uint8_t bit;

//    for (bit = 0x80; bit; bit >>= 1) {
//       /* Shift-out a bit to the MOSI line */
//        write_MOSI((byte_out & bit) ? HIGH : LOW);
//
//        /* Delay for at least the peer's setup time */
//        delay(SPI_SCLK_LOW_TIME);
//
//        /* Pull the clock line high */
//        write_SCLK(HIGH);
//
//        /* Shift-in a bit from the MISO line */
//        if (read_MISO() == HIGH)
//           byte_in |= bit;
//
//        /* Delay for at least the peer's hold time */
//        delay(SPI_SCLK_HIGH_TIME);
//
//        /* Pull the clock line low */
//        write_SCLK(LOW);
//    }

    return 0;//byte_in;
}

void fullFrame()
{
	int i = 0;
	int half = MAX_SAMPLE_VALUE / 2;
	GPIO_setLow(myGpio, GPIO_Number_0);
	for (i = 0; i < FRAME_SIZE; i++)
	{
		//frame[i] = voltage[i];
		//Detect Jenkins now...
		//if (voltage[i] > half)
			//GPIO_setHigh(myGpio, GPIO_Number_0);
		//printf("%d, ", voltage[i]);
	}
}

void stop_adc()
{
	ADC_disable(myAdc);
	ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
	PIE_clearInt(myPie, PIE_GroupNumber_10);
	ADC_disableInt(myAdc, ADC_IntNumber_1);
	CPU_disableInt(myCpu, CPU_IntNumber_10);
}

interrupt void adc_isr(void)
{
    //discard ADCRESULT0 as part of the workaround to the 1st sample errata for rev0
    int a = ADC_readResult(myAdc, ADC_ResultNumber_1);
    int b = ADC_readResult(myAdc, ADC_ResultNumber_2);
    voltage[ConversionCount] = ((a+b)/2) - DCBias;

    if(ConversionCount == FRAME_SIZE && !calibrateDC)
    {
    	fullFrame();
        ConversionCount = 0;
    }
    else if (ConversionCount == FRAME_SIZE && calibrateDC)
    {
    	long bias = 0;
    	int i = 0;
    	for (i = 0; i < FRAME_SIZE; i++)
    	{
    		bias += voltage[i];
    	}
    	bias /= FRAME_SIZE;
    	DCBias = (int) bias;
    	ConversionCount = 0;
    	calibrateDC = false;
    }
    else
    	ConversionCount++;

    // Clear ADCINT1 flag reinitialize for next SOC
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_10);

    return;
}

int calculate_duty_cycle(unsigned int data, int bitResolution)
{
	//const float sampleRate = 2560;

	unsigned int duty = (float)(((float) data/maxValue) * (float) periodForSampleRate);//32767
	//printf("data = %i, duty = %i \r\n", data, duty);

	return duty;
}

void update_compare(EPWM_INFO *epwm_info, struct Audio audio, bool loop)
{
	if (!audioIsInFlash)
	{
		PWM_setCmpB(epwm_info->myPwmHandle, audio.duties[indexToPlay]);

		if (++indexToPlay >= audio.length && loop)
		{
			indexToPlay = 0;
			++loopCount;
		}
	}
	else
	{
		numInterrupt++;
		if (numInterrupt == 7)
		{
			PWM_setCmpB(epwm_info->myPwmHandle, flashDuties[indexToPlay]);
			if (++indexToPlay >= flashLength)
			{
				indexToPlay = 0;
				++loopCount;
				CLK_disableTbClockSync(myClk);
			}
			numInterrupt = 0;
		}
	}
	return;
}

interrupt void epwm2_isr(void)
{
    // Update the CMPB values
	if (!audioIsInFlash)
		update_compare(&epwm2_info, audioToPlay, true);
	else
		update_compare(&epwm2_info, audioToPlay, false);

	if (loopCount > 1000 && loopCount < 2000 && !switchedPitches)
	{
		audioToPlay = secondPitch;
		switchedPitches = true;
	}
	else if (loopCount >= 2000)
	{
		CLK_disableTbClockSync(myClk);
		loopCount = 0;
		indexToPlay = 0;
	}


    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm2);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);
}

//Audio
void InitEPwm2()
{
	indexToPlay = 0;
    CLK_enablePwmClock(myClk, PWM_Number_2);

    // Setup TBCLK
    int period = (((1/sampleRate) * clockRate) / (highSpeedClockDiv));
    int halfOfPeriod = (period / 2);
    //printf("period is %i and halfOfPeriod is %i", period, halfOfPeriod);

    PWM_setPeriod(myPwm2, period);   // Set timer period (int) 136.1 TBCLKs
    PWM_setPhase(myPwm2, 0x0000);               // Phase is 0
    PWM_setCount(myPwm2, 0x0000);               // Clear counter

    // Setup counter mode
    PWM_setCounterMode(myPwm2, PWM_CounterMode_Up); // Count up
    PWM_disableCounterLoad(myPwm2);                     // Disable phase loading
    PWM_setHighSpeedClkDiv(myPwm2, PWM_HspClkDiv_by_10); // Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm2, PWM_ClkDiv_by_1);

    // Setup shadowing
    PWM_setShadowMode_CmpB(myPwm2, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpB(myPwm2, PWM_LoadMode_Zero);

    // Set Compare values
    PWM_setCmpB(myPwm2, halfOfPeriod);        // Set Compare B value

    // Set actions
    PWM_setActionQual_Zero_PwmB(myPwm2, PWM_ActionQual_Set);            // Set PWM2B on Zero
    PWM_setActionQual_CntUp_CmpB_PwmB(myPwm2, PWM_ActionQual_Clear);    // Clear PWM2B on event B, up count

    // Interrupt where we will change the Compare Values
    PWM_setIntMode(myPwm2, PWM_IntMode_CounterEqualZeroOrPeriod);   // Select INT on Zero event
    PWM_enableInt(myPwm2);                                  // Enable INT
    PWM_setIntPeriod(myPwm2, PWM_IntPeriod_ThirdEvent);     // Generate INT on 3rd event

    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;   // increasing CMPB
    epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
    epwm2_info.myPwmHandle = myPwm2;                // Set the pointer to the ePWM module
    epwm2_info.EPwmMaxCMPA = period;        // Setup min/max CMPA/CMPB values
    epwm2_info.EPwmMinCMPA = halfOfPeriod;
    epwm2_info.EPwmMaxCMPB = period;
    epwm2_info.EPwmMinCMPB = halfOfPeriod;
}

void beep_low_then_high()
{
	audioIsInFlash = false;
	switchedPitches = false;
	indexToPlay = 0;
	loopCount = 0;
	audioToPlay = beepLow;
	secondPitch = beepHigh;
	InitEPwm2();
	CLK_enableTbClockSync(myClk);
}

void beep_high_then_low()
{
	audioIsInFlash = false;
	switchedPitches = false;
	indexToPlay = 0;
	loopCount = 0;
	audioToPlay = beepHigh;
	secondPitch = beepLow;
	InitEPwm2();
	CLK_enableTbClockSync(myClk);
}

void main()
{
#if INCLUDE_PRINTF
    volatile int status = 0;
    volatile FILE *fid;
#endif

    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;

    init_audio();
    int i;
	for(i = 0; i < FRAME_SIZE; i++)
	{
		voltage[i] = 0;
		//frame[i] = 0;
	}

	periodForSampleRate = (((1/sampleRate) * clockRate) / (highSpeedClockDiv)); //136 for 44.1 kHz
	maxValue = 32767;

	int counter = 0;
	for (counter = 0; counter < sine.length; counter++)
		sine.duties[counter] = calculate_duty_cycle(sine.data[counter], 16);

	counter = 0;
	for (counter = 0; counter < beepLow.length; counter++)
		beepLow.duties[counter] = calculate_duty_cycle(beepLow.data[counter], 16);

	counter = 0;
	for (counter = 0; counter < beepHigh.length; counter++)
		beepHigh.duties[counter] = calculate_duty_cycle(beepHigh.data[counter], 16);


    // Initialize all the handles needed for this application
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    mySci = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
    myPwm = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    myPwm2 = PWM_init((void *) PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

    // Perform basic system initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    CLK_enableTbClockSync(myClk);
    CLK_enablePwmClock(myClk, PWM_Number_1);
    (*Device_cal)();

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x12 /2 which will yield 60Mhz = 10Mhz * 12 / 2
    // PLL = Phase Locked Loop
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

    // If running from flash copy RAM only functions to RAM
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    // Enable XCLOCKOUT to allow monitoring of oscillator 1
    GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);
    CLK_setClkOutPreScaler(myClk, CLK_ClkOutPreScaler_SysClkOut_by_1);

    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);

#if INCLUDE_PRINTF
    // Initialize SCIA
    scia_init();
#endif

    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_10, PIE_SubGroupNumber_1, (intVec_t)&adc_isr);
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_2, (intVec_t)&epwm2_isr);

    //Initialize the ADC
	ADC_enableBandGap(myAdc);
	ADC_enableRefBuffers(myAdc);
	ADC_powerUp(myAdc);
	ADC_enable(myAdc);
	ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

	// Enable ADCINT1 in PIE
	PIE_enableAdcInt(myPie, ADC_IntNumber_1);
	// Enable CPU Interrupt 1
	CPU_enableInt(myCpu, CPU_IntNumber_10);
	// Enable Global interrupt INTM
	CPU_enableGlobalInts(myCpu);
	// Enable Global realtime interrupt DBGM
	CPU_enableDebugInt(myCpu);

    // Configure ADC
    //Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata
    ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);               //ADCINT1 trips after AdcResults latch
    ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enabled ADCINT1
    ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);          //Disable ADCINT1 Continuous mode
    ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC2);                 //setup EOC2 to trigger ADCINT1 to fire
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A4);    //set SOC0 channel select to ADCINA4
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A4);    //set SOC1 channel select to ADCINA4
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_A2);    //set SOC2 channel select to ADCINA2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

    // Enable PWM clock
    CLK_enablePwmClock(myClk, PWM_Number_1);

   // Setup PWM for ADC interrupt trigger
   //30MHz / 680 = 44.1 kHz
   PWM_enableSocAPulse(myPwm);                                         // Enable SOC on A group.  SOC = start of conversion
   PWM_setSocAPulseSrc(myPwm, PWM_SocPulseSrc_CounterEqualCmpAIncr);   // Select SOC from CPMA on upcount
   PWM_setSocAPeriod(myPwm, PWM_SocPeriod_FirstEvent);                 // Generate pulse on 1st event
   PWM_setCmpA(myPwm, 0x0080);                                         // Set compare A value.
   PWM_setPeriod(myPwm, 680);                                       // Set period for ePWM1.  Trigger the ADC every 680 ePWM clocks.
   PWM_setCounterMode(myPwm, PWM_CounterMode_Up);                      // count up and start

    // Initalize GPIO
    GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);
    GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);

    // Configure GPIO 0-1 as outputs
    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_1, GPIO_0_Mode_GeneralPurpose);

    GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);

    GPIO_setPullUp(myGpio, GPIO_Number_2, GPIO_PullUp_Disable);
    GPIO_setPullUp(myGpio, GPIO_Number_3, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_3_Mode_EPWM2B);

    //Push button input to start
    GPIO_setMode(myGpio, GPIO_Number_12, GPIO_12_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_12, GPIO_Direction_Input);
    GPIO_setPullUp(myGpio, GPIO_Number_12, GPIO_PullUp_Disable);

#if INCLUDE_PRINTF
    //Redirect STDOUT to SCI
    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
    fid = fopen("scia","w");
    freopen("scia:", "w", stdout);
    setvbuf(stdout, NULL, _IONBF, 0);
#endif

    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    CPU_enableInt(myCpu, CPU_IntNumber_3);
    PIE_enablePwmInt(myPie, PWM_Number_2);

    CLK_disableTbClockSync(myClk);

    GPIO_setLow(myGpio, GPIO_Number_0);
    GPIO_setLow(myGpio, GPIO_Number_1);
    //printf("setting low");

    for(;;)
    {
        DELAY_US(500000);
        if (demo)
        {
			if (GPIO_getData(myGpio, GPIO_Number_12) == 1)
			{
				if (demoState == 0)
					beep_low_then_high();
				else if (demoState == 1)
					beep_high_then_low();
				else if (demoState == 2)
				{
					audioIsInFlash = true;
					indexToPlay = 0;
					loopCount = 0;
					InitEPwm2();
					CLK_enableTbClockSync(myClk);
				}

				demoState = ++demoState % 3;
			}
        }
    }
}




