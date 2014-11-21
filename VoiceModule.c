//###########################################################################
//
// FILE:    VoiceModule.c
//
// ASSUMPTIONS:
//
//    This program requires the F2802x0 header files.
//
//    This program makes use of variables stored in OTP during factory
//    test on 2802x TMS devices only.
//    These OTP locations on pre-TMS devices may not be populated.
//    Ensure that the following memory locations in TI OTP are populated
//    (not 0xFFFF) before use:
//
//    0x3D7E90 to 0x3D7EA4
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The 2802x Boot Mode table is shown below.
//
//    $Boot_Table
//    While an emulator is connected to your device, the TRSTn pin = 1,
//    which sets the device into EMU_BOOT boot mode. In this mode, the
//    peripheral boot modes are as follows:
//
//      Boot Mode:   EMU_KEY        EMU_BMODE
//                   (0xD00)         (0xD01)
//      ---------------------------------------
//      Wait         !=0x55AA        X
//      I/O          0x55AA          0x0000
//      SCI          0x55AA          0x0001
//      Wait         0x55AA          0x0002
//      Get_Mode     0x55AA          0x0003
//      SPI          0x55AA          0x0004
//      I2C          0x55AA          0x0005
//      OTP          0x55AA          0x0006
//      Wait         0x55AA          0x0007
//      Wait         0x55AA          0x0008
//      SARAM        0x55AA          0x000A   <-- "Boot to SARAM"
//      Flash        0x55AA          0x000B
//      Wait         0x55AA          Other
//
//   Write EMU_KEY to 0xD00 and EMU_BMODE to 0xD01 via the debugger
//   according to the Boot Mode Table above. Build/Load project,
//   Reset the device, and Run example
//
//   $End_Boot_Table
//	 EPWM2A is on GPIO2
//   EPWM2B is on GPIO3

#include <stdio.h>
#include <file.h>

#include "DSP28x_Project.h"     // DSP28x Headerfile

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

__interrupt void adc_isr(void);
interrupt void epwm2_isr(void);
void Adc_Config(void);
int timeToUpdate = 0;

extern void DSP28x_usDelay(Uint32 Count);
extern const unsigned int sine256Q15[];
extern void WriteDac(Uint16 channel, int16 duty_frac);

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
//    volatile struct EPWM_REGS *EPwmRegHandle;
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
#define EPWM_CMP_DOWN 0

int ConversionCount = 0;
int16_t voltage[FRAME_SIZE];
int16_t frame[FRAME_SIZE];
int indexToPlay = 0;
int loopCount = 0;
const float sampleRate = 1000000;
const float clockRate = 60000000;
const int highSpeedClockDiv = 5;

unsigned int periodForSampleRate;
unsigned int maxValue; //= sine256Q15[64];//2^bitResolution - 1;
unsigned int sineDuties[256];
unsigned int wavDuties[1020];

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
	for (i = 0; i < FRAME_SIZE; i++)
	{
		frame[i] = voltage[i];
		//Detect Jenkins now...
	}
}

__interrupt void adc_isr(void)
{
    //discard ADCRESULT0 as part of the workaround to the 1st sample errata for rev0
	//TODO: Subtract DC Bias
    int a = ADC_readResult(myAdc, ADC_ResultNumber_1);
    int b = ADC_readResult(myAdc, ADC_ResultNumber_2);
    voltage[ConversionCount] = (a+b)/2;

    if (ConversionCount == FRAME_SIZE)
    {
    	// Disable the PIE and all interrupts
    	//TODO: Remove this after we want continuous frames
	   PIE_disable(myPie);
	   PIE_disableAllInts(myPie);
	   CPU_disableGlobalInts(myCpu);
	   CPU_clearIntFlags(myCpu);

	   int i;
	   for (i = 0; i < FRAME_SIZE; i++)
	   {
		   //printf("%d, ", voltage[i]);
	   }
    }

    if(ConversionCount == FRAME_SIZE)
    {
    	fullFrame();
        ConversionCount = 0;
    }
    else ConversionCount++;

    // Clear ADCINT1 flag reinitialize for next SOC
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_10);

    return;
}

int calculate_duty_cycle(unsigned int data, int bitResolution)
{
	//const float sampleRate = 2560;

	unsigned int duty = (((float) data/maxValue) * (float) periodForSampleRate);//32767
	//printf("data = %i, duty = %i \r\n", data, duty);

	return duty;
}

void update_compare(EPWM_INFO *epwm_info, const unsigned int *dataToPlay, bool loop)
{
	//PWM_setCmpA(epwm_info->myPwmHandle, calculate_duty_cycle(dataToPlay[indexToPlay], 16));
	PWM_setCmpB(epwm_info->myPwmHandle, dataToPlay[indexToPlay]);

	if (++indexToPlay >= 256)//2536
	{
		//printf("Loop!");
		indexToPlay = 0;
		++loopCount;
	}
	return;
}

interrupt void epwm2_isr(void)
{
    // Update the CMPA and CMPB values
    update_compare(&epwm2_info, sineDuties, true);
	// Output a sine wave

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

void main()
{
    volatile int status = 0;
    volatile FILE *fid;

    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;

    int i;
	for(i = 0; i < FRAME_SIZE; i++)
	{
		voltage[i] = 0;
		//frame[i] = 0;
	}

	periodForSampleRate = (((1/sampleRate) * clockRate) / (highSpeedClockDiv)); //136 for 44.1 kHz
	maxValue = sine256Q15[64];//2^bitResolution - 1;

	int counter = 0;
	for (counter = 0; counter < 256; counter++)
		sineDuties[counter] = calculate_duty_cycle(sine256Q15[counter], 16);

	counter = 0;
	//for (counter = 0; counter < 1020; counter++)
	//	wavDuties[counter] = calculate_duty_cycle(wavFile[counter], 16);


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
    //CLK_enableTbClockSync(myClk);
    //CLK_enablePwmClock(myClk, PWM_Number_1);
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

    // Initialize SCIA
    //scia_init();

    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_10, PIE_SubGroupNumber_1, (intVec_t)&adc_isr);
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_2, (intVec_t)&epwm2_isr);

    //Initialize the ADC
	//ADC_enableBandGap(myAdc);
	//ADC_enableRefBuffers(myAdc);
	//ADC_powerUp(myAdc);
	//ADC_enable(myAdc);
	//ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

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
    //ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);               //ADCINT1 trips after AdcResults latch
    //ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enabled ADCINT1
    //ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);          //Disable ADCINT1 Continuous mode
    //ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC2);                 //setup EOC2 to trigger ADCINT1 to fire
    //ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A4);    //set SOC0 channel select to ADCINA4
    //ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A4);    //set SOC1 channel select to ADCINA4
    //ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_A2);    //set SOC2 channel select to ADCINA2
    //ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    //ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    //ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM1_ADCSOCA);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    //ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

    // Enable PWM clock
   //CLK_enablePwmClock(myClk, PWM_Number_1);

   // Setup PWM for ADC interrupt trigger
   //30MHz / 680 = 44.1 kHz
   //PWM_enableSocAPulse(myPwm);                                         // Enable SOC on A group.  SOC = start of conversion
   //PWM_setSocAPulseSrc(myPwm, PWM_SocPulseSrc_CounterEqualCmpAIncr);   // Select SOC from CPMA on upcount
   //PWM_setSocAPeriod(myPwm, PWM_SocPeriod_FirstEvent);                 // Generate pulse on 1st event
   //PWM_setCmpA(myPwm, 0x0080);                                         // Set compare A value.
   //PWM_setPeriod(myPwm, 680);                                       // Set period for ePWM1.  Trigger the ADC every 680 ePWM clocks.
   //PWM_setCounterMode(myPwm, PWM_CounterMode_Up);                      // count up and start

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

    //Redirect STDOUT to SCI
    //status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
    //fid = fopen("scia","w");
    //freopen("scia:", "w", stdout);
    //setvbuf(stdout, NULL, _IONBF, 0);

    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    CPU_enableInt(myCpu, CPU_IntNumber_3);
    PIE_enablePwmInt(myPie, PWM_Number_2);

    CLK_disableTbClockSync(myClk);
    InitEPwm2();
    CLK_enableTbClockSync(myClk);
    CLK_enablePwmClock(myClk, PWM_Number_2);

    //printf("yo");

    for(;;)
    {
        //DELAY_US(100000);
    	//if (GPIO_getData(myGpio, GPIO_Number_12) == 1)
    	//{
    		//printf("Loopcount: %i", loopCount);
    	//}
    }
}




