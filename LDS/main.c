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
#include "f2802x_common/include/pwm.h"

CPU_Handle myCpu;
PLL_Handle myPll;
WDOG_Handle myWDog;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
SCI_Handle mySci;
PWM_Handle myPwm4;

void InitEPwmTimer();

void GPIO_setDirection_Test(GPIO_Handle gpioHandle, const GPIO_Number_e gpioNumber, const GPIO_Direction_e direction)
{
    GPIO_Obj *gpio = (GPIO_Obj *)gpioHandle;
    int i = 100;

    printf("GPIO_setDirection_Test: Begin ENABLE_PROTECTED_REGISTER_WRITE_MODE... \r\n");

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    printf("GPIO_setDirection_Test: After ENABLE_PROTECTED_REGISTER_WRITE_MODE... \r\n");

    if(gpioNumber < GPIO_Number_32)
    {
      // clear the bit
      gpio->GPADIR &= (~((uint32_t)1 << gpioNumber));

      // set the bit
      gpio->GPADIR |= (uint32_t)direction << gpioNumber;
    }
    else
    {
	printf("GPIO_setDirection_Test: >GPIO_Number_32... \r\n");
	i = 100; while(i--);
	printf("GPIO_setDirection_Test: Begin Clear\n");
	i = 100; while(i--);

      // clear the bit
      gpio->GPBDIR &= (~((uint32_t)1 << (gpioNumber - GPIO_Number_32)));
      printf("GPIO_setDirection_Test: After Clear\n");
      i = 100; while(i--);
      printf("GPIO_setDirection_Test: Begin Set\n");
      i = 100; while(i--);

      // set the bit
      gpio->GPBDIR |= (uint32_t)direction << (gpioNumber - GPIO_Number_32);
      printf("GPIO_setDirection_Test: After Set\n");
      i = 100; while(i--);

    }
    printf("GPIO_setDirection_Test: Begin DISABLE_PROTECTED_REGISTER_WRITE_MODE\n");
    i = 100; while(i--);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
    printf("GPIO_setDirection_Test: After DISABLE_PROTECTED_REGISTER_WRITE_MODE\n");
    i = 100; while(i--);

    return;
} // end of GPIO_setDirection() function


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
 * main.c
 */
int main(void) {
	

    //init TI MCU ()
    volatile int status = 0;
    volatile FILE *fid;




    int i = 20, n = 20;


    // Configure the period for each timer
        #define EPWM1_TIMER_TBPRD  1000  // Period register
        #define EPWM1_MAX_CMPA     1950
        #define EPWM1_MIN_CMPA       200
        #define EPWM1_MAX_CMPB     1950
        #define EPWM1_MIN_CMPB       500

        #define EPWM2_TIMER_TBPRD  600  // Period register
        #define EPWM2_MAX_CMPA     1950
        #define EPWM2_MIN_CMPA       200
        #define EPWM2_MAX_CMPB     1950
        #define EPWM2_MIN_CMPB       50

        #define EPWM3_TIMER_TBPRD  2000  // Period register
        #define EPWM3_MAX_CMPA      950
        #define EPWM3_MIN_CMPA       50
        #define EPWM3_MAX_CMPB     1950
        #define EPWM3_MIN_CMPB     1050

    // To keep track of which way the compare value is moving
       #define EPWM_CMP_UP   1
       #define EPWM_CMP_DOWN 0

    // Initialize all the handles needed for this application

    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    mySci = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    myPwm4 = PWM_init((void *)PWM_ePWM4_BASE_ADDR, sizeof(PWM_Obj));


    // Perform basic system initialization
    WDOG_disable(myWDog);

    (*Device_cal)();

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x10 /2 which will yield 60Mhz = 10Mhz * 12 / 2
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

#ifdef F_28027
    //XCLKOUT for sysclk monitor
    EALLOW;

    // LOSPCP prescale register settings, normally it will be set to default values

    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;  // GPIO18 = XCLKOUT
    SysCtrlRegs.LOSPCP.all = 0x0002;

    // XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/4 SYSCLKOUT
    SysCtrlRegs.XCLK.bit.XCLKOUTDIV=2; // Set XCLKOUT = SYSCLKOUT/1
    EDIS;
#endif

    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

    // If running from flash copy RAM only functions to RAM
    #ifdef _FLASH
        memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    #endif
  //


	//init serial communication interface
	scia_init();


    // Set the flash OTP wait-states to minimum. This is important

    // for the performance of the temperature conversion function.
    FLASH_setup(myFlash);



	GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
	GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
	GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);
	GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
	GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);

	//Redirect STDOUT to SCI
	status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
	fid = fopen("scia","w");
	freopen("scia:", "w", stdout);
	setvbuf(stdout, NULL, _IONBF, 0);	

	printf("Init Serial COM... \r\n");

	i=100;
	while (i--);

#if 1


 printf("GPIO_setPullUp DOUT enable PullUp... \r\n");

 GPIO_setPullUp(myGpio, GPIO_Number_1, GPIO_PullUp_Enable);
 GPIO_setPullUp(myGpio, GPIO_Number_2, GPIO_PullUp_Enable); 
 GPIO_setPullUp(myGpio, GPIO_Number_3, GPIO_PullUp_Enable);
 GPIO_setPullUp(myGpio, GPIO_Number_4, GPIO_PullUp_Enable); 
 GPIO_setPullUp(myGpio, GPIO_Number_5, GPIO_PullUp_Enable);
 GPIO_setPullUp(myGpio, GPIO_Number_6, GPIO_PullUp_Enable); 
 GPIO_setPullUp(myGpio, GPIO_Number_16, GPIO_PullUp_Enable);
 GPIO_setPullUp(myGpio, GPIO_Number_17, GPIO_PullUp_Enable); 
 GPIO_setPullUp(myGpio, GPIO_Number_18, GPIO_PullUp_Enable);
 GPIO_setPullUp(myGpio, GPIO_Number_19, GPIO_PullUp_Enable); 

 printf("GPIO_setMode DOUT as GPIO... \r\n");

 // Make GPIO34 an Output and set it low
 GPIO_setMode(myGpio, GPIO_Number_1, GPIO_1_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_2, GPIO_2_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_3, GPIO_3_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_4, GPIO_4_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_5, GPIO_5_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_6, GPIO_6_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_16, GPIO_16_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_17, GPIO_17_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_GeneralPurpose);
 GPIO_setMode(myGpio, GPIO_Number_19, GPIO_19_Mode_GeneralPurpose);

 printf("GPIO_setDirection DOUT as Input... \r\n");

/*
 GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Input);
 GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Input);
 GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Input);
 GPIO_setDirection(myGpio, GPIO_Number_4, GPIO_Direction_Input); 
 GPIO_setDirection(myGpio, GPIO_Number_5, GPIO_Direction_Input);
 GPIO_setDirection(myGpio, GPIO_Number_6, GPIO_Direction_Input);
 GPIO_setDirection(myGpio, GPIO_Number_16, GPIO_Direction_Input);
 GPIO_setDirection(myGpio, GPIO_Number_17, GPIO_Direction_Input); 
 GPIO_setDirection(myGpio, GPIO_Number_18,GPIO_Direction_Input);
 GPIO_setDirection(myGpio, GPIO_Number_19,GPIO_Direction_Input);
*/

GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
 GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output);
 GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);
 GPIO_setDirection(myGpio, GPIO_Number_4, GPIO_Direction_Output); 
 GPIO_setDirection(myGpio, GPIO_Number_5, GPIO_Direction_Output);
 GPIO_setDirection(myGpio, GPIO_Number_6, GPIO_Direction_Output);
 GPIO_setDirection(myGpio, GPIO_Number_16, GPIO_Direction_Output);
 GPIO_setDirection(myGpio, GPIO_Number_17, GPIO_Direction_Output); 
 GPIO_setDirection(myGpio, GPIO_Number_18,GPIO_Direction_Output);
 GPIO_setDirection(myGpio, GPIO_Number_19,GPIO_Direction_Output);
  
 printf("Done: GPIO_setDirection DOUT as Input... \r\n"); 

#endif



#if 0
 printf("GPIO_setMode GPIO34... \r\n");

 // Make GPIO34 an Output and set it low
 GPIO_setMode(myGpio, GPIO_Number_34, GPIO_34_Mode_GeneralPurpose);
 printf("GPIO_setDirection GPIO34... \r\n");
 GPIO_setDirection_Test(myGpio, GPIO_Number_34, GPIO_Direction_Output);
  printf("GPIO_setLow GPIO34... \r\n");
 GPIO_setLow(myGpio,GPIO_Number_34);
// GPIO_setHigh(myGpio,GPIO_Number_34);

 printf("Done: Init GPIO34... \r\n");

#else

 printf("GPIO_setMode GPIO 0... \r\n");

 // Make GPIO34 an Output and set it low
 GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
 printf("GPIO_setDirection GPIO 0... \r\n");
 GPIO_setDirection_Test(myGpio, GPIO_Number_0, GPIO_Direction_Output);
  printf("GPIO_setLow GPIO 0... \r\n");
 GPIO_setLow(myGpio,GPIO_Number_0);
// GPIO_setHigh(myGpio,GPIO_Number_34);

 printf("Done: Init GPIO 0... \r\n");

#endif



#if 1

    GPIO_setPullUp(myGpio, GPIO_Number_7, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_7, GPIO_7_Mode_EPWM4B);



    //InitEPwm4Example();

    //mclk 6M
	// Some useful Period vs Frequency values
	//	SYSCLKOUT = 	60 MHz		 40 MHz
	//	--------------------------------------
	//	  Period			Frequency	 Frequency
	//	  1000			  60 kHz	   40 kHz
	//	  800				 75 kHz 	  50 kHz
	//	  600				 100 kHz	  67 kHz
	//	  500				 120 kHz	  80 kHz
	//	  250				 240 kHz	  160 kHz
	//	  200				 300 kHz	  200 kHz
	//	  100				 600 kHz	  400 kHz
	//	  50				1.2 Mhz 	 800 kHz
	//	  25				2.4 Mhz 	 1.6 MHz
	//	  20				3.0 Mhz 	 2.0 MHz
	//	  12				5.0 MHz 	 3.3 MHz
	//	  10				6.0 MHz 	 4.0 MHz
	//	  9 			   6.7 MHz		4.4 MHz
	//	  8 			   7.5 MHz		5.0 MHz
	//	  7 			   8.6 MHz		5.7 MHz
	//	  6 			   10.0 MHz 	6.6 MHz
	//	  5 			   12.0 MHz 	8.0 MHz

    Uint16 period = 2;	
    
    CLK_disableTbClockSync(myClk);
    CLK_enablePwmClock(myClk, PWM_Number_4);

             // Setup TBCLK
             //PWM_setPeriod(myPwm4, EPWM1_TIMER_TBPRD);   // Set timer period 801 TBCLKs
             //PWM_setPhase(myPwm4, 0x0000);               // Phase is 0
             //PWM_setCount(myPwm4, 0x0000);               // Clear counter

             PWM_setPeriodLoad(myPwm4, PWM_PeriodLoad_Immediate);
             PWM_setPeriod(myPwm4, period-1);    // Set timer period
//             PWM_setCmpA(myPwm4, period / 2);
             PWM_setCmpA(myPwm4, 1 );
//             PWM_setCmpAHr(myPwm4, (1 << 8));
//             PWM_setCmpB(myPwm4, period / 2);
             PWM_setCmpB(myPwm4, 0);

             // Set Compare values
            // PWM_setCmpA(myPwm4, EPWM1_MIN_CMPA);    // Set compare A value
            // PWM_setCmpB(myPwm4, EPWM1_MIN_CMPB);    // Set Compare B value

             // Setup counter mode
             PWM_setCounterMode(myPwm4, PWM_CounterMode_Up); // Count up
             PWM_disableCounterLoad(myPwm4);                     // Disable phase loading
             PWM_setHighSpeedClkDiv(myPwm4, PWM_HspClkDiv_by_1); // Clock ratio to SYSCLKOUT
             PWM_setClkDiv(myPwm4, PWM_ClkDiv_by_1);

             // Setup shadowing
             PWM_setShadowMode_CmpA(myPwm4, PWM_ShadowMode_Shadow);
             PWM_setShadowMode_CmpB(myPwm4, PWM_ShadowMode_Shadow);
             PWM_setLoadMode_CmpA(myPwm4, PWM_LoadMode_Zero);
             PWM_setLoadMode_CmpB(myPwm4, PWM_LoadMode_Zero);

#if 0
             // Set actions
             PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Set);      // Set PWM1A on event A, up count
        //     PWM_setActionQual_CntDown_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);  // Clear PWM1A on event A, down count

             PWM_setActionQual_CntUp_CmpB_PwmB(myPwm4, PWM_ActionQual_Clear);      // Set PWM1B on event B, up count
       //      PWM_setActionQual_CntDown_CmpB_PwmB(myPwm4, PWM_ActionQual_Clear);  // Clear PWM1B on event B, down count
#endif

#if 1

             PWM_setActionQual_CntUp_CmpA_PwmB(myPwm4, PWM_ActionQual_Toggle);      // Set PWM1A on event A, up count
             PWM_setActionQual_CntDown_CmpA_PwmB(myPwm4, PWM_ActionQual_Toggle);  // Clear PWM1A on event A, down count

             PWM_setActionQual_CntUp_CmpB_PwmB(myPwm4, PWM_ActionQual_Toggle);      // Set PWM1B on event B, up count
             PWM_setActionQual_CntDown_CmpB_PwmB(myPwm4, PWM_ActionQual_Toggle);  // Clear PWM1B on event B, down count
#endif

             CLK_enableTbClockSync(myClk);









  
   //start pulse
  
   printf("Init AIO10... \r\n");
  
  
   EALLOW;
  
   GpioCtrlRegs.AIOMUX1.bit.AIO10 = 0x0;	 // Dig.IO funct. applies to AIO2,4,6,10,12,14
   GpioCtrlRegs.AIODIR.bit.AIO10 = 0x1;
  
   EDIS;
   printf("Done: Init AIO10... \r\n");
  
   while(1){
  
   printf("Begin set AIO10 High... \r\n");
  
   EALLOW;
   GpioDataRegs.AIODAT.bit.AIO10 = 1;
   EDIS;
  
   printf("Done set AIO10 High... \r\n");
  
   printf("Begin Delay 1... \r\n");
  
   DELAY_US(1000000);
   printf("Done Delay 1... \r\n");
  
   printf("Begin set AIO10 LOW... \r\n");
  
   EALLOW;
   GpioDataRegs.AIODAT.bit.AIO10 = 0;
   EDIS;
   printf("Done set AIO10 LOW... \r\n");
  
   printf("Begin Delay 2... \r\n");
  
   DELAY_US(1000000);
   printf("Done Delay 2... \r\n");
  
  }
  
#endif
 	return 0;
}

#define PWM3_TIMER_MIN 10

void InitEPwmTimer()
{

    CLK_disableTbClockSync(myClk);
    CLK_enablePwmClock(myClk, PWM_Number_4);

    PWM_setCounterMode(myPwm4, PWM_CounterMode_Up);
    PWM_setPeriod(myPwm4, PWM3_TIMER_MIN);
    PWM_setPhase(myPwm4, 0x00000000);
    PWM_setActionQual_Period_PwmA(myPwm4, PWM_ActionQual_Toggle);

    // TBCLK = SYSCLKOUT
    PWM_setHighSpeedClkDiv(myPwm4, PWM_HspClkDiv_by_2);
    PWM_setClkDiv(myPwm4, PWM_ClkDiv_by_1);



    CLK_enableTbClockSync(myClk);

}

