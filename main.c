#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"


#define MPU9150_I2C_ADDRESS     0x68

tI2CMInstance g_sI2CInst; // Global instance structure for the I2C master driver.
tMPU9150 g_sMPU9150Inst; // Global instance structure for the ISL29023 sensor driver.

volatile uint_fast8_t g_vui8I2CDoneFlag; // Global flags to alert main that MPU9150 I2C transaction is complete
volatile uint_fast8_t g_vui8ErrorFlag;  // Global flags to alert main that MPU9150 I2C transaction error has occurred.
volatile uint_fast8_t g_vui8DataFlag;  // Global flags to alert main that MPU9150 data is ready to be retrieved.


//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


//*****************************************************************************
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//*****************************************************************************
void MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    if(ui8Status == I2CM_STATUS_SUCCESS)
        g_vui8I2CDoneFlag = 1;

    // Store the most recent status in case it was an error condition
    g_vui8ErrorFlag = ui8Status;
}


//*****************************************************************************
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//*****************************************************************************
void IntGPIOb(void)
{
    unsigned long ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    // Clear all the pin interrupts that are set
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
        // MPU9150 Data is ready for retrieval and processing.
    	// OR read the register MOT_DETECT_STATUS
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
}


//*****************************************************************************
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//*****************************************************************************
void MPU9150I2CIntHandler(void)
{
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    I2CMIntHandler(&g_sI2CInst);
}


//*****************************************************************************
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//*****************************************************************************
void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    while(1);
}


//*****************************************************************************
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//*****************************************************************************
void MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0));

    // If an error occurred call the error handler immediately.
    if(g_vui8ErrorFlag)
        MPU9150AppErrorHandler(pcFilename, ui32Line);

    // clear the data flag for next use.
    g_vui8I2CDoneFlag = 0;
}


//*****************************************************************************
// Configure the I2C and its pins.  This better be called before MPU9512
//*****************************************************************************
void ConfigureI2C(void)
{
	// The I2C3 peripheral must be enabled before use.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
}


//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureMPU9150Interrupt(void)
{
	// Configure and Enable the GPIO interrupt. Used for INT signal from the
	// MPU9150
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	ROM_IntEnable(INT_GPIOB);
}


//*****************************************************************************
//*****************************************************************************
void SetUpPeripheralsInterrupts(void)
{
	// Keep only some parts of the systems running while in sleep mode.
	// GPIOB is for the MPU9150 interrupt pin.
	// I2C3 is the I2C interface to the ISL29023
	ROM_SysCtlPeripheralClockGating(true);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);

	// Enable interrupts to the processor.
	ROM_IntMasterEnable();
}


//******************************************************************************
// Initialize all sensors inputs such as filters and settings, very specific
// to the appliation, rather than the generic initalization in the Configure fcn
//******************************************************************************
void InitializeMPU9150(void)
{
	// Initialize the MPU9150 Driver.
	MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS, MPU9150AppCallback, &g_sMPU9150Inst);
	// Wait for transaction to complete
	MPU9150AppI2CWait(__FILE__, __LINE__);

    // Modifying the ranges of the sensors
    // look up MPU9150Init function in mpu9150.C to configure this once for all
    MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_ACCEL_CONFIG, ~MPU9150_ACCEL_CONFIG_AFS_SEL_M, MPU9150_ACCEL_CONFIG_AFS_SEL_16G, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
	MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M, MPU9150_GYRO_CONFIG_FS_SEL_2000, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
}


//************************************************************************************************************
// Custom Function as a shorter version instead of repeating it everytime we wanna modify the resiter and wait
//************************************************************************************************************
void Mpu9150ModifyRegister(uint_fast8_t registerAddress, uint_fast8_t ANDedValue, uint_fast8_t ORedValue)
{
	MPU9150ReadModifyWrite(&g_sMPU9150Inst, registerAddress, ANDedValue, ORedValue, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
}


//******************************************************************************
// Set the acceleration threshold at which the interrupt will fire from any axis
//******************************************************************************
void SetMotionInterruptAt(float accelerationThresholdMeterPerSec2, uint8_t motionDurationInMs)
{
	// make sure the accelerometer is not in sleep or standby mode, set the power management registers
	// reset the sleep and cycle bits
	Mpu9150ModifyRegister(MPU9150_O_PWR_MGMT_1, !(MPU9150_PWR_MGMT_1_SLEEP | MPU9150_PWR_MGMT_1_CYCLE), 0);
	// reset all the standby bits for all accelerometer axis
	Mpu9150ModifyRegister(MPU9150_O_PWR_MGMT_2, !(MPU9150_PWR_MGMT_2_STBY_XA | MPU9150_PWR_MGMT_2_STBY_YA | MPU9150_PWR_MGMT_2_STBY_ZA), 0);

	// Set Accel HPF to reset settings (by resetting the first 3 bits)
	Mpu9150ModifyRegister(MPU9150_O_ACCEL_CONFIG, 0b11111000, 0);

	// Set Accel LPF settings to 256 Bandwidth
	Mpu9150ModifyRegister(MPU9150_O_CONFIG, 0b11111000, 0);

	// Set the interrupt register to wake up on motion
	Mpu9150ModifyRegister(MPU9150_O_INT_ENABLE, 0, MPU9150_INT_ENABLE_MOT_EN);

	// Set how many millisecond the value should be above the threshhold for the interrupt to fire
	Mpu9150ModifyRegister(MPU9150_O_MOT_DUR, 0, motionDurationInMs);

	// Get the threshold value: 1LSB = 32mg
	accelerationThresholdMeterPerSec2 /= 9.81; // convert to X = Yg
	uint8_t accVal = (uint8_t)(accelerationThresholdMeterPerSec2 * 1000 / 32); // convert Yg to Z LSB;
	// write the value to the threshold register
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_MOT_THR, &accVal, 1, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);

	// wait for 2 ms delay for accumulation of samples (as written in PS)
	ROM_SysCtlDelay(ROM_SysCtlClockGet() / (2));

	// Set the acceleration high pass frequency to HOLD, it will hold the initial value
	Mpu9150ModifyRegister(MPU9150_O_ACCEL_CONFIG, 0xFF, MPU9150_ACCEL_CONFIG_ACCEL_HPF_HOLD);

	// Set the frequency of the wakeup and set the Gyro to standby
	Mpu9150ModifyRegister(MPU9150_O_PWR_MGMT_2, 0x00, (MPU9150_PWR_MGMT_2_LP_WAKE_CTRL_40 | (MPU9150_PWR_MGMT_2_STBY_XG | MPU9150_PWR_MGMT_2_STBY_YG | MPU9150_PWR_MGMT_2_STBY_ZG)));

	// Enable Cycle Mode (Low Power Mode)
	Mpu9150ModifyRegister(MPU9150_O_PWR_MGMT_1, !(MPU9150_PWR_MGMT_1_SLEEP), MPU9150_PWR_MGMT_1_CYCLE);
}



//*****************************************************************************
// Main application entry point.
//*****************************************************************************
int main(void)
{

    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable port B used for motion interrupt.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    ConfigureI2C();
    ConfigureMPU9150Interrupt();
    SetUpPeripheralsInterrupts();

    // Initialize I2C3 peripheral.
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, ROM_SysCtlClockGet());
    InitializeMPU9150();

    // The main reason why this project exists
    // it assumes a gravity of 9.81 m/sec^2 for 1 ms,
    SetMotionInterruptAt(9.81,1);

	while(1);
}


