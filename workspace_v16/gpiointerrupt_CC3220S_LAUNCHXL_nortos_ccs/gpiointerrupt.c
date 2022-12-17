/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h> // Including this header file to stop the warnings about snprintf

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);
#define TRUE    1
#define FALSE   0
#define GLOBAL_PERIOD   100
#define NUMBER_OF_TASKS  3

// Declaring our task functions
void task_one();
void task_two();
void task_three();

// Driver Handles - Global variables
Timer_Handle timer0;
int16_t setPointTemp = 24; // Setting the initial setPoint to 24 degrees C
int16_t temp; // Initializing temp variable

// Setting button variables to false indicating that they have not been pushed.
int button0 = FALSE;
int button1 = FALSE;

volatile unsigned char TimerFlag = FALSE;
// Setting global period to 100 ms to be used in initTimer()
int global_period = GLOBAL_PERIOD;
// Initializing heat variable
int heat = FALSE;
// Starting at 0 seconds
int seconds = 0;


// A single task in the task list.
struct task_entry
{
    char triggered; // Whether or not the task was triggered.
    void (*f)();   // Function to call to perform the task. (This is a function pointer)
    int elapsed_time; // Amount of time since last triggered.
    int period; // Period of task in ms.
};

// The task list (array of three tasks, NUMBER_OF_TASKS = 3).
struct task_entry tasks[NUMBER_OF_TASKS] =
{
     // Each task has four things: the "triggered" flag, a pointer to a function, the elapsed time,
     // and the period for the task.
     {FALSE, &task_one, 500, 500}, // task 1 checks temp every 500ms
     {FALSE, &task_two, 200, 200}, // task 2 checks buttons every 200ms
     {FALSE, &task_three, 1000, 1000} // task 3 sends output to UART every 1000ms
};


// When the global timer period is up this function gets called
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{

    int x = 0;
    // Looping through all three tasks.
    for (x = 0; x < NUMBER_OF_TASKS; x++)
    {
        // Check if task's interval has expired by checking to see if the task's elapsed time is
        // greater than or equal to the task's period.
        if (tasks[x].elapsed_time >= tasks[x].period)
        {
            tasks[x].triggered = TRUE;
            // Resetting global flag
            TimerFlag = TRUE;
            // Resetting the elapsed_time
            tasks[x].elapsed_time = 0;
        }
        else
        {
            // If task's elapsed time is less than the period, increment by the global period so
            // we can move on to the next task.
            tasks[x].elapsed_time += global_period;
        }
    }
}


void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}


// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;

    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // This function sets button 0 to having been pushed.
    button0 = TRUE;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // This function sets button 1 to having been pushed.
    button1 = TRUE;
}


/*
 * Defining our task functions below
 */

// Checks the temperature.
void task_one()
{
    // Perform processing for task one

    // Storing the read temperature in a variable.
    temp = readTemp();
    printf("Room temp is currently: %02d, setPointTemp is currently: %02d\n", temp, setPointTemp);

    // If the temperature is greater than the set point, turn hear off and turn LED off.
    if(temp >= setPointTemp)
    {
        heat = FALSE;
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
    // If the temperature is less than or equal to the set point, turn heat on and LED on.
    else
    {
        heat = TRUE;
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }
}

// Checks button presses, depending on which was pressed increment or decrement setPointTemp by 1.
void task_two()
{

    // Perform processing for task two
    if(button0 == TRUE)
    {
        setPointTemp += 1;
        button0 = FALSE;
        printf("setPointTemp: %02d\n", setPointTemp);
    }

    if(button1 == TRUE)
    {
        setPointTemp -= 1;
        button1 = FALSE;
        printf("setPointTemp: %02d\n", setPointTemp);
    };

}

// Report to server via UART
void task_three()
{
    // Outputs all relevant info to terminal
    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temp, setPointTemp, heat, seconds));
    ++seconds; // Increments seconds every second
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    // GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); // Commented out because we removed the LED 1 driver from sysconfig
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    // Loop until the inevitable heat-deth of the universe
    while(TRUE)
    {

        while (!TimerFlag){} // Wait for the timer period to elapse (nothing in here)

        int x = 0;
        // Looping through all three tasks.
        // Every 200ms check the button flags
        // Every 500ms read the temperature and update the LED
        // Every second output the following to the UART
        for (x = 0; x < NUMBER_OF_TASKS; x++)
        {
            if (tasks[x].triggered)
            {
                // Calling the task function
                tasks[x].f();
                // Resetting the task trigger.
                tasks[x].triggered = FALSE;
            }
        };
        // Reset everything (e.g flags) and go back to the beginning.
        TimerFlag = FALSE;
    }

}
