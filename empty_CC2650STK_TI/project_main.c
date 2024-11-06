/* C Standard library */
#include <stdio.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/i2c/I2CCC26XX.h>


#include "Board.h"
#include "sensors/mpu9250.h"
#include "Board.h"
#include "buzzer.h" // remember to add buzzer.h and buzzer.c into project directory!



/* Task */
#define STACKSIZE 2048
#define ACC_Z_THRESHOLD 0.4  // Threshold for upward movement
#define GYRO_X_THRESHOLD 150

// Global variable to store the last Z-axis value
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char morseCodeTaskStack[STACKSIZE];
Task_Handle morseCodeTaskHandle;
Task_Params morseCodeTaskParams;

// JTKJ: Teht�v� 3. Tilakoneen esittely
// JTKJ: Exercise 3. Definition of the state machine
enum state { WAITING=1, DATA_READY };
enum state programState = WAITING;

String morseString = "";

// JTKJ: Teht�v� 1. Lis�� painonappien RTOS-muuttujat ja alustus
// JTKJ: Exercise 1. Add pins RTOS-variables and configuration here
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};
// Pinnien alustukset, molemmille pinneille oma konfiguraatio
// Vakio BOARD_BUTTON_0 vastaa toista painonappia
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};


// Vakio Board_LED0 vastaa toista lediä
PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    // JTKJ: Teht�v� 1. Vilkuta jompaa kumpaa ledi�
    // JTKJ: Exercise 1. Blink either led of the device
    // Vaihdetaan led-pinnin tilaa negaatiolla
       uint_t pinValue = PIN_getOutputValue( Board_LED1 );
       pinValue = !pinValue;
       PIN_setOutputValue( ledHandle, Board_LED1, pinValue );
       System_printf(" \n");
       morseString = " ";
       programState = DATA_READY;

}

char messageBuffer[100];
int messageIndex = 0;
volatile bool processMorse = false;

void uartReadCallback(UART_Handle handle, void *buf, size_t count) {
    if (count > 0) {
        char receivedByte = ((char *)buf)[0];

        // Append the received byte to messageBuffer
        if (messageIndex < sizeof(messageBuffer) - 1) {
            messageBuffer[messageIndex++] = receivedByte;
        }

        // Check if this byte is the end of a message (e.g., '\n' or '\r')
        if (receivedByte == '\n') {
            messageBuffer[messageIndex] = '\0';  // Null-terminate the message
            System_printf("Complete message received: %s\n", messageBuffer);
            System_flush();

            // Set flag for Morse code processing
            processMorse = true;

            // Reset message index for the next message
            messageIndex = 0;
        }

        // Restart reading for the next byte
        UART_read(handle, buf, 1);  // Read one byte at a time
    }
}
void morseCodeTask(UArg arg0, UArg arg1) {

    while (1) {
        // Wait until processMorse is set by uartReadCallback
        if (processMorse) {
            // Clear flag to avoid repeated processing
            processMorse = false;

            // Open the buzzer for Morse code output
            buzzerOpen(hBuzzer);
            int i = 0;
            // Process each character in messageBuffer for beeping
            for (i = 0; messageBuffer[i] != '\0'; i++) {
                if (messageBuffer[i] == '.') {
                    // Short beep for dot
                    //System_printf("xxxxxxxxxxxxxxxxxxxxxxx\n");
                    buzzerSetFrequency(2000);  // Set frequency for beep
                    Task_sleep(50000 / Clock_tickPeriod);  // Short beep duration
                    buzzerSetFrequency(0);  // Turn off buzzer
                    Task_sleep(50000 / Clock_tickPeriod);  // Short silence after dot
                } else if (messageBuffer[i] == '-') {
                    // Long beep for dash
                    buzzerSetFrequency(2000);  // Set frequency for beep
                    Task_sleep(500000 / Clock_tickPeriod);  // Long beep duration
                    buzzerSetFrequency(0);  // Turn off buzzer
                    Task_sleep(50000 / Clock_tickPeriod);  // Short silence after dash
                } else if (messageBuffer[i] == ' ') {
                    // Longer pause between words
                    Task_sleep(300000 / Clock_tickPeriod);  // Silence between words
                }
            }

            // Close the buzzer after completing the message
            buzzerClose();
        }

        // Sleep briefly to yield CPU (optional)
        Task_sleep(10000);  // Adjust as needed for system requirements
    }
}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {
    char uartWriteBuffer[100];
    char uartReadBuffer[1];  // Single-byte buffer for reading
    UART_Handle handle;
    UART_Params params;

    UART_Params_init(&params);
    params.baudRate = 9600;
    params.dataLength = UART_LEN_8;
    params.parityType = UART_PAR_NONE;
    params.stopBits = UART_STOP_ONE;
    params.readDataMode = UART_DATA_TEXT;
    params.writeDataMode = UART_DATA_TEXT;
    params.readEcho = UART_ECHO_OFF;

    // Set read mode to non-blocking callback
    params.readMode = UART_MODE_CALLBACK;
    params.readCallback = uartReadCallback;

    handle = UART_open(Board_UART, &params);
    if (handle == NULL) {
        System_abort("Error opening the UART");
    }

    // Start reading one byte at a time
    UART_read(handle, uartReadBuffer, 1);

    while (1) {
        if (programState == DATA_READY) {
            snprintf(uartWriteBuffer, sizeof(uartWriteBuffer), "%s\r\n\0", morseString);
            System_printf("Sending to UART: %s\n", uartWriteBuffer);
            System_flush();

            UART_write(handle, uartWriteBuffer, strlen(uartWriteBuffer)+1);
          /* if (UART_write(handle, uartWriteBuffer, strlen(uartWriteBuffer)) != UART_STATUS_SUCCESS) {
                            System_printf("UART write failed\n");
                        } else {
                            System_printf("UART write succeeded\n");
                        }*/
            programState = WAITING;
        }

        Task_sleep(1000000 / Clock_tickPeriod);  // Control loop frequency
    }
}

Void sensorTaskFxn(UArg arg0, UArg arg1) {

    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;



    // JTKJ: Teht�v� 2. Avaa i2c-v�yl� taskin k�ytt��n
    // JTKJ: Exercise 2. Open the i2c bus
    I2C_Params_init(&i2cMPUParams);
        i2cMPUParams.bitRate = I2C_400kHz;
        // Note the different configuration below
        i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // Avataan yhteys
        PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

           // Wait 100ms for the MPU sensor to power up
           Task_sleep(100000 / Clock_tickPeriod);
           System_printf("MPU9250: Power ON\n");
           System_flush();

           // MPU open i2c
           i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
           if (i2cMPU == NULL) {
               System_abort("Error Initializing I2CMPU\n");
           }
           System_printf("MPU9250: Setup and calibration...\n");

           System_flush();
    mpu9250_setup(&i2cMPU);
    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();
    float ax, ay, az, gx, gy, gz;
    char buffer[150];
    uint32_t timestamp;
   // System_printf("time,   acc_x,      acc_y,     acc_z,     gyro_x,   gyro_y,   gyro_z\n");
    while (1) {

        // JTKJ: Teht�v� 2. Lue sensorilta dataa ja tulosta se Debug-ikkunaan merkkijonona
        // JTKJ: Exercise 2. Read sensor data and print it to the Debug window as string
        timestamp = Clock_getTicks() * Clock_tickPeriod / 1000;
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
        if ((az - 1 > ACC_Z_THRESHOLD) || (az + 1 < -ACC_Z_THRESHOLD)) {
            morseString = ".";
            System_printf(".\n");
            Task_sleep(500000 / Clock_tickPeriod);

            System_flush();

            programState = DATA_READY;
        }
        else if ((abs(gx) > GYRO_X_THRESHOLD)) {
            morseString = "-";
            System_printf("-\n", timestamp);
            Task_sleep(500000 / Clock_tickPeriod);

            System_flush();

            programState = DATA_READY;
        }




       /* sprintf(buffer, ",%-6u  ,%-9.4f  ,%-9.4f  ,%-9.4f  ,%-8.4f  ,%-8.4f  ,%-8.4f\n",
                        timestamp, ax, ay, az, gx, gy, gz);
        System_printf("%s", buffer);
*/
        // JTKJ: Teht�v� 3. Tallenna mittausarvo globaaliin muuttujaan
        //       Muista tilamuutos
        // JTKJ: Exercise 3. Save the sensor value into the global variable
        //       Remember to modify state
        //programState = DATA_READY;
        //System_printf("State changed to DATA_READY\n");
        // Just for sanity check for exercise, you can comment this out
        //System_printf("sensorTask\n");
        System_flush();


        Task_sleep(200000 / Clock_tickPeriod);
    }
}

Int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Initialize board
    Board_initGeneral();
    
    //Otetaan buzzer käyttöön ohjelmassa
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
      if (hBuzzer == NULL) {
        System_abort("Pin open failed!");
      }

    // JTKJ: Teht�v� 2. Ota i2c-v�yl� k�ytt��n ohjelmassa
    // JTKJ: Exercise 2. Initialize i2c bus
    Board_initI2C();

    // JTKJ: Teht�v� 4. Ota UART k�ytt��n ohjelmassa
    // JTKJ: Exercise 4. Initialize UART
    Board_initUART();
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
        if (hMpuPin == NULL) {
            System_abort("Pin open failed!");
        }

    // JTKJ: Teht�v� 1. Ota painonappi ja ledi ohjelman k�ytt��n
    //       Muista rekister�id� keskeytyksen k�sittelij� painonapille
    // JTKJ: Exercise 1. Open the button and led pins
    //       Remember to register the above interrupt handler for button
    // Otetaan pinnit käyttöön ohjelmassa
       buttonHandle = PIN_open(&buttonState, buttonConfig);
       if(!buttonHandle) {
          System_abort("Error initializing button pins\n");
       }
       ledHandle = PIN_open(&ledState, ledConfig);
       if(!ledHandle) {
          System_abort("Error initializing LED pins\n");
       }

       // Asetetaan painonappi-pinnille keskeytyksen käsittelijäksi
       // funktio buttonFxn
       if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
          System_abort("Error registering button callback function");
       }

    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    /* Create the Morse code task */
        Task_Params_init(&morseCodeTaskParams);
        morseCodeTaskParams.stackSize = STACKSIZE;
        morseCodeTaskParams.stack = &morseCodeTaskStack;
        morseCodeTaskParams.priority = 2;  // Set appropriate priority
        morseCodeTaskHandle = Task_create(morseCodeTask, &morseCodeTaskParams, NULL);
        if (morseCodeTaskHandle == NULL) {
            System_abort("Task create failed!");
        }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
