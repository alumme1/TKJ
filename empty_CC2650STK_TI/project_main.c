/*
 * JTK-harjoitustyö
 * Petri Siira: Suunnittelu, toteutus ja testaus
 * Atte Lumme: Suunnittelu, toteutus ja testaus*
 */

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
#include "buzzer.h"



#define STACKSIZE 2048 //SPinon koko
#define ACC_Z_THRESHOLD 0.2  // Kiihtyvyysanturin Z-akselin kynnysarvo ylöspäin liikkeelle
#define GYRO_X_THRESHOLD 150 // Gyroskoopin X-akselin kynnysarvo

// Globaalit muuttujat eri taskien pinoille
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char morseCodeTaskStack[STACKSIZE];
Char ledFlashTaskStack[STACKSIZE];


//Tilakoneeseen liittyvät tilat
enum state { WAITING=1, DATA_READY, MESSAGE_RECEIVED};
enum state programState = WAITING;

// Merkkimuuttuja tallentamaan Morse-merkkejä
char morseChar;

// Painonapin ja LEDin RTOS-muuttujat ja konfiguraatio
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
// MPU9250:n I2C-konfiguraatio
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};
// Painonapin konfiguraatiot
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};


// LEDin konfiguraatiot
PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};
// Summerin konfiguraatiot
static PIN_Handle buzzerHandle;
static PIN_State buzzerState;
PIN_Config buzzerConfig[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};


/* Painonapin keskeytyskäsittelijä */
void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    System_printf("Button pressed\n");
    System_flush();

    morseChar = ' ';
    programState = DATA_READY;  // Vaihdetaan ohjelman tila valmiiksi datan lähetykselle ja ledin väläytykselle
}



/* LED-välähdystaskin toteutus. Ledi välähtää aina jos laite on vastaanottanut komennon */
Void ledFlashTask(UArg arg0, UArg arg1) {
    while (1) {
        if (programState == DATA_READY) {

            // Sytytä LED
            PIN_setOutputValue(ledHandle, Board_LED1, 1);
            Task_sleep(10000 / Clock_tickPeriod);

            // Sammuta LED
            PIN_setOutputValue(ledHandle, Board_LED1, 0);
        }

        Task_sleep(10000 / Clock_tickPeriod);
    }
}

char receivedByte;  // Muuttuja vastaanotetulle tavulle
char messageBuffer[1]; //Puskuri merkeille

void uart_ReadCallback(UART_Handle handle, size_t count) {
    if (count > 0) {
        if (receivedByte != '\r') {
                if (receivedByte != '\n') {
                    // Varastoidaan merkki bufferiin
                    messageBuffer[0] = receivedByte;
                    programState = MESSAGE_RECEIVED;  // Vaihdetaan tila merkin käsittelyä varten
                }
            }

        UART_read(handle, &receivedByte, 1);  // Aloita uuden merkin lukeminen
    }
}

/* Morse-kooditaskin toteutus, jossa viesti toistetaan laitteella */
void morseCodeTask(UArg arg0, UArg arg1) {

    while (1) {
        // Odota kunnes tila on asetettu uart_ReadCallback:ssä
        if (programState == MESSAGE_RECEIVED) {
            // Asetaan tila odotukseski estääksesi uudelleenkäsittelyn
            programState = WAITING;
            if (messageBuffer[0] == '.') {
                // Lyhyt piippaus pisteelle
                buzzerSetFrequency(2000);
                Task_sleep(50000 / Clock_tickPeriod);
                buzzerSetFrequency(0);
                Task_sleep(50000 / Clock_tickPeriod);
            } else if (messageBuffer[0] == '-') {
                // Pitkä piippaus viivalle
                buzzerSetFrequency(2000);
                Task_sleep(500000 / Clock_tickPeriod);
                buzzerSetFrequency(0);
                Task_sleep(50000 / Clock_tickPeriod);
            } else if (messageBuffer[0] == ' ') {
                // Tauko merkkien välissä
                Task_sleep(300000 / Clock_tickPeriod);
            }
        }
        Task_sleep(10000);
    }
}

/* UART-taskin toteutus */
Void uartTaskFxn(UArg arg0, UArg arg1) {
    char uartWriteBuffer[4];
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
    params.readMode = UART_MODE_CALLBACK;
    params.readCallback = uart_ReadCallback;

    handle = UART_open(Board_UART, &params);
    if (handle == NULL) {
        System_abort("Error opening the UART");
    }

    // Aloita yhden tavun lukeminen kerrallaan
    UART_read(handle, &receivedByte, 1);

    while (1) {
        if (programState == DATA_READY) {
            snprintf(uartWriteBuffer, sizeof(uartWriteBuffer), "%c\r\n\0", morseChar);;
            System_printf("Lähetetään uart:iin %s", uartWriteBuffer);
            System_flush();

            UART_write(handle, uartWriteBuffer, strlen(uartWriteBuffer)+1);


            programState = WAITING;
        }

        Task_sleep(1000000 / Clock_tickPeriod);
    }
}
/* Sensor-taskin toteutus */
Void sensorTaskFxn(UArg arg0, UArg arg1) {

    I2C_Handle i2cMPU; // Oma I2C-yhteys MPU9250-anturia varten
    I2C_Params i2cMPUParams;

    // Alusta I2C-väylä tehtävän käyttöön
    I2C_Params_init(&i2cMPUParams);
        i2cMPUParams.bitRate = I2C_400kHz;
        i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // Avataan yhteys
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Odota 100 ms, että MPU-anturi käynnistyy
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU:n avaaminen I2C:lle
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);
    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();
    // Muuttujat kiihtyvyys- ja gyroskooppiarvoille
    float ax, ay, az, gx, gy, gz;

    while (1) {

        // Lue data MPU9250-sensorilta
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);


        // Tarkista Z-akselin kiihtyvyysarvo ylöspäin nykäisyn varalle
        if ((az - 1 > ACC_Z_THRESHOLD) || (az + 1 < -ACC_Z_THRESHOLD)) {
            morseChar = '.';
            // Päivitä ohjelman tila valmiiksi ledin välähdykselle ja merkin lähettämiselle
            programState = DATA_READY;
            // Toista lyhyt piippaus pisteelle
            buzzerSetFrequency(2000);
            Task_sleep(50000 / Clock_tickPeriod);
            buzzerSetFrequency(0);



            // Odota ennen seuraavaa tarkistusta (50ms + 400ms= 450 ms)
            Task_sleep(400000 / Clock_tickPeriod);
        }
        // Tarkista gyroskoopin X-akselin arvo laitteen kääntöliikettä varten
        else if ((abs(gx) > GYRO_X_THRESHOLD)) {
            morseChar = '-';
            // Päivitä ohjelman tila valmiiksi ledin välähdykselle ja merkin lähettämiselle
            programState = DATA_READY;
            // Toista pitkä piippaus viivalle
            buzzerSetFrequency(2000);
            Task_sleep(400000 / Clock_tickPeriod);
            buzzerSetFrequency(0);



            // Odota ennen seuraavaa tarkistusta (400ms + 50ms= 450 ms)
            Task_sleep(50000 / Clock_tickPeriod);
        }

        Task_sleep(200000 / Clock_tickPeriod);
    }
}

Int main(void) {

    // Taskien muuttujat
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle morseCodeTaskHandle;
    Task_Params morseCodeTaskParams;
    Task_Handle ledFlashTaskHandle;
    Task_Params ledFlashTaskParams;


    // Initialize board
    Board_initGeneral();
    
    //Otetaan buzzer käyttöön ohjelmassa ja avataan se
    buzzerHandle = PIN_open(&buzzerState, buzzerConfig);
        if (buzzerHandle == NULL) {
            System_abort("Pin open failed!");
          }
          buzzerOpen(buzzerHandle);

    // Initializing i2c bus
    Board_initI2C();

    // Initializing UART
    Board_initUART();
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
        if (hMpuPin == NULL) {
            System_abort("Pin open failed!");
        }


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

    /* Alustetaan sensorin tehtävän parametrit */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    /* Alustetaan UART-tehtävän parametrit */
    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    /* Luodaan Morse-koodin käsittelyyn liittyvä tehtävä */
    Task_Params_init(&morseCodeTaskParams);
    morseCodeTaskParams.stackSize = STACKSIZE;
    morseCodeTaskParams.stack = &morseCodeTaskStack;
    morseCodeTaskParams.priority = 2;
    morseCodeTaskHandle = Task_create(morseCodeTask, &morseCodeTaskParams, NULL);
    if (morseCodeTaskHandle == NULL) {
        System_abort("Task create failed!");
    }
    /* Luodaan LED:n vilkutukseen liittyvä tehtävä */
    Task_Params_init(&ledFlashTaskParams);
    ledFlashTaskParams.stackSize = STACKSIZE;
    ledFlashTaskParams.stack = &ledFlashTaskStack;
    ledFlashTaskParams.priority = 1;
    ledFlashTaskHandle = Task_create(ledFlashTask, &ledFlashTaskParams, NULL);
    if (ledFlashTaskHandle == NULL) {
        System_abort("LED Flash Task create failed!");
    }


    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
