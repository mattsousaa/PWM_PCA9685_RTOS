/**
 *  \file   main_test.c
 *
 *  \brief  Example application main file. This application will read the data
 *          from eeprom and compares it with the known data.
 *
 */

/*
 * Copyright (C) 2014 - 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdio.h>
#include <string.h>
#include <math.h>

/* TI-RTOS Header files */
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include "I2C_log.h"
#include "I2C_board.h"

#if defined (SOC_AM335X) || defined (SOC_AM437x)
/* EEPROM data -Board specific */
extern char eepromData[I2C_EEPROM_RX_LENGTH];
#endif

//bool *blabla = 0;

const unsigned CheckIfSigned[33]=
{0x00000000,
0x00000001,0x00000002,0x00000004,0x00000008,
0x00000010,0x00000020,0x00000040,0x00000080,
0x00000100,0x00000200,0x00000400,0x00000800,
0x00001000,0x00002000,0x00004000,0x00008000,
0x00010000,0x00020000,0x00040000,0x00080000,
0x00100000,0x00200000,0x00400000,0x00800000,
0x01000000,0x02000000,0x04000000,0x08000000,
0x10000000,0x20000000,0x40000000,0x80000000};

const unsigned ConvertToSigned[32]=
{0xffffffff,
0xfffffffe,0xfffffffc,0xfffffff8,0xfffffff0,
0xffffffe0,0xffffffc0,0xffffff80,0xffffff00,
0xfffffe00,0xfffffc00,0xfffff800,0xfffff000,
0xffffe000,0xffffc000,0xffff8000,0xffff0000,
0xfffe0000,0xfffc0000,0xfff80000,0xfff00000,
0xffe00000,0xffc00000,0xff800000,0xff000000,
0xfe000000,0xfc000000,0xf8000000,0xf0000000,
0xe0000000,0xc0000000,0x80000000};

const unsigned digits2bits[33]=
{0x00000000,
0x00000001,0x00000003,0x00000007,0x0000000f,
0x0000001f,0x0000003f,0x0000007f,0x000000ff,
0x000001ff,0x000003ff,0x000007ff,0x00000fff,
0x00001fff,0x00003fff,0x00007fff,0x0000ffff,
0x0001ffff,0x0003ffff,0x0007ffff,0x000fffff,
0x001fffff,0x003fffff,0x007fffff,0x00ffffff,
0x01ffffff,0x03ffffff,0x07ffffff,0x0fffffff,
0x1fffffff,0x3fffffff,0x7fffffff,0xffffffff};

///////////////////////FTOA/////////////////////////////

void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
int intToStr(int x, char str[], int d, int isNegative)
{
    int i = 0;
    if(x<0)
    {
        x=-x;
    }
    if(x==0)
    {
        str[i++] = '0';
    }
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    if(isNegative)
        str[i++] = '-';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
    float fpart = 0.0;
    int isNegative = 0;

    // Extract integer part
    int ipart = (int)n;


    if(n<0.0)
    {
        // Extract floating part
        fpart = -(n - (float)ipart);
        isNegative = 1;
    }
    else
    {
        // Extract floating part
        fpart = n - (float)ipart;
    }
    // convert integer part to string
    int i = intToStr(ipart, res, 0, isNegative);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint,0);
    }
}

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

#define I2C_TRANSACTION_TIMEOUT         (10000U)


/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/



float convert_raw_to_float (const unsigned source, // dado concatenado
                            const unsigned digits, // 16
                            bool * validation,  // NULL
                            float LowRange,     // -2000
                            float HighRange,    // +2000
                            const float scale,  // 1/2000
                            const float offset) // 0
{
    int tempInt;
    float tempFloat;
    unsigned temp;
    temp = source & CheckIfSigned[digits];
    if (temp != 0)
        temp = (source | ConvertToSigned[digits-1]);
    else
        temp=source;
    memcpy (&tempInt, &temp, sizeof (unsigned));
    tempFloat= (float) tempInt;
    tempFloat=tempFloat*scale + offset;
    if (tempFloat >= LowRange && tempFloat <= HighRange)
        *validation= TRUE;
    else
        *validation = FALSE;
    return tempFloat;
}


/* Data compare function */
bool CompareData(char *expData, char *rxData, unsigned int length);

#if defined (SOC_AM335X) || defined (SOC_AM437x) || defined (SOC_AM571x) || defined (SOC_AM572x) || defined (SOC_AM574x)
/* Probe and runtime bus frequnecy configuration test */
static bool I2C_Probe_BusFrequency_test(I2C_Handle handle);
static bool I2C_timeout_test(I2C_Handle handle);
#endif
/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/*The accelerometer and gyroscope data are 16 bits wide and so data from each axis uses two registers.*/

char deviceADDR  	= 0x40;        /* Chip Adress. */
char MODE1		= 0x00;        /* To wake up the MPU6050. */
char LED0_ON_L		= 0x06;
char LED0_ON_H		= 0x07;
char LED0_OFF_L		= 0x08;
char LED0_OFF_H		= 0x09;
char ALL_LED0_ON_L	= 0xFA;
char ALL_LED0_ON_H	= 0xFB;
char ALL_LED0_OFF_L	= 0xFC;
char ALL_LED0_OFF_H	= 0xFD;
char PRE_SCALE		= 0xFE;	       					//prescale_value = (osc_closk/(4096*up_rate)) - 1

/*
 *  ======== Board_initI2C ========
 */
bool Board_initI2C(void)
{
    Board_initCfg boardCfg;
    Board_STATUS  boardStatus;
#if defined (idkAM571x)
    Board_IDInfo  id;
#endif
    I2C_HwAttrs   i2c_cfg;
#if defined (evmK2G)
    Board_SoCInfo socInfo;
#endif

    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);

    /* Modify the default I2C configurations if necessary */

    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    boardStatus = Board_init(boardCfg);
    if (boardStatus != BOARD_SOK)
    {
        return (false);
    }


    return (true);
}
//
//void set(char reg1, char reg2, char valor){
//
//}

void AppDelay(int x);


int16_t write_sensor(char address, char reg, char value, I2C_Transaction i2cTransaction, I2C_Handle handle){


    char txBuf[2];
    char rxBuf;

    txBuf[0] = reg;
    txBuf[1] = value;

    i2cTransaction.slaveAddress = address;
    i2cTransaction.writeBuf = txBuf;
    i2cTransaction.writeCount = sizeof(txBuf);
    i2cTransaction.readBuf = &rxBuf;
    i2cTransaction.readCount = sizeof(rxBuf);
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;

    return I2C_transfer(handle, &i2cTransaction);

}

int8_t read_sensor(char address, char reg, I2C_Transaction i2cTransaction, I2C_Handle handle){

    char txBuf;
    char rxBuf;

    txBuf = reg;

    i2cTransaction.slaveAddress = address;
    i2cTransaction.writeBuf = &txBuf;
    i2cTransaction.writeCount = sizeof(txBuf);
    i2cTransaction.readBuf = &rxBuf;
    i2cTransaction.readCount = sizeof(rxBuf);
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;

    I2C_transfer(handle, &i2cTransaction);
    return rxBuf;

}

/*
 *  ======== test function ========
 */

bool teste;

/* Variáveis Giroscópio */

float recebe_float1;
float recebe_float2;
float recebe_float3;

char vet1[10];
char vet2[10];
char vet3[10];

/* Variáveis Acelerômetro */

float recebe_float4;
float recebe_float5;
float recebe_float6;

char vet4[10];
char vet5[10];
char vet6[10];

void i2c_test(UArg arg0, UArg arg1){


    I2C_Transaction i2cTransaction;
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_init();

    int8_t rxBuf;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(1, &i2cParams);


    while(1){

	write_sensor(deviceADDR, MODE1, 0x10, i2cTransaction, handle);
        AppDelay(2);
	//Sleep Mode
    	write_sensor(deviceADDR, PRE_SCALE, 0x78, i2cTransaction, handle); 
	//Freq 50 Hz   
   	write_sensor(deviceADDR, LED0_ON_H, 0x00, i2cTransaction, handle);
    	write_sensor(deviceADDR, LED0_ON_L, 0x00, i2cTransaction, handle);
	//Start LED0_on no tick 0
    	write_sensor(deviceADDR, LED0_OFF_H, 0x01, i2cTransaction, handle);
    	write_sensor(deviceADDR, LED0_OFF_L, 0x99, i2cTransaction, handle);
	//Start LED0_off em 10% do periodo, ou seja DutyCycle de 10% - 2ms
    	write_sensor(deviceADDR, MODE1, 0x00, i2cTransaction, handle);
	//Wake module
        AppDelay(20);
	//Giro 90º para a direita

	write_sensor(deviceADDR, MODE1, 0x10, i2cTransaction, handle);
        AppDelay(2);
	//Sleep Mode
    	write_sensor(deviceADDR, PRE_SCALE, 0x78, i2cTransaction, handle); 
	//Freq 50 Hz - Periodo 20ms
   	write_sensor(deviceADDR, LED0_ON_H, 0x00, i2cTransaction, handle);
    	write_sensor(deviceADDR, LED0_ON_L, 0x00, i2cTransaction, handle);
	//Start LED0_on no tick 0
    	write_sensor(deviceADDR, LED0_OFF_H, 0x00, i2cTransaction, handle);
    	write_sensor(deviceADDR, LED0_OFF_L, 0xCC, i2cTransaction, handle);
	//Start LED0_off em 5% do periodo, ou seja DutyCycle de 5% - 1ms
    	write_sensor(deviceADDR, MODE1, 0x00, i2cTransaction, handle);
	//Wake module
        AppDelay(20);
	//Giro 90º para a esquerda

/************************************* Gyroscope Information *********************************************/
    }

    I2C_close(handle);

    while (1) {

    }
}

/*
 *  ======== main ========
 */
int main(void){


    if (Board_initI2C() == false)
    {
        return (0);
    }

    UART_printf("Cheguei aqui, carais! \n");

#if defined (SOC_AM335X) || defined (SOC_AM437x) || defined (SOC_OMAPL137)
    Task_Handle task;
    Error_Block eb;

    Error_init(&eb);

    task = Task_create(i2c_test, NULL, &eb);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }
#endif

    /* Start BIOS */
    BIOS_start();
    return (0);
}

/*
 *  ======== CompareData ========
 */
bool CompareData(char *expData, char *rxData, unsigned int length)
{
    uint32_t idx = 0;
    uint32_t match = 1;
    bool retVal = false;

    for(idx = 0; ((idx < length) && (match != 0)); idx++)
    {
        if(*expData != *rxData) match = 0;
        expData++;
        rxData++;
    }

    if(match == 1) retVal = true;

    return retVal;
}


#if defined (SOC_AM335X) || defined (SOC_AM437x) || defined (SOC_AM571x) || defined (SOC_AM572x) || defined (SOC_AM574x)
static bool I2C_Probe_BusFrequency_test(I2C_Handle handle)
{
    uint32_t busFrequency;
    bool status = false;
    int16_t transferStatus;
    I2C_Transaction i2cTransaction;
    uint32_t slaveAddress;
    int32_t controlStatus;
    char txBuf[I2C_EEPROM_TEST_LENGTH + I2C_EEPROM_ADDR_SIZE] = {0x00, };
    char rxBuf[I2C_EEPROM_TEST_LENGTH];
    uint32_t delayValue;

    /* Set the I2C EEPROM write/read address */
    txBuf[0] = (I2C_EEPROM_TEST_ADDR >> 8) & 0xff; /* EEPROM memory high address byte */
    txBuf[1] = I2C_EEPROM_TEST_ADDR & 0xff;        /* EEPROM memory low address byte */


    /* Test Runtime Configuration of Bus Frequency */

    /* Test runtime configuration of 400 kHz */
    busFrequency = I2C_400kHz;
    I2C_control(handle, I2C_CMD_SET_BUS_FREQUENCY, &busFrequency);

    memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
    i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
    i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
    transferStatus = I2C_transfer(handle, &i2cTransaction);

    if(I2C_STS_SUCCESS != transferStatus)
    {
        I2C_log("\n I2C Test: Dynamic configuration of bus Freq failed. \n");
    }

    status = CompareData(&eepromData[0], &rxBuf[0], I2C_EEPROM_TEST_LENGTH);

    if(true == status)
    {
        /* Test runtime configuration of 100 kHz */
        busFrequency = I2C_100kHz;
        I2C_control(handle, I2C_CMD_SET_BUS_FREQUENCY, &busFrequency);

        memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
        I2C_transactionInit(&i2cTransaction);
        i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
        i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
        i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
        i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
        i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
        i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
        transferStatus = I2C_transfer(handle, &i2cTransaction);

        if(I2C_STS_SUCCESS != transferStatus)
        {
            I2C_log("\n I2C Test: Dynamic configuration of bus Freq failed. \n");
        }

        status = CompareData(&eepromData[0], &rxBuf[0], I2C_EEPROM_TEST_LENGTH);
    }


    /* Test Probe functionality */

    if(true == status)
    {
        /* Probe test with valid slave address */
        slaveAddress = I2C_EEPROM_ADDR;
        controlStatus = I2C_control(handle, I2C_CMD_PROBE, &slaveAddress);

        if(I2C_STATUS_SUCCESS == controlStatus)
        {
            status = true;
        }
        else
        {
            status = false;
            I2C_log("\n I2C Test: Probe test failed. \n");
        }
    }

    if(true == status)
    {
        /* Probe test with invalid slave address */
        slaveAddress = 0x70U;
        controlStatus = I2C_control(handle, I2C_CMD_PROBE, &slaveAddress);

        if(I2C_STATUS_ERROR == controlStatus)
        {
            status = true;
        }
        else
        {
            status = false;
            I2C_log("\n I2C Test: Probe test failed. \n");
        }
    }

    if(true == status)
    {
        /* Test bus recovery functionality */
        delayValue = 2000U;
        controlStatus = I2C_control(handle, I2C_CMD_RECOVER_BUS, &delayValue);

        if(I2C_STATUS_SUCCESS == controlStatus)
        {
            memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
            I2C_transactionInit(&i2cTransaction);
            i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
            i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
            i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
            i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
            i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
            i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
            transferStatus = I2C_transfer(handle, &i2cTransaction);

            if(I2C_STS_SUCCESS != transferStatus)
            {
                I2C_log("\n I2C Test: Bus recovery test failed. \n");
            }

            status = CompareData(&eepromData[0], &rxBuf[0], I2C_EEPROM_TEST_LENGTH);
        }
        else
        {
            status = false;
        }
    }

    return status;
}

static bool I2C_timeout_test(I2C_Handle handle)
{
    uint32_t busFrequency;
    bool status = false;
    int16_t transferStatus;
    I2C_Transaction i2cTransaction;
    char txBuf[I2C_EEPROM_TEST_LENGTH + I2C_EEPROM_ADDR_SIZE] = {0x00, };
    char rxBuf[I2C_EEPROM_TEST_LENGTH];

    /* Set the I2C EEPROM write/read address */
    txBuf[0] = (I2C_EEPROM_TEST_ADDR >> 8) & 0xff; /* EEPROM memory high address byte */
    txBuf[1] = I2C_EEPROM_TEST_ADDR & 0xff;        /* EEPROM memory low address byte */


    /* Test Runtime Configuration of Bus Frequency */

    /* Test runtime configuration of 400 kHz */
    busFrequency = I2C_100kHz;
    I2C_control(handle, I2C_CMD_SET_BUS_FREQUENCY, &busFrequency);

    memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
    i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
    i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
    i2cTransaction.timeout   = 1;
    transferStatus = I2C_transfer(handle, &i2cTransaction);

    if(I2C_STS_ERR_TIMEOUT == transferStatus)
    {
        I2C_log("\n I2C Test: timeout test passed. \n");
        status = true;
    }
    return status;
}

/*
 *  ======== AppDelay ========
 */
void AppDelay(int x)
{
    if(x == 0){
        x = 1;
    }
    unsigned int delayVal = 0xFFFFFFU;
    while(x*delayVal)
    {
        delayVal--;
    }
}
#endif
