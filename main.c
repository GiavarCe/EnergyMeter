/*
 * File:   main.c
 * Author: giavarce
 *
 * Created on 09 March 2020, 18:31
 */

#define INIT                1
#define READ_COEFFICIENTS   5 //BMP180 is reading calibration coefficients
#define IDLE                10
#define READ_TEMPERATURE    20
#define WAIT_BEFORE_START   30

#define BMP180_INIT_CMD         1 //Init command
#define BMP180_READ_T_P_CMD     2 //Read temperature and pressure
#define BMP180_READ_COEFF_CMD   4 //Read calibration coefficients

//***BMP180 module constants***
#define BMP180_I2C_ADDRESS      0x77 //I2C address
#define BMP180_ID_REGISTER      0xD0 //ID address register
#define BMP180_CTRL_MEAS_REG    0xF4 //Measurement control register
#define BMP180_OUT_MSB_REG      0xF6 //MSB output register (out_msb)
//*****************************

#define BMP180_IDLE_STS             10
#define BMP180_READ_T_STS           50 //Read temperature status
#define BMP180_READ_P_STS           52 //Read pressure status
#define BMP180_DONE_STS             100
#define BMP180_ERROR_STS            254
#define BMP180_DEBUG_STS            255

#define XBEE_SEND    150
#define XBEE_RECV    160

#define ERROR       254
#define DEBUG       255

#include "Globals.h"
#include <xc.h>
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/drivers/i2c_simple_master.h"

struct BMP180_cmd_typ{
    uint8_t     commandByte;
};

struct BMP180_sts_typ{
    uint8_t     status;
    uint8_t     error;
    uint16_t    Temperature;
    uint16_t    Pressure;
};

uint8_t g_10ms_tick;
uint8_t g_BMP180_CalibrationCoefficients[22]; //Calibration coefficients
    
int xbeeSend(int, struct BMP180_sts_typ *);
void BMP180(struct BMP180_cmd_typ *, struct BMP180_sts_typ *);

void main(void) {
    unsigned short status=INIT, i;
    int retVal, powerCounter, thickCtr;
    uint8_t xbee_89_Telegram[TELEGRAM_89_LENGTH], BMP180_id;
 
    unsigned char sample, change, slatch=0, RC5_os;
    struct BMP180_cmd_typ BMP180_cmd;
    struct BMP180_sts_typ BMP180_sts;
    
/*Request message
 * B0 - Slave address
 * B1 - Function code (0x04 = reading)
 * B2 - Start address (HI)
 * B3 - Start address (LO)
 * B4 - Number of points (HI)
 * B5 - Number of points (LO)
 * B6 - Error check (LO)
 * B7 - Error check (HI)
 */

    SYSTEM_Initialize();
    TMR0_StartTimer();
    INTCONbits.GIE = 1;
    
    while(1) {
        g_10ms_tick = g_TMR0_tick; //10ms interrupt from TMR0
        
        if (g_10ms_tick)
            g_TMR0_tick = 0;
        
        if (g_10ms_tick)
            thickCtr++;

        switch(status) {
            case INIT:
                thickCtr = 0;
                powerCounter = 0;

                if ( (BMP180_id = i2c_read1ByteRegister(BMP180_I2C_ADDRESS, BMP180_ID_REGISTER)) != 0x55) {
                    status = ERROR;
                    break;
                    }

                BMP180_cmd.commandByte = BMP180_INIT_CMD;
                status = WAIT_BEFORE_START;
                break;
            
            case WAIT_BEFORE_START: //Empty wait
                if (thickCtr > 10) 
                    status = READ_COEFFICIENTS;
                     
                break;
                
            case READ_COEFFICIENTS: //Calibration coefficients
                status = IDLE;
                
                /*if ( (retVal=I2C_ReadMultipleBytes(g_BMP180_CalibrationCoefficients, 22, 0xAA)) == 0)
                    status = IDLE;
                else
                    status = ERROR;
                */
                break;
                
            case IDLE:
                if (thickCtr == 500) {
                    BMP180_cmd.commandByte = BMP180_READ_T_P_CMD;
                    status = READ_TEMPERATURE;
                    break;
                }
                
                if (thickCtr >= 1500) { //15s 
                    thickCtr=0;
                    i=0;
                    status = XBEE_SEND; //MODBUS_SEND
                    }
                
                sample = PORTC;
                sample &= 32; //RC5
                change = sample ^ slatch;
                RC5_os = change & sample;
                slatch = sample;
                
                if (RC5_os)
                    powerCounter++;
                
                break;
                
            case READ_TEMPERATURE:
                if (BMP180_sts.status == BMP180_DONE_STS)
                    status = IDLE;
                
                if (BMP180_sts.status == BMP180_ERROR_STS)
                    status = ERROR;
                
                break;
                
            case XBEE_SEND:
                        IO_RA4_SetHigh(); //DEBUG
                retVal = xbeeSend(powerCounter,&BMP180_sts);
                i=0;
                status = XBEE_RECV;
                break;
                
            case XBEE_RECV: //TODO: add timeout
                while (i != TELEGRAM_89_LENGTH)
                    if (EUSART_is_rx_ready())
                        xbee_89_Telegram[i++] = EUSART_Read();

                IO_RA4_SetLow(); //DEBUG
                
                if (xbee_89_Telegram[5] == 0x00) //Success
                    powerCounter = 0;
                
                status = IDLE;
                break;
            
            case ERROR:
                IO_RA5_SetHigh();
                
                break;
                
            case DEBUG:
                //IO_RA5_SetHigh();
                break;
            } 
        BMP180(&BMP180_cmd, &BMP180_sts);
        
        if (BMP180_sts.status == ERROR)
            status = ERROR;
        
    }
    return;
}

void BMP180(struct BMP180_cmd_typ *BMP180_cmd, struct BMP180_sts_typ *BMP180_sts) {
    static uint8_t wait_t;
    uint8_t MSB, LSB;
    int8_t retVal;
    
    if (BMP180_cmd->commandByte & BMP180_INIT_CMD) { //Bitwise AND
        BMP180_cmd->commandByte &= !BMP180_INIT_CMD;
        BMP180_sts->error = 0;
        BMP180_sts->status = INIT;
    }
    
    switch(BMP180_sts->status) {
        case INIT:
            BMP180_sts->status = BMP180_IDLE_STS;
            break;
            
        case BMP180_IDLE_STS:
            if (BMP180_cmd->commandByte & BMP180_READ_T_P_CMD) {
                BMP180_cmd->commandByte &= !BMP180_READ_T_P_CMD;
                BMP180_sts->status = BMP180_READ_T_STS;
                break;
            }

            break;
            
        case BMP180_READ_T_STS:
            i2c_write1ByteRegister(BMP180_I2C_ADDRESS, BMP180_CTRL_MEAS_REG, 0x2E);
            __delay_ms(5);
            BMP180_sts->Temperature = i2c_read2ByteRegister(BMP180_I2C_ADDRESS, BMP180_OUT_MSB_REG);
            BMP180_sts->status = BMP180_READ_P_STS;
            break;

        case BMP180_READ_P_STS:
            i2c_write1ByteRegister(BMP180_I2C_ADDRESS, BMP180_CTRL_MEAS_REG, 0xF4);
            __delay_ms(30);
            BMP180_sts->Pressure = i2c_read2ByteRegister(BMP180_I2C_ADDRESS, BMP180_OUT_MSB_REG);
            BMP180_sts->status = BMP180_DONE_STS;
            break;
                                
        case BMP180_DONE_STS:
            BMP180_sts->status = BMP180_IDLE_STS;
            break;
            
        case BMP180_DEBUG_STS:
            ;
            break;
            
        case BMP180_ERROR_STS:
            BMP180_sts->status = BMP180_IDLE_STS;
            break;
    }
    return;
}

int xbeeSend(int inPower, struct BMP180_sts_typ *inBMP180_sts) {

    uint8_t xbeeSendMessage[XBEE_SEND_SIZE];
    int tempResult, i;

    xbeeSendMessage[0] = 0x7E; //Start delimiter
    xbeeSendMessage[1] = 0x00; //MSB length
    xbeeSendMessage[2] = 0x0D; //LSB length
    xbeeSendMessage[3] = 0x01; //API identifier (01 = Tx request 16 bit address)
    xbeeSendMessage[4] = 0x01; //ID
    xbeeSendMessage[5] = 0x00; //Dest addr. H
    xbeeSendMessage[6] = 0x01; //Dest addr. L
    xbeeSendMessage[7] = 0x00; //Enable ACK
    xbeeSendMessage[8] = 0x0A;
    xbeeSendMessage[9] = 0x00;
    xbeeSendMessage[10] = inPower & 0xFF; 
    xbeeSendMessage[11] = (inPower >> 8) & 0xFF;
    xbeeSendMessage[12] = inBMP180_sts->Temperature & 0x00FF; //Temperaure LSB
    xbeeSendMessage[13] = inBMP180_sts->Temperature >> 8; //Temperature MSB
    xbeeSendMessage[14] = inBMP180_sts->Pressure & 0x00FF; //Pressure LSB;
    xbeeSendMessage[15] = inBMP180_sts->Pressure >> 8; //Pressure MSB
    
    tempResult=0;
    for (i=3; i<= (XBEE_SEND_SIZE -2 ); i++)
        tempResult += xbeeSendMessage[i];
    tempResult &= 0xFF;
    
    xbeeSendMessage[XBEE_SEND_SIZE - 1] = (0xFF - tempResult); //CRC
    
    i=0;

    while (i != XBEE_SEND_SIZE) {  
        if (EUSART_is_tx_ready()) 
            EUSART_Write(xbeeSendMessage[i++]);
        }

    return 0;
}