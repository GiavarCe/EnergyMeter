/*
 * File:   main.c
 * Author: giavarce
 *
 * Created on 09 March 2020, 18:31
 */

#define INIT                1
#define READ_COEFFICIENTS   5 //BMP180 is reading calibration coefficients
#define SEND_COEFFICIENTS   7 //Send BMP180 coefficients to main
#define IDLE                10
#define READ_TEMPERATURE    20
#define WAIT_BEFORE_START   30

//***BMP180 module commands***
#define BMP180_INIT_CMD         1 //Init command
#define BMP180_READ_T_P_CMD     2 //Read temperature and pressure
#define BMP180_READ_COEFF_CMD   4 //Read calibration coefficients
//****************************

//***BMP180 module constants***
#define BMP180_MODULE_ADDRESS   0x77 //I2C Module address address
#define BMP180_ID_REGISTER      0xD0 //ID address register
#define BMP180_CTRL_MEAS_REG    0xF4 //Measurement control register
#define BMP180_OUT_MSB_REG      0xF6 //MSB output register (out_msb)
#define BMP180_CALIBRATION_REG  0xAA //First calibration register address
//*****************************

//***BMP180 module states***
#define BMP180_IDLE_STS             10
#define BMP180_READ_T_STS           50 //Read temperature status
#define BMP180_READ_T_WAIT          51 //Wait for temperature value ready
#define BMP180_READ_P_WAIT          52 //Wait for pressure value ready
#define BMP180_READ_P_STS           55 //Read pressure status
#define BMP180_DONE_STS             100
#define BMP180_ERROR_STS            254
#define BMP180_DEBUG_STS            255
//***************************

//***Timer constants***
#define TIMER_IDLE_STS          5
#define TIMER_RUNNING_STS      10
#define TIMER_EXPIRED_STS      20
#define TIMER_STOP_CMD         50 //Timer run
#define TIMER_RUN_CMD          60 //Timer run
//******************

#define XBEE_SEND           150
#define XBEE_RECV           160
#define XBEE_SEND_COEFF_CMD 170 //Send coefficients command
#define XBEE_SEND_DATA_CMD  180 //Send data command

#define ERROR       254
#define DEBUG       255

#include "Globals.h"
#include <xc.h>
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/drivers/i2c_simple_master.h"

struct timer_typ{
    uint8_t cmd;
    int     delayTime;
    int     actualTime;
    uint8_t status;
};

struct BMP180_cmd_typ{
    uint8_t     commandByte;
};

struct BMP180_sts_typ{
    uint8_t     status;
    uint8_t     error;
    uint16_t    Temperature;
    uint16_t    Pressure;
};

uint8_t g_10ms_os;
uint8_t g_BMP180_CalibrationCoefficients[22]; //Calibration coefficients
    
int xbeeSend(int, struct BMP180_sts_typ *, uint8_t);
void BMP180(struct BMP180_cmd_typ *, struct BMP180_sts_typ *);
void timer(struct timer_typ *);

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
        g_10ms_os = g_TMR0_tick; //10ms interrupt from TMR0
        
        if (g_10ms_os)
            g_TMR0_tick = 0;
        
        if (g_10ms_os)
            thickCtr++;

        switch(status) {
            case INIT:
                thickCtr = 0;
                powerCounter = 0;

                if ( (BMP180_id = i2c_read1ByteRegister(BMP180_MODULE_ADDRESS, BMP180_ID_REGISTER)) != 0x55) {
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
                i2c_readDataBlock(BMP180_MODULE_ADDRESS, BMP180_CALIBRATION_REG, g_BMP180_CalibrationCoefficients, 22);
                status = SEND_COEFFICIENTS;
                break;
            
            case SEND_COEFFICIENTS:
                //retVal = xbeeSend(powerCounter,&BMP180_sts, XBEE_SEND_COEFF_CMD);
               status = IDLE;
                
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
                retVal = xbeeSend(powerCounter,&BMP180_sts, XBEE_SEND_DATA_CMD);
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
    static struct timer_typ timer1;
    uint8_t MSB, LSB;
    int8_t retVal;
    
    if (BMP180_cmd->commandByte & BMP180_INIT_CMD) { //Bitwise AND
        BMP180_cmd->commandByte &= !BMP180_INIT_CMD;
        BMP180_sts->error = 0;
        BMP180_sts->status = INIT;
    }
    
    switch(BMP180_sts->status) {
        case INIT:
            timer1.cmd = TIMER_STOP_CMD;
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
            i2c_write1ByteRegister(BMP180_MODULE_ADDRESS, BMP180_CTRL_MEAS_REG, 0x2E);
            timer1.delayTime = 2; //20ms max
            timer1.cmd = TIMER_RUN_CMD;
            BMP180_sts->status = BMP180_READ_T_WAIT;
            break;
            
        case BMP180_READ_T_WAIT:
            if (timer1.status == TIMER_EXPIRED_STS) {
                timer1.cmd = TIMER_STOP_CMD;
                BMP180_sts->Temperature = i2c_read2ByteRegister(BMP180_MODULE_ADDRESS, BMP180_OUT_MSB_REG);
                BMP180_sts->status = BMP180_READ_P_STS;
                }
            break;

        case BMP180_READ_P_STS:
            i2c_write1ByteRegister(BMP180_MODULE_ADDRESS, BMP180_CTRL_MEAS_REG, 0xF4);
            timer1.delayTime = 4; //40ms max
            timer1.cmd = TIMER_RUN_CMD;
            BMP180_sts->status = BMP180_READ_P_WAIT;
            break;
        
        case BMP180_READ_P_WAIT:
            if (timer1.status == TIMER_EXPIRED_STS) {
                BMP180_sts->Pressure = i2c_read2ByteRegister(BMP180_MODULE_ADDRESS, BMP180_OUT_MSB_REG);
                timer1.cmd = TIMER_STOP_CMD;
                BMP180_sts->status = BMP180_DONE_STS;
                }
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
    timer(&timer1);
    return;
}

int xbeeSend(int inPower, struct BMP180_sts_typ *inBMP180_sts, uint8_t inMessageType) {

    uint8_t xbeeSendMessage[31], xbeeSendSize;
    int tempResult, i;

    xbeeSendMessage[0] = 0x7E; //Start delimiter
    xbeeSendMessage[1] = 0x00; //MSB length (always, tx length limited to 100 bytes))
    //xbeeSendMessage[2] will be written later
    xbeeSendMessage[3] = 0x01; //API identifier (01 = Tx request 16 bit address)
    xbeeSendMessage[4] = 0x01; //ID
    xbeeSendMessage[5] = 0x00; //Dest addr. H
    xbeeSendMessage[6] = 0x01; //Dest addr. L
    xbeeSendMessage[7] = 0x00; //Options: enable ACK.
    
    switch (inMessageType) {
        case XBEE_SEND_DATA_CMD:
            xbeeSendSize = XBEE_SEND_DATA_SIZE;
            xbeeSendMessage[2] = 0x0D; //LSB length. Fixed
            xbeeSendMessage[8] = 0x0A; //First data byte. Message type
            xbeeSendMessage[9] = 0x00;
            xbeeSendMessage[10] = inPower & 0x00FF; 
            xbeeSendMessage[11] = (inPower >> 8);
            xbeeSendMessage[12] = inBMP180_sts->Temperature & 0x00FF; //Temperaure LSB
            xbeeSendMessage[13] = inBMP180_sts->Temperature >> 8; //Temperature MSB
            xbeeSendMessage[14] = inBMP180_sts->Pressure & 0x00FF; //Pressure LSB;
            xbeeSendMessage[15] = inBMP180_sts->Pressure >> 8; //Pressure MSB
            break;
        
        case XBEE_SEND_COEFF_CMD:
            xbeeSendSize = XBEE_SEND_COEFF_SIZE;
            xbeeSendMessage[2] = 0x1B; //LSB length. Fixed
            xbeeSendMessage[8] = 0x0B; //First data byte. Message type
            xbeeSendMessage[9] = 0x00;
            xbeeSendMessage[10] = 0; 
            xbeeSendMessage[11] = 0;
            xbeeSendMessage[12] = 0;
            xbeeSendMessage[13] = 0;
            xbeeSendMessage[14] = 0;
            xbeeSendMessage[15] = 0;
            xbeeSendMessage[16] = 0;
            xbeeSendMessage[17] = 0;
            xbeeSendMessage[18] = 0;
            xbeeSendMessage[19] = 0;
            xbeeSendMessage[20] = 0;
            xbeeSendMessage[21] = 0;
            xbeeSendMessage[22] = 0;
            xbeeSendMessage[23] = 0;
            xbeeSendMessage[24] = 0;
            xbeeSendMessage[25] = 0;
            xbeeSendMessage[26] = 0;
            xbeeSendMessage[27] = 0;
            xbeeSendMessage[28] = 0;
            xbeeSendMessage[29] = 0;
            xbeeSendMessage[30] = 0;
            break;
    }
    
    tempResult=0;
    for (i=3; i<= (xbeeSendSize -2 ); i++)
        tempResult += xbeeSendMessage[i];
    tempResult &= 0xFF;
    
    xbeeSendMessage[xbeeSendSize - 1] = (0xFF - tempResult); //CRC
    
    i=0;
    while (i != xbeeSendSize) {  
        if (EUSART_is_tx_ready()) 
            EUSART_Write(xbeeSendMessage[i++]);
        }

    return 0;
}

void timer(struct timer_typ *inTimer) {
    static char status;
    
    if (inTimer->cmd == TIMER_STOP_CMD)
        status = TIMER_IDLE_STS;
    
    switch(status) {
        case TIMER_IDLE_STS:
            if (inTimer->cmd == TIMER_RUN_CMD) {
                inTimer->actualTime = 0;
                status = TIMER_RUNNING_STS;
                }
            
            break;
        
        case TIMER_RUNNING_STS:
            if (g_10ms_os)
                if ( (inTimer->actualTime++) >= inTimer->delayTime)
                    status = TIMER_EXPIRED_STS;
            break;
        
        case TIMER_EXPIRED_STS:
            break;
    }
    
    inTimer->status = status;
    return;
}