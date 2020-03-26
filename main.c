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
#define BMP180_READ_TEMP_CMD    2 //Read temperature
#define BMP180_READ_PRESS_CMD   4 //Read pressure
#define BMP180_READ_COEFF_CMD   8 //Read calibration coefficients

#define BMP180_IDLE             10
#define BMP180_READ_TEMPERATURE 50
#define BMP180_READ_PRESSURE    52
#define BMP180_WAIT             55
#define BMP180_READ             60
#define BMP180_DONE             100
#define BMP180_ERROR            254

#define XBEE_SEND    150
#define XBEE_RECV    160

#define ERROR       254
#define DEBUG       255

#include "Globals.h"
#include <xc.h>
#include "mcc_generated_files/mcc.h"

struct BMP180_cmd_typ{
    uint8_t     commandByte;
};

struct BMP180_sts_typ{
    uint8_t     status;
    uint8_t     error;
    uint8_t     UT_TemperatureMSB;
    uint8_t     UT_TemperatureLSB;
    uint8_t     UT_PressureMSB;
    uint8_t     UT_PressureLSB;
};

uint8_t g_10ms_tick;

int xbeeSend(int, struct BMP180_sts_typ *);
void BMP180(struct BMP180_cmd_typ *, struct BMP180_sts_typ *);

int BMP180_WriteCommand(uint8_t);

void main(void) {
    unsigned short status=INIT, i;
    int retVal, powerCounter, thickCtr;
    uint8_t xbee_89_Telegram[TELEGRAM_89_LENGTH];
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
        g_10ms_tick = g_TMR0_tick;
        
        if (g_10ms_tick)
            g_TMR0_tick = 0;
        
        if (g_10ms_tick)
            thickCtr++;

        switch(status) {
            case INIT:
                thickCtr = 0;
                powerCounter = 0;
                status = WAIT_BEFORE_START;
                break;
            
            case WAIT_BEFORE_START: //Empty wait
                if (thickCtr > 10) {
                    BMP180_cmd.commandByte = BMP180_INIT_CMD;
                    status = READ_COEFFICIENTS;
                    } 
                break;
                
            case READ_COEFFICIENTS:
                //BMP180_cmd.commandByte = BMP180_READ_COEFF_CMD;
                status = IDLE;
                break;
                
            case IDLE:
                if (thickCtr == 500) {
                    //BMP180_cmd.commandByte = BMP180_READ_TEMP_CMD;
                    //status = READ_TEMPERATURE;
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
                if (BMP180_sts.status == BMP180_DONE)
                    status = IDLE;
                
                if (BMP180_sts.status == BMP180_ERROR)
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
                break;
            } 
        BMP180(&BMP180_cmd, &BMP180_sts);
        
        if (BMP180_sts.status == ERROR)
            status = ERROR;
        
    }
    return;
}

void BMP180(struct BMP180_cmd_typ *BMP180_cmd, struct BMP180_sts_typ *BMP180_sts) {
    static uint8_t wait_t, requestedCmd;
    uint8_t MSB, LSB;
    int8_t retVal;
    
    if (BMP180_cmd->commandByte & BMP180_INIT_CMD) { //Bitwise AND
        BMP180_cmd->commandByte &= !BMP180_INIT_CMD;
        BMP180_sts->error = 0;
        BMP180_sts->status = INIT;
    }
    
    switch(BMP180_sts->status) {
        case INIT:
            SSP1CON1bits.SSPEN = 1; //Enable serial port
            //Be sure that RC3 = SCL1 & RC4 = SDA1 are configured as input!!!
            BMP180_sts->status = IDLE;
            break;
            
        case BMP180_IDLE:
            if (BMP180_cmd->commandByte & BMP180_READ_TEMP_CMD) {
                BMP180_cmd->commandByte &= !BMP180_READ_TEMP_CMD;
                requestedCmd = CONTROL_REG_READ_TEMP; //0x2E = read temperature
                BMP180_sts->status = BMP180_READ_TEMPERATURE;
                break;
            }

            if (BMP180_cmd->commandByte & BMP180_READ_PRESS_CMD) {
                BMP180_cmd->commandByte &= !BMP180_READ_PRESS_CMD;
                requestedCmd = CONTROL_REG_READ_PRESS;
                BMP180_sts->status = BMP180_READ_PRESSURE;
                break;
            }

            break;
            
        case BMP180_READ_TEMPERATURE:
            if ( BMP180_WriteCommand(requestedCmd) ) {
                BMP180_sts->error = 1;
                BMP180_sts->status = BMP180_ERROR;
            }
            else {
                wait_t = 0; //Reset timer
                BMP180_sts->status = BMP180_WAIT;
            }
            break;

        case BMP180_READ_PRESSURE:
            if ( BMP180_WriteCommand(requestedCmd) ) { 
                BMP180_sts->error = 1;
                BMP180_sts->status = BMP180_ERROR;
            }
            else {
                wait_t = 0; //Reset timer
                BMP180_sts->status = BMP180_WAIT;
            }
            break;
            
/*Only one wait state. Temperature and pressure readings have different delays.
 * Wait 40..50ms. Quite high delay, but we're reading ambient parameters...
 */
        case BMP180_WAIT: //Wait response from module
            if (g_10ms_tick)
                if ( (++wait_t) >= 5)
                    BMP180_sts->status = BMP180_READ;
            break;
            
        case BMP180_READ:
            //Start condition
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1CON2bits.SEN = 1; //Start condition
            while(!PIR3bits.SSP1IF); //SSPIF is set every 9th clock cycle
            
            //Module address write command
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1BUF = MODULE_ADDRESS_WRITE; //Address + write
            while(!PIR3bits.SSP1IF); //SSPIF is set every 9th clock cycle

            if (SSP1CON2bits.ACKSTAT) { //ACK was not received
                SSP1CON2bits.PEN = 1; //Sends stop bit
                while(SSP1CON2bits.PEN);
                BMP180_sts->status = BMP180_ERROR; //TODO: error code
                break;
                }
            
            //Register address
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1BUF = 0xF6; //LSB address
            while(!PIR3bits.SSP1IF); //SSPIF is set every 9th clock cycle

            if (SSP1CON2bits.ACKSTAT) { //ACK was not received
                SSP1CON2bits.PEN = 1; //Sends stop bit
                while(SSP1CON2bits.PEN);
                BMP180_sts->status = BMP180_ERROR; //TODO: error code
                break;
                }
            
            //Master restarts
            SSP1CON2bits.RSEN = 1; //Restart condition
            while(SSP1CON2bits.RSEN);
            
            //Module address read
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1BUF = MODULE_ADDRESS_READ; //Address + read
            while(!PIR3bits.SSP1IF); //SSPIF is set every 9th clock cycle

            if (SSP1CON2bits.ACKSTAT) { //ACK was not received
                SSP1CON2bits.PEN = 1; //Sends stop bit
                while(SSP1CON2bits.PEN);
                BMP180_sts->status = BMP180_ERROR; //TODO: error code
                break;
                }
            
            //Master receives 1 byte
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1CON2bits.RCEN = 1; //Receive enable
            while(!PIR3bits.SSP1IF);
            MSB = SSP1BUF;//Master reads received byte MSB
            
            //I2C_MasterSendACK
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1CON2bits.ACKDT = 0;
            SSP1CON2bits.ACKEN = 1;
            while(!PIR3bits.SSP1IF);
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag

            //Master receives 1 byte
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1CON2bits.RCEN = 1; //Receive enable
            while(!PIR3bits.SSP1IF);
            LSB = SSP1BUF;//Master reads received byte LSB

            //I2C_MasterSendNotACK
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1CON2bits.ACKDT = 1;
            SSP1CON2bits.ACKEN = 1;
            while(!PIR3bits.SSP1IF);
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag

            //Master sends stop
            SSP1CON2bits.PEN = 1; //Sends stop bit
            while(SSP1CON2bits.PEN);
            
            BMP180_sts->status = BMP180_DONE; //TOGLIERE
            
            break;
        
        case BMP180_DONE:
            if (requestedCmd == CONTROL_REG_READ_TEMP) {
                BMP180_sts->UT_TemperatureMSB = MSB;
                BMP180_sts->UT_TemperatureLSB = LSB;
            }

            if (requestedCmd == CONTROL_REG_READ_PRESS) {
                BMP180_sts->UT_PressureMSB = MSB;
                BMP180_sts->UT_PressureLSB = LSB;
            }

            BMP180_sts->status = BMP180_IDLE;
            break;
            
        case DEBUG:
            ;
            break;
            
        case BMP180_ERROR:
            BMP180_sts->status = BMP180_IDLE;
            break;
    }
    return;
}

int BMP180_WriteCommand(uint8_t command) {
            SSP1CON2bits.SEN = 1; //Start condition
            while(SSP1CON2bits.SEN);
            
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1BUF = MODULE_ADDRESS_WRITE; //Address + write
            while(!PIR3bits.SSP1IF)
                ; //SSPIF is set every 9th clock cycle
            
            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            if (SSP1CON2bits.ACKSTAT) { //ACK was not received
                SSP1CON2bits.PEN = 1; //Sends stop bit
                while(SSP1CON2bits.PEN);
                return -1;
                }

            SSP1BUF = 0xF4; //Register address
            while(!PIR3bits.SSP1IF); //SSPIF is set every 9th clock cycle
                
            if (SSP1CON2bits.ACKSTAT) { //ACK was not received
                SSP1CON2bits.PEN = 1; //Sends stop bit
                while(SSP1CON2bits.PEN);
                return -2;
                }

            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1BUF = command;
            while(!PIR3bits.SSP1IF); //SSPIF is set every 9th clock cycle
                
            if (SSP1CON2bits.ACKSTAT) { //ACK was not received
                SSP1CON2bits.PEN = 1; //Sends stop bit
                while(SSP1CON2bits.PEN);
                return -3;
                }

            PIR3bits.SSP1IF = 0; //Clear SSP interrupt flag
            SSP1CON2bits.PEN = 1; //Sends stop bit
            while(SSP1CON2bits.PEN);
    
    return 0;
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
    xbeeSendMessage[12] = 0;//inBMP180_sts->UT_TemperatureLSB; //Data: temperature LSB
    xbeeSendMessage[13] = 0;//inBMP180_sts->UT_TemperatureMSB; //Data: temperature MSB
    xbeeSendMessage[14] = 0;//inBMP180_sts->UT_PressureLSB; //Data: pressure LSB
    xbeeSendMessage[15] = 0;//inBMP180_sts->UT_PressureMSB; //Data: pressure MSB
    
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