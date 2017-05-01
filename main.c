/*
 * File:   main.c
 * Author: john
 *
 * Created on April 26, 2017, 12:19 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include <math.h>

//Configuration FOSC
#pragma config FCKSM = 3
#pragma config OSCIOFNC = 0
#pragma config POSCMD = 3

#pragma config IESO = 0
#pragma config FNOSC = 7

#pragma config PWMPIN = 0
#pragma config HPOL = 1
#pragma config LPOL = 1
#pragma config FWDTEN = 0

#define OUTPUT 0
#define INPUT 1
#define HIGH 1
#define LOW 0
#define TRUE 1
#define FALSE 0

#define PPSUnLock __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock __builtin_write_OSCCONL(OSCCON | 0x40)

/*
 *
 */

enum port_t{
    RA = 0,
    RB
};

struct gpio_t{
    const port_t port;
    const uint8_t pin;
    uint8_t dir;
};

uint8_t gpioSetDir(gpio_t * pin,  uint8_t dir)
{
    pin->dir = dir;
    uint8_t ret = TRUE;
    if(pin->port == RA)
    {
        if(dir == OUTPUT)
        {
            TRISA &= ~(1 << pin);
        }
        else
        {
            TRISA |= (1 << pin);
        }
    }
    else if(pin->port == RB)
    {
        if(dir == OUTPUT)
        {
            TRISB &= ~(1 << pin);
        }
        else
        {
            TRISB |= (1 << pin);
        }
    }
    else
    {
        ret = FALSE;
    }

    return ret;
}

void gpioWrite(gpio_t * pin, uint8_t val)
{
    uint8_t ret = TRUE;
    if(pin->dir == OUTPUT)
    {
        if(pin->port == RA)
        {
            if(val == LOW)
            {
                LATA &= ~(1 << pin);
            }
            else
            {
                LATA |= (1 << pin);
            }
        }
        else if(pni->port == RB)
        {
            if(val == LOW)
            {
                LATB &= ~(1 << pin);
            }
            else
            {
                LATB |= (1 << pin);
            }
        }
        else
        {
            ret = FALSE;
        }
    }
    else
    {
        ret = FALSE;
    }

    return ret;
}

uint8_t gpioRead(gpio_t * pin)
{
    uint8_t ret;
    if(pin->port == RA)
    {
        ret = (PORTA & (1 << pin)) ? 1 : 0;
    }
    else if(pin->port == RB)
    {
        ret = (PORTB & (1 << pin)) ? 1 : 0;
    }

    return ret;
}

void pwmWrite(uint8_t pin, uint8_t value)
{
    if(pin == 1)
    {
        P1DC1 = value;
    }
    else if(pin == 2)
    {
        P1DC2 = value;
    }
}

//value is -1.0 to 1.0
void motorWrite(uint8_t motor, float value)
{
    uint8_t dir = value >= 1 ? 1 : 0;
    uint8_t pwmVal = (value < 0 ? -value : value) * 255;

    if(motor == 1)
    {
        if(dir == 0)
        {
            gpioWrite(MOTOR_1_INA, HIGH);
            gpioWrite(MOTOR_1_INB, LOW);
        }
        else
        {
            gpioWrite(MOTOR_1_INA, LOW);
            gpioWrite(MOTOR_1_INB, HIGH);
        }

        pwmWrite(1, pwmVal);
    }
    else if(motor == 2)
    {
        if(dir == 0)
        {
            gpioWrite(MOTOR_2_INA, HIGH);
            gpioWrite(MOTOR_2_INB, LOW);
        }
        else
        {
            gpioWrite(MOTOR_2_INA, LOW);
            gpioWrite(MOTOR_2_INB, HIGH);
        }

        pwmWrite(2, pwmVal);
    }
}

unsigned char SPI_Transmit(unsigned char TxValue)
{
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = TxValue;
    while(SPI1STATbits.SPIRBF == 0);
    
    return SPI1BUF;
}

unsigned char SPI_Receive()
{
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;
    while(SPI1STATbits.SPIRBF == 0);
    
    return SPI1BUF;
}

void Write_Threshold_Activity_LReg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x20);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Threshold_Activity_HReg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x21);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Threshold_Inactivity_LReg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x23);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Threshold_Inactivity_HReg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x24);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Time_Activity_Reg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x22);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Time_Inactivity_LReg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x25);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Time_Inactivity_HReg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x26);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Active_Inactive_Control_Reg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x27);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Filter_Control_Reg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x2C);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_Power_Control_Reg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x2D);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_INTMAP1_Reg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x2A);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

void Write_INTMAP2_Reg(unsigned char DataByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x0A);
    SPI_Transmit (0x2B);
    SPI_Transmit (DataByte);
    LATBbits.LATB6 = 1;
}

unsigned char Read_Status_Reg()
{
    LATBbits.LATB6 = 0;
    SPI_Transmit(0x0B);
    SPI_Transmit(0x0B);
    unsigned char status = SPI_Receive();
    LATBbits.LATB6 = 1;
    return status;
}

unsigned char Read_8bit_XData()
{
    LATBbits.LATB6 = 0;
    SPI_Transmit(0x0B);
    SPI_Transmit(0x08);
    unsigned char XData = SPI_Receive();
    LATBbits.LATB6 = 1;
    return XData;
}

unsigned char Read_8bit_YData()
{
    LATBbits.LATB6 = 0;
    SPI_Transmit(0x0B);
    SPI_Transmit(0x09);
    unsigned char YData = SPI_Receive();
    LATBbits.LATB6 = 1;
    return YData;
}

unsigned char Read_8bit_ZData()
{
    LATBbits.LATB6 = 0;
    SPI_Transmit(0x0B);
    SPI_Transmit(0x0A);
    unsigned char ZData = SPI_Receive();
    LATBbits.LATB6 = 1;
    return ZData;
}

//Stub
//Should return the robots tilt from vertical in degrees
double getTilt()
{
     //SPIConfiguration MASTER mode sending 8 bits
    SPI1CON1bits.DISSCK = 0;
    SPI1CON1bits.DISSDO = 0;
    SPI1CON1bits.MODE16 = 0;
    SPI1CON1bits.SSEN = 0;
    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.SMP = 0;
    SPI1CON1bits.CKE = 1;
    SPI1CON1bits.CKP = 0;
    SPI1CON1bits.PPRE = 1;
    SPI1CON1bits.SPRE = 7;
    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPIEN = 1;
    
    //Peripheral Pin Select with RP pins
    PPSUnLock;
    RPOR4bits.RP8R = 8;
    RPINR20bits.SCK1R = 8;
    RPOR4bits.RP9R = 7;
    RPINR20bits.SDI1R = 7;
    RPOR3bits.RP6R = 9;
    PPSLock;
    
    //Define I/O
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB7 = 0;
    
    Write_Threshold_Activity_LReg(0x00);
    
    Write_Threshold_Activity_HReg(0x00);
    Write_Threshold_Inactivity_LReg(0x00);
    
    Write_Threshold_Inactivity_HReg(0x00);
    Write_Time_Inactivity_LReg(0x0A);
    
    Write_Active_Inactive_Control_Reg(0x05);
    
    Write_Filter_Control_Reg(0x03);
    Write_Power_Control_Reg(0x02);
    
    double X, Y, Z;
    double thetaX, thetaY, thetaZ;
    
    while((Read_Status_Reg() & 0x01) == 0x00)
        Read_Status_Reg();
    
    X = Read_8bit_XData()*16;
    Y = Read_8bit_YData()*16;
    Z = Read_8bit_ZData()*16;
    
    thetaX = (atan((X)/sqrt(Y*Y + Z*Z)))*180.0/3.14;
    thetaY = (atan((Y)/sqrt(X*X + Z*Z)))*180.0/3.14;
    thetaZ = (atan(sqrt(X*X + Y*Y)/(Z)))*180.0/3.14;
    
    return thetaX;
//    return thetaY;
//    return thetaZ;
}

gpio_t MOTOR_1_INA = {RA, 0, OUTPUT};
gpio_t MOTOR_1_INB = {RA, 1, OUTPUT};
gpio_t MOTOR_2_INA = {RB, 0, OUTPUT};
gpio_t MOTOR_2_INA = {RB, 1, OUTPUT};

int main(int argc, char** argv) {

    OSCTUNbits.TUN = 23;

    CLKDIVbits.FRCDIV = 0;
    CLKDIVbits.DOZE = 0;
    CLKDIVbits.DOZEN = 1;

    P1TCONbits.PTEN = 0;
    P1TCONbits.PTCKPS = 0;
    P1TCONbits.PTMOD = 0;

    P1TMRbits.PTMR = 0;
    P1TPER = 128;

    PWM1CON1bits.PMOD3 = 1;
    PWM1CON1bits.PMOD2 = 1;
    PWM1CON1bits.PMOD1 = 1;
    PWM1CON1bits.PEN1L = 1;
    PWM1CON1bits.PEN1H = 1;
    PWM1CON1bits.PEN2L = 0;
    PWM1CON1bits.PEN2H = 0;
    PWM1CON1bits.PEN3L = 0;
    PWM1CON1bits.PEN3H = 0;

    PWM1CON2bits.IUE = 0;
    PWM1CON2bits.UDIS = 0;

    P1DC1 = 0;
    P1DC2 = 0;

    AD1PCFGL = 0xFFFF;
    gpioSetDir(MOTOR_1_INA, OUTPUT);
    gpioSetDir(MOTOR_1_INB, OUTPUT);
    gpioSetDir(MOTOR_2_INA, OUTPUT);
    gpioSetDir(MOTOR_2_INB, OUTPUT);

    P1TCONbits.PTEN = 1;

    while(true)
    {

    }

    return (EXIT_SUCCESS);
}
