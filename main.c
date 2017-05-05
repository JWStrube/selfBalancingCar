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
#include <libpic30.h>

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

#define ACCEL_THRESHOLD_ACTIVITY_LREG 0x20
#define ACCEL_THRESHOLD_ACTIVITY_HREG 0x21
#define ACCEL_THRESHOLD_INACTIVITY_LREG 0x23
#define ACCEL_THRESHOLD_INACTIVITY_HREG 0x24
#define ACCEL_TIME_ACTIVITY_REG 0x22
#define ACCEL_TIME_INACTIVITY_LREG 0x25
#define ACCEL_TIME_INACTIVITY_HREG 0x26
#define ACCEL_ACTIVE_INACTIVE_CONTROL_REG 0x27
#define ACCEL_FILTER_CONTROL_REG 0x2C
#define ACCEL_POWER_CONTROL_REG 0x2D
#define  ACCEL_INTMAP1_REG 0x2A
#define ACCEL_INTMAP2_REG 0x2B
#define ACCEL_STATUS_REG 0x0B
#define ACCEL_XDATA_REG 0x08
#define ACCEL_YDATA_REG 0x09
#define ACCEL_ZDATA_REG 0x0A

 #define FOSC  8000000LL  // clock-frequecy in Hz with suffix LL (64-bit-long), eg. 32000000LL for 32MHz
 #define FCY       (FOSC/2)  // MCU is running at FCY MIPS
 #define delay_us(x) __delay32(((x*FCY)/1000000L)) // delays x us
 #define delay_ms(x) __delay32(((x*FCY)/1000L))  // delays x ms

/*
 *
 */


typedef enum {
    RA = 0,
    RB
}port_t;

typedef struct {
    const port_t port;
    const uint8_t pin;
    uint8_t dir;
}gpio_t;

gpio_t MOTOR_1_INA = {RA, 0, OUTPUT};
gpio_t MOTOR_1_INB = {RA, 1, OUTPUT};
gpio_t MOTOR_2_INA = {RB, 0, OUTPUT};
gpio_t MOTOR_2_INB = {RB, 1, OUTPUT};

gpio_t ACL_SS = {RB, 6, OUTPUT};
gpio_t ACL_SDI = {RB, 7, INPUT};
gpio_t ACL_SCK = {RB, 8, OUTPUT};
gpio_t ACL_SDO = {RB, 9, OUTPUT};

uint8_t gpioSetDir(gpio_t * pin,  uint8_t dir)
{
    pin->dir = dir;
    uint8_t ret = TRUE;
    if(pin->port == RA)
    {
        if(dir == OUTPUT)
        {
            TRISA &= ~(1 << pin->pin);
        }
        else
        {
            TRISA |= (1 << pin->pin);
        }
    }
    else if(pin->port == RB)
    {
        if(dir == OUTPUT)
        {
            TRISB &= ~(1 << pin->pin);
        }
        else
        {
            TRISB |= (1 << pin->pin);
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
                LATA &= ~(1 << pin->pin);
            }
            else
            {
                LATA |= (1 << pin->pin);
            }
        }
        else if(pin->port == RB)
        {
            if(val == LOW)
            {
                LATB &= ~(1 << pin->pin);
            }
            else
            {
                LATB |= (1 << pin->pin);
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
        ret = (PORTA & (1 << pin->pin)) ? 1 : 0;
    }
    else if(pin->port == RB)
    {
        ret = (PORTB & (1 << pin->pin)) ? 1 : 0;
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
            gpioWrite(&MOTOR_1_INA, HIGH);
            gpioWrite(&MOTOR_1_INB, LOW);
        }
        else
        {
            gpioWrite(&MOTOR_1_INA, LOW);
            gpioWrite(&MOTOR_1_INB, HIGH);
        }

        pwmWrite(1, pwmVal);
    }
    else if(motor == 2)
    {
        if(dir == 0)
        {
            gpioWrite(&MOTOR_2_INA, HIGH);
            gpioWrite(&MOTOR_2_INB, LOW);
        }
        else
        {
            gpioWrite(&MOTOR_2_INA, LOW);
            gpioWrite(&MOTOR_2_INB, HIGH);
        }

        pwmWrite(2, pwmVal);
    }
}

uint8_t SPI_Transmit(uint8_t TxValue)
{
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = TxValue;
    while(SPI1STATbits.SPIRBF == 0);

    return SPI1BUF;
}

uint8_t SPI_Receive()
{
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;
    while(SPI1STATbits.SPIRBF == 0);

    return SPI1BUF;
}

void SPI_Accel_Write(uint8_t Addr, uint8_t Data)
{
    gpioWrite(&ACL_SS, LOW);
    SPI_Transmit(0x0A);
    SPI_Transmit(Addr);
    SPI_Transmit(Data);
    gpioWrite(&ACL_SS, HIGH);
}

uint8_t SPI_Accel_Read(uint8_t Addr)
{
    gpioWrite(&ACL_SS, LOW);
    SPI_Transmit(0x0B);
    SPI_Transmit(Addr);
    uint8_t ret = SPI_Receive();
    gpioWrite(&ACL_SS, HIGH);
    return ret;
}

//Should return the robots tilt from vertical in degrees
double getTilt()
{

    double X, Y, Z;
    double thetaX, thetaY, thetaZ;

    while((SPI_Accel_Read(ACCEL_STATUS_REG) & 0x01) == 0x00)
        SPI_Accel_Read(ACCEL_STATUS_REG);

    X = SPI_Accel_Read(ACCEL_XDATA_REG)*16;
    Y = SPI_Accel_Read(ACCEL_YDATA_REG)*16;
    Z = SPI_Accel_Read(ACCEL_ZDATA_REG)*16;

    thetaX = (atan((X)/sqrt(Y*Y + Z*Z)))*180.0/3.14;
    //thetaY = (atan((Y)/sqrt(X*X + Z*Z)))*180.0/3.14;
    //thetaZ = (atan(sqrt(X*X + Y*Y)/(Z)))*180.0/3.14;

    return thetaX;
//    return thetaY;
//    return thetaZ;
}

double PD(double PV, double SP)
{
    const double Kp = 1;
    //const double Ki = 0;
    const double Kd = 0;

    static double error = 0;
    double prevError;
    //static double integral = 0;
    double derivative;

    prevError = error;
    error = SP - PV;
    //integral = integral + error;
    derivative = error - prevError;

    double CV = (Kp * error) + (Kd * derivative);
    CV = CV > 1.0 ? 1.0 : (CV < -1.0 ? -1.0 : CV);

    return CV;
}

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
    PWM1CON1bits.PEN1L = 0;
    PWM1CON1bits.PEN1H = 1;
    PWM1CON1bits.PEN2L = 0;
    PWM1CON1bits.PEN2H = 1;
    PWM1CON1bits.PEN3L = 0;
    PWM1CON1bits.PEN3H = 0;

    PWM1CON2bits.IUE = 0;
    PWM1CON2bits.UDIS = 0;

    P1DC1 = 0;
    P1DC2 = 0;

    AD1PCFGL = 0xFFFF;
    gpioSetDir(&MOTOR_1_INA, OUTPUT);
    gpioSetDir(&MOTOR_1_INB, OUTPUT);
    gpioSetDir(&MOTOR_2_INA, OUTPUT);
    gpioSetDir(&MOTOR_2_INB, OUTPUT);

    gpioSetDir(&ACL_SS, OUTPUT);
    gpioSetDir(&ACL_SDI, INPUT);
    gpioSetDir(&ACL_SCK, OUTPUT);
    gpioSetDir(&ACL_SDO, OUTPUT);

    P1TCONbits.PTEN = 1;
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
    //TRISBbits.TRISB6 = 0;
    gpioSetDir(&ACL_SS, OUTPUT);
    gpioSetDir(&ACL_SDI, INPUT);
    gpioSetDir(&ACL_SCK, OUTPUT);
    gpioSetDir(&ACL_SDO, OUTPUT);

    SPI_Accel_Write(ACCEL_THRESHOLD_ACTIVITY_LREG, 0x00);

    SPI_Accel_Write(ACCEL_THRESHOLD_ACTIVITY_HREG, 0x00);
    SPI_Accel_Write(ACCEL_THRESHOLD_INACTIVITY_LREG, 0x00);

    SPI_Accel_Write(ACCEL_THRESHOLD_INACTIVITY_HREG, 0x00);
    SPI_Accel_Write(ACCEL_TIME_INACTIVITY_LREG, 0x0A);

    SPI_Accel_Write(ACCEL_ACTIVE_INACTIVE_CONTROL_REG, 0x05);

    SPI_Accel_Write(ACCEL_FILTER_CONTROL_REG, 0x03);
    SPI_Accel_Write(ACCEL_POWER_CONTROL_REG, 0x02);

    while(TRUE)
    {
        double angle = getTilt();
        double motorSpeed = PD(angle, 0);
        motorWrite(1, motorSpeed);
        motorWrite(2, motorSpeed);
        delay_ms(10);
    }

    return (EXIT_SUCCESS);
}
