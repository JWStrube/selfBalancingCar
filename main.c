/*
 * File:   main.c
 * Author: john
 *
 * Created on April 26, 2017, 12:19 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>
#include <math.h>
#include <libpic30.h> //For delay function

//Configuration FOSC
#pragma config FCKSM = 3
#pragma config OSCIOFNC = 0
#pragma config POSCMD = 3

#pragma config IESO = 0
#pragma config FNOSC = 7

//PWM
#pragma config PWMPIN = 0
#pragma config HPOL = 1
#pragma config LPOL = 1
#pragma config FWDTEN = 0

#pragma config JTAGEN = 0

#define OUTPUT 0
#define INPUT 1
#define HIGH 1
#define LOW 0

#define PPSUnLock __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock __builtin_write_OSCCONL(OSCCON | 0x40)

//Accelerometer registers
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
#define ACCEL_XDATA_L_REG 0x0E

//Delay
#define FOSC  8000000LL  // clock-frequecy in Hz with suffix LL (64-bit-long), eg. 32000000LL for 32MHz
#define FCY       (FOSC/2)  // MCU is running at FCY MIPS
#define delay_us(x) __delay32(((x*FCY)/1000000L)) // delays x us
#define delay_ms(x) __delay32(((x*FCY)/1000L))  // delays x ms

#define MOTOR_MAX_VOLTAGE 6.0
#define PSU_VOLTAGE 12.0

#define PI 3.14159265358979323846

//Port RAX or RBX
typedef enum {
    RA = 0,
    RB
} port_t;

//Struct for GPIO pin
typedef struct {
    const port_t port; //RA or RB
    const uint8_t pin;
    uint8_t dir;
} gpio_t;

//IO pin declarations
gpio_t MOTOR_1_INA = {RA, 0, OUTPUT};
gpio_t MOTOR_1_EN = {RA, 1, OUTPUT};
gpio_t MOTOR_2_INA = {RB, 0, OUTPUT};
gpio_t MOTOR_2_EN = {RB, 1, OUTPUT};
gpio_t ACCEL_CS = {RB, 6, OUTPUT};
gpio_t ACL_SDI = {RB, 2, INPUT}; //I dont think this is necessary but it helps me remember what pin its on
gpio_t ACL_SCK = {RB, 3, OUTPUT}; 
gpio_t ACL_SDO = {RB, 5, OUTPUT}; 

//Set the direction of the pin
bool gpioSetDir(gpio_t * pin, uint8_t dir) {
    pin->dir = dir;
    uint8_t ret = TRUE;
    if (pin->port == RA) {
        if (dir == OUTPUT) {
            TRISA &= ~(1 << pin->pin);
        } else {
            TRISA |= (1 << pin->pin);
        }
    } else if (pin->port == RB) {
        if (dir == OUTPUT) {
            TRISB &= ~(1 << pin->pin);
        } else {
            TRISB |= (1 << pin->pin);
        }
    } else {
        ret = false;
    }

    return ret;
}

//GPIO output
bool gpioWrite(gpio_t * pin, uint8_t val) {
    uint8_t ret = TRUE;
    if (pin->dir == OUTPUT) {
        if (pin->port == RA) {
            if (val == LOW) {
                LATA &= ~(1 << pin->pin);
            } else {
                LATA |= (1 << pin->pin);
            }
        } else if (pin->port == RB) {
            if (val == LOW) {
                LATB &= ~(1 << pin->pin);
            } else {
                LATB |= (1 << pin->pin);
            }
        } else {
            ret = false;
        }
    } else {
        ret = false;
    }

    return ret;
}

//GPIO input
bool gpioRead(gpio_t * pin) {
    uint8_t ret;
    if (pin->port == RA) {
        ret = (PORTA & (1 << pin->pin)) ? 1 : 0;
    } else if (pin->port == RB) {
        ret = (PORTB & (1 << pin->pin)) ? 1 : 0;
    }

    return ret;
}

//PWM output where value is 0 to 255
void pwmWrite(uint8_t pin, uint8_t value) {
    if (pin == 1) {
        P1DC1 = value;
    } else if (pin == 2) {
        P1DC2 = value;
    }
}
//Motor control where value is -1.0 to 1.0
void motorWrite(uint8_t motor, float value) {
    uint8_t dir = value >= 0 ? 1 : 0; //Check sign on value then set dir
    //Because our power supply is 12V and our motors are 6V, duty cycle must be below 50% at all times
    //PWM is set to be 8 bits
    uint8_t pwmVal = (float) (value < 0 ? -value : value) * (MOTOR_MAX_VOLTAGE / PSU_VOLTAGE) * 255.0;

    /*Motor driver truth table:    
     * EN | INA | INB | Output
     * 0    X       X   Motor Off
     * 1    1       0   Motor Forward
     * 1    1       1   Motor Brake
     * 1    0       1   Motor Reverse
     * 1    0       0   Motor Brake
     * 
     * Motor control works by switching between Forward/Reverse and Brake, with 
     * PWM on INB
     * If we pulsed EN instead, when the H bridge turns off, the output voltage 
     * slowly ramps down, instead of switching between 12V/-12V and 0V quickly.
     * With a high enough frequency on the PWM, voltage would stay close to 12V/-12V
     * Even at a 50% duty cycle. This caused us to fry one of our motors.
     * */
    if (motor == 1) {
        if (dir == 1) {
            gpioWrite(&MOTOR_1_INA, HIGH); 
            gpioWrite(&MOTOR_1_EN, (value == 0 ? LOW : HIGH));
            pwmVal = 255 - pwmVal; //Invert the duty cycle for forward, because 0 at INB will cause the motor to go           
        } else {
            gpioWrite(&MOTOR_1_INA, LOW);
            gpioWrite(&MOTOR_1_EN, (value == 0 ? LOW : HIGH));

        }

        pwmWrite(1, pwmVal);
    } else if (motor == 2) {
        if (dir == 1) {
            gpioWrite(&MOTOR_2_INA, HIGH);
            gpioWrite(&MOTOR_2_EN, (value == 0 ? LOW : HIGH));
            pwmVal = 255 - pwmVal;
        } else {
            gpioWrite(&MOTOR_2_INA, LOW);
            gpioWrite(&MOTOR_2_EN, (value == 0 ? LOW : HIGH));
        }

        pwmWrite(2, pwmVal);
    }
}

//Transmit byte on SPI
uint8_t SPI_Transmit(uint8_t TxValue) {
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = TxValue;
    while (SPI1STATbits.SPIRBF == 0);

    return SPI1BUF;
}

//Receive byte on SPI
uint8_t SPI_Receive() {
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;
    while (SPI1STATbits.SPIRBF == 0);

    return SPI1BUF;
}

//Write a byte to the accelerometer at register Addr
void SPI_Accel_Write(uint8_t Addr, uint8_t Data) {
    gpioWrite(&ACCEL_CS, LOW);
    SPI_Transmit(0x0A);
    SPI_Transmit(Addr);
    SPI_Transmit(Data);
    gpioWrite(&ACCEL_CS, HIGH);
}

//Read byte from accelerometer at register Addr
uint8_t SPI_Accel_Read(uint8_t Addr) {
    gpioWrite(&ACCEL_CS, LOW);
    SPI_Transmit(0x0B);
    SPI_Transmit(Addr);
    uint8_t ret = SPI_Receive();
    gpioWrite(&ACCEL_CS, HIGH);
    return ret;
}

//Read n number of registers from accelerometer starting at register Addr
//Accelerometer automatically increments register pointer
void SPI_Accel_Read_Bytes(uint8_t Addr, uint8_t numBytes, uint8_t * data) {
    gpioWrite(&ACCEL_CS, LOW);
    SPI_Transmit(0x0B);
    SPI_Transmit(Addr);
    uint8_t i;
    for (i = 0; i < numBytes; i++) {
        data[i] = SPI_Receive();
    }
    gpioWrite(&ACCEL_CS, HIGH);
}

//Normalize a 3D vector
void normalize(float * X, float * Y, float *Z) {
    float magnitude = sqrt(*X * *X + *Y * *Y + *Z * *Z);
    *X /= magnitude;
    *Y /= magnitude;
    *Z /= magnitude;
}

//Returns the robots tilt from the vertical in degrees
double getTilt() {

    float X, Y, Z;
    double thetaX, thetaY, thetaZ;
    
    //Make sure accelerometer is on the bus
    if ((SPI_Accel_Read(ACCEL_STATUS_REG) & 0x01) != 0x0) {

        //Read the 12 bit sign extended X Y and Z acceleration data from the accelerometer
        uint8_t accelData[6];
        SPI_Accel_Read_Bytes(ACCEL_XDATA_L_REG, 6, accelData);
        X = (float) ((((int16_t) accelData[1] << 8)) | accelData[0]); //Max 1140 min -1736
        Y = (float) ((((int16_t) accelData[3] << 8)) | accelData[2]); //Max 1575 min -1457
        Z = (float) ((((int16_t) accelData[5] << 8)) | accelData[4]); //max 1568 min -620
        /*
        Because the accelerometer is not perfect, we must apply offset and scale corrections
        Offset ensures that a value of 0 is 0 m/s^2 of  acceleration
        Scale ensures that the same acceleration from each axis results in the 
        Same number
        These numbers were calculated by hand, by observing the min and max values
        Of each axis
        1G should be close to 1000, but its only important that all axis
         have the same scale, since the values will be normalized so that 
         the gravity vector is of magnitude 1
        */
        const float XOffset = 596.0;
        const float YOffset = -118.0;
        const float ZOffset = -948;
        const float XScale = 0.695;
        const float YScale = 0.660;
        const float ZScale = 0.914;

        X += XOffset;
        Y += YOffset;
        Z += ZOffset;
        X *= XScale;
        Y *= YScale;
        Z *= ZScale;

        normalize(&X, &Y, &Z);

        thetaX = (atan2(Y, Z)) * (180.0 / PI); //Estimation of roll about the X axis

        return thetaX;
    } else {
        return 0;
    }


}

//PID loop for self balancing
//PV is process variable, or the actual tilt of the robot
//SP is set point, or the angle we want it to balance out
double PID(double PV, double SP) {
    //Kp, Ki, Kd values are not final and must be properly tuned for successful self balancing
    const double Kp = 0.035;
    const double Ki = 0.02;
    const double Kd = 0.0;

    static double error = 0;
    double prevError;
    static double integral = 0;
    double derivative;

    prevError = error;
    error = SP - PV; 
    integral = integral + error;
    derivative = error - prevError;

    double CV = (Kp * error) + (Kd * derivative / .01) + (Ki * integral * .01);
    //CV should be from -1.0 to 1.0
    CV = CV > 1.0 ? 1.0 : (CV < -1.0 ? -1.0 : CV);

    return CV;
}

int main(int argc, char** argv) {

    //8 Mhz
    OSCTUNbits.TUN = 23;
    //PWM needs to be fast for smooth motor control, so no division
    CLKDIVbits.FRCDIV = 0;
    CLKDIVbits.DOZE = 0;
    CLKDIVbits.DOZEN = 1;

    //PWM stuff
    P1TCONbits.PTEN = 0;
    P1TCONbits.PTCKPS = 0;
    P1TCONbits.PTMOD = 0;

    P1TMRbits.PTMR = 0;
    P1TPER = 128; //How high PTMR will count for each period. Since I want PWM to be
    //Only 8 bits, set to 256/2

    //Enable 1H and 2H
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

    AD1PCFGL = 0xFFFF; //ADC pins are digital
    //Set direction for all GPIO
    gpioSetDir(&MOTOR_1_INA, OUTPUT);
    gpioSetDir(&MOTOR_1_EN, OUTPUT);
    gpioSetDir(&MOTOR_2_INA, OUTPUT);
    gpioSetDir(&MOTOR_2_EN, OUTPUT);

    gpioSetDir(&ACCEL_CS, OUTPUT);
    gpioSetDir(&ACL_SDI, INPUT);
    gpioSetDir(&ACL_SCK, OUTPUT);
    gpioSetDir(&ACL_SDO, OUTPUT);

    P1TCONbits.PTEN = 1;
    //SPIConfiguration MASTER mode sending 8 bits
    SPI1CON1bits.DISSCK = 0;
    SPI1CON1bits.DISSDO = 0;
    SPI1CON1bits.MODE16 = 0; //byte-wide
    SPI1CON1bits.SSEN = 0;
    SPI1CON1bits.MSTEN = 1; //Master mode
    SPI1CON1bits.SMP = 0; //Input data is sampled at the middle of data output time
    SPI1CON1bits.CKE = 1;
    SPI1CON1bits.CKP = 0; 
    SPI1CON1bits.PPRE = 1;
    SPI1CON1bits.SPRE = 7;
    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPIEN = 1;

    //Peripheral Pin Select with RP pins
    PPSUnLock;
    RPOR1bits.RP3R = 8; //SCK1 RP3

    //RPINR20bits.SCK1R = 8;
    RPOR2bits.RP5R = 7; //SDO RP5
    RPINR20bits.SDI1R = 2; //SDI RP2
    //RPOR3bits.RP6R = 9;
    PPSLock;

    //Initialize accelerometer
    SPI_Accel_Write(ACCEL_THRESHOLD_ACTIVITY_LREG, 0x00);

    SPI_Accel_Write(ACCEL_THRESHOLD_ACTIVITY_HREG, 0x00);
    SPI_Accel_Write(ACCEL_THRESHOLD_INACTIVITY_LREG, 0x00);

    SPI_Accel_Write(ACCEL_THRESHOLD_INACTIVITY_HREG, 0x00);
    SPI_Accel_Write(ACCEL_TIME_INACTIVITY_LREG, 0x0A);

    SPI_Accel_Write(ACCEL_ACTIVE_INACTIVE_CONTROL_REG, 0x05);

    SPI_Accel_Write(ACCEL_FILTER_CONTROL_REG, 0x03);
    SPI_Accel_Write(ACCEL_POWER_CONTROL_REG, 0x02);

    while (true) {
        double angle = getTilt(); //Returns tilt of rover
        double motorSpeed = PID(angle, 0); //Returns how fast to run motors
        //motorWrite(1, 1); //test
        motorWrite(1, motorSpeed); //Write to both motors
        motorWrite(2, motorSpeed);
        delay_ms(10); //Delay for 10ms
    }

    return (EXIT_SUCCESS);
}
