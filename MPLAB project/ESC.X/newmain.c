/*
 * File:   newmain.c
 * Author: Sam
 *
 * Created on 11 de Agosto de 2021, 14:09
 */


#include <xc.h>
#include <pic18f4550.h>
#include <stdint.h> 
#include <stdio.h>
#include "FUSES.h"
#define _XTAL_FREQ 48000000
#include "USART.h"


#define n_poles 93/6*3;
//Buck setpoint
#define bSetPoint 168;

//Debuging 
#define DEBUG( text ) USART_UART_transmitString( text )

//Phases pins
#define AL_PHASE PORTAbits.RA2
#define AH_PHASE PORTAbits.RA3
#define BL_PHASE PORTAbits.RA4
#define BH_PHASE PORTAbits.RA5
#define CL_PHASE PORTEbits.RE0
#define CH_PHASE PORTEbits.RE1

//PWM abstraction
#define PWM1(v) CCPR1L =  v >> 2; CCP1CONbits.DC1B = 0x03 & v
#define PWM2(v) CCPR2L =  v >> 2; CCP2CONbits.DC2B = 0x03 & v

//Utilities
#define constrain(v, max, min) if(v > max ) v = max; else if(v < min) v = min

//Timimg
uint32_t time_us = 0,time_ms = 0, hall_ant_t = 0, rotor_ant_t = 0;

//Hall sensor and something that needs re-work
uint8_t hall = 0, driver_duty = 0, speed;

//Rotor angle estimation variables
uint16_t hall_p = 0, hall_a_p = 0, elapsed_time = 0;
uint24_t hall_dt = 10000, angular_speed = 0;
int16_t rotor_pos = 0, p_err = 0, hall_rotor_pos = 0;
uint8_t rotor_dir = 0, flag_dt = 0;

//Real Pos
uint16_t rotor_real_pos = 0;

//Buck variables
uint16_t bk_voltage = 0, buck_setPoint = 16500/95;
int16_t _buck_duty = 0, error = 0, lastError = 0, d_error = 0;
void (*buck_function)(void);

void _rt_pos_int()
{
    T0CONbits.TMR0ON = 0;
    hall_dt = TMR0; //- hall_ant_t;
    TMR0 = 0;
    T0CONbits.TMR0ON = 1;
    //hall_ant_t = TMR0;
    
    static uint16_t temp_hall_p;
    temp_hall_p = hall_rotor_pos;
    hall_p = PORTB & 0x07;
    
    //001 = 90, 011=150, 010 = 210, 110=270, 100=330, 101 = 30;
    if(hall_p == 0b001 )//&& hall_a_p == 0b101 || hall_p == 0b101 && hall_a_p == 0b001)
        hall_rotor_pos = 90/7;
    else if(hall_p == 0b101 )// && hall_a_p == 0b011 || hall_p == 0b011 && hall_a_p == 0b001)
        hall_rotor_pos = 30/7;
    else if(hall_p == 0b100)// && hall_a_p == 0b110 || hall_p == 0b110 && hall_a_p == 0b010)
        hall_rotor_pos = 330/7;
    else if(hall_p == 0b110)// && hall_a_p == 0b011 || hall_p == 0b011 && hall_a_p == 0b010)
        hall_rotor_pos = 270/7;
    else if(hall_p == 0b010 && hall_a_p == 0b110 || hall_p == 0b110 && hall_a_p == 0b100)
        hall_rotor_pos = 210/7;
    else if(hall_p == 0b011)// && hall_a_p == 0b101 || hall_p == 0b101 && hall_a_p == 0b100)
        hall_rotor_pos = 150/7;
    
//    int pos_d = hall_rotor_pos - temp_hall_p;
//    if(pos_d == 60 )
//        rotor_dir = 0;
//    else if(pos_d == -60)
//        rotor_dir = 1;
    
    hall_a_p = hall_p;
    
    //p_err += hall_rotor_pos - rotor_pos;
    //rotor_pos = hall_rotor_pos;
    flag_dt = 1;
}

//Switch from vcc to buck voltage controling current to not burn our transistor
void buck_start()
{
    bk_voltage = ADRES;
    error = (buck_setPoint/2) - bk_voltage;
    if(error > 0)
        _buck_duty++;
    else _buck_duty = 0;
    constrain(_buck_duty, 524, 0);
    PWM2(_buck_duty);
}

//Buck converter controller
void buck_continuos_controller()
{
    bk_voltage = ADRES;
    error = buck_setPoint - bk_voltage;
    if(error > 0)
        _buck_duty++;
    else _buck_duty = 0;
    constrain(_buck_duty, 1023, 0);
    
    PWM2(_buck_duty);
}

void speed_controller()
{
    driver_duty = ADRES/4;
}

void __interrupt () _int (void)
{
    if(INTCONbits.TMR0IF)
    {
        //while(1);
        //time_ms += 87;
        INTCONbits.TMR0IF = 0;
    }
    if(PIR1bits.ADIF)
    {
        PIR1bits.ADIF = 0;
        switch(ADCON0bits.CHS)
        {   
            case 1:
                buck_function();
                ADCON0 = 0b00000011;
                break;
            case 0:
                speed_controller();
                ADCON0 = 0b00000111;
                break;
        }
    }
//    if(RCIF)
//    {
//        RCIF = 0;
//        USART_UART_doBuff();
//    }
    
    if(INTCONbits.INT0IF)
    {
        INTCON2bits.INTEDG0 = !INTCON2bits.INTEDG0;
        INTCONbits.INT0IF = 0;
        _rt_pos_int();
    }
    else if(INTCON3bits.INT1IF)
    {
        INTCON2bits.INTEDG1 = !INTCON2bits.INTEDG1;
        INTCON3bits.INT1IF = 0;
        _rt_pos_int();
    }
    else if(INTCON3bits.INT2IF)
    {
        INTCON2bits.INTEDG2 = !INTCON2bits.INTEDG2;
        INTCON3bits.INT2IF = 0;
        _rt_pos_int();
    }
}

void turn_off()
{
    BL_PHASE = 0;
    BH_PHASE = 0;
    AH_PHASE = 0;
    AL_PHASE = 0;
    CL_PHASE = 0;
    CH_PHASE = 0;
}

void rotor_pos_estimation()
{
    elapsed_time = time_us - rotor_ant_t;
    rotor_ant_t = time_us;
    
    angular_speed = hall_dt/60;
    if(rotor_pos < (hall_rotor_pos + 60) && rotor_pos > (hall_rotor_pos -60))
    {
        if(rotor_dir)
            rotor_pos -= (elapsed_time/angular_speed);
        else
            rotor_pos += (elapsed_time/angular_speed);
        if(rotor_pos <= 0)
            rotor_pos += 360;
        else if(rotor_pos >= 360)
            rotor_pos -= 360;
    }
}

//inline void pwmABC(const uint8_t A, const uint8_t B, const uint8_t C)
//{
//    static uint8_t Am, Bm, Cm;
//    Am = A/2;Bm = B/2;Cm = C/2;
//    if(TMR2 >= ( 128 - Am ) && TMR2 <= ( 128 + Am ))
//    {
//        if(AL_PHASE == 1)
//        {
//            AL_PHASE = 0;
//            AL_PHASE = 0;
//        }
//        AH_PHASE = 1;
//    }
//    else
//    {
//        if(AH_PHASE == 1)
//        {
//            AH_PHASE = 0;
//            AH_PHASE = 0;
//        }
//        AL_PHASE = 1;
//    }
//    if(TMR2 >= ( 128 - Bm ) && TMR2 <= ( 128 + Bm ))
//    {
//        if(BL_PHASE == 1)
//        {
//            BL_PHASE = 0;
//            BL_PHASE = 0;
//        }
//        BH_PHASE = 1;
//    }
//    else
//    {
//        if(BH_PHASE == 1)
//        {
//            BH_PHASE = 0;
//            BH_PHASE = 0;
//        }
//        BL_PHASE = 1;
//    }
//    if(TMR2 >= ( 128 - Cm ) && TMR2 <= ( 128 + Cm ))
//    {
//        if(CL_PHASE == 1)
//        {
//            CL_PHASE = 0;
//            CL_PHASE = 0;
//        }
//        CH_PHASE = 1;
//    }
//    else
//    {
//        if(CH_PHASE == 1)
//        {
//            CH_PHASE = 0;
//            CH_PHASE = 0;
//        }
//        CL_PHASE = 1;
//    }
//}

void pwmABC(const uint8_t A, const uint8_t B, const uint8_t C)
{
    if(TMR1L > A)
    {
        if(AL_PHASE == 1)
        {
            AL_PHASE = 0;
            __delay_us(1);
        }
        AH_PHASE = 1;
    }
    else
    {
        if(AH_PHASE == 1)
        {
            AH_PHASE = 0;
            __delay_us(1);
        }
        AL_PHASE = 1;
    }
    if(TMR1L > B)
    {
        if(BL_PHASE == 1)
        {
            BL_PHASE = 0;
            __delay_us(1);
        }
        BH_PHASE = 1;
    }
    else
    {
        if(BH_PHASE == 1)
        {
            BH_PHASE = 0;
            __delay_us(1);
        }
        BL_PHASE = 1;
    }
    if(TMR1L > C)
    {
        if(CL_PHASE == 1)
        {
            CL_PHASE = 0;
            __delay_us(1);
        }
        CH_PHASE = 1;
    }
    else
    {
        if(CH_PHASE == 1)
        {
            CH_PHASE = 0;
            __delay_us(1);
        }
        CL_PHASE = 1;
    }
}

/*These are the controllers from */
//Here we use the hall sensors to comutate between the six sectors
//of eletrical revolution
void trapzoidal_control()
{
    //Get HALL C-B-A in this order
    hall = PORTB & 0x07;
    static int trap_duty;
    
    if(flag_dt)
    {
        flag_dt = 0;
        trap_duty += (hall_dt - driver_duty*2+400);
    }
    else if(TMR0 > (hall_dt << 2)) trap_duty = 253;
    
    constrain(trap_duty, 253, 2);   
    
    if(TMR1L <= driver_duty/2)
    {
        switch(hall)
        {
            case 0b001:
                AL_PHASE = 0;
                AH_PHASE = 0;
                BH_PHASE = 0;
                CL_PHASE = 0;
                BL_PHASE = 1;
                CH_PHASE = 1;
                break;
            case 0b101:
                BH_PHASE = 0;
                BL_PHASE = 0;
                AH_PHASE = 0;
                CL_PHASE = 0;
                AL_PHASE = 1;
                CH_PHASE = 1;
                break;
            case 0b100:
                CH_PHASE = 0;
                CL_PHASE = 0;
                BL_PHASE = 0;
                AH_PHASE = 0;
                BH_PHASE = 1;
                AL_PHASE = 1;
                break;
            case 0b110:
                AH_PHASE = 0;
                AL_PHASE = 0;
                CH_PHASE = 0;
                BL_PHASE = 0;
                CL_PHASE = 1;
                BH_PHASE = 1;
                break;
            case 0b010:
                BL_PHASE = 0;
                BH_PHASE = 0;
                AL_PHASE = 0;
                CH_PHASE = 0;
                AH_PHASE = 1;
                CL_PHASE = 1;
                break;
            case 0b011:
                CL_PHASE = 0;
                CH_PHASE = 0;
                BH_PHASE = 0;                   
                AL_PHASE = 0;
                BL_PHASE = 1;
                AH_PHASE = 1;
               break;
            default:
                BL_PHASE = 0;
                BH_PHASE = 0;
                AH_PHASE = 0;
                AL_PHASE = 0;
                CL_PHASE = 0;
                CH_PHASE = 0;
        }
        //Counter clockwise rotation
//        switch(hall)
//        {
//            case 0b001:
//                AL_PHASE = 0;
//                AH_PHASE = 0;
//                BL_PHASE = 0;
//                BH_PHASE = 1;
//                CH_PHASE = 0;
//                CL_PHASE = 1;
//                break;
//            case 0b101:
//                BH_PHASE = 0;
//                BL_PHASE = 0;
//                AL_PHASE = 0;
//                AH_PHASE = 1;
//                CH_PHASE = 0;
//                CL_PHASE = 1;
//                break;
//            case 0b100:
//                CH_PHASE = 0;
//                CL_PHASE = 0;
//                BH_PHASE = 0;
//                BL_PHASE = 1;
//                AL_PHASE = 0;
//                AH_PHASE = 1;
//                break;
//            case 0b110:
//                AH_PHASE = 0;
//                AL_PHASE = 0;
//                CL_PHASE = 0;
//                CH_PHASE = 1;
//                BH_PHASE = 0;
//                BL_PHASE = 1;
//                break;
//            case 0b010:
//                BL_PHASE = 0;
//                BH_PHASE = 0;
//                AH_PHASE = 0;
//                AL_PHASE = 1;
//                CL_PHASE = 0;
//                CH_PHASE = 1;
//                break;
//            case 0b011:
//                CL_PHASE = 0;
//                CH_PHASE = 0;
//                BL_PHASE = 0;
//                BH_PHASE = 1;
//                AH_PHASE = 0;
//                AL_PHASE = 1;
//               break;
//            default:
//                BL_PHASE = 0;
//                BH_PHASE = 0;
//                AH_PHASE = 0;
//                AL_PHASE = 0;
//                CL_PHASE = 0;
//                CH_PHASE = 0;
//        }
    }
    else
    {
        AL_PHASE = 0;
        BL_PHASE = 0;
        CL_PHASE = 0;
        
//        switch(hall)
//        {
//            case 0b001:
//                CH_PHASE = 0;
//                AL_PHASE = 0;
//                AH_PHASE = 0;
//                BH_PHASE = 0;
//                BL_PHASE = 0;
//                CL_PHASE = 1;
//                break;
//            case 0b101:
//                CH_PHASE = 0;
//                BH_PHASE = 0;
//                BL_PHASE = 0;
//                AH_PHASE = 0;
//                AL_PHASE = 0;
//                CL_PHASE = 1;
//                break;
//            case 0b100:
//                BH_PHASE = 0;
//                CH_PHASE = 0;
//                CL_PHASE = 0;
//                AH_PHASE = 0;
//                AL_PHASE = 0;
//                BL_PHASE = 1;
//                break;
//            case 0b110:
//                BH_PHASE = 0;
//                AH_PHASE = 0;
//                CH_PHASE = 0;
//                AL_PHASE = 0;
//                CL_PHASE = 0;
//                BL_PHASE = 1;
//                break;
//            case 0b010:
//                AH_PHASE = 0;
//                BL_PHASE = 0;
//                BH_PHASE = 0;
//                CH_PHASE = 0;
//                CL_PHASE = 0;
//                AL_PHASE = 1;
//                break;
//            case 0b011:
//                AH_PHASE = 0;
//                CL_PHASE = 0;
//                CH_PHASE = 0;
//                BH_PHASE = 0;
//                BL_PHASE = 0;
//                AL_PHASE = 1;
//               break;
//            default:
//                AH_PHASE = 0;
//                BH_PHASE = 0;
//                CH_PHASE = 0;
//                AL_PHASE = 0;
//                BL_PHASE = 0;
//                CL_PHASE = 0;
//        }
    }
        
}

//Sine lookup table
//const uint8_t pwmSin48[] = {127,110,94,78,64,50,37,26,17,10,4,1,0,1,4,10,17,
//                           26,37,50,64,78,94,110,127,144,160,176,191,204,217,228,237,
//                           244,250,253,255,253,250,244,237,228,217,204,191,176,160,144,127};

int pwmSin48[] = {127,110,94,78,64,50,37,26,17,10,4,1,0,
                1,4,10,17,26,37,50,64,78,94,110,127,
                144,160,176,191,204,217,228,237,244,250,253,254,255,
                250,244,237,228,217,204,191,176,160,144 };

const uint8_t senThirthHarmonic[60] = { 127, 147, 166, 184, 199, 212, 222, 230,
                                        234, 237, 237, 237, 236, 235, 234, 233,
                                        234, 235, 236, 237, 237, 237, 234, 230,
                                        222, 212, 199, 184, 166, 147, 127, 107,
                                        88, 70, 55, 42, 32, 24, 20, 17, 17, 17,
                                        18, 19, 20, 21, 20, 19, 18, 17, 17, 17,
                                        20, 24, 32, 42, 55, 70, 88, 107 };

uint8_t stepA = 0, stepB = 16, stepC = 32;

//Open loop control sinusoidal. we need just adjust manually the parameters
void open_loop_sinusoidal_control()
{   
    //static uint8_t hall_T = 0;
    //hall_T = PORTB & 0x07;
    
    //Just to be safe than sorry later i'll edit all that shit just for test a supose
    //if(hall_T == 0 || hall_T == 7) return;
    
    
    static uint8_t stator_angle;
    static uint16_t counterZ;
    stator_angle = 0;//hall_rotor_pos+4;
//    
//    static uint8_t noPWM = 0;
//    if(!noPWM)
        pwmABC( pwmSin48[stepA]>>2, pwmSin48[stepB]>>2, pwmSin48[stepC]>>2 );
    
    counterZ++;
    
    if( counterZ > driver_duty )
    {
//        if(rotor_real_pos < driver_duty*4)
//        {   
            //noPWM = 0;
//            rotor_real_pos++;
            //stator_angle++;
//        }
//        else if(rotor_real_pos > driver_duty*4)
//        {
//            noPWM = 0;
//            rotor_real_pos--;
//            rotor_angle = rotor_angle == 0 ? (sizeof(pwmSin48)-1):--rotor_angle;
//        }
//        else
//            noPWM = 1;
        counterZ = 0;
        stepA++;
        stepB++;
        stepC++;
        stepA %= 48;stepB %= 48;stepC %= 48;
    }
      //  counterZ = 0;
    //}
}

//In this type of vector controller that produces maximum torque cause
//the rotor vector and stator vector are locked orthogonaly to each other. 
void closed_loop_sinusoidal_control()
{
    
}

//This one is the most advanced and complex controller, 
//here we control the rotor and stator vectors.
//mathematicaly: park transform, inverse park transform and clarke transform
//sensor: current sensor in at least two phases. And we need some way to get rotor position.
//Clarke transformation transform the three axis phase into a 
//two axis phase cause it's a plane so we need just two axis to represent it.
//Park transform is used to "cancel" the sine format of clarke
//transform essentially transforming from the stator point of view to
//rotor point. Doc folder in this project has a gif of thoose transforms
void field_oriented_control()
{
    
}


void speed_measure()
{
    static uint8_t last_hall = 0, counter;
    static uint32_t last_time = 0;
    if( last_hall != hall )
    {
        if(++counter == 45)
        {
            speed = (time_ms - last_time)/1000;
            last_time = time_ms;
        }
        
    }
    last_hall = hall;
}

void main(void) 
{
    //Wait for peaks!
    __delay_ms(10);
    //Interrupt config
    INTCON = 0b11000000;
    INTCON3 = 0b00011000;
    
    RCONbits.IPEN = 0;
    INTCON2 |= 0b01110000;
    //INTCONbits.TMR0IE = 1;
    //PIE1bits.RCIE = 1;
    PIE1bits.ADIE = 1;
    
    //ADC config
    ADCON1 = 0x0D;
    ADCON2 = 0b10101110;
    ADCON0 = 0b00000111;
    
    //PORT config
    LATA = 0;
    LATE = 0;
    TRISB = 0xFF;
    TRISA = 0b11000011;
    TRISE = 0b11111100;
    //Just to be safe than sorry!
    LATCbits.LATC0 = 0;
    TRISCbits.RC0 = 0;
    
    //PORTB pull-up
    RBPU = 0;
    
    //Base time 1.3uS++ 87.4ms overflow
    T0CON = 0b10001011;
    //Buck PWM ~47kHz for 10-bit resoulution
    buck_function = &buck_start;
    LATBbits.LATB3 = 0;
    TRISBbits.RB3 = 0;
    CCP2CON = 0b00001101;
    PR2 = 255;
    PWM2(0);
    T2CON = 0b00000100;
    T1CON = 0b00000101;
    
    //Bluetooth 
    USART_UART_init(115200, 1);
    //reset PORTA
    LATA &= 0b11000011;
    
    //Voltage level acceptable to start working and
    //change buck to normal controller
    while(bk_voltage <= buck_setPoint/2 || bk_voltage > buck_setPoint+20);
    _buck_duty = 0;
    buck_function = &buck_continuos_controller;
    LATCbits.LATC0 = 1;
    
    uint16_t count = 0;
    char text[64] = "A:0 B:0 C:0\n";
    uint8_t last_hal = 0, last_hal1 = 0, last_hal2 = 0, hals;  
    while(1)
    {
        //Hangs cause we fucked up !
        while(bk_voltage > 189 || bk_voltage < 126) turn_off();
      //  speed_measure();
//        text[2] =  0x30 + PORTBbits.RB0+0;
//        text[6] =  0x30 + PORTBbits.RB1+0;
//        text[10] = 0x30 + PORTBbits.RB2;
        //rotor_pos_estimation();
        
        //Field oriented control later <3 ! or not '-' I need current sensor T-T
        //Edit 1: hehe we are working on it even it's looks like we neeed another micro!
        //Six step comutation, trapezoidal control BLDC
        //constrain(driver_duty, 250, 5);
//        while(count)
//        {
//            trapzoidal_control();
//            count--;
//        }
//        turn_off();
        open_loop_sinusoidal_control();
        //sprintf(text, "bk: %05d, ADC:%05d, speed:%03d\n", buck_duty, bk_voltage, driver_duty);
//        hals = PORTB & 07;
//        if(hals != last_hal && hals != 000 && hals != 111) 
//        {
//            count++;
//            last_hal2 = last_hal1;
//            last_hal1 = last_hal;
//            last_hal = hals;
//        }
        
        //sprintf(&text[11], " rot: %03d, count: %d\n", rotor_real_pos, driver_duty);
        //DEBUG(text);
    }
}
