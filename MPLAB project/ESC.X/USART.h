/* 
 * File:   EUSART.h
 * Author: Sam
 *
 * Created on 26 de Abril de 2021, 16:38
 */

#ifndef EUSART_H
#define	EUSART_H

#include <xc.h>

typedef struct
{
   uint8_t buff[24], index, avaliable;
} _USART_UART_buff;

_USART_UART_buff Ubuff;

inline void USART_setBaudRate(const uint32_t rate)
{
    if(TXSTAbits.BRGH)
        SPBRG = (_XTAL_FREQ/16/rate)-1;
    else    
        SPBRG = (_XTAL_FREQ/64/rate)-1;
}

void USART_UART_init(const uint32_t rate, uint8_t fastMode)
{
    TRISCbits.RC7 = 1;
    TRISCbits.RC6 = 1;
    TXSTAbits.BRGH = fastMode;
    USART_setBaudRate(rate);
    TXSTAbits.SYNC = 0;
    RCSTAbits.SPEN = 1;
    CREN = 1;
}

void USART_UART_transmitByte(const char data)
{
    TXEN = 1;
    while(!TXIF);
    TXREG = data;
}

void USART_UART_transmitString(char* data)
{
    while(*data) USART_UART_transmitByte(*data++);
}

void USART_UART_transmitBytes(const char* data, char dSize)
{
    for(char i = 0; i < dSize; i++) USART_UART_transmitByte(data[i]);
}

void USART_UART_doBuff()
{
    if(!RCIF) return;
    if(Ubuff.index < sizeof(Ubuff.buff)) 
    {
        Ubuff.buff[Ubuff.index++] = RCREG;
        if(Ubuff.buff[Ubuff.index-1] == 0x0A) 
        {
            Ubuff.index = 0;
            Ubuff.avaliable = 1;
        }
    }
    else {
        if(RCREG != 0x0A) return;
        Ubuff.index = 0;
        Ubuff.avaliable = 0;
    }
}

const uint8_t USART_UART_readBuffer(uint8_t* buffer, uint8_t size)
{
    if(!Ubuff.avaliable) return 0;
    
    uint8_t i;
    for(i = 0;Ubuff.buff[i] != 0x0A && i < size; i++) buffer[i] = Ubuff.buff[i];
    
    Ubuff.avaliable = 0;
    return i;
}

const uint8_t USART_UART_readString(uint8_t* buffer, uint8_t size)
{
    uint8_t result = USART_UART_readBuffer(buffer, size-1);
    
    if(!result) return 0;
    
    buffer[result] = '\0';
    return 1;
}

const uint8_t  USART_UART_receive(char* data)
{
    CREN = 1;
    if(OERR)
        CREN = 0;
    if(RCIF)
    {
        *data = RCREG;
        RCIF = 0;
        return 1;
    }
    return 0;
}

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* EUSART_H */

