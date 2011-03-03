/*
 *  uart.c
 *  TEST
 *
 *  Created by Joe  Schaack on 8/13/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "uart.h"
#include "cpu_init.h"

#if (TX_BUFFER_SIZE > 0 )
uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;
#endif 

#if (RX_BUFFER_SIZE >0)
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head =0;
volatile uint8_t rx_buffer_tail = 0;
#endif

void UART_IRQHandler(void)
{
#if ((TX_BUFFER_SIZE > 0) || (RX_BUFFER_SIZE > 0))
    switch(UART_U0IIR & UART_U0IIR_IntId_MASK)
    {
#if (TX_BUFFER_SIZE > 0)
        case UART_U0IIR_IntId_THRE:
            {
                uint8_t i = 0;
                while(i < 16)
                {
                    uint8_t tail = tx_buffer_tail;
                    uint8_t c = tx_buffer[tail];

                    UART_U0THR = c;

                    if (tail == TX_BUFFER_SIZE-1)
                        tail=0;
                    else
                        tail = tail + 1;

                    if (tail == tx_buffer_head)
                    {
                        UART_U0IER &= ~UART_U0IER_THRE_Interrupt_Enabled;//Disable The interrupt
                        tx_buffer_tail = tail;
                        break;
                    }

                    tx_buffer_tail = tail;
                    i++;
                }
            }
            break;            
#endif
#if (RX_BUFFER_SIZE > 0)
        case UART_U0IIR_IntId_CTI:            
        case UART_U0IIR_IntId_RDA:
            {
                
                
                while(UART_U0LSR & UART_U0LSR_RDR_DATA)
                {
                    uint8_t c = UART_U0RBR;
                    uint8_t head = rx_buffer_head ;
                    if(head == RX_BUFFER_SIZE-1)
                        head = 0;
                    else
                        head = head + 1;
    
                    /*Dont overwrite the buffer !! */
                    if ( head != rx_buffer_tail)
                    {
                        rx_buffer[rx_buffer_head] = c;
                        rx_buffer_head = head;
                    }
                }
                break;
            }
#endif
        default:
            
            break;
    }
#endif
}

void uartInit(uint32_t baudrate)
{
    uint32_t regVal, num;


    NVIC_DisableIRQ(UART_IRQn);

    /* Set 1.6 UART RXD */
    IOCON_PIO1_6 &= ~IOCON_PIO1_6_FUNC_MASK;
    IOCON_PIO1_6 |= IOCON_PIO1_6_FUNC_UART_RXD;

    /* Set 1.7 UART TXD */
    IOCON_PIO1_7 &= ~IOCON_PIO1_7_FUNC_MASK;	
    IOCON_PIO1_7 |= IOCON_PIO1_7_FUNC_UART_TXD;

    /* Enable UART clock */
    SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_UART);
    SCB_UARTCLKDIV = SCB_UARTCLKDIV_DIV1;     /* divided by 1 */

    /* 8 bits, no Parity, 1 Stop bit */
    UART_U0LCR = (UART_U0LCR_Word_Length_Select_8Chars |
            UART_U0LCR_Stop_Bit_Select_1Bits |
            UART_U0LCR_Parity_Disabled |
            UART_U0LCR_Parity_Select_OddParity |
            UART_U0LCR_Break_Control_Disabled |
            UART_U0LCR_Divisor_Latch_Access_Enabled);

    regVal = SCB_UARTCLKDIV;
    num = (((SYS_CLOCK * SCB_SYSAHBCLKDIV)/regVal)/16)/baudrate;

    UART_U0DLM = (num & 0xFF00) >> 8;
    UART_U0DLL = num & 0xFF;

    //   UART_U0DLM = 0x00; // for 38400 baud
    //   UART_U0DLL = 0x51;


    /* Set DLAB back to 0 */
    UART_U0LCR = (UART_U0LCR_Word_Length_Select_8Chars |
            UART_U0LCR_Stop_Bit_Select_1Bits |
            UART_U0LCR_Parity_Disabled |
            UART_U0LCR_Parity_Select_OddParity |
            UART_U0LCR_Break_Control_Disabled |
            UART_U0LCR_Divisor_Latch_Access_Disabled);

    /* Enable and reset TX and RX FIFO. 
       Call the RX interrupt when 14 chars are
       in the FIFO */
    UART_U0FCR = (UART_U0FCR_FIFO_Enabled | 
            UART_U0FCR_Rx_FIFO_Reset | 
            UART_U0FCR_Tx_FIFO_Reset |
            UART_U0FCR_Rx_Trigger_Level_Select_14Char); 

    regVal = UART_U0LSR;

    while (( UART_U0LSR & (UART_U0LSR_THRE|UART_U0LSR_TEMT)) != (UART_U0LSR_THRE|UART_U0LSR_TEMT) );
    while ( UART_U0LSR & UART_U0LSR_RDR_DATA )
    {
        /* Dump data from RX FIFO */
        regVal = UART_U0RBR;
    }
    /* If Using the RX_Buffer then enable the Interrupt
       Else disable it */
#if (RX_BUFFER_SIZE > 0)
    UART_U0IER = UART_U0IER_RBR_Interrupt_Enabled;
#else
    UART_U0IER = 0x0;
#endif

    /* If using any buffering be sure to enable the interrupt */
#if (TX_BUFFER_SIZE > 0 ) || (RX_BUFFER_SIZE > 0)
    NVIC_EnableIRQ(UART_IRQn);
#endif

    UART_U0TER = UART_U0TER_TXEN_Enabled;    
}



/*#######################################################################################
 *
 * TX Section of the code
 *
 *#####################################################################################*/

#if (TX_BUFFER_SIZE > 0 )
void uartSend(char data)
{

    if ( (!(UART_U0LSR & UART_U0LSR_THRE)) || (tx_buffer_head != tx_buffer_tail) )
    {
        uint8_t newhead;
        if (tx_buffer_head == TX_BUFFER_SIZE-1)
            newhead = 0;
        else
            newhead = tx_buffer_head + 1;

        while(newhead == tx_buffer_tail); // make sure a spot is open

        tx_buffer[tx_buffer_head] = data;
        tx_buffer_head = newhead;

        UART_U0IER |= UART_U0IER_THRE_Interrupt_Enabled;
    }
    else
        UART_U0THR = data;
}
#else
void uartSend(char data)
{
    // sit and spin untill fifo is empty.  
    while ( !(UART_U0LSR & UART_U0LSR_THRE) );
    UART_U0THR = data;
}
#endif



/*#######################################################################################
 *
 * RX Section of the code
 *
 *#####################################################################################*/

#if (RX_BUFFER_SIZE > 0)
/* Unlike other implementations this is a true or false
   Function due to the possibility of chars hiding in 
   the FIFO */
uint8_t uartDataAvailable()
{
    /*Data is available if the head is not equal to the
      tail or if data is waiting in the FIFO */
    return (rx_buffer_head != rx_buffer_tail) || (UART_U0LSR & UART_U0LSR_RDR_DATA);
}
/* returns a char from the buffer first then from the 
   FIFO */
uint8_t uartRead()
{
    if ( !uartDataAvailable() )
        return -1;
    if (rx_buffer_head != rx_buffer_tail)
    {
        uint8_t c = rx_buffer[rx_buffer_tail];
        if (rx_buffer_tail == RX_BUFFER_SIZE-1)
            rx_buffer_tail = 0;
        else 
            rx_buffer_tail++;
        return c;
    }
    else
        return UART_U0RBR;
}

#else
uint8_t uartDataAvailable() {
    return UART_U0LSR & UART_U0LSR_RDR_DATA;
}
uint8_t uartRead() {
    return UART_U0RBR;
}
#endif

/**************************************************************************/
void __putchar(const char c)
{
    uartSend(c);
}

/**************************************************************************/
int puts(const char * str)
{
    int i =0;
    while(*str)
    {
        __putchar(*str++);
        i++;
    }
    return i;
}






