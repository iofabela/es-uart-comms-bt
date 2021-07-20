/**********************************************************

        Student:    Israel Fabela
        P:          se_t1_FabelaPerez.c
        Device:     Tiva C - TM4C1294NCPDT
        Subject:    Embedded Systems        *Semester: 2021-1
        Topic:      UART
        HM:         "Development of wireless control of
                            drop-down messages"

***********************************************************/
//Library
#include <stdint.h>
//#include "inc/tm4c1294ncpdt.h"

//Create definitions for Port M registers - Display
#define GPIO_PORTM_DATA_RM       (*((volatile unsigned long *)0x400633FC))
#define GPIO_PORTM_DIR_RM        (*((volatile unsigned long *)0x40063400))
#define GPIO_PORTM_PUR_RM        (*((volatile unsigned long *)0x40063510))
#define GPIO_PORTM_DEN_RM        (*((volatile unsigned long *)0x4006351C))
#define GPIO_PORTM_CR_RM         (*((volatile unsigned long *)0x40063524))
#define GPIO_PORTM_AMSEL_RM      (*((volatile unsigned long *)0x40063528))
#define GPIO_PORTM_PCTL_RM       (*((volatile unsigned long *)0x4006352C))

//Create definitions for Port K registers - Digits
#define GPIO_PORTK_DATA_RK       (*((volatile unsigned long *)0x400613FC))
#define GPIO_PORTK_DIR_RK        (*((volatile unsigned long *)0x40061400))
#define GPIO_PORTK_PUR_RK        (*((volatile unsigned long *)0x40061510))
#define GPIO_PORTK_DEN_RK        (*((volatile unsigned long *)0x4006151C))
#define GPIO_PORTK_CR_RK         (*((volatile unsigned long *)0x40061524))
#define GPIO_PORTK_AMSEL_RK      (*((volatile unsigned long *)0x40061528))
#define GPIO_PORTK_AFSEL_RK      (*((volatile unsigned long *)0x40061420))
#define GPIO_PORTK_PCTL_RK       (*((volatile unsigned long *)0x4006152C))

//Create definitions for Port C registers
#define GPIO_PORTC_AFSEL_R      (*((volatile uint32_t *)0x4005A420))
#define GPIO_PORTC_DEN_R        (*((volatile uint32_t *)0x4005A51C))
#define GPIO_PORTC_AMSEL_R      (*((volatile uint32_t *)0x4005A528))
#define GPIO_PORTC_PCTL_R       (*((volatile uint32_t *)0x4005A52C))

//Create definitions for UART
#define UART7_DR_R              (*((volatile uint32_t *)0x40013000))
#define UART7_FR_R              (*((volatile uint32_t *)0x40013018))

#define UART_FR_TXFF            0x00000020  // FIFO TX UART llena
#define UART_FR_RXFE            0x00000010  // FIFO RX UART vacia
#define UART7_IBRD_R            (*((volatile uint32_t *)0x40013024)) //Divisor de entero de Baud Rate UART (p.1184)
#define UART7_FBRD_R            (*((volatile uint32_t *)0x40013028)) //Divisor de fracción de Baud Rate UART (p.1185)
#define UART7_LCRH_R            (*((volatile uint32_t *)0x4001302C)) // Control de linea UART (p.1186)

#define UART_LCRH_WLEN_8        0x00000060  // palabra de 8 bits
#define UART_LCRH_FEN           0x00000010  // Habilitación de la FIFO de la UART

#define UART7_CTL_R             (*((volatile uint32_t *)0x40013030)) //Control UART (p.1188)
#define UART7_CC_R              (*((volatile uint32_t *)0x40013FC8)) // Configuración de control (p.1213)
#define UART_CC_CS_M            0x0000000F  // UART fuente del reloj de Baud Rate
#define UART_CC_CS_SYSCLK       0x00000000  // Sistema de reloj (basado en fuente de reloj y factor de división)                    // source and divisor factor)
#define UART_CC_CS_PIOSC        0x00000005  // PIOSC
#define SYSCTL_ALTCLKCFG_R      (*((volatile uint32_t *)0x400FE138))//Configuración de reloj alterno (p.280)
#define SYSCTL_ALTCLKCFG_ALTCLK_M                                             \
                                0x0000000F  // Fuente alternativa de reloj
#define SYSCTL_ALTCLKCFG_ALTCLK_PIOSC                                         \
                                0x00000000  // PIOSC
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608)) // Control de reloj GPIO  (p.382)
#define SYSCTL_RCGCUART_R       (*((volatile uint32_t *)0x400FE618)) // Control de reloj UART (p.388)
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08)) // Estado de GPIO (p.499)
//#define SYSCTL_PRGPIO_R0        0x00000004  // Puerto GPIO C listo
#define SYSCTL_PRUART_R         (*((volatile uint32_t *)0x400FEA18))//Estado del UART (p.505)
#define SYSCTL_PRUART_R0        0x00000080  // UART Module 7 del UART listo

volatile uint8_t uart;

void init(void);
void delay(void);
void msg0(void);
void msg1(void);

//Initialize the Ports
void init(void){

    SYSCTL_RCGCUART_R |=  0x00000080; // activa el reloj para el UART7 (p.388)

    while((SYSCTL_PRUART_R&SYSCTL_PRUART_R0) == 0){}; // Se espera a que el reloj se estabilice (p.505)
    UART7_CTL_R &= ~0x00000001; // se deshabilita el UART (p.1188)
    UART7_IBRD_R = 104;  // IBRD = int(9,600 / (16 * 115,200)) = int(104.1667) (p.1184)
    UART7_FBRD_R = 11; // FBRD = round(0.1667 * 64) = 44 (p. 1185)
    // Palabra de 8 bits (sin bits de paridad, un bit stop, FIFOs) (p. 1186)
    UART7_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
    // UART toma su reloj del la fuente alterna como se define por SYSCTL_ALTCLKCFG_R (p. 1213)
    UART7_CC_R = (UART7_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC;
    // La fuente de reloj alterna es el PIOSC (default)(P. 280)
    SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC; //
    // alta velocidad deshabilitada;divide el reloj por 16 en lugar de 8 (default)(1208)
    UART7_CTL_R &= ~0x00000020;
    UART7_CTL_R |= 0x00000001;  // habilita el UART (p.1208)

    SYSCTL_RCGCGPIO_R |= 0xE06;       // Clock Enable, Port K, M and C
    while ((SYSCTL_RCGCGPIO_R & 0xE06) == 0){};

    //Initialize Port M - Display
    GPIO_PORTM_CR_RM = 0x7F;
    GPIO_PORTM_AMSEL_RM = 0x00;        // Disable analog function
    GPIO_PORTM_PCTL_RM = 0x00000000;   // GPIO clear bit PCTL
    GPIO_PORTM_DIR_RM = 0xFF;          // Set PM6-PM0 outputs
    GPIO_PORTM_PUR_RM = 0x00;          // Enable pullup resistors
    GPIO_PORTM_DEN_RM = 0xFF;          // Enable digital pins PM6-PM0


    //Initialize Port K - Digits
    GPIO_PORTK_CR_RK = 0x7F;
    GPIO_PORTK_AMSEL_RK = 0x00;        // Disable analog function
    GPIO_PORTK_PCTL_RK = 0x00000000;   // GPIO clear bit PCTL
    GPIO_PORTK_DIR_RK = 0xFF;          // PK3-PK0 output
    GPIO_PORTK_PUR_RK = 0x00;          // Disable pullup resistors
    GPIO_PORTK_DEN_RK = 0xFF;          // Enable digital pins PK3-PK0

    //Initialize Port C - UART
    GPIO_PORTC_AFSEL_R |= 0x30; // habilita funcion alterna en PC5-4
    GPIO_PORTC_DEN_R  |= 0x30;  // habilita digital I/O en PC5-4
    // configura PC5-4 como UART
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R &0xFF00FFFF)+0x00110000;
    GPIO_PORTC_AMSEL_R &= ~0x30; // deshabilita la funcionabilidad analogica de PC

}

//Messages
void msg0(void){

    GPIO_PORTM_DATA_RM = 0x5E;       //Write D
    GPIO_PORTK_DATA_RK = 0x1; //Write in Digit 1
    delay();

    GPIO_PORTM_DATA_RM = 0x39;       //Write C
    GPIO_PORTK_DATA_RK = 0x2; //Write in Digit 2
    delay();

    GPIO_PORTM_DATA_RM = 0x7C;       //Write B
    GPIO_PORTK_DATA_RK = 0x4; //Write in Digit 3
    delay();

    GPIO_PORTM_DATA_RM = 0x77;       //Write A
    GPIO_PORTK_DATA_RK = 0x8; //Write in Digit 4
    delay();
}

void msg1(void){

    GPIO_PORTM_DATA_RM = 0x66;       //Write 4
    GPIO_PORTK_DATA_RK = 0x1; //Write in Digit 1
    delay();

    GPIO_PORTM_DATA_RM = 0x4F;       //Write 3
    GPIO_PORTK_DATA_RK = 0x2; //Write in Digit 2
    delay();

    GPIO_PORTM_DATA_RM = 0x5B;       //Write 2
    GPIO_PORTK_DATA_RK = 0x4; //Write in Digit 3
    delay();

    GPIO_PORTM_DATA_RM = 0x06;       //Write 1
    GPIO_PORTK_DATA_RK = 0x8; //Write in Digit 4
    delay();
}

void msg2(void){

    GPIO_PORTM_DATA_RM = 0x3E;       //Write U
    GPIO_PORTK_DATA_RK = 0x1; //Write in Digit 1
    delay();

    GPIO_PORTM_DATA_RM = 0x76;       //Write H
    GPIO_PORTK_DATA_RK = 0x2; //Write in Digit 2
    delay();

    GPIO_PORTM_DATA_RM = 0x3E;       //Write U
    GPIO_PORTK_DATA_RK = 0x4; //Write in Digit 3
    delay();

    GPIO_PORTM_DATA_RM = 0x76;       //Write H
    GPIO_PORTK_DATA_RK = 0x8; //Write in Digit 4
    delay();
}

void msg3(void){

    GPIO_PORTM_DATA_RM = 0x79;       //Write E
    GPIO_PORTK_DATA_RK = 0x1; //Write in Digit 1
    delay();

    GPIO_PORTM_DATA_RM = 0x3E;       //Write V
    GPIO_PORTK_DATA_RK = 0x2; //Write in Digit 2
    delay();

    GPIO_PORTM_DATA_RM = 0x06;       //Write I
    GPIO_PORTK_DATA_RK = 0x4; //Write in Digit 3
    delay();

    GPIO_PORTM_DATA_RM = 0x71;       //Write F
    GPIO_PORTK_DATA_RK = 0x8; //Write in Digit 4
    delay();
}

// Delay
void delay(void){
  int t = 0;
  for(t=0;t<20000;t++){} //Aprox. 0.1 ms
}

// Main Program
void main(void){
    init();

    while(1){
        if((UART7_FR_R&UART_FR_RXFE) == 0){
        uart=UART7_DR_R&0xFF;
        }
        else{
        switch(uart){
            case 0x01: //Msg 1
                msg0();
                break;
            case 0x02: //Msg 2
                msg1();
                break;
            case 0x03: //Msg 3
                msg2();
                break;
            case 0x04: //Msg 4
                msg3();
                break;
        }
      }
  }
}
