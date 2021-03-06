/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/



#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{

  // 0 - Analog
  { PORTA, 11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel19, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // ANALOG IN
  // 1..3 - LIN Controller
  { PORTA,  8, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // LIN TX
  { PORTA, 10, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // LIN RX
  { PORTA,  9, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },  // LIN_SLP  


  // 4..7 - SIM IO pins
  // --------------------

  { PORTB,  8, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SIM RESET
  { PORTB,  2, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // SIM TX
  { PORTB,  3, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // SIM RX
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE  }, // SIM PWR KEY

  // 8..9 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
  // ----------------------
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]

  // 10..14 - SD SPI pins (ICSP:MISO,SCK,MOSI), IO
  // ----------------------
  { PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, // SD MOSI: SERCOM4/PAD[0]
  { PORTB, 10, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // SD MISO: SERCOM4/PAD[2]
  { PORTB, 11, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // SD SCK: SERCOM4/PAD[3]
  { PORTA, 13, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },    // SD CS: SERCOM4/PAD[1]
  { PORTA, 14, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  // SD DT
  

  // 15..16 - USB
  // --------------------
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP


  // 17/18 I2C (SDA/SCL and also EDBG:SDA/SCL)
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCL: SERCOM3/PAD[1]

  // 19..24 - CAN Controller
  { PORTA,  2, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // CAN_RST
  { PORTB,  9, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // CAN_INT
  { PORTA, 5, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // CAN MISO: SERCOM1/PAD[3]
  { PORTA, 6, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // CAN MOSI: SERCOM1/PAD[0]
  { PORTA, 7, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // CAN SCK: SERCOM1/PAD[1]
  { PORTA, 4, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // CAN SS: SERCOM1/PAD[2]

  // 25..26 Reset/Set Relay
  { PORTB,  22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // R_RELAY
  { PORTB,  23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // S_RELAY

  // 27 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // 28..35 Unused pins
  
  { PORTA, 15, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_15 }, // NA
  { PORTA, 16, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0 }, // TCC2/WO[0]
  { PORTA, 17, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH1, TCC2_CH1, EXTERNAL_INT_1 }, // TCC2/WO[1]
  { PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_2 }, // TC3/WO[0]
  { PORTA, 19, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, // TCC0/WO[3]
  { PORTA, 20, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_4 }, // NA
  { PORTA, 21, PIO_PWM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH7, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 7
  { PORTA, 27, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only

  // ----------------------
  // 34..35 - Alternate use of A0 (DAC output), 44 SWCLK, 45, SWDIO

  { PORTA, 30, PIO_PWM, PIN_ATTR_DIGITAL|PIO_SERCOM, No_ADC_Channel, NOT_ON_PWM, TCC1_CH0, EXTERNAL_INT_10 }, // SWCLK
  { PORTA, 31, PIO_PWM, PIN_ATTR_DIGITAL|PIO_SERCOM, No_ADC_Channel, NOT_ON_PWM, TCC1_CH1, EXTERNAL_INT_11 }, // SWDIO
} ;

extern "C" {
    unsigned int PINCOUNT_fn() {
        return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
    }
}

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;  // SIM UART port
Uart Serial2( &sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ; // LIN UART port

void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}

void SERCOM5_Handler()
{
  Serial1.IrqHandler();
}

