/**
 * Project: AVR ATtiny USB Tutorial at http://codeandlife.com/
 * Author: Joonas Pihlajamaa, joonas.pihlajamaa@iki.fi
 * Base on V-USB example code by Christian Starkjohann
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v3 (see License.txt)
 */
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "HIDkeys.h"
#include "LightweightRingBuff.h"
#include "usbdrv.h"

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// ************************
// *** USB HID ROUTINES ***
// ************************

// From Frank Zhao's USB Business Card project
// http://www.frank-zhao.com/cache/usbbusinesscard_details.php
PROGMEM const char
    usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
        0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
        0x09, 0x06,  // USAGE (Keyboard)
        0xa1, 0x01,  // COLLECTION (Application)
        0x75, 0x01,  //   REPORT_SIZE (1)
        0x95, 0x08,  //   REPORT_COUNT (8)
        0x05, 0x07,  //   USAGE_PAGE (Keyboard)(Key Codes)
        0x19, 0xe0,  //   USAGE_MINIMUM (Keyboard LeftControl)(224)
        0x29, 0xe7,  //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
        0x15, 0x00,  //   LOGICAL_MINIMUM (0)
        0x25, 0x01,  //   LOGICAL_MAXIMUM (1)
        0x81, 0x02,  //   INPUT (Data,Var,Abs) ; Modifier byte
        0x95, 0x01,  //   REPORT_COUNT (1)
        0x75, 0x08,  //   REPORT_SIZE (8)
        0x81, 0x03,  //   INPUT (Cnst,Var,Abs) ; Reserved byte
        0x95, 0x05,  //   REPORT_COUNT (5)
        0x75, 0x01,  //   REPORT_SIZE (1)
        0x05, 0x08,  //   USAGE_PAGE (LEDs)
        0x19, 0x01,  //   USAGE_MINIMUM (Num Lock)
        0x29, 0x05,  //   USAGE_MAXIMUM (Kana)
        0x91, 0x02,  //   OUTPUT (Data,Var,Abs) ; LED report
        0x95, 0x01,  //   REPORT_COUNT (1)
        0x75, 0x03,  //   REPORT_SIZE (3)
        0x91, 0x03,  //   OUTPUT (Cnst,Var,Abs) ; LED report padding
        0x95, 0x06,  //   REPORT_COUNT (6)
        0x75, 0x08,  //   REPORT_SIZE (8)
        0x15, 0x00,  //   LOGICAL_MINIMUM (0)
        0x25, 0x65,  //   LOGICAL_MAXIMUM (101)
        0x05, 0x07,  //   USAGE_PAGE (Keyboard)(Key Codes)
        0x19, 0x00,  //   USAGE_MINIMUM (Reserved (no event indicated))(0)
        0x29, 0x65,  //   USAGE_MAXIMUM (Keyboard Application)(101)
        0x81, 0x00,  //   INPUT (Data,Ary,Abs)
        0xc0         // END_COLLECTION
};

typedef struct
{
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycode[6];
} keyboard_report_t;

static keyboard_report_t keyboard_report;        // sent to PC
volatile static unsigned char LED_state = 0xff;  // received from PC
static unsigned char idleRate;                   // repeat rate for keyboards

usbMsgLen_t usbFunctionSetup(unsigned char data[8]) {
    usbRequest_t *rq = (void *)data;

    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
        switch (rq->bRequest) {
            case USBRQ_HID_GET_REPORT:  // send "no keys pressed" if asked here
                // wValue: ReportType (highbyte), ReportID (lowbyte)
                usbMsgPtr = (void *)&keyboard_report;  // we only have this one
                keyboard_report.modifier = 0;
                keyboard_report.keycode[0] = 0;
                return sizeof(keyboard_report);
            case USBRQ_HID_SET_REPORT:  // if wLength == 1, should be LED state
                return (rq->wLength.word == 1) ? USB_NO_MSG : 0;
            case USBRQ_HID_GET_IDLE:  // send idle rate to PC as required by spec
                usbMsgPtr = &idleRate;
                return 1;
            case USBRQ_HID_SET_IDLE:  // save idle rate as required by spec
                idleRate = rq->wValue.bytes[1];
                return 0;
        }
    }

    return 0;  // by default don't return any data
}

#define NUM_LOCK 1
#define CAPS_LOCK 2
#define SCROLL_LOCK 4

usbMsgLen_t usbFunctionWrite(uint8_t *data, unsigned char len) {
    if (data[0] == LED_state)
        return 1;
    else
        LED_state = data[0];

    // LED state changed
    if (LED_state & CAPS_LOCK)
        DDRC |= 1 << PC1;  // LED on
    else
        DDRC &= ~(1 << PC1);  // LED off

    return 1;  // Data read, not expecting more
}

// Now only supports letters 'a' to 'z' and 0 (NULL) to clear buttons
void buildReport(unsigned char send_key) {
    keyboard_report.modifier = 0;

    if (send_key >= 'a' && send_key <= 'z')
        keyboard_report.keycode[0] = 4 + (send_key - 'a');
    else
        keyboard_report.keycode[0] = 0;
}

void sendKey(unsigned char _key) {
    keyboard_report.modifier = 0;
    if (_key >= 'a' && _key <= 'z')
        keyboard_report.keycode[0] = 4 + (_key - 'a');
    else if (_key >= 'A' && _key <= 'Z') {
        keyboard_report.modifier = MOD_SHIFT_LEFT;
        keyboard_report.keycode[0] = 4 + (_key - 'A');
    } else if (_key >= '1' && _key <= '9')
        keyboard_report.keycode[0] = 30 + (_key - '1');
    else if (_key == '0')
        keyboard_report.keycode[0] = 39;
    else if (_key == '\n')
        keyboard_report.keycode[0] = KEY_ENTER;
    else if (_key == 0)
        keyboard_report.keycode[0] = 0;
    else if (_key == '-')
        keyboard_report.keycode[0] = KEY_MINUS;
}

void setupHardware(void) {
    UCSRB = (1 << RXEN) |
            (1 << TXEN);  // Turn on the transmission and reception circuitry
    UCSRC =
        (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);  // Use 8-bit character sizes

    UBRRH = (BAUD_PRESCALE >> 8);  // Load upper 8-bits of the baud rate value
                                   // into the high byte of the UBRR register
    UBRRL = BAUD_PRESCALE;         // Load lower 8-bits of the baud rate value into the
                                   // low byte of the UBRR register

    UCSRB |=
        (1 << RXCIE);  // Enable the USART Recieve Complete interrupt (USART_RXC)
}

//volatile unsigned char ready = 0, ReceivedByte;
RingBuff_t USARTtoUSB_Buffer;

int main() {
    unsigned char i;

    DDRC = 1 << PC1;  // PC1 as output
    PORTB = PORTB | 0x38;
    DDRB = DDRB | 0x38;

    setupHardware();
    RingBuffer_InitBuffer(&USARTtoUSB_Buffer);

    for (i = 0; i < sizeof(keyboard_report); i++)  // clear report initially
        ((unsigned char *)&keyboard_report)[i] = 0;

    wdt_enable(WDTO_1S);  // enable 1s watchdog timer

    usbInit();
    DDRC = 0x01;
    PORTC = 0xfc;

    usbDeviceDisconnect();       // enforce re-enumeration
    for (i = 0; i < 250; i++) {  // wait 500 ms
        wdt_reset();             // keep the watchdog happy
        _delay_ms(2);
    }
    usbDeviceConnect();

    TCCR0 |= (1 << CS01);  // timer 0 at clk/8 will generate randomness

    sei();  // Enable interrupts after re-enumeration
    uint8_t dataRaw[10] = {0};
    int ind;

    while (1) {
        wdt_reset();  // keep the watchdog happy
        usbPoll();

        // characters are sent when messageState == STATE_SEND and after receiving
        // the initial LED state from PC (good way to wait until device is
        // recognized)
        if (usbInterruptIsReady()) {
            RingBuff_Count_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
            if (BufferCount >= 10) {
                for (ind = 0; ind < 10; ind++) {
                    //keyboardData[ind] = RingBuffer_Remove(&USARTtoUSB_Buffer);
                    dataRaw[0] = dataRaw[1];
                    dataRaw[1] = dataRaw[2];
                    dataRaw[2] = dataRaw[3];
                    dataRaw[3] = dataRaw[4];
                    dataRaw[4] = dataRaw[5];
                    dataRaw[5] = dataRaw[6];
                    dataRaw[6] = dataRaw[7];
                    dataRaw[7] = dataRaw[8];
                    dataRaw[8] = dataRaw[9];
                    dataRaw[9] = RingBuffer_Remove(&USARTtoUSB_Buffer);
                    if (dataRaw[0] == 0xFF && dataRaw[1] == 0xFE) {
                        keyboard_report.modifier = dataRaw[2];
                        keyboard_report.reserved = dataRaw[3];
                        keyboard_report.keycode[0] = dataRaw[4];
                        keyboard_report.keycode[1] = dataRaw[5];
                        keyboard_report.keycode[2] = dataRaw[6];
                        keyboard_report.keycode[3] = dataRaw[7];
                        keyboard_report.keycode[4] = dataRaw[8];
                        keyboard_report.keycode[5] = dataRaw[9];
                        break;
                    }
                }
            } else {
                sendKey(0);
                DDRC &= ~(1 << PC1);  // LED off
            }

            // start sending
            usbSetInterrupt((void *)&keyboard_report, sizeof(keyboard_report));
        }
    }

    return 0;
}

ISR(USART_RXC_vect) {
    unsigned char ReceivedByte;
    ReceivedByte =
        UDR;  // Fetch the received byte value into the variable "ByteReceived"
    RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}
