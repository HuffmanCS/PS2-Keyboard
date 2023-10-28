//  Created by James Huffman on 1/20/22.
//  Copyright © 2022 James Huffman. All rights reserved.
//  Huffman Computer Science - Hcs
//
//  keyboard.c
//  8051 Keyboard - PS/2 Keyboard From Scratch
//
//  This code was written for an AT89C52/AT89S52 MCU to be programmed as a keyboard adhering to the PS/2 protocol.
//      Another 8051 MCU may be used, so long as it contains the 8052 additions and has enough Ports/Pins.
//
//  I reverse engineered the PS/2 protocol by utilizing limited (and somewhat conflicting) documentation that I could find on the protocol from various sources, as well as by
//      observing an older PS/2 keyboard undergo operation while hooked to an oscilloscope. Eventually I found official documenation from IBM on the Personal System 2, which
//      confirmed I had correctly implemented most of the protocol. In my testing thus far, the "PS/2 Keyboard From Scratch" project I devised from this appears to work and
//      successfully commit a handshake with various operating systems running on varied hardware. Even still, this firmware doesn't strictly follow the protocol to the letter
//      in some aspects and I plan to improve this primarily in a version 2 of this keyboard project with what I have learned.
//
//  It is very important either a 12 or 24 MHz crystal oscilliator is used to drive the MCU! The code will need to be modified to maintain accurate timing for any other clock speed!
//      It is strongly advised a 24 MHz system clock is utilized, as a 12 MHz system clock causes the transmit function to drop its speed below the PS/2 protocol specification.
//
//  The PS/2 protocol specification I read states the clock frequency must be within 10 - 16.7 kHz (testing a real keyboard agrees, but some sources claim different ranges).
//      With a 24 MHz system clock, the clock frequency is programmed at 11.9 kHz (but can be modified faster than spec at a max of 17.2 kHz).
//      With a 12 MHz system clock, the clock frequency will drop as low as 7.5 kHz (but can be modified just below spec at a max of 9.8 kHz).
//
//  Though in my own observations, the keyboard appeared to work just fine with the host even with its clock frequency as low as 7.5 kHz, despite being below the specification.
//      I am not sure if this is just the case with the host hardware I've tested it on, or if hosts are generally receptive and robust with out-of-spec keyboards.
//

// included headers (notably 8052.h allowing SDCC to recognize symbols and compile for the 8051 architecture)
#include <8052.h>
#include <stdint.h>

// definitions
#define CLOCK 24    // the clock speed in MHz driving XTAL1 & XTAL2
#define BREAK 336   // the period between keycode/byte transmissions (in particular for extended/release codes, or multiple argument byte transmissions in a row)
#define EXT 0x02E0  // extension keycode with stop/parity
#define REL 0x03F0  // release keycode with stop/parity
#define ACK 0x03FA  // acknowledge command with stop/parity
#define RE  0x02FE  // resend command with stop/parity
#define NA  0x0300  // NA/error command with stop/parity (unused, experimental)

// version stamp to be included in the binary, only for documentation purposes and fun :)
__code __at (0x1FBF) char VERSION[64] = {"Huffman Computer Science. PS/2 Keyboard From Scratch. v_1.0"};

// 2D array to house key scan codes of key matrix (as well as each code's parity and stop bit in the 2nd-byte)
const uint32_t KEY_SCAN_CODES[14][6] = {
    { 0x0314, 0x0312, 0x0258, 0x020d, 0x020e, 0x0276 },         // { L_CTRL, L_SHFT, CAPS, TAB, B_T, ESC },
    { 0xe0021f, 0x021a, 0x021c, 0x0215, 0x0216, 0x00 },         // { WIN, Z, A, Q, R_1, 0 },
    { 0x0311, 0x0322, 0x031b, 0x031d, 0x031e, 0x0305 },         // { L_ALT, X, S, W, R_2, F_1 },
    { 0x00, 0x0321, 0x0223, 0x0324, 0x0226, 0x0306 },           // { 0, C, D, E, R_3, F_2 },
    { 0x00, 0x022a, 0x032b, 0x032d, 0x0225, 0x0204 },           // { 0, V, F, R, R_4, F_3 },
    { 0x00, 0x0232, 0x0234, 0x022c, 0x032e, 0x030c },           // { 0, BB, G, T, R_5, F_4 },
    { 0x0229, 0x0231, 0x0333, 0x0335, 0x0336, 0x00 },           // { SPACE, N, H, Y, R_6, 0 },
    { 0x00, 0x033a, 0x023b, 0x033c, 0x023d, 0x0303 },           // { 0, M, J, U, R_7, F_5 },
    { 0x00, 0x0341, 0x0342, 0x0243, 0x023e, 0x020b },           // { 0, COMMA, K, I, R_8, F_6 },
    { 0xe00311, 0x0249, 0x034b, 0x0344, 0x0246, 0x0283 },       // { R_ALT, PER, L, O, R_9, F_7 },
    { 0xe00327, 0x024a, 0x024c, 0x034d, 0x0245, 0x030a },       // { WIFN, B_SLA, S_COL, PP, R_0, F_8 },
    { 0xe0022f, 0x00, 0x0252, 0x0254, 0x034e, 0x0201 },         // { MENUS, 0,  F_T, L_BRAK, SUB, F_9 },
    { 0x00, 0x00, 0x0378, 0x025b, 0x0355, 0x0309 },             // { 0, 0, F_11, R_BRAK, EQU, F_10 },
    { 0xe00314, 0x0359, 0x035a, 0x025d, 0x0366, 0x0207 }        // { R_CTRL, R_SHFT, ENTER, F_SLA, BCKSP, F_12 }
};
static unsigned int  LAST_BYTE = 0x00;   // for keeping track of last byte sent to host (for retransmission request)
static unsigned char ENABLE = 1;         // for enabling/disabling keyscanning
static unsigned char REPEAT_RATE = 50;   // for the rate at which a keycode is repeated (1000 / REPEAT_RATE * 10 hertz or cps)
static unsigned char REPEAT_DELAY = 100; // for delay before a pressed key starts repeating (REPEAT_DELAY * 10 milliseconds)
static unsigned char ELAPSED_TIME = 0;   // for counting intervals of 10ms created by Timer 2 to keep track of when to repeat keycodes

// function for handling timer 2 interrupt service routine
void timer2Int(void) __interrupt 5{
    ELAPSED_TIME++; // increment the counter for 10ms intervals for timing keycode repetition
    if( ELAPSED_TIME > 127 ) // prevent ELAPSED_TIME from exceeding 128 to save high bit
        ELAPSED_TIME = 0;
    TF2 = 0;        // clear Timer 2 overflow flag
}//end_timer2Int__interrupt_5

// function that utilizes the 8051's in-circuit Timer 0 to ensure an accurate hardware driven delay (accurate for values greater than 30 microseconds)
// NOTE: a 12 or 24 MHz crystal oscilliator must be used to drive the MCU for this delay function to be accurate
void delay_us(int us){
// check if 24 MHz clock is defined as in use, and double 'us' if so to accomodate for the faster clock speed
#if CLOCK == 24
    us += us;
#endif
    // calculate hex to load into timer high/low bytes
    unsigned int pause = 0xffff - us;
    // set timer mode to 16-bit, load timer high/low bytes with pause
    TMOD = 0x01;
    TL0 = pause & 0xff;
    TH0 = pause >> 8;
    TR0 = 1; // start timer
    while( !TF0 ); // do nothing until timer overflows
    TR0 = 0; // stop timer
    TF0 = 0; // clear flag
}//end_delay_us

// function to transmit keycode over data line to host in unison with clock pulses
// NOTE: if a 12 MHz system clock is used, the device-to-host transmission speed will drop lower than the PS/2 spec unless up/down time is modified to zero
void transmit(unsigned int keycode){
    char bkup = P2; // for maintaining state of P2 prior to transmission (need only if LEDs are connected to bits of Port 2)
    LAST_BYTE = keycode;
    // prepare start bit on keycode being sent (start bit is always zero)
    keycode <<= 1;
    // loop over byte, transmitting it one bit at a time in little endian format over Port 2
    unsigned int index = 0x00;
    while( index < 11 ){
        // set bit for transmission on Port 2
        P2 = keycode | 0x02; // 0000 0011
        // latch clock on falling edge where Port 0.1 is used as clock
        P2_1 ^= 1; // 0000 0010
        keycode >>= 1;
        index++;
        delay_us(16); // downtime
        P2_1 ^= 1; // 0000 0010
        delay_us(14); // uptime
    }
    P2 |= 0x03; // 0000 0011 // data and clock reset high
    P2 |= (0xf8 & bkup); // previous state of other Port 2 bits restored
}//end_transmit

// function to receive commands from host device
int receive(void){
    int buffer = 0;
    // wait for clock to go high and data to go low
    while( !((P2 & 0x02) && !(P2 & 0x01)) );
    // loop to receive data from host (8 data bits and 1 parity bit)
    unsigned int index = 0;
    while( index < 10 ){
        P2_1 ^= 1; // lower clock
        delay_us(16); // downtime
        P2_1 ^= 1; // raise clock
        buffer |= ((P2 & 0x01) << (index++)); // acquire data bit set by host
        delay_us(16); // uptime
    }
    // send ack bit back to host by setting data low and pulsing the clock
    P2_0 = 0;  // 1111 1110
    P2_1 ^= 1; // lower clock
    delay_us(16); // downtime
    P2 |= 0x03; // raise clock and data
    return buffer;
}//end_receive

// function to prepare appropriate keycode to send based on code length and if pressed/released as indicated by keycode parameter (1 is pressed state, 0 is released state)
void sendCode(uint32_t keycode, char keyState){
    EA = 0; // disable interrupts
    // if the keyState is non-zero, this indicates to send the code for pressing the key
    if( keyState ){
        // check if extended code
        if( (keycode & 0xff0000) == 0xe00000 ){
            // transmit extension, then keycode
            transmit(EXT);
            delay_us(BREAK); // BREAK period between code and extension/state transmission
            transmit(keycode);
            delay_us(BREAK);
        }else{
            transmit(keycode);
            delay_us(BREAK);
        }
    }else{ // else the keyState is zero, indicating to send the code for releasing a key
        // check if extended code
        if( (keycode & 0xff0000) == 0xe00000 ){
            // transmit extension, then release code, then finally keycode
            transmit(EXT);
            delay_us(BREAK);
            transmit(REL);
            delay_us(BREAK);
            transmit(keycode);
            delay_us(BREAK);
        }else{
            // transmit release code, then keycode
            transmit(REL);
            delay_us(BREAK);
            transmit(keycode);
            delay_us(BREAK);
        }
    }//end_if_else_keyState
    EA = 1; // enable interrupts
}//end_sendCode

// function to interpret a given command and either send an expected response back to host or only follow command
void followCommand(unsigned int command){
    command &= 0xff; // truncates command for below switch statement
    unsigned int arg; // for receiving bytes back from the host when necessary
    switch( command ){
        case 0xed: // set LEDs
            transmit(ACK);      // acknowledge
            arg = receive();    // read argument byte from host
            transmit(ACK);      // acknowledge
            // check state of CapsLock LED (bit 2), the only "lock-key" present on version 1.0 of this keyboard
            if( (arg & 0x04) ){
                P2_3 = 1; // set CapsLock LED
            }else{
                P2_3 = 0; // clear CapsLock LED // 1111 0111
            }// other LEDs would be: bit 0 == ScrollLock, bit 1 == NumberLock, bit 3 == International purposes (unused in US KBs)
            break;
        case 0xee: // echo
            transmit(0x03ee);   // respond to host with an echo back
            break;
        case 0xf0: // scan code set
            transmit(ACK);      // acknowledge
            arg = receive();    // read argument byte from host
            transmit(ACK);      // acknowledge
            // if the argument received is 0, respond with current scan code set (which is 0x02)
            if( !(arg & 0xff) )
                transmit(0x0341); // respond with scan code set (firmware is hardcoded as always Set 2 or code 0x41)
            break;
        case 0xf2: // read ID
            // respond with device id of 0xab83 (host appears to sometimes disregard the second/low byte, although this may only be due to having seperate ports for KB and mouse)
            transmit(ACK);      // acknowledge
            transmit(0x02ab);
            delay_us(BREAK);    // brief delay to ensure reception
            transmit(0x0283);
            break;
        case 0xf3: // set typematic delay / auto-repeat rate of keycodes
            transmit(ACK);      // acknowledge
            arg = receive();    // read the argument from host
            transmit(ACK);      // acknowledge
            // set requested repeat rate (bits 0-4 of arg, 000X-XXXX) // (1000 / REPEAT_RATE * 10) cps
            if( (arg & 0x1f) >= 0x18 && (arg & 0x1f) <= 0x1f )
                REPEAT_RATE = 50; // 2 cps
            else if( (arg & 0x1f) >= 0x10 && (arg & 0x1f) <= 0x17 )
                REPEAT_RATE = 25; // 4 cps
            else if( (arg & 0x1f) >= 0x08 && (arg & 0x1f) <= 0x0f )
                REPEAT_RATE = 12; // 8.3 cps
            else if( (arg & 0x1f) >= 0x04 && (arg & 0x1f) <= 0x07 )
                REPEAT_RATE = 6; // 16.6 cps
            else
                REPEAT_RATE = 3; // 33.3 cps
            // set the requested delay before keys repeat (bits 5-6 of arg, 0XX0-0000) // (REPEAT_DELAY * 10) milliseconds
            if( (arg & 0x60) == 0x20 )      // 0x01
                REPEAT_DELAY = 50; // 500ms
            else if( (arg & 0x60) == 0x40 ) // 0x10
                REPEAT_DELAY = 75; // 750ms
            else if( (arg & 0x60) == 0x60 ) // 0x11
                REPEAT_DELAY = 100; // 1000ms
            else                            // 0x00
                REPEAT_DELAY = 25; // 250ms
            break;
        case 0xf4: // enable (enables key-matrix scanning)
            transmit(ACK);      // acknowledge
            ENABLE = 1;
            break;
        case 0xf5: // disable (disables key-matrix scanning)
            transmit(ACK);      // acknowledge
            ENABLE = 0;
            break;
        case 0xfe: // resend last byte
            transmit(ACK);      // acknowledge
            transmit(LAST_BYTE);// last byte sent (seems seldom to be called)
            break;
        case 0xff: // reset
            transmit(ACK);      // acknowledge
            delay_us(BREAK);    // brief delay for "simulating" a reset, but also to ensure reception
            transmit(0x03aa);   // report BAT successful
            break;
        // NOTE: commands F6-FD apply to scancode set 3 only. This keyboard firmware currently is hardcoded to scancode set 2, thus these commands can be ignored.
        case 0xf6: // set default
        case 0xf7: // set all keys to typematic/autorepeat
        case 0xf8: // set all keys to make/release
        case 0xf9: // set all keys to make only
        case 0xfa: // set all keys to typematic/autorepeat/make/release
            transmit(ACK);      // acknowledge
            break;
        case 0xfb: // set specific key to typematic/autorepeat only
        case 0xfc: // set specific key to make/release
        case 0xfd: // set specific key to make only
            transmit(ACK);      // acknowledge
            arg = receive();    // read argument byte from host
            transmit(ACK);      // acknowledge
            break;
        default: // command unknown or reception error
            transmit(RE);   // resend
            //P2 |= 0x20;       // DEBUGGING LED
            break;
    }//end_switch
}//end_followCommand

// main routine
void main(void){
    // setup timer 2 in 16-bit auto-reload mode, and load timer registers w/ 65535 - 10000 = 55535 ===> 0xD8EF
    T2CON = 0x00;
    TL2 = 0xef;
    TH2 = 0xd8;
    // the following registers will hold the value to reload Timer 2 with upon overflow
    RCAP2L = 0xef;
    RCAP2H = 0xd8;
// check if 24 MHz clock is defined as in use, and double Timer 2 values if so to accomodate for the faster clock speed
#if CLOCK == 24
    TL2 += TL2;
    TH2 += TH2;
    RCAP2L += RCAP2L;
    RCAP2H += RCAP2H;
#endif
    // enable interrupts to occur, specifically for Timer 2 overflow to signal an interrupt
    EA = 1;
    ET2 = 1;
    TR2 = 1; // start Timer 2
    // prepare ports for I/O operations (P0, P1, and P3 for key-matrix... P2 for data/clock and LEDs)
    P1 = 0xff; // enable input on Port 1 from 1.0 to 1.7 (to collect columns, low-byte)
    P3 = 0xff; // enable input on Port 3 from 3.0 to 3.5 (to collect columns, high-byte)
    P0 = 0x3f; // enable input on Port 0 from 0.0 to 0.5 (to collect rows)
    P2 = 0x0f; // enable output on Port 2, 2.0 as data line and 2.1 as clock. (2.2 as data monitor and 2.3 as clock monitor in external TTL design)
    // declare array to keep track of key-presses and their timestamps, as well as counter variables and a buffer for receiving commands from host
    unsigned char keyStamps[14][6];
    int i = 0, j = 0, buffer = 0;
    // main loop
    while(1){
    start:
        // check if host is attempting to communicate or inhibit communications
        if( !(P2 & 0x02) ){
            //P2 |= 0x20; // DEBUGGING LED (one method I sometimes employ in debugging is to have certain LEDs light under certain conditions)
            delay_us(50);
        // check if host is ready to transmit
        }else if( (P2 & 0x02) && !(P2 & 0x01) ){
            EA = 0; // disable interrupts
            buffer = receive();
            followCommand(buffer);
            EA = 1; // enable interrupts
        // otherwise, if key-matrix scanning is enabled, proceed with scanning for keypresses
        }else if( ENABLE ){
            //P2 |= 0x10; // DEBUGGING LED
            // loops for checking key matrix for pressed keys, first checking Port 1 (bits 1 to 8) columns then Port 3 (bits 1 to 6) columns
            P3 = 0x00, P1 = 0x01;
            for(i = 0; i < 14; i++){
                // check 0.0 to 0.5 for active/past  input
                for(j = 0; j < 6; j++){
                    // check if clock is being pulled low before each keyscan, as device is expected to abort scanning if host requests transmission
                    if( !(P2 & 0x02) )
                        goto start;
                    // if j-th Port bit is active high, determine delay then transmit code
                    if( P0 & (0x01 << j) ){
                        // if the key was not priorly active, immediately transmit code
                        if( !keyStamps[i][j] ){
                            sendCode( KEY_SCAN_CODES[i][j], 1 );
                            keyStamps[i][j] = ELAPSED_TIME; // update key time-stamp
                        }
                        // if key was active, determine if the REPEAT_DELAY has been met or already was met
                        else if( keyStamps[i][j] >= 128 || // high bit indicates REPEAT_DELAY already was met and key is in repeating mode
                                (ELAPSED_TIME > keyStamps[i][j] && (ELAPSED_TIME - keyStamps[i][j] >= REPEAT_DELAY)) ||
                                (ELAPSED_TIME < keyStamps[i][j] && ((127 - keyStamps[i][j]) + ELAPSED_TIME >= REPEAT_DELAY))
                        ){
                            keyStamps[i][j] |= 0x80; // set the high-bit of the byte to indicate key is in repeating mode
                            // proceed to repeat the keycode at the specified REPEAT_RATE interval
                            if( ELAPSED_TIME % REPEAT_RATE == 0 && (keyStamps[i][j] & 0x7f) != ELAPSED_TIME ){
                                keyStamps[i][j] = 0x80 | ELAPSED_TIME; // update key time-stamp in repeating mode
                                sendCode( KEY_SCAN_CODES[i][j], 1 );
                            }
                        }
                    // else if it was active, transmit released state
                    }else if( keyStamps[i][j] ){
                        sendCode( KEY_SCAN_CODES[i][j], 0 );
                        keyStamps[i][j] = 0; // clear key time-stamp
                    }
                }//end_for_rows
                // check if columns are in range of Port 1 or Port 3, shift appropriate Port accordingly
                if(i < 7){
                    P1 <<= 1; // shift Port 1
                }else if(i > 7){
                    P3 <<= 1; // shift Port 3
                }else{
                    // if i == 7, disable Port 1 and check Port 3 (bits 8 to 13) columns
                    P1 = 0x00, P3 = 0x01;
                }
                // NOTE: SFR requires a max of 700 nano-seconds to set the Port data for valid output, which without parasitic capacitance is negligable
                delay_us(100); // fixes potential ghost bug! (ie, parasitic capacitance in circuit causing ghost key-presses in bottom row)
            }//end_for_columns
        }//end_if_else
        delay_us(50);
    }//end_while
}//end_main
