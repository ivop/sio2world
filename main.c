/*
 * SIO2WORLD    - Connect the Atari 8-bit SIO-Port to the World! :)
 *
 * Copyright (C) 2014, 2015 by Ivo van Poorten <ivopvp@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/* Hardware:
 *      Arduino Nano V3 (ATmega328 16MHz, 32kB Flash, 1kB ROM, 2kB RAM)
 *      SD-Card Breakout Board
 *      DS18B20 1-Wire Temperature
 *      DS1307 I2C RTC (+ AT24C32 EEPROM) Breakout Board
 *
 * Perhaps:
 *      WizNet 5100 SPI Breakout Board
 *      SPI or UART WiFi
 *      PS2 analog stick
 *      Some kind of VGA display for 80-columns terminal mode
 */

/* Notes:
 * PAL  : 1773447 Hz
 * NTSC : 1789790 Hz
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "ff/ff.h"

#define RETRIES         10

#define SW_USART_BAUD   9600
#define SW_USART_DELAY  (1000000/SW_USART_BAUD)
#define SW_USART_DDR    DDRD
#define SW_USART_PORT   PORTD
#define SW_USART_BIT    DDD3

#define ONEWIRE_DDR     DDRD
#define ONEWIRE_PORT    PORTD
#define ONEWIRE_PIN     PIND
#define ONEWIRE_BIT     DDD2

#define DDR_SPI         DDRB
#define DD_SDSS         DDB2
#define DD_MOSI         DDB3
#define DD_SCK          DDB5

#define LED_DDR         DDRC
#define LED_PORT        PORTC
#define LED_GREEN       DDC0
#define LED_RED         DDC1

#define BUTTONS_DDR     DDRD
#define BUTTONS_PORT    PORTD
#define BUTTONS_PIN     PIND
#define BUTTON_ONE      DDD5

#define SIO_CMD_DDR     DDRB
#define SIO_CMD_PORT    PORTB
#define SIO_CMD_PIN     PINB
#define SIO_CMD_BIT     PB0

#define SIO_ACK     'A'
#define SIO_NACK    'N'
#define SIO_COMPL   'C'
#define SIO_ERROR   'E'

#define NDRIVES     8

#define SIO2WORLD_CONFIG_FILE   "SIO2WRLD.CFG"
#define CONFIG_FILE_SIZE        ((NDRIVES * sizeof(struct drive)) + 2)

#define STANDARD_POKEY_DIV  40
#define FAST_POKEY_DIV      6

#define MSB(x)      (((x)>>8)&0xff)
#define LSB(x)      ((x)&0xff)

/* ------------------------------------------------------------------------- */

struct __attribute__((packed)) atr_header {
    uint16_t    wMagic;
    uint16_t    wPars;
    uint16_t    wSecSize;
    uint8_t     btParsHigh;
    uint32_t    dwCRC;
    uint32_t    dwUnused;
    uint8_t     btFlags;
};

struct percom {
    uint8_t     ntracks;
    uint8_t     steprate;               // 0-3: 30/20/12/6ms
    uint8_t     sectors_per_track_msb;
    uint8_t     sectors_per_track_lsb;
    uint8_t     nsides_min_one;
    uint8_t     modulation;             // 0 - FM, 4 - MFM
    uint8_t     bytes_per_sector_msb;
    uint8_t     bytes_per_sector_lsb;
    uint8_t     drive_online;           // $ff, XF551 returns $40
    uint8_t     res0, res1, res2;       // reserved
};

struct status {
    uint8_t     drive_status;
    uint8_t     inverted_controller_status;
    uint8_t     timeout;
    uint8_t     unused;
};

struct cmd_frame {
    uint8_t device;
    uint8_t command;
    uint8_t aux1;
    uint8_t aux2;
    uint8_t checksum;
};

#define STATUS_COMMAND_FRAME_ERROR      0x01
#define STATUS_CHECKSUM_ERROR           0x02
#define STATUS_WRITE_ERROR              0x04
#define STATUS_WRITE_PROTECTED          0x08
#define STATUS_MOTOR_IS_ON              0x10
#define STATUS_SECTOR_IS_DD             0x20
#define STATUS_MEDIUM_DENSITY           0x80

#define FDC_CONTROLLER_BUSY             0x01
#define FDC_DATA_REQUEST                0x02
#define FDC_DATA_LOST                   0x04
#define FDC_TRACK0                      0x04
#define FDC_CRC_ERROR                   0x08
#define FDC_RECORD_NOT_FOUND            0x10
#define FDC_RECORD_TYPE                 0x20
#define FDC_HEAD_LOADED                 0x20
#define FDC_WRITE_PROTECTED             0x40
#define FDC_NOT_READY                   0x80

/* ------------------------------------------------------------------------- */

#define TYPE_UNKNOWN    0
#define TYPE_XFD        1
#define TYPE_ATR        2

static struct drive {
    uint8_t         enabled:1;
    uint8_t         read_write:1;
    uint8_t         type:3;
    uint8_t         controller_status;
    uint8_t         bytes_per_sector;   // (128<<bytes_per_sector)
    uint8_t         sides;
    uint8_t         tracks;
    uint16_t        sectors_per_track;
    uint16_t        nsectors;
    uint8_t         first_three_dd;
    char            filename[64];
    FIL             file;
} drive;

static uint8_t current_drive_num;

static FATFS fatfs;
static FIL config_fp;

static uint8_t cmd_frame[5];
static int i, checksum;

static uint8_t timekeeper[8];

static uint8_t pal = 1;

static const uint8_t slow_pokey_div = STANDARD_POKEY_DIV;
static uint8_t fast_pokey_div = FAST_POKEY_DIV;
static uint8_t slow_ubrr, fast_ubrr, current_ubrr;

/* If memory gets tight, these tables can be moved to EEPROM */
/* note: eeprom_read_byte seems to use much more code and the resulting
 * binary is actually 91 bytes bigger after moving these to eeprom */

static const struct pokey_div_to_ubrr_tab {
    uint8_t div, pal_ubrr, ntsc_ubrr;
} PROGMEM pokey_div_to_ubrr_tab[] = {
    {  0,  15,  15},
    {  1,  17,  17},
    {  2,  19,  19},
    {  3,  22,  21},
    {  4,  24,  24},
    {  5,  26,  26},
    {  6,  28,  28},
    {  7,  31,  30},
    {  8,  33,  33},
    {  9,  35,  35},
    { 10,  37,  37},
    { 16,  51,  50},
    { 40, 105, 104}
};

static const struct geometry_tab {
    uint16_t nsectors;
    uint8_t sides, tracks, sectors_per_track;
} PROGMEM geometry_tab[] = {
    {  720, 1, 40, 18 },
    { 1040, 1, 40, 26 },
    { 1440, 2, 40, 18 },
    { 2880, 2, 80, 18 },
    { 5760, 2, 80, 36 },
    { 0,0,0,0 }
};

static uint8_t *playbuf;
static volatile uint8_t toggle, pos;

/* ------------------------------------------------------------------------- */

static uint8_t pokey_div_to_ubrr(uint8_t div) {
    const struct pokey_div_to_ubrr_tab *tab = pokey_div_to_ubrr_tab;
    while (pgm_read_byte_near(&(tab->div)) != 40) {
        if (pgm_read_byte_near(&(tab->div)) == div) break;
        tab++;
    }
    return pgm_read_byte_near(pal ? &(tab->pal_ubrr) : &(tab-> ntsc_ubrr));
}

/* ------------------------------------------------------------------------- */

/* Hardware USART */

static void hw_usart_init(unsigned int ubrr) {
    UBRR0H = ubrr>>8;
    UBRR0L = ubrr;
    UCSR0B = 0;                             // disable all
    UCSR0B = _BV(RXEN0)  | _BV(TXEN0);      // enable RX and TX
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);     // 8N1
    UCSR0A = _BV(U2X0);                     // Double Speed
    if (ubrr < 24)
        UCSR0C |= _BV(USBS0);               // 8N2(!) for pokey div <= 3
}

/* Clarification: 8N2 is for an unmodified Atari with the SIO caps in place.
 * I managed to reach Pokey divisor 0 with this, which wouldn't work with 8N1.
 * Somehow, the longer stop bit helps Pokey keeping up.
 * On a modified Atari, i.e. with its caps clipped, this is not needed.
 * Plan was to make this configurable
 */

static void hw_usart_tx_byte(uint8_t byte) {
    while(!(UCSR0A & _BV(UDRE0))) ;         // wait for port ready to write
    UDR0 = byte;
}

static uint8_t hw_usart_rx_byte(void) {
    while(!(UCSR0A & _BV(RXC0))) ;          // wait for byte received
    return UDR0;
}

static void hw_usart_tx_block(const void *const buf, uint16_t len) {
    const uint8_t *b = buf;
    while (len--)
        hw_usart_tx_byte(*b++);
}

static void hw_usart_rx_block(void *const buf, uint16_t len) {
    uint8_t *b = buf;
    while (len--)
        *b++ = hw_usart_rx_byte();
}

/* ------------------------------------------------------------------------- */

/* Debug Software USART (TX ONLY!) */

static void sw_usart_init(void) {
    SW_USART_PORT |= _BV(SW_USART_BIT);     // pull high
    SW_USART_DDR  |= _BV(SW_USART_BIT);     // output
}

static void sw_usart_tx_byte(uint8_t byte) {
    uint8_t mask;
    SW_USART_PORT &= ~_BV(SW_USART_BIT);
    _delay_us(SW_USART_DELAY);
    for (mask = 1; mask; mask<<=1) {
        if (byte & mask)    SW_USART_PORT |= _BV(SW_USART_BIT);
        else                SW_USART_PORT &= ~_BV(SW_USART_BIT);
        _delay_us(SW_USART_DELAY);
    }
    SW_USART_PORT |= _BV(SW_USART_BIT);     // stop high
    _delay_us(SW_USART_DELAY);
}

static void sw_usart_tx_string(char *s) {
    while(*s) sw_usart_tx_byte(*s++);
}

/* ------------------------------------------------------------------------- */

/* Bitbang 1-Wire */

#define WIRELOW     ONEWIRE_DDR  |=  _BV(ONEWIRE_BIT); \
                    ONEWIRE_PORT &= ~_BV(ONEWIRE_BIT)
#define WIREHIGH    ONEWIRE_DDR  &= ~_BV(ONEWIRE_BIT); \
                    ONEWIRE_PORT |=  _BV(ONEWIRE_BIT)
#define WIREVAL     (!!(ONEWIRE_PIN & _BV(ONEWIRE_BIT)))

static uint8_t onewire_reset(void) {
    uint8_t v;

    WIRELOW;        _delay_us(480);
    WIREHIGH;       _delay_us(66);
    v = WIREVAL;    _delay_us(240);
    if (!WIREVAL)   v=1;

    return v;
}

static void onewire_write_bit(uint8_t bit) {
    if (!bit) {
        WIRELOW;    _delay_us(60);
        WIREHIGH;   _delay_us(4);
    } else {
        WIRELOW;    _delay_us(10);
        WIREHIGH;   _delay_us(54);
    }
}

static uint8_t onewire_read_bit(void) {
    uint8_t bit;
    WIRELOW;        _delay_us(1);
    WIREHIGH;       _delay_us(10);
    bit = WIREVAL;  _delay_us(50);
    return bit;
}

static void onewire_write_byte(uint8_t byte) {
    uint8_t i;
    for (i=0; i<8; i++)
        onewire_write_bit(byte & (1<<i));
    _delay_us(5);
}

static uint8_t onewire_read_byte(void) {
    uint8_t i, byte=0;
    for (i=0; i<8; i++)
        byte |= onewire_read_bit() << i;
    return byte;
}

static uint16_t onewire_ds18b20_read_temperature(void) {
    uint8_t scratchpad[9];

    onewire_reset();
    onewire_write_byte(0xcc);
    onewire_write_byte(0x44);
    for (i=800; !onewire_read_byte() && i; i--) _delay_ms(1);

    if (!i) return 0x7ff;       // timeout, return VERY HOT :)

    onewire_reset();
    onewire_write_byte(0xcc);
    onewire_write_byte(0xbe);
    for (i=0; i<9; i++)
        scratchpad[i] = onewire_read_byte();

    onewire_reset();

    return scratchpad[0] | (scratchpad[1]<<8);
}

/* ------------------------------------------------------------------------- */

/* Hardware TWI (Two Wire Interface, I2C, IIC, ...) */

static void twi_init(void) {
    TWSR = 0x00;    // no prescaler     (bits 0 and 1 [1,4,16,64])
    TWBR = 0x0c;    // F_CPU/(16+(2*1)*0xc) == 400kHz
    TWCR = _BV(TWEN);
}

static void twi_start(void) {
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    while (!(TWCR & _BV(TWINT))) ;
}

static void twi_stop(void) {
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

static void twi_write_byte(uint8_t byte) {
    TWDR = byte;
    TWCR = _BV(TWINT) | _BV(TWEN);
    while (!(TWCR & _BV(TWINT))) ;
}

static uint8_t twi_read_byte_with_ack(void) {
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    while (!(TWCR & _BV(TWINT))) ;
    return TWDR;
}

static uint8_t twi_read_byte_without_ack(void) {
    TWCR = _BV(TWINT) | _BV(TWEN);
    while (!(TWCR & _BV(TWINT))) ;
    return TWDR;
}

static uint8_t twi_get_status(void) {
    return TWSR & 0xf8;
}

static void twi_ds1307_read_timekeeper(void) {
    twi_start();                // status TW_START
    twi_write_byte(0xd0+0);     // status TW_MT_SLA_ACK
    twi_write_byte(0x00);       // status TW_MT_DATA_ACK

    twi_start();                // status TW_REP_START
    twi_write_byte(0xd0+1);     // status TW_MR_SLA_ACK

    for (i=0; i<7; i++) {
        timekeeper[i] = twi_read_byte_with_ack(); // status TW_MR_DATA_ACK
    }
    timekeeper[7] = twi_read_byte_without_ack(); // status TW_MR_DATA_NACK

    twi_stop();
}

static void twi_ds1307_write_timekeeper(void) {
    twi_start();
    twi_write_byte(0xd0+0);
    twi_write_byte(0x00);
    for (i=0; i<8; i++)
        twi_write_byte(timekeeper[i]);
    twi_stop();
}

/* ------------------------------------------------------------------------- */

/* LED Routines */

static inline void led_on(uint8_t which) {
    LED_PORT |= _BV(which);
}

static inline void led_off(uint8_t which) {
    LED_PORT &= ~_BV(which);
}

static void led_init(void) {
    LED_DDR |= _BV(LED_GREEN) | _BV(LED_RED);
    led_off(LED_GREEN);
    led_off(LED_RED);
}

/* ------------------------------------------------------------------------- */

/* Button(s) */

static void buttons_init(void) {
    BUTTONS_PORT |= _BV(BUTTON_ONE);
    BUTTONS_DDR  |= _BV(BUTTON_ONE);
}

static uint8_t button_pressed(uint8_t which) {
    return !(BUTTONS_PIN & _BV(which));
}

/* ------------------------------------------------------------------------- */

static void pwm_init(void) {
    cli();                      // ensure interrupts are off
    TCCR1A |= _BV(COM1A1);      // clear on compare match
    TCCR1A |= _BV(WGM10);       // PWM, Phase Correct, 8-bit
    TCCR1B |= _BV(CS10);        // clk/1
    TIMSK1 = _BV(TOIE1);        // interrupt on overflow
    ICR1L = 0xff;               // 31250 Hz
    DDRB |= _BV(PB1);
    OCR1AH=0x00;
    OCR1AL=0x00;
}

/* ------------------------------------------------------------------------- */

#if DEBUG
static void debug(const char *f, ...) {
    va_list ap;
    char buf[80];
    va_start(ap,f);
    vsnprintf(buf, 79, f, ap);
    va_end(ap);
    buf[79]=0;
    sw_usart_tx_string(buf);
    sw_usart_tx_string("\r\n");
}
#else
#define debug(...)
#endif

static void fatal(const char *const s) {
    debug(s);
    while(1) {
        led_on(LED_RED);
        led_on(LED_GREEN);
        _delay_ms(200);
        led_off(LED_RED);
        led_off(LED_GREEN);
        _delay_ms(200);
    }
}

/* ------------------------------------------------------------------------- */

/* Config file Routines */

static void config_save_current_drive(void) {
    UINT bw;
    f_lseek(&config_fp, current_drive_num * sizeof(struct drive));
    f_write(&config_fp, &drive, sizeof(struct drive), &bw);
    f_sync(&config_fp);
}

static void config_load_current_drive(uint8_t num) {
    UINT br;
    f_lseek(&config_fp, num * sizeof(struct drive));
    f_read(&config_fp, &drive, sizeof(struct drive), &br);
    current_drive_num = num;
}

static void config_create(void) {
    UINT bw;

    for (i=0; i<RETRIES; i++) {
        if (f_open(&config_fp, SIO2WORLD_CONFIG_FILE, 
                            FA_READ|FA_WRITE|FA_CREATE_ALWAYS) == FR_OK) break;
        _delay_ms(50);
    }
    if (i==RETRIES) goto save_error;

    for (i=0; i<RETRIES; i++) {
        if (f_truncate(&config_fp) == FR_OK) break;
        _delay_ms(50);
    }
    if (i==RETRIES) goto save_error;

    memset(&drive, 0, sizeof(struct drive));

    for (i=0; i<NDRIVES; i++) {
        if (f_write(&config_fp, &drive, sizeof(drive), &bw) != FR_OK)
            goto save_error;
    }

    f_write(&config_fp, &pal, sizeof(pal), &bw);
    if (f_write(&config_fp, &fast_pokey_div, sizeof(fast_pokey_div), &bw)
                                                != FR_OK) goto save_error;

    f_sync(&config_fp);

    return;

save_error:
    fatal("cannot create config");
}

static uint8_t config_open(void) {
    UINT br;

    for (i=0; i<RETRIES; i++) {
        if (f_open(&config_fp, SIO2WORLD_CONFIG_FILE,
                            FA_READ|FA_WRITE|FA_OPEN_EXISTING) == FR_OK) break;
        _delay_ms(50);
    }
    if (i==RETRIES) return 0;

    if (config_fp.fsize != CONFIG_FILE_SIZE) goto load_error;

    for (i=0; i<RETRIES; i++) {
        f_lseek(&config_fp, NDRIVES * sizeof(struct drive));
        f_read(&config_fp, &pal, sizeof(pal), &br);
        if (f_read(&config_fp, &fast_pokey_div, sizeof(fast_pokey_div), &br)
                                                               == FR_OK) break;
        _delay_ms(50);
    }
    if (i==RETRIES) goto load_error;

    return 1;

load_error:
    return 0;
}

/* ------------------------------------------------------------------------- */

/* SIO Routines */

static void sio_init(void) {
    SIO_CMD_DDR &= ~_BV(SIO_CMD_BIT);      // input, command line
    SIO_CMD_PORT |= _BV(SIO_CMD_BIT);
    hw_usart_init(current_ubrr);
}

static uint8_t sio_calc_checksum(const void *const buf, uint16_t len) {
    const uint8_t *const b = buf;
    int i;
    uint16_t c;
    for (i=0, c=0; i<len; i++)
        c += b[i];
    c = (c & 0xff) + (c>>8);
    return (c & 0xff) + (c>>8);
}

static void sio_drive_get_status(void) {
    uint8_t buf[5];         // XXX use struct status

    led_on(LED_RED);
    hw_usart_tx_byte(SIO_ACK);
    _delay_us(700);
    hw_usart_tx_byte(SIO_COMPL);
    _delay_us(100);

    buf[0] = STATUS_MOTOR_IS_ON;
    buf[0] |= drive.bytes_per_sector        ? STATUS_SECTOR_IS_DD : 0;
    buf[0] |= drive.sectors_per_track == 26 ? STATUS_MEDIUM_DENSITY : 0;
    buf[0] |= drive.read_write              ? 0 : STATUS_WRITE_PROTECTED;

    buf[1] = ~drive.controller_status;
    buf[2] = 0xe0;
    buf[3] = 0;

    buf[4] = sio_calc_checksum(buf, 4);
    hw_usart_tx_block(buf, 5);

    led_off(LED_RED);
}

/* when memory gets tight, read and write could be combined or
 * ofs calculation can be factored out */

static void sio_drive_write_sector(void) {
    DWORD ofs = 0;
    UINT bw;
    uint16_t bps, sec = cmd_frame[2] | (cmd_frame[3] << 8);
    uint8_t buf[257];

    led_on(LED_RED);
    hw_usart_tx_byte(SIO_ACK);

    // out of bounds or write to read-only disk
    if (sec < 1 || sec > drive.nsectors || !drive.read_write) {
        hw_usart_tx_byte(SIO_ERROR);
        drive.controller_status |= STATUS_WRITE_PROTECTED;
        led_off(LED_RED);
        return;
    }

    bps = 128 << drive.bytes_per_sector;

    sec -= 1;
    if (drive.type == TYPE_ATR) ofs += 0x10;
    if (sec < 3 && !drive.first_three_dd) {
        ofs += sec * 128;
        bps = 128;
        sec = 0;
    } else if (sec > 2 && !drive.first_three_dd) {
        ofs += 384;
        sec -= 3;
    }
    ofs += (DWORD)sec * bps;

    hw_usart_rx_block(buf, bps+1);
    _delay_us(750);
    if (buf[bps] != sio_calc_checksum(buf, bps)) {
        hw_usart_tx_byte(SIO_NACK);
        return;
    }
    hw_usart_tx_byte(SIO_ACK);

    f_lseek(&drive.file, ofs);

    for (i=0; i<RETRIES; i++) {
        if (f_write(&drive.file, buf, bps, &bw) == FR_OK) break;
        _delay_us(5);
    }
    f_sync(&drive.file);

    _delay_us(800);

    if (i == RETRIES) {
        drive.controller_status |= STATUS_WRITE_ERROR;
        hw_usart_tx_byte(SIO_ERROR);
        return;
    }
    hw_usart_tx_byte(SIO_COMPL);
    _delay_us(100);

    drive.controller_status = 0;

    led_off(LED_RED);
}

static void sio_drive_read_sector(void) {
    DWORD ofs = 0;
    UINT br;
    uint16_t bps, sec = cmd_frame[2] | (cmd_frame[3] << 8);
    uint8_t buf[257];

    led_on(LED_RED);
    hw_usart_tx_byte(SIO_ACK);
    _delay_us(700);

    if (sec < 1 || sec > drive.nsectors) {
        hw_usart_tx_byte(SIO_ERROR);
        drive.controller_status |= STATUS_COMMAND_FRAME_ERROR;
        led_off(LED_RED);
        return;
    }

    bps = 128 << drive.bytes_per_sector;

    sec -= 1;
    if (drive.type == TYPE_ATR) ofs += 0x10;
    if (sec < 3 && !drive.first_three_dd) {
        ofs += sec * 128;
        bps = 128;
        sec = 0;
    } else if (sec > 2 && !drive.first_three_dd) {
        ofs += 384;
        sec -= 3;
    }
    ofs += (DWORD)sec * bps;

    f_lseek(&drive.file, ofs);
    f_read(&drive.file, buf, bps, &br);

    hw_usart_tx_byte(SIO_COMPL);
    _delay_us(100);

    buf[bps] = sio_calc_checksum(buf, bps);
    hw_usart_tx_block(buf, bps+1);

    drive.controller_status = 0;

    led_off(LED_RED);
}

static void sio_drive_read_high_speed_index(void) {
    led_on(LED_RED);
    hw_usart_tx_byte(SIO_ACK);
    _delay_us(700);
    hw_usart_tx_byte(SIO_COMPL);
    _delay_us(100);
    hw_usart_tx_byte(fast_pokey_div);
    hw_usart_tx_byte(fast_pokey_div);       // checksum
    led_off(LED_RED);
}

static void sio_drive_write_percom(void) {
    struct percom percom;
    uint8_t checksum;
    uint16_t bytes_per_sector, sectors_per_track;
    uint32_t nsectors;

    led_on(LED_RED);
    hw_usart_tx_byte(SIO_ACK);
    hw_usart_rx_block(&percom, sizeof(percom));
    checksum = hw_usart_rx_byte();
    _delay_us(750);
    if (checksum != sio_calc_checksum(&percom, sizeof(percom))) {
        hw_usart_tx_byte(SIO_NACK);
        return;
    }
    hw_usart_tx_byte(SIO_ACK);
    _delay_us(800);

    bytes_per_sector   = percom.bytes_per_sector_msb;
    bytes_per_sector <<= 8;
    bytes_per_sector  |= percom.bytes_per_sector_lsb;

    sectors_per_track   = percom.sectors_per_track_msb;
    sectors_per_track <<= 8;
    sectors_per_track  |= percom.sectors_per_track_lsb;

    nsectors  = percom.ntracks;
    nsectors *= sectors_per_track;
    nsectors *= percom.nsides_min_one + 1;

    if (nsectors < 1 || nsectors > 65535 ||
        (bytes_per_sector != 128 && bytes_per_sector != 256)) {
        hw_usart_tx_byte(SIO_ERROR);
        goto errout;
    }

    drive.bytes_per_sector  = percom.bytes_per_sector_msb;     // 0 or 1
    drive.sides             = percom.nsides_min_one + 1;
    drive.tracks            = percom.ntracks;
    drive.sectors_per_track = sectors_per_track;
    drive.nsectors          = nsectors;

    hw_usart_tx_byte(SIO_COMPL);

    config_save_current_drive();

errout:
    _delay_us(100);
    led_off(LED_RED);
}

static void sio_drive_read_percom(void) {
    struct percom percom;

    memset(&percom, 0, sizeof(percom));

    led_on(LED_RED);
    hw_usart_tx_byte(SIO_ACK);
    _delay_us(700);
    hw_usart_tx_byte(SIO_COMPL);
    _delay_us(100);

    percom.ntracks               = drive.tracks;
    percom.sectors_per_track_msb = MSB(drive.sectors_per_track);
    percom.sectors_per_track_lsb = LSB(drive.sectors_per_track);
    percom.nsides_min_one        = drive.sides - 1;
    percom.bytes_per_sector_msb  = MSB(128 << drive.bytes_per_sector);
    percom.bytes_per_sector_lsb  = LSB(128 << drive.bytes_per_sector);
    percom.drive_online          = 0xff;

    if (drive.sides != 1 || drive.tracks != 40 || drive.sectors_per_track != 18)
        percom.modulation = 4;      // MFM

    hw_usart_tx_block(&percom, sizeof(percom));
    hw_usart_tx_byte(sio_calc_checksum(&percom, sizeof(percom)));

    led_off(LED_RED);
}

static void sio_drive_format(uint8_t medium) {
    struct atr_header h;
    uint32_t pars;
    UINT bw;

    led_on(LED_RED);
    hw_usart_tx_byte(SIO_ACK);
    _delay_us(700);

    if (!drive.read_write || drive.type != TYPE_ATR) {
        hw_usart_tx_byte(SIO_ERROR);
        drive.controller_status |= STATUS_WRITE_PROTECTED;
        goto errout;
    }

    if (medium) {
        drive.bytes_per_sector  = 0;
        drive.sides             = 1;
        drive.tracks            = 40;
        drive.sectors_per_track = 26;
        drive.nsectors          = 1040;
    } else {
        drive.first_three_dd    = 0;
        drive.nsectors          = drive.sides *
                                  drive.tracks * drive.sectors_per_track;
    }

    memset(&h, 0, sizeof(h));

    pars   = (uint32_t) drive.nsectors * (128 << drive.bytes_per_sector);
    if (drive.bytes_per_sector)       // first_three_dd always false
        pars -= 3*128;
    pars >>= 4;

    h.wMagic     = 0x0296;
    h.wPars      = pars;
    h.wSecSize   = 128 << drive.bytes_per_sector;
    h.btParsHigh = pars >> 16;

    f_lseek(&drive.file, 0);
    f_write(&drive.file, &h, sizeof(h), &bw);

    // expand or truncate file
    for (i=0; i<RETRIES; i++) {
        if (f_lseek(&drive.file, pars*16+16) == FR_OK) break;
        _delay_ms(5);
    }
    f_truncate(&drive.file);
    f_lseek(&drive.file, 16);
    f_sync(&drive.file);

    if (i==RETRIES) {
        hw_usart_tx_byte(SIO_ERROR);
        drive.controller_status |= STATUS_WRITE_ERROR;
        goto errout;
    }

    hw_usart_tx_byte(SIO_COMPL);
    _delay_us(850);

    hw_usart_tx_byte(0xff);
    hw_usart_tx_byte(0xff);
    for (i=0; i<(128<<drive.bytes_per_sector)-2; i++)
        hw_usart_tx_byte(0);
    hw_usart_tx_byte(0xff);     // checksum

    config_save_current_drive();

errout:
    led_off(LED_RED);
    _delay_us(100);
}

/* ------------------------------------------------------------------------- */

/* Drives and disks */

static void enable_drive(void) {
    if (drive.filename[0])
        drive.enabled = 1;
}

static void disable_drive(void) {
    drive.enabled = 0;
}

static void drive_read_write(void) {
    drive.read_write = 1;
}

static void drive_read_only(void) {
    drive.read_write = 0;
}

static uint8_t insert_disk(char *filename) {
    FIL *fp;
    UINT br;
    struct atr_header h;
    uint32_t pars;
    const struct geometry_tab *tab = geometry_tab;

    fp = &drive.file;

    if (strlen(filename) >= sizeof(drive.filename)) goto errout;     // too long

    if (f_open(fp, filename, FA_READ|FA_WRITE) != FR_OK) {
//        debug("insert disk: no such file");
        goto errout;
    }

    f_read(fp, &h, sizeof(h), &br);

    pars   = h.btParsHigh;
    pars <<= 16;
    pars  |= h.wPars;

    if (h.wMagic == 0x0296) {
        drive.type             = TYPE_ATR;
        drive.bytes_per_sector = (h.wSecSize == 0x100);
        drive.nsectors         = (pars >> 3) >> drive.bytes_per_sector;
        drive.first_three_dd   = !((fp->fsize - 10) & 0xff);
        if (drive.bytes_per_sector && !drive.first_three_dd) {
            drive.nsectors += 2;
        }
    } else {
        drive.type = TYPE_XFD;
        drive.bytes_per_sector = 0;
        drive.nsectors         = fp->fsize >> 7;
        drive.first_three_dd   = 0;
    }

    while (pgm_read_word_near(&(tab->nsectors))) {
      if (pgm_read_word_near(&(tab->nsectors)) == drive.nsectors) {
        drive.sides  = pgm_read_byte_near(&(tab->sides));
        drive.tracks = pgm_read_byte_near(&(tab->tracks));
        drive.sectors_per_track = pgm_read_byte_near(&(tab->sectors_per_track));
        break;
      }
      tab++;
    }

    if (!(drive.sides)) {
        drive.sides = 1;
        drive.tracks = 1;
        drive.sectors_per_track = drive.nsectors;
    }

    // safe, because strlen(filename) is assured to be at least one less
    strncpy(drive.filename, filename, sizeof(drive.filename));
    drive.enabled    = 1;

    return 1;

errout:
    memset(&drive, 0, sizeof(struct drive));
    return 0;
}

static void remove_disk(void) {
    memset(&drive, 0, sizeof(struct drive));
}

static void rotate_drives(void) {
    struct drive drive_temp;

    config_save_current_drive();
    config_load_current_drive(0);
    memcpy(&drive_temp, &drive, sizeof(struct drive));

    for (i=1; i<4; i++) {
        config_load_current_drive(i);
        current_drive_num = i-1;
        config_save_current_drive();
    }

    memcpy(&drive, &drive_temp, sizeof(struct drive));
    current_drive_num = 3;
    config_save_current_drive();
}

/* ------------------------------------------------------------------------- */

/* Play 15625 Hz Wave file */

static void play_wav(const char *filename) {
    FIL test;
    uint8_t val[256];
    UINT br;

    pwm_init();

    if (f_open(&test, filename, FA_READ|FA_OPEN_EXISTING) != FR_OK) return;

    playbuf = val;                      // point to buffer on stack
    f_read(&test, &val, 256, &br);      // fill buffer
    pos = 46;                           // skip WAV header

    sei();                              // start playing
    while(1) {
        while (pos < 64) { }
        if (f_read(&test, playbuf, 64, &br) != FR_OK) break;
        while (pos < 128) { }
        if (f_read(&test, playbuf+64, 64, &br) != FR_OK) break;
        while (pos < 192) { }
        if (f_read(&test, playbuf+128, 64, &br) != FR_OK) break;
        while (pos > 191) { }
        if (f_read(&test, playbuf+192, 64, &br) != FR_OK) break;
        if (!(SIO_CMD_PIN & _BV(SIO_CMD_BIT))) break;
    }
    cli();                              // stop playing
    OCR1AL=0x00;
    f_close(&test);
}

ISR(TIMER1_OVF_vect) {
    toggle = !toggle;
    if (toggle) return;
    OCR1AL = playbuf[pos++];
}

/* ------------------------------------------------------------------------- */

int main(void) {
    cli();

    led_init();
    led_on(LED_RED);

    buttons_init();

    //--------------------//

    sw_usart_init();
    debug("\x1b[2J\r\nSIO2WORLD\r\n");

    twi_init();

#if 0
    twi_ds1307_read_timekeeper();
    debug("20%02x-%02x-%02x %02x:%02x:%02x (%02x)",
            timekeeper[6], timekeeper[5], timekeeper[4],
            timekeeper[2], timekeeper[1], timekeeper[0],
            timekeeper[3]);

    i = onewire_ds18b20_read_temperature();
    debug("%i.%i degrees Celcius", i>>4, i&8 ? 5 : 0);
#endif

    for (i=0; i<RETRIES; i++) {
        if (f_mount(&fatfs, "0:", 1) == FR_OK) break;
        _delay_ms(50);
    }
    if (i == RETRIES) fatal("cannot mount card");

    if (!config_open()) config_create();

#if 0
    config_load_current_drive(0);
    insert_disk("/ATR/S2WDEV.ATR");
    config_save_current_drive();

    config_load_current_drive(1);
    insert_disk("/ATR/S2WDEV2.ATR");
    config_save_current_drive();

    config_load_current_drive(2);
    insert_disk("/ATR/DISK3.ATR");
    config_save_current_drive();

    config_load_current_drive(3);
    insert_disk("/ATR/DISK4.ATR");
    config_save_current_drive();

    config_load_current_drive(4);
    insert_disk("/ATR/MYDOS-~1.ATR");
    config_save_current_drive();

    config_load_current_drive(5);
    insert_disk("/ATR/TEST.ATR");
    config_save_current_drive();
#endif

    for (i=0; i<NDRIVES; i++) {             // re-insert all disks
        config_load_current_drive(i);
        insert_disk(drive.filename);
        config_save_current_drive();
    }

    config_load_current_drive(0);
    drive_read_write();
    config_save_current_drive();
    config_load_current_drive(1);
    drive_read_write();
    config_save_current_drive();

    current_ubrr = \
    slow_ubrr = pokey_div_to_ubrr(slow_pokey_div);
    fast_ubrr = pokey_div_to_ubrr(fast_pokey_div);

    sio_init();

    led_off(LED_RED);
    led_on(LED_GREEN);

    for(;;) {
        while (SIO_CMD_PIN & _BV(SIO_CMD_BIT)) {
            if (button_pressed(BUTTON_ONE)) {
                led_on(LED_RED);
                rotate_drives();
                while (button_pressed(BUTTON_ONE)) ;
                led_off(LED_RED);
                _delay_ms(200);
            }
        }

        for (i=0; i<5; i++)
            cmd_frame[i] = hw_usart_rx_byte();

        checksum = sio_calc_checksum(cmd_frame, 4);

        _delay_us(900);

        if (checksum != cmd_frame[4]) {
            if (current_ubrr == slow_ubrr)
                current_ubrr = fast_ubrr;
            else
                current_ubrr = slow_ubrr;
try_again:
            while (!(SIO_CMD_PIN & _BV(SIO_CMD_BIT))) ;
            hw_usart_init(current_ubrr);
            continue;
        }

        _delay_us(50);

        if (cmd_frame[0] >= 0x31 && cmd_frame[0] <= 0x30+NDRIVES) {
            cmd_frame[0] -= 0x31;

            if (current_drive_num != cmd_frame[0])
                config_load_current_drive(cmd_frame[0]);

            if (!drive.enabled) goto try_again;

            switch(cmd_frame[1]) {

            case 0x21:  sio_drive_format(0);                break;
            case 0x22:  sio_drive_format(1);                break; // 1=medium
            case 0x3f:  sio_drive_read_high_speed_index();  break;
            case 0x4e:  sio_drive_read_percom();            break;
            case 0x4f:  sio_drive_write_percom();           break;
            case 0x50:
            case 0x57:  sio_drive_write_sector();           break;
            case 0x52:  sio_drive_read_sector();            break;
            case 0x53:  sio_drive_get_status();             break;

            default:
                debug("%02X %02X %02X %02X %02X   %04x", cmd_frame[0],
                    cmd_frame[1], cmd_frame[2], cmd_frame[3], cmd_frame[4],
                    checksum);
                hw_usart_tx_byte(SIO_NACK);
                break;
            }
        } if (cmd_frame[0] == 'S'+'2'+'W') { // 0xdc == SIO2WORLD Configurator

        }// else if --> other devices here....
    }

    return 0;
}
