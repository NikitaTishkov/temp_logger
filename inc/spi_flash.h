#include "lcd_hd44780.h"
#include "pin.h"
#include "tick.h"
#include <stdint.h>
#include <libprintf/printf.h>
#include <stddef.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
//lcd settings for initialization
static struct sk_lcd lcd = {
    .pin_group_data = &sk_io_lcd_data,
    .pin_rs = &sk_io_lcd_rs,
    .pin_en = &sk_io_lcd_en,
    .pin_rw = &sk_io_lcd_rw,
    .pin_bkl = &sk_io_lcd_bkl,
    .set_backlight_func = NULL,
    .delay_func_us = NULL,
    .delay_func_ms = &sk_tick_delay_ms,
    .is4bitinterface = true,
    .charmap_func = &sk_lcd_charmap_rus_cp1251
};
//Address 
union address {
    uint32_t data;
    unsigned char byte[4];
};
//2 int bytes
union bytes_2{
    uint16_t data;
    uint8_t byte[2];
};
//For test1
struct __attribute__((packed, aligned(1)))
       __attribute__(( scalar_storage_order("little-endian") ))
       flash_jedec_id {
    // order matters here
    uint16_t device_id;
    uint8_t manufacturer;
};
// lcd print
void lcd_putstring(struct sk_lcd *lcd, const char *str);
// SPI initialization
void spi_init(void);
//SPI C# port on
void cs_set(bool state);
//One byte transaction via SPI
void flash_tx(uint32_t len, const void *data);
//One byte reading from SPI
void flash_rx(uint32_t len, void *data);
//Write enable bit to 1
void write_enable(void);
//test
void flash_get_jedec_id(void);
// Getting busy bit    !!! INCORRECT
uint8_t flash_get_busy_bit(void);
// Write one byte to flash
void flash_write_byte(uint32_t addr, uint8_t data);
//Read one byte from flash
uint8_t flash_read_byte(uint32_t addr);
//Flash write 2 bytes
void flash_write_2bytes(uint32_t addr, uint16_t data);
//Read 2 bytes from flash
uint16_t flash_read_2bytes(uint32_t addr);
//Erasing full flash
void flash_erase_full(void);
//Erasing xxKbyte sectors
void flash_erase_64kb(uint32_t addr);

void flash_erase_32kb(uint32_t addr);

void flash_erase_4kb(uint32_t addr);
//test
void flash_test(void);
//Dispaling status register
void flash_show_status_reg(void);
//Setting BL bits to 0
void flash_unblock(void);