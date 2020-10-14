#include "spi_flash.h"
//#include <libprintf/printf.h>

const uint8_t cmd_jedec_id_get = 0x9F;
const uint8_t cmd_write_enable = 0x06;
const uint8_t cmd_write_disable = 0x04;
const uint8_t cmd_write_byte = 0x02;
const uint8_t cmd_erase_full = 0xC7;
const uint8_t cmd_erase_64kb = 0xD8;
const uint8_t cmd_erase_32kb = 0x52;
const uint8_t cmd_erase_4kb = 0x20;
const uint8_t cmd_read_status_reg = 0x05;
const uint8_t cmd_read_slow = 0x03;
const uint8_t cmd_write_status_register = 0x01;


void spi_init(void)
{
    // Setup GPIO

    // Our SST25VF016B memory chip has maximum clock frequency of 80 MHz, so set speed to high
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOD);
    // Pins directly assigned to SPI peripheral
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, 1 << 5);
    gpio_set_af(GPIOA, GPIO_AF5, 1 << 5);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, 1 << 5);

    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, (1 << 5) | (1 << 4));
    gpio_set_af(GPIOB, GPIO_AF5, (1 << 5) | (1 << 4));
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, (1 << 5) | (1 << 4));
    // CS Pin we drive manually
    gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, 1 << 7);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, 1 << 7);
    gpio_set(GPIOD, 1 << 7);

    rcc_periph_clock_enable(RCC_SPI1);
    // Disable SPI1 before configuring
    spi_disable(SPI1);        // not required here, SPI is disabled after Reset
    // SPI1 belongs to APB2 (86 MHz frequency)
    // We have /2, /4, /8, /16, /32, /64, /128 and /256 prescalers
    // Our SPI flash can work up to 50 MHz ... 80 MHz clock (depending on part marking)
    // But to be able to capture data with logic analyzer, which has maximum frequency of 24 MHz,
    // set spi baudrate to /32, which gives us 86/32 = 2.6 MHz SCLK frequency
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_32);
    // Our MCU is master and flash chip is slave
    spi_set_master_mode(SPI1);
    // We work in full duplex (simultaneous transmit and receive)
    spi_set_full_duplex_mode(SPI1);
    // Data frame format is 8 bit, not 16
    spi_set_dff_8bit(SPI1);
    // No CRC calculation is required
    spi_disable_crc(SPI1);
    // Our flash chip requires Most Significant Bit first (datasheet p. 5, Figure 3. SPI Protocol)
    spi_send_msb_first(SPI1);
    // Flash chip can work in Mode 0 (polarity 0, phase 0) and Mode 3 (polarity 1, phase 1)
    // But the chip inputs data on rising edge and outputs data on falling edge, so
    // MCU is better to read data on rising edge and output on falling edge -- select mode 3
    spi_set_clock_polarity_1(SPI1);
    spi_set_clock_phase_1(SPI1);
    // Set hardware control of NSS pin. Because disabling it can cause master to become slave
    // depending on NSS input state.
    // Normally NSS will be hard-wired to slave. But in our case we have other pin connected to
    // slave CS and should drive it manually
    spi_enable_ss_output(SPI1);

    // In this example we will work with flash without using interrupts
    // So simply enable spi peripheral
    spi_enable(SPI1);
}

void cs_set(bool state)
{
    sk_pin_set(sk_io_spiflash_ce, state);
}
//spi byte transaction
void flash_tx(uint32_t len, const void *data)
{
    uint8_t *d = data;
    if ((!len) || (NULL == d))
        return;

    for (int32_t i = len - 1; i >= 0; i--) {
        spi_send(SPI1, d[i]);
        spi_read(SPI1);        // dummy read to provide delay
    }
}
//Spi byte read
void flash_rx(uint32_t len, void *data)
{
    // Note:
    // Our spi chip uses Big Endian byte order, while MCU is Little Endian
    // This means 0xABCD will be represented as ABCD on MCU, and CDAB on spi chip
    // (spi chip expects higher address bytes to be transfered first)
    // This means we either need to declare our structures and arrays as big endian
    // with __attribute__(( scalar_storage_order("big-endian") )), which is more portable
    // but leads to heavier computations,
    // or solve this at transfer level, sending and receiving higher bytes first
    uint8_t *d = data;
    if ((!len) || (NULL == d))
        return;

    for (int32_t i = len - 1; i >= 0; i--) {
        spi_send(SPI1, 0);
        d[i] = spi_read(SPI1);
    }
}
//Setting write enable bit to 1
void write_enable(void)
{
    cs_set(0);
    flash_tx(1, &cmd_write_enable);
    cs_set(1);
}
// Getting and displaying JEDEC-ID
void flash_get_jedec_id(void)
{
    cs_set(0);        // assert enable signal

    
    flash_tx(1, &cmd_jedec_id_get);

    struct flash_jedec_id jedec_id = { 0 };
    flash_rx(sizeof(jedec_id), &jedec_id);

    cs_set(1);
    sk_pin_set(sk_io_led_green, false);

    char buffer[20];
    sk_lcd_cmd_setaddr(&lcd, 0x00, false);
    snprintf(buffer, sizeof(buffer), "Manufacturer:%Xh", (unsigned int)jedec_id.manufacturer);
    lcd_putstring(&lcd, buffer);

    sk_lcd_cmd_setaddr(&lcd, 0x40, false);
    snprintf(buffer, sizeof(buffer), "Serial:%Xh", (unsigned int)jedec_id.device_id);
    lcd_putstring(&lcd, buffer);
    
}

void flash_unblock(void)
{
    uint8_t reg = 0x00000010;
    cs_set(0);
    flash_tx(1, &cmd_write_status_register);
    flash_tx(1, &cmd_write_status_register);
    cs_set(1);
}
// Getting busy bit    !!! INCORRECT
uint8_t flash_get_busy_bit(void)
{
    cs_set(0);
    flash_tx(1, &cmd_read_status_reg);
    uint8_t busy_status;
    flash_rx(1, &busy_status);
    cs_set(1);
    return busy_status;
}
// Write one byte of data to flash
void flash_write_byte(uint32_t addr, uint8_t data)
{
    union address addr1;
    addr1.data = addr;
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    cs_set(0);
    flash_tx(1, &cmd_write_byte);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    flash_tx(1, &data);
    cs_set(1);
}
// Read one byte of data from flash
uint8_t flash_read_byte(uint32_t addr)
{
    union address addr1;
    addr1.data = addr;
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    cs_set(0);
    flash_tx(1, &cmd_read_slow);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    uint8_t data_readed;
    flash_rx(1, &data_readed);
    cs_set(1);
    return data_readed;
}

void flash_write_2bytes(uint32_t addr, uint16_t data)
{
    union address addr1;
    union address addr2;
    union bytes_2 data1;
    addr1.data = addr;
    addr2.data = addr + 1;
    data1.data = data;
    flash_unblock();
    write_enable();
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    cs_set(0);
    flash_tx(1, &cmd_write_byte);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    flash_tx(1, &data1.byte[1]);
    cs_set(1);
    flash_unblock();
    write_enable();
    cs_set(0);
    flash_tx(1, &cmd_write_byte);
    flash_tx(1, &addr2.byte[2]);
    flash_tx(1, &addr2.byte[1]);
    flash_tx(1, &addr2.byte[0]);
    flash_tx(1, &data1.byte[0]);
    cs_set(1);
}

uint16_t flash_read_2bytes(uint32_t addr)
{
    union address addr1;
    union address addr2;
    union bytes_2 data1;
    addr1.data = addr;
    addr2.data = addr + 1;
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    cs_set(0);
    flash_tx(1, &cmd_read_slow);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    flash_rx(1, &data1.byte[1]);
    cs_set(1);
    cs_set(0);
    flash_tx(1, &cmd_read_slow);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    flash_rx(1, &data1.byte[0]);
    cs_set(1);
    return data1.data;
}

// Erasing all flash memory
void flash_erase_full(void)
{
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    cs_set(0);
    flash_tx(1, &cmd_erase_full);
    cs_set(1);
}
// erase 64 kb sector
void flash_erase_64kb(uint32_t addr)
{
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    union address addr1;
    addr1.data = addr;
    cs_set(0);
    flash_tx(1, &cmd_erase_64kb);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    cs_set(1);
}
// erase 32 kb sector
void flash_erase_32kb(uint32_t addr)
{
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    union address addr1;
    addr1.data = addr;
    cs_set(0);
    flash_tx(1, &cmd_erase_32kb);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    cs_set(1);
}
// erase 4 kb sector
void flash_erase_4kb(uint32_t addr)
{
    while((flash_get_busy_bit() & (1 << 0)) != 0);
    union address addr1;
    addr1.data = addr;
    cs_set(0);
    flash_tx(1, &cmd_erase_4kb);
    flash_tx(1, &addr1.byte[2]);
    flash_tx(1, &addr1.byte[1]);
    flash_tx(1, &addr1.byte[0]);
    cs_set(1);
}
// flash demo, recording one byte and read one byte
void flash_test(void)
{
    //write_enable();
    uint32_t addr = 0x000000;
    flash_write_byte(addr, 0x08);
    uint8_t test_byte = flash_read_byte(addr);
    
    char buffer[20];
    sk_lcd_cmd_setaddr(&lcd, 0x00, false);
    snprintf(buffer, sizeof(buffer), "test:");
    lcd_putstring(&lcd, buffer);

    sk_lcd_cmd_setaddr(&lcd, 0x40, false);
    snprintf(buffer, sizeof(buffer), "%Xh", test_byte);
    lcd_putstring(&lcd, buffer);
    
}
// Displaying status register in decimal format
void flash_show_status_reg(void)
{
    cs_set(0);
    flash_tx(1, &cmd_read_status_reg);
    uint8_t reg;
    flash_rx(1, &reg);
    cs_set(1);
    char buffer[20];
    sk_lcd_cmd_setaddr(&lcd, 0x00, false);


	snprintf(buffer, sizeof(buffer), "Reg:%d", reg);
    lcd_putstring(&lcd, buffer);
    
}


