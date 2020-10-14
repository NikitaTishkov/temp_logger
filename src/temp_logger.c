
//#include "lcd_hd44780.h"
//#include "pin.h"
//#include "tick.h"
#include "macro.h"
#include "intrinsics.h"
#include "spi_flash.h"
//#include <libprintf/printf.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/adc.h>
//#include <stdint.h>
//#include <stddef.h>

//------------------------------------------------------------------------------------INIT COLLECTION SPI&CLOCK----------------------------------------------------------------------------------------
static void clock_init(void)
{
    // Set clock to 168 MHz (external 8 MHz generator input from on-board ST-Link
    // multiplied with PLL
    // For more detailed description, see example "168mhz_extclk.c" in 06_clock_system

    rcc_osc_bypass_enable(RCC_HSE);        // bypass load capacitors used for crystals
    rcc_osc_on(RCC_HSE);                // enable High-Speed External clock (HSE)
    while (!rcc_is_osc_ready(RCC_HSE));    // trap until external clock is detected as stable

    rcc_osc_off(RCC_PLL);        // Disable PLL before configuring it

    // * Set PLL multiplication factor as specified at page 226 RM
    // PLLM = 4        -- 8/4 = 2 MHz input to PLL mul stage
    // PLLN = 168   -- F<main> = 2 * 168 = 336 MHz
    // PLLP = 2        -- F<genout> = 336 / 2 = 168 MHz for our CPU and AHB
    // PLLQ = 7        -- F<Qdomain> = 336 / 7 = 48 MHz exactly
    rcc_set_main_pll_hse(4, 168, 2, 7, 0);        // with input from HSE to PLL
    rcc_css_disable();        // Disable clock security system
    rcc_osc_on(RCC_PLL);                // Enable PLL
    while (!rcc_is_osc_ready(RCC_PLL)); // Wait for PLL out clock to stabilize

    // Set all prescalers.
    rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);    // AHB = 168 / 1 = 168 MHz
    rcc_set_ppre1(RCC_CFGR_PPRE_DIV_4);        // APB1 = FAHB / 4 = 168 / 4 = 42 MHz  (<= 42 MHz)
    rcc_set_ppre2(RCC_CFGR_PPRE_DIV_2);        // APB2 = FAHB / 2 = 168 / 2 = 84 MHz  (<= 84 MHz)

    // Enable caches. Flash is slow (around 30 MHz) and CPU is fast (168 MHz)
    flash_dcache_enable();
    flash_icache_enable();

    flash_set_ws(FLASH_ACR_LATENCY_7WS); // IMPORTANT! We must increase flash wait states (latency)


    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);        // Select PLL as AHB bus (and CPU clock) source
    rcc_wait_for_sysclk_status(RCC_PLL);        // Wait for clock domain to be changed to PLL input

    // set by hand since we've not used rcc_clock_setup_pll
    rcc_ahb_frequency = 168000000ul;
    rcc_apb1_frequency = rcc_ahb_frequency / 4;
    rcc_apb2_frequency = rcc_ahb_frequency / 2;
    rcc_osc_off(RCC_HSI);        // Disable internal 16 MHz RC oscillator (HSI)
}


// Among all GL-SK peripherals, only external SPI flash (SST25VF016B) is connected via SPI
// PA5 - SPI1_SCK    - Alternate function AF5
// PB5 - SPI1_MOSI    - Alternate function AF5
// PB4 - SPI1_MISO    - Alternate function AF5
// PD7 - ~CS        - driven manually, use push-pull out with pullup (PULLUP IS IMPORTANT)
//


//--------------------------------------------------------------------------------------------ADC collection

static void adc_temp_init(void)
{
    // * Sensor is connected to ADC12IN9 -- ADC1 and ADC2 (pin is common for them), input 9
    // * By opening STM32F407_Datasheet_(DS8626).pdf at page 50, we can see ADC12_IN9 is
    //   an additional function of PB1 pin
    // * So set PB1 to analog
    rcc_periph_clock_enable(RCC_GPIOB);
    // set ospeed to lowest to minimize noise and power use as application note recommends
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 1 << 1);
    // now set this pin to analog mode
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, 1 << 1);

    // setup ADC
    // * By looking at STM32F407_Datasheet_(DS8626).pdf page 133, we can see that for 3.3 V
    //   power, the ADC frequency fadc should be in range [0.6 MHz ... 36 MHz]
    //   with typical frequency around 30 MHz
    // * We're measuring temperature, which has slow-changing nature, but the input signal
    //   could have some fluctuations and noise. As we're not dealing with fast signal here,
    //   we're interested in long sampling time, which will serve us as a sort of analog
    //     low-pass filter. And filter-out some noise.
    // * So let's set ADC frequency to some low value. Not the lowest possible 0.6 MHz, but
    //   with some margin (around 20 😵 above the lowest value.

    // * Page 390 of Reference manual tells us:
    //     ADCCLK, common to all ADCs is generated from the APB2 clock divided by a programmable
    //   prescaler that allows the ADC to work at f PCLK2 /2, /4, /6 or /8
    // * Our core (by default) is clocked from AHB bus at 16 MHz and we don't change the defaults
    //   in this example
    // * APB2 clock is derived from AHB and has its own dividing prescaler by /2, /4, /6, /8, /16
    // * But our GPIO is also clocked from APB2. And if we set APB2 divisor to maximum /16,
    //   it will also affect GPIO maximum frequency.
    // * So the clock scheme is as follows:
    //             ___AHB___
    //         /              \
    //      [Core, ...]      APB2  with prescaler (/2, /4, /6, /8, /16)
    //                     /         \
    //                TIMER, ...      ADC prescaler (/2, /4, /6 or /8)
    //                                |
    //                               ADC
    // * So if we set ADC frequency to some low value. Not the lowest possible 0.6 MHz, but
    //   with some margin (around 20 😵 above the lowest value, we could have some alternative
    //   lowest frequency variants:
    //       16 MHz / 24 = 0.66 MHz    ( good, divide by /4 and by /6 giving us division by /24 )
    //         16 MHz / 32 = 0.5 MHz   !!!! below allowed, so not an option !!!!
    //          16 MHz / 16 = 1 MHz       ( also good )
    // * RESULT: let's set to 1 MHz with /2 ABP2 prescaler and /8 ADC prescaler

    // * We have ADC12 pin, this means ADC1 and ADC2. Any can be chosen. Let's work with ADC1

    // Set APB2 divisor to /2
    rcc_set_ppre2(RCC_CFGR_PPRE_DIV_2);
    // Enable ADC clock
    rcc_periph_clock_enable(RCC_ADC1);

    // Set ADC prescaler to /8 (bits ADCPRE in ADC_CCR)
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);

    // Set ADC resolution to 12 bit
    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    // Set left-to-right data alignment in adc result register
    adc_set_right_aligned(ADC1);

    // * We will convert 16 times using ADC groups feature, then calculate average value
    //   to further reduce noise and jitter. Set up these groups
    // * Set sampling time. We will sample for 480 fadc cycles, which gives us sampling
    //   frequency of Fs = fadc / 480 = 1 MHz / 480 = 2083.33 Hz
    // * So, set all 16 regular group sampling times to 480 cycles (SMPx bits in ADC_SMPRx)
    uint8_t channels[16];
    for (uint32_t i = 0; i < sk_arr_len(channels); i++) {
        adc_set_sample_time(ADC1, i, ADC_SMPR_SMP_480CYC);
        channels[i] = 9;    // set each element of group to channel 1
    }
    adc_set_regular_sequence(ADC1, sk_arr_len(channels), channels);

    // Configure End of Conversion (EOC) flag to be set after each channel in group
    // is converted. This will raise interrupt, where we read the conversion value
    adc_eoc_after_each(ADC1);

    // Enable scan mode to convert a group of channels
    adc_enable_scan_mode(ADC1);

    // Configure to perform a conversion of group of channels and stop
    adc_set_single_conversion_mode(ADC1);

    adc_enable_eoc_interrupt(ADC1);
    adc_enable_overrun_interrupt(ADC1);

    // Set ADC interrupts in NVIC
    nvic_set_priority(NVIC_ADC_IRQ, 10);
    nvic_enable_irq(NVIC_ADC_IRQ);

    // Enable ADC1. And wait some time to stabilize and calibrate itself
    adc_power_on(ADC1);
    sk_tick_delay_ms(10);
}

static volatile uint16_t __adc_avgval = 0;

void adc_isr(void)
{
    static volatile uint32_t sum = 0;
    static volatile uint32_t cnt = 0;

    if (adc_get_overrun_flag(ADC1)) {
        // means we got to interrupt because of overrun flag
        // we have not read all data and overrun occured -> reset everything
        sum = cnt = 0;
        adc_clear_flag(ADC1, ADC_SR_OVR);    // reset flag to avoid infinite interrupting
        return;
    }

    // otherwise we got here because one of channels was converted
    sum += adc_read_regular(ADC1) & 0x00000FFF;
    cnt++;

    if (cnt >= 16) {        // all sequence was read
        __adc_avgval = sum / cnt;
        sum = cnt = 0;
        adc_clear_flag(ADC1, ADC_SR_STRT);    // clear regular channel
    }

    adc_clear_flag(ADC1, ADC_SR_EOC);    // clear end of conversion flag not to cycle
}

static uint16_t adc_acquire(void)
{
    // start conversion
    adc_start_conversion_regular(ADC1);
    
    // wait for conversion to start
    while (! adc_get_flag(ADC1, ADC_SR_STRT)) {
        __WFI();
    }
    // sleep while adc has not finished coverting all channels in group
    while (adc_get_flag(ADC1, ADC_SR_STRT)) {
        __WFI();
    }

    return __adc_avgval;    // converted value after averaging in ISR
}

static inline float adc_temp_convert(uint16_t adcval, char temp_mode)
{
    float res;
    if(temp_mode == 'C'){
        res = 115.31 - 0.04500 * adcval;
    }else if(temp_mode == 'F'){
        res = ((115.31 - 0.04500 * adcval) * 1.8) + 32;
    }else if(temp_mode == 'K'){
        res = 115.31 - 0.04500 * adcval + 273;
    }
    return res;
}
//-----------------------------------------------------------------------------------temp log functions---------------------------------
//Display recorded data

float temp_mid(uint32_t start_temp_addr, uint8_t records_number){
    uint32_t sum = 0;
    uint8_t temp_buf;
    for(uint16_t i = 0; i < records_number; i++){
        temp_buf = flash_read_byte(start_temp_addr + i);
        sum += temp_buf;
    }
    float middle_tempreture = sum / records_number;
    return middle_tempreture;
}

void temp_log_show(uint32_t start_temp_addr, uint32_t start_time_addr, uint8_t records_number, char temp_mode){
    uint8_t num = 1; // 1< <64
    uint8_t data_temp[2];
    uint8_t data_time[2];
    uint32_t temp_addr = start_temp_addr;
    uint32_t time_addr = start_time_addr;
    while(1){
        data_temp[0] = flash_read_byte(temp_addr);
        sk_tick_delay_ms(20);
        data_time[0] = flash_read_byte(time_addr);
        sk_tick_delay_ms(20);
        data_temp[1] = flash_read_byte(temp_addr + 1);
        sk_tick_delay_ms(20);
        data_time[1] = flash_read_byte(time_addr + 1);
        sk_tick_delay_ms(20);
        char buffer[20];
        sk_lcd_cmd_setaddr(&lcd, 0x00, false);
        snprintf(buffer, sizeof(buffer), "%d: %d°%c %dsec", num, data_temp[0], temp_mode, data_time[0]);
        lcd_putstring(&lcd, buffer);
        
        sk_lcd_cmd_setaddr(&lcd, 0x40, false);
        snprintf(buffer, sizeof(buffer), "%d: %d°%c %dsec", num + 1, data_temp[1], temp_mode, data_time[1]);
        lcd_putstring(&lcd, buffer);
        
        if((sk_pin_read(sk_io_btn_up) == true) && (num > 2)){
            num -= 2;
            temp_addr -= 2;
            time_addr -= 2;
            sk_lcd_cmd_clear(&lcd);
            continue;
        }
        if((sk_pin_read(sk_io_btn_down) == true) && (num < records_number - 2)){
            num += 2;
            temp_addr +=2;
            time_addr +=2;
            sk_lcd_cmd_clear(&lcd);
            continue;
        }
        
        if(sk_pin_read(sk_io_btn_right) == true){
            sk_lcd_cmd_clear(&lcd);
            break;
        }
        
        if(sk_pin_read(sk_io_btn_left) == true){
            sk_lcd_cmd_clear(&lcd);
            float temp_average = temp_mid(start_temp_addr, records_number);
            char tmp[20];
            snprintf(tmp, sk_arr_len(tmp), "%5.1f", temp_average);
            
            while(1){
                char buffer[20];
                sk_lcd_cmd_setaddr(&lcd, 0x00, false);
                snprintf(buffer, sizeof(buffer), "averege:");
                lcd_putstring(&lcd, buffer);
                
                sk_lcd_cmd_setaddr(&lcd, 0x40, false);
                snprintf(buffer, sizeof(buffer), "%-5s °C", tmp);
                lcd_putstring(&lcd, buffer);
                
                if(sk_pin_read(sk_io_btn_left) == true){
                    sk_lcd_cmd_clear(&lcd);
                    break;
                }
            }
        }
        
        
    }
    
}
//Tracing tempreture values and recording them in two parallel sectors
void temp_log_trace(uint32_t start_temp_addr, uint32_t start_time_addr, uint8_t records_number, char temp_mode, uint8_t interval){
    
    write_enable();
    uint16_t timer = 0;
    for(uint16_t i = 0; i < records_number; i++){
        sk_lcd_cmd_clear(&lcd);
        flash_unblock();
        uint16_t adcval = adc_acquire();
        uint8_t temp = adc_temp_convert(adcval, temp_mode);
        write_enable();
        flash_write_byte(start_temp_addr + i, temp);
        flash_unblock();
        write_enable();
        flash_write_byte(start_time_addr + i, timer);
        char buffer[20];
        sk_lcd_cmd_setaddr(&lcd, 0x00, false);
        snprintf(buffer, sizeof(buffer), "T: %d°%c", temp, temp_mode);
        lcd_putstring(&lcd, buffer);
        
        sk_lcd_cmd_setaddr(&lcd, 0x40, false);
        snprintf(buffer, sizeof(buffer), "   %dsec", timer);
        lcd_putstring(&lcd, buffer);
        
        sk_tick_delay_ms(900);
        
        sk_tick_delay_ms(1000*(interval - 1));
        
        timer += interval;
        sk_pin_toggle(sk_io_led_orange);
    }
    sk_pin_toggle(sk_io_led_green);
    temp_log_show(start_temp_addr, start_time_addr, records_number, temp_mode);
    
}
//Interface for setting number of tracing recording into flash
uint8_t change_record_number(void){
    char buffer[20];
    
    uint8_t rec_var[4] = {16, 32, 64, 128};
    uint8_t inc = 0;
    uint8_t rec_num = rec_var[0];
    while(1){
        rec_num = rec_var[inc];
        sk_lcd_cmd_setaddr(&lcd, 0x00, false);
        snprintf(buffer, sizeof(buffer), "record`s number");
        lcd_putstring(&lcd, buffer);
        sk_lcd_cmd_setaddr(&lcd, 0x40, false);
        snprintf(buffer, sizeof(buffer), "   <  %d  >", rec_num);
        lcd_putstring(&lcd, buffer);
        
        if((sk_pin_read(sk_io_btn_left) == true) && inc > 0){
            inc--;
            sk_lcd_cmd_clear(&lcd);
            continue;
        }
        
        if((sk_pin_read(sk_io_btn_right) == true) && inc < 3){
            inc++;
            sk_lcd_cmd_clear(&lcd);
            continue;
        }
        
        if(sk_pin_read(sk_io_btn_mid) == true){
            sk_lcd_cmd_clear(&lcd);
            break;
        }
    }
    return rec_num;
}
//Interface for setting time interval for recordings
uint8_t change_interval(void)
{
    char buffer[20];
    
    uint8_t interval_var[4] = {1, 2, 3, 4};
    uint8_t inc = 0;
    uint8_t interval = interval_var[0];
    while(1){
        interval = interval_var[inc];
        sk_lcd_cmd_setaddr(&lcd, 0x00, false);
        snprintf(buffer, sizeof(buffer), "interval s.");
        lcd_putstring(&lcd, buffer);
        sk_lcd_cmd_setaddr(&lcd, 0x40, false);
        snprintf(buffer, sizeof(buffer), "   <  %d  >", interval);


		lcd_putstring(&lcd, buffer);
        
        if((sk_pin_read(sk_io_btn_left) == true) && inc > 0){
            inc--;
            sk_lcd_cmd_clear(&lcd);
            continue;
        }
        
        if((sk_pin_read(sk_io_btn_right) == true) && inc < 3){
            inc++;
            sk_lcd_cmd_clear(&lcd);
            continue;
        }
        
        if(sk_pin_read(sk_io_btn_mid) == true){
            sk_lcd_cmd_clear(&lcd);
            break;
        }
    }
    return interval;
}

void change_mode(char *temp_mode, char *triger_mode){
    char temp_modes[2] = {'C', 'F'};
    char triger_modes[2] = {'M', 'T'};
    uint8_t inc1 = 0;
    uint8_t inc2 = 0;
    while(1){
        char buffer[20];
        *temp_mode = temp_modes[inc1];
        *triger_mode = triger_modes[inc2];
        sk_lcd_cmd_setaddr(&lcd, 0x00, false);
        snprintf(buffer, sizeof(buffer), " ^ ");
        lcd_putstring(&lcd, buffer);
        sk_lcd_cmd_setaddr(&lcd, 0x40, false);
        snprintf(buffer, sizeof(buffer), " v %c  < %c >", *triger_mode, *temp_mode);
        lcd_putstring(&lcd, buffer);

        if(sk_pin_read(sk_io_btn_mid) == true){
                sk_lcd_cmd_clear(&lcd);
                break;
            }

        if((sk_pin_read(sk_io_btn_left) == true) && inc1 > 0){
                inc1--;
                sk_lcd_cmd_clear(&lcd);
                continue;
            }
            
            if((sk_pin_read(sk_io_btn_right) == true) && inc1 < 1){
                inc1++;
                sk_lcd_cmd_clear(&lcd);
                continue;
            }
        if((sk_pin_read(sk_io_btn_down) == true) && inc2 > 0){
                inc2--;
                sk_lcd_cmd_clear(&lcd);
                continue;
            }
            
        if((sk_pin_read(sk_io_btn_up) == true) && inc2 < 1){
                inc2++;
                sk_lcd_cmd_clear(&lcd);
                continue;
            }    
    }

}
//Finding the ferst empty address in sector
uint32_t flash_find_last_empty_addr(uint32_t sector, uint32_t sector_size)
{
    uint32_t last_addr = 0;
    uint8_t test_read;
    for(uint32_t i = 0; i < sector_size; i++){
        test_read = flash_read_byte(sector + i);
        if(test_read == 255){
            last_addr = sector + i;
            break;
        }
    }
    return last_addr;
}

int main(void)
{
    rcc_periph_clock_enable(RCC_GPIOE);        // lcd is connected to port E
    rcc_periph_clock_enable(RCC_GPIOD);        // DISCOVERY LEDS are connected to Port E
    glsk_pins_init(false);

    sk_pin_set(sk_io_led_green, true);    // turn on LED until everything is configured
    clock_init();
    sk_pin_set(sk_io_led_green, false);

    sk_tick_init(168000000ul / 10000ul, (2 << 2 | 0));

    cm_enable_interrupts();

    sk_lcd_init(&lcd);
    sk_lcd_set_backlight(&lcd, 200);

    spi_init();
    
    adc_temp_init();
    
    write_enable();
    
 
    //Start values
    
    uint8_t interval = 1; //Period of recordings in seconds
    uint8_t records_number = 16; //Number of recordings
    // memory controll values
    uint32_t last_addr_time = flash_find_last_empty_addr(0x000800, 2048);
    uint32_t last_addr_temp = flash_find_last_empty_addr(0x000000, 2048);
    
    if(last_addr_temp == 0){ 
        flash_erase_4kb(0x000A00);
        flash_erase_4kb(0x000800);
        flash_erase_4kb(0x000400);
        flash_erase_4kb(0x000000);
    }
    //Trigger mode controll variables
    uint8_t triger_mode = 0;
    char mode_sign = 'M';
    uint8_t triger_temp_min = 15;
    uint8_t triger_temp_max = 26;
    //Temp mode
    char temp_mode = 'C';
    //history
    uint8_t history_rec_num = 0;
    uint32_t history_time_addr;
    uint32_t history_temp_addr;
    
    while (1) {
        // ADC read and display
        uint16_t adcval = adc_acquire();
        uint8_t temp = adc_temp_convert(adcval, temp_mode);
        char buffer[20];
        sk_lcd_cmd_setaddr(&lcd, 0x00, false);
        snprintf(buffer, sk_arr_len(buffer), "Temp logger %c", mode_sign);
        lcd_putstring(&lcd, buffer);

        sk_lcd_cmd_setaddr(&lcd, 0x40, false);
        snprintf(buffer, sk_arr_len(buffer), "T=%d°%c l.a:%X", temp, temp_mode, last_addr_temp);
        lcd_putstring(&lcd, buffer);
        
        
        // Start tracing on click
        if(  sk_pin_read(sk_io_btn_mid) == true ){
            temp_log_trace(last_addr_temp, last_addr_time, records_number, temp_mode, interval);
            history_temp_addr = last_addr_temp;
            history_time_addr = last_addr_time;
            history_rec_num = records_number;
            last_addr_time = flash_find_last_empty_addr(0x000800, 2048);
            last_addr_temp = flash_find_last_empty_addr(0x000000, 2048);
        }
        // Start tracing in trigger mode
        
        if( ( triger_mode == 1 ) && (temp_mode == 'C') && ( (temp >= triger_temp_max) || ( temp <= triger_temp_min ) ) ){
            temp_log_trace(last_addr_temp, last_addr_time, records_number, temp_mode, interval);
            history_temp_addr = last_addr_temp;
            history_time_addr = last_addr_time;
            history_rec_num = records_number;
            last_addr_time = flash_find_last_empty_addr(0x000800, 2048);
            last_addr_temp = flash_find_last_empty_addr(0x000000, 2048);

        }
        
        sk_tick_delay_ms(100);
        if( ( triger_mode == 1 ) && (temp_mode == 'F') && ( (temp >= 80) || ( temp <= 60 ) ) ){
            temp_log_trace(last_addr_temp, last_addr_time, records_number, temp_mode, interval);
            history_temp_addr = last_addr_temp;
            history_time_addr = last_addr_time;
            history_rec_num = records_number;
            
        }
        
        sk_tick_delay_ms(100);
        //Checking adress
        last_addr_time = flash_find_last_empty_addr(0x000800, 2048);
        last_addr_temp = flash_find_last_empty_addr(0x000000, 2048);
        


        //Erasing sectors, if they`re full
        if(last_addr_temp == 0){ 
        flash_erase_4kb(0x000A00);
        flash_erase_4kb(0x000800);
        flash_erase_4kb(0x000400);
        flash_erase_4kb(0x000000);
    }
        //Show history
        if( ( sk_pin_read(sk_io_btn_up) == true ) && ( history_rec_num != 0 ) ){
            temp_log_show(history_temp_addr, history_time_addr, history_rec_num, temp_mode);
        }
        //Changing number of records
        if(sk_pin_read(sk_io_btn_left) == true){
            records_number = change_record_number();
        }
        //Changing interval
        if(sk_pin_read(sk_io_btn_right) == true){
            interval = change_interval();
        }
        
        if(sk_pin_read(sk_io_btn_down) == true){
            sk_lcd_cmd_clear(&lcd);
            change_mode(&temp_mode, &mode_sign);
            if(mode_sign == 'T'){
                triger_mode = 1;
            }else{
                triger_mode = 0;
            }
        }
        
        sk_tick_delay_ms(100);
    }
}