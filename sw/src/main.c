#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

#include "../build/gg_capture.pio.h"
#include "../build/lcd_send.pio.h"

#define dw 20
#define cl2  21
#define led_pin 25
#define pixel_amount 300

#define lcd_rst 8
#define lcd_cs 9
#define lcd_dcx 10 // 0 -> Send command; 1 -> Send data
#define lcd_wr 11
#define lcd_rd 12

uint16_t * framebuffer = (uint16_t *)(0x20000000 + (1024 * 64));

/*static inline __attribute__((always_inline)) uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}*/

static inline __attribute__((always_inline)) void wait() {
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
}

static inline __attribute__((always_inline)) void select_lcd() {
	gpio_put(lcd_cs, 0);
}

static inline __attribute__((always_inline)) void deselect_lcd() {
	gpio_put(lcd_cs, 1);
}

static inline __attribute__((always_inline)) void write_command(uint8_t command) {	
	pio_sm_set_enabled(pio1, 0, false);
	pio_sm_put_blocking(pio1, 0, command);
	pio_sm_exec(pio1, 0, pio_encode_pull(false, true));
	pio_sm_exec(pio1, 0, pio_encode_mov_reverse(pio_osr, pio_osr) | pio_encode_sideset_opt(3, 0b100));
	pio_sm_exec(pio1, 0, pio_encode_out(pio_null, 24));
	pio_sm_exec(pio1, 0, pio_encode_out(pio_pins, 8));

	wait();
	wait();
	wait();
	wait();

	pio_sm_exec(pio1, 0, pio_encode_nop() | pio_encode_sideset_opt(3, 0b110));
	wait();
	pio_sm_exec(pio1, 0, pio_encode_nop() | pio_encode_sideset_opt(3, 0b111));
	pio_sm_set_enabled(pio1, 0, true);
}

static inline __attribute__((always_inline)) void write_data(uint8_t data) {	
	pio_sm_set_enabled(pio1, 0, false);
	pio_sm_put_blocking(pio1, 0, data);
	pio_sm_exec(pio1, 0, pio_encode_pull(false, true));
	pio_sm_exec(pio1, 0, pio_encode_mov_reverse(pio_osr, pio_osr) | pio_encode_sideset_opt(3, 0b101));
	pio_sm_exec(pio1, 0, pio_encode_out(pio_null, 24));
	pio_sm_exec(pio1, 0, pio_encode_out(pio_pins, 8));

	wait();
	wait();
	wait();
	wait();

	pio_sm_exec(pio1, 0, pio_encode_nop() | pio_encode_sideset_opt(3, 0b111));
	pio_sm_set_enabled(pio1, 0, true);
}

static inline __attribute__((always_inline)) void write_data_fast(uint8_t pix_data) {	
	pio_sm_put(pio1, 0, pix_data);
}

/*__attribute__((long_call, section(".data"))) uint8_t read_data() {
	uint8_t data;

	gpio_set_dir_in_masked(0xFF);

	gpio_put(lcd_dcx, 1);
	gpio_put(lcd_wr, 1);

	gpio_put(lcd_rd, 0);

	for (uint32_t w = 0; w < 40; w++) {
		wait();
	}

	gpio_put(lcd_rd, 1);

	data = gpio_get_all() & 255;

	return reverse(data);
}*/

void printFramebuffer() {
	for (uint32_t y = 0; y < 192; y++) {
		for (uint32_t x = 0; x < 256; x++) {
			putchar(framebuffer[y * pixel_amount + x + 44] >> 0);
			putchar(framebuffer[y * pixel_amount + x + 44] >> 8);
		}
	}
}

uint32_t pot_history[16];
uint32_t brightness = 0;
uint32_t scanlines_to_skip = 8;

__attribute__((long_call, section(".data"))) void update_lcd() {	
	//while(1) {
	uint32_t avg = 0;
	for (uint32_t c = 15; c > 0; c--)
	{
		avg += pot_history[c];
		pot_history[c] = pot_history[c - 1];
	}
	pot_history[0] = adc_fifo_get();
	avg += pot_history[0];
	avg = avg >> 7;
	brightness = avg;
	brightness &= 0b11110000; //LSBs have too much noise

	uint32_t is_gg = !gpio_get(15);

	scanlines_to_skip = is_gg ? 1 : 11;

	//Wait for frame to start being captured into the framebuffer
	while(!dma_channel_is_busy(0)) 
		;

	//Transfering the entire framebuffer to the LCD takes 13-14-ish ms
	//Thus, we need to give the PIO filling the framebuffer a bit of a head start
	//Wait for the first 40 scanlines of active area to be filled before starting LCD update
	while(dma_hw->ch[0].transfer_count > (262 - scanlines_to_skip - 40) * 192)
		;

	select_lcd();

	write_command(0x53); //Brightness control
	write_data(0b00101100);

	write_command(0x55); //Content Adaptive Brightness Control Value
  write_data(0); //Turn it off

	write_command(0x51); //Write brightness
	write_data(brightness);

	write_command(0x2A); // Column and page set
	write_data(0);
	write_data(0);
	write_data(1);
	write_data(0xDF);

	write_command(0x2B);
	write_data(0);
	write_data(0);
	write_data(1);
	write_data(0x3F);

	write_command(0x2C);

	pio_sm_put_blocking(pio0, 1, 192 + scanlines_to_skip + 3);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	dma_hw->ch[2].transfer_count = pixels_a_capturar * scanlines_to_skip;

	uint32_t line_counter = 0;
	uint32_t base = pixels_a_capturar + 44 + (is_gg ? 44 : 0);

	for (uint32_t y = 0; y < 320 * 256; y += is_gg ? 114 : 155)
	{
		if (line_counter >= 320)
			break;
		uint32_t pix_counter = 0;
		uint32_t y_base = base + (y >> 8) * pixel_amount;

		for (uint32_t x = 0; x < 480 * 64; x += is_gg ? 21 : 32)
		{
			if (pix_counter >= 480)
				break;
			uint32_t pixIndex = y_base + (x >> 6);

			write_data_fast((framebuffer[pixIndex] & 15) << 4);
			write_data_fast(((framebuffer[pixIndex] >> 4) & 15) << 4);
			write_data_fast(((framebuffer[pixIndex] >> 8) & 15) << 4);
			// write_pixel(framebuffer[pixIndex]);

			pix_counter++;
		}

		line_counter++;
		}

		deselect_lcd();
	//}
}

void init_lcd() {
	//Setup sequence for the ILI9486 LCD controller
	deselect_lcd();

	gpio_put(lcd_rst, 1);
	sleep_ms(100);

	select_lcd();

	write_command(0x11); //Sleep out
	sleep_ms(100);

  write_command(0xB1); //Frame-rate control
  write_data(0b10100000);
  write_data(0b10000);

  write_command(0x3A); //Interface pixel format
  write_data(0x66); //18-bit per pixel

  write_command(0x36); //Memory access control
  write_data(0b00110000);

  write_command(0xC2); //Power control 2
  write_data(0x44);

  write_command(0xC5); // VCOM control 1
  write_data(0);
  write_data(0);
  write_data(0);
  write_data(0);

  write_command(0xE0); //Positive gamma control
  write_data(0x0F);
  write_data(0x1F);
  write_data(0x1C);
  write_data(0x0C);
  write_data(0x0F);
  write_data(0x08);
  write_data(0x48);
  write_data(0x98);
  write_data(0x37);
  write_data(0x0A); 
  write_data(0x13); 
  write_data(0x04); 
  write_data(0x11); 
  write_data(0x0D); 
  write_data(0x00);

  write_command(0xE1); //Negative gamma control
  write_data(0x0F);
  write_data(0x32); 
  write_data(0x2E); 
  write_data(0x0B); 
  write_data(0x0D); 
  write_data(0x05); 
  write_data(0x47); 
  write_data(0x75);
	write_data(0x37); 
  write_data(0x06); 
  write_data(0x10); 
  write_data(0x03); 
  write_data(0x24);
  write_data(0x20); 
  write_data(0x00);

  write_command(0xE2); //Digital gamma control
  write_data(0x0F); 
  write_data(0x32); 
  write_data(0x2E); 
  write_data(0x0B); 
  write_data(0x0D); 
  write_data(0x05); 
  write_data(0x47); 
  write_data(0x75);
	write_data(0x37); 
  write_data(0x06); 
  write_data(0x10); 
  write_data(0x03); 
  write_data(0x24); 
  write_data(0x20); 
  write_data(0x00);

  write_command(0x38); //Idle mode off

  write_command(0x20); //Display inversion off

  write_command(0x13); //Normal display mode

	write_command(0x53); //Brightness control
	write_data(0b00101100);
	write_command(0x53);
	write_data(0b00001100);
	write_command(0x53);
	write_data(0b00101100);

  write_command(0x51);
	write_data(0xFF);

  write_command(0x29); //Display on

  deselect_lcd();
}

int main() {
	vreg_set_voltage(VREG_VOLTAGE_1_10);
	set_sys_clock_khz(256000, 1); //8x the GG pixel clock
	stdio_init_all();

	adc_init();
	adc_select_input(2);
	//Pin 28: Brightness potentiometer input
	adc_gpio_init(28);
	adc_run(true);
	adc_fifo_setup(true, false, 0, 0, 0);

	gpio_init_mask(0b11111111111111111111111111111111);
	gpio_set_dir_out_masked(1 << led_pin);

	//Pins 0-7: LCD 8-bit parallel data bus
	//Pins 8-12: LCD control signals
	gpio_set_dir_out_masked(0b1111111111111);

	gpio_put(9, 1);
	gpio_put(10, 1);
	gpio_put(11, 1);
	gpio_put(12, 1);
	gpio_put(lcd_rst, 0);

	pio_clear_instruction_memory(pio1);
	uint8_t lcd_send = pio_add_program(pio1, &lcd_send_program);
	pio_sm_config lcd_send_config = lcd_send_program_get_default_config(lcd_send);
	sm_config_set_clkdiv(&lcd_send_config, 1);
	sm_config_set_sideset(&lcd_send_config, 4, true, false);
	sm_config_set_sideset_pins(&lcd_send_config, 10);
	sm_config_set_out_pins(&lcd_send_config, 0, 8);
	sm_config_set_out_shift(&lcd_send_config, true, false, 32);
	sm_config_set_in_shift(&lcd_send_config, false, false, 32);

	for (uint32_t c = 0; c < 8; c++) {
		pio_gpio_init(pio1, c);
	}

	pio_gpio_init(pio1, 10);
	pio_gpio_init(pio1, 11);
	pio_gpio_init(pio1, 12);

	pio_sm_set_pindirs_with_mask(pio1, 0, 0xFF, 0xFF);
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << 10), (1 << 10));
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << 11), (1 << 11));
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << 12), (1 << 12));
	pio_sm_init(pio1, 0, lcd_send, &lcd_send_config);
	pio_sm_set_enabled(pio1, 0, true);

	pio_clear_instruction_memory(pio0);
	uint8_t detect_vblank = pio_add_program(pio0, &detect_vblank_program);
	pio_sm_config detect_vblank_config = detect_vblank_program_get_default_config(detect_vblank);
	//sm_config_set_sideset_pins(&detect_vblank_config, 26);
	sm_config_set_clkdiv(&detect_vblank_config, 1);

	uint8_t detect_hblank = pio_add_program(pio0, &detect_hblank_program);
	pio_sm_config detect_hblank_config = detect_hblank_program_get_default_config(detect_hblank);
	//sm_config_set_sideset_pins(&detect_hblank_config, 26);
	sm_config_set_clkdiv(&detect_hblank_config, 1);

	uint8_t get_data = pio_add_program(pio0, &get_data_program);
	pio_sm_config get_data_config = get_data_program_get_default_config(get_data);
	sm_config_set_clkdiv(&get_data_config, 1);
	sm_config_set_sideset_pins(&get_data_config, 26);
	//Pins 16-19: GG pixel data bus
	sm_config_set_in_pins(&get_data_config, 16);
	sm_config_set_in_shift(&get_data_config, false, false, 32);
	sm_config_set_jmp_pin(&get_data_config, 22);

	pio_gpio_init(pio0, 26);
	pio_sm_set_pindirs_with_mask(pio0, 2, (1 << 26), 0b11111111111111111111111111111111);

	pio_sm_init(pio0, 0, detect_vblank, &detect_vblank_config);
	pio_sm_init(pio0, 1, detect_hblank, &detect_hblank_config);
	pio_sm_init(pio0, 2, get_data, &get_data_config);

	pio_sm_put_blocking(pio0, 1, 192 + scanlines_to_skip + 3);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	/*pio_sm_put_blocking(pio0, 2, 47 * 3 * 2); //Pixels a ignorar * número de canais * 2 clocks por canal
	pio_sm_exec(pio0, 2, pio_encode_pull(false, true));
	pio_sm_exec(pio0, 2, pio_encode_mov(pio_isr, pio_osr));*/

	pio_sm_put_blocking(pio0, 2, pixel_amount - 1); //Pixels to capture
	pio_sm_exec(pio0, 2, pio_encode_pull(false, true));

	uint32_t dummy;

	dma_channel_config g = dma_channel_get_default_config(2);
	channel_config_set_transfer_data_size(&g, DMA_SIZE_16);
	channel_config_set_enable(&g, true);
	channel_config_set_chain_to(&g, 0);
	channel_config_set_read_increment(&g, false);
	channel_config_set_write_increment(&g, false);
	channel_config_set_dreq(&g, pio_get_dreq(pio0, 2, false));
	channel_config_set_high_priority(&g, 1);
	dma_channel_configure(
			2,
			&g,
			&dummy,
			&pio0_hw->rxf[2],
			pixel_amount * scanlines_to_skip,
			true);

	dma_channel_config f = dma_channel_get_default_config(0);
	channel_config_set_transfer_data_size(&f, DMA_SIZE_16);
	channel_config_set_enable(&f, true);
	channel_config_set_chain_to(&f, 1);
	channel_config_set_read_increment(&f, false);
	channel_config_set_write_increment(&f, true);
	channel_config_set_dreq(&f, pio_get_dreq(pio0, 2, false));
	channel_config_set_high_priority(&f, 1);
	dma_channel_configure(
			0,
			&f,
			&framebuffer[0],
			&pio0_hw->rxf[2],
			pixel_amount * 196,
			false);

	uint32_t start_addr = (0x20000000 + (1024 * 64));

	dma_channel_config e = dma_channel_get_default_config(1);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	channel_config_set_chain_to(&e, 2);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	channel_config_set_high_priority(&e, 1);
	dma_channel_configure(
			1,
			&e,
			&(dma_hw->ch[0].write_addr),
			&start_addr,
			1,
			false);

	pio_enable_sm_mask_in_sync(pio0, 0b111);

	gpio_put(led_pin, 1);

	uint8_t state = 0;
	uint8_t comando;

	while(1) {
		//comando = getchar();

		init_lcd();

		//interpret_command(comando, &state);
		while(1) {
			uint32_t comeco = time_us_32();
			update_lcd();
			uint32_t fim = time_us_32();
			//printf("Brilho é %i; ", brightness);
			//printf("Frame anterior levou %i us\n", fim - comeco);
		}

		//printFramebuffer();
	}

	return 0;
}
