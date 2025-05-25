#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/structs/pwm.h"
#include "hardware/pwm.h"
#include "hardware/interp.h"

#include "../build/gg_capture.pio.h"
#include "../build/lcd_send.pio.h"

#include "libdvi/dvi.h"
#include "libdvi/dvi_serialiser.h"
#include "libdvi/dvi_timing.h"

#define DVI_TIMING dvi_timing_640x480p_60hz

#define dw 20
#define cl2  21
#define led_pin 25
#define SMS_pin 15
#define pixels_in_scanline 300
#define scanlines_in_active_area 192

#define lcd_D0 7
#define lcd_D1 8
#define lcd_D2 9
#define lcd_D3 10
#define lcd_den 26
#define lcd_clk 27
#define lcd_rst 6
#define lcd_backlight 14

#define fdbck 28

#define brightness_pot 29

#define lcd_width 320
#define lcd_channels 3
#define lcd_active_lines 240
#define lcd_hblank_len 95 
#define lcd_blank_lines 3

#define scanlines_to_skip 11

struct dvi_inst dvi0;

static const struct dvi_serialiser_cfg pico_gg_lcd_conf = {
	.pio = pio1,
	.sm_tmds = {1, 2, 3},
	.pins_tmds = {23, 2, 0},
	.pins_clk = 4,
	.invert_diffpairs = false
};

void core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
	dvi_scanbuf_main_12bpp(&dvi0);
}

//Use two framebuffers to prevent tearing
uint16_t * framebuffer = (uint16_t *)(0x20000000 + (1024 * 30));
uint16_t * framebuffer2 = (uint16_t *)(0x20000000 + (1024 * 30) + (pixels_in_scanline * scanlines_in_active_area));

uint32_t pot_history[16];
uint32_t brightness = 0;
uint8_t pwm_backlight_slice;
uint32_t is_gg;
uint32_t gg_now;
uint32_t last_gg;

uint32_t dma_chan0;
uint32_t dma_chan1;
uint32_t dma_chan2;
uint32_t dma_chan3;
uint32_t dma_chan4;
uint32_t dma_chan5;

static inline __attribute__ ((always_inline)) uint32_t unpack(uint32_t rgb_value) {
	uint32_t temp = rgb_value >> 8;
	temp |= rgb_value << 20;
	temp |= rgb_value << 6;
	temp &= 0b111100000011110000001111;

	return temp;
}

__attribute__ ((long_call, section (".time_critical"))) void update_lcd_gg() {	
	//Scanning a frame out into the LCD is faster than ~16ms
	//So, we wait until the console starts sending out a new frame before we update the LCD again
	//If neither of these two DMA channels is busy, then the console must be in vblank
	while(!dma_channel_is_busy(dma_chan0) && !dma_channel_is_busy(dma_chan4)) ;

	uint16_t * curr_framebuffer;

	//See which of the two framebuffer is currently being written to and pick the other one to send to the LCD
	//This introduces a single frame of latency
	if(dma_channel_is_busy(dma_chan0)) curr_framebuffer = framebuffer2;
	else curr_framebuffer = framebuffer;

	pio_sm_put_blocking(pio0, 1, scanlines_in_active_area + 1 - 1);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	dma_hw->ch[2].transfer_count = pixels_in_scanline * 1;
	dma_hw->ch[3].transfer_count = pixels_in_scanline * 1;

	//Because of the LCD's orientation, we actually need to scan the image out upside down and flipped
	//base points to the bottom right of the original image
	uint32_t base = pixels_in_scanline + 48 - 52 + (pixels_in_scanline * scanlines_in_active_area) - pixels_in_scanline * 51;

	for (uint32_t y = 0; y < 240; y ++)
	{
		//Where in the framebuffer the current scanline starts
		uint32_t y_base = base - ((y * 310) >> 9) * pixels_in_scanline + 256;

		//Tell LCD we're starting the active portion of the scanline
		gpio_put(lcd_den, 0);

		///unpack transforms 0b0000rrrrggggbbbb word into 0bbbb000000gggg000000rrrr word
		//Channel order is changed to bgr because that's what the LCD expects
		//Channel bits are spread out so we can do SIMD-within-a-word
		//The interpolator will do linear interpolation on all three channels at once, using a single 32-bit word
		uint32_t old_pix_value = unpack(curr_framebuffer[y_base] & 4095);

		for (uint32_t x = 0; x < 320; x ++)
		{
			//sub_pixel is a 24.8 bit fixed point coordinate
			uint32_t sub_pixel = (x * 133);

			//Use integer part to pick a pixel from the framebuffer
			uint32_t pixIndex = y_base - (sub_pixel >> 8);
			uint32_t new_pix_value = unpack(curr_framebuffer[pixIndex] & 4095);

			//Values to be interpolated
			interp0->base[0] = old_pix_value;
			interp0->base[1] = new_pix_value;

			//Fractional part of sub_pixel tells the interpolator how much of each pixel to use
			interp0->accum[1] = sub_pixel & 0b11111100;

			uint32_t pix_value = interp0->peek[1];
			//uint32_t pix_value = new_pix_value; //Uncomment this line to use nearest-neighbor instead of linear interpolation

			old_pix_value = new_pix_value;

			//By the time we get here, the PIO is probably gonna be done sending the last pixel anyways
			while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;

			//This way, we only have to poll on the FIFO status once and then can just blast the three words at once
			//Seems to be significantly faster than pio_sm_put_blocking on each word
			pio_sm_put(pio1, 0, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(pio1, 0, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(pio1, 0, pix_value & 15);
		}

		//Signal end of active scanline portion
		gpio_put(lcd_den, 1);

		//Send empty pixels during Hblank
		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(pio1, 0, 0);
		}
	}

	//Start of vblank
	gpio_put(lcd_den, 1);

	//Send empty data for the entirety of vblank
	for(uint32_t wait_line = 0; wait_line < lcd_blank_lines; wait_line ++) {
		for(uint32_t pixel = 0; pixel < lcd_width; pixel ++) {
			for(uint32_t channel = 0; channel < lcd_channels; channel ++) {
				pio_sm_put_blocking(pio1, 0, 0);
			}
		}

		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(pio1, 0, 0);
		}
	}
}

//The only thing that changes for SMS mode are the scaling constants
//However, putting those values in variables considerably slows down this function
//So, instead, we use magic constants baked into the code
__attribute__ ((long_call, section (".time_critical"))) void update_lcd_sms() {	
    while(!dma_channel_is_busy(dma_chan0) && !dma_channel_is_busy(dma_chan4)) ;
	uint16_t * curr_framebuffer;

	if(dma_channel_is_busy(dma_chan0)) curr_framebuffer = framebuffer2;
	else curr_framebuffer = framebuffer;

	pio_sm_put_blocking(pio0, 1, scanlines_in_active_area + scanlines_to_skip - 1);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	dma_hw->ch[2].transfer_count = pixels_in_scanline * scanlines_to_skip;
	dma_hw->ch[3].transfer_count = pixels_in_scanline * scanlines_to_skip;

	uint32_t line_counter = 0;
	uint32_t base = pixels_in_scanline + 256 + 48 - 3 + (pixels_in_scanline * scanlines_in_active_area) - pixels_in_scanline * 2;

	for (uint32_t y = 0; y < 240; y ++)
	{
		uint32_t y_base = base - ((y * 413) >> 9) * pixels_in_scanline;

		gpio_put(lcd_den, 0);

		uint32_t old_pix_value = unpack(curr_framebuffer[y_base] & 4095);

		for (uint32_t x = 0; x < 320; x ++)
		{
			uint32_t sub_pixel = (x * 210);
			uint32_t pixIndex = y_base - (sub_pixel >> 8);
			uint32_t new_pix_value = unpack(curr_framebuffer[pixIndex] & 4095);

			interp0->base[0] = old_pix_value;
			interp0->base[1] = new_pix_value;
			interp0->accum[1] = sub_pixel;

			uint32_t pix_value = interp0->peek[1];
			old_pix_value = new_pix_value;

			while(!pio_sm_is_tx_fifo_empty(pio1, 0)) ;

			pio_sm_put(pio1, 0, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(pio1, 0, pix_value & 15);
			pix_value >>= 10;
			pio_sm_put(pio1, 0, pix_value & 15);
		}

		gpio_put(lcd_den, 1);

		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(pio1, 0, 0);
		}
	}

	gpio_put(lcd_den, 1);

	for(uint32_t wait_line = 0; wait_line < lcd_blank_lines; wait_line ++) {
		for(uint32_t pixel = 0; pixel < lcd_width; pixel ++) {
			for(uint32_t channel = 0; channel < lcd_channels; channel ++) {
				pio_sm_put_blocking(pio1, 0, 0);
			}
		}

		for(uint32_t wait = 0; wait < lcd_hblank_len; wait ++) {
			pio_sm_put_blocking(pio1, 0, 0);
		}
	}
}

void config_pios() {
	pio_clear_instruction_memory(pio1);

	//PIO1 is used to send data to the LCD without hogging the CPU
	pio_sm_claim(pio1, 0);

	uint8_t lcd_send = pio_add_program(pio1, &lcd_send_program);
	pio_sm_config lcd_send_config = lcd_send_program_get_default_config(lcd_send);
	sm_config_set_clkdiv(&lcd_send_config, 1);
	sm_config_set_sideset_pins(&lcd_send_config, lcd_clk);
	sm_config_set_out_pins(&lcd_send_config, lcd_D0, 4);
	sm_config_set_out_shift(&lcd_send_config, true, false, 32);
	sm_config_set_in_shift(&lcd_send_config, false, false, 32);

	pio_gpio_init(pio1, lcd_clk);
	pio_gpio_init(pio1, lcd_D0);
	pio_gpio_init(pio1, lcd_D1);
	pio_gpio_init(pio1, lcd_D2);
	pio_gpio_init(pio1, lcd_D3);

	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_clk), (1 << lcd_clk));
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_D0), (1 << lcd_D0));
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_D1), (1 << lcd_D1));
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_D2), (1 << lcd_D2));
	pio_sm_set_pindirs_with_mask(pio1, 0, (1 << lcd_D3), (1 << lcd_D3));
	pio_sm_init(pio1, 0, lcd_send, &lcd_send_config);
	pio_sm_set_enabled(pio1, 0, true);

	pio_clear_instruction_memory(pio0);

	//PIO0 captures data from the GG's video bus
	pio_sm_claim(pio0, 0);
	pio_sm_claim(pio0, 1);
	pio_sm_claim(pio0, 2);

	uint8_t detect_vblank = pio_add_program(pio0, &detect_vblank_program);
	pio_sm_config detect_vblank_config = detect_vblank_program_get_default_config(detect_vblank);
	sm_config_set_clkdiv(&detect_vblank_config, 1);

	uint8_t detect_hblank = pio_add_program(pio0, &detect_hblank_program);
	pio_sm_config detect_hblank_config = detect_hblank_program_get_default_config(detect_hblank);
	sm_config_set_clkdiv(&detect_hblank_config, 1);

	uint8_t get_data = pio_add_program(pio0, &get_data_program);
	pio_sm_config get_data_config = get_data_program_get_default_config(get_data);
	sm_config_set_clkdiv(&get_data_config, 1);
	sm_config_set_in_pins(&get_data_config, 16);
	sm_config_set_in_shift(&get_data_config, false, false, 32);
	sm_config_set_jmp_pin(&get_data_config, 22);

	pio_sm_init(pio0, 0, detect_vblank, &detect_vblank_config);
	pio_sm_init(pio0, 1, detect_hblank, &detect_hblank_config);
	pio_sm_init(pio0, 2, get_data, &get_data_config);

	pio_sm_put_blocking(pio0, 1, scanlines_in_active_area + scanlines_to_skip - 1);
	pio_sm_exec(pio0, 1, pio_encode_pull(false, true));

	pio_sm_put_blocking(pio0, 2, pixels_in_scanline - 1); 
	pio_sm_exec(pio0, 2, pio_encode_pull(false, true));

	pio_enable_sm_mask_in_sync(pio0, 0b111);
}

uint32_t dummy;

void config_dma() {
	for(uint32_t c = 0; c < 12; c++) {
		dma_channel_cleanup(c);
    dma_channel_unclaim(c);
	}

	//DVI code uses claim_unused_channel, so we must as well, instead of explicitly picking DMA channels
	dma_chan0 = dma_claim_unused_channel(true);
	dma_chan1 = dma_claim_unused_channel(true);
	dma_chan2 = dma_claim_unused_channel(true);
	dma_chan3 = dma_claim_unused_channel(true);
	dma_chan4 = dma_claim_unused_channel(true);
	dma_chan5 = dma_claim_unused_channel(true);

	//PIO0 SM2 is the one that actually sends out pixel data
	dma_channel_config g = dma_channel_get_default_config(dma_chan2);
	channel_config_set_transfer_data_size(&g, DMA_SIZE_16);
	channel_config_set_enable(&g, true);
	channel_config_set_chain_to(&g, dma_chan0);
	channel_config_set_read_increment(&g, false);
	channel_config_set_write_increment(&g, false);
	channel_config_set_dreq(&g, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan2,
			&g,
			&dummy,
			&pio0_hw->rxf[2],
			//The first few active display scanlines are actually blank
			//We skip them by taking data from the PIO and discarding it into the dummy variable
			//We than chain to dma_chan0 
			pixels_in_scanline * scanlines_to_skip,
			false);

	//Now we can save the scanlines that actually have image data into the framebuffer
	dma_channel_config f = dma_channel_get_default_config(dma_chan0);
	channel_config_set_transfer_data_size(&f, DMA_SIZE_16);
	channel_config_set_enable(&f, true);
	channel_config_set_chain_to(&f, dma_chan1);
	channel_config_set_read_increment(&f, false);
	channel_config_set_write_increment(&f, true);
	channel_config_set_dreq(&f, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan0,
			&f,
			//Note that we're filling the first framebuffer here
			//There's two of them, for double buffering, so we don't get screen tearing
			&framebuffer[0],
			&pio0_hw->rxf[2],
			//Again, the active area has border scanlines that don't actually contain game graphics
			//We only save the *real* scanlines into the framebuffer
			//Then chain to dma_chan1
			pixels_in_scanline * scanlines_in_active_area,
			false);

	//Reset dma_chan0's write address register to point to the start of framebuffer0
	//Then chain to dma_chan3
	dma_channel_config e = dma_channel_get_default_config(dma_chan1);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	channel_config_set_chain_to(&e, dma_chan3);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	dma_channel_configure(
			dma_chan1,
			&e,
			&(dma_hw->ch[dma_chan0].write_addr),
			&framebuffer,
			1,
			false);

	//Skip unwanted lines, but now for the second frame
	g = dma_channel_get_default_config(dma_chan3);
	channel_config_set_transfer_data_size(&g, DMA_SIZE_16);
	channel_config_set_enable(&g, true);
	channel_config_set_chain_to(&g, dma_chan4);
	channel_config_set_read_increment(&g, false);
	channel_config_set_write_increment(&g, false);
	channel_config_set_dreq(&g, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan3,
			&g,
			&dummy,
			&pio0_hw->rxf[2],
			pixels_in_scanline * scanlines_to_skip,
			false);

	//Save active scanlines into second framebuffer
	f = dma_channel_get_default_config(dma_chan4);
	channel_config_set_transfer_data_size(&f, DMA_SIZE_16);
	channel_config_set_enable(&f, true);
	channel_config_set_chain_to(&f, dma_chan5);
	channel_config_set_read_increment(&f, false);
	channel_config_set_write_increment(&f, true);
	channel_config_set_dreq(&f, pio_get_dreq(pio0, 2, false));
	dma_channel_configure(
			dma_chan4,
			&f,
			&framebuffer2[0],
			&pio0_hw->rxf[2],
			pixels_in_scanline * scanlines_in_active_area,
			false);

	//Reset dma_chan4's write address and chain into dma_chan2 to restart the whole DMA chain
	e = dma_channel_get_default_config(dma_chan5);
	channel_config_set_transfer_data_size(&e, DMA_SIZE_32);
	channel_config_set_enable(&e, true);
	channel_config_set_chain_to(&e, dma_chan2);
	channel_config_set_read_increment(&e, false);
	channel_config_set_write_increment(&e, false);
	dma_channel_configure(
			dma_chan5,
			&e,
			&(dma_hw->ch[dma_chan4].write_addr),
			&framebuffer2,
			1,
			false);
}

void config_backlight_supply(uint8_t pwm_backlight_slice) {
	adc_init();
	adc_select_input(fdbck - 26);
	adc_gpio_init(fdbck);
	adc_run(true);
	adc_fifo_setup(true, false, 0, 0, 0);

	pwm_hw->slice[pwm_backlight_slice].cc = 250;
	sleep_ms(50);

	float tensao_media = 0;

	/*while(tensao_media < 19 || tensao_media > 20) {
		for(uint32_t m = 0; m < 5; m++) {
			sleep_ms(2);
			tensao_media += adc_fifo_get();
		}

		tensao_media *= 3.3;
		tensao_media *= 11;
		tensao_media /= 5;
		tensao_media /= 4095;

		if(tensao_media < 10) {
			pwm_hw->slice[pwm_backlight_slice].cc += 50;
		}
		else if(tensao_media < 15) {
			pwm_hw->slice[pwm_backlight_slice].cc += 20;
		}
		else if(tensao_media < 19) {
			pwm_hw->slice[pwm_backlight_slice].cc += 10;
		}
		else if(tensao_media > 21) {
			pwm_hw->slice[pwm_backlight_slice].cc -= 20;
		}
		else if(tensao_media > 19) {
			pwm_hw->slice[pwm_backlight_slice].cc -= 10;
		}

		gpio_put(led_pin, !gpio_get(led_pin));
	}*/
	pwm_hw->slice[pwm_backlight_slice].cc = 3500;
	//pwm_hw->slice[pwm_backlight_slice].cc = 20;
}

uint8_t config_backlight_pwm() {
	gpio_set_function(lcd_backlight, GPIO_FUNC_PWM);
	pwm_backlight_slice = pwm_gpio_to_slice_num(lcd_backlight);

	pwm_config config = pwm_get_default_config();
	pwm_config_set_phase_correct(&config, false);
	pwm_config_set_clkdiv_int(&config, 1);
	pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
	pwm_config_set_wrap(&config, (DVI_TIMING.bit_clk_khz * 1000) / 50000);
	pwm_init(pwm_backlight_slice, &config, true);

	return pwm_backlight_slice;
}

void config_interp() {
	//Claim both lanes
	interp_claim_lane_mask(interp0, 0b11);

	interp_config cfg = interp_default_config();
	//Only interpolator 0 has " blend " mode, which does linear interpolation
	interp_config_set_blend(&cfg, true);
	interp_set_config(interp0, 0, &cfg);

	cfg = interp_default_config();
	interp_set_config(interp0, 1, &cfg);
}

void init_lcd() {
	gpio_put(lcd_rst, 1);
	sleep_ms(10);
	gpio_put(lcd_rst, 0);
	sleep_ms(10);
	gpio_put(lcd_rst, 1);
}

void fill_framebuffer_with_test_pattern() {
	for(uint32_t y = 0; y < scanlines_in_active_area; y++) {
		for(uint32_t x = 0; x < pixels_in_scanline; x++) {
			uint16_t pixel = 0;

			if(x > 150 && y > 96) pixel |= 15;
			if(x < 150 && y < 96) pixel |= 15;

			if(x < 150 && y > 96) pixel |= (15 << 8);
			if(x <= 150 && y <= 96) pixel |= (15 << 8);

			if(x > 150 && y < 96) pixel |= (15 << 4);
			if(x <= 150 && y <= 96) pixel |= (15 << 4);

			framebuffer[x + y * pixels_in_scanline] = pixel;
			framebuffer2[x + y * pixels_in_scanline] = ~pixel;
		}
	}
}

int main() {
	vreg_set_voltage(VREG_VOLTAGE_1_10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
	stdio_init_all();

	gpio_init_mask(0b11111111111111111111111111111111);
	gpio_set_dir_out_masked(1 << led_pin);
	gpio_set_dir_out_masked(1 << lcd_rst);
	gpio_set_dir_out_masked(1 << lcd_den);
	gpio_set_dir_out_masked(1 << lcd_backlight);

	uint8_t pwm_backlight_slice = config_backlight_pwm();
	config_backlight_supply(pwm_backlight_slice);

	//These functions MUST be called before dvi_init, since they unclaim all DMA channels and clear PIO memory
	init_lcd();
	config_pios();
	config_dma();
	config_interp();

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = pico_gg_lcd_conf;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	multicore_launch_core1(core1_main);

	gpio_put(lcd_den, 1);
	gpio_put(lcd_clk, 0);
	gpio_put(lcd_backlight, 0);

	adc_init();
	adc_select_input(brightness_pot - 26);
	adc_gpio_init(brightness_pot);
	adc_run(true);
	adc_fifo_setup(true, false, 0, 0, 0);

	fill_framebuffer_with_test_pattern();

	dma_channel_start(dma_chan2);

	while(1) {
		last_gg = gg_now;
		gg_now = !gpio_get(SMS_pin);

		if(last_gg == gg_now) is_gg = gg_now;

		uint32_t start = time_us_32();
		if(!is_gg) update_lcd_gg();
		else update_lcd_sms();
		uint32_t end = time_us_32();

		uint32_t avg = 0;
		for (uint32_t c = 15; c > 0; c--)
		{
			avg += pot_history[c];
			pot_history[c] = pot_history[c - 1];
		}
		pot_history[0] = adc_fifo_get();
		avg += pot_history[0];
		avg >>= 4;
		brightness = avg;
		brightness *= 6;

		//printf("Pot: %i\n", brightness);
		printf("Rendering time: %i us\n", end - start);
	}

	return 0;
}
 
