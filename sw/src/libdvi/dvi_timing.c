#include "dvi.h"
#include "dvi_timing.h"
#include "hardware/dma.h"

// This file contains:
// - Timing parameters for DVI modes (horizontal + vertical counts, best
//   achievable bit clock from 12 MHz crystal)
// - Helper functions for generating DMA lists based on these timings

// Pull into RAM but apply unique section suffix to allow linker GC
#define __dvi_func(x) __not_in_flash_func(x)
#define __dvi_const(x) __not_in_flash_func(x)

// VGA -- we do this mode properly, with a pretty comfortable clk_sys (252 MHz)
const struct dvi_timing __dvi_const(dvi_timing_640x480p_60hz) = {
	.h_sync_polarity   = false,
	.h_front_porch     = 16,
	.h_sync_width      = 96,
	.h_back_porch      = 48,
	.h_active_pixels   = 640,

	.v_sync_polarity   = false,
	.v_front_porch     = 10,
	.v_sync_width      = 2,
	.v_back_porch      = 33,
	.v_active_lines    = 480,

	.bit_clk_khz       = 252000
};

// ----------------------------------------------------------------------------

// The DMA scheme is:
//
// - One channel transferring data to each of the three PIO state machines
//   performing TMDS serialisation
//
// - One channel programming the registers of each of these data channels,
//   triggered (CHAIN_TO) each time the corresponding data channel completes
//
// - Lanes 1 and 2 have one block for blanking and one for video data
//
// - Lane 0 has one block for each horizontal region (front porch, hsync, back
//   porch, active)
//
// - The IRQ_QUIET flag is used to select which data block on the sync lane is
//   allowed to generate an IRQ upon completion. This is the block immediately
//   before the horizontal active region. The IRQ is entered at ~the same time
//   as the last data transfer starts
//
// - The IRQ points the control channels at new blocklists for next scanline.
//   The DMA starts the new list automatically at end-of-scanline, via
//   CHAIN_TO.
//
// The horizontal active region is the longest continuous transfer, so this
// gives the most time to handle the IRQ and load new blocklists.
//
// Note a null trigger IRQ is not suitable because we get that *after* the
// last data transfer finishes, and the FIFOs bottom out very shortly
// afterward. For pure DVI (four blocks per scanline), it works ok to take
// four regular IRQs per scanline and return early from 3 of them, but this
// breaks down when you have very short scanline sections like guard bands.

// Each symbol appears twice, concatenated in one word. Note these must be in
// RAM because they see a lot of DMA traffic
const uint32_t __dvi_const(dvi_ctrl_syms)[4] = {
	0xd5354,
	0x2acab,
	0x55154,
	0xaaeab
};

// Output solid red scanline if we are given NULL for tmdsbuff
#if DVI_SYMBOLS_PER_WORD == 2
static uint32_t __dvi_const(empty_scanline_tmds)[3] = {
	0x7fd00u, // 0x00, 0x00
	0x7fd00u, // 0x00, 0x00
	0xbfa01u  // 0xfc, 0xfc
};
#else
static uint32_t __attribute__((aligned(8))) __dvi_const(empty_scanline_tmds)[6] = {
	0x100u, 0x1ffu, // 0x00, 0x00
	0x100u, 0x1ffu, // 0x00, 0x00
	0x201u, 0x2feu  // 0xfc, 0xfc
};
#endif

void dvi_timing_state_init(struct dvi_timing_state *t) {
	t->v_ctr = 0;
	t->v_state = DVI_STATE_FRONT_PORCH;
}

void __dvi_func(dvi_timing_state_advance)(const struct dvi_timing *t, struct dvi_timing_state *s) {
		s->v_ctr++;
		if ((s->v_state == DVI_STATE_FRONT_PORCH && s->v_ctr == t->v_front_porch) || 
		    (s->v_state == DVI_STATE_SYNC && s->v_ctr == t->v_sync_width) ||
		    (s->v_state == DVI_STATE_BACK_PORCH && s->v_ctr == t->v_back_porch) ||
		    (s->v_state == DVI_STATE_ACTIVE && s->v_ctr == t->v_active_lines)) {

			s->v_state = (s->v_state + 1) % DVI_STATE_COUNT;
			s->v_ctr = 0;
		}
}

void dvi_scanline_dma_list_init(struct dvi_scanline_dma_list *dma_list) {
	*dma_list = (struct dvi_scanline_dma_list){};	
}

static const uint32_t *get_ctrl_sym(bool vsync, bool hsync) {
	return &dvi_ctrl_syms[!!vsync << 1 | !!hsync];
}

// Make a sequence of paced transfers to the relevant FIFO
static void _set_data_cb(dma_cb_t *cb, const struct dvi_lane_dma_cfg *dma_cfg,
		const void *read_addr, uint transfer_count, uint read_ring, bool irq_on_finish) {
	cb->read_addr = read_addr;
	cb->write_addr = dma_cfg->tx_fifo;
	cb->transfer_count = transfer_count;
	cb->c = dma_channel_get_default_config(dma_cfg->chan_data);
	channel_config_set_ring(&cb->c, false, read_ring);
	channel_config_set_dreq(&cb->c, dma_cfg->dreq);
	// Call back to control channel for reconfiguration:
	channel_config_set_chain_to(&cb->c, dma_cfg->chan_ctrl);
	// Note we never send a null trigger, so IRQ_QUIET is an IRQ suppression flag
	channel_config_set_irq_quiet(&cb->c, !irq_on_finish);
}

void dvi_setup_scanline_for_vblank(const struct dvi_timing *t, const struct dvi_lane_dma_cfg dma_cfg[],
		bool vsync_asserted, struct dvi_scanline_dma_list *l) {

	bool vsync = t->v_sync_polarity == vsync_asserted;
	const uint32_t *sym_hsync_off = get_ctrl_sym(vsync, !t->h_sync_polarity);
	const uint32_t *sym_hsync_on  = get_ctrl_sym(vsync,  t->h_sync_polarity);
	const uint32_t *sym_no_sync   = get_ctrl_sym(false,  false             );

	dma_cb_t *synclist = dvi_lane_from_list(l, TMDS_SYNC_LANE);
	// The symbol table contains each control symbol *twice*, concatenated into 20 LSBs of table word, so we can always do word-repeat.
	_set_data_cb(&synclist[0], &dma_cfg[TMDS_SYNC_LANE], sym_hsync_off, t->h_front_porch   / DVI_SYMBOLS_PER_WORD, 2, false);
	_set_data_cb(&synclist[1], &dma_cfg[TMDS_SYNC_LANE], sym_hsync_on,  t->h_sync_width    / DVI_SYMBOLS_PER_WORD, 2, false);
	_set_data_cb(&synclist[2], &dma_cfg[TMDS_SYNC_LANE], sym_hsync_off, t->h_back_porch    / DVI_SYMBOLS_PER_WORD, 2, true);
	_set_data_cb(&synclist[3], &dma_cfg[TMDS_SYNC_LANE], sym_hsync_off, t->h_active_pixels / DVI_SYMBOLS_PER_WORD, 2, false);

	for (int i = 0; i < N_TMDS_LANES; ++i) {
		if (i == TMDS_SYNC_LANE)
			continue;
		dma_cb_t *cblist = dvi_lane_from_list(l, i);
		_set_data_cb(&cblist[0], &dma_cfg[i], sym_no_sync,(t->h_front_porch + t->h_sync_width + t->h_back_porch) / DVI_SYMBOLS_PER_WORD, 2, false);
		_set_data_cb(&cblist[1], &dma_cfg[i], sym_no_sync, t->h_active_pixels / DVI_SYMBOLS_PER_WORD, 2, false);
	}
}

void dvi_setup_scanline_for_active(const struct dvi_timing *t, const struct dvi_lane_dma_cfg dma_cfg[],
		uint32_t *tmdsbuf, struct dvi_scanline_dma_list *l) {

	const uint32_t *sym_hsync_off = get_ctrl_sym(!t->v_sync_polarity, !t->h_sync_polarity);
	const uint32_t *sym_hsync_on  = get_ctrl_sym(!t->v_sync_polarity,  t->h_sync_polarity);
	const uint32_t *sym_no_sync   = get_ctrl_sym(false,                false             );

	dma_cb_t *synclist = dvi_lane_from_list(l, TMDS_SYNC_LANE);
	_set_data_cb(&synclist[0], &dma_cfg[TMDS_SYNC_LANE], sym_hsync_off, t->h_front_porch / DVI_SYMBOLS_PER_WORD, 2, false);
	_set_data_cb(&synclist[1], &dma_cfg[TMDS_SYNC_LANE], sym_hsync_on,  t->h_sync_width  / DVI_SYMBOLS_PER_WORD, 2, false);
	_set_data_cb(&synclist[2], &dma_cfg[TMDS_SYNC_LANE], sym_hsync_off, t->h_back_porch  / DVI_SYMBOLS_PER_WORD, 2, true);

	for (int i = 0; i < N_TMDS_LANES; ++i) {
		dma_cb_t *cblist = dvi_lane_from_list(l, i);
		if (i != TMDS_SYNC_LANE) {
			_set_data_cb(&cblist[0], &dma_cfg[i], sym_no_sync,
				(t->h_front_porch + t->h_sync_width + t->h_back_porch) / DVI_SYMBOLS_PER_WORD, 2, false);
		}
		int target_block = i == TMDS_SYNC_LANE ? DVI_SYNC_LANE_CHUNKS - 1 :  DVI_NOSYNC_LANE_CHUNKS - 1;
		if (tmdsbuf) {
			// Non-repeating DMA for the freshly-encoded TMDS buffer
			_set_data_cb(&cblist[target_block], &dma_cfg[i], tmdsbuf + i * (t->h_active_pixels / DVI_SYMBOLS_PER_WORD),
				t->h_active_pixels / DVI_SYMBOLS_PER_WORD, 0, false);
		}
		else {
			// Use read ring to repeat the correct DC-balanced symbol pair on blank scanlines (4 or 8 byte period)
			_set_data_cb(&cblist[target_block], &dma_cfg[i], &empty_scanline_tmds[2 * i / DVI_SYMBOLS_PER_WORD],
				t->h_active_pixels / DVI_SYMBOLS_PER_WORD, DVI_SYMBOLS_PER_WORD == 2 ? 2 : 3, false);
		}
	}
}

void __dvi_func(dvi_update_scanline_data_dma)(const struct dvi_timing *t, const uint32_t *tmdsbuf, struct dvi_scanline_dma_list *l) {
	for (int i = 0; i < N_TMDS_LANES; ++i) {
#if DVI_MONOCHROME_TMDS
		const uint32_t *lane_tmdsbuf = tmdsbuf;
#else
		const uint32_t *lane_tmdsbuf = tmdsbuf + i * t->h_active_pixels / DVI_SYMBOLS_PER_WORD;
#endif
		if (i == TMDS_SYNC_LANE)
			dvi_lane_from_list(l, i)[3].read_addr = lane_tmdsbuf;
		else
			dvi_lane_from_list(l, i)[1].read_addr = lane_tmdsbuf;
	}
}

