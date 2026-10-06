/*
 *			GPAC - Multimedia Framework C SDK
 *
 *  This file is part of GPAC / AV2 video decoder filter
 *  based on dav2d (https://code.videolan.org/videolan/dav2d)
 *
 *  Whole-file filter on the AV2 low-overhead bitstream format (".obu"):
 *  GPAC has no GF_CODECID for AV2 and no demuxer produces an AV2 pid, so
 *  there is no framed pid to sit behind - this filter takes the elementary
 *  stream itself and splits it into temporal units.
 *
 *  The output pid is deliberately NOT created in configure_pid: a whole-file
 *  filter knows nothing about the picture before it has read the data, and a
 *  raw video pid declared without width, height and pixel format gives the
 *  resolver no route to an encoder (it links straight to the muxer and writes
 *  uncompressed video instead). The pid is declared from process(), once the
 *  sequence header has been parsed and the geometry is known for real.
 */

#include <gpac/filters.h>
#include <gpac/constants.h>
#include <string.h>

#include <dav2d/dav2d.h>

/*dav2d sizes its worker stacks through pthread_attr_setstacksize. With
n_threads = 1 no thread is ever created, but the call site survives the link
and the symbol is exported by neither solver - and a side module with an
unresolved import does not instantiate at all, so the filter would simply never
register. Defined here rather than added to the solver export list, which would
mean rebuilding the solver for a call that never runs.*/
int pthread_attr_setstacksize(void *attr, size_t stacksize)
{
	(void)attr;
	(void)stacksize;
	return 0;
}

/*no timing in a raw OBU stream. 25 fps is what dav2d's own CLI assumes when it
writes y4m from a section-5 file, and the page has no way to pass a filter
option (so this default is the only value this filter can ever be used at).*/
#define AV2DEC_FPS			25
#define AV2DEC_TIMESCALE	1000

typedef struct
{
	GF_FilterPid *ipid, *opid;

	Dav2dContext *decoder;

	u32 width, height, pix_fmt;
	u64 next_cts;
	Bool is_playing;
} GF_AV2DecCtx;

/*reads a leb128 as used by the low-overhead bitstream format. Returns the
number of bytes consumed, or 0 if the value does not fit in what is left.*/
static u32 av2dec_leb128(const u8 *data, u32 size, u64 *value)
{
	u32 i;
	*value = 0;
	for (i = 0; i < 8; i++)
	{
		if (i >= size) return 0;
		*value |= ((u64)(data[i] & 0x7f)) << (i * 7);
		if (!(data[i] & 0x80)) return i + 1;
	}
	return 0;
}

/*returns the size in bytes of the temporal unit starting at data, i.e. the run
of OBUs up to (but excluding) the next temporal delimiter - the split dav2d
expects on its input, and the one tools/input/section5.c performs.*/
static u32 av2dec_temporal_unit_size(const u8 *data, u32 size)
{
	u32 pos = 0;
	Bool first = GF_TRUE;

	while (pos < size)
	{
		u64 obu_size;
		u32 len_bytes, obu_type;

		len_bytes = av2dec_leb128(data + pos, size - pos, &obu_size);
		if (!len_bytes) break;
		if (pos + len_bytes + obu_size > size) break;
		if (!obu_size) break;

		obu_type = (data[pos + len_bytes] >> 2) & 0x1f;

		if (first)
		{
			/*a temporal unit must open on a temporal delimiter; if it does not
			the stream is not in the expected format and there is nothing
			sensible to cut on*/
			if (obu_type != DAV2D_OBU_TD) return 0;
			first = GF_FALSE;
		}
		else if (obu_type == DAV2D_OBU_TD)
		{
			break;
		}
		pos += len_bytes + (u32)obu_size;
	}
	return pos;
}

static u32 av2dec_get_pixel_format(int layout, int bpc)
{
	Bool is_10b = (bpc > 8) ? GF_TRUE : GF_FALSE;
	switch (layout)
	{
	case DAV2D_PIXEL_LAYOUT_I420:
		return is_10b ? GF_PIXEL_YUV_10 : GF_PIXEL_YUV;
	case DAV2D_PIXEL_LAYOUT_I422:
		return is_10b ? GF_PIXEL_YUV422_10 : GF_PIXEL_YUV422;
	case DAV2D_PIXEL_LAYOUT_I444:
		return is_10b ? GF_PIXEL_YUV444_10 : GF_PIXEL_YUV444;
	default:
		/*I400 is monochrome: a greyscale pid has no adaptation path in this
		build, so it is refused rather than sent down a chain that cannot
		carry it*/
		return 0;
	}
}

static GF_Err av2dec_configure_pid(GF_Filter *filter, GF_FilterPid *pid, Bool is_remove)
{
	GF_AV2DecCtx *ctx = (GF_AV2DecCtx *)gf_filter_get_udta(filter);

	if (is_remove)
	{
		if (ctx->opid)
		{
			gf_filter_pid_remove(ctx->opid);
			ctx->opid = NULL;
		}
		ctx->ipid = NULL;
		return GF_OK;
	}
	if (!gf_filter_pid_check_caps(pid))
		return GF_NOT_SUPPORTED;

	ctx->ipid = pid;
	/*the output pid is declared from process(), see the note at the top*/
	return GF_OK;
}

static Bool av2dec_process_event(GF_Filter *filter, const GF_FilterEvent *evt)
{
	GF_AV2DecCtx *ctx = (GF_AV2DecCtx *)gf_filter_get_udta(filter);
	switch (evt->base.type)
	{
	case GF_FEVT_PLAY:
		ctx->is_playing = GF_TRUE;
		return GF_FALSE;
	case GF_FEVT_STOP:
		ctx->is_playing = GF_FALSE;
		return GF_FALSE;
	default:
		return GF_FALSE;
	}
}

/*declares the output pid with the real geometry, or updates it if the sequence
changes mid-stream*/
static GF_Err av2dec_declare_output(GF_AV2DecCtx *ctx, GF_Filter *filter, const Dav2dPicture *pic)
{
	u32 pix_fmt = av2dec_get_pixel_format(pic->p.layout, pic->p.bpc);
	u32 stride;

	if (!pix_fmt)
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[AV2Dec] Unsupported output format (layout %d, %d bpc)\n", pic->p.layout, pic->p.bpc));
		return GF_NOT_SUPPORTED;
	}
	stride = pic->p.w * ((pic->p.bpc > 8) ? 2 : 1);

	if (!ctx->opid)
	{
		ctx->opid = gf_filter_pid_new(filter);
		if (!ctx->opid) return GF_OUT_OF_MEM;
		gf_filter_pid_copy_properties(ctx->opid, ctx->ipid);
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_STREAM_TYPE, &PROP_UINT(GF_STREAM_VISUAL));
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CODECID, &PROP_UINT(GF_CODECID_RAW));
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_TIMESCALE, &PROP_UINT(AV2DEC_TIMESCALE));
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_FPS, &PROP_FRAC_INT(AV2DEC_FPS, 1));
	}

	if ((ctx->width != (u32)pic->p.w) || (ctx->height != (u32)pic->p.h) || (ctx->pix_fmt != pix_fmt))
	{
		ctx->width = pic->p.w;
		ctx->height = pic->p.h;
		ctx->pix_fmt = pix_fmt;
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_WIDTH, &PROP_UINT(ctx->width));
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_HEIGHT, &PROP_UINT(ctx->height));
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_STRIDE, &PROP_UINT(stride));
		gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_PIXFMT, &PROP_UINT(pix_fmt));
	}
	return GF_OK;
}

static void av2dec_send_picture(GF_AV2DecCtx *ctx, const Dav2dPicture *pic)
{
	GF_FilterPacket *dst_pck;
	u8 *output;
	u32 i, y, out_size, nb_planes;
	u32 bytes_per_sample = (pic->p.bpc > 8) ? 2 : 1;
	u32 cw, ch;

	switch (pic->p.layout)
	{
	case DAV2D_PIXEL_LAYOUT_I420: cw = (pic->p.w + 1) / 2; ch = (pic->p.h + 1) / 2; nb_planes = 3; break;
	case DAV2D_PIXEL_LAYOUT_I422: cw = (pic->p.w + 1) / 2; ch = pic->p.h;           nb_planes = 3; break;
	case DAV2D_PIXEL_LAYOUT_I444: cw = pic->p.w;           ch = pic->p.h;           nb_planes = 3; break;
	default: return;
	}

	out_size = (pic->p.w * pic->p.h + 2 * cw * ch) * bytes_per_sample;

	dst_pck = gf_filter_pck_new_alloc(ctx->opid, out_size, &output);
	if (!dst_pck) return;

	/*gpac expects the planes packed one after the other with no padding, so
	they are copied row by row, dropping dav2d's own stride*/
	for (i = 0; i < nb_planes; i++)
	{
		u32 pw = i ? cw : (u32)pic->p.w;
		u32 ph = i ? ch : (u32)pic->p.h;
		u32 row_size = pw * bytes_per_sample;
		ptrdiff_t stride = pic->stride[i ? 1 : 0];
		const u8 *plane = (const u8 *)pic->data[i];

		for (y = 0; y < ph; y++)
		{
			memcpy(output, plane + (size_t)y * stride, row_size);
			output += row_size;
		}
	}

	gf_filter_pck_set_cts(dst_pck, ctx->next_cts);
	gf_filter_pck_set_duration(dst_pck, AV2DEC_TIMESCALE / AV2DEC_FPS);
	gf_filter_pck_set_sap(dst_pck, GF_FILTER_SAP_1);
	gf_filter_pck_send(dst_pck);

	ctx->next_cts += AV2DEC_TIMESCALE / AV2DEC_FPS;
}

/*drains every picture dav2d is holding. DAV2D_ERR(EAGAIN) means "no picture
yet", anything else negative is a real failure.*/
static GF_Err av2dec_drain(GF_AV2DecCtx *ctx, GF_Filter *filter)
{
	for (;;)
	{
		Dav2dPicture pic;
		int res;

		memset(&pic, 0, sizeof(pic));
		res = dav2d_get_picture(ctx->decoder, &pic);
		if (res == DAV2D_ERR(EAGAIN)) return GF_OK;
		if (res < 0)
		{
			GF_LOG(GF_LOG_WARNING, GF_LOG_CODEC, ("[AV2Dec] dav2d_get_picture failed (%d)\n", res));
			return GF_OK;
		}

		if (av2dec_declare_output(ctx, filter, &pic) == GF_OK)
			av2dec_send_picture(ctx, &pic);

		dav2d_picture_unref(&pic);
	}
}

static GF_Err av2dec_process(GF_Filter *filter)
{
	GF_FilterPacket *pck;
	const u8 *data;
	u32 size, pos;
	GF_AV2DecCtx *ctx = (GF_AV2DecCtx *)gf_filter_get_udta(filter);

	pck = gf_filter_pid_get_packet(ctx->ipid);
	if (!pck)
	{
		if (gf_filter_pid_is_eos(ctx->ipid))
		{
			if (ctx->opid) gf_filter_pid_set_eos(ctx->opid);
			return GF_EOS;
		}
		return GF_OK;
	}

	data = gf_filter_pck_get_data(pck, &size);
	if (!data)
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_IO_ERR;
	}

	/*whole-file input: the elementary stream arrives as one packet, which is
	walked here one temporal unit at a time*/
	pos = 0;
	while (pos < size)
	{
		Dav2dData in;
		u32 tu_size = av2dec_temporal_unit_size(data + pos, size - pos);
		u8 *buf;
		int res;

		if (!tu_size)
		{
			GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[AV2Dec] Not an AV2 low-overhead bitstream (no temporal delimiter at offset %u)\n", pos));
			break;
		}

		memset(&in, 0, sizeof(in));
		buf = dav2d_data_create(&in, tu_size);
		if (!buf)
		{
			gf_filter_pid_drop_packet(ctx->ipid);
			return GF_OUT_OF_MEM;
		}
		memcpy(buf, data + pos, tu_size);
		pos += tu_size;

		/*dav2d_send_data consumes the buffer progressively: it may take only
		part of it and ask to be called again once pictures have been pulled
		out*/
		while (in.sz)
		{
			res = dav2d_send_data(ctx->decoder, &in);
			if (res < 0 && res != DAV2D_ERR(EAGAIN))
			{
				GF_LOG(GF_LOG_WARNING, GF_LOG_CODEC, ("[AV2Dec] dav2d_send_data failed (%d)\n", res));
				dav2d_data_unref(&in);
				break;
			}
			av2dec_drain(ctx, filter);
		}
		av2dec_drain(ctx, filter);
	}

	/*no more input is coming, so pull out the pictures still held for reorder*/
	dav2d_flush(ctx->decoder);
	av2dec_drain(ctx, filter);

	gf_filter_pid_drop_packet(ctx->ipid);

	if (ctx->opid) gf_filter_pid_set_eos(ctx->opid);
	return GF_EOS;
}

static GF_Err av2dec_initialize(GF_Filter *filter)
{
	GF_AV2DecCtx *ctx = (GF_AV2DecCtx *)gf_filter_get_udta(filter);
	Dav2dSettings s;
	int res;

	dav2d_default_settings(&s);
	/*side modules are single-threaded: dav2d only spawns workers when
	n_threads resolves to more than one, so pinning it to 1 keeps the whole
	decode in the calling thread. Left at its default (0 = auto) it would size
	the pool from the CPU count and queue work no one ever runs.*/
	s.n_threads = 1;
	s.max_frame_delay = 1;

	res = dav2d_open(&ctx->decoder, &s);
	if (res < 0)
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[AV2Dec] dav2d_open failed (%d)\n", res));
		return GF_NOT_SUPPORTED;
	}
	return GF_OK;
}

static void av2dec_finalize(GF_Filter *filter)
{
	GF_AV2DecCtx *ctx = (GF_AV2DecCtx *)gf_filter_get_udta(filter);
	if (ctx->decoder)
	{
		dav2d_close(&ctx->decoder);
		ctx->decoder = NULL;
	}
}

static const GF_FilterCapability AV2DecCaps[] =
	{
		CAP_UINT(GF_CAPS_INPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_FILE),
		CAP_STRING(GF_CAPS_INPUT, GF_PROP_PID_FILE_EXT, "obu|av2"),
		CAP_STRING(GF_CAPS_INPUT, GF_PROP_PID_MIME, "video/AV2|video/x-av2|video/obu"),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_VISUAL),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_CODECID, GF_CODECID_RAW),
};

GF_FilterRegister AV2DecoderRegister = {
	.name = "av2dec",
	GF_FS_SET_DESCRIPTION("AV2 video decoder")
		GF_FS_SET_HELP("This filter decodes AV2 video elementary streams (low-overhead bitstream format, .obu) using dav2d.")
			.private_size = sizeof(GF_AV2DecCtx),
	SETCAPS(AV2DecCaps),
	.initialize = av2dec_initialize,
	.configure_pid = av2dec_configure_pid,
	.process = av2dec_process,
	.process_event = av2dec_process_event,
	.finalize = av2dec_finalize,
};

const GF_FilterRegister * EMSCRIPTEN_KEEPALIVE av2dec_register(GF_FilterSession *session)
{
	return &AV2DecoderRegister;
}

/*Bevara: side modules register their own filters at load time.*/
#include "filter_register.h"
__attribute__((constructor))
void register_av2dec(void) {
    gf_filter_auto_register("av2dec", av2dec_register);
}
