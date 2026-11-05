#include "vdec_rk.hpp"
#include <atomic>
#include <algorithm>
#include "../utils/ltrace.hpp"

#include <assert.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <sys/ioctl.h>

#include "common.hpp"
#include "frame_mode.hpp"
#include "utils/scheduling_helper.hpp"
#include "utils/time_util.h"

// Which set of decoder buffers prime_fds refer to (DecodedUnit::buf_epoch).
static std::atomic<uint32_t> s_buf_epoch{1};


// void VdecRK::init_buffer(MppFrame frame) {
// 	RK_U32 width = mpp_frame_get_width(frame);
//     RK_U32 height = mpp_frame_get_height(frame);
//     RK_U32 stride = mpp_frame_get_hor_stride(frame);
//     MppFrameFormat fmt = mpp_frame_get_fmt(frame);

//     assert(fmt == MPP_FMT_YUV420SP);

//     int ret = mpp_buffer_group_get_external(&mpi.frm_grp, MPP_BUFFER_TYPE_DRM);
//     assert(ret == 0);

//     for (int i = 0; i < MAX_FRAMES; i++) {
//         DrmBuffer out;
//         if (!renderer->create_drm_nv12_buffer(stride, height, out, true)) {
// 			printf("create_drm_nv12_buffer failed\n");
//             exit(1);
//         }

//         mpi.frame_to_drm[i].handle = out.handle;
//         mpi.frame_to_drm[i].fb_id = out.fb_id;

//         MppBufferInfo info = {};
//         info.type = MPP_BUFFER_TYPE_DRM;
//         info.size = stride * height * 3 / 2;
//         info.fd = out.prime_fd;
//         ret = mpp_buffer_commit(mpi.frm_grp, &info);
//         assert(ret == 0);
//         mpi.frame_to_drm[i].prime_fd = info.fd;
//         if (info.fd != out.prime_fd) close(out.prime_fd);
//     }

//     mpi.mpi->control(mpi.ctx, MPP_DEC_SET_EXT_BUF_GROUP, mpi.frm_grp);
//     mpi.mpi->control(mpi.ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
// 	printf("init_buffer done\n");
// }

// The picture buffers init_buffer made, given back: on a resolution change,
// and when the decoder goes. Order: framebuffers, then MPP's group (it
// closes the prime_fds it holds), then the dumb buffers. A picture still held
// elsewhere (an imported texture, a DecodedUnit) keeps its memory alive.
void VdecRK::free_frame_buffers() {
	if (!mpi.frm_grp) return;
	const int fd = dev ? dev->drm_fd : -1;
	// 1. Remove DRM framebuffers (kernel ref-counts, safe even if still displayed)
	for (int i = 0; i < MAX_FRAMES; i++) {
		if (mpi.frame_to_drm[i].fb_id) {
			if (fd >= 0) drmModeRmFB(fd, mpi.frame_to_drm[i].fb_id);
			mpi.frame_to_drm[i].fb_id = 0;
		}
	}
	// 2. Release MPP buffer group (closes prime_fds owned by MPP). Cleared
	// first, as MPP's own decoder test does on an info change: a buffer
	// something still references is marked discard, so it is freed when
	// that lets go. Put back with one still in use, MPP 1.0.3 would
	// otherwise keep it - and the orphaned group - for good.
	mpp_buffer_group_clear(mpi.frm_grp);
	mpp_buffer_group_put(mpi.frm_grp);
	mpi.frm_grp = NULL;
	// 3. Destroy underlying GEM dumb buffers
	for (int i = 0; i < MAX_FRAMES; i++) {
		if (mpi.frame_to_drm[i].handle) {
			if (fd >= 0) {
				struct drm_mode_destroy_dumb dmd = {};
				dmd.handle = mpi.frame_to_drm[i].handle;
				ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmd);
			}
			mpi.frame_to_drm[i].handle = 0;
		}
		mpi.frame_to_drm[i].prime_fd = -1;
	}
}

// Everything the constructor and init_buffer took. Before this, a decoder
// that went (each recording played in the gallery has its own) kept its MPP
// context and its 16 picture buffers - about 65 MB at 1080p - for good.
VdecRK::~VdecRK() {
	{
		std::lock_guard<std::mutex> lock(early_mutex_);
		early_stop_ = true;
	}
	early_cv_.notify_all();
	if (early_thread_.joinable()) early_thread_.join();
	if (mpi.ctx) {
		mpi.mpi->reset(mpi.ctx);
		mpp_destroy(mpi.ctx);   // its threads stop: nothing writes the buffers after this
		mpi.ctx = NULL;
	}
	free_frame_buffers();
	if (packet) {
		void *data = mpp_packet_get_data(packet);
		mpp_packet_deinit(&packet);
		free(data);
	}
	for (int i = 0; i < NUM_SLICES_BUFFERS; i++) {
		if (slices_buffers[i]) {
			free(slices_buffers[i]);
		}
	}
}

void VdecRK::init_buffer(MppFrame frame) {
	uint64_t video_frm_width = mpp_frame_get_width(frame);
	uint64_t video_frm_height = mpp_frame_get_height(frame);
	RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
	RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
	MppFrameFormat fmt = mpp_frame_get_fmt(frame);
	assert((fmt == MPP_FMT_YUV420SP) || (fmt == MPP_FMT_YUV420SP_10BIT));

	printf("Frame info changed %lu(%d)x%lu(%d)\n", video_frm_width, hor_stride, video_frm_height, ver_stride);

	// On resolution change, free all old DRM/MPP resources before reallocating.
	// Order: flush renderer cache → remove FBs → put MPP group (closes prime fds) → destroy dumb handles.
	if (mpi.frm_grp) {
		printf("VdecRK: resolution change — flushing renderer and freeing old DRM resources\n");
		// 0. Ask renderer to drain its frame queue and clear its display_buffers cache.
		//    This prevents false prime_fd matches after fd numbers are reused by new allocations.
		if (renderer) renderer->request_flush();
		// The screen recorder's pictures too: a buffer still referenced when
		// the group is put back below is never freed by MPP 1.0.3.
		if (dev) dev->drop_screen_pictures();
		// The prime_fds' numbers may come back for the new buffers: a new epoch.
		s_buf_epoch++;
		free_frame_buffers();
	}

	// output_list->video_fb_x = 0;
	// output_list->video_fb_y = 0;
	// output_list->video_fb_width = output_list->mode.hdisplay;
	// output_list->video_fb_height =output_list->mode.vdisplay;

	// osd_vars.video_width = output_list->video_frm_width;
	// osd_vars.video_height = output_list->video_frm_height;

	// create new external frame group and allocate (commit flow) new DRM buffers and DRM FB
	int ret = mpp_buffer_group_get_external(&mpi.frm_grp, MPP_BUFFER_TYPE_DRM);
	assert(!ret);

	for (int i=0; i<MAX_FRAMES; i++) {
		
		// new DRM buffer
		struct drm_mode_create_dumb dmcd;
		memset(&dmcd, 0, sizeof(dmcd));
		dmcd.bpp = fmt==MPP_FMT_YUV420SP?8:10;
		dmcd.width = hor_stride;
		dmcd.height = ver_stride*2; // documentation say not v*2/3 but v*2 (additional info included)
		do {
			ret = ioctl(dev->drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &dmcd);
		} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
		assert(!ret);
		// assert(dmcd.pitch==(fmt==MPP_FMT_YUV420SP?hor_stride:hor_stride*10/8));
		// assert(dmcd.size==(fmt == MPP_FMT_YUV420SP?hor_stride:hor_stride*10/8)*ver_stride*2);
		mpi.frame_to_drm[i].handle = dmcd.handle;
		
		// commit DRM buffer to frame group
		struct drm_prime_handle dph;
		memset(&dph, 0, sizeof(struct drm_prime_handle));
		dph.handle = dmcd.handle;
		dph.fd = -1;
		do {
			ret = ioctl(dev->drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &dph);
		} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
		assert(!ret);
		MppBufferInfo info;
		memset(&info, 0, sizeof(info));
		info.type = MPP_BUFFER_TYPE_DRM;
		info.size = dmcd.width*dmcd.height;
		info.fd = dph.fd;
		ret = mpp_buffer_commit(mpi.frm_grp, &info);
		assert(!ret);
		mpi.frame_to_drm[i].prime_fd = info.fd; // dups fd						
		if (dph.fd != info.fd) {
			ret = close(dph.fd);
			assert(!ret);
		}

		// allocate DRM FB from DRM buffer
		uint32_t handles[4], pitches[4], offsets[4];
		memset(handles, 0, sizeof(handles));
		memset(pitches, 0, sizeof(pitches));
		memset(offsets, 0, sizeof(offsets));
		handles[0] = mpi.frame_to_drm[i].handle;
		offsets[0] = 0;
		pitches[0] = hor_stride;						
		handles[1] = mpi.frame_to_drm[i].handle;
		offsets[1] = pitches[0] * ver_stride;
		pitches[1] = pitches[0];
        
        // Debug print
        // printf("VdecRK: Creating FB %d/%d for handle %u\n", i, MAX_FRAMES, handles[0]);
		ret = drmModeAddFB2(dev->drm_fd, video_frm_width, video_frm_height, DRM_FORMAT_NV12, handles, pitches, offsets, &mpi.frame_to_drm[i].fb_id, 0);
        if (ret != 0) {
            printf("VdecRK: drmModeAddFB2 failed: %d (errno=%d)\n", ret, errno);
        }
		assert(!ret);
    }
	// printf("VdecRK: init_buffer loop complete\n");

	// register external frame group
	ret = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_EXT_BUF_GROUP, mpi.frm_grp);
    // if (ret) printf("VdecRK: MPP_DEC_SET_EXT_BUF_GROUP failed ret=%d\n", ret);
    // printf("VdecRK: Set Ext Buf Group Done\n");
    
	ret = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
     // if (ret) printf("VdecRK: MPP_DEC_SET_INFO_CHANGE_READY failed ret=%d\n", ret);
    // printf("VdecRK: Set Info Change Ready Done\n");

	// ret = modeset_perform_modeset(dev->drm_fd, output_list, output_list->video_request, &output_list->video_plane, mpi.frame_to_drm[0].fb_id, osd_vars.video_width, osd_vars.video_height, video_zpos);
	// assert(ret >= 0);
    // printf("VdecRK: init_buffer returning\n");
}


static int consec_put_stalls = 0;   // consecutive 1s put-packet stalls (decoder wedge detector)

void VdecRK::run_frame()
{
	printf("VdecRK run_frame thread starting...\n" );
    SchedulingHelper::configure_thread(SchedulingHelper::ThreadRole::VDec);
	int i, ret;
	MppFrame  frame  = NULL;
	uint64_t last_frame_time;

	while (!*should_stop) { // TODO frm_eos for proper closing.
		struct timespec ts, ats;
		assert(!frame);
		ret = mpi.mpi->decode_get_frame(mpi.ctx, &frame);
		if (ret != 0) {
			static int err_count = 0;
			if (err_count++ % 100 == 0) {
				printf("VdecRK: decode_get_frame returned error %d\n", ret);
			}
		}
		clock_gettime(CLOCK_MONOTONIC, &ats);
		if (frame) {
			first_successful_decode = true;
			last_decoded_frame_time_ms = get_time_ms();
			fed_since_out_.store(0, std::memory_order_relaxed);
			uint64_t pts =  mpp_frame_get_pts(frame);
			static int frame_count = 0;
            // if (frame_count++ % 30 == 0) {
            //     printf("VdecRK: Got decoded frame from MPP! PTS=%lu, size=%dx%d\n", pts, mpp_frame_get_width(frame), mpp_frame_get_height(frame));
            // }
            uint32_t frm_width = mpp_frame_get_width(frame);
            uint32_t frm_height = mpp_frame_get_height(frame);
            MppFrameFormat fmt = mpp_frame_get_fmt(frame);
			if (mpp_frame_get_info_change(frame)) {
                // printf("VdecRK: Calling init_buffer\n");
                init_buffer(frame);
                // printf("VdecRK: Returned from init_buffer\n");
			} else {
                // ... (omitted for brevity, assume existing else block)
				// regular frame received
				MppBuffer buffer = mpp_frame_get_buffer(frame);					
				if (buffer) {
					MppBufferInfo info;
					ret = mpp_buffer_info_get(buffer, &info);
					assert(!ret);
					for (i=0; i<MAX_FRAMES; i++) {
						if (mpi.frame_to_drm[i].prime_fd == info.fd) break;
					}
					assert(i!=MAX_FRAMES);

                    uint64_t now_us = get_time_us();
                    if (ltrace::on()) {
                        // Mean luma of an 8x4 grid, sparsely sampled: enough to
                        // see a light switched on in front of the camera.
                        const uint8_t* y = (const uint8_t*)mpp_buffer_get_ptr(buffer);
                        const uint32_t stride = mpp_frame_get_hor_stride(frame);
                        uint8_t grid[32];
                        for (int c = 0; c < 32 && y; c++) {
                            const uint32_t x0 = (c % 8) * frm_width / 8, y0 = (c / 8) * frm_height / 4;
                            uint32_t sum = 0;
                            for (int j = 0; j < 8; j++)
                                for (int i = 0; i < 8; i++)
                                    sum += y[(size_t)(y0 + (2 * j + 1) * frm_height / 64) * stride +
                                             x0 + (2 * i + 1) * frm_width / 128];
                            grid[c] = (uint8_t)(sum / 64);
                        }
                        ltrace::rec(ltrace::kDecOut, now_us, (uint32_t)pts, 0, grid, y ? 32 : 0);
                    }
                    // What comes out of the decoder, every 5 s: pictures, how many
                    // MPP flagged, and keyframes among them.
                    {
                        static uint64_t win_start = 0;
                        static unsigned n_out = 0, n_err = 0, n_key = 0;
                        n_out++;
                        if (mpp_frame_get_errinfo(frame) || mpp_frame_get_discard(frame)) n_err++;
                        if (!win_start) win_start = now_us;
                        if (now_us - win_start >= 5000000) {
                            printf("VdecRK: %u pictures out in 5 s, %u flagged with errors, %u keyframes%s, "
                                   "%u early tops, %u refused as stale\n",
                                   n_out, n_err, n_key, stream_ok_ ? " (stream mode available)" : "",
                                   early_tops_.exchange(0), early_stale_.exchange(0));
                            win_start = now_us; n_out = n_err = n_key = 0;
                        }
                        std::lock_guard<std::mutex> lock(decoding_stats_mutex);
                        auto it = decoding_stats.find(pts);
                        if (it != decoding_stats.end() && it->second.is_keyframe) n_key++;
                    }
                    out_w_ = frm_width;
                    out_h_ = frm_height;
                    out_hs_ = mpp_frame_get_hor_stride(frame);
                    out_vs_ = mpp_frame_get_ver_stride(frame);
                    if (early_ok_) strm_picture_out((int64_t)pts, now_us);
                    bool found_stats = false;
                    timing_stats_t t_stats = {};
                    {
                        std::lock_guard<std::mutex> lock(decoding_stats_mutex);
                        auto it = decoding_stats.find(pts);
                        if (it != decoding_stats.end()) {
                            found_stats = true;
                            t_stats = it->second;
                            decoding_stats.erase(it);
                        }
                    }

                    if (found_stats) {
                        auto du_stat = std::make_shared<DecodedUnit>();
                        du_stat->pts = pts;
                        du_stat->width = frm_width;
                        du_stat->height = frm_height;
                        du_stat->recv_ts = t_stats.recv_start_us;
                        du_stat->dec_start_ts = t_stats.decode_start_us;
                        du_stat->dec_end_ts = now_us;
                        du_stat->tx_encode_delay_us = t_stats.tx_encode_delay_us;
                        du_stat->tx_processing_delay_us = t_stats.tx_processing_delay_us;
                        du_stat->is_keyframe = t_stats.is_keyframe;
                        // Its slices' rows are all decoded now - those not
                        // already marked (a streamed picture's top half).
                        if (t_stats.slices) {
                            SliceTimes &st = *t_stats.slices;
                            const int n = std::min(st.n.load(), SliceTimes::kMax);
                            for (int i = 0; i < n; i++)
                                if (!st.s[i].done_us.load()) st.s[i].done_us.store(now_us);
                        }
                        du_stat->slices = t_stats.slices;
                        
                        du_stat->has_prime_fd = true;
                        du_stat->prime_fd = info.fd;
                        du_stat->drm_pixel_format = DRM_FORMAT_NV12;
                        
                        // Pass layout info for NV12
                        du_stat->pitches[0] = mpp_frame_get_hor_stride(frame);
                        du_stat->offsets[0] = 0;
                        du_stat->pitches[1] = du_stat->pitches[0];
                        du_stat->offsets[1] = du_stat->pitches[0] * mpp_frame_get_ver_stride(frame);

                        emit_decoded(du_stat, buffer);

                    } else {
                        // This shouldn't happen if MPP preserves PTS
                        static uint32_t fail_count = 0;
                        if (fail_count++ % 100 == 0) printf("VdecRK: Warning! Received PTS %lu not found in tracking map\n", pts);
                        
                        auto du_fallback = std::make_shared<DecodedUnit>();
                        du_fallback->pts = pts;
                        du_fallback->width = frm_width;
                        du_fallback->height = frm_height;
                        du_fallback->has_prime_fd = true;
                        du_fallback->prime_fd = info.fd;
                        du_fallback->drm_pixel_format = DRM_FORMAT_NV12;
                        
                        du_fallback->pitches[0] = mpp_frame_get_hor_stride(frame);
                        du_fallback->offsets[0] = 0;
                        du_fallback->pitches[1] = du_fallback->pitches[0];
                        du_fallback->offsets[1] = du_fallback->pitches[0] * mpp_frame_get_ver_stride(frame);

                        emit_decoded(du_fallback, buffer);
                    }
				}
			}
            
			frm_eos = mpp_frame_get_eos(frame);
            // printf("VdecRK: Deinitializing frame %p\n", frame);
			mpp_frame_deinit(&frame);
            // printf("VdecRK: Frame deinitialized\n");
			frame = NULL;
		};
	}
	printf("VdecRK run_frame thread done.\n");
}

// Deliver a decoded NV12 frame either to a registered sink (background video,
// GL-texture path) or to the renderer's video plane (live feed). For the sink
// path the underlying MPP buffer is ref-held via du->frame_ref so it stays
// valid until the consumer finishes its (deferred) EGL import.
static std::shared_ptr<void> hold_mpp_buffer(void* b) {
    mpp_buffer_inc_ref((MppBuffer)b);
    return std::shared_ptr<void>(b, [](void* p) { mpp_buffer_put((MppBuffer)p); });
}

void VdecRK::emit_decoded(std::shared_ptr<DecodedUnit> du, MppBuffer buffer) {
    du->buf = buffer;
    du->hold = hold_mpp_buffer;
    du->buf_epoch = s_buf_epoch.load(std::memory_order_relaxed);
    if (frame_sink_) {
        mpp_buffer_inc_ref(buffer);
        du->frame_ref = std::shared_ptr<void>(buffer, [](void* b) {
            mpp_buffer_put((MppBuffer)b);
        });
        frame_sink_(du);
    } else {
        // While the screen is recorded, the picture is held from here, while
        // MPP still has it: the recorder reads it after the display has
        // (screen_tap.hpp), and the hold - not MPP's least-recently-used
        // reuse - is what keeps the decoder from writing into it meanwhile.
        if (dev && dev->tap.recording()) du->frame_ref = hold_mpp_buffer(buffer);
        renderer->queue_frame(du);
    }
}

void VdecRK::feed_packet_to_decoder(void* data_p, int data_len, int64_t pts, uint64_t recv_ts, uint8_t nal_type, uint32_t encode_delay_us, uint32_t processing_delay_us, bool is_key_override){
    if (data_len <= 0) return;

    // Cache H.265 VPS (32), SPS (33), PPS (34) or H.264 SPS (7), PPS (8)
    if (nal_type == 32) {
        cached_vps.assign((uint8_t*)data_p, (uint8_t*)data_p + data_len);
        return;
    }
    if (nal_type == 33 || nal_type == 7) {
        cached_sps.assign((uint8_t*)data_p, (uint8_t*)data_p + data_len);
        return;
    }
    if (nal_type == 34 || nal_type == 8) {
        cached_pps.assign((uint8_t*)data_p, (uint8_t*)data_p + data_len);
        return;
    }
    // Skip SEI entirely
    if (nal_type == 39 || nal_type == 40 || nal_type == 6) {
        return;
    }

    if (data_len > READ_BUF_SIZE) {
        printf("VdecRK: packet/slice too large %d\n", data_len);
        return;
    }

    static int feed_count = 0;
    feed_count++;

    // Copy to a circular buffer to keep the data pointer valid for asynchronous parsing/decoding
    uint8_t* dest_buf = slices_buffers[current_slice_buffer_idx];
    current_slice_buffer_idx = (current_slice_buffer_idx + 1) % NUM_SLICES_BUFFERS;

    int final_len = 0;
    bool is_key = is_key_override || (nal_type >= 19 && nal_type <= 21) || nal_type == 5;

    if (is_key) {
        size_t vps_sz = cached_vps.size();
        size_t sps_sz = cached_sps.size();
        size_t pps_sz = cached_pps.size();

        if (vps_sz + sps_sz + pps_sz + data_len > READ_BUF_SIZE) {
            printf("VdecRK: combined packet too large %zu\n", vps_sz + sps_sz + pps_sz + data_len);
            return;
        }

        uint8_t* p = dest_buf;
        if (vps_sz > 0) {
            memcpy(p, cached_vps.data(), vps_sz);
            p += vps_sz;
        }
        if (sps_sz > 0) {
            memcpy(p, cached_sps.data(), sps_sz);
            p += sps_sz;
        }
        if (pps_sz > 0) {
            memcpy(p, cached_pps.data(), pps_sz);
            p += pps_sz;
        }
        memcpy(p, data_p, data_len);
        final_len = vps_sz + sps_sz + pps_sz + data_len;
    } else {
        memcpy(dest_buf, data_p, data_len);
        final_len = data_len;
    }

    // if (feed_count % 100 == 1) {
    //     printf("VdecRK: feed_packet_to_decoder - size=%d (orig=%d), PTS=%ld, NAL=%d, Key=%d (Total feed calls: %d)\n", 
    //            final_len, data_len, pts, nal_type, is_key, feed_count);
    // }

    MppPacket pkt = nullptr;
    int ret = mpp_packet_init(&pkt, dest_buf, final_len);
    if (ret != MPP_OK) {
        printf("VdecRK: mpp_packet_init failed: %d\n", ret);
        return;
    }
    mpp_packet_set_pts(pkt, (RK_S64)pts);
#ifdef MPP_PACKET_FLAG_STREAM_START
    if (next_packet_flags_)
        mpp_packet_set_flag(pkt, next_packet_flags_);
#endif

    // Feed the data to mpp until either timeout (in which case the decoder might have stalled)
    // or success
    uint64_t data_feed_begin = get_time_ms();

    uint64_t now_ms = get_time_ms();
    if (first_successful_decode && (now_ms - last_decoded_frame_time_ms > 2000)) {
        first_successful_decode = false;
        idr_requested = false;
    }

    if (!first_successful_decode) {
        if (now_ms - last_idr_request_ms > 1000) {
            if (idr_request_callback) {
                idr_request_callback();
                last_idr_request_ms = now_ms;
                idr_requested = true;
            }
        }
    } else if (!idr_requested && idr_request_callback) {
       idr_request_callback();
       idr_requested = true;
    }

    {
        std::lock_guard<std::mutex> lock(decoding_stats_mutex);
        decoding_stats[pts] = timing_stats_t{ 
            .recv_start_us = recv_ts,
            .decode_start_us = get_time_us(), 
            .tx_encode_delay_us = encode_delay_us,
            .tx_processing_delay_us = processing_delay_us,
            // is_key, not is_key_override: the override is only a hint from the
            // caller. Recording the raw override here left is_keyframe false for
            // every source that relies on NAL-type detection (the AR8030 path
            // passes no override), so Renderer::render_frame never told the OSD
            // a keyframe had landed, the OSD stayed in BACKGROUND state, and the
            // fps/resolution/bitrate labels were never drawn.
            .is_keyframe = is_key,
            .slices = std::move(next_slice_times_),
        };
        next_slice_times_.reset();
        // MPP drops the pictures it cannot decode, and their entries were
        // never collected: over a broken stream this grew by every picture
        // sent. No picture spends 128 others inside the decoder.
        if (decoding_stats.size() > 256) {
            for (auto it = decoding_stats.begin(); it != decoding_stats.end();)
                it = (it->first < pts - 128) ? decoding_stats.erase(it) : std::next(it);
        }
    }
    
    const uint64_t put_start_us = get_time_us();
    {
        // What has arrived of the picture goes to the decoder now: the
        // whole picture, or a streamed picture's first slice.
        std::lock_guard<std::mutex> lock(decoding_stats_mutex);
        auto it = decoding_stats.find(pts);
        if (it != decoding_stats.end()) slices_to_decoder(it->second.slices.get(), put_start_us);
    }
    int attempt = 0;
    while (MPP_OK != (ret = mpi.mpi->decode_put_packet(mpi.ctx, pkt))) {
        attempt++;
        uint64_t elapsed = get_time_ms() - data_feed_begin;
        if (attempt % 500 == 1) {
            printf("VdecRK: decode_put_packet returned %d, elapsed=%lu ms, attempt=%d, pts=%ld, len=%d\n", ret, elapsed, attempt, pts, data_len);
        }
        if (elapsed > 1000 || *should_stop) { // Increased from 8ms to 1000ms to avoid deadlock under heavy slice load
            printf("VdecRK: decode_put_packet failed/stalled with ret=%d after %lu ms, attempt=%d\n", ret, elapsed, attempt);
            decoder_stalled_count++;
            mpp_packet_deinit(&pkt);
            // A wedged decoder (output backpressure after e.g. a PTS reset from
            // an air reboot) never recovers by dropping packets — observed
            // stuck in the -1012 loop for tens of thousands of packets. Hard
            // reset MPP after 3 consecutive stalls; the air re-sends
            // params+IDR (link-up burst / gnd retry) so decode resumes on the
            // next keyframe within a second or two.
            if (++consec_put_stalls >= 3) {
                printf("VdecRK: %d consecutive stalls — resetting the MPP decoder\n",
                       consec_put_stalls);
                mpi.mpi->reset(mpi.ctx);
                key_in_.store(false, std::memory_order_relaxed);   // the reset took its parameter sets
                {
                    std::lock_guard<std::mutex> lock(decoding_stats_mutex);
                    decoding_stats.clear();
                }
                consec_put_stalls = 0;
            }
            return;
        }
        // Do NOT sched_yield() here: these threads run SCHED_FIFO, so yielding
        // returns immediately when no equal-prio thread is runnable and the loop
        // busy-spins, burning RT bandwidth (triggers "sched: RT throttling
        // activated") and starving the frame/renderer threads that free the
        // decode buffers -> the -1012 never clears. Sleep briefly instead so the
        // drain threads get the CPU; 200us keeps well under a 60fps frame time.
        usleep(200);
    }
    consec_put_stalls = 0;
    // MPP holds parameter sets only once a keyframe went in with them. With
    // intra refresh nothing sends another unless asked, so a keyframe that
    // never got here - or that MPP could not use, a run of packets with no
    // picture out - leaves every later slice refused ("PPS id out of range")
    // for good (2026-10-22, twice in 12 restarts at 864p120). keyframe_in()
    // says so, and the source asks for a keyframe until one works.
    if (is_key && !cached_pps.empty()) {
        key_in_.store(true, std::memory_order_relaxed);
        fed_since_out_.store(0, std::memory_order_relaxed);
    } else if (fed_since_out_.fetch_add(1, std::memory_order_relaxed) + 1 == kKeyLostPackets &&
               key_in_.exchange(false, std::memory_order_relaxed)) {
        printf("VdecRK: %u packets in and no picture out - the decoder needs a keyframe\n",
               kKeyLostPackets);
    }
    if (ltrace::on()) ltrace::rec(ltrace::kDecPut, put_start_us, (uint32_t)pts, get_time_us());
    
    if (attempt > 1) {
        printf("VdecRK: decode_put_packet succeeded after attempt=%d, elapsed=%lu ms\n", attempt, get_time_ms() - data_feed_begin);
    }
    mpp_packet_deinit(&pkt);
}

// Pictures decoded as their slices arrive: the decoder starts on a picture's
// first slice while the next is still on the air, and finishes it about a
// millisecond after the last one lands instead of a whole decode later. It
// takes fpvOS's MPP (MPP_PACKET_FLAG_STREAM_START, MPP_DEC_SET_STREAM_APPEND)
// on fpvOS's kernel (rkvdec stream mode); built against an MPP without them,
// or run where MPP answers no, every picture goes whole as before.
#ifdef MPP_PACKET_FLAG_STREAM_START
void VdecRK::probe_stream() {
    const char *env = getenv("KESTREL_STREAM_DECODE");
    if (env && atoi(env) == 0) {
        printf("VdecRK: stream decode off (KESTREL_STREAM_DECODE=0)\n");
        return;
    }
    stream_ok_ = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_STREAM_APPEND, nullptr) == MPP_OK;
    printf("VdecRK: %s\n", stream_ok_
           ? "pictures can be decoded as their slices arrive (stream mode)"
           : "no stream mode in this MPP or kernel: pictures go whole");
    if (!stream_ok_) return;

    // Early presentation, on unless KESTREL_EARLY_PRESENT=0; it needs an MPP
    // with MPP_DEC_GET_STREAM_TOP.
#ifdef MPP_STREAM_TOP_READY
    const char *early = getenv("KESTREL_EARLY_PRESENT");
    MppDecStreamTop probe = {};
    probe.pts = -1;
    early_ok_ = !(early && atoi(early) == 0) && renderer &&
                mpi.mpi->control(mpi.ctx, MPP_DEC_GET_STREAM_TOP, &probe) == MPP_OK;
#endif
    printf("VdecRK: early presentation %s\n", early_ok_
           ? "on: a picture's top half may go to the screen before its bottom half"
           : "off");
    if (early_ok_) early_thread_ = std::thread(&VdecRK::early_loop, this);
}

// One streamed picture at a time: wait until its top half is decoded, then
// hand the renderer the picture as it stands, with the estimate of when the
// rest will be decoded.
void VdecRK::early_loop() {
#ifdef MPP_STREAM_TOP_READY
    SchedulingHelper::set_thread_params_max_realtime("VDEC_EARLY", 10);
    for (;;) {
        int64_t pts;
        {
            std::unique_lock<std::mutex> lock(early_mutex_);
            early_cv_.wait(lock, [this] { return early_stop_ || !early_q_.empty(); });
            if (early_stop_) return;
            pts = early_q_.front();
            early_q_.pop_front();
            // behind: only the newest is worth showing early
            if (!early_q_.empty()) continue;
        }
        MppDecStreamTop t = {};
        t.pts = pts;
        t.timeout_ms = 40;
        if (mpi.mpi->control(mpi.ctx, MPP_DEC_GET_STREAM_TOP, &t) != MPP_OK ||
            t.state != MPP_STREAM_TOP_READY || t.buf_fd < 0)
            continue;
        auto du = std::make_shared<DecodedUnit>();
        {
            std::lock_guard<std::mutex> lock(decoding_stats_mutex);
            auto it = decoding_stats.find(pts);
            if (it == decoding_stats.end()) continue;
            du->recv_ts = it->second.recv_start_us;
            du->dec_start_ts = it->second.decode_start_us;
            du->tx_encode_delay_us = it->second.tx_encode_delay_us;
            du->tx_processing_delay_us = it->second.tx_processing_delay_us;
            du->is_keyframe = it->second.is_keyframe;
            du->slices = it->second.slices;
        }
        // A top "decoded" before this picture's first slice was even handed to
        // the decoder is another picture's: its buffer has not been decoded
        // into yet and still holds whatever picture was in it before.
        if (t.time_us > 0 && du->dec_start_ts && (uint64_t)t.time_us + 200 < du->dec_start_ts) {
            early_stale_++;
            if (ltrace::on()) ltrace::rec(ltrace::kTopDone, get_time_us(), (uint32_t)pts, 1ULL << 63);
            continue;
        }
        // The first slice's rows are decoded: the top of the picture - its
        // own latency, whether or not it goes up early.
        if (du->slices && du->slices->n.load() > 0) {
            uint64_t z = 0;
            du->slices->s[0].done_us.compare_exchange_strong(z, (uint64_t)t.time_us);
        }
        if (ltrace::on()) ltrace::rec(ltrace::kTopDone, get_time_us(), (uint32_t)pts, (uint64_t)t.time_us);
        // Tops early only in SPLIT (and not with /tmp/kestrel-early-off).
        if (g_frame_mode.load(std::memory_order_relaxed) != kFrameSplit ||
            access("/tmp/kestrel-early-off", F_OK) == 0) continue;
        const uint32_t w = out_w_, h = out_h_, hs = out_hs_, vs = out_vs_;
        if (!w || !h || !hs || !vs) continue;
        du->pts = pts;
        du->width = w;
        du->height = h;
        du->dec_end_ts = (uint64_t)t.time_us;
        {
            std::lock_guard<std::mutex> lock(early_mutex_);
            auto it = strm_pics_.find(pts);
            if (it == strm_pics_.end()) continue;
            du->bottom_eta_us = it->second.eta;
        }
        du->has_prime_fd = true;
        du->prime_fd = t.buf_fd;
        du->drm_pixel_format = DRM_FORMAT_NV12;
        du->pitches[0] = du->pitches[1] = hs;
        du->offsets[0] = 0;
        du->offsets[1] = hs * vs;
        du->early = true;
        du->buf_epoch = s_buf_epoch.load(std::memory_order_relaxed);
        early_tops_++;
        renderer->queue_frame(du);
    }
#endif
}

// A percentile of a ring of samples.
static uint32_t pct_of(const std::vector<uint32_t> &v, unsigned pct) {
    std::vector<uint32_t> s(v);
    std::sort(s.begin(), s.end());
    return s.empty() ? 0 : s[std::min(s.size() - 1, s.size() * pct / 100)];
}

// A streamed picture came out: what its slices' gap and its tail were, for
// the estimates of the next ones.
void VdecRK::strm_picture_out(int64_t pts, uint64_t now_us) {
    std::lock_guard<std::mutex> lock(early_mutex_);
    auto it = strm_pics_.find(pts);
    if (it == strm_pics_.end()) return;
    const StrmPic p = it->second;
    strm_pics_.erase(it);
    for (auto i = strm_pics_.begin(); i != strm_pics_.end();)
        i = (i->first < pts - 64) ? strm_pics_.erase(i) : std::next(i);
    if (!p.last_us || p.last_us < p.first_us || now_us < p.last_us) return;
    const size_t kRing = 256;
    auto push = [&](std::vector<uint32_t> &v, size_t &n, uint32_t x) {
        if (v.size() < kRing) v.push_back(x); else v[n % kRing] = x;
        n++;
    };
    push(gaps_, gap_n_, (uint32_t)std::min<uint64_t>(p.last_us - p.first_us, 1000000));
    push(tails_, tail_n_, (uint32_t)std::min<uint64_t>(now_us - p.last_us, 1000000));
    if (tail_n_ % 32 == 0) {
        FILE *f = fopen("/tmp/kestrel-early.conf", "r");
        unsigned g = 90, t = 90;
        if (f) {
            if (fscanf(f, "%u %u", &g, &t) != 2) { g = 90; t = 90; }
            fclose(f);
        }
        if (g != gap_pct_ || t != tail_pct_)
            printf("VdecRK: early presentation estimates now gap p%u, tail p%u\n", g, t);
        gap_pct_ = std::min(g, 100u);
        tail_pct_ = std::min(t, 100u);
        gap_p90_ = pct_of(gaps_, gap_pct_);
        tail_p90_ = pct_of(tails_, tail_pct_);
        if (tail_n_ % 512 == 0)
            printf("VdecRK: streamed pictures: slice gap p%u %.2f ms, last slice -> picture out p%u %.2f ms\n",
                   gap_pct_, gap_p90_ / 1000.0, tail_pct_, tail_p90_ / 1000.0);
    }
}

void VdecRK::stream_start(void* data_p, int data_len, int64_t pts, uint64_t recv_ts, uint8_t nal_type,
                          uint32_t encode_delay_us, uint32_t processing_delay_us, int slices) {
    next_packet_flags_ = (stream_ok_ && slices > 1)
        ? (MPP_PACKET_FLAG_STREAM_START | MPP_PACKET_STREAM_SLICES(slices)) : 0;
    const bool streamed = next_packet_flags_ != 0;
    if (streamed && early_ok_) {
        // Not known until the last slice is in (stream_append): only then is the
        // bottom half due a decoder's tail later. Guessed from the usual gap
        // between slices, it went up too soon for kestrel-air's intra-refresh
        // restarts - a 2-row first slice, the rest in a slice six times the
        // usual size, never marked last - and the rows below the first slice
        // showed whatever picture the buffer held before.
        std::lock_guard<std::mutex> lock(early_mutex_);
        StrmPic &p = strm_pics_[pts];
        p.first_us = get_time_us();
        p.last_us = 0;
        p.eta = std::make_shared<std::atomic<uint64_t>>(UINT64_MAX);
    }
    feed_packet_to_decoder(data_p, data_len, pts, recv_ts, nal_type, encode_delay_us, processing_delay_us);
    next_packet_flags_ = 0;
    if (streamed && early_ok_) {
        {
            std::lock_guard<std::mutex> lock(early_mutex_);
            early_q_.push_back(pts);
        }
        early_cv_.notify_one();
    }
}

bool VdecRK::stream_append(const void* data, int len, int64_t pts, bool last) {
    if (!stream_ok_ || len <= 0) return false;
    MppDecStreamAppend a;
    a.data = data;
    a.size = (RK_U32)len;
    a.flags = last ? MPP_STREAM_APPEND_LAST : 0;
    a.pts = pts;
    const uint64_t in_us = get_time_us();
    const bool ok = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_STREAM_APPEND, &a) == MPP_OK;
    if (ok) {
        // a streamed picture's later slices reach the decoder as appended
        std::lock_guard<std::mutex> lock(decoding_stats_mutex);
        auto it = decoding_stats.find(pts);
        if (it != decoding_stats.end()) slices_to_decoder(it->second.slices.get(), in_us);
    }
    if (last && ok && early_ok_) {
        // the whole picture is in: its bottom half is due a tail from now.
        // Refused, it is not: the estimate stays unknown and the top waits.
        std::lock_guard<std::mutex> lock(early_mutex_);
        auto it = strm_pics_.find(pts);
        if (it != strm_pics_.end()) {
            it->second.last_us = get_time_us();
            if (it->second.eta) it->second.eta->store(it->second.last_us + tail_p90_);
        }
    }
    if (last) {
        // The picture is all here: what the decoder takes from now on is its
        // decode time proper, as for a picture fed whole.
        std::lock_guard<std::mutex> lock(decoding_stats_mutex);
        auto it = decoding_stats.find(pts);
        if (it != decoding_stats.end()) it->second.decode_start_us = get_time_us();
    }
    return ok;
}

void VdecRK::stream_end(int64_t pts) {
    if (!stream_ok_) return;
    MppDecStreamAppend a;
    a.data = nullptr;
    a.size = 0;
    a.flags = MPP_STREAM_APPEND_LAST;
    a.pts = pts;
    mpi.mpi->control(mpi.ctx, MPP_DEC_SET_STREAM_APPEND, &a);
}
#else
// Built against MPP headers without the stream patch (fpvOS's rockchip-mpp
// 0002): every Frame Mode then decodes whole pictures. Said once, loudly - a
// build from a sysroot without it looked like a latency regression.
void VdecRK::probe_stream() {
    stream_ok_ = false;
    printf("VdecRK: WARNING: built without stream decode (MPP headers lack MPP_STREAM_TOP_READY):\n"
           "VdecRK: every Frame Mode decodes whole pictures - build against fpvOS's rockchip-mpp\n");
}

void VdecRK::early_loop() {}

void VdecRK::strm_picture_out(int64_t, uint64_t) {}

void VdecRK::stream_start(void* data_p, int data_len, int64_t pts, uint64_t recv_ts, uint8_t nal_type,
                          uint32_t encode_delay_us, uint32_t processing_delay_us, int slices) {
    (void)slices;
    feed_packet_to_decoder(data_p, data_len, pts, recv_ts, nal_type, encode_delay_us, processing_delay_us);
}

bool VdecRK::stream_append(const void*, int, int64_t, bool) { return false; }

void VdecRK::stream_end(int64_t) {}
#endif

void VdecRK::cleanup() {
	printf("Feeding eos\n");
    mpp_packet_set_eos(packet);
    mpp_packet_set_length(packet, 0);
    int ret=0;
    int retry_count = 0;
    while (MPP_OK != (ret = mpi.mpi->decode_put_packet(mpi.ctx, packet))) {
        usleep(10000);
        if (retry_count++ > 10) {
            printf("VdecRK: cleanup timeout feeding EOS\n");
            break; 
        }
    }
}

void VdecRK::set_control_verbose(MpiCmd control,RK_U32 enable){
    RK_U32 res = mpi.mpi->control(mpi.ctx, control, &enable);
    if(res){
        printf("Could not set control %d %d\n",control,enable);
        assert(false);
    }
}

void VdecRK::set_mpp_decoding_parameters() {
    // config for runtime mode
    MppDecCfg cfg       = NULL;
    mpp_dec_cfg_init(&cfg);
    // get default config from decoder context
    int ret = mpi.mpi->control(mpi.ctx, MPP_DEC_GET_CFG, cfg);
    if (ret) {
        printf("%p failed to get decoder cfg ret %d\n", mpi.ctx, ret);
        assert(false);
    }
    // split_parse is to enable mpp internal frame spliter when the input
    // packet is not aplited into frames.
    RK_U32 need_split   = 0;
    ret = mpp_dec_cfg_set_u32(cfg, "base:split_parse", need_split);
    if (ret) {
        printf("%p failed to set split_parse ret %d\n", mpi.ctx, ret);
        assert(false);
    }
    ret = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_CFG, cfg);
    if (ret) {
        printf("%p failed to set cfg %p ret %d\n", mpi.ctx, cfg, ret);
        assert(false);
    }
    mpp_dec_cfg_deinit(cfg);   // MPP_DEC_SET_CFG copied it
	int mpp_split_mode = 0;
    set_control_verbose(MPP_DEC_SET_PARSER_SPLIT_MODE, mpp_split_mode);
    set_control_verbose(MPP_DEC_SET_DISABLE_ERROR, 1);
    set_control_verbose(MPP_DEC_SET_IMMEDIATE_OUT, 1);
    set_control_verbose(MPP_DEC_SET_ENABLE_FAST_PLAY, 1);
    //set_control_verbose(mpi,ctx,MPP_DEC_SET_ENABLE_DEINTERLACE, 1);
    // Docu fast mode:
    // and improve the
    // parallelism of decoder hardware and software
    // we probably don't want that, since we don't need pipelining to hit our bitrate(s)
    int fast_mode = 0;  // pipelining off: ride through corrupt/partial frames instead of HW timeout+reset storm
    set_control_verbose(MPP_DEC_SET_PARSER_FAST_MODE,fast_mode);
}