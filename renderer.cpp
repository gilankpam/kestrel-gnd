extern "C"{
    #include <sys/ioctl.h>
}
#include <algorithm>
#include <bitset>
#include <iostream>
#include <iomanip>

#include "renderer.hpp"
#include "frame_mode.hpp"

std::atomic<int> g_frame_mode{kFrameWhole};
#include "settings.hpp"
#include <cmath>
#include <time.h>
#include "utils/ltrace.hpp"
#include "dvr.hpp"



void Renderer::init_buffers(DecodedUnit* du) {
    display_buffers.resize((render_mode==FrontBuffer) ? 1 : MAX_DISPLAY_BUFFERS);
	buffer_initialized = true;
}

void Renderer::copy_sw_frame(DisplayBufferInfo& buf_info, DecodedUnit* du) {
    if (!du || !buf_info.sw_base ) {
        fprintf(stderr, "copy_sw_frame: Invalid input or unmapped buffer.\n");
        return;
    }
    
    // Map source if needed (and possible)
    uint8_t* src_y = du->data[0];
    uint8_t* src_uv = du->data[1];
    int src_stride_y = du->linesize[0];
    int src_stride_uv = du->linesize[1];
    void* mapped_ptr = nullptr;
    size_t mapped_size = 0;

    if (!src_y && du->has_prime_fd) {
        // We need to map the IMPORTED frame to read from it.
        // NOTE: This usually requires the buffer to be created as mappable (dumb buffer or specific heap).
        // If it's a pure hardware secured buffer, this will fail.
        struct drm_mode_map_dumb mreq = {};
        // We can't map a prime_fd directly easily without importing it to a GEM handle first.
        // Luckily, we likely already imported it or can do so.
        // But DecodedUnit just has prime_fd.
        
        // Quick/Dirty: Import to handle, map, read, close.
        uint32_t handle = 0;
        if (drmPrimeFDToHandle(dev->drm_fd, du->prime_fd, &handle) == 0) {
           mreq.handle = handle;
           if (drmIoctl(dev->drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) == 0) {
               // Approximate size for NV12
               mapped_size = du->width * du->height * 2; 
               mapped_ptr = mmap(0, mapped_size, PROT_READ, MAP_SHARED, dev->drm_fd, mreq.offset);
               if (mapped_ptr != MAP_FAILED) {
                   src_y = (uint8_t*)mapped_ptr;
                   // Mpp/NV12 usually contiguous: Y...UV
                   // Stride? Assume aligned to width or passed keys
                   src_stride_y = du->pitches[0] ? du->pitches[0] : du->width;
                   src_stride_uv = du->pitches[1] ? du->pitches[1] : du->width; 
                   
                   // Determine offsets
                   // If offsets are 0, assume standard packed
                   uint32_t off_y = du->offsets[0];
                   uint32_t off_uv = du->offsets[1];
                   if (off_uv == 0) off_uv = src_stride_y * du->height;
                   
                   src_uv = src_y + (off_uv - off_y);
               }
           }
        }
        // Don't close handle yet if mmap needs it? DRM handles can be closed after mmap technically, 
        // but let's keep it simple and leaky for this transient scope if needed, or close at end.
        // Actually, closing handle doesn't unmap.
        if (handle) drmCloseBufferHandle(dev->drm_fd, handle);
    }
    
    if (!src_y) {
         // Still null?
         return;
    }

    int width = du->width;
    int height = du->height;

    // Copy Y plane
    for (int i = 0; i < height; ++i) {
        memcpy(buf_info.sw_base + i * buf_info.stride,
               src_y + i * src_stride_y,
               width);
    }

    // Copy UV interleaved plane (NV12)
    uint8_t* uv_dst = buf_info.sw_base + buf_info.sw_uv_offset;
    for (int i = 0; i < height / 2; ++i) {
        memcpy(uv_dst + i * buf_info.stride,
               src_uv + i * src_stride_uv,
               width);
    }
    
    if (mapped_ptr) {
        munmap(mapped_ptr, mapped_size);
    }
}

int Renderer::get_or_create_display_buffer(DecodedUnit* du, DisplayBufferInfo& out_info) {
    if (!du) {
        fprintf(stderr, "Null frame passed to get_or_create_display_buffer.\n");
        return -1;
    }

    if (render_mode == FrontBuffer) {
        int slot_idx = 0;
        if(display_buffers[slot_idx].drm_fb_id != 0) {
            out_info = display_buffers[slot_idx];
            return slot_idx;
        }
        // Fallthrough to create SW buffer at index 0
        // We trick the subsequent logic by temporarily clearing has_prime_fd if it's set
        // But we can't modify 'du' destructively safely.
        // Instead, we just jump to Case 2 logic.
        goto create_sw_buffer;
    }

    // -------------------------
    // Case 1: Hardware buffer (with prime_fd)
    // -------------------------
    if (du->has_prime_fd) {
        // Find available slot (Match prime_fd or reuse oldest)
        int slot_idx = -1;
        for (int i = 0; i < display_buffers.size(); ++i) {
            if (display_buffers[i].prime_fd == du->prime_fd && display_buffers[i].drm_fb_id != 0) {
                out_info = display_buffers[i];
                return i;
            }
        }

        // No match? Use FIFO reuse
        slot_idx = next_display_buffer_idx;
        next_display_buffer_idx = (next_display_buffer_idx + 1) % display_buffers.size();

        // Clean up previous
        if (display_buffers[slot_idx].drm_fb_id != 0) {
            cleanup_display_buffer(display_buffers[slot_idx]);
        }

        DisplayBufferInfo new_buf_info;
        new_buf_info.prime_fd = du->prime_fd;

        if (drmPrimeFDToHandle(dev->drm_fd, du->prime_fd, &new_buf_info.gem_handle)) {
            perror("Failed to import prime FD to GEM handle");
            return -1;
        }

        du->handles[0] = new_buf_info.gem_handle;
        du->handles[1] = new_buf_info.gem_handle;
        du->handles[2] = 0;
        du->handles[3] = 0;

        if (drmModeAddFB2WithModifiers(dev->drm_fd, du->width, du->height,
                    du->drm_pixel_format,
                    du->handles, du->pitches, du->offsets,
                    du->modifiers,
                    &new_buf_info.drm_fb_id, DRM_MODE_FB_MODIFIERS)) {
            perror("drmModeAddFB2WithModifiers failed");
            drmCloseBufferHandle(dev->drm_fd, new_buf_info.gem_handle);
            return -1;
        }

        display_buffers[slot_idx] = new_buf_info;
        out_info = new_buf_info;
        return slot_idx;
    }

    // -------------------------
    // Case 2: Software-rendered buffer (no prime_fd) OR FrontBuffer
    // -------------------------
    create_sw_buffer:

    int slot_idx = -1;
    for (int i = 0; i < display_buffers.size(); ++i) {
        // todo for live copy just skip the in_use check
        if (display_buffers[i].drm_fb_id == 0 || !display_buffers[i].in_use) {
            slot_idx = i;
            break;
        }
    }

    if (slot_idx == -1) {
        fprintf(stderr, "No available SW display buffer slots.\n");
        exit(1);
        return -1;
    }

    if(display_buffers[slot_idx].drm_fb_id != 0) {
        // buffer has already been created, reuse.
        out_info = display_buffers[slot_idx];
        return slot_idx;
    }

    // Determine DRM format based on input frame pixel format
    // Force NV12 because the hardware (N100/i915) seemingly rejected DRM_FORMAT_YUV420 for AddFB2.
    // We will perform the conversion (interleaving) during copy if needed.
    uint32_t drm_fmt = DRM_FORMAT_NV12;
    // if (du->drm_pixel_format == DRM_FORMAT_YUV420) {
    //    drm_fmt = DRM_FORMAT_YUV420;
    // }

    DrmBuffer out;
    if (!create_drm_buffer(du->width, du->height, drm_fmt, out)) {
        fprintf(stderr, "Failed to create SW DRM buffer (fmt=0x%x).\n", drm_fmt);
        return -1;
    }

    // Map dumb buffer to memory
    struct drm_mode_map_dumb mreq = {};
    mreq.handle = out.handle;

    if (drmIoctl(dev->drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq)) {
        perror("DRM_IOCTL_MODE_MAP_DUMB failed");
        return -1;
    }

    uint8_t* base = (uint8_t*)mmap(0, out.size, PROT_READ | PROT_WRITE, MAP_SHARED, dev->drm_fd, mreq.offset);
    if (base == MAP_FAILED) {
        perror("mmap failed");
        return -1;
    }

    // Fill DisplayBufferInfo
    DisplayBufferInfo sw_buf;
    sw_buf.gem_handle = out.handle;
    sw_buf.drm_fb_id = out.fb_id;
    sw_buf.width = du->width;
    sw_buf.height = du->height;
    sw_buf.stride = out.stride; // driver-aligned dumb-buffer pitch (matches the FB)
    sw_buf.sw_base = base;

    // Calculate offsets based on format (in terms of the aligned pitch, so the
    // copy lands exactly where drmModeAddFB2 expects each plane).
    if (drm_fmt == DRM_FORMAT_YUV420) {
        // YUV420P: Y plane, then U plane, then V plane.
        sw_buf.sw_u_offset = out.stride * du->height;
        sw_buf.sw_v_offset = sw_buf.sw_u_offset + (out.stride / 2 * du->height / 2);
    } else {
         // NV12: Y plane, then interleaved UV plane
         sw_buf.sw_uv_offset = out.stride * du->height;
    }
    
    sw_buf.prime_fd = 0;
    sw_buf.format = drm_fmt;

    display_buffers[slot_idx] = sw_buf;
    out_info = sw_buf;
    return slot_idx;
}

bool Renderer::create_drm_buffer(uint32_t width, uint32_t height, uint32_t format, DrmBuffer& out, bool export_fd) {
    struct drm_mode_create_dumb creq = {};
    creq.width = width;
    creq.height = height; 
    
    // Calculate total bpp / height multiplier for dumb buffer size logic
    // NV12 and YUV420 = 12bpp effectively -> height * 1.5
    creq.height = height * 3 / 2; 
    creq.bpp = 8;

    if (drmIoctl(dev->drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) {
        perror("DRM_IOCTL_MODE_CREATE_DUMB failed");
        return false;
    }

    out.handle = creq.handle;

    if (export_fd) {
        struct drm_prime_handle dph = {};
        dph.handle = creq.handle;
        if (drmIoctl(dev->drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &dph) < 0) {
            perror("DRM_IOCTL_PRIME_HANDLE_TO_FD failed");
            return false;
        }
        out.prime_fd = dph.fd;
    }

    // Use the driver-aligned pitch the kernel actually allocated, not `width`.
    // amdgpu rounds the dumb-buffer pitch up to its hardware alignment (e.g. 256
    // bytes); declaring an unaligned pitch=width to drmModeAddFB2 gets the FB
    // rejected with EINVAL. The luma/chroma offsets and the copy stride must all
    // follow creq.pitch too, which is why we hand it back via out.stride.
    uint32_t pitch = creq.pitch;
    out.stride = pitch;

    uint32_t handles[4] = {}, pitches[4] = {}, offsets[4] = {};
    handles[0] = creq.handle;
    pitches[0] = pitch;
    offsets[0] = 0;

    if (format == DRM_FORMAT_YUV420) {
        // Y
        handles[1] = creq.handle;
        pitches[1] = pitch / 2;
        offsets[1] = pitch * height;

        // U (or V depending on exact fourcc, YUV420 usually I420 = Y, U, V)
        handles[2] = creq.handle;
        pitches[2] = pitch / 2;
        offsets[2] = offsets[1] + (pitch / 2 * height / 2);
    } else {
        // NV12
        handles[1] = creq.handle;
        pitches[1] = pitch;
        offsets[1] = pitch * height;
    }
    
    if (drmModeAddFB2(dev->drm_fd, width, height, format, handles, pitches, offsets, &out.fb_id, 0) < 0) {
        perror("drmModeAddFB2 failed");
        struct drm_mode_destroy_dumb dreq = { .handle = creq.handle };
        drmIoctl(dev->drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
        return false;
    }

    out.size = creq.size;

    return true;
}

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#endif

void Renderer::convert_yuv420_to_nv12(DisplayBufferInfo& buf_info, DecodedUnit* du) {
    if (!du || !buf_info.sw_base) return;

    int width = du->width;
    int height = du->height;
    
    // Destination (Mapped DRM Buffer, NV12)
    uint8_t* y_dst = buf_info.sw_base;
    uint8_t* uv_dst = buf_info.sw_base + buf_info.sw_uv_offset;

    // Source (Decoded Frame, YUV420P)
    uint8_t* y_src = du->data[0];
    uint8_t* u_src = du->data[1];
    uint8_t* v_src = du->data[2];

    int y_stride_src = du->linesize[0];
    int u_stride_src = du->linesize[1];
    int v_stride_src = du->linesize[2];
    int stride_dst = buf_info.stride;

    // 1. Copy Y Plane (memcpy is usually SIMD-optimized by libc)
    if (y_stride_src == stride_dst) {
         memcpy(y_dst, y_src, width * height);
    } else {
        for (int i = 0; i < height; ++i) {
            memcpy(y_dst + i * stride_dst, y_src + i * y_stride_src, width);
        }
    }

    // 2. Interleave U/V into NV12 UV Plane using SSE2 intrinsics
    // NV12 UV layout: U0 V0 U1 V1 ...
    int uv_width = width / 2;
    int uv_height = height / 2;
    
    for (int y = 0; y < uv_height; ++y) {
        uint8_t* u_row = u_src + y * u_stride_src;
        uint8_t* v_row = v_src + y * v_stride_src;
        uint8_t* dest_row = uv_dst + y * stride_dst;
        
        int x = 0;
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
        // Process 16 UV pixels (consume 16 U bytes + 16 V bytes -> produce 32 NV12 bytes) per iter
        for (; x <= uv_width - 16; x += 16) {
            __m128i u_val = _mm_loadu_si128((const __m128i*)(u_row + x));
            __m128i v_val = _mm_loadu_si128((const __m128i*)(v_row + x));
            
            // Interleave bytes: 
            // lo: u0 v0 u1 v1 ... u7 v7
            // hi: u8 v8 u9 v9 ... u15 v15
            __m128i lo = _mm_unpacklo_epi8(u_val, v_val);
            __m128i hi = _mm_unpackhi_epi8(u_val, v_val);
            
            _mm_storeu_si128((__m128i*)(dest_row + 2 * x), lo);
            _mm_storeu_si128((__m128i*)(dest_row + 2 * x + 16), hi);
        }
#endif

        // Handle remaining pixels scalar
        for (; x < uv_width; ++x) {
            dest_row[2*x] = u_row[x];
            dest_row[2*x+1] = v_row[x];
        }
    }
}


void Renderer::cleanup_display_buffer(DisplayBufferInfo& buffer_info) {
    if (buffer_info.drm_fb_id != 0) {
        drmModeRmFB(dev->drm_fd, buffer_info.drm_fb_id);
        buffer_info.drm_fb_id = 0;
    }
    if (buffer_info.gem_handle != 0) {
        // struct drm_gem_close args = { .handle = buffer_info.gem_handle };
        // drmIoctl(dev->drm_fd, DRM_IOCTL_GEM_CLOSE, &args); // Close the GEM handle we created from import
        // More simply with libdrm >= 2.4.59:
        drmCloseBufferHandle(dev->drm_fd, buffer_info.gem_handle);
        buffer_info.gem_handle = 0;
    }
    // The original prime_fd (buffer_info.prime_fd) is owned by the decoder's AVFrame.
    // We don't close it directly unless we dup'd it.
    // It will be released when buffer_info.av_frame_ref is unref'd.
    // if (buffer_info.av_frame_ref) {
    //     av_frame_unref(buffer_info.av_frame_ref);
    //     buffer_info.av_frame_ref = nullptr;
    // }
    buffer_info.prime_fd = -1;
}

void Renderer::queue_frame(std::shared_ptr<DecodedUnit> du) {
    if (!du->early) {
        pictures_in_.fetch_add(1, std::memory_order_relaxed);
        pictures_total_.fetch_add(1, std::memory_order_relaxed);
    }
    // The queue drops a repeated id: an early picture and its whole one share a
    // pts, so the early one goes under an id of its own.
    const uint64_t id = du->early ? (du->pts | (1ULL << 63)) : du->pts;
    if (DisplayLock *l = lock_live_.load(std::memory_order_acquire)) {
        uint64_t last_in = 0;
        if (du->slices) {
            const int n = std::min(du->slices->n.load(std::memory_order_acquire), (int)SliceTimes::kMax);
            for (int i = 0; i < n; i++) last_in = std::max(last_in, du->slices->s[i].in_us.load(std::memory_order_relaxed));
        }
        l->note_unit(du->pts, du->early, get_time_us(), last_in, halves_live_.load(std::memory_order_relaxed),
                     du->slices ? du->slices->frame_cap_us : 0);
    }
    decoded_unit_queue->put(DataMsg<std::shared_ptr<DecodedUnit>>(id, du));
}

void Renderer::run(){
    SchedulingHelper::configure_thread(SchedulingHelper::ThreadRole::Renderer);
    
    std::shared_ptr<DecodedUnit> latest_frame = nullptr;

    // Deadline commit. A picture is not committed the moment it is decoded but
    // just before the next vblank, with whatever is newest by then. The flip
    // then lands at that very vblank, so it is never still pending when the
    // next picture arrives. Committing at once instead let a picture decoded
    // while a flip was pending go out just after that flip's vblank, pending a
    // whole refresh and blocking the next picture the same way: at 120 Hz that
    // put the median flip-to-vblank wait at 6 ms, and at 100 Hz output with
    // 100 fps video it chained on every picture. Photodiode, 3 pairs each:
    // median 46.7 -> 44.4 ms, p90 68.2 -> 61.5 ms.
    //
    // The margin is how long before the vblank the commit is made. On this
    // VOP2 a commit takes 0.2 to over 0.8 ms to be latched: at 0.8 ms before
    // the vblank 23% missed it and went out a refresh later, at 1.2 ms 1%. A
    // picture less than commit_min_slack_ms from a vblank waits for the next.
    // Under VRR there is no deadline: the commit itself ends the frame.
    const bool vrr = dev->vrr_active();
    const bool deadline_commit = deadline_commit_ =
        Settings::getInstance().getInt("deadline_commit", 1) != 0 && !vrr;
    const double commit_margin_us = commit_margin_us_ = Settings::getInstance().getFloat("commit_margin_ms", 1.2f) * 1000.0;
    const double commit_min_slack_us = commit_min_slack_us_ = Settings::getInstance().getFloat("commit_min_slack_ms", 0.6f) * 1000.0;
    if (deadline_commit && render_mode == Atomic)
        printf("renderer: committing %.1f ms before each vblank\n", commit_margin_us / 1000);
    // DISPLAY > Refresh Sync: the vblank held just after the pictures arrive,
    // aimed at this deadline (and a little for this thread's own work).
    if (deadline_commit && render_mode == Atomic && !lock_) {
        lock_.reset(new DisplayLock(dev, commit_margin_us + 100));
        lock_live_.store(lock_.get(), std::memory_order_release);
        lock_->start();
    }
    // VRR: there is no fixed vblank to aim for - a commit ends the frame being
    // stretched, so each picture goes out the moment it is decoded.
    if (vrr && render_mode == Atomic)
        printf("renderer: VRR - committing each picture as soon as it is decoded\n");
    if (const char *e = getenv("KESTREL_SPLIT")) split_enabled_ = atoi(e) != 0;
    // Under VRR a commit starts a frame: a picture in halves takes two, its
    // top's as soon as the top is decoded, its rest's when that is. On the
    // RK3588 at 4K120 that once underflowed the display controller
    // (POST_BUF_EMPTY, a black screen) - hence the switch.
    if (vrr) {
        vrr_halves_ = Settings::getInstance().getInt("vrr_halves", 1) != 0;
        if (const char *e = getenv("KESTREL_VRR_HALVES")) vrr_halves_ = atoi(e) != 0;
    }
    const bool split_off_for_vrr = vrr && split_enabled_ && !vrr_halves_;
    if (split_off_for_vrr) split_enabled_ = false;
    if (render_mode == Atomic && dev->has_split_plane())
        printf("renderer: a picture can go up in halves, its top before its rest%s\n",
               split_off_for_vrr ? " - off under VRR" :
               split_enabled_ ? "" : " - off (KESTREL_SPLIT=0)");

    while(!*should_stop) {
        // Handle resolution-change flush request from the decoder thread.
        // All display_buffers access happens here in the renderer thread — no mutex needed.
        if (flush_requested.load()) {
            if (dev->is_flip_pending()) dev->wait_for_flip_completion(50);
            // Drain any queued frames with stale prime_fds
            while (decoded_unit_queue->size() > 0) decoded_unit_queue->tryGet();
            latest_frame = nullptr;
            keyframe_pending_ = false;
            dev->set_flip_picture(nullptr);
            split_bottom_.reset();
            halves_reset();
            // Clean up all cached display buffer entries (gem handles + DRM FBs)
            for (auto& buf : display_buffers) {
                if (buf.drm_fb_id != 0) cleanup_display_buffer(buf);
            }
            buffer_initialized = false;
            next_display_buffer_idx = 0;
            flush_requested.store(false);
            flush_done.store(true);
            continue;
        }

        // Each half on its own, where the screen has the second plane for it.
        {
            const bool want = halves_wanted();
            if (want != halves_mode_) {
                printf("renderer: %s (%s)\n", want ? "each half of a picture on its own (two planes)"
                                                     : "whole pictures (one plane)", halves_why_.c_str());
                if (want) {
                    halves_reset();          // nothing from before the classic stretch
                    if (latest_frame) halves_take(latest_frame);
                    latest_frame = nullptr;
                } else {
                    // The newest picture there is goes up whole, as before.
                    for (auto it = pics_.rbegin(); it != pics_.rend() && !latest_frame; ++it)
                        if (!it->top_shown || !it->bottom_shown)
                            latest_frame = it->whole ? it->whole : it->top;
                    halves_reset();
                }
                halves_mode_ = want;
                halves_live_.store(want, std::memory_order_relaxed);
            }
            if (halves_mode_) {
                halves_step();
                continue;
            }
        }

        uint64_t now = get_time_us();

        // 1. DRAIN QUEUE: Find the most recent decoded frame
        bool got_new = false;
        while (decoded_unit_queue->size() > 0) {
            auto m = decoded_unit_queue->tryGet();
            if (m) {
                // static_cast, not dynamic_cast: decoded_unit_queue has exactly one
                // producer (put() below) and it only ever pushes this type. The
                // reference-form dynamic_cast threw std::bad_cast on a mismatch, which
                // in this thread is a terminate() rather than a dropped frame - and it
                // cost an RTTI lookup on every frame.
                auto& dm = static_cast<DataMsg<std::shared_ptr<DecodedUnit>>&>(*m);
                std::shared_ptr<DecodedUnit> du = dm.getPayload();
                // The whole of a picture already shown early: nothing to flip.
                if (!du->early && adopt_early(du)) continue;
                // An early picture never replaces a whole one still waiting to be
                // shown: that one goes first, and the early one's whole follows.
                if (du->early && latest_frame && !latest_frame->early) continue;
                if (latest_frame && !latest_frame->early) {
                    // Replaced before it was shown: not sampled (the next one shown
                    // counts it as never shown). A keyframe takes the longest to decode, so the next picture
                    // is often ready right behind it and replaces it here. Its
                    // being a keyframe must not go with it: the switch from the
                    // background to live video waits for one, and with intra
                    // refresh one comes only on request - replaced every time,
                    // the goggle stayed on the background with video decoding.
                    if (latest_frame->is_keyframe) keyframe_pending_ = true;
                }
                latest_frame = du;
                got_new = true;
            }
        }

        // 2. Statistics Heartbeat (Update even if no frames)
        update_stats(nullptr, 0);
        heartbeat(now);

        // 2b. Deadline commit: hold the newest picture until just before the
        // next vblank. Sleeping returns to the top, so anything decoded in the
        // meantime replaces it (the drain above keeps only the newest).
        // Early pictures go only through the deadline commit, which knows the vblank.
        if (latest_frame && latest_frame->early && !(deadline_commit && render_mode == Atomic)) {
            latest_frame = nullptr;
            continue;
        }
        if (deadline_commit && render_mode == Atomic && latest_frame) {
            if (dev->is_flip_pending()) {
                dev->wait_for_flip_completion(2);
                continue;
            }
            uint64_t vb = 0;
            double period = dev->frame_period_us();
            if (period > 0 && dev->last_vblank(&vb)) {
                uint64_t t = get_time_us();
                double since = t > vb ? (double)(t - vb) : 0.0;
                double to_next = period - fmod(since, period);
                double sleep_us = 0;
                if (to_next > commit_margin_us) sleep_us = to_next - commit_margin_us;                  // early: wait
                else if (to_next < commit_min_slack_us) sleep_us = to_next + period - commit_margin_us; // too late for this one
                if (sleep_us > 50) {
                    struct timespec ts = { 0, (long)(sleep_us * 1000) };
                    nanosleep(&ts, nullptr);
                    continue;
                }
                // An early picture, at this vblank, only if its missing part is
                // decoded by the time the scan-out gets there. If not, look again
                // at the next one - the whole picture may be here by then.
                // Looked at again shortly, not after the vblank: the whole picture
                // arriving meanwhile must still make this commit.
                if (latest_frame->early && !early_safe(latest_frame.get(), to_next, t)) {
                    if (early_held_pts_ != latest_frame->pts) {
                        early_held_pts_ = latest_frame->pts;
                        early_held_++;
                    }
                    struct timespec ts = { 0, 200 * 1000 };
                    nanosleep(&ts, nullptr);
                    continue;
                }
            } else if (latest_frame->early) {
                latest_frame = nullptr;
                continue;
            }
        }

        // 3. If hardware is busy, wait for it to clear BEFORE attempting render
        // EXCEPTION: FrontBuffer mode does NOT wait. It renders (memcopies) immediately and uses flip ONLY for async stats.
        if (latest_frame && dev->is_flip_pending() && render_mode != FrontBuffer) {
            dev->wait_for_flip_completion(2);
            // After waiting, we MUST 'continue' to the start of the while loop 
            // so we can drain any NEWER frames that arrived while we were sleeping.
            continue; 
        }
        
        // 4. If no frames available at all, wait for the decoder
        if (!latest_frame) {
            auto m = decoded_unit_queue->get(5); 
            if (m) {
                // static_cast, not dynamic_cast: decoded_unit_queue has exactly one
                // producer (put() below) and it only ever pushes this type. The
                // reference-form dynamic_cast threw std::bad_cast on a mismatch, which
                // in this thread is a terminate() rather than a dropped frame - and it
                // cost an RTTI lookup on every frame.
                auto& dm = static_cast<DataMsg<std::shared_ptr<DecodedUnit>>&>(*m);
                std::shared_ptr<DecodedUnit> du = dm.getPayload();
                if (!du->early && adopt_early(du)) continue;
                latest_frame = du;
                if (deadline_commit && render_mode == Atomic) continue;   // through the deadline above
            } else {
                continue;
            }
        }
        
        // The device is told which picture it is about to show, for the
        // screen recorder (screen_tap.hpp). That reads the picture after the
        // display has, so while it runs the picture is held from now until
        // the tap lets it go.
        if (render_mode == Atomic || render_mode == FrontBuffer) {
            if (dev->tap.recording() && !latest_frame->frame_ref && latest_frame->hold && latest_frame->buf)
                latest_frame->frame_ref = latest_frame->hold(latest_frame->buf);
            dev->set_flip_picture(latest_frame);
        }

        // 5. Hardware is guaranteed free (or we have no choice but to try), so render.
        if (render_frame(latest_frame.get())) {
            if (latest_frame->early) {
                early_shown_pts_ = latest_frame->pts;
                early_flips_++;
            }
            latest_frame = nullptr; 
        }
    }
}

// May an early picture (DecodedUnit::early) go out at the vblank to_next_us
// away? Only if what is still missing of it - from about where its first
// slice ends, which is never above the middle with the air unit's two halves -
// is decoded before the scan-out reaches it, with a millisecond to spare.
bool Renderer::early_safe(const DecodedUnit *du, double to_next_us, uint64_t now_us) {
    // Not while recording (the recorder copies the picture as the flip has
    // it), and not with more than two slices (where the first ends is not
    // learned for those).
    if (dev->tap.recording()) return false;
    if (du->slices && du->slices->expected > 2) return false;
    const uint64_t eta = du->bottom_eta_us ? du->bottom_eta_us->load() : 0;
    const int rows = dev->screen_rows();
    // 0 or UINT64_MAX: not known yet (its last slice is not in)
    if (!eta || eta == UINT64_MAX || rows <= 0 || !du->height) return false;
    // a few rows above where the first slice ends (the middle, unless the
    // source says otherwise): the deblocking across the slice boundary
    // changes them once the rest is decoded
    const uint32_t end = du->slices ? du->slices->first_end() : 0;
    const uint32_t from = end > 32 ? end - 32 : du->height / 2 - 16;
    const int row = dev->screen_row_of(from, du->height);   // a tile or Picture Size included
    const uint64_t reach = now_us + (uint64_t)to_next_us + (uint64_t)dev->row_time_us(row);
    if ((int64_t)reach < (int64_t)eta + early_margin_us_) return false;
    early_boundary_us_ = reach;
    return true;
}

// The whole of the picture shown early has come: it is on screen already
// (the decoder finished it in the buffer being scanned), so it is not flipped
// again - it takes the early one's place and its keyframe counts.
bool Renderer::adopt_early(std::shared_ptr<DecodedUnit> &du) {
    if (du->pts != early_shown_pts_) return false;
    early_shown_pts_ = UINT64_MAX;
    early_adopted_++;
    if (du->dec_end_ts > early_boundary_us_) early_late_++;
    if (render_mode == Atomic) dev->set_flip_picture(du);
    if (osd) osd->notify_video_frame(du->is_keyframe || keyframe_pending_);
    keyframe_pending_ = false;
    return true;
}

bool Renderer::split_active() const {
    // Not while the screen is recorded: the recorder copies the video plane's
    // picture only, whose lower part may not be decoded yet.
    return split_enabled_ && !split_off_ && g_frame_mode.load(std::memory_order_relaxed) == kFrameSplit &&
           render_mode == Atomic && dev->has_split_plane() &&
           !dev->tap.recording();
}

// Every 5 s: the presentation counters, and the runtime switches.
void Renderer::heartbeat(uint64_t now) {
    static uint64_t last_alive = 0;
    if (now - last_alive <= 5000000) return;
    if (halves_whole_ || halves_split_ || halves_lower_)
        printf("renderer: each half on its own, 5 s: %u flips whole (%u of them early), %u top over an "
               "older lower part, %u lower part alone, %u tops a vblank late behind an older one, "
               "%u pictures let go (backlog), %u lower parts never decoded\n",
               halves_whole_, early_flips_, halves_split_, halves_lower_, halves_waited_,
               halves_dropped_, halves_lower_skipped_);
    if (halves_holds_ || halves_lower_first_ || halves_lower_superseded_)
        printf("renderer: each half, 5 s: policy %d, %u deadlines with a top waiting and nothing put up, "
               "%u older lower parts put up before a whole picture, %u older lower parts superseded\n",
               halves_policy_, halves_holds_, halves_lower_first_, halves_lower_superseded_);
    halves_holds_ = halves_lower_first_ = halves_lower_superseded_ = 0;
    halves_whole_ = halves_split_ = halves_lower_ = halves_waited_ = halves_dropped_ = halves_lower_skipped_ = 0;
    {
        FILE *f = fopen("/tmp/kestrel-halves.conf", "r");
        int pol = 0;
        if (f) {
            if (fscanf(f, " policy %d", &pol) != 1) pol = 0;
            fclose(f);
        }
        if (pol < 0 || pol > 2) pol = 0;
        if (pol != halves_policy_) printf("renderer: each half: policy %d -> %d\n", halves_policy_, pol);
        halves_policy_ = pol;
    }
    {
        FILE *f = fopen("/tmp/kestrel-early.conf", "r");
        unsigned g, t;
        int m = 1000;
        if (f) {
            if (fscanf(f, "%u %u %d", &g, &t, &m) != 3) m = 1000;
            fclose(f);
        }
        early_margin_us_ = m;
    }
    if (!halves_mode_ && (early_flips_ || early_held_))
        printf("renderer: early pictures in 5 s: %u shown before their bottom half "
               "(%u where it came late), %u held to the whole picture, %u whole ones taken over\n",
               early_flips_, early_late_, early_held_, early_adopted_);
    early_flips_ = early_late_ = early_held_ = early_adopted_ = 0;
    {
        const bool off = access("/tmp/kestrel-split-off", F_OK) == 0;
        if (off != split_off_)
            printf("renderer: pictures in halves %s (/tmp/kestrel-split-off)\n", off ? "off" : "on");
        split_off_ = off;
    }
    // printf("Renderer: Loop heartbeat alive. flip_pending=%s, queue_size=%zu\n", 
    //        dev->is_flip_pending() ? "YES" : "NO", decoded_unit_queue->size());
    // fflush(stdout);
    last_alive = now;
}

// Each half on its own only on a screen clearly faster than the video: there,
// every half of every picture has a vblank of its own to go up at. On one
// that is not (100 fps on 60 Hz, or on 100 Hz with drift) the halves queue up
// behind each other and it is slower than whole pictures (+8 to +16 ms).
bool Renderer::halves_wanted() {
    if (!(split_active() && (deadline_commit_ || vrr_halves_) && dev->split_possible())) {
        halves_why_ = split_active() ? "deadline commit off" :
                      dev->tap.recording() ? "the screen is being recorded" :
                      dev->has_split_plane() ? "switched off" : "no second video plane";
        return false;
    }
    const uint64_t now = get_time_us();
    if (!rate_t0_) { rate_t0_ = now; rate_n0_ = pictures_total_.load(); }
    if (now - rate_t0_ >= 1000000) {
        const uint64_t n = pictures_total_.load();
        pic_rate_hz_ = (double)(n - rate_n0_) * 1e6 / (double)(now - rate_t0_);
        rate_t0_ = now; rate_n0_ = n;
    }
    const double period = dev->frame_period_us();
    const double refresh = period > 0 ? 1e6 / period : 0;
    if (pic_rate_hz_ < 5 || refresh <= 0) return halves_mode_;   // no video yet: as it is
    const bool faster = refresh >= pic_rate_hz_ * (halves_mode_ ? 1.05 : 1.15);
    // Or the display lock holds the vblank just after the tops arrive
    // (DISPLAY > Refresh Sync): no spare refresh, but each top makes the one
    // after it - and the lower part goes up a refresh later, under the next
    // picture's top. Nothing waits behind a late top there (halves_step).
    const DisplayLock *l = lock_live_.load(std::memory_order_relaxed);
    // Only when the lock follows the tops (DISPLAY > Sync To SLICE 1): timed
    // to the last slice, each picture goes up whole just after it instead -
    // in halves its lower part would wait a second refresh behind its top.
    halves_locked_ = !faster && l && l->in_step() && g_display_lock_slice.load(std::memory_order_relaxed) != 2;
    char b[96];
    snprintf(b, sizeof(b), "%.0f Hz screen, %.0f pictures a second%s", refresh, pic_rate_hz_,
             halves_locked_ ? ", in step (Refresh Sync)" : "");
    halves_why_ = b;
    return faster || halves_locked_;
}

// Back to nothing in halves. What is on screen is held one flip longer (the
// next commit replaces it) - its buffers must not go back to the decoder first.
void Renderer::halves_reset() {
    pics_.clear();
    if (on_top_) prev_top_ = std::move(on_top_);
    if (on_bottom_) prev_bottom_ = std::move(on_bottom_);
    on_top_.reset();
    on_bottom_.reset();
    on_cut_ = 0;
}

// A picture from the decoder: its top decoded (early) or all of it (whole).
void Renderer::halves_take(std::shared_ptr<DecodedUnit> du) {
    if (!du->early) {
        // The whole of a picture on screen: its buffer is held through it now
        // (an early one holds no decoder frame).
        if (on_top_ && on_top_->pts == du->pts && on_top_->early) {
            on_top_ = du;
            early_adopted_++;
            if (du->dec_end_ts > early_boundary_us_) early_late_++;
        }
        if (on_bottom_ && on_bottom_->pts == du->pts && on_bottom_->early) on_bottom_ = du;
    }
    HalfPic *p = nullptr;
    for (auto &q : pics_) if (q.pts == du->pts) { p = &q; break; }
    if (!p) {
        // Shown and gone already, or older than what is up: dropped - but a
        // keyframe still counts (the switch to live video waits for one).
        if ((!pics_.empty() && du->pts <= pics_.back().pts) || (on_top_ && du->pts <= on_top_->pts)) {
            if (du->is_keyframe) keyframe_pending_ = true;
            return;
        }
        pics_.emplace_back();
        p = &pics_.back();
        p->pts = du->pts;
        p->seen_us = get_time_us();
    }
    if (du->early) {
        if (!p->top && !p->whole) p->top = du;
    } else {
        p->whole = du;
        if (!p->top_shown) p->top = du;
    }
    if (du->is_keyframe) p->key = true;
}

// Will picture p's lower part be decoded by the time the scan-out reaches it
// at the vblank to_next_us away?
bool Renderer::lower_ready(const HalfPic &p, double to_next_us, uint64_t now_us) {
    if (p.whole) return true;
    return p.top && p.top->early && early_safe(p.top.get(), to_next_us, now_us);
}

// One commit: top on the video plane - all of the screen, or above cut with
// bottom's lower part on the second plane.
bool Renderer::halves_commit(const std::shared_ptr<DecodedUnit> &top, const std::shared_ptr<DecodedUnit> &bottom,
                             uint32_t cut) {
    // Both planes stay on: a whole picture is its own two halves, one buffer
    // on both. Switching the second plane on or off costs the VOP2 a frame -
    // every such commit landed a vblank late (97 of 97 each way), and the
    // vblank after it was lost with it - while one that only moves it or
    // changes its buffer lands like any other (1% late, as one plane).
    std::shared_ptr<DecodedUnit> low = bottom;
    if (!low) {
        // Where a whole picture's two halves meet does not matter at 1:1, but
        // scaled each plane filters its own part: one fixed place, not the
        // last split's (which jumps to the top on intra-refresh starts).
        low = top;
        cut = (top->height / 2) & ~1u;
        if (on_cut_ >= top->height / 4 && on_cut_ < top->height) cut = on_cut_;
    }
    split_next_row_ = cut;
    split_bottom_ = low;
    if (dev->tap.recording() && !top->frame_ref && top->hold && top->buf)
        top->frame_ref = top->hold(top->buf);
    dev->set_flip_picture(top);
    const bool ok = render_frame(top.get());
    const bool split = split_next_row_ != 0;
    split_next_row_ = 0;
    split_bottom_.reset();
    if (!ok) return false;
    prev_top_ = std::move(on_top_);
    prev_bottom_ = std::move(on_bottom_);
    on_top_ = top;
    on_bottom_ = split ? low : nullptr;
    on_cut_ = split ? cut : 0;
    return true;
}

// Each half of each picture on its own. At each vblank's deadline: the top of
// the oldest picture whose top is decoded and not up yet, and the lower part
// of the oldest picture whose lower part will be decoded in time and is not up
// yet - in one commit. A picture whose both halves are ready goes up whole on
// one plane. Nothing is skipped (a screen faster than the video has room for
// every half), unless more than two tops are waiting after a stall.
void Renderer::halves_step() {
    uint64_t now = get_time_us();
    while (decoded_unit_queue->size() > 0) {
        auto m = decoded_unit_queue->tryGet();
        if (!m) break;
        auto& dm = static_cast<DataMsg<std::shared_ptr<DecodedUnit>>&>(*m);
        halves_take(dm.getPayload());
    }
    update_stats(nullptr, 0);
    heartbeat(now);

    // Done with: both halves up, or never coming (300 ms).
    while (!pics_.empty()) {
        HalfPic &f = pics_.front();
        if (!(f.top_shown && f.bottom_shown) && now < f.seen_us + 300000) break;
        if (!f.top_shown) {
            halves_dropped_++;
            if (f.key) keyframe_pending_ = true;
        }
        pics_.pop_front();
    }
    if (pics_.empty()) {
        if (auto m = decoded_unit_queue->get(5)) {
            auto& dm = static_cast<DataMsg<std::shared_ptr<DecodedUnit>>&>(*m);
            halves_take(dm.getPayload());
        }
        return;
    }
    if (dev->is_flip_pending()) {
        dev->wait_for_flip_completion(2);
        return;
    }
    uint64_t vb = 0;
    const uint64_t t0 = get_time_us();
    double to_next;
    if (vrr_halves_) {
        // No deadline: whatever is ready goes up now, and the frame starts
        // about a millisecond after the commit (the VOP2's latch).
        to_next = 1000;
        vb = ++vrr_hold_n_;
    } else {
        const double period = dev->frame_period_us();
        if (period <= 0 || !dev->last_vblank(&vb)) {
            usleep(1000);
            return;
        }
        const double since = t0 > vb ? (double)(t0 - vb) : 0.0;
        to_next = period - fmod(since, period);
        double sleep_us = 0;
        if (to_next > commit_margin_us_) sleep_us = to_next - commit_margin_us_;
        else if (to_next < commit_min_slack_us_) sleep_us = to_next + period - commit_margin_us_;
        if (sleep_us > 50) {
            struct timespec ts = { 0, (long)(sleep_us * 1000) };
            nanosleep(&ts, nullptr);
            return;
        }
    }
    const uint64_t t = t0;

    // Tops, oldest first; a backlog past two is let go down to two - past one
    // at one refresh a picture (halves_locked_): with no spare refresh to catch
    // up in, a top that missed its vblank would hold every one after it a
    // refresh late for good, so it gives way to the newer one.
    const int keep = halves_locked_ ? 1 : 2;
    int waiting = 0;
    for (auto &p : pics_) if (!p.top_shown) waiting++;
    for (auto &p : pics_) {
        if (waiting <= keep) break;
        if (p.top_shown) continue;
        p.top_shown = p.bottom_shown = true;
        waiting--;
        halves_dropped_++;
        if (p.key) keyframe_pending_ = true;
    }
    HalfPic *T = nullptr;
    for (auto &p : pics_) if (!p.top_shown) { T = &p; break; }
    const uint64_t top_pts = T ? T->pts : (on_top_ ? on_top_->pts : 0);
    // The lower part: the oldest not up yet that will be decoded in time, of a
    // picture whose top is up or going up now. An older one that never got
    // decoded (lost) gives way to it.
    HalfPic *B = nullptr;
    for (auto &p : pics_) {
        if (p.bottom_shown) continue;
        if (p.pts > top_pts) break;
        if (lower_ready(p, to_next, t)) { B = &p; break; }
    }
    if (B)
        for (auto &p : pics_) {
            if (&p == B) break;
            if (!p.bottom_shown && p.top_shown) { p.bottom_shown = true; halves_lower_skipped_++; }
        }
    auto best = [](const HalfPic &p) { return p.whole ? p.whole : p.top; };
    auto cut_for = [](const DecodedUnit *du) -> uint32_t {
        const uint32_t end = du->slices ? du->slices->first_end() : 0;
        if (end <= 32 + 2 || end >= du->height) return 0;
        return (end - 32) & ~1u;
    };
    const bool can_split = dev->split_possible();

    std::shared_ptr<DecodedUnit> top_du, low_du;
    uint32_t cut = 0;
    HalfPic *up_top = nullptr, *up_low = nullptr;
    // T could go up whole, but an older picture's lower part is waiting.
    bool lower_first = false;
    if (T && B && B != T && lower_ready(*T, to_next, t)) {
        if (halves_policy_ == 2) {
            for (auto &p : pics_) {
                if (p.pts >= T->pts) break;
                if (!p.bottom_shown) { p.bottom_shown = true; halves_lower_superseded_++; }
            }
            B = T;
        } else if (halves_policy_ == 1) {
            lower_first = true;
        }
    }
    if (lower_first) {
        // The older lower part now, under its own top if that is still up
        // (the picture whole), else under the top that is; T next vblank.
        if (on_top_ && B->pts == on_top_->pts) {
            top_du = best(*B);
            up_low = B;
            halves_lower_first_++;
        } else if (can_split && on_top_ && on_cut_ && B->pts < on_top_->pts) {
            top_du = on_top_;
            low_du = best(*B);
            cut = on_cut_;
            up_low = B;
            halves_lower_first_++;
        } else {
            B->bottom_shown = true;
            halves_lower_superseded_++;
            top_du = best(*T);           // nothing older to put up: T whole
            up_top = up_low = T;
        }
    } else if (T && B == T) {
        // Both halves of this picture: whole, on one plane.
        top_du = best(*T);
        up_top = up_low = T;
    } else if (T) {
        // Its top over the newest lower part there is: B's, or what is up now.
        std::shared_ptr<DecodedUnit> low = B ? best(*B) : (on_bottom_ ? on_bottom_ : on_top_);
        const std::shared_ptr<DecodedUnit> &tdu = T->whole ? T->whole : T->top;
        cut = cut_for(tdu.get());
        if (can_split && low && low->pts < T->pts && cut && low->width == tdu->width && low->height == tdu->height) {
            top_du = tdu;
            low_du = low;
            up_top = T;
            up_low = B;
        } else if (B && (!on_top_ || B->pts >= on_top_->pts)) {
            // Not in halves: the older picture goes up whole first (never one
            // older than the top already up).
            top_du = best(*B);
            up_low = B;
            halves_waited_++;
        }
    } else if (B) {
        if (on_top_ && B->pts == on_top_->pts) {
            top_du = best(*B);            // the lower part under its own top: whole now
            up_low = B;
        } else if (can_split && on_top_ && on_cut_) {
            top_du = on_top_;             // the top stays, the lower part moves on
            low_du = best(*B);
            cut = on_cut_;
            up_low = B;
        } else {
            B->bottom_shown = true;       // older than what is up: never shown
            halves_lower_skipped_++;
        }
    }
    if (!top_du) {
        // Nothing to put up at this vblank yet: look again shortly - a lower
        // part may still be decoded in time for it.
        if (T && vb != hold_traced_vb_) halves_holds_++;
        if (T && ltrace::on() && vb != hold_traced_vb_) {
            const std::shared_ptr<DecodedUnit> &tdu = T->whole ? T->whole : T->top;
            const std::shared_ptr<DecodedUnit> low = B ? best(*B) : (on_bottom_ ? on_bottom_ : on_top_);
            const unsigned why = !can_split ? 1 : !cut_for(tdu.get()) ? 2 : !low ? 3 : low->pts >= T->pts ? 4 : 5;
            ltrace::rec(ltrace::kHold, t, (uint32_t)T->pts, (uint64_t)why | ((uint64_t)to_next << 8));
        }
        if (T) hold_traced_vb_ = vb;
        struct timespec ts = { 0, 200 * 1000 };
        nanosleep(&ts, nullptr);
        return;
    }
    if (!halves_commit(top_du, low_du, cut)) return;
    if (ltrace::on())
        ltrace::rec(ltrace::kHalves, t, (uint32_t)top_du->pts,
                    (low_du ? (low_du->pts & 0xffffffffu) : 0) | ((uint64_t)cut << 32) |
                    ((uint64_t)(lower_first ? 4 : !low_du ? 1 : up_top ? 2 : 3) << 48));
    if (up_top) up_top->top_shown = true;
    if (up_low) up_low->bottom_shown = true;
    if (!low_du) {
        halves_whole_++;
        if (top_du->early) {
            early_flips_++;
            early_shown_pts_ = top_du->pts;
        }
    } else if (up_top) {
        halves_split_++;
    } else {
        halves_lower_++;
    }
}

// A nominal exposure + readout for the capture end of the latency figures,
// when the source has no measured one. The AR8030 path measures its own (see
// Ar8030Source::air_delay_for): capture -> encoder out in tx_encode_delay_us
// and the rest of the air delay in tx_processing_delay_us.
uint64_t Renderer::sensor_offset_us(const DecodedUnit* du) {
    if (du->tx_encode_delay_us || du->tx_processing_delay_us || !osd) return 0;   // kAirUnknown too
    uint32_t fps = osd->get_sky_framerate();
    return fps > 0 ? (uint64_t)osd->get_sky_exposure_us() + 1000000 / fps : 0;
}

bool Renderer::render_frame(DecodedUnit *du) {
	if (render_mode == Disable) {
        update_stats(du, 0);
        return true;
    }

    uint64_t render_start_ts = get_time_us();
    
    if(!buffer_initialized) {
		init_buffers(du);
	}

    DisplayBufferInfo buffer_to_display;
    int new_buf_idx = get_or_create_display_buffer(du, buffer_to_display);
    if (new_buf_idx < 0 || buffer_to_display.drm_fb_id == 0) {
        fprintf(stderr, "Failed to get/create display buffer for frame.\n");
        return false;
    }

    int fb_id = buffer_to_display.drm_fb_id;
    current_display_buffer_idx = new_buf_idx;
    if (!du->has_prime_fd || render_mode == FrontBuffer) {
        if (du->drm_pixel_format == DRM_FORMAT_YUV420 && buffer_to_display.format == DRM_FORMAT_NV12) {
             convert_yuv420_to_nv12(buffer_to_display, du);
        } else {
            copy_sw_frame(buffer_to_display, du);
        }
    }
    // FrontBuffer: the copy above IS the presentation - the display is already
    // scanning this buffer, so the frame is live now (modulo where the raster
    // happens to be, which is also why this mode tears). Record the latency
    // here. The probe flip further down is kept for render cadence only, and is
    // given recv_ts = 0 so it cannot overwrite these numbers with its own.
    // (recorded below, once the frame is actually submitted)

    bool performed_modeset = false;

    if (du->width != dev->video_frm_width || du->height != dev->video_frm_height) {
        printf("Frame info changed to %dx%d\n", du->width, du->height);
        dev->set_frame_size(du->width, du->height);
        dev->perform_modeset_video(fb_id);
        performed_modeset = true;
        if(osd) osd->set_video_resolution(du->width, du->height);
	    if(dvr) dvr->init(du->width, du->height);
        // Button-driven recordings are created later, so remember the size.
        DvrRecorder::instance().set_frame_size(du->width, du->height);
        // Do not return here, see logic below
    }

    // Ensure OSD knows the resolution even if it didn't change from DRM default
    static bool first_frame = true;
    if (first_frame) {
        if(osd) osd->set_video_resolution(du->width, du->height);
        first_frame = false;
    }

    bool submitted = false;
    if (performed_modeset) {
        // Modeset is blocking and displays the frame effectively.
        // We skip page_flip to avoid 'flip_pending' deadlock (no event for no-op flip).
        submitted = true;
    } else if (render_mode == Atomic) {
        uint64_t hw_offset_us = sensor_offset_us(du);
        uint32_t total_tx_age = du->tx_processing_delay_us == kAirUnknown ? 0
                              : du->tx_encode_delay_us + du->tx_processing_delay_us + hw_offset_us;
        if (ltrace::on()) ltrace::rec(ltrace::kRender, get_time_us(), (uint32_t)du->pts, (uint64_t)fb_id);
        int bottom_fb = 0;
        if (split_next_row_ && split_bottom_) {
            DisplayBufferInfo bi;
            if (get_or_create_display_buffer(split_bottom_.get(), bi) >= 0) bottom_fb = (int)bi.drm_fb_id;
        }
        if (!bottom_fb) split_next_row_ = 0;
        submitted = dev->page_flip(fb_id, du->recv_ts, du->dec_start_ts, du->dec_end_ts, render_start_ts, total_tx_age, du->slices,
                                   split_next_row_, bottom_fb, bottom_fb ? split_bottom_->slices : nullptr);
    } else if (render_mode == FrontBuffer) {
        // FrontBuffer: Always "submitted" because we already wrote to memory.
        uint64_t hw_offset_us = sensor_offset_us(du);
        if (ltrace::on()) ltrace::rec(ltrace::kRender, get_time_us(), (uint32_t)du->pts, (uint64_t)fb_id);
        dev->direct_shown();
        dev->record_direct_frame(du->recv_ts, du->dec_start_ts, du->dec_end_ts,
                                 render_start_ts,
                                 du->tx_processing_delay_us == kAirUnknown ? 0
                                 : du->tx_encode_delay_us + du->tx_processing_delay_us + hw_offset_us);
        submitted = true;
    }
        
    if (submitted) {
        if (osd) osd->signal_render(prof::kWakeVideo);
        // drives background→live transition; a skipped keyframe counts (keyframe_pending_)
        if (osd) osd->notify_video_frame(du->is_keyframe || keyframe_pending_);
        keyframe_pending_ = false;
        update_stats(du, render_start_ts);
        
        // FrontBuffer mode: Asynchronously request a flip event for stats ONLY, after rendering is complete.
        // This is fire-and-forget - if flip is pending, we just skip it. No blocking whatsoever.
        if (render_mode == FrontBuffer && !dev->is_flip_pending()) {
             uint64_t hw_offset_us = 0;
             if (osd) {
                 uint32_t exp_us = osd->get_sky_exposure_us();
                 uint32_t fps = osd->get_sky_framerate();
                 if (fps > 0) {
                     uint32_t readout_us = 1000000 / fps;
                     hw_offset_us = (uint64_t)exp_us + readout_us;
                 }
             }
             // recv_ts = 0 on purpose: the flip handler skips its stats block
             // when recv_ts is 0, so this probe only feeds render cadence and
             // leaves record_direct_frame()'s numbers alone.
             dev->page_flip(fb_id, 0, 0, 0, 0, 0);
        }
        
        return true;
    }
    
    // Track failed submissions (retries), but don't spam as 'drops'
    static uint64_t last_retries_report = 0;
    static int retry_count = 0;
    retry_count++;
    uint64_t now_ms = get_time_ms();
    if (now_ms - last_retries_report > 5000) {
        if (retry_count > 500) { // Only log if we are retrying excessively
             // This is a debug log, suppressed for user cleanliness
             // printf("DEBUG: %d flip retries in last 5s\n", retry_count);
        }
        retry_count = 0;
        last_retries_report = now_ms;
    }

    return false;
}

void Renderer::reset_stats(uint64_t now){
        stats_.decoding_stats_start = now;
        stats_.frame_counter = 0;
        stats_.proc_latency_max = 0;
        stats_.proc_latency_min = 1e18f;
        stats_.total_latency_max = 0;
        stats_.total_latency_min = 1e18f;
        stats_.decoding_latency_max = 0;
        stats_.decoding_latency_min = 1e18f;
        stats_.display_latency_max = 0;
        stats_.display_latency_min = 1e18f;
        stats_.tx_latency_max = 0;
        stats_.tx_latency_min = 1e18f;
        stats_.frame_pace_max = 0;
        stats_.frame_pace_min = 1e18f;
}

void Renderer::update_stats(DecodedUnit *du, uint64_t display_start_ts) {
    static bool first_call = true;
    if (first_call) {
        printf("Renderer: update_stats called for the first time. console_stats=%s\n", console_stats ? "TRUE" : "FALSE");
        // Initial sync reset: Set TX loop delay to 0 to start fresh
        if (cmd_cb) {
             cmd_cb(0x11, 0); 
        }
        fflush(stdout);
        first_call = false;
    }

    auto now = get_time_us();
	
    if ((now-stats_.decoding_stats_start) >= (osd ? (osd->refresh_frequency_ms*1000) : 1000000)) {
        uint64_t delta_us = now - stats_.decoding_stats_start;
        


        latency_stats osd_stats_ = {}; // Zero initialize POD fields, constructs vectors
        // Pictures decoded, not latency samples: a picture is sampled only once
        // all its slices are on screen.
        float current_framerate = (float)((double)pictures_in_.exchange(0) * 1000000.0 / delta_us);
        
        if (delta_us > 2000000) current_framerate = 0.0f; 

        if (stats_.frame_counter > 0) {
            // Net latency
            float proc_latency_sum = 0;
            for (int i = 0; i < stats_.frame_counter; ++i) {
                proc_latency_sum += stats_.proc_latency_avg[i];
                if (stats_.proc_latency_avg[i] > stats_.proc_latency_max) {
                    stats_.proc_latency_max = stats_.proc_latency_avg[i];
                }
                if (stats_.proc_latency_avg[i] < stats_.proc_latency_min) {
                    stats_.proc_latency_min = stats_.proc_latency_avg[i];
                }
                osd_stats_.proc_latency.values.push_back(stats_.proc_latency_avg[i] / 1000.0);
            }
            osd_stats_.proc_latency.avg = proc_latency_sum / (stats_.frame_counter) / 1000.0;
            osd_stats_.proc_latency.max = stats_.proc_latency_max / 1000.0;
            osd_stats_.proc_latency.min = stats_.proc_latency_min / 1000.0;

            // Decoding latency
            float decoding_latency_sum = 0;
            for (int i = 0; i < stats_.frame_counter; ++i) {
                decoding_latency_sum += stats_.decoding_latency_avg[i];
                if (stats_.decoding_latency_avg[i] > stats_.decoding_latency_max) {
                    stats_.decoding_latency_max = stats_.decoding_latency_avg[i];
                }
                if (stats_.decoding_latency_avg[i] < stats_.decoding_latency_min) {
                    stats_.decoding_latency_min = stats_.decoding_latency_avg[i];
                }
                osd_stats_.decoding_latency.values.push_back(stats_.decoding_latency_avg[i] / 1000.0);
            }
            osd_stats_.decoding_latency.avg = decoding_latency_sum / (stats_.frame_counter) / 1000.0;
            osd_stats_.decoding_latency.max = stats_.decoding_latency_max / 1000.0;
            osd_stats_.decoding_latency.min = stats_.decoding_latency_min / 1000.0;

            // Display latency
            float display_latency_sum = 0;
            for (int i = 0; i < stats_.frame_counter; ++i) {
                display_latency_sum += stats_.display_latency_avg[i];
                if (stats_.display_latency_avg[i] > stats_.display_latency_max) {
                    stats_.display_latency_max = stats_.display_latency_avg[i];
                }
                if (stats_.display_latency_avg[i] < stats_.display_latency_min) {
                    stats_.display_latency_min = stats_.display_latency_avg[i];
                }
                osd_stats_.display_latency.values.push_back(stats_.display_latency_avg[i] / 1000.0);
            }
            osd_stats_.display_latency.avg = display_latency_sum / (stats_.frame_counter) / 1000.0;
            osd_stats_.display_latency.max = stats_.display_latency_max / 1000.0;
            osd_stats_.display_latency.min = stats_.display_latency_min / 1000.0;

            // Total latency
            float total_latency_sum = 0;
            for (int i = 0; i < stats_.frame_counter; ++i) {
                total_latency_sum += stats_.total_latency_avg[i];
                if (stats_.total_latency_avg[i] > stats_.total_latency_max) {
                    stats_.total_latency_max = stats_.total_latency_avg[i];
                }
                if (stats_.total_latency_avg[i] < stats_.total_latency_min) {
                    stats_.total_latency_min = stats_.total_latency_avg[i];
                }
                osd_stats_.total_latency.values.push_back(stats_.total_latency_avg[i] / 1000.0);
            }
            osd_stats_.total_latency.avg = total_latency_sum / (stats_.frame_counter) / 1000.0;
            osd_stats_.total_latency.max = stats_.total_latency_max / 1000.0;
            osd_stats_.total_latency.min = stats_.total_latency_min / 1000.0;

            // TX latency
            float tx_latency_sum = 0;
            for (int i = 0; i < stats_.frame_counter; ++i) {
                tx_latency_sum += stats_.tx_latency_avg[i];
                if (stats_.tx_latency_avg[i] > stats_.tx_latency_max) {
                    stats_.tx_latency_max = stats_.tx_latency_avg[i];
                }
                if (stats_.tx_latency_avg[i] < stats_.tx_latency_min) {
                    stats_.tx_latency_min = stats_.tx_latency_avg[i];
                }
                osd_stats_.tx_latency.values.push_back(stats_.tx_latency_avg[i] / 1000.0);
            }
            osd_stats_.tx_latency.avg = tx_latency_sum / (stats_.frame_counter) / 1000.0;
            osd_stats_.tx_latency.max = stats_.tx_latency_max / 1000.0;
            osd_stats_.tx_latency.min = stats_.tx_latency_min / 1000.0;

            // Frame pace
            float frame_pace_sum = 0;
            for (int i = 0; i < stats_.frame_counter; ++i) {
                frame_pace_sum += stats_.frame_pace_avg[i];
                if (stats_.frame_pace_avg[i] > stats_.frame_pace_max) {
                    stats_.frame_pace_max = stats_.frame_pace_avg[i];
                }
                if (stats_.frame_pace_avg[i] < stats_.frame_pace_min) {
                    stats_.frame_pace_min = stats_.frame_pace_avg[i];
                }
                osd_stats_.frame_pace.values.push_back(stats_.frame_pace_avg[i] / 1000.0);
            }
            osd_stats_.frame_pace.avg = frame_pace_sum / (stats_.frame_counter) / 1000.0;
            osd_stats_.frame_pace.max = stats_.frame_pace_max / 1000.0;
            osd_stats_.frame_pace.min = stats_.frame_pace_min / 1000.0;

            // Stats collected successfully

            // Worst frame (frame with max total latency)
            int worst_idx = 0;
            float max_total = 0;
            for (int i = 0; i < stats_.frame_counter; ++i) {
                if (stats_.total_latency_avg[i] > max_total) {
                    max_total = stats_.total_latency_avg[i];
                    worst_idx = i;
                }
            }
            osd_stats_.proc_latency_worst = stats_.proc_latency_avg[worst_idx] / 1000.0;
            osd_stats_.decoding_latency_worst = stats_.decoding_latency_avg[worst_idx] / 1000.0;
            osd_stats_.display_latency_worst = stats_.display_latency_avg[worst_idx] / 1000.0;
            osd_stats_.tx_latency_worst = stats_.tx_latency_avg[worst_idx] / 1000.0;
            
            if (osd) osd->update_stats((int)current_framerate, osd_stats_);
            
            // Pacing/Sync logic completely removed - relying on immediate submission
            if (current_framerate > 10) {
                 // update_stats heartbeat only
            }
        } else {
            // Even if zero frames, send current state to OSD to show 0 FPS
            if (osd) {
                // No latency sample in the window, pictures or not: the
                // decoded picture rate still stands.
                osd->update_stats((int)current_framerate, osd_stats_);
                
                // Maintain timeline speed: calculate how many frames "should" have happened 
                // in this windows to keep the 5s window consistent.
                int missing_frames = (120 * delta_us) / 1000000;
                if (missing_frames <= 0) missing_frames = 1;
                if (missing_frames > 120) missing_frames = 120; // Cap to 1s worth

                for (int i = 0; i < missing_frames; ++i) {
                    // osd->add_latency_frame({0,0,0,0,0,0});
                }
            }
        }
        reset_stats(now);
    }

    // A picture timed slice by slice gets its sample from the flip that put
    // it up, once all its slices are decoded (below). Others, here: their
    // pieces as one, the display figure that of the last flip.
    // Also without flips that report vblanks (FrontBuffer, Disable): there the
    // slices' rows are never marked up, so the picture is timed whole.
    if (du && (!du->slices || render_mode != Atomic) && (int64_t)du->pts != last_whole_sample_pts_) {
        last_whole_sample_pts_ = (int64_t)du->pts;
        samples_whole_++;
        uint64_t dec_lat = du->dec_end_ts > du->recv_ts ? du->dec_end_ts - du->recv_ts : 0;
        
        uint64_t hw_offset_us = sensor_offset_us(du);

        // Without an air-side split, the nominal exposure + readout counts as the encoder's.
        // The air part not known (kAirUnknown): the ground's only, and marked so.
        const bool air = du->tx_processing_delay_us != kAirUnknown;
        uint64_t tx_enc = air ? du->tx_encode_delay_us + hw_offset_us : 0;
        uint64_t tx_proc = air ? du->tx_processing_delay_us : 0;
        
        // DEFAULT: Start with software submission time
        uint64_t disp_lat = (now > du->dec_end_ts) ? now - du->dec_end_ts : 0;
        uint64_t total_lat = (now > du->recv_ts ? now - du->recv_ts : 0) + tx_enc + tx_proc;

        // ACCURATE: Override with hardware-accurate VSync stats from previous flips if available
        CompletedStats hw_stats = dev->get_latest_stats();
        if (hw_stats.available) {
            disp_lat = hw_stats.display_latency_us;
            total_lat = hw_stats.total_latency_us;
        }
        unsigned skipped = 0;
        if (last_shown_pts_ >= 0 && (int64_t)du->pts > last_shown_pts_ + 1 && (int64_t)du->pts - last_shown_pts_ < 1000)
            skipped = (unsigned)((int64_t)du->pts - last_shown_pts_ - 1);
        if ((int64_t)du->pts > last_shown_pts_) last_shown_pts_ = (int64_t)du->pts;
        add_latency_sample(tx_enc, tx_proc, dec_lat, disp_lat, total_lat, du->is_keyframe, skipped,
                           nullptr, 0, nullptr, nullptr, 0, 0, air);
    }

    // Each slice on its own: from its first row's capture to the scan-out
    // lighting that row, through the flip that put that slice's rows up (a
    // picture shown in halves has its top and its rest go up at different
    // vblanks). The picture's figure is its slowest slice's - no slice waits
    // for another. Taken once every slice is decoded and on screen.
    std::vector<SliceFlip> fresh;
    dev->take_slice_flips(fresh);
    for (auto& f : fresh) {
        bool known = false;
        for (auto& w : slice_wait_) if (w.slices == f.slices) known = true;
        // Up again after its sample was taken (a re-flip keeping it on screen).
        if (!known && f.slices->counted.load()) { samples_again_++; known = true; }
        if (known) continue;
        // On screen for the first time: the pictures between it and the last
        // one that came up never did (flips come in order; samples need not).
        const SliceTimes &st = *f.slices;
        if (st.pts >= 0) {
            if (last_shown_pts_ >= 0 && st.pts > last_shown_pts_ + 1 && st.pts - last_shown_pts_ < 1000)
                f.skipped = (unsigned)(st.pts - last_shown_pts_ - 1);
            if (st.pts > last_shown_pts_) last_shown_pts_ = st.pts;
        }
        slice_wait_.push_back(std::move(f));
    }
    // Each picture on its own, once all its slices are decoded and on screen
    // (or 100 ms after it came up): one that is not done holds no other back.
    for (auto it = slice_wait_.begin(); it != slice_wait_.end();) {
        SliceFlip& f = *it;
        const SliceTimes& st = *f.slices;
        const int n = std::min(st.n.load(), SliceTimes::kMax);
        // ...and all of them there: a picture's second slice may not even have
        // arrived when its top goes up.
        bool all = n > 0 && st.complete.load();
        for (int i = 0; i < n; i++)
            if (!st.s[i].done_us.load() || !st.lit_vblank(st.s[i].row)) all = false;
        if (!all && now < f.vblank_us + 100000) { ++it; continue; }   // a slice still on its way
        if (!all) slice_timeouts_++;
        if (f.slices->counted.exchange(true)) { samples_again_++; it = slice_wait_.erase(it); continue; }
        // One copy of the mode for every row of this picture.
        const drmModeModeInfo mode = dev->current_mode();
        auto row_us = [mode](int r) { return DrmDevice::row_time_of(mode, r); };
        const double period = dev->frame_period_us();
        const int rows = mode.vdisplay;
        SliceLatency l;
        int which = 0;
        // Each slice on its own, top to bottom (the HUD and the stats in SPLIT;
        // the first and the last also for the log): kLatSlices of them at most,
        // spread from the first to the last when there are more.
        float slice_ms[kLatSlices] = {0, 0, 0, 0};
        SliceStages sl[kLatSlices];
        int nslices = 0;
        if (n >= 2) {
            const int m = std::min(n, kLatSlices);
            bool all = true;
            for (int k = 0; k < m; k++) {
                const int i = m == n ? k : (int)std::lround((double)k * (n - 1) / (m - 1));
                SliceLatency e;
                if (!slice_latency(st, i, period, rows, row_us, e)) { all = false; continue; }
                slice_ms[k] = e.total() / 1000.0f;
                sl[k].off = st.s[i].cap_us > st.frame_cap_us ? (st.s[i].cap_us - st.frame_cap_us) / 1000.0f : 0.0f;
                sl[k].enc = e.enc / 1000.0f;
                sl[k].rf = e.rf / 1000.0f;
                sl[k].dec = e.dec / 1000.0f;
                sl[k].dec_wait = e.dec_wait / 1000.0f;
                sl[k].disp = e.disp / 1000.0f;
                sl[k].disp_wait = e.disp_wait / 1000.0f;
                sl[k].enc_read = e.enc_read / 1000.0f;
                sl[k].exp = e.exposure / 1000.0f;
                if (i == 0) slice_log_.v[0].push_back(e);
                if (i == n - 1) slice_log_.v[1].push_back(e);
            }
            nslices = all ? m : 0;
        }
        if (slowest_slice(st, period, rows, row_us, l, &which)) {
            add_latency_sample(l.enc, l.rf, l.dec, l.disp, l.total(), st.key, skipped_pending_ + f.skipped,
                               slice_ms, nslices, &l, sl, st.lit_vblank(0), st.frame_cap_us);
            skipped_pending_ = 0;
            samples_sliced_++;
            slice_log_.slowest[which == 0 ? 0 : which == n - 1 ? 2 : 1]++;
        } else if (whole_picture_sample(st, period, rows, row_us, skipped_pending_ + f.skipped)) {
            skipped_pending_ = 0;
            samples_whole_++;
        } else {
            skipped_pending_ += f.skipped;     // goes with the next sample
            // Why not: 0 no slice, 1 a slice not decoded, 2 rows never up,
            // 3 capture/encoder stamps missing or out of order, 4 decoded after 4 refreshes up
            int why = n == 0 ? 0 : 4;
            for (int i = 0; i < n && why == 4; i++) {
                const SliceTimes::Slice &s = st.s[i];
                if (!s.done_us.load()) why = 1;
                else if (!st.lit_vblank(s.row)) why = 2;
                else if (!s.cap_us || !s.out_us || !s.here_us || s.out_us < s.cap_us ||
                         s.here_us < s.out_us || s.done_us.load() < s.here_us) why = 3;
            }
            slice_fail_[why]++;
        }
        it = slice_wait_.erase(it);
    }
    // Every 5 s: the top and bottom slice each on its own, medians.
    if (!slice_log_.since) slice_log_.since = now;
    if (now - slice_log_.since >= 5000000) {
        auto med = [](std::vector<SliceLatency>& v, uint32_t SliceLatency::*m) {
            if (v.empty()) return 0.0;
            std::vector<uint32_t> x;
            for (auto& e : v) x.push_back(e.*m);
            std::nth_element(x.begin(), x.begin() + x.size() / 2, x.end());
            return x[x.size() / 2] / 1000.0;
        };
        auto med_total = [](std::vector<SliceLatency>& v) {
            if (v.empty()) return 0.0;
            std::vector<uint32_t> x;
            for (auto& e : v) x.push_back(e.total());
            std::nth_element(x.begin(), x.begin() + x.size() / 2, x.end());
            return x[x.size() / 2] / 1000.0;
        };
        for (int k = 0; k < 2; k++) {
            auto& v = slice_log_.v[k];
            if (!v.empty())
                printf("renderer: %s slice, capture of its first row -> lit, %zu pictures: "
                       "enc %.2f (sensor read %.2f) rf %.2f dec %.2f (wait %.2f) disp %.2f (wait %.2f) = %.2f ms (medians), "
                       "exposure before it %.2f\n",
                       k ? "last" : "first", v.size(), med(v, &SliceLatency::enc),
                       med(v, &SliceLatency::enc_read), med(v, &SliceLatency::rf),
                       med(v, &SliceLatency::dec), med(v, &SliceLatency::dec_wait),
                       med(v, &SliceLatency::disp), med(v, &SliceLatency::disp_wait), med_total(v),
                       med(v, &SliceLatency::exposure));
            v.clear();
        }
        if (slice_log_.slowest[0] + slice_log_.slowest[1] + slice_log_.slowest[2])
            printf("renderer: slowest slice: the first %u, a middle one %u, the last %u times\n",
                   slice_log_.slowest[0], slice_log_.slowest[1], slice_log_.slowest[2]);
        if (slice_fail_[0] + slice_fail_[1] + slice_fail_[2] + slice_fail_[3] + slice_fail_[4])
            printf("renderer: shown pictures with no latency figure: %u no slice, %u a slice not decoded, "
                   "%u rows never up, %u stamps missing or out of order, %u decoded 4+ refreshes late\n",
                   slice_fail_[0], slice_fail_[1], slice_fail_[2], slice_fail_[3], slice_fail_[4]);
        for (unsigned &c : slice_fail_) c = 0;
        printf("renderer: latency samples in 5 s: %u timed slice by slice, %u as whole pictures, "
               "%u pictures up again not counted again, %u taken at 100 ms with a slice missing\n",
               samples_sliced_, samples_whole_, samples_again_, slice_timeouts_);
        samples_sliced_ = samples_whole_ = samples_again_ = slice_timeouts_ = 0;
        slice_log_.slowest[0] = slice_log_.slowest[1] = slice_log_.slowest[2] = 0;
        slice_log_.since = now;
    }
}

// A picture whose slices have no encoder-out stamps (an air unit whose headers
// lack them): timed as one piece, from its capture (row 0) to its first row
// lit - its first slice's arrival, decode and display, encoder share unknown.
template <typename RowTime>
bool Renderer::whole_picture_sample(const SliceTimes &st, double period, int rows, RowTime row_us,
                                    unsigned skipped) {
    if (st.n.load() < 1 || !st.frame_cap_us) return false;
    const SliceTimes::Slice &s = st.s[0];
    const uint64_t done = s.done_us.load(), vb = st.lit_vblank(s.row);
    if (!s.here_us || !done || !vb || s.here_us < st.frame_cap_us || done < s.here_us) return false;
    double lit = (double)vb + row_us((int)((uint64_t)s.row * rows / std::max<uint32_t>(1, st.height)));
    for (int k = 0; k < 4 && period > 0 && (double)done > lit; k++) lit += period;
    if ((double)done > lit) return false;
    const uint64_t rf = s.here_us - st.frame_cap_us, dec = done - s.here_us, disp = (uint64_t)(lit - (double)done);
    add_latency_sample(0, rf, dec, disp, rf + dec + disp, st.key, skipped, nullptr, 0, nullptr, nullptr,
                       st.lit_vblank(0), st.frame_cap_us);
    return true;
}

void Renderer::add_latency_sample(uint64_t tx_enc, uint64_t tx_proc, uint64_t dec_lat,
                                  uint64_t disp_lat, uint64_t total_lat, bool key, unsigned skipped,
                                  const float* slice_ms, int nslices, const SliceLatency* waits,
                                  const SliceStages* sl, uint64_t shown_us, uint64_t cap_us, bool air) {
    if (stats_.frame_counter >= Stats::kMaxFrames) return;
    const uint64_t now = get_time_us();
    uint64_t pace_lat = (stats_.last_frame_ts > 0 && now > stats_.last_frame_ts) ? now - stats_.last_frame_ts : 0;

    // Sanity check: if any latency is more than 1 second, it's likely a wrap-around or clock jump
    if (dec_lat > 1000000) dec_lat = 0;
    if (disp_lat > 1000000) disp_lat = 0;
    if (total_lat > 5000000) total_lat = 0; // 5s for total is generous but safer
    // Sanity check TX-side timing: a bad clock sync can produce absurd tx_enc/tx_proc values
    // (e.g. when clock_offset_us is corrupt). Cap to 1 second each.
    if (tx_enc  > 1000000) tx_enc  = 0;
    if (tx_proc > 1000000) tx_proc = 0;

    stats_.proc_latency_avg[stats_.frame_counter] = tx_proc; // RF: encoder out -> here
    stats_.decoding_latency_avg[stats_.frame_counter] = dec_lat; // here -> decoded
    stats_.display_latency_avg[stats_.frame_counter] = disp_lat;
    stats_.tx_latency_avg[stats_.frame_counter] = tx_enc; // capture -> encoder out
    stats_.total_latency_avg[stats_.frame_counter] = total_lat;
    stats_.frame_pace_avg[stats_.frame_counter] = pace_lat;
    stats_.last_frame_ts = now;

    // Feed the per-frame graph
    if (osd) {
        LatencyFrame lf = {
            .processing_ms = (float)tx_enc / 1000.0f,
            .net_ms = (float)tx_proc / 1000.0f,
            .dec_ms = (float)dec_lat / 1000.0f,
            .disp_ms = (float)disp_lat / 1000.0f,
            .key = (uint8_t)(key ? 1 : 0),
            .skipped = (uint16_t)std::min(skipped, 65535u),
        };
        lf.shown_us = shown_us;
        lf.cap_us = cap_us;
        lf.air = air ? 1 : 0;
        if (waits) {
            lf.processing_wait_ms = waits->enc_wait / 1000.0f;
            lf.net_wait_ms = waits->rf_wait / 1000.0f;
            lf.dec_wait_ms = waits->dec_wait / 1000.0f;
            lf.disp_wait_ms = waits->disp_wait / 1000.0f;
        }
        if (slice_ms && nslices >= 2 && nslices <= kLatSlices) {
            lf.nslices = (uint8_t)nslices;
            for (int k = 0; k < nslices; k++) lf.slice_ms[k] = slice_ms[k];
            if (sl)
                for (int k = 0; k < nslices; k++) lf.sl[k] = sl[k];
        }
        osd->add_latency_frame(lf);
    }

    stats_.frame_counter++; 
}
void Renderer::present_frame(uint32_t fb_id, uint32_t width, uint32_t height, uint64_t pts, uint64_t recv_ts, uint64_t dec_start_ts, uint64_t dec_end_ts, uint32_t tx_age_us) {
    uint64_t render_start_ts = get_time_us();
    if (width != dev->video_frm_width || height != dev->video_frm_height) {
        dev->set_frame_size(width, height);
        dev->perform_modeset_video(fb_id);
        if(osd) osd->set_video_resolution(width, height);
    }
    bool submitted = false;
    if (render_mode == Atomic) {
        uint64_t hw_offset_us = 0;
        if (osd) {
            uint32_t exp_us = osd->get_sky_exposure_us();
            uint32_t fps = osd->get_sky_framerate();
            if (fps > 0) {
                uint32_t readout_us = 1000000 / fps;
                hw_offset_us = (uint64_t)exp_us + readout_us;
            }
        }
        submitted = dev->page_flip(fb_id, recv_ts, dec_start_ts, dec_end_ts, render_start_ts, tx_age_us + hw_offset_us); 
    }
    if (osd) osd->signal_render();
}
