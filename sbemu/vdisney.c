// Disney Sound Source / Covox Speech Thing emulation for SBEMU
//
// How it works:
//   - Parallel port DAC. Programs write 8-bit PCM samples to LPT1 data port (0x378).
//   - The hardware has a 16-byte FIFO. Status port bit 6 = "FIFO not full" (1 = not full).
//   - Stereo: control port bit1 falling edge = right latch, bit0 falling edge = left latch.
//   - Detection: games read 0x379, bit7 mirrors bit7 of data byte (inverted on real HW).
//
// SBEMU integration:
//   - Trap ports 0x378, 0x278, 0x3BC (LPT1, LPT2, LPT3).
//   - Use a CIRCULAR ring buffer per channel. IO writes push to the ring.
//   - GenSamples drains the ring at a *fixed* rate with linear interpolation.
//   - Fixed rate = 7000 Hz (Disney FIFO clock). Games that use Covox directly
//     write faster; we handle that via the adaptive sample-rate estimator below.

#include "platform.h"
#include "vdisney.h"
#include <string.h>
#include <stdlib.h>

#if SBEMU_DISNEY

// ---------------------------------------------------------------------------
// Ring buffer
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  buf[VDISNEY_FIFO];
    int      head;       // next read position
    int      tail;       // next write position
    int      count;      // bytes currently in buffer
    uint32_t total_written;
} VDISNEY_Ring;

static inline void ring_push(VDISNEY_Ring *r, uint8_t v)
{
    // Overwrite oldest if full (prevents blocking TSR)
    if(r->count == VDISNEY_FIFO) {
        r->head = (r->head + 1) & (VDISNEY_FIFO - 1);
        r->count--;
    }
    r->buf[r->tail] = v;
    r->tail = (r->tail + 1) & (VDISNEY_FIFO - 1);
    r->count++;
    r->total_written++;
}

// Peek at offset i from head (no bounds check - caller must verify)
static inline uint8_t ring_peek(VDISNEY_Ring *r, int i)
{
    return r->buf[(r->head + i) & (VDISNEY_FIFO - 1)];
}

static inline void ring_consume(VDISNEY_Ring *r, int n)
{
    if(n > r->count) n = r->count;
    r->head  = (r->head + n) & (VDISNEY_FIFO - 1);
    r->count -= n;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static struct {
    uint8_t       data;        // last byte written to data port
    uint8_t       control;     // last byte written to control port
    int           interface_ext; // FIFO-mode detected (bit3 strobe counter)

    VDISNEY_Ring  ch[2];       // ch[0]=left/mono, ch[1]=right

    int           stereo;
    int           leader;      // which channel drives timing (0 or 1)

    // Adaptive sample rate
    uint32_t      last_total;  // total_written snapshot at last GenSamples
    int           rate_est;    // current estimated sample rate (Hz)

    // Fixed-point resampling accumulator
    uint32_t      phase_fp;    // fractional position (16.16)

    int           initialized;
    int           running;

    int           output_freq;
} vds;

// ---------------------------------------------------------------------------
// Conversion: unsigned 8-bit PCM -> signed 16-bit
// ---------------------------------------------------------------------------
static inline int16_t u8_to_s16(uint8_t v)
{
    return (int16_t)(((int)v - 128) * 256);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void VDISNEY_Init(int output_freq)
{
    memset(&vds, 0, sizeof(vds));
    vds.output_freq = output_freq;
    vds.rate_est    = 7000;   // Default Disney FIFO clock
    vds.initialized = 1;
}

void VDISNEY_Shutdown(void)
{
    vds.initialized = 0;
}

int VDISNEY_IsActive(void)
{
    return vds.initialized;
}

// ---------------------------------------------------------------------------
// IO Handler
// ---------------------------------------------------------------------------
uint32_t VDISNEY_IOHandler(uint32_t port, uint32_t val, uint32_t out)
{
    if(!vds.initialized) return val;

    // Suporta LPT1 (0x378), LPT2 (0x278) e LPT3 (0x3BC)
    static const uint16_t bases[] = {0x378, 0x278, 0x3BC};
    int base = -1;
    int offset = -1;

    for(int i = 0; i < 3; i++) {
        if(port >= bases[i] && port <= bases[i] + 2) {
            base = bases[i];
            offset = (int)(port - base);
            break;
        }
    }

    // Se não for uma porta LPT conhecida, retorna sem interceptar
    if(base == -1) return val;

    if(out)
    {
        switch(offset)
        {
            case 0: // Data port (base+0)
                vds.data = (uint8_t)(val & 0xFF);
                ring_push(&vds.ch[0], vds.data);
                vds.running = 1;
                break;

            case 1: // Status port (base+1) — read-only, ignore writes
                break;

            case 2: // Control port (base+2)
            {
                uint8_t prev = vds.control;
                uint8_t cur  = (uint8_t)(val & 0xFF);

                // Bit1 falling edge -> right channel
                if((prev & 0x02) && !(cur & 0x02)) {
                    ring_push(&vds.ch[1], vds.data);
                    vds.running = 1;
                }
                // Bit0 falling edge -> left channel
                if((prev & 0x01) && !(cur & 0x01)) {
                    ring_push(&vds.ch[0], vds.data);
                    vds.running = 1;
                }
                // Bit3 falling edge -> FIFO-mode device
                if((prev & 0x08) && !(cur & 0x08)) {
                    vds.interface_ext++;
                }

                vds.control = cur;
                break;
            }
        }
    }
    else
    {
        switch(offset)
        {
            case 0: // Data read (unusual but possible)
                return vds.data;

            case 1: // Status read
            {
                uint8_t ret = 0x07;
                if(vds.interface_ext > 5) {
                    if(vds.ch[vds.leader].count >= 16)
                        ret |= 0x40; // FIFO full
                    else
                        ret &= ~0x04;
                }
                if(!(vds.data & 0x80)) ret |= 0x80;
                return ret;
            }

            case 2: // Control read
                return vds.control;
        }
    }
    return val;
}

// ---------------------------------------------------------------------------
// Audio generation
// ---------------------------------------------------------------------------
void VDISNEY_GenSamples(int16_t *pcm16, int samples, int freq, int domix)
{
    if(!vds.initialized || !vds.running) return;

    // --- 1. Detect stereo ---
    vds.stereo = (vds.ch[1].total_written > 32) ? 1 : 0;
    vds.leader = 0;

    // --- 2. Silence / timeout check ---
    uint32_t new_writes = vds.ch[vds.leader].total_written - vds.last_total;
    vds.last_total = vds.ch[vds.leader].total_written;

    if(vds.ch[vds.leader].count == 0 && new_writes == 0) {
        vds.running = 0;
        vds.rate_est = 7000; // reset rate for next session
        if(!domix) memset(pcm16, 0, samples * 2 * sizeof(int16_t));
        return;
    }

    // --- 3. Stable rate via buffer fill level feedback ---
    int fill = vds.ch[vds.leader].count;

    // Only update rate if there's enough data to avoid reacting to empty buffer
    if(fill > 4) {
        int fill_rate = (int)((double)fill * freq / samples);
        // Clamp to reasonable audio range (minimum raised to 6000 Hz)
        if(fill_rate < 6000)  fill_rate = 6000;
        if(fill_rate > 50000) fill_rate = 50000;

        // More aggressive smoothing: 1/2 old + 1/2 new
        vds.rate_est = (vds.rate_est + fill_rate) / 2;
    }

    // Final clamp (also raised minimum)
    if(vds.rate_est < 6000)  vds.rate_est = 6000;
    if(vds.rate_est > 50000) vds.rate_est = 50000;

    // --- 4. Resample with linear interpolation ---
    uint32_t step_fp = (uint32_t)((double)vds.rate_est / freq * 65536.0 + 0.5);

    for(int i = 0; i < samples; i++)
    {
        int s0   = (int)(vds.phase_fp >> 16);
        int frac = (int)(vds.phase_fp & 0xFFFF);

        uint8_t a0_l, a1_l, a0_r, a1_r;

        if(vds.stereo) {
            a0_l = (vds.ch[0].count > s0)     ? ring_peek(&vds.ch[0], s0)     :
                   (vds.ch[0].count > 0)        ? ring_peek(&vds.ch[0], vds.ch[0].count-1) : 128;
            a1_l = (vds.ch[0].count > s0 + 1)  ? ring_peek(&vds.ch[0], s0 + 1) : a0_l;
            a0_r = (vds.ch[1].count > s0)     ? ring_peek(&vds.ch[1], s0)     :
                   (vds.ch[1].count > 0)        ? ring_peek(&vds.ch[1], vds.ch[1].count-1) : 128;
            a1_r = (vds.ch[1].count > s0 + 1)  ? ring_peek(&vds.ch[1], s0 + 1) : a0_r;
        } else {
            a0_l = a0_r = (vds.ch[0].count > s0)     ? ring_peek(&vds.ch[0], s0)     :
                           (vds.ch[0].count > 0)        ? ring_peek(&vds.ch[0], vds.ch[0].count-1) : 128;
            a1_l = a1_r = (vds.ch[0].count > s0 + 1)  ? ring_peek(&vds.ch[0], s0 + 1) : a0_l;
        }

        int32_t sl_raw = ((int32_t)a0_l * (65536 - frac) + (int32_t)a1_l * frac) >> 16;
        int32_t sr_raw = ((int32_t)a0_r * (65536 - frac) + (int32_t)a1_r * frac) >> 16;

        int16_t sl = u8_to_s16((uint8_t)sl_raw);
        int16_t sr = u8_to_s16((uint8_t)sr_raw);

        if(domix) {
            int32_t nl = (int32_t)pcm16[i*2]   + sl;
            int32_t nr = (int32_t)pcm16[i*2+1] + sr;
            pcm16[i*2]   = nl >  32767 ?  32767 : nl < -32768 ? -32768 : (int16_t)nl;
            pcm16[i*2+1] = nr >  32767 ?  32767 : nr < -32768 ? -32768 : (int16_t)nr;
        } else {
            pcm16[i*2]   = sl;
            pcm16[i*2+1] = sr;
        }

        vds.phase_fp += step_fp;
    }

    // --- 5. Consume processed source samples ---
    int desired_consume = (int)(vds.phase_fp >> 16);
    int max_consume = (vds.stereo) ? 
        (vds.ch[0].count < vds.ch[1].count ? vds.ch[0].count : vds.ch[1].count) :
        vds.ch[0].count;

    int consumed = (desired_consume > max_consume) ? max_consume : desired_consume;

    if(consumed < desired_consume) {
        // Buffer underflow: reset phase to start fresh next block
        vds.phase_fp = 0;
    } else {
        // Remove the integer part consumed, keep fractional part for continuity
        vds.phase_fp -= (uint32_t)consumed * 65536;
    }

    ring_consume(&vds.ch[0], consumed);
    ring_consume(&vds.ch[1], consumed);
}

#endif // SBEMU_DISNEY