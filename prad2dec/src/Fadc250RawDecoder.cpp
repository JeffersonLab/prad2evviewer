#include "Fadc250RawDecoder.h"
#include <iostream>

using namespace fdec;

//=============================================================================
// FADC250 hardware-format word types (bits 31:27 = type tag)
//
// From JLab FADC250 firmware / rol1.c:
//   0x00 (0):  Block Header   — slot(5), module_id(4), block#(10), nevents(8)
//   0x01 (1):  Block Trailer  — slot(5), nwords(22)
//   0x02 (2):  Event Header   — slot(5), trigger#(22)
//   0x03 (3):  Trigger Time   — time_low(24); continuation: time_high(24)
//   0x04 (4):  Window Raw Data— channel(4), window_width(12)
//              then (width+1)/2 continuation words, 2 ADC samples each
//   0x1F (31): Filler         — skip
//
// Sample data words (continuation after type 0x04 header):
//   Bits 28:16 = ADC sample i   (13 bits, 12-bit value + valid flag at bit 29)
//   Bits 12:0  = ADC sample i+1 (13 bits, 12-bit value + valid flag at bit 13)
//   Bit 31     = 1 for last sample word, 0 for more
//=============================================================================

namespace {

inline uint32_t type_tag(uint32_t w) { return (w >> 27) & 0x1F; }

// Block Header
inline uint32_t bh_slot(uint32_t w)     { return (w >> 22) & 0x1F; }

// Event Header
inline uint32_t eh_trigger(uint32_t w)  { return w & 0x003FFFFF; }

// Trigger Time
inline uint32_t tt_time(uint32_t w)     { return w & 0x00FFFFFF; }

// Window Raw Data header
inline uint32_t wr_channel(uint32_t w)  { return (w >> 23) & 0x0F; }
inline uint32_t wr_width(uint32_t w)    { return w & 0x0FFF; }

// Sample extraction (13-bit ADC values, 2 per word)
inline uint16_t sample_hi(uint32_t w)   { return (w >> 16) & 0x1FFF; }
inline uint16_t sample_lo(uint32_t w)   { return w & 0x1FFF; }

} // anonymous namespace

int Fadc250RawDecoder::DecodeRoc(const uint32_t *data, size_t nwords, RocData &roc)
{
    if (!data || nwords == 0) return 0;

    int nslots = 0;
    int cur_slot = -1;
    SlotData *sd = nullptr;

    for (size_t i = 0; i < nwords; ++i) {
        uint32_t w = data[i];
        uint32_t tt = type_tag(w);

        switch (tt) {

        case 0x00: { // Block Header
            uint32_t slot_id = bh_slot(w);
            if (slot_id >= MAX_SLOTS) {
                std::cerr << "Fadc250RawDecoder: slot_id=" << slot_id
                          << " >= MAX_SLOTS\n";
                cur_slot = -1;
                sd = nullptr;
                break;
            }
            cur_slot = static_cast<int>(slot_id);
            sd = &roc.slots[cur_slot];
            sd->present = true;
            sd->nchannels = 0;
            sd->channel_mask = 0;
            nslots++;
            break;
        }

        case 0x01: { // Block Trailer
            cur_slot = -1;
            sd = nullptr;
            break;
        }

        case 0x02: { // Event Header
            if (!sd) break;
            sd->trigger = static_cast<int32_t>(eh_trigger(w));
            break;
        }

        case 0x03: { // Trigger Time
            if (!sd) break;
            uint64_t time_low = tt_time(w);
            // Continuation word has bit 31 = 0 and carries high 24 bits
            if (i + 1 < nwords && (data[i + 1] >> 31) == 0) {
                ++i;
                uint64_t time_high = tt_time(data[i]);
                sd->timestamp = static_cast<int64_t>(time_low | (time_high << 24));
            } else {
                sd->timestamp = static_cast<int64_t>(time_low);
            }
            break;
        }

        case 0x04: { // Window Raw Data header
            if (!sd) break;
            uint32_t ch = wr_channel(w);
            uint32_t width = wr_width(w);
            if (ch >= MAX_CHANNELS) break;

            ChannelData &cd = sd->channels[ch];
            uint32_t nsamp = 0;
            uint32_t max_samp = (width < static_cast<uint32_t>(MAX_SAMPLES))
                                ? width : static_cast<uint32_t>(MAX_SAMPLES);

            // Read (width+1)/2 continuation words, each packing 2 samples
            uint32_t nwords_expected = (width + 1) / 2;
            for (uint32_t j = 0; j < nwords_expected && i + 1 < nwords; ++j) {
                ++i;
                uint32_t sw = data[i];
                if (nsamp < max_samp) cd.samples[nsamp++] = sample_hi(sw);
                if (nsamp < max_samp) cd.samples[nsamp++] = sample_lo(sw);
            }

            cd.nsamples = static_cast<int>(nsamp);
            sd->channel_mask |= (1ull << ch);
            sd->nchannels++;
            break;
        }

        case 0x1F: // Filler — skip
            break;

        default:
            break;
        }
    }

    roc.nslots = nslots;
    return nslots;
}
