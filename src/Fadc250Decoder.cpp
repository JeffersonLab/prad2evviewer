//
// A simple decoder to process the JLab FADC250 data
// Reference: https://www.jlab.org/Hall-B/ftof/manuals/FADC250UsersManual.pdf
//
// Author: Chao Peng
// Date: 2020/08/22
//
// Updated: 2025 - Added composite data type decoding
//   tag 0xe101 format "c,i,l,N(c,Ns)" — packed, native-endian
//

#include "Fadc250Decoder.h"

using namespace fdec;

#define SET_BIT(n,i)  ( (n) |= (1ULL << i) )

inline void print_word(uint32_t word)
{
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << word << std::dec << "\n";
}

inline Fadc250Data &get_channel(Fadc250Event &ev, uint32_t ch)
{
    return ev.channels[ch];
}

template<class Container>
inline uint32_t fill_in_words(const uint32_t *buf, size_t beg, Container &raw_data, size_t max_words = -1)
{
    uint32_t nwords = 0;
    for (uint32_t i = beg + 1; raw_data.size() < max_words; ++i, ++nwords) {
        auto data = buf[i];
        if ((data & 0x80000000) && nwords > 0) {
            return nwords;
        }
        if (!(data & 0x20000000)) {
            raw_data.push_back((data >> 16) & 0x1FFF);
        }
        if (!(data & 0x2000)) {
            raw_data.push_back((data & 0x1FFF));
        }
    }
    return nwords;
}


Fadc250Decoder::Fadc250Decoder(double clk)
: _clk(clk)
{
}

Fadc250Event Fadc250Decoder::DecodeEvent(const uint32_t *buf, size_t len, size_t nchans) const
{
    Fadc250Event evt;
    evt.channels.resize(nchans);
    DecodeEvent(evt, buf, len);
    return evt;
}

// ===================================================================
//  Legacy decoder: raw FADC250 module words
// ===================================================================
struct PeakBuffer {
    uint32_t height = 0, integral = 0, time = 0;
    bool in_data = false;
};

void Fadc250Decoder::DecodeEvent(Fadc250Event &res, const uint32_t *buf, size_t buflen) const
{
    res.Clear();
    if (!buflen) return;

    auto header = buf[0];
    if (!(header & 0x80000000) || ((header >> 27) & 0xF) != EventHeader) {
        std::cout << "Fadc250Decoder Error: incorrect event header:";
        print_word(buf[0]);
        return;
    }

    res.number = (header & 0x3FFFFF);
    std::vector<std::vector<PeakBuffer>> peak_buffers(res.channels.size());
    uint32_t type = FillerWord;

    for (size_t iw = 1; iw < buflen; ++iw) {
        uint32_t data = buf[iw];
        bool new_type = (data & 0x80000000);
        if (new_type) {
            type = (data >> 27) & 0xF;
            SET_BIT(res.mode, type);
        }

        switch (type) {
        case TriggerTime:
            res.time.push_back(data & 0xFFFFFF);
            break;
        case WindowRawData:
            if (new_type) {
                uint32_t ch = (data >> 23) & 0xF;
                size_t nwords = (data & 0xFFF);
                auto &raw_data = get_channel(res, ch).raw;
                raw_data.clear();
                iw += fill_in_words(buf, iw, raw_data, nwords);
            }
            break;
        case PulseRawData:
            break;
        case PulseIntegral:
            {
                uint32_t ch = (data >> 23) & 0xF;
                uint32_t pulse_num = (data >> 21) & 0x3;
                if (peak_buffers[ch].size() < pulse_num + 1)
                    peak_buffers[ch].resize(4);
                peak_buffers[ch][pulse_num].integral = data & 0x7FFFF;
                peak_buffers[ch][pulse_num].in_data = true;
            }
            break;
        case PulseTime:
            {
                uint32_t ch = (data >> 23) & 0xF;
                uint32_t pulse_num = (data >> 21) & 0x3;
                if (peak_buffers[ch].size() < pulse_num + 1)
                    peak_buffers[ch].resize(4);
                peak_buffers[ch][pulse_num].time = data & 0xFFFF;
                peak_buffers[ch][pulse_num].in_data = true;
            }
            break;
        case Scaler:
        case InvalidData:
        case FillerWord:
            break;
        default:
            return;
        }
    }

    for (size_t i = 0; i < peak_buffers.size(); ++i) {
        for (auto &peak : peak_buffers[i]) {
            if (!peak.in_data) continue;
            res.channels[i].peaks.emplace_back(
                static_cast<double>(peak.height),
                static_cast<double>(peak.integral),
                static_cast<double>(peak.time) * 15.625 / _clk);
        }
    }
}


// ===================================================================
//  Composite decoder: format "c,i,l,N(c,Ns)"
//
//  Packed byte stream, native-endian (as returned by evRead on LE host).
//  Multiple slots are stored back-to-back in one payload.
// ===================================================================

std::vector<CompositeSlot>
Fadc250Decoder::DecodeComposite(const uint8_t *data, size_t nbytes) const
{
    std::vector<CompositeSlot> slots;
    if (!data || nbytes == 0) return slots;

    size_t pos = 0;

    while (pos < nbytes) {
        // Need at least: c(1) + i(4) + l(8) + N(4) = 17 bytes for slot header
        if (pos + 17 > nbytes) break;

        CompositeSlot s;

        // c -> slot (1 byte)
        s.slot = data[pos]; pos += 1;

        // i -> trigger (4 bytes, native endian)
        std::memcpy(&s.trigger, data + pos, 4); pos += 4;

        // l -> timestamp (8 bytes, native endian)
        std::memcpy(&s.timestamp, data + pos, 8); pos += 8;

        // N -> nChannels (4 bytes, native endian)
        uint32_t nchan;
        std::memcpy(&nchan, data + pos, 4); pos += 4;

        s.channels.reserve(nchan);

        for (uint32_t ich = 0; ich < nchan; ++ich) {
            // c -> channel number (1 byte)
            if (pos + 1 + 4 > nbytes) break;
            uint8_t ch_num = data[pos]; pos += 1;

            // N -> nSamples (4 bytes, native endian)
            uint32_t nsamp;
            std::memcpy(&nsamp, data + pos, 4); pos += 4;

            Fadc250Data chdata;
            size_t sample_bytes = static_cast<size_t>(nsamp) * 2;
            if (pos + sample_bytes > nbytes) {
                nsamp = static_cast<uint32_t>((nbytes - pos) / 2);
                sample_bytes = nsamp * 2;
            }

            chdata.raw.resize(nsamp);
            // s -> samples (2 bytes each, native endian)
            std::memcpy(chdata.raw.data(), data + pos, sample_bytes);
            pos += sample_bytes;

            s.channels.emplace_back(ch_num, std::move(chdata));
        }

        slots.push_back(std::move(s));
    }

    return slots;
}
