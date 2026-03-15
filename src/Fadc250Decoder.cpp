#include "Fadc250Decoder.h"
#include <cstring>
#include <iostream>

using namespace fdec;

std::vector<SlotData> Fadc250Decoder::Decode(const uint8_t *data, size_t nbytes)
{
    std::vector<SlotData> slots;
    if (!data || !nbytes) return slots;

    size_t pos = 0;
    while (pos + 17 <= nbytes) {       // c(1)+i(4)+l(8)+N(4)
        SlotData s;
        s.slot = data[pos]; pos += 1;
        std::memcpy(&s.trigger,   data + pos, 4); pos += 4;
        std::memcpy(&s.timestamp, data + pos, 8); pos += 8;

        uint32_t nchan;
        std::memcpy(&nchan, data + pos, 4); pos += 4;

        if (nchan > 16) {
            std::cerr << "Fadc250Decoder: bad nchan=" << nchan
                      << " slot=" << (int)s.slot << " pos=" << pos - 4 << "/" << nbytes << "\n";
            break;
        }

        s.channels.resize(nchan);
        for (uint32_t i = 0; i < nchan; ++i) {
            if (pos + 5 > nbytes) break;
            s.channels[i].channel = data[pos]; pos += 1;

            uint32_t nsamp;
            std::memcpy(&nsamp, data + pos, 4); pos += 4;

            if (nsamp > 4096) {
                std::cerr << "Fadc250Decoder: bad nsamp=" << nsamp << "\n";
                return slots;
            }

            size_t bytes = size_t(nsamp) * 2;
            if (pos + bytes > nbytes) { nsamp = (nbytes - pos) / 2; bytes = nsamp * 2; }

            s.channels[i].samples.resize(nsamp);
            std::memcpy(s.channels[i].samples.data(), data + pos, bytes);
            pos += bytes;
        }
        slots.push_back(std::move(s));
    }
    return slots;
}
