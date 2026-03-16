// test/test_main.cpp
// Usage: evc_test <evio_file> [max_events]

#include "EvChannel.h"
#include "Fadc250Data.h"
#include <iostream>
#include <cstdlib>

using namespace evc;

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <evio_file> [max_events]\n";
        return 1;
    }
    int max_ev = (argc >= 3) ? std::atoi(argv[2]) : 0;

    EvChannel ch;
    if (ch.Open(argv[1]) != status::success) {
        std::cerr << "Failed to open " << argv[1] << "\n";
        return 1;
    }

    // pre-allocate once, reuse for every event
    fdec::EventData event;
    int total = 0;

    while (ch.Read() == status::success) {
        if (!ch.Scan()) continue;
        auto hdr = ch.GetEvHeader();

        // skip non-physics events
        if (hdr.tag != 0xfe && !(hdr.tag >= 0xFF50 && hdr.tag <= 0xFF8F))
            continue;

        int nevt = ch.GetNEvents();
        for (int i = 0; i < nevt; ++i) {
            if (!ch.DecodeEvent(i, event)) continue;
            ++total;

            std::cout << "Event " << total << ": " << event.nrocs << " ROCs\n";
            for (int r = 0; r < event.nrocs; ++r) {
                auto &roc = event.rocs[r];
                std::cout << "  ROC 0x" << std::hex << roc.tag << std::dec
                          << " (" << roc.nslots << " slots)\n";

                for (int s = 0; s < fdec::MAX_SLOTS; ++s) {
                    if (!roc.slots[s].present) continue;
                    auto &slot = roc.slots[s];

                    std::cout << "    slot=" << s
                              << " ev=" << slot.trigger
                              << " ts=0x" << std::hex << (uint64_t)slot.timestamp << std::dec
                              << " |";

                    for (int c = 0; c < fdec::MAX_CHANNELS; ++c) {
                        if (!(slot.channel_mask & (1u << c))) continue;
                        std::cout << " ch" << c << ":" << slot.channels[c].nsamples;
                    }
                    std::cout << "\n";
                }
            }

            if (max_ev > 0 && total >= max_ev) goto done;
        }
    }
done:
    std::cout << "Done. " << total << " event(s) decoded.\n";
    ch.Close();
    return 0;
}
