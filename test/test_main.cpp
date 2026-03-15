// test/test_main.cpp
// Usage: evc_test <evio_file> [max_events]

#include "EvChannel.h"
#include "Fadc250Decoder.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>

using namespace evc;

static void hexdump(const uint8_t *p, size_t n, size_t max = 128)
{
    size_t show = std::min(n, max);
    for (size_t i = 0; i < show; ++i) {
        if (i && i % 16 == 0) std::cout << "\n      ";
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)p[i] << " ";
    }
    if (n > show) std::cout << "...";
    std::cout << std::dec << std::setfill(' ');
}

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

    int n = 0;
    while (ch.Read() == status::success) {
        ++n;
        if (!ch.Scan()) continue;
        auto hdr = ch.GetEvHeader();

        std::cout << "=== Event " << n
                  << "  tag=0x" << std::hex << hdr.tag << std::dec
                  << "  num=" << hdr.num
                  << "  length=" << hdr.length << " ===\n";

        // print tree
        ch.PrintTree(std::cout);

        // for each composite bank, dump raw bytes and try decode
        for (auto *node : ch.FindByTag(0xe101)) {
            size_t nbytes;
            auto *payload = ch.GetCompositePayload(*node, nbytes);
            if (!payload) continue;

            // find the parent ROC bank
            int pi = node->parent;
            uint32_t roc_tag = (pi >= 0) ? ch.GetNodes()[pi].tag : 0;

            std::cout << "  >> Composite 0xe101 in ROC 0x" << std::hex << roc_tag << std::dec
                      << "  payload=" << nbytes << " bytes\n";
            std::cout << "     raw: ";
            hexdump(payload, nbytes, 64);
            std::cout << "\n";

            auto slots = fdec::Fadc250Decoder::Decode(payload, nbytes);
            if (slots.empty()) {
                std::cout << "     (no slots decoded)\n";
            }
            for (auto &s : slots) {
                std::cout << "     slot=" << (int)s.slot
                          << " trig=" << s.trigger
                          << " ts=0x" << std::hex << (uint64_t)s.timestamp << std::dec
                          << " nch=" << s.channels.size() << " |";
                for (auto &c : s.channels)
                    std::cout << " ch" << (int)c.channel << ":" << c.samples.size();
                std::cout << "\n";
            }
        }

        std::cout << "\n";
        if (max_ev > 0 && n >= max_ev) break;
    }

    std::cout << "Done. " << n << " event(s).\n";
    ch.Close();
    return 0;
}
