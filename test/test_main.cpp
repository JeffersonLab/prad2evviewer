// test/test_main.cpp
// Usage:
//   evc_test <evio_file> [max_events]
//   evc_test --et <ip> <port> <et_file> <station>

#include "EvChannel.h"
#include "EtChannel.h"
#include "Fadc250Decoder.h"
#include <iostream>
#include <string>
#include <cstdlib>

using namespace evc;

static constexpr uint32_t TAG_COMPOSITE = 0xe101;

static void usage(const char *prog)
{
    std::cerr << "Usage:\n"
              << "  " << prog << " <evio_file> [max_events]\n"
              << "  " << prog << " --et <ip> <port> <et_file> <station>\n";
}

static void processEvent(EvChannel &ch, const fdec::Fadc250Decoder &decoder, int iev)
{
    auto hdr = ch.GetEvHeader();
    std::cout << "Event " << iev
              << "  tag=0x" << std::hex << hdr.tag << std::dec
              << "  length=" << hdr.length;

    if (!ch.ScanBanks({TAG_COMPOSITE})) {
        std::cout << "\n";
        return;
    }

    auto &composites = ch.GetCompositeInfos();
    std::cout << "  composites=" << composites.size() << "\n";

    for (auto &ci : composites) {
        size_t nbytes;
        const uint8_t *data = ch.GetCompositeData(ci, nbytes);
        auto slots = decoder.DecodeComposite(data, nbytes);

        std::cout << "  ROC=0x" << std::hex << ci.roc << std::dec
                  << "  slots=" << slots.size() << "\n";
        for (auto &s : slots) {
            std::cout << "    slot=" << (int)s.slot
                      << " trig=" << s.trigger
                      << " ts=0x" << std::hex << (uint64_t)s.timestamp << std::dec
                      << " nch=" << s.channels.size() << " |";
            for (auto &[ch_num, chdata] : s.channels) {
                std::cout << " ch" << (int)ch_num << ":" << chdata.raw.size();
            }
            std::cout << "\n";
        }
    }
}

static int testFile(const std::string &path, int max_events)
{
    EvChannel ch;
    fdec::Fadc250Decoder decoder(250.0);

    if (ch.Open(path) != status::success) {
        std::cerr << "Failed to open: " << path << "\n";
        return 1;
    }

    int n = 0;
    while (ch.Read() == status::success) {
        processEvent(ch, decoder, ++n);
        if (max_events > 0 && n >= max_events) break;
    }
    std::cout << "Done. " << n << " event(s).\n";
    ch.Close();
    return 0;
}

static int testET(const std::string &ip, int port,
                  const std::string &et_file, const std::string &station)
{
    EtChannel ch;
    fdec::Fadc250Decoder decoder(250.0);

    if (ch.Connect(ip, port, et_file) != status::success) {
        std::cerr << "Failed to connect to ET.\n";
        return 1;
    }
    if (ch.Open(station) != status::success) {
        std::cerr << "Failed to open station.\n";
        ch.Disconnect();
        return 1;
    }

    int n = 0;
    status st;
    while (n < 20 && (st = ch.Read()) != status::failure) {
        if (st == status::empty) continue;
        processEvent(ch, decoder, ++n);
    }
    std::cout << "Done. " << n << " event(s).\n";
    ch.Disconnect();
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 2) { usage(argv[0]); return 1; }
    std::string first = argv[1];
    if (first == "--et") {
        if (argc < 6) { usage(argv[0]); return 1; }
        return testET(argv[2], std::atoi(argv[3]), argv[4], argv[5]);
    }
    int max_ev = (argc >= 3) ? std::atoi(argv[2]) : 0;
    return testFile(first, max_ev);
}
