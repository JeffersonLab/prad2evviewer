// test/test_main.cpp
// Dump the full evio bank tree for every event in a file.
// For composite banks (tag 0xe101, format "c,i,l,N(c,Ns)"), decodes the
// payload and prints slot/trigger/timestamp/channel/sample information.
//
// Usage:
//   evc_test <evio_file>                          -- dump all events
//   evc_test <evio_file> <max_events>             -- dump first N events
//   evc_test --et <ip> <port> <file> <station>    -- read from ET

#include "EvChannel.h"
#include "EtChannel.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <cstring>
#include <algorithm>

using namespace evc;

// --------------------------------------------------------------------------
// Human-readable type name
// --------------------------------------------------------------------------
static const char *typeName(uint32_t type)
{
    switch (type) {
    case DATA_UNKNOWN32:   return "UNKNOWN32";
    case DATA_UINT32:      return "UINT32";
    case DATA_FLOAT32:     return "FLOAT32";
    case DATA_CHARSTAR8:   return "STRING";
    case DATA_SHORT16:     return "SHORT16";
    case DATA_USHORT16:    return "USHORT16";
    case DATA_CHAR8:       return "CHAR8";
    case DATA_UCHAR8:      return "UCHAR8";
    case DATA_DOUBLE64:    return "DOUBLE64";
    case DATA_LONG64:      return "LONG64";
    case DATA_ULONG64:     return "ULONG64";
    case DATA_INT32:       return "INT32";
    case DATA_TAGSEGMENT:  return "TAGSEGMENT";
    case DATA_ALSOSEGMENT: return "SEGMENT(0xd)";
    case DATA_ALSOBANK:    return "BANK(0xe)";
    case DATA_COMPOSITE:   return "COMPOSITE";
    case DATA_BANK:        return "BANK";
    case DATA_SEGMENT:     return "SEGMENT";
    default:               return "???";
    }
}

static bool isContainerType(uint32_t type)
{
    return type == DATA_BANK      || type == DATA_ALSOBANK ||
           type == DATA_SEGMENT   || type == DATA_ALSOSEGMENT ||
           type == DATA_TAGSEGMENT;
}

// --------------------------------------------------------------------------
static void indent(int depth)
{
    for (int i = 0; i < depth; ++i) std::cout << "  ";
}

static void hexPreview(const uint32_t *buf, size_t nwords, size_t maxshow = 8)
{
    size_t n = std::min(nwords, maxshow);
    std::cout << std::hex;
    for (size_t i = 0; i < n; ++i) {
        std::cout << " 0x" << std::setw(8) << std::setfill('0') << buf[i];
    }
    if (nwords > maxshow) std::cout << " ...";
    std::cout << std::dec << std::setfill(' ');
}

// --------------------------------------------------------------------------
// Forward declarations
// --------------------------------------------------------------------------
static void walkBank(const uint32_t *buf, size_t maxlen, int depth);
static void walkSegment(const uint32_t *buf, size_t maxlen, int depth);
static void walkTagSegment(const uint32_t *buf, size_t maxlen, int depth);
static void walkChildren(const uint32_t *buf, size_t nwords, uint32_t parent_type, int depth);
static void walkComposite(const uint32_t *buf, size_t nwords, uint32_t tag, int depth);

// --------------------------------------------------------------------------
// Decode composite payload for format "c,i,l,N(c,Ns)"
// (tag 0xe101 / 57601 — FADC250 window raw data, integrated pulse, etc.)
//
// Layout (packed, little-endian as returned by evRead on LE host):
//   Repeating per slot until end of data:
//     c  : uint8   slot number
//     i  : int32   trigger number (LE)
//     l  : int64   timestamp (LE)
//     N  : uint32  number of channels (LE, repeat count)
//     Per channel (N times):
//       c  : uint8   channel number
//       N  : uint32  number of samples (LE, repeat count)
//       s  : int16   sample value (LE), repeated N times
// --------------------------------------------------------------------------
static void decodeComposite_e101(const uint8_t *data, size_t nbytes, int depth)
{
    size_t pos = 0;
    int islot = 0;

    while (pos < nbytes) {
        // --- slot header: c, i, l, N ---
        if (pos + 1 + 4 + 8 + 4 > nbytes) break;

        uint8_t  slot = data[pos]; pos += 1;
        int32_t  trig;  memcpy(&trig, data + pos, 4); pos += 4;
        int64_t  ts;    memcpy(&ts,   data + pos, 8); pos += 8;
        uint32_t nchan; memcpy(&nchan, data + pos, 4); pos += 4;

        indent(depth);
        std::cout << "slot=" << (int)slot
                  << "  trigger=" << trig
                  << "  timestamp=0x" << std::hex << (uint64_t)ts << std::dec
                  << "  nChannels=" << nchan << "\n";

        for (uint32_t ich = 0; ich < nchan; ++ich) {
            if (pos + 1 + 4 > nbytes) {
                indent(depth + 1);
                std::cout << "(truncated at channel " << ich << ")\n";
                return;
            }

            uint8_t  ch    = data[pos]; pos += 1;
            uint32_t nsamp; memcpy(&nsamp, data + pos, 4); pos += 4;

            indent(depth + 1);
            std::cout << "ch=" << (int)ch << "  nSamp=" << nsamp;

            // print first few samples
            size_t nshow = std::min<size_t>(nsamp, 6);
            size_t nbytes_needed = nsamp * 2;
            if (pos + nbytes_needed > nbytes) {
                std::cout << "  (truncated)\n";
                return;
            }

            std::cout << "  [";
            for (size_t s = 0; s < nshow; ++s) {
                uint16_t val; memcpy(&val, data + pos + s * 2, 2);
                if (s) std::cout << ",";
                std::cout << val;
            }
            if (nsamp > nshow) std::cout << ",...";
            std::cout << "]\n";

            pos += nbytes_needed;
        }
        ++islot;
    }

    indent(depth);
    std::cout << "(" << islot << " slot(s), " << pos << "/" << nbytes << " bytes consumed)\n";
}

// --------------------------------------------------------------------------
// Walk a BANK node (2-word header)
// --------------------------------------------------------------------------
static void walkBank(const uint32_t *buf, size_t maxlen, int depth)
{
    if (maxlen < 2) return;
    BankHeader hdr(buf);

    indent(depth);
    std::cout << "BANK  tag=0x" << std::hex << hdr.tag << std::dec
              << " (" << hdr.tag << ")"
              << "  type=" << typeName(hdr.type) << "(0x" << std::hex << hdr.type << std::dec << ")"
              << "  num=" << hdr.num
              << "  length=" << hdr.length << " words"
              << "\n";

    size_t data_nwords = hdr.length - 1;
    const uint32_t *data = buf + 2;

    if (hdr.type == DATA_COMPOSITE) {
        walkComposite(data, data_nwords, hdr.tag, depth + 1);
    } else if (isContainerType(hdr.type)) {
        walkChildren(data, data_nwords, hdr.type, depth + 1);
    } else {
        indent(depth + 1);
        std::cout << "[" << data_nwords << " words]";
        hexPreview(data, data_nwords);
        std::cout << "\n";
    }
}

// --------------------------------------------------------------------------
// Walk a SEGMENT node (1-word header)
// --------------------------------------------------------------------------
static void walkSegment(const uint32_t *buf, size_t maxlen, int depth)
{
    if (maxlen < 1) return;
    SegmentHeader hdr(buf);
    uint32_t seg_len = hdr.num;

    indent(depth);
    std::cout << "SEG   tag=0x" << std::hex << hdr.tag << std::dec
              << " (" << hdr.tag << ")"
              << "  type=" << typeName(hdr.type) << "(0x" << std::hex << hdr.type << std::dec << ")"
              << "  length=" << seg_len << " words"
              << "\n";

    const uint32_t *data = buf + 1;

    if (isContainerType(hdr.type)) {
        walkChildren(data, seg_len, hdr.type, depth + 1);
    } else {
        indent(depth + 1);
        std::cout << "[" << seg_len << " words]";
        hexPreview(data, seg_len);
        std::cout << "\n";
    }
}

// --------------------------------------------------------------------------
// Walk a TAGSEGMENT node (1-word header)
// --------------------------------------------------------------------------
static void walkTagSegment(const uint32_t *buf, size_t maxlen, int depth)
{
    if (maxlen < 1) return;
    TagSegmentHeader hdr(buf);

    indent(depth);
    std::cout << "TSEG  tag=0x" << std::hex << hdr.tag << std::dec
              << " (" << hdr.tag << ")"
              << "  type=" << typeName(hdr.type) << "(0x" << std::hex << hdr.type << std::dec << ")"
              << "  length=" << hdr.length << " words"
              << "\n";

    const uint32_t *data = buf + 1;

    // Print format string if it's a string type
    if (hdr.type == DATA_CHARSTAR8 || hdr.type == DATA_CHAR8) {
        indent(depth + 1);
        const char *str = reinterpret_cast<const char*>(data);
        size_t nbytes = hdr.length * 4;
        std::cout << "format: \"";
        for (size_t i = 0; i < nbytes && str[i]; ++i) std::cout << str[i];
        std::cout << "\"\n";
    } else if (isContainerType(hdr.type)) {
        walkChildren(data, hdr.length, hdr.type, depth + 1);
    } else {
        indent(depth + 1);
        std::cout << "[" << hdr.length << " words]";
        hexPreview(data, hdr.length);
        std::cout << "\n";
    }
}

// --------------------------------------------------------------------------
// Walk children inside a container
// --------------------------------------------------------------------------
static void walkChildren(const uint32_t *buf, size_t nwords, uint32_t parent_type, int depth)
{
    size_t pos = 0;
    while (pos < nwords) {
        switch (parent_type) {
        case DATA_BANK:
        case DATA_ALSOBANK:
        {
            if (pos + 2 > nwords) return;
            BankHeader child(buf + pos);
            size_t child_total = child.length + 1;
            walkBank(buf + pos, nwords - pos, depth);
            pos += child_total;
            break;
        }
        case DATA_SEGMENT:
        case DATA_ALSOSEGMENT:
        {
            if (pos + 1 > nwords) return;
            SegmentHeader child(buf + pos);
            uint32_t seg_len = child.num;
            walkSegment(buf + pos, nwords - pos, depth);
            pos += 1 + seg_len;
            break;
        }
        case DATA_TAGSEGMENT:
        {
            if (pos + 1 > nwords) return;
            TagSegmentHeader child(buf + pos);
            walkTagSegment(buf + pos, nwords - pos, depth);
            pos += 1 + child.length;
            break;
        }
        default:
            return;
        }
    }
}

// --------------------------------------------------------------------------
// Walk composite data: tagsegment(format) + bank(data)
// --------------------------------------------------------------------------
static void walkComposite(const uint32_t *buf, size_t nwords, uint32_t parent_tag, int depth)
{
    if (nwords < 3) {
        indent(depth);
        std::cout << "[composite: too short, " << nwords << " words]\n";
        return;
    }

    // TagSegment with format string
    TagSegmentHeader ts(buf);
    walkTagSegment(buf, nwords, depth);

    // Inner Bank with actual data
    size_t inner_start = TagSegmentHeader::size() + ts.length;
    if (inner_start + 2 > nwords) {
        indent(depth);
        std::cout << "[composite: no inner bank]\n";
        return;
    }

    BankHeader inner(buf + inner_start);
    size_t data_nw = inner.length - 1;
    size_t data_off = inner_start + 2;

    indent(depth);
    std::cout << "BANK  tag=0x" << std::hex << inner.tag << std::dec
              << " (" << inner.tag << ")"
              << "  type=" << typeName(inner.type) << "(0x" << std::hex << inner.type << std::dec << ")"
              << "  num=" << inner.num
              << "  length=" << inner.length << " words"
              << "  (payload: " << data_nw * 4 << " bytes)"
              << "\n";

    if (data_off + data_nw > nwords) {
        indent(depth + 1);
        std::cout << "[payload overflows bank]\n";
        return;
    }

    const uint8_t *payload = reinterpret_cast<const uint8_t*>(buf + data_off);
    size_t payload_bytes = data_nw * 4;

    // Dispatch based on parent bank tag
    switch (parent_tag) {
    case 0xe101:   // "c,i,l,N(c,Ns)" — FADC250 integrated pulse / window raw
    case 0xe102:
    case 0xe103:
        decodeComposite_e101(payload, payload_bytes, depth + 1);
        break;
    default:
        // Unknown composite format: just hex dump
        indent(depth + 1);
        std::cout << "[" << data_nw << " words]";
        hexPreview(buf + data_off, data_nw, 12);
        std::cout << "\n";
        break;
    }
}

// --------------------------------------------------------------------------
// Dump one event
// --------------------------------------------------------------------------
static void dumpEvent(const uint32_t *buf, size_t bufsize, int event_num)
{
    if (bufsize < 2) return;
    std::cout << "========== Event " << event_num << " ==========\n";
    walkBank(buf, bufsize, 0);
    std::cout << "\n";
}

// --------------------------------------------------------------------------
static void usage(const char *prog)
{
    std::cerr << "Usage:\n"
              << "  " << prog << " <evio_file> [max_events]\n"
              << "  " << prog << " --et <ip> <port> <et_file> <station>\n";
}

// --------------------------------------------------------------------------
static int testFile(const std::string &path, int max_events)
{
    EvChannel ch;
    if (ch.Open(path) != status::success) {
        std::cerr << "Failed to open: " << path << "\n";
        return 1;
    }

    int nevents = 0;
    status st;
    while ((st = ch.Read()) == status::success) {
        ++nevents;
        auto hdr = ch.GetEvHeader();
        dumpEvent(ch.GetRawBuffer(), hdr.length + 1, nevents);
        if (max_events > 0 && nevents >= max_events) break;
    }

    std::cout << "Done. Read " << nevents << " event(s). Final status: "
              << static_cast<int>(st) << "\n";
    ch.Close();
    return 0;
}

// --------------------------------------------------------------------------
static int testET(const std::string &ip, int port,
                  const std::string &et_file, const std::string &station)
{
    EtChannel ch;
    if (ch.Connect(ip, port, et_file) != status::success) {
        std::cerr << "Failed to connect to ET at " << ip << ":" << port << "\n";
        return 1;
    }
    if (ch.Open(station) != status::success) {
        std::cerr << "Failed to open station: " << station << "\n";
        ch.Disconnect();
        return 1;
    }

    int nevents = 0, max_events = 20;
    status st;
    while (nevents < max_events && (st = ch.Read()) != status::failure) {
        if (st == status::empty) continue;
        ++nevents;
        auto hdr = ch.GetEvHeader();
        dumpEvent(ch.GetRawBuffer(), hdr.length + 1, nevents);
    }

    std::cout << "Done. Read " << nevents << " event(s).\n";
    ch.Disconnect();
    return 0;
}

// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 2) { usage(argv[0]); return 1; }

    std::string first = argv[1];
    if (first == "--et") {
        if (argc < 6) { usage(argv[0]); return 1; }
        return testET(argv[2], std::atoi(argv[3]), argv[4], argv[5]);
    }

    int max_events = (argc >= 3) ? std::atoi(argv[2]) : 0;
    return testFile(first, max_events);
}
