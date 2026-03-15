#include "EvChannel.h"
#include "evio.h"
#include <cstring>
#include <iostream>
#include <iomanip>
#include <exception>
#include <algorithm>

using namespace evc;

static inline status evio_status(int code)
{
    if (static_cast<unsigned int>(code) == S_EVFILE_UNXPTDEOF) {
        return status::incomplete;
    }
    switch (code) {
    case S_SUCCESS:      return status::success;
    case EOF:            return status::eof;
    case S_EVFILE_TRUNC: return status::incomplete;
    default:             return status::failure;
    }
}

EvChannel::EvChannel(size_t buflen) : fHandle(-1)
{
    buffer.resize(buflen);
}

status EvChannel::Open(const std::string &path)
{
    if (fHandle > 0) Close();
    char *cpath = strdup(path.c_str()), *copt = strdup("r");
    int status = evOpen(cpath, copt, &fHandle);
    free(cpath); free(copt);
    return evio_status(status);
}

void EvChannel::Close()
{
    evClose(fHandle);
    fHandle = -1;
}

status EvChannel::Read()
{
    return evio_status(evRead(fHandle, &buffer[0], buffer.size()));
}

bool EvChannel::ScanBanks(const std::vector<uint32_t> &banks)
{
    buffer_info.clear();
    composite_info.clear();

    auto evh = BankHeader(&buffer[0]);
    size_t iword = BankHeader::size();

    if (evh.length > buffer.size()) {
        std::cerr << "EvChannel Error: event length " << evh.length
                  << " exceeds buffer size " << buffer.size() << "\n";
        return false;
    }

    // top-level event must be a bank-of-banks
    if (evh.type != DATA_BANK && evh.type != DATA_ALSOBANK) {
        return false;
    }

    try {
        // iterate all child banks — don't assume a fixed order
        while (iword < evh.length + 1) {
            auto child = BankHeader(&buffer[iword]);

            // trigger bank: contains segments (type = DATA_SEGMENT or DATA_ALSOSEGMENT)
            if (child.type == DATA_SEGMENT || child.type == DATA_ALSOSEGMENT) {
                iword += child.length + 1;  // skip it
                continue;
            }

            // ROC data bank: contains sub-banks
            if (child.type == DATA_BANK || child.type == DATA_ALSOBANK) {
                iword += scanRocBank(&buffer[iword], iword, banks);
                continue;
            }

            // anything else (UINT32 control banks, STRING banks, etc.): skip
            iword += child.length + 1;
        }
    } catch (std::exception const &e) {
        std::cerr << e.what() << std::endl;
        return false;
    }

    return true;
}

size_t EvChannel::scanRocBank(const uint32_t *buf, size_t gindex, const std::vector<uint32_t> &banks)
{
    auto header = BankHeader(buf);
    size_t iword = BankHeader::size();

    // ROC bank should be bank-of-banks
    if (header.type != DATA_BANK && header.type != DATA_ALSOBANK) {
        // not a container — just skip
        return header.length + 1;
    }

    while (iword < header.length + 1) {
        auto bh = BankHeader(buf + iword);
        iword += BankHeader::size();

        bool interested = banks.empty() || (std::find(banks.begin(), banks.end(), bh.tag) != banks.end());

        if (interested) {
            if (bh.type == DATA_COMPOSITE) {
                scanCompositeBank(&buf[iword], bh.length - 1, header.tag, bh.tag, gindex + iword);
            } else {
                scanDataBank(&buf[iword], bh.length - 1, header.tag, bh.tag, gindex + iword);
            }
        }

        iword += bh.length - 1;
    }

    return header.length + 1;
}

void EvChannel::scanDataBank(const uint32_t *buf, size_t buflen, uint32_t roc, uint32_t bank, size_t gindex)
{
    uint32_t slot = 0, type;
    std::vector<BufferInfo> event_buffers;

    for (size_t iword = 0; iword < buflen; ++iword) {
        if (!(buf[iword] & 0x80000000)) continue;
        type = (buf[iword] >> 27) & 0xF;

        switch (type) {
        case BLOCK_HEADER:
            event_buffers.clear();
            slot = BlockHeader(buf + iword).slot;
            break;
        case BLOCK_TRAILER:
            if (event_buffers.size()) {
                event_buffers.back().len = iword - event_buffers.back().len;
                buffer_info[BufferAddress(roc, bank, slot)] = event_buffers;
            }
            break;
        case EVENT_HEADER:
            if (event_buffers.size()) {
                event_buffers.back().len = iword - event_buffers.back().len;
            }
            event_buffers.emplace_back(gindex + iword, iword);
            break;
        default:
            break;
        }
    }
}

void EvChannel::scanCompositeBank(const uint32_t *buf, size_t buflen, uint32_t roc, uint32_t bank, size_t gindex)
{
    if (buflen < 4) return;

    CompositeHeader ch(buf);
    if (ch.data_offset + ch.data_nwords > buflen) return;

    composite_info.emplace_back(roc, bank,
        static_cast<uint32_t>(gindex + ch.data_offset),
        static_cast<uint32_t>(ch.data_nwords));
}
