#pragma once

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

#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <cstring>
#include "Fadc250Data.h"


namespace fdec
{

// FADC250 raw data word types (used by the legacy decoder)
enum Fadc250Type {
    BlockHeader = 0,
    BlockTrailer = 1,
    EventHeader = 2,
    TriggerTime = 3,
    WindowRawData = 4,
    WindowSum = 5,
    PulseRawData = 6,
    PulseIntegral = 7,
    PulseTime = 8,
    StreamingRawData = 9,
    Scaler = 12,
    EventTrailer = 13,
    InvalidData = 14,
    FillerWord = 15,
};

class Fadc250Decoder
{
public:
    Fadc250Decoder(double clk = 250.);

    // ---------------------------------------------------------------
    //  Legacy: decode raw FADC250 module words
    // ---------------------------------------------------------------
    void DecodeEvent(Fadc250Event &event, const uint32_t *buf, size_t len) const;
    Fadc250Event DecodeEvent(const uint32_t *buf, size_t len, size_t nchans = 16) const;

    // ---------------------------------------------------------------
    //  Composite tag 0xe101: format "c,i,l,N(c,Ns)"
    //
    //  Packed byte stream, native-endian (LE on x86), as returned by
    //  evRead(). Contains multiple slots back-to-back:
    //
    //    Per slot:
    //      c  (uint8)  : slot number
    //      i  (int32)  : trigger number
    //      l  (int64)  : timestamp
    //      N  (uint32) : number of channels (repeat count)
    //      Per channel (N times):
    //        c  (uint8)  : channel number
    //        N  (uint32) : number of samples (repeat count)
    //        s  (int16)  : sample values, repeated N times
    //
    //  `data` / `nbytes` point to the composite data payload
    //  (after the TagSegment + format string + inner Bank header).
    // ---------------------------------------------------------------
    std::vector<CompositeSlot> DecodeComposite(const uint8_t *data, size_t nbytes) const;

private:
    double _clk;
};

}; // namespace fdec
