#pragma once

#include <vector>
#include <cstdint>
#include <cstring>

#define FADC250_MAX_NPEAKS 4
#define FADC250_MAX_NSAMPLES 256

// data structures
namespace fdec
{

class Peak {
public:
    double height, integral, time;
    uint32_t pos, left, right;
    bool overflow;

    Peak(double h = 0., double i = 0., double t = 0., uint32_t p = 0, uint32_t l = 0, uint32_t r = 0, bool o = false)
        : pos(p), left(l), right(r), height(h), integral(i), time(t), overflow(o)
    {}

    bool Inside(uint32_t i) {
        return (i >= pos - left) && (i <= pos + right);
    }
};

class Pedestal {
public:
    double mean, err;
    Pedestal(double m = 0., double e = 0.) : mean(m), err(e) {}
};

class Fadc250Data
{
public:
    Pedestal ped;
    std::vector<Peak> peaks;
    std::vector<uint16_t> raw;

    Fadc250Data(): ped(0., 0.)
    {
        peaks.reserve(FADC250_MAX_NPEAKS);
        raw.reserve(FADC250_MAX_NSAMPLES);
    }

    void Clear() { ped = Pedestal(0., 0.), peaks.clear(), raw.clear(); }
};

// Decoded composite slot: format "c,i,l,N(c,Ns)"
// One per slot within a composite bank payload.
struct CompositeSlot
{
    uint8_t  slot;
    int32_t  trigger;
    int64_t  timestamp;
    // channels[i].first = channel number, channels[i].second = sample data
    std::vector<std::pair<uint8_t, Fadc250Data>> channels;

    void Clear() { slot = 0; trigger = 0; timestamp = 0; channels.clear(); }
};

// Legacy event structure (kept for backward compat with raw FADC250 words)
class Fadc250Event
{
public:
    uint32_t number, mode;
    std::vector<uint32_t> time;
    std::vector<Fadc250Data> channels;

    Fadc250Event(uint32_t n = 0, uint32_t nch = 16)
        : number(n), mode(0)
    {
        channels.resize(nch);
    }

    void Clear()
    {
        mode = 0;
        time.clear();
        for (auto &ch : channels) { ch.Clear(); }
    }
};

}; // namespace fdec
