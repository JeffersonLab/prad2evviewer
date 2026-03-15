#pragma once
//=============================================================================
// Fadc250Decoder.h — decode FADC250 data from composite evio banks
//=============================================================================

#include "Fadc250Data.h"
#include <vector>
#include <cstdint>

namespace fdec
{

class Fadc250Decoder
{
public:
    // Decode composite payload: format "c,i,l,N(c,Ns)"
    // Packed, native-endian. Returns one SlotData per slot in the payload.
    static std::vector<SlotData> Decode(const uint8_t *data, size_t nbytes);
};

} // namespace fdec
