#pragma once
//=============================================================================
// MatchingTools.h — detector matching tools for PRad2
//
// adapted from PRadAnalyzer/PRadDetMatch.cpp
//=============================================================================

#include "HyCalCluster.h"
#include <vector>
#include <cstdint>
#include <cmath>

// --- GEM hit (to be moved to a dedicated header later) ----------------------
namespace fdec {
struct GEMHit {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
};
} // namespace fdec

namespace analysis {

// --- matching flag bit positions --------------------------------------------
enum MatchFlag : uint32_t {
    kGEM1Match = 0,
    kGEM2Match = 1,
    kGEM3Match = 2,
    kGEM4Match = 3,
};

struct ProjectHit
{
    float x_proj;
    float y_proj;
    float z_proj;

    ProjectHit(float x, float y, float z) : x_proj(x), y_proj(y), z_proj(z) {};
};

class MatchHit
{
    public:
        fdec::ClusterHit cluster;
        std::vector<fdec::GEMHit> gem1;
        std::vector<fdec::GEMHit> gem2;
        std::vector<fdec::GEMHit> gem3;
        std::vector<fdec::GEMHit> gem4;

        MatchHit(const fdec::ClusterHit &cl, std::vector<fdec::GEMHit> &g1, std::vector<fdec::GEMHit> &g2,
                 const std::vector<fdec::GEMHit> &g3, const std::vector<fdec::GEMHit> &g4)
            : cluster(cl), gem1(g1), gem2(g2), gem3(g3), gem4(g4) {}

        // --- added for matching logic ----------------------------------------
        fdec::GEMHit gem{};         // best-matched GEM hit
        uint32_t     mflag = 0;     // matching flags (see MatchFlag enum)
        size_t       hycal_idx = 0; // index into original hycal vector
};

class MatchingTools
{
public:
    MatchingTools() = default;

    ProjectHit GetProjectionHits(float x, float y, float z, float projection_z) const;

    std::vector<MatchHit> Match(std::vector<fdec::ClusterHit> &hycalHits,
                            const std::vector<fdec::GEMHit> &gem1,
                            const std::vector<fdec::GEMHit> &gem2,
                            const std::vector<fdec::GEMHit> &gem3,
                            const std::vector<fdec::GEMHit> &gem4) const;

    // configuration setters
    void SetMatchRange(float range)    { matchRange_ = range; }
    void SetHyCalZ(float z)            { hycal_z_ = z; }
    void SetSquareSelection(bool sq)   { squareSel_ = sq; }

private:
    float hycal_z_    = 6225.f; // mm, default HyCal z position from target
    float matchRange_ = 15.f;   // mm, spatial matching window
    bool  squareSel_  = true;   // true = square window, false = circular

    float ProjectionDistance(const fdec::ClusterHit &h, const fdec::GEMHit &g) const;
    float ProjectionDistance(const fdec::GEMHit &g1, const fdec::GEMHit &g2, float ref_z) const;
    bool  PreMatch(const fdec::ClusterHit &hycal, const fdec::GEMHit &gem) const;
    void  PostMatch(MatchHit &h) const;
};

} // namespace analysis
