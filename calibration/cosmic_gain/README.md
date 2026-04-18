# Cosmic Gain

ROOT macros for iteratively tuning HyCal PMT high voltages using cosmic-ray
peak-height measurements. `read_cosmic_json.C` loads a `cosmic_modules_runN.json`
summary (per-module peak height/integral from a cosmic run) and histograms the
per-module means for both PWO (W1–W1156) and LG (G-IDs) channels. `new_vset.C`
reads the current `vset_iterN.json` together with the latest cosmic summary and
produces the next `vset_iter{N+1}.json`: V0Set is nudged by ±5/±10/±20 V per
module to drive the peak-height mean into the 33–37 ADC target band, clamped at
1270 V (PWO) and 1800 V (LG). `vset_iter0.json` is the seed voltage set used at
the start of the iteration.
